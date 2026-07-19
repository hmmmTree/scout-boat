"""Boat driver station — sends UDP commands to the ESP32 and shows live status.

Notes:
  * Windows showing "No internet" on the BoatControl network is NORMAL —
    the boat is an access point with no internet. The link still works.
  * Windows sometimes auto-switches back to your home WiFi. The WIFI pill
    in the header shows the network you are actually on right now.
"""
import socket
import subprocess
import threading
import time

import pygame

ESP32_IP   = "192.168.4.1"
ESP32_PORT = 4210
SEND_HZ    = 50
BOAT_SSID  = "BoatControl"

AXIS_LY = 1
AXIS_RX = 2
BTN_CROSS    = 0
BTN_CIRCLE   = 1
BTN_SQUARE   = 2
BTN_TRIANGLE = 3
BTN_L1       = 9
BTN_R1       = 10
DEADZONE = 0.15
TIMEOUT = 3

W, H = 960, 620

# ---- theme ----
BG      = (13, 15, 20)
PANEL   = (26, 30, 38)
PANEL2  = (36, 41, 52)
OUTLINE = (52, 58, 72)
WHITE   = (236, 239, 245)
GREY    = (124, 132, 146)
DIM     = (88, 95, 108)
GREEN   = (74, 222, 128)
RED     = (244, 78, 78)
ORANGE  = (251, 146, 60)
BLUE    = (96, 165, 250)
YELLOW  = (250, 204, 21)

F_BIG = F_MED = F_SM = F_LBL = F_HUGE = None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pct(cmd):
    return int(round((cmd - 90) / 90.0 * 100))


def safe_axis(js, i):
    try:
        return js.get_axis(i) if i < js.get_numaxes() else 0.0
    except Exception:
        return 0.0


def safe_button(js, i):
    try:
        return bool(js.get_button(i)) if i < js.get_numbuttons() else False
    except Exception:
        return False


# ---- current WiFi SSID, polled in a background thread (netsh is slow) ----
current_ssid = ["?"]

def _ssid_poller():
    while True:
        try:
            out = subprocess.run(
                ["netsh", "wlan", "show", "interfaces"],
                capture_output=True, text=True, timeout=5,
                creationflags=subprocess.CREATE_NO_WINDOW,
            ).stdout
            ssid = ""
            for line in out.splitlines():
                s = line.strip()
                if s.startswith("SSID") and not s.startswith("SSID BSSID"):
                    ssid = s.split(":", 1)[1].strip()
                    break
            current_ssid[0] = ssid or "(none)"
        except Exception:
            current_ssid[0] = "?"
        time.sleep(2)


# ---- drawing helpers ----
def text(surf, font, s, x, y, color=WHITE, center=False):
    img = font.render(s, True, color)
    r = img.get_rect()
    if center:
        r.center = (x, y)
    else:
        r.topleft = (x, y)
    surf.blit(img, r)


def panel(surf, x, y, w, h, fill=PANEL):
    pygame.draw.rect(surf, fill, (x, y, w, h), border_radius=12)
    pygame.draw.rect(surf, OUTLINE, (x, y, w, h), width=1, border_radius=12)


def pill(surf, x, y, w, h, label, ok, detail=""):
    panel(surf, x, y, w, h, PANEL2)
    col = GREEN if ok else RED
    pygame.draw.circle(surf, col, (x + 22, y + h // 2), 7)
    pygame.draw.circle(surf, BG, (x + 22, y + h // 2), 7, width=1)
    text(surf, F_LBL, label, x + 40, y + 9, GREY)
    text(surf, F_MED, detail if detail else ("OK" if ok else "LOST"),
         x + 40, y + 24, col)


def arm_banner(surf, x, y, w, h, armed):
    col = GREEN if armed else RED
    panel(surf, x, y, w, h, PANEL2)
    pygame.draw.rect(surf, col, (x, y, 6, h), border_radius=3)
    msg = "ARMED" if armed else "DISARMED"
    text(surf, F_BIG, msg, x + w // 2, y + h // 2 - 8, col, center=True)
    hint = "SQUARE / SPACE to disarm" if armed else "SQUARE / SPACE to arm"
    text(surf, F_LBL, hint, x + w // 2, y + h - 14, GREY, center=True)


def speed_bar(surf, x, y, w, h, speed, max_speed=2.0):
    panel(surf, x, y, w, h)
    pad = 16
    inner = w - 2 * pad
    text(surf, F_LBL, "SPEED LIMIT", x + pad, y + 12, GREY)
    text(surf, F_MED, f"{speed:.1f}x", x + w - pad - 44, y + 10, YELLOW)
    track_y = y + h - 24
    pygame.draw.rect(surf, PANEL2, (x + pad, track_y, inner, 10), border_radius=5)
    fill = int(inner * clamp(speed / max_speed, 0.0, 1.0))
    if fill > 0:
        pygame.draw.rect(surf, YELLOW, (x + pad, track_y, fill, 10), border_radius=5)
    for frac in (0.25, 0.5, 0.75):
        tx = x + pad + int(inner * frac)
        pygame.draw.line(surf, PANEL, (tx, track_y), (tx, track_y + 10), 1)


def winch_ind(surf, x, y, w, h, w_cmd):
    panel(surf, x, y, w, h)
    if w_cmd > 100:
        s, col = "IN  ^", GREEN
    elif w_cmd < 80:
        s, col = "OUT  v", ORANGE
    else:
        s, col = "STOP", GREY
    text(surf, F_LBL, "WINCH", x + 16, y + 12, GREY)
    text(surf, F_MED, s, x + w // 2, y + h - 20, col, center=True)


def motor_bar(surf, x, y, w, h, cmd, label, armed):
    panel(surf, x, y, w, h)
    cx = x + w // 2
    mid = y + h // 2
    pygame.draw.line(surf, DIM, (x + 12, mid), (x + w - 12, mid), 2)
    for frac in (0.5, 1.0):                     # tick marks at 50% / 100%
        for sgn in (-1, 1):
            ty = mid - int(sgn * frac * (h // 2 - 30))
            pygame.draw.line(surf, PANEL2, (x + 12, ty), (x + w - 12, ty), 1)
    dev = (cmd - 90) / 90.0
    span = h // 2 - 30
    bar_h = int(abs(dev) * span)
    bw = w - 30
    col = (GREEN if dev >= 0 else ORANGE) if armed else DIM
    if bar_h > 0:
        if dev >= 0:
            pygame.draw.rect(surf, col, (x + 15, mid - bar_h, bw, bar_h), border_radius=4)
        else:
            pygame.draw.rect(surf, col, (x + 15, mid, bw, bar_h), border_radius=4)
    text(surf, F_LBL, label, cx, y + 15, GREY, center=True)
    text(surf, F_MED, f"{pct(cmd):+d}%", cx, y + h - 18, WHITE if armed else DIM, center=True)


def stick_box(surf, x, y, size, ax, ay, label):
    panel(surf, x, y, size, size)
    cx, cy = x + size // 2, y + size // 2
    rad = size // 2 - 18
    pygame.draw.circle(surf, PANEL2, (cx, cy), rad, width=1)
    pygame.draw.circle(surf, PANEL2, (cx, cy), int(rad * DEADZONE), width=1)
    pygame.draw.line(surf, PANEL2, (x + 12, cy), (x + size - 12, cy), 1)
    pygame.draw.line(surf, PANEL2, (cx, y + 12), (cx, y + size - 12), 1)
    dx = cx + int(clamp(ax, -1, 1) * rad)
    dy = cy + int(clamp(ay, -1, 1) * rad)
    pygame.draw.line(surf, DIM, (cx, cy), (dx, dy), 2)
    pygame.draw.circle(surf, BLUE, (dx, dy), 9)
    pygame.draw.circle(surf, WHITE, (dx, dy), 9, width=1)
    text(surf, F_LBL, label, cx, y + size - 16, GREY, center=True)


def main():
    global F_BIG, F_MED, F_SM, F_LBL, F_HUGE

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    threading.Thread(target=_ssid_poller, daemon=True).start()

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("BOAT DRIVER STATION")
    clock = pygame.time.Clock()

    F_HUGE = pygame.font.SysFont("Consolas", 40, bold=True)
    F_BIG  = pygame.font.SysFont("Consolas", 26, bold=True)
    F_MED  = pygame.font.SysFont("Consolas", 18, bold=True)
    F_SM   = pygame.font.SysFont("Consolas", 16)
    F_LBL  = pygame.font.SysFont("Consolas", 12, bold=True)

    js = None
    speed = 1.0
    motors_on = False          # SAFETY: always start disarmed
    winch_cmd = 90
    w_cmd = 90
    left_cmd = right_cmd = 90
    prev_square = False
    prev_l1 = False
    prev_r1 = False
    last_ack = 0.0
    ack_count = 0
    disconnect_time = 0.0
    running = True

    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_UP, pygame.K_RIGHT):
                    speed = clamp(round(speed + 0.1, 1), 0.0, 2.0)
                elif e.key in (pygame.K_DOWN, pygame.K_LEFT):
                    speed = clamp(round(speed - 0.1, 1), 0.0, 2.0)
                elif e.key == pygame.K_SPACE:
                    motors_on = not motors_on
                elif e.key == pygame.K_ESCAPE:
                    running = False

        try:
            count = pygame.joystick.get_count()
        except Exception:
            count = 0
        if count > 0 and js is None:
            try:
                js = pygame.joystick.Joystick(0)
                js.init()
            except Exception:
                js = None
        elif count == 0 and js is not None:
            js = None
            disconnect_time = time.time()
        connected = js is not None

        if connected:
            forward = -safe_axis(js, AXIS_LY)
            turn    =  safe_axis(js, AXIS_RX)
            if abs(forward) < DEADZONE:
                forward = 0.0
            if abs(turn) < DEADZONE:
                turn = 0.0

            square   = safe_button(js, BTN_SQUARE)
            triangle = safe_button(js, BTN_TRIANGLE)
            circle   = safe_button(js, BTN_CIRCLE)
            cross    = safe_button(js, BTN_CROSS)

            if square and not prev_square:
                motors_on = not motors_on
            prev_square = square

            l1 = safe_button(js, BTN_L1)
            r1 = safe_button(js, BTN_R1)
            if r1 and not prev_r1:
                speed = clamp(round(speed + 0.1, 1), 0.0, 2.0)
            if l1 and not prev_l1:
                speed = clamp(round(speed - 0.1, 1), 0.0, 2.0)
            prev_r1 = r1
            prev_l1 = l1

            if triangle:
                winch_cmd = 180
            elif circle:
                winch_cmd = 90
            elif cross:
                winch_cmd = 0

            if motors_on:
                lp = clamp(forward + turn, -1.0, 1.0)
                rp = clamp(forward - turn, -1.0, 1.0)
                rng = 90 * speed
                left_cmd  = int(clamp(90 + lp * rng, 0, 180))
                right_cmd = int(clamp(90 + rp * rng, 0, 180))
                w_cmd = winch_cmd
            else:
                left_cmd = right_cmd = 90
                w_cmd = 90
        else:
            left_cmd = right_cmd = 90
            if time.time() - disconnect_time < TIMEOUT:
                w_cmd = winch_cmd
            else:
                w_cmd = 90
                winch_cmd = 90

        # E:0 tells the boat to hard-stop instantly (no ramp-down)
        en = 1 if motors_on else 0
        try:
            sock.sendto(f"L:{left_cmd},R:{right_cmd},W:{w_cmd},E:{en}".encode(),
                        (ESP32_IP, ESP32_PORT))
        except OSError:
            pass
        try:
            while True:
                data, _ = sock.recvfrom(32)
                if data:
                    last_ack = time.time()
                    ack_count += 1
        except OSError:
            pass
        boat_ok = (time.time() - last_ack) < 1.0

        lx = safe_axis(js, 0) if connected else 0.0
        ly = safe_axis(js, 1) if connected else 0.0
        rx = safe_axis(js, 2) if connected else 0.0
        ry = safe_axis(js, 3) if connected else 0.0

        # ---- draw ----
        screen.fill(BG)
        text(screen, F_HUGE, "BOAT DRIVER STATION", 30, 22, WHITE)
        pygame.draw.line(screen, OUTLINE, (30, 72), (W - 30, 72), 1)

        ssid = current_ssid[0]
        on_boat_wifi = (ssid == BOAT_SSID)
        wifi_detail = ssid if len(ssid) <= 16 else ssid[:15] + "…"

        pill(screen, 30, 86, 218, 48, "CONTROLLER", connected,
             "OK" if connected else "NOT FOUND")
        pill(screen, 258, 86, 218, 48, "WIFI NETWORK", on_boat_wifi, wifi_detail)
        age = time.time() - last_ack
        pill(screen, 486, 86, 218, 48, "BOAT LINK", boat_ok,
             f"OK  {int(age*1000)}ms" if boat_ok else "LOST")
        arm_banner(screen, 714, 86, 216, 48, motors_on)

        # wifi guidance line — the single most common gotcha
        if not on_boat_wifi:
            text(screen, F_SM,
                 f"Connect this computer to WiFi \"{BOAT_SSID}\" — \"No internet\" on it is normal.",
                 30, 146, ORANGE)
        elif not boat_ok:
            text(screen, F_SM,
                 "On boat WiFi but no reply — is the boat powered? (it acks every packet)",
                 30, 146, ORANGE)

        speed_bar(screen, 30, 172, 560, 62, speed)
        winch_ind(screen, 606, 172, 324, 62, w_cmd)

        motor_bar(screen, 30, 252, 120, 316, left_cmd, "LEFT", motors_on)
        motor_bar(screen, 162, 252, 120, 316, right_cmd, "RIGHT", motors_on)

        stick_box(screen, 306, 262, 186, lx, ly, "LEFT STICK")
        stick_box(screen, 508, 262, 186, rx, ry, "RIGHT STICK")

        panel(screen, 710, 262, 220, 186)
        text(screen, F_LBL, "TELEMETRY", 726, 276, GREY)
        text(screen, F_SM, f"L out : {pct(left_cmd):+d}%", 726, 300, WHITE)
        text(screen, F_SM, f"R out : {pct(right_cmd):+d}%", 726, 324, WHITE)
        text(screen, F_SM, f"W out : {pct(w_cmd):+d}%", 726, 348, WHITE)
        text(screen, F_SM, f"acks  : {ack_count}", 726, 380, GREY)
        text(screen, F_SM, f"-> {ESP32_IP}:{ESP32_PORT}", 726, 404, DIM)

        text(screen, F_SM,
             "L1/R1 or Arrows = speed    SQUARE or SPACE = arm/disarm    "
             "TRI/O/X = winch in/stop/out    ESC = quit",
             30, H - 34, GREY)

        pygame.display.flip()
        clock.tick(SEND_HZ)

    try:
        sock.sendto(b"L:90,R:90,W:90,E:0", (ESP32_IP, ESP32_PORT))
    except OSError:
        pass
    pygame.quit()


if __name__ == "__main__":
    main()
