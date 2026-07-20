"""Boat driver station — drive panel + navigation/mission panel.

Keys / controls:
  F11        fullscreen toggle          SPACE / Square   arm-disarm
  Arrows or L1/R1  speed limit          ESC              exit fullscreen / quit
  TRI/O/X    winch in/stop/out
  Map: left-click = add waypoint, right-click = delete last, C = clear all

Network protocol (UDP to 192.168.4.1:4210):
  send: "L:<0-180>,R:<0-180>,W:<0-180>,E:<0|1>"   50x per second
  recv: "OK"                                  plain ack (current firmware)
        "OK T:<lat>,<lon>,<heading>,<sats>"   ack with GPS telemetry (future
        firmware with GPS+compass fitted) — map, compass and missions
        activate automatically when these arrive.

Windows note: "No internet" on the BoatControl network is normal — the boat
is an access point with no internet behind it. The WIFI pill shows the
network you are actually on.
"""
import math
import socket
import subprocess
import threading
import time
from collections import deque

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

# virtual canvas — everything is drawn at this size, then scaled to the window
W, H = 1280, 720

# ---- theme ----
BG      = (11, 13, 18)
PANEL   = (24, 28, 36)
PANEL2  = (34, 39, 50)
OUTLINE = (50, 56, 70)
WHITE   = (236, 239, 245)
GREY    = (124, 132, 146)
DIM     = (80, 87, 100)
GREEN   = (74, 222, 128)
RED     = (244, 78, 78)
ORANGE  = (251, 146, 60)
BLUE    = (96, 165, 250)
YELLOW  = (250, 204, 21)
CYAN    = (94, 234, 212)

FONTS = {}


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
                if s.startswith("SSID") and "BSSID" not in s:
                    ssid = s.split(":", 1)[1].strip()
                    break
            current_ssid[0] = ssid or "(none)"
        except Exception:
            current_ssid[0] = "?"
        time.sleep(2)


# ---- GPS state (filled in when the boat firmware sends T: telemetry) ----
class Nav:
    def __init__(self):
        self.has_fix = False
        self.lat = self.lon = 0.0
        self.heading = None          # degrees, None = unknown
        self.sats = 0
        self.origin = None           # (lat, lon) of first fix = map origin
        self.trail = deque(maxlen=3000)   # local (x_m, y_m) points
        self.last_update = 0.0

    def feed(self, lat, lon, heading, sats):
        self.lat, self.lon, self.sats = lat, lon, sats
        self.heading = heading
        self.has_fix = sats >= 4
        self.last_update = time.time()
        if self.has_fix:
            if self.origin is None:
                self.origin = (lat, lon)
            self.trail.append(self.to_local(lat, lon))

    def to_local(self, lat, lon):
        """Equirectangular lat/lon -> meters east/north of origin."""
        if self.origin is None:
            return (0.0, 0.0)
        olat, olon = self.origin
        x = (lon - olon) * 111320.0 * math.cos(math.radians(olat))
        y = (lat - olat) * 110540.0
        return (x, y)

    @property
    def alive(self):
        return (time.time() - self.last_update) < 3.0


def parse_ack(data, nav):
    """Ack is 'OK' or 'OK T:<lat>,<lon>,<hdg>,<sats>'."""
    try:
        s = data.decode(errors="replace").strip()
    except Exception:
        return
    if s.startswith("OK T:"):
        try:
            lat, lon, hdg, sats = s[5:].split(",")[:4]
            nav.feed(float(lat), float(lon), float(hdg), int(sats))
        except (ValueError, IndexError):
            pass


# ---- drawing helpers ----
def F(name):
    return FONTS[name]


def text(surf, fname, s, x, y, color=WHITE, center=False):
    img = F(fname).render(s, True, color)
    r = img.get_rect()
    if center:
        r.center = (x, y)
    else:
        r.topleft = (x, y)
    surf.blit(img, r)


def panel(surf, x, y, w, h, fill=PANEL):
    pygame.draw.rect(surf, fill, (x, y, w, h), border_radius=10)
    pygame.draw.rect(surf, OUTLINE, (x, y, w, h), width=1, border_radius=10)


def pill(surf, x, y, w, h, label, ok, detail=""):
    panel(surf, x, y, w, h, PANEL2)
    col = GREEN if ok else RED
    pygame.draw.circle(surf, col, (x + 18, y + h // 2), 6)
    text(surf, "lbl", label, x + 34, y + 7, GREY)
    text(surf, "med", detail if detail else ("OK" if ok else "LOST"),
         x + 34, y + 21, col)


def button(surf, rect, label, enabled=True, accent=BLUE):
    x, y, w, h = rect
    panel(surf, x, y, w, h, PANEL2 if enabled else PANEL)
    col = accent if enabled else DIM
    text(surf, "med", label, x + w // 2, y + h // 2, col, center=True)
    return pygame.Rect(rect)


def arm_banner(surf, x, y, w, h, armed):
    col = GREEN if armed else RED
    panel(surf, x, y, w, h, PANEL2)
    pygame.draw.rect(surf, col, (x, y, 5, h), border_radius=2)
    text(surf, "big", "ARMED" if armed else "DISARMED",
         x + w // 2, y + h // 2 - 7, col, center=True)
    text(surf, "lbl", "SQUARE / SPACE", x + w // 2, y + h - 11, GREY, center=True)


def speed_bar(surf, x, y, w, h, speed, max_speed=2.0):
    panel(surf, x, y, w, h)
    pad = 14
    inner = w - 2 * pad
    text(surf, "lbl", "SPEED LIMIT", x + pad, y + 10, GREY)
    text(surf, "med", f"{speed:.1f}x", x + w - pad - 40, y + 8, YELLOW)
    ty = y + h - 20
    pygame.draw.rect(surf, PANEL2, (x + pad, ty, inner, 9), border_radius=4)
    fill = int(inner * clamp(speed / max_speed, 0.0, 1.0))
    if fill > 0:
        pygame.draw.rect(surf, YELLOW, (x + pad, ty, fill, 9), border_radius=4)


def winch_ind(surf, x, y, w, h, w_cmd):
    panel(surf, x, y, w, h)
    if w_cmd > 100:
        s, col = "IN ^", GREEN
    elif w_cmd < 80:
        s, col = "OUT v", ORANGE
    else:
        s, col = "STOP", GREY
    text(surf, "lbl", "WINCH", x + 12, y + 10, GREY)
    text(surf, "med", s, x + w - 12 - F("med").size(s)[0], y + h - 26, col)


def motor_bar(surf, x, y, w, h, cmd, label, armed):
    panel(surf, x, y, w, h)
    cx = x + w // 2
    mid = y + h // 2
    pygame.draw.line(surf, DIM, (x + 10, mid), (x + w - 10, mid), 2)
    dev = (cmd - 90) / 90.0
    span = h // 2 - 26
    bar_h = int(abs(dev) * span)
    col = (GREEN if dev >= 0 else ORANGE) if armed else DIM
    if bar_h > 0:
        if dev >= 0:
            pygame.draw.rect(surf, col, (x + 12, mid - bar_h, w - 24, bar_h), border_radius=4)
        else:
            pygame.draw.rect(surf, col, (x + 12, mid, w - 24, bar_h), border_radius=4)
    text(surf, "lbl", label, cx, y + 13, GREY, center=True)
    text(surf, "med", f"{pct(cmd):+d}%", cx, y + h - 15, WHITE if armed else DIM, center=True)


def stick_box(surf, x, y, size, ax, ay, label):
    panel(surf, x, y, size, size)
    cx, cy = x + size // 2, y + size // 2
    rad = size // 2 - 15
    pygame.draw.circle(surf, PANEL2, (cx, cy), rad, width=1)
    pygame.draw.circle(surf, PANEL2, (cx, cy), int(rad * DEADZONE), width=1)
    pygame.draw.line(surf, PANEL2, (x + 10, cy), (x + size - 10, cy), 1)
    pygame.draw.line(surf, PANEL2, (cx, y + 10), (cx, y + size - 10), 1)
    dx = cx + int(clamp(ax, -1, 1) * rad)
    dy = cy + int(clamp(ay, -1, 1) * rad)
    pygame.draw.line(surf, DIM, (cx, cy), (dx, dy), 2)
    pygame.draw.circle(surf, BLUE, (dx, dy), 7)
    text(surf, "lbl", label, cx, y + size - 13, GREY, center=True)


def compass(surf, x, y, size, heading):
    panel(surf, x, y, size, size)
    cx, cy = x + size // 2, y + size // 2 + 6
    rad = size // 2 - 24
    pygame.draw.circle(surf, PANEL2, (cx, cy), rad, width=2)
    for ang, name in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
        a = math.radians(ang - 90)
        tx = cx + int(math.cos(a) * (rad - 12))
        ty = cy + int(math.sin(a) * (rad - 12))
        text(surf, "lbl", name, tx, ty, RED if name == "N" else GREY, center=True)
    text(surf, "lbl", "HEADING", x + size // 2, y + 12, GREY, center=True)
    if heading is None:
        text(surf, "big", "--", cx, cy, DIM, center=True)
    else:
        a = math.radians(heading - 90)
        hx = cx + int(math.cos(a) * (rad - 26))
        hy = cy + int(math.sin(a) * (rad - 26))
        pygame.draw.line(surf, CYAN, (cx, cy), (hx, hy), 3)
        pygame.draw.circle(surf, CYAN, (cx, cy), 4)
        text(surf, "med", f"{int(heading):03d}°", cx, y + size - 20, CYAN, center=True)


class MapView:
    """Local-frame map: meters relative to first GPS fix (or planning origin)."""

    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self.m_per_px = 0.5          # zoom: meters per pixel
        self.waypoints = []          # [(x_m, y_m), ...]

    def world_to_px(self, wx, wy):
        cx = self.rect.centerx + wx / self.m_per_px
        cy = self.rect.centery - wy / self.m_per_px
        return int(cx), int(cy)

    def px_to_world(self, px, py):
        wx = (px - self.rect.centerx) * self.m_per_px
        wy = (self.rect.centery - py) * self.m_per_px
        return wx, wy

    def zoom(self, direction):
        self.m_per_px = clamp(self.m_per_px * (0.8 if direction > 0 else 1.25),
                              0.05, 10.0)

    def click(self, pos, btn):
        if not self.rect.collidepoint(pos):
            return False
        if btn == 1:
            self.waypoints.append(self.px_to_world(*pos))
        elif btn == 3 and self.waypoints:
            self.waypoints.pop()
        return True

    def draw(self, surf, nav):
        r = self.rect
        panel(surf, r.x, r.y, r.w, r.h, (16, 19, 26))
        clip_prev = surf.get_clip()
        surf.set_clip(r.inflate(-4, -4))

        # grid: pick a "nice" spacing near 60px
        step_m = self.m_per_px * 60
        nice = min((1, 2, 5, 10, 20, 50, 100, 200, 500), key=lambda n: abs(n - step_m))
        step_px = nice / self.m_per_px
        ox, oy = self.world_to_px(0, 0)
        gx = ox % step_px
        while gx < r.right:
            if gx > r.left:
                pygame.draw.line(surf, (26, 31, 41), (gx, r.top), (gx, r.bottom))
            gx += step_px
        gy = oy % step_px
        while gy < r.bottom:
            if gy > r.top:
                pygame.draw.line(surf, (26, 31, 41), (r.left, gy), (r.right, gy))
            gy += step_px

        # home marker at origin
        hx, hy = self.world_to_px(0, 0)
        pygame.draw.circle(surf, YELLOW, (hx, hy), 6, width=2)
        text(surf, "lbl", "HOME", hx, hy - 14, YELLOW, center=True)

        # waypoints + route
        pts = [self.world_to_px(*wp) for wp in self.waypoints]
        if pts:
            route = [(hx, hy)] + pts
            pygame.draw.lines(surf, (60, 90, 140), False, route, 2)
        for i, p in enumerate(pts):
            pygame.draw.circle(surf, BLUE, p, 7)
            pygame.draw.circle(surf, WHITE, p, 7, width=1)
            text(surf, "lbl", str(i + 1), p[0], p[1], WHITE, center=True)

        # boat trail + boat
        if len(nav.trail) > 1:
            pygame.draw.lines(surf, (40, 160, 120), False,
                              [self.world_to_px(*p) for p in nav.trail], 2)
        if nav.alive and nav.has_fix:
            bx, by = self.world_to_px(*nav.trail[-1]) if nav.trail else (hx, hy)
            hdg = nav.heading if nav.heading is not None else 0
            a = math.radians(hdg - 90)
            tip = (bx + int(math.cos(a) * 12), by + int(math.sin(a) * 12))
            l = (bx + int(math.cos(a + 2.5) * 9), by + int(math.sin(a + 2.5) * 9))
            rr = (bx + int(math.cos(a - 2.5) * 9), by + int(math.sin(a - 2.5) * 9))
            pygame.draw.polygon(surf, CYAN, (tip, l, rr))
        else:
            text(surf, "med", "NO GPS — showing planning grid",
                 r.centerx, r.top + 22, DIM, center=True)

        # scale legend
        text(surf, "lbl", f"grid {nice} m   scroll = zoom",
             r.left + 12, r.bottom - 22, GREY)
        surf.set_clip(clip_prev)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    threading.Thread(target=_ssid_poller, daemon=True).start()

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((W, H), pygame.RESIZABLE)
    pygame.display.set_caption("BOAT DRIVER STATION")
    canvas = pygame.Surface((W, H))
    clock = pygame.time.Clock()
    fullscreen = False

    FONTS.update(
        huge=pygame.font.SysFont("Consolas", 34, bold=True),
        big=pygame.font.SysFont("Consolas", 24, bold=True),
        med=pygame.font.SysFont("Consolas", 17, bold=True),
        sm=pygame.font.SysFont("Consolas", 15),
        lbl=pygame.font.SysFont("Consolas", 12, bold=True),
    )

    nav = Nav()
    mapview = MapView((652, 96, 598, 520))

    js = None
    speed = 1.0
    motors_on = False              # SAFETY: always start disarmed
    mission_on = False
    winch_cmd = 90
    w_cmd = 90
    left_cmd = right_cmd = 90
    prev_square = prev_l1 = prev_r1 = False
    last_ack = 0.0
    ack_count = 0
    disconnect_time = 0.0
    running = True

    def to_canvas(pos):
        """window coords -> virtual canvas coords (inverse of the letterbox)."""
        win_w, win_h = screen.get_size()
        scale = min(win_w / W, win_h / H)
        ox = (win_w - W * scale) / 2
        oy = (win_h - H * scale) / 2
        return ((pos[0] - ox) / scale, (pos[1] - oy) / scale)

    btn_start = pygame.Rect(0, 0, 0, 0)
    btn_clear = pygame.Rect(0, 0, 0, 0)

    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_F11:
                    fullscreen = not fullscreen
                    if fullscreen:
                        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                    else:
                        screen = pygame.display.set_mode((W, H), pygame.RESIZABLE)
                elif e.key == pygame.K_ESCAPE:
                    if fullscreen:
                        fullscreen = False
                        screen = pygame.display.set_mode((W, H), pygame.RESIZABLE)
                    else:
                        running = False
                elif e.key in (pygame.K_UP, pygame.K_RIGHT):
                    speed = clamp(round(speed + 0.1, 1), 0.0, 2.0)
                elif e.key in (pygame.K_DOWN, pygame.K_LEFT):
                    speed = clamp(round(speed - 0.1, 1), 0.0, 2.0)
                elif e.key == pygame.K_SPACE:
                    motors_on = not motors_on
                elif e.key == pygame.K_c:
                    mapview.waypoints.clear()
                    mission_on = False
            elif e.type == pygame.MOUSEBUTTONDOWN:
                cpos = to_canvas(e.pos)
                if e.button in (4, 5):
                    if mapview.rect.collidepoint(cpos):
                        mapview.zoom(1 if e.button == 4 else -1)
                elif btn_start.collidepoint(cpos):
                    if nav.has_fix and mapview.waypoints:
                        mission_on = not mission_on
                elif btn_clear.collidepoint(cpos):
                    mapview.waypoints.clear()
                    mission_on = False
                else:
                    mapview.click(cpos, e.button)
            elif e.type == pygame.MOUSEWHEEL:
                cpos = to_canvas(pygame.mouse.get_pos())
                if mapview.rect.collidepoint(cpos):
                    mapview.zoom(e.y)

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
                data, _ = sock.recvfrom(128)
                if data:
                    last_ack = time.time()
                    ack_count += 1
                    parse_ack(data, nav)
        except OSError:
            pass
        boat_ok = (time.time() - last_ack) < 1.0

        lx = safe_axis(js, 0) if connected else 0.0
        ly = safe_axis(js, 1) if connected else 0.0
        rx = safe_axis(js, 2) if connected else 0.0
        ry = safe_axis(js, 3) if connected else 0.0

        # ================= draw =================
        canvas.fill(BG)
        text(canvas, "huge", "BOAT DRIVER STATION", 28, 18, WHITE)
        text(canvas, "lbl", "F11 FULLSCREEN", W - 130, 30, DIM)
        pygame.draw.line(canvas, OUTLINE, (28, 62), (W - 28, 62), 1)

        ssid = current_ssid[0]
        on_boat_wifi = (ssid == BOAT_SSID)
        wifi_detail = ssid if len(ssid) <= 14 else ssid[:13] + "…"

        # -- left column: drive --
        pill(canvas, 28, 78, 196, 42, "CONTROLLER", connected,
             "OK" if connected else "NOT FOUND")
        pill(canvas, 232, 78, 196, 42, "WIFI", on_boat_wifi, wifi_detail)
        age = time.time() - last_ack
        pill(canvas, 436, 78, 196, 42, "BOAT LINK", boat_ok,
             f"OK {int(age*1000)}ms" if boat_ok else "LOST")

        if not on_boat_wifi:
            text(canvas, "sm", f'Join WiFi "{BOAT_SSID}" — "No internet" there is normal.',
                 28, 128, ORANGE)
        elif not boat_ok:
            text(canvas, "sm", "On boat WiFi but no reply — is the boat powered?",
                 28, 128, ORANGE)

        arm_banner(canvas, 28, 150, 300, 64, motors_on)
        speed_bar(canvas, 336, 150, 296, 64, speed)

        motor_bar(canvas, 28, 230, 92, 268, left_cmd, "LEFT", motors_on)
        motor_bar(canvas, 128, 230, 92, 268, right_cmd, "RIGHT", motors_on)
        stick_box(canvas, 236, 230, 130, lx, ly, "L STICK")
        stick_box(canvas, 236, 368, 130, rx, ry, "R STICK")
        winch_ind(canvas, 374, 230, 120, 64, w_cmd)

        panel(canvas, 374, 302, 258, 196)
        text(canvas, "lbl", "TELEMETRY", 388, 314, GREY)
        text(canvas, "sm", f"L out : {pct(left_cmd):+d}%", 388, 336, WHITE)
        text(canvas, "sm", f"R out : {pct(right_cmd):+d}%", 388, 358, WHITE)
        text(canvas, "sm", f"W out : {pct(w_cmd):+d}%", 388, 380, WHITE)
        text(canvas, "sm", f"acks  : {ack_count}", 388, 402, GREY)
        text(canvas, "sm", f"-> {ESP32_IP}:{ESP32_PORT}", 388, 424, DIM)
        text(canvas, "sm", f"send  : {SEND_HZ} Hz", 388, 446, DIM)

        panel(canvas, 502, 230, 130, 64)
        text(canvas, "lbl", "GPS", 514, 240, GREY)
        if nav.alive and nav.has_fix:
            text(canvas, "med", f"{nav.sats} SAT", 514, 262, GREEN)
        else:
            text(canvas, "med", "NO FIX", 514, 262, DIM)

        text(canvas, "sm",
             "L1/R1|Arrows speed   SQUARE|SPACE arm   TRI/O/X winch   F11 fullscreen",
             28, H - 30, GREY)

        # -- right column: navigation / mission --
        text(canvas, "lbl", "NAVIGATION", 652, 70, GREY)
        mapview.draw(canvas, nav)

        # mission bar under the map
        bar_y = 96 + 520 + 8
        panel(canvas, 652, bar_y, 598, 44)
        wp_n = len(mapview.waypoints)
        text(canvas, "med", f"WAYPOINTS: {wp_n}", 668, bar_y + 13, WHITE)
        mission_ok = nav.has_fix and wp_n > 0
        btn_start = button(canvas, (860, bar_y + 6, 150, 32),
                           "STOP MISSION" if mission_on else "START MISSION",
                           enabled=mission_ok, accent=GREEN if not mission_on else RED)
        btn_clear = button(canvas, (1020, bar_y + 6, 100, 32), "CLEAR (C)")
        if not nav.has_fix:
            text(canvas, "lbl", "needs GPS", 1136, bar_y + 17, DIM)

        # GPS detail + compass to the left of map bottom? -> place compass in map corner
        if nav.alive and nav.has_fix:
            text(canvas, "sm", f"{nav.lat:.6f}, {nav.lon:.6f}",
                 668, bar_y - 24, CYAN)

        # compass: bottom-right corner of the map
        comp_size = 120
        compass(canvas, 1250 - comp_size - 8,
                96 + 520 - comp_size - 8, comp_size, nav.heading)

        # ---- letterbox scale to window ----
        win_w, win_h = screen.get_size()
        scale = min(win_w / W, win_h / H)
        sw, sh = int(W * scale), int(H * scale)
        screen.fill((0, 0, 0))
        if (sw, sh) == (W, H):
            screen.blit(canvas, ((win_w - W) // 2, (win_h - H) // 2))
        else:
            screen.blit(pygame.transform.smoothscale(canvas, (sw, sh)),
                        ((win_w - sw) // 2, (win_h - sh) // 2))
        pygame.display.flip()
        clock.tick(SEND_HZ)

    try:
        sock.sendto(b"L:90,R:90,W:90,E:0", (ESP32_IP, ESP32_PORT))
    except OSError:
        pass
    pygame.quit()


if __name__ == "__main__":
    main()
