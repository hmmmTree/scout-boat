import socket
import time
import pygame

ESP32_IP   = "192.168.4.1"
ESP32_PORT = 4210
SEND_HZ    = 50

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


W, H = 920, 600


BG     = (18, 20, 26)
PANEL  = (30, 34, 42)
PANEL2 = (40, 45, 56)
WHITE  = (236, 239, 245)
GREY   = (128, 136, 150)
GREEN  = (74, 222, 128)
RED    = (239, 80, 80)
ORANGE = (251, 146, 60)
BLUE   = (96, 165, 250)
YELLOW = (250, 204, 21)

F_BIG = F_MED = F_SM = F_LBL = None 


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


def safe_hat_x(js):
    try:
        return js.get_hat(0)[0] if js.get_numhats() > 0 else 0
    except Exception:
        return 0



def text(surf, font, s, x, y, color=WHITE, center=False):
    img = font.render(s, True, color)
    r = img.get_rect()
    if center:
        r.center = (x, y)
    else:
        r.topleft = (x, y)
    surf.blit(img, r)


def panel(surf, x, y, w, h):
    pygame.draw.rect(surf, PANEL, (x, y, w, h), border_radius=10)


def pill(surf, x, y, w, h, label, ok):
    pygame.draw.rect(surf, PANEL2, (x, y, w, h), border_radius=8)
    pygame.draw.circle(surf, GREEN if ok else RED, (x + 20, y + h // 2), 8)
    text(surf, F_MED, label, x + 38, y + h // 2, WHITE)
    state = "OK" if ok else "LOST"
    text(surf, F_MED, state, x + w - 46, y + h // 2, GREEN if ok else RED, center=True)


def motor_bar(surf, x, y, w, h, cmd, label):
    panel(surf, x, y, w, h)
    cx = x + w // 2
    mid = y + h // 2
    pygame.draw.line(surf, GREY, (x + 10, mid), (x + w - 10, mid), 2)
    dev = (cmd - 90) / 90.0
    span = h // 2 - 28
    bar_h = int(abs(dev) * span)
    bw = w - 28
    if dev >= 0:
        pygame.draw.rect(surf, GREEN, (x + 14, mid - bar_h, bw, bar_h), border_radius=4)
    else:
        pygame.draw.rect(surf, ORANGE, (x + 14, mid, bw, bar_h), border_radius=4)
    text(surf, F_LBL, label, cx, y + 14, GREY, center=True)
    text(surf, F_SM, str(pct(cmd)), cx, y + h - 16, WHITE, center=True)


def stick_box(surf, x, y, size, ax, ay, label):
    panel(surf, x, y, size, size)
    cx, cy = x + size // 2, y + size // 2
    pygame.draw.line(surf, PANEL2, (x + 10, cy), (x + size - 10, cy), 1)
    pygame.draw.line(surf, PANEL2, (cx, y + 10), (cx, y + size - 10), 1)
    rad = size // 2 - 16
    dx = cx + int(clamp(ax, -1, 1) * rad)
    dy = cy + int(clamp(ay, -1, 1) * rad)
    pygame.draw.circle(surf, BLUE, (dx, dy), 9)
    text(surf, F_LBL, label, cx, y + size - 16, GREY, center=True)


def speed_bar(surf, x, y, w, h, speed, max_speed=2.0):
    panel(surf, x, y, w, h)
    pad = 14
    inner = w - 2 * pad
    text(surf, F_LBL, "SPEED", x + pad, y + 12, GREY)
    text(surf, F_MED, f"{speed:.1f}x", x + pad + 76, y + 8, YELLOW)
    track_y = y + h - 22
    pygame.draw.rect(surf, PANEL2, (x + pad, track_y, inner, 12), border_radius=6)
    fill = int(inner * clamp(speed / max_speed, 0.0, 1.0))
    pygame.draw.rect(surf, YELLOW, (x + pad, track_y, fill, 12), border_radius=6)


def winch_ind(surf, x, y, w, h, w_cmd):
    panel(surf, x, y, w, h)
    if w_cmd > 100:
        s, col = "IN  ^", GREEN
    elif w_cmd < 80:
        s, col = "OUT v", ORANGE
    else:
        s, col = "STOP", GREY
    text(surf, F_LBL, "WINCH", x + 14, y + 12, GREY)
    text(surf, F_MED, s, x + w // 2, y + h - 18, col, center=True)


def main():
    global F_BIG, F_MED, F_SM, F_LBL

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("BOAT DRIVER STATION")
    clock = pygame.time.Clock()

    F_BIG = pygame.font.SysFont("Consolas", 34, bold=True)
    F_MED = pygame.font.SysFont("Consolas", 20, bold=True)
    F_SM  = pygame.font.SysFont("Consolas", 17)
    F_LBL = pygame.font.SysFont("Consolas", 13, bold=True)

    js = None
    speed = 1.0
    motors_on = True
    winch_cmd = 90
    w_cmd = 90
    left_cmd = right_cmd = 90
    prev_square = False
    prev_l1 = False
    prev_r1 = False
    last_ack = 0.0
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


        try:
            sock.sendto(f"L:{left_cmd},R:{right_cmd},W:{w_cmd}".encode(),
                        (ESP32_IP, ESP32_PORT))
        except OSError:
            pass
        try:
            while True:
                data, _ = sock.recvfrom(32)
                if data:
                    last_ack = time.time()
        except OSError:
            pass
        boat_ok = (time.time() - last_ack) < 1.0

        # ---- stick positions for display ----
        lx = safe_axis(js, 0) if connected else 0.0
        ly = safe_axis(js, 1) if connected else 0.0
        rx = safe_axis(js, 2) if connected else 0.0
        ry = safe_axis(js, 3) if connected else 0.0

        screen.fill(BG)
        text(screen, F_BIG, "BOAT DRIVER STATION", 30, 26, WHITE)

        pill(screen, 30, 64, 270, 46, "CONTROLLER", connected)
        pill(screen, 318, 64, 270, 46, "BOAT LINK", boat_ok)

        panel(screen, 606, 64, 284, 46)
        text(screen, F_MED,
             "MOTORS " + ("ENABLED" if motors_on else "DISABLED"),
             606 + 142, 64 + 23, GREEN if motors_on else RED, center=True)

        speed_bar(screen, 30, 126, 558, 60, speed)
        winch_ind(screen, 606, 126, 284, 60, w_cmd)

        motor_bar(screen, 30, 206, 110, 320, left_cmd, "LEFT")
        motor_bar(screen, 152, 206, 110, 320, right_cmd, "RIGHT")

        stick_box(screen, 300, 216, 180, lx, ly, "LEFT STICK")
        stick_box(screen, 498, 216, 180, rx, ry, "RIGHT STICK")

        panel(screen, 700, 216, 190, 180)
        text(screen, F_LBL, "OUTPUT", 716, 230, GREY)
        text(screen, F_SM, f"L : {pct(left_cmd)}", 716, 256, WHITE)
        text(screen, F_SM, f"R : {pct(right_cmd)}", 716, 282, WHITE)
        text(screen, F_SM, f"W : {pct(w_cmd)}", 716, 308, WHITE)
        text(screen, F_SM, f"-> {ESP32_IP}", 716, 350, GREY)

        text(screen, F_SM,
             "Controller: L1/R1 = speed   Square = kill    |    Keys: Arrows = speed  Space = kill  Esc = quit",
             30, 562, GREY)

        pygame.display.flip()
        clock.tick(SEND_HZ)


    try:
        sock.sendto(b"L:90,R:90,W:90", (ESP32_IP, ESP32_PORT))
    except OSError:
        pass
    pygame.quit()


if __name__ == "__main__":
    main()
