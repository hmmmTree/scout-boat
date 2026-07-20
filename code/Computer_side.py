"""Boat driver station — drive, navigation, camera and 3D attitude view.

Keys / controls:
  F11        fullscreen toggle          SPACE / Square   arm-disarm
  Arrows or L1/R1  speed limit          ESC              exit fullscreen / quit
  TRI/O/X    winch in/stop/out          C                clear waypoints
  Map: click = add waypoint, drag one = move it, right-click one = delete,
       Ctrl+Z = undo, C = clear all, scroll = zoom
  3D view: drag to orbit

Network protocol (UDP to 192.168.4.1:4210):
  send: "L:<0-180>,R:<0-180>,W:<0-180>,E:<0|1>"     50x per second
  recv: "OK"                                        plain ack (current firmware)
        "OK T:<lat>,<lon>,<hdg>,<sats>[,<spd_ms>[,<pitch>,<roll>]]"
        ack with telemetry once the M10 GPS (+ 9-axis IMU) firmware is in:
        map, compass, speed, heading, 3D attitude and missions activate
        automatically when the fields appear.

Camera: set CAMERA_URL to the ESP32-CAM MJPEG stream (default is the
standard ESP32-CAM AP-mode stream address). Panel shows NO CAMERA until
frames arrive.

Windows note: "No internet" on the BoatControl network is normal — the boat
is an access point with no internet behind it.

Requires: pygame-ce (or pygame), numpy.
The 3D view loads assets/boat_mesh.npz — converted from "Twin v2.step".
"""
import io
import math
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque

import numpy as np
import pygame

# Give the 50Hz network thread timely GIL slices even while the 3D
# renderer is busy (default switch interval is 5ms, too coarse here).
sys.setswitchinterval(0.001)

ESP32_IP   = "192.168.4.1"
ESP32_PORT = 4210
SEND_HZ    = 50
BOAT_SSID  = "BoatControl"
CAMERA_URL = "http://192.168.4.10:81/stream"  # XIAO ESP32-S3 Sense (boat_cam.ino)
MODEL_YAW_OFFSET = 0    # degrees; set 180 if the 3D boat renders stern-first

# ---- autonomous mode tuning ----
WP_RADIUS_M   = 3.0    # waypoint reached within this many meters
AUTO_CRUISE   = 0.55   # cruise throttle fraction while on course
AUTO_TURN_GAIN = 0.7   # how hard heading error steers
STICK_OVERRIDE = 0.25  # stick deflection that kicks AUTO back to TELEOP

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
W, H = 1600, 900
# default window size (the canvas letterbox-scales into whatever you resize to)
WIN_W, WIN_H = 1280, 720

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


# ---- camera: MJPEG reader thread, latest JPEG bytes in cam_frame[0] ----
cam_frame = [None]          # raw jpeg bytes
cam_status = ["connecting"]

def _camera_thread():
    while True:
        try:
            cam_status[0] = "connecting"
            req = urllib.request.Request(CAMERA_URL, headers={"User-Agent": "boat-ds"})
            with urllib.request.urlopen(req, timeout=4) as r:
                cam_status[0] = "live"
                buf = b""
                while True:
                    chunk = r.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
                    if start != -1 and end != -1:
                        cam_frame[0] = buf[start:end + 2]
                        buf = buf[end + 2:]
                    if len(buf) > 1_000_000:      # runaway garbage guard
                        buf = b""
        except Exception:
            cam_frame[0] = None
            cam_status[0] = "no camera"
            time.sleep(5)


# ---- network worker: fixed 50Hz command send + ack drain ----
# Runs in its own thread so the packet rate NEVER depends on GUI frame
# rate (the 3D render or a slow frame must not starve the boat of
# commands — the boat failsafes after 300ms of silence).
def _net_worker(sock, net, nav):
    period = 1.0 / SEND_HZ
    next_t = time.perf_counter()
    while net["run"]:
        next_t += period
        l, r, w, en = net["cmd"]
        try:
            sock.sendto(f"L:{l},R:{r},W:{w},E:{en}".encode(),
                        (ESP32_IP, ESP32_PORT))
        except OSError:
            pass
        try:
            while True:
                data, _ = sock.recvfrom(160)
                if data:
                    net["last_ack"] = time.time()
                    net["ack_count"] += 1
                    parse_ack(data, nav)
        except OSError:
            pass
        # absolute deadline: GIL waits inside a cycle shorten the next
        # sleep instead of stretching every cycle
        left = next_t - time.perf_counter()
        if left > 0:
            time.sleep(left)
        elif left < -period * 4:
            next_t = time.perf_counter()   # fell far behind; resync


# ---- GPS / IMU state (filled by T: telemetry from the boat) ----
class Nav:
    def __init__(self):
        self.has_fix = False
        self.lat = self.lon = 0.0
        self.heading = None          # degrees, None = unknown
        self.speed = None            # m/s over ground, None = unknown
        self.pitch = self.roll = None
        self.sats = 0
        self.origin = None
        self.trail = deque(maxlen=3000)
        self.last_update = 0.0

    def feed(self, lat, lon, heading, sats, speed=None, pitch=None, roll=None):
        self.lat, self.lon, self.sats = lat, lon, sats
        self.heading = heading
        self.speed = speed
        self.pitch, self.roll = pitch, roll
        self.has_fix = sats >= 4
        self.last_update = time.time()
        if self.has_fix:
            if self.origin is None:
                self.origin = (lat, lon)
            self.trail.append(self.to_local(lat, lon))

    def to_local(self, lat, lon):
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
    """Ack: 'OK' or 'OK T:<lat>,<lon>,<hdg>,<sats>[,<spd>[,<pitch>,<roll>]]'."""
    try:
        s = data.decode(errors="replace").strip()
    except Exception:
        return
    if not s.startswith("OK T:"):
        return
    p = s[5:].split(",")
    try:
        lat, lon, hdg, sats = float(p[0]), float(p[1]), float(p[2]), int(p[3])
        spd = float(p[4]) if len(p) > 4 else None
        pitch = float(p[5]) if len(p) > 6 else None
        roll = float(p[6]) if len(p) > 6 else None
        nav.feed(lat, lon, hdg, sats, spd, pitch, roll)
    except (ValueError, IndexError):
        pass


def autopilot_step(nav, waypoints, idx, speed_limit):
    """One guidance step toward waypoints[idx].

    Returns (left_cmd, right_cmd, new_idx, done, info_str). Station-side
    autopilot: uses GPS position + compass heading from telemetry and
    steers with the same differential mix as teleop.
    """
    bx, by = nav.trail[-1]
    tx, ty = waypoints[idx]
    dx, dy = tx - bx, ty - by
    dist = math.hypot(dx, dy)
    if dist < WP_RADIUS_M:
        idx += 1
        if idx >= len(waypoints):
            return 90, 90, idx, True, "MISSION COMPLETE"
        return 90, 90, idx, False, f"WP {idx} reached"
    bearing = math.degrees(math.atan2(dx, dy)) % 360.0   # 0 = north
    err = (bearing - nav.heading + 540.0) % 360.0 - 180.0
    turn = clamp(err / 60.0, -1.0, 1.0) * AUTO_TURN_GAIN
    if abs(err) > 70:
        fwd = 0.12                       # mostly rotate in place
    else:
        fwd = AUTO_CRUISE * max(0.25, math.cos(math.radians(err)))
    cap = clamp(min(speed_limit, 1.0), 0.0, 1.0)
    lp = clamp(fwd + turn, -1.0, 1.0) * cap
    rp = clamp(fwd - turn, -1.0, 1.0) * cap
    left_cmd  = int(clamp(90 + lp * 90, 0, 180))
    right_cmd = int(clamp(90 + rp * 90, 0, 180))
    return left_cmd, right_cmd, idx, False, \
        f"WP {idx + 1}/{len(waypoints)}  {dist:.0f}m  err {err:+.0f}°"


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
    text(surf, "med", label, x + w // 2, y + h // 2,
         accent if enabled else DIM, center=True)
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
    text(surf, "lbl", "WINCH", x + 12, y + h // 2 - 8, GREY)
    text(surf, "med", s, x + w - 70, y + h // 2 - 8, col)


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
    cx, cy = x + size // 2, y + size // 2 + 4
    rad = size // 2 - 22
    pygame.draw.circle(surf, PANEL2, (cx, cy), rad, width=2)
    for ang, name in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
        a = math.radians(ang - 90)
        tx = cx + int(math.cos(a) * (rad - 11))
        ty = cy + int(math.sin(a) * (rad - 11))
        text(surf, "lbl", name, tx, ty, RED if name == "N" else GREY, center=True)
    text(surf, "lbl", "COMPASS", x + size // 2, y + 11, GREY, center=True)
    if heading is None:
        text(surf, "big", "--", cx, cy, DIM, center=True)
    else:
        a = math.radians(heading - 90)
        hx = cx + int(math.cos(a) * (rad - 22))
        hy = cy + int(math.sin(a) * (rad - 22))
        pygame.draw.line(surf, CYAN, (cx, cy), (hx, hy), 3)
        pygame.draw.circle(surf, CYAN, (cx, cy), 4)


class MapView:
    """Local-frame map: meters relative to first GPS fix.

    Waypoint editing: click empty water = add, drag a waypoint = move it,
    right-click a waypoint = delete it, Ctrl+Z = undo any of the above.
    """

    HIT_PX = 13

    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self.m_per_px = 0.5
        self.waypoints = []          # [[x_m, y_m], ...]
        self._history = []
        self._drag_idx = None

    def world_to_px(self, wx, wy):
        return (int(self.rect.centerx + wx / self.m_per_px),
                int(self.rect.centery - wy / self.m_per_px))

    def px_to_world(self, px, py):
        return [(px - self.rect.centerx) * self.m_per_px,
                (self.rect.centery - py) * self.m_per_px]

    def zoom(self, direction):
        self.m_per_px = clamp(self.m_per_px * (0.8 if direction > 0 else 1.25),
                              0.05, 10.0)

    # -- undo plumbing: snapshot before every mutation --
    def _push(self):
        self._history.append([wp.copy() for wp in self.waypoints])
        if len(self._history) > 100:
            self._history.pop(0)

    def undo(self):
        if self._history:
            self.waypoints = self._history.pop()
            self._drag_idx = None

    def clear_all(self):
        if self.waypoints:
            self._push()
            self.waypoints = []

    def _hit(self, pos):
        for i, wp in enumerate(self.waypoints):
            px, py = self.world_to_px(*wp)
            if (px - pos[0]) ** 2 + (py - pos[1]) ** 2 <= self.HIT_PX ** 2:
                return i
        return None

    def mouse_down(self, pos, btn):
        if not self.rect.collidepoint(pos):
            return False
        i = self._hit(pos)
        if btn == 1:
            self._push()
            if i is not None:
                self._drag_idx = i           # grab existing waypoint
            else:
                self.waypoints.append(self.px_to_world(*pos))
        elif btn == 3 and i is not None:
            self._push()
            self.waypoints.pop(i)
        return True

    def mouse_move(self, pos):
        if self._drag_idx is not None and self._drag_idx < len(self.waypoints):
            self.waypoints[self._drag_idx] = self.px_to_world(
                clamp(pos[0], self.rect.left + 6, self.rect.right - 6),
                clamp(pos[1], self.rect.top + 6, self.rect.bottom - 6))

    def mouse_up(self):
        self._drag_idx = None

    def draw(self, surf, nav, target_idx=None):
        r = self.rect
        panel(surf, r.x, r.y, r.w, r.h, (16, 19, 26))
        clip_prev = surf.get_clip()
        surf.set_clip(r.inflate(-4, -4))

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

        hx, hy = self.world_to_px(0, 0)
        pygame.draw.circle(surf, YELLOW, (hx, hy), 6, width=2)
        text(surf, "lbl", "HOME", hx, hy - 14, YELLOW, center=True)

        pts = [self.world_to_px(*wp) for wp in self.waypoints]
        if pts:
            pygame.draw.lines(surf, (60, 90, 140), False, [(hx, hy)] + pts, 2)
        for i, p in enumerate(pts):
            grabbed = (i == self._drag_idx)
            active = (i == target_idx)
            pygame.draw.circle(surf, YELLOW if grabbed else BLUE, p, 9 if grabbed else 7)
            pygame.draw.circle(surf, WHITE, p, 9 if grabbed else 7, width=1)
            if active:
                pygame.draw.circle(surf, ORANGE, p, 13, width=2)
            text(surf, "lbl", str(i + 1), p[0], p[1], BG if grabbed else WHITE, center=True)
        # guidance line: boat -> active target
        if target_idx is not None and target_idx < len(pts) and nav.trail:
            bp = self.world_to_px(*nav.trail[-1])
            pygame.draw.line(surf, ORANGE, bp, pts[target_idx], 2)

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
            text(surf, "med", "NO GPS — planning grid",
                 r.centerx, r.top + 20, DIM, center=True)

        text(surf, "lbl", f"grid {nice} m   scroll = zoom",
             r.left + 12, r.bottom - 22, GREY)
        surf.set_clip(clip_prev)


class Boat3D:
    """Software-rendered 3D view of the Twin v2 CAD mesh.

    Yaw follows the boat's heading; pitch/roll follow the IMU when present.
    Drag inside the panel to orbit the view. Without telemetry the model
    idles on a slow turntable so orientation is still inspectable.
    """

    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self.ok = False
        here = os.path.dirname(os.path.abspath(__file__))
        for cand in (os.path.join(here, "assets", "boat_mesh.npz"),
                     os.path.join(here, "..", "assets", "boat_mesh.npz")):
            if os.path.exists(cand):
                d = np.load(cand)
                self.verts = d["verts"]
                self.faces = d["faces"]
                self.normals = d["normals"]
                self.edges = d["edges"]        # (E,2) vertex ids
                self.eadj = d["eadj"]          # (E,2) adjacent face ids (-1 = none)
                self.ecrease = d["ecrease"]    # (E,) bool: sharp/boundary edge
                # per-face edge ids (uniq is lexicographically sorted, so
                # rebuilding with np.unique reproduces the same edge order)
                e = np.sort(self.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
                _, inv = np.unique(e, axis=0, return_inverse=True)
                self.face_edges = inv.reshape(-1, 3)
                self.ok = True
                break
        self.view_yaw = 0.0        # user orbit offset
        self.view_pitch = 26.0
        self.dragging = False
        self.last_mouse = (0, 0)
        self.idle = 0.0
        # rendering happens on a worker thread (a full render takes ~30ms,
        # which would drag the 50Hz control loop down to ~15Hz otherwise)
        self.cache = None
        self._want = None
        self._done = None
        self._lock = threading.Lock()
        if self.ok:
            threading.Thread(target=self._worker, daemon=True).start()

    def _worker(self):
        while True:
            with self._lock:
                want = self._want
                done = self._done
            if want is None or want == done:
                time.sleep(0.01)
                continue
            t0 = time.perf_counter()
            surf = self._render(*want)
            with self._lock:
                self.cache = surf
                self._done = want
            # cap render rate so this thread leaves the GIL mostly free
            # for the control loop
            time.sleep(max(0.02, 0.15 - (time.perf_counter() - t0)))

    def mouse_down(self, pos):
        if self.rect.collidepoint(pos):
            self.dragging = True
            self.last_mouse = pos
            return True
        return False

    def mouse_up(self):
        self.dragging = False

    def mouse_move(self, pos):
        if self.dragging:
            dx = pos[0] - self.last_mouse[0]
            dy = pos[1] - self.last_mouse[1]
            self.view_yaw = (self.view_yaw + dx * 0.6) % 360
            self.view_pitch = clamp(self.view_pitch + dy * 0.4, 5, 80)
            self.last_mouse = pos

    def draw(self, surf, nav, dt):
        r = self.rect
        panel(surf, r.x, r.y, r.w, r.h, (16, 19, 26))
        text(surf, "lbl", "BOAT 3D — TWIN V2", r.x + 12, r.y + 10, GREY)
        if not self.ok:
            text(surf, "med", "boat_mesh.npz missing", r.centerx, r.centery, DIM, center=True)
            return

        live = nav.alive and nav.heading is not None
        if live:
            yaw = nav.heading + MODEL_YAW_OFFSET
            pitch = nav.pitch or 0.0
            roll = nav.roll or 0.0
        else:
            self.idle = (self.idle + dt * 12.0) % 360
            yaw, pitch, roll = self.idle, 0.0, 0.0

        with self._lock:
            self._want = (r.w, r.h, round(yaw), round(pitch), round(roll),
                          round(self.view_yaw), round(self.view_pitch))
            cache = self.cache
        if cache is not None:
            surf.blit(cache, (r.x, r.y))

        if live:
            info = f"HDG {int(yaw) % 360:03d}°"
            if nav.pitch is not None:
                info += f"   P {nav.pitch:+.0f}°  R {nav.roll:+.0f}°"
            text(surf, "med", info, r.x + 12, r.bottom - 26, CYAN)
        else:
            text(surf, "lbl", "NO TELEMETRY — turntable view, drag to orbit",
                 r.x + 12, r.bottom - 24, DIM)

    def _render(self, w, h, yaw, pitch, roll, view_yaw=None, view_pitch=None):
        if view_yaw is None:
            view_yaw = self.view_yaw
        if view_pitch is None:
            view_pitch = self.view_pitch
        out = pygame.Surface((w, h), pygame.SRCALPHA)

        def rot(axis, deg):
            c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            if axis == "x":
                return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
            if axis == "y":
                return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        # model attitude (compass yaw is clockwise -> -yaw about Z), then view
        R = rot("x", view_pitch) @ rot("z", view_yaw) \
            @ rot("z", -yaw) @ rot("x", pitch) @ rot("y", roll)
        v = self.verts @ R.T
        n = self.normals @ R.T
        s = min(w, h) * 0.62
        px = w / 2 + v[:, 0] * s
        py = h / 2 + 6 - v[:, 2] * s
        depth = v[self.faces].mean(axis=1)[:, 1]
        vis = n[:, 1] < 0
        order = np.argsort(-depth)
        vis_order = order[vis[order]]        # back-to-front, culled up front
        lightv = np.array([0.35, -0.55, 0.76])
        lum = np.clip(-(n @ lightv), 0, 1)
        cols = (26 + 22 * lum).astype(np.int32)

        # Outline style: near-flat dark fill (only there to hide what's
        # behind it), with silhouette + sharp-crease edges drawn on top.
        a0 = np.where(self.eadj[:, 0] >= 0, self.eadj[:, 0], 0)
        a1 = np.where(self.eadj[:, 1] >= 0, self.eadj[:, 1], self.eadj[:, 0])
        edge_on = self.ecrease | (vis[a0] != vis[a1])
        emask = edge_on[self.face_edges]     # (F,3): which edges to draw per face
        eany = emask.any(axis=1)
        EDGE_COL = (152, 178, 208)
        faces = self.faces
        fedges = self.face_edges
        edges = self.edges
        for fi in vis_order:
            f = faces[fi]
            c = cols[fi]
            pygame.draw.polygon(out, (c, c + 4, c + 10),
                                ((px[f[0]], py[f[0]]),
                                 (px[f[1]], py[f[1]]),
                                 (px[f[2]], py[f[2]])))
            if eany[fi]:
                for k in range(3):
                    if emask[fi, k]:
                        e0, e1 = edges[fedges[fi, k]]
                        pygame.draw.line(out, EDGE_COL,
                                         (px[e0], py[e0]), (px[e1], py[e1]))
        return out


def draw_camera(surf, rect, decoded):
    x, y, w, h = rect
    panel(surf, x, y, w, h, (16, 19, 26))
    text(surf, "lbl", "CAMERA", x + 12, y + 10, GREY)
    if decoded is not None:
        iw, ih = decoded.get_size()
        scale = min((w - 16) / iw, (h - 40) / ih)
        img = pygame.transform.smoothscale(decoded, (int(iw * scale), int(ih * scale)))
        surf.blit(img, (x + (w - img.get_width()) // 2,
                        y + 28 + (h - 40 - img.get_height()) // 2))
        text(surf, "lbl", "LIVE", x + w - 46, y + 10, RED)
    else:
        text(surf, "med", "NO CAMERA", x + w // 2, y + h // 2 - 8, DIM, center=True)
        text(surf, "lbl", CAMERA_URL, x + w // 2, y + h // 2 + 16, DIM, center=True)


def tile(surf, x, y, w, h, label, value, unit="", col=WHITE):
    panel(surf, x, y, w, h)
    text(surf, "lbl", label, x + 14, y + 10, GREY)
    text(surf, "huge", value, x + 14, y + 28, col)
    if unit:
        text(surf, "med", unit, x + 14 + F("huge").size(value)[0] + 8, y + 42, GREY)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    threading.Thread(target=_ssid_poller, daemon=True).start()
    threading.Thread(target=_camera_thread, daemon=True).start()

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
    pygame.display.set_caption("BOAT DRIVER STATION")
    canvas = pygame.Surface((W, H))
    clock = pygame.time.Clock()
    fullscreen = False

    FONTS.update(
        huge=pygame.font.SysFont("Consolas", 30, bold=True),
        big=pygame.font.SysFont("Consolas", 23, bold=True),
        med=pygame.font.SysFont("Consolas", 17, bold=True),
        sm=pygame.font.SysFont("Consolas", 15),
        lbl=pygame.font.SysFont("Consolas", 12, bold=True),
    )

    nav = Nav()
    mapview = MapView((472, 96, 610, 580))
    boat3d = Boat3D((1098, 400, 478, 330))
    cam_rect = (1098, 96, 478, 292)

    js = None
    speed = 1.0
    motors_on = False              # SAFETY: always start disarmed
    mode = "TELEOP"                # "TELEOP" | "AUTO"
    wp_index = 0
    auto_info = ""
    winch_cmd = 90
    w_cmd = 90
    left_cmd = right_cmd = 90
    prev_square = prev_l1 = prev_r1 = False
    disconnect_time = 0.0
    cam_decoded = None
    cam_last_decode = 0.0
    running = True
    connected = False

    net = {"run": True, "cmd": (90, 90, 90, 0), "last_ack": 0.0, "ack_count": 0}
    threading.Thread(target=_net_worker, args=(sock, net, nav), daemon=True).start()

    def to_canvas(pos):
        win_w, win_h = screen.get_size()
        scale = min(win_w / W, win_h / H)
        ox = (win_w - W * scale) / 2
        oy = (win_h - H * scale) / 2
        return ((pos[0] - ox) / scale, (pos[1] - oy) / scale)

    btn_start = pygame.Rect(0, 0, 0, 0)
    btn_clear = pygame.Rect(0, 0, 0, 0)

    def auto_ready():
        return (motors_on and connected and mapview.waypoints
                and nav.alive and nav.has_fix and nav.heading is not None
                and len(nav.trail) > 0)

    def toggle_mode():
        nonlocal mode, wp_index, auto_info
        if mode == "AUTO":
            mode = "TELEOP"
            auto_info = ""
        elif auto_ready():
            mode = "AUTO"
            wp_index = 0
            auto_info = "engaging"

    while running:
        dt = clock.get_time() / 1000.0
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_F11:
                    fullscreen = not fullscreen
                    screen = pygame.display.set_mode(
                        (0, 0) if fullscreen else (WIN_W, WIN_H),
                        pygame.FULLSCREEN if fullscreen else pygame.RESIZABLE)
                elif e.key == pygame.K_ESCAPE:
                    if fullscreen:
                        fullscreen = False
                        screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
                    else:
                        running = False
                elif e.key in (pygame.K_UP, pygame.K_RIGHT):
                    speed = clamp(round(speed + 0.1, 1), 0.0, 2.0)
                elif e.key in (pygame.K_DOWN, pygame.K_LEFT):
                    speed = clamp(round(speed - 0.1, 1), 0.0, 2.0)
                elif e.key == pygame.K_SPACE:
                    motors_on = not motors_on
                elif e.key == pygame.K_m:
                    toggle_mode()
                elif e.key == pygame.K_c:
                    mapview.clear_all()
                    mode = "TELEOP"
                elif e.key == pygame.K_z and (e.mod & (pygame.KMOD_CTRL | pygame.KMOD_META)):
                    mapview.undo()
            elif e.type == pygame.MOUSEBUTTONDOWN:
                cpos = to_canvas(e.pos)
                if e.button in (4, 5):
                    if mapview.rect.collidepoint(cpos):
                        mapview.zoom(1 if e.button == 4 else -1)
                elif e.button == 1 and boat3d.mouse_down(cpos):
                    pass
                elif btn_start.collidepoint(cpos):
                    toggle_mode()
                elif btn_clear.collidepoint(cpos):
                    mapview.clear_all()
                    mode = "TELEOP"
                else:
                    mapview.mouse_down(cpos, e.button)
            elif e.type == pygame.MOUSEBUTTONUP:
                boat3d.mouse_up()
                mapview.mouse_up()
            elif e.type == pygame.MOUSEMOTION:
                cpos = to_canvas(e.pos)
                boat3d.mouse_move(cpos)
                mapview.mouse_move(cpos)
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
                mode = "TELEOP"          # disarm always exits AUTO

            # ---- AUTO mode: station-side autopilot drives L/R ----
            if mode == "AUTO":
                if abs(forward) > STICK_OVERRIDE or abs(turn) > STICK_OVERRIDE:
                    mode = "TELEOP"       # pilot grabbed the sticks
                    auto_info = "stick override"
                elif not (nav.alive and nav.has_fix and nav.heading is not None
                          and nav.trail and wp_index < len(mapview.waypoints)):
                    left_cmd = right_cmd = 90   # telemetry lost: hold neutral
                    auto_info = "AUTO PAUSED — no telemetry"
                else:
                    left_cmd, right_cmd, wp_index, done, auto_info = \
                        autopilot_step(nav, mapview.waypoints, wp_index, speed)
                    if done:
                        mode = "TELEOP"
        else:
            left_cmd = right_cmd = 90
            mode = "TELEOP"               # controller is the deadman switch
            if time.time() - disconnect_time < TIMEOUT:
                w_cmd = winch_cmd
            else:
                w_cmd = 90
                winch_cmd = 90

        # hand the command to the 50Hz network thread (E:0 = instant stop)
        net["cmd"] = (left_cmd, right_cmd, w_cmd, 1 if motors_on else 0)
        last_ack = net["last_ack"]
        ack_count = net["ack_count"]
        boat_ok = (time.time() - last_ack) < 1.0

        # decode at most ~15 camera frames/sec
        if cam_frame[0] is not None and time.time() - cam_last_decode > 1 / 15:
            try:
                cam_decoded = pygame.image.load(io.BytesIO(cam_frame[0]))
            except Exception:
                cam_decoded = None
            cam_last_decode = time.time()
        if cam_status[0] != "live":
            cam_decoded = None

        lx = safe_axis(js, 0) if connected else 0.0
        ly = safe_axis(js, 1) if connected else 0.0
        rx = safe_axis(js, 2) if connected else 0.0
        ry = safe_axis(js, 3) if connected else 0.0

        # ================= draw =================
        canvas.fill(BG)
        text(canvas, "huge", "BOAT DRIVER STATION", 24, 16, WHITE)
        text(canvas, "lbl", "F11 FULLSCREEN", W - 126, 28, DIM)
        pygame.draw.line(canvas, OUTLINE, (24, 60), (W - 24, 60), 1)

        ssid = current_ssid[0]
        on_boat_wifi = (ssid == BOAT_SSID)
        wifi_detail = ssid if len(ssid) <= 14 else ssid[:13] + "…"

        # -- left column: drive --
        pill(canvas, 24, 96, 208, 42, "CONTROLLER", connected,
             "OK" if connected else "NOT FOUND")
        pill(canvas, 240, 96, 208, 42, "WIFI", on_boat_wifi, wifi_detail)
        age = time.time() - last_ack
        pill(canvas, 24, 146, 208, 42, "BOAT LINK", boat_ok,
             f"OK {int(age*1000)}ms" if boat_ok else "LOST")
        pill(canvas, 240, 146, 208, 42, "GPS",
             nav.alive and nav.has_fix,
             f"{nav.sats} SATS" if (nav.alive and nav.has_fix) else "NO FIX")

        arm_banner(canvas, 24, 200, 424, 62, motors_on)
        speed_bar(canvas, 24, 270, 424, 56, speed)

        motor_bar(canvas, 24, 338, 96, 262, left_cmd, "LEFT", motors_on)
        motor_bar(canvas, 128, 338, 96, 262, right_cmd, "RIGHT", motors_on)
        stick_box(canvas, 232, 338, 126, lx, ly, "L STICK")
        stick_box(canvas, 232, 474, 126, rx, ry, "R STICK")
        compass(canvas, 366, 338, 82, None if not (nav.alive and nav.heading is not None)
                else nav.heading)
        panel(canvas, 366, 428, 82, 172)
        text(canvas, "lbl", "OUT", 380, 440, GREY)
        text(canvas, "sm", f"L {pct(left_cmd):+d}", 380, 462, WHITE)
        text(canvas, "sm", f"R {pct(right_cmd):+d}", 380, 484, WHITE)
        text(canvas, "sm", f"W {pct(w_cmd):+d}", 380, 506, WHITE)
        text(canvas, "sm", f"ack", 380, 540, DIM)
        text(canvas, "sm", f"{ack_count}", 380, 560, GREY)

        winch_ind(canvas, 24, 608, 424, 44, w_cmd)

        if not on_boat_wifi:
            text(canvas, "sm",
                 f'Join WiFi "{BOAT_SSID}" — "No internet" there is normal.',
                 24, 664, ORANGE)
        elif not boat_ok:
            text(canvas, "sm", "On boat WiFi but no reply — boat powered?",
                 24, 664, ORANGE)

        # -- middle column: navigation --
        text(canvas, "lbl", "NAVIGATION", 472, 74, GREY)
        mapview.draw(canvas, nav,
                     wp_index if mode == "AUTO" and wp_index < len(mapview.waypoints)
                     else None)
        if mode == "AUTO":
            text(canvas, "med", "AUTO", mapview.rect.right - 46,
                 mapview.rect.top + 20, ORANGE, center=True)

        bar_y = 96 + 580 + 8
        panel(canvas, 472, bar_y, 610, 44)
        wp_n = len(mapview.waypoints)
        in_auto = (mode == "AUTO")
        # mode chip
        pygame.draw.rect(canvas, PANEL2, (482, bar_y + 6, 118, 32), border_radius=8)
        text(canvas, "med", mode, 482 + 59, bar_y + 22,
             ORANGE if in_auto else BLUE, center=True)
        text(canvas, "med", f"WP: {wp_n}", 616, bar_y + 13, WHITE)
        btn_start = button(canvas, (688, bar_y + 6, 156, 32),
                           "STOP AUTO (M)" if in_auto else "START AUTO (M)",
                           enabled=in_auto or auto_ready(),
                           accent=RED if in_auto else GREEN)
        btn_clear = button(canvas, (854, bar_y + 6, 100, 32), "CLEAR (C)")
        if in_auto:
            text(canvas, "lbl", auto_info, 966, bar_y + 17, ORANGE)
        elif not auto_ready():
            need = "needs: "
            if not motors_on: need += "ARM "
            if not connected: need += "controller "
            if not (nav.alive and nav.has_fix): need += "GPS "
            elif nav.heading is None: need += "heading "
            if not wp_n: need += "waypoints"
            text(canvas, "lbl", need.strip(), 966, bar_y + 17, DIM)

        # tiles under mission bar: SPEED / HEADING / POSITION
        t_y = bar_y + 52
        spd = nav.speed if (nav.alive and nav.speed is not None and nav.has_fix) else None
        tile(canvas, 472, t_y, 196, 78, "SPEED (GPS)",
             f"{spd:.1f}" if spd is not None else "--",
             "m/s" if spd is not None else "", CYAN if spd is not None else DIM)
        hdg = nav.heading if (nav.alive and nav.heading is not None) else None
        tile(canvas, 676, t_y, 196, 78, "HEADING",
             f"{int(hdg) % 360:03d}°" if hdg is not None else "--",
             "", CYAN if hdg is not None else DIM)
        panel(canvas, 880, t_y, 202, 78)
        text(canvas, "lbl", "POSITION", 894, t_y + 10, GREY)
        if nav.alive and nav.has_fix:
            text(canvas, "sm", f"{nav.lat:+.6f}", 894, t_y + 30, WHITE)
            text(canvas, "sm", f"{nav.lon:+.6f}", 894, t_y + 50, WHITE)
        else:
            text(canvas, "med", "--", 894, t_y + 36, DIM)

        # -- right column: camera + 3D --
        text(canvas, "lbl", "CAMERA / MODEL", 1098, 74, GREY)
        draw_camera(canvas, cam_rect, cam_decoded)
        boat3d.draw(canvas, nav, dt)

        panel(canvas, 1098, 738, 478, 62)
        text(canvas, "lbl", "SENSORS", 1112, 748, GREY)
        text(canvas, "sm",
             "GPS: u-blox M10   IMU: 9-axis   CAM: XIAO S3 Sense",
             1112, 768, GREY)

        text(canvas, "sm",
             "SQUARE|SPACE arm   M auto/teleop (sticks always override)   "
             "L1/R1|Arrows speed   TRI/O/X winch   "
             "map: click add / drag move / r-click del / Ctrl+Z undo   F11",
             24, H - 26, GREY)

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

    net["run"] = False
    time.sleep(0.05)
    try:
        sock.sendto(b"L:90,R:90,W:90,E:0", (ESP32_IP, ESP32_PORT))
    except OSError:
        pass
    pygame.quit()


if __name__ == "__main__":
    main()
