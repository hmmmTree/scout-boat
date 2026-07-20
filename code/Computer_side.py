"""Boat driver station — drive, navigation, camera and 3D attitude view.

Keys / controls:
  F11        fullscreen toggle          SPACE / Square   arm-disarm
  Arrows or L1/R1  speed limit          ESC              exit fullscreen / quit
  I / K / O  winch in/stop/out          M / Options      AUTO <-> TELEOP
  R2/L2 hold winch in/out (momentary)   H                controls reference
  E          EDIT MODE (keyboard only)  F / Share        re-follow boat on map
  Map (always): drag = pan, scroll = zoom.  OSM tiles cache to assets/tiles
  and work offline once downloaded (Philippines overview pre-cached).
  EDIT MODE only: click = add/select waypoint, drag one = move it,
  right-click = delete, DEL = delete selected, [ / ] = hold time -/+5s,
  Ctrl+Z = undo, C = clear all.  Purple pin = this computer (IP location).
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

Requires: pygame-ce (or pygame), numpy. Optional: pyserial for the wired
USB link — if the boat is plugged in and WiFi is silent for 2s, commands
switch to the USB cable automatically (BOAT LINK pill shows "USB");
unplugging falls back to WiFi. Same protocol either way.
The 3D view loads assets/boat_mesh.npz — converted from "Twin v2.step".
"""
import io
import json
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
HOLD_STEP_S   = 5      # [ / ] adjust a waypoint's hold time by this many seconds

# ---- map / tiles ----
TILESIZE = 256
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
TILE_UA  = "ScoutBoatDriverStation/1.0 (personal robotics project)"
PH_CENTER = (12.8797, 121.7740)     # Philippines overview
_here = os.path.dirname(os.path.abspath(__file__))
ASSET_DIR = (os.path.join(_here, "assets")
             if os.path.isdir(os.path.join(_here, "assets"))
             else os.path.normpath(os.path.join(_here, "..", "assets")))


def _ll_to_world(lat, lon, z):
    """lat/lon -> Web Mercator pixel coords at zoom z."""
    n = (1 << z) * TILESIZE
    x = (lon + 180.0) / 360.0 * n
    lr = math.radians(clamp(lat, -85.05, 85.05))
    y = (1.0 - math.log(math.tan(lr) + 1.0 / math.cos(lr)) / math.pi) / 2.0 * n
    return x, y


def _world_to_ll(x, y, z):
    n = (1 << z) * TILESIZE
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


def geo_dist_bearing(lat1, lon1, lat2, lon2):
    """meters + compass bearing between nearby points (equirectangular)."""
    x = (lon2 - lon1) * 111320.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
    y = (lat2 - lat1) * 110540.0
    return math.hypot(x, y), math.degrees(math.atan2(x, y)) % 360.0


# ---- computer location (IP geolocation, city precision; cached offline) ----
pc_loc = [None]

def load_pc_cache():
    try:
        return json.load(open(os.path.join(ASSET_DIR, "pc_location.json")))
    except Exception:
        return None

def _pc_locator():
    pc_loc[0] = load_pc_cache()
    try:
        with urllib.request.urlopen(
                "http://ip-api.com/json/?fields=status,lat,lon,city", timeout=8) as r:
            d = json.loads(r.read().decode())
        if d.get("status") == "success":
            pc_loc[0] = {"lat": d["lat"], "lon": d["lon"], "city": d.get("city", "")}
            os.makedirs(ASSET_DIR, exist_ok=True)
            json.dump(pc_loc[0], open(os.path.join(ASSET_DIR, "pc_location.json"), "w"))
    except Exception:
        pass

AXIS_LY = 1
AXIS_RX = 2
AXIS_L2 = 4            # analog triggers: rest = -1, pressed = +1
AXIS_R2 = 5
BTN_CROSS    = 0
BTN_CIRCLE   = 1
BTN_SQUARE   = 2
BTN_TRIANGLE = 3
BTN_SHARE    = 4
BTN_OPTIONS  = 6
BTN_L1       = 9
BTN_R1       = 10
DEADZONE = 0.15
TRIGGER_ON = 0.25      # trigger travel (0..1) that counts as pressed
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
#
# Transports: WiFi UDP by default; if no acks arrive for 2s and a boat
# is plugged in over USB (CP210x/CH340 serial), the worker switches to
# the cable — same protocol, one line per command. Unplugging falls
# back to WiFi automatically. net["transport"] says which is active.
def _find_boat_port():
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        desc = (p.description or "").upper()
        if p.vid in (0x10C4, 0x1A86) or "CP210" in desc or "CH340" in desc \
                or "USB-SERIAL" in desc:
            return p.device
    return None


def _net_worker(sock, net, nav):
    try:
        import serial as pyserial
    except ImportError:
        pyserial = None
    ser = None
    sbuf = b""
    last_ser_try = 0.0
    ser_opened_at = 0.0
    period = 1.0 / SEND_HZ
    next_t = time.perf_counter()
    while net["run"]:
        next_t += period
        l, r, w, en = net["cmd"]
        msg = f"L:{l},R:{r},W:{w},E:{en}".encode()

        if ser is not None:
            try:
                ser.write(msg + b"\n")
                if ser.in_waiting:
                    sbuf += ser.read(ser.in_waiting)
                    while b"\n" in sbuf:
                        line, sbuf = sbuf.split(b"\n", 1)
                        line = line.strip()
                        if line.startswith(b"OK"):
                            net["last_ack"] = time.time()
                            net["ack_count"] += 1
                            parse_ack(line, nav)
                if len(sbuf) > 4096:
                    sbuf = b""
                # cable present but boat not answering (old firmware or
                # wrong device): give the port back and return to WiFi
                if time.time() - max(net["last_ack"], ser_opened_at) > 3.0:
                    raise OSError("no acks over USB")
            except Exception:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                net["transport"] = "WiFi"
        else:
            try:
                sock.sendto(msg, (ESP32_IP, ESP32_PORT))
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
            # WiFi silent and a USB boat present? switch to the cable.
            now = time.time()
            if (pyserial and now - net["last_ack"] > 2.0
                    and now - last_ser_try > 3.0):
                last_ser_try = now
                port = _find_boat_port()
                if port:
                    try:
                        s = pyserial.Serial()
                        s.port = port
                        s.baudrate = 115200
                        s.timeout = 0
                        s.dtr = False       # avoid resetting the ESP32
                        s.rts = False
                        s.open()
                        ser = s
                        sbuf = b""
                        ser_opened_at = time.time()
                        net["transport"] = "USB"
                    except Exception:
                        ser = None

        left = next_t - time.perf_counter()
        if left > 0:
            time.sleep(left)
        elif left < -period * 4:
            next_t = time.perf_counter()   # fell far behind; resync
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


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
        # feed() runs on the network thread, drawing on the GUI thread
        self._lock = threading.Lock()

    def trail_points(self):
        with self._lock:
            return list(self.trail)

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
            with self._lock:
                self.trail.append((lat, lon))   # geographic trail

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


def _phase_seq(wp):
    """Ordered action phases executed at a waypoint after arrival."""
    seq = []
    wa = wp.get("wa", "none")
    if wa in ("out", "sample"):
        seq.append("winch_out")
    if wp.get("hold", 0) > 0:
        seq.append("dwell")
    if wa in ("in", "sample"):
        seq.append("winch_in")
    return seq


def autopilot_step(nav, waypoints, st, speed_limit, now):
    """One guidance step. st is the mission state dict:
        {"idx": waypoint index, "phase": "transit"|..., "until": t}

    Returns (left, right, winch_or_None, done, info). Waypoints are dicts
    {'lat','lon','hold','wa','ws'}: on arrival the boat runs its winch
    action ('out'/'in'/'sample') for 'ws' seconds around a 'hold' dwell —
    lower probe -> soak -> raise probe — then sails on. winch=None means
    the pilot's manual winch commands stay live (transit only).
    """
    idx = st["idx"]
    wp = waypoints[idx]
    phase = st["phase"]

    if phase != "transit":
        rem = st["until"] - now
        if rem > 0:
            if phase == "winch_out":
                return 90, 90, 0, False, f"WP{idx+1}: probe down {rem:.0f}s"
            if phase == "winch_in":
                return 90, 90, 180, False, f"WP{idx+1}: probe up {rem:.0f}s"
            return 90, 90, 90, False, f"WP{idx+1}: dwell {rem:.0f}s"
        seq = _phase_seq(wp)
        pos = seq.index(phase) if phase in seq else len(seq) - 1
        if pos + 1 < len(seq):
            nxt = seq[pos + 1]
            st["phase"] = nxt
            st["until"] = now + (wp.get("hold", 0) if nxt == "dwell"
                                 else wp.get("ws", 5))
            return 90, 90, 90, False, f"WP{idx+1}: next {nxt}"
        st["idx"] += 1
        st["phase"] = "transit"
        if st["idx"] >= len(waypoints):
            return 90, 90, 90, True, "MISSION COMPLETE"
        return 90, 90, 90, False, f"WP {idx+1} done"

    dist, bearing = geo_dist_bearing(nav.lat, nav.lon, wp["lat"], wp["lon"])
    if dist < WP_RADIUS_M:
        seq = _phase_seq(wp)
        if seq:
            st["phase"] = seq[0]
            st["until"] = now + (wp.get("hold", 0) if seq[0] == "dwell"
                                 else wp.get("ws", 5))
            return 90, 90, 90, False, f"WP{idx+1}: arrived, {seq[0]}"
        st["idx"] += 1
        if st["idx"] >= len(waypoints):
            return 90, 90, 90, True, "MISSION COMPLETE"
        return 90, 90, 90, False, f"WP {idx+1} reached"
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
    return left_cmd, right_cmd, None, False, \
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


def compass(surf, x, y, size, heading, wp_bearing=None):
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
    if wp_bearing is not None:
        # next-waypoint pin: orange diamond on the rim at its bearing
        a = math.radians(wp_bearing - 90)
        px = cx + math.cos(a) * rad
        py = cy + math.sin(a) * rad
        s = 5
        pygame.draw.polygon(surf, ORANGE,
                            ((px, py - s), (px + s, py), (px, py + s), (px - s, py)))
        pygame.draw.polygon(surf, BG,
                            ((px, py - s), (px + s, py), (px, py + s), (px - s, py)),
                            width=1)
    if heading is None:
        text(surf, "big", "--", cx, cy, DIM, center=True)
    else:
        a = math.radians(heading - 90)
        hx = cx + int(math.cos(a) * (rad - 22))
        hy = cy + int(math.sin(a) * (rad - 22))
        pygame.draw.line(surf, CYAN, (cx, cy), (hx, hy), 3)
        pygame.draw.circle(surf, CYAN, (cx, cy), 4)


class MapView:
    """Slippy-tile world map (OpenStreetMap) with mission planning.

    View mode (always): drag = pan, scroll = zoom, F = re-follow the boat.
    EDIT mode (keyboard E only): click water = add waypoint, click one =
    select, drag one = move, right-click one = delete, DEL = delete
    selected, [ / ] = hold time -/+, Ctrl+Z = undo. Tiles cache to
    assets/tiles and keep working offline once downloaded.
    """

    HIT_PX = 13

    def __init__(self, rect, center, zoom):
        self.rect = pygame.Rect(rect)
        self.lat, self.lon = center
        self.z = zoom
        self.waypoints = []          # [{'lat','lon','hold'}]
        self.sel = None
        self.follow = True
        self.touched = False         # user panned/zoomed at least once
        self._history = []
        self._drag_idx = None
        self._panning = False
        self._down = None
        self._last = None
        self._moved = False
        self._grab_clean = False     # drag started but nothing moved yet
        self._tiles = {}             # (z,x,y) -> Surface
        self._pending = set()
        self._fail = {}              # (z,x,y) -> retry-after timestamp
        self._qlock = threading.Lock()
        threading.Thread(target=self._fetcher, daemon=True).start()

    # ---- geo <-> screen ----
    def ll_to_screen(self, lat, lon):
        cx, cy = _ll_to_world(self.lat, self.lon, self.z)
        wx, wy = _ll_to_world(lat, lon, self.z)
        return (int(self.rect.centerx + (wx - cx)),
                int(self.rect.centery + (wy - cy)))

    def screen_to_ll(self, px, py):
        cx, cy = _ll_to_world(self.lat, self.lon, self.z)
        return _world_to_ll(cx + (px - self.rect.centerx),
                            cy + (py - self.rect.centery), self.z)

    def zoom_at(self, pos, direction):
        nz = clamp(self.z + (1 if direction > 0 else -1), 3, 19)
        if nz == self.z:
            return
        anchor = self.screen_to_ll(*pos)      # keep this point under cursor
        self.z = nz
        ax, ay = _ll_to_world(*anchor, self.z)
        self.lat, self.lon = _world_to_ll(
            ax - (pos[0] - self.rect.centerx),
            ay - (pos[1] - self.rect.centery), self.z)
        self.touched = True

    # ---- tile machinery ----
    def _tile_file(self, z, x, y):
        return os.path.join(ASSET_DIR, "tiles", str(z), str(x), f"{y}.png")

    def _fetcher(self):
        while True:
            with self._qlock:
                key = self._pending.pop() if self._pending else None
            if key is None:
                time.sleep(0.05)
                continue
            z, x, y = key
            fp = self._tile_file(z, x, y)
            try:
                if not os.path.exists(fp):
                    req = urllib.request.Request(
                        TILE_URL.format(z=z, x=x, y=y),
                        headers={"User-Agent": TILE_UA})
                    with urllib.request.urlopen(req, timeout=8) as r:
                        data = r.read()
                    os.makedirs(os.path.dirname(fp), exist_ok=True)
                    open(fp, "wb").write(data)
                    time.sleep(0.05)          # politeness to the tile server
            except Exception:
                self._fail[key] = time.time() + 30.0

    def _get_tile(self, z, x, y):
        n = 1 << z
        if not (0 <= y < n):
            return None
        x %= n
        key = (z, x, y)
        surf = self._tiles.get(key)
        if surf is not None:
            return surf
        fp = self._tile_file(z, x, y)
        if os.path.exists(fp):
            try:
                surf = pygame.image.load(fp).convert()
                if len(self._tiles) > 500:      # simple eviction
                    self._tiles.clear()
                self._tiles[key] = surf
                return surf
            except Exception:
                try:
                    os.remove(fp)               # corrupt/partial download
                except OSError:
                    pass
        if key not in self._fail or time.time() > self._fail.get(key, 0):
            self._fail.pop(key, None)
            with self._qlock:
                self._pending.add(key)
        return None

    # ---- undo plumbing: snapshot before every mutation ----
    def _push(self):
        self._history.append([dict(wp) for wp in self.waypoints])
        if len(self._history) > 100:
            self._history.pop(0)

    def undo(self):
        if self._history:
            self.waypoints = self._history.pop()
            self._drag_idx = None
            if self.sel is not None and self.sel >= len(self.waypoints):
                self.sel = None

    def clear_all(self):
        if self.waypoints:
            self._push()
            self.waypoints = []
            self.sel = None

    def delete_sel(self):
        if self.sel is not None and self.sel < len(self.waypoints):
            self._push()
            self.waypoints.pop(self.sel)
            self.sel = None

    def adjust_hold(self, delta):
        if self.sel is not None and self.sel < len(self.waypoints):
            self._push()
            wp = self.waypoints[self.sel]
            wp["hold"] = clamp(wp.get("hold", 0) + delta, 0, 600)

    WA_CYCLE = ["none", "out", "in", "sample"]
    WA_LABEL = {"none": "—", "out": "v OUT", "in": "^ IN", "sample": "v^ SMP"}

    def cycle_winch(self):
        if self.sel is not None and self.sel < len(self.waypoints):
            self._push()
            wp = self.waypoints[self.sel]
            cur = self.WA_CYCLE.index(wp.get("wa", "none"))
            wp["wa"] = self.WA_CYCLE[(cur + 1) % len(self.WA_CYCLE)]

    def adjust_ws(self, delta):
        if self.sel is not None and self.sel < len(self.waypoints):
            self._push()
            wp = self.waypoints[self.sel]
            wp["ws"] = clamp(wp.get("ws", 5) + delta, 1, 120)

    def _hit(self, pos):
        for i, wp in enumerate(self.waypoints):
            px, py = self.ll_to_screen(wp["lat"], wp["lon"])
            if (px - pos[0]) ** 2 + (py - pos[1]) ** 2 <= self.HIT_PX ** 2:
                return i
        return None

    # ---- mouse ----
    def mouse_down(self, pos, btn, edit):
        if not self.rect.collidepoint(pos):
            return False
        i = self._hit(pos)
        if btn == 1:
            self._down = pos
            self._last = pos
            self._moved = False
            if edit and i is not None:
                self._push()
                self._grab_clean = True
                self._drag_idx = i
                self.sel = i
            else:
                self._panning = True
        elif btn == 3 and edit and i is not None:
            self._push()
            self.waypoints.pop(i)
            if self.sel is not None:
                if self.sel == i:
                    self.sel = None
                elif self.sel > i:
                    self.sel -= 1
        return True

    def mouse_move(self, pos):
        if self._drag_idx is not None and self._drag_idx < len(self.waypoints):
            lat, lon = self.screen_to_ll(
                clamp(pos[0], self.rect.left + 6, self.rect.right - 6),
                clamp(pos[1], self.rect.top + 6, self.rect.bottom - 6))
            self.waypoints[self._drag_idx]["lat"] = lat
            self.waypoints[self._drag_idx]["lon"] = lon
            self._moved = True
            self._grab_clean = False
        elif self._panning and self._last is not None:
            dx = pos[0] - self._last[0]
            dy = pos[1] - self._last[1]
            if dx or dy:
                cx, cy = _ll_to_world(self.lat, self.lon, self.z)
                self.lat, self.lon = _world_to_ll(cx - dx, cy - dy, self.z)
                self._moved = True
                self.follow = False
                self.touched = True
        self._last = pos

    def mouse_up(self, pos, edit):
        if (edit and self._down is not None and not self._moved
                and self._drag_idx is None and self._panning):
            # a clean click on empty map: add a waypoint there
            lat, lon = self.screen_to_ll(*self._down)
            self._push()
            self.waypoints.append({"lat": lat, "lon": lon, "hold": 0,
                                   "wa": "none", "ws": 5})
            self.sel = len(self.waypoints) - 1
        if self._grab_clean:
            # grabbed a waypoint but never moved it: drop the no-op undo entry
            self._history.pop()
        self._grab_clean = False
        self._drag_idx = None
        self._panning = False
        self._down = None

    def center_on(self, lat, lon, z=None):
        self.lat, self.lon = lat, lon
        if z is not None:
            self.z = z

    def set_rect(self, rect):
        self.rect = pygame.Rect(rect)

    # ---- drawing ----
    def draw(self, surf, nav, target_idx=None, edit=False):
        r = self.rect
        if self.follow and nav.alive and nav.has_fix:
            self.lat, self.lon = nav.lat, nav.lon
        clip_prev = surf.get_clip()
        surf.set_clip(r)
        pygame.draw.rect(surf, (16, 19, 26), r)

        # tiles
        cx, cy = _ll_to_world(self.lat, self.lon, self.z)
        left_w = cx - r.w / 2
        top_w = cy - r.h / 2
        tx0 = int(left_w // TILESIZE)
        ty0 = int(top_w // TILESIZE)
        tx1 = int((left_w + r.w) // TILESIZE)
        ty1 = int((top_w + r.h) // TILESIZE)
        for tx in range(tx0, tx1 + 1):
            for ty in range(ty0, ty1 + 1):
                sx = r.x + int(tx * TILESIZE - left_w)
                sy = r.y + int(ty * TILESIZE - top_w)
                t = self._get_tile(self.z, tx, ty)
                if t is not None:
                    surf.blit(t, (sx, sy))
                else:
                    pygame.draw.rect(surf, (20, 24, 32),
                                     (sx, sy, TILESIZE, TILESIZE))
                    pygame.draw.rect(surf, (26, 31, 41),
                                     (sx, sy, TILESIZE, TILESIZE), width=1)

        # PC (this computer) pin — IP geolocation, city precision
        pc = pc_loc[0]
        if pc:
            px, py = self.ll_to_screen(pc["lat"], pc["lon"])
            if r.collidepoint(px, py):
                pygame.draw.circle(surf, (180, 120, 255), (px, py), 6)
                pygame.draw.circle(surf, WHITE, (px, py), 6, width=1)
                text(surf, "lbl", "PC", px, py - 14, (180, 120, 255), center=True)

        # HOME (first GPS fix)
        if nav.origin:
            hx, hy = self.ll_to_screen(*nav.origin)
            pygame.draw.circle(surf, YELLOW, (hx, hy), 6, width=2)
            text(surf, "lbl", "HOME", hx, hy - 14, YELLOW, center=True)

        # waypoint route
        pts = [self.ll_to_screen(w["lat"], w["lon"]) for w in self.waypoints]
        if len(pts) > 1:
            pygame.draw.lines(surf, (60, 90, 140), False, pts, 2)
        for i, p in enumerate(pts):
            grabbed = (i == self._drag_idx)
            selected = (i == self.sel and edit)
            active = (i == target_idx)
            colr = YELLOW if grabbed else (CYAN if selected else BLUE)
            pygame.draw.circle(surf, colr, p, 9 if (grabbed or selected) else 7)
            pygame.draw.circle(surf, WHITE, p, 9 if (grabbed or selected) else 7, width=1)
            if active:
                pygame.draw.circle(surf, ORANGE, p, 14, width=2)
            text(surf, "lbl", str(i + 1), p[0], p[1],
                 BG if (grabbed or selected) else WHITE, center=True)
            hold = self.waypoints[i].get("hold", 0)
            if hold:
                text(surf, "lbl", f"{hold:.0f}s", p[0], p[1] + 16, YELLOW, center=True)
            wa = self.waypoints[i].get("wa", "none")
            if wa != "none":
                ws = self.waypoints[i].get("ws", 5)
                mark = {"out": "v", "in": "^", "sample": "v^"}[wa]
                wcol = {"out": ORANGE, "in": GREEN, "sample": CYAN}[wa]
                text(surf, "lbl", f"{mark}{ws:.0f}s", p[0],
                     p[1] + (30 if hold else 16), wcol, center=True)

        # boat trail + boat
        trail = nav.trail_points()
        if len(trail) > 1:
            pygame.draw.lines(surf, (40, 160, 120), False,
                              [self.ll_to_screen(*q) for q in trail], 2)
        if nav.alive and nav.has_fix:
            bx, by = self.ll_to_screen(nav.lat, nav.lon)
            if target_idx is not None and target_idx < len(pts):
                pygame.draw.line(surf, ORANGE, (bx, by), pts[target_idx], 2)
            hdg = nav.heading if nav.heading is not None else 0
            a = math.radians(hdg - 90)
            tip = (bx + int(math.cos(a) * 13), by + int(math.sin(a) * 13))
            lft = (bx + int(math.cos(a + 2.5) * 9), by + int(math.sin(a + 2.5) * 9))
            rgt = (bx + int(math.cos(a - 2.5) * 9), by + int(math.sin(a - 2.5) * 9))
            pygame.draw.polygon(surf, CYAN, (tip, lft, rgt))
            pygame.draw.polygon(surf, BG, (tip, lft, rgt), width=1)

        # edit-mode overlay: banner + ArduPilot-style waypoint table
        if edit:
            pygame.draw.rect(surf, ORANGE, r, width=2)
            pygame.draw.rect(surf, (40, 30, 14), (r.x + 2, r.y + 2, 170, 26))
            text(surf, "med", "EDIT MODE", r.x + 12, r.y + 7, ORANGE)
            if self.waypoints:
                rows = min(len(self.waypoints), 14)
                th = 20 * rows + 30
                ov = pygame.Surface((252, th), pygame.SRCALPHA)
                ov.fill((14, 17, 23, 225))
                surf.blit(ov, (r.x + 2, r.y + 32))
                text(surf, "lbl", " #    LEG   HOLD   WINCH", r.x + 12, r.y + 38, GREY)
                prev = None
                total = 0.0
                for i, w in enumerate(self.waypoints):
                    if prev is not None:
                        d, _ = geo_dist_bearing(prev["lat"], prev["lon"],
                                                w["lat"], w["lon"])
                        total += d
                    else:
                        d = 0.0
                    if i < rows:
                        ycol = CYAN if i == self.sel else WHITE
                        wa = w.get("wa", "none")
                        wtxt = ("  --" if wa == "none" else
                                f"{self.WA_LABEL[wa]} {w.get('ws', 5):.0f}s")
                        text(surf, "sm",
                             f"{i+1:>2} {d:>5.0f}m {w.get('hold',0):>3.0f}s  {wtxt}",
                             r.x + 12, r.y + 54 + 20 * i, ycol)
                    prev = w
                text(surf, "lbl", f"route {total:.0f}m",
                     r.x + 12, r.y + 40 + 20 * rows + 14, GREY)

        # HUD line: zoom + scale + follow state + cursor coords + attribution
        mpp = 40075016.7 * math.cos(math.radians(self.lat)) / ((1 << self.z) * TILESIZE)
        scale_txt = (f"{mpp * 100:.0f}m/100px" if mpp * 100 >= 10
                     else f"{mpp * 100:.1f}m/100px")
        hud = (f"z{self.z}  {scale_txt}   "
               f"{'FOLLOW' if self.follow else 'free pan (F=follow)'}")
        if self._last is not None and self.rect.collidepoint(self._last):
            clat, clon = self.screen_to_ll(*self._last)
            hud += f"   cursor {clat:.6f}, {clon:.6f}"
        if edit and self.sel is not None and self.sel < len(self.waypoints):
            sw = self.waypoints[self.sel]
            hud += f"   WP{self.sel+1} {sw['lat']:.6f}, {sw['lon']:.6f}"
        text(surf, "lbl", hud, r.left + 10, r.bottom - 20, GREY)
        text(surf, "lbl", "(c) OpenStreetMap", r.right - 118, r.bottom - 20, GREY)
        if not (nav.alive and nav.has_fix):
            text(surf, "lbl", "NO GPS — boat position unknown",
                 r.centerx, r.top + 12, ORANGE, center=True)

        surf.set_clip(clip_prev)
        pygame.draw.rect(surf, OUTLINE, r, width=1, border_radius=2)


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
    threading.Thread(target=_pc_locator, daemon=True).start()
    pc_cached = load_pc_cache()
    if pc_cached:
        map_center, map_zoom = (pc_cached["lat"], pc_cached["lon"]), 12
    else:
        map_center, map_zoom = PH_CENTER, 6
    mapview = MapView((472, 96, 610, 580), map_center, map_zoom)
    boat3d = Boat3D((1098, 400, 478, 330))
    cam_rect = (1098, 96, 478, 292)

    MAP_RECT_NORMAL = (472, 96, 610, 580)
    MAP_RECT_EDIT = (24, 72, W - 48, H - 160)

    js = None
    speed = 1.0
    motors_on = False              # SAFETY: always start disarmed
    mode = "TELEOP"                # "TELEOP" | "AUTO"
    edit_mode = False              # keyboard E only — mission editing
    help_open = False
    help_scroll = 0
    mission = {"idx": 0, "phase": "transit", "until": 0.0}
    auto_info = ""
    winch_cmd = 90
    w_cmd = 90
    left_cmd = right_cmd = 90
    prev_square = prev_l1 = prev_r1 = False
    prev_options = prev_share = False
    prev_hat_y = 0
    w_btns = []                    # on-screen winch buttons: (Rect, cmd)

    HELP_LINES = [
        ("KEYBOARD", ""),
        ("SPACE", "arm / disarm (also: Square)"),
        ("M", "AUTO <-> TELEOP (also: Options)"),
        ("E", "edit mode: full-screen map, everything disabled"),
        ("F", "map follows the boat again (also: Share)"),
        ("I / K / O", "winch in / stop / out (driving screen)"),
        ("Arrows", "speed limit +/- (also: L1/R1 or D-pad up/down)"),
        ("C", "clear all waypoints"),
        ("H", "this panel"),
        ("F11 / ESC", "fullscreen / leave fullscreen or quit"),
        ("", ""),
        ("EDIT MODE (keyboard E)", ""),
        ("click", "add waypoint / select one"),
        ("drag waypoint", "move it"),
        ("right-click / DEL", "delete waypoint / delete selected"),
        ("[ / ]", "selected waypoint hold (dwell) time -/+5s"),
        ("W", "cycle winch action: — / vOUT / ^IN / v^SAMPLE"),
        ("- / =", "winch action run time -/+1s"),
        ("", "at the waypoint: probe down -> dwell -> probe up"),
        ("Ctrl+Z", "undo any mission edit"),
        ("", ""),
        ("CONTROLLER (DS4)", ""),
        ("Left stick Y", "throttle"),
        ("Right stick X", "steer"),
        ("R2 / L2 (hold)", "winch in / out — releases to stop"),
        ("Triangle / Circle / Cross", "winch latched in / stop / out"),
        ("Square", "arm / disarm"),
        ("Options", "AUTO <-> TELEOP"),
        ("Share", "map re-follows boat"),
        ("L1 / R1, D-pad", "speed limit -/+"),
        ("", ""),
        ("MAP", ""),
        ("drag / scroll", "pan / zoom (zoom keeps cursor position)"),
        ("purple PC pin", "this computer (IP location, city precision)"),
        ("offline tiles", "python download_map.py  (run at home)"),
    ]
    disconnect_time = 0.0
    cam_decoded = None
    cam_last_decode = 0.0
    running = True
    connected = False

    net = {"run": True, "cmd": (90, 90, 90, 0), "last_ack": 0.0,
           "ack_count": 0, "transport": "WiFi"}
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
                and not edit_mode
                and nav.alive and nav.has_fix and nav.heading is not None)

    def toggle_mode():
        nonlocal mode, auto_info, mission
        if mode == "AUTO":
            mode = "TELEOP"
            auto_info = ""
        elif auto_ready():
            mode = "AUTO"
            mission = {"idx": 0, "phase": "transit", "until": 0.0}
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
                    if help_open:
                        help_open = False
                    elif fullscreen:
                        fullscreen = False
                        screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
                    else:
                        running = False
                elif e.key == pygame.K_h:
                    help_open = not help_open
                    help_scroll = 0
                elif e.key in (pygame.K_UP, pygame.K_RIGHT):
                    speed = clamp(round(speed + 0.1, 1), 0.0, 2.0)
                elif e.key in (pygame.K_DOWN, pygame.K_LEFT):
                    speed = clamp(round(speed - 0.1, 1), 0.0, 2.0)
                elif e.key == pygame.K_SPACE and not edit_mode:
                    motors_on = not motors_on
                elif e.key == pygame.K_m:
                    toggle_mode()
                elif e.key == pygame.K_e:
                    edit_mode = not edit_mode
                    winch_cmd = 90            # no surprise winch motion
                    if edit_mode:
                        mode = "TELEOP"       # editing disables the drive
                        motors_on = False
                        mapview.set_rect(MAP_RECT_EDIT)
                    else:
                        mapview.sel = None
                        mapview.set_rect(MAP_RECT_NORMAL)
                elif e.key == pygame.K_f:
                    mapview.follow = True
                elif e.key == pygame.K_w and edit_mode:
                    mapview.cycle_winch()
                elif e.key == pygame.K_MINUS and edit_mode:
                    mapview.adjust_ws(-1)
                elif e.key == pygame.K_EQUALS and edit_mode:
                    mapview.adjust_ws(+1)
                elif e.key == pygame.K_i and not edit_mode:
                    winch_cmd = 180
                elif e.key == pygame.K_k and not edit_mode:
                    winch_cmd = 90
                elif e.key == pygame.K_o and not edit_mode:
                    winch_cmd = 0
                elif e.key == pygame.K_c:
                    mapview.clear_all()
                    mode = "TELEOP"
                elif e.key == pygame.K_DELETE and edit_mode:
                    mapview.delete_sel()
                elif e.key == pygame.K_LEFTBRACKET and edit_mode:
                    mapview.adjust_hold(-HOLD_STEP_S)
                elif e.key == pygame.K_RIGHTBRACKET and edit_mode:
                    mapview.adjust_hold(+HOLD_STEP_S)
                elif e.key == pygame.K_z and (e.mod & (pygame.KMOD_CTRL | pygame.KMOD_META)):
                    mapview.undo()
            elif e.type == pygame.MOUSEBUTTONDOWN:
                cpos = to_canvas(e.pos)
                if help_open:
                    if e.button == 4:
                        help_scroll = max(0, help_scroll - 3)
                    elif e.button == 5:
                        help_scroll += 3
                    elif e.button == 1:
                        help_open = False
                elif e.button in (4, 5):
                    if mapview.rect.collidepoint(cpos):
                        mapview.zoom_at(cpos, 1 if e.button == 4 else -1)
                elif e.button == 1 and not edit_mode and boat3d.mouse_down(cpos):
                    pass
                elif e.button == 1 and next(
                        (True for r, _ in w_btns if r.collidepoint(cpos)), False):
                    winch_cmd = next(c for r, c in w_btns if r.collidepoint(cpos))
                elif btn_start.collidepoint(cpos):
                    toggle_mode()
                elif btn_clear.collidepoint(cpos):
                    mapview.clear_all()
                    mode = "TELEOP"
                else:
                    mapview.mouse_down(cpos, e.button, edit_mode)
            elif e.type == pygame.MOUSEBUTTONUP:
                boat3d.mouse_up()
                mapview.mouse_up(to_canvas(e.pos), edit_mode)
            elif e.type == pygame.MOUSEMOTION:
                cpos = to_canvas(e.pos)
                if not edit_mode:
                    boat3d.mouse_move(cpos)
                mapview.mouse_move(cpos)
            elif e.type == pygame.MOUSEWHEEL:
                cpos = to_canvas(pygame.mouse.get_pos())
                if help_open:
                    help_scroll = max(0, help_scroll - e.y * 3)
                elif mapview.rect.collidepoint(cpos):
                    mapview.zoom_at(cpos, e.y)

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
        winch_live = None            # trigger hold-to-run, set while connected

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
            options  = safe_button(js, BTN_OPTIONS)
            share    = safe_button(js, BTN_SHARE)

            if square and not prev_square and not edit_mode:
                motors_on = not motors_on
            prev_square = square
            if options and not prev_options:
                toggle_mode()
            prev_options = options
            if share and not prev_share:
                mapview.follow = True
            prev_share = share

            l1 = safe_button(js, BTN_L1)
            r1 = safe_button(js, BTN_R1)
            if r1 and not prev_r1:
                speed = clamp(round(speed + 0.1, 1), 0.0, 2.0)
            if l1 and not prev_l1:
                speed = clamp(round(speed - 0.1, 1), 0.0, 2.0)
            prev_r1 = r1
            prev_l1 = l1

            # D-pad up/down also steps the speed limit
            try:
                hat_y = js.get_hat(0)[1] if js.get_numhats() > 0 else 0
            except Exception:
                hat_y = 0
            if hat_y == 1 and prev_hat_y != 1:
                speed = clamp(round(speed + 0.1, 1), 0.0, 2.0)
            elif hat_y == -1 and prev_hat_y != -1:
                speed = clamp(round(speed - 0.1, 1), 0.0, 2.0)
            prev_hat_y = hat_y

            if triangle:
                winch_cmd = 180
            elif circle:
                winch_cmd = 90
            elif cross:
                winch_cmd = 0

            # analog triggers: hold-to-run winch, releases back to latched cmd
            l2 = (safe_axis(js, AXIS_L2) + 1.0) / 2.0
            r2 = (safe_axis(js, AXIS_R2) + 1.0) / 2.0
            if r2 > TRIGGER_ON:
                winch_live = 180
            elif l2 > TRIGGER_ON:
                winch_live = 0

            if motors_on:
                lp = clamp(forward + turn, -1.0, 1.0)
                rp = clamp(forward - turn, -1.0, 1.0)
                rng = 90 * speed
                left_cmd  = int(clamp(90 + lp * rng, 0, 180))
                right_cmd = int(clamp(90 + rp * rng, 0, 180))
                w_cmd = winch_live if winch_live is not None else winch_cmd
            else:
                left_cmd = right_cmd = 90
                w_cmd = 90
                mode = "TELEOP"          # disarm always exits AUTO

            # ---- AUTO mode: station-side autopilot drives L/R (+winch) ----
            if mode == "AUTO":
                if abs(forward) > STICK_OVERRIDE or abs(turn) > STICK_OVERRIDE:
                    mode = "TELEOP"       # pilot grabbed the sticks
                    auto_info = "stick override"
                elif not (nav.alive and nav.has_fix and nav.heading is not None
                          and mission["idx"] < len(mapview.waypoints)):
                    left_cmd = right_cmd = 90   # telemetry lost: hold neutral
                    auto_info = "AUTO PAUSED — no telemetry"
                else:
                    left_cmd, right_cmd, w_auto, done, auto_info = \
                        autopilot_step(nav, mapview.waypoints, mission, speed,
                                       time.time())
                    if w_auto is not None:
                        w_cmd = w_auto        # planned winch action running
                    if done:
                        mode = "TELEOP"
        else:
            left_cmd = right_cmd = 90
            mode = "TELEOP"               # controller is the deadman for AUTO
            # keyboard / on-screen winch still works while armed
            w_cmd = winch_cmd if motors_on else 90

        # hand the command to the 50Hz network thread (E:0 = instant stop)
        # EDIT mode: everything hard-disabled (winch actions are PLANNED
        # into waypoints here, not run live — the autopilot executes them)
        if edit_mode:
            left_cmd = right_cmd = 90
            w_cmd = 90
            en = 0
        else:
            en = 1 if motors_on else 0
        net["cmd"] = (left_cmd, right_cmd, w_cmd, en)
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
        age = time.time() - last_ack
        w_btns = []

        # bearing/distance to the next waypoint (AUTO target, else WP1)
        wp_bear = wp_dist = None
        tgt_i = mission["idx"] if mode == "AUTO" else 0
        if (mapview.waypoints and tgt_i < len(mapview.waypoints)
                and nav.alive and nav.has_fix):
            _wpt = mapview.waypoints[tgt_i]
            wp_dist, wp_bear = geo_dist_bearing(nav.lat, nav.lon,
                                                _wpt["lat"], _wpt["lon"])

        if not edit_mode:
            # -- left column: drive --
            pill(canvas, 24, 96, 208, 42, "CONTROLLER", connected,
                 "OK" if connected else "NOT FOUND")
            pill(canvas, 240, 96, 208, 42, "WIFI", on_boat_wifi, wifi_detail)
            pill(canvas, 24, 146, 208, 42, "BOAT LINK", boat_ok,
                 f"{net['transport']} {int(age*1000)}ms" if boat_ok else "LOST")
            pill(canvas, 240, 146, 208, 42, "GPS",
                 nav.alive and nav.has_fix,
                 f"{nav.sats} SATS" if (nav.alive and nav.has_fix) else "NO FIX")

            arm_banner(canvas, 24, 200, 424, 62, motors_on)
            speed_bar(canvas, 24, 270, 424, 56, speed)

            motor_bar(canvas, 24, 338, 96, 262, left_cmd, "LEFT", motors_on)
            motor_bar(canvas, 128, 338, 96, 262, right_cmd, "RIGHT", motors_on)
            stick_box(canvas, 232, 338, 126, lx, ly, "L STICK")
            stick_box(canvas, 232, 474, 126, rx, ry, "R STICK")
            compass(canvas, 366, 338, 82,
                    None if not (nav.alive and nav.heading is not None)
                    else nav.heading,
                    wp_bear)
            panel(canvas, 366, 428, 82, 172)
            text(canvas, "lbl", "OUT", 380, 440, GREY)
            text(canvas, "sm", f"L {pct(left_cmd):+d}", 380, 462, WHITE)
            text(canvas, "sm", f"R {pct(right_cmd):+d}", 380, 484, WHITE)
            text(canvas, "sm", f"W {pct(w_cmd):+d}", 380, 506, WHITE)
            text(canvas, "sm", f"ack", 380, 540, DIM)
            text(canvas, "sm", f"{ack_count}", 380, 560, GREY)

            # winch: state + clickable actuation buttons (keys I/K/O)
            panel(canvas, 24, 608, 424, 44)
            state = ("IN ^" if w_cmd > 100 else
                     ("OUT v" if w_cmd < 80 else "STOP"))
            scol = (GREEN if w_cmd > 100 else
                    (ORANGE if w_cmd < 80 else GREY))
            text(canvas, "lbl", "WINCH", 36, 608 + 8, GREY)
            text(canvas, "med", state, 36, 608 + 22, scol)
            w_btns = [
                (button(canvas, (208, 614, 72, 32), "IN (I)",
                        accent=GREEN if winch_cmd == 180 else BLUE), 180),
                (button(canvas, (286, 614, 72, 32), "STOP(K)",
                        accent=WHITE if winch_cmd == 90 else BLUE), 90),
                (button(canvas, (364, 614, 72, 32), "OUT (O)",
                        accent=ORANGE if winch_cmd == 0 else BLUE), 0),
            ]

            if net["transport"] == "USB" and boat_ok:
                text(canvas, "sm", "Wired to the boat over USB — WiFi not needed.",
                     24, 664, CYAN)
            elif not on_boat_wifi:
                text(canvas, "sm",
                     f'Join WiFi "{BOAT_SSID}" — or plug the boat in over USB. '
                     '("No internet" on boat WiFi is normal.)',
                     24, 664, ORANGE)
            elif not boat_ok:
                text(canvas, "sm", "On boat WiFi but no reply — boat powered?",
                     24, 664, ORANGE)

        # -- navigation map (full-screen in edit mode) --
        # first successful geolocation: recenter an untouched overview map
        if pc_loc[0] and not mapview.touched and mapview.z <= 6 and not nav.has_fix:
            mapview.center_on(pc_loc[0]["lat"], pc_loc[0]["lon"], 12)
        if not edit_mode:
            text(canvas, "lbl", "NAVIGATION", 472, 74, GREY)
        mapview.draw(canvas, nav,
                     mission["idx"] if mode == "AUTO"
                     and mission["idx"] < len(mapview.waypoints) else None,
                     edit_mode)
        if mode == "AUTO":
            text(canvas, "med", "AUTO", mapview.rect.right - 46,
                 mapview.rect.top + 20, ORANGE, center=True)

        # mission bar: under the map normally, full width in edit mode
        if edit_mode:
            bar_x, bar_y, bar_w = 24, H - 76, W - 48
        else:
            bar_x, bar_y, bar_w = 472, 96 + 580 + 8, 610
        panel(canvas, bar_x, bar_y, bar_w, 44)
        wp_n = len(mapview.waypoints)
        in_auto = (mode == "AUTO")
        pygame.draw.rect(canvas, PANEL2, (bar_x + 10, bar_y + 6, 118, 32),
                         border_radius=8)
        chip = "EDIT" if edit_mode else mode
        text(canvas, "med", chip, bar_x + 69, bar_y + 22,
             CYAN if edit_mode else (ORANGE if in_auto else BLUE), center=True)
        text(canvas, "med", f"WP: {wp_n}", bar_x + 144, bar_y + 13, WHITE)
        btn_start = button(canvas, (bar_x + 216, bar_y + 6, 156, 32),
                           "STOP AUTO (M)" if in_auto else "START AUTO (M)",
                           enabled=in_auto or auto_ready(),
                           accent=RED if in_auto else GREEN)
        btn_clear = button(canvas, (bar_x + 382, bar_y + 6, 100, 32), "CLEAR (C)")
        if edit_mode:
            sel_wp = (mapview.waypoints[mapview.sel]
                      if mapview.sel is not None
                      and mapview.sel < len(mapview.waypoints) else None)
            if sel_wp:
                wa = sel_wp.get("wa", "none")
                text(canvas, "med",
                     f"WP{mapview.sel+1}: winch {MapView.WA_LABEL[wa]}"
                     + (f" {sel_wp.get('ws',5):.0f}s" if wa != "none" else "")
                     + f"   hold {sel_wp.get('hold',0):.0f}s",
                     bar_x + 500, bar_y + 13, CYAN)
            else:
                text(canvas, "sm", "select a waypoint to set its winch action",
                     bar_x + 500, bar_y + 14, DIM)
            text(canvas, "sm",
                 "W winch action   -/= seconds   [ ] hold   Ctrl+Z undo   E exit   H help",
                 bar_x + 940, bar_y + 14, GREY)
        elif in_auto:
            text(canvas, "lbl", auto_info, bar_x + 494, bar_y + 17, ORANGE)
        elif not auto_ready():
            need = "needs: "
            if not motors_on: need += "ARM "
            if not connected: need += "controller "
            if not (nav.alive and nav.has_fix): need += "GPS "
            elif nav.heading is None: need += "heading "
            if not wp_n: need += "waypoints"
            text(canvas, "lbl", need.strip(), bar_x + 494, bar_y + 17, DIM)

        if not edit_mode:
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
            if wp_bear is not None:
                d_txt = (f"{wp_dist:.0f}m" if wp_dist < 1000
                         else f"{wp_dist/1000:.1f}km")
                text(canvas, "lbl",
                     f"WP{tgt_i+1} {int(wp_bear):03d}° {d_txt}",
                     676 + 100, t_y + 12, ORANGE)
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
                 "H = all controls    SPACE arm    M auto/teleop    "
                 "E edit mode    I/K/O winch    F11 fullscreen",
                 24, H - 26, GREY)

        # -- help overlay (H): scrollable controls reference --
        if help_open:
            shade = pygame.Surface((W, H), pygame.SRCALPHA)
            shade.fill((0, 0, 0, 150))
            canvas.blit(shade, (0, 0))
            hw, hh = 640, 560
            hx, hy = (W - hw) // 2, (H - hh) // 2
            panel(canvas, hx, hy, hw, hh, PANEL)
            text(canvas, "big", "CONTROLS", hx + 24, hy + 16, WHITE)
            text(canvas, "lbl", "scroll to browse — H or ESC to close",
                 hx + hw - 250, hy + 26, GREY)
            row_h = 24
            view_rows = (hh - 70) // row_h
            max_scroll = max(0, len(HELP_LINES) - view_rows)
            help_scroll = clamp(help_scroll, 0, max_scroll)
            clip_prev = canvas.get_clip()
            canvas.set_clip((hx, hy + 56, hw, hh - 70))
            for r_i, (k, desc) in enumerate(
                    HELP_LINES[help_scroll:help_scroll + view_rows + 1]):
                yy = hy + 60 + r_i * row_h
                if desc == "" and k:
                    text(canvas, "med", k, hx + 24, yy, CYAN)
                else:
                    text(canvas, "sm", k, hx + 40, yy, YELLOW)
                    text(canvas, "sm", desc, hx + 240, yy, WHITE)
            canvas.set_clip(clip_prev)
            # scroll bar
            if max_scroll > 0:
                track_h = hh - 80
                thumb_h = max(30, int(track_h * view_rows / len(HELP_LINES)))
                thumb_y = hy + 60 + int((track_h - thumb_h)
                                        * help_scroll / max_scroll)
                pygame.draw.rect(canvas, PANEL2,
                                 (hx + hw - 14, hy + 60, 6, track_h),
                                 border_radius=3)
                pygame.draw.rect(canvas, GREY,
                                 (hx + hw - 14, thumb_y, 6, thumb_h),
                                 border_radius=3)

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
