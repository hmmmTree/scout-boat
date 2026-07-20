"""Offline map downloader for the boat driver station.

Fetches OpenStreetMap tiles into assets/tiles/ where the driver station
(6767.py / Computer_side.py) reads them. Run this at home while you have
internet; on the water the station then works fully offline.

Usage:
  python download_map.py                     # Philippines overview + area
                                             # around this computer (via IP)
  python download_map.py --at 14.505 121.03  # detail around a lat/lon
  python download_map.py --at 14.5 121.0 --radius 0.4 --zmax 15

Options:
  --at LAT LON    center of the detail area (default: computer's IP location)
  --radius DEG    half-size of the detail box in degrees (default 0.20)
  --zmin/--zmax   detail zoom range (default 9..14; 15+ grows fast!)
  --no-overview   skip the country-wide Philippines base layer

Please be considerate: this uses the free openstreetmap.org tile server.
The built-in delay keeps the request rate polite; don't crank the zoom
range higher than you need. (c) OpenStreetMap contributors.
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = (os.path.join(HERE, "assets") if os.path.isdir(os.path.join(HERE, "assets"))
          else os.path.normpath(os.path.join(HERE, "..", "assets")))
TILES = os.path.join(ASSETS, "tiles")
UA = {"User-Agent": "ScoutBoatDriverStation/1.0 (personal robotics project)"}
PH_BBOX = (21.5, 116.5, 4.2, 127.0)          # N, W, S, E


def deg2tile(lat, lon, z):
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    lr = math.radians(lat)
    y = int((1.0 - math.log(math.tan(lr) + 1.0 / math.cos(lr)) / math.pi) / 2.0 * n)
    return x, y


def fetch(z, x, y):
    """Download one tile if not cached. Returns True if downloaded."""
    d = os.path.join(TILES, str(z), str(x))
    fp = os.path.join(d, f"{y}.png")
    if os.path.exists(fp):
        return False
    os.makedirs(d, exist_ok=True)
    req = urllib.request.Request(
        f"https://tile.openstreetmap.org/{z}/{x}/{y}.png", headers=UA)
    with urllib.request.urlopen(req, timeout=10) as r:
        data = r.read()
    open(fp, "wb").write(data)
    time.sleep(0.06)                          # politeness to the tile server
    return True


def fetch_box(north, west, south, east, zmin, zmax, label):
    got = have = err = 0
    for z in range(zmin, zmax + 1):
        x0, y0 = deg2tile(north, west, z)
        x1, y1 = deg2tile(south, east, z)
        for x in range(min(x0, x1), max(x0, x1) + 1):
            for y in range(min(y0, y1), max(y0, y1) + 1):
                try:
                    if fetch(z, x, y):
                        got += 1
                    else:
                        have += 1
                except Exception:
                    err += 1
        print(f"  {label} z{z}: +{got} new ({have} cached, {err} failed)")
    return got


def locate_pc():
    try:
        with urllib.request.urlopen(
                "http://ip-api.com/json/?fields=status,lat,lon,city", timeout=8) as r:
            d = json.loads(r.read().decode())
        if d.get("status") == "success":
            pc = {"lat": d["lat"], "lon": d["lon"], "city": d.get("city", "")}
            os.makedirs(ASSETS, exist_ok=True)
            json.dump(pc, open(os.path.join(ASSETS, "pc_location.json"), "w"))
            return pc
    except Exception:
        pass
    try:
        return json.load(open(os.path.join(ASSETS, "pc_location.json")))
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--at", nargs=2, type=float, metavar=("LAT", "LON"))
    ap.add_argument("--radius", type=float, default=0.20)
    ap.add_argument("--zmin", type=int, default=9)
    ap.add_argument("--zmax", type=int, default=14)
    ap.add_argument("--no-overview", action="store_true")
    a = ap.parse_args()

    if a.zmax > 16:
        sys.exit("zmax > 16 refused - that is a huge number of tiles.")

    total = 0
    if not a.no_overview:
        print("Philippines overview (z5-8):")
        total += fetch_box(*PH_BBOX, 5, 8, "overview")

    if a.at:
        lat, lon = a.at
        where = f"{lat:.3f},{lon:.3f}"
    else:
        pc = locate_pc()
        if not pc:
            sys.exit("No --at given and could not locate this computer "
                     "(offline?). Re-run with --at LAT LON.")
        lat, lon = pc["lat"], pc["lon"]
        where = f"{pc.get('city') or 'PC location'} ({lat:.3f},{lon:.3f})"

    print(f"Detail around {where}, z{a.zmin}-{a.zmax}, ±{a.radius}°:")
    r = a.radius
    total += fetch_box(lat + r, lon - r, lat - r, lon + r, a.zmin, a.zmax, "detail")
    print(f"Done. {total} new tiles in {TILES}")


if __name__ == "__main__":
    main()
