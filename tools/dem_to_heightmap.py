#!/usr/bin/env python3
"""
dem_to_heightmap.py — fetch public DEM tiles for a route and bake a heightfield.

Pulls AWS Terrain Tiles (terrarium-encoded PNGs: elev = R*256 + G + B/256 -
32768, metres) covering a route's bounding box + margin, mosaics them, and
samples a regular grid in the SAME local East-North-Up frame route_to_world.py
used. No GDAL/rasterio — just urllib + PIL + numpy.

Outputs (default data/):
  heights.bin   float32 LE, row-major, grid_h*grid_w, row 0 = NORTH edge.
                Absolute elevation in metres (same datum as the route's y).
  world.json    grid size + placement so Godot lays each cell at the right
                local-metric (x,z) and the road draped on it lines up.

Tiles are cached under data/tiles/ so re-runs are offline. Terrain tiles are
public-domain-ish aggregated SRTM/3DEP/etc; fine for a personal prototype.
"""
import argparse
import json
import math
import os
import sys
import urllib.request

import numpy as np
from PIL import Image

TILE_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
R_EARTH = 6371000.0
M_PER_DEG = math.pi * R_EARTH / 180.0


def deg2tile(lat, lon, z):
    n = 2 ** z
    xt = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    yt = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n
    return xt, yt


def fetch_tile(z, x, y, cache):
    fn = os.path.join(cache, f"{z}_{x}_{y}.png")
    if not os.path.exists(fn):
        url = TILE_URL.format(z=z, x=x, y=y)
        req = urllib.request.Request(url, headers={"User-Agent": "ride-sim-world/0.1"})
        with urllib.request.urlopen(req, timeout=20) as r, open(fn, "wb") as f:
            f.write(r.read())
    return np.asarray(Image.open(fn).convert("RGB"), dtype=np.float64)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--route", default="data/route.json")
    ap.add_argument("--out-dir", default="data")
    ap.add_argument("--zoom", type=int, default=13, help="tile zoom (13~15 m/px, 14~7.6)")
    ap.add_argument("--grid", type=int, default=512, help="max heightfield dimension (low-poly)")
    ap.add_argument("--margin-m", type=float, default=800.0, help="terrain beyond route bbox")
    args = ap.parse_args()

    with open(args.route) as f:
        route = json.load(f)
    o = route["origin"]
    bb = route["bbox"]
    lat0, lon0 = o["lat"], o["lon"]
    coslat0 = math.cos(math.radians(lat0))

    dlat = args.margin_m / M_PER_DEG
    dlon = args.margin_m / (M_PER_DEG * coslat0)
    lat_min, lat_max = bb["lat_min"] - dlat, bb["lat_max"] + dlat
    lon_min, lon_max = bb["lon_min"] - dlon, bb["lon_max"] + dlon

    cache = os.path.join(args.out_dir, "tiles")
    os.makedirs(cache, exist_ok=True)
    z = args.zoom

    # tile coverage (north = smaller ytile)
    xt0, yt_top = deg2tile(lat_max, lon_min, z)
    xt1, yt_bot = deg2tile(lat_min, lon_max, z)
    tx0, tx1 = int(math.floor(xt0)), int(math.floor(xt1))
    ty0, ty1 = int(math.floor(yt_top)), int(math.floor(yt_bot))
    ntiles = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
    print(f"zoom {z}: {ntiles} tiles ({tx1-tx0+1} x {ty1-ty0+1}) — fetching/caching...")

    # assemble mosaic of decoded elevation
    th = (ty1 - ty0 + 1) * 256
    tw = (tx1 - tx0 + 1) * 256
    mosaic = np.empty((th, tw), dtype=np.float64)
    for j, ty in enumerate(range(ty0, ty1 + 1)):
        for i, tx in enumerate(range(tx0, tx1 + 1)):
            rgb = fetch_tile(z, tx, ty, cache)
            elev = rgb[:, :, 0] * 256.0 + rgb[:, :, 1] + rgb[:, :, 2] / 256.0 - 32768.0
            mosaic[j*256:(j+1)*256, i*256:(i+1)*256] = elev
        print(f"  row {j+1}/{ty1-ty0+1}", file=sys.stderr)

    # grid sized to keep cells ~square, capped at --grid
    span_x = (lon_max - lon_min) * coslat0 * M_PER_DEG
    span_z = (lat_max - lat_min) * M_PER_DEG
    if span_x >= span_z:
        gw = args.grid
        gh = max(2, int(round(args.grid * span_z / span_x)))
    else:
        gh = args.grid
        gw = max(2, int(round(args.grid * span_x / span_z)))

    cols = np.linspace(lon_min, lon_max, gw)
    rows = np.linspace(lat_max, lat_min, gh)   # row 0 = north
    lon_grid, lat_grid = np.meshgrid(cols, rows)

    # global tile-pixel coords -> mosaic-pixel coords, bilinear sample
    n = 2 ** z
    gx = (lon_grid + 180.0) / 360.0 * n
    lat_r = np.radians(lat_grid)
    gy = (1.0 - np.arcsinh(np.tan(lat_r)) / math.pi) / 2.0 * n
    px = gx * 256.0 - tx0 * 256.0
    py = gy * 256.0 - ty0 * 256.0
    px = np.clip(px, 0, tw - 1.001)
    py = np.clip(py, 0, th - 1.001)
    x0i = np.floor(px).astype(int); y0i = np.floor(py).astype(int)
    fx = px - x0i; fy = py - y0i
    h00 = mosaic[y0i, x0i]; h10 = mosaic[y0i, x0i + 1]
    h01 = mosaic[y0i + 1, x0i]; h11 = mosaic[y0i + 1, x0i + 1]
    heights = ((h00 * (1 - fx) + h10 * fx) * (1 - fy)
               + (h01 * (1 - fx) + h11 * fx) * fy).astype(np.float32)

    heights.tofile(os.path.join(args.out_dir, "heights.bin"))

    x0 = (lon_min - lon0) * coslat0 * M_PER_DEG
    z0 = (lat_max - lat0) * M_PER_DEG
    world = {
        "grid_w": gw, "grid_h": gh,
        "x0": round(x0, 2), "z0": round(z0, 2),
        "mpp_x": round(span_x / (gw - 1), 4),
        "mpp_z": round(-span_z / (gh - 1), 4),   # rows march south
        "elev_min": float(round(heights.min(), 2)),
        "elev_max": float(round(heights.max(), 2)),
        "heights_bin": "heights.bin",
    }
    with open(os.path.join(args.out_dir, "world.json"), "w") as f:
        json.dump(world, f, indent=2)

    print(f"grid {gw} x {gh}  cells ~{world['mpp_x']:.1f} x {abs(world['mpp_z']):.1f} m")
    print(f"  elevation {world['elev_min']:.0f}..{world['elev_max']:.0f} m")
    print(f"  -> {args.out_dir}/heights.bin, {args.out_dir}/world.json")


if __name__ == "__main__":
    main()
