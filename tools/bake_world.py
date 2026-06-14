#!/usr/bin/env python3
"""
bake_world.py — one command: a route file → a ready-to-ride Godot world.
========================================================================
Orchestrates the whole pipeline into a single output directory (default the
Godot app's data/):

    route_to_world.py   route (.gpx/.tcx/.fit*) → route.json   (local ENU frame)
    dem_to_heightmap.py route.json              → heights.bin + world.json
    osm_to_features.py  route.json              → features.json (+ caches)
    gpx_to_tcx.py       route                   → <name>.tcx    (the ride_sim ride
                                                  file: SAME route + DEM grade)

Baking the world AND the TCX from one source guarantees ride_sim and the Godot
world drive the SAME physical route (otherwise positions don't line up).

Re-runs are offline: DEM tiles cache in <out>/tiles and the OSM response in
<out>/osm_cache.json. Pass --refetch to force a fresh OSM pull; delete <out>/tiles
to force fresh DEM.

Usage:
    python bake_world.py myroute.gpx                 # bake into godot/data (active)
    python bake_world.py myroute.gpx --out-dir worlds/sf
    python bake_world.py course.fit --reverse --avg 18

* route_to_world reads .gpx/.tcx; a .fit input still bakes terrain/OSM via its
  positions but the TCX step (gpx_to_tcx) is what reads .fit/.gpx for the ride.
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent
DEFAULT_DATA = REPO / "godot" / "data"


def run_step(n: int, total: int, title: str, argv: list) -> None:
    print(f"\n[{n}/{total}] {title}", flush=True)
    print("    $ " + " ".join(str(a) for a in argv), flush=True)
    t0 = time.time()
    r = subprocess.run([sys.executable] + [str(a) for a in argv], cwd=str(TOOLS))
    if r.returncode != 0:
        sys.exit(f"\n✗ step failed: {title} (exit {r.returncode}). World not complete.")
    print(f"    ✓ {time.time() - t0:.1f}s", flush=True)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("route", help="route file (.gpx, .tcx, or .fit)")
    ap.add_argument("--out-dir", default=str(DEFAULT_DATA),
                    help="world output directory (default: godot/data, the active world)")
    ap.add_argument("--zoom", type=int, default=14,
                    help="DEM tile zoom (default 14, ~7.6 m/px)")
    ap.add_argument("--grid", type=int, default=1024,
                    help="heightfield max dimension (default 1024)")
    ap.add_argument("--margin-m", type=float, default=500.0,
                    help="OSM feature margin beyond the route bbox (default 500)")
    ap.add_argument("--dem-margin-m", type=float, default=800.0,
                    help="terrain margin beyond the route bbox (default 800)")
    ap.add_argument("--no-buildings", action="store_true",
                    help="skip OSM building extrusion")
    ap.add_argument("--refetch", action="store_true",
                    help="ignore the cached OSM response and pull fresh")
    ap.add_argument("--no-tcx", action="store_true",
                    help="skip generating the matching ride_sim TCX")
    ap.add_argument("--avg", type=float, default=22.0,
                    help="synthesized average speed km/h for the TCX (default 22)")
    ap.add_argument("--reverse", action="store_true",
                    help="reverse the route for the TCX (one-way-street course workaround)")
    args = ap.parse_args()

    route = Path(args.route).resolve()
    if not route.exists():
        sys.exit(f"route not found: {route}")
    out = Path(args.out_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    route_json = out / "route.json"
    ext = route.suffix.lower()

    want_tcx = (not args.no_tcx) and ext in (".gpx", ".fit")
    total = 4 if want_tcx else 3
    t_all = time.time()
    print(f"Baking world from {route.name}  →  {out}", flush=True)

    run_step(1, total, "route → route.json", [
        TOOLS / "route_to_world.py", route, "--out", route_json])

    run_step(2, total, f"DEM terrain (zoom {args.zoom}, grid {args.grid})", [
        TOOLS / "dem_to_heightmap.py", "--route", route_json, "--out-dir", out,
        "--zoom", args.zoom, "--grid", args.grid, "--margin-m", args.dem_margin_m])

    osm = [TOOLS / "osm_to_features.py", "--route", route_json, "--out-dir", out,
           "--margin-m", args.margin_m]
    if args.no_buildings:
        osm.append("--no-buildings")
    if args.refetch:
        osm.append("--refetch")
    run_step(3, total, "OSM features (roads / water / landuse / buildings)", osm)

    tcx_path = None
    if want_tcx:
        tcx_path = out / (route.stem + ".tcx")
        gx = [TOOLS / "gpx_to_tcx.py", route, tcx_path,
              "--avg", args.avg, "--dem-zoom", args.zoom,
              "--dem-tiles", out / "tiles"]
        if args.reverse:
            gx.append("--reverse")
        run_step(4, total, "ride_sim TCX (matching route + grade)", gx)

    active = out.resolve() == DEFAULT_DATA.resolve()
    print(f"\n✓ world baked in {time.time() - t_all:.0f}s  →  {out}")
    if active:
        print("  This is the active world — just run the Godot app.")
    else:
        print(f"  To activate: copy its contents into {DEFAULT_DATA} "
              "(or point the Godot project's data path here).")
    if tcx_path is not None:
        print(f"  In ride_sim, load the matching ride file:  {tcx_path}")


if __name__ == "__main__":
    main()
