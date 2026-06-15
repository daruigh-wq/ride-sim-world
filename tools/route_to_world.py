#!/usr/bin/env python3
"""
route_to_world.py — TCX/GPX ride track -> route.json in a local metric frame.

Reads a recorded ride (the road centerline) and emits a route the Godot world
consumes: points in a local East-North-Up frame (metres), with cumulative
distance — the SAME distance parameter ride_sim drives the camera with. The
first trackpoint is the local origin; the DEM step reuses this origin so terrain
and road align.

Output route.json:
  {
    "origin": {"lat":.., "lon":.., "elev":..},   # local frame anchor
    "bbox":   {"lat_min":.., "lat_max":.., "lon_min":.., "lon_max":..},
    "length_m": ..,
    "points": [{"x":east_m, "y":elev_m, "z":north_m, "d":dist_m}, ...]
  }

Godot uses Y-up; we map East->+X, North->+Z, elevation->+Y. Stdlib only.
"""
import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET

R_EARTH = 6371000.0


def _strip_ns(tag):
    return tag.rsplit("}", 1)[-1]


def parse_fit(path):
    """Return list of (lat, lon, elev) from a FIT file (Garmin/RWGPS course or
    activity). lat/lon are semicircles; elevation = enhanced_altitude/altitude."""
    try:
        import fitparse
    except ModuleNotFoundError:
        sys.exit("reading .fit needs the fitparse package: pip install fitparse")
    SEMI = 180.0 / 2 ** 31
    pts = []
    for m in fitparse.FitFile(path).get_messages("record"):
        d = {f.name: f.value for f in m}
        la, lo = d.get("position_lat"), d.get("position_long")
        if la is None or lo is None:
            continue
        e = d.get("enhanced_altitude")
        if e is None:
            e = d.get("altitude")
        pts.append((la * SEMI, lo * SEMI, float(e) if e is not None else 0.0))
    return pts


def parse_track(path):
    """Return list of (lat, lon, elev) from a TCX, GPX, or FIT file."""
    if path.lower().endswith(".fit"):
        return parse_fit(path)
    tree = ET.parse(path)
    root = tree.getroot()
    pts = []

    # TCX: Trackpoint/Position/LatitudeDegrees + AltitudeMeters
    # GPX: trkpt[@lat,@lon]/ele
    for el in root.iter():
        tag = _strip_ns(el.tag)
        if tag == "Trackpoint":
            lat = lon = ele = None
            for c in el.iter():
                ct = _strip_ns(c.tag)
                if ct == "LatitudeDegrees":
                    lat = float(c.text)
                elif ct == "LongitudeDegrees":
                    lon = float(c.text)
                elif ct == "AltitudeMeters" and c.text is not None:
                    ele = float(c.text)
            if lat is not None and lon is not None:
                pts.append((lat, lon, ele if ele is not None else 0.0))
        elif tag == "trkpt":
            lat = float(el.get("lat"))
            lon = float(el.get("lon"))
            ele = 0.0
            for c in el:
                if _strip_ns(c.tag) == "ele" and c.text is not None:
                    ele = float(c.text)
            pts.append((lat, lon, ele))
    return pts


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(a))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("track", help="ride .tcx, .gpx, or .fit")
    ap.add_argument("--out", default="data/route.json")
    ap.add_argument("--min-step-m", type=float, default=2.0,
                    help="drop points closer than this (decimate dense logs)")
    ap.add_argument("--reverse", action="store_true",
                    help="reverse the route (match a reversed ride TCX)")
    args = ap.parse_args()

    pts = parse_track(args.track)
    if args.reverse:
        pts = pts[::-1]
    if len(pts) < 2:
        sys.exit(f"only {len(pts)} trackpoints parsed from {args.track}")

    lat0, lon0, elev0 = pts[0]
    coslat = math.cos(math.radians(lat0))
    M_PER_DEG = math.pi * R_EARTH / 180.0   # ~111195 m/deg

    out_pts = []
    dist = 0.0
    plat, plon = lat0, lon0
    lat_min = lat_max = lat0
    lon_min = lon_max = lon0

    for i, (lat, lon, ele) in enumerate(pts):
        if i > 0:
            step = haversine(plat, plon, lat, lon)
            if step < args.min_step_m and i != len(pts) - 1:
                continue
            dist += step
        x = (lon - lon0) * coslat * M_PER_DEG   # East
        z = (lat - lat0) * M_PER_DEG            # North
        out_pts.append({"x": round(x, 2), "y": round(ele, 2),
                        "z": round(z, 2), "d": round(dist, 2)})
        plat, plon = lat, lon
        lat_min, lat_max = min(lat_min, lat), max(lat_max, lat)
        lon_min, lon_max = min(lon_min, lon), max(lon_max, lon)

    world = {
        "origin": {"lat": lat0, "lon": lon0, "elev": elev0},
        "bbox": {"lat_min": lat_min, "lat_max": lat_max,
                 "lon_min": lon_min, "lon_max": lon_max},
        "length_m": round(dist, 1),
        "points": out_pts,
    }
    with open(args.out, "w") as f:
        json.dump(world, f)
    span_x = (lon_max - lon_min) * coslat * M_PER_DEG
    span_z = (lat_max - lat_min) * M_PER_DEG
    print(f"{len(out_pts)} points, {dist/1000:.2f} km route")
    print(f"  bbox ~ {span_x:.0f} m E x {span_z:.0f} m N")
    print(f"  -> {args.out}")


if __name__ == "__main__":
    main()
