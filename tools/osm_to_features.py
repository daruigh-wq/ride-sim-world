#!/usr/bin/env python3
"""
osm_to_features.py — fetch OSM roads/water/landuse for a route, project to ENU.

Queries the OpenStreetMap Overpass API for the route's bounding box (+ margin),
projects every feature into the SAME local East-North-Up metric frame
route_to_world.py / dem_to_heightmap.py use (origin = route's first trackpoint),
and writes features.json. The Godot world drapes these onto the DEM terrain so
the empty flats get cross-streets, paths, creeks, and land cover.

Features carry only (x, z) in metres — no elevation; Godot samples terrain height
and drapes them, exactly like the main road ribbon.

Output features.json:
  {
    "origin": {"lat":.., "lon":..},
    "roads":     [{"class":"residential", "pts":[[x,z],...]}, ...],   # polylines
    "waterways": [{"class":"stream",      "pts":[[x,z],...]}, ...],   # polylines
    "water":     [{"pts":[[x,z],...]}, ...],                          # polygons
    "landuse":   [{"class":"forest",      "pts":[[x,z],...]}, ...]    # polygons
  }

The raw Overpass response is cached (data/osm_cache.json) so re-runs are offline,
matching the DEM tile cache. Stdlib only — no osmium/overpy.

  python tools/osm_to_features.py --route godot/data/route.json \
         --out-dir godot/data
"""
import argparse
import json
import math
import os
import sys
import urllib.parse
import urllib.request

R_EARTH = 6371000.0
M_PER_DEG = math.pi * R_EARTH / 180.0   # ~111195 m/deg, matches the other tools

# Public Overpass mirrors, tried in order.
OVERPASS_URLS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
]


def build_query(s, w, n, e):
    """Overpass QL: roads, waterways, water bodies, land cover in the bbox."""
    bb = f"({s},{w},{n},{e})"
    return f"""[out:json][timeout:120];
(
  way["highway"]{bb};
  way["waterway"]{bb};
  way["natural"="water"]{bb};
  way["landuse"]{bb};
  way["leisure"~"park|nature_reserve|golf_course|pitch"]{bb};
  way["natural"="wood"]{bb};
);
(._;>;);
out body qt;
"""


def fetch(query, cache):
    if os.path.exists(cache):
        print(f"using cached {cache}")
        with open(cache) as f:
            return json.load(f)
    body = urllib.parse.urlencode({"data": query}).encode()
    last = None
    for url in OVERPASS_URLS:
        try:
            print(f"querying {url} ...")
            req = urllib.request.Request(
                url, data=body, headers={"User-Agent": "ride-sim-world/0.1"})
            with urllib.request.urlopen(req, timeout=180) as r:
                obj = json.loads(r.read().decode())
            with open(cache, "w") as f:
                json.dump(obj, f)
            return obj
        except Exception as ex:           # noqa: BLE001 — try the next mirror
            print(f"  failed: {ex}", file=sys.stderr)
            last = ex
    sys.exit(f"all Overpass endpoints failed; last error: {last}")


def build_building_query(s, w, n, e):
    """Overpass QL: building footprints only (queried/cached separately — big)."""
    return f"""[out:json][timeout:180];
(
  way["building"]({s},{w},{n},{e});
);
(._;>;);
out body qt;
"""


def densify(points, step=3.0):
    """Sample a route polyline (list of {x,z}) to ~uniform `step` m points."""
    out = []
    for i in range(len(points) - 1):
        ax, az = points[i]["x"], points[i]["z"]
        bx, bz = points[i + 1]["x"], points[i + 1]["z"]
        seg = math.hypot(bx - ax, bz - az)
        if seg < 1e-6:
            continue
        k = max(1, int(seg // step))
        for j in range(k):
            f = j / k
            out.append((ax + (bx - ax) * f, az + (bz - az) * f))
    out.append((points[-1]["x"], points[-1]["z"]))
    return out


def point_in_poly(x, z, poly):
    """Ray-cast point-in-polygon. poly is a list of [x, z] (closed or open)."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, zi = poly[i][0], poly[i][1]
        xj, zj = poly[j][0], poly[j][1]
        if (zi > z) != (zj > z) and x < (xj - xi) * (z - zi) / (zj - zi) + xi:
            inside = not inside
        j = i
    return inside


def _seg_dist(px, pz, ax, az, bx, bz):
    dx, dz = bx - ax, bz - az
    l2 = dx * dx + dz * dz
    if l2 < 1e-9:
        return math.hypot(px - ax, pz - az)
    t = max(0.0, min(1.0, ((px - ax) * dx + (pz - az) * dz) / l2))
    return math.hypot(px - (ax + t * dx), pz - (az + t * dz))


def dist_to_poly(px, pz, poly):
    return min(_seg_dist(px, pz, poly[i][0], poly[i][1], poly[i + 1][0], poly[i + 1][1])
               for i in range(len(poly) - 1))


def parse_height(tags):
    """Best-effort building height in metres from OSM tags."""
    h = tags.get("height")
    if h:
        try:
            return max(2.5, float(str(h).split()[0].replace(",", ".")))
        except ValueError:
            pass
    lv = tags.get("building:levels")
    if lv:
        try:
            return max(2.5, float(str(lv).split(";")[0]) * 3.0)
        except ValueError:
            pass
    return 6.0   # ~2 storeys


def classify(tags):
    """(kind, class) for a way, or (None, None) to skip. kind drives geometry."""
    if "highway" in tags:
        return "road", tags["highway"]
    if "waterway" in tags:
        return "waterway", tags["waterway"]
    if tags.get("natural") == "water":
        return "water", "water"
    if "landuse" in tags:
        return "landuse", tags["landuse"]
    if tags.get("natural") == "wood":
        return "landuse", "forest"
    if "leisure" in tags:
        return "landuse", tags["leisure"]
    return None, None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--route", default="data/route.json")
    ap.add_argument("--out-dir", default="data")
    ap.add_argument("--margin-m", type=float, default=500.0,
                    help="extend query past route bbox (scenery headroom)")
    ap.add_argument("--building-margin", type=float, default=200.0,
                    help="keep only buildings within ~this distance of the route")
    ap.add_argument("--road-clearance", type=float, default=6.0,
                    help="cull buildings the route enters or passes within this of (0=off)")
    ap.add_argument("--no-buildings", action="store_true",
                    help="skip the (large) building footprint query")
    ap.add_argument("--refetch", action="store_true",
                    help="ignore the cache and re-query Overpass")
    args = ap.parse_args()

    with open(args.route) as f:
        route = json.load(f)
    o = route["origin"]
    bb = route["bbox"]
    lat0, lon0 = o["lat"], o["lon"]
    coslat0 = math.cos(math.radians(lat0))

    dlat = args.margin_m / M_PER_DEG
    dlon = args.margin_m / (M_PER_DEG * coslat0)
    s, n = bb["lat_min"] - dlat, bb["lat_max"] + dlat
    w, e = bb["lon_min"] - dlon, bb["lon_max"] + dlon

    cache = os.path.join(args.out_dir, "osm_cache.json")
    if args.refetch and os.path.exists(cache):
        os.remove(cache)
    data = fetch(build_query(s, w, n, e), cache)

    nodes = {}
    ways = []
    for el in data.get("elements", []):
        if el["type"] == "node":
            nodes[el["id"]] = (el["lat"], el["lon"])
        elif el["type"] == "way":
            ways.append(el)

    def proj(lat, lon):
        return [round((lon - lon0) * coslat0 * M_PER_DEG, 1),
                round((lat - lat0) * M_PER_DEG, 1)]

    out = {"origin": {"lat": lat0, "lon": lon0},
           "roads": [], "waterways": [], "water": [], "landuse": [], "buildings": []}

    for way in ways:
        tags = way.get("tags", {})
        kind, cls = classify(tags)
        if kind is None:
            continue
        refs = way.get("nodes", [])
        pts = [proj(*nodes[r]) for r in refs if r in nodes]
        if len(pts) < 2:
            continue

        if kind in ("road", "waterway"):
            out[kind + "s" if kind == "road" else "waterways"].append(
                {"class": cls, "pts": pts})
        else:  # area: water / landuse — needs a closed ring of >=3 distinct pts
            if len(pts) < 4:
                continue
            if pts[0] != pts[-1]:
                pts.append(pts[0])          # close the ring
            if kind == "water":
                out["water"].append({"pts": pts})
            else:
                out["landuse"].append({"class": cls, "pts": pts})

    # Buildings: separate query/cache (huge), kept only near the route so dense
    # urban bboxes don't explode the mesh. Grid-bin the route, keep a building if
    # its centroid cell or a neighbor is occupied (~within building_margin).
    if not args.no_buildings:
        cell = args.building_margin
        route_cells = set()
        for p in route["points"]:
            route_cells.add((int(p["x"] // cell), int(p["z"] // cell)))

        bcache = os.path.join(args.out_dir, "osm_buildings_cache.json")
        if args.refetch and os.path.exists(bcache):
            os.remove(bcache)
        bdata = fetch(build_building_query(s, w, n, e), bcache)
        bnodes = {}
        bways = []
        for el in bdata.get("elements", []):
            if el["type"] == "node":
                bnodes[el["id"]] = (el["lat"], el["lon"])
            elif el["type"] == "way" and "building" in el.get("tags", {}):
                bways.append(el)

        # densified route + fine grid for the in-the-road clearance test (route.json
        # is coarse, so test against a 3 m-sampled line, not its raw vertices).
        clear = args.road_clearance
        ccell = max(15.0, clear * 3.0)
        dense = densify(route["points"], 3.0)
        rgrid = {}
        for rx, rz in dense:
            rgrid.setdefault((int(rx // ccell), int(rz // ccell)), []).append((rx, rz))

        kept = dropped = in_road = 0
        for way in bways:
            pts = [proj(*bnodes[r]) for r in way.get("nodes", []) if r in bnodes]
            if len(pts) < 4:
                continue
            cx = sum(p[0] for p in pts) / len(pts)
            cz = sum(p[1] for p in pts) / len(pts)
            gx, gz = int(cx // cell), int(cz // cell)
            near = any((gx + dx, gz + dz) in route_cells
                       for dx in (-1, 0, 1) for dz in (-1, 0, 1))
            if not near:
                dropped += 1
                continue
            if pts[0] != pts[-1]:
                pts.append(pts[0])
            if clear > 0:
                xs = [p[0] for p in pts]
                zs = [p[1] for p in pts]
                cand = []
                for gx2 in range(int((min(xs) - clear) // ccell), int((max(xs) + clear) // ccell) + 1):
                    for gz2 in range(int((min(zs) - clear) // ccell), int((max(zs) + clear) // ccell) + 1):
                        cand.extend(rgrid.get((gx2, gz2), []))
                if any(point_in_poly(rx, rz, pts) or dist_to_poly(rx, rz, pts) < clear
                       for rx, rz in cand):
                    in_road += 1
                    continue
            out["buildings"].append({"pts": pts, "h": parse_height(way.get("tags", {}))})
            kept += 1
        print(f"buildings {kept} kept, {dropped} far, {in_road} culled in-road "
              f"(<{clear:.0f} m / inside footprint)")

    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, "features.json")
    with open(path, "w") as f:
        json.dump(out, f)

    print(f"roads {len(out['roads'])}  waterways {len(out['waterways'])}  "
          f"water {len(out['water'])}  landuse {len(out['landuse'])}  "
          f"buildings {len(out['buildings'])}")
    print(f"  -> {path}")


if __name__ == "__main__":
    main()
