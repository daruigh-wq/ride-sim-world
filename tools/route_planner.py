#!/usr/bin/env python3
"""
route_planner.py — trace a road-following route through OSM streets (offline).

Builds a graph from OSM bikeable highways in the waypoints' bounding box, snaps
each waypoint to the nearest graph node, and Dijkstra-routes between consecutive
waypoints along real streets. Emits a GPX the normal pipeline ingests
(route_to_world -> dem_to_heightmap -> osm_to_features). No account, no web
router — just Overpass (cached) + stdlib.

  python tools/route_planner.py --out godot/data/sf_route.gpx --loop \
         --waypoint 37.7955,-122.3937 --waypoint 37.7952,-122.4027 ...

Waypoints are lat,lon. --loop closes back to the first. --plot writes a PNG.
"""
import argparse
import heapq
import json
import math
import os
import sys
import urllib.parse
import urllib.request

R_EARTH = 6371000.0
OVERPASS_URLS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
]

# Highways a bike can plausibly use; excludes motorway/trunk and bare footway/steps.
BIKEABLE = {
    "primary", "secondary", "tertiary", "residential", "unclassified",
    "living_street", "road", "cycleway", "service", "pedestrian", "path", "track",
    "primary_link", "secondary_link", "tertiary_link",
}


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(a))


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
        except Exception as ex:                      # noqa: BLE001
            print(f"  failed: {ex}", file=sys.stderr)
            last = ex
    sys.exit(f"all Overpass endpoints failed; last error: {last}")


def build_graph(data):
    nodes = {}
    for el in data.get("elements", []):
        if el["type"] == "node":
            nodes[el["id"]] = (el["lat"], el["lon"])
    adj = {}
    for el in data.get("elements", []):
        if el["type"] != "way":
            continue
        if el.get("tags", {}).get("highway") not in BIKEABLE:
            continue
        refs = [r for r in el.get("nodes", []) if r in nodes]
        for a, b in zip(refs, refs[1:]):
            la, lb = nodes[a], nodes[b]
            d = haversine(la[0], la[1], lb[0], lb[1])
            adj.setdefault(a, []).append((b, d))
            adj.setdefault(b, []).append((a, d))     # bidirectional (ignore oneway)
    return nodes, adj


def largest_component(adj):
    """Biggest connected set of nodes — avoids snapping onto a disconnected stub."""
    seen, best = set(), set()
    for start in adj:
        if start in seen:
            continue
        comp, stack = set(), [start]
        while stack:
            u = stack.pop()
            if u in comp:
                continue
            comp.add(u)
            seen.add(u)
            for v, _ in adj[u]:
                if v not in comp:
                    stack.append(v)
        if len(comp) > len(best):
            best = comp
    return best


def nearest_node(nodes, candidates, lat, lon):
    best, bd = None, 1e18
    for nid in candidates:
        la, lo = nodes[nid]
        d = (la - lat) ** 2 + (lo - lon) ** 2
        if d < bd:
            bd, best = d, nid
    return best


def dijkstra(adj, src, dst):
    if src == dst:
        return [src]
    dist = {src: 0.0}
    prev = {}
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == dst:
            break
        if d > dist.get(u, 1e18):
            continue
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, 1e18):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if dst not in dist:
        return None
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    path.reverse()
    return path


def write_gpx(coords, path):
    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<gpx version="1.1" creator="route_planner">', "<trk><trkseg>"]
    for la, lo in coords:
        lines.append(f'<trkpt lat="{la:.7f}" lon="{lo:.7f}"></trkpt>')
    lines += ["</trkseg></trk>", "</gpx>"]
    with open(path, "w") as f:
        f.write("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--waypoint", action="append", required=True,
                    metavar="LAT,LON", help="repeat for each via point, in order")
    ap.add_argument("--out", default="godot/data/route.gpx")
    ap.add_argument("--cache", default="godot/data/osm_graph_cache.json")
    ap.add_argument("--margin-m", type=float, default=1000.0,
                    help="graph bbox margin past the waypoints")
    ap.add_argument("--loop", action="store_true", help="return to the first waypoint")
    ap.add_argument("--plot", default="", help="optional PNG sanity plot path")
    args = ap.parse_args()

    wps = []
    for s in args.waypoint:
        la, lo = (float(x) for x in s.split(","))
        wps.append((la, lo))
    if len(wps) < 2:
        sys.exit("need at least 2 waypoints")

    lat0 = math.radians(sum(w[0] for w in wps) / len(wps))
    dlat = args.margin_m / (math.pi * R_EARTH / 180.0)
    dlon = dlat / max(0.2, math.cos(lat0))
    s = min(w[0] for w in wps) - dlat
    n = max(w[0] for w in wps) + dlat
    w = min(w[1] for w in wps) - dlon
    e = max(w[1] for w in wps) + dlon

    query = (f"[out:json][timeout:120];\n(way[\"highway\"]({s},{w},{n},{e}););\n"
             f"(._;>;);\nout body qt;\n")
    data = fetch(query, args.cache)
    nodes, adj = build_graph(data)
    comp = largest_component(adj)
    print(f"graph: {len(adj)} routable nodes, largest component {len(comp)}")

    wp_nodes = [nearest_node(nodes, comp, la, lo) for la, lo in wps]
    if args.loop:
        wp_nodes.append(wp_nodes[0])

    full = []
    for i, (a, b) in enumerate(zip(wp_nodes, wp_nodes[1:])):
        seg = dijkstra(adj, a, b)
        if seg is None:
            sys.exit(f"no street path between waypoint {i} and {i+1}")
        if full and seg and full[-1] == seg[0]:
            seg = seg[1:]
        full.extend(seg)

    coords = [nodes[nid] for nid in full]
    # drop consecutive duplicates
    coords = [c for i, c in enumerate(coords) if i == 0 or c != coords[i - 1]]
    length = sum(haversine(*coords[i], *coords[i + 1]) for i in range(len(coords) - 1))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    write_gpx(coords, args.out)
    print(f"routed {len(coords)} pts, {length/1000:.2f} km -> {args.out}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        fig, ax = plt.subplots(figsize=(11, 11))
        segs = []
        for el in data.get("elements", []):
            if el["type"] == "way" and el.get("tags", {}).get("highway") in BIKEABLE:
                pl = [(nodes[r][1], nodes[r][0]) for r in el.get("nodes", []) if r in nodes]
                if len(pl) > 1:
                    segs.append(pl)
        ax.add_collection(LineCollection(segs, colors="0.8", linewidths=0.4))
        ax.plot([c[1] for c in coords], [c[0] for c in coords], "-r", linewidth=1.8)
        for i, (la, lo) in enumerate(wps):
            ax.plot(lo, la, "o", color="blue", markersize=6)
            ax.annotate(str(i), (lo, la), fontsize=9, color="blue")
        ax.plot(coords[0][1], coords[0][0], "o", color="green", markersize=10)
        ax.set_aspect(1.0 / math.cos(lat0))
        ax.set_title(f"routed SF path (red) over streets — {length/1000:.1f} km")
        fig.savefig(args.plot, dpi=110, bbox_inches="tight")
        print(f"  plot -> {args.plot}")


if __name__ == "__main__":
    main()
