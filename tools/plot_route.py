#!/usr/bin/env python3
"""
plot_route.py — top-down sanity plot of route.json over features.json.

Renders the ridden centerline on top of the OSM road network in the SAME local
ENU frame (East=+x right, North=+z up — standard map orientation), so you can
eyeball whether the route matches reality, sits on its road, and isn't mirrored.

  python tools/plot_route.py --data-dir godot/data --out /tmp/route_check.png
"""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="godot/data")
    ap.add_argument("--out", default="/tmp/route_check.png")
    args = ap.parse_args()

    route = json.load(open(os.path.join(args.data_dir, "route.json")))
    pts = [(p["x"], p["z"]) for p in route["points"]]

    fig, ax = plt.subplots(figsize=(12, 12))

    fpath = os.path.join(args.data_dir, "features.json")
    if os.path.exists(fpath):
        feats = json.load(open(fpath))
        segs = [[(a[0], a[1]) for a in r["pts"]] for r in feats.get("roads", [])]
        ax.add_collection(LineCollection(segs, colors="0.75", linewidths=0.4, zorder=1))
        water = [[(a[0], a[1]) for a in w["pts"]] for w in feats.get("waterways", [])]
        ax.add_collection(LineCollection(water, colors="#3a7", linewidths=0.6, zorder=2))

    # ridden route
    xs = [p[0] for p in pts]
    zs = [p[1] for p in pts]
    ax.plot(xs, zs, "-", color="red", linewidth=1.4, zorder=5, label="ridden route")

    # direction markers: dots at every ~5 km along the route
    step = 5000.0
    nextd = 0.0
    for p in route["points"]:
        if p["d"] >= nextd:
            ax.plot(p["x"], p["z"], "o", color="blue", markersize=4, zorder=6)
            ax.annotate(f"{p['d']/1000:.0f}km", (p["x"], p["z"]), fontsize=8,
                        color="blue", zorder=7)
            nextd += step

    ax.plot(xs[0], zs[0], "o", color="green", markersize=10, zorder=8, label="START")
    ax.plot(xs[-1], zs[-1], "s", color="black", markersize=8, zorder=8, label="END")

    ax.set_aspect("equal")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("route (red) over OSM roads — North up, East right")
    ax.legend(loc="upper right")
    ax.margins(0.02)
    fig.savefig(args.out, dpi=110, bbox_inches="tight")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
