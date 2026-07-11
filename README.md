# ride-sim-world

A fully **offline**, real-elevation cycling world: public DEM data → low-poly
shaded terrain with your actual road draped on it, flown on-rails by
[ride_sim](https://github.com/daruigh-wq/ride-sim)'s distance signal. No account,
no fantasy scenery, no 360-camera capture — the lightweight alternative to the
splat path (a photogrammetry-based world, planned/not yet public).

> **Status: working prototype.** The data pipeline (route + DEM → Godot assets)
> runs end-to-end; a Godot 4 project builds the terrain, road, and on-rails
> camera procedurally and accepts live ride_sim telemetry. Tested on the
> 33 km San Jose test route. Visual polish (textures, vegetation, LOD) is next.

## Why this exists

Indoor cycling apps that use *real* terrain are tethered to accounts and servers;
the offline ones use fantasy scenery. This renders the road you actually ride,
from open elevation data, and is driven by the same `distance_along_route` signal
ride_sim already produces — so it's the same on-rails idea as the splat track,
just sourced from a DEM instead of photogrammetry. Far less effort, no GPU
training, runs anywhere Godot runs.

## Pipeline

```
1. ROUTE    tools/route_to_world.py   ride .tcx/.gpx -> data/route.json
            (road centerline in a local metric ENU frame + cumulative distance)
2. TERRAIN  tools/dem_to_heightmap.py fetch DEM tiles -> data/heights.bin + world.json
            (AWS Terrain Tiles, terrarium PNG -> sampled heightfield; tiles cached)
3. WORLD    godot/                     Godot 4 builds terrain + draped road,
            flies a camera by distance (demo auto-advance OR live ride_sim UDP)
```

Steps 1–2 are stdlib + numpy + PIL (no GDAL). Step 3 needs **Godot 4.2+**.

## Quick start

```bash
# 1. road centerline from any recorded ride
python tools/route_to_world.py myride.tcx --out godot/data/route.json

# 2. terrain for that route's bbox (fetches ~a dozen tiles, then offline)
python tools/dem_to_heightmap.py --route godot/data/route.json \
       --out-dir godot/data --zoom 13 --grid 512

# 3. open godot/ in Godot 4 and press Play (F5)
#    — camera auto-flies the route in demo mode.
```

Keys in the world: **SPACE** pause, **+/−** demo speed, **ESC** quit.

## Driving it from ride_sim (live)

The world listens on `udp:5005` for JSON-lines telemetry matching
`ride-sim/docs/engine_interface.md`:

```json
{"distance_m": 1234.5, "speed_mps": 7.0}
```

It snaps the camera to `distance_m` and dead-reckons with `speed_mps` between
packets, so 4 Hz is plenty smooth. Test the path without ride_sim:

```bash
python tools/mock_feed.py --route godot/data/route.json --speed 7
```

Wiring real ride_sim is a few lines in its sync loop (it already has
`virtual_dist_m` and `speed_mps_smoothed` in SharedState) — emit a UDP packet
next to the existing grade send. Kept out of ride_sim for now to preserve the
shippable beta.

## Tuning

- `--zoom` 13 (~15 m/px, light) … 14 (~7.6 m/px, sharper, ~4× tiles).
- `--grid` max heightfield dimension — 512 is low-poly and fast; raise for detail.
- `--margin-m` how far terrain extends past the route (scenery headroom).
- In Godot: `road_width`, `eye_height`, `look_ahead_m`, `demo_speed`, `udp_port`
  are exported vars on the Main node.

## Data & licensing

Terrain tiles come from the public **AWS Terrain Tiles** dataset (aggregated
SRTM / 3DEP / etc.). Fine for a personal prototype; check the source datasets'
terms before redistributing baked assets. Raw tiles, heightfields, and route
exports are git-ignored — regenerate them from your own rides.

## Relationship to ride_sim & the splat track

Three repos, one contract:
- **ride-sim** — the app; emits `distance_along_route` (+ speed/grade/heading).
- **ride-sim-world** (this) — DEM terrain world, the easy/offline backend.
- **ride-sim-splat** — photogrammetric splat world, the high-fidelity backend *(planned; not yet a public repo)*.

Both world repos render the same on-rails camera from the same distance signal
(`ride-sim/docs/engine_interface.md`). Pick a backend per route by effort vs
fidelity.
