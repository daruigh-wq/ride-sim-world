# Ride Sim — Product Roadmap

_Last updated: 2026-06-12_

A planning document spanning the three repos. Living doc — edit freely.

## Vision

**One offline desktop app** that lets a rider experience a real route on a smart
trainer through **either** of two world sources:

- **A — Video world:** the rider's own (ideally stabilized) ride footage, played
  in sync with trainer telemetry. _(today: works on Windows, rough on macOS.)_
- **B — Generated world:** a 3D world built from **public open data** (DEM
  terrain + OSM roads/buildings/landuse), flown on-rails by the same telemetry.
  _(today: terrain + road + chase camera render; layers pending.)_

**Non-negotiables:**
- **No account, no web portal, no per-ride cloud.** Riding is always 100% offline.
- The only network use is a **one-time, local, pre-cacheable** fetch of public
  open data when *building* a new virtual route. Processing is always local.
- One launcher, one mental model. Two renderers behind a shared telemetry contract.

**Fidelity tiers (deliberate):**
- DEM/OSM procedural world → *good stylized* (clean low-poly, readable, offline).
- Splat world (separate hard track) → *photoreal*, GPU-bound, later.

## Architecture (single app, two renderers, one contract)

```
            ride_sim  (Python / Qt — the brain)
   BLE FTMS + HR · telemetry · HUD · recording · route mgmt · "which world?" picker
                       |                         |
        distance/speed |  (video: QMediaPlayer)  | distance/speed over UDP
                       v                         v
              [ Video renderer ]        [ Godot world renderer ]
              user footage, synced      DEM terrain + OSM layers, on-rails
                                                 ^
                                        (same socket later accepts the splat world)
```

Contract: `ride-sim/docs/engine_interface.md` — JSON-lines UDP
`{"distance_m":.., "speed_mps":..}`. Keeps graphics (Godot) and hardware/logic
(Python) decoupled so neither rewrites the other.

## End-user workflow

**Path A — Video ride**
1. Ride outdoors with a stabilized camera (GoPro HyperSmooth, or post via the
   open-source offline **Gyroflow**) + record GPS.
2. App → *New Video Ride* → pick video + GPX/TCX.
3. Ride. Telemetry drives video playback rate. Offline.

**Path B — Virtual ride**
1. Obtain a route as GPX/TCX (Garmin / phone / Strava export / drawn route).
2. App → *New Virtual Ride* → pick the GPX.
3. App bakes the world (progress bar): DEM terrain → OSM features → assembled
   world. One-time internet, minutes, cached to disk.
4. Ride. Telemetry drives the on-rails camera. Offline from here on.

## Phases

Effort in **build sessions** (one focused Claude Code working session).
Calendar assumes ~2 sessions/week, part-time.

| # | Phase | Scope | Sessions |
|---|-------|-------|----------|
| P0 | **Foundation** ✅ | Pipeline + Godot terrain/road/on-rails camera; chase cam; finer bake | done |
| P1 | **OSM feature layer** | `osm_to_features.py` (Overpass, route bbox, same ENU frame). Cross-streets/paths draped & classified (width/color). Water + landuse → terrain tint. | 2–3 |
| P2 | **Buildings** | Extrude OSM `building=*` footprints; height from tag / `levels×3 m` / default. Low-poly town along the flats. | 2–3 |
| P3 | **Visual polish** | Terrain textures/triplanar; vegetation billboards (`natural=tree`, forest polys); time-of-day sun via solar ephemeris (Shadowmap-style); fog/LOD/cull tuning. | 3–5 |
| P4 | **Bake UX** | One entry point: GPX in → world dir out, with progress + caching + offline pre-fetch. Robust to missing tags / sparse coverage. | 2–3 |
| P5 | **Unification** | ride_sim startup "which world?" picker; launch Godot for virtual rides; the few-line live UDP emitter (virtual_dist_m + speed_mps_smoothed already in SharedState). | 2–4 |
| P6 | **Distribution** | Godot export templates; PyInstaller for ride_sim; bundle both; offline map tiles; first-run UX. | 3–5 |
| P7 | **Video-path hardening** | macOS audio stutter (the hard one — possibly native AVFoundation backend); offline map; Gyroflow stabilization hook. | 3–6 |
| P8 | **Splat path** _(stretch)_ | Photoreal backend on the same UDP socket. GPU-bound, separate repo. | TBD |

**To a polished offline *unified* app (P1–P7): ~20–35 sessions ≈ 3–4 months part-time.**
Virtual-world-only milestone (P1–P5): ~11–16 sessions.

## Resource usage

**Development (our token budget).** The cost driver is *debugging loops* (Godot
4.6 quirks, mac specifics), not code volume. A clean build session is modest; a
debug-heavy one is several × that. Mitigation: batch related work per session,
verify locally before iterating, keep the UDP decoupling so failures are isolated
to one renderer. Rough order: virtual-world track (P1–P5) is a meaningful but
bounded budget; P6–P7 (packaging + mac audio) carry the most uncertainty.

**End-user machine (per virtual route).**

| Resource | Estimate |
|----------|----------|
| One-time network | DEM tiles + OSM for a ~33 km route ≈ tens of MB (cached after) |
| Disk per route | heights.bin ~3–4 MB (zoom 14, 822×1024) + OSM JSON (single-digit MB) + tile cache. Video rides: the video file (GB-scale). |
| Bake time | minutes (network-bound first run, then instant) |
| RAM at ride time | ~1–2 GB (Godot world) |
| GPU | Apple Silicon / any modern discrete or decent integrated GPU. Terrain is ~842k verts; needs LOD work (P3) for low-end. |
| CPU | Light at ride time; Python BLE loop is 4 Hz. |

## Recommended build order ("go for the gold")

Bottom-up so each phase is independently visible and bugs stay contained:

**P1 → P2 → P4 → P5** first (gets a *complete, ridable, unified* virtual world
with real streets and buildings, baked from one GPX, driven live by the trainer —
the end-to-end thesis), **then P3** (make it pretty) **then P6** (ship it),
**then P7** (fix the video path), **P8** stretch.

Rationale: P3 polish on an incomplete world is wasted re-work; getting the full
data+unification skeleton solid first means polish lands once.

## Open risks

- **macOS video audio stutter** (P7) — unsolved; may force a backend change.
- **OSM coverage variance** — rural routes have sparse buildings; need graceful
  fallbacks (P2/P4).
- **Godot perf on big terrain** — LOD/chunking may be needed for long routes (P3).
- **Open-data licensing** — fine for personal use; check terms before
  redistributing *baked* assets (already noted in repo READMEs).

## Repos

- **ride-sim** — the app/brain (Python/Qt). Beta-shipping; keep changes scoped.
- **ride-sim-world** (this) — DEM/OSM procedural world (Godot). Active dev.
- **ride-sim-splat** — photoreal splat world. Scaffold only, deferred.
