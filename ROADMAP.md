# Ride Sim — Product Roadmap

_Last updated: 2026-06-14 (P6 distribution strategy added)_

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
| P1 | **OSM feature layer** ✅ | `osm_to_features.py` (Overpass, route bbox, same ENU frame). Cross-streets/paths draped & classified (width/color). Water + landuse → terrain tint. | done |
| P2 | **Buildings** ✅ | Extrude OSM `building=*` footprints; height from tag / `levels×3 m` / default; in-road cull. Low-poly town along the flats. | done |
| P3 | **Visual polish** | Terrain textures/triplanar; vegetation billboards (`natural=tree`, forest polys); time-of-day sun via solar ephemeris (Shadowmap-style); fog/LOD/cull tuning. | 3–5 |
| P4 | **Bake UX** ✅ | `bake_world.py`: one command, route (.gpx/.tcx/.fit) → world dir (route+DEM+OSM) **+ matching ride_sim TCX**; progress + per-world tile/OSM caching (offline re-runs). | done |
| P5 | **Unification** ✅ | Live UDP emitter (ride_sim drives Godot, SIM+BLE) + startup "Ride type" picker (Video / Virtual world) that launches Godot and forces map-drive. | done |
| P6 | **Distribution** | One app, internally modular (see "P6 distribution strategy" below): PyInstaller ride_sim bundling the bake pipeline + an exported Godot renderer; in-app "New Virtual Ride" bakes; baked data in a user dir. Retires the launcher pickers. | 3–5 |
| P7 | **Video-path hardening** | macOS audio stutter (the hard one — possibly native AVFoundation backend); offline map; Gyroflow stabilization hook. | 3–6 |
| P8 | **Splat path** _(stretch)_ | Photoreal backend on the same UDP socket. GPU-bound, separate repo. | TBD |

**To a polished offline *unified* app (P1–P7): ~20–35 sessions ≈ 3–4 months part-time.**
Virtual-world-only milestone (P1–P5): ~11–16 sessions.

## P6 — Distribution strategy (decided 2026-06-14)

**Principle: one app is the single front door; the three concerns stay decoupled
*internally* (they already talk only through files + the UDP contract), but the
user never sees or wires up separate tools.** The dev-time "applets" (DEM fetch,
world build, renderer) become internal steps, not user-facing downloads.

**Three concerns, three lifetimes — and how each is packaged:**

| Concern | Runs | Packaged as |
|---|---|---|
| Build a world (route → terrain+OSM+TCX) | once per route, needs network | **internal module**, lazy-imported, run as a **subprocess** from "New Virtual Ride" (progress bar; a bake crash can't take down the ride app). numpy/PIL ride along in the bundle. |
| Render the world (Godot) | every ride, GPU | **exported native binary** (Godot export templates), **bundled inside** the app; ride_sim launches it over UDP → **retires the "Godot app" / "World project" pickers**. |
| Brain/ride (BLE, telemetry, HUD, map, recording) | every ride | **the bundle itself** — PyInstaller `.app`/`.exe`. ride_sim is already the brain + the P5 launcher. |

**Artifact layout:**
```
RideSim.app/
  ride_sim            (PyInstaller: brain + bake modules + numpy/PIL)
  world/RideSimWorld  (exported Godot renderer, launched over UDP)
~/<app-support>/RideSim/worlds/<route>/   (baked per route, generated on first use,
                                           never in the bundle — regenerable)
```

**End-user workflow:**
1. Install **RideSim** (one download).
2. *New Virtual Ride* → pick a GPX/TCX/FIT → app bakes (progress; one-time DEM+OSM
   fetch) → saved to the worlds dir. *(New Video Ride* is the same app.)
3. Ride → app launches the bundled renderer and drives it; fully offline from here.

**Open trade-off (decide at P6 build time):** bundling the Godot renderer +
numpy/PIL adds ~100–200 MB that a *video-only* user doesn't need.
- **Ship bundle-everything first** (recommended — don't optimize size early), then
- only if size becomes a real complaint, split a downloadable **"Virtual World"
  component** (renderer + bake deps fetched on first virtual ride). Still one app,
  one front door, just a deferred install.

**Why not separate applets:** the manual "download DEM tool → run world-maker →
point ride_sim at the output" dance is exactly the dev-time mess; most users won't
finish it. The clean internal seams mean unifying costs nothing architecturally.

### P6 progress

- **PyInstaller spike ✅ (2026-06-14, the biggest risk):** the full `ride_sim.py`
  bundles and launches clean on macOS arm64 (PySide6 6.11, PyInstaller 6.20). A
  minimal bundled binary proved **QtWebEngine + QtMultimedia both initialize from
  inside the bundle** — the fragile pieces survive packaging. ride_sim was already
  `sys._MEIPASS`-aware (Windows-tested), so Windows is likely close too.
- **Size finding:** the bundle is **~529 MB, and QtWebEngine is ~500 MB of that** —
  and QtWebEngine is the *Leaflet map*, which lives in the **core** app (every user,
  even video-only). So the Godot renderer + numpy/PIL are *incremental*. The real
  size lever is the map, not the world: a later swap to a lighter map (QtLocation /
  static tiles) would cut ~500 MB. Reframes the "optional component" question.
- **Offline gap:** the Leaflet map loads JS/CSS from unpkg CDN and tiles from carto
  — needs internet at ride time. Closing this (bundle Leaflet locally + offline
  tiles) is a P6/P7 task, separate from the world.
- **Godot export ✅ (step 2, 2026-06-14):** export templates installed (4.6.3.stable);
  `godot/export_presets.cfg` (macOS, universal, codesign off). Cleared blockers:
  `import_etc2_astc=true` in project.godot (required for arm64); `include_filter=
  "data/*"` (raw .json/.bin aren't "resources", so the default filter stripped them →
  null-deref crash). arm64 needs an **ad-hoc signature** (`codesign --force --deep -s -`)
  or it SIGKILLs/segfaults. Result: a standalone signed `RideSimWorld.app` that runs
  without the editor and **binds UDP :5005** (verified).
- **Step 2b — external data (new P6 sub-task):** the world hard-codes `res://data/`, so
  the export embeds the world. The bundled-renderer model wants the binary
  **data-agnostic**, loading from a path/user dir passed at launch (so one renderer
  serves any baked route). Needed before ride_sim launches the bundled binary.
- **Next: step 3** — ride_sim launches the exported binary instead of
  `/Applications/Godot.app` (retires the "Godot app" picker).

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
