extends Node3D
#
# ride-sim-world — DEM terrain + draped road + on-rails camera.
#
# Loads data baked by tools/ (res://data/), builds a low-poly terrain mesh and a
# road ribbon procedurally, and flies a camera along the route by distance.
#
# Drive signal: cumulative distance along the route (metres) — identical to
# ride_sim's virtual_dist_m (see ride-sim/docs/engine_interface.md). Two sources:
#   * DEMO mode (default): distance auto-advances at demo_speed.
#   * LIVE mode: ride_sim sends JSON-lines UDP {"distance_m":..,"speed_mps":..}
#     to udp_port; we snap to it and dead-reckon with speed between packets.
#
# Keys: SPACE pause/resume demo, "." frame-step while paused (tap=1 frame, hold=crawl;
# for diagnosing pack motion), +/- demo speed, M minimap, G toggle auto pace-
# ghost (off by default), J jump-to-km box (test singularities; also env
# RIDESIM_WORLD_SEEK_KM at startup), V cycle camera (chase/cockpit/drone/free;
# drone+free steer with ←→↑↓ / right-drag and zoom with [ ] / wheel), ESC exit
# fullscreen (then quit if standalone; ride_sim-driven ride only un-fullscreens).

@export var udp_port: int = 5005
@export var back_port: int = 5006        # back-channel to ride_sim (e.g. Space → pause)
@export var view_smooth_tau: float = 0.15  # render-distance smoothing (s); eases the
										 # ~4 Hz packet snaps so the avatar glides. 0 = off
@export var wait_for_telemetry: bool = true  # hold at the start line until ride_sim's
										 # first packet (no phantom demo advance / snap-
										 # back). Untick for standalone route preview.
@export var demo_speed: float = 5.5      # m/s (~20 km/h) when no telemetry; +/- adjusts
@export var look_ahead_m: float = 25.0   # camera aims this far down-route
@export var aim_height: float = 0.5      # look-target height above road
@export var cam_back_m: float = 8.0      # camera trails this far behind the rider
@export var cam_height: float = 5.0      # camera height above road (chase view)
@export var cam_yaw_rate_deg: float = 120.0  # max camera turn rate (deg/s); damps the
										 # heading so sharp route reversals (dead-end
										 # out-and-backs) pan smoothly instead of flipping
@export var cockpit_height: float = 1.6    # rider eye height for the cockpit/POV cam
@export var drone_pitch_deg: float = 35.0  # drone looks down at this angle
@export var drone_dist_m: float = 22.0     # drone distance from the rider
@export var drone_orbit_rate_deg: float = 8.0  # slow auto-orbit in drone view (0=off)
@export var cam_clearance_m: float = 2.5   # keep drone/free cam this far above terrain
@export var auto_ghost_default: bool = false  # synthetic pace ghost when ride_sim sends
										 # none; off by default, toggle in-world with G
@export var road_width: float = 8.0
@export var road_lift: float = 0.5       # sit road just above terrain; finer
										 # (~11 m) bake means no 5 m floating hack
@export var road_bank_max_deg: float = 5.0  # cap road cross-slope (camber). On steep
										 # hillsides draping each edge to its own terrain
										 # banks the road wildly (45°); this flattens it
										 # toward the centerline. 0 = dead flat across.

# Road bench (cut/fill the DEM along the route, like a graded roadbed) — kills the
# "roller-coaster" where steep cross-slopes pumped the leveled road up and down. The
# terrain is carved to a smoothed centerline grade, so road/rider/camera (all read
# _terrain_y) sit flat and the hillside meets the road instead of floating/diving.
@export var carve_road_bench: bool = true
@export var carve_bench_m: float = 12.0      # flat bench full width (cut to grade)
@export var carve_blend_m: float = 32.0      # graded shoulder out to original terrain —
										 # wide enough to grade the cut bank over >1 DEM
										 # cell so it doesn't wall up / encroach on the road
@export var carve_grade_smooth_m: float = 45.0  # window to smooth the road grade (m)
@export var terrain_slope_shading: bool = true  # grey steep faces (rock) vs flats (ground)

# Terrain<->road stitch: the uniform grid only pins heights at its vertices, so on a
# coarse bake a cell wider than the bench has no vertex inside the road and its triangle
# leans over it ("mudslide" bleed). The carve (below) now solves this in the HEIGHTFIELD
# domain — it pins the cells bracketing the road to grade, so no triangle can lean over.
#
# The apron below was an earlier GEOMETRY-domain attempt (delete the route's grid cells,
# fill the gap with an offset shoulder ribbon). It self-folded on switchback terrain whose
# turn radius < the ~21 m outer offset (offset ribbons always self-intersect there) and the
# steep-cross-slope drape read as translucent "fogbank" sheets. Parked OFF by default; the
# carve-domain stitch is the robust primitive (pure heights[] edits can't go non-manifold).
@export var road_apron: bool = false
@export var apron_reach_m: float = 10.0   # lateral half-extent the apron covers = hole radius

# OSM feature layers (P1) — draped on the terrain like the main road.
@export var show_roads: bool = true
@export var show_centerline: bool = true # the yellow route stripe (toggle in Detail panel)
@export var centerline_alpha: float = 0.55  # stripe opacity (1 = solid, lower = see-through)
@export var centerline_lift: float = 0.03   # stripe height above tarmac (m); small = wheels don't bury
@export var show_paths: bool = true      # footways/cycleways/tracks — farm-road web
@export var show_service: bool = true    # driveways/ranch/service roads
@export var show_water: bool = true
@export var show_landuse: bool = true
@export var show_trees: bool = true      # scatter low-poly trees in OSM forest/wood polys
@export var tree_spacing_m: float = 14.0 # ~one tree per this much area (smaller = denser)
@export var tree_height_m: float = 28.0  # base tree height (redwood-tall); per-instance ×0.6–1.5
@export var tree_max: int = 60000        # global instance cap (perf)
@export var show_buildings: bool = true  # extruded OSM footprints near the route
@export var building_height_scale: float = 1.0
@export var show_barriers: bool = true   # OSM fences / hedges / walls (vertical strips)
@export var show_power: bool = true      # OSM power lines + poles
@export var power_height: float = 9.0    # pole height / wire elevation (m)
@export var show_bridges: bool = true    # decks + railings on bridge=yes road spans
@export var feature_lift: float = 0.3    # cross-streets sit just below the ridden road
@export var cull_route_overlap: bool = true  # hide the OSM road that runs parallel UNDER
										 # the ridden route (kills the double-road) while
										 # keeping cross streets — they're landmarks
@export var cull_overlap_m: float = 9.0      # within this dist of the route AND…
@export var cull_parallel_deg: float = 28.0  # …this aligned = duplicate → drop the segment
@export var resample_m: float = 4.5      # resample route to uniform spacing (m) at
										 # load; densifies coarse OSM-routed paths. 0 = off
@export var route_smooth: int = 3        # centerline moving-average radius (pts);
										 # smooths GPS jitter / stop-light loops. 0 = off
@export var show_perf: bool = true       # top-left fps / frame-time / draw-call readout (toggle: P)
@export var show_cam: bool = true        # bottom-left view-controls readout (toggle: C)
@export var cam_label_fade_s: float = 30.0  # auto-hide the controls this long into the ride (0 = never)
@export var show_minimap: bool = true    # north-up route map overlay (toggle: M)
@export var minimap_size: int = 480      # minimap square size in px
@export var show_avatar: bool = true     # rider marker following the path
@export var avatar_scale: float = 1.0    # scale the rider (bump up for the drone view)
@export var show_ghost: bool = true      # ghost rider (different glow color)
@export var ghost_gap_m: float = 60.0    # demo: ghost this far up the route (+ahead/-behind)
# Rung-1 rig animation: child nodes of rider.glb whose name contains these spin as
# the bike rolls (distance-driven, so they speed/slow/stop/reverse with the ride;
# no telemetry needed). Wheels roll by circumference; the crank by "development"
# (meters of travel per pedal revolution = effective gear). Dormant unless the
# loaded model actually has matching named child nodes — placeholder is unaffected.
@export var wheel_name_match: String = "wheel"   # case-insensitive substring
@export var crank_name_match: String = "crank"   # case-insensitive substring
@export var wheel_diameter_m: float = 0.679      # measured SW tire OD (circ ≈ 2.133 m)
@export var crank_dev_m: float = 6.0             # m of travel per crank rev (gear)
# --- pedal-induced bike rock: seated riders rock the bike side-to-side once per crank
# rev (right downstroke one way, left the other), harder at high torque. Roll about the
# tire line (origin is at y=0, so the existing bank composition is already correct) and
# a smaller ~90°-offset yaw wag. Zero when coasting. Set pedal_rock_deg = 0 to disable.
@export var pedal_rock_deg: float = 2.0          # max roll (deg) at the reference torque
@export var pedal_rock_yaw_frac: float = 0.3     # yaw wag amplitude as a fraction of roll
@export var wheel_spin_sign: float = 1.0         # flip if wheels spin backward
@export var crank_spin_sign: float = -1.0        # flip if cranks spin backward
@export var cassette_spin_sign: float = -1.0     # flip if the cassette spins backward
@export var rig_spin_axis: Vector3 = Vector3.RIGHT  # local axle axis (X for fwd=-Z)

# --- peloton stress harness (env RIDESIM_PELOTON_N overrides peloton_count) ---
# Clones the rider behind the player on the same path to find the perf cliff.
# Each clone is a FULL articulated avatar (own rig + leg IK) so the test measures
# the worst case: N× draw calls AND N× per-frame animation. Riders are strung out
# (distance gap) and fanned across the road (lateral offset) so they don't overlap
# — a whole pack ghosting THROUGH each other looks wrong (the ghost doing it is a
# fun bug; a peloton doing it is just weird). Faked banking leans them into turns.
@export var peloton_count: int = 0           # # of extra riders (0 = off); env wins
@export var peloton_gap_m: float = 8.0       # base spacing between riders along path
@export var peloton_lateral_m: float = 1.4   # max half-width of the lateral fan
@export var peloton_bank_max_deg: float = 32.0  # cap on the faked corner lean
@export var peloton_bank_sign: float = -1.0  # lean INTO the turn (flip if wrong way)
@export var peloton_bank_tau: float = 0.30   # bank ease time-constant (s); 0 = instant
@export var peloton_bank_hero: bool = true   # lean the player's own bike mesh (horizon
											 # stays level — camera never rolls, so no vertigo)
# --- naturalism v1 (peloton-only; breaks the robotic lockstep) ---------------
# Racing line: the pack drifts toward the INSIDE of a bend ∝ curvature, then back
# out — instead of holding fixed lanes through corners. Negative flips inside/out.
@export var peloton_race_line_m: float = 1.8   # max inside-drift (m); 0 = off, <0 = flip side
@export var peloton_race_tau: float = 0.6      # ease time for the drift (s)
@export var peloton_apex_gain: float = 50.0    # curvature→drift gain (higher = commits harder)
@export var peloton_apex_lookahead_m: float = 8.0  # sample κ this far AHEAD so riders turn IN
											   # early and chase the apex (0 = drift at the corner)
# Wander: a slow per-rider lateral weave so nobody is pinned to a lane. DISTANCE-based
# (keyed on the rider's along-route distance s, the "pack-phase") so pace only scrolls
# it — the choreography is smooth + frame-rate independent and never needs reactive fixing.
@export var peloton_wander_m: float = 0.25     # weave amplitude (m); 0 = off
@export var peloton_weave_wl_m: float = 60.0   # road distance per weave cycle (m); ± per-rider spread
@export var peloton_weave_wl_spread: float = 0.35  # fractional spread of the per-rider wavelength
# --- PREORDAINED LANES (replaces reactive rider-rider separation) ---------------
# Each rider rides a deterministic lane; lateral = lane center + racing-line drift +
# the distance-keyed weave above. Two riders only collide when their DISTANCES coincide,
# so distinct lanes make the pack collision-free BY CONSTRUCTION — no per-frame O(N^2)
# separation, no 60 Hz limit cycle. Overtaking is LONGITUDINAL (come alongside in a
# neighbour lane). Lane gap must clear a bar width + both riders' weave.
@export var peloton_lane_gap_m: float = 1.1    # lateral spacing between lanes (m)
@export var peloton_fan_half_m: float = 3.0    # half-width of the lane fan across the road (m)
@export var peloton_footprint_m: float = 0.55  # rider+bike lateral width; drift/weave stay within
											   # (lane_gap − footprint)/2 of the lane centre → adjacent
											   # lanes always keep ≥ 1 footprint apart (no cross-lane clip)
# --- P2 SCHEDULED LANE-BORROW (the "autoroute keepout" for same-lane catch-ups) ---
# When more riders than lanes exist, two can share a lane and converge. The OVERTAKER
# (the one behind + closing) pulls out into a free adjacent lane, holds through the pass,
# and returns when clear. It's a COMMITTED maneuver — hysteresis (engage < release), only
# the overtaker moves, eased over pass_tau — so it never chatters or limit-cycles. The
# pass side is biased toward the road centre and only taken if that lane is actually free.
@export var peloton_pass_enable: bool = true
@export var peloton_pass_engage_m: float = 2.6  # gap ahead at which the overtaker pulls out (m)
@export var peloton_pass_release_m: float = 3.2 # gap at which it may return home (> engage = hysteresis)
@export var peloton_pass_tau: float = 0.7       # lane-change ease time (s); higher = lazier pull-out
# Coast-with-inside-pedal-up: in hard corners riders stop pedaling and hold the
# OUTSIDE pedal down / inside pedal up (real cornering technique to dodge pedal
# strike). The trigger lean is computed from the frame's geometry below.
@export var peloton_coast_pedal_up: bool = true
@export var bb_height_m: float = 0.25          # bottom-bracket height (this frame: low BB)
@export var crank_len_m: float = 0.17          # crank-arm length
@export var pedal_outboard_m: float = 0.10     # pedal lateral offset from bike centerline
@export var coast_margin_deg: float = 8.0      # lift the inside pedal this far BEFORE strike
@export var coast_tau: float = 0.35            # ease in/out of the coast pose (s)
@export var coast_phase_rad: float = 0.0       # crank angle that sits cranks vertical / a pedal
											   # down — tune on the demo so coasting looks upright
@export var coast_flip: bool = false           # swap which pedal lifts vs. turn direction
# --- naturalism v2: autonomous riders (own speed → overtaking + accordion) ----
# Each rider integrates its OWN position from its OWN speed = pack speed, pulled
# toward a slot relative to the player (keeps the bunch coherent) + a slow per-rider
# surge (so they trade places and occasionally pass the player). Off = the old
# rigid locked-gap formation.
@export var peloton_autonomous: bool = true
@export var peloton_speed_var: float = 0.09    # surge amplitude as a fraction of pack speed
@export var peloton_surge_min_s: float = 8.0   # surge cycle range (s): per-rider period in
@export var peloton_surge_max_s: float = 22.0  # this band → unsynced surging/fading
@export var peloton_station_k: float = 0.12    # leash stiffness once a rider strays past the window (1/s)
@export var peloton_leash_speed_max: float = 2.5  # cap the leash catch-up to a human surge (m/s) so a
											   # far-strayed rider closes the gap at a plausible pace,
											   # never a teleport-sprint (was the start-line rush bug)
@export var peloton_speed_tau: float = 1.5     # speed easing (riders don't snap speed)
@export var peloton_pace_tau: float = 0.6      # low-pass on the shared pack pace: _meas_speed is a
											   # finite-diff of the eased position, so it carries the
											   # 4 Hz packet + ~1 Hz TCX-trackpoint pulses that make
											   # the WHOLE pack surge together fore/aft. EMA it first.
# --- BOUNDED-STATION longitudinal model ("2 trains leave Chicago"): each rider's along-
# route position is pos = pack_ref + home + drift(s), drift a BOUNDED oscillation keyed on
# the pack phase s = pack_ref. Same-lane riders' homes are spaced > 2·drift + bike length,
# so their positions can NEVER converge — no same-lane collision BY CONSTRUCTION, no live
# reaction. Overtaking is cross-lane (different lateral = collision-free) and preordained.
@export var peloton_station_gap_m: float = 11.0  # along-route spacing between same-lane riders (m)
@export var peloton_bike_len_m: float = 2.0      # min centre-to-centre same-lane gap kept above this
@export var peloton_bubble_m: float = 3.0        # MAKE-ROOM max shift (m): reserved out of the
                                                 # drift budget → AI–AI no-convergence holds.
                                                 # = hold_m + the Schmitt flip threshold.
@export var peloton_bubble_hold_m: float = 2.0   # rendered clearance the hold maintains (m):
                                                 # a latched rider is kept ≥ this away — one
                                                 # bike length, so wheels never overlap.
@export var peloton_bubble_zone_m: float = 8.0   # along-route half-zone the yield engages over
@export var peloton_bubble_rate_mps: float = 3.0 # max speed a yield adds/removes (keeps it calm)
@export var peloton_drift_wl_m: float = 280.0    # SLOW surge wavelength (m): the visible ebb,
                                                 # ~tens of seconds per cycle at cruise
@export var peloton_drift_tex_wl_m: float = 45.0 # FAST texture wavelength (m): small pace jitter
@export var peloton_drift_wl_spread: float = 0.40  # per-rider surge-wavelength spread (fraction)
@export var peloton_drift_speed_frac: float = 0.20  # max fore/aft surge as a fraction of pack pace
                                                    # (0.20 → riders ride 0.8–1.2× the pack)
@export var peloton_drift_row_phase: float = 0.6 # slow-wave phase step per station row (rad):
                                                 # surges PROPAGATE through the pack like a real
                                                 # accordion wave; 0 = every rider ebbs alone
											   # (caps the surge VELOCITY so a rider never stalls/reverses
											   # — amp is auto-shrunk from drift_cap to honour this)
@export var peloton_pack_follow_tau: float = 4.0 # how slowly the pack re-centres on you (s): bigger =
											   # you range further fore/aft THROUGH the field per surge
# Per-rider ABILITY (FTP/category spread): each rider has a persistent strength in
# [-1,+1] → its natural speed is pack_speed·(1 + ability_spread·ability). Strong riders
# drift FORWARD, weak DROP BACK (real overtaking + a pack that sorts itself), instead of
# everyone locked to the player's exact speed. A wider spread = more mixed field (rec);
# tighter = even/fast field (pro). ride_sim's "Peloton level" picker sets these
# (RIDESIM_PELOTON_LEVEL → _apply_peloton_level); these are the un-leveled defaults.
@export var peloton_ability_spread: float = 0.08  # top/bottom rider speed = pack ±8%
@export var peloton_leash_m: float = 75.0      # a rider may range this far from the player before
											   # being herded back (keeps the sorted pack on-screen)
# FREE PACE ("try to stay with Tadej") vs RIDE-AROUND-YOU: the pack's pace comes from its
# own CAPABILITY, NOT from amplifying the player's speed. FREE PACE = ride that capability
# pace regardless of you (drops a slower rider, but never faster than the riders could
# actually hold). Otherwise = match your pace, capped at capability. ride_sim's "Race pace
# (don't wait for me)" sets free pace.
# CAPABILITY = physics, not a hardcoded cruise speed: the pack holds peloton_wkg watts/kg
# and its speed on any grade falls out of the road power equation
#   P = (½·ρ·CdA·v² + Crr·m·g + m·g·grade)·v
# — so it automatically crawls up walls and flies down descents, no grade fudge factors.
# The W/kg dial is CONTINUOUS (Detail panel); category presets are just detents on it.
# COMPANION mode: the pack's watts follow the PLAYER's — implied watts are computed from
# the player's own in-world speed + grade through the SAME model (model errors cancel, no
# power meter needed), then × companion_factor: 1.0 = evenly matched at any fitness,
# >1 = they try to drop you, <1 = a recovery bunch you can beat.
@export var peloton_free_pace: bool = false
@export var peloton_wkg: float = 2.8               # pack ability dial (W/kg of peloton_mass_kg)
@export var peloton_mass_kg: float = 75.0          # rider + bike mass per pack rider (kg)
@export var peloton_cda: float = 0.30              # effective CdA (m²) — bunch rides part-sheltered
@export var peloton_crr: float = 0.004             # rolling resistance
@export var peloton_companion: bool = false        # pack watts follow the player's implied watts
@export var peloton_companion_factor: float = 1.0  # × player watts (companion mode)
@export var peloton_companion_tau: float = 90.0    # rolling window for the player's implied watts (s)
@export var peloton_cruise_tau: float = 3.0        # ease the pack pace across grade breaks (s)
@export var peloton_cap_headroom: float = 1.25     # ride-around mode may match you up to cruise×this
@export var peloton_speed_max_mps: float = 22.0    # absolute pace sanity cap (~79 km/h) vs any glitch
# Long-period jockeying: a slow per-rider migration up/back through the bunch over
# minutes (on top of the short surge) so the pack churns like a real peloton, not a grid.
@export var peloton_drift_var: float = 0.06    # migration amplitude (fraction of pace)
@export var peloton_drift_min_s: float = 45.0  # migration cycle range (s): per-rider period in
@export var peloton_drift_max_s: float = 150.0 # this band → slow, unsynced up-and-back movement
# Cadence: gear for a realistic spin and clamp to a peloton-plausible band, so legs
# never blur. Cadence rises a touch when a rider is going faster than the pack.
@export var peloton_cadence_rpm: float = 88.0  # preferred cadence (flats)
@export var peloton_cadence_spread: float = 9.0   # ± per-rider preference
@export var peloton_cadence_k: float = 4.0     # rpm gained per m/s above pack speed
@export var peloton_cadence_min: float = 60.0  # band floor (steep climb)
@export var peloton_cadence_max: float = 118.0 # band ceiling (no blur)
# --- player racing line (the rider takes a sensible line too) -----------------
# The player's own avatar drifts toward the inside of bends like the pack — but
# gentler, because the chase/cockpit camera tracks the SAME offset (too much
# lateral motion under a following camera reads as vertigo). The camera samples
# this exact offset so it stays locked behind the player, not the centerline.
@export var player_race_line_m: float = 2.4    # max inside-drift for the player (m); 0 = off
@export var player_race_tau: float = 0.6       # ease time for the player's drift (s)
# --- ghost: paces by the strict TCX distance from ride_sim, but otherwise rides
# like a peloton rider (lean, racing line, slow weave) AND side-steps the player
# so it passes BESIDE instead of driving through. ghost_tint recolors the
# placeholder in one switch (a textured glb keeps its own materials).
@export var ghost_pass_m: float = 1.7          # lateral separation when overtaking the player (m)
@export var ghost_pass_zone_m: float = 12.0    # blend the side-step in within this gap (m)
@export var ghost_pass_side: float = 1.0       # +1 = pass on the right, -1 = left
@export var ghost_lane_m: float = 0.0          # ghost's own base lane bias when clear of the player
@export var ghost_tint: Color = Color(1.0, 0.45, 0.1)  # placeholder/emissive ghost color

var world: Dictionary
var heights: PackedFloat32Array
var gw: int
var gh: int
var x0: float
var z0: float
var mpp_x: float
var mpp_z: float

var pts: Array = []        # [{x,y,z,d}, ...]
var route_len: float = 0.0
var road_center_y := PackedFloat32Array()   # leveled ridden-road center height per pts
											# index; rider + road share it so nothing buries
var road_grade := PackedFloat32Array()       # smoothed road elevation per pts index —
											# the course's OWN recorded grade (not the noisy
											# DEM); terrain is carved to meet it
var route_dir := PackedVector2Array()        # unit route heading per pts index (for cull)
var carve_cut := PackedFloat32Array()        # per terrain cell: 0=natural ground, 1=fresh
											# carved cut bank (→ dirt in the terrain shader)
var _noise_tex: NoiseTexture2D               # shared procedural grain (lazy, no shipped asset)
var _features: Dictionary = {}               # parsed features.json (lazy; terrain + features use it)
var _features_loaded: bool = false
var _route_grid := {}      # Vector2i cell → PackedInt32Array of route indices (cull lookup)
const ROUTE_CELL := 20.0   # cull grid cell size (m); must exceed cull_overlap_m
var _curv_lut := PackedFloat32Array()   # distance-smoothed signed curvature (rad/m) LUT
const CURV_LUT_STEP := 2.0              # m between LUT samples
const CURV_LUT_SMOOTH := 3              # box half-width in samples (±6 m)

var dist: float = 0.0
var live := false
var cur_speed: float = 0.0
var paused := false
var _sim_active := false           # this frame the sim advances (running, or a frame-step)
var _step_frames := 0              # queued single-frame steps (KEY_PERIOD while paused)
var _peldbg_next := 0.0            # next _pelo_t to log peloton debug (RIDESIM_PELOTON_DEBUG)
var seg_i := 0             # cached segment index for distance lookup
var cam_fwd := Vector2.ZERO   # damped camera heading (world x,z); 0 until first frame
var ghost_live_dist := 0.0    # ride_sim's real ghost position (m), when sent
var ghost_live_speed := 0.0   # ghost speed (m/s) for dead-reckoning between packets
var ghost_is_live := false    # true once ride_sim sends ghost_distance_m
var cur_cadence := 0.0        # rider cadence (rpm) from FTMS; 0 = coasting
var cur_power := 0.0          # rider watts from ride_sim (drives pedal-rock amplitude)
var crank_phase := 0.0        # integrated drivetrain angle (rad): crank+cassette+legs
var view_dist := 0.0          # render distance: eases toward dist to hide the
var view_ghost_dist := 0.0    # ~4 Hz packet snaps (smooth avatar/camera motion)

# User-controllable camera (P3). CHASE is the original on-rails view; the rest
# are opt-in via the V key. DRONE/FREE share orbit_* state (seeded on entry).
enum CamMode { CHASE, COCKPIT, DRONE, FREE, ZENITH }
var cam_mode: int = CamMode.CHASE
var orbit_yaw_deg := 0.0      # yaw around the rider (deg); +180 base = behind
var orbit_pitch_deg := 30.0   # look-down pitch (deg), clamped 1..89
var orbit_dist := 22.0        # camera distance from the rider (m); also zenith altitude
var cam_zoom := 1.0           # CHASE zoom multiplier (scales back + height)
var mouse_orbiting := false   # right-button drag orbits in DRONE/FREE
var _fog_base := 0.00025      # baseline fog density; thinned at god-zoom (see _apply_god_view)
var cam_label: Label
var perf_label: Label             # fps / frame-time / draw-call readout (P)
var _perf_accum := 0.0            # throttle the perf text refresh (~4 Hz)
var _avdbg_accum := 0.0           # throttle the avatar debug print (RIDESIM_AVATAR_DEBUG)
var auto_ghost := false       # show a synthetic pace ghost when none is live (G)
var seek_input: LineEdit      # J: type a km to jump to (test singularities)

var udp := PacketPeerUDP.new()
var back_udp := PacketPeerUDP.new()   # sends commands back to ride_sim
var cam: Camera3D
var minimap: Minimap
var minimap_layer: CanvasLayer
var show_leaderboard := true       # pack leaderboard panel (Detail-panel toggle)
var _lb_panel: PanelContainer      # leaderboard container (top-left)
var _lb_label: Label
var _lb_accum := 0.0               # refresh throttle (updates every 0.5 s)
var avatar: Node3D
var ghost: Node3D
# Cached rig nodes (wheels/cranks) + their rest basis, per avatar. Empty dicts
# when the model has no matching nodes (e.g. the procedural placeholder).
var _avatar_rig := {"wheels": [], "cranks": []}
var _ghost_rig := {"wheels": [], "cranks": []}
var _avatar_legs: LegRig = null    # rung-2 procedural pedaling IK (null if no rig)
var _ghost_legs: LegRig = null

# Peloton stress harness: each entry = {node, rig, legs, gap, lateral, bank}.
# `bank` is the eased lean carried frame-to-frame. Empty unless RIDESIM_PELOTON_N
# (or peloton_count) > 0. See [[project-virtual-peloton]].
var peloton: Array = []
var _peloton_level := "default"    # FTP/category preset name (RIDESIM_PELOTON_LEVEL), for the log
var _pack_cruise := 0.0            # free-pace: the pack's own sticky cruise speed (m/s)
var _pack_pace := 0.0              # low-passed shared pack pace (m/s) — kills the fore/aft surge
var _companion_watts := 0.0        # player's rolling implied watts (companion pacing anchor)
var _live_paused := false          # ride_sim says the ride is paused → hold the AI pack
var _pack_ref_prev := 0.0          # last _pack_ref (→ actual pack advance rate, bubble slew)
var _pack_ref := 0.0               # bounded-station model: the pack's along-route reference (scroll)
var _pack_ref_inited := false
var _pelo_t := 0.0                 # peloton clock (s) for time-based weave/surge (frozen when stopped)
var _ride_t := 0.0                 # cumulative time the ride has been MOVING (s) — drives the cam fade
var _last_d := 0.0                 # previous frame's render distance (for measured speed)
var _view_snapped := false         # view_dist teleported this frame (first pkt/scrub) → speed is garbage
var _hero_bank := 0.0              # eased lean for the player's avatar (if enabled)
var _meas_speed := 0.0            # measured ride speed (m/s) from frame distance; shared peloton/ghost
const MEAS_SPEED_GLITCH_MAX := 35.0   # raw frame-speed above this = a position-jump glitch, not a ride
var _player_lat := 0.0            # eased player racing-line lateral offset (m)
var _player_yaw := 0.0            # steer-into-the-line yaw (rad) — crab fix for the avatar
var _player_off := Vector3.ZERO   # that offset as a world vector (cameras add it to track the player)
var _ghost_state: Dictionary = {} # persistent ghost rider state (eased lean/line/coast/crank)
var _fps_hist: PackedFloat32Array = PackedFloat32Array()  # rolling per-frame fps

# --- quality / LOD foundation (live-tunable; profile on the target GPU) ------
# Scene handles the quality system mutates. sun/env/cam always exist; the
# MultiMesh handles are null when their layer is off (guarded everywhere).
var sun: DirectionalLight3D        # the single directional sun (shadows)
var env: Environment               # world environment (glow / fog / sky)
var tree_mmi: MultiMeshInstance3D  # ALL trees as one MultiMesh (51k = the big cost)
var pole_mmi: MultiMeshInstance3D  # power poles as one MultiMesh
var quality_label: Label           # top-left quality/LOD state readout (toggle: Q)
# Live state — preset sets these, _apply_quality() pushes them to the scene.
# Defaults = "high" so the look is unchanged until a preset/env says otherwise.
var q_preset := "high"             # low | medium | high (env RIDESIM_WORLD_QUALITY)
var q_shadows := true              # directional sun shadows at all
var q_tree_shadows := true         # trees+poles CAST shadows (51k casters = brutal)
var q_trees := true                # draw trees at all
var q_glow := true                 # screen-space bloom (Tron avatar)
var q_far := 12000.0               # camera far plane (m) — frustum-culls distant geo
var q_shadow_dist := 200.0         # directional shadow max distance (m); smaller=cheaper
var q_fog := 0.00025               # fog density; higher masks the far cutoff (but greyer)
var q_msaa := 0                    # MSAA: 0=off, 2=2x, 4=4x
var q_render_scale := 1.0          # 3D render resolution (FSR upscale); THE 4K fill-rate lever
var q_vsync := true                # vsync on caps to refresh; off reveals the true fps ceiling
var show_quality := false          # quality readout visible (Q)
var detail_panel: PanelContainer   # mouse-driven World Detail panel (⚙ button)
var detail_open := false
var _ui_render: HSlider            # World Detail widgets (mouse control, no keyboard)
var _ui_far: HSlider
var _ui_wkg: HSlider               # peloton ability dial (W/kg)
var _ui_wkg_lbl: Label
var _ui_companion: CheckBox        # companion pacing toggle
var _ui_leaderboard: CheckBox      # pack leaderboard toggle
var _ui_cfactor: HSlider           # companion factor (× player effort)
var _ui_cfactor_lbl: Label
var _ui_render_lbl: Label
var _ui_far_lbl: Label
var _ui_vsync: CheckBox
var _ui_sun: CheckBox
var _ui_treeshadow: CheckBox
var _ui_trees: CheckBox
var _ui_centerline: CheckBox
var _ui_perf: CheckBox
var _ui_cam: CheckBox
var centerline_mi: MeshInstance3D   # the yellow route stripe, kept for runtime toggle
var _ui_glow: CheckBox
var _syncing := false              # guard: preset sync sets widgets without re-firing

var data_dir := "res://data"   # where world.json/heights.bin/route.json/features.json
							   # live; overridden by the RIDESIM_WORLD_DIR env var so one
							   # exported binary can serve any baked route (P6).
var loaded := false            # false if no world data was found (render nothing, no crash)


# Resolve a data file against the active data dir (external dir or bundled res://data).
func _dpath(name: String) -> String:
	return data_dir + "/" + name


func _ready() -> void:
	auto_ghost = auto_ghost_default
	_apply_world_screen()
	_load_data()
	if loaded:
		_carve_road_bench()   # must precede everything that reads _terrain_y
		_build_terrain()
		_build_road()
		_build_features()
		_build_trees()
	_build_camera_and_sky()
	_init_quality()        # read RIDESIM_WORLD_QUALITY, push tier to sun/env/cam/trees
	_build_detail_panel()  # mouse-driven World Detail controls (⚙ button)
	if loaded:
		_build_minimap()
		_build_leaderboard()
		_build_avatar()
	back_udp.set_dest_address("127.0.0.1", back_port)
	if udp.bind(udp_port) == OK:
		if wait_for_telemetry:
			print("listening on udp:%d — holding at start until ride_sim connects (untick wait_for_telemetry for standalone demo)" % udp_port)
		else:
			print("listening for ride_sim telemetry on udp:%d (standalone demo running)" % udp_port)
	else:
		push_warning("could not bind udp:%d — demo mode only" % udp_port)

	# Startup jump: RIDESIM_WORLD_SEEK_KM=2.66 launches straight to that km, paused,
	# for inspecting a known singularity without riding there.
	if loaded:
		var sk := OS.get_environment("RIDESIM_WORLD_SEEK_KM")
		if sk != "":
			wait_for_telemetry = false   # standalone inspection: SPACE rides the demo
			_seek_to_km(sk.to_float())


func _apply_world_screen() -> void:
	# ride_sim launches us with RIDESIM_WORLD_SCREEN_POS="x,y" (a global point
	# inside the monitor it wants the world on) for a dual-monitor ride. Move there
	# and go fullscreen so the dashboard (other screen) and world don't overlap.
	# Unset (standalone demo / single screen) → leave the window as-is.
	var hint := OS.get_environment("RIDESIM_WORLD_SCREEN_POS")
	if hint == "":
		return
	var parts := hint.split(",")
	var target := -1
	var n := DisplayServer.get_screen_count()
	if parts.size() == 2:
		var pt := Vector2i(int(parts[0]), int(parts[1]))
		for i in range(n):
			var r := Rect2i(DisplayServer.screen_get_position(i),
							DisplayServer.screen_get_size(i))
			if r.has_point(pt):
				target = i
				break
	if target < 0:
		# Coordinate spaces differ across hiDPI (Qt points vs Godot pixels) — fall
		# back to the largest screen, which is how ride_sim picked the world screen.
		var best_area := -1
		for i in range(n):
			var s := DisplayServer.screen_get_size(i)
			var a := s.x * s.y
			if a > best_area:
				best_area = a
				target = i
	if target >= 0:
		DisplayServer.window_set_current_screen(target)
		DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_FULLSCREEN)
		print("world screen: %d of %d (fullscreen)" % [target, n])


func _load_data() -> void:
	# Prefer an external baked-world dir (ride_sim sets RIDESIM_WORLD_DIR when it
	# launches the bundled renderer); fall back to the bundled res://data.
	var ext := OS.get_environment("RIDESIM_WORLD_DIR")
	if ext != "" and DirAccess.dir_exists_absolute(ext):
		data_dir = ext
		print("world data dir: %s (external)" % ext)
	else:
		data_dir = "res://data"

	var world_str := FileAccess.get_file_as_string(_dpath("world.json"))
	if world_str == "":
		push_error("no world data found at %s — rendering empty scene. Bake a route or set RIDESIM_WORLD_DIR." % data_dir)
		return
	world = JSON.parse_string(world_str)
	gw = int(world.grid_w); gh = int(world.grid_h)
	x0 = float(world.x0); z0 = float(world.z0)
	mpp_x = float(world.mpp_x); mpp_z = float(world.mpp_z)

	var f := FileAccess.open(_dpath("heights.bin"), FileAccess.READ)
	heights = f.get_buffer(f.get_length()).to_float32_array()
	f.close()

	var route: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(_dpath("route.json")))
	pts = route.points
	route_len = float(route.length_m)
	var src_len := route_len   # ride_sim drives an ABSOLUTE distance_m on THIS length
	# Un-mirror: mapping ENU East->+X / North->+Z into Godot's axes is a reflection
	# (flips chirality, so right turns render as left bends). Negate East here, and
	# matching negations in _build_terrain/_terrain_y/_build_features, so the whole
	# world shares one corrected frame. pts is the single source for road+cam+map.
	for i in range(pts.size()):
		pts[i]["x"] = -float(pts[i].x)
	_resample_route(resample_m)
	_smooth_route(route_smooth)
	_rescale_distance(src_len)
	loaded = true
	print("route %.2f km, terrain %d x %d cells, %d pts" % [route_len / 1000.0, gw, gh, pts.size()])


# Resample + smooth can corner-cut and shorten the path (smoothing the SF route
# loses ~165 m at the crooked block). ride_sim drives an absolute distance_m
# defined on the ORIGINAL route length, so on the shortened route the map marker
# and the world position diverge — worst exactly at the curves the smoothing cut.
# Rescale cumulative d back to the source length. Linear scale → demo speed stays
# uniform; position(distance_m) now tracks ride_sim's GPS map.
func _rescale_distance(target_len: float) -> void:
	var n := pts.size()
	if n < 2 or route_len <= 0.0 or target_len <= 0.0:
		return
	var k := target_len / route_len
	for i in range(n):
		pts[i]["d"] = float(pts[i].d) * k
	route_len = target_len


# Resample the centerline to ~uniform `spacing` m (arc-length walk + linear interp).
# Densifies coarse OSM-routed paths so the rider, camera, and road ribbon stop
# showing angular kinks at sparse nodes, and makes the moving-average smoothing
# uniform. Rebuilds cumulative d.
func _resample_route(spacing: float) -> void:
	var n := pts.size()
	if spacing <= 0.0 or n < 2:
		return
	var out := []
	out.append({"x": float(pts[0].x), "y": float(pts[0].y), "z": float(pts[0].z), "d": 0.0})
	var total := 0.0
	var carry := 0.0
	for i in range(1, n):
		var ax := float(pts[i - 1].x); var ay := float(pts[i - 1].y); var az := float(pts[i - 1].z)
		var bx := float(pts[i].x); var by := float(pts[i].y); var bz := float(pts[i].z)
		var dx := bx - ax; var dy := by - ay; var dz := bz - az
		var seg := sqrt(dx * dx + dz * dz)
		if seg < 0.0001:
			continue
		var t := spacing - carry
		while t <= seg:
			var f := t / seg
			total += spacing
			out.append({"x": ax + dx * f, "y": ay + dy * f, "z": az + dz * f, "d": total})
			t += spacing
		carry = seg - (t - spacing)
	var last_p = pts[n - 1]
	var prev_p = out[out.size() - 1]
	var ex := float(last_p.x) - float(prev_p.x); var ez := float(last_p.z) - float(prev_p.z)
	var tail := sqrt(ex * ex + ez * ez)
	if tail > 0.1:
		out.append({"x": float(last_p.x), "y": float(last_p.y), "z": float(last_p.z),
				"d": float(prev_p.d) + tail})
	pts = out
	route_len = float(out[out.size() - 1].d)


# Moving-average the centerline x/z in place (keeps each point's distance d and
# elevation). Tames GPS jitter and stop-light loops so the road ribbon doesn't
# fold and the camera doesn't twitch. The road and camera read the same pts, so
# they stay aligned.
func _smooth_route(radius: int) -> void:
	var n := pts.size()
	if radius < 1 or n < 2 * radius + 1:
		return
	var sx := PackedFloat32Array(); sx.resize(n)
	var sz := PackedFloat32Array(); sz.resize(n)
	for i in range(n):
		var ax := 0.0; var az := 0.0; var cnt := 0
		for j in range(maxi(0, i - radius), mini(n, i + radius + 1)):
			ax += float(pts[j].x); az += float(pts[j].z); cnt += 1
		sx[i] = ax / cnt; sz[i] = az / cnt
	for i in range(n):
		pts[i]["x"] = sx[i]
		pts[i]["z"] = sz[i]
	# Smoothing moved the points but stored d are the ORIGINAL arc-lengths; left
	# as-is, position(d) advances at a varying euclidean speed (the demo pulses,
	# obvious in a close/low chase). Recompute cumulative distance to match.
	var total := 0.0
	pts[0]["d"] = 0.0
	for i in range(1, n):
		var dx := sx[i] - sx[i - 1]
		var dz := sz[i] - sz[i - 1]
		total += sqrt(dx * dx + dz * dz)
		pts[i]["d"] = total
	route_len = total


# --- terrain sampling -------------------------------------------------------

func _terrain_y(x: float, z: float) -> float:
	if heights.is_empty():
		return 0.0                      # called before the heightfield loaded
	var c := (-x - x0) / mpp_x          # East is negated world-wide (see _load_data)
	var r := (z - z0) / mpp_z
	c = clampf(c, 0.0, float(gw - 1) - 0.001)
	r = clampf(r, 0.0, float(gh - 1) - 0.001)
	var ci := int(c); var ri := int(r)
	var fx := c - ci; var fz := r - ri
	var h00 := heights[ri * gw + ci]
	var h10 := heights[ri * gw + ci + 1]
	var h01 := heights[(ri + 1) * gw + ci]
	var h11 := heights[(ri + 1) * gw + ci + 1]
	return lerpf(lerpf(h00, h10, fx), lerpf(h01, h11, fx), fz)


func _elev_color(y: float) -> Color:
	var t := clampf((y - float(world.elev_min)) / maxf(1.0, float(world.elev_max) - float(world.elev_min)), 0.0, 1.0)
	var green := Color(0.27, 0.45, 0.22)
	var tan := Color(0.55, 0.5, 0.36)
	var gray := Color(0.6, 0.6, 0.62)
	if t < 0.5:
		return green.lerp(tan, t / 0.5)
	return tan.lerp(gray, (t - 0.5) / 0.5)


# --- procedural materials (triplanar noise, no shipped image assets) --------

# One shared tileable simplex texture, generated at load (FastNoiseLite). Both the
# terrain and tarmac shaders sample it triplanar in world space for grain.
func _grain_noise() -> NoiseTexture2D:
	if _noise_tex != null:
		return _noise_tex
	var fn := FastNoiseLite.new()
	fn.noise_type = FastNoiseLite.TYPE_SIMPLEX
	fn.frequency = 0.05
	fn.fractal_octaves = 4
	var nt := NoiseTexture2D.new()
	nt.noise = fn
	nt.seamless = true
	nt.width = 256
	nt.height = 256
	_noise_tex = nt
	return nt


# Terrain: biome base (vertex COLOR.rgb) + slope-rock greying + procedural grain +
# dirt on carved cut banks (vertex COLOR.a). cull_disabled — East-negation flips winding.
const TERRAIN_SHADER := """
shader_type spatial;
render_mode cull_disabled;
uniform sampler2D detail_noise : repeat_enable;
uniform float detail_scale = 0.06;
uniform float detail_amt = 0.30;
uniform float slope_shade = 1.0;
uniform vec3 rock_color : source_color = vec3(0.34, 0.31, 0.29);
uniform vec3 dirt_color : source_color = vec3(0.43, 0.33, 0.22);
varying vec3 w_pos;
varying vec3 w_nrm;
void vertex() {
	w_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	w_nrm = normalize((MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz);
}
void fragment() {
	vec3 bw = abs(w_nrm);
	bw /= (bw.x + bw.y + bw.z + 0.0001);
	float nx = texture(detail_noise, w_pos.zy * detail_scale).r;
	float ny = texture(detail_noise, w_pos.xz * detail_scale).r;
	float nz = texture(detail_noise, w_pos.xy * detail_scale).r;
	float detail = nx * bw.x + ny * bw.y + nz * bw.z;
	vec3 col = COLOR.rgb;
	float slope = clamp(1.0 - w_nrm.y, 0.0, 1.0);
	float rockf = smoothstep(0.22, 0.72, slope) * 0.85 * slope_shade;
	col = mix(col, rock_color, rockf);
	col = mix(col, dirt_color, COLOR.a);
	col *= (1.0 - detail_amt) + detail_amt * 2.0 * detail;
	ALBEDO = col;
	ROUGHNESS = 1.0;
}
"""


func _terrain_material() -> ShaderMaterial:
	var sh := Shader.new()
	sh.code = TERRAIN_SHADER
	var mat := ShaderMaterial.new()
	mat.shader = sh
	mat.set_shader_parameter("detail_noise", _grain_noise())
	mat.set_shader_parameter("slope_shade", 1.0 if terrain_slope_shading else 0.0)
	return mat


# Asphalt: dark tarmac with a fine procedural grain (road is near-planar → xz mapping).
const TARMAC_SHADER := """
shader_type spatial;
render_mode cull_disabled;
uniform sampler2D grain_noise : repeat_enable;
uniform float grain_scale = 0.25;
uniform vec3 tarmac : source_color = vec3(0.17, 0.17, 0.19);
varying vec3 w_pos;
void vertex() { w_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz; }
void fragment() {
	float g = texture(grain_noise, w_pos.xz * grain_scale).r;
	float g2 = texture(grain_noise, w_pos.xz * grain_scale * 4.0).r;
	float n = mix(g, g2, 0.5);
	ALBEDO = tarmac * (0.78 + 0.50 * n);
	ROUGHNESS = 0.9 - 0.18 * n;
}
"""


func _tarmac_material() -> ShaderMaterial:
	var sh := Shader.new()
	sh.code = TARMAC_SHADER
	var mat := ShaderMaterial.new()
	mat.shader = sh
	mat.set_shader_parameter("grain_noise", _grain_noise())
	return mat


# Parse features.json once (terrain landuse bake + _build_features both read it).
func _load_features() -> Dictionary:
	if _features_loaded:
		return _features
	_features_loaded = true
	if not FileAccess.file_exists(_dpath("features.json")):
		return _features
	var parsed = JSON.parse_string(FileAccess.get_file_as_string(_dpath("features.json")))
	if typeof(parsed) == TYPE_DICTIONARY:
		_features = parsed
	return _features


# Rasterize OSM landuse polygons into a per-terrain-cell tint (alpha>0 where covered),
# so landcover is baked INTO the terrain mesh (conformed, opaque) instead of floating as a
# translucent draped overlay (which read as a green stripe / "fog over the road"). bbox-
# culled point-in-polygon per cell — cheap for the usual many-small-parcels case.
func _landuse_grid() -> PackedColorArray:
	var tint := PackedColorArray(); tint.resize(gw * gh)   # default Color(0,0,0,0)
	if not show_landuse:
		return tint
	for lu in _load_features().get("landuse", []):
		var pts_arr = _flipx(lu["pts"])
		var poly := PackedVector2Array()
		var minc := gw; var maxc := -1; var minr := gh; var maxr := -1
		for p in pts_arr:
			var px := float(p[0]); var pz := float(p[1])
			poly.append(Vector2(px, pz))
			var cf := (-px - x0) / mpp_x
			var rf := (pz - z0) / mpp_z
			minc = mini(minc, int(floor(cf))); maxc = maxi(maxc, int(ceil(cf)))
			minr = mini(minr, int(floor(rf))); maxr = maxi(maxr, int(ceil(rf)))
		if poly.size() < 3:
			continue
		minc = maxi(minc, 0); maxc = mini(maxc, gw - 1)
		minr = maxi(minr, 0); maxr = mini(maxr, gh - 1)
		var col := _landuse_color(lu["class"])
		for r in range(minr, maxr + 1):
			for c in range(minc, maxc + 1):
				var wx := -(x0 + c * mpp_x)
				var wz := z0 + r * mpp_z
				if Geometry2D.is_point_in_polygon(Vector2(wx, wz), poly):
					tint[r * gw + c] = Color(col.r, col.g, col.b, 1.0)
	return tint


# A single low-poly tree: a square trunk + a faceted cone canopy, two surfaces (brown /
# green) so one MultiMesh can draw thousands cheaply. Local origin at the trunk base (y=0).
func _make_tree_mesh() -> ArrayMesh:
	var mesh := ArrayMesh.new()
	var h := tree_height_m
	var th := h * 0.40                              # bare trunk portion (redwood-tall)
	var tw := maxf(0.22, h * 0.016)                 # trunk half-width
	var tr := SurfaceTool.new(); tr.begin(Mesh.PRIMITIVE_TRIANGLES)
	var corners := [Vector2(-tw, -tw), Vector2(tw, -tw), Vector2(tw, tw), Vector2(-tw, tw)]
	for i in range(4):
		var a2: Vector2 = corners[i]
		var b2: Vector2 = corners[(i + 1) % 4]
		var a0 := Vector3(a2.x, 0.0, a2.y); var b0 := Vector3(b2.x, 0.0, b2.y)
		var a1 := Vector3(a2.x, th, a2.y); var b1 := Vector3(b2.x, th, b2.y)
		for v in [a0, a1, b0, b0, a1, b1]:
			tr.add_vertex(v)
	tr.generate_normals()
	tr.commit(mesh)
	var cn := SurfaceTool.new(); cn.begin(Mesh.PRIMITIVE_TRIANGLES)
	var cr := maxf(1.5, h * 0.13)                   # slender canopy radius (redwood profile)
	var cb := th * 0.70                             # canopy base (overlaps trunk top)
	var apex := Vector3(0.0, h, 0.0)
	var sides := 8
	for i in range(sides):
		var a := TAU * float(i) / float(sides)
		var b := TAU * float(i + 1) / float(sides)
		var p0 := Vector3(cos(a) * cr, cb, sin(a) * cr)
		var p1 := Vector3(cos(b) * cr, cb, sin(b) * cr)
		for v in [p0, apex, p1]:
			cn.add_vertex(v)
		for v in [p0, p1, Vector3(0.0, cb, 0.0)]:   # base cap (so it's not see-through)
			cn.add_vertex(v)
	cn.generate_normals()
	cn.commit(mesh)
	var mt := StandardMaterial3D.new(); mt.albedo_color = Color(0.30, 0.23, 0.15); mt.roughness = 1.0
	var mc := StandardMaterial3D.new(); mc.albedo_color = Color(0.19, 0.34, 0.17); mc.roughness = 1.0
	mesh.surface_set_material(0, mt)
	mesh.surface_set_material(1, mc)
	return mesh


# Scatter trees inside OSM forest/wood polygons (rejection-sampled, draped to terrain, kept
# off the road), as one MultiMeshInstance3D. Seeded RNG → stable placement across runs.
func _build_trees() -> void:
	if not show_trees:
		return
	var polys := []
	for lu in _load_features().get("landuse", []):
		var cls = lu["class"]
		# forest/wood = trees; nature_reserve = open-space preserves, densely wooded in
		# redwood country (the largest treed land in many routes — don't skip it).
		if cls == "forest" or cls == "wood" or cls == "nature_reserve":
			polys.append(_flipx(lu["pts"]))
	if polys.is_empty():
		print("trees: no forest/wood landuse in this world")
		return
	var rng := RandomNumberGenerator.new()
	rng.seed = 20260618
	var xforms: Array[Transform3D] = []
	for poly_arr in polys:
		if xforms.size() >= tree_max:
			break
		var poly := PackedVector2Array()
		var minx := INF; var maxx := -INF; var minz := INF; var maxz := -INF
		for p in poly_arr:
			var px := float(p[0]); var pz := float(p[1])
			poly.append(Vector2(px, pz))
			minx = minf(minx, px); maxx = maxf(maxx, px)
			minz = minf(minz, pz); maxz = maxf(maxz, pz)
		if poly.size() < 3:
			continue
		var target := int((maxx - minx) * (maxz - minz) / (tree_spacing_m * tree_spacing_m))
		target = mini(target, 20000)                # per-poly cap (big preserves fill in)
		for k in range(target):
			if xforms.size() >= tree_max:
				break
			var x := rng.randf_range(minx, maxx)
			var z := rng.randf_range(minz, maxz)
			if not Geometry2D.is_point_in_polygon(Vector2(x, z), poly):
				continue
			var ni := _nearest_route(Vector2(x, z))  # keep trees off the road
			if ni >= 0 and Vector2(x, z).distance_to(Vector2(float(pts[ni].x), float(pts[ni].z))) < road_width:
				continue
			var basis := Basis().scaled(Vector3.ONE * rng.randf_range(0.6, 1.5))
			basis = basis.rotated(Vector3.UP, rng.randf() * TAU)
			xforms.append(Transform3D(basis, Vector3(x, _terrain_y(x, z), z)))
	if xforms.is_empty():
		return
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = _make_tree_mesh()
	mm.instance_count = xforms.size()
	for i in range(xforms.size()):
		mm.set_instance_transform(i, xforms[i])
	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = mm
	add_child(mmi)
	tree_mmi = mmi   # quality system toggles its visibility + shadow casting
	print("trees: %d instances across %d forest/wood polys" % [xforms.size(), polys.size()])


# --- mesh building ----------------------------------------------------------

# Cut/fill a graded roadbed into the DEM heightfield along the route (the digital
# D6 bench). Carving heights[] BEFORE any mesh build means road, rider, camera and
# features — all of which read _terrain_y — automatically sit on the bench, and the
# banking-cap leveling becomes a no-op (edges now sample flat ground). This removes
# the manufactured vertical roller-coaster on steep-walled, near-flat sections.
# The road's vertical profile. PREFER the route's own recorded elevation (pts.y) —
# it's the true, smooth grade (a Garmin course / GPS track) — over sampling the DEM,
# which at coarse cell sizes manufactures hops the road then climbs. Fall back to the
# DEM only if the course elevation is missing/untrustworthy (far off the terrain).
# Lightly smoothed over distance to shed any residual jitter.
func _compute_road_grade() -> void:
	var n := pts.size()
	var course := PackedFloat32Array(); course.resize(n)
	var demc := PackedFloat32Array(); demc.resize(n)
	var diff := 0.0
	for i in range(n):
		course[i] = float(pts[i].y)
		demc[i] = _terrain_y(float(pts[i].x), float(pts[i].z))
		diff += absf(course[i] - demc[i])
	diff /= float(maxi(1, n))
	var use_course := diff < 40.0   # course tracks the terrain → it's real elevation
	var src := course if use_course else demc
	road_grade = PackedFloat32Array(); road_grade.resize(n)
	var half_win := carve_grade_smooth_m * 0.5
	var j0 := 0
	var j1 := 0
	for i in range(n):
		var di := float(pts[i].d)
		while j0 < n - 1 and float(pts[j0].d) < di - half_win:
			j0 += 1
		while j1 < n - 1 and float(pts[j1 + 1].d) <= di + half_win:
			j1 += 1
		var acc := 0.0
		for k in range(j0, j1 + 1):
			acc += src[k]
		road_grade[i] = acc / float(maxi(1, j1 - j0 + 1))
	print("road grade: %s source (mean |course-DEM| = %.1f m)" % [
		"course" if use_course else "DEM", diff])


func _carve_road_bench() -> void:
	if not carve_road_bench or pts.is_empty() or heights.is_empty():
		return
	var n := pts.size()
	_compute_road_grade()   # fills road_grade (course elevation preferred over DEM)
	# Stamp the corridor into heights[] (nearest route station wins per cell):
	#    flat bench within half-width, smoothstep back to original over the shoulder.
	var orig := heights.duplicate()
	# Pin a flat bench at least one grid cell wider than the road on each side: this
	# guarantees the terrain verts BRACKETING the road sit AT road level, so no triangle
	# can lean over the road edge ("mudslide" bleed) — the heightfield-domain stitch that
	# replaces the offset apron ribbon (which self-folded on switchbacks). Pure heights[]
	# edits can't self-intersect. Cost: a slightly wider flat cut on coarse bakes.
	var cell_diag := sqrt(mpp_x * mpp_x + mpp_z * mpp_z)
	var bench_hw := maxf(carve_bench_m * 0.5, road_width * 0.5 + cell_diag)
	var reach := bench_hw + carve_blend_m
	var best := PackedFloat32Array(); best.resize(gw * gh); best.fill(INF)
	carve_cut = PackedFloat32Array(); carve_cut.resize(gw * gh); carve_cut.fill(0.0)
	var rad_c := int(ceil(reach / absf(mpp_x))) + 1
	var rad_r := int(ceil(reach / absf(mpp_z))) + 1
	for i in range(n):
		var wx := float(pts[i].x)
		var wz := float(pts[i].z)
		var g := road_grade[i]
		var cc := int(floor((-wx - x0) / mpp_x))
		var rc := int(floor((wz - z0) / mpp_z))
		for r in range(maxi(rc - rad_r, 0), mini(rc + rad_r, gh - 1) + 1):
			for c in range(maxi(cc - rad_c, 0), mini(cc + rad_c, gw - 1) + 1):
				var cellx := -(x0 + c * mpp_x)
				var cellz := z0 + r * mpp_z
				var dlat := sqrt((cellx - wx) * (cellx - wx) + (cellz - wz) * (cellz - wz))
				if dlat > reach:
					continue
				var idx := r * gw + c
				if dlat >= best[idx]:
					continue
				best[idx] = dlat
				# cut-bank factor: full dirt across the bench, fading to natural ground by
				# the shoulder edge (nearest station wins, same as the height stamp).
				carve_cut[idx] = 1.0 - smoothstep(bench_hw, reach, dlat)
				if dlat <= bench_hw:
					heights[idx] = g
				else:
					var w := smoothstep(0.0, 1.0, (dlat - bench_hw) / carve_blend_m)
					heights[idx] = lerpf(g, orig[idx], w)
	print("carved road bench: %d stations, bench %.0fm + %.0fm shoulder" % [
		n, carve_bench_m, carve_blend_m])


func _build_terrain() -> void:
	var verts := PackedVector3Array(); verts.resize(gw * gh)
	var norms := PackedVector3Array(); norms.resize(gw * gh)
	var cols := PackedColorArray(); cols.resize(gw * gh)
	var lu := _landuse_grid()   # OSM landcover baked into the mesh (conformed, opaque)

	for r in range(gh):
		for c in range(gw):
			var i := r * gw + c
			var y := heights[i]
			verts[i] = Vector3(-(x0 + c * mpp_x), y, z0 + r * mpp_z)   # East negated
			# normal from heightfield central differences (X-comp flips with East)
			var cl := maxi(c - 1, 0); var cr := mini(c + 1, gw - 1)
			var ru := maxi(r - 1, 0); var rd := mini(r + 1, gh - 1)
			var dydx := (heights[r * gw + cr] - heights[r * gw + cl]) / ((cr - cl) * mpp_x)
			var dydz := (heights[rd * gw + c] - heights[ru * gw + c]) / ((rd - ru) * mpp_z)
			var nrm := Vector3(dydx, 1.0, -dydz).normalized()
			norms[i] = nrm
			# Slope shading: grey out steep faces (rock/cut bank) vs green-tan flats,
			# so canyon walls read as walls. nrm.y = 1 flat → 0 vertical.
			# Vertex color carries the biome base (rgb, elevation ramp) + the carve cut-bank
			# factor (alpha). Slope-rock greying, procedural grain, and the dirt cut bank are
			# applied per-fragment in the terrain shader (triplanar, procedural — no assets).
			var col := _elev_color(y)
			if lu[i].a > 0.0:
				col = col.lerp(Color(lu[i].r, lu[i].g, lu[i].b), 0.55)
			col.a = carve_cut[i] if carve_cut.size() == gw * gh else 0.0
			cols[i] = col

	# Punch a hole along the route: skip every cell within apron_reach of the route,
	# leaving a gap the apron mesh (_build_road_apron) fills so terrain ends at the road
	# edge instead of leaning over it. Only when the apron is on, else the gap would show.
	var blocked := _blocked_cells() if (road_apron and not pts.is_empty()) else PackedByteArray()
	var idx := PackedInt32Array()
	for r in range(gh - 1):
		for c in range(gw - 1):
			if not blocked.is_empty() and blocked[r * gw + c] != 0:
				continue
			var a := r * gw + c
			var b := r * gw + c + 1
			var d := (r + 1) * gw + c
			var e := (r + 1) * gw + c + 1
			idx.push_back(a); idx.push_back(d); idx.push_back(b)
			idx.push_back(b); idx.push_back(d); idx.push_back(e)

	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = norms
	arr[Mesh.ARRAY_COLOR] = cols
	arr[Mesh.ARRAY_INDEX] = idx
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

	mesh.surface_set_material(0, _terrain_material())

	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)


# Cells whose center lies within apron_reach of the route — the ones the apron replaces.
# Mirrors the carve's station-stamp loop + coordinate inversion (East negated).
func _blocked_cells() -> PackedByteArray:
	var blocked := PackedByteArray(); blocked.resize(gw * gh); blocked.fill(0)
	var reach := apron_reach_m
	var rad_c := int(ceil(reach / absf(mpp_x))) + 1
	var rad_r := int(ceil(reach / absf(mpp_z))) + 1
	for i in range(pts.size()):
		var wx := float(pts[i].x)
		var wz := float(pts[i].z)
		var cc := int(floor((-wx - x0) / mpp_x))
		var rc := int(floor((wz - z0) / mpp_z))
		for r in range(maxi(rc - rad_r, 0), mini(rc + rad_r, gh - 2) + 1):
			for c in range(maxi(cc - rad_c, 0), mini(cc + rad_c, gw - 2) + 1):
				var cx := -(x0 + (c + 0.5) * mpp_x)   # cell center
				var cz := z0 + (r + 0.5) * mpp_z
				if (cx - wx) * (cx - wx) + (cz - wz) * (cz - wz) <= reach * reach:
					blocked[r * gw + c] = 1
	return blocked


# Fill the route hole with a graded shoulder: a strip down each road edge whose inner edge
# is the asphalt edge (road height, a shared seam) and whose outer edge drapes to true
# terrain past the hole — so terrain triangles terminate at the road edge, no bleed, and
# the cut bank still follows the real hillside (no over-flattening).
func _build_road_apron() -> void:
	var n := pts.size()
	if n < 2 or road_center_y.size() != n:
		return
	var hw := road_width * 0.5
	var cell_diag := sqrt(mpp_x * mpp_x + mpp_z * mpp_z)
	var outer := apron_reach_m + cell_diag        # reach past the hole so there's no gap
	# Per-station miter normal + scale (same math as _add_centered_ribbon).
	var nrm_at := PackedVector2Array(); nrm_at.resize(n)
	var scale_at := PackedFloat32Array(); scale_at.resize(n)
	for i in range(n):
		var here := Vector2(float(pts[i].x), float(pts[i].z))
		var din := Vector2.ZERO
		var dout := Vector2.ZERO
		if i > 0:
			din = (here - Vector2(float(pts[i - 1].x), float(pts[i - 1].z))).normalized()
		if i < n - 1:
			dout = (Vector2(float(pts[i + 1].x), float(pts[i + 1].z)) - here).normalized()
		var ref := dout if dout != Vector2.ZERO else din
		var tang := din + dout
		if tang.length() < 0.00001:
			tang = ref
		tang = tang.normalized()
		var nrm := Vector2(tang.y, -tang.x)
		var refn := Vector2(ref.y, -ref.x)
		var dotp := nrm.dot(refn)
		var s := 1.0
		if absf(dotp) > 0.25:
			s = 1.0 / dotp
		nrm_at[i] = nrm
		scale_at[i] = clampf(s, -3.0, 3.0)
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for side in [1.0, -1.0]:                       # +1 left edge, -1 right edge
		for i in range(n - 1):
			_apron_quad(st, i, i + 1, side, hw, outer, nrm_at, scale_at)
	var mesh := st.commit()
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.roughness = 1.0
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mesh.surface_set_material(0, mat)
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)
	print("road apron: %.0fm reach, hole + shoulder along %d stations" % [outer, n])


func _apron_quad(st: SurfaceTool, i: int, j: int, side: float, hw: float, outer: float,
		nrm_at: PackedVector2Array, scale_at: PackedFloat32Array) -> void:
	var ci := Vector2(float(pts[i].x), float(pts[i].z))
	var cj := Vector2(float(pts[j].x), float(pts[j].z))
	# Inner edge keeps the asphalt's miter (so it stays glued to the road edge); the OUTER
	# edge uses a plain bisector offset (NO 1/dotp magnification) — magnifying a ~20 m offset
	# at a bend flings vertices tens of metres out and tears the mesh into non-manifold spikes.
	var inn_i := ci + nrm_at[i] * (side * hw * scale_at[i])
	var inn_j := cj + nrm_at[j] * (side * hw * scale_at[j])
	var out_i := ci + nrm_at[i] * (side * outer)
	var out_j := cj + nrm_at[j] * (side * outer)
	# inner edge at road height (coincident with the asphalt edge); outer drapes to terrain,
	# nudged just under it so the surviving grid wins any overlap fringe (no z-fight).
	var I0 := Vector3(inn_i.x, road_center_y[i] + road_lift, inn_i.y)
	var I1 := Vector3(inn_j.x, road_center_y[j] + road_lift, inn_j.y)
	var O0 := Vector3(out_i.x, _terrain_y(out_i.x, out_i.y) - 0.05, out_i.y)
	var O1 := Vector3(out_j.x, _terrain_y(out_j.x, out_j.y) - 0.05, out_j.y)
	_apron_tri(st, I0, O0, I1)
	_apron_tri(st, I1, O0, O1)


func _apron_tri(st: SurfaceTool, a: Vector3, b: Vector3, c: Vector3) -> void:
	var nrm := (b - a).cross(c - a).normalized()
	if nrm.y < 0.0:
		nrm = -nrm
	# Color exactly like _build_terrain: elevation ramp + slope shading, so a steep apron
	# face greys to rock like the cut bank it is and blends with the surrounding terrain.
	for v in [a, b, c]:
		var col := _elev_color(v.y)
		if terrain_slope_shading:
			var steep := clampf((1.0 - nrm.y - 0.22) / 0.5, 0.0, 1.0)
			col = col.lerp(Color(0.34, 0.31, 0.29), steep * 0.8)
		st.set_color(col)
		st.set_normal(nrm)
		st.add_vertex(v)


func _build_road() -> void:
	# The ridden road: asphalt + a bright center line that streams past as a motion
	# cue. Built as ONE coplanar flat-bed ribbon — asphalt, center line, and the
	# rider all sit on the same leveled center height (road_center_y), so nothing
	# z-fights or buries (the old code draped each independently → buried line/rider).
	_build_route_index()
	_build_curv_lut()
	_compute_road_center_y()
	var line := PackedVector2Array()
	for p in pts:
		line.append(Vector2(float(p.x), float(p.z)))
	_road_surface(line, road_width * 0.5, road_lift, Color(0.30, 0.30, 0.33), 0.9, _tarmac_material())
	# Translucent centerline, lifted only a hair above the tarmac so the rider's
	# wheels don't bury in it. Kept as a handle so the Detail panel can toggle it.
	var cl_mat := StandardMaterial3D.new()
	cl_mat.albedo_color = Color(0.95, 0.82, 0.15, clampf(centerline_alpha, 0.0, 1.0))
	cl_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	cl_mat.roughness = 0.6
	cl_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	centerline_mi = _road_surface(line, 0.30, road_lift + centerline_lift,
			Color(0.95, 0.82, 0.15), 0.6, cl_mat)
	if centerline_mi != null:
		centerline_mi.visible = show_centerline
	if road_apron:
		_build_road_apron()


# Precompute per-route-point heading + a coarse spatial grid, so the OSM cull can
# find the nearest route point/direction for any feature segment in ~O(1).
func _build_route_index() -> void:
	var n := pts.size()
	route_dir = PackedVector2Array(); route_dir.resize(n)
	_route_grid.clear()
	for i in range(n):
		var here := Vector2(float(pts[i].x), float(pts[i].z))
		var nxt := here
		if i < n - 1:
			nxt = Vector2(float(pts[i + 1].x), float(pts[i + 1].z))
		elif i > 0:
			here = Vector2(float(pts[i - 1].x), float(pts[i - 1].z))
			nxt = Vector2(float(pts[i].x), float(pts[i].z))
		var dir := (nxt - here)
		route_dir[i] = dir.normalized() if dir.length() > 0.001 else Vector2.RIGHT
		var key := _cell_key(float(pts[i].x), float(pts[i].z))
		if not _route_grid.has(key):
			_route_grid[key] = PackedInt32Array()
		_route_grid[key].append(i)


func _cell_key(x: float, z: float) -> Vector2i:
	return Vector2i(int(floor(x / ROUTE_CELL)), int(floor(z / ROUTE_CELL)))


# Nearest route-point index to p, or -1 if none within the 3×3 cell neighborhood.
func _nearest_route(p: Vector2) -> int:
	var best := -1
	var bestd := INF
	var c := _cell_key(p.x, p.y)
	for dx in range(-1, 2):
		for dz in range(-1, 2):
			var key := Vector2i(c.x + dx, c.y + dz)
			if _route_grid.has(key):
				for i in _route_grid[key]:
					var rp := Vector2(float(pts[i].x), float(pts[i].z))
					var dd := rp.distance_squared_to(p)
					if dd < bestd:
						bestd = dd
						best = i
	return best


# The leveled ridden-road center height at each route point — same banking-cap
# logic the old ribbon used per edge, but resolved to ONE center value per station.
func _compute_road_center_y() -> void:
	var n := pts.size()
	road_center_y = PackedFloat32Array(); road_center_y.resize(n)
	# When the bench carve ran, the road rides its course-derived grade and the
	# terrain was carved to meet it — use it directly (no DEM leveling needed).
	if road_grade.size() == n and not road_grade.is_empty():
		for i in range(n):
			road_center_y[i] = road_grade[i]
		return
	# Fallback (carve disabled): the old DEM banking-cap leveling.
	var hw := road_width * 0.5
	var max_dh := hw * tan(deg_to_rad(road_bank_max_deg))
	for i in range(n):
		var c := Vector2(float(pts[i].x), float(pts[i].z))
		var din := Vector2.ZERO
		var dout := Vector2.ZERO
		if i > 0:
			din = (c - Vector2(float(pts[i - 1].x), float(pts[i - 1].z))).normalized()
		if i < n - 1:
			dout = (Vector2(float(pts[i + 1].x), float(pts[i + 1].z)) - c).normalized()
		var tang := din + dout
		if tang.length() < 0.00001:
			tang = dout if dout != Vector2.ZERO else din
		tang = tang.normalized()
		var nrm := Vector2(tang.y, -tang.x)
		var lft := c + nrm * hw
		var rgt := c - nrm * hw
		var hc := _terrain_y(c.x, c.y)
		var hl := _terrain_y(lft.x, lft.y)
		var hr := _terrain_y(rgt.x, rgt.y)
		var h_ref := maxf(hc, maxf(hl, hr) - max_dh)
		var yl := clampf(hl, h_ref - max_dh, h_ref + max_dh)
		var yr := clampf(hr, h_ref - max_dh, h_ref + max_dh)
		road_center_y[i] = (yl + yr) * 0.5


# Flat-across mitered ribbon along the route, every station pinned to its
# precomputed center height + lift. One mesh, one material.
func _road_surface(line: PackedVector2Array, hw: float, lift: float,
		col: Color, rough: float, mat_override: Material = null) -> MeshInstance3D:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	if not _add_centered_ribbon(st, line, hw, lift):
		return null
	var mesh := st.commit()
	var mat: Material = mat_override
	if mat == null:
		var sm := StandardMaterial3D.new()
		sm.albedo_color = col
		sm.roughness = rough
		sm.cull_mode = BaseMaterial3D.CULL_DISABLED
		mat = sm
	mesh.surface_set_material(0, mat)
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)
	return mi


func _add_centered_ribbon(st: SurfaceTool, line: PackedVector2Array,
		hw: float, lift: float) -> bool:
	var n := line.size()
	if n < 2 or road_center_y.size() != n:
		return false
	var left := PackedVector2Array(); left.resize(n)
	var right := PackedVector2Array(); right.resize(n)
	for i in range(n):
		var din := Vector2.ZERO
		var dout := Vector2.ZERO
		if i > 0:
			din = (line[i] - line[i - 1]).normalized()
		if i < n - 1:
			dout = (line[i + 1] - line[i]).normalized()
		var ref := dout if dout != Vector2.ZERO else din
		var tang := din + dout
		if tang.length() < 0.00001:
			tang = ref
		tang = tang.normalized()
		var nrm := Vector2(tang.y, -tang.x)
		var refn := Vector2(ref.y, -ref.x)
		var dotp := nrm.dot(refn)
		var scale := hw
		if absf(dotp) > 0.25:
			scale = hw / dotp
		scale = clampf(scale, -hw * 3.0, hw * 3.0)
		left[i] = line[i] + nrm * scale
		right[i] = line[i] - nrm * scale
	var added := false
	for i in range(n - 1):
		var y0 := road_center_y[i] + lift
		var y1 := road_center_y[i + 1] + lift
		var L0 := Vector3(left[i].x, y0, left[i].y)
		var R0 := Vector3(right[i].x, y0, right[i].y)
		var L1 := Vector3(left[i + 1].x, y1, left[i + 1].y)
		var R1 := Vector3(right[i + 1].x, y1, right[i + 1].y)
		for v in [L0, L1, R0, R0, L1, R1]:
			st.set_normal(Vector3.UP)
			st.add_vertex(v)
		added = true
	return added


# Drop OSM road segments that run parallel AND close to the ridden route (the
# duplicate under our ribbon), splitting each road at the culled spans so cross
# streets and divergent stretches survive intact. Returns filtered [x,z] lines.
func _cull_overlap(lines: Array) -> Array:
	if not cull_route_overlap or pts.is_empty():
		return lines
	var par_cos := cos(deg_to_rad(cull_parallel_deg))
	var out := []
	for line in lines:
		if line.size() < 2:
			out.append(line)
			continue
		var run := []
		for si in range(line.size() - 1):
			var a := Vector2(float(line[si][0]), float(line[si][1]))
			var b := Vector2(float(line[si + 1][0]), float(line[si + 1][1]))
			var sdir := b - a
			var cull := false
			if sdir.length() > 0.001:
				sdir = sdir.normalized()
				var mid := (a + b) * 0.5
				var ni := _nearest_route(mid)
				if ni >= 0:
					var rp := Vector2(float(pts[ni].x), float(pts[ni].z))
					if mid.distance_to(rp) <= cull_overlap_m \
							and absf(sdir.dot(route_dir[ni])) >= par_cos:
						cull = true
			if cull:
				if run.size() >= 2:
					out.append(run)
				run = []
			else:
				if run.is_empty():
					run.append(line[si])
				run.append(line[si + 1])
		if run.size() >= 2:
			out.append(run)
	return out


# Append one polyline to a SurfaceTool as a MITERED ribbon: each joint shares
# angle-averaged left/right vertices (scaled to hold width through the bend, with
# a spike clamp), so segments connect instead of tearing into overlaps/gaps the
# way independent per-segment quads do. Roads are up-facing; normal forced up.
# Returns true if anything was added.
func _add_ribbon(st: SurfaceTool, line: PackedVector2Array, hw: float, lift: float) -> bool:
	var n := line.size()
	if n < 2:
		return false
	var left := PackedVector2Array(); left.resize(n)
	var right := PackedVector2Array(); right.resize(n)
	for i in range(n):
		var din := Vector2.ZERO
		var dout := Vector2.ZERO
		if i > 0:
			din = (line[i] - line[i - 1]).normalized()
		if i < n - 1:
			dout = (line[i + 1] - line[i]).normalized()
		var ref := dout if dout != Vector2.ZERO else din
		var tang := din + dout
		if tang.length() < 0.00001:
			tang = ref                       # near-reversal: fall back to one side
		tang = tang.normalized()
		var nrm := Vector2(tang.y, -tang.x)  # left perpendicular of the tangent
		var refn := Vector2(ref.y, -ref.x)   # left perpendicular of a segment
		var dotp := nrm.dot(refn)
		var scale := hw
		if absf(dotp) > 0.25:
			scale = hw / dotp                # miter length holds constant width
		scale = clampf(scale, -hw * 3.0, hw * 3.0)
		left[i] = line[i] + nrm * scale
		right[i] = line[i] - nrm * scale
	# Per-point edge heights with a banking cap: draping each edge to its own
	# terrain banks the road to the hillside cross-slope (up to ~45°). Limit how
	# far each edge may sit from the centerline (max_dh = hw*tan(cap)). Raise the
	# reference so the higher edge is buried at most max_dh — keeps the road from
	# diving under the uphill bank while staying near-flat across.
	var max_dh := hw * tan(deg_to_rad(road_bank_max_deg))
	var ly := PackedFloat32Array(); ly.resize(n)
	var ry := PackedFloat32Array(); ry.resize(n)
	for i in range(n):
		var hc := _terrain_y(line[i].x, line[i].y)
		var hl := _terrain_y(left[i].x, left[i].y)
		var hr := _terrain_y(right[i].x, right[i].y)
		var h_ref := maxf(hc, maxf(hl, hr) - max_dh)
		ly[i] = clampf(hl, h_ref - max_dh, h_ref + max_dh) + lift
		ry[i] = clampf(hr, h_ref - max_dh, h_ref + max_dh) + lift
	var added := false
	for i in range(n - 1):
		var l0 := left[i]; var r0 := right[i]
		var l1 := left[i + 1]; var r1 := right[i + 1]
		var L0 := Vector3(l0.x, ly[i], l0.y)
		var R0 := Vector3(r0.x, ry[i], r0.y)
		var L1 := Vector3(l1.x, ly[i + 1], l1.y)
		var R1 := Vector3(r1.x, ry[i + 1], r1.y)
		for v in [L0, L1, R0, R0, L1, R1]:
			st.set_normal(Vector3.UP)
			st.add_vertex(v)
		added = true
	return added


# --- OSM feature layers (roads, water, landuse) -----------------------------

func _build_features() -> void:
	var data := _load_features()
	if data.is_empty():
		print("no features.json — skipping OSM layers (run tools/osm_to_features.py)")
		return

	if show_roads:
		var route_to_bucket := {
			"motorway": "major", "trunk": "major", "primary": "major",
			"secondary": "major", "tertiary": "major", "motorway_link": "major",
			"trunk_link": "major", "primary_link": "major", "secondary_link": "major",
			"tertiary_link": "major",
			"residential": "minor", "unclassified": "minor", "living_street": "minor",
			"road": "minor", "pedestrian": "minor",
			"service": "service",
			"footway": "path", "cycleway": "path", "path": "path", "track": "path",
			"steps": "path", "bridleway": "path", "corridor": "path",
		}
		var bucket := {"major": [], "minor": [], "service": [], "path": []}
		for r in data.get("roads", []):
			var b = route_to_bucket.get(r["class"], "minor")
			if b == "service" and not show_service:
				continue
			if b == "path" and not show_paths:
				continue
			bucket[b].append(_flipx(r["pts"]))
		var style := {
			"major": {"hw": 4.5, "col": Color(0.36, 0.36, 0.38)},
			"minor": {"hw": 2.6, "col": Color(0.33, 0.33, 0.35)},
			"service": {"hw": 1.6, "col": Color(0.30, 0.30, 0.31)},
			"path": {"hw": 0.9, "col": Color(0.52, 0.43, 0.30)},
		}
		for b in ["major", "minor", "service", "path"]:
			if not bucket[b].is_empty():
				var lines := _cull_overlap(bucket[b])  # drop the parallel route duplicate
				if not lines.is_empty():
					_drape_lines(lines, style[b]["hw"], style[b]["col"], feature_lift, 0.95)

	if show_water:
		var wlines := []
		for w in data.get("waterways", []):
			wlines.append(_flipx(w["pts"]))
		if not wlines.is_empty():
			_drape_lines(wlines, 1.5, Color(0.20, 0.40, 0.55), feature_lift * 0.4, 0.2)
		var wpolys := []
		for w in data.get("water", []):
			wpolys.append(_flipx(w["pts"]))
		if not wpolys.is_empty():
			_drape_polygons(wpolys, Color(0.18, 0.38, 0.55), feature_lift * 0.3, 0.85)

	# Landuse landcover is baked into the terrain vertex colors (conformed, opaque) in
	# _landuse_grid()/_build_terrain — no floating translucent overlay / green stripe.

	_build_buildings(data)
	_build_barriers()
	_build_power()
	_build_bridges()


# Extrude OSM building footprints: walls + flat roof, base sunk to the lowest
# terrain under the footprint so it doesn't float. All buildings batch into two
# meshes (walls, roofs). Footprint x is East-negated like every other feature.
func _build_buildings(data: Dictionary) -> void:
	if not show_buildings:
		return
	var blds: Array = data.get("buildings", [])
	if blds.is_empty():
		return
	var walls := SurfaceTool.new(); walls.begin(Mesh.PRIMITIVE_TRIANGLES)
	var roofs := SurfaceTool.new(); roofs.begin(Mesh.PRIMITIVE_TRIANGLES)
	# small palettes so the block of buildings isn't one flat grey — varied per building
	var wall_pal := [Color(0.74, 0.70, 0.62), Color(0.66, 0.62, 0.58), Color(0.80, 0.76, 0.68),
		Color(0.60, 0.56, 0.52), Color(0.72, 0.66, 0.60), Color(0.68, 0.64, 0.66)]
	var roof_pal := [Color(0.44, 0.27, 0.20), Color(0.34, 0.33, 0.35), Color(0.30, 0.27, 0.25),
		Color(0.50, 0.31, 0.22), Color(0.26, 0.30, 0.31), Color(0.38, 0.20, 0.17)]
	var any := false
	for b in blds:
		var ring := PackedVector2Array()
		for p in b["pts"]:
			ring.append(Vector2(-float(p[0]), float(p[1])))
		if ring.size() > 1 and ring[0] == ring[ring.size() - 1]:
			ring.remove_at(ring.size() - 1)
		if ring.size() < 3:
			continue
		var h := float(b.get("h", 6.0)) * building_height_scale
		var base_y := INF
		var cx := 0.0; var cz := 0.0
		for v in ring:
			base_y = minf(base_y, _terrain_y(v.x, v.y))
			cx += v.x; cz += v.y
		cx /= ring.size(); cz /= ring.size()
		var roof_y := base_y + h
		var key := int(absf(cx) * 0.7 + absf(cz) * 1.3)   # deterministic per-building tint
		var wcol: Color = wall_pal[key % wall_pal.size()]
		var rcol: Color = roof_pal[(key / 2) % roof_pal.size()]
		var n := ring.size()
		for i in range(n):
			var a := ring[i]
			var c := ring[(i + 1) % n]
			var ex := c.x - a.x; var ez := c.y - a.y
			var elen := sqrt(ex * ex + ez * ez)
			if elen < 0.001:
				continue
			var nx := ez / elen; var nz := -ex / elen
			if nx * ((a.x + c.x) * 0.5 - cx) + nz * ((a.y + c.y) * 0.5 - cz) < 0.0:
				nx = -nx; nz = -nz          # face outward (away from centroid)
			var nrm := Vector3(nx, 0.0, nz)
			var va := Vector3(a.x, base_y, a.y)
			var vb := Vector3(c.x, base_y, c.y)
			var vc := Vector3(a.x, roof_y, a.y)
			var vd := Vector3(c.x, roof_y, c.y)
			for v in [va, vc, vb, vb, vc, vd]:
				walls.set_color(wcol); walls.set_normal(nrm); walls.add_vertex(v)
		# pitched hip roof: each footprint edge → a triangle up to a central apex
		var roof_h := clampf(h * 0.28, 1.5, 4.5)
		var apex := Vector3(cx, roof_y + roof_h, cz)
		for j in range(n):
			var ea := ring[j]
			var ec := ring[(j + 1) % n]
			var ra := Vector3(ea.x, roof_y, ea.y)
			var rb := Vector3(ec.x, roof_y, ec.y)
			var rn := (rb - ra).cross(apex - ra).normalized()
			if rn.y < 0.0:
				rn = -rn
			for v in [ra, rb, apex]:
				roofs.set_color(rcol); roofs.set_normal(rn); roofs.add_vertex(v)
		any = true
	if not any:
		return
	_commit_vcol(walls, 0.9)
	_commit_vcol(roofs, 0.85)


func _commit_surface(st: SurfaceTool, col: Color, rough: float) -> void:
	var mesh := st.commit()
	if mesh == null:
		return
	var mat := StandardMaterial3D.new()
	mat.albedo_color = col
	mat.roughness = rough
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mesh.surface_set_material(0, mat)
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)


# Commit a SurfaceTool that already carries per-vertex colors (vertex_color_use_as_albedo).
func _commit_vcol(st: SurfaceTool, rough: float) -> void:
	var mesh := st.commit()
	if mesh == null:
		return
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.roughness = rough
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mesh.surface_set_material(0, mat)
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)


# A vertical wall strip along a draped polyline (fences, hedges, walls, railings):
# each segment is a quad from terrain (+base_lift) up to +height, per-vertex colored.
func _add_vertical_ribbon(st: SurfaceTool, line: PackedVector2Array, height: float,
		base_lift: float, col: Color) -> void:
	for i in range(line.size() - 1):
		var a := line[i]; var b := line[i + 1]
		var ya := _terrain_y(a.x, a.y) + base_lift
		var yb := _terrain_y(b.x, b.y) + base_lift
		var dx := b.x - a.x; var dz := b.y - a.y
		var dl := sqrt(dx * dx + dz * dz)
		if dl < 0.001:
			continue
		var nrm := Vector3(dz / dl, 0.0, -dx / dl)
		var A0 := Vector3(a.x, ya, a.y)
		var B0 := Vector3(b.x, yb, b.y)
		var A1 := Vector3(a.x, ya + height, a.y)
		var B1 := Vector3(b.x, yb + height, b.y)
		for v in [A0, A1, B0, B0, A1, B1]:
			st.set_color(col); st.set_normal(nrm); st.add_vertex(v)


# A thin square post from y=0 to y=h (power poles), one mesh for MultiMesh instancing.
func _make_post_mesh(h: float, w: float, col: Color) -> ArrayMesh:
	var st := SurfaceTool.new(); st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var corners := [Vector2(-w, -w), Vector2(w, -w), Vector2(w, w), Vector2(-w, w)]
	for i in range(4):
		var a2: Vector2 = corners[i]
		var b2: Vector2 = corners[(i + 1) % 4]
		var a0 := Vector3(a2.x, 0.0, a2.y); var b0 := Vector3(b2.x, 0.0, b2.y)
		var a1 := Vector3(a2.x, h, a2.y); var b1 := Vector3(b2.x, h, b2.y)
		for v in [a0, a1, b0, b0, a1, b1]:
			st.set_color(col); st.add_vertex(v)
	st.generate_normals()
	var mesh := ArrayMesh.new(); st.commit(mesh)
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true; mat.roughness = 1.0
	mesh.surface_set_material(0, mat)
	return mesh


# OSM barriers: fences / hedges / walls as draped vertical strips, colored by type.
func _build_barriers() -> void:
	if not show_barriers:
		return
	var bars: Array = _load_features().get("barriers", [])
	if bars.is_empty():
		return
	var st := SurfaceTool.new(); st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var any := false
	for bar in bars:
		var line := PackedVector2Array()
		for p in _flipx(bar["pts"]):
			line.append(Vector2(float(p[0]), float(p[1])))
		if line.size() < 2:
			continue
		var cls = bar.get("class", "fence")
		var h := 1.1
		var col := Color(0.46, 0.41, 0.34)              # fence: weathered wood/wire
		if cls == "hedge":
			h = 1.6; col = Color(0.21, 0.33, 0.17)      # hedge: green
		elif cls == "wall" or cls == "dry_stone_wall" or cls == "retaining_wall":
			h = 1.2; col = Color(0.50, 0.49, 0.47)      # wall: grey stone
		_add_vertical_ribbon(st, line, h, 0.0, col)
		any = true
	if any:
		_commit_vcol(st, 1.0)
		print("barriers: %d" % bars.size())


# OSM power: wires as thin dark ribbons elevated on poles + posts at each tower/pole.
func _build_power() -> void:
	if not show_power:
		return
	var data := _load_features()
	var plines := []
	for pl in data.get("powerlines", []):
		plines.append(_flipx(pl["pts"]))
	if not plines.is_empty():
		_drape_lines(plines, 0.12, Color(0.05, 0.05, 0.06), power_height, 0.4)
	var poles: Array = data.get("power_poles", [])
	if poles.is_empty():
		return
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = _make_post_mesh(power_height, 0.16, Color(0.27, 0.21, 0.15))
	mm.instance_count = poles.size()
	for i in range(poles.size()):
		var x := -float(poles[i][0]); var z := float(poles[i][1])
		mm.set_instance_transform(i, Transform3D(Basis(), Vector3(x, _terrain_y(x, z), z)))
	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = mm
	add_child(mmi)
	pole_mmi = mmi   # quality system toggles its shadow casting
	print("power: %d lines, %d poles" % [plines.size(), poles.size()])


# Bridge spans (road bridge=yes): a level-ish deck (lerp between approach heights so it
# clears the dip) + side railings, so water crossings read as bridges not sunken roads.
func _build_bridges() -> void:
	if not show_bridges:
		return
	var st := SurfaceTool.new(); st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var rail := SurfaceTool.new(); rail.begin(Mesh.PRIMITIVE_TRIANGLES)
	var hw := 2.4
	var deck_col := Color(0.32, 0.32, 0.34)
	var rail_col := Color(0.52, 0.50, 0.47)
	var any := false
	for r in _load_features().get("roads", []):
		if not r.get("bridge", false):
			continue
		var line := PackedVector2Array()
		for p in _flipx(r["pts"]):
			line.append(Vector2(float(p[0]), float(p[1])))
		var n := line.size()
		if n < 2:
			continue
		var y0 := _terrain_y(line[0].x, line[0].y)
		var y1 := _terrain_y(line[n - 1].x, line[n - 1].y)
		var left := PackedVector2Array(); var right := PackedVector2Array()
		var deck_y := PackedFloat32Array(); deck_y.resize(n)
		for i in range(n):
			var t := float(i) / float(n - 1)
			# level deck across the span, but never below the local ground
			deck_y[i] = maxf(lerpf(y0, y1, t), _terrain_y(line[i].x, line[i].y)) + feature_lift + 0.4
			var din := Vector2.ZERO; var dout := Vector2.ZERO
			if i > 0:
				din = (line[i] - line[i - 1]).normalized()
			if i < n - 1:
				dout = (line[i + 1] - line[i]).normalized()
			var tang := din + dout
			if tang.length() < 0.0001:
				tang = dout if dout != Vector2.ZERO else din
			tang = tang.normalized()
			var nrm := Vector2(tang.y, -tang.x)
			left.append(line[i] + nrm * hw)
			right.append(line[i] - nrm * hw)
		for i in range(n - 1):
			var L0 := Vector3(left[i].x, deck_y[i], left[i].y)
			var R0 := Vector3(right[i].x, deck_y[i], right[i].y)
			var L1 := Vector3(left[i + 1].x, deck_y[i + 1], left[i + 1].y)
			var R1 := Vector3(right[i + 1].x, deck_y[i + 1], right[i + 1].y)
			for v in [L0, L1, R0, R0, L1, R1]:
				st.set_color(deck_col); st.set_normal(Vector3.UP); st.add_vertex(v)
		_add_rail_at(rail, left, deck_y, 0.9, rail_col)
		_add_rail_at(rail, right, deck_y, 0.9, rail_col)
		any = true
	if any:
		_commit_vcol(st, 0.9)
		_commit_vcol(rail, 0.9)


# Railing strip along an edge at explicit per-vertex heights (bridge decks).
func _add_rail_at(st: SurfaceTool, edge: PackedVector2Array, ybase: PackedFloat32Array,
		height: float, col: Color) -> void:
	for i in range(edge.size() - 1):
		var a := edge[i]; var b := edge[i + 1]
		var dx := b.x - a.x; var dz := b.y - a.y
		var dl := sqrt(dx * dx + dz * dz)
		var nrm := Vector3(dz, 0.0, -dx) / (dl if dl > 0.001 else 1.0)
		var A0 := Vector3(a.x, ybase[i], a.y)
		var B0 := Vector3(b.x, ybase[i + 1], b.y)
		var A1 := Vector3(a.x, ybase[i] + height, a.y)
		var B1 := Vector3(b.x, ybase[i + 1] + height, b.y)
		for v in [A0, A1, B0, B0, A1, B1]:
			st.set_color(col); st.set_normal(nrm); st.add_vertex(v)


# Negate East on a list of [x,z] points (OSM is an independent data source; see
# the un-mirror note in _load_data).
func _flipx(arr: Array) -> Array:
	var out := []
	for p in arr:
		out.append([-float(p[0]), float(p[1])])
	return out


func _landuse_color(cls: String) -> Color:
	match cls:
		"forest", "wood":
			return Color(0.16, 0.32, 0.16)
		"grass", "meadow", "park", "recreation_ground", "village_green", "pitch", "golf_course":
			return Color(0.34, 0.50, 0.24)
		"farmland", "farmyard", "orchard", "vineyard":
			return Color(0.52, 0.49, 0.28)
		"residential":
			return Color(0.42, 0.40, 0.40)
		"commercial", "retail", "industrial":
			return Color(0.40, 0.40, 0.44)
		_:
			return Color(0.38, 0.42, 0.30)


# Drape many polylines (each an Array of [x,z]) as one batched ribbon mesh.
func _drape_lines(lines: Array, hw: float, col: Color, lift: float, rough: float) -> void:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var added := false
	for line in lines:
		var v2 := PackedVector2Array()
		for p in line:
			v2.append(Vector2(float(p[0]), float(p[1])))
		if _add_ribbon(st, v2, hw, lift):
			added = true
	if not added:
		return
	var mesh := st.commit()
	var mat := StandardMaterial3D.new()
	mat.albedo_color = col
	mat.roughness = rough
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mesh.surface_set_material(0, mat)
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)


# Triangulate + drape closed polygons (each an Array of [x,z]) onto the terrain.
func _drape_polygons(polys: Array, col: Color, lift: float, alpha: float) -> void:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var added := false
	for poly in polys:
		var ring := PackedVector2Array()
		for p in poly:
			ring.append(Vector2(float(p[0]), float(p[1])))
		if ring.size() > 1 and ring[0] == ring[ring.size() - 1]:
			ring.remove_at(ring.size() - 1)
		if ring.size() < 3:
			continue
		var tris := Geometry2D.triangulate_polygon(ring)
		if tris.is_empty():
			continue
		for ti in tris:
			var v := ring[ti]
			st.set_normal(Vector3.UP)
			st.add_vertex(Vector3(v.x, _terrain_y(v.x, v.y) + lift, v.y))
		added = true
	if not added:
		return
	var mesh := st.commit()
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(col.r, col.g, col.b, alpha)
	mat.roughness = 1.0
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	if alpha < 1.0:
		mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mesh.surface_set_material(0, mat)
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)


func _build_camera_and_sky() -> void:
	sun = DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-45, 130, 0)
	sun.shadow_enabled = true
	sun.light_energy = 1.25                       # brighter so terrain color/grain reads
	sun.light_color = Color(1.0, 0.97, 0.90)      # slightly warm daylight
	add_child(sun)

	env = Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.fog_enabled = true
	env.fog_density = 0.00025            # lighter aerial haze (was 0.0008 = heavy wash-out)
	_fog_base = env.fog_density          # remember it so god-zoom can thin + restore the haze
	env.glow_enabled = true              # bloom for the emissive avatar (Tron look)
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	cam = Camera3D.new()
	cam.fov = 70.0
	cam.far = 12000.0
	add_child(cam)
	cam.make_current()
	_update_camera(0.0, 0.0)

	# Small bottom-left readout of the active camera mode + its controls.
	var cl := CanvasLayer.new()
	add_child(cl)
	cam_label = Label.new()
	cam_label.anchor_top = 1.0; cam_label.anchor_bottom = 1.0
	# sit in a box that ends 24 px above the screen bottom (was font 150, running off)
	cam_label.offset_left = 24.0; cam_label.offset_top = -120.0; cam_label.offset_bottom = -24.0
	cam_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	cam_label.add_theme_color_override("font_color", Color(0.6, 0.9, 1.0))
	cam_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.85))
	cam_label.add_theme_constant_override("outline_size", 8)
	cam_label.add_theme_font_size_override("font_size", 64)   # match the fps readout
	cam_label.visible = show_cam
	cl.add_child(cam_label)
	_update_cam_label()

	# Top-left perf readout: fps / frame-time / draw calls / primitives (P toggles).
	perf_label = Label.new()
	perf_label.offset_left = 24.0; perf_label.offset_top = 16.0
	perf_label.add_theme_color_override("font_color", Color(0.6, 1.0, 0.6))
	perf_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.85))
	perf_label.add_theme_constant_override("outline_size", 8)
	perf_label.add_theme_font_size_override("font_size", 64)
	perf_label.visible = show_perf
	cl.add_child(perf_label)
	_update_perf_label()

	# Quality/LOD state readout, just under the perf line (Q toggles). Shows the
	# active preset + which levers are on, so a profiling pass on the target GPU
	# is "press a key, read fps (P) and what changed (here)".
	quality_label = Label.new()
	quality_label.offset_left = 24.0; quality_label.offset_top = 96.0
	quality_label.add_theme_color_override("font_color", Color(1.0, 0.85, 0.4))
	quality_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.85))
	quality_label.add_theme_constant_override("outline_size", 8)
	quality_label.add_theme_font_size_override("font_size", 56)
	quality_label.visible = show_quality
	cl.add_child(quality_label)

	# Hidden km-jump box (J): type a distance to teleport to for artifact hunting.
	seek_input = LineEdit.new()
	seek_input.placeholder_text = "jump to km…"
	seek_input.alignment = HORIZONTAL_ALIGNMENT_CENTER
	seek_input.anchor_left = 0.5; seek_input.anchor_right = 0.5
	seek_input.offset_left = -90.0; seek_input.offset_right = 90.0
	seek_input.offset_top = 12.0; seek_input.offset_bottom = 42.0
	seek_input.visible = false
	seek_input.text_submitted.connect(_on_seek_submitted)
	cl.add_child(seek_input)


# --- quality / LOD foundation ----------------------------------------------
# SimBin philosophy: defer detail by distance, don't delete it. LOW targets
# ~RTX 2070-class (the Quadro RTX 4000); HIGH is for 4070+/M-series Max.
# ride_sim picks a tier via RIDESIM_WORLD_QUALITY; in-world keys 1/2/3 switch
# live, H/Y/T/B/9/0 nudge single levers — profile on the target GPU watching P.
func _init_quality() -> void:
	var q := OS.get_environment("RIDESIM_WORLD_QUALITY").to_lower()
	if q == "low" or q == "medium" or q == "high":
		_set_preset(q)
	# else: no env → keep the member-default ("high"-ish current look)
	_apply_quality()
	_update_quality_label()


# Set the q_* state vars for a tier (does NOT push to the scene — caller applies).
func _set_preset(name: String) -> void:
	match name:
		"low":      # half-res render (4K→~1080p+FSR), no shadows, near cull — Quadro-friendly
			q_preset = "low"
			q_shadows = false; q_tree_shadows = false; q_trees = true; q_glow = false
			q_far = 1800.0; q_shadow_dist = 120.0; q_fog = 0.0006; q_msaa = 0
			q_render_scale = 0.5
		"medium":   # 75% render, cheap shadows (terrain/road only, not the 51k trees)
			q_preset = "medium"
			q_shadows = true; q_tree_shadows = false; q_trees = true; q_glow = true
			q_far = 4000.0; q_shadow_dist = 150.0; q_fog = 0.00035; q_msaa = 2
			q_render_scale = 0.75
		_:          # "high": native res, everything on, far draw — strong GPUs only
			q_preset = "high"
			q_shadows = true; q_tree_shadows = true; q_trees = true; q_glow = true
			q_far = 12000.0; q_shadow_dist = 300.0; q_fog = 0.00025; q_msaa = 4
			q_render_scale = 1.0


# Push the current q_* state to the live scene. Safe to call anytime; all
# handles are null-guarded (a layer may be off, so its MultiMesh won't exist).
func _apply_quality() -> void:
	if sun != null:
		sun.shadow_enabled = q_shadows
		sun.directional_shadow_max_distance = q_shadow_dist
	if env != null:
		env.glow_enabled = q_glow
		env.fog_density = q_fog
	if cam != null:
		cam.far = q_far
	var cs := GeometryInstance3D.SHADOW_CASTING_SETTING_ON if q_tree_shadows \
		else GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	if tree_mmi != null:
		tree_mmi.visible = q_trees
		tree_mmi.cast_shadow = cs
	if pole_mmi != null:
		pole_mmi.cast_shadow = cs
	var vp := get_viewport()
	if vp != null:
		if q_msaa >= 4:
			vp.msaa_3d = Viewport.MSAA_4X
		elif q_msaa >= 2:
			vp.msaa_3d = Viewport.MSAA_2X
		else:
			vp.msaa_3d = Viewport.MSAA_DISABLED
		# Render the 3D below native then FSR-upscale — the dominant 4K fill-rate
		# lever (0.5 = quarter the pixels). UI/overlays stay at native res.
		vp.scaling_3d_mode = Viewport.SCALING_3D_MODE_FSR
		vp.scaling_3d_scale = q_render_scale
	DisplayServer.window_set_vsync_mode(
		DisplayServer.VSYNC_ENABLED if q_vsync else DisplayServer.VSYNC_DISABLED)


func _update_quality_label() -> void:
	if quality_label == null:
		return
	quality_label.text = "Q:%s  render:%d%%  vsync:%s  sun:%s  treeShadow:%s  trees:%s  glow:%s  far:%dm  msaa:%dx" % [
		q_preset, int(round(q_render_scale * 100.0)),
		"on" if q_vsync else "off",
		"on" if q_shadows else "off",
		"on" if q_tree_shadows else "off",
		"on" if q_trees else "off",
		"on" if q_glow else "off",
		int(q_far), q_msaa]


# Any in-world quality keypress routes here: reveal the readout, apply, refresh.
func _quality_key() -> void:
	show_quality = true
	if quality_label != null:
		quality_label.visible = true
	_apply_quality()
	_update_quality_label()


# Mouse-driven World Detail panel — checkboxes + sliders for the quality levers,
# so tuning is all clicking (watch fps on the P overlay). A "⚙ Detail" button
# toggles it. Large fonts via a Theme so it's legible at 4K.
func _build_detail_panel() -> void:
	var layer := CanvasLayer.new()
	layer.layer = 60
	add_child(layer)

	var th := Theme.new()
	th.default_font_size = 40

	var btn := Button.new()
	btn.text = "⚙ Detail"
	btn.theme = th
	btn.anchor_left = 1.0; btn.anchor_right = 1.0
	btn.offset_left = -260.0; btn.offset_top = 16.0
	btn.offset_right = -24.0; btn.offset_bottom = 88.0
	btn.pressed.connect(_on_detail_toggle)
	layer.add_child(btn)

	detail_panel = PanelContainer.new()
	detail_panel.theme = th
	detail_panel.anchor_left = 1.0; detail_panel.anchor_right = 1.0
	detail_panel.offset_left = -600.0; detail_panel.offset_top = 104.0
	detail_panel.offset_right = -24.0
	detail_panel.visible = false
	layer.add_child(detail_panel)

	var mc := MarginContainer.new()
	for side in ["left", "right", "top", "bottom"]:
		mc.add_theme_constant_override("margin_" + side, 20)
	detail_panel.add_child(mc)
	var vb := VBoxContainer.new()
	vb.add_theme_constant_override("separation", 14)
	mc.add_child(vb)

	var title := Label.new()
	title.text = "World Detail"
	vb.add_child(title)

	var hb := HBoxContainer.new()
	hb.add_theme_constant_override("separation", 10)
	vb.add_child(hb)
	for p in ["low", "medium", "high"]:
		var pb := Button.new()
		pb.text = p.capitalize()
		pb.custom_minimum_size = Vector2(160, 72)
		pb.pressed.connect(_on_preset_pressed.bind(p))
		hb.add_child(pb)

	_ui_render_lbl = Label.new()
	_ui_render = _add_slider(vb, _ui_render_lbl, 0.4, 1.0, 0.05, q_render_scale)
	_ui_render.value_changed.connect(_on_slider.bind("render"))
	_ui_far_lbl = Label.new()
	_ui_far = _add_slider(vb, _ui_far_lbl, 500.0, 16000.0, 500.0, q_far)
	_ui_far.value_changed.connect(_on_slider.bind("far"))

	_ui_vsync = _add_check(vb, "VSync (cap to refresh)", q_vsync, "vsync")
	_ui_sun = _add_check(vb, "Sun shadows", q_shadows, "sun")
	_ui_treeshadow = _add_check(vb, "Tree / pole shadows", q_tree_shadows, "treeshadow")
	_ui_trees = _add_check(vb, "Trees", q_trees, "trees")
	_ui_glow = _add_check(vb, "Bloom / glow", q_glow, "glow")
	_ui_centerline = _add_check(vb, "Centerline stripe", show_centerline, "centerline")
	_ui_perf = _add_check(vb, "FPS / perf readout", show_perf, "perf")
	_ui_cam = _add_check(vb, "View controls", show_cam, "cam")

	# --- Peloton pacing: a CONTINUOUS ability dial (the category presets from ride_sim
	# are just detents on it) + companion mode. Live — no respawn needed, the capability
	# pace reads these every frame.
	var ptitle := Label.new()
	ptitle.text = "Peloton"
	vb.add_child(ptitle)
	_ui_wkg_lbl = Label.new()
	_ui_wkg = _add_slider(vb, _ui_wkg_lbl, 0.6, 6.0, 0.1, peloton_wkg)
	_ui_wkg.value_changed.connect(_on_slider.bind("wkg"))
	_ui_companion = _add_check(vb, "Companion pace (match my effort)", peloton_companion, "companion")
	_ui_leaderboard = _add_check(vb, "Pack leaderboard", show_leaderboard, "leaderboard")
	_ui_cfactor_lbl = Label.new()
	_ui_cfactor = _add_slider(vb, _ui_cfactor_lbl, 0.85, 1.15, 0.01, peloton_companion_factor)
	_ui_cfactor.value_changed.connect(_on_slider.bind("cfactor"))
	_update_slider_labels()


func _add_slider(parent: VBoxContainer, lbl: Label, lo: float, hi: float, step: float, val: float) -> HSlider:
	parent.add_child(lbl)
	var s := HSlider.new()
	s.min_value = lo; s.max_value = hi; s.step = step; s.value = val
	s.custom_minimum_size = Vector2(520, 52)
	parent.add_child(s)
	return s


func _add_check(parent: VBoxContainer, label_text: String, on: bool, which: String) -> CheckBox:
	var c := CheckBox.new()
	c.text = label_text
	c.button_pressed = on
	c.toggled.connect(_on_toggle.bind(which))
	parent.add_child(c)
	return c


func _on_detail_toggle() -> void:
	detail_open = not detail_open
	detail_panel.visible = detail_open


func _on_preset_pressed(p: String) -> void:
	_set_preset(p)
	_apply_quality()
	_update_quality_label()
	_sync_panel_controls()


func _on_slider(v: float, which: String) -> void:
	if _syncing:
		return
	if which == "render":
		q_render_scale = v
	elif which == "far":
		q_far = v
	elif which == "wkg":
		peloton_wkg = v
		_update_slider_labels()
		return   # pacing dial: no render-quality re-apply needed
	elif which == "cfactor":
		peloton_companion_factor = v
		_update_slider_labels()
		return
	_apply_quality()
	_update_slider_labels()
	_update_quality_label()


func _on_toggle(on: bool, which: String) -> void:
	if _syncing:
		return
	match which:
		"vsync": q_vsync = on
		"companion":
			peloton_companion = on
			return   # pacing toggle: no render-quality re-apply
		"leaderboard":
			show_leaderboard = on
			if _lb_panel != null:
				_lb_panel.visible = on and not peloton.is_empty()
			return
		"sun": q_shadows = on
		"treeshadow": q_tree_shadows = on
		"trees": q_trees = on
		"glow": q_glow = on
		"centerline":
			show_centerline = on
			if centerline_mi != null:
				centerline_mi.visible = on
			return   # not a quality setting; nothing else to re-apply
		"perf":
			show_perf = on
			if perf_label != null:
				perf_label.visible = show_perf
			return
		"cam":
			show_cam = on
			if on:
				_ride_t = 0.0   # re-show the controls for another fade interval
			return
	_apply_quality()
	_update_quality_label()


func _update_slider_labels() -> void:
	if _ui_render_lbl != null:
		_ui_render_lbl.text = "Render scale: %d%%" % int(round(q_render_scale * 100.0))
	if _ui_far_lbl != null:
		_ui_far_lbl.text = "Draw distance: %d m" % int(q_far)
	if _ui_wkg_lbl != null:
		# show the flat speed this dial position buys, so the number means something
		var flat := _speed_from_power(peloton_wkg * peloton_mass_kg, 0.0)
		_ui_wkg_lbl.text = "Pack ability: %.1f W/kg (~%d km/h flat)" % [peloton_wkg, int(round(flat * 3.6))]
	if _ui_cfactor_lbl != null:
		_ui_cfactor_lbl.text = "Companion factor: %.2f× my effort" % peloton_companion_factor


# Push current q_* state back into the widgets (after a preset button), without
# re-firing their change handlers (guarded by _syncing).
func _sync_panel_controls() -> void:
	_syncing = true
	if _ui_render != null: _ui_render.value = q_render_scale
	if _ui_far != null: _ui_far.value = q_far
	if _ui_vsync != null: _ui_vsync.button_pressed = q_vsync
	if _ui_sun != null: _ui_sun.button_pressed = q_shadows
	if _ui_treeshadow != null: _ui_treeshadow.button_pressed = q_tree_shadows
	if _ui_trees != null: _ui_trees.button_pressed = q_trees
	if _ui_glow != null: _ui_glow.button_pressed = q_glow
	if _ui_centerline != null: _ui_centerline.button_pressed = show_centerline
	if _ui_perf != null: _ui_perf.button_pressed = show_perf
	if _ui_cam != null: _ui_cam.button_pressed = show_cam
	_syncing = false
	_update_slider_labels()


func _build_minimap() -> void:
	minimap_layer = CanvasLayer.new()
	add_child(minimap_layer)
	minimap = Minimap.new()
	var sz := float(minimap_size)
	minimap.anchor_left = 1.0; minimap.anchor_right = 1.0
	minimap.anchor_top = 0.0; minimap.anchor_bottom = 0.0
	minimap.offset_left = -sz - 12.0; minimap.offset_top = 12.0
	minimap.offset_right = -12.0; minimap.offset_bottom = sz + 12.0
	minimap_layer.add_child(minimap)
	var route2 := PackedVector2Array()
	for p in pts:
		route2.append(Vector2(-float(p.x), float(p.z)))   # world x=-East; map shows true East-right
	minimap.setup(route2, Vector2(sz, sz))
	minimap_layer.visible = show_minimap


# Pack leaderboard: rank + condensed standings (top 10, your neighbours, the lantern
# rouge) at the top-left, refreshed at 2 Hz by _update_leaderboard. Hidden when no
# peloton is loaded; toggled from the Detail panel.
func _build_leaderboard() -> void:
	var layer := CanvasLayer.new()
	layer.layer = 55
	add_child(layer)
	_lb_panel = PanelContainer.new()
	_lb_panel.offset_left = 24.0
	_lb_panel.offset_top = 230.0   # below the (optional) 2-line perf readout
	_lb_panel.self_modulate = Color(1, 1, 1, 0.75)
	_lb_panel.visible = false
	layer.add_child(_lb_panel)
	var mc := MarginContainer.new()
	for side in ["left", "right", "top", "bottom"]:
		mc.add_theme_constant_override("margin_" + side, 14)
	_lb_panel.add_child(mc)
	_lb_label = Label.new()
	_lb_label.add_theme_font_size_override("font_size", 42)
	_lb_label.add_theme_color_override("font_color", Color(0.95, 0.95, 0.98))
	mc.add_child(_lb_label)


# Standings from the bounded-station positions (prev_pos_d is each rider's true rendered
# along-route distance this frame). Also pushes the pack's dots to the in-world minimap.
func _update_leaderboard(d: float) -> void:
	var have := not peloton.is_empty()
	if _lb_panel != null:
		_lb_panel.visible = show_leaderboard and have
	if not have:
		if minimap != null:
			minimap.set_pack(PackedVector2Array())
		return
	var rows := []                     # [rendered distance, bib]
	var pack_pts := PackedVector2Array()
	for r in peloton:
		var node := r["node"] as Node3D
		rows.append([float(r.get("prev_pos_d", 0.0)), int(r.get("bib", 0))])
		if node.visible:
			var p := node.global_position
			pack_pts.append(Vector2(-p.x, p.z))   # same axis flip as the route (x=-East)
	if minimap != null:
		minimap.set_pack(pack_pts)
	if _lb_label == null or not show_leaderboard:
		return
	rows.sort_custom(func(a, b): return float(a[0]) > float(b[0]))
	var merged := []                   # standings with the player inserted: [label, dist]
	for row in rows:
		merged.append(["#%d" % int(row[1]), float(row[0])])
	var rank := 1
	for row in rows:
		if float(row[0]) > d:
			rank += 1
	merged.insert(rank - 1, ["YOU", d])
	var lead: float = merged[0][1]
	# condensed selection: top 10 + (player ± 1) + last, with … where ranks are skipped
	var want := {}
	for i in mini(10, merged.size()):
		want[i] = true
	for i in [rank - 2, rank - 1, rank]:
		if i >= 0 and i < merged.size():
			want[i] = true
	want[merged.size() - 1] = true
	var keys := want.keys()
	keys.sort()
	var lines := PackedStringArray()
	lines.append("POS %d / %d" % [rank, merged.size()])
	var prev_i := -2
	for i in keys:
		if i != prev_i + 1 and prev_i >= 0:
			lines.append("  …")
		var gap := lead - float(merged[i][1])
		var mark := "► " if merged[i][0] == "YOU" else "  "
		lines.append("%s%2d. %-4s %s" % [mark, i + 1, merged[i][0],
				"—" if i == 0 else "+%dm" % int(round(gap))])
		prev_i = i
	_lb_label.text = "\n".join(lines)


# --- rider avatar -----------------------------------------------------------

# Hero (player) avatar model. Defaults to the male bike; override with the
# RIDESIM_RIDER_GLB env var (e.g. res://assets/female_opt.glb) to swap the rider
# without a rebuild. Peloton clones + ghost use the colored male base (male_opt.glb).
func _rider_glb_path() -> String:
	var p := OS.get_environment("RIDESIM_RIDER_GLB")
	return p if p != "" else "res://assets/male_opt.glb"


func _build_avatar() -> void:
	# Drop in res://assets/rider.glb (e.g. your decimated SolidWorks export) and it
	# is used automatically; otherwise build a stylized emissive placeholder. Ghost
	# is the same idea in a contrasting color (its own ghost.glb if present).
	if show_avatar:
		var rider_glb := _rider_glb_path()
		avatar = _spawn_avatar(rider_glb, Color(0.15, 0.9, 1.0))
		_avatar_rig = _collect_rig(avatar)
		_avatar_legs = _setup_legs(avatar)
		if OS.has_environment("RIDESIM_AVATAR_DEBUG"):
			var meshes := avatar.find_children("*", "MeshInstance3D", true, false)
			print("[AVDBG] avatar built  path=", rider_glb, "  glb_exists=", ResourceLoader.exists(rider_glb),
				"  mesh_children=", meshes.size(), "  scale=", avatar.scale, "  visible=", avatar.visible,
				"  wheels=", _avatar_rig.wheels.size(), " cranks=", _avatar_rig.cranks.size(),
				" cassettes=", _avatar_rig.cassettes.size(), " legs=", "yes" if _avatar_legs != null else "no")
	if show_ghost:
		# ghost rides the colored male base, made translucent so it reads as a ghost
		ghost = _spawn_avatar("res://assets/male_opt.glb", ghost_tint)
		_make_ghostly(ghost)
		_ghost_rig = _collect_rig(ghost)
		_ghost_legs = _setup_legs(ghost)
	_build_peloton()
	_update_avatar(0.0, 0.0)


# Peloton variety: models alternated for a mixed pack, and the material names each
# colored base carries so per-rider tints can find the jersey + frame across both models.
const PELOTON_MODELS := ["res://assets/male_opt.glb", "res://assets/female_opt.glb"]
const JERSEY_MATS := ["kit_jersey", "kit_burgundy"]         # male / female jersey
const FRAME_MATS := ["kit_frame", "candy apple", "kit_red"] # male frame+fork / female frame + fork

# Spawn N extra riders behind the player for the perf stress test. Gated by the
# RIDESIM_PELOTON_N env var (ride_sim / launch wins) or the peloton_count export.
# Each clone reuses male_opt.glb (full rig + leg IK) and gets a stable, deterministic
# distance gap + lateral lane so the pack strings out and fans across the road
# instead of telescoping into one another. No-op (and zero cost) when N <= 0.
func _build_peloton() -> void:
	var n := peloton_count
	var env_n := OS.get_environment("RIDESIM_PELOTON_N")
	if env_n != "":
		n = int(env_n.to_int())
	if n <= 0:
		return
	_apply_peloton_level(OS.get_environment("RIDESIM_PELOTON_LEVEL"))   # FTP/category preset
	var fp := OS.get_environment("RIDESIM_PELOTON_FREE")
	if fp != "":
		peloton_free_pace = (fp == "1" or fp.to_lower() == "true")
	var mix := {"male": 0, "female": 0}
	# Lane count that fits the fan at the chosen gap (odd → a centered lane on the road axis).
	var lane_count := maxi(1, int(floor(2.0 * peloton_fan_half_m / maxf(peloton_lane_gap_m, 0.1))) + 1)
	# Bounded-station layout: riders per lane, the strung field length, and the max drift
	# amplitude that STILL guarantees same-lane riders never come within a bike length
	# (station_gap − 2·drift ≥ bike_len). Player sits ~35% back from the front.
	var per_lane := int(ceil(float(n) / float(lane_count)))
	var field_len := float(per_lane - 1) * peloton_station_gap_m
	# Half the player bubble comes out of the drift budget: worst case is the rider NEAREST
	# the player shifting a full bubble toward its same-lane neighbour ahead (who, at the
	# zone edge, shifts ~0), so min gap = station − 2·Σamp − bubble ≥ bike_len needs
	# Σamp ≤ (station − bike_len − bubble)/2.
	var drift_cap := maxf(0.0, (peloton_station_gap_m - peloton_bike_len_m - peloton_bubble_m) * 0.5)
	for i in range(n):
		# alternate male/female models for a mixed pack, then give each rider a unique
		# deterministic kit via per-instance material tints (golden-angle hue spread so
		# jerseys are evenly spread around the wheel; frames get an offset, muted hue).
		var model_path: String = PELOTON_MODELS[i % PELOTON_MODELS.size()]
		mix["female" if model_path.contains("female") else "male"] += 1
		var node := _spawn_avatar(model_path, Color(0.15, 0.9, 1.0))
		var jhue := fposmod(float(i) * 0.61803398875, 1.0)
		_tint_materials(node, JERSEY_MATS, Color.from_hsv(jhue, 0.62, 0.82))
		_tint_materials(node, FRAME_MATS, Color.from_hsv(fposmod(jhue + 0.42, 1.0), 0.5, 0.42))
		# Fanned across the road into 5 lanes so neighbours never share one. Each rider
		# gets a "home" slot = a signed offset from the player (+ ahead, − behind),
		# spread ~30% ahead / 70% behind so the player rides toward the front. With
		# autonomous speed the riders surge around these slots → overtaking + accordion.
		# PREORDAINED LANE: each rider gets a fixed lane, centered on the road axis. Round-
		# robin (i % lane_count) so same-lane riders are lane_count apart in spawn order →
		# far apart in "home" distance below → they don't share a distance (collision-free).
		# For N ≤ lane_count every rider is unique. lateral = lane center; drift/weave add on.
		var lane_idx := i % lane_count
		var lateral := (float(lane_idx) - float(lane_count - 1) / 2.0) * peloton_lane_gap_m
		lateral = clampf(lateral, -peloton_fan_half_m, peloton_fan_half_m)
		# Bounded-station: fixed slot along this rider's lane (k=0 = front of the lane), the
		# whole field biased so the player sits ~35% back from the front. drift_amp ≤ drift_cap
		# GUARANTEES same-lane riders never converge (see peloton_station_gap_m).
		var k := i / lane_count                          # slot index within the lane (0 = front)
		var home := field_len * 0.35 - float(k) * peloton_station_gap_m
		# TWO-HARMONIC surge: a LONG slow wave (the visible ebb — tens of seconds per cycle at
		# cruise) + a SHORT low-amplitude texture wave (pace jitter). One sine can't give a long
		# period AND a wide speed range AND tight stations — speed swing = amp·TAU/wl, so a long
		# wl needs a huge amp the station budget won't fit. Splitting the budgets works: the slow
		# wave takes most of the EXCURSION, the fast wave most of the SPEED range. Both caps
		# apply to the SUM, so the same two guarantees hold exactly as for one sine:
		#   Σamp ≤ drift_cap                     → same-lane riders never converge
		#   Σ amp·TAU/wl ≤ drift_speed_frac      → speed within pace·(1 ± frac): no stop-and-dart
		var wl_s := peloton_drift_wl_m * (1.0 + peloton_drift_wl_spread * (2.0 * fposmod(float(i) * 0.382, 1.0) - 1.0))
		var wl_f := peloton_drift_tex_wl_m * (1.0 + peloton_drift_wl_spread * (2.0 * fposmod(float(i) * 0.754, 1.0) - 1.0))
		var amp_s := minf(drift_cap * 0.75 * lerpf(0.7, 1.0, fposmod(float(i) * 0.618, 1.0)),
				0.45 * peloton_drift_speed_frac * wl_s / TAU)
		var amp_f := minf(drift_cap - amp_s,
				0.55 * peloton_drift_speed_frac * wl_f / TAU)
		peloton.append({
			"node": node,
			"rig": _collect_rig(node),
			"legs": _setup_legs(node),
			"lateral": lateral,
			"lane_idx": lane_idx,                         # fixed lane
			"bib": i + 1,                                 # race number (leaderboard)
			"home": home,                                 # fixed station along the lane (m from pack ref)
			"bank": 0.0, "race": 0.0, "coast": 0.0,       # eased lean / racing drift / coast
			# BOUNDED two-harmonic fore/aft surge (keyed on pack phase). Slow-wave phase =
			# row phase (propagating accordion) + a ±1 rad per-rider jitter; texture phase free.
			"d_amp_s": amp_s, "d_wl_s": wl_s,
			"d_ph_s": float(k) * peloton_drift_row_phase + (fposmod(float(i) * 0.71, 1.0) - 0.5) * 2.0,
			"d_amp_f": amp_f, "d_wl_f": wl_f,
			"d_ph_f": fposmod(float(i) * 2.39996323, TAU),
			"prev_pos_d": 0.0,                            # last along-route pos (→ rider speed for lean/cadence)
			# deterministic per-rider character (golden-angle spread, no RNG → identical
			# pack every run): crank start phase, preferred cadence, weave timing.
			"crank_ph": fposmod(float(i) * 2.39996323, TAU),
			"pref_rpm": peloton_cadence_rpm + peloton_cadence_spread * sin(float(i) * 1.7),
			"weave_off": fposmod(float(i) * 1.111, TAU),
			# per-rider weave WAVELENGTH in metres (distance-keyed, not seconds) → pace-independent
			"weave_wl": peloton_weave_wl_m * (1.0 + peloton_weave_wl_spread * (2.0 * fposmod(float(i) * 0.618, 1.0) - 1.0)),
		})
	print("[PELOTON] spawned %d riders (%dM/%dF, varied kits)  level=%s  race=%.1fm  free_pace=%s"
		% [n, mix["male"], mix["female"], _peloton_level, peloton_race_line_m, "ON" if peloton_free_pace else "off"])
	print("          bounded-station: %d lanes × ~%d/lane  lane_gap=%.2fm  station_gap=%.1fm  drift≤±%.1fm (slow %.0fm + tex %.0fm, ±%.0f%% pace)  weave=±%.2fm/%.0fm"
		% [lane_count, per_lane, peloton_lane_gap_m, peloton_station_gap_m, drift_cap, peloton_drift_wl_m,
		   peloton_drift_tex_wl_m, peloton_drift_speed_frac * 100.0, peloton_wander_m, peloton_weave_wl_m])


# Map ride_sim's "Peloton level" (FTP/category) to the bounded-station aggression: higher
# category = TIGHTER lanes + TIGHTER fore/aft stations, CLEANER lines (less wander, a
# punchier/shorter slow-surge wave), and sharper apexes. Recreational = strung out, loose,
# wandery, with the longest laziest ebb. The
# station-gap invariant (drift auto-capped at (station_gap−bike_len)/2) means same-lane
# riders never converge at ANY level. Blank/unknown leaves the exported defaults. Called
# before seeding so the spawn layout + per-rider drift take it.
func _apply_peloton_level(lvl: String) -> void:
	_peloton_level = lvl if lvl != "" else "default"
	# W/kg detents are on peloton_mass_kg TOTAL (rider+bike) with the bunch's effective
	# CdA — they're dial positions that land the flat speeds below, not physiology.
	# drift_speed_frac falls sharply with category: wide shots of a pro tour peloton look
	# CALM — the riders' speed variance is tiny even though the pace is brutal. A rec
	# group ride visibly concertinas. (Read at spawn: it caps the drift amplitudes.)
	match lvl.to_lower().replace(" ", "").replace("-", ""):
		"recreational", "rec":
			peloton_wkg = 1.0           # ~24 km/h flat — honestly recreational now
			peloton_race_line_m = 1.4;  peloton_station_gap_m = 14.0; peloton_drift_wl_m = 320.0
			peloton_lane_gap_m = 1.40; peloton_wander_m = 0.35; peloton_weave_wl_m = 45.0
			peloton_drift_speed_frac = 0.20
		"cat3", "cat3amateur":
			peloton_wkg = 2.2           # ~32 km/h flat
			peloton_race_line_m = 1.7;  peloton_station_gap_m = 12.5; peloton_drift_wl_m = 300.0
			peloton_lane_gap_m = 1.25; peloton_wander_m = 0.28; peloton_weave_wl_m = 55.0
			peloton_drift_speed_frac = 0.15
		"cat2", "cat2amateur":
			peloton_wkg = 2.8           # ~36 km/h flat
			peloton_race_line_m = 1.9;  peloton_station_gap_m = 11.0; peloton_drift_wl_m = 280.0
			peloton_lane_gap_m = 1.15; peloton_wander_m = 0.22; peloton_weave_wl_m = 62.0
			peloton_drift_speed_frac = 0.12
		"cat1", "cat1elite":
			peloton_wkg = 3.8           # ~40 km/h flat
			peloton_race_line_m = 2.0;  peloton_station_gap_m = 9.5;  peloton_drift_wl_m = 260.0
			peloton_lane_gap_m = 1.05; peloton_wander_m = 0.16; peloton_weave_wl_m = 70.0
			peloton_drift_speed_frac = 0.09
		"pro", "protour", "worldtour":
			peloton_wkg = 5.0           # ~44 km/h flat
			peloton_race_line_m = 2.2;  peloton_station_gap_m = 8.0;  peloton_drift_wl_m = 240.0
			peloton_lane_gap_m = 0.95; peloton_wander_m = 0.12; peloton_weave_wl_m = 80.0
			peloton_drift_speed_frac = 0.06
		_:
			pass   # "" / unknown → keep the exported defaults


# Build the rung-2 leg IK for an avatar (call at spawn, nodes still at rest pose).
# Returns null if the model has no recognizable leg segments (e.g. the placeholder),
# so the rest of the pipeline stays a no-op on un-rigged avatars.
func _setup_legs(root: Node3D) -> LegRig:
	if root == null:
		return null
	var rig := LegRig.new()
	return rig if rig.setup(root) else null


# Walk the avatar's node tree and cache any Node3D whose name contains the wheel/
# crank match strings, along with its rest basis (so we rotate RELATIVE to however
# the part was modeled). Pure-distance spin is applied later in _spin_rig().
func _collect_rig(root: Node3D) -> Dictionary:
	var rig := {"wheels": [], "cranks": [], "cassettes": []}
	if root == null:
		return rig
	var wlc := wheel_name_match.to_lower()
	var clc := crank_name_match.to_lower()
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		for c in n.get_children():
			stack.push_back(c)
		if n is Node3D:
			var nm := String(n.name).to_lower()
			if nm.begins_with("cs-") or nm.contains("cassette"):
				# rear cog cluster: spins with the crank (lockstep), about its own
				# origin which sits on the rear-hub axle (parent-X).
				rig.cassettes.append({"node": n, "rest": (n as Node3D).transform.basis})
			elif wlc != "" and nm.contains(wlc):
				rig.wheels.append({"node": n, "rest": (n as Node3D).transform.basis})
			elif clc != "" and nm.contains(clc):
				rig.cranks.append({"node": n, "rest": (n as Node3D).transform.basis})
	return rig


# Spin a rig's wheels/cranks to match distance d (meters). Absolute (not
# incremental) so there's no drift; paused = still, scrub-back = reverse.
func _spin_wheels(rig: Dictionary, d: float) -> void:
	var circ := PI * wheel_diameter_m
	if circ <= 0.001:
		return
	var wb := Basis(rig_spin_axis.normalized(), fposmod(d / circ, 1.0) * TAU * wheel_spin_sign)
	for w in rig.wheels:
		(w.node as Node3D).transform.basis = w.rest * wb


# Drive the crank (about its BB origin) and cassette (about the rear-hub axle) to an
# absolute drivetrain angle (radians). The cassette is LOCKED to the crank — same
# angle — so it spins at pedaling speed and freezes when coasting. The angle is fed
# from integrated cadence for the live rider, or development (distance) otherwise.
func _drive_crank(rig: Dictionary, angle: float) -> void:
	var cb := Basis(rig_spin_axis.normalized(), angle * crank_spin_sign)
	for c in rig.cranks:
		(c.node as Node3D).transform.basis = c.rest * cb
	var sb := Basis(Vector3.RIGHT, angle * cassette_spin_sign)
	for s in rig.cassettes:
		(s.node as Node3D).transform.basis = sb * s.rest


# Make a real-glb avatar read as a ghost: translucent, slightly cool-tinted. Per-surface
# override so it doesn't touch the shared loaded materials (player/peloton stay solid).
func _make_ghostly(root: Node3D) -> void:
	if root == null:
		return
	for mi in root.find_children("*", "MeshInstance3D", true, false):
		var m := mi as MeshInstance3D
		if m.mesh == null:
			continue
		for s in m.mesh.get_surface_count():
			var src := m.get_active_material(s)
			var gm: StandardMaterial3D
			if src is StandardMaterial3D:
				gm = (src as StandardMaterial3D).duplicate()
			else:
				gm = StandardMaterial3D.new()
			gm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
			var c := gm.albedo_color
			gm.albedo_color = Color(lerpf(c.r, 0.6, 0.4), lerpf(c.g, 0.7, 0.4), lerpf(c.b, 0.9, 0.4), 0.4)
			m.set_surface_override_material(s, gm)


# Per-instance recolor: override any surface whose material name matches a token with a
# tinted DUPLICATE (so it doesn't touch the shared loaded material → other riders keep
# their own colors). Used to give each peloton rider a unique jersey/frame.
func _tint_materials(root: Node3D, tokens: Array, color: Color) -> void:
	if root == null:
		return
	for mi in root.find_children("*", "MeshInstance3D", true, false):
		var m := mi as MeshInstance3D
		if m.mesh == null:
			continue
		for s in m.mesh.get_surface_count():
			var src := m.get_active_material(s)
			if src == null:
				continue
			var nm := src.resource_name.to_lower()
			for t in tokens:
				if nm.contains(t):
					var nmat: StandardMaterial3D
					if src is StandardMaterial3D:
						nmat = (src as StandardMaterial3D).duplicate()
					else:
						nmat = StandardMaterial3D.new()
					nmat.albedo_color = color
					m.set_surface_override_material(s, nmat)
					break


func _spawn_avatar(glb_path: String, col: Color) -> Node3D:
	var node: Node3D
	if ResourceLoader.exists(glb_path):
		node = (load(glb_path) as PackedScene).instantiate()
	else:
		node = _make_tron_avatar(col)
	node.scale = Vector3.ONE * avatar_scale
	add_child(node)
	return node


func _make_tron_avatar(col: Color) -> Node3D:
	var root := Node3D.new()
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(col.r * 0.3, col.g * 0.3, col.b * 0.3)
	mat.emission_enabled = true
	mat.emission = col
	mat.emission_energy_multiplier = 2.5
	mat.metallic = 0.2
	mat.roughness = 0.3
	# glowing ground pad — the part that reads from a high chase view
	var pad := MeshInstance3D.new()
	var pad_mesh := CylinderMesh.new()
	pad_mesh.top_radius = 0.9; pad_mesh.bottom_radius = 0.9; pad_mesh.height = 0.08
	pad.mesh = pad_mesh
	pad.material_override = mat
	pad.position = Vector3(0, 0.06, 0)
	root.add_child(pad)
	# forward-leaning torso (local -Z is travel direction after look_at)
	var body := MeshInstance3D.new()
	var body_mesh := CapsuleMesh.new()
	body_mesh.radius = 0.22; body_mesh.height = 1.1
	body.mesh = body_mesh
	body.material_override = mat
	body.rotation_degrees = Vector3(-50, 0, 0)
	body.position = Vector3(0, 0.7, -0.1)
	root.add_child(body)
	# head
	var head := MeshInstance3D.new()
	var head_mesh := SphereMesh.new()
	head_mesh.radius = 0.16; head_mesh.height = 0.32
	head.mesh = head_mesh
	head.material_override = mat
	head.position = Vector3(0, 1.15, -0.5)
	root.add_child(head)
	return root


# Player's own racing line + lean, computed ONCE before the camera so the chase/
# cockpit view can track the SAME lateral offset (otherwise the rider appears to
# slide off the centerline the camera is locked to). Also measures the ride speed
# (shared with the peloton + ghost) and advances the moving-time clocks.
func _update_player_line(d: float, delta: float) -> void:
	# Measured speed = how fast the render distance is actually advancing (0 when
	# paused/held at the line). On a teleport frame (view_dist snapped to a far dist —
	# first packet / scrub) the apparent speed is garbage, so HOLD the last value rather
	# than inject the 50 m/s clamp (which would poison the free-pace cruise for ~40 s).
	if not _view_snapped:
		# A view_dist discontinuity SMALLER than the 50 m snap threshold (e.g. the seek
		# settling, or a lag hitch) still yields an absurd 1-frame speed that clamps to
		# 50 m/s (180 km/h). If the pack seeds its riders at that, the bunch rockets off for
		# ~1.5 s at the start. So reject any raw speed above a sane ceiling — hold the last
		# value — since no demo (<=30) or real ride approaches it; only a position jump does.
		var raw := (d - _last_d) / maxf(delta, 0.0001)
		if raw <= MEAS_SPEED_GLITCH_MAX:
			_meas_speed = clampf(raw, 0.0, 50.0)
	_last_d = d
	if _meas_speed > 0.15 and _sim_active:
		_ride_t += delta
		_pelo_t += delta          # weave/surge clock (peloton + ghost), frozen when stopped/paused
	_update_cam_fade(delta)
	# racing line: same apex-chasing line as the pack, gentler (the camera follows it)
	var lat_t := _race_line_lat(d, player_race_line_m)
	_player_lat = lat_t if player_race_tau <= 0.0 \
			else lerpf(_player_lat, lat_t, 1.0 - exp(-delta / player_race_tau))
	# steer the avatar into the line's slope (crab fix; same idea as the pack riders)
	_player_yaw = -atan((_race_line_lat(d + 2.0, player_race_line_m) - lat_t) / 2.0)
	# lean the player's own bike (horizon stays level — camera never rolls, no vertigo)
	if peloton_bank_hero:
		var ht := _bank_target_rad(d, _meas_speed)
		_hero_bank = ht if peloton_bank_tau <= 0.0 \
				else lerpf(_hero_bank, ht, 1.0 - exp(-delta / peloton_bank_tau))
	else:
		_hero_bank = 0.0
	# world-space lateral offset for the cameras (perpendicular to the travel heading,
	# same basis _place_on_route_banked uses, so avatar + camera shift identically).
	var here := _pos_xz_at(d)
	var fwd := _pos_xz_at(d + 4.0)
	var horiz := Vector3(fwd.x - here.x, 0.0, fwd.z - here.z)
	_player_off = horiz.normalized().cross(Vector3.UP).normalized() * _player_lat \
			if horiz.length() > 0.01 else Vector3.ZERO


func _update_avatar(d: float, delta: float) -> void:
	if avatar != null:
		# Place on the player's racing line + lean (computed in _update_player_line).
		# Pedal rock: live rides use the trainer's real watts + cadence (you stomp, your
		# avatar rocks); the standalone demo has neither, so imply both from speed.
		var rw := cur_power
		var rc := cur_cadence
		if not live:
			rc = _meas_speed / maxf(crank_dev_m, 0.001) * 60.0
			rw = _power_for_speed(_meas_speed, _grade_at(d))
		var rock := _pedal_rock(crank_phase, rw, rc)
		_place_on_route_banked(avatar, d, _player_lat, _hero_bank + rock.x, _player_yaw + rock.y)
		_spin_wheels(_avatar_rig, d)
		_drive_crank(_avatar_rig, crank_phase)
		if _avatar_legs != null:
			_avatar_legs.pose()
	_update_ghost(delta)


# The ghost paces by the strict TCX distance ride_sim sends (or a synthetic gap in
# the standalone demo), but otherwise behaves like a peloton rider — leaning,
# tracking the racing line, weaving slowly — and side-steps the player so it passes
# BESIDE instead of through. Hidden unless a live ghost exists or auto_ghost (G).
func _update_ghost(delta: float) -> void:
	if ghost == null:
		return
	var show := ghost_is_live or auto_ghost
	ghost.visible = show
	if not show:
		return
	if _ghost_state.is_empty():
		_ghost_state = {"bank": 0.0, "race": 0.0, "coast": 0.0, "crank_ph": 0.0}
	var st: Dictionary = _ghost_state
	var gd := clampf(view_ghost_dist, 0.0, route_len) if ghost_is_live \
			else fposmod(view_dist + ghost_gap_m, route_len)
	var gspd := ghost_live_speed if ghost_is_live else _meas_speed

	# lean from the ghost's OWN speed (eased), like a pack rider
	var bank := _bank_target_rad(gd, gspd)
	bank = bank if peloton_bank_tau <= 0.0 \
			else lerpf(float(st["bank"]), bank, 1.0 - exp(-delta / peloton_bank_tau))
	st["bank"] = bank

	# racing line + slow weave (same character as the pack); side-step to pass the
	# player when their distances overlap, blended by proximity so it's smooth.
	var race_t := _race_line_lat(gd, peloton_race_line_m)
	var race := race_t if peloton_race_tau <= 0.0 \
			else lerpf(float(st["race"]), race_t, 1.0 - exp(-delta / peloton_race_tau))
	st["race"] = race
	var weave := peloton_wander_m * sin(_pelo_t * TAU / 19.0 + 1.7)
	var own_lat := ghost_lane_m + race + weave
	var prox := clampf(1.0 - absf(gd - view_dist) / maxf(ghost_pass_zone_m, 0.1), 0.0, 1.0)
	var side := 1.0 if ghost_pass_side >= 0.0 else -1.0
	var pass_lat := _player_lat + side * ghost_pass_m   # guaranteed clear of the player
	var lateral := lerpf(own_lat, pass_lat, prox)
	# crab fix: race-line slope (per metre) + the time-based weave's rate converted to a
	# slope by the ghost's own speed (dlat/ds = (dlat/dt)/v; ~straight when crawling).
	var drace := (_race_line_lat(gd + 2.0, peloton_race_line_m) - race_t) / 2.0
	var dweave := peloton_wander_m * TAU / 19.0 * cos(_pelo_t * TAU / 19.0 + 1.7) \
			/ maxf(gspd, 1.0)
	var yaw := -atan(drace + dweave)
	# pedal rock like a pack rider (implied watts at the ghost's own speed; last frame's
	# crank angle — see the peloton note), faded by its coast blend
	var rock := _pedal_rock(float(st["crank_ph"]),
			_power_for_speed(gspd, _grade_at(gd)) * (1.0 - float(st["coast"])),
			peloton_cadence_rpm)
	_place_on_route_banked(ghost, gd, lateral, bank + rock.x, yaw + rock.y)

	# drivetrain: wheels by distance; cadence geared into the band; coast pedal-up in bends
	_spin_wheels(_ghost_rig, gd)
	var cadence := clampf(peloton_cadence_rpm, peloton_cadence_min, peloton_cadence_max) \
			* clampf(gspd / 1.5, 0.0, 1.0)
	st["crank_ph"] = float(st["crank_ph"]) + cadence / 60.0 * TAU * delta
	var pedaling := float(st["crank_ph"])
	var coast_thresh := _pedal_strike_rad() - deg_to_rad(coast_margin_deg)
	var want_coast := 1.0 if (peloton_coast_pedal_up and absf(bank) > coast_thresh) else 0.0
	var coast := want_coast if coast_tau <= 0.0 \
			else lerpf(float(st["coast"]), want_coast, 1.0 - exp(-delta / coast_tau))
	st["coast"] = coast
	var crank_ang := pedaling
	if coast > 0.001:
		var inside_left := bank > 0.0
		if coast_flip:
			inside_left = not inside_left
		crank_ang = lerp_angle(pedaling, coast_phase_rad + (PI if inside_left else 0.0), coast)
	_drive_crank(_ghost_rig, crank_ang)
	if _ghost_legs != null:
		_ghost_legs.pose()


# Height of the leveled ridden-road surface at distance d. Assumes the caller just
# did _pos_xz_at(d) so seg_i points at d's segment (read before any look-ahead call
# moves it). Falls back to raw terrain if road heights aren't built. `p` supplies
# the x/z for that fallback.
func _road_y_seg(d: float, p: Vector3) -> float:
	if road_center_y.size() == pts.size() and not road_center_y.is_empty():
		var i := seg_i
		var span := float(pts[i + 1].d) - float(pts[i].d)
		var t := 0.0 if span < 0.001 else clampf((d - float(pts[i].d)) / span, 0.0, 1.0)
		return lerpf(road_center_y[i], road_center_y[i + 1], t) + road_lift
	return _terrain_y(p.x, p.z) + road_lift


# Centered, smoothed horizontal heading at distance d. A forward-only secant (d→d+4)
# rotates the lateral offset ~4 m BEFORE a polyline bend, so a rider's side-position
# shifts out of sync with the visible road corner. A centered ±3 m secant keeps the
# lateral synced to the bend and rotates smoothly through it. Always returns a unit
# vector (FORWARD fallback on a degenerate/zero-length span).
func _heading_at(d: float) -> Vector3:
	var a := _pos_xz_at(d - 3.0)
	var b := _pos_xz_at(d + 3.0)
	var h := Vector3(b.x - a.x, 0.0, b.z - a.z)
	return h.normalized() if h.length() > 0.01 else Vector3.FORWARD


func _place_on_route(node: Node3D, d: float) -> void:
	var here := _pos_xz_at(d)   # sets seg_i to d's segment
	# Ride ON the leveled road surface (not raw terrain) so the rider never buries.
	here.y = _road_y_seg(d, here)
	# Pitch the bike to the local road grade over ~one wheelbase, so BOTH wheels sit
	# on the road on climbs/descents. A level bike floats one wheel and buries the
	# other wherever the grade changes — the "wheels above/below" wander.
	var an := _pos_xz_at(d + 0.6)
	var ay := _road_y_seg(d + 0.6, an)
	var bn := _pos_xz_at(d - 0.6)
	var by := _road_y_seg(d - 0.6, bn)
	node.global_position = here
	var horiz := _heading_at(d)                          # centered yaw heading (horizontal)
	if horiz.length() > 0.01:
		var grade := (ay - by) / 1.2                     # rise over run (2 × 0.6 m)
		node.look_at(here + horiz + Vector3.UP * grade, Vector3.UP)


# Like _place_on_route, but slides the rider into a side lane (perpendicular to the
# heading) and rolls it into a faked corner lean. Used by the peloton so riders fan
# across the road and bank through turns instead of stacking on the centerline.
# yaw_rad steers the bike INTO its own lateral motion (the slope of its lateral path,
# atan(dlat/ds)) — without it a weaving rider translates sideways while pointing
# along-route, which reads as the tires sliding across the tarmac ("crabbing").
func _place_on_route_banked(node: Node3D, d: float, lateral: float, bank_rad: float,
		yaw_rad: float = 0.0) -> void:
	var here := _pos_xz_at(d)
	here.y = _road_y_seg(d, here)
	var an := _pos_xz_at(d + 0.6)
	var ay := _road_y_seg(d + 0.6, an)
	var bn := _pos_xz_at(d - 0.6)
	var by := _road_y_seg(d - 0.6, bn)
	var horiz := _heading_at(d)                          # centered heading → lateral synced to bends
	if horiz.length() <= 0.01:
		node.global_position = here
		return
	# right-hand offset = heading × UP (horizontal); +lateral pushes to one side.
	here += horiz.cross(Vector3.UP).normalized() * lateral
	node.global_position = here
	var grade := (ay - by) / 1.2
	# +yaw about UP turns LEFT (−X for a −Z heading), so steering toward +lateral
	# (right) needs the caller to pass yaw = −atan(dlat/ds).
	var fwd := horiz if absf(yaw_rad) < 0.0001 else horiz.rotated(Vector3.UP, yaw_rad)
	node.look_at(here + fwd + Vector3.UP * grade, Vector3.UP)
	if absf(bank_rad) > 0.0001:
		node.rotate_object_local(Vector3.FORWARD, bank_rad)   # roll about travel axis


# Faked centripetal lean for distance d at the given speed: sample the path's signed
# turn rate (curvature κ, rad/m) over a short span, then θ = atan(v²·κ/g) — the angle
# a real bike leans to balance cornering force. Capped + sign-flippable. Zero on
# straights and when stopped. Not a physics package, just the geometry that reads right.
# Signed path curvature κ (rad/m) at distance d: how fast the heading turns, sampled
# over a short span. + / − = turning one way / the other. Shared by banking and the
# racing line so they always agree on which way (and how hard) the road bends.
func _path_curvature(d: float) -> float:
	var ds := 3.0
	var a := _pos_xz_at(d - ds)
	var b := _pos_xz_at(d)
	var c := _pos_xz_at(d + ds)
	var h1 := Vector3(b.x - a.x, 0.0, b.z - a.z)
	var h2 := Vector3(c.x - b.x, 0.0, c.z - b.z)
	if h1.length() < 0.01 or h2.length() < 0.01:
		return 0.0
	return h1.normalized().signed_angle_to(h2.normalized(), Vector3.UP) / (2.0 * ds)


# Smooth, bounded lane-band confinement: like clampf(x, -band, band) but with a
# continuous derivative, so a corner's apex pull saturating the band can't STEP the
# lateral velocity (the old clamp kink = sudden sideways lurch onset). tanh compresses
# a little inside the band too (~24% at the edge) — the price of smoothness; the
# cross-lane guarantee only needs |result| < band, which tanh satisfies strictly.
func _band_soft(x: float, band: float) -> float:
	if band <= 0.001:
		return 0.0
	return band * tanh(x / band)


# Precompute SMOOTHED signed path curvature along the whole route (2 m steps, ±6 m box).
# _path_curvature reads the raw polyline, so it jumps every time one of its ±3 m sample
# points crosses a vertex — that jag went straight into the racing-line lateral (small
# sideways lurches the steer-into-path yaw can't track: its analytic derivative is
# smooth). Smoothing in DISTANCE (not time) keeps it a pure function of d — deterministic
# and pace-independent — and a LUT lerp is cheaper per query than 3 polyline position
# lookups, which matters at 100 riders × several curvature reads per frame.
func _build_curv_lut() -> void:
	var n := int(ceil(route_len / CURV_LUT_STEP)) + 1
	if n < 2:
		return
	var raw := PackedFloat32Array()
	raw.resize(n)
	for i in n:
		raw[i] = _path_curvature(float(i) * CURV_LUT_STEP)
	_curv_lut.resize(n)
	for i in n:
		var s := 0.0
		var c := 0
		for k in range(maxi(0, i - CURV_LUT_SMOOTH), mini(n - 1, i + CURV_LUT_SMOOTH) + 1):
			s += raw[k]
			c += 1
		_curv_lut[i] = s / float(c)


# Smoothed curvature at d (LUT lerp; raw polyline fallback before the LUT is built).
func _curv_at(d: float) -> float:
	var n := _curv_lut.size()
	if n < 2:
		return _path_curvature(d)
	var f := clampf(d / CURV_LUT_STEP, 0.0, float(n - 1))
	var i := int(f)
	if i >= n - 1:
		return _curv_lut[n - 1]
	return lerpf(_curv_lut[i], _curv_lut[i + 1], f - float(i))


# Racing-line lateral offset (m) at distance d, capped to ±max_m. Samples curvature a
# bit AHEAD (peloton_apex_lookahead_m) so the rider turns in EARLY and chases the apex
# rather than drifting at the corner; ×peloton_bank_sign keeps the drift on the inside
# of the lean. Shared by the player, the pack, and the ghost so all take the same line.
func _race_line_lat(d: float, max_m: float) -> float:
	if max_m == 0.0:
		return 0.0
	return clampf(_curv_at(d + peloton_apex_lookahead_m) * peloton_apex_gain, -1.0, 1.0) \
			* peloton_bank_sign * max_m


func _bank_target_rad(d: float, speed: float) -> float:
	var theta := atan(speed * speed * _curv_at(d) / 9.81)   # lean to balance v²/r
	var cap := deg_to_rad(peloton_bank_max_deg)
	return clampf(theta, -cap, cap) * peloton_bank_sign


# Road slope (fraction, + = uphill) at along-route distance d, from the smoothed ground
# over a ±35 m window. Used ONLY for pacing (capability + companion watts): the wide
# window is deliberate — a ±12 m one hit −45% single-point spikes at hairpins/DEM cliffs
# (bench, stage-19 km 52.4) and the whole pack's pace surged through them. Riders
# anticipate grade anyway; the avatar PITCH uses its own short-window sampling.
func _grade_at(d: float) -> float:
	var ds := 35.0
	return (_smooth_ground(d + ds) - _smooth_ground(d - ds)) / (2.0 * ds)


# Road cycling power at speed v on this grade: P = (½·ρ·CdA·v² + Crr·m·g + m·g·grade)·v.
# Also used INVERTED (below) and to infer the player's implied watts in companion mode —
# using the same model both ways makes its constant errors cancel.
func _power_for_speed(v: float, grade: float) -> float:
	var a := 0.5 * 1.225 * peloton_cda
	var b := peloton_mass_kg * 9.81 * (peloton_crr + grade)
	return a * v * v * v + b * v


# Sustainable speed (m/s) at `watts` on `grade`: Newton-solve the cubic above, starting
# from ABOVE (v=30) — the cubic's positive leading coefficient makes f convex and f'
# positive there, so the iteration is monotone and can't be trapped on descents where
# the linear term goes negative. Floor keeps a wall from track-standing the bunch.
func _speed_from_power(watts: float, grade: float) -> float:
	var a := 0.5 * 1.225 * peloton_cda
	var b := peloton_mass_kg * 9.81 * (peloton_crr + grade)
	var v := 30.0
	for _i in 12:
		var fp := 3.0 * a * v * v + b
		if fp <= 0.0001:
			break
		var step := (a * v * v * v + b * v - watts) / fp
		v -= step
		if absf(step) < 0.005:
			break
	return clampf(v, 2.2, 30.0)


# The pack's sustainable pace (m/s) on this grade, from what its riders can actually
# put out: the W/kg dial (or, in companion mode, the player's rolling implied watts ×
# the companion factor).
func _capability_pace(grade: float) -> float:
	var watts := peloton_wkg * peloton_mass_kg
	if peloton_companion and _companion_watts > 10.0:
		watts = _companion_watts * peloton_companion_factor
	return _speed_from_power(watts, grade)


# Pedal-induced bike rock at this crank angle: returns (roll, yaw) in radians. Amplitude
# scales with the torque proxy watts/cadence against a 250 W @ 90 rpm reference, fades in
# from a standstill, and is zero when coasting (cadence ~0) — so it composes with the
# coast-pedal-up behavior for free. One full left-right cycle per crank revolution; the
# yaw wag leads the roll by a quarter cycle (the bar counter-steer that squares it up).
func _pedal_rock(crank_ph: float, watts: float, cadence: float) -> Vector2:
	if pedal_rock_deg <= 0.001 or cadence < 20.0 or watts <= 1.0:
		return Vector2.ZERO
	var torque := watts / cadence                       # ∝ N·m (W per rpm is fine as a ratio)
	var amp := deg_to_rad(pedal_rock_deg) * clampf(torque / (250.0 / 90.0), 0.0, 1.6) \
			* clampf((cadence - 20.0) / 20.0, 0.0, 1.0)
	return Vector2(amp * sin(crank_ph),
			amp * pedal_rock_yaw_frac * sin(crank_ph + PI * 0.5))


# Lean (rad) at which the inside pedal, at 6 o'clock, would strike the tarmac for this
# frame: tan θ = (BB height − crank) / pedal-outboard. Drives when riders coast pedal-up.
func _pedal_strike_rad() -> float:
	return atan((bb_height_m - crank_len_m) / maxf(pedal_outboard_m, 0.001))


# Per-frame peloton update: string each clone out behind the player, ease its lean,
# and drive its rig + legs exactly like the hero (so the test pays the full cost).
func _update_peloton(d: float, delta: float) -> void:
	if peloton.is_empty():
		return
	# Pack speed + the weave/surge clock are measured/advanced in _update_player_line
	# (shared with the ghost), so a paused/held ride has spd 0 → riders settle and the
	# clock freezes (no side-to-side drift while stopped).
	# SURGE LOW-PASS: _meas_speed is a per-frame finite-diff of the eased position, so it
	# re-exposes the 4 Hz packet-correction + ~1 Hz TCX-trackpoint pulses. Feeding it raw to
	# the shared ref pace made the WHOLE pack breathe fore/aft in ~1 s beats. EMA it into
	# _pack_pace first → smooth base pace; per-rider ability/surge then vary it as intended.
	# The pack RUNS only when the ride does. _sim_active covers the world's own pause;
	# beyond that, HOLD while (a) the world is still waiting for ride_sim's first packet
	# (startup: the free-pace pack used to roll away while the terrain was still building)
	# and (b) ride_sim says the ride is paused (packet "paused": distance freezes but the
	# world's own flag knows nothing — the bunch used to ride on through every pause).
	var pack_run := _sim_active
	if live and _live_paused:
		pack_run = false
	if not live and wait_for_telemetry:
		pack_run = false
	if pack_run:
		_pack_pace = lerpf(_pack_pace, _meas_speed, 1.0 - exp(-delta / maxf(peloton_pace_tau, 0.01)))
	# COMPANION anchor: the player's rolling IMPLIED watts — their in-world speed + grade
	# pushed through the same power model the pack rides by (so the model's constant errors
	# cancel; no power meter or protocol change needed). Only accumulates while actually
	# moving, so pauses/stops don't dilute the average.
	if pack_run and _meas_speed > 1.0 and not _view_snapped:
		var pw := _power_for_speed(_meas_speed, _grade_at(d))
		_companion_watts = pw if _companion_watts <= 0.0 \
				else lerpf(_companion_watts, pw, 1.0 - exp(-delta / maxf(peloton_companion_tau, 1.0)))
	# CAPABILITY-BASED PACE: the pack's target speed is its category cruise adjusted for the
	# grade under it (slower up, faster down) — NOT an amplification of the player's speed.
	# This bounds it to what the riders could actually hold (no free-pace runaway to 100 km/h,
	# no glitch-driven warp). FREE PACE: ride that capability pace regardless of you (drops a
	# slower rider). Otherwise: match your (low-passed) pace, capped at capability so a strong
	# effort or a speed spike can't rocket the bunch. Absolute cap as a final sanity bound.
	var cap_pace := _capability_pace(_grade_at(_pack_ref))
	if _pack_cruise <= 0.0:
		_pack_cruise = cap_pace
	if pack_run:
		_pack_cruise = lerpf(_pack_cruise, cap_pace, 1.0 - exp(-delta / maxf(peloton_cruise_tau, 0.05)))
	# Companion mode paces like free-pace — the bunch rides its own capability — but that
	# capability IS the player's rolling effort × factor, so it's beatable by definition.
	var pace_free := peloton_free_pace or peloton_companion
	var ref_spd: float
	if pace_free:
		ref_spd = _pack_cruise
	else:
		ref_spd = minf(_pack_pace, _pack_cruise * peloton_cap_headroom)
	ref_spd = clampf(ref_spd, 0.0, peloton_speed_max_mps)
	var spd := ref_spd
	var strike := _pedal_strike_rad()
	var coast_thresh := strike - deg_to_rad(coast_margin_deg)
	# watts behind every pack rider's pedal rock (same source as _capability_pace)
	var rock_watts := peloton_wkg * peloton_mass_kg
	if peloton_companion and _companion_watts > 10.0:
		rock_watts = _companion_watts * peloton_companion_factor
	# Advance the pack reference (the "scroll"). NORMAL: slowly re-centre on the player so
	# you can surge fore/aft THROUGH the field but never run off it (bounded excursion, set
	# by peloton_pack_follow_tau). FREE PACE: the bunch rolls at its own cruise and can ride
	# away from you. A seek/loop (_view_snapped) recentres it so the whole pack follows.
	if not _pack_ref_inited:
		_pack_ref = d
		_pack_ref_inited = true
	if _view_snapped:
		_pack_ref = d
	elif pack_run:
		if pace_free:
			_pack_ref += ref_spd * delta
		else:
			_pack_ref = lerpf(_pack_ref, d, 1.0 - exp(-delta / maxf(peloton_pack_follow_tau, 0.1)))
	# The pack's ACTUAL advance rate this frame (≠ ref_spd while accelerating from a
	# seek, or in follow mode) — the bubble slew keys off this so a decaying yield can
	# never outrun a slow-moving pack (= rider visibly rolling backward at startup).
	var ref_rate := 0.0 if _view_snapped \
			else clampf((_pack_ref - _pack_ref_prev) / maxf(delta, 0.0001), 0.0, 30.0)
	_pack_ref_prev = _pack_ref
	for r in peloton:
		var node: Node3D = r["node"]

		# --- BOUNDED-STATION position: pos = pack_ref + fixed home + bounded drift(pack_ref).
		# The drift is the fore/aft surge/accordion; its amplitude was capped at spawn so
		# same-lane riders can NEVER converge → no collision, no live reaction. Rider speed
		# (for lean/cadence) is the along-route rate of this smooth position. ---
		var rd: float
		var rider_speed := spd
		if peloton_autonomous:
			rd = _pack_ref + float(r["home"]) \
					+ float(r["d_amp_s"]) * sin(_pack_ref / maxf(float(r["d_wl_s"]), 1.0) * TAU + float(r["d_ph_s"])) \
					+ float(r["d_amp_f"]) * sin(_pack_ref / maxf(float(r["d_wl_f"]), 1.0) * TAU + float(r["d_ph_f"]))
			# MAKE-ROOM BUBBLE: a rider in the player's lane yields fore/aft as the player
			# closes in (on top of the lateral lean-away) — the bunch opens and closes around
			# them. SCHMITT LATCH + SLEW: the yield side is latched, so at rel=0 the rider is
			# HELD a full bubble away (an odd/stateless repulsor is 0 at rel=0 by symmetry —
			# bench showed slow encounters drifting straight through the player, 30% overlap
			# frames on the flat). The latch flips ONLY when the rider's nominal station is
			# decisively past (±1 m beyond the player against the held side) — and because
			# the applied shift is rate-limited, the flip is a deliberate ~1.5 s glide
			# through, not the old teleport, and it can't pin a passed rider inside you
			# (the flip updates the direction the moment the pass is real).
			# latw = SMOOTH lateral proximity to the player (1 inside half a lane gap, ramping
			# to 0 at a full gap) — a hard lane test here would snap the shift on/off when
			# the player's racing line crosses the boundary while alongside.
			var latw := clampf(2.0 - absf(float(r["lateral"]) - _player_lat)
					/ maxf(peloton_lane_gap_m * 0.5, 0.01), 0.0, 1.0)
			# The hold is defined in RENDERED space: while latched, the shift target is
			# exactly what keeps the rider's RENDERED position ≥ hold_m away on the latched
			# side (btgt = hold − rel, one-sided). A ramp shaped in BASE-station space
			# (earlier attempt) parked riders dead on the player inside the hysteresis
			# window — the hold and the flip logic must live in the same frame.
			var btgt := 0.0
			var rel := rd - d
			var ft := maxf(peloton_bubble_m - peloton_bubble_hold_m, 0.2)   # flip threshold
			var bside: float = float(r.get("bub_side", 0.0))
			if peloton_bubble_m <= 0.001 or latw <= 0.0 or absf(rel) >= peloton_bubble_zone_m:
				bside = 0.0                                    # disengaged
			elif bside == 0.0:
				bside = 1.0 if rel >= 0.0 else -1.0            # engage on the current side
			elif bside > 0.0 and rel < -ft:
				bside = -1.0                                   # station truly past → commit through
			elif bside < 0.0 and rel > ft:
				bside = 1.0
			r["bub_side"] = bside
			if bside != 0.0:
				# sqrt(latw): a rider half a lane over still gets ~3/4 hold (it has its own
				# lateral clearance). One-sided: no shift once naturally clear on that side.
				var h := peloton_bubble_hold_m * sqrt(latw)
				btgt = maxf(0.0, h - rel) if bside > 0.0 else minf(0.0, -h - rel)
				btgt = clampf(btgt, -peloton_bubble_m, peloton_bubble_m)
			if _sim_active:
				# Slew scaled DOWN with the pack's ACTUAL pace: a flat ±3 m/s yield is
				# ±100% of a climbing crawl — riders visibly rolled backward on a 16% wall
				# (bench). 0.45·pace keeps the yield ≤ half the real speed at any grade,
				# including the acceleration ramp after a seek/start.
				var brate := minf(peloton_bubble_rate_mps, 0.45 * ref_rate + 0.05) * delta
				r["bub"] = clampf(btgt, float(r.get("bub", 0.0)) - brate,
						float(r.get("bub", 0.0)) + brate)
			rd += float(r.get("bub", 0.0))
			var prev := float(r.get("prev_pos_d", rd))
			var rawv := (rd - prev) / maxf(delta, 0.0001)
			r["rider_speed_raw"] = rawv          # unclamped along-route speed (for diagnostics)
			rider_speed = clampf(rawv, 0.0, 30.0)
			r["prev_pos_d"] = rd
			if r == peloton[0] and _pelo_t < 8.0 and _pelo_t >= _peldbg_next \
					and OS.has_environment("RIDESIM_PELOTON_DEBUG"):
				_peldbg_next = _pelo_t + 0.1
				print("[PELDBG] t=%.2f pack_ref=%.1f d=%.1f | r0 home=%+.1f drift=%+.2f rd=%.1f spd=%.2f (%.0f km/h)"
					% [_pelo_t, _pack_ref, d, float(r["home"]), rd - _pack_ref - float(r["home"]),
					   rd, rider_speed, rider_speed * 3.6])
		else:
			rd = d - float(r["home"])                    # legacy locked formation
		if rd < 0.0 or rd > route_len:
			node.visible = false                         # off either end of the route
			continue
		node.visible = true

		# --- lean (eased), from the rider's OWN speed (a surging rider leans more) ---
		var bank := _bank_target_rad(rd, rider_speed)
		bank = bank if peloton_bank_tau <= 0.0 \
				else lerpf(float(r["bank"]), bank, 1.0 - exp(-delta / peloton_bank_tau))
		r["bank"] = bank

		# --- lateral = PREORDAINED lane + apex drift + weave, but the drift+weave are CONFINED
		# to this rider's lane band so it can never cross into a neighbour lane. That band =
		# half the lane gap minus half a rider footprint → adjacent lanes keep ≥ 1 footprint of
		# clearance at ALL times, on any corner, GUARANTEED (the old free apex drift ≈ a whole
		# lane wide, so on bends the whole pack piled onto the apex line = cross-lane clipping).
		# apex is a pure function of rd (no per-rider temporal easing → no history divergence).
		# _band_soft (not clampf) so saturation can't step the lateral velocity.
		var wwl: float = maxf(float(r["weave_wl"]), 1.0)
		var woff: float = float(r["weave_off"])
		var apex := _race_line_lat(rd, peloton_race_line_m)
		var weave := peloton_wander_m * sin(rd / wwl * TAU + woff)
		var band := maxf(0.0, peloton_lane_gap_m * 0.5 - peloton_footprint_m * 0.5)
		var lane_c := float(r["lateral"])
		var lateral := lane_c + _band_soft(apex + weave, band)
		# CRAB FIX: yaw the bike into the slope of its own lateral path so a weaving/apexing
		# rider STEERS sideways instead of translating sideways (tires sliding across tarmac).
		# The lateral is closed-form in rd, so the slope is exact: chain rule through the soft
		# band, analytic weave derivative, finite-diff apex derivative.
		var yaw := 0.0
		if band > 0.001:
			var dapex := (_race_line_lat(rd + 2.0, peloton_race_line_m) - apex) / 2.0
			var dweave := peloton_wander_m * TAU / wwl * cos(rd / wwl * TAU + woff)
			var th := tanh((apex + weave) / band)
			yaw = -atan((1.0 - th * th) * (dapex + dweave))
		# Lean OUT of the player's way when they pass through, but STAY IN-LANE so this can
		# never break the cross-lane guarantee (the old dodge shoved riders to _player_lat ±
		# ghost_pass_m ≈ 1.7 m, far outside the band → it was the source of the near-player
		# clipping). Push only to the FAR EDGE of my own lane, on the side away from the
		# player; the player may still lightly overlap a rider directly in its lane, but AI
		# bikes never collide with each other.
		# prox × the same smooth lateral weight as the bubble — the old hard lane test made
		# the dodge snap on/off when the player's line crossed the lane boundary alongside.
		var prox := clampf(1.0 - absf(rd - d) / maxf(ghost_pass_zone_m, 0.1), 0.0, 1.0)
		prox *= clampf(2.0 - absf(lane_c - _player_lat) / maxf(peloton_lane_gap_m * 0.5, 0.01), 0.0, 1.0)
		# Dodge side is LATCHED for the whole encounter (prox > 0), chosen from where the
		# player's line sits at entry. Recomputing sign(lane_c − _player_lat) every frame
		# flipped it at 60 Hz whenever the player's line hovered on the lane center — a
		# ±band square wave (the "transporter" flitter), exposed when the old blanket
		# time-ease was removed for the crab fix.
		if prox > 0.0 and float(r.get("dodge_side", 0.0)) == 0.0:
			r["dodge_side"] = 1.0 if lane_c >= _player_lat else -1.0
		elif prox <= 0.0:
			r["dodge_side"] = 0.0
		# The dodge COMPONENT (only) is time-eased: it's genuinely temporal (player-relative),
		# and the ease also rounds the latch's edge cases (immediate re-entry on the other
		# side). The lane+weave stay un-eased — that blanket ease was the low-speed sideslip.
		var dodge_t := 0.0
		if float(r.get("dodge_side", 0.0)) != 0.0:
			var soft := _band_soft(apex + weave, band)
			dodge_t = prox * (float(r["dodge_side"]) * band - soft)
		if _sim_active:
			r["dodge"] = lerpf(float(r.get("dodge", dodge_t)), dodge_t, 1.0 - exp(-delta / 0.25))
		lateral += float(r.get("dodge", 0.0))
		r["lat"] = lateral
		# pedal rock from LAST frame's crank angle (the drivetrain updates below; one
		# frame of lag is invisible) at the rider's preferred cadence, faded out by its
		# coast blend so a rider coasting through a bend doesn't rock a frozen crank.
		var rock := _pedal_rock(float(r["crank_ph"]), rock_watts * (1.0 - float(r["coast"])),
				float(r["pref_rpm"]))
		_place_on_route_banked(node, rd, lateral, bank + rock.x, yaw + rock.y)
		# TELEPORT CATCHER: world-position jump in one frame, split into along-route
		# (d_rd) vs sideways (d_lat) so we know which. >1 m = teleport; <50 m skips the
		# legit whole-pack snap. Gated by RIDESIM_PELOTON_DEBUG=1.
		if _sim_active and not _view_snapped and r.has("prev_pos") \
				and OS.has_environment("RIDESIM_PELOTON_DEBUG"):
			var pj := node.global_position.distance_to(r["prev_pos"])
			if pj > 1.0 and pj < 50.0:
				print("[PELTELE] pos %.2f m  d_rd=%+.2f d_lat=%+.2f  t=%.2f lat=%+.2f prox=%.2f rd=%.0f spd=%.1f"
					% [pj, rd - float(r.get("prev_rd", rd)), lateral - float(r.get("prev_lat", lateral)),
					   _pelo_t, lateral, prox, rd, rider_speed])
		r["prev_pos"] = node.global_position
		r["prev_rd"] = rd
		r["prev_lat"] = lateral

		# --- drivetrain: cadence geared into a real band; coast pedal-up in hard bends ---
		_spin_wheels(r["rig"], rd)                        # wheels roll by their own distance
		# realistic cadence: preferred rpm + a little for going faster than the pack,
		# clamped to band; scaled to ~0 near a standstill so they don't pedal in place.
		var cadence := clampf(float(r["pref_rpm"]) + (rider_speed - ref_spd) * peloton_cadence_k,
				peloton_cadence_min, peloton_cadence_max) * clampf(rider_speed / 1.5, 0.0, 1.0)
		r["crank_ph"] = float(r["crank_ph"]) + cadence / 60.0 * TAU * delta
		var pedaling := float(r["crank_ph"])
		var want_coast := 1.0 if (peloton_coast_pedal_up and absf(bank) > coast_thresh) else 0.0
		var coast := want_coast if coast_tau <= 0.0 \
				else lerpf(float(r["coast"]), want_coast, 1.0 - exp(-delta / coast_tau))
		r["coast"] = coast
		var crank_ang := pedaling
		if coast > 0.001:
			# outside pedal down: the up-pedal is on the inside (the way it leans).
			var inside_left := bank > 0.0
			if coast_flip:
				inside_left = not inside_left
			var coast_target := coast_phase_rad + (PI if inside_left else 0.0)
			crank_ang = lerp_angle(pedaling, coast_target, coast)   # freeze toward upright
		_drive_crank(r["rig"], crank_ang)
		if r["legs"] != null:
			(r["legs"] as LegRig).pose()

	_lb_accum += delta
	if _lb_accum >= 0.5:
		_lb_accum = 0.0
		_update_leaderboard(d)


# avg / min / 1%-low fps over the rolling window (for the peloton perf readout).
func _fps_stats() -> Dictionary:
	if _fps_hist.is_empty():
		return {"avg": 0.0, "min": 0.0, "low1": 0.0}
	var s := _fps_hist.duplicate()
	s.sort()
	var sum := 0.0
	for f in s:
		sum += f
	var i1 := int(s.size() * 0.01)                       # 1st-percentile (worst frames)
	return {"avg": sum / s.size(), "min": s[0], "low1": s[i1]}


# --- distance -> world position --------------------------------------------

func _pos_xz_at(d: float) -> Vector3:
	if pts.is_empty():
		return Vector3.ZERO          # called before the route loaded — safe default
	d = clampf(d, 0.0, route_len)
	# advance/rewind cached segment so pts[seg_i].d <= d <= pts[seg_i+1].d
	while seg_i < pts.size() - 2 and float(pts[seg_i + 1].d) < d:
		seg_i += 1
	while seg_i > 0 and float(pts[seg_i].d) > d:
		seg_i -= 1
	var p0 = pts[seg_i]
	var p1 = pts[seg_i + 1]
	var span := float(p1.d) - float(p0.d)
	var t := 0.0
	if span >= 0.001:
		t = (d - float(p0.d)) / span
	var x := lerpf(float(p0.x), float(p1.x), t)
	var z := lerpf(float(p0.z), float(p1.z), t)
	return Vector3(x, 0, z)


# Smoothed ground height over the camera->target span: averaging terrain across
# the view distance low-passes bumps so the camera doesn't bob, while still
# following real hills. Includes road_lift.
func _smooth_ground(d: float) -> float:
	var d0 := d - cam_back_m
	var d1 := d + look_ahead_m
	var steps := 6
	var acc := 0.0
	for i in range(steps + 1):
		var p := _pos_xz_at(lerpf(d0, d1, float(i) / steps))
		acc += _terrain_y(p.x, p.z)
	return acc / float(steps + 1) + road_lift


func _update_heading(d: float, delta: float) -> void:
	# Damped look heading: rate-limit how fast the aim direction turns. Aiming
	# straight at a far look-ahead point makes the camera somersault where the
	# route reverses (dead-end out-and-back): the target lands behind the camera
	# and the aim vector swings through a degenerate pose. Slewing the heading at
	# cam_yaw_rate_deg makes such reversals pan smoothly, and also removes the
	# jaggy yaw on coarse polylines. Shared by every camera mode.
	var behind := _pos_xz_at(d - cam_back_m)
	var ahead := _pos_xz_at(d + look_ahead_m)
	var des := Vector2(ahead.x - behind.x, ahead.z - behind.z)
	if des.length() > 0.05:
		des = des.normalized()
		if cam_fwd == Vector2.ZERO:
			cam_fwd = des
		else:
			var step := clampf(cam_fwd.angle_to(des),
					-deg_to_rad(cam_yaw_rate_deg) * delta,
					 deg_to_rad(cam_yaw_rate_deg) * delta)
			cam_fwd = cam_fwd.rotated(step).normalized()


func _update_camera(d: float, delta: float) -> void:
	_update_heading(d, delta)
	match cam_mode:
		CamMode.COCKPIT:
			_cam_cockpit(d)
		CamMode.DRONE:
			_cam_orbit(d, delta, true)
		CamMode.FREE:
			_cam_orbit(d, delta, false)
		CamMode.ZENITH:
			_cam_zenith(d)
		_:
			_cam_chase(d)


# Keep a camera above the terrain at its own spot AND at the midpoint toward the
# rider, so a rise/cut lip between camera and rider can't occlude or bury it. Used
# by every following camera (rider_xz only needs valid x/z).
func _cam_floor(cpos: Vector3, rider_xz: Vector3) -> Vector3:
	var mid := cpos.lerp(rider_xz, 0.5)
	var floor_y := maxf(_terrain_y(cpos.x, cpos.z), _terrain_y(mid.x, mid.z)) + cam_clearance_m
	cpos.y = maxf(cpos.y, floor_y)
	return cpos


func _cam_chase(d: float) -> void:
	# Horizon-locked chase view: position plumb-locked to the (smoothed) track.
	# Camera height AND aim height share one smoothed ground value, so the pitch is
	# constant — no terrain-induced bobbing — while still rising over real hills.
	# +_player_off keeps the camera locked behind the player's racing line, not the
	# bare centerline, so the rider doesn't appear to slide sideways out of frame.
	var back_m := cam_back_m * cam_zoom          # wheel/[ ] pull the chase cam back + up
	var behind := _pos_xz_at(d - back_m) + _player_off
	var ground := _smooth_ground(d)
	var rider := _pos_xz_at(d) + _player_off
	var cpos := _cam_floor(Vector3(behind.x, ground + cam_height * cam_zoom, behind.z), rider)
	cam.global_position = cpos
	if cam_fwd != Vector2.ZERO:
		var aim_dist := back_m + look_ahead_m
		var target := Vector3(behind.x + cam_fwd.x * aim_dist, ground + aim_height,
							   behind.z + cam_fwd.y * aim_dist)
		cam.look_at(target, Vector3.UP)


func _cam_cockpit(d: float) -> void:
	# Rider's-eye POV: sit on the road surface (same height the rider rides), look
	# down the road. Uses the leveled road height, NOT _smooth_ground — the latter
	# averages raw terrain and sinks the POV underground in a cut.
	var here := _pos_xz_at(d) + _player_off   # sit on the player's line, not the centerline
	var road_y := _road_y_seg(d, here)
	cam.global_position = Vector3(here.x, road_y + cockpit_height, here.z)
	if cam_fwd != Vector2.ZERO:
		var target := Vector3(here.x + cam_fwd.x * look_ahead_m, road_y + aim_height,
							   here.z + cam_fwd.y * look_ahead_m)
		cam.look_at(target, Vector3.UP)


func _cam_orbit(d: float, delta: float, follow_heading: bool) -> void:
	# Orbit around the rider. DRONE (follow_heading) keeps the route oriented and
	# auto-orbits slowly; FREE is world-locked and fully user-steered. Both read
	# orbit_yaw_deg / orbit_pitch_deg / orbit_dist (seeded in _set_cam_mode).
	var here := _pos_xz_at(d) + _player_off   # orbit the player's line, not the centerline
	var ground := _smooth_ground(d)
	var pivot := Vector3(here.x, ground + 1.5, here.z)
	var base := 0.0
	if follow_heading:
		if drone_orbit_rate_deg != 0.0:
			orbit_yaw_deg += drone_orbit_rate_deg * delta
		if cam_fwd != Vector2.ZERO:
			base = rad_to_deg(atan2(cam_fwd.x, cam_fwd.y)) + 180.0  # +180 = behind
	var yaw := deg_to_rad(base + orbit_yaw_deg)
	var pitch := deg_to_rad(clampf(orbit_pitch_deg, 1.0, 89.0))
	var horiz := orbit_dist * cos(pitch)
	var offset := Vector3(sin(yaw) * horiz, orbit_dist * sin(pitch), cos(yaw) * horiz)
	# Don't fly into the mountain: stay above terrain under the camera and the ridge
	# between it and the rider (canyon routes). Shared with the chase cam.
	cam.global_position = _cam_floor(pivot + offset, pivot)
	cam.look_at(pivot, Vector3.UP)


func _cam_zenith(d: float) -> void:
	# Straight-down "zenith" map view over the rider. orbit_dist = altitude (zoom out to
	# god-view the whole built world). The route heading maps to screen-up so it reads like
	# a moving map. NOT terrain-floored — it's meant to fly high above everything.
	var here := _pos_xz_at(d) + _player_off
	var ground := _smooth_ground(d)
	var pivot := Vector3(here.x, ground + 1.5, here.z)
	cam.global_position = Vector3(pivot.x, pivot.y + orbit_dist, pivot.z)
	var up := Vector3(cam_fwd.x, 0.0, cam_fwd.y) if cam_fwd != Vector2.ZERO else Vector3(0, 0, -1)
	cam.look_at(pivot, up)


func _god_max() -> float:
	# Zoom-out ceiling for the god cameras: ~the built world's larger dimension (so the
	# whole thing fits), clamped to a sane band. Falls back if the grid isn't loaded yet.
	if gw <= 0 or gh <= 0:
		return 12000.0
	return clampf(1.15 * maxf(float(gw) * mpp_x, float(gh) * mpp_z), 1500.0, 30000.0)


func _zoom_step(notch: float) -> void:
	# notch > 0 = zoom OUT, < 0 = IN. CHASE zooms multiplicatively (its base is small);
	# the orbit/zenith cams step PROPORTIONALLY so a few notches cover metres → kilometres.
	match cam_mode:
		CamMode.CHASE:
			cam_zoom = clampf(cam_zoom * (1.0 + 0.12 * notch), 0.3, 15.0)
		CamMode.DRONE:
			orbit_dist = clampf(orbit_dist + notch * maxf(2.0, orbit_dist * 0.1), 2.0, 600.0)
		CamMode.FREE, CamMode.ZENITH:
			orbit_dist = clampf(orbit_dist + notch * maxf(2.0, orbit_dist * 0.15), 2.0, _god_max())
		# COCKPIT: not zoomable
	_apply_god_view()


func _apply_god_view() -> void:
	# At god-zoom the terrain right under the camera is kilometres away, so push the far
	# plane past it and THIN the aerial haze proportionally (else the whole world washes to
	# sky colour). Restores both when zoomed back in / in the ground cameras.
	var orbiting := cam_mode == CamMode.FREE or cam_mode == CamMode.ZENITH or cam_mode == CamMode.DRONE
	if cam != null:
		cam.far = maxf(12000.0, orbit_dist * 2.0 + 2000.0) if orbiting else 12000.0
	if env != null:
		var f := clampf(250.0 / maxf(orbit_dist, 1.0), 0.04, 1.0) if orbiting else 1.0
		env.fog_density = _fog_base * f


func _set_cam_mode(m: int) -> void:
	cam_mode = m
	match m:
		CamMode.DRONE:
			orbit_yaw_deg = 0.0          # behind (base adds +180)
			orbit_pitch_deg = drone_pitch_deg
			orbit_dist = drone_dist_m
		CamMode.FREE:
			# Seed world-locked yaw to the current heading so the view is continuous,
			# then the user owns it. +180 puts the camera behind to start.
			orbit_yaw_deg = (rad_to_deg(atan2(cam_fwd.x, cam_fwd.y)) + 180.0
							 if cam_fwd != Vector2.ZERO else 180.0)
			orbit_pitch_deg = 18.0
			orbit_dist = 16.0
		CamMode.ZENITH:
			orbit_dist = 140.0           # start a few hundred feet up; wheel/[ ] zoom to god view
	if avatar != null:
		avatar.visible = show_avatar and m != CamMode.COCKPIT  # don't sit inside the rider
	_apply_god_view()                    # set far plane + haze for the new mode's zoom
	_update_cam_label()


func _update_cam_label() -> void:
	if cam_label == null:
		return
	var names := ["CHASE", "COCKPIT", "DRONE", "FREE", "ZENITH"]
	# Kept short so it stays on one line at the 4K-readable font size.
	cam_label.text = "Cam: %s   Ghost: %s   [V J G  P fps  C this]" % [
		names[cam_mode], "ON" if auto_ghost else "OFF"]


# Ease the view-controls readout's opacity: visible while show_cam is on, but auto-
# hidden once the ride has been moving for cam_label_fade_s (declutters the view).
# Pressing C (or re-checking it) resets _ride_t, so it reappears for another window.
func _update_cam_fade(delta: float) -> void:
	if cam_label == null:
		return
	var faded := cam_label_fade_s > 0.0 and _ride_t > cam_label_fade_s
	var target_a := 1.0 if (show_cam and not faded) else 0.0
	cam_label.modulate.a = move_toward(cam_label.modulate.a, target_a, delta / 0.8)
	cam_label.visible = cam_label.modulate.a > 0.01


func _update_perf_label() -> void:
	if perf_label == null:
		return
	var fps := Engine.get_frames_per_second()
	var ms := 1000.0 / maxf(fps, 1.0)
	var draws := RenderingServer.get_rendering_info(
		RenderingServer.RENDERING_INFO_TOTAL_DRAW_CALLS_IN_FRAME)
	var prims := RenderingServer.get_rendering_info(
		RenderingServer.RENDERING_INFO_TOTAL_PRIMITIVES_IN_FRAME)
	perf_label.text = "%d fps   %.1f ms   |   %d draws   %.1fM prims" % [
		int(round(fps)), ms, draws, float(prims) / 1000000.0]
	# Peloton stress readout: rider count + min / 1%-low / avg over the window —
	# the numbers that tell you where the cliff is, not the smoothed instant fps.
	if not peloton.is_empty():
		var st := _fps_stats()
		perf_label.text += "\n%d riders   min %d   1%%low %d   avg %d fps" % [
			peloton.size(), int(st.min), int(st.low1), int(st.avg)]


# Teleport to a route distance and hold there — for inspecting singularities at a
# known km. Effective standalone; in a ride_sim-driven ride the next packet wins.
func _seek_to_km(km: float) -> void:
	dist = clampf(km * 1000.0, 0.0, route_len)
	seg_i = 0
	view_dist = dist          # snap the render (skip the ease)
	view_ghost_dist = dist
	paused = true             # hold for inspection; SPACE resumes
	print("seek → %.3f km (%.0f m), paused. demo pace %.1f m/s (%.0f km/h) — SPACE to ride, +/- to adjust" % [dist / 1000.0, dist, demo_speed, demo_speed * 3.6])


func _on_seek_submitted(text: String) -> void:
	var s := text.strip_edges()
	if s != "":
		_seek_to_km(s.to_float())
	seek_input.visible = false
	seek_input.release_focus()


# --- main loop --------------------------------------------------------------

func _process(delta: float) -> void:
	# Setting fullscreen in _apply_world_screen() pumps a frame mid-_ready, BEFORE
	# the camera is built and the route/terrain arrays are loaded. Skip those frames
	# (and the no-world fallback) so per-frame code never touches a null cam / empty
	# pts — otherwise startup floods the log with out-of-bounds / Nil errors.
	if cam == null or pts.is_empty():
		return
	_fps_hist.append(1.0 / maxf(delta, 0.0001))   # true per-frame fps for 1%-low
	while _fps_hist.size() > 240:
		_fps_hist.remove_at(0)
	_perf_accum += delta
	if _perf_accum >= 0.25:
		_perf_accum = 0.0
		_update_perf_label()
	# Held-key zoom for every zoomable cam ([ in / ] out), plus orbit steering (arrows) for
	# the orbit cams. COCKPIT is the only fixed view.
	if cam_mode != CamMode.COCKPIT:
		if Input.is_key_pressed(KEY_BRACKETLEFT):
			_zoom_step(-6.0 * delta)
		if Input.is_key_pressed(KEY_BRACKETRIGHT):
			_zoom_step(6.0 * delta)
	if cam_mode == CamMode.DRONE or cam_mode == CamMode.FREE:
		var rs := 60.0 * delta
		if Input.is_key_pressed(KEY_LEFT):
			orbit_yaw_deg -= rs
		if Input.is_key_pressed(KEY_RIGHT):
			orbit_yaw_deg += rs
		if Input.is_key_pressed(KEY_UP):
			orbit_pitch_deg = clampf(orbit_pitch_deg + rs, 1.0, 89.0)
		if Input.is_key_pressed(KEY_DOWN):
			orbit_pitch_deg = clampf(orbit_pitch_deg - rs, 1.0, 89.0)

	while udp.get_available_packet_count() > 0:
		var line := udp.get_packet().get_string_from_utf8()
		var msg = JSON.parse_string(line)
		if typeof(msg) == TYPE_DICTIONARY and msg.has("distance_m"):
			dist = float(msg.distance_m)
			cur_speed = float(msg.get("speed_mps", cur_speed))
			cur_cadence = float(msg.get("cadence_rpm", cur_cadence))
			cur_power = float(msg.get("power_w", cur_power))
			_live_paused = bool(msg.get("paused", false))
			live = true
			if msg.has("ghost_distance_m"):
				ghost_live_dist = float(msg.ghost_distance_m)
				ghost_live_speed = float(msg.get("ghost_speed_mps", ghost_live_speed))
				ghost_is_live = true

	# The sim advances when running, or for one frame per queued step (KEY_PERIOD while
	# paused) — a frame-stepper to inspect the pack (see _update_player_line/_update_peloton).
	_sim_active = not paused
	if paused and _step_frames > 0:
		_sim_active = true
		_step_frames -= 1

	if live:
		dist += cur_speed * delta          # dead-reckon between packets
		if ghost_is_live:
			ghost_live_dist += ghost_live_speed * delta   # dead-reckon ghost too
	elif (not _sim_active) or wait_for_telemetry:
		pass                                # hold at the start until ride_sim connects
	else:
		dist += demo_speed * delta         # standalone demo auto-advance (or one step)

	if dist >= route_len:
		if live:
			dist = route_len                # ride_sim owns position: hold at the
			seg_i = pts.size() - 2          # finish, don't loop or restart
		else:
			dist = 0.0                      # loop the standalone demo
			seg_i = 0
	elif dist < 0.0:
		dist = 0.0

	# Ease the render distance toward the authoritative dist so the small ~4 Hz
	# packet snaps don't jerk the avatar; snap hard on big jumps (scrub/first pkt).
	# A hard snap is a TELEPORT (a real frame moves <1 m even at 50 m/s), so flag it:
	# the apparent speed that frame is garbage and must NOT feed the pack pace (it would
	# seed the free-pace cruise at the 50 m/s clamp and the bunch would rocket off).
	_view_snapped = absf(view_dist - dist) > 50.0
	# On a teleport (seek, or the demo looping at route_len) the bounded-station model
	# recentres the whole pack via `_pack_ref = d` in _update_peloton (gated on _view_snapped),
	# so riders keep their stations relative to the player instead of being left behind.
	if view_smooth_tau > 0.0 and not _view_snapped:
		view_dist = lerpf(view_dist, dist, 1.0 - exp(-delta / view_smooth_tau))
	else:
		view_dist = dist
	if ghost_is_live:
		if view_smooth_tau > 0.0 and absf(view_ghost_dist - ghost_live_dist) <= 50.0:
			view_ghost_dist = lerpf(view_ghost_dist, ghost_live_dist, 1.0 - exp(-delta / view_smooth_tau))
		else:
			view_ghost_dist = ghost_live_dist

	# Drivetrain phase (crank/cassette/legs). Live rider pedals at the reported
	# cadence — cadence 0 holds the cranks while the bike rolls on (coasting). With no
	# live telemetry (standalone/demo) fall back to development so it still pedals.
	if live:
		crank_phase += cur_cadence / 60.0 * TAU * delta
	elif crank_dev_m > 0.001:
		crank_phase = dist / crank_dev_m * TAU

	_update_player_line(view_dist, delta)   # measure speed + player line BEFORE the camera
	_update_camera(view_dist, delta)
	_update_avatar(view_dist, delta)
	_update_peloton(view_dist, delta)

	if OS.has_environment("RIDESIM_AVATAR_DEBUG") and avatar != null:
		_avdbg_accum += delta
		if _avdbg_accum >= 1.0:
			_avdbg_accum = 0.0
			print("[AVDBG] d=%.0f  avatar.gpos=%s visible=%s  cam.gpos=%s  cam_to_avatar=%.1fm  mode=%d"
				% [view_dist, avatar.global_position, avatar.visible,
				   cam.global_position, cam.global_position.distance_to(avatar.global_position), cam_mode])

	if minimap != null and minimap_layer.visible:
		var here := _pos_xz_at(view_dist)
		var fwd := _pos_xz_at(view_dist + 8.0)
		minimap.set_marker(-here.x, here.z, -(fwd.x - here.x), fwd.z - here.z)


func _managed_by_ridesim() -> bool:
	# True when ride_sim launched us (data dir or world-screen hint set) or is
	# actively driving us — ride_sim then owns quitting, not Escape.
	return live or OS.get_environment("RIDESIM_WORLD_DIR") != "" \
		or OS.get_environment("RIDESIM_WORLD_SCREEN_POS") != ""


func _unhandled_input(e: InputEvent) -> void:
	# Mouse: wheel zooms, right-drag orbits (DRONE/FREE).
	if e is InputEventMouseButton:
		var mb := e as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			_zoom_step(-1.0)                       # zoom in (per-mode; cockpit ignores)
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			_zoom_step(1.0)                        # zoom out
		elif mb.button_index == MOUSE_BUTTON_RIGHT:
			mouse_orbiting = mb.pressed
		return
	if e is InputEventMouseMotion and mouse_orbiting:
		var mm := e as InputEventMouseMotion
		orbit_yaw_deg += mm.relative.x * 0.25
		orbit_pitch_deg = clampf(orbit_pitch_deg - mm.relative.y * 0.25, 1.0, 89.0)
		return

	var k := e as InputEventKey
	if k == null or not k.pressed:
		return
	if k.keycode == KEY_PERIOD:          # frame-step while paused (tap = 1 frame, hold = crawl)
		if paused:
			_step_frames += 1
		return
	if k.echo:
		return
	match k.keycode:
		KEY_V:
			_set_cam_mode((cam_mode + 1) % (CamMode.ZENITH + 1))  # ZENITH stays last
		KEY_G:
			auto_ghost = not auto_ghost
			_update_cam_label()
		KEY_J:
			if seek_input != null:
				seek_input.visible = not seek_input.visible
				if seek_input.visible:
					seek_input.text = ""
					seek_input.grab_focus()
		KEY_ESCAPE:
			# Escape un-fullscreens first (macOS convention). Only quit when
			# standalone — in a ride_sim-driven ride, quitting here would strand
			# the ride_sim dashboard, so let ride_sim own the lifecycle.
			if DisplayServer.window_get_mode() == DisplayServer.WINDOW_MODE_FULLSCREEN:
				DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
			elif not _managed_by_ridesim():
				get_tree().quit()
		KEY_SPACE:
			if live:
				# ride_sim owns position; ask it to toggle pause (it freezes us via
				# speed 0). One-way UDP, so we don't flip local state here.
				back_udp.put_packet(JSON.stringify({"cmd": "toggle_pause"}).to_utf8_buffer())
			else:
				paused = not paused
		KEY_EQUAL, KEY_KP_ADD:
			demo_speed = minf(demo_speed + 1.0, 30.0)
			print("demo_speed = %.1f m/s (%.0f km/h)" % [demo_speed, demo_speed * 3.6])
		KEY_MINUS, KEY_KP_SUBTRACT:
			demo_speed = maxf(demo_speed - 1.0, 1.0)
			print("demo_speed = %.1f m/s (%.0f km/h)" % [demo_speed, demo_speed * 3.6])
		KEY_SLASH:
			# slow-mo cycle — scales the WHOLE engine (player + pack + easing) uniformly, so
			# a fast transient (the start roar) plays out watchably. '/' cycles, prints scale.
			var scales := [1.0, 0.5, 0.25, 0.1, 0.05]
			var si := scales.find(Engine.time_scale)
			Engine.time_scale = scales[(si + 1) % scales.size()] if si >= 0 else 0.25
			print("time_scale = %.2f" % Engine.time_scale)
		KEY_M:
			if minimap_layer != null:
				minimap_layer.visible = not minimap_layer.visible
		KEY_P:
			show_perf = not show_perf
			if perf_label != null:
				perf_label.visible = show_perf
			if _ui_perf != null:
				_syncing = true; _ui_perf.button_pressed = show_perf; _syncing = false
		KEY_C:                       # show/hide the view-controls readout
			show_cam = not show_cam
			if show_cam:
				_ride_t = 0.0        # re-show for another fade interval
			if _ui_cam != null:
				_syncing = true; _ui_cam.button_pressed = show_cam; _syncing = false
		# --- quality / LOD profiling (watch fps on P, state on Q) ---
		KEY_1:
			_set_preset("low"); _quality_key()
		KEY_2:
			_set_preset("medium"); _quality_key()
		KEY_3:
			_set_preset("high"); _quality_key()
		KEY_H:                       # all dynamic sun shadows
			q_shadows = not q_shadows; _quality_key()
		KEY_Y:                       # tree + pole shadow CASTING (the 51k-caster cost)
			q_tree_shadows = not q_tree_shadows; _quality_key()
		KEY_T:                       # draw trees at all
			q_trees = not q_trees; _quality_key()
		KEY_B:                       # bloom / glow
			q_glow = not q_glow; _quality_key()
		KEY_9:                       # draw distance −500 m
			q_far = maxf(q_far - 500.0, 500.0); _quality_key()
		KEY_0:                       # draw distance +500 m
			q_far = minf(q_far + 500.0, 16000.0); _quality_key()
		KEY_Q:                       # show/hide the quality readout
			show_quality = not show_quality
			if quality_label != null:
				quality_label.visible = show_quality


# --- minimap overlay --------------------------------------------------------
# North-up 2D map of the route with a live position marker. Drawn in screen
# space on a CanvasLayer; the route is pre-projected once in setup().
class Minimap extends Control:
	var route_px := PackedVector2Array()
	var msize := Vector2.ZERO
	var margin := 12.0
	var scl := 1.0
	var wxmin := 0.0
	var wzmin := 0.0
	var start_px := Vector2.ZERO
	var end_px := Vector2.ZERO
	var pos_px := Vector2.ZERO
	var head_px := Vector2.ZERO
	var pack_px := PackedVector2Array()   # peloton dots (screen px)
	var has_pos := false
	var _last_px := Vector2(-999, -999)

	func _w2p(x: float, z: float) -> Vector2:
		# +z is North -> screen up, so flip the vertical axis
		return Vector2(margin + (x - wxmin) * scl, msize.y - margin - (z - wzmin) * scl)

	func setup(route2: PackedVector2Array, sz: Vector2) -> void:
		msize = sz
		if route2.is_empty():
			return
		var xmin := route2[0].x; var xmax := route2[0].x
		var zmin := route2[0].y; var zmax := route2[0].y
		for p in route2:
			xmin = minf(xmin, p.x); xmax = maxf(xmax, p.x)
			zmin = minf(zmin, p.y); zmax = maxf(zmax, p.y)
		wxmin = xmin; wzmin = zmin
		scl = minf((sz.x - 2.0 * margin) / maxf(1.0, xmax - xmin),
				   (sz.y - 2.0 * margin) / maxf(1.0, zmax - zmin))
		route_px.resize(route2.size())
		for i in range(route2.size()):
			route_px[i] = _w2p(route2[i].x, route2[i].y)
		start_px = route_px[0]
		end_px = route_px[route_px.size() - 1]
		queue_redraw()

	func set_pack(world_pts: PackedVector2Array) -> void:
		# peloton dots (world x already flipped by the caller, same as the route)
		pack_px.resize(world_pts.size())
		for i in range(world_pts.size()):
			pack_px[i] = _w2p(world_pts[i].x, world_pts[i].y)
		queue_redraw()

	func set_marker(x: float, z: float, hx: float, hz: float) -> void:
		pos_px = _w2p(x, z)
		var h := Vector2(hx, -hz)            # same vertical flip as _w2p
		head_px = pos_px
		if h.length() > 0.0001:
			head_px = pos_px + h.normalized() * maxf(12.0, msize.x / 22.0)
		has_pos = true
		if pos_px.distance_to(_last_px) > 1.5:
			_last_px = pos_px
			queue_redraw()

	func _draw() -> void:
		draw_rect(Rect2(Vector2.ZERO, msize), Color(0.05, 0.05, 0.07, 0.55))
		draw_rect(Rect2(Vector2.ZERO, msize), Color(1, 1, 1, 0.15), false, 1.0)
		# scale line/marker weights with map size so a big minimap stays legible
		var s := maxf(1.0, msize.x / 300.0)
		if route_px.size() > 1:
			draw_polyline(route_px, Color(0.95, 0.82, 0.15), 1.5 * s)
		draw_circle(start_px, 4.0 * s, Color(0.2, 0.9, 0.2))
		draw_circle(end_px, 3.5 * s, Color(0.1, 0.1, 0.1))
		for pp in pack_px:                     # peloton: small cyan dots under the player dot
			draw_circle(pp, 2.2 * s, Color(0.35, 0.8, 1.0, 0.9))
		if has_pos:
			if head_px != pos_px:
				draw_line(pos_px, head_px, Color(1, 1, 1, 0.9), 2.0 * s)
			draw_circle(pos_px, 4.5 * s, Color(0.95, 0.2, 0.2))
		draw_string(ThemeDB.fallback_font, Vector2(msize.x * 0.5 - 4.0 * s, 15.0 * s), "N",
				HORIZONTAL_ALIGNMENT_LEFT, -1, int(12 * s), Color(1, 1, 1, 0.7))
