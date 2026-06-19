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
# Keys: SPACE pause/resume demo, +/- demo speed, M minimap, G toggle auto pace-
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
@export var demo_speed: float = 6.7      # m/s (~15 mph) when no telemetry
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
@export var show_minimap: bool = true    # north-up route map overlay (toggle: M)
@export var minimap_size: int = 480      # minimap square size in px
@export var show_avatar: bool = true     # rider marker following the path
@export var avatar_scale: float = 1.0    # scale the rider (bump up for the drone view)
@export var show_ghost: bool = true      # ghost rider (different glow color)
@export var ghost_gap_m: float = 60.0    # demo: ghost this far up the route (+ahead/-behind)

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

var dist: float = 0.0
var live := false
var cur_speed: float = 0.0
var paused := false
var seg_i := 0             # cached segment index for distance lookup
var cam_fwd := Vector2.ZERO   # damped camera heading (world x,z); 0 until first frame
var ghost_live_dist := 0.0    # ride_sim's real ghost position (m), when sent
var ghost_is_live := false    # true once ride_sim sends ghost_distance_m
var view_dist := 0.0          # render distance: eases toward dist to hide the
var view_ghost_dist := 0.0    # ~4 Hz packet snaps (smooth avatar/camera motion)

# User-controllable camera (P3). CHASE is the original on-rails view; the rest
# are opt-in via the V key. DRONE/FREE share orbit_* state (seeded on entry).
enum CamMode { CHASE, COCKPIT, DRONE, FREE }
var cam_mode: int = CamMode.CHASE
var orbit_yaw_deg := 0.0      # yaw around the rider (deg); +180 base = behind
var orbit_pitch_deg := 30.0   # look-down pitch (deg), clamped 1..89
var orbit_dist := 22.0        # camera distance from the rider (m)
var mouse_orbiting := false   # right-button drag orbits in DRONE/FREE
var cam_label: Label
var auto_ghost := false       # show a synthetic pace ghost when none is live (G)
var seek_input: LineEdit      # J: type a km to jump to (test singularities)

var udp := PacketPeerUDP.new()
var back_udp := PacketPeerUDP.new()   # sends commands back to ride_sim
var cam: Camera3D
var minimap: Minimap
var minimap_layer: CanvasLayer
var avatar: Node3D
var ghost: Node3D

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
	if loaded:
		_build_minimap()
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
	_compute_road_center_y()
	var line := PackedVector2Array()
	for p in pts:
		line.append(Vector2(float(p.x), float(p.z)))
	_road_surface(line, road_width * 0.5, road_lift, Color(0.30, 0.30, 0.33), 0.9, _tarmac_material())
	_road_surface(line, 0.30, road_lift + 0.12, Color(0.95, 0.82, 0.15), 0.6)
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
		col: Color, rough: float, mat_override: Material = null) -> void:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	if not _add_centered_ribbon(st, line, hw, lift):
		return
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
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-45, 130, 0)
	sun.shadow_enabled = true
	sun.light_energy = 1.25                       # brighter so terrain color/grain reads
	sun.light_color = Color(1.0, 0.97, 0.90)      # slightly warm daylight
	add_child(sun)

	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.fog_enabled = true
	env.fog_density = 0.00025            # lighter aerial haze (was 0.0008 = heavy wash-out)
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
	cam_label.offset_left = 24.0; cam_label.offset_top = -150.0
	cam_label.add_theme_color_override("font_color", Color(0.6, 0.9, 1.0))
	cam_label.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.85))
	cam_label.add_theme_constant_override("outline_size", 10)
	cam_label.add_theme_font_size_override("font_size", 150)
	cl.add_child(cam_label)
	_update_cam_label()

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


# --- rider avatar -----------------------------------------------------------

func _build_avatar() -> void:
	# Drop in res://assets/rider.glb (e.g. your decimated SolidWorks export) and it
	# is used automatically; otherwise build a stylized emissive placeholder. Ghost
	# is the same idea in a contrasting color (its own ghost.glb if present).
	if show_avatar:
		avatar = _spawn_avatar("res://assets/rider.glb", Color(0.15, 0.9, 1.0))
	if show_ghost:
		ghost = _spawn_avatar("res://assets/ghost.glb", Color(1.0, 0.45, 0.1))
	_update_avatar(0.0)


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


func _update_avatar(d: float) -> void:
	if avatar != null:
		_place_on_route(avatar, d)
	if ghost != null:
		# Show the ghost only when ride_sim sends a real one, OR the user turned on
		# the synthetic pace ghost (G). Off by default so no phantom rider appears.
		var show := ghost_is_live or auto_ghost
		ghost.visible = show
		if show:
			if ghost_is_live:
				_place_on_route(ghost, clampf(view_ghost_dist, 0.0, route_len))
			else:
				_place_on_route(ghost, fposmod(d + ghost_gap_m, route_len))


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


func _place_on_route(node: Node3D, d: float) -> void:
	var here := _pos_xz_at(d)   # sets seg_i to d's segment
	# Ride ON the leveled road surface (not raw terrain) so the rider never buries.
	here.y = _road_y_seg(d, here)
	var fwd := _pos_xz_at(d + 4.0)
	node.global_position = here
	var dir := Vector3(fwd.x - here.x, 0.0, fwd.z - here.z)
	if dir.length() > 0.01:
		node.look_at(here + dir, Vector3.UP)


# --- distance -> world position --------------------------------------------

func _pos_xz_at(d: float) -> Vector3:
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
	var behind := _pos_xz_at(d - cam_back_m)
	var ground := _smooth_ground(d)
	var rider := _pos_xz_at(d)
	var cpos := _cam_floor(Vector3(behind.x, ground + cam_height, behind.z), rider)
	cam.global_position = cpos
	if cam_fwd != Vector2.ZERO:
		var aim_dist := cam_back_m + look_ahead_m
		var target := Vector3(behind.x + cam_fwd.x * aim_dist, ground + aim_height,
							   behind.z + cam_fwd.y * aim_dist)
		cam.look_at(target, Vector3.UP)


func _cam_cockpit(d: float) -> void:
	# Rider's-eye POV: sit on the road surface (same height the rider rides), look
	# down the road. Uses the leveled road height, NOT _smooth_ground — the latter
	# averages raw terrain and sinks the POV underground in a cut.
	var here := _pos_xz_at(d)
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
	var here := _pos_xz_at(d)
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
	if avatar != null:
		avatar.visible = show_avatar and m != CamMode.COCKPIT  # don't sit inside the rider
	_update_cam_label()


func _update_cam_label() -> void:
	if cam_label == null:
		return
	var names := ["CHASE", "COCKPIT", "DRONE", "FREE"]
	# Kept short so it stays on one line at the large 4K-readable font size.
	cam_label.text = "Cam: %s   Ghost: %s   [V J G]" % [
		names[cam_mode], "ON" if auto_ghost else "OFF"]


# Teleport to a route distance and hold there — for inspecting singularities at a
# known km. Effective standalone; in a ride_sim-driven ride the next packet wins.
func _seek_to_km(km: float) -> void:
	dist = clampf(km * 1000.0, 0.0, route_len)
	seg_i = 0
	view_dist = dist          # snap the render (skip the ease)
	view_ghost_dist = dist
	paused = true             # hold for inspection; SPACE resumes
	print("seek → %.3f km (%.0f m), paused" % [dist / 1000.0, dist])


func _on_seek_submitted(text: String) -> void:
	var s := text.strip_edges()
	if s != "":
		_seek_to_km(s.to_float())
	seek_input.visible = false
	seek_input.release_focus()


# --- main loop --------------------------------------------------------------

func _process(delta: float) -> void:
	# Held-key camera steering (DRONE/FREE): smooth orbit + zoom.
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
		if Input.is_key_pressed(KEY_BRACKETLEFT):
			orbit_dist = maxf(orbit_dist - 30.0 * delta, 2.0)
		if Input.is_key_pressed(KEY_BRACKETRIGHT):
			orbit_dist = minf(orbit_dist + 30.0 * delta, 600.0)

	while udp.get_available_packet_count() > 0:
		var line := udp.get_packet().get_string_from_utf8()
		var msg = JSON.parse_string(line)
		if typeof(msg) == TYPE_DICTIONARY and msg.has("distance_m"):
			dist = float(msg.distance_m)
			cur_speed = float(msg.get("speed_mps", cur_speed))
			live = true
			if msg.has("ghost_distance_m"):
				ghost_live_dist = float(msg.ghost_distance_m)
				ghost_is_live = true

	if live:
		dist += cur_speed * delta          # dead-reckon between packets
	elif paused or wait_for_telemetry:
		pass                                # hold at the start until ride_sim connects
	else:
		dist += demo_speed * delta         # standalone demo auto-advance

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
	if view_smooth_tau > 0.0 and absf(view_dist - dist) <= 50.0:
		view_dist = lerpf(view_dist, dist, 1.0 - exp(-delta / view_smooth_tau))
	else:
		view_dist = dist
	if ghost_is_live:
		if view_smooth_tau > 0.0 and absf(view_ghost_dist - ghost_live_dist) <= 50.0:
			view_ghost_dist = lerpf(view_ghost_dist, ghost_live_dist, 1.0 - exp(-delta / view_smooth_tau))
		else:
			view_ghost_dist = ghost_live_dist

	_update_camera(view_dist, delta)
	_update_avatar(view_dist)

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
			orbit_dist = maxf(orbit_dist - 2.0, 2.0)
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			orbit_dist = minf(orbit_dist + 2.0, 600.0)
		elif mb.button_index == MOUSE_BUTTON_RIGHT:
			mouse_orbiting = mb.pressed
		return
	if e is InputEventMouseMotion and mouse_orbiting:
		var mm := e as InputEventMouseMotion
		orbit_yaw_deg += mm.relative.x * 0.25
		orbit_pitch_deg = clampf(orbit_pitch_deg - mm.relative.y * 0.25, 1.0, 89.0)
		return

	var k := e as InputEventKey
	if k == null or not k.pressed or k.echo:
		return
	match k.keycode:
		KEY_V:
			_set_cam_mode((cam_mode + 1) % (CamMode.FREE + 1))  # FREE stays last
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
		KEY_MINUS, KEY_KP_SUBTRACT:
			demo_speed = maxf(demo_speed - 1.0, 1.0)
		KEY_M:
			if minimap_layer != null:
				minimap_layer.visible = not minimap_layer.visible


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
		if has_pos:
			if head_px != pos_px:
				draw_line(pos_px, head_px, Color(1, 1, 1, 0.9), 2.0 * s)
			draw_circle(pos_px, 4.5 * s, Color(0.95, 0.2, 0.2))
		draw_string(ThemeDB.fallback_font, Vector2(msize.x * 0.5 - 4.0 * s, 15.0 * s), "N",
				HORIZONTAL_ALIGNMENT_LEFT, -1, int(12 * s), Color(1, 1, 1, 0.7))
