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
# Keys: SPACE pause/resume demo, +/- demo speed, ESC quit.

@export var udp_port: int = 5005
@export var demo_speed: float = 6.7      # m/s (~15 mph) when no telemetry
@export var look_ahead_m: float = 25.0   # camera aims this far down-route
@export var aim_height: float = 0.5      # look-target height above road
@export var cam_back_m: float = 8.0      # camera trails this far behind the rider
@export var cam_height: float = 5.0      # camera height above road (chase view)
@export var road_width: float = 8.0
@export var road_lift: float = 0.5       # sit road just above terrain; finer
										 # (~11 m) bake means no 5 m floating hack

# OSM feature layers (P1) — draped on the terrain like the main road.
@export var show_roads: bool = true
@export var show_paths: bool = false     # footways/cycleways (~12k ways) — opt in
@export var show_service: bool = false   # driveways/parking aisles (~8.5k) — opt in
@export var show_water: bool = true
@export var show_landuse: bool = true
@export var show_buildings: bool = true  # extruded OSM footprints near the route
@export var building_height_scale: float = 1.0
@export var feature_lift: float = 0.3    # cross-streets sit just below the ridden road
@export var resample_m: float = 4.5      # resample route to uniform spacing (m) at
										 # load; densifies coarse OSM-routed paths. 0 = off
@export var route_smooth: int = 3        # centerline moving-average radius (pts);
										 # smooths GPS jitter / stop-light loops. 0 = off
@export var show_minimap: bool = true    # north-up route map overlay (toggle: M)
@export var minimap_size: int = 300      # minimap square size in px
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

var dist: float = 0.0
var live := false
var cur_speed: float = 0.0
var paused := false
var seg_i := 0             # cached segment index for distance lookup

var udp := PacketPeerUDP.new()
var cam: Camera3D
var minimap: Minimap
var minimap_layer: CanvasLayer
var avatar: Node3D
var ghost: Node3D


func _ready() -> void:
	_load_data()
	_build_terrain()
	_build_road()
	_build_features()
	_build_camera_and_sky()
	_build_minimap()
	_build_avatar()
	if udp.bind(udp_port) == OK:
		print("listening for ride_sim telemetry on udp:%d" % udp_port)
	else:
		push_warning("could not bind udp:%d — demo mode only" % udp_port)


func _load_data() -> void:
	world = JSON.parse_string(FileAccess.get_file_as_string("res://data/world.json"))
	gw = int(world.grid_w); gh = int(world.grid_h)
	x0 = float(world.x0); z0 = float(world.z0)
	mpp_x = float(world.mpp_x); mpp_z = float(world.mpp_z)

	var f := FileAccess.open("res://data/heights.bin", FileAccess.READ)
	heights = f.get_buffer(f.get_length()).to_float32_array()
	f.close()

	var route: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://data/route.json"))
	pts = route.points
	route_len = float(route.length_m)
	# Un-mirror: mapping ENU East->+X / North->+Z into Godot's axes is a reflection
	# (flips chirality, so right turns render as left bends). Negate East here, and
	# matching negations in _build_terrain/_terrain_y/_build_features, so the whole
	# world shares one corrected frame. pts is the single source for road+cam+map.
	for i in range(pts.size()):
		pts[i]["x"] = -float(pts[i].x)
	_resample_route(resample_m)
	_smooth_route(route_smooth)
	print("route %.2f km, terrain %d x %d cells, %d pts" % [route_len / 1000.0, gw, gh, pts.size()])


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


# --- mesh building ----------------------------------------------------------

func _build_terrain() -> void:
	var verts := PackedVector3Array(); verts.resize(gw * gh)
	var norms := PackedVector3Array(); norms.resize(gw * gh)
	var cols := PackedColorArray(); cols.resize(gw * gh)

	for r in range(gh):
		for c in range(gw):
			var i := r * gw + c
			var y := heights[i]
			verts[i] = Vector3(-(x0 + c * mpp_x), y, z0 + r * mpp_z)   # East negated
			cols[i] = _elev_color(y)
			# normal from heightfield central differences (X-comp flips with East)
			var cl := maxi(c - 1, 0); var cr := mini(c + 1, gw - 1)
			var ru := maxi(r - 1, 0); var rd := mini(r + 1, gh - 1)
			var dydx := (heights[r * gw + cr] - heights[r * gw + cl]) / ((cr - cl) * mpp_x)
			var dydz := (heights[rd * gw + c] - heights[ru * gw + c]) / ((rd - ru) * mpp_z)
			norms[i] = Vector3(dydx, 1.0, -dydz).normalized()

	var idx := PackedInt32Array()
	for r in range(gh - 1):
		for c in range(gw - 1):
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

	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.roughness = 1.0
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED   # East-negation flips winding
	mesh.surface_set_material(0, mat)

	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)


func _build_road() -> void:
	# The ridden road: asphalt + a bright center line that streams past as a
	# motion cue. Uses the same mitered draping as the OSM streets.
	var road_pts := []
	for p in pts:
		road_pts.append([float(p.x), float(p.z)])
	_drape_lines([road_pts], road_width * 0.5, Color(0.30, 0.30, 0.33), road_lift, 0.9)
	_drape_lines([road_pts], 0.30, Color(0.95, 0.82, 0.15), road_lift + 0.1, 0.6)


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
	var added := false
	for i in range(n - 1):
		var l0 := left[i]; var r0 := right[i]
		var l1 := left[i + 1]; var r1 := right[i + 1]
		var L0 := Vector3(l0.x, _terrain_y(l0.x, l0.y) + lift, l0.y)
		var R0 := Vector3(r0.x, _terrain_y(r0.x, r0.y) + lift, r0.y)
		var L1 := Vector3(l1.x, _terrain_y(l1.x, l1.y) + lift, l1.y)
		var R1 := Vector3(r1.x, _terrain_y(r1.x, r1.y) + lift, r1.y)
		for v in [L0, L1, R0, R0, L1, R1]:
			st.set_normal(Vector3.UP)
			st.add_vertex(v)
		added = true
	return added


# --- OSM feature layers (roads, water, landuse) -----------------------------

func _build_features() -> void:
	if not FileAccess.file_exists("res://data/features.json"):
		print("no features.json — skipping OSM layers (run tools/osm_to_features.py)")
		return
	var parsed = JSON.parse_string(FileAccess.get_file_as_string("res://data/features.json"))
	if typeof(parsed) != TYPE_DICTIONARY:
		push_warning("features.json did not parse")
		return
	var data: Dictionary = parsed

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
				_drape_lines(bucket[b], style[b]["hw"], style[b]["col"], feature_lift, 0.95)

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

	if show_landuse:
		var by_class := {}
		for lu in data.get("landuse", []):
			var c = lu["class"]
			if not by_class.has(c):
				by_class[c] = []
			by_class[c].append(_flipx(lu["pts"]))
		for c in by_class:
			_drape_polygons(by_class[c], _landuse_color(c), 0.05, 0.45)

	_build_buildings(data)


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
				walls.set_normal(nrm); walls.add_vertex(v)
		var tris := Geometry2D.triangulate_polygon(ring)
		for ti in tris:
			var v := ring[ti]
			roofs.set_normal(Vector3.UP)
			roofs.add_vertex(Vector3(v.x, roof_y, v.y))
		any = true
	if not any:
		return
	_commit_surface(walls, Color(0.62, 0.60, 0.57), 0.9)
	_commit_surface(roofs, Color(0.40, 0.39, 0.41), 0.85)


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
	add_child(sun)

	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.fog_enabled = true
	env.fog_density = 0.0008
	env.glow_enabled = true              # bloom for the emissive avatar (Tron look)
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	cam = Camera3D.new()
	cam.fov = 70.0
	cam.far = 12000.0
	add_child(cam)
	cam.make_current()
	_update_camera(0.0)


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
		_place_on_route(ghost, fposmod(d + ghost_gap_m, route_len))


func _place_on_route(node: Node3D, d: float) -> void:
	var here := _pos_xz_at(d)
	var fwd := _pos_xz_at(d + 4.0)
	here.y = _terrain_y(here.x, here.z) + road_lift
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


func _update_camera(d: float) -> void:
	# Horizon-locked chase view: position plumb-locked to the (smoothed) track,
	# steered by yaw. Camera height AND aim height share one smoothed ground value,
	# so the pitch is constant — no terrain-induced bobbing — while the camera
	# still rises and falls over real hills.
	var behind := _pos_xz_at(d - cam_back_m)
	var ahead := _pos_xz_at(d + look_ahead_m)
	var ground := _smooth_ground(d)
	cam.global_position = Vector3(behind.x, ground + cam_height, behind.z)
	var target := Vector3(ahead.x, ground + aim_height, ahead.z)
	if absf(behind.x - ahead.x) + absf(behind.z - ahead.z) > 0.1:
		cam.look_at(target, Vector3.UP)


# --- main loop --------------------------------------------------------------

func _process(delta: float) -> void:
	while udp.get_available_packet_count() > 0:
		var line := udp.get_packet().get_string_from_utf8()
		var msg = JSON.parse_string(line)
		if typeof(msg) == TYPE_DICTIONARY and msg.has("distance_m"):
			dist = float(msg.distance_m)
			cur_speed = float(msg.get("speed_mps", cur_speed))
			live = true

	if live:
		dist += cur_speed * delta          # dead-reckon between packets
	elif not paused:
		dist += demo_speed * delta

	if dist >= route_len:
		dist = 0.0                          # loop the demo
		seg_i = 0
	_update_camera(dist)
	_update_avatar(dist)

	if minimap != null and minimap_layer.visible:
		var here := _pos_xz_at(dist)
		var fwd := _pos_xz_at(dist + 8.0)
		minimap.set_marker(-here.x, here.z, -(fwd.x - here.x), fwd.z - here.z)


func _unhandled_input(e: InputEvent) -> void:
	var k := e as InputEventKey
	if k == null or not k.pressed or k.echo:
		return
	match k.keycode:
		KEY_ESCAPE:
			get_tree().quit()
		KEY_SPACE:
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
			head_px = pos_px + h.normalized() * 12.0
		has_pos = true
		if pos_px.distance_to(_last_px) > 1.5:
			_last_px = pos_px
			queue_redraw()

	func _draw() -> void:
		draw_rect(Rect2(Vector2.ZERO, msize), Color(0.05, 0.05, 0.07, 0.55))
		draw_rect(Rect2(Vector2.ZERO, msize), Color(1, 1, 1, 0.15), false, 1.0)
		if route_px.size() > 1:
			draw_polyline(route_px, Color(0.95, 0.82, 0.15), 1.5)
		draw_circle(start_px, 4.0, Color(0.2, 0.9, 0.2))
		draw_circle(end_px, 3.5, Color(0.1, 0.1, 0.1))
		if has_pos:
			if head_px != pos_px:
				draw_line(pos_px, head_px, Color(1, 1, 1, 0.9), 2.0)
			draw_circle(pos_px, 4.5, Color(0.95, 0.2, 0.2))
		draw_string(ThemeDB.fallback_font, Vector2(msize.x * 0.5 - 4.0, 15.0), "N",
				HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(1, 1, 1, 0.7))
