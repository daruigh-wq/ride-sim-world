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
@export var eye_height: float = 1.6      # camera height above road, metres
@export var look_ahead_m: float = 25.0   # camera aims this far down-route
@export var road_width: float = 6.0

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


func _ready() -> void:
	_load_data()
	_build_terrain()
	_build_road()
	_build_camera_and_sky()
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
	print("route %.2f km, terrain %d x %d cells" % [route_len / 1000.0, gw, gh])


# --- terrain sampling -------------------------------------------------------

func _terrain_y(x: float, z: float) -> float:
	var c := (x - x0) / mpp_x
	var r := (z - z0) / mpp_z
	c = clampf(c, 0.0, float(gw - 1) - 0.001)
	r = clampf(r, 0.0, float(gh - 1) - 0.001)
	var ci := int(c); var ri := int(r)
	var fx := c - ci; var fz := r - ri
	var h00 := heights[ri * gw + ci]
	var h10 := heights[ri * gw + ci + 1]
	var h01 := heights[(ri + 1) * gw + ci]
	var h11 := heights[(ri + 1) * gw + ci + 1]
	return lerp(lerp(h00, h10, fx), lerp(h01, h11, fx), fz)


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
			verts[i] = Vector3(x0 + c * mpp_x, y, z0 + r * mpp_z)
			cols[i] = _elev_color(y)
			# normal from heightfield central differences
			var cl := maxi(c - 1, 0); var cr := mini(c + 1, gw - 1)
			var ru := maxi(r - 1, 0); var rd := mini(r + 1, gh - 1)
			var dydx := (heights[r * gw + cr] - heights[r * gw + cl]) / ((cr - cl) * mpp_x)
			var dydz := (heights[rd * gw + c] - heights[ru * gw + c]) / ((rd - ru) * mpp_z)
			norms[i] = Vector3(-dydx, 1.0, -dydz).normalized()

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
	mesh.surface_set_material(0, mat)

	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	add_child(mi)


func _build_road() -> void:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var hw := road_width * 0.5
	for i in range(pts.size() - 1):
		var p0 := Vector3(pts[i].x, 0, pts[i].z)
		var p1 := Vector3(pts[i + 1].x, 0, pts[i + 1].z)
		var dir := (p1 - p0)
		if dir.length() < 0.01:
			continue
		dir = dir.normalized()
		var perp := Vector3(dir.z, 0, -dir.x)
		var a := p0 - perp * hw
		var b := p0 + perp * hw
		var c := p1 - perp * hw
		var d := p1 + perp * hw
		a.y = _terrain_y(a.x, a.z) + 0.25
		b.y = _terrain_y(b.x, b.z) + 0.25
		c.y = _terrain_y(c.x, c.z) + 0.25
		d.y = _terrain_y(d.x, d.z) + 0.25
		for v in [a, c, b, b, c, d]:
			st.add_vertex(v)
	st.generate_normals()
	var mesh := st.commit()
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.17, 0.17, 0.19)
	mat.roughness = 0.95
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
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	cam = Camera3D.new()
	cam.fov = 70.0
	cam.far = 12000.0
	add_child(cam)
	_update_camera(0.0)


# --- distance -> world position --------------------------------------------

func _pos_xz_at(d: float) -> Vector3:
	d = clampf(d, 0.0, route_len)
	# advance/rewind cached segment so pts[seg_i].d <= d <= pts[seg_i+1].d
	while seg_i < pts.size() - 2 and float(pts[seg_i + 1].d) < d:
		seg_i += 1
	while seg_i > 0 and float(pts[seg_i].d) > d:
		seg_i -= 1
	var p0 = pts[seg_i]; var p1 = pts[seg_i + 1]
	var span := float(p1.d) - float(p0.d)
	var t := 0.0 if span < 0.001 else (d - float(p0.d)) / span
	var x := lerp(float(p0.x), float(p1.x), t)
	var z := lerp(float(p0.z), float(p1.z), t)
	return Vector3(x, 0, z)


func _update_camera(d: float) -> void:
	var here := _pos_xz_at(d)
	var ahead := _pos_xz_at(d + look_ahead_m)
	here.y = _terrain_y(here.x, here.z) + eye_height
	ahead.y = _terrain_y(ahead.x, ahead.z) + eye_height
	cam.global_position = here
	if here.distance_to(ahead) > 0.1:
		cam.look_at(ahead, Vector3.UP)


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


func _unhandled_input(e: InputEvent) -> void:
	if e is InputEventKey and e.pressed and not e.echo:
		match e.keycode:
			KEY_ESCAPE:
				get_tree().quit()
			KEY_SPACE:
				paused = not paused
			KEY_EQUAL, KEY_KP_ADD:
				demo_speed = minf(demo_speed + 1.0, 30.0)
			KEY_MINUS, KEY_KP_SUBTRACT:
				demo_speed = maxf(demo_speed - 1.0, 1.0)
