extends Node3D
# Standalone isolated viewer for assets/rider.glb — flat ground at y=0, neutral
# lighting, slow auto-orbit camera, and forward/up axis markers so we can judge
# orientation, scale, and seating without the full world's terrain/stitching.
# Run:  Godot --path godot res://RiderPreview.tscn

const GLB := "res://assets/rider.glb"

var model: Node3D
var orbit := 0.0
var cam: Camera3D
var radius := 5.0
var height := 1.8

func _ready() -> void:
	# --- ground plane at y = 0 (1 m grid texture) ---
	var plane := MeshInstance3D.new()
	var pm := PlaneMesh.new(); pm.size = Vector2(40, 40)
	plane.mesh = pm
	var gmat := StandardMaterial3D.new()
	gmat.albedo_color = Color(0.30, 0.32, 0.34)
	plane.material_override = gmat
	add_child(plane)
	_add_grid(20)   # 1 m grid lines so scale is readable

	# --- forward/up reference markers (Godot: -Z = forward, +Y = up) ---
	_marker(Vector3(0, 0.05, -2.0), Color(0.1, 1.0, 0.2))  # GREEN = forward (-Z)
	_marker(Vector3(0, 0.05,  2.0), Color(1.0, 0.2, 0.1))  # RED   = behind (+Z)
	_marker(Vector3(2.0, 0.05, 0.0), Color(0.2, 0.5, 1.0)) # BLUE  = right (+X)

	# --- lighting + sky ---
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-50, -35, 0)
	sun.light_energy = 1.2
	add_child(sun)
	var we := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.55, 0.62, 0.72)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.5, 0.5, 0.55)
	env.ambient_light_energy = 0.6
	we.environment = env
	add_child(we)

	# --- the model under test ---
	if ResourceLoader.exists(GLB):
		model = (load(GLB) as PackedScene).instantiate()
		add_child(model)
		var aabb := _scene_aabb(model)
		print("RIDER_AABB pos=", aabb.position, " size=", aabb.size, " end=", aabb.end)
		radius = maxf(aabb.size.length() * 1.4, 4.0)
		height = maxf(aabb.size.y * 1.1, 1.5)
	else:
		push_error("rider.glb not found at " + GLB)

	cam = Camera3D.new()
	add_child(cam)
	print("RIDER_PREVIEW_READY  (green=forward/-Z, red=back/+Z, blue=right/+X)")

func _add_grid(half: int) -> void:
	var im := ImmediateMesh.new()
	var mi := MeshInstance3D.new(); mi.mesh = im
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color(0.45, 0.47, 0.50)
	mat.vertex_color_use_as_albedo = false
	mi.material_override = mat
	im.surface_begin(Mesh.PRIMITIVE_LINES, mat)
	for i in range(-half, half + 1):
		im.surface_add_vertex(Vector3(i, 0.01, -half))
		im.surface_add_vertex(Vector3(i, 0.01,  half))
		im.surface_add_vertex(Vector3(-half, 0.01, i))
		im.surface_add_vertex(Vector3( half, 0.01, i))
	im.surface_end()
	add_child(mi)

func _marker(pos: Vector3, col: Color) -> void:
	var m := MeshInstance3D.new()
	var s := SphereMesh.new(); s.radius = 0.12; s.height = 0.24
	m.mesh = s
	var mat := StandardMaterial3D.new()
	mat.albedo_color = col
	mat.emission_enabled = true; mat.emission = col; mat.emission_energy_multiplier = 1.5
	m.material_override = mat
	m.position = pos
	add_child(m)

func _scene_aabb(n: Node) -> AABB:
	var out := AABB()
	var first := true
	for c in n.find_children("*", "MeshInstance3D", true, false):
		var mi := c as MeshInstance3D
		var a := mi.get_aabb()
		a = mi.global_transform * a
		if first:
			out = a; first = false
		else:
			out = out.merge(a)
	return out

func _process(delta: float) -> void:
	orbit += delta * 0.4
	var x := cos(orbit) * radius
	var z := sin(orbit) * radius
	cam.global_position = Vector3(x, height, z)
	cam.look_at(Vector3(0, height * 0.45, 0), Vector3.UP)
