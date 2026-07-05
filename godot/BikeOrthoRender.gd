extends Node3D

# Standalone diagnostic: renders rider.glb in TRUE ORTHOGRAPHIC side view (zero
# perspective — flatter than any long lens) sitting on a plane whose top edge is
# exactly y=0. The gap between the tire bottoms and that ground line is the bake
# error to correct. Prints the model's world AABB (min.y = how far the lowest
# point sits off the ground) and saves bike_side.png in the repo root.
#
# Run:  /opt/homebrew/bin/godot4 --path godot res://BikeOrthoRender.tscn
# (windowed — the game renderer needs a real swapchain to grab an image).

@export var glb_path: String = "res://assets/rider.glb"
@export var out_png: String = "res://../bike_side.png"

func _ready() -> void:
	var envp := OS.get_environment("RIDESIM_GLB")
	if envp != "": glb_path = envp
	var envo := OS.get_environment("RIDESIM_OUT")
	if envo != "": out_png = envo
	DisplayServer.window_set_size(Vector2i(1600, 1000))

	# ground SLAB: a solid box whose TOP face is exactly y=0. Tires should rest on
	# it; any gap (float) or overlap (sink) is then unmistakable in the side view.
	var slab := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(10, 0.6, 10)
	slab.mesh = bm
	slab.position.y = -0.3            # top face at y=0
	var gmat := StandardMaterial3D.new()
	gmat.albedo_color = Color(0.42, 0.46, 0.40)
	slab.material_override = gmat
	add_child(slab)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-32, 35, 0)
	sun.light_energy = 1.3
	add_child(sun)
	var we := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.90, 0.92, 0.95)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.65, 0.65, 0.68)
	env.ambient_light_energy = 1.0
	we.environment = env
	add_child(we)

	if not ResourceLoader.exists(glb_path):
		push_error("missing " + glb_path)
		get_tree().quit(1)
		return
	var bike := (load(glb_path) as PackedScene).instantiate() as Node3D
	add_child(bike)

	var ab := _world_aabb(bike)
	var miny := ab.position.y
	var maxy := ab.position.y + ab.size.y
	print("AABB  min=", ab.position, "  size=", ab.size)
	print("  lowest point y = %.4f m  (gap above ground; should be ~0)" % miny)
	print("  height = %.4f m   length(z) = %.4f m" % [maxy - miny, ab.size.z])

	# Per-mesh world min.y, so we can name the part that defines the floor and the
	# tires' true bottom. The gap to fix = (tire min.y) - (global min.y).
	var rows := _mesh_min_ys(bike)
	rows.sort_custom(func(a, b): return a.y < b.y)
	print("--- lowest 8 parts (world min.y) ---")
	for i in mini(8, rows.size()):
		print("  %+0.4f  %s" % [rows[i].y, rows[i].name])
	# tire geometry: each tire's center (x,y,z) and bottom, to compute the pitch
	print("--- tire meshes (world center & bottom) ---")
	var tcenters := []
	for n in _all_mesh(bike):
		if String(n.name).to_lower().contains("tire"):
			var a := _one_aabb(n)
			var c := a.position + a.size * 0.5
			print("  %-28s center=(%.3f, %.3f, %.3f)  bottom=%.4f" % [n.name, c.x, c.y, c.z, a.position.y])
			tcenters.append(c)
	if tcenters.size() >= 2:
		# sort by z (forward = -Z), so [0]=front, last=rear
		tcenters.sort_custom(func(a, b): return a.z < b.z)
		var fwd: Vector3 = tcenters[0]
		var rear: Vector3 = tcenters[tcenters.size() - 1]
		var dz: float = rear.z - fwd.z
		var dy: float = rear.y - fwd.y
		var pitch_deg: float = rad_to_deg(atan2(dy, absf(dz)))
		print("--- hub-line pitch about X = %.3f deg (rear %.4f vs front %.4f over %.3f m) ---" % [pitch_deg, rear.y, fwd.y, absf(dz)])
	var tire_min := INF
	var tire_name := ""
	for r in rows:
		if r.name.to_lower().contains("tire"):
			if r.y < tire_min:
				tire_min = r.y; tire_name = r.name
	if tire_min < INF:
		print("--- lowest tire bottom = %.4f m (%s) ---" % [tire_min, tire_name])

	# a second hairline (cyan) at the tire bottom so the gap is visible in the png
	if tire_min < INF and tire_min > 0.001:
		var tl := MeshInstance3D.new()
		var tlm := BoxMesh.new()
		tlm.size = Vector3(8, 0.003, 0.003)
		tl.mesh = tlm
		tl.position.y = tire_min
		var tmat := StandardMaterial3D.new()
		tmat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		tmat.albedo_color = Color(0, 1, 1)
		tl.material_override = tmat
		add_child(tl)

	# Ortho camera looking along -X (drive side). Fixed framing (NOT the AABB, which
	# glTF accessor bounds can inflate): y=-0.25..1.95, centered on the wheelbase.
	var cam := Camera3D.new()
	cam.projection = Camera3D.PROJECTION_ORTHOGONAL
	var cz := -0.05                          # wheelbase z-center (≈ -0.5..0.5)
	var frame_h := 2.2                       # world metres top-to-bottom
	cam.size = frame_h
	cam.keep_aspect = Camera3D.KEEP_HEIGHT
	cam.near = 0.05
	cam.far = 200.0
	add_child(cam)
	cam.look_at_from_position(Vector3(30.0, 0.85, cz), Vector3(0.0, 0.85, cz), Vector3.UP)
	cam.make_current()

	await get_tree().process_frame
	await get_tree().process_frame
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var err := img.save_png(out_png)
	print("save_png(", out_png, ") -> ", err)
	get_tree().quit()

func _all_mesh(root: Node) -> Array:
	var out := []
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		for c in n.get_children():
			stack.push_back(c)
		if n is MeshInstance3D and (n as MeshInstance3D).mesh != null:
			out.append(n)
	return out

func _one_aabb(mi: MeshInstance3D) -> AABB:
	var local := mi.get_aabb()
	var xf := mi.global_transform
	var out := AABB()
	for i in 8:
		var corner := local.position + Vector3(
			local.size.x if (i & 1) else 0.0,
			local.size.y if (i & 2) else 0.0,
			local.size.z if (i & 4) else 0.0)
		var w := xf * corner
		if i == 0:
			out = AABB(w, Vector3.ZERO)
		else:
			out = out.expand(w)
	return out

func _mesh_min_ys(root: Node) -> Array:
	var rows := []
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		for c in n.get_children():
			stack.push_back(c)
		if n is MeshInstance3D and (n as MeshInstance3D).mesh != null:
			var mi := n as MeshInstance3D
			var local := mi.get_aabb()
			var xf := mi.global_transform
			var lo := INF
			for i in 8:
				var corner := local.position + Vector3(
					local.size.x if (i & 1) else 0.0,
					local.size.y if (i & 2) else 0.0,
					local.size.z if (i & 4) else 0.0)
				lo = minf(lo, (xf * corner).y)
			rows.append({"name": String(mi.name), "y": lo})
	return rows

func _world_aabb(root: Node) -> AABB:
	var out := AABB()
	var first := true
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		for c in n.get_children():
			stack.push_back(c)
		if n is MeshInstance3D and (n as MeshInstance3D).mesh != null:
			var mi := n as MeshInstance3D
			var local := mi.get_aabb()
			var xf := mi.global_transform
			# transform all 8 corners to world, merge
			for i in 8:
				var corner := local.position + Vector3(
					local.size.x if (i & 1) else 0.0,
					local.size.y if (i & 2) else 0.0,
					local.size.z if (i & 4) else 0.0)
				var w := xf * corner
				if first:
					out = AABB(w, Vector3.ZERO); first = false
				else:
					out = out.expand(w)
	return out
