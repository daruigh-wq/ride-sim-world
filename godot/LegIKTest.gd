extends Node3D
# Visual test for LegRig: pose the legs at a crank angle (env RIDESIM_CRANK_DEG,
# default 90) and render an ortho drive-side view to bike_side_ik.png.
# Run: RIDESIM_CRANK_DEG=90 Godot --path godot res://LegIKTest.tscn

func _ready() -> void:
	DisplayServer.window_set_size(Vector2i(1600, 1000))
	# ground slab, top face at y=0
	var slab := MeshInstance3D.new()
	var bm := BoxMesh.new(); bm.size = Vector3(10, 0.6, 10)
	slab.mesh = bm; slab.position.y = -0.3
	var gmat := StandardMaterial3D.new(); gmat.albedo_color = Color(0.42, 0.46, 0.40)
	slab.material_override = gmat; add_child(slab)
	var sun := DirectionalLight3D.new(); sun.rotation_degrees = Vector3(-32, 35, 0)
	sun.light_energy = 1.3; add_child(sun)
	var we := WorldEnvironment.new(); var env := Environment.new()
	env.background_mode = Environment.BG_COLOR; env.background_color = Color(0.90, 0.92, 0.95)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.65, 0.65, 0.68); env.ambient_light_energy = 1.0
	we.environment = env; add_child(we)

	var glb := OS.get_environment("RIDESIM_GLB") if OS.has_environment("RIDESIM_GLB") else "res://assets/rider.glb"
	var rider := (load(glb) as PackedScene).instantiate() as Node3D
	add_child(rider)

	# set up the IK at rest, THEN rotate the crank to the test angle
	var rig := LegRig.new()
	var ok := rig.setup(rider)
	print("LegRig.setup ok=", ok, " legs=", rig.legs.size())
	for L in rig.legs:
		var fo: Vector3 = L.foot_off          # ankle_rest - pedal_rest (rider-local)
		# toe points along ft = -foot_off; pitch below horizontal in the YZ (sagittal) plane
		var pitch := rad_to_deg(atan2(-(-fo.y), Vector2(fo.z, 0.0).length()))
		print("  leg: thigh_len=%.3f calf_len=%.3f ankle=%s foot_off=%s toe_pitch=%.1fdeg pedal=%s" %
			[L.thigh_len, L.calf_len, str(L.ankle_rest), str(fo), pitch, (L.pedal as Node3D).name])

	var crank := _find(rider, "crank")
	var deg := float(OS.get_environment("RIDESIM_CRANK_DEG")) if OS.has_environment("RIDESIM_CRANK_DEG") else 90.0
	if crank != null:
		var rest := crank.transform.basis
		crank.transform.basis = rest * Basis(Vector3.RIGHT, deg_to_rad(deg))
		rider.force_update_transform()
	# spin the cassette the same way Main._drive_crank does (about parent-X / hub axle)
	var cass := _find(rider, "cs-")
	if cass != null:
		print("cassette found: ", cass.name)
		cass.transform.basis = Basis(Vector3.RIGHT, deg_to_rad(deg)) * cass.transform.basis
		rider.force_update_transform()
	rig.pose()
	# POSED foot metrics: after pose(), report each foot's world toe-pitch and the
	# lowest foot point vs its pedal axle (world Y) — the numbers that actually render.
	for L in rig.legs:
		var fnode := L.foot as MeshInstance3D
		var mw := fnode.global_transform
		var lo := INF; var hi := -INF
		var ends := []
		var amin := Vector3(INF, INF, INF); var amax := Vector3(-INF, -INF, -INF)
		for s in fnode.mesh.get_surface_count():
			for v in (fnode.mesh.surface_get_arrays(s)[Mesh.ARRAY_VERTEX] as PackedVector3Array):
				var w: Vector3 = mw * v
				lo = minf(lo, w.y); hi = maxf(hi, w.y)
				amin = amin.min(w); amax = amax.max(w)
		var pedal_y: float = (L.pedal as Node3D).global_position.y
		print("  POSED foot: min_y=%.3f max_y=%.3f pedal_axle_y=%.3f  toe(min)-axle=%+.3f" %
			[lo, hi, pedal_y, lo - pedal_y])

	# optionally hide everything except the leg segments so they read clearly
	if OS.has_environment("RIDESIM_LEGS_ONLY"):
		for mi in rider.find_children("*", "MeshInstance3D", true, false):
			var nm := String(mi.name).to_lower()
			var keep := nm.contains("thigh") or nm.contains("calf") or nm.contains("foot") \
					or nm.contains("toe") or nm.contains("pedal")
			(mi as MeshInstance3D).visible = keep

	var view := OS.get_environment("RIDESIM_VIEW") if OS.has_environment("RIDESIM_VIEW") else "side"
	var cam := Camera3D.new()
	cam.projection = Camera3D.PROJECTION_ORTHOGONAL
	cam.near = 0.05; cam.far = 200.0; add_child(cam)
	var tag := view
	if view == "top":
		# look straight down on the cranks/feet — toe-in shows as feet angling inward
		cam.size = 1.6; add_child(cam)
		cam.look_at_from_position(Vector3(0, 30, 0.05), Vector3(0, 0, 0.05), Vector3(0, 0, -1))
	elif view == "front":
		# look along +z at the front of the rider — shows lateral foot/knee angle
		cam.size = 1.4
		cam.look_at_from_position(Vector3(0, 0.55, -30), Vector3(0, 0.55, 0), Vector3.UP)
	elif view == "foot":
		# tight close-up on the BB/pedal/foot zone to judge pedal-follows-sole
		cam.size = 0.85
		cam.look_at_from_position(Vector3(30, 0.30, 0.0), Vector3(0, 0.30, 0.0), Vector3.UP)
	elif view == "rear":
		# close-up on the rear hub/cassette to confirm it spins in place (no wobble)
		cam.size = 0.55
		cam.look_at_from_position(Vector3(30, 0.34, 0.5), Vector3(0, 0.34, 0.5), Vector3.UP)
	else:
		cam.size = 2.2
		cam.look_at_from_position(Vector3(30, 0.85, -0.05), Vector3(0, 0.85, -0.05), Vector3.UP)
	cam.make_current()
	await get_tree().process_frame
	await get_tree().process_frame
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var name := "res://../bike_%s_ik_%d.png" % [tag, int(deg)]
	print("save ", name, " -> ", img.save_png(name))
	get_tree().quit()

func _find(root: Node, token: String) -> Node3D:
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		for c in n.get_children():
			stack.push_back(c)
		if n is Node3D and String(n.name).to_lower().contains(token):
			return n
	return null
