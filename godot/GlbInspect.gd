extends Node3D
# Headless GLB inspector. Point it at any GLB via env RIDESIM_GLB (res:// or abs path).
# Prints: full node tree with type, per-mesh world AABB, surface count + material
# names/albedo (the color-region info), tire/wheel/crank rig-name hits, and the
# overall world AABB (scale + tire-gap). No window needed.
#
# Run: /Applications/Godot.app/Contents/MacOS/Godot --headless --path godot res://GlbInspect.tscn

func _ready() -> void:
	var path := OS.get_environment("RIDESIM_GLB")
	if path == "":
		path = "res://assets/rider.glb"
	print("=== GLB INSPECT: ", path, " ===")
	if not ResourceLoader.exists(path):
		push_error("missing " + path); get_tree().quit(1); return
	var root := (load(path) as PackedScene).instantiate() as Node3D
	add_child(root)

	print("\n--- NODE TREE (name [Type] mesh:surfaces) ---")
	_tree(root, 0)

	print("\n--- MATERIALS (unique, by name -> albedo) ---")
	var mats := {}
	for mi in _all_mesh(root):
		var m: Mesh = (mi as MeshInstance3D).mesh
		for s in m.get_surface_count():
			var mat := (mi as MeshInstance3D).get_active_material(s)
			var nm := "<null>"
			var alb := "?"
			if mat != null:
				nm = mat.resource_name if mat.resource_name != "" else str(mat)
				if mat is StandardMaterial3D:
					var c: Color = (mat as StandardMaterial3D).albedo_color
					alb = "(%.2f,%.2f,%.2f)" % [c.r, c.g, c.b]
			if not mats.has(nm):
				mats[nm] = {"albedo": alb, "count": 0}
			mats[nm]["count"] += 1
	for k in mats:
		print("  %-40s albedo=%s  x%d surfaces" % [k, mats[k]["albedo"], mats[k]["count"]])
	print("  (", mats.size(), " unique materials)")

	print("\n--- BODY/KIT PARTS (mesh -> material) ---")
	for mi in _all_mesh(root):
		var lo := String(mi.name).to_lower()
		var toks := ["torso","trunk","pelvis","jersey","body","chest","uprarm","forearm","hand","thigh","calf","foot","toe","head","hair","sponge","short","shoe","fork","bar","stem","seat","saddle","parlee","ouray"]
		var hit := false
		for t in toks:
			if lo.contains(t): hit = true; break
		if not hit: continue
		var m: Mesh = (mi as MeshInstance3D).mesh
		var mn := ""
		for s in m.get_surface_count():
			var mat := (mi as MeshInstance3D).get_active_material(s)
			mn += (mat.resource_name if mat != null and mat.resource_name != "" else "<?>") + " "
		print("  %-30s -> %s" % [mi.name, mn])

	print("\n--- RIG NAME HITS (wheel/tire/crank/cassette/fork) ---")
	for mi in _all_mesh(root):
		var lo := String(mi.name).to_lower()
		for key in ["wheel", "tire", "crank", "cassette", "cs-", "fork", "pedal", "foot", "shoe", "leg"]:
			if lo.contains(key):
				var a := _one_aabb(mi)
				var c := a.position + a.size * 0.5
				print("  %-32s [%s] center=(%.3f,%.3f,%.3f)" % [mi.name, key, c.x, c.y, c.z])
				break

	# duplicate node-name histogram + triangle totals
	print("\n--- NODE-NAME DUPLICATES (name -> count, >1 only) ---")
	var names := {}
	var stack2: Array[Node] = [root]
	while not stack2.is_empty():
		var n: Node = stack2.pop_back()
		for c in n.get_children(): stack2.push_back(c)
		var nm := String(n.name)
		names[nm] = int(names.get(nm, 0)) + 1
	var dups := []
	for k in names:
		if int(names[k]) > 1: dups.append([k, int(names[k])])
	dups.sort_custom(func(a, b): return a[1] > b[1])
	for d in dups:
		print("  x%-4d %s" % [d[1], d[0]])
	print("  (", dups.size(), " names appear more than once)")

	print("\n--- TRIANGLE / SURFACE TOTALS ---")
	var tot_tris := 0
	var tot_verts := 0
	var mesh_res := {}      # unique Mesh resource -> [tris, instances]  (detects shared vs duplicated)
	for mi in _all_mesh(root):
		var m: Mesh = (mi as MeshInstance3D).mesh
		var tris := 0
		var verts := 0
		for s in m.get_surface_count():
			var arr := m.surface_get_arrays(s)
			var vc := (arr[Mesh.ARRAY_VERTEX] as PackedVector3Array).size()
			verts += vc
			var idx: PackedInt32Array = arr[Mesh.ARRAY_INDEX] if arr[Mesh.ARRAY_INDEX] != null else PackedInt32Array()
			tris += (idx.size() / 3) if idx.size() > 0 else (vc / 3)
		tot_tris += tris
		tot_verts += verts
		var rid := m.get_rid()
		if not mesh_res.has(rid): mesh_res[rid] = [tris, 0, String(mi.name)]
		mesh_res[rid][1] += 1
	print("  total triangles = ", tot_tris, "   total verts = ", tot_verts)
	print("  mesh instances = ", _all_mesh(root).size(), "   unique Mesh resources = ", mesh_res.size())
	# heaviest single meshes
	var heavy := []
	for rid in mesh_res: heavy.append(mesh_res[rid])
	heavy.sort_custom(func(a, b): return a[0] * a[1] > b[0] * b[1])
	print("  --- heaviest meshes (tris x instances, name) ---")
	for i in mini(12, heavy.size()):
		print("    %8d tris x%d  %s" % [heavy[i][0], heavy[i][1], heavy[i][2]])

	var ab := _world_aabb(root)
	print("\n--- WORLD AABB ---")
	print("  min=", ab.position, "  size=", ab.size)
	print("  height(y)=%.4f  length(z)=%.4f  width(x)=%.4f" % [ab.size.y, ab.size.z, ab.size.x])
	print("  lowest y = %.4f (tire gap above ground; want ~0)" % ab.position.y)
	print("  surface/mesh count = ", _all_mesh(root).size())
	get_tree().quit()

func _tree(n: Node, depth: int) -> void:
	var pad := ""
	for i in depth: pad += "  "
	var extra := ""
	if n is MeshInstance3D and (n as MeshInstance3D).mesh != null:
		extra = " mesh:%d surf" % (n as MeshInstance3D).mesh.get_surface_count()
	print("%s%s [%s]%s" % [pad, n.name, n.get_class(), extra])
	if depth < 6:
		for c in n.get_children(): _tree(c, depth + 1)

func _all_mesh(root: Node) -> Array:
	var out := []
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		for c in n.get_children(): stack.push_back(c)
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
		if i == 0: out = AABB(w, Vector3.ZERO)
		else: out = out.expand(w)
	return out

func _world_aabb(root: Node) -> AABB:
	var out := AABB()
	var first := true
	for mi in _all_mesh(root):
		var a := _one_aabb(mi)
		if first: out = a; first = false
		else: out = out.merge(a)
	return out
