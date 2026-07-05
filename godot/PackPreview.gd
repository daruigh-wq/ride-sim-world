extends Node3D
# Static preview of the varied mixed pack: N riders alternating male/female with the same
# per-rider jersey/frame tint the peloton uses, lined up so you can see the variety.
# Run: RIDESIM_PACK_N=8 Godot --path godot res://PackPreview.tscn

const JERSEY_MATS := ["kit_jersey", "kit_burgundy"]
const FRAME_MATS := ["kit_frame", "candy apple", "kit_red"]
const MODELS := ["res://assets/male_opt.glb", "res://assets/female_opt.glb"]

func _ready() -> void:
	DisplayServer.window_set_size(Vector2i(1800, 900))
	var slab := MeshInstance3D.new()
	var bm := BoxMesh.new(); bm.size = Vector3(40, 0.6, 6); slab.mesh = bm; slab.position.y = -0.3
	var gmat := StandardMaterial3D.new(); gmat.albedo_color = Color(0.42, 0.46, 0.40)
	slab.material_override = gmat; add_child(slab)
	var sun := DirectionalLight3D.new(); sun.rotation_degrees = Vector3(-38, 28, 0)
	sun.light_energy = 1.4; add_child(sun)
	var we := WorldEnvironment.new(); var env := Environment.new()
	env.background_mode = Environment.BG_COLOR; env.background_color = Color(0.90, 0.92, 0.95)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.66, 0.66, 0.70); env.ambient_light_energy = 1.0
	we.environment = env; add_child(we)

	var n := int(OS.get_environment("RIDESIM_PACK_N")) if OS.has_environment("RIDESIM_PACK_N") else 8
	var span := 1.0
	for i in n:
		var r := (load(MODELS[i % MODELS.size()]) as PackedScene).instantiate() as Node3D
		r.position = Vector3((float(i) - (n - 1) / 2.0) * span, 0.0, 0.0)
		r.rotation_degrees = Vector3(0, 18, 0)     # slight 3/4 so jersey + frame both read
		add_child(r)
		var jhue := fposmod(float(i) * 0.61803398875, 1.0)
		_tint(r, JERSEY_MATS, Color.from_hsv(jhue, 0.62, 0.82))
		_tint(r, FRAME_MATS, Color.from_hsv(fposmod(jhue + 0.42, 1.0), 0.5, 0.42))

	var cam := Camera3D.new()
	cam.projection = Camera3D.PROJECTION_ORTHOGONAL
	cam.size = 2.1; cam.near = 0.05; cam.far = 200.0; add_child(cam)
	var w := float(n) * span
	cam.look_at_from_position(Vector3(0, 0.95, w * 0.9 + 4.0), Vector3(0, 0.95, 0), Vector3.UP)
	cam.size = maxf(2.2, w * 0.62)
	cam.make_current()
	await get_tree().process_frame
	await get_tree().process_frame
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	print("PackPreview n=", n, " save -> ", img.save_png("res://../pack_preview.png"))
	get_tree().quit()

func _tint(root: Node3D, tokens: Array, color: Color) -> void:
	for mi in root.find_children("*", "MeshInstance3D", true, false):
		var m := mi as MeshInstance3D
		if m.mesh == null: continue
		for s in m.mesh.get_surface_count():
			var src := m.get_active_material(s)
			if src == null: continue
			var nm := src.resource_name.to_lower()
			for t in tokens:
				if nm.contains(t):
					var nmat := (src as StandardMaterial3D).duplicate() if src is StandardMaterial3D else StandardMaterial3D.new()
					nmat.albedo_color = color
					m.set_surface_override_material(s, nmat)
					break
