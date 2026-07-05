extends Node3D
func _ready() -> void:
	var n := (load("res://assets/rider.glb") as PackedScene).instantiate()
	add_child(n); _walk(n)
	get_tree().quit()
func _walk(node: Node) -> void:
	var nm := String(node.name).to_lower()
	if nm.contains("cassette") or nm.contains("cs-") or nm.contains("cs_") or nm.contains("r9200") or nm.contains("wheel_tire") or nm.contains("cog") or nm.contains("sprocket") or nm.contains("freehub") or nm.contains("11-34"):
		var p := (node as Node3D).global_position if node is Node3D else Vector3.ZERO
		print("  ", node.name, "  gpos=(%.3f,%.3f,%.3f)" % [p.x,p.y,p.z])
	for c in node.get_children(): _walk(c)
