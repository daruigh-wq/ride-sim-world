extends Node3D
# Verifies the new ZENITH cam + god-zoom by driving the REAL Main camera (does NOT
# override it). Switches to ZENITH, grabs a mid-altitude shot, then a full god-zoom
# (orbit_dist = _god_max) shot so we can see the whole built world.
#   RIDESIM_WORLD_SEEK_KM=12 Godot --path godot res://GodCamTest.tscn

const ZENITH := 4
var main: Node3D
var t := 0.0
var stage := 0
var busy := false        # guard: _grab awaits across frames → don't re-enter a stage

func _ready() -> void:
	DisplayServer.window_set_size(Vector2i(1600, 1000))
	main = (load("res://Main.tscn") as PackedScene).instantiate() as Node3D
	add_child(main)

func _process(delta: float) -> void:
	if main == null or busy: return
	t += delta
	if t < 1.0: return
	busy = true
	match stage:
		0:
			main.set("wait_for_telemetry", false)
			main.call("_set_cam_mode", ZENITH)
			main.set("orbit_dist", 300.0)
			main.call("_apply_god_view")
			await _grab("godcam_zenith_mid")
			var gm: float = main.call("_god_max")
			print("god_max = %.0f m" % gm)
			main.set("orbit_dist", gm)
			main.call("_apply_god_view")
			stage = 1
		1:
			await _grab("godcam_zenith_full")
			get_tree().quit()
	busy = false

func _grab(name: String) -> void:
	await RenderingServer.frame_post_draw
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	print("%s -> %s" % [name, img.save_png("res://../%s.png" % name)])
