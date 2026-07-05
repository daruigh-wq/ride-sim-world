extends Node3D
# Visual check for pedal-induced bike rock: park a camera low behind one pack rider and
# save two frames half a crank cycle apart (~0.34 s at 88 rpm) — the bike should lean
# opposite ways. RIDESIM_WORLD_SEEK_KM/RIDESIM_PELOTON_N as usual.
#   Godot --path godot res://RockTest.tscn → rocktest_{a,b}.png

var main: Node3D
var cam: Camera3D
var t := 0.0
var stage := 0
var next_shot := 6.0     # first shot after settle; second half a crank cycle later
var busy := false

func _ready() -> void:
	DisplayServer.window_set_size(Vector2i(1400, 900))
	main = (load("res://Main.tscn") as PackedScene).instantiate() as Node3D
	add_child(main)
	cam = Camera3D.new()
	cam.fov = 40.0
	add_child(cam)

func _process(_delta: float) -> void:
	if main == null or busy:
		return
	if t == 0.0:
		main.set("wait_for_telemetry", false)
		main.set("paused", false)
	t += get_process_delta_time()
	if t < next_shot:
		return
	var pel: Array = main.get("peloton")
	if pel == null or pel.is_empty():
		return
	var node: Node3D = pel[0]["node"]
	# low 3/4-rear view, close enough that 2° of roll is visible against the frame
	var fwd: Vector3 = -node.global_transform.basis.z
	cam.global_position = node.global_position - fwd * 4.0 + Vector3(0.8, 1.3, 0)
	cam.look_at(node.global_position + Vector3(0, 0.9, 0))
	cam.make_current()
	busy = true
	match stage:
		0:
			await _grab("rocktest_a")
			stage = 1
			next_shot = t + 0.34    # half a crank rev at 88 rpm → opposite lean
		1:
			await _grab("rocktest_b")
			get_tree().quit()
	busy = false

func _grab(name: String) -> void:
	await RenderingServer.frame_post_draw
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	print("%s -> %s" % [name, img.save_png("res://../%s.png" % name)])
