extends Node3D
# Interactive ortho pedal-stroke inspector for verifying LegRig pedaling intent.
#   Scroll wheel      : advance / retreat the crank (5° per notch)
#   Shift + scroll    : zoom the ortho camera
#   Space             : toggle auto-pedal at 10 rpm (slow demo cadence)
#   Left / Right      : nudge crank 1°
#   T                 : toggle camera tracking (near pedal <-> bottom-bracket)
#   R                 : reset crank to 0°
#   Esc               : quit
# Load any rig with:  RIDESIM_GLB=res://assets/female_opt.glb Godot --path godot res://PedalInspect.tscn

const RPM := 10.0
const STEP_DEG := 5.0
const CRANK_SIGN := -1.0     # match Main._drive_crank (crank_spin_sign)

var _rider: Node3D
var _rig: LegRig
var _crank: Node3D
var _crank_rest: Basis
var _near_pedal: Node3D
var _bb: Vector3
var _crank_deg := 0.0
var _cam_size := 0.9
var _auto := false
var _track := true
var _cam: Camera3D
var _label: Label
var _glb_name := ""

func _ready() -> void:
	DisplayServer.window_set_size(Vector2i(1400, 1000))
	_build_stage()
	var glb := OS.get_environment("RIDESIM_GLB") if OS.has_environment("RIDESIM_GLB") else "res://assets/male_opt.glb"
	if not ResourceLoader.exists(glb):
		push_warning("PedalInspect: '%s' not found — falling back to male_opt.glb" % glb)
		glb = "res://assets/male_opt.glb"
	_glb_name = glb.get_file()
	_rider = (load(glb) as PackedScene).instantiate() as Node3D
	add_child(_rider)
	_rig = LegRig.new()
	var ok := _rig.setup(_rider)
	_crank = _find(_rider, "crank")
	if _crank != null:
		_crank_rest = _crank.transform.basis
		_bb = _crank.global_position
	_near_pedal = _find(_rider, "pedal_r_asm")   # drive side (nearest the camera)
	if _near_pedal == null:
		_near_pedal = _find(_rider, "pedal")

	_cam = Camera3D.new()
	_cam.projection = Camera3D.PROJECTION_ORTHOGONAL
	_cam.near = 0.05; _cam.far = 200.0
	add_child(_cam)
	_apply()   # pose at 0° and frame

	var cl := CanvasLayer.new(); add_child(cl)
	_label = Label.new()
	_label.position = Vector2(16, 12)
	_label.add_theme_color_override("font_color", Color.BLACK)
	_label.add_theme_font_size_override("font_size", 20)
	cl.add_child(_label)
	print("PedalInspect: LegRig ok=", ok, " legs=", _rig.legs.size(), "  glb=", glb)
	_update_label()

func _build_stage() -> void:
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
	env.ambient_light_color = Color(0.66, 0.66, 0.70); env.ambient_light_energy = 1.0
	we.environment = env; add_child(we)

func _process(delta: float) -> void:
	if _auto:
		_crank_deg = fposmod(_crank_deg + RPM / 60.0 * 360.0 * delta, 360.0)
		_apply()

func _zoom(f: float) -> void:
	_cam_size = clampf(_cam_size * f, 0.2, 3.0)
	_apply()

func _unhandled_input(e: InputEvent) -> void:
	if e is InputEventMouseButton and e.pressed:
		# read the modifier off the event itself (Input.is_key_pressed is unreliable at
		# scroll time on macOS). Also map the MX Master thumb wheel (horizontal) to zoom.
		var mb := e as InputEventMouseButton
		match mb.button_index:
			MOUSE_BUTTON_WHEEL_UP:
				if mb.shift_pressed: _zoom(0.9)
				else: _crank_deg = fposmod(_crank_deg + STEP_DEG, 360.0); _apply()
			MOUSE_BUTTON_WHEEL_DOWN:
				if mb.shift_pressed: _zoom(1.1)
				else: _crank_deg = fposmod(_crank_deg - STEP_DEG, 360.0); _apply()
			MOUSE_BUTTON_WHEEL_LEFT: _zoom(1.1)     # thumb wheel = zoom out
			MOUSE_BUTTON_WHEEL_RIGHT: _zoom(0.9)    # thumb wheel = zoom in
	elif e is InputEventKey and e.pressed:
		match (e as InputEventKey).keycode:
			KEY_SPACE: _auto = not _auto
			KEY_LEFT: _crank_deg = fposmod(_crank_deg - 1.0, 360.0); _apply()
			KEY_RIGHT: _crank_deg = fposmod(_crank_deg + 1.0, 360.0); _apply()
			KEY_EQUAL, KEY_KP_ADD: _zoom(0.9)       # '+' zoom in
			KEY_MINUS, KEY_KP_SUBTRACT: _zoom(1.1)  # '-' zoom out
			KEY_T: _track = not _track; _apply()
			KEY_R: _crank_deg = 0.0; _apply()
			KEY_ESCAPE: get_tree().quit()

func _apply() -> void:
	if _crank != null:
		_crank.transform.basis = _crank_rest * Basis(Vector3.RIGHT, deg_to_rad(_crank_deg) * CRANK_SIGN)
		_rider.force_update_transform()
	if _rig != null:
		_rig.pose()
	# camera: ortho drive-side, tracking the near pedal (or the BB)
	var t := _bb
	if _track and _near_pedal != null:
		_rider.force_update_transform()
		t = _near_pedal.global_position
	_cam.size = _cam_size
	_cam.look_at_from_position(Vector3(30.0, t.y, t.z), Vector3(0.0, t.y, t.z), Vector3.UP)
	_cam.make_current()
	_update_label()

func _update_label() -> void:
	if _label == null: return
	_label.text = "MODEL: %s\ncrank %3d°   zoom %.2f   %s   track:%s\nscroll=advance  zoom: shift+scroll / thumbwheel / +- keys  space=auto10rpm  L/R=1°  T=track  R=reset" % [
		_glb_name, int(round(_crank_deg)), _cam_size, ("AUTO 10rpm" if _auto else "manual"), ("pedal" if _track else "BB")]

func _find(root: Node, token: String) -> Node3D:
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		for c in n.get_children():
			stack.push_back(c)
		if n is Node3D and String(n.name).to_lower().contains(token):
			return n
	return null
