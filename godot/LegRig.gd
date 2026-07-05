class_name LegRig
extends RefCounted

# Procedural rung-2 leg IK for the rigid-segment rider (no armature/skinning).
# Each leg is hip→knee→ankle as separate rigid meshes. The pedals are children of
# the distance-driven `crank`, so they already orbit correctly; this just reads each
# pedal's live position, sets the ankle there (via a fixed foot offset), solves the
# knee with planar 2-bone IK, and rigidly re-seats thigh/calf/foot between the joints.
#
# All math is done in RIDER-LOCAL space (the glb frame: x=lateral, y=up, z=fore-aft,
# forward=-z), which is static regardless of where the avatar is placed on the route.

const KNEE_POLE := Vector3(0, -0.15, -1.0)   # bend knees forward (+ slightly down)

var _root: Node3D
var legs: Array = []        # one dict per leg (see _build_leg)
var ready := false

# --- setup: call once right after the avatar is spawned (nodes at rest pose) -------
func setup(root: Node3D) -> bool:
	_root = root
	var inv := root.global_transform.affine_inverse()
	# Anchor used only to tell which end of each thigh is the hip (the end nearest
	# this centroid). Prefer a real "pelvis"; fall back to the merged "trunk" body
	# (the static-trunk ship model archived pelvis-1 into trunk-1) or a torso/hip.
	# Trunk's centroid sits higher than the old pelvis but is still well above the
	# knee, so the nearest-end test still picks the hip correctly.
	var pelvis: Node3D = null
	for tok in ["pelvis", "trunk", "torso", "hip"]:
		pelvis = _find(root, [tok])
		if pelvis != null:
			break
	if pelvis == null:
		return false
	var pelvis_c := _centroid_L(pelvis, inv)

	# pedals (children of crank — already orbit with distance)
	var pedals: Array[Node3D] = []
	for n in [_find(root, ["pedal_l_asm"]), _find(root, ["pedal_r_asm"])]:
		if n != null:
			pedals.append(n)
	if pedals.is_empty():
		return false

	# gather the two legs by side token, fall back to geometry if a side is missing.
	# Sides match BOTH the canonical naming matrix ({variant}_{l|r}_{part}, see
	# assets/README.md) and the legacy SolidWorks names (left = "Mirror...", right = "r...").
	for side in ["l_", "r_"]:
		var thigh := _find(root, ["thigh"], side)
		var calf := _find(root, ["calf"], side)
		var foot := _find(root, ["foot"], side)
		if thigh == null or calf == null or foot == null:
			continue
		legs.append(_build_leg(thigh, calf, foot, pelvis_c, pedals, inv))
		# the toe is a separate rigid mesh — ride it along with the foot
		var toe := _find(root, ["toe"], side)
		if toe != null:
			toe.reparent(foot, true)
	ready = legs.size() >= 1
	return ready

func _build_leg(thigh: Node3D, calf: Node3D, foot: Node3D, pelvis_c: Vector3,
		pedals: Array, inv: Transform3D) -> Dictionary:
	var tv := _verts_L(thigh, inv)
	var cv := _verts_L(calf, inv)
	var fv := _verts_L(foot, inv)
	var te := _ends(tv)      # [p0,p1] extreme points along the limb
	var ce := _ends(cv)
	var fe := _ends(fv)
	# hip = thigh end nearer pelvis; knee_t = far end
	var hip: Vector3 = te[0] if te[0].distance_to(pelvis_c) < te[1].distance_to(pelvis_c) else te[1]
	var knee_t: Vector3 = te[1] if hip == te[0] else te[0]
	# calf end nearer knee_t = knee; far = ankle
	var knee_c: Vector3 = ce[0] if ce[0].distance_to(knee_t) < ce[1].distance_to(knee_t) else ce[1]
	var ankle_c: Vector3 = ce[1] if knee_c == ce[0] else ce[0]
	# foot end nearer ankle = ankle side; far = toe
	var ank_f: Vector3 = fe[0] if fe[0].distance_to(ankle_c) < fe[1].distance_to(ankle_c) else fe[1]
	var toe_f: Vector3 = fe[1] if ank_f == fe[0] else fe[0]
	var knee := (knee_t + knee_c) * 0.5
	var ankle := (ankle_c + ank_f) * 0.5

	# pair to the nearest pedal (robust against L/R name confusion)
	var pedal: Node3D = pedals[0]
	var best := INF
	for p in pedals:
		var pl: Vector3 = inv * (p as Node3D).global_position
		var dd := pl.distance_to(ankle)
		if dd < best:
			best = dd; pedal = p
	var pedal_rest_L: Vector3 = inv * pedal.global_position

	return {
		"thigh": thigh, "calf": calf, "foot": foot, "pedal": pedal,
		"hip": hip, "knee_rest": knee, "ankle_rest": ankle, "toe_rest": toe_f,
		"thigh_len": hip.distance_to(knee), "calf_len": knee.distance_to(ankle),
		# lateral (x) offsets along the limb — the leg is planar-about-X, so each
		# segment keeps a fixed small x-splay; the IK solves in the YZ plane.
		"thigh_xoff": knee.x - hip.x, "calf_xoff": ankle.x - knee.x,
		"foot_off": ankle - pedal_rest_L,            # ankle relative to pedal (constant)
		"pedal_rest": pedal_rest_L,                  # pedal position at rest (rider-local)
		"t_rest": _local_xf(thigh, inv),
		"c_rest": _local_xf(calf, inv),
		"f_rest": _local_xf(foot, inv),
		# pedal rest orientation (rider-local) — at rest the platform sits flat under
		# the sole, so applying the foot's about-X angle keeps the pedal parallel to it.
		"pedal_rest_basis": (inv * pedal.global_transform).basis,
	}

# --- per-frame: re-pose both legs from the live pedal positions --------------------
func pose() -> void:
	if not ready:
		return
	var xf := _root.global_transform
	var inv := xf.affine_inverse()
	for L in legs:
		var hip: Vector3 = L.hip
		var pedal_L: Vector3 = inv * (L.pedal as Node3D).global_position
		var ankle_t: Vector3 = pedal_L + L.foot_off
		var knee := _solve2(hip, ankle_t, L.thigh_len, L.calf_len, L.thigh_xoff, L.calf_xoff)
		# thigh: hip fixed, rotate so rest knee-dir aligns to solved knee-dir
		_seat(L.thigh, L.t_rest, hip, hip, L.knee_rest - hip, knee - hip, xf)
		# calf: knee_rest→knee, ankle dir
		_seat(L.calf, L.c_rest, L.knee_rest, knee, L.ankle_rest - L.knee_rest, ankle_t - knee, xf)
		# foot: RIGIDLY attach to the pedal — keep the rest orientation and the rest
		# offset from the pedal, translating only by how far the pedal moved from rest.
		# (The old scheme rotated the foot to point its toe AT the pedal, which mangled any
		# rest pose whose toe didn't already aim there — e.g. the female's authored feet,
		# and the male's L/R sole-angle asymmetry. Rigid attach preserves the authored
		# foot-on-pedal pose exactly, so the sole rides the pedal all the way round.)
		var delta: Vector3 = pedal_L - L.pedal_rest
		L.foot.global_transform = xf * Transform3D(L.f_rest.basis, L.f_rest.origin + delta)
		# pedal keeps its authored (level) rest orientation under the now-fixed sole
		_orient_pedal(L.pedal, L.pedal_rest_basis, 0.0, xf)

# rigidly re-seat a segment with a PURE rotation about world-X (the SolidWorks joint
# axis: all leg joints are normal to the sagittal/YZ plane). Using shortest-arc here
# would tilt the segment's lateral axis and toe the lower leg/foot inward. The angle
# is the signed YZ-plane angle from dir_rest to dir_tgt; x-components are preserved.
func _seat(node: Node3D, rest_L: Transform3D, pivot_rest: Vector3, pivot_tgt: Vector3,
		dir_rest: Vector3, dir_tgt: Vector3, root_xf: Transform3D) -> void:
	var ar := dir_rest
	var at := dir_tgt
	if Vector2(ar.y, ar.z).length() < 1e-5 or Vector2(at.y, at.z).length() < 1e-5:
		return
	var theta := atan2(ar.y * at.z - ar.z * at.y, ar.y * at.y + ar.z * at.z)
	var rb := Basis(Vector3.RIGHT, theta)
	var new_basis := rb * rest_L.basis
	var new_origin := pivot_tgt - rb * (pivot_rest - rest_L.origin)
	node.global_transform = root_xf * Transform3D(new_basis, new_origin)

# Re-orient a pedal so its platform stays parallel to the sole: set its rider-local
# basis to (rotation-about-X by the foot angle) * rest, converted back through the
# crank parent. Position (orbit) is left untouched — only the spin about the axle.
func _orient_pedal(pedal: Node3D, rest_basis: Basis, theta: float, root_xf: Transform3D) -> void:
	var parent := pedal.get_parent() as Node3D
	if parent == null:
		return
	var desired_rider := Basis(Vector3.RIGHT, theta) * rest_basis
	var desired_global := root_xf.basis * desired_rider
	pedal.transform.basis = parent.global_transform.basis.inverse() * desired_global


# planar (YZ) 2-bone IK with X-projected segment lengths, so the solved knee is
# consistent with the pure-about-X seating above (keeps the leg's fixed x-splay).
func _solve2(hip: Vector3, ankle: Vector3, a_len: float, b_len: float,
		a_xoff: float, b_xoff: float) -> Vector3:
	var a := sqrt(maxf(a_len * a_len - a_xoff * a_xoff, 0.0))   # thigh length in YZ
	var b := sqrt(maxf(b_len * b_len - b_xoff * b_xoff, 0.0))   # calf length in YZ
	var dy := ankle.y - hip.y
	var dz := ankle.z - hip.z
	var d := sqrt(dy * dy + dz * dz)
	d = clampf(d, absf(a - b) + 1e-4, a + b - 1e-4)
	var uy := dy / d
	var uz := dz / d
	var l := (d * d + a * a - b * b) / (2.0 * d)
	var h := sqrt(maxf(a * a - l * l, 0.0))
	# perpendicular in YZ; pick the side that matches the knee pole (forward+down)
	var py := -uz
	var pz := uy
	if py * KNEE_POLE.y + pz * KNEE_POLE.z < 0.0:
		py = -py; pz = -pz
	var ky := hip.y + uy * l + py * h
	var kz := hip.z + uz * l + pz * h
	return Vector3(hip.x + a_xoff, ky, kz)            # knee x fixed by the X-splay

# --- helpers -----------------------------------------------------------------------
func _find(root: Node, tokens: Array, side: String = "") -> Node3D:
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		for c in n.get_children():
			stack.push_back(c)
		if n is Node3D:
			var nm := String(n.name).to_lower()
			var ok := true
			for t in tokens:
				if not nm.contains(t):
					ok = false; break
			if ok and _side_ok(nm, side):
				return n
	return null

# Does this (lowercased) node name belong to the requested side? Accepts the canonical
# matrix grammar — a "_l_" / "_r_" segment anywhere, or an "l_" / "r_" start — plus the
# legacy model names (left = "mirror...", right = anything starting with "r").
func _side_ok(nm: String, side: String) -> bool:
	if side == "":
		return true
	if side == "l_":
		return nm.begins_with("l_") or nm.contains("_l_") or nm.begins_with("mirror")
	return nm.contains("_r_") or (nm.begins_with("r") and not nm.begins_with("mirror"))

func _verts_L(node: Node3D, inv: Transform3D) -> PackedVector3Array:
	var out := PackedVector3Array()
	var mi := node as MeshInstance3D
	if mi == null or mi.mesh == null:
		return out
	var rel := inv * node.global_transform     # mesh-local → rider-local
	for s in mi.mesh.get_surface_count():
		var arr := mi.mesh.surface_get_arrays(s)
		var vs: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		for v in vs:
			out.append(rel * v)
	return out

func _ends(v: PackedVector3Array) -> Array:
	if v.size() < 2:
		return [Vector3.ZERO, Vector3.ZERO]
	# double-farthest-point: approximates the segment's long-axis endpoints
	var p0 := v[0]
	var a := _farthest(v, p0)
	var b := _farthest(v, a)
	var a2 := _farthest(v, b)
	return [a2, b]

func _farthest(v: PackedVector3Array, from: Vector3) -> Vector3:
	var best := from
	var bd := -1.0
	for p in v:
		var dd := p.distance_squared_to(from)
		if dd > bd:
			bd = dd; best = p
	return best

func _centroid_L(node: Node3D, inv: Transform3D) -> Vector3:
	var v := _verts_L(node, inv)
	if v.is_empty():
		return inv * node.global_position
	var s := Vector3.ZERO
	for p in v:
		s += p
	return s / float(v.size())

func _local_xf(node: Node3D, inv: Transform3D) -> Transform3D:
	return inv * node.global_transform
