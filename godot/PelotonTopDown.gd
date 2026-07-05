extends Node3D
# Drives the REAL Main scene (world + peloton + sim) for a few seconds, from a top-down
# ortho camera centered on the pack, so we can SEE the preordained lanes + distance-keyed
# weave choreography. Also logs each rider's lateral (offset from pack centerline) per
# frame → prove the motion is smooth (no 60 Hz jitter). Windowed (not --headless) so the
# viewport can be captured, like PackPreview.
#   RIDESIM_WORLD_SEEK_KM=12 RIDESIM_PELOTON_N=15 Godot --path godot res://PelotonTopDown.tscn

var main: Node3D
var cam: Camera3D
var t := 0.0
var shots := [6.0, 12.0, 18.0]   # seconds at which to grab a frame (long enough for passes)
var shot_i := 0
var peak_borrow := 0             # most riders mid-lane-borrow at once (P2 fired if > 0)
var min_lat_gap := 1e9           # smallest lateral gap between any longitudinally-overlapping pair
var closest_pair := ""           # which lanes that closest pair was in
var collision_frames := 0        # frames where some pair clipped (lat gap < footprint)
var pace_max := 0.0              # peak _pack_cruise seen (m/s) — catch the runaway
var prev_rd := {}                # rider → last along-route pos (for surge-speed check)
var rider_v_min := 1e9           # slowest any rider went (m/s); < 0 = rolling backward = bug
var rider_v_max := -1e9          # fastest any rider went (m/s)
var laterals := {}               # rider index → Array of lateral samples (jitter check)
var prev_wpos := {}              # rider index → last world position (crab-angle check)
var crab_sum := 0.0              # Σ |angle(velocity, node forward)| — residual crab AFTER fix
var crab_max := 0.0
var crab_old_sum := 0.0          # Σ |angle(velocity, route heading)| — what crab WOULD be
var crab_old_max := 0.0          # without the steer-into-path yaw (the pre-fix behaviour)
var crab_n := 0
# SLOPE-RESIDUAL: the velocity metric above is dominated by polyline-discretization noise
# (displacement follows raw segments; the bike rightly points along the smoothed heading).
# This one is exact + SIGNED: true lateral slope atan(Δlat/Δrd) from the sim's own values
# vs the yaw actually applied (signed angle heading→forward about UP). Correct fix → ≈ 0;
# WRONG SIGN → residual = 2× slope (the velocity metric can't distinguish these).
var prev_latrd := {}             # rider index → Vector2(lat, rd) last frame
var slope_sum := 0.0
var slope_max := 0.0
var slope_n := 0

func _ready() -> void:
	DisplayServer.window_set_size(Vector2i(1600, 1000))
	main = (load("res://Main.tscn") as PackedScene).instantiate() as Node3D
	# Force same-lane convergence so P2 passes actually develop in a short test: tight
	# leash (dense pack) + wide ability spread (big speed differences → lots of catching).
	# Set BEFORE add_child so the spawn (in Main._ready) uses them.
	if OS.has_environment("RIDESIM_TD_TRAFFIC"):
		main.set("peloton_leash_m", 14.0)
		main.set("peloton_ability_spread", 0.35)
	add_child(main)                # its _ready loads the world + spawns the peloton
	cam = Camera3D.new()
	cam.projection = Camera3D.PROJECTION_ORTHOGONAL
	cam.size = 22.0                # metres across (fan is ~6 m, plenty of length)
	cam.near = 0.5; cam.far = 4000.0
	add_child(cam)

func _process(delta: float) -> void:
	if main == null: return
	# Let Main settle one frame, then ride (it seeks paused when SEEK_KM is set).
	if t == 0.0:
		main.set("wait_for_telemetry", false)
		main.set("paused", false)
	t += delta
	var pel: Array = main.get("peloton")
	if pel == null or pel.is_empty():
		return
	# center on the pack centroid; look straight down, +route-forward = screen up-ish
	var c := Vector3.ZERO
	for r in pel:
		c += (r["node"] as Node3D).global_position
	c /= float(pel.size())
	cam.global_position = c + Vector3(0, 60, 0)
	cam.look_at(c, Vector3(0, 0, -1))
	cam.make_current()
	# sample the TRUE per-rider lateral offset (metres from route centerline) that Main
	# computes + eases — r["lat"]. This is the real signal; deriving it from world position
	# would fold in the (large) longitudinal spread + centroid drift = false jitter.
	for i in mini(8, pel.size()):
		if not laterals.has(i): laterals[i] = []
		laterals[i].append(float(pel[i].get("lat", 0.0)))
	# COLLISION CHECK: for EVERY pair overlapping along-route (|Δrd| < bike_len), track the
	# smallest lateral gap. If it drops below a rider footprint the bikes are clipping. Also
	# log the closest pair's lanes so we know if it's same-lane or cross-lane.
	if t > 2.0:
		for a in pel.size():
			for b in range(a + 1, pel.size()):
				var drd: float = absf(float(pel[a].get("prev_pos_d", 0.0)) - float(pel[b].get("prev_pos_d", 0.0)))
				if drd >= 2.0:
					continue
				var dlat: float = absf(float(pel[a].get("lat", 0.0)) - float(pel[b].get("lat", 0.0)))
				if dlat < min_lat_gap:
					min_lat_gap = dlat
					closest_pair = "lanes %d/%d drd=%.2f dlat=%.2f" % [
						int(pel[a].get("lane_idx", -1)), int(pel[b].get("lane_idx", -1)), drd, dlat]
				if dlat < 0.55:
					collision_frames += 1
	pace_max = maxf(pace_max, float(main.get("_pack_cruise")))
	# CRAB CHECK: angle between a rider's actual horizontal velocity and (a) its node's
	# forward = residual crab after the steer-into-path yaw, (b) the raw route heading =
	# the pre-fix crab (what the tires-sliding-sideways artifact measured). Same run
	# yields both, so no need to rebuild the old code for a baseline.
	if t > 2.0:
		var player_d: float = float(main.get("view_dist"))
		var pass_zone: float = float(main.get("ghost_pass_zone_m"))
		for i in pel.size():
			var n: Node3D = pel[i]["node"]
			var rd_now: float = float(pel[i].get("prev_pos_d", 0.0))
			# skip riders mid player-dodge: that lateral is time-domain (deliberately not
			# covered by the distance-slope yaw) and would pollute both metrics.
			if absf(rd_now - player_d) < pass_zone:
				prev_wpos.erase(i); prev_latrd.erase(i)
				continue
			var p: Vector3 = n.global_position
			var f: Vector3 = -n.global_transform.basis.z
			f.y = 0.0
			if prev_wpos.has(i):
				var v: Vector3 = (p - prev_wpos[i]) / maxf(delta, 0.0001)
				v.y = 0.0
				if v.length() > 2.0 and f.length() > 0.01:
					var a := absf(rad_to_deg(v.normalized().angle_to(f.normalized())))
					var h: Vector3 = main.call("_heading_at", rd_now)
					var a_old := absf(rad_to_deg(v.normalized().angle_to(h)))
					crab_sum += a; crab_max = maxf(crab_max, a)
					crab_old_sum += a_old; crab_old_max = maxf(crab_old_max, a_old)
					crab_n += 1
			prev_wpos[i] = p
			var lat_now: float = float(pel[i].get("lat", 0.0))
			if prev_latrd.has(i) and f.length() > 0.01:
				var pv: Vector2 = prev_latrd[i]
				var drd := rd_now - pv.y
				if drd > 0.02:
					var slope := atan((lat_now - pv.x) / drd)
					var h2: Vector3 = main.call("_heading_at", rd_now)
					var applied: float = h2.signed_angle_to(f.normalized(), Vector3.UP)
					var resid := absf(rad_to_deg(slope + applied))   # yaw should be −atan(slope)
					slope_sum += resid; slope_max = maxf(slope_max, resid)
					slope_n += 1
					if slope_n % 500 == 0 and OS.has_environment("RIDESIM_CRAB_DEBUG"):
						print("[CRABDBG] i=%d rd=%.1f slope=%+.3f° applied=%+.3f° resid=%.3f°"
							% [i, rd_now, rad_to_deg(slope), rad_to_deg(applied), resid])
			prev_latrd[i] = Vector2(lat_now, rd_now)
	# SURGE CHECK: Main computes each rider's along-route speed with MATCHED delta (accurate);
	# read that (unclamped) instead of finite-differencing here (which hits frame-time jitter).
	if t > 2.0:
		for i in pel.size():
			var v: float = float(pel[i].get("rider_speed_raw", 0.0))
			rider_v_min = minf(rider_v_min, v)
			rider_v_max = maxf(rider_v_max, v)
			if v < 0.5 and OS.has_environment("RIDESIM_SURGE_DEBUG"):
				var pd: float = float(main.get("view_dist"))
				print("[SURGEDBG] t=%.2f bib=%d v=%.2f rel=%.2f bub=%+.2f side=%+.0f lane=%d"
					% [t, int(pel[i].get("bib", i)), v, float(pel[i].get("prev_pos_d", 0.0)) - pd,
					   float(pel[i].get("bub", 0.0)), float(pel[i].get("bub_side", 0.0)),
					   int(pel[i].get("lane_idx", -1))])
	var borrowing := 0   # legacy P2 counter (now always 0 — passes are preordained cross-lane)
	if shot_i < shots.size() and t >= shots[shot_i]:
		await RenderingServer.frame_post_draw
		var img := get_viewport().get_texture().get_image()
		var fn := "res://../peloton_topdown_%d.png" % shot_i
		print("shot %d @ %.1fs  borrowing=%d/%d -> %s" % [shot_i, t, borrowing, pel.size(), img.save_png(fn)])
		shot_i += 1
		prev_rd.clear()   # the await skipped frames → re-baseline the surge-speed finite-diff
		prev_wpos.clear() # same for the crab-angle velocity finite-diff
		prev_latrd.clear()
	if shot_i >= shots.size():
		_report_jitter()
		get_tree().quit()

func _report_jitter() -> void:
	print("COLLISION: min lateral gap among overlapping pairs = %.2f m  [%s]  clip-frames=%d" % [min_lat_gap, closest_pair, collision_frames])
	print("PACE: peak _pack_cruise = %.1f m/s (%.0f km/h)" % [pace_max, pace_max * 3.6])
	print("SURGE: rider along-route speed range %.1f .. %.1f m/s (min < 0 = rolling backward = bug)" % [rider_v_min, rider_v_max])
	if crab_n > 0:
		print("CRAB: residual mean %.2f° max %.2f°  |  pre-fix (vs route heading) mean %.2f° max %.2f°  (n=%d)"
			% [crab_sum / crab_n, crab_max, crab_old_sum / crab_n, crab_old_max, crab_n])
	if slope_n > 0:
		print("CRAB-SLOPE: |atan(Δlat/Δrd) + applied_yaw| mean %.3f° max %.3f° (n=%d; ≈0 = yaw correct, 2×slope = sign wrong)"
			% [slope_sum / slope_n, slope_max, slope_n])
	# max abs frame-to-frame lateral step per rider; a 60 Hz jitter shows as a large,
	# sign-alternating step. Smooth choreography → tiny, monotone-ish steps.
	for i in laterals:
		var s: Array = laterals[i]
		var maxstep := 0.0
		var flips := 0
		for k in range(2, s.size()):
			var d0: float = s[k - 1] - s[k - 2]
			var d1: float = s[k] - s[k - 1]
			maxstep = maxf(maxstep, absf(d1))
			if d0 * d1 < 0.0 and absf(d1) > 0.02: flips += 1
		print("rider %d: samples=%d  max|Δlat/frame|=%.4f m  sign-flips>2cm=%d" % [i, s.size(), maxstep, flips])
