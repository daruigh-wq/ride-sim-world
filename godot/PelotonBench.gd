extends Node3D
# Course-characterization bench: drives the REAL Main scene over one route segment and
# prints pack telemetry — NO screenshots (no awaits), so every finite-diff metric sees
# true consecutive frames. Focus: the player's FRONT WINDOW (5–50 m ahead), where the
# artifacts matter most. Run once per segment:
#   RIDESIM_WORLD_DIR=<world> RIDESIM_BENCH_KM=24.0 RIDESIM_BENCH_S=45 \
#   RIDESIM_PELOTON_N=50 RIDESIM_PELOTON_LEVEL=pro RIDESIM_PELOTON_FREE=1 \
#   Godot --path godot res://PelotonBench.tscn
# The demo player rides at ~92% of the pack's capability pace on the segment's grade,
# so the bunch flows past through the whole window (constant realistic encounters).

var main: Node3D
var t := 0.0
var dur := 45.0
var warm := 4.0                 # settle time before metrics start
# pack pace
var pace_sum := 0.0
var pace_n := 0
var cap_sum := 0.0
var grade_sum := 0.0
var prev_ref := NAN
# rider speed spread (vs the pack reference speed the same frame)
var v_min := 1e9
var v_max := -1e9
var ratio_min := 1e9
var ratio_max := -1e9
# slow-surge period: sign changes of rider 0..2 drift offset (rd − ref − home)
var drift_sign := {}
var drift_flips := {}
# front window (player+5 .. player+50)
var fw_lat := {}                # bib → last lat (per-frame step tracking)
var fw_maxstep := 0.0
var fw_flips := 0
var fw_laststep := {}
# player clearance (all riders, both directions)
var clear_min := 1e9
var overlap_frames := 0
var frames := 0

func _ready() -> void:
	DisplayServer.window_set_size(Vector2i(1280, 720))
	dur = OS.get_environment("RIDESIM_BENCH_S").to_float() if OS.has_environment("RIDESIM_BENCH_S") else 45.0
	main = (load("res://Main.tscn") as PackedScene).instantiate() as Node3D
	add_child(main)
	var km := OS.get_environment("RIDESIM_BENCH_KM").to_float()
	main.call("_seek_to_km", km)
	main.set("wait_for_telemetry", false)
	# player pace: 92% of the pack's capability on the SEGMENT-AVERAGE grade (a single
	# point can be a hairpin/DEM spike) → pack slowly overtakes through the window
	var g := 0.0
	for k in 7:
		g += float(main.call("_grade_at", km * 1000.0 + float(k) * 100.0)) / 7.0
	var cap: float = main.call("_capability_pace", g)
	main.set("demo_speed", cap * 0.92)
	main.set("paused", false)
	print("[BENCH] km=%.1f grade=%+.1f%% cap=%.1f m/s player=%.1f m/s dur=%.0fs"
		% [km, g * 100.0, cap, cap * 0.92, dur])

func _process(delta: float) -> void:
	if main == null:
		return
	t += delta
	if t < warm:
		return
	var pel: Array = main.get("peloton")
	if pel == null or pel.is_empty():
		return
	frames += 1
	var d: float = float(main.get("view_dist"))
	var plat: float = float(main.get("_player_lat"))
	var ref: float = float(main.get("_pack_ref"))
	# pack pace + capability + grade
	if not is_nan(prev_ref):
		var pv := (ref - prev_ref) / maxf(delta, 0.0001)
		pace_sum += pv
		pace_n += 1
		var gg: float = main.call("_grade_at", ref)
		grade_sum += gg
		cap_sum += main.call("_capability_pace", gg)
		# rider spread vs the pack this frame
		for i in pel.size():
			var v: float = float(pel[i].get("rider_speed_raw", 0.0))
			v_min = minf(v_min, v)
			v_max = maxf(v_max, v)
			if pv > 1.0:
				ratio_min = minf(ratio_min, v / pv)
				ratio_max = maxf(ratio_max, v / pv)
	prev_ref = ref
	# slow-surge sign flips on 3 riders → period estimate
	for i in mini(3, pel.size()):
		var off: float = float(pel[i].get("prev_pos_d", 0.0)) - ref - float(pel[i].get("home", 0.0))
		var s := 1 if off >= 0.0 else -1
		if drift_sign.has(i) and s != drift_sign[i]:
			drift_flips[i] = int(drift_flips.get(i, 0)) + 1
		drift_sign[i] = s
	# front-window lateral smoothness + player clearance
	for i in pel.size():
		var rd: float = float(pel[i].get("prev_pos_d", 0.0))
		var lat: float = float(pel[i].get("lat", 0.0))
		var rel := rd - d
		# clearance ellipse: 1.0 at (2 m long, 0.6 m lat) — < 1 means overlapping the player
		var ce := sqrt(pow(rel / 2.0, 2.0) + pow((lat - plat) / 0.6, 2.0))
		clear_min = minf(clear_min, ce)
		if ce < 1.0:
			overlap_frames += 1
			if overlap_frames % 40 == 1:
				print("[BENCH-OVL] bib=%d rel=%+.2f dlat=%+.2f bub=%+.2f side=%+.0f lane=%d ce=%.2f"
					% [int(pel[i].get("bib", i)), rel, lat - plat, float(pel[i].get("bub", 0.0)),
					   float(pel[i].get("bub_side", 0.0)), int(pel[i].get("lane_idx", -1)), ce])
		if rel > 5.0 and rel < 50.0:
			var bib: int = int(pel[i].get("bib", i))
			if fw_lat.has(bib):
				var step: float = lat - float(fw_lat[bib])
				fw_maxstep = maxf(fw_maxstep, absf(step))
				if fw_laststep.has(bib) and float(fw_laststep[bib]) * step < 0.0 and absf(step) > 0.02:
					fw_flips += 1
				fw_laststep[bib] = step
			fw_lat[bib] = lat
		else:
			var bib2: int = int(pel[i].get("bib", i))
			fw_lat.erase(bib2)
			fw_laststep.erase(bib2)
	if t >= warm + dur:
		_report()
		get_tree().quit()

func _report() -> void:
	var pace := pace_sum / maxf(pace_n, 1)
	var flips_tot := 0
	for k in drift_flips:
		flips_tot += int(drift_flips[k])
	# each drift cycle = 2 sign flips; period = measured time / cycles (3 riders tracked)
	var cycles := float(flips_tot) / 2.0 / 3.0
	var period := (dur / cycles) if cycles > 0.25 else -1.0
	print("[BENCH] RESULT grade=%+.1f%%  pack=%.1f m/s (%.0f km/h)  cap=%.1f m/s" % [
		grade_sum / maxf(pace_n, 1) * 100.0, pace, pace * 3.6, cap_sum / maxf(pace_n, 1)])
	print("[BENCH] riders: v %.1f..%.1f m/s  ratio %.2f..%.2f×pack  surge period ≈ %.0f s" % [
		v_min, v_max, ratio_min, ratio_max, period])
	print("[BENCH] front 5-50m: max|Δlat/frame|=%.4f m  flips>2cm=%d" % [fw_maxstep, fw_flips])
	print("[BENCH] player: min clearance %.2f (1.0 = touching)  overlap-frames=%d / %d" % [
		clear_min, overlap_frames, frames])
