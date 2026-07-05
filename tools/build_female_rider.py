import bpy, bmesh, os, math
from mathutils import Matrix, Vector

SRC = "/Users/davidruigh/ride-sim-world/godot/assets/female_test.glb"
OUT = "/Users/davidruigh/ride-sim-world/godot/assets/female_opt.glb"

DECIMATE = {"cs-r9200": 0.35, "fc-r9200": 0.40, "rd-r9250": 0.35, "fd-r9250": 0.35,
            "st-r9270": 0.35, "udh": 0.30, "rt-mt900": 0.45}

# SolidWorks' 2001 export bug blanks image-appearances to white AND collapses many
# surfaces onto the shared default material_0. So we repaint PER-MESH keyed on the
# (intact) semantic IK mesh names, to flat reference colors. Deterministic every export.
SRGB = {
    "burgundy": (0.36, 0.09, 0.12),   # jersey
    "charcoal": (0.15, 0.15, 0.17),   # shorts
    "tan":      (0.64, 0.47, 0.33),   # skin
    "blonde":   (0.82, 0.60, 0.22),   # hair
    "black":    (0.03, 0.03, 0.035),  # saddle / bars / crank / derailleurs / pedals
    "red":      (0.80, 0.02, 0.02),   # fork (match frame)
    "hood":     (0.12, 0.12, 0.13),   # STI brake-lever hoods (match the man)
    "steel":    (0.55, 0.56, 0.60),   # brake rotors (bare steel)
}
# mesh-name token -> whole-mesh flat color (single-purpose parts)
FLATTEN = [
    ("torso", "burgundy"), ("trunk", "burgundy"),
    ("pelvis", "charcoal"),
    ("calf", "tan"), ("forearm", "tan"),
    ("uprarm", "tan"), ("hand", "tan"),
    # thigh is NOT flattened — the shorts hem crosses it; painted per-face below.
    ("fork", "red"),
    ("bar and stem", "black"), ("saddle", "black"), ("house of pain", "black"),
    # Groupset shipped white (Material_0, the SW bug) — the male's reads black/steel, so
    # match it. Part names differ between the two models, so key on the Shimano codes.
    ("fc-r9200", "black"),      # crank + chainrings
    ("rd-r9250", "black"),      # rear derailleur
    ("fd-r9250", "black"),      # front derailleur
    ("cs-r9200", "black"),      # cassette (still named CS-... here; renamed after recolor)
    ("udh", "black"),           # derailleur hanger
    ("st-r9270", "hood"),       # brake-lever hoods — one mesh, so dark-gray whole (hoods)
    ("rt-mt900", "steel"),      # brake rotors → bare steel, not white
    ("pedal", "black"),         # pedals shipped debug purple/cyan/red → black
]
# meshes whose slots are per-material. head = face skin + hair; upper_arm = burgundy
# jersey sleeve + tan skin (its slot names survived, but the mesh is "upper_arm" not the
# male's "uprarm" token, so it fell through FLATTEN and kept white SW-blanked slots).
PER_MAT = [("sponge", "blonde"), ("burgundy", "burgundy"), ("skin", "tan"), ("material_0", "tan")]
PER_MAT_MESH = ["head", "upper_arm"]  # meshes handled per-slot instead of flattened

def srgb_to_linear(c):
    out = []
    for s in c:
        out.append(s / 12.92 if s <= 0.04045 else ((s + 0.055) / 1.055) ** 2.4)
    return out

def weld_all(dist=1e-5):
    for o in bpy.data.objects:
        if o.type != 'MESH': continue
        bm = bmesh.new(); bm.from_mesh(o.data)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)
        bm.to_mesh(o.data); bm.free()

def decimate_groupset():
    hit = []
    for o in list(bpy.data.objects):
        if o.type != 'MESH': continue
        n = o.name.lower()
        r = next((v for k, v in DECIMATE.items() if k in n), None)
        if r is None: continue
        mod = o.modifiers.new("dec", 'DECIMATE'); mod.ratio = r
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.modifier_apply(modifier=mod.name)
        hit.append(o.name)
    return hit

_MATCACHE = {}
def flat_mat(colorname):
    if colorname in _MATCACHE:
        return _MATCACHE[colorname]
    lin = srgb_to_linear(SRGB[colorname])
    m = bpy.data.materials.new("kit_" + colorname)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (lin[0], lin[1], lin[2], 1.0)
    bsdf.inputs["Roughness"].default_value = 0.85
    if "Metallic" in bsdf.inputs: bsdf.inputs["Metallic"].default_value = 0.0
    m.diffuse_color = (lin[0], lin[1], lin[2], 1.0)
    _MATCACHE[colorname] = m
    return m

def recolor():
    hit = {}
    for o in bpy.data.objects:
        if o.type != 'MESH': continue
        nl = o.name.lower()
        # per-material meshes (head): recolor each slot by its material name
        if any(t in nl for t in PER_MAT_MESH):
            for slot in o.material_slots:
                mn = (slot.material.name.lower() if slot.material else "")
                for key, col in PER_MAT:
                    if key in mn:
                        slot.material = flat_mat(col)
                        hit[col] = hit.get(col, 0) + 1
                        break
            continue
        # flatten meshes: every slot -> one color
        col = next((c for tok, c in FLATTEN if tok in nl), None)
        if col is None: continue
        m = flat_mat(col)
        o.data.materials.clear()
        o.data.materials.append(m)
        hit[col] = hit.get(col, 0) + 1
    return hit

def paint_thigh_shorts(hem_frac=0.45):
    # The shorts/skin split-line on the thigh collapsed into material_0 (the SW bug), so
    # rebuild it by geometry. To get a CLEAN STRAIGHT hem (not a triangle-zigzag), first
    # bisect the thigh with a plane at hem_frac along its own hip->knee axis (creating a
    # real edge loop), THEN paint each side: above the hem = charcoal shorts, below = tan.
    tan = flat_mat("tan"); charcoal = flat_mat("charcoal")
    painted = []
    for o in list(bpy.data.objects):
        nl = o.name.lower()
        if o.type != 'MESH' or 'thigh' not in nl:
            continue
        me = o.data
        wv = [o.matrix_world @ v.co for v in me.vertices]
        if len(wv) < 2:
            continue
        hip = max(wv, key=lambda p: p.z)      # top of the limb (Blender Z-up)
        knee = min(wv, key=lambda p: p.z)     # bottom
        axis = (knee - hip)
        me.materials.clear()
        me.materials.append(tan)              # slot 0 = skin
        me.materials.append(charcoal)         # slot 1 = shorts
        mwi = o.matrix_world.inverted()
        co_w = hip + axis * hem_frac
        co_l = mwi @ co_w
        no_l = (mwi.to_3x3() @ axis).normalized()   # plane normal, points toward the knee
        bm = bmesh.new(); bm.from_mesh(me)
        bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:],
                               plane_co=co_l, plane_no=no_l, clear_inner=False, clear_outer=False)
        for f in bm.faces:
            side = (f.calc_center_median() - co_l).dot(no_l)   # <0 = hip side (shorts)
            f.material_index = 1 if side < 0.0 else 0
        bm.to_mesh(me); bm.free()
        me.update()
        painted.append(o.name)
    return painted

def snap_feet_to_pedal(ball_frac=0.62, pitch_deg=-8.0, clearance=0.010):
    # LegRig FREEZES whatever foot<->pedal relationship it captures at rest and rigidly
    # translates the foot with the pedal (no ankling). The female's authored rest feet sit
    # too low + wrong pitch, so the sole hangs below the axle every stroke. Rebuild a clean
    # rest pose (like the male's): rigidly re-place each foot so the BALL sits over the axle
    # (fore-aft), the sole rests just above it, and the foot is at a natural ~8deg toe-down.
    # Blender Z-up; bike-forward = +Y (front wheel at +Y). Runs BEFORE build_rig (pedals
    # still at top level with their authored world positions).
    pedals = {}
    for o in bpy.data.objects:
        nl = o.name.lower()
        if o.type == 'EMPTY' and 'pedal_l_asm' in nl: pedals['l'] = o.matrix_world.translation.copy()
        if o.type == 'EMPTY' and 'pedal_r_asm' in nl: pedals['r'] = o.matrix_world.translation.copy()
    out = {}
    for o in list(bpy.data.objects):
        nl = o.name.lower()
        if o.type != 'MESH' or 'foot' not in nl or 'for ik' not in nl:
            continue
        side = 'l' if 'ikl_' in nl else 'r'
        ped = pedals.get(side)
        if ped is None:
            continue
        mw = o.matrix_world; minv = mw.inverted()
        wv = [mw @ v.co for v in o.data.vertices]
        heel = min(wv, key=lambda p: p.y)       # back
        toe = max(wv, key=lambda p: p.y)        # front (+Y)
        cur_pitch = math.atan2(toe.z - heel.z, toe.y - heel.y)
        ball = heel + (toe - heel) * ball_frac
        # 1) rotate about X through the ball to set a natural toe-down pitch
        R = Matrix.Rotation(math.radians(pitch_deg) - cur_pitch, 4, 'X')
        wv = [ball + (R @ (w - ball)) for w in wv]
        # 2) translate: ball over the axle fore-aft, sole just above the axle
        ball2 = min(wv, key=lambda p: p.y) + (max(wv, key=lambda p: p.y) - min(wv, key=lambda p: p.y)) * ball_frac
        sole_z = min(w.z for w in wv)
        shift = Vector((0.0, ped.y - ball2.y, (ped.z + clearance) - sole_z))
        for i, v in enumerate(o.data.vertices):
            v.co = minv @ (wv[i] + shift)
        o.data.update()
        out[o.name] = (round(math.degrees(math.radians(pitch_deg)), 1),)
    return out

def _reparent(child, parent):
    w = child.matrix_world.copy()
    child.parent = parent
    child.matrix_world = w

def build_rig():
    # Give the raw SolidWorks export the same rig scaffold the male bike got, so the
    # existing Main.gd _drive_crank + LegRig work unchanged. The female ALREADY ships
    # pedal_l_asm / pedal_r_asm empties and an FC-R9200 empty at the BB; we only need to
    # (1) turn that BB empty into the "crank" and reparent the pedals under it, and
    # (2) rename legs to LegRig's side tokens (left="Mirror...", right="r_...").
    info = {}
    # 1) crank = a FRESH WORLD-ALIGNED (identity-basis) empty at the BB. The SW FC-R9200
    # empty carries a 90° twist (its local-X is fore-aft), so renaming it made the crank
    # spin about Z = shaft-drive/propeller. The male has a fresh identity crank empty;
    # mirror that so _drive_crank's Basis(RIGHT,..) rotates about the true lateral axle.
    fc = None
    for o in bpy.data.objects:
        nl = o.name.lower()
        if o.type == 'EMPTY' and 'fc-r9200' in nl and 'mesh' not in nl:
            fc = o; break
    if fc is None:
        print("WARN build_rig: no FC-R9200 empty found"); return info
    bb = fc.matrix_world.translation.copy()
    crank = bpy.data.objects.new("crank", None)      # None data → Empty
    bpy.context.scene.collection.objects.link(crank)
    crank.matrix_world = Matrix.Translation(bb)       # identity basis, at the BB
    _reparent(fc, crank)                              # chainrings under the crank
    info["crank_at"] = tuple(round(v, 3) for v in bb)
    # 2) reparent both pedal assemblies under the crank (keep world transform → they orbit)
    peds = []
    for o in list(bpy.data.objects):
        nl = o.name.lower()
        if o.type == 'EMPTY' and ('pedal_l_asm' in nl or 'pedal_r_asm' in nl):
            _reparent(o, crank); peds.append(o.name)
    info["pedals_under_crank"] = peds
    # 3) rename leg segments so LegRig's side finder matches (l→Mirror, r→r_)
    ren = {}
    for o in list(bpy.data.objects):
        nl = o.name.lower()
        part = next((p for p in ("thigh", "calf", "foot") if p in nl), None)
        if part is None:
            continue
        if 'for ikl_' in nl:
            o.name = "Mirrorr_" + part + "_woman"; ren[nl] = o.name
        elif 'for ikr_' in nl:
            o.name = "r_" + part + "_woman"; ren[nl] = o.name
    info["legs_renamed"] = ren
    # 4) the cassette is an EMPTY (at the rear-hub axle) wrapping a MESH, both named
    # "CS-...". Both match Main._collect_rig's "cs-" test → the cog mesh would be
    # double-rotated. Rename the inner mesh so ONLY the axle empty drives the cassette.
    for o in bpy.data.objects:
        nl = o.name.lower()
        if o.type == 'MESH' and nl.startswith("cs-") and "mesh" in nl:
            o.name = "R9200_cog_cluster"
            info["cassette_mesh_renamed"] = o.name
    return info

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=SRC)
weld_all()
dec = decimate_groupset()
rec = recolor()
thighs = paint_thigh_shorts()
feet = snap_feet_to_pedal()
rig = build_rig()
bpy.context.view_layer.update()
bpy.ops.export_scene.gltf(filepath=OUT, export_format='GLB', export_yup=True,
                          use_selection=False, export_apply=True)
print("=== BUILD DONE ===")
print("decimated:", dec)
print("recolored slots:", rec)
print("thighs painted:", thighs)
print("feet snapped:", feet)
print("rig:", rig)
print("size: %.1f MB" % (os.path.getsize(OUT)/1e6))
