import bpy, bmesh, math
from mathutils import Vector

# Repaint the MALE rider (rider.glb is monochrome grey — its SolidWorks appearances
# collapsed onto Material_0 exactly like the female's, but it was never repainted).
# rider.glb is ALREADY optimized + rigged, so we ONLY recolor here (no weld/decimate/
# rig-build/foot-snap). Output male_opt.glb — the colored base the peloton + ghost use.
# Distinct BLUE kit so male vs female read apart before per-instance variety tints.
SRC = "/Users/davidruigh/ride-sim-world/godot/assets/rider.glb"
OUT = "/Users/davidruigh/ride-sim-world/godot/assets/male_opt.glb"

SRGB = {
    "jersey": (0.09, 0.20, 0.55),   # blue (vs female burgundy)
    "shorts": (0.05, 0.05, 0.06),   # black bibs
    "skin":   (0.62, 0.46, 0.34),   # tan
    "shoe":   (0.90, 0.90, 0.93),   # white
    "frame":  (0.08, 0.13, 0.38),   # deep blue frame (vs female red)
    "black":  (0.03, 0.03, 0.035),  # bars / saddle / post
    "tire":   (0.06, 0.06, 0.07),   # near-black rubber (both wheels the SAME)
}
# whole-mesh flat color by name token (single-purpose parts)
FLATTEN = [
    ("head", "skin"), ("uprarm", "skin"), ("forearm", "skin"), ("hand", "skin"),
    ("calf", "skin"),
    ("foot", "shoe"), ("toe", "shoe"),
    ("road frame carbon", "frame"), ("carbon road fork", "frame"),
    ("bar and stem", "black"), ("saddle", "black"), ("post", "black"), ("gspring", "black"),
    # tires: rear shipped a blue-purple slot (0.5,0.5,1.0), front a dark grey — force BOTH
    # to one rubber black so the wheels match. pedals shipped debug colors → black.
    ("tire", "tire"), ("pedal", "black"),
    # trunk = jersey/shorts split (below); thigh = shorts-hem split (below)
]

def srgb_to_linear(c):
    return [s / 12.92 if s <= 0.04045 else ((s + 0.055) / 1.055) ** 2.4 for s in c]

_MATCACHE = {}
def flat_mat(name):
    if name in _MATCACHE:
        return _MATCACHE[name]
    lin = srgb_to_linear(SRGB[name])
    m = bpy.data.materials.new("kit_" + name); m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (lin[0], lin[1], lin[2], 1.0)
    b.inputs["Roughness"].default_value = 0.85
    if "Metallic" in b.inputs: b.inputs["Metallic"].default_value = 0.0
    m.diffuse_color = (lin[0], lin[1], lin[2], 1.0)
    _MATCACHE[name] = m
    return m

def recolor_flatten():
    hit = {}
    for o in bpy.data.objects:
        if o.type != 'MESH':
            continue
        nl = o.name.lower()
        col = next((c for tok, c in FLATTEN if tok in nl), None)
        if col is None:
            continue
        o.data.materials.clear(); o.data.materials.append(flat_mat(col))
        hit[col] = hit.get(col, 0) + 1
    return hit

def split_mesh(o, plane_co, plane_no, mat_pos, mat_neg):
    # bisect a mesh into two flat-colored regions (clean straight boundary edge loop)
    me = o.data
    me.materials.clear(); me.materials.append(flat_mat(mat_pos)); me.materials.append(flat_mat(mat_neg))
    mwi = o.matrix_world.inverted()
    co_l = mwi @ plane_co
    no_l = (mwi.to_3x3() @ plane_no).normalized()
    bm = bmesh.new(); bm.from_mesh(me)
    bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:],
                           plane_co=co_l, plane_no=no_l, clear_inner=False, clear_outer=False)
    for f in bm.faces:
        f.material_index = 0 if (f.calc_center_median() - co_l).dot(no_l) >= 0 else 1
    bm.to_mesh(me); bm.free(); me.update()

def paint_trunk(waist_frac=0.45):
    # trunk-1 = torso+pelvis merged; jersey ABOVE the waist, shorts BELOW (horizontal cut)
    for o in bpy.data.objects:
        if o.type == 'MESH' and 'trunk' in o.name.lower():
            zs = [(o.matrix_world @ v.co).z for v in o.data.vertices]
            wz = min(zs) + waist_frac * (max(zs) - min(zs))
            split_mesh(o, Vector((0, 0, wz)), Vector((0, 0, 1)), "jersey", "shorts")
            return o.name

def paint_thighs(hem_frac=0.38):
    # shorts continue onto the upper thigh; top hem_frac (along hip->knee) = shorts, rest skin
    done = []
    for o in bpy.data.objects:
        if o.type != 'MESH' or 'thigh' not in o.name.lower():
            continue
        wv = [o.matrix_world @ v.co for v in o.data.vertices]
        hip = max(wv, key=lambda p: p.z); knee = min(wv, key=lambda p: p.z)
        axis = knee - hip
        split_mesh(o, hip + axis * hem_frac, axis, "skin", "shorts")   # +axis(toward knee) side = skin
        done.append(o.name)
    return done

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=SRC)
flat = recolor_flatten()
trunk = paint_trunk()
thighs = paint_thighs()
import os
bpy.ops.export_scene.gltf(filepath=OUT, export_format='GLB', export_yup=True,
                          use_selection=False, export_apply=True)
print("=== MALE REPAINT DONE ===")
print("flattened:", flat)
print("trunk:", trunk, " thighs:", thighs)
print("size: %.1f MB" % (os.path.getsize(OUT) / 1e6))
