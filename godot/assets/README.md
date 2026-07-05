# Avatar models (drop-in, never shipped)

`Main.gd` → `_spawn_avatar()` loads a rider/ghost model from this folder if present,
otherwise it builds the procedural emissive "Tron" placeholder. So these files are
**optional and local-only**:

| File | Used as |
|------|---------|
| `rider.glb` | the ridden avatar |
| `ghost.glb` | the pace/ghost rider (falls back to placeholder if absent) |

## ⚠ Licensing — read before adding a model
Model binaries here are **git-ignored** (`*.glb`, `*.gltf`, `*.import`, `*.blend`) and
must **never be committed or shipped** unless you own the rights to distribute every
part of them. A release built with **no** model in this folder is safe by default —
the loader simply uses the placeholder. Only drop in a model you authored or that
carries a redistribution-permissive license (e.g. CC0).

## Orientation / scale contract (so it drops in correctly)
The model must follow the same convention as the placeholder:

- **Format:** `.glb` (binary glTF); `.gltf` also works.
- **Up:** `+Y`. **Forward (travel direction):** local `−Z` (Godot/glTF convention).
- **Origin:** tire/ground contact at `y = 0`, centered in X/Y, so it seats on the
  road surface instead of floating or burying.
- **Scale:** real-world meters (~1.7 m tall rider). `avatar_scale` multiplies on top.

If exported from Blender authored Y-up, stand it into Blender Z-up before a `+Y-up`
glTF export so it lands upright in Godot.

## Part-naming matrix (canonical, for the variant pipeline)

One grammar for every rider + bike part, unique across the whole variant library so
models can be merged/compared in Blender/SolidWorks without name collisions:

```
{variant}_{side}_{part}            e.g.  f2_l_calf,  m1_r_thigh,  f2_c_trunk
{variant}_{part}                   for bike parts / unsided parts:  m1_wheel_f,  f2_crank
```

- **variant** — `m1`, `f1`, `f2`, … (rider model line + revision). Never reuse.
- **side** — `l` / `r` (legs, arms, pedals), `c` (or omit) for centered parts.
- **part** — the canonical token from the table below, lowercase snake_case.
  The engine matches by SUBSTRING/PATTERN, so the variant prefix is free — but the
  canonical token must appear intact in the name.

| Part | Canonical token | Engine behavior (matcher) |
|------|-----------------|---------------------------|
| Front/rear wheel | `wheel_f` / `wheel_r` | spins by distance (`_collect_rig`: contains `wheel`; origin at hub, axle on local X) |
| Crankset | `crank` | spins by development/cadence (contains `crank`; origin at BB) |
| Cassette | `cassette` | locked to crank (contains `cassette`; origin on rear-hub axle). Legacy `cs-` prefix also works |
| Pedal assemblies | `pedal_l_asm` / `pedal_r_asm` | IK targets (exact-token contains; children of the crank) |
| Thigh / calf / foot / toe | `thigh` `calf` `foot` `toe` | LegRig IK, sided: name must carry `l`/`r` as `l_`/`r_` prefix or `_l_`/`_r_` segment (legacy `Mirror…`=left, `r…`=right still accepted) |
| Trunk (torso+pelvis) | `trunk` | LegRig hip anchor (contains one of `pelvis`/`trunk`/`torso`/`hip`) |
| Head | `head` | per-material mesh in the build scripts (kept un-flattened) |
| Upper arm / forearm | `upper_arm` / `forearm` | per-material mesh (female build script) |

**Materials** (matched by MATERIAL name, not node name — used by the per-rider tint):

| Material | Meaning |
|----------|---------|
| `kit_jersey` | tintable jersey (peloton golden-angle hue) |
| `kit_frame` | tintable frame color |
| `tire`, `hood`, `steel`, `black` | fixed darks (the "when in doubt, black" set) |
| `tan` / `skin` | skin tone |

Rules of thumb for a new variant:
1. Prefix EVERY node with the variant id — that alone guarantees library-wide uniqueness.
2. Keep the canonical token intact somewhere in the name; the engine only ever
   substring-matches, it never needs the exact full name.
3. Spinning parts: mesh origin at the rotation center, axle on local X.
4. Run `LegIKTest.tscn` (`RIDESIM_GLB=res://assets/<file>.glb`) after export —
   it must print `LegRig.setup ok=true legs=2`.

## Rung-1 rolling rig (optional, distance-driven)
If child nodes are named so their names contain `wheel` / `crank` (see the
`wheel_name_match` / `crank_name_match` exports), they spin as the bike rolls —
wheels by rolling circumference (`wheel_diameter_m`), the crank by development
(`crank_dev_m`, meters of travel per pedal revolution). Spin is a pure function of
distance: it speeds/slows/stops/reverses with the ride, no telemetry needed. Keep
each spinning part's **mesh origin at its rotation center** (hub / bottom-bracket
axis) and its axle on local X (or override `rig_spin_axis`).
