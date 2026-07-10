# CyberCadKernel — mechanical example gallery

Parametric mechanical CAD pieces built through the CyberCadKernel Python binding (`python/cybercadkernel`), on the real OCCT-backed engine. Every piece is a genuine B-rep solid: the volumes and bounding boxes below come from the kernel's exact mass-property query, not the mesh.

## How to build

```sh
# 1. Build the real-engine dylib (Homebrew OCCT at /opt/homebrew/opt/opencascade)
CLEAN=1 bash scripts/build-macos-dylib.sh

# 2. Point the binding at it and regenerate the whole gallery
export CYBERCADKERNEL_DYLIB="$PWD/build-mac/libcybercadkernel.dylib"
python3 examples/run_all.py
```

Each script is self-contained (`python3 examples/01_pipe_flange.py`) with its parametric constants at the top and a `build(kernel)` returning the `Shape`. Artifacts land in `examples/out/<name>/`: a STEP model, a binary STL, a glTF (`.glb`) and a PNG thumbnail.

## Pieces

### Pipe flange (6-bolt raised face)

<img src="out/01_pipe_flange/01_pipe_flange.png" width="360" alt="Pipe flange (6-bolt raised face)">

Round flange: a thick disc with a central pipe bore and a 6-hole bolt circle, outer rim filleted.

- **Features:** extruded discs, boolean cut, circular hole pattern, edge fillet
- **Volume:** 122,776.8 mm³ · **Surface area:** 27,917.5 mm²
- **Bounding box:** 120.0 × 120.0 × 14.0 mm
- **Artifacts:** [STEP](out/01_pipe_flange/01_pipe_flange.step) · [STL](out/01_pipe_flange/01_pipe_flange.stl) · [GLB](out/01_pipe_flange/01_pipe_flange.glb) · [PNG](out/01_pipe_flange/01_pipe_flange.png)
- **Script:** [`01_pipe_flange.py`](01_pipe_flange.py)

### L-bracket with gusset

<img src="out/02_l_bracket/02_l_bracket.png" width="360" alt="L-bracket with gusset">

Right-angle mounting bracket: two plates fused with a triangular gusset, a mounting hole per flange, filleted inner corner, chamfered outer edges.

- **Features:** boolean fuse, boolean cut, edge fillet, edge chamfer
- **Volume:** 95,902.9 mm³ · **Surface area:** 18,711.0 mm²
- **Bounding box:** 70.0 × 60.0 × 70.0 mm
- **Artifacts:** [STEP](out/02_l_bracket/02_l_bracket.step) · [STL](out/02_l_bracket/02_l_bracket.stl) · [GLB](out/02_l_bracket/02_l_bracket.glb) · [PNG](out/02_l_bracket/02_l_bracket.png)
- **Script:** [`02_l_bracket.py`](02_l_bracket.py)

### Pillow-block bearing housing

<img src="out/03_bearing_block/03_bearing_block.png" width="360" alt="Pillow-block bearing housing">

Bearing housing: a slotted base plate carrying a bored cylindrical boss (bearing seat with a back wall) and a filleted boss rim.

- **Features:** box + cylinder, boolean fuse/cut, bored seat, edge fillet
- **Volume:** 116,971.0 mm³ · **Surface area:** 28,031.9 mm²
- **Bounding box:** 120.0 × 62.0 × 46.0 mm
- **Artifacts:** [STEP](out/03_bearing_block/03_bearing_block.step) · [STL](out/03_bearing_block/03_bearing_block.stl) · [GLB](out/03_bearing_block/03_bearing_block.glb) · [PNG](out/03_bearing_block/03_bearing_block.png)
- **Script:** [`03_bearing_block.py`](03_bearing_block.py)

### V-belt pulley (single groove)

<img src="out/04_v_pulley/04_v_pulley.png" width="360" alt="V-belt pulley (single groove)">

Single-groove V-belt pulley revolved from one cross-section: bored hub, thin web, and a rim carrying a symmetric ~38 deg V-groove.

- **Features:** revolve (hand-built profile), integral bore, V-groove section
- **Volume:** 200,504.9 mm³ · **Surface area:** 46,237.0 mm²
- **Bounding box:** 120.0 × 34.0 × 120.0 mm
- **Artifacts:** [STEP](out/04_v_pulley/04_v_pulley.step) · [STL](out/04_v_pulley/04_v_pulley.stl) · [GLB](out/04_v_pulley/04_v_pulley.glb) · [PNG](out/04_v_pulley/04_v_pulley.png)
- **Script:** [`04_v_pulley.py`](04_v_pulley.py)

### Shelled electronics enclosure

<img src="out/05_enclosure/05_enclosure.png" width="360" alt="Shelled electronics enclosure">

Rounded-corner enclosure hollowed to a 2.4 mm wall (open top), with filleted outer edges and four internal lid-locating bosses.

- **Features:** rounded-rect extrude, shell, edge fillet, boolean fuse (bosses)
- **Volume:** 39,719.1 mm³ · **Surface area:** 33,058.4 mm²
- **Bounding box:** 100.0 × 64.0 × 32.0 mm
- **Artifacts:** [STEP](out/05_enclosure/05_enclosure.step) · [STL](out/05_enclosure/05_enclosure.stl) · [GLB](out/05_enclosure/05_enclosure.glb) · [PNG](out/05_enclosure/05_enclosure.png)
- **Script:** [`05_enclosure.py`](05_enclosure.py)

### Spur gear (simplified, 20 teeth)

<img src="out/06_spur_gear_simplified/06_spur_gear_simplified.png" width="360" alt="Spur gear (simplified, 20 teeth)">

Stylised 20-tooth spur gear: a blank with a circular array of trapezoidal tooth-gap cuts, a central bore and a keyway. Not a true involute profile.

- **Features:** cylinder primitive, circular boolean pattern, boolean cut (bore + keyway)
- **Volume:** 127,711.8 mm³ · **Surface area:** 26,906.4 mm²
- **Bounding box:** 109.7 × 109.7 × 16.0 mm
- **Artifacts:** [STEP](out/06_spur_gear_simplified/06_spur_gear_simplified.step) · [STL](out/06_spur_gear_simplified/06_spur_gear_simplified.stl) · [GLB](out/06_spur_gear_simplified/06_spur_gear_simplified.glb) · [PNG](out/06_spur_gear_simplified/06_spur_gear_simplified.png)
- **Script:** [`06_spur_gear_simplified.py`](06_spur_gear_simplified.py)

### Hydraulic manifold block

<img src="out/07_manifold_block/07_manifold_block.png" width="360" alt="Hydraulic manifold block">

Manifold block with a through horizontal gallery, two vertical ports that intersect it internally, four corner mounting holes and a chamfered top rim.

- **Features:** box primitive, intersecting cross-drillings, corner hole pattern, edge chamfer
- **Volume:** 142,866.0 mm³ · **Surface area:** 17,128.2 mm²
- **Bounding box:** 90.0 × 40.0 × 40.0 mm
- **Artifacts:** [STEP](out/07_manifold_block/07_manifold_block.step) · [STL](out/07_manifold_block/07_manifold_block.stl) · [GLB](out/07_manifold_block/07_manifold_block.glb) · [PNG](out/07_manifold_block/07_manifold_block.png)
- **Script:** [`07_manifold_block.py`](07_manifold_block.py)

## Rendering

Thumbnails are rendered offscreen by `cybercadkernel.viz.render_png`, which tries the trimesh OpenGL path first and falls back to a headless matplotlib rasterizer. In this run the renders resolved via: **matplotlib**.

## Coming when the binding completes

These parts are intentionally *not* built here (and not faked). Two reasons appear below:

**Reserved for the binding-completion track** — the feature is already bound and working in the current dylib, but the piece belongs to that concurrent track and is left for it to add rather than duplicated here:

- **Round-to-square transition (loft adapter)** — `Kernel.loft` / `Kernel.loft_wires` (bound and working) — owned by the binding-completion track.
- **Swept coolant tube** — `Kernel.sweep` / `Kernel.twisted_sweep` (bound and working) — owned by the binding-completion track.
- **Threaded hex bolt** — `Kernel.helical_thread` + `Shape.thread_apply` (bound and working) — owned by the binding-completion track.

**Not yet exposed through the Python facade** — no method in `api.py` yet, so genuinely unbuildable through the binding today:

- **Sheet-metal bracket (bends + flanges)** — sheet-metal ops (flange / bend / unfold) — no method in `api.py` yet.
- **Draft-moulded housing** — a draft-face op (pull-direction draft angle on side walls) — not exposed in `api.py` yet.
- **2-D section / HLR drawing view** — `cc_hlr_project` + a section op — no hidden-line / planar-section method in `api.py` yet.

