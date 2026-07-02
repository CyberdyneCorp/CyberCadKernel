# Features

The public surface is **72 `cc_*` functions**: the **57** that mirror CyberCad's
`KernelBridgeAPI.h` contract (the exact set the app relies on) plus **15
additive** functions introduced by Phases 1–3. Additive means every original
signature is unchanged and binary-compatible — new capability is opt-in.

## Core B-rep surface (the 57-function contract)

Grouped as the engine adapter groups them:

| Group | Representative `cc_*` | What it does |
|---|---|---|
| **Construction** | `cc_solid_extrude`, `cc_solid_revolve`, `cc_solid_loft`, `cc_solid_sweep`, `cc_twisted_sweep`, `cc_loft_along_rail`, `cc_guided_sweep`, `cc_wrap_emboss`, `cc_helical_thread`, `cc_tapered_thread`, `cc_tapered_shank`, `cc_solid_extrude_holes`, `cc_solid_extrude_profile*` | prisms, revolutions, lofts, sweeps, threads, and profiled/holed extrusions (true arc/line/circle/spline edges) |
| **Features** | `cc_fillet_edges`, `cc_fillet_edges_variable`, `cc_chamfer_edges`, `cc_shell`, `cc_offset_face`, `cc_fillet_face`, `cc_replace_face*`, `cc_split_plane`, `cc_offset_face_boundary` | edge/face fillets & chamfers, shell/hollow, offsets, face retarget, plane split |
| **Boolean & transform** | `cc_boolean`, `cc_translate_shape`, `cc_rotate_shape_about`, `cc_scale_shape*`, `cc_mirror_shape`, `cc_place_on_frame` | fuse/cut/common; rigid + scale transforms; place-on-plane |
| **Tessellation** | `cc_tessellate`, `cc_face_meshes`, `cc_edge_polylines` | display/export meshes; per-face meshes + edge polylines for picking |
| **Query** | `cc_mass_properties`, `cc_principal_moments`, `cc_bounding_box`, `cc_face_axis`, `cc_subshape_ids`, `cc_tangent_chain`, `cc_outer_rim_chain` | exact volume/area/CoM/inertia, bbox, stable sub-shape ids, edge-chain growth |
| **Exchange** | `cc_step_export/import`, `cc_iges_export/import` | STEP / IGES round-trip in true millimetres |
| **Lifecycle** | `cc_brep_available`, `cc_last_error`, `cc_shape_release`, `cc_*_free` | availability probe, error retrieval, handle + buffer lifetime |

Every constructed shape is validated (`BRepCheck_Analyzer::IsValid`) before it is
accepted; an invalid result returns `0` so the caller never renders/persists a
bad body.

## Additive capabilities (Phases 1–3)

### Phase 1 — multi-core acceleration
| `cc_*` | Purpose |
|---|---|
| `cc_set_parallel(int)` | toggle multi-core execution of the boolean/mesh paths (default on) |
| `cc_parallel_enabled(void)` | query current parallel state — enables serial-vs-parallel A/B |

Behind these: OCCT parallel booleans (`SetRunParallel` + tuned fuzzy value) and
parallel meshing (`BRepMesh` `InParallel`), a bounded worker pool, and a
fine-thread boolean gate. Verified bit-identical serial-vs-parallel.

### Phase 2 — GPU acceleration (Metal)
| `cc_*` | Purpose |
|---|---|
| `cc_set_gpu_tessellation(int)` | route GPU-eligible faces through Metal surface-eval in `cc_tessellate` (default off) |
| `cc_gpu_tessellation_enabled(void)` | query GPU-tessellation state |

A face is GPU-eligible only when provably equivalent to its trimmed form
(single outer wire, no holes, untrimmed rectangular UV patch); **every other face
falls back to OCCT `BRepMesh`**. GPU work is fp32 (display mesh); topology stays
on the CPU. The Metal backend also powers standalone GPU modules (surface eval,
LBVH, picking, normals), each verified against a CPU reference.

### Phase 3 — native features OCCT lacks
| `cc_*` | Feature | Notes |
|---|---|---|
| `cc_ref_plane_from_points`, `cc_ref_plane_offset`, `cc_ref_plane_from_face`, `cc_ref_axis_from_points`, `cc_ref_axis_from_edge`, `cc_ref_axis_from_face` | **Reference geometry** (#datum) | datum planes/axes as POD; correct to 1e-9 |
| `cc_wrap_emboss` (robust path) | **Robust wrap-emboss** (#290) | cap-and-side + healed sew replaces fragile ThruSections |
| `cc_thread_apply` | **Robust thread↔shaft boolean** (#286) | feature-based; completes in seconds, no minutes-long hang |
| `cc_full_round_fillet`, `cc_full_round_fillet_faces` | **Rolling-ball / full-round fillet** (#285) | consumes the middle face; parallel-wall cases (non-parallel falls back to edge fillet) |
| `cc_fillet_edges_g2` | **Curvature-continuous (G2) blend fillet** (#284) | measured G2 at straight seams (OCCT is G1-only) |

See per-feature verification numbers in [STATUS-phase-3.md](STATUS-phase-3.md).

## What is *not* here (by design)

- No GPU path for the exact fp64 modeling core — Apple GPUs have no fp64, so
  booleans/topology stay on the CPU.
- Full-round fillet for **non-parallel** walls and G2 for **non-straight** seams
  fall back to a valid standard fillet (documented, not faked).
- The native (non-OCCT) engine — that's Phase 4.
