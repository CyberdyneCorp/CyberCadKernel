# Proposal — moat-dm2-replace-face-to-plane (M-DM DM2, native `cc_replace_face_to_plane`)

## Why

`cc_replace_face_to_plane(body, faceId, px,py,pz, nx,ny,nz)` — retarget ONE planar
face of a solid onto a new target plane, re-solving the adjacent faces so the solid
stays watertight — is the **second Direct-Modeling verb** on the MOAT roadmap
(`openspec/MOAT-ROADMAP.md` §M-DM). It is the app's push/pull "move a face to a target
plane". Today the verb exists behind the `cc_*` facade but is **OCCT-only**:
`NativeEngine::replace_face_to_plane` unconditionally `CC_NATIVE_BODY_UNSUPPORTED` →
`fallback().replace_face_to_plane(...)` (`src/engine/native/native_engine.cpp`), so
every call routes to OCCT `BRepBuilderAPI_MakeFace` + `BRepPrimAPI_MakeHalfSpace` +
`BRepAlgoAPI_Cut` (`src/engine/occt/occt_feature.cpp:263`).

DM2 is the direct successor of DM1 (`cc_split_plane`, archived
`2026-07-08-moat-dm1-split-plane`): the OCCT oracle for move-face is **exactly a
half-space cut against the target plane** — the same machinery DM1 already reproduces
natively. Move-face decomposes into three reachable convex-planar cases, each built
ADDITIVELY from already-landed, already-gated verbs, re-deriving no geometry:

- **The DM1 planar split** — `boolean/split_plane.h` `splitByPlane(operand, o, n,
  keepPositive)` — the landed BSP half-space cut, self-verified watertight, gated
  HOST-analytic (`V(below)+V(above)=V(whole)`, box halves fp-exact) + SIM-vs-OCCT. A
  **pure trim** (pull-in / inward tilt that only removes an outward slab from every
  neighbour) IS a DM1 split keeping the body bulk.
- **The analytic BSP/CSG boolean** — `boolean/native_boolean.h` `boolean_solid(a,b,op)`
  — the clean-room planar-polyhedron `Fuse`/`Cut`/`Common` (`Op::Fuse=0, Cut=1,
  Common=2`, `native_boolean_fwd.h`). A **pure grow** (push-out along the face normal)
  is `Fuse(operand, slab)` where `slab` is the picked face extruded to the target
  plane; a **general convex re-solve** (a tilted target plane, mixed grow/trim on a
  convex planar polyhedron) is the `Common`-chain reconstruction of the polytope
  `∩_{i≠k} H_i ∩ H_target` (replace the picked face's inward half-space `H_k` by the
  target plane's inward half-space `H_target`).
- **The extrusion + heal substrates** — `construct/native_construct.h` `build_prism`
  (the slab), `heal/*` tolerant sew / weld, and the engine's `watertightVolume` +
  `tessellate::isWatertight` audit (the SAME self-verify predicate DM1 uses).

DM2 buys the app's push/pull operation on the native path with a **closed-form volume
oracle** (moving a face by signed normal distance `d` changes the enclosed volume by
`A_face · d̄`, where `d̄` is the average signed distance the target plane displaces the
face — for a parallel offset `d̄ = d`, for a tilt about an in-face axis through the
centroid `d̄` is the centroid offset), and **no `cc_*` ABI change**.

## What Changes

1. **A new header-only, OCCT-free module `src/native/directmodel/`** in namespace
   `cybercad::native::directmodel`, consuming DM1 + the BSP boolean + construct + heal
   + topology READ-ONLY (zero OCCT includes; `src/native/directmodel/**` includes only
   `src/native/{boolean,construct,heal,topology,tessellate,math}` headers):
   - **`replace_face.h`** — the public `replaceFaceToPlane(solid, faceId, targetPlane)`
     entry. It reads the picked face's plane and outward normal from `topology`
     (READ-ONLY), classifies the move relative to the operand, and dispatches:
     - **Pure trim** (target plane keeps the body bulk and removes only an outward
       slab from every neighbour, no face added/removed) →
       `boolean::splitByPlane(operand, tp, n, keepBulk)` (DM1), CONSUMED unchanged.
     - **Pure grow, parallel** (target plane strictly outside, face moved along its
       own normal) → `boolean::boolean_solid(operand,
       construct::build_prism(faceLoop, +nFace·d), Op::Fuse)` then heal weld.
     - **General convex re-solve** (a tilted target plane, or mixed grow/trim on a
       convex planar polyhedron) → the polytope `∩_{i≠k} H_i ∩ H_target` reconstructed
       by a `boolean::boolean_solid(_, halfSpaceBox_i, Op::Common)` chain over the
       operand's face planes with the picked plane swapped for the target plane.

2. **A native `replace_face_to_plane` branch** in
   `src/engine/native/native_engine.cpp`, taken ONLY when `body` is a native solid in
   the reachable domain; otherwise the existing `fallback().replace_face_to_plane(...)`
   is preserved BYTE-IDENTICAL (guarded by `CYBERCAD_HAS_NUMSCI`, exactly as the DM1
   `split_plane` branch is).

3. **A mandatory re-solve self-verify** reusing the engine's existing
   `watertightVolume` + `tessellate::isWatertight` audit: the native result is accepted
   ONLY when it is a **closed watertight 2-manifold with positive enclosed volume**,
   the **moved face's samples lie on the target plane** within tolerance, it is a
   **single lump** (Euler χ = 2), and **no neighbour face inverted or was removed /
   created** (the face count and adjacency are preserved). A candidate that fails is
   DISCARDED and the call falls through to OCCT (`OcctEngine::replace_face_to_plane` —
   a move-face operand is always OCCT-reconstructible, so OCCT is the true
   fall-through). The engine NEVER emits an unverified, leaky, inverted, or wrong
   solid, and NEVER hands a native void to OCCT.

4. **Honest declines → NULL → OCCT** for every case outside the reachable slice, each
   labelled and verified as a fall-through, never faked: a **curved neighbour** that
   would need retrimming to a non-planar incidence (a cylinder / cone / sphere / spline
   side face the planar re-solve cannot extend); a **target plane that clips past an
   adjacent face** — severs the body into multiple lumps, inverts / removes a neighbour,
   or introduces a new face (a topology change the convex re-solve cannot represent); a
   **non-convex** operand whose half-space intersection would not reproduce the concave
   solid; a **degenerate** target plane (coincident with the picked face, or leaving a
   zero-volume sliver / empty solid); a **non-planar picked face**; a **foreign /
   mesh-only** body with no native B-rep. Under the native engine each such case returns
   the EXACT OCCT result of the same call.

5. **`src/native/**` stays OCCT-free and the `cc_*` ABI is unchanged.** DM1
   `splitByPlane`, `boolean_solid`, `build_prism`, and the heal verbs are consumed
   BYTE-IDENTICAL (no edit). The assembly is additive engine glue in
   `src/engine/native/` plus the new `src/native/directmodel/` module.

## Capabilities

### Added Capabilities

- `native-directmodel`: ADDS the SECOND direct-modeling verb on the native path —
  native `cc_replace_face_to_plane` (the app's push/pull move-face) — retargeting one
  planar face of a convex planar solid onto a target plane and re-solving the adjacent
  planar faces to stay watertight, via the landed DM1 split (pure trim), the BSP `Fuse`
  of an extruded slab (parallel grow), and the BSP `Common`-chain polytope
  reconstruction (tilted / mixed convex re-solve), self-verified (watertight
  2-manifold, positive volume, moved face on the target plane, single lump, face
  adjacency preserved) and gated by the closed-form volume oracle `ΔV = A_face · d̄`,
  with an honest decline → OCCT for curved-neighbour / topology-changing / non-convex /
  degenerate / non-planar-face / foreign cases.

## Impact

- **`src/native/directmodel/replace_face.h`** — NEW header-only, OCCT-free module
  (namespace `cybercad::native::directmodel`). Consumes DM1 `boolean::splitByPlane`,
  `boolean::boolean_solid`, `construct::build_prism`, `heal` weld, `topology`
  face-plane read, and the `tessellate` watertight/volume audit. Zero OCCT includes.
- `src/engine/native/native_engine.cpp` — `NativeEngine::replace_face_to_plane` gains
  the native branch + the re-solve self-verify + labelled decline → `fallback()`
  (mirroring the DM1 `split_plane` branch, `CYBERCAD_HAS_NUMSCI`-guarded). Cognitive
  complexity kept in the systems band: the driver delegates to move-classification,
  dispatch, and self-verify helpers; the geometry lives in the landed verbs.
- `src/engine/native/native_engine.h` — no signature change (the
  `replace_face_to_plane` override already exists at line 225).
- `src/native/boolean/split_plane.h`, `src/native/boolean/native_boolean.h`,
  `src/native/construct/native_construct.h`, `src/native/heal/**` — CONSUMED unchanged
  (no edit).
- **`cc_*` ABI:** UNCHANGED. `cc_replace_face_to_plane`
  (`include/cybercadkernel/cc_kernel.h:424`), `IEngine::replace_face_to_plane`, the
  facade `src/facade/cc_kernel.cpp:605`, and the OCCT
  `OcctEngine::replace_face_to_plane` fallback are all byte-identical; DM2 only supplies
  a native path behind the existing seam.
- **Host gate (a) — analytic, OCCT-free:** for the axis-aligned box, moving one face to
  a parallel target plane at signed normal distance `d` yields a watertight solid whose
  volume equals `V₀ + A_face · d` (fp-exact for a box) — push (`d>0`) grows, pull
  (`d<0`) shrinks; the moved face lies exactly on the target plane. A tilted target
  plane (about an in-face axis) yields a watertight convex polytope whose volume equals
  `V₀ + A_face · d̄` (the wedge integral). The result is a closed 2-manifold, single
  lump, with the original face count preserved.
- **Sim gate (b) — native-vs-OCCT parity:** on a booted iOS simulator, compare the
  native re-solved solid against the OCCT move-face oracle (`OcctEngine::
  replace_face_to_plane` for a trim; the equivalent OCCT plane-cut-and-extend / box of
  new extents for a grow) on volume, area, watertightness (closed 2-manifold), topology
  (Euler χ = 2, single solid), and bounding box, at fixed (never-widened) tolerances.
- **Out of scope (declines, documented not faked):** a curved neighbour face; a target
  plane that severs / inverts / adds / removes a neighbour (topology change); a
  non-convex operand; a coincident / sliver / empty degenerate target; a non-planar
  picked face; a foreign / OCCT / mesh-only body; the sibling DM verbs
  (`cc_replace_face` offset+tilt, project). Each returns the EXACT OCCT result. No
  `cc_*` ABI change; no OCCT in `src/native/**`; no tolerance weakened; no dead code.
