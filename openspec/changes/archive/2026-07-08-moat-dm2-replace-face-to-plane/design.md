# Design — moat-dm2-replace-face-to-plane

## Context

`cc_replace_face_to_plane` is the app's push/pull "move a face to a target plane": pick
one planar face of a solid, hand it a target plane `(tp, n)`, and re-solve the adjacent
faces so the solid stays watertight. The OCCT oracle
(`OcctEngine::replace_face_to_plane`, `src/engine/occt/occt_feature.cpp:263`) implements
this as a **half-space cut**: build the target plane as a large face, make the
half-space on the side the picked face faces (outward), and `BRepAlgoAPI_Cut` it from
the body so the target plane becomes the new face and the neighbours are trimmed to it.

That is exactly the machinery DM1 (`cc_split_plane`) already reproduces natively. DM2 is
the additive assembly of DM1 + the landed BSP boolean + construct + heal behind the
existing facade seam, with a mandatory re-solve self-verify and honest declines.

## Goals / Non-Goals

**Goals.**
- Native `replace_face_to_plane` for a **convex planar polyhedron** (box / convex
  prism), moving ONE planar face to a target plane that keeps the result a bounded
  convex 2-manifold with the SAME face adjacency (each original neighbour still
  contributes one planar face; the picked face is now on the target plane).
- Cover **parallel push (grow)**, **parallel pull (trim)**, and a **tilted target
  plane (mixed grow/trim re-solve)**.
- A **closed-form volume oracle**: `ΔV = A_face · d̄`.
- Self-verify → OCCT fallback; NEVER emit a wrong/leaky/inverted/multi-lump solid.

**Non-Goals.**
- Curved neighbours (cylinder / cone / sphere / spline side faces) — DECLINE.
- Topology-changing moves (sever, invert, add/remove a neighbour) — DECLINE.
- Non-convex operands — DECLINE (the half-space intersection would not reproduce a
  concavity).
- The sibling DM verbs (`cc_replace_face` offset+tilt, project) — separate slices.
- Any `cc_*` ABI change; any OCCT in `src/native/**`; any tolerance weakening.

## Decisions

### D1 — Move semantics and the volume oracle

The picked planar face `F` has outward normal `n_F` and lies on plane `P_F`. The target
plane is `P_t = (tp, n)`. Every point currently on `F` is displaced onto `P_t`. For a
point `p` on `F`, its signed displacement along `n_F` is `δ(p) = ((tp − p) · n) / (n ·
n_F)` (the ray from `p` along `n_F` to `P_t`). Because `F` is planar and `P_t` is a
plane, `δ` is **affine over F**, so the swept wedge volume is the integral of `δ` over
`F`, which equals `A_F · d̄` with `d̄ = δ(centroid(F))` the average displacement:

> `ΔV = V' − V₀ = A_F · d̄`.

- **Parallel offset** (`n ∥ n_F`): `d̄ = d`, the constant normal offset. Box: `ΔV`
  fp-exact `A_F · d`.
- **Tilt** (`P_t` rotated about an in-face axis through the centroid): `d̄` is the
  centroid offset (the raised corners cancel the lowered corners about the axis).

Sign: `d̄ > 0` is a **grow** (target plane beyond `F` outward), `d̄ < 0` is a **trim**.

### D2 — Three-way dispatch, all from landed verbs

`replaceFaceToPlane(solid, faceId, P_t)` reads `F` and `n_F` from `topology`
(READ-ONLY) and dispatches:

1. **Pure trim** — `d̄ < 0` AND the target plane keeps the body bulk and only removes
   an outward slab from every neighbour (no neighbour fully clipped away). This is a
   DM1 split keeping the bulk half:
   `boolean::splitByPlane(operand, tp, n, keepPositive = bulk-side)`.
   CONSUMED byte-identical; already self-verifies watertight and has the DM1
   partition-closure gate.

2. **Pure grow, parallel** — `d̄ > 0` AND `n ∥ n_F`. The added region is the slab `F`
   extruded to `P_t`:
   `slab = construct::build_prism(faceLoop(F), n_F · d)`;
   `result = boolean::boolean_solid(operand, slab, Op::Fuse)`; then heal weld.

3. **General convex re-solve (GROW-then-TRIM)** — a tilted `P_t`, or a mixed grow/trim
   on a convex polyhedron where part of the moved face travels OUTWARD past `P_F`. The
   re-solve is TWO already-gated ops, NOT an N-cut half-space chain (see the chain-blocker
   in Risks):
   (a) **grow** the operand outward so it fully covers the target plane's reach — a
   parallel slab `slab = construct::build_prism(faceLoop(F), n_F · G)` extruded to a
   depth `G` past `max_p∈F δ(p)`, fused: `grown = boolean::boolean_solid(operand, slab,
   Op::Fuse)`;
   (b) **trim** by a SINGLE tilted cut to the target plane, keeping the bulk:
   `S' = boolean::splitByPlane(grown, tp, −n, keepPositive = bulk-side)`;
   then heal weld. One Fuse + one BSP Cut, each individually watertight-gated — the same
   two primitives cases 1–2 use, so it inherits their robustness. A pure-trim tilt (the
   moved face stays inside the operand everywhere) skips step (a) and is a single cut like
   case 1.

All three feed the SAME self-verify (D3). Cases 1–2 are cheap specializations that keep
the box push/pull on the tightest possible path; case 3 is the uniform fallback for the
tilted re-solve. If a specialization declines, the driver may retry via case 3 before
declining to OCCT.

### D3 — Mandatory re-solve self-verify (the gate before acceptance)

The native result is accepted as native ONLY when ALL hold (measured on the mesher's
deflection ladder, planar meshes exact — the SAME `watertightVolume` /
`tessellate::isWatertight` predicate DM1 uses):

1. **Watertight** closed 2-manifold (every undirected mesh edge used by exactly two
   triangles).
2. **Positive** enclosed volume.
3. **Moved face on the target plane**: every sample of the face now on `P_t` satisfies
   `|n·(p − tp)| ≤ tol`.
4. **Single lump**: one connected solid, Euler χ = 2 (the move did not sever the body).
5. **Face adjacency preserved**: the resolved face count equals the original (no
   neighbour inverted / removed, no new face introduced) — the convex-re-solve
   invariant. (Face IDs are reissued, as OCCT also reissues; the check is on count +
   watertightness, not on stable IDs.)

If the candidate is NULL (a consumed verb declined) OR fails ANY check, the engine
DISCARDS it and returns EXACTLY `OcctEngine::replace_face_to_plane` for that call.

### D4 — Honest declines (first-class, measured, never faked)

The native branch returns NULL → OCCT for every case outside the convex-planar slice:

- **Curved neighbour** — a picked-face neighbour is a `Cylinder`/`Cone`/`Sphere`/
  `BSpline`/`Bezier` face; the planar re-solve cannot retrim a curved side to the moved
  plane.
- **Topology-changing target plane** — the plane clips PAST an adjacent face (fully
  removes a neighbour), inverts a neighbour, severs the body into >1 lump, or introduces
  a new face. Detected by the D3 single-lump + face-count checks failing.
- **Non-convex operand** — the half-space intersection would not reproduce a concave
  solid (case-3 reconstruction fails the watertight / volume self-verify vs the
  operand's own volume).
- **Degenerate target** — coincident with the picked face (`|d̄| ≤ tol`, no-op), or a
  zero-volume sliver / empty result.
- **Non-planar picked face** — `surfaceOf(faceId)` is not a `Plane`.
- **Foreign / mesh-only body** — no native B-rep to re-solve.

Under the native engine each returns the EXACT OCCT result of the same call, proving
fall-through with no native interception. No case is stubbed, faked, or partially
emitted; a wrong, open, inverted, or multi-lump solid is NEVER emitted.

### D5 — Module boundary and OCCT-freeness

New module `src/native/directmodel/` (namespace `cybercad::native::directmodel`),
header-only, `clang++ -std=c++20`, includes ONLY
`src/native/{boolean,construct,heal,topology,tessellate,math}` headers — **zero OCCT
includes**. It CONSUMES the landed verbs read-only and edits none of them. The engine
glue lives in `src/engine/native/native_engine.cpp` (which may reference the native
headers but not OCCT), `CYBERCAD_HAS_NUMSCI`-guarded exactly like the DM1 branch, so
without the numsci substrate the native path is absent and the call falls straight to
the honest OCCT fall-through (the pre-DM2 behaviour).

## Risks / Trade-offs

- **Convex-only re-solve.** The grow-then-trim reconstruction (case 3) is exact only for
  convex polyhedra. A concave planar solid whose move would stay valid is DECLINED (safe,
  not wrong) — a later slice can add a per-face local retrim. The D3 volume self-verify
  catches any concavity mismatch.
- **N-cut half-space chain is NOT robust (DIAGNOSE finding, empirical).** Reconstructing
  a polytope as `∩_i H_i` by chaining bbox-scaled BSP half-space cuts breaks down: probing
  the box on the host (no OCCT), a 6-axis reconstruction of `[0,10]³` from a superset box
  fails the watertight self-verify at the **4th cut** (`HalfSpaceCutDecline::NotWatertight`)
  as the accumulated near-coplanar cutting-box faces produce slivers the BSP rejects. The
  design therefore uses **grow-then-trim (1 Fuse + 1 single Cut)** for the tilted re-solve
  instead of a Common/Cut chain. On the same probe, cases 1/2/3(single-cut trim)/4a/4b
  (grow+trim, `d̄ = 0` and `d̄ = +2`) all yield watertight solids matching the closed-form
  `A·d̄` volume to ≤ 1e-12 (fp-exact); a curved-neighbour move NULLs out (honest decline).
- **Face-ID reissue.** The re-solve emits a fresh solid, so downstream face IDs change
  (OCCT does the same). Acceptable for a move-face; the gate is geometric, not ID-based.
- **Grow vs OCCT's cut-only oracle.** OCCT's shipped `replace_face_to_plane` only
  trims (a half-space cut cannot grow). The SIM grow gate therefore compares against an
  OCCT **plane-cut-and-extend** reference (a box of the new extents / `BRepFeat`
  prism), which is a legitimate OCCT move-face oracle, at fixed tolerances.

## Migration / Rollout

Additive behind the existing facade seam. No `cc_*` signature changes, no OCCT in
`src/native/**`. DM2 is inert without the numsci substrate (guarded), so it cannot
regress the OCCT path. Verification is the two gates (host analytic + sim
native-vs-OCCT); an honest decline for the out-of-scope cases is a first-class outcome.
