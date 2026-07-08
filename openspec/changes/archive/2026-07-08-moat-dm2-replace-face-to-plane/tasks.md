# Tasks — moat-dm2-replace-face-to-plane (MOAT M-DM DM2)

Order: substrate build → move classification + face read → three-way dispatch (trim /
parallel grow / convex re-solve) → re-solve self-verify → additive engine branch → host
analytic gate (A) → sim native-vs-OCCT gate (B) → docs, or HONEST DECLINE. All new
native code is header-only, OCCT-free, host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::directmodel`. DM2 CONSUMES DM1 `split_plane.h` + the BSP boolean +
construct + heal READ-ONLY; it does NOT modify them. `cc_*` additive-only; no existing
signature changes. No tolerance weakened; a measured decline (curved neighbour /
topology change / non-convex / degenerate / non-planar face / foreign body) is a
first-class outcome — a wrong, open, inverted, or multi-lump solid is NEVER emitted.

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
      export `CYBERCAD_NUMSCI_DIR` to `build-numsci/host` (host gate) /
      `build-numsci/iossim` (sim gate).
- [x] 0.2 Confirm the consumed parts build and are on the verified path:
      DM1 `boolean::splitByPlane` (`boolean/split_plane.h`), `boolean::boolean_solid`
      with `Op::{Fuse,Cut,Common}` (`boolean/native_boolean.h`,
      `native_boolean_fwd.h`), `construct::build_prism`
      (`construct/native_construct.h`), the `heal/*` weld, and the engine
      `watertightVolume` / `tessellate::isWatertight` audit — all READ-ONLY from
      `directmodel/`.

## 1. Move classification + face read (`src/native/directmodel/replace_face.h`, OCCT-free)

- [x] 1.1 Read the picked face `F` from `topology` by `faceId` (READ-ONLY);
      `surfaceOf(F)` must be a `Plane` else DECLINE (non-planar face).
- [x] 1.2 Compute the outward normal `n_F`, plane `P_F`, face loop, area `A_F`, and
      centroid; build the target plane `P_t = (tp, normalize(n))` (DECLINE on a
      degenerate `n`).
- [x] 1.3 Compute the average displacement `d̄ = δ(centroid)` with
      `δ(p) = ((tp − p)·n)/(n·n_F)`; classify grow (`d̄ > tol`) / trim (`d̄ < −tol`) /
      no-op (`|d̄| ≤ tol` → DECLINE, degenerate/coincident).

## 2. Three-way dispatch — all from landed verbs (`replace_face.h`, OCCT-free)

- [x] 2.1 **Pure trim** (`d̄ < 0`, target keeps the body bulk, no neighbour fully
      clipped) → `boolean::splitByPlane(operand, tp, n, keepPositive = bulk-side)`
      (DM1), CONSUMED byte-identical.
- [x] 2.2 **Pure grow, parallel** (`d̄ > 0`, `n ∥ n_F`) →
      `slab = construct::build_prism(faceLoop(F), n_F · d)`;
      `boolean::boolean_solid(operand, slab, Op::Fuse)`; then heal weld.
- [x] 2.3 **General convex re-solve** (tilted `P_t` / mixed grow-trim) → **grow-then-trim**
      (NOT an N-cut chain — that breaks the watertight self-verify at ~4 cuts, per the
      DIAGNOSE finding): `grown = boolean::boolean_solid(operand, build_prism(faceLoop(F),
      n_F·G), Op::Fuse)` extended past the target reach, then a SINGLE
      `boolean::splitByPlane(grown, tp, −n, keepPositive = bulk-side)`; heal weld. A
      pure-trim tilt skips the grow (single cut).
- [x] 2.4 A declining specialization (2.1 / 2.2) MAY retry via 2.3 before declining to
      OCCT.

## 3. Re-solve self-verify (`replace_face.h`, the gate before acceptance)

- [x] 3.1 Accept the candidate ONLY when: watertight closed 2-manifold; positive
      enclosed volume; every moved-face sample on `P_t` (`|n·(p−tp)| ≤ tol`); single
      lump (Euler χ = 2); and face count preserved (no neighbour inverted/removed, no
      new face). Uses the SAME `watertightVolume` / `tessellate::isWatertight` predicate
      as DM1.
- [x] 3.2 NULL (a consumed verb declined) OR any failed check → return a typed decline
      (never emit the candidate).

## 4. Additive engine branch (`src/engine/native/native_engine.cpp`)

- [x] 4.1 Replace the `CC_NATIVE_BODY_UNSUPPORTED` body of
      `NativeEngine::replace_face_to_plane` with: `if (!isNative(body)) return
      fallback().replace_face_to_plane(...)`; then a `CYBERCAD_HAS_NUMSCI`-guarded
      native attempt via `directmodel::replaceFaceToPlane`.
- [x] 4.2 On a verified native result (`§3` passes AND `watertightVolume > 0`) → track
      and return `wrapNative`. Else fall through to
      `fallback().replace_face_to_plane(...)` (OCCT) — the true fall-through for a
      move-face operand.
- [x] 4.3 No signature change (`native_engine.h:225` override already exists); mirror
      the DM1 `split_plane` branch shape. Keep cognitive complexity in the systems band
      (delegate classify / dispatch / self-verify to helpers).

## 5. Host analytic gate (A) — no OCCT

- [x] 5.1 Axis-aligned box, move one face to a **parallel** target at signed `d`:
      result watertight, single lump, moved face on `P_t`, face count preserved, and
      `V' = V₀ + A_F·d` fp-exact — push (`d>0`) grows, pull (`d<0`) shrinks.
- [x] 5.2 Axis-aligned box, move one face to a **tilted** target (about an in-face axis
      through the centroid): result watertight convex polytope,
      `V' = V₀ + A_F·d̄` within the deflection tolerance.
- [x] 5.3 A verified native result is read back (mass / bbox / sub-shape ids /
      tessellation) by the native body-consuming paths with no OCCT fallback call.

## 6. Sim native-vs-OCCT gate (B) — booted simulator, OCCT oracle

- [x] 6.1 For the box trim (pull) fixture, compare the native result vs
      `OcctEngine::replace_face_to_plane` on volume, area, watertightness, topology
      (Euler χ = 2, single solid), and bbox, at fixed (never-widened) tolerances.
- [x] 6.2 For the box grow (push) and the tilted re-solve, compare vs the OCCT
      plane-cut-and-extend reference (box of new extents / `BRepFeat` prism) on the same
      quantities and tolerances.

## 7. Declines (first-class, measured, never faked)

- [x] 7.1 Curved neighbour (cylinder/cone/sphere/spline side face) → NULL → OCCT;
      identical to `cc_set_engine(0)`.
- [x] 7.2 Topology-changing target plane (severs / inverts / removes / adds a
      neighbour) → NULL → OCCT (caught by the §3 single-lump + face-count checks).
- [x] 7.3 Non-convex operand → NULL → OCCT (caught by the §3 volume self-verify).
- [x] 7.4 Degenerate target (coincident / sliver / empty) and non-planar picked face →
      NULL → OCCT.
- [x] 7.5 Foreign / mesh-only body → NULL → OCCT.

## 8. Discipline + docs

- [x] 8.1 `src/native/directmodel/**` has ZERO OCCT includes (grep-clean); consumed
      verbs edited nowhere.
- [x] 8.2 `cc_*` ABI byte-identical (facade / `IEngine` / OCCT fallback unchanged); no
      tolerance weakened; no dead code.
- [x] 8.3 Update `openspec/MOAT-ROADMAP.md` §M-DM DM2 status; `openspec validate
      moat-dm2-replace-face-to-plane --strict` passes.
