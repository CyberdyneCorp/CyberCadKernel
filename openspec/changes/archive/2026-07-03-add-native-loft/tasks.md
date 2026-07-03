# Tasks — add-native-loft

Verification levels: **host** = the native construction + math libraries compile and
unit-test with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT and NO
simulator (they MAY include `src/native/math` + `src/native/topology` +
`src/native/tessellate`), asserting EXACT/convergent closed-form properties on the
built native B-rep and its native tessellation (watertight, exact side-face / cap /
edge / vertex counts, degree-1 ruled skin surfaces — or `Plane` when coplanar,
analytic volume) — the first roadmap gate; **sim-parity** = on a booted iOS
simulator (OCCT linked), the SAME two `cc_*` calls are issued native
(`cc_set_engine(1)`) vs OCCT (default) and compared through the facade on mass
properties / bbox / sub-shape counts / watertightness against
`BRepOffsetAPI_ThruSections(ruled=true)` within a documented tolerance, and the
deferred configurations are asserted identical under both engines — the second
roadmap gate. A requirement is done only when BOTH gates are green AND every
existing suite (`scripts/run-sim-suite.sh` 221/221, host CTest, GPU/Phase-3) stays
green with the engine left at its OCCT default. The default engine SHALL remain
OCCT; the toggle is opt-in. Honest scope: mismatched point counts, punctual
sections, non-planar wires, non-closeable rulings, and every 3+-section / guided /
rail loft fall through to OCCT (do NOT fake them).

## 0. OCCT-free seam preserved
- [x] 0.1 New/extended builders live under `src/native/construct/` (and any ruled-
  surface helper under `src/native/math/`) with NO OCCT include in any TU (MAY
  include `src/native/math` + `src/native/topology` + `src/native/tessellate`); host
  CTest builds them with `clang++ -std=c++20` (no OCCT, no simulator). (**host**)
- [x] 0.2 Grep gate: no OCCT header, and no `IEngine` / `EngineShape` / OCCT type,
  appears in any new/edited `src/native/` TU. (**host**)

## 1. Ruled / skin surface (native-math)
- [x] 1.1 Confirm whether the existing `src/native/math` `Bezier`/`BSpline` surface
  primitives already express a degree-1 skin between two straight edges. If not, add
  a small ruled/skin-surface helper `S(u,v) = (1−v)·A(u) + v·B(u)` (point + normal),
  degree-1 in both directions, recognising the coplanar-corner limit as an exact
  `Plane`. Keep it OCCT-free, host-buildable. (**host**)
- [x] 1.2 Host test — analytic: a known bilinear patch point equals the hand-computed
  value; a coplanar four-corner patch equals its supporting `Plane` (normal + on-plane
  residual within fp64 tolerance). (**host**)

## 2. Native two-section ruled loft builder (`src/native/construct/`)
- [x] 2.1 Implement `build_loft_ruled(sectionA_XYZ, sectionB_XYZ, n)`: validate equal
  count `n ≥ 3`, ≥ 3 distinct points per section, neither punctual, each planar within
  tolerance, no self-intersection (else NULL). Build the `2n` vertices, `n` shared
  `Line` connecting edges `a[i]→b[i]`, `2n` section edges; one ruled side face per
  corresponding edge pair (degree-1 skin, or `Plane` when coplanar), oriented outward;
  bottom + top `Plane` caps. Assemble one closed watertight `Shell` → `Solid`; NULL if
  not watertight/manifold. Isolate the per-edge-pair loop + the planar-vs-ruled
  classifier (≤ 15 / systems-band ≤ 25). (**host**)
- [x] 2.2 Add the `cc_solid_loft` adapter `build_loft(bottomXY, topXY, n, depth)`:
  lift `bottomXY` to `z=0` and `topXY` to `z=depth`, then call `build_loft_ruled`.
  (**host**)
- [x] 2.3 Add the `cc_solid_loft_wires` adapter `build_loft_wires(aXYZ, bXYZ, n)`:
  pass the two 3D point lists straight to `build_loft_ruled`. (**host**)

## 3. Host analytic tests — `cc_solid_loft` (XY sections)
- [x] 3.1 Identical square at `z=0` and `z=d` → a native box: watertight
  (`boundaryEdgeCount==0`); exactly 6 faces (4 ruled sides + 2 caps), 12 edges, 8
  vertices; every ruled side is `Plane` (coplanar limit); volume `= area·d`, area
  matches the box. (**host**)
- [x] 3.2 Square → concentric smaller square at `z=d` (a frustum-like prismatoid):
  watertight; 4 ruled side faces + 2 caps; the side faces carry a degree-1 skin (non-
  coplanar) or `Plane` per corner test; volume equals the prismatoid formula
  `d/6·(A_bottom + 4·A_mid + A_top)`. (**host**)
- [x] 3.3 Pentagon (n=5) bottom → pentagon top → watertight; 5 side faces + 2 caps;
  correct edge/vertex counts; convergent volume. (**host**)

## 4. Host analytic tests — `cc_solid_loft_wires` (3D wires)
- [x] 4.1 Two planar squares in parallel planes given as 3D wires (equal count) →
  same watertight solid as §3.1 (proves the wire path), volume `= area·gap`.
  (**host**)
- [x] 4.2 Two planar squares in parallel-but-offset planes (a skewed prismatoid) with
  non-coplanar side quads → watertight; ruled (degree-1 skin) side faces present;
  convergent volume by the prismatoid formula. (**host**)

## 5. Host tests — deferred configurations return NULL (fall-through boundary)
- [x] 5.1 Mismatched point counts (`nA ≠ nB`) → `build_loft`/`build_loft_wires`
  returns NULL. (**host**)
- [x] 5.2 A section degenerating to a point (all points coincident) → NULL. (**host**)
- [x] 5.3 A `cc_solid_loft_wires` section that is NOT planar within tolerance → NULL.
  (**host**)
- [x] 5.4 A section with < 3 distinct points, or a self-intersecting section /
  ruling → NULL. (**host**)

## 6. NativeEngine glue (native-else-fallback)
- [x] 6.1 Change the two `NativeEngine` overrides — `solid_loft`, `solid_loft_wires`
  — from pure `fallback()` delegation to: call the native builder; on a non-NULL
  native `Shape`, `track(wrapNative(...))`; on NULL, fall through to
  `fallback().<same>(...)` unchanged. No `native_engine.h` change. (**host**)
- [x] 6.2 Keep OCCT strictly under `CYBERCAD_HAS_OCCT`; host build binds fallback to
  the stub and still compiles. The native builder references no OCCT / `IEngine` /
  `EngineShape`. (**host** compiles with stub fallback)
- [x] 6.3 Host test — engine routing: a supported two-section loft builds natively (no
  fallback call) and reads back via native tessellate / mass / bbox / subshape; a
  deferred config (mismatched count / punctual / non-planar) forwards to the fallback
  unchanged. (**host**)

## 7. Facade (no change) — confirm
- [x] 7.1 Confirm `cc_solid_loft` and `cc_solid_loft_wires` already route through the
  active engine and NO `cc_*` signature or POD layout changed by this change (diff the
  header). (**host**)

## 8. Simulator native-vs-OCCT parity (gate 2)
- [x] 8.1 Extend the sim parity harness (a `tests/sim/native_loft_parity.mm` +
  `tests/native/` checks): `cc_solid_loft` native (`cc_set_engine(1)`) vs OCCT
  (default) for (a) an identical-square box loft and (b) a square→smaller-square
  frustum-like loft — compare mass properties / bbox / sub-shape counts / watertight
  tessellation against `BRepOffsetAPI_ThruSections(ruled=true)`; restore default in
  teardown. (**sim-parity**)
- [x] 8.2 Same for `cc_solid_loft_wires` with two parallel-plane 3D square wires (full
  + offset planes). (**sim-parity**)
- [x] 8.3 Fall-through parity: with native active, the deferred configs — mismatched
  point counts, a punctual section, and a non-planar `cc_solid_loft_wires` section —
  produce results IDENTICAL to the OCCT default (fall-through proof, no native
  interception). (**sim-parity**)

## 9. Regression + docs
- [x] 9.1 Run `scripts/run-sim-suite.sh` (expect 221/221), host CTest, GPU / Phase-3
  suites with the engine left at its OCCT default — all green, no behavioural change.
  (**sim-parity** + **host**)
- [x] 9.2 Update `openspec/NATIVE-REWRITE.md` (#4b Tier B done at the bar — two-section
  ruled loft native; list which configs are native vs still-fall-through) and
  `docs/STATUS-phase-4.md` (native vs OCCT-fallthrough deltas; the mismatched-count /
  punctual / non-planar / 3+-section deferrals stated honestly). Sync/extend the
  living spec `openspec/specs/native-construction`. Update `openspec/ROADMAP.md` and
  the root README "Where OCCT is still required" split. (**docs**)
- [x] 9.3 `openspec validate add-native-loft --strict` green.

## Deferred (NOT delivered native in this change — fall through to OCCT, not faked)
Tier B is scoped to the TWO-section ruled loft with EQUAL point counts. These remain
OCCT-fallback via `NativeEngine`'s delegation (native builder returns NULL →
`NativeEngine` forwards to OCCT) and are tracked as later #4b tiers, NOT part of this
bar. Where noted, the fall-through path itself is verified in gate 2 (native active,
result identical to the OCCT default):
- [ ] Mismatched point-count sections in `cc_solid_loft` / `cc_solid_loft_wires` —
  needs a section-compatibility (re-parameterise / point-insertion) step. STILL
  OCCT-fallthrough; fall-through to be VERIFIED (task 8.3).
- [ ] Punctual (point) sections — a cone-like apex cap topology. STILL
  OCCT-fallthrough; fall-through VERIFIED (task 8.3).
- [ ] Non-planar section wires (`cc_solid_loft_wires`) whose cap is not a single
  plane — needs a general non-planar cap. STILL OCCT-fallthrough; VERIFIED (task 8.3).
- [ ] 3+ section lofts, `cc_loft_along_rail`, `cc_guided_sweep` — Tier C. STILL
  OCCT-fallthrough.
- [ ] Sweep + variants (`cc_solid_sweep`, `cc_twisted_sweep`), threads
  (`cc_helical_thread`, `cc_tapered_thread`, `cc_tapered_shank`), `cc_wrap_emboss` —
  later #4b tiers. STILL OCCT-fallthrough.
- [ ] Every feature / boolean / query / transform / exchange op — out of
  construction; delegated to the fallback.
