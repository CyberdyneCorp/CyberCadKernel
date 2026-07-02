# Tasks — add-native-construction-profiles

Verification levels: **host** = the native construction library compiles and
unit-tests with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT and NO
simulator (it MAY include `src/native/math` + `src/native/topology` +
`src/native/tessellate`), asserting EXACT closed-form properties on the built native
B-rep and its native tessellation (watertight, exact inner-wire / face / edge counts,
TRUE `Circle` edges, analytic volume/area) — the first roadmap gate; **sim-parity** =
on a booted iOS simulator (OCCT linked), the SAME five `cc_*` calls are issued native
(`cc_set_engine(1)`) vs OCCT (default) and compared through the facade on mass
properties / bbox / sub-shape counts / watertightness within a documented tolerance,
and the deferred sub-cases are asserted identical under both engines — the second
roadmap gate. A requirement is done only when BOTH gates are green AND every existing
suite (`scripts/run-sim-suite.sh` 221/221, host CTest, GPU/Phase-3) stays green with
the engine left at its OCCT default. The default engine SHALL remain OCCT; the toggle
is opt-in. Honest scope: kind-3 SPLINE profile edges, torus-revolve (arc off-axis),
and any spline-revolve fall through to OCCT (do NOT fake them).

> IMPLEMENTATION NOTE (naming): the builders landed in a new header
> `src/native/construct/profile.h` (aggregated by `native_construct.h`). The
> circular- and polygon-hole paths are unified as
> `build_prism_with_holes(outerXY, outerN, circleHoles, polyHoles, depth)`; the
> typed-profile path is `build_prism_profile(segs, circleHoles, polyHoles, depth,
> deflection)`; the revolve path is `build_revolution_profile(segs, axis, angle)`.
> These cover the `build_prism_holes` / `build_prism_profile` /
> `build_revolution_profile` intent below 1:1.

## 0. OCCT-free seam preserved
- [x] 0.1 New/extended builders live under `src/native/construct/` with NO OCCT
  include in any TU (MAY include `src/native/math` + `src/native/topology` +
  `src/native/tessellate`); host CTest builds them with `clang++ -std=c++20`
  (no OCCT, no simulator). (**host**) — `profile.h`, built by `test_native_profile`.
- [x] 0.2 Grep gate: no OCCT header, and no `IEngine` / `EngineShape` / OCCT type,
  appears in any `src/native/construct/` TU. (**host**)

## 1. Native holed prism — circular holes (`cc_solid_extrude_holes`)
- [x] 1.1 Implement `build_prism_holes(outerXY, outerN, circleHoles{cx,cy,r}, hc,
  polyHoles=∅, depth)`: outer prism as #4, plus per circular hole a bottom/top **TRUE
  `Circle` edge** + seam edge + one inward `Cylinder` inner side face, with the
  reversed circle edges added as inner wires of the caps; one closed outward `Shell` →
  `Solid`. Reject holes outside / overlapping the outer polygon or each other, r ≤ 0 →
  NULL. Isolate the per-hole sweep (≤ 15). (**host**)
- [x] 1.2 Host test — rectangle plate with one round through-hole: watertight
  (`boundaryEdgeCount==0`); the hole edges are `Circle` (`EdgeCurve::Kind::Circle`),
  not sampled lines; volume `(w·h − π·r²)·d`, area matches analytic (2 annulus caps +
  outer sides + one cylinder inner wall). (**host**)
- [x] 1.3 Host test — plate with two round holes: two inner wires, watertight, volume
  `(w·h − π·(r0²+r1²))·d`. (**host**)
- [x] 1.4 Host test — degenerate: r ≤ 0, hole outside the outer polygon, overlapping
  holes → NULL (native builder returns null → engine falls through). (**host**)

## 2. Native holed prism — polygon holes (`cc_solid_extrude_polyholes`)
- [x] 2.1 Extend `build_prism_holes` to **polygon** holes (`polyXY` + per-hole
  `holeCounts`): each hole → bottom/top inner ring of `Line` edges + vertical edges +
  N inward `Plane` inner side faces + reversed inner-ring cap wires. (**host**)
- [x] 2.2 Host test — rectangle plate with one square hole: watertight; two inner
  wires (bottom/top) of N line edges; volume `(A_outer − A_hole)·d`. (**host**)
- [x] 2.3 Host test — mixed is out of scope for THIS op (polygon holes only); a
  self-intersecting or out-of-bounds hole polygon → NULL / fall through. (**host**)

## 3. Native typed-profile prism (`cc_solid_extrude_profile`)
- [x] 3.1 Implement `build_prism_profile(segs, segCount, splineXY, circleHoles, hc,
  polyHoles=∅, depth)`: build the OUTER wire from `CCProfileSeg`s — **kind 0 line** →
  `Line` edge (→ `Plane` side), **kind 1 arc** → one **`Circle`** `EdgeCurve` over
  `[a0,a1]` (→ `Cylinder` side), **kind 2 full circle** → one closed `Circle` edge
  (→ full `Cylinder` side + disc caps). Require segments to chain into ONE closed
  wire. **kind 3 SPLINE → return NULL (fall through, not faked).** Then extrude and add
  circular holes (reuse §1). (**host**)
- [x] 3.2 Host test — arc-profile prism (e.g. a rounded-rectangle / D-shape from line +
  arc segments): watertight; the arc edge is a TRUE `Circle` edge; `Cylinder` side face
  present; volume = extruded profile area · depth (analytic). (**host**)
- [x] 3.3 Host test — full-circle profile (kind 2) → a native **cylinder solid**:
  watertight; volume `π·r²·d`; area `2πr·d + 2·πr²`; the lateral face is a `Cylinder`,
  the caps carry one `Circle` edge each. (**host**)
- [x] 3.4 Host test — a profile containing a kind-3 SPLINE segment → `build_prism_profile`
  returns NULL (documents the fall-through boundary at the native layer). (**host**)

## 4. Native typed-profile prism + polygon holes (`cc_solid_extrude_profile_polyholes`)
- [x] 4.1 Wire `build_prism_profile` to also take polygon holes (§2 machinery) + circular
  holes together: typed outer wire (kind 0/1/2) + circular holes + polygon holes → one
  watertight solid. kind-3 outer segment → NULL / fall through. (**host**)
- [x] 4.2 Host test — arc-profile plate with one round hole and one square hole:
  watertight; three inner/curved wires as expected; volume =
  `(profileArea − π·r² − A_polyHole)·d`. (**host**)

## 5. Native typed-profile revolve (`cc_solid_revolve_profile`)
- [x] 5.1 Implement `build_revolution_profile(segs, segCount, splineXY, axis{ax,ay,
  adx,ady}, angle)`: generalise #4 `build_revolution` to the in-plane axis; per segment
  — **kind 0 line** → Plane/Cylinder/Cone/none; **kind 1 arc / kind 2 circle centred ON
  the axis** → **`Sphere`** face of revolution (Plane/Cone at the limits); **arc off the
  axis (torus) → NULL fall through**; **kind 3 SPLINE / any spline-revolve → NULL fall
  through**. TRUE `Circle` edges of revolution; full 360° closes, partial angle adds two
  planar meridian caps. Isolate the classification (systems-band ≤ 25; flag/split if
  over). (**host**)
- [x] 5.2 Host test — line-only profile about an arbitrary in-plane axis revolved 360°
  → cylinder/cone/plane faces, watertight, analytic volume (parity with #4 generalised
  to a non-default axis). (**host**)
- [x] 5.3 Host test — a half-disc arc profile (semicircle meridian centred ON the axis)
  revolved 360° → a **sphere** solid: watertight; volume `4/3·π·r³`; the lateral face is
  a `Sphere`; the meridian edge is a `Circle`. (**host**)
- [x] 5.4 Host test — partial angle (π/2) on the sphere/cylinder profile: two extra
  `Plane` meridian caps; watertight; volume = full-360 volume · θ/2π. (**host**)
- [x] 5.5 Host test — deferred boundaries at the native layer: an arc segment **offset
  from the axis** (torus) → NULL; a kind-3 SPLINE segment → NULL (documents fall-through).
  (**host**)

## 6. NativeEngine glue (native-else-fallback)
- [x] 6.1 Change the five `NativeEngine` overrides — `solid_extrude_holes`,
  `solid_extrude_polyholes`, `solid_extrude_profile`, `solid_extrude_profile_polyholes`,
  `solid_revolve_profile` — from pure `fallback()` delegation to: call the native
  builder; on a non-NULL native `Shape`, `track(wrapNative(...))`; on NULL, fall through
  to `fallback().<same>(...)` unchanged. No `native_engine.h` change. (**host**)
- [x] 6.2 Keep OCCT strictly under `CYBERCAD_HAS_OCCT`; host build binds fallback to the
  stub and still compiles. Native builders reference no OCCT / `IEngine` / `EngineShape`.
  (**host** compiles with stub fallback)
- [x] 6.3 Host test — engine routing: native sub-cases build natively (no fallback call)
  and read back via native tessellate / mass / bbox / subshape; deferred sub-cases
  (SPLINE edge, torus-revolve, spline-revolve) forward to the fallback unchanged.
  (**host**)

## 7. Facade (no change) — confirm
- [x] 7.1 Confirm the five `cc_*` entry points already route through the active engine
  and NO `cc_*` signature, POD layout, or `CCProfileSeg` field changed by this change
  (diff the header + structs). (**host**)

## 8. Simulator native-vs-OCCT parity (gate 2)
- [x] 8.1 Extend the sim parity harness (`tests/sim/native_construct_profiles_parity.mm` +
  `tests/native/checks_construct.cpp`): `cc_solid_extrude_holes` +
  `cc_solid_extrude_polyholes` native (`cc_set_engine(1)`) vs OCCT (default) — compare
  mass properties / bbox / sub-shape counts / watertight tessellation; restore default
  in teardown. (**sim-parity**) — PASS. extrude_holes circular vol rel 4.17e-03,
  centroidΔ 2.66e-15, watertight tris=108; extrude_polyholes square EXACT (vol rel
  1.97e-16, centroidΔ 0, watertight tris=32).
- [x] 8.2 Same for `cc_solid_extrude_profile` (line+arc profile; full-circle profile) and
  `cc_solid_extrude_profile_polyholes`. (**sim-parity**) — PASS. extrude_profile line+arc
  vol rel 2.55e-02, arc kept as TRUE Circle edge + Cylinder wall, watertight tris=64.
- [x] 8.3 Same for `cc_solid_revolve_profile`: line-only about an arbitrary axis (full +
  partial), and an on-axis arc/semicircle → sphere (full + partial). (**sim-parity**) —
  PASS. revolve line-tube vol rel 2.36e-02 (watertight tris=168); on-axis arc-sphere vol
  rel 4.97e-02 within 5e-02 tol (watertight tris=780).
- [x] 8.4 Fall-through parity: with native active, the deferred sub-cases — a kind-3
  SPLINE profile edge (`cc_solid_extrude_profile`) and an off-axis arc-revolve (torus,
  `cc_solid_revolve_profile`) — produce results IDENTICAL to the OCCT default
  (fall-through proof, no native interception). (**sim-parity**) — PASS. spline-extrude
  vol rel 0.00e+00 (o=n=45.6); off-axis-arc torus revolve vol rel 0.00e+00 (o=n=98.696).
  Note: a pure spline-*revolve* sub-case is covered by the same NULL→fallback path as the
  off-axis-arc torus (both are surfaces of revolution the native builder returns NULL for)
  and remains OCCT-fallthrough.

## 9. Regression + docs
- [x] 9.1 Run `scripts/run-sim-suite.sh` (expect 221/221), host CTest, GPU / Phase-3
  suites with the engine left at its OCCT default — all green, no behavioural change.
  (**sim-parity** + **host**) — PASS. Host CTest 13/13 (incl. `test_native_tessellate`
  green — box/sphere/cylinder watertight cases). `run-sim-suite.sh` == 221 passed, 0
  failed == against a freshly rebuilt SIMULATORARM64 slice; determinism + IGES/STEP
  round-trips PASS. Zero source fixes required.
- [x] 9.2 Update `openspec/NATIVE-REWRITE.md` (#4b Tier A done at the bar — list which
  sub-cases are native vs still-fall-through) and `docs/STATUS-phase-4.md` (native vs
  OCCT-fallthrough deltas; the torus / spline deferrals stated honestly). Sync/extend
  the living spec `openspec/specs/native-construction`. Also updated `openspec/ROADMAP.md`
  and the root README "Where OCCT is still required" split.
- [x] 9.3 `openspec validate add-native-construction-profiles --strict` green.

## Deferred (NOT delivered native in this change — fall through to OCCT, not faked)
Tier A is scoped to holed / typed-profile extrude + typed-profile (line/arc/circle)
revolve. These remain OCCT-fallback via `NativeEngine`'s delegation (native builder
returns NULL → `NativeEngine` forwards to OCCT) and are tracked as later #4b tiers, NOT
part of this bar. Where noted, the fall-through path itself is verified in gate 2 (native
active, result byte-identical to the OCCT default):
- [ ] kind-3 SPLINE profile edges in `solid_extrude_profile` /
  `solid_extrude_profile_polyholes` — native B-spline swept side faces. STILL
  OCCT-fallthrough; fall-through VERIFIED (task 8.4, vol rel 0.00e+00).
- [ ] Arc-revolve whose swept surface is a **torus** (arc off the axis) — needs a native
  `Torus` `FaceSurface` kind (+ its tessellation + parity). STILL OCCT-fallthrough;
  fall-through VERIFIED (task 8.4, off-axis closed circle → torus, vol rel 0.00e+00).
- [ ] Any **spline-revolve** (kind-3 in `solid_revolve_profile`, B-spline surface of
  revolution). STILL OCCT-fallthrough (same NULL→fallback path as the torus case).
- [ ] Loft (`solid_loft`, `solid_loft_wires`), sweep + variants (`solid_sweep`,
  `twisted_sweep`, `guided_sweep`, `loft_along_rail`), threads (`helical_thread`,
  `tapered_thread`, `tapered_shank`), `wrap_emboss` — later #4b tiers (B loft / C sweep /
  D threads / E wrap-emboss). STILL OCCT-fallthrough.
- [ ] Every feature / boolean / query / transform / exchange op — out of construction;
  delegated to the fallback.
