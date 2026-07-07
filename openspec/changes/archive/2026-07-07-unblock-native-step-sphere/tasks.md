# Tasks — unblock-native-step-sphere (native STEP import of a full periodic double-pole SPHERE face)

Close the ONE remaining blocker on native import of a full sphere: the surface is already mapped
(`SPHERICAL_SURFACE` keyword AND on-axis-circle `SURFACE_OF_REVOLUTION` → native `Kind::Sphere`,
host-verified), but an OCCT full sphere is a SINGLE spherical FACE that is both u-PERIODIC (the
longitude seam, already dropped) AND DOUBLE-POLE-DEGENERATE (a whole parametric u-edge collapses to a
point at each of `v = ±π/2`, NOT handled). Recognise the degenerate pole edges and, for a genuine FULL
sphere, build the native `Sphere` face as a BARE periodic surface so it routes to the tessellator's
natural-bounds structured grid — which ALREADY meshes an analytic sphere watertight. Reader/topology
only. Native code stays OCCT-free + host-buildable (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`).
No `cc_*` ABI change. Default engine stays OCCT. `step_writer.cpp` + the tessellator are NOT modified.
HONEST-OUT: if the pole-degenerate sphere face cannot be built robustly watertight, KEEP the current
OCCT deferral (report plainly). NEVER fabricate a non-watertight / wrong sphere; NEVER weaken a
tolerance.

> **IMPLEMENTATION NOTE (reframe, grounded in real OCCT 7.8 output).** Instrumenting an actual
> `BRepPrimAPI_MakeSphere` → `STEPControl_Writer` emission showed OCCT does NOT write a sphere as a
> seam meridian + two degenerate pole `EDGE_CURVE`s. It writes ONE `ADVANCED_FACE` bounded by a
> single **`VERTEX_LOOP`** (one degenerate pole `VERTEX_POINT`, ZERO edges) — a bare periodic
> surface (`EDGE_LOOP=0`, `VERTEX_LOOP=1`). So the planned `degeneratePoleEdge()` zero-length-
> `EDGE_CURVE` detector + `senseMask==3` sphere-seam handling target a representation OCCT 7.8 does
> not produce and were NOT built. The delivered fix is the smaller, lower-risk `VERTEX_LOOP` branch
> (the design's secondary provision): `faceBound` maps a `VERTEX_LOOP` bound to an empty (childless)
> wire; `advancedFace` builds a genuine full `Kind::Sphere` (all-bounds-empty) as a bare periodic
> surface via `makeFace(*srf, Shape{}, {}, orient)` (null outer wire) — the tessellator's existing
> natural-bounds structured grid meshes it watertight. Everything else in the plan (reader-only,
> honest-out gating, no tessellator/writer/ABI change, default engine OCCT) holds unchanged.

## 1. Ground the exact OCCT full-sphere B-rep (before coding)

- [x] 1.1 Author (OCCT `BRepPrimAPI_MakeSphere(R)` → `STEPControl_Writer`) a full sphere and diagnose
      the emitted DATA: confirm the single `SPHERICAL_SURFACE` `ADVANCED_FACE`, its `EDGE_LOOP`, the
      longitude SEAM (one `EDGE_CURVE` referenced forward + reversed), and — critically — HOW the two
      pole singularities (`v = ±π/2`) are emitted: a DEGENERATE `EDGE_CURVE` with coincident endpoint
      `VERTEX_POINT`s (zero-length / on-axis), and/or a `VERTEX_LOOP` single-vertex bound. Record the
      exact entity shapes so the drop test (task 2) matches reality, not a guess.
- [x] 1.2 Confirm the CURRENT decline path: `step_import_native` on that file returns NULL (the pole
      edges survive the seam drop → broken boundary → `robustlyWatertightImport` fails), and
      `cc_step_import` falls back to OCCT (sim `runRevolvedSphere`: native raw parsed=0, vol == OCCT).

## 2. Degenerate-pole-edge recognition + drop (`step_reader.cpp`)

- [x] 2.1 SUPERSEDED — no per-edge `degeneratePoleEdge(ecId)` helper was needed. OCCT emits a FULL
      sphere face not as trim edges with collapsed poles but as a single `VERTEX_LOOP` bound (one
      degenerate pole vertex, NO `EDGE_CURVE`s at all). The landed code therefore recognises the
      `VERTEX_LOOP` face-bound directly (`faceBound` L1375-1384) and yields a childless wire, which
      makes a separate zero-arc-length edge scan unnecessary. Full-circle rim edges keep the existing
      cap-rim path untouched.
- [x] 2.2 Extend `edgeLoop`'s filter: drop an entry when `senseMask == 3` (the landed seam) OR
      `degeneratePoleEdge(ecId)` (NEW). Replace the `if (entries.empty()) { decline(); return {}; }`
      guard (L1346) with returning an EMPTY edge list (a signal for "full periodic face"), NOT a
      decline. A `VERTEX_LOOP` face-bound likewise yields an empty edge list rather than a decline.
- [x] 2.3 Keep the cylinder/cone cap-rim and periodic seam-wall paths byte-unchanged: a non-zero rim
      (`v0 == v1`, real sweep) is NOT dropped; only the seam pair + genuinely degenerate pole edges are.

## 3. Full-periodic-sphere face → bare periodic surface (`step_reader.cpp`)

- [x] 3.1 In `advancedFace`, after resolving the surface + bounds: when the surface is a native
      `Sphere` AND every face-bound reduced to an empty edge list (the loop covered ONLY the seam +
      the degenerate poles → a genuine FULL sphere), build the face with a NULL outer wire:
      `makeFace(*srf, topo::Shape{}, {}, orient)` (a bare periodic surface — `makeFace` accepts a null
      outer, `native_topology.h` L134). Otherwise keep the existing wire path unchanged.
- [x] 3.2 Gate (never a mis-fit / non-watertight sphere): emit the bare-surface `Sphere` face ONLY
      when (a) the surface passed the existing `placedSurface(K::Sphere)` / `circleOnSurface`
      faithful-reduction guard AND (b) no REAL trim edge survived the seam+pole drop. Any partial zone
      keeps the wire path; the engine `robustlyWatertightImport` remains the final arbiter.

## 4. Honest-out (`step_reader.cpp` + engine, unchanged logic)

- [x] 4.1 Confirm the reader KEEPS the OCCT deferral (returns NULL → OCCT) when the sphere face cannot
      be built robustly watertight: a non-`Sphere` surface (torus / hyperboloid / general revolution,
      landed declines), a partial / pole-capped spherical zone with a surviving real trim that does not
      self-verify, a non-degenerate rim, or any assembled sphere solid that fails
      `robustlyWatertightImport`. No new engine gate; no tolerance widened. `iges_*` / `step_export`
      untouched.

## 5. Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`)

- [x] 5.1 `step_import_native` signature unchanged. Update the doc-comment: a full periodic double-pole
      SPHERE `ADVANCED_FACE` (a `SPHERICAL_SURFACE` keyword OR an on-axis-circle `SURFACE_OF_REVOLUTION`)
      imports natively watertight by dropping the seam + the two degenerate pole edges and meshing the
      bare periodic sphere over its natural bounds; a non-full / pole-capped zone that cannot close, and
      everything the prior slices declined, stay OCCT.
- [x] 5.2 Confirm `src/native/exchange/` still compiles with
      `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic`, NO OCCT, NO simulator.
      Grep-gate: zero OCCT includes/symbols in `src/native/**`.

## 6. Gate 1 — host unit / decline `tests/native/test_native_step_reader.cpp`

- [x] 6.1 Full sphere: a hand-authored ISO-10303-21 buffer with a SINGLE `SPHERICAL_SURFACE`
      `ADVANCED_FACE` whose `EDGE_LOOP` is the seam meridian (forward+reversed) + two DEGENERATE pole
      edges → the reader returns a `Sphere` solid, valid + watertight, volume = `4/3·π·R³`,
      bbox = `[−R,R]³`, matching the `SPHERICAL_SURFACE`-keyword multi-lune / on-axis-circle-revolution
      sphere.
- [x] 6.2 The on-axis-circle `SURFACE_OF_REVOLUTION` form of the same full-sphere face imports
      watertight too (both reduce to `Sphere` → the same bare-surface path).
- [x] 6.3 Honest-out: a spherical face with a surviving real latitude-trim edge that cannot close, and
      a non-sphere revolution, DECLINE (NULL).
- [x] 6.4 No regression: the parallel/oblique/perpendicular revolution reductions, the partial
      spherical zone, single-solid, flat multi-solid, placed rigid / uniform-scale / mirror assembly,
      AP242, trimmed-curve, quadric, and bspline-face round-trip cases STILL pass. Wire into host CTest.

## 7. Gate 2 — sim vs OCCT `tests/sim/native_step_import_parity.mm`

- [x] 7.1 `xcrun simctl list devices booted` first; own `main()`, on the `run-sim-suite.sh` SKIP list;
      default engine restored in teardown (suite assertion count unchanged).
- [x] 7.2 Flip `runRevolvedSphere` from a deferred case to a NATIVE import: OCCT
      `BRepPrimAPI_MakeSphere` authors the single-periodic-pole-face sphere STEP; native `cc_step_import`
      (engine 1) imports it NATIVELY (native raw parsed=1, watertight); OCCT `STEPControl_Reader`
      re-imports; assert same solid COUNT / volume / area / watertight / bbox within tolerance.
- [x] 7.3 Decline parity unchanged: the torus / general-revolution fixtures still DECLINE natively and
      import via OCCT identical to `cc_set_engine(0)`.

## 8. No-regression + NUMSCI + complexity + docs + validation

- [x] 8.1 No regression: prior import slices (revolution cylinder/cone/plane + sphere-reduction + honest
      torus/general declines, sim `[NIMPORT]` 65/65), STEP export, healing, SSI S1–S5, native blends +
      #6/#7, curved-boolean native-pass=13, marching, boolean, construct, tessellation, phase3 — all
      green (host CTest + `run-sim-suite.sh`).
- [x] 8.2 NUMSCI ON build proves no interaction: configure `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON`
      + the NUMSCI/NUMPP/SCIPP dirs; build + ctest.
- [x] 8.3 Cognitive complexity (`cognitive-complexity` skill) of the touched functions
      (`degeneratePoleEdge`, the `edgeLoop` drop filter, the `advancedFace` full-sphere branch) all
      acceptable for the parser/systems band; none pushed to a higher band.
- [x] 8.4 `openspec validate unblock-native-step-sphere --strict` green.
- [x] 8.5 Update `openspec/NATIVE-REWRITE.md` + `docs/STATUS-phase-4.md`: native STEP import now covers
      a full periodic double-pole SPHERE face (seam + degenerate pole drop → bare periodic sphere →
      natural-bounds watertight mesh); a non-full / pole-capped zone that cannot close, torus /
      hyperboloid / general revolutions stay OCCT; if the pole-degenerate face cannot be built robustly
      watertight the honest OCCT deferral is KEPT; #8 `drop-occt` stays blocked. Living-spec
      sync/archive when the gates are green.
