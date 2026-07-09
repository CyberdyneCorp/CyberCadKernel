# Design — moat-m6l-section-fuzz (MOAT M6-breadth-12)

## Context

The MOAT completeness bar (M6) requires *proven* native correctness across every domain
before OCCT can be removed. The section-curve service (`cybercad::native::section::
sectionByPlane`, `src/native/section/section.h`, GS2) is native-served through the additive
`cc_section_plane` facade and covered so far only by a small FIXED-fixture parity gate
(`native_section_parity.mm`). This change adds a SEEDED differential FUZZER — the twelfth
independent domain on the bar.

## Why section is the right pick

- **Distinct, un-fuzzed.** No seeded fuzzer exercises the section service today; the parity
  gate is a handful of hand-chosen cuts. Random dimensions AND random cut planes (including
  OBLIQUE) are new coverage.
- **Pristine EXACT arbiter.** The section of an elementary solid is an elementary conic
  whose perimeter and area are closed-form in fp64. The native loop's `length()`/`area()`
  ARE those closed forms, so native-vs-analytic is EXACT to machine epsilon — a large-margin
  correctness oracle that no meshing-deflection fuzzer has.
- **Stable / not-M3-overlapping.** `src/native/section/` is a separate directory from the
  M3-active `src/native/{blend,feature,boolean}`; it reads `src/native/{math,topology,ssi}`
  READ-ONLY (also not M3-touched). No fillet / shell / offset / wrap-emboss is involved.
- **OCCT-free, no numsci.** Like the parity gate, the harness compiles only the OCCT-free
  native math TUs + header-only section/topology/ssi-S1, and links the OCCT oracle
  toolkits.

## System under test and the two independent references

- **SUT — the native section service.** `sectionByPlane(solid, cutPlane)` intersects every
  face of the solid with the cut plane (closed-form conics via the SSI Stage-S1
  intersector), clips edges to the finite face, and assembles closed loops. It returns EXACT
  analytic loops with a closed-form `length()`/`area()`, or HONESTLY DECLINES (Empty /
  Declined) a config it does not robustly handle (plane coincident/tangent to a face, an
  open section, a curved-face conic trimmed by the finite face, a freeform/torus face).
- **PRIMARY reference — the closed-form conic.** Rectangle `2(w+h)` / `w·h`; circle `2πR` /
  `πR²`; axial rectangle `2(2R+H)` / `2R·H`; ellipse Ramanujan-II perimeter / `πab`
  (`a=R/|cosθ|`, `b=R`); sphere circle `2πr` / `πr²` (`r=√(R²−d²)`). Exact for the
  elementary targets — the certifying correctness signal.
- **SECONDARY reference — OCCT `BRepAlgoAPI_Section`.** Loop count via
  `ShapeAnalysis_FreeBounds` wire recovery, edge length via `GCPnts_AbscissaPoint::Length`
  (the adaptive arc-length integrator `native_section_parity` proved converges to the true
  perimeter — `BRepGProp::LinearProperties` under-resolves an analytic Ellipse by ~1e-4),
  capped area via `BRepGProp::SurfaceProperties` on the section face(s).

## Tolerance (deflection-matched, never widened)

Section geometry is EXACT analytic — there is NO tessellation / deflection. So:

- **Straight / circular** length + area: TIGHT `1e-9` relative (native = analytic = OCCT to
  machine epsilon; observed `dLen == 0`).
- **Ellipse PERIMETER only** (the sole approximated quantity): `1e-4` relative — the
  Ramanujan-II series error (≪1e-4 at these eccentricities) matched to OCCT's arc-length
  integrator accuracy, the SAME bound `native_section_parity` proved. Never widened here.
- **Area** is exact closed-form for every family (`πab` is exact): `1e-6` relative vs OCCT
  `SurfaceProperties`.

The tight analytic bound is the certifying arbiter; the OCCT bound is matched to OCCT's own
integrator accuracy and is NOT loosened to launder a disagreement.

## Families and generation

Six families, the first `F_COUNT` trials forced so every family is exercised, then uniform:

| family      | solid    | cut plane                                   | section  | analytic |
|-------------|----------|---------------------------------------------|----------|----------|
| `BOX`       | box      | axis-aligned interior (x/y/z = const)       | rectangle| exact    |
| `CYL_PERP`  | cylinder | interior z = const                          | circle   | exact    |
| `CYL_AXIAL` | cylinder | through the axis (random azimuth)           | rectangle| exact    |
| `CYL_OBL`   | cylinder | oblique θ∈~7°..36°, H tall (ellipse in-band)| ellipse  | exact*   |
| `SPHERE`    | sphere   | random unit normal, |d| < R                 | circle   | exact    |
| `DECLINE`   | box/sphr | plane MISSING / COINCIDENT with a face       | (none)   | —        |

*ellipse perimeter via Ramanujan-II (≪1e-4); area `πab` exact.

The RNG is splitmix64 → xoshiro256** keyed ONLY by an explicit `FUZZ_SEED` (argv/env, fixed
default) — NO clock, NO `rand()`; same seed → byte-identical batch.

## Classifier

- **AGREED** — native produced a section AND native = closed-form analytic = OCCT within the
  family tolerance (loop count matches too).
- **HONESTLY-DECLINED** — native reported Empty/Declined (plane missing / coincident /
  tangent, or a numerically marginal cut its self-verify rejects) → OCCT ships. First-class,
  logged, NOT a bar failure.
- **DISAGREED** — native produced a section OUTSIDE the analytic truth, OR produced a
  section on a config the harness expects it to decline. A genuine SILENT WRONG SECTION —
  FAILS the bar, printed with seed + case index + full descriptor.
- **ORACLE_UNRELIABLE** — native matches exact math while OCCT does not (OCCT the outlier).
  Native vindicated; logged, gated off, NOT a native fault.
- **ORACLE_BAD** — OCCT produced no usable section where native + analytic both cover it.
  Gated off; investigate the oracle.

The bar: `DISAGREED == 0 && ORACLE_UNRELIABLE == 0 && ORACLE_BAD == 0` with each AGREE
family ≥1 AGREED and the decline exerciser ≥1 HONESTLY-DECLINED, across ≥2 seeds at N≥60.

## Key decision — the exact-tangency knife-edge is NOT forced (and why that is honest)

An early run classified a few "plane tangent to a sphere pole" trials as DISAGREED. Localised
at the C++ boundary:

- At `d == R` EXACTLY the native service DECLINES (open-conic arc-trim not in slice).
- At `d = R − 1e-9` it returns the mathematically-CORRECT sub-micron circle
  (`r ≈ 1e-4`, `length ≈ 6.3e-4`).
- OCCT rounds that tiny circle to an empty section (0 wires).

So the apparent DISAGREE was native being RIGHT (a plane a sub-nanometre INSIDE the sphere
DOES cut a real tiny circle) while OCCT dropped it — the OPPOSITE of a native fault. Forcing
a decline on that measure-zero knife-edge would flag a correct native section as wrong. The
decline exerciser was therefore re-scoped to only UNAMBIGUOUS declines (plane clearly
missing the solid / coincident with a planar face). NO tolerance was touched, NO native
disagreement was hidden, and NO product code changed — this is a test-generator scoping fix,
not a paper-over.

## Non-goals (documented, deferred)

- **Oblique BOX cut** (a triangle / pentagon polygon — still exact, but not a simple
  rectangle closed form) — future breadth. The box AGREE family is a clean axis-aligned
  rectangle.
- **Freeform / torus faces** and **arc-trimmed curved-face conics** — honestly DECLINED by
  the native service; route to HONESTLY-DECLINED, never a DISAGREE.
- **Non-rigid section metrics** beyond length / loop count / capped area — out of scope.
