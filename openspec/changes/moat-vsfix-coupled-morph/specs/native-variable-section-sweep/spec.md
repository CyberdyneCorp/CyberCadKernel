# native-variable-section-sweep

## ADDED Requirements

### Requirement: Coupled morph×scale is native on a straight/smooth spine (≥3 stations)

The native variable-section sweep `cc_variable_sweep` SHALL build a correct solid NATIVELY
when the section BOTH morphs (profile A → profile B, `A ≠ B`) AND is guide-scaled by a
varying splay `scale(f) = dist(spine(f), guide(f)) / dist(spine(0), guide(0))` at the same
time — the COUPLED morph×scale regime — for a straight or smooth-planar spine, not merely
in the two decoupled sub-regimes.

The section radius at spine fraction `f` is the PRODUCT `g(f) = morph(f) · scale(f)` and the
swept section area varies as `g(f)²` (degree ≤ 4 in `f`). Because a two-station LINEAR
ruling chords `g` and its area then varies as `(chord g)²`, a two-station tube DROPS the
morph×scale cross-term whenever `g` is non-linear, diverging from the exact `∫ g² ` volume
(measured 1–20% versus OCCT `BRepOffsetAPI_MakePipeShell` multi-section and an exact
polygon-clip integral, the two agreeing to ~1e-4). To track the cross-term the native
STRAIGHT-spine guided builder SHALL use `≥ 3` stations when `g` is non-linear, DENSIFYING so
the piecewise-linear-`g` swept-area integral `∫(chord g)²` matches the true `∫g²` to within
a FIXED relative-volume bound; the enclosed volume SHALL then converge to the exact
`∫ area df` law as the station count increases. The curved-spine guided builder SHALL raise
its tangent-turn-driven station count to at least this coupled count so a curved coupled
morph tracks the cross-term too.

The densification SHALL NOT change the two exact sub-regimes: when the section is constant
(`A == B`) OR the guide scale is constant (`scale ≡ 1`), `g` is LINEAR, so the builder SHALL
keep exactly TWO stations (the pre-fix path, byte-identical), preserving the exact frustum /
prism results.

The fix SHALL be an INTERNAL change to `build_variable_sweep_tube` only; the `cc_*` ABI and
the `cc_variable_sweep` signature SHALL be UNCHANGED. `src/native/**` SHALL stay OCCT-FREE,
the change confined to `src/native/construct/sweep.h`, reusing the landed perpendicular /
RMF framing, `assembleRingTube`, and `sectionSweepUnsafe` self-fold guard BYTE-IDENTICALLY,
and leaving the tessellator, boolean, blend, analysis, interference, and exchange modules
UNTOUCHED. No tolerance SHALL be widened to accept the coupled case; the fix SHALL be proven
by convergence to the exact closed form and by the differential fuzzer against OCCT.

#### Scenario: Coupled morph×scale on a straight spine matches the polygon-clip closed form

- **WHEN** `cc_variable_sweep` morphs a circle of radius `r0` into a circle of radius `r1`
  (equal vertex count) along a straight spine of length `H` WHILE a guide splays the section
  scale linearly from `1` to `k`, under the native engine
- **THEN** the result is a watertight, consistently-oriented single lump (Euler χ = 2) whose
  enclosed volume equals the exact polygon-clip closed form
  `C·H·∫₀¹ [(r0 + (r1−r0)f)(1 + (k−1)f)]² df` (with `C = ½N·sin(2π/N)` the exact N-gon area
  factor, the degree-4 integrand integrated in closed form) to a tight, deflection-bounded
  tolerance — NOT the 1–20% divergence of the previous two-station ruling

#### Scenario: The coupled case converges as station and polygon counts increase

- **WHEN** the same coupled morph×scale straight sweep is built at increasing profile
  polygon counts (the builder densifying stations to the coupled non-linearity)
- **THEN** the relative error of the native volume versus the closed form stays within the
  fixed band and does NOT grow as the polygon refines (it converges toward the exact value)

#### Scenario: The two decoupled sub-regimes stay exact (no regression)

- **WHEN** either the section is constant (`A == B`) with a splaying guide (pure guide-scale
  → a similar-section frustum) OR the section morphs (`A ≠ B`) with a spine-parallel guide
  (`scale ≡ 1` → the plain morph)
- **THEN** the native builder keeps two stations and the enclosed volume equals the exact
  prismatoid / frustum closed form to machine precision, byte-identical to the pre-fix path

#### Scenario: Coupled morph×scale matches the OCCT MakePipeShell oracle (parity)

- **WHEN** the coupled morph×scale straight sweep is built through the `cc_variable_sweep`
  facade under the native engine and under the OCCT engine
  (`BRepOffsetAPI_MakePipeShell` multi-section + `BRepGProp`)
- **THEN** the native volume, area, bounding box, and centroid agree with the OCCT oracle
  within the deflection band, and the native mesh is watertight — with no existing vsweep
  parity case regressing
