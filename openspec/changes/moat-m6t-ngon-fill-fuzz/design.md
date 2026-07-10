# Design — moat-m6t-ngon-fill-fuzz

## Context

The M6 completeness bar drives random valid inputs through BOTH the native path and the
OCCT oracle and classifies every trial, requiring `DISAGREED == 0`. A NEW native surface
landed: the bounded N-SIDED FILL (`src/native/surface/{ngon_fill,fill_solid}.h`) reached
through the shipping `cc_fill_ngon` facade. The curated `native_surfacing_parity.mm`
harness proves both engines on FOUR hand-picked fixtures + one decline; this change turns
it into a *seeded, randomized batch* with a per-trial classifier over random 3–6-sided
analytic boundary loops.

## The two oracle regimes (why native-vs-OCCT alone is insufficient)

`BRepFill_Filling` is an ENERGY-(area-)MINIMIZING surface; the native patch is a
TRANSFINITE Coons/Gregory interpolant. The two INTERPOLATE the SAME analytic boundary but
their INTERIORS differ — OCCT can bulge past the loop hull, and for a warped loop the
transfinite surface is more crumpled (larger area). So a native-vs-OCCT-only test cannot
distinguish a native miss from a legitimate surface difference, and interior-vertex
identity is meaningless. The design splits the arbiter by family:

### Planar families — the EXACT closed form is PRIMARY

For a coplanar straight-edged loop the native planar fast-path (`ngon_fill.h`
`tryPlanarFill`) ear-clips the EXACT polygon, so the patch area equals the exact 3-D
polygon area:

> **`A = ½ |Σ Pᵢ × Pᵢ₊₁|`** (Newell) — exact for any planar polygon in any plane.

This is engine-independent ground truth. `planar-Ngon` uses a general random plane;
`planar-hole-completion` uses a COORDINATE plane (the box-face-restore pose), where the
same closed form is ALSO exactly the missing planar face area a `fillHoleSolid` weld would
restore (exact volume restore). Because it is exact, a native area matching it while OCCT
is the outlier is ORACLE-INACCURATE (native vindicated), never a bar failure. OCCT area is
cross-checked within the same tight band as an oracle-trust gate.

### Non-planar families — OCCT area band + bbox-containment + boundary residual

`saddle-4sided` (corners alternating ±h out of a base plane) and `arc-boundary` (≥1
circular-arc side) have no closed-form area. The arbiter is:

1. **OCCT area** within a FIXED band (`kAreaTolNP = 1.2e-1`) — the only area oracle here.
2. **BBOX-containment** — the native Coons patch stays inside the loop hull, so its bbox
   must be CONTAINED in OCCT's fill bbox grown by `kBoxTolNP = 8e-2` (OCCT may bulge).
3. **Analytic-boundary residual** — an OCCT-INDEPENDENT self-check that every native
   boundary sample lies on its straight/circular-arc curve (`kResidTol = 1e-6`). The
   native sampler places boundary rows AS the analytic samples (bit-exact), so this is 0
   by construction for a valid patch; the harness recomputes the truth to guard it.

The saddle out-of-plane amplitude is bounded to `h ≤ 0.28·edge` (edge ≈ `2R·sin(π/N)`) —
the SMALL-WARP regime the parity harness proved co-areas within the band. This is a
GENERATOR SCOPE BOUND (staying where OCCT is a valid area oracle for the transfinite
patch), NOT a tolerance widening: a strongly-warped loop is a different valid surface where
area no longer discriminates, so it is out of this domain's scope. (During bring-up, an
unbounded saddle at `h ≈ 0.6·R` on N=6 produced native area ~18% over OCCT — two different
valid surfaces, not a native fault — which the bound removes.)

## Facade drive + the open-patch area read

Each trial builds the SAME loop's POD arrays (`boundaryXYZ`, per-side `edgeKinds`,
per-arc `arcMids`) and fills through the SAME `cc_fill_ngon` facade twice:
`cc_set_engine(1)` → NativeEngine `surface::fillNGon` (Coons for N∈{3,4}, Gregory-style
trailing-side merge for N∈{5,6}, planar fast-path for coplanar straight loops, NULL → OCCT
on any out-of-bound pose); `cc_set_engine(0)` → OCCT `BRepFill_Filling`.

The fill patch is an OPEN surface (a face / mesh soup, not a closed solid). The native
`mass_properties` sets `valid = isWatertight(mesh) && volume > 0`, which is FALSE for an
open patch — but it still computes `area = surfaceArea(mesh)`. The facade documents the
patch is "served by `cc_mass_properties` (area) … like an imported STL soup", so the
harness reads `mp.area` DIRECTLY (not gated on `mp.valid`) — the meaningful metric for a
surface patch. OCCT's `mass_properties` on the fill FACE likewise returns the face area.

## The honest-decline exerciser

A sparse (every-11th, N≥4) trial forces a SELF-INTERSECTING bowtie (two non-adjacent
corners swapped) the native planar ear-clip cannot triangulate → `NGonDecline::
SelfIntersecting` → `cc_fill_ngon` returns 0 → the facade falls to OCCT. This proves the
native scope bound: for a bad loop the native arm emits a clean NULL, never a wrong patch.
In practice OCCT also refuses the bowtie, so these land BOTH-DECLINED (both engines
correctly refuse) — a first-class, logged outcome, never a bar failure. An N=3 degenerate
(collinear triangle) is NOT probed because it CRASHES OCCT's `BRepFill_Filling` (an
oracle-robustness fragility outside the native scope); the N≥4 bowtie exercises the same
native decline path with an oracle that survives.

## The bar

`std::_Exit(0)` IFF `DISAGREED == 0 && ORACLE_UNRELIABLE == 0` with each of the four
families ≥1 AGREED, proven across ≥2 seeds (N ≥ 60/seed; the runner fails if ANY seed
fails). Same seed → byte-identical batch (splitmix64 → xoshiro256**, no clock / `rand()`).
Measured: seed 0xF117A11FEE → 68/0/0/4, seed 0x5EEDF111A6 → 68/0/0/4, every planar AGREE
native==OCCT==closed-form to ~6 significant digits; determinism re-verified.
