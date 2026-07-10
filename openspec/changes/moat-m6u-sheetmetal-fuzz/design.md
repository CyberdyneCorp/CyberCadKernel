# Design — moat-m6u-sheetmetal-fuzz

## Context

The M6 completeness bar drives random valid inputs through the native path and classifies
every trial, requiring `DISAGREED == 0`. A NEW native surface landed: the constant-thickness
SHEET-METAL first slice (`src/native/sheetmetal/{base_flange,edge_flange,unfold,common}.h`)
reached through the shipping `cc_sheet_base_flange` / `cc_sheet_edge_flange` /
`cc_sheet_unfold` facade under the NativeEngine. The curated `native_sheetmetal_selftest.mm`
proves the ops on FIVE hand-picked fixtures; this change turns it into a *seeded, randomized
batch* with a per-trial closed-form + invariant classifier over random profiles, thicknesses,
edge selections, bend radii, angles, and k-factors.

## Why there is NO OCCT oracle (stated explicitly)

Every other native fuzzer in this campaign compares the NativeEngine against the OCCT
adapter through the same facade. **That is impossible here: OCCT core has NO sheet-metal
module.** The sheet-metal ops are NATIVE-ONLY — a case the native builder cannot robustly
build HONEST-DECLINES (`cc_sheet_*` returns 0, `cc_last_error` set) and is NEVER forwarded to
OCCT nor faked. So the arbiter cannot be a second engine; it must be CLOSED FORM +
INVARIANTS, which for these three ops are exact ground truth by construction.

## The three arbiters

### BASE flange — exact planar-prism volume

A base flange is the 2D profile extruded by the sheet gauge; its volume is
`|profileArea|·thickness` with `profileArea` the shoelace area of the loop. The native body
is a planar prism, so its MESH volume is EXACT (no curved face), and the arbiter is a HARD
equality at the fp floor (band `kExactAbs = 1e-6`). Three profile families — rectangle,
regular n-gon, convex-jittered n-gon (3..8 corners) — exercise a range of simple polygons.

### EDGE flange (fold) — closed-form volume converging from below

The fold is base + a cylindrical BEND (a partial annulus, inner radius `r`, outer `r+t`,
swept through `θ`) + a planar FLANGE WALL of length `height`:

> **`V = L·W·t + ½·θ·((r+t)²−r²)·W + height·t·W`**

The bend is a TRUE cylinder tessellated to a deflection (a fan of inscribed strips), so the
MESHED volume converges FROM BELOW to the closed form as the fan resolves. The AGREE band is
the SAME 1.5% the product's own `common.h::verifySolid` gates the fold at (`kBendBand =
0.015`) — a fixed deflection bound, NEVER widened — and the harness additionally requires the
native volume NOT to EXCEED the closed form (an inscribed convergent mesh only under-fills).
Measured: `dRel` ranges ~1e-5..3e-4, comfortably inside the band.

### fold→unfold AREA INVARIANT — the load-bearing round-trip check

Unfolding unrolls the bend about its neutral fibre. The bend allowance is
`BA = θ·(r + k·t)` (the k-factor formula), the developed run is `L + BA + height`, and the
flat blank is `devLength × W × t`. The developed FOOTPRINT AREA

> **`A_dev = devLength·W = baseArea + BA·W + flangeArea`**

is INVARIANT under fold→unfold. The unfold blank is a planar prism, so its volume ==
`A_dev·t` EXACTLY. The harness asserts the invariant two ways: (1) the native unfold volume
matches the closed-form `A_dev·t` (band `kExactAbs = 1e-6`); (2) the additive decomposition
`baseArea + bendDevelopedLen·W + flangeArea` equals the direct `devLength·W` (residual
`kInvarAbs = 1e-6`). The unfold is paired with a REAL folded body (it develops the fold built
in the same trial), so the round trip is genuine, not a re-derivation. Measured: invariant
residual ≤ ~5e-13, blank volume vs closed form to ~1e-16.

## The validity gate + the GS6 localized finding

Every built part must be a valid CLOSED 2-MANIFOLD: watertight, consistently oriented,
non-degenerate, finite, Euler χ=2, positive volume — the SAME contract the product's own
`verifySolid` and the host Gate (a) enforce. The harness reads this through the facade from
`cc_check_solid`'s per-check breakdown (`closed_manifold ∧ consistent_orientation ∧
no_degenerate ∧ finite`) AND `cc_mass_properties` (`watertight ∧ vol>0`).

`cc_check_solid` also runs a SEPARATE GS6 `no_self_intersection` sub-check that is NOT part
of the sheet-metal builder's contract. On the edge-flange FOLD it FALSE-POSITIVES
(`first_failure` code 5 = `CC_VALID_SELF_INTERSECT`): the bend is a fan of near-coplanar
planar strips approximating a true cylinder, and adjacent tight-bend facets read as
self-intersecting to the M0-mesh detector even though the body is genuinely
watertight/oriented/χ=2/volume-exact. This reproduces on the BASE commit with the LANDED
code — the landed `native_sheetmetal_selftest.mm`'s `edge_flange cc_check_solid valid` line
FAILs there too (10 passed / 1 failed). It is therefore a PRE-EXISTING GS6-vs-tessellated-
cylinder product interaction, NOT a fold-geometry fault and NOT introduced by this test-infra
change. Per the track discipline it is REPORTED (a future product GS6 fix), NOT fixed here,
and classified **NATIVE-CHECK-INACCURATE** — the no-OCCT-oracle analogue of the sibling
fuzzers' ORACLE-INACCURATE: the geometry is correct by every arbiter this fuzzer owns, only a
separate native CHECK misreports it. It is logged + measured, never a bar DISAGREE, and it
still counts toward per-op coverage (it is geometrically correct).

## The honest-decline exerciser

Out-of-slice poses the native arm MUST refuse (return 0), exercised deterministically:

- a WRONG / non-straight edge id (a base side/bottom edge, or out of range) →
  `EdgeNotStraight` / `NotSingleBendPart` / `EdgeNotFound` → NULL;
- a DEGENERATE profile (a collinear zero-area triangle) → `BadProfile` → NULL;
- a bend angle OUTSIDE `(0°,180°)` → `BadParam` → NULL;
- UNFOLD of a body that is NOT a recognised single-bend fold (a plain base flange) →
  `NotSingleBendPart` → NULL.

(A large-θ "self-collision" was tried as a decline probe and DROPPED: the builder legitimately
accepts a large-angle long-wall fold when its mesh self-verifies watertight — it is a valid,
if unusual, part, not a decliner. The four probes above are reliable.)

## The bar

`std::_Exit(0)` IFF `DISAGREED == 0` with each of the three ops (base-flange /
edge-flange-fold / unfold) ≥1 geometrically-correct trial (AGREED or NATIVE-CHECK-INACCURATE),
proven across ≥2 seeds (N ≥ 60/seed; the runner fails if ANY seed fails). Same seed →
byte-identical batch (splitmix64 → xoshiro256**, no clock / `rand()`). Measured: seed
0x5EE7EA1F00 → 40 AGREED / 14 HONESTLY-DECLINED / 0 DISAGREED / 18 NATIVE-CHECK-INACCURATE;
seed 0xB3ADF01DCC → 40 / 14 / 0 / 18; every base + unfold AGREE native==closed-form to
~1e-16, every folded part volume within the 1.5% band, area invariant residual ≤ ~5e-13.
