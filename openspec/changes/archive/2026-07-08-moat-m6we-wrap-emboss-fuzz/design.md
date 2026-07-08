# Design — moat-m6we-wrap-emboss-fuzz

## Context

`src/native/feature/wrap_emboss.h` is the OCCT-free native wrap-emboss builder behind the
`cc_wrap_emboss` ABI. It wraps a footprint drawn in a profile `(px, py)` space onto a
cylinder lateral face by the map `u = px/R` (arc-length → angle about the axis) and
`v = py + vMid` (axial), then rebuilds the WHOLE embossed/debossed cylinder as one
deflection-bounded planar-facet soup welded watertight through the boolean `assembleSolid`.
It claims three native tracks: a raised **rectangular pad**, a recessed **rectangular
deboss** pocket, and a **non-rectangular polygon** emboss/deboss. Anything else (a
non-cylindrical base, a self-intersecting / >2π footprint, a deboss depth ≥ R, non-positive
height) returns a NULL Shape → the shipping engine forwards to OCCT.

The curated `native_wrap_emboss_parity.mm` proves a handful of hand-picked cases through
the `cc_*` facade. This change adds the SEEDED-BATCH differential the four landed M6
fuzzers established.

## The differential and why the oracle is closed-form-primary

Unlike the boolean / blend fuzzers, OCCT exposes NO single wrap-emboss API, so — exactly as
in the construction and blend fuzzers — the **PRIMARY** correctness oracle is a CLOSED-FORM
computation, not an OCCT call. The native wrap raises/recesses the material between radius
`R` and the target radius `Rout` over the footprint. Because `u = px/R` linearly rescales
`px`, a footprint of flat (shoelace) area `A` in `(px, py)` covers a `(u, v)` measure `A/R`,
and the changed volume is the radial shell over that measure:

```
ΔV = ∫∫_(u,v footprint) ∫_R^Rout r dr du dv = (A/R) · (Rout² − R²)/2 = A·|Rout² − R²|/(2R)
     emboss:  Rout = R + height   (added)
     deboss:  Rout = R − depth    (removed)
```

This is **curvature-corrected** — NOT the naive flat `A·height` — and it is **universal**:
it depends only on the `(u, v)` measure `A/R`, so the SAME formula arbitrates rectangle AND
polygon footprints. The native densifier keeps every generated (u,v) polygon edge straight
(it interpolates collinearly along the straight edge), so the wrapped footprint's `(u,v)`
area is exactly `A/R` and the closed form is the exact smooth ground truth for the native
solid.

The native builder facets the whole cylinder, so its measured volume/area are an INSCRIBED
polygonal approximation a small, deflection-bounded amount below the smooth closed form. The
harness holds that bias FAR under a FIXED tolerance (`kVolRelTol = 2e-2`, `kAreaRelTol =
3e-2`, never widened) and logs the measured max bias so the margin is auditable.

### The SECONDARY OCCT reconstruction (rectangle families, where clean)

A rectangle footprint wraps to an EXACT angular sector, so the embossed/debossed rectangle
solid is reconstructable by an OCCT boolean:

- **Emboss** — `Fuse(base cylinder, outer pie wedge of radius R+height over the sector)`.
  The union adds exactly the outer shell over the sector (two OVERLAPPING solids → a robust
  single boolean, no coincident-face fragility). Its volume equals the closed form exactly.
- **Deboss** — `Cut(base, shell wedge [R−depth, R+margin] over the sector)`, where the inner
  core wedge is grown slightly in angle/axial extent so its faces do NOT coincide with the
  outer wedge's (keeps both booleans transversal / robust). Only the in-base part
  (radius `[R−depth, R]`) is removed → the pocket; its removed volume equals the closed form
  exactly.

The reconstruction is **best-effort**: it is guarded by `IsDone()` / `BRepCheck` and, on any
boolean failure, is simply marked unavailable and the trial falls back to the authoritative
closed form (never a bar failure — the closed form is primary). Where it IS available it
cross-checks the closed form and supplies the only independent AREA oracle. Because it
validates the SAME `A·|Rout²−R²|/(2R)` formula on the rectangle families, it transitively
supports the identical closed form used for the polygon families — for which an OCCT
reconstruction is NOT clean (a wrapped polygon pad would re-implement the feature, arcs and
all) and is therefore honestly DECLINED.

## Angular invariance

Volume and area are invariant to WHERE the pad sits angularly, so the OCCT reconstruction
places its wedge at an arbitrary angle (span only) — it does NOT need to match the native
frame's angular origin. This is what makes the rectangle reconstruction clean and robust.

## Classifier (five-way, fixed tolerance)

Per core-family trial (`nativeUsable` = native present, watertight, positive volume):

- `analyticMatch` = `relDiff(natVol, expVol) < kVolRelTol`, `expVol = πR²H ± ΔV`.
- For rectangles with a valid OCCT reconstruction: `occtVolMatch`, `occtAreaMatch`.

```
!nativeUsable                          → HONESTLY-DECLINED   (in-scope self-verify discard → OCCT ships)
 nativeUsable & !analyticMatch         → DISAGREED           (watertight but WRONG vs exact math — the bug)
 nativeUsable &  analyticMatch:
    occtOk & !occtVolMatch             → ORACLE-INACCURATE   (native right by math, OCCT reconstruction outlier)
    occtOk & !occtAreaMatch            → DISAGREED           (volume right but AREA wrong — a real defect)
    else                               → AGREED
```

Correctness is gated on the exact-math VOLUME, so an area-only excursion can never produce a
false DISAGREED on a polygon family (which has no area oracle); on a rectangle family the OCCT
area oracle is available and an area defect with matching volume IS caught. A native result is
exonerated ONLY when it POSITIVELY matches the closed form; the tolerance is never widened to
reclassify a disagreement.

For the out-of-scope DECLINE-exercisers the native builder MUST return NULL/non-watertight →
**BOTH-DECLINED** (no oracle, no wrong result). If the native builder instead returns a
watertight solid for an out-of-scope input, that is a **guard leak** — flagged as a SURPRISE
and FAILS the bar (it is a real native-guard concern, not laundered).

Note on the self-intersecting exerciser: the pentagram is a count-5 star. A crossed *quad*
would be recovered by the native `rectFootprint` as its axis-aligned bounding rectangle
(whose four corners sit at the bbox extremes, order-independent) — correct native behaviour,
but it would NOT exercise the polygon self-intersection guard. A count≠4 star routes through
`polyFootprint`, whose proper-crossing test rejects it.

## Deflection-bounded sensitivity (honest limitation)

Because both `natVol` and `expVol` are dominated by the base cylinder `πR²H`, the total-volume
relative comparison inherits the base's faceting bias; the differential's sensitivity to the
PAD is bounded by that bias relative to the whole solid. So the harness catches gross
wrong-volume / broken-topology / wrong-area pads (and every non-watertight failure) — the
defects this bar exists to catch — while a sub-tolerance faceting gap is honestly logged, not
laundered. The polygon pad cap's few-triangle inscribing floor is deflection-INDEPENDENT
(n=3 is a single triangle), so the generator BOUNDS the polygon circumradius (`≤ 0.6R`) and
amount (`≤ 0.35–0.4R`) to keep that floor comfortably under the fixed tolerance — the same
bounded-feature-size discipline the blend fuzz uses for rim/edge size. Rectangle pads
(sagitta-tiled, exact OCCT sector oracle) stay generous. Measured across 6 seeds (~1500
trials) the max bias is ≤ 9.5e-3 vs the fixed 2e-2 bar.

## Alternatives considered

- **OCCT reconstruction as the primary oracle.** Rejected: OCCT has no wrap-emboss API and a
  wrapped-polygon reconstruction is not clean; the closed form is exact, universal, and
  cheaper. OCCT is the secondary cross-check where clean.
- **Isolating the pad delta** (native embossed minus a native base cylinder). Rejected: the
  base-only mesh uses a different angular sampling than the embossed body's window sampling,
  so the subtraction introduces its own bias comparable to ΔV. Total-volume comparison at a
  fixed tol (the sibling discipline) is cleaner and honestly bounded.
- **Widening the tolerance for wide polygon pads.** Forbidden by the M6 discipline. Instead
  the generator bounds the footprint so the inscribed-cap floor stays under the FIXED tol.
