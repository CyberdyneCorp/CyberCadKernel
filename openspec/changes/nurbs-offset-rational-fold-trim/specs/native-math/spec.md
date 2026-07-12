# native-math

## ADDED Requirements

### Requirement: Rational NURBS surface offset via homogeneous-form fitting

The native math library SHALL provide, additively alongside the existing non-rational
`offsetSurface` (which stays byte-unchanged), a routine `offsetSurfaceRational` in the same
OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated module that constructs the offset of a tensor-product
NURBS surface `S(u,v)` at signed distance `d` as a fitted **rational** B-spline surface when
the input is rational. It SHALL sample the true offset locus `O = S + d·N` (with `S`, `N`
evaluated rational-aware in homogeneous form), assign each sample the input surface's
effective rational weight at that node, lift each sample to its homogeneous point
`(w·x, w·y, w·z, w)`, and fit a rational approximant through the samples with the Layer-7
rational interpolation (`interpolateRationalSurface`), refining the grid until the fitted
surface's maximum deviation from the true offset locus is within a caller tolerance or the
refinement budget is spent. It SHALL report the ACHIEVED maximum offset error (never
widened). For a non-rational input, or when the rational fit does not improve on the
non-rational fit, it SHALL fall back to the non-rational offset result (never worse). It
SHALL NOT fabricate weights — the rational fit uses only the input's prescribed weight
pattern.

#### Scenario: A NURBS sphere offsets to a concentric sphere of radius R±d to a tight bound

- GIVEN a rational NURBS surface representing a sphere of radius `R` and a signed distance `d` with `R + d > 0`
- WHEN the rational offset surface is constructed
- THEN every point of the fitted offset SHALL lie at distance `R + d` from the sphere centre to within `1e-6`, and the fitted offset surface SHALL be rational (non-empty weights)

#### Scenario: A NURBS cylinder offsets to a coaxial cylinder of radius r±d to a tight bound

- GIVEN a rational NURBS quarter-cylinder of radius `r` about an axis and a signed distance `d` with `r + d > 0`
- WHEN the rational offset surface is constructed
- THEN every point of the fitted offset SHALL lie at distance `r + d` from the cylinder axis to within `1e-6`, and the fitted offset SHALL be rational (non-empty weights)

#### Scenario: Rational offset round-trips under negation

- GIVEN a rational NURBS surface `S` and a signed distance `d` whose offset is fold-free
- WHEN `S` is offset by `d` and the result is offset again by `−d`
- THEN the twice-offset surface SHALL recover `S` to within the fitting tolerance at a dense grid of points

### Requirement: Self-intersecting offset is trimmed to the maximal fold-free region

The native math library SHALL provide, additively, a routine `offsetSurfaceTrimmed` that,
when the offset of `S` at distance `d` would self-intersect (fold) over PART of the
parameter domain, TRIMS the domain to the maximal fold-free axis-aligned parameter
rectangle and returns a VALID offset over that trimmed region rather than declining
outright. The fold-free region SHALL be determined from the offset map's principal Jacobian
factors `(1 + d·κ)` computed from the surface's first and second fundamental forms on a
dense analysis grid: a node is fold-free iff every `(1 + d·κ) > 0`. The routine SHALL report
whether the result was trimmed and the kept parameter rectangle (in the input surface's
parameter coordinates). When the entire domain is fold-free the routine SHALL return the
full offset untrimmed. When no fold-free region of meaningful area remains, the routine SHALL
DECLINE with a self-intersection status and no surface. In no case SHALL the routine return a
self-intersecting (folded) surface as valid, and in no case SHALL it crash.

#### Scenario: An offset that folds over part of the domain is trimmed, not declined

- GIVEN a surface with a high-curvature region and a signed distance `d` whose magnitude exceeds the minimum principal radius of curvature over PART of the domain but not all of it
- WHEN the trimmed offset is requested
- THEN the routine SHALL return `ok = true` with a valid offset surface over the maximal fold-free parameter rectangle, SHALL report that the result was trimmed, and SHALL report the kept rectangle as a strict sub-region of the full parameter domain

#### Scenario: The kept region is provably fold-free and matches the true offset locus

- GIVEN a trimmed offset returned over a kept parameter rectangle
- WHEN the input surface's principal curvatures are sampled across the kept rectangle
- THEN every `(1 + d·κ)` over the kept region SHALL keep CONSTANT positive sign (the offset Jacobian never degenerates), and every point of the kept offset SHALL lie at distance `|d|` from `S` to within the reported error

#### Scenario: A fully-folding offset still declines honestly

- GIVEN a surface and a signed distance `d` whose offset folds over essentially the entire domain (no fold-free region of meaningful area)
- WHEN the trimmed offset is requested
- THEN the routine SHALL return `ok = false` with a self-intersection status and no surface — it SHALL NOT return a folded surface

#### Scenario: A fold-free offset is returned untrimmed

- GIVEN a surface and a signed distance `d` whose offset is fold-free over the whole domain
- WHEN the trimmed offset is requested
- THEN the routine SHALL return the full offset with `trimmed = false` and the kept rectangle equal to the full parameter domain
