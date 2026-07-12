# Design — nurbs-offset-rational-fold-trim

## Rational offset (homogeneous locus + prescribed-weight fit)

The exact offset of a rational surface is not rational in general (the unit normal `N`
carries a square root). But for the analytically-important conics it IS: a NURBS cylinder
of radius `r` offsets to a coaxial cylinder of radius `r±d` with the SAME weights and knot
structure; a sphere offsets to a concentric sphere; a plane translates. The rational path
recovers these by:

1. **Sample the true offset locus** `O(u,v) = S(u,v) + d·N(u,v)` on a `g×g` parameter grid.
   `S` and `N` are computed rational-aware (`nurbsSurfacePoint` / `surfaceNormal`, which
   already use the homogeneous derivatives internally — Piegl & Tiller §4.4 quotient rule).
   This is exactly the same locus the non-rational path samples; the offset point itself is
   Euclidean.

2. **Assign a weight to each locus sample.** For a rational input we reuse the input
   surface's own weight *pattern*: sample the input surface's rational weight field at each
   grid node (the effective weight `w(u,v) = Σ Nᵢ(u)Nⱼ(v) wᵢⱼ`, evaluated by summing the
   weighted basis — a byproduct of the rational evaluation) and use it as the prescribed
   weight for that offset sample. For the conic case this reproduces the exact arc/circle
   weight profile (the middle-of-arc `cos θ` weight), so the fitted offset is the exact
   offset conic.

3. **Fit rationally with prescribed weights.** Lift each sample `Q(i,j)` to the homogeneous
   point `(w·x, w·y, w·z, w)` and run `interpolateRationalSurface` (Layer-7). The projected
   rational surface passes through every Euclidean `Q(i,j)` exactly; where the offset is an
   exact rational conic the deviation from the true locus is ≤1e-6.

4. **Refine + measure** exactly as the non-rational path: geometric deviation via
   `closest_point_on_surface` (parametrization-independent, `max |dist − |d||`). Keep the
   best fit; report the true achieved error. If the input is non-rational, or the rational
   fit does not beat the non-rational one, fall back to `offsetSurface` (never worse).

**Round-trip.** Offsetting the fitted rational offset by `−d` must recover `S` within
tolerance — verified as an oracle. For an exact conic this holds to fit tolerance.

## Fold trimming (maximal fold-free parameter rectangle)

The offset map `S ↦ S + d·N` has principal Jacobian factors `(1 + d·κᵢ)`. It is REGULAR
(no fold) wherever both factors are `> 0`, and FOLDS where some `(1 + d·κ) ≤ 0`. The
existing guard rejects when the *global* minimum factor `≤ 0`.

For trimming we instead build a **regularity map** on a dense analysis grid: at each node
mark `regular = min over κ of (1 + d·κ) > εfold`. Then find the **maximal axis-aligned
fold-free rectangle** in parameter space:

- Reduce to the largest all-regular sub-block of the boolean grid. We use a robust,
  deterministic largest-all-true-rectangle scan (histogram / stack method over grid rows,
  O(gridcells)) to get the block with the greatest parameter AREA whose every analysis node
  is regular.
- Map that block's node indices back to a parameter rectangle `[uₐ,u_b]×[vₐ,v_b]`, shrunk
  inward by half a cell so the rectangle's interior is provably regular (the marked nodes
  bound it), avoiding a fold exactly on the trimmed edge.
- Re-run the offset core (sample → fit → refine) restricted to that sub-rectangle. Because
  the kept region is fold-free, the fit converges normally and the result is a valid,
  fold-free offset over the trimmed domain.

**Reporting.** `OffsetResult` gains `trimmed` (bool) and `keptU0/keptU1/keptV0/keptV1` (the
kept parameter rectangle, in the INPUT surface's parameter coordinates). The trimmed-away
region is the complement within `[u0,u1]×[v0,v1]`. When the whole domain is fold-free,
`trimmed=false` and the kept rectangle is the full domain.

**Honest decline.** If the largest fold-free rectangle covers less than a small fraction
(`kMinKeptFraction`, e.g. 5%) of the domain area — no meaningful offset remains — return
`SelfIntersection` with no surface, exactly as before. We NEVER emit a self-intersecting
surface as valid.

**Fold-free verification (oracle).** Over the returned kept region, sample the input's
principal curvatures and assert `(1 + d·κ)` keeps CONSTANT SIGN (all positive) — the offset
Jacobian never degenerates — and the offset points match `S + d·N` (distance `|d|` to `S`).

## Complexity / structure

Both new entry points delegate the shared sample→fit→refine loop to a small internal
`offsetCore(S, d, tol, uRange, vRange, rational)` helper so the two public functions stay
thin (backend cognitive-complexity target 15). The largest-rectangle scan is a standalone
helper with a single loop nest.

## Invariants

- Existing `offsetSurface` signature + behaviour byte-unchanged (additive-only).
- `cc_*` ABI untouched (no facade edit).
- `src/native/**` OCCT-free.
- Never widen a tolerance; never return a folded/self-intersecting surface as valid.
