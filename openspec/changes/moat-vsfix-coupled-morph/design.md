# Design — moat-vsfix-coupled-morph

## The bug, precisely

`build_variable_sweep_tube` places, at each spine station `f ∈ [0,1]`, the ring

```
ring_i(f) = origin(f) + x·[ (A_i.x + (B_i.x − A_i.x)·f) · sc(f) ]
                      + y·[ (A_i.y + (B_i.y − A_i.y)·f) · sc(f) ]
```

where `sc(f) = guideScaleAt(f)` is the guide splay (≡ 1 with no guide). The per-vertex
coordinate is `morph_i(f) · sc(f)` — a PRODUCT of a linear morph term and the scale term.

On a straight spine the frame is constant, so the swept solid is a stack of planar
cross-sections whose area at fraction `f` is `Area(f) = C · g(f)²`, with
`g(f) = r(f)·sc(f)` the section-radius envelope (`r(f)` the linear morph radius, `C` a
shape constant). The exact volume on a straight spine of length `H` is
`V = H · ∫₀¹ Area(f) df = C·H·∫₀¹ g(f)² df`.

The old code used `nStations = 2`: the ruled tube chords `g` LINEARLY between `f=0` and
`f=1`, so its area varies as `(chord g)²` and its volume is `C·H·∫₀¹ (chord g)² df`. When
`g` is linear (either the morph is absent — `A==B` — or the scale is constant — `sc≡const`)
the chord equals `g` and the volume is exact. But when BOTH vary, `g` is a genuine
non-linear product (its `g²` area is degree-4 in `f`), the chord drops the curvature, and
`∫(chord g)² < ∫g²` by up to ~19% for a strong splay — the measured M6-breadth-19
divergence, confirmed against OCCT's 24-section `MakePipeShell` and an exact polygon-clip
integral (both agree to ~1e-4).

## The fix: densify the straight guided path to bound the coupled volume error

`straightCoupledStations(morphR0, morphR1, guideScaleAt)` returns the station count. It
models the section-radius envelope `g(f) = (morphR0 + (morphR1−morphR0)·f) · sc(f)` and
finds the fewest UNIFORM bands `nb` for which the piecewise-linear-`g` swept area integral
matches the true one:

```
trueInt  = ∫₀¹ g(f)² df                          (fine midpoint reference)
chordInt = ∫₀¹ (piecewise-linear-g over nb bands)(f)² df
grow nb until  |chordInt − trueInt| / trueInt ≤ kMaxCoupledVolErr   (0.2%)
return nb + 1                                     (stations = bands + 1)
```

Why bound the AREA integral (`∫g²`) rather than the pointwise `g` deviation: the volume
error is driven by `g²`, not `g`. A pointwise-`g` chord bound over-densifies where `g` is
steep-but-straight and under-densifies where the product curves — bounding `∫g²` directly
targets the quantity the gate measures. (Measured: a pointwise-`g` bound left the volume at
1.24% while the area-integral bound reaches 0.2% with the same station budget.)

Key properties:

- **Two exact sub-regimes untouched.** If `morphR0 == morphR1` (no morph) OR `sc` is
  constant (no scale variation), `g` is linear, the piecewise-linear-`g` chord equals `g`
  for any `nb`, the integral error is 0 at `nb = 1`, and the function returns `2` — the
  pre-fix station count, so those paths are byte-identical.
- **Coupled case tracks the curvature.** For the reference coupled case (circle r0=5→r1=2,
  scale 1→2) it returns 11 stations; a steeper splay (scale 1→5) returns 20. The volume then
  converges: `∫(chord g)²` → `∫g²` as `nb` grows.
- **Bounded.** Capped at `kMaxDensifyStations` (the shared 512 station cap), so a
  pathological guide cannot blow up the tiling.

The CURVED-spine guided path keeps its turn-driven `kMaxBandTurn` densification and simply
raises the count to `max(turnCount, coupledCount)`, so a curved coupled morph tracks the
cross-term with the same guarantee. The perpendicular / RMF framing, `assembleRingTube`,
and `sectionSweepUnsafe` self-fold guard are reused unchanged.

## The HOST analytic oracle (Gate a)

For a circle(r0)→circle(r1) morph the lerp of two concentric regular N-gons is a regular
N-gon of the lerped radius `r(f) = r0 + (r1−r0)f`, area `C·r(f)²` with
`C = ½N·sin(2π/N)`. With a linear splay `sc(f) = 1 + (k−1)f` the exact volume is

```
V = C·H·∫₀¹ [(r0 + Δr·f)(1 + Δs·f)]² df ,   Δr = r1−r0, Δs = k−1
```

The integrand is a degree-4 polynomial, integrated in closed form (`∫₀¹ fⁿ = 1/(n+1)`).
This is the polygon-clip closed form the regression test compares against; the native
volume converges to it as station and polygon counts grow, with the two sub-regimes exact.

## The fuzzer broadening (Gate b, differential)

The `GUIDED_STRAIGHT` family previously drew only the two exact sub-regimes because the
coupled regime was a reported limitation. Now it draws a THIRD regime (morphing section +
splaying guide) certified against the SAME exact closed-form volume as the other straight
families. The straight-spine section area is degree ≤ 4 in `f` (blend ≤ 2, guide-scale² ≤
2), so the closed-form quadrature is upgraded from composite Simpson over 4 intervals
(exact only to degree 3 per interval — a ~5e-4 residual on the coupled f⁴ term) to **Boole's
rule** (5 samples, weights `2h/45·[7,32,12,32,7]`, exact ≤ degree 5), making the arbiter
exact for the coupled area including the cross-term — no tolerance is widened.

## Discipline

- OCCT-free; only `src/native/construct/sweep.h` changes in `src/native/**`.
- `cc_*` ABI unchanged (internal builder fix).
- No tolerance widened; the fix is proven by convergence to an EXACT closed form and by the
  differential fuzzer against OCCT.
