# Design — nurbs-skinning-loft

## Placement & conventions

New module `src/native/math/bspline_skin.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_ops.{h,cpp}` and `bspline_fit.{h,cpp}`. Reuses `math::Point3` (`native/math/vec.h`), the
evaluators (`findSpan`, `basisFuns` from `bspline.h`), the **Layer-1 exact ops**
(`elevateDegreeCurve`, `refineKnotCurve` from `bspline_ops.h`) for compatibility, and the
**Layer-1 data types** `BsplineCurveData` (sections in) / `BsplineSurfaceData` (surface out).
OCCT-free, fp64, deterministic. Added to the `native_math.h` aggregator.

**numsci gate.** The V-interpolation solves the collocation system through the numsci facade
(`numerics::lin_solve`), so the whole `.cpp` is under `CYBERCAD_HAS_NUMSCI`, exactly like
`bspline_fit.cpp`: the header declares everything; with the guard OFF the implementation TU is
inert and the functions are absent. `CYBERCAD_HAS_NUMSCI` is defined library-wide
(`target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_NUMSCI=1)`), so
`bspline_skin.cpp` — though in the default `src/native` glob — sees it when the option is ON.

Conventions match the rest of the kernel: **flat clamped knot vectors** (degree+1 end
multiplicity, length `nPoles + degree + 1`); **row-major, U-outer** surface poles
`pole(i,j) = poles[i*nPolesV + j]`; **non-rational** (weights empty).

## Section compatibility (§10.3)

`makeSectionsCompatible(sections)` returns the sections raised to a common representation:

1. **Common degree** — `maxDeg = max_k section_k.degree`. Raise every section by
   `elevateDegreeCurve(c, maxDeg − c.degree)` (a no-op when already at `maxDeg`). Exact: the curve
   is geometrically unchanged; only its degree + interior-knot multiplicities rise.
2. **Union knot vector** — collect every distinct interior/end knot value that appears in ANY
   section, tagged with the MAXIMUM multiplicity it reaches across the sections (`knotUnion`,
   tolerance `1e-9`). For each section, the multiset of knots to INSERT is `unionMult − existingMult`
   copies of each value (`knotsToInsert`); apply once with `refineKnotCurve`. Exact: refinement is a
   sequence of Boehm insertions, each geometry-preserving.

Afterwards every section shares degree `p`, the union knot vector, and control-point count
`N = knots.size() − p − 1` — verified as a post-condition (identical degree/N/knots to `1e-7`,
else `ok=false`). Pre-condition: the sections share a common parameter domain `[a,b]`
(reparametrize first if not). **Guards:** a rational section (non-empty weights) makes the whole
call decline (`ok=false`) — non-rational scope, never a silently-wrong result; an empty section,
degree < 1, or a malformed knot vector also declines.

## Skinning (Algorithm A10.3)

`skinSurface(sections, degreeV)`:

1. `makeSectionsCompatible` → `p`, common U-knots, `N` control points per section, `K` sections.
2. **Section parameters `v_k`** (`sectionParams`): for each control-point index `i`, the chord-length
   parameters of the polyline `{P_i^0, …, P_i^{K−1}}` in V; AVERAGE these over the `N` indices
   (§10.3). Coincident control indices contribute nothing; if every section is coincident there is
   no V length to normalize and the function returns empty → skin declines (`ok=false`). Ends pinned
   `v_0=0`, `v_{K−1}=1`.
3. **V degree** `q = clamp(degreeV, 1, K−1)` — few sections ⇒ a lower-degree loft. **V-knots** =
   averaging knots (Eq 9.8) of `v_k` at degree `q`.
4. **V-interpolation** (`interpolateAcrossV`): assemble the `K × K` collocation matrix
   `A(k,j) = N_{j,q}(v_k)` ONCE (it depends only on `v_k` and the V-knots, shared by every control
   index and coordinate). For each control-point index `i`, the three RHS are the `K` section poles
   at index `i` (x/y/z); `numerics::lin_solve` yields the `K` V-control points for column `i`.
   A singular solve returns `ok=false`.
5. **Assemble** the `N × K` surface: U = section direction (degree `p`, common U-knots, `N` poles);
   V = across-sections (degree `q`, averaging V-knots, `K` poles). The net from step 4 is already
   row-major U-outer (`pole(i,j) = net[i*K + j]`). Weights empty ⇒ non-rational.

## Oracle strategy (why this layer is airtight)

| Property | Exact invariant (HOST, no OCCT) | Achieved |
|---|---|---|
| Section containment | surface iso-curve `S(·,v_k) == section_k`, dense u-sample | ~1e-15 |
| Compatibility exactness | each compatible section `== original`, dense u-sample; shared degree/knots/N | ~1e-15 |
| Known-surface containment | iso-curves of a KNOWN surface, re-skinned, contained | ~1e-15 |
| Idempotence (full surface) | skin → re-extract iso-curves at same `v_k` → re-skin ≡ original, POINTWISE | ~1e-15 |
| Degenerate guards | <2 / coincident / rational / recoverable handled honestly | — |

**Why containment is exact.** After compatibility every section has the SAME U-basis (degree `p`,
common U-knots). The surface control net's column `i` interpolates the section poles `{P_i^k}` in V.
At `v = v_k`, the V-basis reproduces exactly `P_i^k` for every `i` (interpolation), so the iso-curve
`S(·,v_k) = Σ_i N_{i,p}(u) P_i^k` is precisely the compatible section `k`. This is a closed-form
identity, not a fit — hence machine precision.

**Why the full-surface round-trip needs idempotence.** Extracting iso-curves from a KNOWN surface at
arbitrary stations and re-skinning does NOT reproduce the surface *between* stations in general: the
skin re-parametrizes V by chord length across the control polygons and builds its OWN averaging
knots, which need not equal the source's V-knots (the same parametrization confound the Layer-7
round-trip documents). So the KNOWN-surface test asserts exact iso-curve *containment*, and the
full-surface *identity* is proven by IDEMPOTENCE — re-skinning a skin's own iso-curves at its own
`v_k`, where the V-parametrization is a fixed point, reconstructs the whole surface exactly.

## Complexity & structure

Chapter 10's algorithm is index-dense; per the cognitive-complexity policy the compilers/parsers
band (25–35) applies. Each routine is one focused function with the book's Eq/algorithm numbers in
comments; the knot-union / knots-to-insert / section-params / V-interpolation helpers are single
small functions, not copy-paste. The compatibility and skinning paths reuse the Layer-1 ops and the
Layer-7 collocation idiom rather than re-implementing them.

## Risks & honest residuals

- **Non-rational only.** Rational/weighted skinning (interpolating the section weights) is
  materially harder and deliberately out of scope; the module returns non-rational surfaces (empty
  weights) and declines on rational sections. Documented in `docs/NURBS-SCOPE.md` Layer-6 row.
- **Single-family skinning, not general surfacing.** Skinning interpolates one family of parallel
  sections. Gordon surfaces (two transversal curve families), N-sided/boundary filling, plate/energy
  surfaces, and exact swept surfaces (spine + orientation frames) are distinct constructions and
  remain demand-gated residuals — this module never fakes them.
- **Parametrization/knot heuristics are the standard book choices** (chord-length across control
  polygons + averaging knots). Pathological section families (extreme clustering) can ill-condition
  the V-collocation matrix; the dense solve returns empty on a singular system and the skin declines
  with `ok=false` (honest), never a silently-wrong surface.
- **Common domain pre-condition.** Sections must share the parameter domain `[a,b]`; the module does
  not auto-reparametrize (Layer-1 `reparamCurve` is available to the caller). A domain mismatch shows
  up as a failed compatibility post-condition (`ok=false`), not a wrong surface.
