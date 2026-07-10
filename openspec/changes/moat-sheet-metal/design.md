# Design — moat-sheet-metal (MOAT M-SM first slice)

## What the landed substrate already gives us

- `construct::build_prism(profileXY, count, depth)` — a closed polygon profile → a
  watertight planar-faced prism SOLID with the exact `profileArea·depth` volume. The
  base flange and the unfold blank ARE prisms, so both reuse it directly.
- `topology::ShapeBuilder` + `construct::detail::planarFace` — build vertices / edges
  (with pcurves) / faces / shells / solids; a planar face from a CCW vertex loop welds
  watertight when adjacent faces share vertices.
- `math::Ax3` / elementary `Cylinder` — analytic surfaces; `math::Mat3`/`Transform` for
  placement.
- The mesh audit — `tess::isWatertight` / `isConsistentlyOriented` / `enclosedVolume`
  and `dm::rfdetail::eulerChar` — the SAME self-verify vocabulary `draft_faces` and
  `split_plane` use. Nothing new; nothing widened.
- The engine idiom — `NativeShape` holder, `wrapNative`/`track`, `isNative`,
  `watertightVolume`, and the additive `IEngine` virtual + facade `cyber::guard` wrapper
  (mirrors `cc_draft_faces`).

## 1. The closed-form arbiter (there is NO OCCT sheet-metal oracle)

OCCT core has no sheet-metal module, so M-SM cannot two-gate against OCCT like the other
native ops. The arbiter is CLOSED FORM. For a constant-thickness single-bend part with
base run `L`, width `W`, thickness `t`, inner bend radius `r`, bend angle `θ`, flange
wall height `h`, and k-factor `k`:

- **base flange volume** `V₀ = A·t` (profile area × thickness).
- **folded volume** `V = L·W·t + ½·θ·((r+t)² − r²)·W + h·t·W`
  = base sheet + partial annulus × width + flange-wall prism.
- **developed (unfold) area** `A_dev = L·W + BA·W + h·W`, with the bend allowance
  `BA = θ·(r + k·t)` (the neutral-fibre developed length). The flat blank is
  `A_dev × t`.
- **round-trip invariant** — `A_dev` from the folded part's recovered `(L,W,t,r,θ,h)`
  equals `A_dev` computed directly from the blank: fold → unfold is AREA-INVARIANT.

`verifySolid` asserts every built solid is a watertight, consistently-oriented, single
genus-0 (χ=2) lump enclosing a positive volume within a **deflection-set** band of the
closed form. A curved (cylindrical) bend meshes to `defl`, so its volume converges from
below; the band is the deflection bound, NOT a widened fudge. A flat part (no curved
face) is exact.

## 2. Edge-flange geometry (one contiguous watertight cross-section)

The first slice flanges a rectangular base `[0,L]×[0,W]` at `z∈[0,t]` off its +X rim,
bending upward. Rather than a boolean fuse of base + bend + wall (which would need the
BSP-CSG substrate and a robust weld), the part is built as ONE solid whose cross-section
in the local XZ plane is swept along +Y over `[0,W]`:

- bend centre `C = (L, t+r)`; radial dir at bend parameter φ is `(sinφ, −cosφ)`, so at
  φ=0 the inner fibre is `(L,t)` (base TOP rim) and the outer fibre is `(L,0)` (base
  BOTTOM rim) — the bend CONTINUES the base cross-section exactly. Tangent `(cosφ,sinφ)`;
  the wall leaves the arc end (φ=θ) along `T(θ)` for `h`.
- faces: base bottom/top/−X-cap + 2 base ±Y rails; the bend's outer and inner walls as a
  fan of N planar strips (N sized from the deflection sagitta on the outer radius) + 2
  annular-wedge end caps (y=0, y=W); the flange wall's outer/inner faces + end cap + 2
  ±Y rails. Every quad is AUTO-oriented (its loop is flipped if its winding normal
  opposes the desired outward normal), so the shell is consistently oriented regardless
  of corner order — this is what makes the mesh weld watertight and the volume sign right.

The fan makes the bend a tessellation-driven approximation, but the self-verify asserts
the CLOSED-FORM volume band, so an under-resolved fan is REJECTED, never accepted wrong.
A larger radius auto-subdivides finer (the sagitta bound), so accuracy scales with the
feature.

## 3. Unfold without mesh reverse-engineering

`cc_sheet_unfold(body, kFactor)` needs the fold parameters. Recovering `(θ,r,h,L)` from
an arbitrary folded mesh is ambiguous and fragile, so the engine RECORDS the closed-form
`FoldRecord` on the `NativeShape` when `sheet_edge_flange` builds it (additive,
process-internal — it never crosses the `cc_*` ABI). `sheet_unfold` reads that record and
develops the blank EXACTLY. A body with no valid record (an arbitrary solid, an OCCT body,
a mesh import) HONEST-DECLINES `NotSingleBendPart`. This keeps the unfold exact and
robust while honouring the `(body, kFactor)` ABI shape.

## 4. Honest decline (native-only, never an OCCT forward)

Because there is no OCCT sheet-metal op, a decline is terminal, not a fall-through: the
facade returns 0 with `cc_last_error` set to a measured reason (`smDeclineText`). Declined
cases: bad parameter (thickness ≤ 0 / height < 0 / radius < 0 / angle outside (0,180));
a non-straight bend line; a non-recognised (non-rectangular / freeform) base; a
self-colliding flange (caught by the self-verify as a leak/overlap); an unrecognised body
for unfold. The engine NEVER hands a native void to OCCT.

## 5. Verification

- Gate (a) host `test_native_sheetmetal` (9 cases): closed-form volume on every built
  solid, the unfold BA + area, the fold→unfold area invariant, and the measured declines.
- Gate (b) sim `native_sheetmetal_selftest.mm`: under the native engine, `cc_check_solid`
  valid + `cc_mass_properties` volume = closed form + determinism + honest decline. NO
  OCCT comparison exists — stated in the harness.

## Cognitive complexity

`edgeFlange` is a linear face assembler with a small fan loop and a documented systems-band
audit tail (≈15, backend target). The recogniser/self-verify helpers are each ≤ ~8.
