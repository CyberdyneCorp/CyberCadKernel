# Proposal — nurbs-gordon-surface (NURBS roadmap Layer 6)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 1 (the
exact-NURBS *geometry kernel*, `src/native/math/bspline_ops.{h,cpp}`), Layer 7 (fitting /
approximation, `src/native/math/bspline_fit.{h,cpp}`), and the first Layer-6 slices — **skinning /
lofting** (`bspline_skin.{h,cpp}`) and **swept surfaces** (`bspline_sweep.{h,cpp}`) — are landed.
Skinning interpolates a surface through ONE family of parallel section curves; the next Layer-6
capability is the **GORDON / NETWORK surface**: given a NETWORK of curves in BOTH parameter
directions — `K` u-direction curves `C_k(u)` and `L` v-direction curves `D_l(v)` that intersect at a
`K × L` grid — construct a single tensor-product B-spline **surface that interpolates every network
curve** (the surface contains every `C_k` and every `D_l` as an iso-curve). This is the "curve
network → smooth skin" workflow (the way designers build a freeform patch from a wireframe of
profile + guide curves).

This slice is worth building **now** because it (a) is small and well-bounded (*The NURBS Book*
§10.5 — the Gordon surface is the classic **boolean sum** `G = S_u ⊕ S_v ⊖ T`), (b) is built entirely
on machinery that already exists — the Layer-6 skin/compatibility path to loft each direction's
family, the Layer-7 tensor-product interpolation for the grid, and the **Layer-1 exact** surface ops
(`elevateDegreeSurface` / `refineKnotSurface`) to bring the three summands to one common basis so
their control nets add/subtract — and (c) has a **uniquely airtight oracle**: the finished surface
must contain every input network curve *pointwise* (closed-form containment residual → 0), and a
known tensor-product surface's own iso-curve network, fed back through the Gordon builder, must
reconstruct it. It composes verified lower layers into a new capability with a machine-precision
oracle, and DECLINES honestly on an inconsistent network.

## What

A new OCCT-free module `src/native/math/bspline_gordon.{h,cpp}` (namespace
`cybercad::native::math`, beside `bspline_skin` / `bspline_sweep`), **numsci-gated**
(`CYBERCAD_HAS_NUMSCI`, like `bspline_skin.cpp`) because the family/grid interpolations solve linear
systems through the numsci facade. It reuses the Layer-1 `BsplineCurveData` / `BsplineSurfaceData`
types for its input (the curve network) and output (the surface). **Non-rational network only** (all
weights = 1); rational/weighted Gordon surfaces are an explicit residual.

From *The NURBS Book* (Piegl & Tiller, 2nd ed.), §10.5 — the Gordon surface as a BOOLEAN SUM:

    G = S_u ⊕ S_v ⊖ T

1. **Network consistency** (`verifyNetwork`) — a curve network only interpolates when its curves
   actually form a consistent grid: for every `(k, l)` the u-curve evaluated at the v-curve's
   station, `C_k(u_l)`, must equal the v-curve evaluated at the u-curve's station, `D_l(v_k)` (both
   equal the grid point `Q_{k,l}`), to within tolerance; the station params must be strictly
   monotone; both families non-rational and well-formed. `verifyNetwork` checks this and reports the
   worst mismatch; `gordonSurface` DECLINES (`ok=false`, with a reason) on an inconsistent or
   degenerate network — never a surface that silently misses its own curves.
2. **Boolean sum** (`gordonSurface`, §10.5) — build the three summands, each interpolated at the
   PRESCRIBED station params: `S_u` = the surface lofting the `K` u-curves across v at `vParams`;
   `S_v` = the surface lofting the `L` v-curves across u at `uParams`; `T` = the tensor-product
   interpolant of the `K × L` intersection grid at `(uParams, vParams)`. Raise the three to a COMMON
   degree and merge them onto COMMON knot vectors in each direction with the exact Layer-1 surface
   ops, then form the Gordon net pointwise `poles(G) = poles(S_u) + poles(S_v) − poles(T)`. The
   Coons/Gordon cancellation makes `G` interpolate every network curve: on `v = v_k`, `S_u` reduces
   to `C_k` and `S_v − T` cancels, so `G(·, v_k) = C_k`; symmetrically `G(u_l, ·) = D_l`.

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_gordon.cpp`:
  1. **Network containment (the core oracle)** — the Gordon surface contains EVERY input u-curve and
     v-curve POINTWISE: `S(·, v_k)` reproduces `C_k` and `S(u_l, ·)` reproduces `D_l` on a dense
     sample to ~1e-8 (achieved ~5e-15).
  2. **Grid intersection** — the `K × L` intersection grid points lie on the surface exactly:
     `S(u_l, v_k) == Q_{k,l}` to ~1e-8 (achieved ~1e-15).
  3. **Known-surface round-trip** — extract a u/v iso-curve network from a KNOWN tensor-product
     surface at its Greville abscissae → Gordon → recover the original surface closely (~1e-6 for a
     uniform-knot source; the averaging-knot vs source-knot confound is documented). The MACHINE-EXACT
     full-surface identity is IDEMPOTENCE: build `G1`, extract `G1`'s own iso-curve network at the SAME
     station params, rebuild `G2` → `G1 ≡ G2` pointwise to machine precision (achieved ~4e-15).
  4. **Honest declines** — an inconsistent network (curves that do not intersect on the grid),
     fewer than two curves in a direction, a rational curve, mismatched param sizes, and
     non-monotone stations are declined (`ok=false`, with a reason), never a silently-wrong surface,
     never a crash.
- **SIM native-vs-OCCT parity** — OPTIONAL cross-check against OCCT `GeomFill_BSplineCurves` /
  `BRepFill` network surfacing (a separate track; HOST is primary and sufficient).

## Scope

- Adds `src/native/math/bspline_gordon.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob. Added to `native_math.h`.
- Adds `tests/native/test_native_nurbs_gordon.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_nurbs_skin`.
- Only `#include`s `bspline_ops.h` (Layer 1), `bspline_skin.h` (Layer 6), the evaluators, and the
  numerics facade — it does NOT modify them.
- **`cc_*` ABI unchanged.** Layer 6 is an internal geometry-algorithm library; its consumers are
  later surfacing features, not the app today. No ABI is added until a consumer needs it —
  consistent with the demand-driven policy.

## Non-goals

- **No rational / weighted Gordon surfaces** — interpolating the network's weights is materially
  harder and is an explicit residual for a later slice. This module builds non-rational surfaces from
  non-rational networks only and never fabricates weights.
- **No irregular / N-sided networks** — the boolean sum requires a REGULAR (grid) network of two
  transversal families intersecting at a `K × L` grid. Irregular networks, N-sided / boundary-curve
  filling, trimmed-boundary patches, and plate/energy surfaces remain demand-gated residuals.
- **No exact GeomFill/BRepFill continuous surfacing** — the boolean-sum Gordon surface reconstructs a
  known surface up to an averaging-knot parametrization residual (~1e-6); the exact continuous
  BRepFill construction is a distinct residual.
- No error-driven adaptive knot refinement; no automatic degree/knot selection; no new `cc_*` ABI;
  no change to STEP admission, the tessellator, or any evaluator signature.
