# Tasks — nurbs-surface-join-g1-g2 (NURBS roadmap Layer 6)

## 1. Module + join API (additive; every builder API byte-unchanged)
- [x] 1.1 Create `src/native/math/bspline_join.h` — `SurfaceEdge`, `EdgeSpec`, `EdgeCompatResult`,
      `JoinResult`, and the `makeCompatibleAlongEdge` / `joinG1` / `joinG2` declarations. Reuses the
      Layer-1 `BsplineSurfaceData` type from `bspline_ops.h`. Leave every existing module untouched.

## 2. Edge compatibility
- [x] 2.1 `makeCompatibleAlongEdge` — degree-match along the shared edge by `elevateDegreeSurface`
      (elevate the lower-degree side; exact) in the along-edge param direction.
- [x] 2.2 Knot-merge along the edge: union of interior knots (the other side's, affinely remapped to
      this side's domain) inserted via `refineKnotSurface` (exact). Verify the along-edge pole counts
      then match.
- [x] 2.3 Decline honestly on a malformed patch, a rationality mismatch, a degenerate along-edge knot
      domain, or a shared boundary that is not actually coincident (not C0).

## 3. G1 join (minimal-movement tangent-plane)
- [x] 3.1 Extract three control rows parallel to the edge (0 = boundary/frozen, 1 = cross-tangent,
      2 = cross-2nd-derivative), honouring the reversed-orientation along-edge station map.
- [x] 3.2 No-op guard: measure the G1 residual (max unit-normal mismatch across the edge) FIRST; if
      already below tol, return movement 0.
- [x] 3.3 Closed-form minimal-movement scalar `s > 0`: minimise `Σ|B_row1[i] − (P0 − s·A1off[i])|²`
      (`s = Σ d·g / Σ g·g`, `g = −A1off`, `d = B_row1cur − P0`); magnitude-match fallback keeps `s > 0`.
- [x] 3.4 Place B's row 1 on `P0 − s·A1off` (collinear shared ribbon → unit-normal continuous). Optional
      `adjustBoth` splits the move symmetrically onto A's row 1.
- [x] 3.5 Movement cap: decline honestly if the required max displacement exceeds the caller cap (would
      distort the surface); never widen a tolerance.

## 4. G2 join (additionally match the second cross-derivative)
- [x] 4.1 No-op guard: if already G1 (≤ tol) AND G2 (relative normal-curvature ≤ 1e-5), return
      movement 0 (the analytic cylinder-split case).
- [x] 4.2 Enforce G1 first (row 1); recover the used `s` from the placed row.
- [x] 4.3 Place B's row 2 on `P0 + s²·A2off − 2s(s+1)·A1off` so the cross-boundary second difference
      matches (normal curvature continuous), G1 + C0 preserved. Decline if the patch lacks a third row
      (cross-degree < 2) or the movement exceeds the cap.

## 5. Verification (HOST-analytic, airtight oracle)
- [x] 5.1 `tests/native/test_native_nurbs_join.cpp` — coplanar no-op (movement 0), G1-enforced normal
      continuity (≤ 1e-7 rad) with boundary frozen (≤ 1e-12), G2-enforced curvature continuity
      (relative ≤ 1e-5) with G1 held, cylinder-halves-split no-op, and honest over-cap decline.
- [x] 5.2 Confirm `src/native/**` stays OCCT-free (0 OCCT/Geom/BRep/TK refs in the changed files) and
      the `cc_*` facade is byte-unchanged (additive only).
- [x] 5.3 Wire the test into `CMakeLists.txt` under `CYBERCAD_HAS_NUMSCI` (list + `_SRC` + compile-def).
      Configure + build + run green (`ctest -R nurbs_join`).
