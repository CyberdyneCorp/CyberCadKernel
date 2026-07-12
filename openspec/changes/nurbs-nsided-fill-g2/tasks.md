# Tasks — nurbs-nsided-fill-g2

## 1. Module + G2 API (additive; C0 + G1 APIs byte-unchanged)
- [x] 1.1 Create `src/native/math/bspline_nsided_g2.h` — `CrossCurvatureField` (alias of the shared
      `CrossTangentField` shape), `NSidedFillG2Result`, and the `nSidedFillG2` declaration. Reuses
      `NSidedBoundary` / `verifyNSidedBoundary` from `bspline_nsided.h` and `CrossTangentField` from
      `bspline_nsided_g1.h`. Add to `native_math.h`. Leave `bspline_nsided.{h,cpp}` and
      `bspline_nsided_g1.{h,cpp}` byte-unchanged.

## 2. G2 Gregory pie-slice construction
- [x] 2.1 Reuse `verifyNSidedBoundary` (non-closed / rational / degenerate / N<3 decline honestly).
- [x] 2.2 Corners `V[i]=edges[i](0)`, centroid `C=mean(V[i])`; degenerate-corner guard. Pre-elevate each
      edge to degree ≥ 5 (exact Layer-1 A5.9) so each slice has three distinct seam-adjacent u-columns
      per side; boundary interpolation stays exact.
- [x] 2.3 Per-edge cross-tangent + cross-curvature fields (prescribed or natural minimal-energy);
      G1-incompatibility guard on a parallel cross-tangent (never widen tolerance).
- [x] 2.4 Shared per-corner spoke: the quintic position column `V[k]→C` plus a 1st-inward rib and a
      2nd-inward rib, injected into the three seam-adjacent u-columns' interior v-levels via the exact
      clamped-endpoint u-derivative identities so `∂S/∂u` AND `∂²S/∂u²` along each seam are the identical
      vectors on both incident slices (C0 + G1 + G2 by pole equality). Degenerate hub apex kept finite by
      radially-tapered interior 1st/2nd rows.
- [x] 2.5 G2-feasibility guards — decline a G1-infeasible creased corner (inherited), a
      curvature-discontinuous boundary corner (incident edges' curvature vectors disagree), and a
      prescribed cross-curvature whose incident-corner values point in opposing directions.
- [x] 2.6 Emit N non-rational quintic-in-v sub-patches (u = edge basis, v = degree 5) + the centroid.

## 3. HOST-analytic gate (no OCCT — the airtight oracle)
- [x] 3.1 `tests/native/test_native_nurbs_nsided_g2.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_nsided_g1`: `_SRC` + `CYBERCAD_TESTS` list and the per-target
      `CYBERCAD_HAS_NUMSCI` compile-definition under the option block).
- [x] 3.2 Oracles — boundary interpolation ≤ 1e-12; G2 across spokes (unit normal ≤ 1e-6 rad AND normal
      curvature relative ≤ 1e-5); planar → planar fill ≤ 1e-10; analytic sphere point + curvature match;
      degenerate-hub no-blowup; honest declines (non-closed / rational / N<3 / malformed / wrong tangent-
      or curvature-field count / parallel cross-tangent / curvature-creased boundary / corner-incompatible
      curvature).

## 4. Invariants + docs
- [x] 4.1 `src/native/**` OCCT-free (0 OCCT/Geom/BRep/TK refs in the changed files); `cc_*` facade
      untouched (ABI byte-unchanged); CMake `if(`/`endif(` balance preserved.
- [x] 4.2 `openspec validate nurbs-nsided-fill-g2 --strict` passes; full NURBS-family ctest green.
- [x] 4.3 Update `docs/NURBS-SCOPE.md` Layer-6 N-sided row (Gregory G2 blend: residual → landed).
