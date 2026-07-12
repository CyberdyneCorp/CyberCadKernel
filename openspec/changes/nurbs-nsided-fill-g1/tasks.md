# Tasks — nurbs-nsided-fill-g1

## 1. Module + G1 API (additive; C0 API byte-unchanged)
- [x] 1.1 Create `src/native/math/bspline_nsided_g1.h` — `CrossTangentField`, `NSidedFillG1Result`, and
      the `nSidedFillG1` declaration. Reuses `NSidedBoundary` / `verifyNSidedBoundary` from
      `bspline_nsided.h` and the Layer-1 `BsplineCurveData` / `BsplineSurfaceData` types. Add to
      `native_math.h` if present. Leave `bspline_nsided.{h,cpp}` untouched.

## 2. Gregory pie-slice construction
- [x] 2.1 Reuse `verifyNSidedBoundary` (non-closed / rational / degenerate / N<3 decline honestly).
- [x] 2.2 Corners `V[i]=edges[i](0)`, centroid `C=mean(V[i])`; degenerate-corner guard (a corner
      coinciding with the centroid). Pre-elevate each edge to degree ≥ 3 (exact Layer-1 A5.9) so each
      slice has two distinct interior u-columns for independent seam ribs; boundary interpolation stays
      exact.
- [x] 2.3 Per-edge cross-tangent field (prescribed `CrossTangentField` or natural radial-to-centroid);
      G1-incompatibility guard — decline a prescribed cross-tangent (anti-)parallel to the boundary
      tangent (no tangent plane; never widen tolerance).
- [x] 2.4 Shared corner spoke columns `V[k]→C` (cubic Hermite) built identically for both incident
      slices (exact C0 seam), and a shared cross-spoke rib field per corner injected into the second /
      second-to-last u-columns' interior v-levels so `∂S/∂u` along each seam is the identical vector on
      both slices (C1 ⇒ G1 by pole equality). Degenerate hub apex kept finite by the radial twist row.
- [x] 2.5 G1-feasibility guard at corners — decline a boundary that creases in 3-D (non-collinear edge
      tangents not coplanar with the spoke): no tangent plane across the incident spokes.
- [x] 2.6 Emit N non-rational bicubic sub-patches (u = edge basis, v = cubic) + the centroid.

## 3. HOST-analytic gate (no OCCT — the airtight oracle)
- [x] 3.1 `tests/native/test_native_nurbs_nsided_g1.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_nsided`: `_SRC` + `CYBERCAD_TESTS` list and the per-target
      `CYBERCAD_HAS_NUMSCI` compile-definition under the option block).
- [x] 3.2 Oracles — boundary interpolation ≤ 1e-12; G1 across spokes ≤ 1e-6 rad (planar pentagon /
      triangle / heptagon → 0; smooth non-planar loop → ~1e-16); planar → planar fill ≤ 1e-10;
      degenerate-hub no-blowup; honest declines (non-closed / rational / N<3 / malformed / wrong field
      count / parallel cross-tangent / creased 3-D corner); C0-vs-G1 crease contrast on the creased case.

## 4. Invariants + docs
- [x] 4.1 `src/native/**` OCCT-free (0 OCCT/Geom/BRep/TK refs in the changed files); `cc_*` facade
      untouched (ABI byte-unchanged); CMake `if(`/`endif(` balance preserved.
- [x] 4.2 `openspec validate nurbs-nsided-fill-g1 --strict` passes; full NURBS-family ctest green.
- [x] 4.3 Update `docs/NURBS-SCOPE.md` Layer-6 N-sided row (Gregory G1 blend: residual → landed).
