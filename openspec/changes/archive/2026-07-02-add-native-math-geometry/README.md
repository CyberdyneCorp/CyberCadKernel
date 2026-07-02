# add-native-math-geometry

Phase 4 capability #1 (`native-math`): an OCCT-free, host-buildable C++20 math and
geometry-evaluation foundation library under `src/native/math/`. Provides fp64
value types (`vec3`/`point3`/`dir3`/4×4 transform), parametric **curve** evaluation
(Bézier via de Casteljau; B-spline/NURBS point + derivatives via basis functions /
de Boor with rational weights), tensor-product **surface** evaluation
(Bézier/B-spline/NURBS point + partial derivatives + normal), and **elementary
surfaces** (plane / cylinder / cone / sphere) point + normal. Clean-room from
*The NURBS Book* (FindSpan A2.1, BasisFuns A2.2, CurvePoint A3.1, CurveDerivs
A3.2, SurfacePoint A3.5, SurfaceDerivs A3.6) and de Casteljau, with OCCT
(`gp_*`, `BSplCLib`/`BSplSLib`/`PLib`/`ElSLib`) as a numeric oracle only — not
copied. Every requirement is verified by (a) a host analytic unit test compiled
with `clang++ -std=c++20` (no OCCT) AND (b) a native-vs-OCCT numeric parity test
on the iOS simulator within a tight fp64 tolerance. This change delivers the math
library and its verification ONLY — no engine wiring and no `cc_*` ABI change
(that starts with `native-topology` / `native-construction`).
