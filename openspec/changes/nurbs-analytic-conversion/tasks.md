# Tasks — nurbs-analytic-conversion

## 1. Module scaffold
- [x] 1.1 Create `src/native/math/analytic_nurbs.h` — analytic descriptors (`Circle`,
      `Arc`, `Ellipse`, `LineSegment`), construction declarations, recognition result
      types (`CurveRecognition`, `SurfaceRecognition`, `CurveKind`, `SurfaceKind`).
- [x] 1.2 Guard the TU under `CYBERCAD_HAS_NUMSCI` (recognition uses `primitive_fit`),
      mirroring `primitive_fit.cpp`; OCCT-free.

## 2. Analytic → NURBS (exact)
- [x] 2.1 Rational-quadratic arc segment builder (P&T §7.3, weight `cos(halfSweep)`,
      tangent pole at `r/cos(half)`); `circleToNurbs` (4 quarter segments, 9 poles),
      `arcToNurbs` (≤90° segments), `ellipseToNurbs` (affine image of the circle net),
      `lineToNurbs` (degree-1 two-pole segment).
- [x] 2.2 Surface-of-revolution assembly (P&T §7.5); `planeToNurbs` (bilinear patch),
      `cylinderToNurbs`, `coneToNurbs`, `sphereToNurbs`, `torusToNurbs`.

## 3. NURBS → analytic (recognition + algebraic certificate)
- [x] 3.1 Curve recognition: line (collinear net + constant weight), circle/arc
      (coplanar net + Kåsa circle through on-curve poles + tangent-pole/weight pattern),
      ellipse (central-conic fit through dense on-curve samples + affine-circle tangent
      pattern). Control-net residual ≤ tol required for acceptance.
- [x] 3.2 Surface recognition: build the fitted quadric matrix Q and CERTIFY the surface
      lies on it via the numerator polynomial N(u,v)=p̃ᵀQp̃ ≡ 0 (dense-grid vanishing ⇒
      identically zero), fed by a primitive fit on evaluated surface samples (not the
      off-surface tangent poles). Plane / cylinder / cone / sphere.
- [x] 3.3 Honest "general" fallback — a fit with small RMS but a non-vanishing algebraic
      certificate is rejected, never forced.

## 4. Host gate (airtight oracles)
- [x] 4.1 Round-trip: `recognize(toNurbs(prim)) == prim` for circle/arc/ellipse/line and
      plane/cylinder/cone/sphere (params ≤ 1e-12).
- [x] 4.2 Evaluation exactness: points on `circleToNurbs`/`cylinderToNurbs`/`sphereToNurbs`/
      `torusToNurbs` lie on the TRUE primitive to ≤ 1e-13 (rational quadratic is exact).
- [x] 4.3 Discrimination: a bicubic freeform bump → General; a weight-perturbed and a
      pole-displaced almost-circle → General, not a spurious primitive.
- [x] 4.4 Wire the module + test into `CMakeLists.txt` under the `CYBERCAD_HAS_NUMSCI`
      block; verify a clean configure and a green `ctest`.
