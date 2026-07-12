// SPDX-License-Identifier: Apache-2.0
//
// freeform_freeform_multiseam_fixture.h — the reachable proof fixture for L3-d STAGE 5
// MULTI-SEAM freeform↔freeform CUT / COMMON: two coaxial freeform cups whose curved
// walls intersect in TWO disjoint closed circular seams (the SSI returns 2 loops), and
// its closed-form volume oracles. Shared by the host analytic gate and the sim
// native-vs-OCCT gate. OCCT-FREE; requires CYBERCAD_HAS_NUMSCI for the M1 seam trace.
//
// ── The two operands (both freeform cups, SAME degree-4 family) ──────────────────
//   * A = the "valley cup": a degree-4 Bézier VALLEY wall z_A = a·(r² − ρ₀²)² (a ring-
//     shaped moat: opens up on BOTH sides of the ring bottom at r = ρ₀), trimmed by a rim
//     CIRCLE of radius R, closed by a flat TOP LID above everything. Solid A occupies the
//     material ABOVE the valley wall, up to the lid.
//   * B = the "dome cup": B is A MIRRORED in z about z = H/2 — a degree-4 Bézier DOME wall
//     z_B = H − a·(r² − ρ₀²)² (an M-shaped ridge), trimmed by the SAME rim R, closed by a
//     flat BOTTOM LID below everything. Solid B occupies the material BELOW the dome wall.
//   Both walls are the SAME degree-4 family with IDENTICAL curvature magnitude (mirror
//   images), so the M0 mesher's curvature-driven boundary refinement MATCHES on both sides
//   of every shared seam — the two curved annuli weld watertight. (A degree-MISMATCHED
//   pair — e.g. a degree-4 valley ∩ a degree-2 dome — would leave a small T-junction
//   residual at the higher-curvature seam, which the verb HONEST-DECLINES, never leaks.)
//
// ── The TWO shared closed curved seams ───────────────────────────────────────────
// A's valley and B's dome meet where a·(r²−ρ₀²)² = H − a·(r²−ρ₀²)², i.e.
// (r²−ρ₀²)² = H/(2a) ⟹ r² = ρ₀² ± √(H/2a) — TWO CONCENTRIC CIRCLES at
// r₁ = √(ρ₀²−√(H/2a)) and r₂ = √(ρ₀²+√(H/2a)), both interior to the rim R, both at height
// z* = H/2. This is the multi-seam case: the SSI trace returns TWO closed loops, and each
// wall must be split by BOTH before the survivors can be sewn watertight.
//
// ── Closed-form volume oracles (no OCCT) ─────────────────────────────────────────
// The COMMON A ∩ B is the ANNULAR LENS between the two seams: over the ring r ∈ [r₁,r₂]
// where the dome is above the valley, bounded below by A's valley and above by B's dome.
//   V(A∩B) = ∫_{r₁}^{r₂} (z_B(r) − z_A(r))·2πr dr   (evaluated in closed form below).
// The CUT A − B = V(A) − V(A∩B).
//
#ifndef CYBERCAD_TESTS_NATIVE_FREEFORM_FREEFORM_MULTISEAM_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_FREEFORM_FREEFORM_MULTISEAM_FIXTURE_H

#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace freeform_freeform_multiseam_fixture {

namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;

// Pose: the valley RIM (r=R) is the wall's global MAX (R > ρ₀√2 ⇒ the flat rim seals the
// lid); B is A mirrored about z=H/2 so both walls share the SAME degree-4 curvature; the
// lens is H thick at its widest (clears the mesh ON-band at the working deflections, so
// membership votes crisply). Seams: r₁ ≈ 0.131, r₂ ≈ 0.374, both interior to R = 0.45.
inline constexpr double kA = 4.0;       // valley amplitude
inline constexpr double kRho0 = 0.28;   // valley ring-bottom radius (z_A = a(r²−ρ₀²)²)
inline constexpr double kH = 0.03;      // dome/valley z-mirror separation (lens max height)
inline constexpr double kR = 0.45;      // rim radius (both seams r₁,r₂ interior to it)
inline constexpr int kRimSegs = 96;     // rim circle polygon resolution
inline constexpr double kPi = 3.14159265358979323846;

inline double rho2() { return kRho0 * kRho0; }
inline double rho4() { return rho2() * rho2(); }
inline double zA(double r) { const double s = r * r - rho2(); return kA * s * s; }
inline double zB(double r) { return kH - zA(r); }  // B is A mirrored about z = H/2

// The two seam radii: r² = ρ₀² ± √(H/2a).
inline double seamR1() { return std::sqrt(rho2() - std::sqrt(kH / (2.0 * kA))); }  // ≈ 0.131
inline double seamR2() { return std::sqrt(rho2() + std::sqrt(kH / (2.0 * kA))); }  // ≈ 0.374

// lids sit EXACTLY at the (flat, radially-symmetric) wall rim height so the wall face and
// the flat lid SHARE the identical rim curve and the sewn cup is watertight. A's valley rim
// is its MAX (A valid: z_A ≤ lidA); B's dome rim is its MIN (z_B ≥ botB). The lids never
// touch the OTHER operand's wall (A's lid z=z_A(R) sits above B's dome max z=H−0; B's lid
// z=z_B(R) sits below A's valley floor z=0), so NO extra seam.
inline double lidA() { return zA(kR); }
inline double botB() { return zB(kR); }

// ── closed-form volume oracles (no OCCT) ─────────────────────────────────────────
inline double volA() {
  const double R2 = kR * kR, p = rho2();
  // π∫₀^{R²} (lidA − a(s−ρ₀²)²) ds.
  const double antid = lidA() * R2 - kA * ((R2 * R2 * R2) / 3.0 - p * R2 * R2 + p * p * R2);
  return kPi * antid;
}
inline double volB() { return volA(); }  // B is A mirrored ⇒ equal volume by symmetry
inline double volCommon() {
  // Annular lens over s ∈ [s₁,s₂]: π∫ (z_B − z_A) ds = π∫ (H − 2a(s−ρ₀²)²) ds.
  const double s1 = rho2() - std::sqrt(kH / (2.0 * kA));
  const double s2 = rho2() + std::sqrt(kH / (2.0 * kA));
  const double p = rho2();
  auto antid = [&](double s) {
    return kH * s - 2.0 * kA * (s * s * s / 3.0 - p * s * s + p * p * s);
  };
  return kPi * (antid(s2) - antid(s1));
}
inline double volCut() { return volA() - volCommon(); }

// ── A's degree-4 VALLEY Bézier (5×5 poles), z = a·(x²+y²−ρ₀²)², computed exactly by
// tensor Bernstein conversion (poles precomputed for a=4, ρ₀=0.28; x,y ∈ [−0.5,0.5]). ──
inline std::vector<fmath::Point3> valleyPoles() {
  const double xc[5] = {-0.5, -0.25, 0.0, 0.25, 0.5};
  // Z[i][j] = the exact degree-4 tensor-Bézier z-poles reproducing a(x²+y²−ρ₀²)².
  const double Z[5][5] = {
      {0.710986, -0.132214, 0.253386, -0.132214, 0.710986},
      {-0.132214, -0.475414, 0.076853, -0.475414, -0.132214},
      {0.253386, 0.076853, 0.684675, 0.076853, 0.253386},
      {-0.132214, -0.475414, 0.076853, -0.475414, -0.132214},
      {0.710986, -0.132214, 0.253386, -0.132214, 0.710986}};
  std::vector<fmath::Point3> poles;
  poles.reserve(25);
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) poles.push_back(fmath::Point3{xc[i], xc[j], Z[i][j]});
  return poles;
}
// ── B's DOME Bézier: z = H − z_A (A's poles mirrored: z ↦ H − z) ──────────────────
inline std::vector<fmath::Point3> domePoles() {
  std::vector<fmath::Point3> poles = valleyPoles();
  for (fmath::Point3& p : poles) p.z = kH - p.z;
  return poles;
}

inline topo::FaceSurface bezierSurface(const std::vector<fmath::Point3>& poles) {
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::Bezier;
  s.nPolesU = 5;
  s.nPolesV = 5;
  s.poles = poles;
  return s;
}
inline ssi::SurfaceAdapter valleyAdapter() { return ssi::makeBezierAdapter(valleyPoles(), 5, 5); }
inline ssi::SurfaceAdapter domeAdapter() { return ssi::makeBezierAdapter(domePoles(), 5, 5); }

// The real M1 seams: trace A's valley ∩ B's dome → the TWO closed WLines (circles r₁,r₂).
// The degree-4↔degree-4 trace is expensive (~2–3 min), so it is CACHED in a function-local
// static — every test in the process shares the ONE trace (the trace is a pure function of
// the fixed poles). Const-correct: callers only read the returned loops.
inline const std::vector<ssi::WLine>& closedSeams() {
  static const std::vector<ssi::WLine> cached = [] {
    const ssi::TraceSet tr = ssi::trace_intersection(valleyAdapter(), domeAdapter());
    std::vector<ssi::WLine> out;
    for (const ssi::WLine& w : tr.lines)
      if (w.points.size() >= 3 && w.isClosed()) out.push_back(w);
    return out;
  }();
  return cached;
}

// The rim circle in a wall's (u,v): (½ + R·cosθ, ½ + R·sinθ), CCW, kRimSegs samples.
inline std::vector<fmath::Point3> rimUV() {
  std::vector<fmath::Point3> uv;
  uv.reserve(kRimSegs);
  for (int k = 0; k < kRimSegs; ++k) {
    const double th = 2.0 * kPi * static_cast<double>(k) / kRimSegs;
    uv.push_back(fmath::Point3{0.5 + kR * std::cos(th), 0.5 + kR * std::sin(th), 0.0});
  }
  return uv;
}

// ── Build a cup solid: a degree-4 freeform wall (circular UV trim) closed by a flat lid
// at z = lidZ. Mirrors freeform_freeform_cut_fixture::buildCup. ──────────────────────
inline topo::Shape buildCup(const std::vector<fmath::Point3>& poles, double lidZ) {
  const topo::FaceSurface wall = bezierSurface(poles);
  tess::SurfaceEvaluator eval(wall, topo::Location{});
  const std::vector<fmath::Point3> uv = rimUV();
  const int m = static_cast<int>(uv.size());

  std::vector<fmath::Point3> rim3d(m);
  std::vector<topo::Shape> vRim(m);
  for (int k = 0; k < m; ++k) {
    rim3d[k] = eval.value(uv[k].x, uv[k].y);
    vRim[k] = topo::ShapeBuilder::makeVertex(rim3d[k]);
  }

  // rim edges as degree-2 Bézier 3-D curves (over a straight UV chord); built ONCE and
  // shared by the wall face + the lid. With kRimSegs=96 the degree-2 arc tracks the
  // degree-4 wall to ~5e-8 ≪ the mesher's kSnapEps, so the cup sews watertight.
  std::vector<topo::Shape> rimEdges(m);
  std::vector<fmath::Point3> rimCtrl(m);
  for (int k = 0; k < m; ++k) {
    const int k1 = (k + 1) % m;
    const fmath::Point3 mid{(uv[k].x + uv[k1].x) * 0.5, (uv[k].y + uv[k1].y) * 0.5, 0.0};
    const fmath::Point3 Sm = eval.value(mid.x, mid.y);
    rimCtrl[k] = fmath::Point3{2 * Sm.x - 0.5 * (rim3d[k].x + rim3d[k1].x),
                               2 * Sm.y - 0.5 * (rim3d[k].y + rim3d[k1].y),
                               2 * Sm.z - 0.5 * (rim3d[k].z + rim3d[k1].z)};
    topo::EdgeCurve c{};
    c.kind = topo::EdgeCurve::Kind::Bezier;
    c.degree = 2;
    c.poles = {rim3d[k], rimCtrl[k], rim3d[k1]};
    rimEdges[k] = topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, vRim[k], vRim[k1]);
  }

  std::vector<topo::Shape> faces;

  // wall (freeform), circular UV trim. Reversed ⇒ outward normal points away from the
  // solid interior (consistent closed shell — the B1 recogniser audit).
  {
    const topo::Shape node = topo::ShapeBuilder::makeFace(wall, topo::Shape{});
    std::vector<topo::Shape> we;
    we.reserve(m);
    for (int k = 0; k < m; ++k) {
      const int k1 = (k + 1) % m;
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = fmath::Point3{uv[k].x, uv[k].y, 0.0};
      pc.dir2d = fmath::Vec3{uv[k1].x - uv[k].x, uv[k1].y - uv[k].y, 0.0};
      we.push_back(topo::ShapeBuilder::addPCurve(rimEdges[k], node.tshape(), pc));
    }
    faces.push_back(topo::ShapeBuilder::makeFace(wall, topo::ShapeBuilder::makeWire(std::move(we)),
                                                 {}, topo::Orientation::Reversed));
  }

  // lid (plane z = lidZ), bounded by the SAME rim edges, reversed order + orientation.
  {
    topo::FaceSurface pl{};
    pl.kind = topo::FaceSurface::Kind::Plane;
    pl.frame.origin = fmath::Point3{0, 0, lidZ};
    pl.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
    pl.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
    pl.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
    const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
    std::vector<topo::Shape> we;
    we.reserve(m);
    for (int k = m - 1; k >= 0; --k) {
      const int k1 = (k + 1) % m;
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::BSpline;
      pc.degree = 2;
      pc.poles2d = {fmath::Point3{rim3d[k].x, rim3d[k].y, 0.0},
                    fmath::Point3{rimCtrl[k].x, rimCtrl[k].y, 0.0},
                    fmath::Point3{rim3d[k1].x, rim3d[k1].y, 0.0}};
      pc.knots = {0, 0, 0, 1, 1, 1};
      we.push_back(topo::ShapeBuilder::addPCurve(rimEdges[k], node.tshape(), pc).reversedShape());
    }
    faces.push_back(topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(we)),
                                                 {}, topo::Orientation::Forward));
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// A = the valley cup (top lid at z=z_A(R)); B = the dome cup (bottom lid at z=z_B(R)).
inline topo::Shape buildA() { return buildCup(valleyPoles(), lidA()); }
inline topo::Shape buildB() { return buildCup(domePoles(), botB()); }

}  // namespace freeform_freeform_multiseam_fixture

#endif  // CYBERCAD_TESTS_NATIVE_FREEFORM_FREEFORM_MULTISEAM_FIXTURE_H
