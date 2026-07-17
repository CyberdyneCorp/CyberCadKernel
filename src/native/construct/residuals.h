// SPDX-License-Identifier: Apache-2.0
//
// residuals.h — Tier-1 + Tier-2#4 native-geometry COMPLETION batch
// ("construct-residuals"): the two profile-builder residuals that were
// OCCT-fallthrough after Tier-A/B/C/D (see openspec/NATIVE-REWRITE.md #4b):
//
//   1. kind-3 SPLINE profile edges — a B-spline curve edge (via native-math
//      NURBS) in a TYPED profile, for BOTH extrude and revolve.
//
//   2. OFF-AXIS-ARC revolve → a TORUS surface of revolution (native-math Torus
//      added in src/native/math/torus.h) — an arc whose circle centre sits at a
//      NON-ZERO distance from the revolution axis sweeps a torus, not a sphere.
//
// This header EXTENDS the construct builders in its OWN file (it does not touch
// construct.h / profile.h / loft.h / sweep.h / thread.h). Its public entries are
// thin wrappers the engine-wiring step will call:
//
//   build_prism_profile_spline(segs, splineXY, splineN, holes…, depth)
//        Extrude a typed profile that MAY contain kind-3 spline segments. A spline
//        segment becomes a TRUE B-spline cap edge (fitted through its control
//        points) + a degree-1-in-depth B-spline "extruded surface" side wall. Line
//        / arc segments reuse the profile.h machinery (Plane / Cylinder walls).
//        Watertight → NATIVE. Degenerate / self-crossing profile → NULL (OCCT).
//
//   build_revolution_profile_spline(segs, splineXY, splineN, axis, angle)
//        Revolve a typed profile that MAY contain a kind-3 spline meridian OR an
//        OFF-AXIS circular arc. A spline meridian sweeps a general rational
//        surface of revolution; an off-axis arc sweeps a TORUS band. Both are
//        emitted as EXACT rational-quadratic B-spline patches (the angular circle
//        is an exact rational Bézier/NURBS; the current tessellator meshes the
//        BSpline face kind without a new surface kind). Line segments → Plane /
//        Cylinder / Cone; on-axis arc → Sphere (delegated to profile.h). The
//        result is SELF-VERIFY-gated by the CALLER (the engine meshes it and
//        checks watertight + volume); a meridian that FOLDS / crosses the axis (a
//        genuinely self-intersecting surface of revolution — Tier-4 SSI territory)
//        makes this builder return NULL so the engine falls through to OCCT. Never
//        faked.
//
// SELF-INTERSECTION / HONEST FALLBACK (mandatory, per every prior native tier):
//   * A spindle torus (major radius R < minor r — the arc would cross the axis)
//     → NULL (OCCT).
//   * A spline generatrix that dips to r ≤ 0 (crosses the axis) → NULL (OCCT).
//   * Any profile whose swept surface would self-intersect (tight fold) is NOT
//     patched here — it needs surface-surface intersection (Tier 4). Return NULL.
//
// REFERENCE ORACLE ONLY (nothing copied): OCCT GeomFill_Pipe / Geom_SurfaceOf-
// Revolution + Geom_ToroidalSurface and BRepBuilderAPI_MakeEdge(Geom_BSplineCurve)
// were consulted to confirm the surface-of-revolution decomposition and the exact
// rational-NURBS torus representation (a torus is bicubic-rational = the product of
// two rational-quadratic circles); the B-spline interpolation follows *The NURBS
// Book* (Piegl & Tiller) A9.1 global interpolation.
//
// OCCT-FREE. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_CONSTRUCT_RESIDUALS_H
#define CYBERCAD_NATIVE_CONSTRUCT_RESIDUALS_H

#include "native/construct/construct.h"
#include "native/construct/profile.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::construct {

namespace residual_detail {

using math::Point3;
using math::Vec3;

// ─────────────────────────────────────────────────────────────────────────────
// B-spline interpolation of a point list (The NURBS Book A9.1).
//
// Fit a clamped B-spline of degree p (≤ 3, reduced when few points) that passes
// through the given control points, chord-length parametrized with averaged knots.
// Returns the poles + flat knot vector for an EdgeCurve::Kind::BSpline (poles ==
// interpolation solved poles; for a genuinely INTERPOLATING spline we solve a
// banded linear system, but for the kernel's purposes a control-polygon spline
// (the points as poles) is a faithful, monotone, self-consistent curve that the
// tessellator + edge cache discretize deterministically. We use the points AS the
// poles with a clamped uniform-ish knot vector: this is exactly the representation
// OCCT's addSplineEdge side-channel implies (a B-spline whose control points are
// the supplied stream), so the native curve matches the oracle's control net).
// ─────────────────────────────────────────────────────────────────────────────

// Clamped knot vector for `nPoles` control points of degree `p` over [0,1], with
// interior knots uniformly spaced. Length = nPoles + p + 1.
inline std::vector<double> clampedKnots(int nPoles, int p) {
  std::vector<double> U;
  const int m = nPoles + p + 1;  // total knots
  U.reserve(static_cast<std::size_t>(m));
  for (int i = 0; i < p + 1; ++i) U.push_back(0.0);
  const int interior = nPoles - p - 1;  // # interior knots
  for (int i = 1; i <= interior; ++i)
    U.push_back(static_cast<double>(i) / static_cast<double>(interior + 1));
  for (int i = 0; i < p + 1; ++i) U.push_back(1.0);
  return U;
}

// ── GLOBAL B-SPLINE INTERPOLATION (The NURBS Book A9.1) ───────────────────────
// Solve for the poles of a clamped degree-p B-spline that PASSES THROUGH the given
// points `Q` at chord-length parameters, so the native curve matches OCCT's
// GeomAPI_PointsToBSpline (which interpolates) rather than merely using the points
// as control poles (which bows inward, off by ~40% area on a bowed profile).
//
// Steps (A9.1): (1) chord-length parameters ū_k (eq 9.5); (2) averaged interior
// knots (eq 9.8); (3) coefficient matrix A[k][i] = N_{i,p}(ū_k); (4) solve A·P = Q
// per coordinate by Gaussian elimination with partial pivoting. Returns the poles
// (same count as Q) and the averaged knot vector; empty on a singular/degenerate
// system so the caller can fall back to the control-polygon representation.

// Chord-length parameters (eq 9.5): normalized cumulative distance, ū_0=0, ū_n=1.
inline std::vector<double> chordParams(const std::vector<Point3>& Q) {
  const int n = static_cast<int>(Q.size()) - 1;
  std::vector<double> u(static_cast<std::size_t>(n) + 1, 0.0);
  double total = 0.0;
  for (int k = 1; k <= n; ++k) total += math::distance(Q[k], Q[k - 1]);
  if (!(total > 0.0)) return {};
  for (int k = 1; k < n; ++k) u[k] = u[k - 1] + math::distance(Q[k], Q[k - 1]) / total;
  u[n] = 1.0;
  return u;
}

// Averaged clamped knot vector (eq 9.8) for interpolation parameters `u`, degree p.
inline std::vector<double> averagedKnots(const std::vector<double>& u, int p) {
  const int n = static_cast<int>(u.size()) - 1;
  std::vector<double> U(static_cast<std::size_t>(n) + p + 2, 0.0);
  for (int i = 0; i <= p; ++i) U[static_cast<std::size_t>(n) + 1 + i] = 1.0;
  for (int j = 1; j <= n - p; ++j) {
    double s = 0.0;
    for (int i = j; i <= j + p - 1; ++i) s += u[i];
    U[static_cast<std::size_t>(j) + p] = s / p;
  }
  return U;
}

// Solve A·x = b (n×n) in place by Gaussian elimination with partial pivoting.
// Each rhs column is one coordinate. Returns false if A is singular.
inline bool solveLinear(std::vector<double>& A, std::vector<Point3>& b, int n) {
  auto at = [&](int r, int c) -> double& { return A[static_cast<std::size_t>(r) * n + c]; };
  for (int col = 0; col < n; ++col) {
    int piv = col;
    double best = std::fabs(at(col, col));
    for (int r = col + 1; r < n; ++r) {
      const double v = std::fabs(at(r, col));
      if (v > best) { best = v; piv = r; }
    }
    if (best < 1e-12) return false;
    if (piv != col) {
      for (int c = 0; c < n; ++c) std::swap(at(col, c), at(piv, c));
      std::swap(b[col], b[piv]);
    }
    for (int r = col + 1; r < n; ++r) {
      const double f = at(r, col) / at(col, col);
      for (int c = col; c < n; ++c) at(r, c) -= f * at(col, c);
      b[r] = Point3{b[r].x - f * b[col].x, b[r].y - f * b[col].y, b[r].z - f * b[col].z};
    }
  }
  for (int r = n - 1; r >= 0; --r) {
    Point3 s = b[r];
    for (int c = r + 1; c < n; ++c) {
      s = Point3{s.x - at(r, c) * b[c].x, s.y - at(r, c) * b[c].y, s.z - at(r, c) * b[c].z};
    }
    const double d = at(r, r);
    b[r] = Point3{s.x / d, s.y / d, s.z / d};
  }
  return true;
}

// Global interpolation (A9.1): return poles of a degree-p B-spline through Q with
// knot vector `U`. Empty on a singular system.
inline std::vector<Point3> interpolatePoles(const std::vector<Point3>& Q,
                                            const std::vector<double>& u,
                                            const std::vector<double>& U, int p) {
  const int n = static_cast<int>(Q.size()) - 1;
  std::vector<double> A(static_cast<std::size_t>(n + 1) * (n + 1), 0.0);
  std::array<double, 64> N{};
  for (int k = 0; k <= n; ++k) {
    const int span = math::findSpan(n, p, u[k], {U.data(), U.size()});
    math::basisFuns(span, u[k], p, {U.data(), U.size()},
                    {N.data(), static_cast<std::size_t>(p + 1)});
    for (int j = 0; j <= p; ++j)
      A[static_cast<std::size_t>(k) * (n + 1) + (span - p + j)] = N[j];
  }
  std::vector<Point3> P(Q);
  if (!solveLinear(A, P, n + 1)) return {};
  return P;
}

// A fitted open B-spline curve: degree, poles, clamped knot vector over [0,1].
//
// `sampled` is a DENSE polyline evaluation of the true NURBS curve (curvePoint at
// uniform u∈[0,1]). It exists because the tessellator samples a free-form PCURVE by
// LINEAR interpolation of its 2D poles (pcurveValue's Bézier/BSpline arm), while the
// shared 3D EdgeCache discretizes the TRUE curve — the two disagree for a genuinely
// curved spline, opening the cap↔wall seam. Representing the extrude edges/pcurves/
// wall as a common DEGREE-1 spline through `sampled` makes all three (3D edge curve,
// cap pcurve, wall surface) evaluate to the IDENTICAL points at every shared
// fraction, so the seam welds watertight. The curve SHAPE is still the fitted NURBS
// (sampled finely); only the B-rep storage is the deflection-bounded polyline.
struct SplineCurve {
  int degree = 3;
  std::vector<Point3> poles;
  std::vector<double> knots;   // flat, [0,1]
  std::vector<Point3> sampled; // dense polyline of the true curve, z=0 (extrude use)
  bool valid = false;
};

// Number of dense samples for the extrude polyline representation of a fitted spline
// (fixed, curvature-independent — a modest count that the mesher's deflection grid
// refines further along the depth; kept small to bound the face count).
inline constexpr int kSplinePolySamples = 24;

// Build a clamped B-spline from a stream of (x,y) pairs at z=0 (the profile plane).
// Requires ≥ 2 points; degree = min(3, nPoints-1). Deduplicates coincident
// consecutive points so the curve is non-degenerate.
inline SplineCurve fitSplineXY(const double* xy, int offset, int count) {
  SplineCurve c;
  if (xy == nullptr || count < 2) return c;
  std::vector<Point3> pts;
  pts.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const Point3 p{xy[(offset + i) * 2], xy[(offset + i) * 2 + 1], 0.0};
    if (!pts.empty() && math::distance(pts.back(), p) < kProfileTol) continue;
    pts.push_back(p);
  }
  if (pts.size() < 2) return c;
  c.degree = std::min(3, static_cast<int>(pts.size()) - 1);
  // Solve for INTERPOLATING poles (A9.1) so the curve PASSES THROUGH the input
  // points — matching OCCT's GeomAPI_PointsToBSpline. A degenerate chord length or
  // singular system falls back to the control-polygon representation (still a valid,
  // monotone curve, just not through the interior points).
  const std::vector<double> up = chordParams(pts);
  if (!up.empty()) {
    const std::vector<double> U = averagedKnots(up, c.degree);
    const std::vector<Point3> poles = interpolatePoles(pts, up, U, c.degree);
    if (poles.size() == pts.size()) {
      c.poles = poles;
      c.knots = U;
    }
  }
  if (c.poles.empty()) {
    c.poles = pts;
    c.knots = clampedKnots(static_cast<int>(c.poles.size()), c.degree);
  }
  // Dense polyline of the true curve for the watertight extrude representation.
  c.sampled.reserve(kSplinePolySamples + 1);
  for (int i = 0; i <= kSplinePolySamples; ++i) {
    const double u = static_cast<double>(i) / kSplinePolySamples;
    c.sampled.push_back(math::curvePoint(c.degree, {c.poles.data(), c.poles.size()},
                                         {c.knots.data(), c.knots.size()}, u));
  }
  c.valid = true;
  return c;
}

// ── EXTRUDE spline WALL (single B-spline side face per spline segment) ─────────
// A spline segment extrudes into ONE B-spline side FACE (matching OCCT's single
// spline face → face-count parity), sharing its bottom/top spline RIM edges with the
// two caps so the mesher welds them watertight. The wall surface is a degree-(p,1)
// B-spline: the U direction is the fitted profile spline (poles Pi at z=0), the V
// direction is a straight extrude in +Z (two rows: Pi at z=0, Pi at z=depth). Because
// the cap's spline boundary EDGE and the wall's bottom RIM edge carry the SAME 3D
// EdgeCurve (identical poles/knots/degree), edge_mesher discretizes them identically
// and the two faces place the SAME boundary points ⇒ watertight (the two-stage shared-
// 1D-discretization contract, now honoured for a curved shared edge — the same weld the
// arc/cylinder wall already relies on). A straight (degree-1 / collinear) spline is
// still a valid B-spline wall (one flat quad-like patch).
//
// A spline 3D EdgeCurve at height z (poles lifted to z), clamped over its knot domain.
inline topo::EdgeCurve splineEdgeCurve(const SplineCurve& sc, double z) {
  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::BSpline;
  c.degree = sc.degree;
  c.poles.reserve(sc.poles.size());
  for (const Point3& p : sc.poles) c.poles.push_back(Point3{p.x, p.y, z});
  c.knots = sc.knots;
  return c;
}

// The B-spline side wall FaceSurface for a spline segment extruded to `depth`. Row-
// major, U outer (nPolesU = spline pole count, degreeU = spline degree), V inner
// (nPolesV = 2, degreeV = 1): row 0 at z=0, row 1 at z=depth.
inline topo::FaceSurface splineWallSurface(const SplineCurve& sc, double depth) {
  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::BSpline;
  s.degreeU = sc.degree;
  s.degreeV = 1;
  s.nPolesU = static_cast<int>(sc.poles.size());
  s.nPolesV = 2;
  s.poles.reserve(sc.poles.size() * 2);
  for (const Point3& p : sc.poles) {
    s.poles.push_back(Point3{p.x, p.y, 0.0});
    s.poles.push_back(Point3{p.x, p.y, depth});
  }
  s.knotsU = sc.knots;
  s.knotsV = {0.0, 0.0, 1.0, 1.0};
  return s;
}

// Build the single B-spline side FACE for a spline segment extruded to `depth`, sharing
// its bottom/top rim vertices (bi/bj at z=0, ti/tj at z=depth) with the caps. Boundary:
// bottom rim (v=0) → right seam (u=u1) → top rim reversed (v=1) → left seam reversed
// (u=u0). Rims carry a LINE pcurve (u,vEnd) — the edge param IS the surface U, so a
// B-spline pcurve would break the S(pcurve)=C_edge weld contract; seams carry a LINE
// pcurve (uEnd, t/len) mapping the seam arc length to v∈[0,1]. Orientation is chosen so
// the effective surface normal points outward (curve tangent × +Z vs the radial).
inline topo::Shape splineWallFace(const SplineCurve& sc, const topo::Shape& bi,
                                  const topo::Shape& bj, const topo::Shape& ti,
                                  const topo::Shape& tj, double depth) {
  const topo::FaceSurface surf = splineWallSurface(sc, depth);
  const double u0 = sc.knots.front(), u1 = sc.knots.back();
  auto rim = [&](double z, const topo::Shape& va, const topo::Shape& vb, double vEnd) -> topo::Shape {
    const topo::Shape e = topo::ShapeBuilder::makeEdge(splineEdgeCurve(sc, z), u0, u1, va, vb);
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = Point3{u0, vEnd, 0.0};
    pc.dir2d = math::Vec3{1.0, 0.0, 0.0};  // u = edge parameter, v = vEnd
    return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
  };
  auto seam = [&](const topo::Shape& va, const topo::Shape& vb, double uEnd) -> topo::Shape {
    const Point3 a = *topo::pointOf(va), b = *topo::pointOf(vb);
    const double len = math::distance(a, b);
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame = math::Ax3{a, math::Dir3{b - a}, math::Dir3{0, 1, 0}, math::Dir3{0, 0, 1}};
    const topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, len, va, vb);
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = Point3{uEnd, 0.0, 0.0};
    pc.dir2d = math::Vec3{0.0, len > kProfileTol ? 1.0 / len : 1.0, 0.0};
    return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
  };
  std::vector<topo::Shape> wireEdges{rim(0.0, bi, bj, 0.0), seam(bj, tj, u1),
                                     rim(depth, ti, tj, 1.0).oriented(topo::Orientation::Reversed),
                                     seam(bi, ti, u0).oriented(topo::Orientation::Reversed)};
  const double umid = 0.5 * (u0 + u1);
  const Point3 mid =
      math::curvePoint(sc.degree, {sc.poles.data(), sc.poles.size()}, {sc.knots.data(), sc.knots.size()}, umid);
  const math::Vec3 tang =
      math::curvePoint(sc.degree, {sc.poles.data(), sc.poles.size()}, {sc.knots.data(), sc.knots.size()},
                       umid + 1e-4) - mid;
  const math::Vec3 nat = math::cross(tang, math::Vec3{0, 0, 1});  // outward for CCW loop
  const math::Vec3 outward{mid.x, mid.y, 0.0};                    // radial from origin
  const topo::Orientation o =
      math::dot(nat, outward) < 0.0 ? topo::Orientation::Reversed : topo::Orientation::Forward;
  return topo::ShapeBuilder::makeFace(surf, topo::ShapeBuilder::makeWire(std::move(wireEdges)), {}, o);
}

// ── 2D closed-polyline self-intersection test (profile validity guard) ────────
// Two NON-adjacent segments of the closed boundary polyline crossing means the
// profile self-intersects (a figure-eight spline, a folded outline) — not a valid
// simple region to extrude/revolve. Adjacent segments (sharing an endpoint) are
// skipped. Proper crossings only (shared endpoints do not count). O(m²) over the
// dense boundary sample count m (bounded, a few hundred).
inline bool segmentsProperlyCross(const Point3& a, const Point3& b, const Point3& c,
                                  const Point3& d) noexcept {
  auto orient = [](const Point3& p, const Point3& q, const Point3& r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
  };
  const double o1 = orient(a, b, c), o2 = orient(a, b, d);
  const double o3 = orient(c, d, a), o4 = orient(c, d, b);
  // Strict opposite signs on BOTH → a proper interior crossing (endpoints touching,
  // i.e. one orientation ≈ 0, is NOT a crossing — that is a shared vertex).
  return ((o1 > kProfileTol && o2 < -kProfileTol) || (o1 < -kProfileTol && o2 > kProfileTol)) &&
         ((o3 > kProfileTol && o4 < -kProfileTol) || (o3 < -kProfileTol && o4 > kProfileTol));
}

inline bool polylineSelfIntersects(const std::vector<Point3>& poly) noexcept {
  const std::size_t m = poly.size();
  if (m < 4) return false;
  for (std::size_t i = 0; i + 1 < m; ++i)
    for (std::size_t j = i + 2; j + 1 < m; ++j) {
      if (i == 0 && j + 1 == m) continue;  // first & last share the closing vertex
      if (segmentsProperlyCross(poly[i], poly[i + 1], poly[j], poly[j + 1])) return true;
    }
  return false;
}

}  // namespace residual_detail

// ─────────────────────────────────────────────────────────────────────────────
// build_prism_profile_spline_walls — extrude a mixed line/arc/SPLINE closed outer
// profile (≥2 segments, no holes) with each SPLINE segment as ONE true B-spline side
// FACE (face-count parity with OCCT). Returns NULL for the single-segment / holed /
// stray-circle cases the caller then handles via the polyline path. Self-verified by
// the engine. This is the preferred representation (matches the OCCT face count).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_prism_profile_spline_walls(
    const std::vector<ProfileSegment>& segs, const double* splineXY, int splineN,
    const std::vector<CircleHole>& circleHoles,
    const std::vector<std::vector<math::Point3>>& polyHoles, double depth) {
  using namespace residual_detail;
  if (!(depth > kMinDepth)) return {};
  // Only the outer-only, ≥2-segment, no-stray-circle case is assembled here.
  if (!circleHoles.empty() || !polyHoles.empty()) return {};
  for (const ProfileSegment& s : segs)
    if (s.kind == 2) return {};
  if (segs.size() < 2) return {};

  // ── Assemble the mixed outer profile as: 2 trimmed planar caps + 1 side FACE per
  // segment (line → Plane quad, arc → Cylinder patch, spline → ONE B-spline wall).
  // Each spline segment therefore contributes exactly ONE side face (face-count parity
  // with OCCT's single spline face), and every side face shares its bottom/top boundary
  // EDGE with the caps (same 3D EdgeCurve ⇒ the mesher discretizes it once and both
  // faces place the identical boundary points ⇒ watertight). ──────────────────────

  // Per-segment resolved geometry: ordered edge endpoints (z=0) + the fitted spline
  // (kind 3). We build one BSpline curve per spline segment, reused for its edges.
  struct SegGeom {
    int kind = 0;                    // 0 line, 1 arc, 3 spline
    Point3 p0{}, p1{};               // z=0 endpoints (segment start/end)
    double cx = 0, cy = 0, r = 0, a0 = 0, a1 = 0;  // arc
    SplineCurve sc;                  // spline (kind 3)
  };
  std::vector<SegGeom> geom;
  geom.reserve(segs.size());
  for (const ProfileSegment& s : segs) {
    SegGeom g;
    g.kind = s.kind;
    if (s.kind == 0) {
      g.p0 = Point3{s.x0, s.y0, 0.0};
      g.p1 = Point3{s.x1, s.y1, 0.0};
    } else if (s.kind == 1) {
      if (!(s.r > kProfileTol)) return {};
      g.cx = s.cx; g.cy = s.cy; g.r = s.r; g.a0 = s.a0; g.a1 = s.a1;
      g.p0 = Point3{s.cx + s.r * std::cos(s.a0), s.cy + s.r * std::sin(s.a0), 0.0};
      g.p1 = Point3{s.cx + s.r * std::cos(s.a1), s.cy + s.r * std::sin(s.a1), 0.0};
    } else if (s.kind == 3) {
      if (s.ptCount < 2 || s.ptOffset < 0 || (s.ptOffset + s.ptCount) * 2 > splineN) return {};
      g.sc = fitSplineXY(splineXY, s.ptOffset, s.ptCount);
      if (!g.sc.valid) return {};
      g.p0 = g.sc.poles.front();  // clamped B-spline interpolates its endpoints
      g.p1 = g.sc.poles.back();
    } else {
      return {};
    }
    geom.push_back(std::move(g));
  }
  const std::size_t n = geom.size();
  if (n < 2) return {};

  // Reject a SELF-INTERSECTING profile (a figure-eight spline, a folded outline): the
  // extruded wall would self-cross — a valid solid needs surface-surface intersection
  // (Tier 4), so DECLINE (→ NULL → the caller / engine falls through to OCCT, which also
  // rejects it as invalid; never a faked self-intersecting native solid). The boundary is
  // densely sampled (splines/arcs to their sampled polyline, lines to their endpoints).
  {
    std::vector<Point3> boundary;
    for (const SegGeom& g : geom) {
      if (g.kind == 3) {
        for (const Point3& p : g.sc.sampled) boundary.push_back(Point3{p.x, p.y, 0.0});
      } else if (g.kind == 1) {
        const double span = g.a1 - g.a0;
        const int ns = std::clamp(static_cast<int>(std::ceil(std::fabs(span) / (kFullTurn / 32.0))),
                                  2, 64);
        for (int k = 0; k < ns; ++k) {
          const double a = g.a0 + span * (static_cast<double>(k) / ns);
          boundary.push_back(Point3{g.cx + g.r * std::cos(a), g.cy + g.r * std::sin(a), 0.0});
        }
      } else {
        boundary.push_back(g.p0);
      }
    }
    if (!boundary.empty()) boundary.push_back(boundary.front());  // close the loop
    if (polylineSelfIntersects(boundary)) return {};
  }

  // Winding: signed area of the endpoint polygon (arcs/splines approximated by their
  // chords — sufficient for orientation). Force CCW so caps/walls face outward.
  double area2 = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const Point3& a = geom[i].p0;
    const Point3& b = geom[(i + 1) % n].p0;
    area2 += a.x * b.y - b.x * a.y;
  }
  if (area2 < 0.0) {
    std::reverse(geom.begin(), geom.end());
    for (SegGeom& g : geom) {
      std::swap(g.p0, g.p1);
      if (g.kind == 1) std::swap(g.a0, g.a1);
      if (g.kind == 3) { std::reverse(g.sc.poles.begin(), g.sc.poles.end());
                         for (double& k : g.sc.knots) k = 1.0 - k;
                         std::reverse(g.sc.knots.begin(), g.sc.knots.end()); }
    }
  }

  // Junction vertices (segment starts), shared by both caps and the side walls.
  std::vector<topo::Shape> bot(n), top(n);
  for (std::size_t i = 0; i < n; ++i) {
    bot[i] = topo::ShapeBuilder::makeVertex(Point3{geom[i].p0.x, geom[i].p0.y, 0.0});
    top[i] = topo::ShapeBuilder::makeVertex(Point3{geom[i].p0.x, geom[i].p0.y, depth});
  }
  const math::Ax3 botFrame{Point3{0, 0, 0}, math::Dir3{1, 0, 0}, math::Dir3{0, 1, 0}, math::Dir3{0, 0, 1}};
  const math::Ax3 topFrame{Point3{0, 0, depth}, math::Dir3{1, 0, 0}, math::Dir3{0, 1, 0}, math::Dir3{0, 0, 1}};

  // One cap-boundary edge for segment i at height z, sharing vertices v0/v1. Line →
  // Line edge + Line pcurve (u,v)=(x,y); arc → Circle edge + Circle pcurve; spline →
  // BSpline edge + BSpline pcurve (2D poles = spline (x,y), same knots/degree — so the
  // planar cap surface evaluated at the pcurve equals the 3D edge point).
  auto capEdge = [&](std::size_t i, double z, const math::Ax3& frame, const topo::Shape& v0,
                     const topo::Shape& v1) -> topo::Shape {
    const SegGeom& g = geom[i];
    if (g.kind == 0) {
      const double len = math::distance(g.p0, g.p1);
      topo::EdgeCurve c;
      c.kind = topo::EdgeCurve::Kind::Line;
      c.frame = math::Ax3{Point3{g.p0.x, g.p0.y, z}, math::Dir3{g.p1 - g.p0}, math::Dir3{0, 1, 0},
                          math::Dir3{0, 0, 1}};
      const topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, len, v0, v1);
      // Line pcurve: origin (x,y) on the plane, UNIT per-parameter step (edge param runs
      // [0,len]) so origin2d + dir2d·len = (p1.x,p1.y) — matches construct.h planarEdge.
      topo::PCurve pc;
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = Point3{g.p0.x, g.p0.y, 0.0};
      pc.dir2d = len > kProfileTol ? (g.p1 - g.p0) / len : math::Vec3{1, 0, 0};
      return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
    }
    if (g.kind == 1) {
      topo::EdgeCurve c;
      c.kind = topo::EdgeCurve::Kind::Circle;
      c.frame = math::Ax3{Point3{g.cx, g.cy, z}, math::Dir3{1, 0, 0}, math::Dir3{0, 1, 0},
                          math::Dir3{0, 0, 1}};
      c.radius = g.r;
      const topo::Shape e = topo::ShapeBuilder::makeEdge(c, g.a0, g.a1, v0, v1);
      topo::PCurve pc;
      pc.kind = topo::EdgeCurve::Kind::Circle;
      pc.origin2d = Point3{g.cx, g.cy, 0.0};
      pc.dir2d = math::Vec3{g.r, 0.0, 0.0};
      return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
    }
    // spline
    const topo::EdgeCurve c = splineEdgeCurve(g.sc, z);
    const topo::Shape e = topo::ShapeBuilder::makeEdge(c, g.sc.knots.front(), g.sc.knots.back(), v0, v1);
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::BSpline;
    pc.degree = g.sc.degree;
    pc.knots = g.sc.knots;
    for (const Point3& p : g.sc.poles) pc.poles2d.push_back(Point3{p.x, p.y, 0.0});
    pc.origin2d = Point3{g.sc.poles.front().x, g.sc.poles.front().y, 0.0};
    return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
  };

  // Cap faces (trimmed planar). Both caps keep the SAME CCW edge order (each edge from
  // its segment-start vertex to the next); the plane frame normal is +Z for both, so the
  // BOTTOM cap is flagged Reversed (effective outward normal −Z) and the TOP cap Forward
  // (+Z). Keeping the edge/curve direction consistent (start→end) avoids a vertex/curve
  // direction mismatch that would desync the shared boundary discretization.
  auto capFace = [&](double z, const math::Ax3& frame, const std::vector<topo::Shape>& ring,
                     topo::Orientation faceOrient) -> topo::Shape {
    std::vector<topo::Shape> edges;
    edges.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
      edges.push_back(capEdge(i, z, frame, ring[i], ring[(i + 1) % n]));
    topo::FaceSurface s;
    s.kind = topo::FaceSurface::Kind::Plane;
    s.frame = frame;
    return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(std::move(edges)), {},
                                        faceOrient);
  };
  std::vector<topo::Shape> faces;
  faces.reserve(n + 2);
  faces.push_back(capFace(0.0, botFrame, bot, topo::Orientation::Reversed));
  faces.push_back(capFace(depth, topFrame, top, topo::Orientation::Forward));

  // Side walls, one FACE per segment. Reuse the profile.h arc-wall helper for arcs;
  // build a Plane quad for lines and ONE B-spline wall for splines.
  constexpr double kMaxSpan = kFullTurn / 2.0;  // 180° per cylinder patch (matches profile.h)
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    const SegGeom& g = geom[i];
    if (g.kind == 0) {
      const math::Vec3 edge = geom[j].p0 - g.p0;
      const math::Vec3 nrm = math::cross(edge, math::Vec3{0, 0, 1});
      std::vector<topo::Shape> quad{bot[i], bot[j], top[j], top[i]};
      faces.push_back(detail::planarFace(quad, math::Dir3{nrm}, topo::Orientation::Forward));
    } else if (g.kind == 1) {
      detail::TypedEdge te;
      te.isArc = true; te.cx = g.cx; te.cy = g.cy; te.r = g.r; te.a0 = g.a0; te.a1 = g.a1;
      te.p0 = g.p0; te.p1 = g.p1;
      const double span = std::fabs(g.a1 - g.a0);
      const int nSpan = std::max(1, static_cast<int>(std::ceil(span / kMaxSpan - 1e-9)));
      for (int k = 0; k < nSpan; ++k) {
        const double s0 = g.a0 + (g.a1 - g.a0) * (static_cast<double>(k) / nSpan);
        const double s1 = g.a0 + (g.a1 - g.a0) * (static_cast<double>(k + 1) / nSpan);
        faces.push_back(detail::arcWallPatch(te, s0, s1, depth, /*outwardCcw=*/true));
      }
    } else {  // spline → ONE B-spline wall face (shares its rim edges with the caps)
      faces.push_back(splineWallFace(g.sc, bot[i], bot[j], top[i], top[j], depth));
    }
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// build_prism_profile_spline — POLYLINE fallback for a spline-bearing profile the
// B-spline-wall path declines (a single open spline segment, a holed profile). Each
// spline expands into a run of straight LINE segments through the fitted curve's dense
// polyline and routes through profile.h's watertight typed-extrude (straight-edge weld,
// no free-form face). More faces than OCCT (representational), but always watertight.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_prism_profile_spline_polyline(
    const std::vector<ProfileSegment>& segs, const double* splineXY, int splineN,
    const std::vector<CircleHole>& circleHoles,
    const std::vector<std::vector<math::Point3>>& polyHoles, double depth) {
  using namespace residual_detail;
  std::vector<ProfileSegment> expanded;
  expanded.reserve(segs.size() * 4);
  for (const ProfileSegment& s : segs) {
    if (s.kind != 3) { expanded.push_back(s); continue; }
    if (s.ptCount < 2 || s.ptOffset < 0 || (s.ptOffset + s.ptCount) * 2 > splineN) return {};
    const SplineCurve sc = fitSplineXY(splineXY, s.ptOffset, s.ptCount);
    if (!sc.valid) return {};
    // Drop a point COLLINEAR with its neighbours (a straight run collapses to its
    // endpoints — avoids zero-area cap slivers that break watertightness).
    std::vector<Point3> pl;
    pl.reserve(sc.sampled.size());
    for (const Point3& p : sc.sampled) {
      if (pl.size() >= 2) {
        const Point3& a = pl[pl.size() - 2];
        const Point3& b = pl[pl.size() - 1];
        const double cross2d = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if (std::fabs(cross2d) < kProfileTol) { pl.back() = p; continue; }
      }
      pl.push_back(p);
    }
    for (std::size_t k = 0; k + 1 < pl.size(); ++k) {
      ProfileSegment ls;
      ls.kind = 0;
      ls.x0 = pl[k].x;     ls.y0 = pl[k].y;
      ls.x1 = pl[k + 1].x; ls.y1 = pl[k + 1].y;
      expanded.push_back(ls);
    }
  }
  // Reject a SELF-INTERSECTING expanded outline (figure-eight spline) — the extruded
  // wall would self-cross (Tier-4 SSI), so DECLINE → OCCT (which also rejects it).
  {
    std::vector<Point3> boundary;
    boundary.reserve(expanded.size() + 1);
    for (const ProfileSegment& e : expanded)
      if (e.kind == 0) boundary.push_back(Point3{e.x0, e.y0, 0.0});
    if (!boundary.empty()) boundary.push_back(boundary.front());
    if (polylineSelfIntersects(boundary)) return {};
  }
  return build_prism_profile(expanded, circleHoles, polyHoles, depth);
}

// ─────────────────────────────────────────────────────────────────────────────
// PUBLIC ENTRY — build_prism_profile_spline
//
// Extrude a typed profile that MAY contain kind-3 SPLINE segments (as well as
// line/arc). A line/arc-only profile delegates to profile.h's typed path. A
// spline-bearing CLOSED outer profile (≥2 segments, no holes) is built with each
// spline as ONE true B-spline side FACE (build_prism_profile_spline_walls — face-count
// parity with OCCT). Anything that path declines (a single open spline, a holed
// spline profile) falls back to the watertight POLYLINE expansion. The engine self-
// verifies the mesh and forwards to OCCT if it is not robustly watertight.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_prism_profile_spline(const std::vector<ProfileSegment>& segs,
                                              const double* splineXY, int splineN,
                                              const std::vector<CircleHole>& circleHoles,
                                              const std::vector<std::vector<math::Point3>>& polyHoles,
                                              double depth) {
  using namespace residual_detail;
  if (!(depth > kMinDepth)) return {};
  bool hasSpline = false;
  for (const ProfileSegment& s : segs)
    if (s.kind == 3) hasSpline = true;
  if (!hasSpline) return build_prism_profile(segs, circleHoles, polyHoles, depth);

  // Preferred: one true B-spline side face per spline segment (OCCT face-count parity).
  const topo::Shape walls =
      build_prism_profile_spline_walls(segs, splineXY, splineN, circleHoles, polyHoles, depth);
  if (!walls.isNull()) return walls;
  // Fallback: the watertight polyline expansion (single open spline / holed profile).
  return build_prism_profile_spline_polyline(segs, splineXY, splineN, circleHoles, polyHoles, depth);
}

// ─────────────────────────────────────────────────────────────────────────────
// TORUS / general surface-of-revolution helpers (revolve residual).
// ─────────────────────────────────────────────────────────────────────────────
namespace residual_detail {

// A revolved-generatrix band emitted as an EXACT rational B-spline surface. The
// generatrix (a meridian in the axis (r,h) half-plane) is sampled to poles along
// V; the angular direction U over [t0,t1] is an EXACT rational-quadratic arc of a
// circle (control triple {P0, weightedMid, P2}, weight cos(Δ/2)). The tensor
// product is a rational B-spline surface (degreeU=2 rational, degreeV = generatrix
// degree). Tessellated via the BSpline face kind (nurbsSurfacePoint). Splitting the
// full turn into ≤ 120° spans keeps each rational arc well-conditioned (weight > 0).
//
// `merid` are the 3D points of the generatrix at θ=0 (world), degreeV/meridKnots
// its V-parametrization. This routine rotates each generatrix pole about the axis
// to build the 3-column (U) rational net for the span [t0,t1].

// Build the rational-B-spline surface-of-revolution band for one angular span.
// nV = #generatrix poles. degreeV = generatrix degree. The U (angular) direction is
// a single rational-quadratic Bézier segment (3 poles, clamped knots {0,0,0,1,1,1}).
inline topo::FaceSurface revSurfaceBand(const detail::AxisFrame& f,
                                        const std::vector<Point3>& meridPoles, int degreeV,
                                        const std::vector<double>& meridKnots, double t0,
                                        double t1) {
  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::BSpline;
  s.degreeU = 2;
  s.degreeV = degreeV;
  s.nPolesU = 3;
  s.nPolesV = static_cast<int>(meridPoles.size());

  const double half = 0.5 * (t1 - t0);
  const double w = std::cos(half);  // rational-quadratic circle middle weight
  const double tmid = 0.5 * (t0 + t1);
  // Row-major, U outer (3 columns), V inner (nV). For each U index (0,1,2) we place
  // the generatrix rotated to θ = t0 / tmid / t1. The MIDDLE column must be the
  // rational-arc control point = rotated point at tmid scaled by 1/w about the axis
  // (so that the projected rational curve is the exact circle).
  s.poles.reserve(static_cast<std::size_t>(3) * meridPoles.size());
  s.weights.reserve(static_cast<std::size_t>(3) * meridPoles.size());
  // We store poles row-major as U-outer: pole(uIdx, vIdx). But the grid layout in
  // FaceSurface is poles[i*nCols + j] with i over U (nRows=nPolesU), j over V. So we
  // iterate uIdx outer, vIdx inner.
  for (int uIdx = 0; uIdx < 3; ++uIdx) {
    for (const Point3& mp : meridPoles) {
      double r, h;
      f.decompose(mp, r, h);
      if (uIdx == 0) {
        s.poles.push_back(detail::revolved(f, r, h, t0));
        s.weights.push_back(1.0);
      } else if (uIdx == 2) {
        s.poles.push_back(detail::revolved(f, r, h, t1));
        s.weights.push_back(1.0);
      } else {
        // Middle control point of the rational-quadratic arc: on the tangent-line
        // intersection, at radius r / cos(half) from the axis at angle tmid.
        const double rMid = w > 1e-9 ? r / w : r;
        s.poles.push_back(detail::revolved(f, rMid, h, tmid));
        s.weights.push_back(w);
      }
    }
  }
  s.knotsU = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};  // one clamped quadratic Bézier segment
  s.knotsV = meridKnots;
  return s;
}

}  // namespace residual_detail

// ─────────────────────────────────────────────────────────────────────────────
// PUBLIC ENTRY — build_revolution_profile_spline
//
// Revolve a typed profile that MAY contain a kind-3 SPLINE meridian and/or an
// OFF-AXIS circular arc (a TORUS band), a FULL 2π turn only. A profile with NO
// residual segment (line / on-axis arc → Sphere) is delegated to profile.h::
// build_revolution_profile (already native); this entry ADDS the residual cases.
// Every segment (line / arc / spline) is emitted UNIFORMLY as rational-B-spline
// surface-of-revolution bands so adjacent segments' shared angular rim circles weld
// band↔band (a mixed band/analytic-face representation left those rims open).
//
// HONEST DEFERRALS → NULL (OCCT fallback, verified, never faked):
//   * a PARTIAL turn (< 2π) — the band↔planar-meridian-cap curved seam is not
//     robustly weldable here (see the guard below);
//   * a spindle torus (arc centre distance rc ≤ arc radius R — the arc would cross
//     the axis) or any generatrix that dips to r ≤ 0 mid-span (a self-intersecting
//     surface of revolution — Tier-4 SSI territory);
//   * a stray full-circle (kind-2) mid-loop segment.
// The engine additionally self-verifies the meshed result (watertight + volume) and
// discards it to OCCT if it is not a valid watertight solid.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_revolution_profile_spline(const std::vector<ProfileSegment>& segs,
                                                   const double* splineXY, int splineN,
                                                   const RevolveAxis& axisIn, double angle) {
  using namespace residual_detail;
  if (segs.empty() || !(angle > kMinDepth)) return {};

  // If no residual (spline / off-axis arc) segment is present, delegate to the
  // existing native typed revolve (line / on-axis arc → Sphere).
  bool needsResidual = false;
  const math::Dir3 adirCheck{axisIn.adx, axisIn.ady, 0.0};
  detail::AxisFrame fchk;
  if (adirCheck.valid()) {
    fchk.origin = detail::xy(axisIn.ax, axisIn.ay, 0.0);
    fchk.z = adirCheck;
    fchk.x = math::Dir3{Vec3{axisIn.ady, -axisIn.adx, 0.0}};
    fchk.y = math::Dir3{math::cross(fchk.z.vec(), fchk.x.vec())};
  }
  for (const ProfileSegment& s : segs) {
    if (s.kind == 3) needsResidual = true;
    if (s.kind == 1) {  // arc: off-axis (r_centre > 0) is a torus → residual
      double rc, hc;
      detail::rh(fchk, s.cx, s.cy, rc, hc);
      if (rc > kProfileTol) needsResidual = true;
    }
  }
  if (!needsResidual) return build_revolution_profile(segs, axisIn, angle);
  if (!adirCheck.valid()) return {};

  const detail::AxisFrame& f = fchk;
  const bool full = angle >= kFullTurn - 1e-9;
  const double sweep = full ? kFullTurn : angle;

  // HONEST DEFERRAL — a PARTIAL-turn residual revolve needs two planar MERIDIAN end
  // caps that weld to the RATIONAL bands along the (curved) generatrix meridian rim.
  // That band(BSpline)↔planar-cap curved seam does not robustly weld here without
  // threading the shared meridian-rim edge nodes into the cap wires (the way a curved
  // shared edge welds only by NODE identity, not by two independent discretizations).
  // Rather than emit a leaky solid, a partial-turn spline/torus revolve FALLS THROUGH
  // to OCCT (verified, never faked). The FULL-turn case is fully native (no caps).
  if (!full) return {};

  // Profile winding in (r,h) (endpoints of every segment) — matches profile.h.
  double area2 = 0.0;
  for (const ProfileSegment& s : segs) {
    double r0, h0, r1, h1;
    detail::rh(f, s.x0, s.y0, r0, h0);
    detail::rh(f, s.x1, s.y1, r1, h1);
    area2 += r0 * h1 - r1 * h0;
  }
  const double windingSign = area2 >= 0.0 ? 1.0 : -1.0;
  (void)windingSign;  // rational bands are oriented via natural normal below

  constexpr double kMaxSpan = kFullTurn / 3.0;  // 120° spans (well-conditioned arc)
  const int nSpan = std::max(1, static_cast<int>(std::ceil(sweep / kMaxSpan - 1e-9)));
  std::vector<double> stations(static_cast<std::size_t>(nSpan) + 1);
  for (int k = 0; k <= nSpan; ++k) stations[k] = sweep * k / nSpan;

  std::vector<topo::Shape> faces;

  // The band wire + orientation + emit helpers close over the axis frame `f`; they
  // are lambdas (not free functions) so the AxisFrame-typed helpers stay private to
  // this builder. `emitBands` (below) drives them, rejecting a self-folding meridian.
  auto bandWire = [&](const std::vector<Point3>& meridPoles, int degV,
                      const std::vector<double>& meridKnots, double t0, double t1) -> topo::Shape {
    // Corners: a0 = generatrix start @ t0, aN = generatrix end @ t0, then @ t1.
    const Point3 p0 = meridPoles.front(), pN = meridPoles.back();
    double r0, h0, rN, hN;
    f.decompose(p0, r0, h0);
    f.decompose(pN, rN, hN);
    const bool p0Axis = r0 < kProfileTol, pNAxis = rN < kProfileTol;
    const topo::Shape a0 = topo::ShapeBuilder::makeVertex(detail::revolved(f, r0, h0, t0));
    const topo::Shape aN = topo::ShapeBuilder::makeVertex(detail::revolved(f, rN, hN, t0));
    const topo::Shape b0 = p0Axis ? a0 : topo::ShapeBuilder::makeVertex(detail::revolved(f, r0, h0, t1));
    const topo::Shape bN = pNAxis ? aN : topo::ShapeBuilder::makeVertex(detail::revolved(f, rN, hN, t1));

    // Generatrix rim edges (BSpline in 3D) at θ=t0 and θ=t1, with a BSpline pcurve
    // (u=const along one U end, v runs the generatrix). Angular rim edges are Circle
    // arcs at the two generatrix ENDPOINTS (radius r0 / rN about the axis).
    auto meridEdge = [&](const topo::Shape& v0, const topo::Shape& v1, double theta,
                         double uEnd) -> topo::Shape {
      topo::EdgeCurve c;
      c.kind = topo::EdgeCurve::Kind::BSpline;
      c.degree = degV;
      c.poles.reserve(meridPoles.size());
      for (const Point3& mp : meridPoles) {
        double r, h;
        f.decompose(mp, r, h);
        c.poles.push_back(detail::revolved(f, r, h, theta));
      }
      c.knots = meridKnots;
      const topo::Shape e = topo::ShapeBuilder::makeEdge(c, meridKnots.front(), meridKnots.back(),
                                                         v0, v1);
      topo::PCurve pc;
      pc.kind = topo::EdgeCurve::Kind::BSpline;
      pc.degree = degV;
      const int nV = static_cast<int>(meridPoles.size());
      for (int i = 0; i < nV; ++i)
        pc.poles2d.push_back(Point3{uEnd, static_cast<double>(i) / std::max(1, nV - 1), 0.0});
      pc.knots = meridKnots;
      pc.origin2d = Point3{uEnd, 0.0, 0.0};
      return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
    };
    auto angularEdge = [&](const topo::Shape& v0, const topo::Shape& v1, double r, double h,
                           double s0, double s1, double vEnd) -> topo::Shape {
      // A true Circle arc about the axis at radius r, height h, over the RADIAN range
      // [s0,s1]. The band surface's U domain is [0,1] with u=0 at θ=t0, u=1 at θ=t1,
      // so the pcurve maps the true circle parameter t → u = (t − t0)/(t1 − t0). v is
      // constant (0 at the generatrix start rim, 1 at the end rim).
      topo::EdgeCurve c;
      c.kind = topo::EdgeCurve::Kind::Circle;
      c.frame = math::Ax3{f.origin + f.z.vec() * h, f.x, f.y, f.z};
      c.radius = r;
      const topo::Shape e = topo::ShapeBuilder::makeEdge(c, s0, s1, v0, v1);
      const double span = (t1 - t0) != 0.0 ? (t1 - t0) : 1.0;
      topo::PCurve pc;
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = Point3{-t0 / span, vEnd, 0.0};   // u(t=t0)=0
      pc.dir2d = Vec3{1.0 / span, 0.0, 0.0};         // u(t) = (t − t0)/(t1 − t0)
      return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
    };

    std::vector<topo::Shape> edges;
    // Wire: meridian @ t0 (u=0, forward) → angular rim at generatrix END (v=1) →
    // meridian @ t1 (u=1, reversed) → angular rim at generatrix START (v=0).
    edges.push_back(meridEdge(a0, aN, t0, 0.0));
    if (!pNAxis) edges.push_back(angularEdge(aN, bN, rN, hN, t0, t1, 1.0));
    edges.push_back(meridEdge(bN, b0, t1, 1.0).oriented(topo::Orientation::Reversed));
    if (!p0Axis) edges.push_back(angularEdge(b0, a0, r0, h0, t1, t0, 0.0));
    return topo::ShapeBuilder::makeWire(std::move(edges));
  };

  auto bandOrient = [&](const std::vector<Point3>& meridPoles, double t0, double t1) -> topo::Orientation {
    // Natural normal at the band centre vs material-outward radial direction.
    const std::size_t mid = meridPoles.size() / 2;
    double r, h;
    f.decompose(meridPoles[mid], r, h);
    const double tMid = 0.5 * (t0 + t1);
    // Generatrix tangent (dr,dh) at mid (finite difference along the pole list).
    const std::size_t a = mid == 0 ? 0 : mid - 1;
    const std::size_t b = mid + 1 < meridPoles.size() ? mid + 1 : mid;
    double ra, ha, rb, hb;
    f.decompose(meridPoles[a], ra, ha);
    f.decompose(meridPoles[b], rb, hb);
    const double dr = rb - ra, dh = hb - ha;
    const double nr = windingSign * dh, nh = windingSign * (-dr);
    const Vec3 outward = f.x.vec() * (nr * std::cos(tMid)) + f.y.vec() * (nr * std::sin(tMid)) +
                         f.z.vec() * nh;
    // Natural surface normal ≈ angular tangent (∂S/∂u) × generatrix tangent (∂S/∂v)
    // sampled at the band centre.
    const Vec3 angTan = f.x.vec() * (-r * std::sin(tMid)) + f.y.vec() * (r * std::cos(tMid));
    const Vec3 genTan = f.x.vec() * (dr * std::cos(tMid)) + f.y.vec() * (dr * std::sin(tMid)) +
                        f.z.vec() * dh;
    const Vec3 nat = math::cross(angTan, genTan);
    return math::dot(nat, outward) < 0.0 ? topo::Orientation::Reversed : topo::Orientation::Forward;
  };

  auto emitBands = [&](const std::vector<Point3>& meridPoles, int degV,
                       const std::vector<double>& meridKnots) -> bool {
    for (std::size_t i = 0; i < meridPoles.size(); ++i) {
      double r, h;
      f.decompose(meridPoles[i], r, h);
      const bool endpoint = (i == 0 || i + 1 == meridPoles.size());
      if (r < -kProfileTol) return false;
      if (r < kProfileTol && !endpoint) return false;  // interior on axis → self-fold
    }
    for (int k = 0; k < nSpan; ++k) {
      topo::FaceSurface s = revSurfaceBand(f, meridPoles, degV, meridKnots, stations[k],
                                           stations[k + 1]);
      faces.push_back(topo::ShapeBuilder::makeFace(
          s, bandWire(meridPoles, degV, meridKnots, stations[k], stations[k + 1]), {},
          bandOrient(meridPoles, stations[k], stations[k + 1])));
    }
    return true;
  };

  // ── Per-segment dispatch ──────────────────────────────────────────────────────
  //
  // UNIFORM RATIONAL-BAND STRATEGY. Every profile segment (line / arc / spline)
  // is converted to a GENERATRIX pole list + a V-parametrization, then emitted as
  // rational B-spline bands via emitBands. Emitting ALL segments the SAME way is
  // what makes the shared angular RIM circles between adjacent segments WELD
  // watertight: two BSpline bands both place their rim vertices by evaluating their
  // own surface at the SAME shared edge fractions (verified band↔band, e.g. the two
  // half-tube bands of a full torus). A mixed representation (a BSpline band abutting
  // an analytic Plane annulus / Cylinder / Sphere via a SEPARATE rim node) meshes the
  // two rims on different code paths (structured-grid vs ear-clip) and leaves the
  // seam OPEN — hence the uniform band treatment here (the analytic sphere/cone
  // classification remains available in profile.h for the pure-analytic revolve).
  auto lineGeneratrix = [&](const ProfileSegment& s, std::vector<Point3>& merid, int& degV,
                            std::vector<double>& mk) {
    merid = {Point3{s.x0, s.y0, 0.0}, Point3{s.x1, s.y1, 0.0}};
    degV = 1;
    mk = {0.0, 0.0, 1.0, 1.0};
  };
  auto arcGeneratrix = [&](const ProfileSegment& s, std::vector<Point3>& merid, int& degV,
                           std::vector<double>& mk) {
    // Sample the arc into a generatrix pole list. The ANGULAR direction stays exact-
    // rational; the meridian arc is a modest curvature-sampled B-spline (deflection-
    // bounded, watertight). Enough poles to keep the sagitta small.
    std::vector<Point3> pts;
    const double aspan = std::fabs(s.a1 - s.a0);
    const int nm = std::clamp(static_cast<int>(std::ceil(aspan / (kFullTurn / 24.0))), 2, 64);
    pts.reserve(static_cast<std::size_t>(nm) + 1);
    for (int i = 0; i <= nm; ++i) {
      const double a = s.a0 + (s.a1 - s.a0) * (static_cast<double>(i) / nm);
      pts.push_back(Point3{s.cx + s.r * std::cos(a), s.cy + s.r * std::sin(a), 0.0});
    }
    degV = std::min(3, static_cast<int>(pts.size()) - 1);
    mk = clampedKnots(static_cast<int>(pts.size()), degV);
    merid = std::move(pts);
  };

  for (const ProfileSegment& s : segs) {
    std::vector<Point3> merid;
    int degV = 1;
    std::vector<double> mk;
    if (s.kind == 0) {  // line segment
      detail::RevPoint p, q;
      detail::rh(f, s.x0, s.y0, p.r, p.h);
      detail::rh(f, s.x1, s.y1, q.r, q.h);
      if (p.r < kProfileTol && q.r < kProfileTol) continue;  // segment on the axis
      lineGeneratrix(s, merid, degV, mk);
    } else if (s.kind == 1) {  // arc (on-axis OR off-axis)
      if (!(s.r > kProfileTol)) return {};
      double rc, hc;
      detail::rh(f, s.cx, s.cy, rc, hc);
      if (rc > kProfileTol && rc <= s.r) return {};  // off-axis arc dips through axis → OCCT
      arcGeneratrix(s, merid, degV, mk);
    } else if (s.kind == 3) {  // spline meridian
      if (s.ptCount < 2 || s.ptOffset < 0 || (s.ptOffset + s.ptCount) * 2 > splineN) return {};
      const SplineCurve sc = fitSplineXY(splineXY, s.ptOffset, s.ptCount);
      if (!sc.valid) return {};
      merid = sc.poles;
      degV = sc.degree;
      mk = sc.knots;
    } else {  // kind 2 stray full circle → defer
      return {};
    }
    if (!emitBands(merid, degV, mk)) return {};
  }

  if (faces.empty()) return {};
  // (Partial-turn end caps are handled by the early `if (!full) return {}` guard
  //  above — a partial residual revolve falls through to OCCT. Full turn needs none.)

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// build_torus — the BARE periodic Kind::Torus B-rep primitive.
//
// A native REVOLVE of an off-axis full circle builds RATIONAL B-SPLINE bands
// (build_revolution_torus above), whose faces carry Kind::BSpline — so the
// boolean's `recogniseCurvedSolid` (which HAS a Torus arm) never sees a true
// analytic torus and the torus∩{cyl/sphere/cone/torus} families defer to OCCT.
// This builder emits a torus as ONE doubly-periodic analytic Kind::Torus face
// with a NULL outer wire (the whole natural (u,v)∈[0,2π)² rectangle) — exactly
// the form `recogniseCurvedSolid` admits and the tessellator meshes watertight
// (both seams weld, no poles). It is the SAME shape the STEP reader maps a
// TOROIDAL_SURFACE to.
//
// Placement: a right-handed frame `pos` (origin = torus centre on the axis,
// z = revolution axis, x = the u=0 reference in the X–Y plane); (R,r) are the
// MAJOR (axis → tube-centre) and MINOR (tube) radii. A RING torus is required
// (R > r > 0); a spindle/degenerate torus (R ≤ r or r ≤ 0) returns a NULL Shape
// so the caller falls through to OCCT — never a self-intersecting body.
inline topo::Shape build_torus(double majorRadius, double minorRadius, const math::Ax3& pos) {
  if (!(minorRadius > 1e-9) || !(majorRadius > minorRadius + 1e-9)) return {};  // ring torus only
  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Torus;
  s.frame = pos;
  s.radius = majorRadius;
  s.minorRadius = minorRadius;
  const topo::Shape face = topo::ShapeBuilder::makeFace(s, topo::Shape{});
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell({face})});
}

// Convenience overload: a torus centred at `centre` about a unit `axis`.
inline topo::Shape build_torus(double majorRadius, double minorRadius,
                               const math::Point3& centre, const math::Dir3& axis) {
  // Pick any reference direction not parallel to the axis for the frame's x.
  const math::Vec3 a = axis.vec();
  math::Vec3 ref = std::fabs(a.z) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
  ref = ref - a * math::dot(ref, a);
  const double n = math::norm(ref);
  if (!(n > 1e-12)) return {};
  const math::Ax3 pos = math::Ax3::fromAxisAndRef(centre, axis, math::Dir3{ref * (1.0 / n)});
  return build_torus(majorRadius, minorRadius, pos);
}

}  // namespace cybercad::native::construct

#endif  // CYBERCAD_NATIVE_CONSTRUCT_RESIDUALS_H
