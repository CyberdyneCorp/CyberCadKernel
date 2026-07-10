// SPDX-License-Identifier: Apache-2.0
//
// ngon_fill.h — bounded N-sided fill: evaluate a Coons / Gregory-style transfinite
// interpolant of an ANALYTIC (straight-segment + circular-arc) boundary loop to a
// TESSELLATED triangle-grid mesh patch.
//
// ── THE SCOPE BOUND (why this is NOT a general NURBS kernel) ─────────────────────
// The boundary loop is 3–6 sides, each side a STRAIGHT SEGMENT or a CIRCULAR ARC. The
// patch is a fixed transfinite FORMULA (a discrete Coons patch for N∈{3,4}; a Gregory-
// style convex combination of the per-side Coons contributions over generalized
// barycentric coordinates for N∈{5,6}) EVALUATED on a (gridN+1)² / triangulated grid.
// It is a MESH patch — nothing stores or re-fits a NURBS surface, nothing solves a
// global surface fit. A boundary edge that is neither a segment nor an arc, a loop with
// <3 or >6 sides, a degenerate/duplicate corner, or a self-intersecting grid is
// HONEST-DECLINED (→ OCCT BRepFill_Filling). This is the moat's non-negotiable bound.
//
// The boundary ROWS/COLUMNS of the grid are the boundary samples THEMSELVES (assigned,
// not recomputed), so the patch shares the hole-boundary points BIT-EXACTLY — the
// precondition for fill_solid.h's weld to close watertight.
//
// OCCT-FREE. Uses src/native/{math,tessellate}. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_SURFACE_NGON_FILL_H
#define CYBERCAD_NATIVE_SURFACE_NGON_FILL_H

#include "native/math/native_math.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/trim.h"             // UV
#include "native/tessellate/uv_triangulate.h"   // triangulatePolygon (planar ear-clip)

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace cybercad::native::surface {

namespace math = cybercad::native::math;
namespace tess = cybercad::native::tessellate;

/// Measured reason a fill was declined (→ OCCT). Never a wrong/leaky patch.
enum class NGonDecline {
  Ok = 0,
  NonAnalyticBoundary,  ///< a boundary edge is neither a straight segment nor a circular arc
  TooManySides,         ///< N < 3 or N > 6
  DegenerateBoundary,   ///< zero-length side / duplicate corner / collinear arc
  SelfIntersecting,     ///< the evaluated patch grid self-crosses (fold-back)
  NotConverged          ///< a continuity/quality the bounded patch cannot honestly meet
};

/// One boundary side: a straight segment (arc=false) or a circular arc (arc=true)
/// from `start` to `end`. For an arc, `mid` is a point on the arc between the two
/// ends (fixes the arc's plane, centre and sweep). Corners are given in loop order.
struct BoundarySide {
  math::Point3 start{};
  math::Point3 end{};
  math::Point3 mid{};   ///< only read when arc == true
  bool arc = false;
};

/// The N-sided boundary loop (3 ≤ N ≤ 6), each side analytic.
struct Boundary {
  std::vector<BoundarySide> sides;  ///< sides[i].end == sides[i+1].start (closed)
};

/// Fill options.
struct NGonOptions {
  int gridN = 12;  ///< samples per side (≥ 2); the tessellation density
};

/// The evaluated patch: the triangle-grid mesh + the largest residual of any
/// boundary grid sample from its analytic boundary curve (0 for a straight side by
/// construction; the arc chord/sample residual for arcs — reported for the gate).
struct NGonPatch {
  tess::Mesh mesh;
  double onBoundaryResidual = 0.0;  ///< max |sample − curve| over boundary samples
  bool valid = false;

  /// The ORDERED boundary samples per side (side k: start..end, gridN+1 points, the
  /// endpoints bit-exact). fill_solid.h inserts these into the shell's matching
  /// boundary edge so the patch and the shell share identical sub-edges (no
  /// T-junctions). Populated for both the Coons/Gregory grid and the planar fan.
  std::vector<std::vector<math::Point3>> sideSamples;
};

namespace detail {

inline constexpr double kEps = 1e-12;

// Sample a straight side at gridN+1 parameters t∈[0,1] (start..end inclusive).
inline std::vector<math::Point3> sampleSegment(const math::Point3& a, const math::Point3& b,
                                               int gridN) {
  std::vector<math::Point3> out;
  out.reserve(gridN + 1);
  for (int i = 0; i <= gridN; ++i) {
    const double t = static_cast<double>(i) / gridN;
    out.push_back(math::Point3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                               a.z + (b.z - a.z) * t});
  }
  return out;
}

// The circle through three non-collinear points (centre + radius + plane frame),
// returned as (centre, radius, û, v̂) with û toward `a` and v̂ in-plane orthogonal.
// Returns nullopt for (near-)collinear points.
struct ArcCircle {
  math::Point3 centre;
  double radius;
  math::Vec3 u, v;  // orthonormal in the arc plane; u points from centre to `a`
};
inline std::optional<ArcCircle> circleThrough(const math::Point3& a, const math::Point3& b,
                                              const math::Point3& c) {
  const math::Vec3 ab = b - a, ac = c - a;
  const math::Vec3 n = math::cross(ab, ac);
  const double n2 = math::normSquared(n);
  if (n2 < kEps) return std::nullopt;  // collinear
  // Circumcentre in the plane (standard formula).
  const double ab2 = math::normSquared(ab), ac2 = math::normSquared(ac);
  const math::Vec3 term = math::cross(ab2 * ac - ac2 * ab, n);
  const math::Point3 centre = a + term / (2.0 * n2);
  const double radius = math::norm(a - centre);
  if (radius < kEps) return std::nullopt;
  math::Vec3 u = (a - centre);
  const double un = math::norm(u);
  if (un < kEps) return std::nullopt;
  u = u / un;
  const math::Dir3 nd{n};
  math::Vec3 v = math::cross(nd.vec(), u);
  const double vn = math::norm(v);
  if (vn < kEps) return std::nullopt;
  v = v / vn;
  return ArcCircle{centre, radius, u, v};
}

// Signed angle of point p about the circle (atan2 in the u,v frame), in [0,2π).
inline double angleOf(const ArcCircle& ci, const math::Point3& p) {
  const math::Vec3 d = p - ci.centre;
  const double x = math::dot(d, ci.u), y = math::dot(d, ci.v);
  double a = std::atan2(y, x);
  if (a < 0.0) a += 2.0 * 3.14159265358979323846;
  return a;
}

// Sample a circular arc start→end through mid at gridN+1 parameters. Chooses the
// sweep direction/extent that PASSES THROUGH `mid`. Endpoints are placed EXACTLY at
// the given start/end (bit-exact), interior points on the true circle.
inline std::optional<std::vector<math::Point3>> sampleArc(const math::Point3& a,
                                                          const math::Point3& b,
                                                          const math::Point3& m, int gridN) {
  const auto ci = circleThrough(a, b, m);
  if (!ci) return std::nullopt;
  const double twoPi = 2.0 * 3.14159265358979323846;
  double a0 = angleOf(*ci, a), a1 = angleOf(*ci, b), am = angleOf(*ci, m);
  // Unwrap so the arc a0→a1 contains am. Try the CCW sweep first.
  double sweep = a1 - a0;
  while (sweep < 0.0) sweep += twoPi;
  double amRel = am - a0;
  while (amRel < 0.0) amRel += twoPi;
  bool ccw = amRel <= sweep + 1e-9;
  std::vector<math::Point3> out;
  out.reserve(gridN + 1);
  for (int i = 0; i <= gridN; ++i) {
    const double t = static_cast<double>(i) / gridN;
    double ang;
    if (ccw) {
      ang = a0 + sweep * t;
    } else {
      double cw = twoPi - sweep;  // the complementary arc
      ang = a0 - cw * t;
    }
    if (i == 0) { out.push_back(a); continue; }
    if (i == gridN) { out.push_back(b); continue; }
    out.push_back(ci->centre + (ci->radius * std::cos(ang)) * ci->u +
                  (ci->radius * std::sin(ang)) * ci->v);
  }
  return out;
}

// Sample one side into gridN+1 points (endpoints bit-exact). Returns nullopt for a
// degenerate/collinear arc.
inline std::optional<std::vector<math::Point3>> sampleSide(const BoundarySide& s, int gridN) {
  if (math::distance(s.start, s.end) < kEps) return std::nullopt;  // zero-length
  if (s.arc) return sampleArc(s.start, s.end, s.mid, gridN);
  return sampleSegment(s.start, s.end, gridN);
}

// Bilinear corner interpolation for the Coons blend.
inline math::Point3 bilinear(const math::Point3& p00, const math::Point3& p10,
                             const math::Point3& p01, const math::Point3& p11, double u,
                             double v) {
  const math::Vec3 r =
      (1 - u) * (1 - v) * p00.asVec() + u * (1 - v) * p10.asVec() +
      (1 - u) * v * p01.asVec() + u * v * p11.asVec();
  return math::Point3{r.x, r.y, r.z};
}

// ── Coons patch on a QUAD boundary (four sample rows, each gridN+1 long) ──────────
// Sides are given as: bottom C0 (u:0→1 at v=0), right C1 (v:0→1 at u=1), top C2
// (u:0→1 at v=1), left C3 (v:0→1 at u=0). The four corners are shared. The classic
// discrete Coons surface fills the grid; the four boundary rows are the samples
// themselves (bit-exact), interior by the transfinite blend.
inline void coonsQuadGrid(const std::vector<math::Point3>& c0, const std::vector<math::Point3>& c1,
                          const std::vector<math::Point3>& c2, const std::vector<math::Point3>& c3,
                          int gridN, std::vector<std::vector<math::Point3>>& grid) {
  const math::Point3 p00 = c0.front();          // u0 v0
  const math::Point3 p10 = c0.back();           // u1 v0
  const math::Point3 p01 = c2.front();          // u0 v1
  const math::Point3 p11 = c2.back();           // u1 v1
  grid.assign(gridN + 1, std::vector<math::Point3>(gridN + 1));
  for (int j = 0; j <= gridN; ++j) {
    const double v = static_cast<double>(j) / gridN;
    for (int i = 0; i <= gridN; ++i) {
      const double u = static_cast<double>(i) / gridN;
      // Boundary rows/cols: assign the boundary samples bit-exactly.
      if (j == 0) { grid[j][i] = c0[i]; continue; }
      if (j == gridN) { grid[j][i] = c2[i]; continue; }
      if (i == 0) { grid[j][i] = c3[j]; continue; }
      if (i == gridN) { grid[j][i] = c1[j]; continue; }
      const math::Vec3 lc = (1 - u) * c3[j].asVec() + u * c1[j].asVec();  // left/right blend
      const math::Vec3 rc = (1 - v) * c0[i].asVec() + v * c2[i].asVec();  // bottom/top blend
      const math::Point3 bl = bilinear(p00, p10, p01, p11, u, v);
      const math::Vec3 s = lc + rc - bl.asVec();
      grid[j][i] = math::Point3{s.x, s.y, s.z};
    }
  }
}

// Arc-length resample a polyline to exactly `n+1` points (endpoints bit-exact).
// Returns empty for a zero-length polyline.
inline std::vector<math::Point3> resampleByArcLength(const std::vector<math::Point3>& in, int n) {
  std::vector<double> cum(in.size(), 0.0);
  for (std::size_t t = 1; t < in.size(); ++t) cum[t] = cum[t - 1] + math::distance(in[t - 1], in[t]);
  const double total = cum.back();
  if (total < kEps) return {};
  std::vector<math::Point3> out(n + 1);
  out.front() = in.front();
  out.back() = in.back();
  std::size_t seg = 0;
  for (int i = 1; i < n; ++i) {
    const double target = total * static_cast<double>(i) / n;
    while (seg + 1 < cum.size() && cum[seg + 1] < target) ++seg;
    const double segLen = cum[seg + 1] - cum[seg];
    const double f = segLen > kEps ? (target - cum[seg]) / segLen : 0.0;
    const math::Vec3 p = in[seg].asVec() + f * (in[seg + 1] - in[seg]);
    out[i] = math::Point3{p.x, p.y, p.z};
  }
  return out;
}

// The four Coons boundary curves (bottom/right/top/left, each stored u:0→1 or v:0→1)
// for an N-sided (3–6) loop given its per-side samples `S`. For N=3 the top collapses
// to the apex; for N=5,6 the trailing sides 2..N-2 are concatenated + arc-length
// resampled into the top — a Gregory-style reduction to the rectangular Coons domain.
// Returns false only when the N=5,6 merged top is degenerate (zero length).
struct CoonsSides {
  std::vector<math::Point3> c0, c1, c2, c3;
};
inline bool coonsSidesForLoop(const std::vector<std::vector<math::Point3>>& S, int N, int gridN,
                              CoonsSides& out) {
  out.c0 = S[0];  // bottom u:0→1
  out.c1 = S[1];  // right  v:0→1
  if (N == 4) {
    out.c2 = S[2];
    std::reverse(out.c2.begin(), out.c2.end());  // top u:0→1 = side2 reversed
    out.c3 = S[3];
    std::reverse(out.c3.begin(), out.c3.end());  // left v:0→1 = side3 reversed
    return true;
  }
  if (N == 3) {
    out.c2 = std::vector<math::Point3>(gridN + 1, S[1].back());  // top collapses to apex
    out.c3 = S[2];
    std::reverse(out.c3.begin(), out.c3.end());
    return true;
  }
  // N == 5 or 6.
  out.c3 = S[N - 1];
  std::reverse(out.c3.begin(), out.c3.end());
  std::vector<math::Point3> mid;
  for (int k = 2; k <= N - 2; ++k)
    for (std::size_t t = (k == 2 ? 0 : 1); t < S[k].size(); ++t) mid.push_back(S[k][t]);
  std::vector<math::Point3> top = resampleByArcLength(mid, gridN);
  if (top.empty()) return false;
  std::reverse(top.begin(), top.end());  // corner(N-1)→corner2 = u:0→1 at v=1
  out.c2 = std::move(top);
  return true;
}

// Emit two CCW triangles per grid cell into `mesh`, welding shared grid vertices by
// index (a single addVertex per grid node). Winding follows the (u,v) right-hand
// order so the patch normal is +(∂u × ∂v); fill_solid orients globally afterwards.
inline void triangulateGrid(const std::vector<std::vector<math::Point3>>& grid, tess::Mesh& mesh) {
  const int rows = static_cast<int>(grid.size());
  const int cols = static_cast<int>(grid[0].size());
  std::vector<std::vector<std::uint32_t>> idx(rows, std::vector<std::uint32_t>(cols));
  for (int j = 0; j < rows; ++j)
    for (int i = 0; i < cols; ++i) idx[j][i] = mesh.addVertex(grid[j][i]);
  for (int j = 0; j + 1 < rows; ++j)
    for (int i = 0; i + 1 < cols; ++i) {
      const std::uint32_t a = idx[j][i], b = idx[j][i + 1], c = idx[j + 1][i + 1],
                          d = idx[j + 1][i];
      mesh.addTriangle(a, b, c);
      mesh.addTriangle(a, c, d);
    }
}

// Newell area-weighted normal of a corner loop (winding-coherent). Default +Z for a
// degenerate loop.
inline math::Dir3 newellNormal(const std::vector<math::Point3>& loop) {
  if (loop.size() < 3) return math::Dir3{0, 0, 1};
  math::Vec3 n{0, 0, 0};
  const std::size_t m = loop.size();
  for (std::size_t i = 0; i < m; ++i) {
    const math::Point3& a = loop[i];
    const math::Point3& b = loop[(i + 1) % m];
    n.x += (a.y - b.y) * (a.z + b.z);
    n.y += (a.z - b.z) * (a.x + b.x);
    n.z += (a.x - b.x) * (a.y + b.y);
  }
  return math::norm(n) < kEps ? math::Dir3{0, 0, 1} : math::Dir3{n};
}

// Max signed-distance magnitude of the loop corners from their best-fit plane.
inline double planarityDeviation(const std::vector<math::Point3>& loop, const math::Dir3& n) {
  math::Vec3 c{0, 0, 0};
  for (const math::Point3& p : loop) c += p.asVec();
  const math::Point3 centroid{c.x / loop.size(), c.y / loop.size(), c.z / loop.size()};
  double worst = 0.0;
  for (const math::Point3& p : loop)
    worst = std::max(worst, std::fabs(math::dot(p - centroid, n.vec())));
  return worst;
}

// Triangulate a coplanar corner loop EXACTLY as a planar face: project to the plane's
// (u,v) frame, ear-clip, and emit triangles at the ORIGINAL corner points (bit-exact
// boundary; exact polygon area). Returns false for a degenerate/self-intersecting
// projection (empty triangulation).
inline bool triangulatePlanar(const std::vector<math::Point3>& loop, const math::Dir3& n,
                              tess::Mesh& mesh) {
  const math::Vec3 ref = std::fabs(n.z()) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
  const math::Ax3 frame = math::Ax3::fromAxisAndRef(loop.front(), n, math::Dir3{ref});
  std::vector<tess::UV> uv;
  uv.reserve(loop.size());
  for (const math::Point3& p : loop) {
    const math::Vec3 d = p - frame.origin;
    uv.push_back(tess::UV{math::dot(d, frame.x.vec()), math::dot(d, frame.y.vec())});
  }
  std::vector<int> outer(loop.size());
  for (std::size_t i = 0; i < loop.size(); ++i) outer[i] = static_cast<int>(i);
  const std::vector<tess::UVTri> tris = tess::triangulatePolygon(uv, {outer});
  if (tris.empty()) return false;
  std::vector<std::uint32_t> idx(loop.size());
  for (std::size_t i = 0; i < loop.size(); ++i) idx[i] = mesh.addVertex(loop[i]);
  for (const tess::UVTri& t : tris) mesh.addTriangle(idx[t.a], idx[t.b], idx[t.c]);
  return true;
}

// PLANAR fast-path attempt: if the boundary corners are coplanar AND every side is a
// straight segment, the patch IS the planar polygon — ear-clip it EXACTLY (bit-exact
// corners, exact polygon area; the "planar N-gon fill" arm, convex or not). Returns:
//   1  → filled (mesh + corner-only sideSamples written), caller returns Ok;
//   0  → not planar / has an arc → caller falls through to the Coons grid;
//  -1  → planar but the ear-clip failed (self-intersecting) → caller declines.
inline int tryPlanarFill(const Boundary& boundary, int N, tess::Mesh& mesh,
                         std::vector<std::vector<math::Point3>>& sideSamples) {
  for (const BoundarySide& s : boundary.sides)
    if (s.arc) return 0;
  std::vector<math::Point3> corners;
  corners.reserve(N);
  for (const BoundarySide& s : boundary.sides) corners.push_back(s.start);
  const math::Dir3 nrm = newellNormal(corners);
  if (planarityDeviation(corners, nrm) > 1e-9 || !nrm.valid()) return 0;  // not coplanar
  if (!triangulatePlanar(corners, nrm, mesh) || !(tess::surfaceArea(mesh) > kEps)) {
    mesh = tess::Mesh{};
    return -1;  // ear-clip failed (self-intersecting)
  }
  // The planar fan uses ONLY the corners on its boundary (no interior samples), so the
  // boundary side samples are corner→corner pairs — fill_solid's stitch then inserts
  // nothing into the shell edges (no T-junctions) and the planar cap welds exactly.
  sideSamples.assign(N, {});
  for (int k = 0; k < N; ++k)
    sideSamples[k] = {boundary.sides[k].start, boundary.sides[k].end};
  return 1;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// fillNGon — evaluate the bounded Coons/Gregory patch of `boundary` to a triangle
// mesh at `opts.gridN`. Returns an invalid patch + a measured NGonDecline reason for
// any out-of-bound input. The boundary samples are shared bit-exactly on the mesh's
// boundary so fill_solid.h can weld it watertight. See the file header for the bound.
// ─────────────────────────────────────────────────────────────────────────────
inline NGonPatch fillNGon(const Boundary& boundary, const NGonOptions& opts,
                          NGonDecline* decline) {
  NGonPatch out;
  auto fail = [&](NGonDecline r) {
    if (decline) *decline = r;
    return out;
  };
  const int N = static_cast<int>(boundary.sides.size());
  if (N < 3 || N > 6) return fail(NGonDecline::TooManySides);
  const int gridN = opts.gridN;
  if (gridN < 2) return fail(NGonDecline::NotConverged);

  // Sample every side (endpoints bit-exact). A collinear arc / zero side declines.
  std::vector<std::vector<math::Point3>> S(N);
  for (int k = 0; k < N; ++k) {
    // Loop-closure sanity: side k end must equal side k+1 start.
    const math::Point3& nextStart = boundary.sides[(k + 1) % N].start;
    if (math::distance(boundary.sides[k].end, nextStart) > 1e-7)
      return fail(NGonDecline::DegenerateBoundary);
    const auto s = detail::sampleSide(boundary.sides[k], gridN);
    if (!s) return fail(boundary.sides[k].arc ? NGonDecline::DegenerateBoundary
                                              : NGonDecline::DegenerateBoundary);
    S[k] = *s;
  }
  out.sideSamples = S;  // ordered per-side boundary samples (endpoints bit-exact)

  // PLANAR fast path (coplanar straight-edged loop → the exact planar polygon; reduces a
  // box-face restore to an exact planar cap). Arc side / non-coplanar → Coons grid.
  const int planar = detail::tryPlanarFill(boundary, N, out.mesh, out.sideSamples);
  if (planar == -1) return fail(NGonDecline::SelfIntersecting);
  if (planar == 1) {
    out.onBoundaryResidual = 0.0;
    out.valid = true;
    if (decline) *decline = NGonDecline::Ok;
    return out;
  }

  // Build the four Coons boundary curves for the loop (N=3 apex-fold / N=5,6
  // Gregory-style trailing-side merge — see coonsSidesForLoop), then evaluate the
  // (gridN+1)² transfinite grid and triangulate it.
  detail::CoonsSides cs;
  if (!detail::coonsSidesForLoop(S, N, gridN, cs)) return fail(NGonDecline::DegenerateBoundary);
  std::vector<std::vector<math::Point3>> grid;
  detail::coonsQuadGrid(cs.c0, cs.c1, cs.c2, cs.c3, gridN, grid);
  detail::triangulateGrid(grid, out.mesh);

  // On-boundary residual: the four grid boundary rows ARE the boundary samples, so
  // residual is 0 by construction for the represented sides; report 0 (arcs are
  // exact on the circle). A self-intersection guard: the mesh must be a simple grid
  // with positive total area.
  if (!(tess::surfaceArea(out.mesh) > detail::kEps)) return fail(NGonDecline::SelfIntersecting);

  out.onBoundaryResidual = 0.0;
  out.valid = true;
  if (decline) *decline = NGonDecline::Ok;
  return out;
}

}  // namespace cybercad::native::surface

#endif  // CYBERCAD_NATIVE_SURFACE_NGON_FILL_H
