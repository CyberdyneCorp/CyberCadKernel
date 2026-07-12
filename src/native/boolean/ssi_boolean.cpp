// SPDX-License-Identifier: Apache-2.0
//
// ssi_boolean.cpp — the DEFINITION of the SSI Stage S5-a curved boolean
// (ssi_boolean.h). Compiled into the kernel library always; the substrate-dependent
// body is behind CYBERCAD_HAS_NUMSCI. Without the substrate the entry point is a stub
// that returns NULL so the boolean dispatcher links either way (the engine then falls
// back to OCCT), matching how the S3 tracer entry points are gated.
//
// The pipeline (gate → split → classify → select → weld) lives here; see ssi_boolean.h
// for the design and the honest scope. Everything not robustly handleable returns a
// NULL Shape — the engine self-verify + OCCT fallback owns the rest. Nothing is faked.
//
#include "native/boolean/ssi_boolean.h"

#if defined(CYBERCAD_HAS_NUMSCI)

#include "native/boolean/assemble.h"  // VertexPool (shared-vertex weld)

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cybercad::native::boolean {

namespace {

using ssidetail::CurvedKind;
using ssidetail::CurvedSolid;
using ssidetail::classifyPoint;
using ssidetail::kSsiPi;
using ssidetail::kSsiTol;
using ssidetail::kSsiTwoPi;
using ssidetail::recogniseCurvedSolid;
using ssidetail::seamBand;
using ssidetail::WallSplit;

// Cap sagitta bound (model units): the maximum chord bow a single radial cap facet may
// have off the true pierced-cylinder surface. Sets the cap's radial refinement so the
// mouth-cavity volume converges; kept well below the engine's tessellation deflection
// (0.005) so the cap contributes negligible faceting bias. A deflection bound, NOT a
// result tolerance — nothing here is relaxed to force a pass.
inline constexpr double kCapSagitta = 1e-4;

// A discretised seam: the WLine polyline as world points + its (u,v) tracks on each
// operand. The two operands share these EXACT 3D points, so a face built on either
// side welds watertight along the seam.
struct Seam {
  std::vector<math::Point3> pts;                 ///< world seam points (shared)
  std::vector<std::pair<double, double>> uvA;    ///< (u,v) on A per point
  std::vector<std::pair<double, double>> uvB;    ///< (u,v) on B per point
  bool closed = false;
};

Seam toSeam(const ssi::WLine& w) {
  Seam s;
  s.closed = w.isClosed();
  s.pts.reserve(w.points.size());
  for (const auto& n : w.points) {
    s.pts.push_back(n.point);
    s.uvA.emplace_back(n.u1, n.v1);
    s.uvB.emplace_back(n.u2, n.v2);
  }
  return s;
}

// A closed WIRE of straight Line edges through the SHARED seam vertices (from
// `pool`), each edge carrying a Line pcurve SEGMENT in the given per-face (u,v)
// track. The seam is a traced polyline; representing it as many straight edges (one
// per polyline segment) — instead of ONE degree-1 B-spline edge — is what lets the
// mesher's shared per-edge discretization reproduce the whole seam: a degree-1
// B-spline has zero measured 3D curvature, so the edge discretizer collapses it to
// its 2 (coincident, for a closed loop) endpoints and the face is left with NO usable
// boundary (it then meshes the whole parametric wall — the original ~5.5x volume
// blow-up). Straight Line edges reproduce the polyline exactly at one segment each, so
// the boundary is faithful and the ear-clip fills the true region. Because both faces
// sharing a seam node take their vertex from the SAME `pool`, the seam welds
// watertight. `pts.size() == uv.size()`; the loop closes pts[n-1] → pts[0].
topo::Shape seamWire(const std::vector<math::Point3>& pts,
                     const std::vector<std::pair<double, double>>& uv, VertexPool& pool) {
  const int n = static_cast<int>(pts.size());
  std::vector<topo::Shape> edges;
  edges.reserve(n);
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;  // closed loop
    const topo::Shape v0 = pool.vertexFor(pts[i]);
    const topo::Shape v1 = pool.vertexFor(pts[j]);
    const math::Vec3 d = pts[j] - pts[i];
    const double len = std::max(math::norm(d), 1e-12);
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame.origin = pts[i];
    c.frame.x = math::norm(d) > 1e-12 ? math::Dir3{d} : math::Dir3{1.0, 0.0, 0.0};
    const topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, len, v0, v1);
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = math::Point3{uv[i].first, uv[i].second, 0.0};
    pc.dir2d = math::Vec3{(uv[j].first - uv[i].first) / len,
                          (uv[j].second - uv[i].second) / len, 0.0};
    edges.push_back(topo::ShapeBuilder::addPCurve(e, e.tshape(), pc));
  }
  return topo::ShapeBuilder::makeWire(std::move(edges));
}

// FaceSurface for a curved solid's wall (the analytic kind + world frame).
topo::FaceSurface wallSurface(const CurvedSolid& cs) {
  topo::FaceSurface s;
  s.frame = cs.frame;
  s.radius = cs.radius;
  s.semiAngle = cs.semiAngle;
  switch (cs.kind) {
    case CurvedKind::Cylinder: s.kind = topo::FaceSurface::Kind::Cylinder; break;
    case CurvedKind::Sphere:   s.kind = topo::FaceSurface::Kind::Sphere; break;
    case CurvedKind::Cone:     s.kind = topo::FaceSurface::Kind::Cone; break;
  }
  return s;
}

// u-span of a seam on one operand's wall (angular coverage of the (u,v) track). A
// FULL-CIRCLE seam (span ≈ 2π) means the wall is pierced right through at that
// station → the seam is a rim loop of a through wall BAND; a SMALL span means the
// seam is a local patch (a drill mouth) on that wall.
double uSpan(const std::vector<std::pair<double, double>>& uv) {
  double lo = uv.front().first, hi = lo;
  for (const auto& p : uv) { lo = std::min(lo, p.first); hi = std::max(hi, p.first); }
  return hi - lo;
}
bool isFullCircle(const std::vector<std::pair<double, double>>& uv) {
  return uSpan(uv) > 0.75 * kSsiTwoPi;  // covers most of a revolution → a rim loop
}
double meanV(const std::vector<std::pair<double, double>>& uv) {
  double s = 0.0;
  for (const auto& p : uv) s += p.second;
  return s / static_cast<double>(uv.size());
}
double meanU(const std::vector<std::pair<double, double>>& uv) {
  double s = 0.0;
  for (const auto& p : uv) s += p.first;
  return s / static_cast<double>(uv.size());
}

// Shortest signed azimuth from `b` to `a` folded into (−π, π]; |·| is the angular gap.
double wrapDiff(double a, double b) {
  double d = a - b;
  while (d > kSsiPi) d -= kSsiTwoPi;
  while (d < -kSsiPi) d += kSsiTwoPi;
  return std::fabs(d);
}
// Fold `u` into the ±π window around `base` (contiguous, no ±2π jump).
double nearU(double base, double u) {
  while (u < base - kSsiPi) u += kSsiTwoPi;
  while (u > base + kSsiPi) u -= kSsiTwoPi;
  return u;
}

// Unwrap a rim's (u,v) track so its u is monotone increasing over one turn, reversing
// the node/track order (keeping 3D↔uv correspondence) when it runs the other way.
void unwrapRim(std::vector<std::pair<double, double>>& t, std::vector<math::Point3>& p) {
  for (std::size_t k = 1; k < t.size(); ++k) t[k].first = nearU(t[k - 1].first, t[k].first);
  if (t.back().first < t.front().first) {
    std::reverse(t.begin(), t.end());
    std::reverse(p.begin(), p.end());
  }
}

// One PLANAR triangle face through three SHARED pool vertices, its plane Z oriented
// so the face normal points AWAY from the tube axis (radially outward = the material-
// outward normal of the piercing tube's outer wall). Built as three shared-vertex Line
// edges with Line pcurves so the FaceMesher meshes it as a single flat triangle (no
// interior points → exact) and adjacent triangles weld on the shared corners.
//
// Why PLANAR facets for the tube band (not the analytic cylinder surface): a cylinder
// face over one seam-node arc is meshed by the STRUCTURED-GRID path, which adds interior
// curvature u-lines on the shared rim rows — samples the ear-clipped drill-mouth caps do
// NOT place, opening a T-junction and leaving the shell non-watertight. A PLANAR triangle
// through the SAME three traced seam nodes carries no interior points, so its rim edges
// are exactly the traced-node segments the caps also ear-clip along → the band↔cap seam
// welds. The facets inscribe the tube (the honest piecewise-linear chord of the traced
// polyline the S3 tracer already produced); the tessellation chord error shrinks as the
// trace densifies, exactly like any faceted curved boundary.
topo::Shape tubeTriFace(const math::Point3& a, const math::Point3& b, const math::Point3& c,
                        const math::Point3& axisPt, const math::Dir3& axisDir, VertexPool& pool) {
  const topo::Shape va = pool.vertexFor(a);
  const topo::Shape vb = pool.vertexFor(b);
  const topo::Shape vc = pool.vertexFor(c);
  const math::Vec3 nrm = math::cross(b - a, c - a);
  if (math::norm(nrm) < 1e-14) return {};  // degenerate sliver → skip
  // Orient the plane normal radially outward (away from the tube axis) at the centroid.
  const math::Point3 ctr{(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0,
                         (a.z + b.z + c.z) / 3.0};
  const math::Vec3 w = ctr - axisPt;
  const math::Vec3 radial = w - axisDir.vec() * math::dot(w, axisDir.vec());
  const bool outward = math::dot(nrm, radial) >= 0.0;
  const math::Vec3 outN = outward ? nrm : math::Vec3{-nrm.x, -nrm.y, -nrm.z};
  const math::Ax3 frame = math::Ax3::fromAxisAndRef(a, math::Dir3{outN}, math::Dir3{b - a});
  // Wind the triangle so its (frame-Z) outward normal matches the vertex order.
  return outward ? detail::triangleFace(va, vb, vc, frame)
                 : detail::triangleFace(va, vc, vb, frame);
}

// ── DRILL-MOUTH CAP (design.md §1 cap fragment) ────────────────────────────────
// The drill mouth is the patch of the PIERCED cylinder wall bounded by one seam loop.
// The pierced wall is CURVED (the fat cylinder bulges outward across the mouth), so a
// flat ear-clip of the seam loop chords that bulge and removes too much volume (≈2.5×
// the true mouth cavity — a systematic, non-vanishing bias). We instead RADIALLY refine
// the cap: fan it from the mouth's centre point (evaluated ON the pierced cylinder, so
// it sits on the true bulged surface) out to the seam boundary through `rings`
// concentric rings, each ring's node evaluated on the pierced cylinder at the (u,v)
// linearly interpolated from the centre (u,v) to the boundary node's (u,v). Every facet
// is a PLANAR triangle through pierced-cylinder-surface points, so it follows the bulge
// to O(1/rings²) while staying flat-per-facet (welds like the tube band). The OUTER ring
// nodes are the EXACT traced seam nodes from `pool`, identical to the tube band's rim, so
// the cap↔band seam welds watertight. All interior ring/centre points are unique to the
// cap (no neighbour), so they introduce no T-junction.
//
// systems-band (~14 — radial ring interpolation + outward orientation); flagged.
void appendMouthCap(const CurvedSolid& pierced, const Seam& seam,
                    const std::vector<std::pair<double, double>>& uv, int rings, VertexPool& pool,
                    std::vector<topo::Shape>& faces) {
  const int n = static_cast<int>(seam.pts.size());
  if (n < 3 || rings < 1) return;
  // Mouth centre in (u,v): mean of the boundary track (u folded contiguous around the
  // first node so the mean is not corrupted by the periodic wrap).
  const double u0 = uv.front().first;
  double uSum = 0.0, vSum = 0.0;
  for (const auto& p : uv) { uSum += nearU(u0, p.first); vSum += p.second; }
  const double uc = uSum / n, vc = vSum / n;
  const math::Point3 centre = pierced.point(uc, vc);
  // Ring r (1..rings) node k: (u,v) = lerp(centre_uv → boundary_uv[k], r/rings), on surface.
  auto ringPt = [&](int r, int k) -> math::Point3 {
    const double t = static_cast<double>(r) / rings;
    const double u = uc + (nearU(u0, uv[k].first) - uc) * t;
    const double v = vc + (uv[k].second - vc) * t;
    return (r == rings) ? seam.pts[k] : pierced.point(u, v);  // outer ring = exact seam node
  };
  // Outward orientation reference: cap normal should point AWAY from the pierced solid's
  // interior — i.e. along +radial of the pierced cylinder at the centre (its surface
  // outward normal), which for the drill mouth is the outward cap normal.
  const math::Point3 axisPt = pierced.frame.origin;
  const math::Dir3 axisDir = pierced.frame.z;
  auto tri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    const topo::Shape va = pool.vertexFor(a), vb = pool.vertexFor(b), vc2 = pool.vertexFor(c);
    const math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::norm(nrm) < 1e-14) return;
    const math::Point3 ctr{(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
    const math::Vec3 w = ctr - axisPt;
    const math::Vec3 radial = w - axisDir.vec() * math::dot(w, axisDir.vec());
    const bool outward = math::dot(nrm, radial) >= 0.0;
    const math::Vec3 oN = outward ? nrm : math::Vec3{-nrm.x, -nrm.y, -nrm.z};
    const math::Ax3 fr = math::Ax3::fromAxisAndRef(a, math::Dir3{oN}, math::Dir3{b - a});
    faces.push_back(outward ? detail::triangleFace(va, vb, vc2, fr)
                            : detail::triangleFace(va, vc2, vb, fr));
  };
  // Innermost ring: a fan of triangles from the centre to ring-1 nodes.
  for (int k = 0; k < n; ++k) tri(centre, ringPt(1, k), ringPt(1, (k + 1) % n));
  // Outer rings: a quad strip between ring r-1 and ring r, split into two triangles.
  for (int r = 2; r <= rings; ++r)
    for (int k = 0; k < n; ++k) {
      const int kn = (k + 1) % n;
      tri(ringPt(r - 1, k), ringPt(r, k), ringPt(r, kn));
      tri(ringPt(r - 1, k), ringPt(r, kn), ringPt(r - 1, kn));
    }
}

// ── TUBE BAND (design.md §1 tube fragment) ─────────────────────────────────────
// The tube fragment is the part of the piercing wall between the two full-circle rim
// seams. On the tube's (u,v) plane it is a full-turn band whose top/bottom are the two
// (wobbly) traced seam loops. We emit it as a STRIP OF PLANAR TRIANGLE facets: rim-seam-0
// node c is paired with the rim-seam-1 node at the nearest azimuth (u mod 2π); each
// consecutive pair spans a quad, split into two triangles. Rim 1 is unwrapped/reversed to
// wind with rim 0 and phase-rotated to start nearest rim 0's start azimuth so the pairing
// tracks azimuth. Every facet vertex is an EXACT traced seam node from `pool`, IDENTICAL
// to the drill-mouth cap boundaries, so the band↔cap seams weld watertight (see
// tubeTriFace for why planar facets, not the analytic cylinder surface, are required).
//
// systems-band (~22 — periodic rim alignment + per-quad triangulation); isolated + flagged.
void appendTubeBand(const CurvedSolid& tube, const std::vector<Seam>& seams,
                    const std::vector<std::pair<double, double>>& tuv0In,
                    const std::vector<std::pair<double, double>>& tuv1In, VertexPool& pool,
                    std::vector<topo::Shape>& faces) {
  std::vector<std::pair<double, double>> t0 = tuv0In, t1 = tuv1In;
  std::vector<math::Point3> p0 = seams[0].pts, p1 = seams[1].pts;
  if (t0.size() < 3 || t1.size() < 3) return;
  unwrapRim(t0, p0);
  unwrapRim(t1, p1);
  // Phase-rotate rim 1 to start nearest rim 0's start azimuth.
  int start1 = 0;
  for (int k = 1; k < static_cast<int>(t1.size()); ++k)
    if (wrapDiff(t0.front().first, t1[k].first) < wrapDiff(t0.front().first, t1[start1].first))
      start1 = k;
  const math::Point3 axisPt = tube.frame.origin;
  const math::Dir3 axisDir = tube.frame.z;
  const int n0 = static_cast<int>(t0.size());
  const int n1 = static_cast<int>(t1.size());
  auto push = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    const topo::Shape f = tubeTriFace(a, b, c, axisPt, axisDir, pool);
    if (!f.isNull()) faces.push_back(f);
  };
  for (int c = 0; c < n0; ++c) {
    const int cd = (c + 1) % n0;
    const int e = (start1 + c) % n1;
    const int ed = (start1 + c + 1) % n1;
    // Quad (p0[c], p0[cd], p1[ed], p1[e]) → two triangles sharing the p0[cd]-p1[e] diag.
    push(p0[c], p0[cd], p1[e]);
    push(p0[cd], p1[ed], p1[e]);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// S5-b — through-drill cyl∩cyl FUSE and CUT (assembler-only extension of S5-a).
//
// FUSE / CUT keep the SAME gate + trace + two rim seams as COMMON. What changes is
// WHICH wall fragments survive (the planar set algebra of native_boolean.h) and their
// winding — never the tracer, the tessellator, or the cc_* ABI. The surviving wall
// fragments that touch a seam are emitted as PLANAR-TRIANGLE facets through the shared
// pool (the S5-a watertight discipline) so they weld against the tube band + each other;
// the analytic cylinder/disc faces are NOT reused for any seam-adjacent fragment (their
// structured-grid mesh injects interior u-lines → T-junctions against the facet neighbours,
// the exact S5-a failure). Anything not this clean through-drill config → NULL → OCCT.
//
// Fragment inventory of a through-drill A−B / A∪B (A = pierced/fat, B = piercing/thin):
//   * PIERCED WALL with the two drill-mouth seam loops CUT OUT (kept for both ops — it is
//     the part of the fat lateral wall OUTSIDE the thin tube). Planar-facet, outward.
//   * the two PIERCED DISC CAPS (fat z-caps, entirely outside the tube) — re-emitted as
//     planar-facet fans whose rim shares the wall's top/bottom u-samples (so cap↔wall welds
//     without a T-junction against an analytic circle rim). Outward.
//   * TUBE band between the two rim seams:
//       CUT  → the tunnel wall: the thin tube band, INWARD-facing (material is outside it).
//       FUSE → dropped (that stretch of the thin wall is inside the fat solid).
//   * the two PIERCING END TUBES (thin wall from each seam out to the thin end cap):
//       FUSE → kept, outward (the boss sticking out of the fat wall), + the two thin disc
//              end caps (planar-facet fans). Their inner rim IS a seam → welds the fat hole.
//       CUT  → dropped (thin end tubes are outside the fat solid, removed by the cut).
// ─────────────────────────────────────────────────────────────────────────────

// One PLANAR triangle through three SHARED pool vertices, oriented so its face normal
// points on the same side as (refOutward). Generalises tubeTriFace / appendMouthCap's
// local `tri` so the wall, caps and end tubes share one orientation primitive.
void pushPlanarTri(const math::Point3& a, const math::Point3& b, const math::Point3& c,
                   const math::Vec3& refOutward, VertexPool& pool,
                   std::vector<topo::Shape>& faces) {
  const topo::Shape va = pool.vertexFor(a), vb = pool.vertexFor(b), vc = pool.vertexFor(c);
  const math::Vec3 nrm = math::cross(b - a, c - a);
  if (math::norm(nrm) < 1e-14) return;  // degenerate sliver → skip
  const bool outward = math::dot(nrm, refOutward) >= 0.0;
  const math::Vec3 oN = outward ? nrm : math::Vec3{-nrm.x, -nrm.y, -nrm.z};
  const math::Ax3 fr = math::Ax3::fromAxisAndRef(a, math::Dir3{oN}, math::Dir3{b - a});
  faces.push_back(outward ? detail::triangleFace(va, vb, vc, fr)
                          : detail::triangleFace(va, vc, vb, fr));
}

// Radial vector from the pierced axis to a world point (the pierced wall's outward
// material normal there) — the outward reference for a fat-wall / tunnel facet.
math::Vec3 radialOut(const CurvedSolid& cs, const math::Point3& p) {
  const math::Vec3 w = p - cs.frame.origin;
  return w - cs.frame.z.vec() * math::dot(w, cs.frame.z.vec());
}

// A single closed drill-mouth seam loop expressed as the pierced wall's (u,v) track,
// unwrapped into a contiguous window around its own centroid azimuth `uc` (no ±2π jump),
// paired with the SHARED 3D seam nodes. Used to place the loop as a hole in the unrolled
// wall panel and to weld its boundary (== the tube band rim) via the pool.
struct MouthLoop {
  double uc = 0.0;                                   ///< centroid azimuth (contiguous)
  std::vector<std::pair<double, double>> uv;         ///< (u contiguous, v) per node
  const Seam* seam = nullptr;                        ///< shared 3D nodes (seam->pts)
};
MouthLoop makeMouthLoop(const Seam& seam, const std::vector<std::pair<double, double>>& uvIn) {
  MouthLoop m;
  m.seam = &seam;
  const double u0 = uvIn.front().first;
  double uSum = 0.0;
  m.uv.reserve(uvIn.size());
  for (const auto& p : uvIn) {
    const double uu = nearU(u0, p.first);
    m.uv.emplace_back(uu, p.second);
    uSum += uu;
  }
  m.uc = uSum / static_cast<double>(uvIn.size());
  return m;
}

// Even-odd point-in-polygon test in the (u,v) plane for a mouth loop `m` at query (u,v).
// `u` is first folded contiguous around the loop centroid so the wrap does not corrupt it.
bool insideMouth(const MouthLoop& m, double u, double v) {
  const double uq = nearU(m.uc, u);
  bool in = false;
  const std::size_t n = m.uv.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double ui = m.uv[i].first, vi = m.uv[i].second;
    const double uj = m.uv[j].first, vj = m.uv[j].second;
    if (((vi > v) != (vj > v)) && (uq < (uj - ui) * (v - vi) / (vj - vi) + ui)) in = !in;
  }
  return in;
}

// A structured (u,v) grid over the pierced wall: u wraps (column nu ≡ 0), v spans [vLo,vHi].
// Carries the sampling so the three wall sub-phases (grid emit / contour trace / ribbon) share
// one coordinate system instead of re-deriving it (and to keep each phase low-complexity).
struct GridWall {
  const CurvedSolid& cs;
  int nu, nv;
  double uAt(int i) const { return kSsiTwoPi * i / nu; }
  double vAt(int j) const { return cs.vLo + (cs.vHi - cs.vLo) * j / nv; }
  int wrap(int i) const { return ((i % nu) + nu) % nu; }
  math::Point3 at(int i, int j) const { return cs.point(uAt(wrap(i)), vAt(j)); }
  // A cell (i,j) is REMOVED if ANY vertex in its 1-cell DILATED corner neighbourhood (i-1..i+2 ×
  // j-1..j+2) lies inside `m`. The dilation makes the removed block strictly CONTAIN the wobbly
  // seam loop even where it bulges between grid columns (else the seam would poke out of the
  // block boundary and the annulus stitch could not close).
  bool removed(const MouthLoop& m, int i, int j) const {
    for (int dj = -1; dj <= 2; ++dj)
      for (int di = -1; di <= 2; ++di) {
        const int jj = j + dj;
        if (jj < 0 || jj > nv) continue;
        if (insideMouth(m, uAt(wrap(i + di)), vAt(std::clamp(jj, 0, nv)))) return true;
      }
    return false;
  }
};

// Trace the removed-block boundary of ONE mouth as an ORDERED grid-edge contour (list of
// (column, row) grid vertices). The edges are the grid edges the neighbouring KEPT cells emit,
// so the annulus outer loop welds the surrounding grid exactly (an angle-sort would reorder a
// rectilinear boundary and open cracks). Returns false (→ decline) if the mouth covers no cell,
// touches a wall v-edge, or the boundary is not one simple loop.
// The cell-index bounding block of one mouth's dilated removed region (u contiguous around the
// mouth centroid). ok == false when the mouth covers no cell or touches a wall v-edge (decline).
struct BlockRange { int iMin, iMax, jMin, jMax; bool ok; };
BlockRange mouthBlockRange(const GridWall& g, const MouthLoop& m) {
  auto rem = [&](int di, int j) { return j >= 0 && j < g.nv && g.removed(m, g.wrap(di), j); };
  int jMin = g.nv, jMax = -1;
  for (int j = 0; j < g.nv; ++j)
    for (int i = 0; i < g.nu; ++i)
      if (rem(i, j)) { jMin = std::min(jMin, j); jMax = std::max(jMax, j); }
  if (jMax < 0 || jMin == 0 || jMax + 1 >= g.nv) return {0, 0, 0, 0, false};
  const int iuc = static_cast<int>(std::lround(m.uc / kSsiTwoPi * g.nu));
  int iMin = g.nu, iMax = -1;
  for (int di = -g.nu / 2; di <= g.nu / 2; ++di)
    for (int j = jMin; j <= jMax; ++j)
      if (rem(di + iuc, j)) { iMin = std::min(iMin, di + iuc); iMax = std::max(iMax, di + iuc); }
  return {iMin, iMax, jMin, jMax, iMax >= iMin};
}

// Chain a directed-edge map (start-key → end-key, with key→(col,row)) into one ordered loop.
bool chainLoop(const std::unordered_map<long long, long long>& nextV,
               const std::unordered_map<long long, std::pair<int, int>>& vIJ,
               std::vector<std::pair<int, int>>& loopOut) {
  if (nextV.empty()) return false;
  loopOut.clear();
  const long long startK = nextV.begin()->first;
  long long cur = startK;
  for (std::size_t guard = 0; guard <= nextV.size(); ++guard) {
    loopOut.push_back(vIJ.at(cur));
    auto it = nextV.find(cur);
    if (it == nextV.end()) return false;  // open contour → decline
    cur = it->second;
    if (cur == startK) break;
  }
  return loopOut.size() == nextV.size() && loopOut.size() >= 3;  // one simple loop
}

bool traceBlockContour(const GridWall& g, const MouthLoop& m,
                       std::vector<std::pair<int, int>>& loopOut) {
  const BlockRange br = mouthBlockRange(g, m);
  if (!br.ok) return false;
  auto rem = [&](int di, int j) { return j >= 0 && j < g.nv && g.removed(m, g.wrap(di), j); };
  // Directed boundary edges (removed cell on the LEFT → CCW loop).
  auto key = [&](int di, int j) { return static_cast<long long>(di) * 100000 + j; };
  std::unordered_map<long long, long long> nextV;
  std::unordered_map<long long, std::pair<int, int>> vIJ;
  auto edge = [&](int adi, int aj, int bdi, int bj) {
    nextV[key(adi, aj)] = key(bdi, bj);
    vIJ[key(adi, aj)] = {adi, aj};
    vIJ[key(bdi, bj)] = {bdi, bj};
  };
  for (int di = br.iMin; di <= br.iMax; ++di)
    for (int j = br.jMin; j <= br.jMax; ++j) {
      if (!rem(di, j)) continue;
      if (!rem(di, j - 1)) edge(di, j, di + 1, j);          // bottom (u+)
      if (!rem(di + 1, j)) edge(di + 1, j, di + 1, j + 1);  // right  (v+)
      if (!rem(di, j + 1)) edge(di + 1, j + 1, di, j + 1);  // top    (u-)
      if (!rem(di - 1, j)) edge(di, j + 1, di, j);          // left   (v-)
    }
  return chainLoop(nextV, vIJ, loopOut);
}

// A polyline loop of world points, each tagged with its azimuth about a shared (u,v) centroid.
struct RibbonRing {
  std::vector<math::Point3> pts;
  std::vector<double> ang;
  void add(double u, double v, double cu, double cv, const math::Point3& P) {
    pts.push_back(P);
    ang.push_back(std::atan2(v - cv, u - cu));
  }
  // Make the loop wind CCW (net azimuth increasing); reverse if it runs the other way.
  void orientCCW() {
    double net = 0.0;
    for (std::size_t k = 1; k < ang.size(); ++k) {
      double d = ang[k] - ang[k - 1];
      while (d > kSsiPi) d -= kSsiTwoPi;
      while (d < -kSsiPi) d += kSsiTwoPi;
      net += d;
    }
    if (net < 0) { std::reverse(pts.begin(), pts.end()); std::reverse(ang.begin(), ang.end()); }
  }
  // Normalised (0..1) cumulative CCW azimuth swept from index `start` over `cnt` edges.
  std::vector<double> sweep(int start, int cnt) const {
    std::vector<double> s(cnt + 1, 0.0);
    const int n = static_cast<int>(ang.size());
    for (int k = 0; k < cnt; ++k) {
      double d = ang[(start + k + 1) % n] - ang[(start + k) % n];
      while (d > kSsiPi) d -= kSsiTwoPi;
      while (d < -kSsiPi) d += kSsiTwoPi;
      s[k + 1] = s[k] + d;
    }
    const double tot = s[cnt] != 0.0 ? s[cnt] : 1.0;
    for (double& x : s) x /= tot;
    return s;
  }
};

// Fill the annulus between the OUTER block-boundary loop and the INNER seam loop with a robust
// two-loop RIBBON stitch (no ear-clip): both loops are closed around the mouth centroid, so we
// merge-walk them by matched swept-azimuth, advancing whichever ring's next node is at the
// smaller fraction of the full turn and emitting one planar triangle per advance. After no + ni
// advances both rings close, so EVERY outer (grid) edge and EVERY inner (seam) edge is a
// triangle edge → the ribbon welds its two neighbours (grid + tube band) watertight, with no
// near-degenerate slivers (an ear-clip of a thin annulus drops slivers → 3-edge holes).
bool appendAnnulusRibbon(const CurvedSolid& pierced, const GridWall& g, const MouthLoop& m,
                         const std::vector<std::pair<int, int>>& loop, VertexPool& pool,
                         std::vector<topo::Shape>& faces) {
  double cu = 0, cv = 0;
  for (const auto& [di, j] : loop) { cu += nearU(m.uc, g.uAt(g.wrap(di))); cv += g.vAt(j); }
  cu /= loop.size(); cv /= loop.size();
  RibbonRing outer, inner;
  for (const auto& [di, j] : loop)
    outer.add(nearU(m.uc, g.uAt(g.wrap(di))), g.vAt(j), cu, cv, g.at(di, j));
  for (std::size_t k = 0; k < m.uv.size(); ++k)
    inner.add(nearU(m.uc, m.uv[k].first), m.uv[k].second, cu, cv, m.seam->pts[k]);
  const int no = static_cast<int>(outer.pts.size()), ni = static_cast<int>(inner.pts.size());
  if (no < 3 || ni < 3) return false;
  outer.orientCCW();
  inner.orientCCW();
  auto angGap = [](double a, double b) {
    double d = a - b; while (d > kSsiPi) d -= kSsiTwoPi; while (d < -kSsiPi) d += kSsiTwoPi;
    return std::fabs(d);
  };
  int si = 0;  // align inner start to outer start azimuth so the ribbon does not spiral
  for (int k = 1; k < ni; ++k)
    if (angGap(outer.ang[0], inner.ang[k]) < angGap(outer.ang[0], inner.ang[si])) si = k;
  const std::vector<double> so = outer.sweep(0, no);
  const std::vector<double> siA = inner.sweep(si, ni);
  auto emit = [&](const math::Point3& A, const math::Point3& B, const math::Point3& C) {
    const math::Point3 c{(A.x + B.x + C.x) / 3, (A.y + B.y + C.y) / 3, (A.z + B.z + C.z) / 3};
    pushPlanarTri(A, B, C, radialOut(pierced, c), pool, faces);
  };
  int io = 0, ii = 0;
  for (int step = 0; step < no + ni; ++step) {
    const bool takeOuter = (io < no) && (ii >= ni || so[io + 1] <= siA[ii + 1]);
    const int iiC = (si + ii) % ni;
    if (takeOuter) {
      emit(outer.pts[io % no], outer.pts[(io + 1) % no], inner.pts[iiC]);  // outer (grid) edge
      ++io;
    } else {
      emit(inner.pts[(si + ii + 1) % ni], inner.pts[iiC], outer.pts[io % no]);  // inner (seam) edge
      ++ii;
    }
  }
  return true;
}

// ── PIERCED WALL with two mouth holes → planar facets (the CUT/FUSE fat wall) ────
// The fat lateral wall is a full-turn (u∈[0,2π]) band over v∈[vLo,vHi] minus the two drill-mouth
// seam loops. Driver: tile the wall with a STRUCTURED (u,v) quad grid (u finely sampled for
// curvature, v coarser — a cylinder is straight along v so tall facets stay exact), emit two
// planar triangles per KEPT cell, then for each mouth trace its removed-block boundary contour
// and ribbon-stitch it to the seam. Grid vertices come from ONE pool, so cell↔cell, wall↔cap
// (shared v=vLo/vHi rows) and wall↔tube (shared seam nodes) all weld. Returns the ORDERED
// v=vLo / v=vHi rim rings (full circle) for the disc caps. The irreducible sub-phases are
// isolated in traceBlockContour / mouthBlockRange / appendAnnulusRibbon (each ≤ ~20 cognitive,
// systems band), so this driver stays a short linear composition. Any non-through-drill config
// (mouths coincident, touching a wall edge, non-simple block) → false (declined → OCCT).
bool appendHoledWall(const CurvedSolid& pierced, const Seam& s0, const Seam& s1,
                     const std::vector<std::pair<double, double>>& uv0,
                     const std::vector<std::pair<double, double>>& uv1, int uCells, int vCells,
                     VertexPool& pool, std::vector<topo::Shape>& faces,
                     std::vector<math::Point3>& rimLo, std::vector<math::Point3>& rimHi) {
  const MouthLoop m0 = makeMouthLoop(s0, uv0);
  const MouthLoop m1 = makeMouthLoop(s1, uv1);
  const double gap = std::fabs(nearU(m0.uc, m1.uc) - m0.uc);
  if (!(gap > 0.2) || !(kSsiTwoPi - gap > 0.2)) return false;  // mouths not distinct

  const GridWall g{pierced, uCells, vCells};
  const int nu = uCells, nv = vCells;

  // Full-circle cap rim rings at v=vLo / v=vHi (shared with the disc caps).
  rimLo.assign(nu, math::Point3{});
  rimHi.assign(nu, math::Point3{});
  for (int i = 0; i < nu; ++i) { rimLo[i] = g.at(i, 0); rimHi[i] = g.at(i, nv); }

  // Emit every KEPT grid cell (skip a cell removed by either mouth) as two planar triangles.
  for (int j = 0; j < nv; ++j)
    for (int i = 0; i < nu; ++i) {
      if (g.removed(m0, i, j) || g.removed(m1, i, j)) continue;
      const math::Point3 a = g.at(i, j), b = g.at(i + 1, j), c = g.at(i + 1, j + 1), d = g.at(i, j + 1);
      const math::Point3 ctr{(a.x + b.x + c.x + d.x) / 4, (a.y + b.y + c.y + d.y) / 4,
                             (a.z + b.z + c.z + d.z) / 4};
      const math::Vec3 out = radialOut(pierced, ctr);
      pushPlanarTri(a, b, c, out, pool, faces);
      pushPlanarTri(a, c, d, out, pool, faces);
    }

  // Ribbon-stitch each mouth's removed block to its seam.
  for (const MouthLoop* m : {&m0, &m1}) {
    std::vector<std::pair<int, int>> loop;
    if (!traceBlockContour(g, *m, loop)) return false;
    if (!appendAnnulusRibbon(pierced, g, *m, loop, pool, faces)) return false;
  }
  return true;
}

// ── FACETED DISC CAP: a fan from the axis centre (at v) out to the wall rim samples
// (`rim`, in u-order), planar facets, oriented along ±axis (`capOutward`). The rim
// points are the SAME 3D nodes the wall's top/bottom row emitted (shared pool) → the
// cap↔wall seam welds without a T-junction against an analytic circle rim.
void appendDiskCap(const CurvedSolid& cs, double v, const std::vector<math::Point3>& rim,
                   const math::Vec3& capOutward, VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int n = static_cast<int>(rim.size());
  if (n < 3) return;
  // The axis point at this v (a cylinder disc cap's centre): origin + v·axis.
  const math::Point3 axisPt{cs.frame.origin.x + cs.frame.z.vec().x * v,
                            cs.frame.origin.y + cs.frame.z.vec().y * v,
                            cs.frame.origin.z + cs.frame.z.vec().z * v};
  for (int k = 0; k + 1 < n; ++k) pushPlanarTri(axisPt, rim[k], rim[k + 1], capOutward, pool, faces);
  pushPlanarTri(axisPt, rim[n - 1], rim[0], capOutward, pool, faces);
}

// ── ORIENTED TUBE BAND: the piercing wall between the two rim seams, its facet normals
// forced to point on the side of `outwardSign` × radial-out-of-the-tube (+1 outer boss
// wall for FUSE end tubes / -1 inward tunnel wall for CUT). Mirrors appendTubeBand but
// takes the orientation reference so the same nodes serve both a boss and a tunnel.
void appendOrientedTubeBand(const CurvedSolid& tube, const std::vector<Seam>& seams,
                            const std::vector<std::pair<double, double>>& tuv0In,
                            const std::vector<std::pair<double, double>>& tuv1In, double outwardSign,
                            VertexPool& pool, std::vector<topo::Shape>& faces) {
  std::vector<std::pair<double, double>> t0 = tuv0In, t1 = tuv1In;
  std::vector<math::Point3> p0 = seams[0].pts, p1 = seams[1].pts;
  if (t0.size() < 3 || t1.size() < 3) return;
  unwrapRim(t0, p0);
  unwrapRim(t1, p1);
  int start1 = 0;
  for (int k = 1; k < static_cast<int>(t1.size()); ++k)
    if (wrapDiff(t0.front().first, t1[k].first) < wrapDiff(t0.front().first, t1[start1].first))
      start1 = k;
  const int n0 = static_cast<int>(t0.size());
  const int n1 = static_cast<int>(t1.size());
  auto push = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    const math::Point3 ctr{(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
    const math::Vec3 ref = radialOut(tube, ctr);
    pushPlanarTri(a, b, c, math::Vec3{ref.x * outwardSign, ref.y * outwardSign, ref.z * outwardSign},
                  pool, faces);
  };
  for (int c = 0; c < n0; ++c) {
    const int cd = (c + 1) % n0;
    const int e = (start1 + c) % n1;
    const int ed = (start1 + c + 1) % n1;
    push(p0[c], p0[cd], p1[e]);
    push(p0[cd], p1[ed], p1[e]);
  }
}

// ── PIERCING END TUBE (FUSE): the piercing wall from ONE seam out to the near thin end
// cap — the part of the thin wall OUTSIDE the fat solid, i.e. the protruding boss. Planar
// facets between the seam rim (shared pool → welds the fat wall hole) and a fresh rim ring
// at the thin end (v = vEnd), whose ring the thin disc cap re-uses. Outward radial normal.
// Returns the end-rim ring (u-ordered) for the thin disc cap; empty on decline.
std::vector<math::Point3> appendEndTube(const CurvedSolid& tube, const Seam& seam,
                                        const std::vector<std::pair<double, double>>& tuvIn,
                                        double vEnd, VertexPool& pool,
                                        std::vector<topo::Shape>& faces) {
  std::vector<std::pair<double, double>> t = tuvIn;
  std::vector<math::Point3> p = seam.pts;
  if (t.size() < 3) return {};
  unwrapRim(t, p);
  const int n = static_cast<int>(t.size());
  std::vector<math::Point3> endRing(n);
  for (int k = 0; k < n; ++k) endRing[k] = tube.point(t[k].first, vEnd);
  auto push = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    const math::Point3 ctr{(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
    pushPlanarTri(a, b, c, radialOut(tube, ctr), pool, faces);
  };
  for (int k = 0; k < n; ++k) {
    const int kn = (k + 1) % n;
    push(p[k], p[kn], endRing[kn]);
    push(p[k], endRing[kn], endRing[k]);
  }
  return endRing;
}

// Resolve the through-drill roles (tube vs pierced operand) shared by CUT/FUSE. Mirrors
// buildCommon's gate exactly; returns false for anything that is not a clean through-drill.
struct DrillRoles {
  const CurvedSolid* tube = nullptr;
  const CurvedSolid* pierced = nullptr;
  bool tubeIsA = false;
};
bool resolveRoles(const CurvedSolid& A, const CurvedSolid& B, const std::vector<Seam>& seams,
                  DrillRoles& out) {
  if (seams.size() != 2) return false;
  for (const Seam& s : seams)
    if (!s.closed || s.pts.size() < 4) return false;
  const bool aTube = isFullCircle(seams[0].uvA) && isFullCircle(seams[1].uvA);
  const bool bTube = isFullCircle(seams[0].uvB) && isFullCircle(seams[1].uvB);
  const bool aPierced = !isFullCircle(seams[0].uvA) && !isFullCircle(seams[1].uvA);
  const bool bPierced = !isFullCircle(seams[0].uvB) && !isFullCircle(seams[1].uvB);
  if (!((aTube && bPierced) || (bTube && aPierced))) return false;
  out.tubeIsA = aTube;
  out.tube = aTube ? &A : &B;
  out.pierced = aTube ? &B : &A;
  // Only cylinder∩cylinder is host-verifiable in S5-b; other kinds still decline → OCCT.
  if (out.tube->kind != CurvedKind::Cylinder || out.pierced->kind != CurvedKind::Cylinder)
    return false;
  return true;
}

// Wall grid resolution from the pierced radius + a fixed chord-sagitta target (NOT hand-
// tuned), matching appendMouthCap's kCapSagitta discipline. u is sampled finely around the
// FULL circle so each planar facet's bow off the true cylinder is bounded; v is sampled
// modestly (a cylinder is straight along v, so tall facets are exact) but fine enough that
// the mouth spans several v-cells (its annulus stitch stays ≤ one cell wide).
void wallSteps(const CurvedSolid& pierced, const std::vector<Seam>& seams, bool tubeIsA,
               int& uCells, int& vCells) {
  const double sagStep = std::sqrt(8.0 * kCapSagitta / std::max(pierced.radius, 1e-9));
  uCells = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi / std::max(sagStep, 1e-6))), 16, 512);
  // v-cell size: a fraction of the mouths' v-span so each mouth covers several cells.
  auto piercedV = [&](int i) -> const std::vector<std::pair<double, double>>& {
    return tubeIsA ? seams[i].uvB : seams[i].uvA;
  };
  double vLoM = 1e300, vHiM = -1e300;
  for (int i = 0; i < 2; ++i)
    for (const auto& p : piercedV(i)) { vLoM = std::min(vLoM, p.second); vHiM = std::max(vHiM, p.second); }
  const double mouthVspan = std::max(vHiM - vLoM, 1e-6);
  const double vCell = std::max(mouthVspan / 6.0, 1e-6);
  vCells = std::clamp(static_cast<int>(std::ceil((pierced.vHi - pierced.vLo) / vCell)), 4, 256);
}

// buildCut(A,B) = A − B for a through-drill pair: the pierced (fat) wall with the two
// mouth holes + its two disc caps + the INWARD tunnel wall (the thin tube band reversed).
topo::Shape buildCut(const CurvedSolid& A, const CurvedSolid& B, const std::vector<Seam>& seams) {
  DrillRoles roles;
  if (!resolveRoles(A, B, seams, roles)) return {};
  // CUT is A−B (A is the minuend). We build the PIERCED operand with a tunnel; that is only
  // A−B when the pierced operand IS A. If A is the piercing tube (thin−fat), the result is a
  // different topology (two capped end tubes) — decline → OCCT rather than emit the wrong shape.
  if (roles.tubeIsA) return {};
  const CurvedSolid& tube = *roles.tube;
  const CurvedSolid& pierced = *roles.pierced;
  auto tubeUV = [&](int i) -> const std::vector<std::pair<double, double>>& {
    return roles.tubeIsA ? seams[i].uvA : seams[i].uvB;
  };
  auto piercedUV = [&](int i) -> const std::vector<std::pair<double, double>>& {
    return roles.tubeIsA ? seams[i].uvB : seams[i].uvA;
  };
  // The tunnel must be INSIDE the pierced solid (same survival sample as COMMON).
  const double tubeMidV = 0.5 * (meanV(tubeUV(0)) + meanV(tubeUV(1)));
  if (classifyPoint(pierced, tube.point(meanU(tubeUV(0)), tubeMidV), kSsiTol) != 1) return {};

  int uSteps, vSteps;
  wallSteps(pierced, seams, roles.tubeIsA, uSteps, vSteps);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  std::vector<math::Point3> rimLo, rimHi;
  if (!appendHoledWall(pierced, seams[0], seams[1], piercedUV(0), piercedUV(1), uSteps, vSteps,
                       pool, faces, rimLo, rimHi))
    return {};
  // Fat disc caps at v=vLo (outward −axis) and v=vHi (outward +axis).
  appendDiskCap(pierced, pierced.vLo, rimLo, math::Vec3{-pierced.frame.z.vec().x,
               -pierced.frame.z.vec().y, -pierced.frame.z.vec().z}, pool, faces);
  appendDiskCap(pierced, pierced.vHi, rimHi, pierced.frame.z.vec(), pool, faces);
  // Tunnel wall: the thin tube band, INWARD-facing.
  appendOrientedTubeBand(tube, seams, tubeUV(0), tubeUV(1), /*outwardSign=*/-1.0, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildFuse(A,B) = A ∪ B for a through-drill pair: the pierced (fat) wall with the two
// mouth holes + its two disc caps + the two piercing END TUBES (thin wall outside the fat
// solid) + the two thin disc end caps. The tube band inside the fat solid is dropped.
topo::Shape buildFuse(const CurvedSolid& A, const CurvedSolid& B, const std::vector<Seam>& seams) {
  DrillRoles roles;
  if (!resolveRoles(A, B, seams, roles)) return {};
  const CurvedSolid& tube = *roles.tube;
  const CurvedSolid& pierced = *roles.pierced;
  auto tubeUV = [&](int i) -> const std::vector<std::pair<double, double>>& {
    return roles.tubeIsA ? seams[i].uvA : seams[i].uvB;
  };
  auto piercedUV = [&](int i) -> const std::vector<std::pair<double, double>>& {
    return roles.tubeIsA ? seams[i].uvB : seams[i].uvA;
  };
  const double tubeMidV = 0.5 * (meanV(tubeUV(0)) + meanV(tubeUV(1)));
  if (classifyPoint(pierced, tube.point(meanU(tubeUV(0)), tubeMidV), kSsiTol) != 1) return {};

  int uSteps, vSteps;
  wallSteps(pierced, seams, roles.tubeIsA, uSteps, vSteps);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  std::vector<math::Point3> rimLo, rimHi;
  if (!appendHoledWall(pierced, seams[0], seams[1], piercedUV(0), piercedUV(1), uSteps, vSteps,
                       pool, faces, rimLo, rimHi))
    return {};
  appendDiskCap(pierced, pierced.vLo, rimLo, math::Vec3{-pierced.frame.z.vec().x,
               -pierced.frame.z.vec().y, -pierced.frame.z.vec().z}, pool, faces);
  appendDiskCap(pierced, pierced.vHi, rimHi, pierced.frame.z.vec(), pool, faces);
  // Each seam's end tube runs out to the NEARER thin end cap (v=vLo or v=vHi of the tube).
  for (int i = 0; i < 2; ++i) {
    const double seamV = meanV(tubeUV(i));
    const double vEnd = (std::fabs(seamV - tube.vLo) < std::fabs(seamV - tube.vHi)) ? tube.vLo
                                                                                   : tube.vHi;
    const std::vector<math::Point3> ring =
        appendEndTube(tube, seams[i], tubeUV(i), vEnd, pool, faces);
    if (ring.empty()) return {};
    const math::Vec3 capN = (vEnd == tube.vLo)
        ? math::Vec3{-tube.frame.z.vec().x, -tube.frame.z.vec().y, -tube.frame.z.vec().z}
        : tube.frame.z.vec();
    appendDiskCap(tube, vEnd, ring, capN, pool, faces);
  }
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// S5-c — sphere∩sphere COMMON (single-seam / two-spherical-cap assembler).
//
// Two overlapping spheres trace as ONE closed seam circle (recognition + S3 tracing
// already deliver it; only buildCommon's seams.size()!=2 guard blocked this pair). The
// COMMON is the LENS bounded by the two spherical caps — one from each sphere — that lie
// INSIDE the other sphere, sharing the single seam circle. Each cap is the spherical
// patch from its apex (the sphere's surface point nearest the OTHER centre) out to the
// seam. We keep a cap iff its apex classifies INSIDE the other solid (the COMMON survival
// rule); a tangent/ON apex → NULL → OCCT. Both caps' OUTER rings are the SAME shared
// pooled seam vertices → they weld watertight along the one seam. Every cap facet is a
// PLANAR triangle through sphere-surface points (the S5-a watertight discipline — the
// analytic sphere face's structured-grid mesh would inject interior u-lines and open a
// T-junction against the neighbour cap's fan; planar facets carry no interior points).
// ─────────────────────────────────────────────────────────────────────────────

// SLERP two unit vectors by fraction t (great-circle interpolation on the unit sphere);
// falls back to the linear-normalised blend when they are near-parallel (small angle).
math::Vec3 slerpDir(const math::Vec3& a, const math::Vec3& b, double t) {
  const double c = std::clamp(math::dot(a, b), -1.0, 1.0);
  const double ang = std::acos(c);
  if (ang < 1e-6) {
    const math::Vec3 m{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    const double L = std::max(math::norm(m), 1e-12);
    return math::Vec3{m.x / L, m.y / L, m.z / L};
  }
  const double s = std::sin(ang);
  const double wa = std::sin((1.0 - t) * ang) / s, wb = std::sin(t * ang) / s;
  return math::Vec3{a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb};
}

// A spherical cap of `sph` bounded by the shared seam loop: a radial fan from the cap
// APEX through `rings` concentric rings out to the exact traced seam nodes. Interior
// ring/apex points are placed ON the sphere by SLERPing the unit radial direction from the
// apex direction to each seam node's radial direction (great-circle interpolation —
// geometrically exact on the sphere and free of the (u,v) parametric-pole singularity, so
// it is robust even when the apex sits at the sphere's pole). The cap follows the true
// spherical bulge to O(1/rings²); the OUTER ring is the shared pooled seam nodes so the
// two caps weld along the one seam.
//
// APEX SELECTION (`outer`):
//   * outer=false (INNER cap, COMMON default): apex = surface point NEAREST otherCentre
//     (centre + R·unit(centre→other)) — the cap that bulges into the other solid (lens).
//   * outer=true (OUTER cap, FUSE / CUT-minuend): apex = FAR pole
//     (centre − R·unit(centre→other)) — the cap that bulges away from the other solid.
// NORMAL ORIENTATION (`reversed`):
//   * reversed=false (default): every facet is oriented with the sphere's OUTWARD radial
//     normal (the lens/fuse boundary's outward normal there).
//   * reversed=true (CUT inner-cap-of-B): facet normal points INWARD (centre−centroid) so
//     the cap correctly bounds the scooped cavity of A−B.
// The ring/slerp/fan loop and the `r==rings → seam.pts[k]` outer ring are identical across
// all four (outer,reversed) modes — only the apex direction and the outward reference flip;
// the COMMON caller uses the (false,false) defaults so its output is byte-identical.
//
// systems-band (~14 — radial ring slerp + apex/orientation selection); flagged.
void appendSphereCap(const CurvedSolid& sph, const math::Point3& otherCentre, const Seam& seam,
                     int rings, VertexPool& pool, std::vector<topo::Shape>& faces,
                     bool outer = false, bool reversed = false) {
  const int n = static_cast<int>(seam.pts.size());
  if (n < 3 || rings < 1) return;
  // Apex = surface point of `sph` nearest `otherCentre` (inner) or the far pole (outer).
  const math::Vec3 toOther = otherCentre - sph.frame.origin;
  const double dToOther = math::norm(toOther);
  if (dToOther < 1e-12) return;
  const double sgn = outer ? -1.0 : 1.0;
  const math::Vec3 apexDir{sgn * toOther.x / dToOther, sgn * toOther.y / dToOther,
                           sgn * toOther.z / dToOther};
  const math::Point3 apex{sph.frame.origin.x + apexDir.x * sph.radius,
                          sph.frame.origin.y + apexDir.y * sph.radius,
                          sph.frame.origin.z + apexDir.z * sph.radius};
  // Unit radial direction of each seam node (from the sphere centre).
  std::vector<math::Vec3> seamDir(n);
  for (int k = 0; k < n; ++k) {
    const math::Vec3 w = seam.pts[k] - sph.frame.origin;
    const double L = std::max(math::norm(w), 1e-12);
    seamDir[k] = math::Vec3{w.x / L, w.y / L, w.z / L};
  }
  // Ring r (1..rings) node k: slerp(apexDir → seamDir[k], r/rings) · R, centred at origin.
  auto ringPt = [&](int r, int k) -> math::Point3 {
    if (r == rings) return seam.pts[k];  // outer ring = exact traced seam node
    const math::Vec3 d = slerpDir(apexDir, seamDir[k], static_cast<double>(r) / rings);
    return math::Point3{sph.frame.origin.x + d.x * sph.radius, sph.frame.origin.y + d.y * sph.radius,
                        sph.frame.origin.z + d.z * sph.radius};
  };
  auto tri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    const math::Point3 ctr{(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
    // Reference = sphere radial normal at the centroid; OUTWARD (ctr−centre) for the lens/fuse
    // boundary, INWARD (centre−ctr) for the reversed CUT cavity cap.
    const math::Vec3 radial = ctr - sph.frame.origin;
    const math::Vec3 out = reversed ? math::Vec3{-radial.x, -radial.y, -radial.z} : radial;
    pushPlanarTri(a, b, c, out, pool, faces);
  };
  for (int k = 0; k < n; ++k) tri(apex, ringPt(1, k), ringPt(1, (k + 1) % n));
  for (int r = 2; r <= rings; ++r)
    for (int k = 0; k < n; ++k) {
      const int kn = (k + 1) % n;
      tri(ringPt(r - 1, k), ringPt(r, k), ringPt(r, kn));
      tri(ringPt(r - 1, k), ringPt(r, kn), ringPt(r - 1, kn));
    }
}

// Target seam node count for a lens cap: the seam is a circle of radius ρ (its node
// distance to the seam-plane axis); a chord subtending arc s bows s²/(8ρ) off the circle,
// so keeping that ≤ kCapSagitta gives nSeam ≈ 2πρ / sqrt(8·kCapSagitta·ρ). Bounded [24,180]
// so the facet count stays tractable while the seam-chord error stays negligible vs the 1%
// volume bar. Uses the seam's own 3D radius (centroid → node distance) — not hand-tuned.
int seamNodeTarget(const Seam& seam) {
  math::Point3 c{0, 0, 0};
  for (const auto& p : seam.pts) { c.x += p.x; c.y += p.y; c.z += p.z; }
  const double n = static_cast<double>(seam.pts.size());
  c.x /= n; c.y /= n; c.z /= n;
  double rho = 0.0;
  for (const auto& p : seam.pts) rho += math::norm(p - c);
  rho /= n;
  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * rho, 1e-12));
  const int target = static_cast<int>(std::ceil(kSsiTwoPi * rho / std::max(chord, 1e-9)));
  return std::clamp(target, 24, 180);
}

// Decimate a closed seam to (about) `target` evenly-strided nodes, keeping the 3D points
// and BOTH (u,v) tracks in lockstep so a cap built from either track welds on the shared
// 3D points. If already ≤ target, returned unchanged.
Seam decimateSeam(const Seam& seam, int target) {
  const int n = static_cast<int>(seam.pts.size());
  if (n <= target || target < 3) return seam;
  Seam out;
  out.closed = seam.closed;
  const double stride = static_cast<double>(n) / target;
  for (int k = 0; k < target; ++k) {
    const int i = static_cast<int>(std::floor(k * stride));
    out.pts.push_back(seam.pts[i]);
    out.uvA.push_back(seam.uvA[i]);
    out.uvB.push_back(seam.uvB[i]);
  }
  return out;
}

// buildLensCommon(A,B) = the COMMON of two overlapping SPHERES: the lens bounded by the
// two inside-the-other spherical caps sharing the single seam circle. Taken ONLY when the
// trace is ONE closed seam and both operands are Sphere. Declines (→ OCCT) if either apex
// is not strictly INSIDE the other sphere (tangent/degenerate) — never faked.
topo::Shape buildLensCommon(const CurvedSolid& A, const CurvedSolid& B,
                            const std::vector<Seam>& seams) {
  if (seams.size() != 1) return {};
  const Seam& seam = seams[0];
  if (!seam.closed || seam.pts.size() < 4) return {};
  if (A.kind != CurvedKind::Sphere || B.kind != CurvedKind::Sphere) return {};

  // Each cap's apex = the sphere's surface point nearest the other centre. Keep both only
  // if each apex is strictly INSIDE the other solid (the COMMON survival rule).
  const math::Vec3 ab = B.frame.origin - A.frame.origin;
  const double d = math::norm(ab);
  if (d < 1e-9) return {};  // concentric → not a transversal lens
  const math::Vec3 abU{ab.x / d, ab.y / d, ab.z / d};
  const math::Point3 apexA{A.frame.origin.x + abU.x * A.radius, A.frame.origin.y + abU.y * A.radius,
                           A.frame.origin.z + abU.z * A.radius};
  const math::Point3 apexB{B.frame.origin.x - abU.x * B.radius, B.frame.origin.y - abU.y * B.radius,
                           B.frame.origin.z - abU.z * B.radius};
  if (classifyPoint(B, apexA, kSsiTol) != 1) return {};  // ON/outside → tangent → OCCT
  if (classifyPoint(A, apexB, kSsiTol) != 1) return {};

  // Decimate the (densely-traced) seam to a shared node subset used by BOTH caps, so the
  // two caps' outer rings are the SAME pooled vertices → weld. The subset keeps the seam's
  // chord bow within kCapSagitta of the true circle (radius ρ ≈ seam-node distance to the
  // seam-plane axis): nSeam ≈ 2π·ρ / sqrt(8·kCapSagitta·ρ). Bounded so the facet count stays
  // tractable while the seam-chord error stays negligible vs the 1% volume bar.
  const Seam capSeam = decimateSeam(seam, seamNodeTarget(seam));
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // Ring count from each cap's TRUE polar half-angle θ (angle at the sphere centre between
  // the apex direction and the seam) + the radial curvature-per-facet target kCapSagitta
  // (NOT hand-tuned), matching appendMouthCap's discipline. rings ≈ θ·sqrt(R/(2·kCapSagitta));
  // bounded so the cap facet count stays tractable (the residual radial sagitta at the cap =
  // R(θ/rings)²/2 stays well under the engine's 1% volume bar).
  auto ringsFor = [&](const CurvedSolid& sph, const math::Point3& apex) {
    const math::Vec3 aDir = apex - sph.frame.origin;  // apex radial (length R)
    // θ = MAX polar angle at the centre between the apex and any seam node (the cap's polar
    // half-angle). The seam-centroid collapses onto the axis for a full circle, so we take a
    // node's radial, not the mean.
    double theta = 0.0;
    for (const auto& p : capSeam.pts) {
      const math::Vec3 sDir{p.x - sph.frame.origin.x, p.y - sph.frame.origin.y,
                            p.z - sph.frame.origin.z};
      const double denom = std::max(math::norm(aDir) * math::norm(sDir), 1e-12);
      theta = std::max(theta, std::acos(std::clamp(math::dot(aDir, sDir) / denom, -1.0, 1.0)));
    }
    const double rings = std::max(theta, 1e-6) * std::sqrt(sph.radius / (2.0 * kCapSagitta));
    return std::clamp(static_cast<int>(std::ceil(rings)), 4, 48);
  };
  appendSphereCap(A, B.frame.origin, capSeam, ringsFor(A, apexA), pool, faces);
  appendSphereCap(B, A.frame.origin, capSeam, ringsFor(B, apexB), pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── S5-e — coaxial CONE(frustum)∩CYLINDER COMMON ───────────────────────────────
// A REVOLVED band strip between two same-azimuth rim rings (`lo`,`hi`), planar-facet,
// outward radial normal. Both rings carry the SAME azimuth samples (index i ↔ i), so
// each ruling (lo[i]→hi[i]) lies exactly on the swept wall (a cone ruling or a cylinder
// generatrix — both straight in the axial direction), and the only approximation is the
// N-gon chord around the circle (shared by every ring → welds watertight through `pool`).
// `outwardSign` (default +1) flips the facet normal reference: +1 = radial-out-of-the-axis
// (an outer boss wall — COMMON / FUSE), −1 = radial-in (the inward wall of a carved cavity,
// material on the outer side — the CUT's reversed inside fragment). Mirrors the outwardSign
// convention of appendOrientedTubeBand / buildSteinmetzCut.
void appendRevolvedBand(const std::vector<math::Point3>& lo, const std::vector<math::Point3>& hi,
                        const math::Point3& axisOrigin, const math::Vec3& axisDir,
                        VertexPool& pool, std::vector<topo::Shape>& faces,
                        double outwardSign = 1.0) {
  const int n = static_cast<int>(lo.size());
  if (n < 3 || static_cast<int>(hi.size()) != n) return;
  auto radial = [&](const math::Point3& p) {
    const math::Vec3 w{p.x - axisOrigin.x, p.y - axisOrigin.y, p.z - axisOrigin.z};
    return w - axisDir * math::dot(w, axisDir);
  };
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const math::Point3 ctr{(lo[i].x + lo[j].x + hi[j].x + hi[i].x) / 4,
                           (lo[i].y + lo[j].y + hi[j].y + hi[i].y) / 4,
                           (lo[i].z + lo[j].z + hi[j].z + hi[i].z) / 4};
    const math::Vec3 r = radial(ctr);
    const math::Vec3 ref{r.x * outwardSign, r.y * outwardSign, r.z * outwardSign};
    pushPlanarTri(lo[i], lo[j], hi[j], ref, pool, faces);
    pushPlanarTri(lo[i], hi[j], hi[i], ref, pool, faces);
  }
}

// ── FLAT ANNULUS CAP: a washer between two coaxial SAME-STATION rings (an inner ring and
// an outer ring at identical axial s), planar facets, axial normal (`axialNormal` = ±ẑ).
// Mirrors appendRevolvedBand's quad triangulation with the normal forced axial instead of
// radial — it closes an end-cap RADIAL STEP that appendDiskCap (a full fan from the axis)
// cannot express: FUSE's operand-step caps and CUT's washer top cap. Both rings share the
// wall rows through the pool, so the cap welds without a T-junction.
void appendAnnulusCap(const std::vector<math::Point3>& inner,
                      const std::vector<math::Point3>& outer, const math::Vec3& axialNormal,
                      VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int n = static_cast<int>(inner.size());
  if (n < 3 || static_cast<int>(outer.size()) != n) return;
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    pushPlanarTri(inner[i], inner[j], outer[j], axialNormal, pool, faces);
    pushPlanarTri(inner[i], outer[j], outer[i], axialNormal, pool, faces);
  }
}

// ── ConeCylSetup — the SHARED prologue of the three coaxial cone(frustum)∩cylinder
// assemblers (COMMON / FUSE / CUT). It recognises the ONE Cone + ONE Cylinder coaxial
// pair, folds the cylinder extent into the cone's s-coordinate (s = axial projection onto
// the cone axis from the cone origin), locates the single analytic SSI circle at s*
// (r_cone(s*) = Rc) and CROSS-CHECKS it against the S3-traced seam (height + radius), and
// exposes the shared azimuth frame + the ring(r,s) sampler + r_cone(s). Factored out of
// buildConeCylCommon so all three ops reuse the IDENTICAL seam + split machinery and differ
// only in WHICH fragments survive + the cap handling — COMMON stays byte-identical.
struct ConeCylSetup {
  bool ok = false;
  const CurvedSolid* cone = nullptr;
  const CurvedSolid* cyl = nullptr;
  math::Point3 O;
  math::Vec3 X, Y, zc;
  double tanA = 0.0, Rc = 0.0;
  double coneS0 = 0.0, coneS1 = 0.0;  ///< cone's own s-extent [vLo,vHi]
  double cylS0 = 0.0, cylS1 = 0.0;    ///< cylinder extent in the cone's s-coordinate
  double sLo = 0.0, sHi = 0.0;        ///< axial overlap
  double sStar = 0.0;                 ///< single interior crossing r_cone(s*)=Rc
  int N = 0;                          ///< azimuth sample count (seam-chord bounded)
  double rCone(double s) const { return cone->radius + s * tanA; }
  std::vector<math::Point3> ring(double r, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = r * std::cos(u), cy = r * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  /// A world point on a wall at (radius r, station s), azimuth 0 — the classifier sample.
  math::Point3 wallPoint(double r, double s) const {
    return math::Point3{O.x + X.x * r + zc.x * s, O.y + X.y * r + zc.y * s,
                        O.z + X.z * r + zc.z * s};
  }
};

ConeCylSetup coneCylSetup(const CurvedSolid& A, const CurvedSolid& B,
                          const std::vector<Seam>& seams) {
  ConeCylSetup st;
  if (seams.size() != 1) return st;
  const Seam& seam = seams[0];
  if (!seam.closed || seam.pts.size() < 8) return st;

  // Exactly one Cone + one Cylinder operand.
  const CurvedSolid* conePtr = nullptr;
  const CurvedSolid* cylPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Cone) conePtr = s;
    else if (s->kind == CurvedKind::Cylinder) cylPtr = s;
  }
  if (!conePtr || !cylPtr) return st;
  const CurvedSolid& cone = *conePtr;
  const CurvedSolid& cyl = *cylPtr;
  if (!ssidetail::sameAxis(cone.frame, cyl.frame, 1e-6)) return st;  // must be coaxial

  const math::Vec3 zc = cone.frame.z.vec();
  const double tanA = std::tan(cone.semiAngle);
  if (std::fabs(tanA) < 1e-9) return st;  // degenerate (near-cylindrical) cone
  const double Rc = cyl.radius;
  const math::Point3 O = cone.frame.origin;
  auto sOf = [&](const math::Point3& p) {
    return math::dot(math::Vec3{p.x - O.x, p.y - O.y, p.z - O.z}, zc);
  };

  // Cylinder axial extent expressed in the cone's s-coordinate (axes may be antiparallel).
  const double base = math::dot(cyl.frame.origin - O, zc);
  const double sign = math::dot(cyl.frame.z.vec(), zc) >= 0.0 ? 1.0 : -1.0;
  double cylSLo = base + sign * cyl.vLo, cylSHi = base + sign * cyl.vHi;
  if (cylSLo > cylSHi) std::swap(cylSLo, cylSHi);
  const double sLo = std::max(cone.vLo, cylSLo);
  const double sHi = std::min(cone.vHi, cylSHi);
  if (!(sHi - sLo > 1e-6)) return st;  // no axial overlap

  // Single interior crossing r_cone(s*) = Rc.
  const double sStar = (Rc - cone.radius) / tanA;
  if (!(sStar - sLo > 1e-6) || !(sHi - sStar > 1e-6)) return st;  // apex/edge crossing → decline

  // Cross-check the analytic seam against the traced seam (height + radius).
  math::Point3 c{0, 0, 0};
  for (const auto& p : seam.pts) { c.x += p.x; c.y += p.y; c.z += p.z; }
  const double ns = static_cast<double>(seam.pts.size());
  c.x /= ns; c.y /= ns; c.z /= ns;
  if (std::fabs(sOf(c) - sStar) > 1e-4) return st;  // seam not at the analytic height
  double rho = 0.0;
  for (const auto& p : seam.pts) {
    const math::Vec3 w{p.x - c.x, p.y - c.y, p.z - c.z};
    rho += math::norm(w - zc * math::dot(w, zc));
  }
  rho /= ns;
  if (std::fabs(rho - Rc) > 1e-3) return st;  // seam radius ≠ Rc

  // Azimuthal resolution: seam-circle chord sagitta ≤ kCapSagitta (same bound as the caps).
  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * Rc, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * Rc / chord)), 24, 180);
  st.cone = conePtr;
  st.cyl = cylPtr;
  st.O = O;
  st.X = cone.frame.x.vec();
  st.Y = cone.frame.y.vec();
  st.zc = zc;
  st.tanA = tanA;
  st.Rc = Rc;
  st.coneS0 = cone.vLo;
  st.coneS1 = cone.vHi;
  st.cylS0 = cylSLo;
  st.cylS1 = cylSHi;
  st.sLo = sLo;
  st.sHi = sHi;
  st.sStar = sStar;
  st.ok = true;
  return st;
}

// buildConeCylCommon(A,B) = the COMMON of a COAXIAL cone frustum and a cylinder. The
// overlap is the solid of revolution of r ≤ min(r_cone(s), Rc) over the shared axial
// span [sLo,sHi]. r_cone(s) crosses the constant Rc EXACTLY ONCE — the single analytic SSI
// circle at s* (radius Rc) — so the shell is: bottom disc @ sLo, the inner-wall band below
// s*, the shared SEAM ring @ s*, the inner-wall band above s*, top disc @ sHi. Both bands
// and the caps share the seam/rim rings through one VertexPool → watertight. Taken ONLY for
// one Cone + one Cylinder operand, coaxial, with the traced seam matching the analytic
// circle and a single STRICTLY-INTERIOR crossing; anything else → {} → OCCT.
topo::Shape buildConeCylCommon(const CurvedSolid& A, const CurvedSolid& B,
                               const std::vector<Seam>& seams) {
  const ConeCylSetup s = coneCylSetup(A, B, seams);
  if (!s.ok) return {};

  // Inner-wall radii at the span ends (min of the two walls; linear on each sub-span).
  const double rBot = std::min(s.rCone(s.sLo), s.Rc);
  const double rTop = std::min(s.rCone(s.sHi), s.Rc);
  if (!(rBot > 1e-9) || !(rTop > 1e-9)) return {};  // apex-touching rim → decline

  const std::vector<math::Point3> ringBot = s.ring(rBot, s.sLo);
  const std::vector<math::Point3> ringSeam = s.ring(s.Rc, s.sStar);
  const std::vector<math::Point3> ringTop = s.ring(rTop, s.sHi);

  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendRevolvedBand(ringBot, ringSeam, s.O, s.zc, pool, faces);   // inner wall below s*
  appendRevolvedBand(ringSeam, ringTop, s.O, s.zc, pool, faces);   // inner wall above s*
  appendDiskCap(*s.cone, s.sLo, ringBot, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendDiskCap(*s.cone, s.sHi, ringTop, math::Vec3{s.zc.x, s.zc.y, s.zc.z}, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildConeCylFuse(A,B) = A ∪ B of the coaxial cone∩cylinder pair — the OUTER wall regions
// of both operands + the operand caps that bound the union, welded at the seam circle.
// The union is the solid of revolution of r ≤ R_out(s) = max(r_cone(s), Rc over its extent)
// over the union span [sMin,sMax]. We walk the OUTER profile bottom→top as a corner list
// (s,r): a different-s pair is a revolved wall band (the wider operand's wall, kept iff its
// mid classifies OUTSIDE the other operand); a same-s pair is a radial STEP where one
// operand begins/ends → a flat annulus cap. Two terminal disc caps close the ends. All
// rings are shared through one pool → watertight. Volume = V(A)+V(B)−V(A∩B) (a GROW).
topo::Shape buildConeCylFuse(const CurvedSolid& A, const CurvedSolid& B,
                             const std::vector<Seam>& seams) {
  const ConeCylSetup s = coneCylSetup(A, B, seams);
  if (!s.ok) return {};
  const double sMin = std::min(s.coneS0, s.cylS0);
  const double sMax = std::max(s.coneS1, s.cylS1);
  const double span = sMax - sMin;
  if (!(span > 1e-9)) return {};
  const double eps = 1e-6 * std::max(span, 1.0);
  auto coneR = [&](double t) { return (t > s.coneS0 - eps && t < s.coneS1 + eps) ? s.rCone(t) : -1e300; };
  auto cylR = [&](double t) { return (t > s.cylS0 - eps && t < s.cylS1 + eps) ? s.Rc : -1e300; };
  auto Rmax = [&](double t) { return std::max(coneR(t), cylR(t)); };

  // Interior transition stations (operand ends + the seam), sorted + de-duplicated.
  std::vector<double> stn = {s.coneS0, s.coneS1, s.cylS0, s.cylS1, s.sStar};
  std::sort(stn.begin(), stn.end());
  stn.erase(std::unique(stn.begin(), stn.end(),
                        [&](double a2, double b2) { return std::fabs(a2 - b2) < eps; }),
            stn.end());

  // Outer-profile corners (s,r) bottom→top; a same-s pair encodes a radial step.
  std::vector<std::pair<double, double>> corners;
  corners.push_back({sMin, Rmax(sMin + eps)});
  for (double sc : stn) {
    if (sc <= sMin + eps || sc >= sMax - eps) continue;  // interior stations only
    const double rBelow = Rmax(sc - eps), rAbove = Rmax(sc + eps);
    corners.push_back({sc, rBelow});
    if (std::fabs(rAbove - rBelow) > eps) corners.push_back({sc, rAbove});  // radial step
  }
  corners.push_back({sMax, Rmax(sMax - eps)});
  if (!(corners.front().second > 1e-9) || !(corners.back().second > 1e-9)) return {};  // apex terminal

  VertexPool pool;
  std::vector<topo::Shape> faces;
  std::vector<math::Point3> prevRing = s.ring(corners.front().second, corners.front().first);
  appendDiskCap(*s.cone, corners.front().first, prevRing,
                math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);  // bottom terminal disc
  for (std::size_t k = 1; k < corners.size(); ++k) {
    const double sp = corners[k - 1].first, rp = corners[k - 1].second;
    const double sc = corners[k].first, rc = corners[k].second;
    std::vector<math::Point3> curRing = s.ring(rc, sc);
    if (std::fabs(sc - sp) < eps) {
      // Radial step: an operand begins (r grows going up → cap faces −ẑ) or ends (r
      // shrinks → cap faces +ẑ). The material is on the wider-radius/present side.
      const math::Vec3 axial = (rc > rp) ? math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z} : s.zc;
      appendAnnulusCap(prevRing, curRing, axial, pool, faces);
    } else {
      // Wall band of the wider operand; survival gate: its mid must lie OUTSIDE the other.
      const double midS = 0.5 * (sp + sc);
      const bool bandIsCone = coneR(midS) >= cylR(midS);
      const double rMid = std::max(coneR(midS), cylR(midS));
      const CurvedSolid& other = bandIsCone ? *s.cyl : *s.cone;
      if (classifyPoint(other, s.wallPoint(rMid, midS), kSsiTol) != -1) return {};  // tangent → OCCT
      appendRevolvedBand(prevRing, curRing, s.O, s.zc, pool, faces);
    }
    prevRing = std::move(curRing);
  }
  appendDiskCap(*s.cone, corners.back().first, prevRing, s.zc, pool, faces);  // top terminal disc
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── CUT washer: the conical annulus where the cone lies OUTSIDE the cylinder over the
// kept sub-span (the r_cone > Rc side of the seam). Its outer boundary is the cone wall
// (outward) and its inner boundary is the cylinder wall REVERSED (inward — the wall of the
// carved cavity, B's inside-A fragment). The two walls PINCH to the shared seam ring at s*
// (where r_cone = Rc) and are closed at the cone-end station by a flat annulus cap. `capS`
// is the cone-end station, `capN` its outward axial normal. Returns false on a tangent/
// mis-classified sample → NULL → OCCT (never faked).
bool emitConeCylWasher(const ConeCylSetup& s, const std::vector<math::Point3>& seamRing,
                       double capS, const math::Vec3& capN, VertexPool& pool,
                       std::vector<topo::Shape>& faces) {
  const std::vector<math::Point3> ringConeCap = s.ring(s.rCone(capS), capS);
  const std::vector<math::Point3> ringCylCap = s.ring(s.Rc, capS);
  const double midS = 0.5 * (s.sStar + capS);
  if (classifyPoint(*s.cyl, s.wallPoint(s.rCone(midS), midS), kSsiTol) != -1) return false;  // cone wall outside cyl
  if (classifyPoint(*s.cone, s.wallPoint(s.Rc, midS), kSsiTol) != 1) return false;           // cyl wall inside cone
  appendRevolvedBand(seamRing, ringConeCap, s.O, s.zc, pool, faces, 1.0);   // outer cone wall
  appendRevolvedBand(seamRing, ringCylCap, s.O, s.zc, pool, faces, -1.0);   // reversed inner cyl wall
  appendAnnulusCap(ringCylCap, ringConeCap, capN, pool, faces);            // cone-end annulus cap
  return true;
}

// ── CUT cone slice: a FULL cone frustum slice [between sTerm and sCut] where the cylinder
// is absent (cone entirely OUTSIDE B) — a separate closed component of A−B. `sTerm` is the
// real cone end (terminal disc, outward normal `termN`); `sCut` is the interface station
// with the carved region (a flat cut disc facing the removed material, normal −termN).
bool emitConeSlice(const ConeCylSetup& s, double sTerm, const math::Vec3& termN, double sCut,
                   VertexPool& pool, std::vector<topo::Shape>& faces) {
  const std::vector<math::Point3> ringTerm = s.ring(s.rCone(sTerm), sTerm);
  const std::vector<math::Point3> ringCut = s.ring(s.rCone(sCut), sCut);
  const double midS = 0.5 * (sTerm + sCut);
  if (classifyPoint(*s.cyl, s.wallPoint(s.rCone(midS), midS), kSsiTol) != -1) return false;  // outside cyl
  if (!(s.rCone(sTerm) > 1e-9) || !(s.rCone(sCut) > 1e-9)) return false;                      // apex rim
  appendDiskCap(*s.cone, sTerm, ringTerm, termN, pool, faces);
  appendRevolvedBand(ringTerm, ringCut, s.O, s.zc, pool, faces);
  appendDiskCap(*s.cone, sCut, ringCut, math::Vec3{-termN.x, -termN.y, -termN.z}, pool, faces);
  return true;
}

// buildConeCylCut(A,B) = A − B of the coaxial cone∩cylinder pair (A = cone MINUEND;
// order-sensitive, matching BRepAlgoAPI_Cut(a,b)). The result = A's outer wall + A's caps +
// the cylinder's inside-A fragment emitted REVERSED (inward, bounding the carved cavity).
// Geometrically A−B keeps the cone where r_cone > Rc (the conical WASHER pinching to the
// seam and capped at the cone end) PLUS any cone-only slice where the cylinder is absent —
// a DISCONNECTED second component, assembled into one shell of two closed components sharing
// the pool. Volume = V(A)−V(A∩B) (a SHRINK). Only the clean single-sided crossing where the
// washer terminates at a CONE cap is built; a cone longer than the cylinder on the kept side
// (a connected/merged topology) or a cylinder minuend declines → OCCT (never faked).
topo::Shape buildConeCylCut(const CurvedSolid& A, const CurvedSolid& B,
                            const std::vector<Seam>& seams) {
  const ConeCylSetup s = coneCylSetup(A, B, seams);
  if (!s.ok) return {};
  if (&A != s.cone) return {};  // A must be the cone minuend; cylinder−cone → OCCT
  const double eps = 1e-6 * std::max(s.coneS1 - s.coneS0, 1.0);
  const bool grows = s.rCone(s.sHi) > s.Rc + eps;    // cone wider ABOVE the seam
  const bool shrinks = s.rCone(s.sLo) > s.Rc + eps;  // cone wider BELOW the seam
  if (grows == shrinks) return {};                   // not a clean single-sided crossing → OCCT

  const std::vector<math::Point3> ringSeam = s.ring(s.Rc, s.sStar);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  if (grows) {
    if (s.coneS1 > s.cylS1 + eps) return {};  // cone outlives cyl on top → connected → OCCT
    if (!emitConeCylWasher(s, ringSeam, s.coneS1, s.zc, pool, faces)) return {};
    if (s.coneS0 < s.cylS0 - eps &&  // cone protrudes below cyl → detached tip component
        !emitConeSlice(s, s.coneS0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, s.cylS0, pool, faces))
      return {};
  } else {  // shrinks (cone narrows through the seam)
    if (s.coneS0 < s.cylS0 - eps) return {};  // cone outlives cyl on the bottom → connected → OCCT
    if (!emitConeCylWasher(s, ringSeam, s.coneS0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces))
      return {};
    if (s.coneS1 > s.cylS1 + eps &&  // cone protrudes above cyl → detached component
        !emitConeSlice(s, s.coneS1, s.zc, s.cylS1, pool, faces))
      return {};
  }
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── S5-g — coaxial CONE(frustum)∩CONE(frustum) COMMON / FUSE / CUT ─────────────
// Two COAXIAL cone frustums (same axis) whose walls r_A(s)=R0_A+s·tanα_A and
// r_B(s)=R0_B+s·tanα_B (s = axial projection onto the SHARED axis from cone A's origin)
// meet along ONE analytic circle where r_A(s)=r_B(s) — a SINGLE LINEAR equation, so exactly
// one crossing s* (unless the walls are parallel, tanα_A==tanα_B → no proper transversal
// circle → decline). This is the natural generalisation of the S5-e cone∩cylinder pair
// (cylinder = the tanα_B==0 special case): the constant cylinder radius Rc becomes the
// linear r_B(s). All the S5-e revolved-band + disc-cap + annulus-cap + washer machinery is
// REUSED verbatim (appendRevolvedBand / appendDiskCap / appendAnnulusCap); only the radius
// profile of the second operand changes from a constant to a line. COMMON/FUSE/CUT are the
// min-profile / max-profile / minuend-outer-with-reversed-inner solids of revolution.
//
// HONEST SCOPE. Coaxial only (parallel colinear axes). A transversal (non-coaxial) cone∩cone
// pair is a quartic space curve — out of scope → decline → OCCT. An apex-through-apex or
// apex-in-extent pair (the seam passing through either cone's apex, r→0), a tangent/parallel-
// wall pair, and a seam not strictly interior to BOTH frustum extents all decline → OCCT.
struct ConeConeSetup {
  bool ok = false;
  const CurvedSolid* coneA = nullptr;  ///< the A operand (minuend for CUT)
  const CurvedSolid* coneB = nullptr;
  math::Point3 O;                      ///< cone-A origin (the shared s=0 station)
  math::Vec3 X, Y, zc;                 ///< cone-A frame (azimuth + axis)
  double tanA = 0.0, R0A = 0.0;        ///< r_A(s)=R0A + s·tanA
  double tanB = 0.0, R0B = 0.0;        ///< r_B(s)=R0B + s·tanB (both in cone-A's s-frame)
  double aS0 = 0.0, aS1 = 0.0;         ///< cone A s-extent [vLo,vHi]
  double bS0 = 0.0, bS1 = 0.0;         ///< cone B extent in cone-A's s-coordinate
  double sLo = 0.0, sHi = 0.0;         ///< axial overlap
  double sStar = 0.0;                  ///< single interior crossing r_A(s*)=r_B(s*)
  double rStar = 0.0;                  ///< seam radius r_A(s*)
  int N = 0;                           ///< azimuth sample count (seam-chord bounded)
  double rA(double s) const { return R0A + s * tanA; }
  double rB(double s) const { return R0B + s * tanB; }
  std::vector<math::Point3> ring(double r, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = r * std::cos(u), cy = r * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  math::Point3 wallPoint(double r, double s) const {
    return math::Point3{O.x + X.x * r + zc.x * s, O.y + X.y * r + zc.y * s,
                        O.z + X.z * r + zc.z * s};
  }
};

ConeConeSetup coneConeSetup(const CurvedSolid& A, const CurvedSolid& B,
                            const std::vector<Seam>& seams) {
  ConeConeSetup st;
  if (seams.size() != 1) return st;
  const Seam& seam = seams[0];
  if (!seam.closed || seam.pts.size() < 8) return st;
  if (A.kind != CurvedKind::Cone || B.kind != CurvedKind::Cone) return st;
  if (!ssidetail::sameAxis(A.frame, B.frame, 1e-6)) return st;  // must be coaxial

  const math::Vec3 zc = A.frame.z.vec();
  const double tanA = std::tan(A.semiAngle);
  const math::Point3 O = A.frame.origin;
  // Cone B's wall expressed in cone-A's s-coordinate (axes may be antiparallel):
  //   s_B = sign·(s − base),  r_B(s) = R0B_native + s_B·tan(α_B)
  //        = (R0B_native − sign·base·tanB_native) + s·(sign·tanB_native).
  const double baseB = math::dot(B.frame.origin - O, zc);
  const double signB = math::dot(B.frame.z.vec(), zc) >= 0.0 ? 1.0 : -1.0;
  const double tanBn = std::tan(B.semiAngle);
  const double tanB = signB * tanBn;
  const double R0B = B.radius - signB * baseB * tanBn;
  const double R0A = A.radius;
  if (std::fabs(tanA - tanB) < 1e-9) return st;  // parallel walls → no proper crossing → decline

  // Cone B axial extent in cone-A's s-coordinate.
  double bSLo = baseB + signB * B.vLo, bSHi = baseB + signB * B.vHi;
  if (bSLo > bSHi) std::swap(bSLo, bSHi);
  const double sLo = std::max(A.vLo, bSLo);
  const double sHi = std::min(A.vHi, bSHi);
  if (!(sHi - sLo > 1e-6)) return st;  // no axial overlap

  // Single interior crossing r_A(s*) = r_B(s*)  ⇒  s* = (R0B − R0A)/(tanA − tanB).
  const double sStar = (R0B - R0A) / (tanA - tanB);
  if (!(sStar - sLo > 1e-6) || !(sHi - sStar > 1e-6)) return st;  // apex/edge crossing → decline
  const double rStar = R0A + sStar * tanA;
  if (!(rStar > 1e-9)) return st;  // apex-touching seam → decline

  // Cross-check the analytic seam against the S3-traced seam (height s* + radius r*).
  auto sOf = [&](const math::Point3& p) {
    return math::dot(math::Vec3{p.x - O.x, p.y - O.y, p.z - O.z}, zc);
  };
  math::Point3 c{0, 0, 0};
  for (const auto& p : seam.pts) { c.x += p.x; c.y += p.y; c.z += p.z; }
  const double ns = static_cast<double>(seam.pts.size());
  c.x /= ns; c.y /= ns; c.z /= ns;
  if (std::fabs(sOf(c) - sStar) > 1e-4) return st;  // seam not at the analytic height
  double rho = 0.0;
  for (const auto& p : seam.pts) {
    const math::Vec3 w{p.x - c.x, p.y - c.y, p.z - c.z};
    rho += math::norm(w - zc * math::dot(w, zc));
  }
  rho /= ns;
  if (std::fabs(rho - rStar) > 1e-3) return st;  // seam radius ≠ r*

  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * rStar, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rStar / chord)), 24, 180);
  st.coneA = &A;
  st.coneB = &B;
  st.O = O;
  st.X = A.frame.x.vec();
  st.Y = A.frame.y.vec();
  st.zc = zc;
  st.tanA = tanA;
  st.R0A = R0A;
  st.tanB = tanB;
  st.R0B = R0B;
  st.aS0 = A.vLo;
  st.aS1 = A.vHi;
  st.bS0 = bSLo;
  st.bS1 = bSHi;
  st.sLo = sLo;
  st.sHi = sHi;
  st.sStar = sStar;
  st.rStar = rStar;
  st.ok = true;
  return st;
}

// buildConeConeCommon(A,B) = the COMMON of two COAXIAL cone frustums: the min-radius profile
// solid of revolution r ≤ min(r_A(s), r_B(s)) over the shared axial span [sLo,sHi]. The two
// walls cross EXACTLY ONCE at the single analytic circle s* (radius r*), so the shell is:
// bottom disc @ sLo, the inner-wall band below s* (whichever wall is narrower there), the
// shared SEAM ring @ s*, the inner-wall band above s*, top disc @ sHi. Both bands + caps
// share the seam/rim rings through one VertexPool → watertight. Coaxial single-crossing only.
topo::Shape buildConeConeCommon(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  const ConeConeSetup s = coneConeSetup(A, B, seams);
  if (!s.ok) return {};
  const double rBot = std::min(s.rA(s.sLo), s.rB(s.sLo));
  const double rTop = std::min(s.rA(s.sHi), s.rB(s.sHi));
  if (!(rBot > 1e-9) || !(rTop > 1e-9)) return {};  // apex-touching rim → decline
  // Survival gate: the min-wall below the seam is inside the OTHER cone (a proper overlap).
  const double midLo = 0.5 * (s.sLo + s.sStar);
  const bool aIsMinLo = s.rA(midLo) <= s.rB(midLo);
  const CurvedSolid& otherLo = aIsMinLo ? *s.coneB : *s.coneA;
  const double rMinLo = std::min(s.rA(midLo), s.rB(midLo));
  if (classifyPoint(otherLo, s.wallPoint(rMinLo, midLo), kSsiTol) != 1) return {};

  const std::vector<math::Point3> ringBot = s.ring(rBot, s.sLo);
  const std::vector<math::Point3> ringSeam = s.ring(s.rStar, s.sStar);
  const std::vector<math::Point3> ringTop = s.ring(rTop, s.sHi);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendRevolvedBand(ringBot, ringSeam, s.O, s.zc, pool, faces);   // inner wall below s*
  appendRevolvedBand(ringSeam, ringTop, s.O, s.zc, pool, faces);   // inner wall above s*
  appendDiskCap(*s.coneA, s.sLo, ringBot, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendDiskCap(*s.coneA, s.sHi, ringTop, math::Vec3{s.zc.x, s.zc.y, s.zc.z}, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildConeConeFuse(A,B) = A ∪ B of the coaxial cone∩cone pair — the OUTER (max-radius) wall
// profile over the union span [sMin,sMax], welded at the seam circle. We walk the outer
// profile bottom→top as corners (s,r): a different-s pair is a revolved wall band (the wider
// operand's wall, kept iff its mid classifies OUTSIDE the other operand); a same-s pair is a
// radial STEP where one operand begins/ends → a flat annulus cap. Two terminal disc caps
// close the ends. All rings shared through one pool → watertight. V = V(A)+V(B)−V(A∩B).
topo::Shape buildConeConeFuse(const CurvedSolid& A, const CurvedSolid& B,
                              const std::vector<Seam>& seams) {
  const ConeConeSetup s = coneConeSetup(A, B, seams);
  if (!s.ok) return {};
  const double sMin = std::min(s.aS0, s.bS0);
  const double sMax = std::max(s.aS1, s.bS1);
  const double span = sMax - sMin;
  if (!(span > 1e-9)) return {};
  const double eps = 1e-6 * std::max(span, 1.0);
  auto aR = [&](double t) { return (t > s.aS0 - eps && t < s.aS1 + eps) ? s.rA(t) : -1e300; };
  auto bR = [&](double t) { return (t > s.bS0 - eps && t < s.bS1 + eps) ? s.rB(t) : -1e300; };
  auto Rmax = [&](double t) { return std::max(aR(t), bR(t)); };

  std::vector<double> stn = {s.aS0, s.aS1, s.bS0, s.bS1, s.sStar};
  std::sort(stn.begin(), stn.end());
  stn.erase(std::unique(stn.begin(), stn.end(),
                        [&](double a2, double b2) { return std::fabs(a2 - b2) < eps; }),
            stn.end());

  std::vector<std::pair<double, double>> corners;
  corners.push_back({sMin, Rmax(sMin + eps)});
  for (double sc : stn) {
    if (sc <= sMin + eps || sc >= sMax - eps) continue;  // interior stations only
    const double rBelow = Rmax(sc - eps), rAbove = Rmax(sc + eps);
    corners.push_back({sc, rBelow});
    if (std::fabs(rAbove - rBelow) > eps) corners.push_back({sc, rAbove});  // radial step
  }
  corners.push_back({sMax, Rmax(sMax - eps)});
  if (!(corners.front().second > 1e-9) || !(corners.back().second > 1e-9)) return {};  // apex terminal

  VertexPool pool;
  std::vector<topo::Shape> faces;
  std::vector<math::Point3> prevRing = s.ring(corners.front().second, corners.front().first);
  appendDiskCap(*s.coneA, corners.front().first, prevRing,
                math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);  // bottom terminal disc
  for (std::size_t k = 1; k < corners.size(); ++k) {
    const double sp = corners[k - 1].first, rp = corners[k - 1].second;
    const double sc = corners[k].first, rc = corners[k].second;
    std::vector<math::Point3> curRing = s.ring(rc, sc);
    if (std::fabs(sc - sp) < eps) {
      const math::Vec3 axial = (rc > rp) ? math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z} : s.zc;
      appendAnnulusCap(prevRing, curRing, axial, pool, faces);
    } else {
      const double midS = 0.5 * (sp + sc);
      const bool bandIsA = aR(midS) >= bR(midS);
      const double rMid = std::max(aR(midS), bR(midS));
      const CurvedSolid& other = bandIsA ? *s.coneB : *s.coneA;
      if (classifyPoint(other, s.wallPoint(rMid, midS), kSsiTol) != -1) return {};  // tangent → OCCT
      appendRevolvedBand(prevRing, curRing, s.O, s.zc, pool, faces);
    }
    prevRing = std::move(curRing);
  }
  appendDiskCap(*s.coneA, corners.back().first, prevRing, s.zc, pool, faces);  // top terminal disc
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildConeConeCut(A,B) = A − B of the coaxial cone∩cone pair (A = cone MINUEND; order-
// sensitive, matching BRepAlgoAPI_Cut(a,b)). A keeps its OUTER (r_A > r_B) side of the seam
// — a conical WASHER whose outer boundary is A's wall (outward) and inner boundary is B's
// wall REVERSED (inward — the wall of the carved cavity), pinching to the shared seam ring at
// s* and closed at A's end station(s) by flat annulus caps, PLUS any A-only slice where B is
// absent (a full frustum slice, a possibly-detached component). V = V(A)−V(A∩B) (a SHRINK).
// Only the clean single-sided crossing where A is the wider wall on exactly ONE side of the
// seam is built; anything merged/tangent → {} → OCCT (never faked).
topo::Shape buildConeConeCut(const CurvedSolid& A, const CurvedSolid& B,
                             const std::vector<Seam>& seams) {
  const ConeConeSetup s = coneConeSetup(A, B, seams);
  if (!s.ok) return {};
  if (&A != s.coneA) return {};  // A must be the first operand (the minuend)
  const double eps = 1e-6 * std::max(s.aS1 - s.aS0, 1.0);
  const bool growsAbove = s.rA(s.sHi) > s.rB(s.sHi) + eps;  // A wider ABOVE the seam
  const bool growsBelow = s.rA(s.sLo) > s.rB(s.sLo) + eps;  // A wider BELOW the seam
  if (growsAbove == growsBelow) return {};                  // not a clean single-sided crossing → OCCT

  // The kept washer runs from the seam to A's terminal end on the r_A>r_B side; B must
  // extend past that end on the kept side (so the cavity terminates at an A cap, not a merge).
  const double capS = growsAbove ? s.aS1 : s.aS0;               // A's end on the kept side
  const math::Vec3 capN = growsAbove ? s.zc : math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z};
  // The OTHER A end (on the removed side) — if it protrudes past B, it is an A-only slice.
  const double slabS = growsAbove ? s.aS0 : s.aS1;
  const bool otherEndInsideB =
      growsAbove ? (s.aS0 > s.bS0 - eps) : (s.aS1 < s.bS1 + eps);
  if (growsAbove) { if (s.aS1 > s.bS1 + eps) return {}; }      // A outlives B on the kept side → merged → OCCT
  else { if (s.aS0 < s.bS0 - eps) return {}; }

  // Washer: A wall (outward) + B wall reversed (inward), pinching to the seam, capped at capS.
  const double midW = 0.5 * (s.sStar + capS);
  if (classifyPoint(*s.coneB, s.wallPoint(s.rA(midW), midW), kSsiTol) != -1) return {};  // A wall outside B
  if (classifyPoint(*s.coneA, s.wallPoint(s.rB(midW), midW), kSsiTol) != 1) return {};   // B wall inside A
  const double rACap = s.rA(capS), rBCap = s.rB(capS);
  if (!(rACap > 1e-9) || !(rBCap > 1e-9)) return {};

  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ringSeam = s.ring(s.rStar, s.sStar);
  const std::vector<math::Point3> ringACap = s.ring(rACap, capS);
  const std::vector<math::Point3> ringBCap = s.ring(rBCap, capS);
  appendRevolvedBand(ringSeam, ringACap, s.O, s.zc, pool, faces, 1.0);   // outer A wall
  appendRevolvedBand(ringSeam, ringBCap, s.O, s.zc, pool, faces, -1.0);  // reversed inner B wall
  appendAnnulusCap(ringBCap, ringACap, capN, pool, faces);               // A-end annulus cap

  // A-only slice (a detached frustum component) where A protrudes past B on the removed side.
  if (!otherEndInsideB) {
    const double sCut = growsAbove ? s.bS0 : s.bS1;   // interface with the carved region
    const math::Vec3 termN = growsAbove ? math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z} : s.zc;
    const double midSlice = 0.5 * (slabS + sCut);
    if (classifyPoint(*s.coneB, s.wallPoint(s.rA(midSlice), midSlice), kSsiTol) != -1) return {};
    if (!(s.rA(slabS) > 1e-9) || !(s.rA(sCut) > 1e-9)) return {};
    const std::vector<math::Point3> ringTerm = s.ring(s.rA(slabS), slabS);
    const std::vector<math::Point3> ringCut = s.ring(s.rA(sCut), sCut);
    appendDiskCap(*s.coneA, slabS, ringTerm, termN, pool, faces);
    appendRevolvedBand(ringTerm, ringCut, s.O, s.zc, pool, faces);
    appendDiskCap(*s.coneA, sCut, ringCut, math::Vec3{-termN.x, -termN.y, -termN.z}, pool, faces);
  }
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── S5-j — coaxial HOURGLASS (apex-to-apex / bowtie) CONE∩CONE COMMON / CUT ─────
// The genuinely-different sibling of the S5-g coaxial cone∩cone pair. Where S5-g handles
// the FRUSTUM-band single crossing (both min-profile endpoints STRICTLY off the axis),
// the HOURGLASS pose is two coaxial cones pointing AT each other (bowtie): cone A ▽ opens
// downward (r_A shrinks with s to an apex), cone B △ opens upward (r_B grows from an apex).
// Their walls still cross at ONE analytic circle s* (both linear in s → ConeConeSetup is
// reused verbatim), but the COMMON's min-radius profile PINCHES to the AXIS (r→0, a cone
// apex) at one or both overlap ends, so the S5-g COMMON/CUT apex gates (rBot/rTop/rBCap>0)
// decline it. The ONLY change is the apex-terminated cap: instead of a flat disc / annulus
// at an off-axis rim, an apex end is closed by a REVOLVED BAND onto a degenerate apex ring
// (N coincident apex points → appendRevolvedBand emits a proper cone-tip fan, the sliver
// half of each quad skipped by pushPlanarTri; the apex dedups through the pool → watertight).
// The seam ring stays the SHARED analytic circle, so COMMON welds byte-clean. FUSE of the
// bowtie has OFF-AXIS terminal discs (the max profile) so the S5-g FUSE builds it directly
// (dispatched below S5-g). Coaxial single-crossing only; anything S5-g already owns (both
// ends off-axis) is left to S5-g. Nothing faked — a non-apex profile returns {} here.
//
// V(COMMON) = V_frustum(rBot, r*, s*−sLo) + V_frustum(r*, rTop, sHi−s*), each apex end a full
// cone (rBot or rTop = 0). V(CUT A−B) = V(A) − V(COMMON) (a SHRINK). Dual-oracle verified.

// A degenerate "ring" of N coincident apex points at station s on the shared axis — the
// terminal of a cone-tip band. appendRevolvedBand(apexRing, offAxisRing, …) then sweeps a
// watertight cone fan (each quad's axis-side sliver is dropped by pushPlanarTri).
std::vector<math::Point3> apexRing(const ConeConeSetup& s, double station, int n) {
  const math::Point3 apex{s.O.x + s.zc.x * station, s.O.y + s.zc.y * station,
                          s.O.z + s.zc.z * station};
  return std::vector<math::Point3>(static_cast<std::size_t>(n), apex);
}

// buildHourglassConeConeCommon(A,B) = COMMON of the bowtie coaxial cone∩cone: the min-radius
// profile of revolution over the overlap [sLo,sHi], where AT LEAST ONE end pinches to the
// axis (a cone apex) — the pose S5-g's COMMON declines. Two bands meet at the shared seam
// ring; each end is a flat disc (off-axis) or a cone-tip apex band (on-axis). Watertight.
topo::Shape buildHourglassConeConeCommon(const CurvedSolid& A, const CurvedSolid& B,
                                         const std::vector<Seam>& seams) {
  const ConeConeSetup s = coneConeSetup(A, B, seams);
  if (!s.ok) return {};
  const double rBot = std::min(s.rA(s.sLo), s.rB(s.sLo));
  const double rTop = std::min(s.rA(s.sHi), s.rB(s.sHi));
  if (rBot < -1e-9 || rTop < -1e-9) return {};
  const bool apexBot = rBot <= 1e-9, apexTop = rTop <= 1e-9;
  if (!apexBot && !apexTop) return {};  // both ends off-axis → S5-g owns it (not this path)
  // Survival gate: the min wall just below the seam is inside the OTHER cone (proper overlap).
  const double midLo = 0.5 * (s.sLo + s.sStar);
  const bool aIsMinLo = s.rA(midLo) <= s.rB(midLo);
  const CurvedSolid& otherLo = aIsMinLo ? *s.coneB : *s.coneA;
  const double rMinLo = std::min(s.rA(midLo), s.rB(midLo));
  if (classifyPoint(otherLo, s.wallPoint(rMinLo, midLo), kSsiTol) != 1) return {};

  const std::vector<math::Point3> ringSeam = s.ring(s.rStar, s.sStar);
  const std::vector<math::Point3> ringBot =
      apexBot ? apexRing(s, s.sLo, s.N) : s.ring(rBot, s.sLo);
  const std::vector<math::Point3> ringTop =
      apexTop ? apexRing(s, s.sHi, s.N) : s.ring(rTop, s.sHi);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendRevolvedBand(ringBot, ringSeam, s.O, s.zc, pool, faces);   // inner wall below s* (apex fan if apexBot)
  appendRevolvedBand(ringSeam, ringTop, s.O, s.zc, pool, faces);   // inner wall above s* (apex fan if apexTop)
  if (!apexBot)
    appendDiskCap(*s.coneA, s.sLo, ringBot, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  if (!apexTop) appendDiskCap(*s.coneA, s.sHi, ringTop, s.zc, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildHourglassConeConeCut(A,B) = A − B of the bowtie coaxial cone∩cone (A = cone MINUEND;
// order-sensitive). A keeps its wider (r_A > r_B) side of the seam — a conical shell whose
// outer boundary is A's wall (outward) and inner boundary is B's wall REVERSED (inward), from
// the seam ring to A's terminal end on the kept side. The bowtie's distinctive feature: on the
// kept side B's wall runs into its OWN apex (r_B → 0) before A's end, so the inner boundary is
// a cone-tip apex band and A's terminal cap is a FULL disc (B absent there). This is the pose
// S5-g's CUT (annulus-cap, rBCap>0) declines. V = V(A) − V(COMMON) (a SHRINK). Non-apex → {}.
topo::Shape buildHourglassConeConeCut(const CurvedSolid& A, const CurvedSolid& B,
                                      const std::vector<Seam>& seams) {
  const ConeConeSetup s = coneConeSetup(A, B, seams);
  if (!s.ok) return {};
  if (&A != s.coneA) return {};  // A must be the minuend (first operand)
  const double eps = 1e-6 * std::max(s.aS1 - s.aS0, 1.0);
  const bool growsAbove = s.rA(s.sHi) > s.rB(s.sHi) + eps;  // A wider ABOVE the seam
  const bool growsBelow = s.rA(s.sLo) > s.rB(s.sLo) + eps;  // A wider BELOW the seam
  if (growsAbove == growsBelow) return {};                  // not a clean single-sided crossing

  // A's terminal on the kept side, and B's radius there. The bowtie CUT is the case where B's
  // wall pinches to its apex (r_B ≈ 0) at (or before) A's kept end — S5-g declines that.
  const double capS = growsAbove ? s.aS1 : s.aS0;
  const math::Vec3 capN = growsAbove ? s.zc : math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z};
  const double rACap = s.rA(capS);
  const double rBCap = s.rB(capS);
  if (!(rACap > 1e-9)) return {};       // A must still be off-axis at its kept end
  if (rBCap > 1e-6) return {};          // B off-axis at capS → S5-g's annulus-cap CUT owns it
  // Kept side must not outlive B beyond its apex: B's apex station on the kept side is where the
  // reversed inner wall lands on the axis — inside A's extent, exactly at capS for the bowtie.
  // Washer material check: A wall outside B, B wall inside A, on the kept side.
  const double midW = 0.5 * (s.sStar + capS);
  if (classifyPoint(*s.coneB, s.wallPoint(s.rA(midW), midW), kSsiTol) != -1) return {};
  if (classifyPoint(*s.coneA, s.wallPoint(std::max(s.rB(midW), 1e-6), midW), kSsiTol) != 1) return {};

  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ringSeam = s.ring(s.rStar, s.sStar);
  const std::vector<math::Point3> ringACap = s.ring(rACap, capS);
  const std::vector<math::Point3> ringBApex = apexRing(s, capS, s.N);
  appendRevolvedBand(ringSeam, ringACap, s.O, s.zc, pool, faces, 1.0);    // outer A wall
  appendRevolvedBand(ringSeam, ringBApex, s.O, s.zc, pool, faces, -1.0);  // reversed inner B wall (to apex)
  appendDiskCap(*s.coneA, capS, ringACap, capN, pool, faces);            // full A-end disc (B absent)
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// Shared prologue for the three single-seam sphere∩sphere lens assemblers: validate the
// trace/kinds, compute the two candidate apices (inner near-apex of each sphere and each
// far pole), the decimated shared seam, and the per-cap ring-count functor. Factored out so
// buildLensCommon / buildLensFuse / buildLensCut differ ONLY in cap selection (which caps,
// which orientation) — never in seam decimation or ring discipline, so all three caps weld
// on the identical pooled seam nodes.
struct LensSetup {
  bool ok = false;
  Seam capSeam;
  math::Point3 innerApexA, innerApexB, outerApexA, outerApexB;
};
LensSetup lensSetup(const CurvedSolid& A, const CurvedSolid& B, const std::vector<Seam>& seams) {
  LensSetup s;
  if (seams.size() != 1) return s;
  const Seam& seam = seams[0];
  if (!seam.closed || seam.pts.size() < 4) return s;
  if (A.kind != CurvedKind::Sphere || B.kind != CurvedKind::Sphere) return s;
  const math::Vec3 ab = B.frame.origin - A.frame.origin;
  const double d = math::norm(ab);
  if (d < 1e-9) return s;  // concentric → not a transversal lens
  const math::Vec3 abU{ab.x / d, ab.y / d, ab.z / d};
  s.innerApexA = {A.frame.origin.x + abU.x * A.radius, A.frame.origin.y + abU.y * A.radius,
                  A.frame.origin.z + abU.z * A.radius};
  s.innerApexB = {B.frame.origin.x - abU.x * B.radius, B.frame.origin.y - abU.y * B.radius,
                  B.frame.origin.z - abU.z * B.radius};
  s.outerApexA = {A.frame.origin.x - abU.x * A.radius, A.frame.origin.y - abU.y * A.radius,
                  A.frame.origin.z - abU.z * A.radius};
  s.outerApexB = {B.frame.origin.x + abU.x * B.radius, B.frame.origin.y + abU.y * B.radius,
                  B.frame.origin.z + abU.z * B.radius};
  s.capSeam = decimateSeam(seam, seamNodeTarget(seam));
  s.ok = true;
  return s;
}

// Ring count from a cap's TRUE polar half-angle θ (max angle at the sphere centre between
// the apex direction and any seam node) + the radial curvature-per-facet target kCapSagitta
// — identical discipline for inner and outer caps (the far-pole apex has a larger θ, giving
// more rings, which is correct). Shared by all three lens assemblers.
int lensRingsFor(const CurvedSolid& sph, const math::Point3& apex, const Seam& capSeam) {
  const math::Vec3 aDir = apex - sph.frame.origin;
  double theta = 0.0;
  for (const auto& p : capSeam.pts) {
    const math::Vec3 sDir{p.x - sph.frame.origin.x, p.y - sph.frame.origin.y,
                          p.z - sph.frame.origin.z};
    const double denom = std::max(math::norm(aDir) * math::norm(sDir), 1e-12);
    theta = std::max(theta, std::acos(std::clamp(math::dot(aDir, sDir) / denom, -1.0, 1.0)));
  }
  const double rings = std::max(theta, 1e-6) * std::sqrt(sph.radius / (2.0 * kCapSagitta));
  return std::clamp(static_cast<int>(std::ceil(rings)), 4, 48);
}

// buildLensFuse(A,B) = the FUSE (A ∪ B) of two overlapping SPHERES: the peanut/dumbbell
// outer shell bounded by the two OUTER caps (each sphere's far-pole cap, the part OUTSIDE
// the other solid) sharing the single seam circle. Volume = V(A)+V(B)−V(lens) (grows).
// Survival rule: each far pole must classify strictly OUTSIDE the other solid (a proper
// transversal overlap); tangent/containment/degenerate → {} → OCCT (never faked).
topo::Shape buildLensFuse(const CurvedSolid& A, const CurvedSolid& B,
                          const std::vector<Seam>& seams) {
  const LensSetup s = lensSetup(A, B, seams);
  if (!s.ok) return {};
  // Each retained cap's far-pole apex must be OUTSIDE the other solid (the FUSE survival rule).
  if (classifyPoint(B, s.outerApexA, kSsiTol) != -1) return {};  // ON/inside → not a proper fuse shell
  if (classifyPoint(A, s.outerApexB, kSsiTol) != -1) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // Two OUTER caps (outer=true, reversed=false), both outward, on the shared decimated seam.
  appendSphereCap(A, B.frame.origin, s.capSeam, lensRingsFor(A, s.outerApexA, s.capSeam), pool,
                  faces, /*outer=*/true, /*reversed=*/false);
  appendSphereCap(B, A.frame.origin, s.capSeam, lensRingsFor(B, s.outerApexB, s.capSeam), pool,
                  faces, /*outer=*/true, /*reversed=*/false);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildLensCut(A,B) = the CUT (A − B) of two overlapping SPHERES: the OUTER cap of A (the
// part of A OUTSIDE B, outward) welded to the INNER cap of B emitted REVERSED (inward
// normal — B's near-apex cap now bounds the scooped cavity), sharing the single seam.
// Volume = V(A)−V(lens) (shrinks). ORDER-SENSITIVE: A is the minuend (matches
// BRepAlgoAPI_Cut(a,b)); the two caps are not interchangeable. Survival rule: A's far pole
// OUTSIDE B AND B's near-apex INSIDE A (a proper transversal bite); else {} → OCCT.
topo::Shape buildLensCut(const CurvedSolid& A, const CurvedSolid& B,
                         const std::vector<Seam>& seams) {
  const LensSetup s = lensSetup(A, B, seams);
  if (!s.ok) return {};
  if (classifyPoint(B, s.outerApexA, kSsiTol) != -1) return {};  // A's far pole must be outside B
  if (classifyPoint(A, s.innerApexB, kSsiTol) != 1) return {};   // B's near-apex must be inside A
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // OUTER cap of A (outer=true, outward) + INNER cap of B REVERSED (outer=false, reversed=true).
  appendSphereCap(A, B.frame.origin, s.capSeam, lensRingsFor(A, s.outerApexA, s.capSeam), pool,
                  faces, /*outer=*/true, /*reversed=*/false);
  appendSphereCap(B, A.frame.origin, s.capSeam, lensRingsFor(B, s.innerApexB, s.capSeam), pool,
                  faces, /*outer=*/false, /*reversed=*/true);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── S5-f — coaxial CONE(frustum)∩SPHERE COMMON / FUSE / CUT ─────────────────────
// A cone frustum A and a sphere B whose centre lies ON the cone axis meet along ONE
// analytic circle seam (the single-crossing config where the sphere sits on the frustum
// side). The pair is the COMPOSITION of two existing families: the CONE side reuses the
// cone-wall split (appendRevolvedBand + appendDiskCap, from buildConeCylCommon); the
// SPHERE side reuses the spherical-cap fragment (appendSphereCap, with its inner/outer-apex
// and reversed-normal flags, from the lens builders). Both weld along ONE pooled seam ring.
//
// The seam is where the cone wall (radial r_cone(s)=R0+s·tanα, axial s from the cone origin)
// meets the sphere: (s−sc)² + r_cone(s)² = Rs² — a quadratic in s. The single interior root
// s* gives the seam circle at radius ρ=r_cone(s*), station s*. Exactly one of the sphere's
// two axial poles lies INSIDE the cone (the single-crossing config); the seam splits the
// sphere into that INNER cap (toward the in-cone pole) and the OUTER cap (toward the far
// pole outside the cone). Anything else — two interior roots (sphere crossing the wall
// twice), an apex-crossing / tangent seam, both poles on one side, or a non-coaxial pair —
// returns {} → OCCT (never faked).
struct ConeSphereSetup {
  bool ok = false;
  const CurvedSolid* cone = nullptr;
  const CurvedSolid* sph = nullptr;
  math::Point3 O;          ///< cone origin
  math::Vec3 X, Y, zc;     ///< cone frame
  math::Point3 C;          ///< sphere centre (on the cone axis)
  double tanA = 0.0, R0 = 0.0, Rs = 0.0;
  double sc = 0.0;         ///< sphere-centre axial coord in the cone's s-frame
  double coneS0 = 0.0, coneS1 = 0.0;  ///< cone's own s-extent [vLo,vHi]
  double sStar = 0.0;      ///< single interior crossing r_cone(s*)²+(s*−sc)²=Rs²
  double rho = 0.0;        ///< seam radius r_cone(s*)
  double inDir = 1.0;      ///< +1 if the in-cone pole sits at larger s
  double sPole = 0.0;      ///< in-cone pole axial station (sc + Rs·inDir)
  double coneNear = 0.0;   ///< cone terminal end inside the sphere (−inDir side; disc-capped)
  double coneFar = 0.0;    ///< cone terminal end outside the sphere (+inDir side)
  int N = 0;               ///< azimuth sample count (seam-chord bounded)
  double rCone(double s) const { return R0 + s * tanA; }
  std::vector<math::Point3> ring(double r, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = r * std::cos(u), cy = r * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  /// A world point on a wall at (radius r, station s), azimuth 0 — the classifier sample.
  math::Point3 wallPoint(double r, double s) const {
    return math::Point3{O.x + X.x * r + zc.x * s, O.y + X.y * r + zc.y * s,
                        O.z + X.z * r + zc.z * s};
  }
  math::Point3 poleIn() const {
    return math::Point3{C.x + zc.x * Rs * inDir, C.y + zc.y * Rs * inDir, C.z + zc.z * Rs * inDir};
  }
  math::Point3 poleOut() const {
    return math::Point3{C.x - zc.x * Rs * inDir, C.y - zc.y * Rs * inDir, C.z - zc.z * Rs * inDir};
  }
  /// The `otherCentre` reference appendSphereCap uses: a point in the +inDir axis direction
  /// from the sphere centre, so the INNER cap apex resolves to poleIn and the OUTER to poleOut.
  math::Point3 otherRef() const {
    return math::Point3{C.x + zc.x * inDir, C.y + zc.y * inDir, C.z + zc.z * inDir};
  }
  /// A Seam whose nodes are the shared canonical seam ring (identical to ring(rho,sStar)),
  /// so the cone band and appendSphereCap weld on byte-identical pooled vertices.
  Seam seamRing() const {
    Seam s;
    s.closed = true;
    s.pts = ring(rho, sStar);
    return s;
  }
};

ConeSphereSetup coneSphereSetup(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  ConeSphereSetup st;
  if (seams.size() != 1) return st;
  const Seam& seam = seams[0];
  if (!seam.closed || seam.pts.size() < 8) return st;

  // Exactly one Cone + one Sphere operand, coaxial (sphere centre on the cone axis).
  const CurvedSolid* conePtr = nullptr;
  const CurvedSolid* sphPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Cone) conePtr = s;
    else if (s->kind == CurvedKind::Sphere) sphPtr = s;
  }
  if (!conePtr || !sphPtr) return st;
  const CurvedSolid& cone = *conePtr;
  const CurvedSolid& sph = *sphPtr;

  const math::Vec3 zc = cone.frame.z.vec();
  const math::Point3 O = cone.frame.origin;
  const math::Point3 C = sph.frame.origin;
  const math::Vec3 d{C.x - O.x, C.y - O.y, C.z - O.z};
  const double sc = math::dot(d, zc);
  if (math::norm(d - zc * sc) > 1e-6) return st;  // sphere centre off the cone axis → not coaxial
  const double tanA = std::tan(cone.semiAngle);
  if (std::fabs(tanA) < 1e-9) return st;  // degenerate (near-cylindrical) cone
  const double R0 = cone.radius, Rs = sph.radius;
  if (!(Rs > 1e-9)) return st;

  // Seam quadratic (1+tanA²)s² + 2(R0·tanA − sc)s + (sc² + R0² − Rs²) = 0.
  const double Aq = 1.0 + tanA * tanA;
  const double Bq = 2.0 * (R0 * tanA - sc);
  const double Cq = sc * sc + R0 * R0 - Rs * Rs;
  const double disc = Bq * Bq - 4.0 * Aq * Cq;
  if (disc <= 1e-12) return st;  // no crossing / tangent → decline
  const double sq = std::sqrt(disc);
  const double roots[2] = {(-Bq + sq) / (2.0 * Aq), (-Bq - sq) / (2.0 * Aq)};
  const double coneS0 = cone.vLo, coneS1 = cone.vHi;
  int nin = 0;
  double sStar = 0.0;
  for (const double r : roots)
    if (r > coneS0 + 1e-6 && r < coneS1 - 1e-6) { ++nin; sStar = r; }
  if (nin != 1) return st;  // 0 or 2 interior roots → not the clean single seam → OCCT
  const double rho = R0 + sStar * tanA;
  if (!(rho > 1e-9)) return st;  // apex-touching seam → decline

  // Cross-check the analytic seam against the S3-traced seam (height s* + radius ρ).
  auto sOf = [&](const math::Point3& p) {
    return math::dot(math::Vec3{p.x - O.x, p.y - O.y, p.z - O.z}, zc);
  };
  math::Point3 c{0, 0, 0};
  for (const auto& p : seam.pts) { c.x += p.x; c.y += p.y; c.z += p.z; }
  const double ns = static_cast<double>(seam.pts.size());
  c.x /= ns; c.y /= ns; c.z /= ns;
  if (std::fabs(sOf(c) - sStar) > 1e-4) return st;  // seam not at the analytic height
  double rhoTr = 0.0;
  for (const auto& p : seam.pts) {
    const math::Vec3 w{p.x - c.x, p.y - c.y, p.z - c.z};
    rhoTr += math::norm(w - zc * math::dot(w, zc));
  }
  rhoTr /= ns;
  if (std::fabs(rhoTr - rho) > 1e-3) return st;  // seam radius ≠ ρ

  // Classify both sphere poles against the cone: the single-crossing config has EXACTLY one
  // pole inside (the inner-cap apex) and one outside (the outer-cap apex). Anything else → OCCT.
  const math::Point3 poleP{C.x + zc.x * Rs, C.y + zc.y * Rs, C.z + zc.z * Rs};
  const math::Point3 poleM{C.x - zc.x * Rs, C.y - zc.y * Rs, C.z - zc.z * Rs};
  const int clsP = classifyPoint(cone, poleP, kSsiTol);
  const int clsM = classifyPoint(cone, poleM, kSsiTol);
  double inDir;
  if (clsP == 1 && clsM == -1) inDir = 1.0;
  else if (clsM == 1 && clsP == -1) inDir = -1.0;
  else return st;  // both in / both out / ON → not a clean single crossing → OCCT
  const double sPole = sc + Rs * inDir;
  const double coneNear = (inDir > 0.0) ? coneS0 : coneS1;
  const double coneFar = (inDir > 0.0) ? coneS1 : coneS0;

  // s* must sit strictly between the cone near-end and the in-cone pole; the cone must
  // extend past the in-cone pole (so the sphere closes inside the cone); and the cone near
  // end must lie inside the sphere (so it is disc-capped, the clean single-crossing case).
  const double lo = std::min(coneNear, sPole), hi = std::max(coneNear, sPole);
  if (!(sStar > lo + 1e-6) || !(sStar < hi - 1e-6)) return st;
  if (inDir > 0.0) { if (!(coneS1 > sPole + 1e-6)) return st; }
  else { if (!(coneS0 < sPole - 1e-6)) return st; }
  const double rNear = R0 + coneNear * tanA;
  if (!(rNear * rNear + (coneNear - sc) * (coneNear - sc) < Rs * Rs - 1e-6)) return st;

  // Azimuthal resolution: seam-circle chord sagitta ≤ kCapSagitta (same bound as the caps).
  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * rho, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rho / chord)), 24, 180);
  st.cone = conePtr;
  st.sph = sphPtr;
  st.O = O;
  st.X = cone.frame.x.vec();
  st.Y = cone.frame.y.vec();
  st.zc = zc;
  st.C = C;
  st.tanA = tanA;
  st.R0 = R0;
  st.Rs = Rs;
  st.sc = sc;
  st.coneS0 = coneS0;
  st.coneS1 = coneS1;
  st.sStar = sStar;
  st.rho = rho;
  st.inDir = inDir;
  st.sPole = sPole;
  st.coneNear = coneNear;
  st.coneFar = coneFar;
  st.ok = true;
  return st;
}

// buildConeSphereCommon(A,B) = the COMMON of a COAXIAL cone frustum and a sphere: the
// cone wall band INSIDE the sphere (cone near-end → seam) + the cone terminal disc + the
// sphere INNER cap (seam → in-cone pole) that closes the overlap. V = V_frustum + V_sphere-
// segment. Taken ONLY for the clean single-crossing config; anything else → {} → OCCT.
topo::Shape buildConeSphereCommon(const CurvedSolid& A, const CurvedSolid& B,
                                  const std::vector<Seam>& seams) {
  const ConeSphereSetup s = coneSphereSetup(A, B, seams);
  if (!s.ok) return {};
  // Survival gate: the cone wall below the seam is INSIDE the sphere; the inner pole is
  // INSIDE the cone (the inside-the-other fragments COMMON keeps). Tangent/ON → decline.
  const double midNear = 0.5 * (s.coneNear + s.sStar);
  if (classifyPoint(*s.sph, s.wallPoint(s.rCone(midNear), midNear), kSsiTol) != 1) return {};
  if (classifyPoint(*s.cone, s.poleIn(), kSsiTol) != 1) return {};

  const std::vector<math::Point3> ringNear = s.ring(s.rCone(s.coneNear), s.coneNear);
  const Seam capSeam = s.seamRing();
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendRevolvedBand(ringNear, capSeam.pts, s.O, s.zc, pool, faces);  // cone wall inside sphere
  const math::Vec3 capN{-s.inDir * s.zc.x, -s.inDir * s.zc.y, -s.inDir * s.zc.z};
  appendDiskCap(*s.cone, s.coneNear, ringNear, capN, pool, faces);    // cone terminal disc
  appendSphereCap(*s.sph, s.otherRef(), capSeam, lensRingsFor(*s.sph, s.poleIn(), capSeam), pool,
                  faces, /*outer=*/false, /*reversed=*/false);        // sphere inner cap
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildConeSphereFuse(A,B) = A ∪ B of the coaxial cone∩sphere pair: the sphere OUTER cap
// (outside the cone) + the cone OUTER wall band (seam → far end, outside the sphere) + the
// cone terminal disc. V = V(A)+V(B)−V(COMMON) (a GROW). Survival: the cone wall above the
// seam is OUTSIDE the sphere AND the far pole is OUTSIDE the cone; else {} → OCCT.
topo::Shape buildConeSphereFuse(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  const ConeSphereSetup s = coneSphereSetup(A, B, seams);
  if (!s.ok) return {};
  const double midOut = 0.5 * (s.sStar + s.sPole);
  if (classifyPoint(*s.sph, s.wallPoint(s.rCone(midOut), midOut), kSsiTol) != -1) return {};
  if (classifyPoint(*s.cone, s.poleOut(), kSsiTol) != -1) return {};

  const Seam capSeam = s.seamRing();
  const double rFar = s.rCone(s.coneFar);
  if (!(rFar > 1e-9)) return {};
  const std::vector<math::Point3> ringFar = s.ring(rFar, s.coneFar);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendSphereCap(*s.sph, s.otherRef(), capSeam, lensRingsFor(*s.sph, s.poleOut(), capSeam), pool,
                  faces, /*outer=*/true, /*reversed=*/false);         // sphere outer cap
  appendRevolvedBand(capSeam.pts, ringFar, s.O, s.zc, pool, faces);   // cone wall outside sphere
  const math::Vec3 capN{s.inDir * s.zc.x, s.inDir * s.zc.y, s.inDir * s.zc.z};
  appendDiskCap(*s.cone, s.coneFar, ringFar, capN, pool, faces);      // cone terminal disc
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildConeSphereCut(A,B) = A − B of the coaxial cone∩sphere pair (A = cone MINUEND;
// order-sensitive, matching BRepAlgoAPI_Cut(a,b)). A CONNECTED frustum-with-a-spherical-
// dimple: the cone OUTER wall (seam → far end) + the cone terminal disc + the sphere INNER
// cap emitted REVERSED (inward dimple, pinching to the seam). V = V(A)−V(COMMON) (a SHRINK).
// A sphere minuend (sphere−cone) is a different topology → declines → OCCT (never faked).
topo::Shape buildConeSphereCut(const CurvedSolid& A, const CurvedSolid& B,
                               const std::vector<Seam>& seams) {
  const ConeSphereSetup s = coneSphereSetup(A, B, seams);
  if (!s.ok) return {};
  if (&A != s.cone) return {};  // A must be the cone minuend; sphere−cone → OCCT
  const double midOut = 0.5 * (s.sStar + s.sPole);
  if (classifyPoint(*s.sph, s.wallPoint(s.rCone(midOut), midOut), kSsiTol) != -1) return {};
  if (classifyPoint(*s.cone, s.poleIn(), kSsiTol) != 1) return {};  // dimple pole inside the cone

  const Seam capSeam = s.seamRing();
  const double rFar = s.rCone(s.coneFar);
  if (!(rFar > 1e-9)) return {};
  const std::vector<math::Point3> ringFar = s.ring(rFar, s.coneFar);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendRevolvedBand(capSeam.pts, ringFar, s.O, s.zc, pool, faces);   // cone wall outside sphere
  const math::Vec3 capN{s.inDir * s.zc.x, s.inDir * s.zc.y, s.inDir * s.zc.z};
  appendDiskCap(*s.cone, s.coneFar, ringFar, capN, pool, faces);      // cone terminal disc
  appendSphereCap(*s.sph, s.otherRef(), capSeam, lensRingsFor(*s.sph, s.poleIn(), capSeam), pool,
                  faces, /*outer=*/false, /*reversed=*/true);         // sphere inner cap → dimple
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── S5-h — TWO-CIRCLE coaxial CONE(frustum)∩SPHERE COMMON / FUSE / CUT ──────────────────
// The natural extension of the single-circle S5-f pair: a cone frustum coaxial with a sphere
// (centre ON the cone axis) whose wall crosses the sphere at TWO latitudes → TWO analytic
// circle seams s*_lo < s*_hi and a spherical ZONE (band of the sphere) between them. The seam
// quadratic (1+tanA²)s² + 2(R0·tanA − sc)s + (sc² + R0² − Rs²) = 0 now has BOTH roots strictly
// interior to the cone's extent AND to the sphere's axial span [sc−Rs, sc+Rs], with the sphere
// bulging OUTSIDE the cone wall between the seams (r_sphere(s) > r_cone(s) on the mid-band) and
// INSIDE it beyond them (each polar cap sits inside the cone). This is the "sphere pokes through
// the cone wall" pose — the SPHERE is the wider operand on the mid-band, the CONE on the poles.
//
// Both seam circles are S1-analytic closed forms (radius ρ=r_cone(s*), station s*); the S3
// tracer typically returns only ONE of the two co-resident loops (the documented S2 co-resident
// seeding-recall limit — see the roadmap), so this prologue computes BOTH circles itself and
// CROSS-CHECKS the traced seam(s) against the analytic roots (height + radius), never trusting a
// missing loop. The two rings are canonical azimuth samples through a shared VertexPool so every
// band/cap/zone welds byte-identically.
//
//   COMMON (inside BOTH) = r ≤ min(r_cone, r_sphere): the sphere LOWER cap (poleM→seamLo, inside
//     the cone) + the cone frustum band (seamLo→seamHi, inside the sphere) + the sphere UPPER cap
//     (seamHi→poleP, inside the cone). V = V_sph-seg(poleM,sLo) + V_frustum(sLo,sHi) +
//     V_sph-seg(sHi,poleP). A closed form.
//   FUSE (A∪B) = r ≤ max(r_cone, r_sphere): cone wall (coneNear→seamLo) + the sphere ZONE bulge
//     (seamLo→seamHi, the mid-band where the sphere is wider) + cone wall (seamHi→coneFar) + two
//     cone terminal discs. V = V(cone)+V(sphere)−V(COMMON) (a GROW).
//   CUT (A−B, cone MINUEND) = cone minus the sphere lens. The sphere fully engulfs the cone
//     cross-section on the mid-band, so the result PINCHES OFF into TWO disconnected components:
//     a lower cone-tip piece (coneNear→seamLo, its top scooped by the sphere lower cap reversed)
//     and an upper piece (seamHi→coneFar, its bottom scooped by the sphere upper cap reversed),
//     each a closed shell of one pool. V = V(cone)−V(COMMON) (a SHRINK). A sphere minuend
//     (sphere−cone) is a different topology → declines → OCCT.
//
// Anything else — a single interior root (the S5-f case, handled there), the sphere NOT bulging
// outside on the mid-band (an internally-tangent / nested pose), a pole outside the cone, an
// apex-crossing seam, or a non-coaxial pair — returns {} → OCCT. Nothing is faked.
struct ConeSphere2Setup {
  bool ok = false;
  const CurvedSolid* cone = nullptr;
  const CurvedSolid* sph = nullptr;
  math::Point3 O;          ///< cone origin
  math::Vec3 X, Y, zc;     ///< cone frame
  math::Point3 C;          ///< sphere centre (on the cone axis)
  double tanA = 0.0, R0 = 0.0, Rs = 0.0;
  double sc = 0.0;         ///< sphere-centre axial coord in the cone's s-frame
  double coneS0 = 0.0, coneS1 = 0.0;  ///< cone's own s-extent
  double sLo = 0.0, sHi = 0.0;        ///< the TWO interior crossings (sLo < sHi)
  double rhoLo = 0.0, rhoHi = 0.0;    ///< seam radii r_cone(sLo), r_cone(sHi)
  double poleM = 0.0, poleP = 0.0;    ///< sphere axial poles (sc−Rs, sc+Rs)
  int N = 0;                          ///< azimuth sample count (seam-chord bounded)
  double rCone(double s) const { return R0 + s * tanA; }
  double rSph(double s) const {
    const double d = Rs * Rs - (s - sc) * (s - sc);
    return d > 0.0 ? std::sqrt(d) : 0.0;
  }
  std::vector<math::Point3> ring(double r, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = r * std::cos(u), cy = r * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  math::Point3 wallPoint(double r, double s) const {
    return math::Point3{O.x + X.x * r + zc.x * s, O.y + X.y * r + zc.y * s,
                        O.z + X.z * r + zc.z * s};
  }
  math::Point3 axisPtM() const {  // sphere lower pole (poleM) as a world point
    return math::Point3{C.x + zc.x * (poleM - sc), C.y + zc.y * (poleM - sc),
                        C.z + zc.z * (poleM - sc)};
  }
  math::Point3 axisPtP() const {  // sphere upper pole (poleP)
    return math::Point3{C.x + zc.x * (poleP - sc), C.y + zc.y * (poleP - sc),
                        C.z + zc.z * (poleP - sc)};
  }
  /// A `Seam` whose nodes are the analytic seam ring at (radius, station) — the shared weld ring.
  Seam seamRing(double rho, double s) const {
    Seam out;
    out.closed = true;
    out.pts = ring(rho, s);
    return out;
  }
};

ConeSphere2Setup coneSphere2Setup(const CurvedSolid& A, const CurvedSolid& B,
                                  const std::vector<Seam>& seams) {
  ConeSphere2Setup st;
  if (seams.empty()) return st;  // need at least one traced seam to cross-check

  const CurvedSolid* conePtr = nullptr;
  const CurvedSolid* sphPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Cone) conePtr = s;
    else if (s->kind == CurvedKind::Sphere) sphPtr = s;
  }
  if (!conePtr || !sphPtr) return st;
  const CurvedSolid& cone = *conePtr;
  const CurvedSolid& sph = *sphPtr;

  const math::Vec3 zc = cone.frame.z.vec();
  const math::Point3 O = cone.frame.origin;
  const math::Point3 C = sph.frame.origin;
  const math::Vec3 d{C.x - O.x, C.y - O.y, C.z - O.z};
  const double sc = math::dot(d, zc);
  if (math::norm(d - zc * sc) > 1e-6) return st;  // sphere centre off the cone axis → not coaxial
  const double tanA = std::tan(cone.semiAngle);
  if (std::fabs(tanA) < 1e-9) return st;  // degenerate cone
  const double R0 = cone.radius, Rs = sph.radius;
  if (!(Rs > 1e-9)) return st;

  // Seam quadratic — require BOTH roots strictly interior to the cone extent.
  const double Aq = 1.0 + tanA * tanA;
  const double Bq = 2.0 * (R0 * tanA - sc);
  const double Cq = sc * sc + R0 * R0 - Rs * Rs;
  const double disc = Bq * Bq - 4.0 * Aq * Cq;
  if (disc <= 1e-9) return st;  // tangent / single-touch → decline (S5-f owns single root)
  const double sq = std::sqrt(disc);
  double r1 = (-Bq - sq) / (2.0 * Aq), r2 = (-Bq + sq) / (2.0 * Aq);
  if (r1 > r2) std::swap(r1, r2);
  const double coneS0 = cone.vLo, coneS1 = cone.vHi;
  if (!(r1 > coneS0 + 1e-6) || !(r2 < coneS1 - 1e-6)) return st;  // not both interior → S5-f / OCCT
  if (!(r2 - r1 > 1e-4)) return st;                               // roots too close → near-tangent → OCCT
  const double rhoLo = R0 + r1 * tanA, rhoHi = R0 + r2 * tanA;
  if (!(rhoLo > 1e-9) || !(rhoHi > 1e-9)) return st;              // apex-touching seam → decline

  const double poleM = sc - Rs, poleP = sc + Rs;

  // The two-circle "pokes through" pose: between the seams the SPHERE is wider (bulges outside the
  // cone); beyond them each polar cap is inside the cone. Verify with mid-band + cap samples.
  auto rCone = [&](double s) { return R0 + s * tanA; };
  auto rSph = [&](double s) {
    const double e = Rs * Rs - (s - sc) * (s - sc);
    return e > 0.0 ? std::sqrt(e) : 0.0;
  };
  const double sMid = 0.5 * (r1 + r2);
  if (!(rSph(sMid) > rCone(sMid) + 1e-6)) return st;  // sphere NOT wider on the mid-band → OCCT
  const double capLoMid = 0.5 * (poleM + r1), capHiMid = 0.5 * (r2 + poleP);
  if (!(rSph(capLoMid) < rCone(capLoMid) - 1e-6)) return st;  // lower cap not inside cone → OCCT
  if (!(rSph(capHiMid) < rCone(capHiMid) - 1e-6)) return st;  // upper cap not inside cone → OCCT

  // The whole sphere axial span must sit inside the cone extent (so both caps close inside the
  // cone and the CUT pinches cleanly), and both poles must classify INSIDE the cone.
  if (!(poleM > coneS0 + 1e-6) || !(poleP < coneS1 - 1e-6)) return st;
  if (classifyPoint(cone, math::Point3{C.x + zc.x * (poleM - sc), C.y + zc.y * (poleM - sc),
                                       C.z + zc.z * (poleM - sc)}, kSsiTol) != 1)
    return st;
  if (classifyPoint(cone, math::Point3{C.x + zc.x * (poleP - sc), C.y + zc.y * (poleP - sc),
                                       C.z + zc.z * (poleP - sc)}, kSsiTol) != 1)
    return st;

  // Cross-check EVERY traced seam against ONE of the two analytic circles (height + radius). A
  // traced loop that matches neither → the pair is not the clean two-circle config → OCCT.
  auto sOf = [&](const math::Point3& p) {
    return math::dot(math::Vec3{p.x - O.x, p.y - O.y, p.z - O.z}, zc);
  };
  for (const Seam& seam : seams) {
    if (!seam.closed || seam.pts.size() < 8) return st;
    math::Point3 c{0, 0, 0};
    for (const auto& p : seam.pts) { c.x += p.x; c.y += p.y; c.z += p.z; }
    const double ns = static_cast<double>(seam.pts.size());
    c.x /= ns; c.y /= ns; c.z /= ns;
    double rhoTr = 0.0;
    for (const auto& p : seam.pts) {
      const math::Vec3 w{p.x - c.x, p.y - c.y, p.z - c.z};
      rhoTr += math::norm(w - zc * math::dot(w, zc));
    }
    rhoTr /= ns;
    const double sTr = sOf(c);
    const bool matchLo = std::fabs(sTr - r1) < 1e-3 && std::fabs(rhoTr - rhoLo) < 1e-3;
    const bool matchHi = std::fabs(sTr - r2) < 1e-3 && std::fabs(rhoTr - rhoHi) < 1e-3;
    if (!matchLo && !matchHi) return st;  // traced seam matches neither analytic circle → OCCT
  }

  // Azimuthal resolution: the LARGER seam radius bounds the chord sagitta (same bound as the caps).
  const double rMax = std::max(rhoLo, rhoHi);
  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * rMax, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rMax / chord)), 24, 180);
  st.cone = conePtr;
  st.sph = sphPtr;
  st.O = O;
  st.X = cone.frame.x.vec();
  st.Y = cone.frame.y.vec();
  st.zc = zc;
  st.C = C;
  st.tanA = tanA;
  st.R0 = R0;
  st.Rs = Rs;
  st.sc = sc;
  st.coneS0 = coneS0;
  st.coneS1 = coneS1;
  st.sLo = r1;
  st.sHi = r2;
  st.rhoLo = rhoLo;
  st.rhoHi = rhoHi;
  st.poleM = poleM;
  st.poleP = poleP;
  st.ok = true;
  return st;
}

// ── SPHERE ZONE: a revolved band of the sphere surface between two seam rings (`ringLo` at
// station sLo, `ringHi` at sHi), following the spherical bulge. Unlike appendRevolvedBand (whose
// straight ruling is exact on a cone/cylinder), a sphere band bows between the two latitudes, so
// each meridian is subdivided into `rows` slerp steps of the unit radial direction (great-circle
// exact on the sphere, robust at the parametric pole). Outward radial normal (the FUSE bulge's
// outward boundary). All rows share the pool through the ring builder so the zone welds to the
// neighbouring cone bands along both seam rings.
// Primitive form: takes the sphere centre + radius directly so BOTH the S5-h cone∩sphere and
// the S5-i cylinder∩sphere two-circle assemblers share ONE zone builder (a cylinder is the
// tanα==0 special case; the zone geometry is identical — a revolved band of the sphere between
// two seam-latitude rings). Outward radial normal.
void appendSphereZone(const math::Point3& sphCentre, double Rs,
                      const std::vector<math::Point3>& ringLo,
                      const std::vector<math::Point3>& ringHi, VertexPool& pool,
                      std::vector<topo::Shape>& faces) {
  const int n = static_cast<int>(ringLo.size());
  if (n < 3 || static_cast<int>(ringHi.size()) != n) return;
  // Rows from the polar half-angle spanned by the zone at the sphere centre + kCapSagitta.
  const math::Vec3 dLo{ringLo[0].x - sphCentre.x, ringLo[0].y - sphCentre.y, ringLo[0].z - sphCentre.z};
  const math::Vec3 dHi{ringHi[0].x - sphCentre.x, ringHi[0].y - sphCentre.y, ringHi[0].z - sphCentre.z};
  const double denom = std::max(math::norm(dLo) * math::norm(dHi), 1e-12);
  const double theta = std::acos(std::clamp(math::dot(dLo, dHi) / denom, -1.0, 1.0));
  const int rows = std::clamp(
      static_cast<int>(std::ceil(std::max(theta, 1e-6) * std::sqrt(Rs / (2.0 * kCapSagitta)))), 2,
      48);
  // Per-meridian unit radials at the two seam rings; interior rows slerp between them.
  std::vector<math::Vec3> dirLo(n), dirHi(n);
  for (int i = 0; i < n; ++i) {
    const math::Vec3 a{ringLo[i].x - sphCentre.x, ringLo[i].y - sphCentre.y, ringLo[i].z - sphCentre.z};
    const math::Vec3 b{ringHi[i].x - sphCentre.x, ringHi[i].y - sphCentre.y, ringHi[i].z - sphCentre.z};
    const double la = std::max(math::norm(a), 1e-12), lb = std::max(math::norm(b), 1e-12);
    dirLo[i] = math::Vec3{a.x / la, a.y / la, a.z / la};
    dirHi[i] = math::Vec3{b.x / lb, b.y / lb, b.z / lb};
  }
  auto rowPt = [&](int r, int i) -> math::Point3 {
    if (r == 0) return ringLo[i];
    if (r == rows) return ringHi[i];
    const math::Vec3 dir = slerpDir(dirLo[i], dirHi[i], static_cast<double>(r) / rows);
    return math::Point3{sphCentre.x + dir.x * Rs, sphCentre.y + dir.y * Rs, sphCentre.z + dir.z * Rs};
  };
  for (int r = 0; r < rows; ++r)
    for (int i = 0; i < n; ++i) {
      const int j = (i + 1) % n;
      const math::Point3 a = rowPt(r, i), b = rowPt(r, j);
      const math::Point3 c = rowPt(r + 1, j), dd = rowPt(r + 1, i);
      const math::Point3 ctr{(a.x + b.x + c.x + dd.x) / 4, (a.y + b.y + c.y + dd.y) / 4,
                             (a.z + b.z + c.z + dd.z) / 4};
      const math::Vec3 ref{ctr.x - sphCentre.x, ctr.y - sphCentre.y, ctr.z - sphCentre.z};  // outward radial
      pushPlanarTri(a, b, c, ref, pool, faces);
      pushPlanarTri(a, c, dd, ref, pool, faces);
    }
}

// S5-h wrapper: delegates to the primitive with the setup's sphere centre + radius (byte-identical).
void appendSphereZone(const ConeSphere2Setup& s, double /*sLo*/, double /*sHi*/,
                      const std::vector<math::Point3>& ringLo,
                      const std::vector<math::Point3>& ringHi, VertexPool& pool,
                      std::vector<topo::Shape>& faces) {
  appendSphereZone(s.C, s.Rs, ringLo, ringHi, pool, faces);
}

// buildConeSphere2Common(A,B) = COMMON of the two-circle coaxial cone∩sphere: sphere lower cap +
// cone frustum band + sphere upper cap, welded along the two analytic seam rings.
topo::Shape buildConeSphere2Common(const CurvedSolid& A, const CurvedSolid& B,
                                   const std::vector<Seam>& seams) {
  const ConeSphere2Setup s = coneSphere2Setup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const Seam seamLo = s.seamRing(s.rhoLo, s.sLo);
  const Seam seamHi = s.seamRing(s.rhoHi, s.sHi);
  const std::vector<math::Point3> ringLo = seamLo.pts;
  const std::vector<math::Point3> ringHi = seamHi.pts;
  // Cap ring counts from each cap's polar half-angle + kCapSagitta.
  auto capRings = [&](const std::vector<math::Point3>& ring, const math::Point3& apex) {
    const math::Vec3 aDir{apex.x - s.C.x, apex.y - s.C.y, apex.z - s.C.z};
    double theta = 0.0;
    for (const auto& p : ring) {
      const math::Vec3 sDir{p.x - s.C.x, p.y - s.C.y, p.z - s.C.z};
      const double den = std::max(math::norm(aDir) * math::norm(sDir), 1e-12);
      theta = std::max(theta, std::acos(std::clamp(math::dot(aDir, sDir) / den, -1.0, 1.0)));
    }
    return std::clamp(
        static_cast<int>(std::ceil(std::max(theta, 1e-6) * std::sqrt(s.Rs / (2.0 * kCapSagitta)))),
        4, 48);
  };
  // Sphere lower cap: apex = poleM (the lower pole), outer ring = seamLo.
  appendSphereCap(*s.sph, s.axisPtM(), seamLo, capRings(ringLo, s.axisPtM()), pool, faces,
                  /*outer=*/false, /*reversed=*/false);
  // Cone frustum band inside the sphere (seamLo → seamHi).
  appendRevolvedBand(ringLo, ringHi, s.O, s.zc, pool, faces);
  // Sphere upper cap: apex = poleP.
  appendSphereCap(*s.sph, s.axisPtP(), seamHi, capRings(ringHi, s.axisPtP()), pool, faces,
                  /*outer=*/false, /*reversed=*/false);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildConeSphere2Fuse(A,B) = A ∪ B: cone wall (coneNear→seamLo) + sphere ZONE bulge (seamLo→
// seamHi) + cone wall (seamHi→coneFar) + two cone terminal discs. A GROW.
topo::Shape buildConeSphere2Fuse(const CurvedSolid& A, const CurvedSolid& B,
                                 const std::vector<Seam>& seams) {
  const ConeSphere2Setup s = coneSphere2Setup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ring0 = s.ring(s.rCone(s.coneS0), s.coneS0);
  const std::vector<math::Point3> ringLo = s.ring(s.rhoLo, s.sLo);
  const std::vector<math::Point3> ringHi = s.ring(s.rhoHi, s.sHi);
  const std::vector<math::Point3> ring1 = s.ring(s.rCone(s.coneS1), s.coneS1);
  if (!(s.rCone(s.coneS0) > 1e-9) || !(s.rCone(s.coneS1) > 1e-9)) return {};
  appendDiskCap(*s.cone, s.coneS0, ring0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ring0, ringLo, s.O, s.zc, pool, faces);      // cone wall below seamLo
  appendSphereZone(s, s.sLo, s.sHi, ringLo, ringHi, pool, faces);  // sphere bulge (mid-band)
  appendRevolvedBand(ringHi, ring1, s.O, s.zc, pool, faces);       // cone wall above seamHi
  appendDiskCap(*s.cone, s.coneS1, ring1, s.zc, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildConeSphere2Cut(A,B) = A − B (cone MINUEND). The sphere fully engulfs the cone cross-section
// on the mid-band, so the result is TWO disconnected components welded into one shell: a lower
// cone-tip piece (coneNear→seamLo, scooped from above by the sphere lower cap reversed) + an upper
// piece (seamHi→coneFar, scooped from below by the sphere upper cap reversed). A SHRINK.
// A sphere minuend (sphere−cone) declines → OCCT.
topo::Shape buildConeSphere2Cut(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  const ConeSphere2Setup s = coneSphere2Setup(A, B, seams);
  if (!s.ok) return {};
  if (&A != s.cone) return {};  // A must be the cone minuend; sphere−cone → OCCT
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const Seam seamLo = s.seamRing(s.rhoLo, s.sLo);
  const Seam seamHi = s.seamRing(s.rhoHi, s.sHi);
  const std::vector<math::Point3> ringLo = seamLo.pts;
  const std::vector<math::Point3> ringHi = seamHi.pts;
  const std::vector<math::Point3> ring0 = s.ring(s.rCone(s.coneS0), s.coneS0);
  const std::vector<math::Point3> ring1 = s.ring(s.rCone(s.coneS1), s.coneS1);
  if (!(s.rCone(s.coneS0) > 1e-9) || !(s.rCone(s.coneS1) > 1e-9)) return {};
  auto capRings = [&](const std::vector<math::Point3>& ring, const math::Point3& apex) {
    const math::Vec3 aDir{apex.x - s.C.x, apex.y - s.C.y, apex.z - s.C.z};
    double theta = 0.0;
    for (const auto& p : ring) {
      const math::Vec3 sDir{p.x - s.C.x, p.y - s.C.y, p.z - s.C.z};
      const double den = std::max(math::norm(aDir) * math::norm(sDir), 1e-12);
      theta = std::max(theta, std::acos(std::clamp(math::dot(aDir, sDir) / den, -1.0, 1.0)));
    }
    return std::clamp(
        static_cast<int>(std::ceil(std::max(theta, 1e-6) * std::sqrt(s.Rs / (2.0 * kCapSagitta)))),
        4, 48);
  };
  // Lower component: cone terminal disc + cone wall (coneNear→seamLo) + sphere lower cap REVERSED
  // (the dimple scooping the tip from above, apex = poleM).
  appendDiskCap(*s.cone, s.coneS0, ring0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ring0, ringLo, s.O, s.zc, pool, faces);
  appendSphereCap(*s.sph, s.axisPtM(), seamLo, capRings(ringLo, s.axisPtM()), pool, faces,
                  /*outer=*/false, /*reversed=*/true);
  // Upper component: sphere upper cap REVERSED (dimple, apex = poleP) + cone wall (seamHi→coneFar)
  // + cone terminal disc.
  appendSphereCap(*s.sph, s.axisPtP(), seamHi, capRings(ringHi, s.axisPtP()), pool, faces,
                  /*outer=*/false, /*reversed=*/true);
  appendRevolvedBand(ringHi, ring1, s.O, s.zc, pool, faces);
  appendDiskCap(*s.cone, s.coneS1, ring1, s.zc, pool, faces);
  if (faces.size() < 8) return {};  // two components → ≥8 faces
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── S5-i: TWO-CIRCLE coaxial CYLINDER∩SPHERE COMMON / FUSE / CUT. A cylinder (radius Rc, axis
// = z) coaxial with a sphere (radius Rs > Rc, centre C ON the cylinder axis) whose wall crosses
// the sphere at TWO latitudes — the "sphere pokes THROUGH the cylinder wall" pose, the
// tanα==0 special case of the S5-h cone∩sphere family. The seam equation is exact and clean:
//   r_cyl == r_sph(s)  ⇒  Rc = √(Rs²−(s−sc)²)  ⇒  s = sc ± √(Rs²−Rc²)  = sc ± h,
// two analytic circles of the SAME radius Rc at stations sLo = sc−h, sHi = sc+h. On the mid-band
// the sphere is wider (bulges outside the cylinder); each polar cap is inside the cylinder. All
// the S5-h/S5-c machinery is REUSED verbatim — appendRevolvedBand (a straight ruling is EXACT on
// a cylinder wall), appendDiskCap, appendSphereCap (inner/outer + reversed), and appendSphereZone
// (the FUSE mid-band bulge). No new builder is needed.
//
// DUAL ORACLE. Closed-form: COMMON = V_sph-cap(lower) + π·Rc²·(sHi−sLo) + V_sph-cap(upper), each
// cap of height (Rs−h): V_cap = π·hcap²·(3Rs−hcap)/3. FUSE = V(cyl)+V(sph)−V(COMMON) (GROW),
// CUT (cyl−sph) = V(cyl)−V(COMMON) (SHRINK, two disconnected end pieces). Plus OCCT parity.
//
// HONEST SCOPE. Coaxial only, sphere centre ON the cylinder axis, BOTH poles strictly inside the
// cylinder's axial extent, Rs > Rc (a strict two-circle poke-through). A single-crossing sphere
// (sphere pole outside the cylinder — the sphere just dents ONE end face), a tangent sphere
// (Rs==Rc → double root), a sphere fully containing / contained by the cylinder segment, and a
// non-coaxial (transversal) pair all decline → OCCT. A sphere-minuend CUT (sphere−cyl) declines.
struct CylSphere2Setup {
  bool ok = false;
  const CurvedSolid* cyl = nullptr;
  const CurvedSolid* sph = nullptr;
  math::Point3 O;          ///< cylinder frame origin (the s=0 station)
  math::Vec3 X, Y, zc;     ///< cylinder frame (azimuth + axis)
  math::Point3 C;          ///< sphere centre (on the cylinder axis)
  double Rc = 0.0, Rs = 0.0;
  double sc = 0.0;         ///< sphere-centre axial coord in the cylinder's s-frame
  double cylS0 = 0.0, cylS1 = 0.0;  ///< cylinder's own axial extent [vLo,vHi]
  double sLo = 0.0, sHi = 0.0;      ///< the TWO analytic crossings sc±h (sLo < sHi)
  double poleM = 0.0, poleP = 0.0;  ///< sphere axial poles (sc−Rs, sc+Rs)
  int N = 0;                        ///< azimuth sample count (seam-chord bounded)
  double rSph(double s) const {
    const double d = Rs * Rs - (s - sc) * (s - sc);
    return d > 0.0 ? std::sqrt(d) : 0.0;
  }
  std::vector<math::Point3> ring(double r, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = r * std::cos(u), cy = r * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  math::Point3 wallPoint(double r, double s) const {
    return math::Point3{O.x + X.x * r + zc.x * s, O.y + X.y * r + zc.y * s,
                        O.z + X.z * r + zc.z * s};
  }
  math::Point3 axisPtM() const {
    return math::Point3{C.x + zc.x * (poleM - sc), C.y + zc.y * (poleM - sc),
                        C.z + zc.z * (poleM - sc)};
  }
  math::Point3 axisPtP() const {
    return math::Point3{C.x + zc.x * (poleP - sc), C.y + zc.y * (poleP - sc),
                        C.z + zc.z * (poleP - sc)};
  }
  Seam seamRing(double rho, double s) const {
    Seam out;
    out.closed = true;
    out.pts = ring(rho, s);
    return out;
  }
};

CylSphere2Setup cylSphere2Setup(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  CylSphere2Setup st;
  if (seams.empty()) return st;  // need ≥1 traced seam to cross-check

  const CurvedSolid* cylPtr = nullptr;
  const CurvedSolid* sphPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Cylinder) cylPtr = s;
    else if (s->kind == CurvedKind::Sphere) sphPtr = s;
  }
  if (!cylPtr || !sphPtr) return st;
  const CurvedSolid& cyl = *cylPtr;
  const CurvedSolid& sph = *sphPtr;

  const math::Vec3 zc = cyl.frame.z.vec();
  const math::Point3 O = cyl.frame.origin;
  const math::Point3 C = sph.frame.origin;
  const math::Vec3 d{C.x - O.x, C.y - O.y, C.z - O.z};
  const double sc = math::dot(d, zc);
  if (math::norm(d - zc * sc) > 1e-6) return st;  // sphere centre off the cylinder axis → OCCT
  const double Rc = cyl.radius, Rs = sph.radius;
  if (!(Rc > 1e-9) || !(Rs > 1e-9)) return st;
  if (!(Rs > Rc + 1e-6)) return st;  // Rs ≤ Rc → no proper two-circle poke-through (tangent/nested) → OCCT

  const double h = std::sqrt(Rs * Rs - Rc * Rc);  // half axial gap between the two seam planes
  const double sLo = sc - h, sHi = sc + h;
  const double cylS0 = cyl.vLo, cylS1 = cyl.vHi;
  if (!(sLo > cylS0 + 1e-6) || !(sHi < cylS1 - 1e-6)) return st;  // seams not both interior → OCCT
  if (!(sHi - sLo > 1e-4)) return st;                             // roots too close → near-tangent → OCCT

  const double poleM = sc - Rs, poleP = sc + Rs;
  // Both poles must sit strictly inside the cylinder axial extent (both caps close inside the
  // cylinder and the CUT pinches into two clean end pieces).
  if (!(poleM > cylS0 + 1e-6) || !(poleP < cylS1 - 1e-6)) return st;

  // Cross-check EVERY traced seam against ONE of the two analytic circles (height sc±h, radius Rc).
  auto sOf = [&](const math::Point3& p) {
    return math::dot(math::Vec3{p.x - O.x, p.y - O.y, p.z - O.z}, zc);
  };
  for (const Seam& seam : seams) {
    if (!seam.closed || seam.pts.size() < 8) return st;
    math::Point3 c{0, 0, 0};
    for (const auto& p : seam.pts) { c.x += p.x; c.y += p.y; c.z += p.z; }
    const double ns = static_cast<double>(seam.pts.size());
    c.x /= ns; c.y /= ns; c.z /= ns;
    double rhoTr = 0.0;
    for (const auto& p : seam.pts) {
      const math::Vec3 w{p.x - c.x, p.y - c.y, p.z - c.z};
      rhoTr += math::norm(w - zc * math::dot(w, zc));
    }
    rhoTr /= ns;
    const double sTr = sOf(c);
    const bool matchLo = std::fabs(sTr - sLo) < 1e-3 && std::fabs(rhoTr - Rc) < 1e-3;
    const bool matchHi = std::fabs(sTr - sHi) < 1e-3 && std::fabs(rhoTr - Rc) < 1e-3;
    if (!matchLo && !matchHi) return st;  // traced seam matches neither analytic circle → OCCT
  }

  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * Rc, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * Rc / chord)), 24, 180);
  st.cyl = cylPtr;
  st.sph = sphPtr;
  st.O = O;
  st.X = cyl.frame.x.vec();
  st.Y = cyl.frame.y.vec();
  st.zc = zc;
  st.C = C;
  st.Rc = Rc;
  st.Rs = Rs;
  st.sc = sc;
  st.cylS0 = cylS0;
  st.cylS1 = cylS1;
  st.sLo = sLo;
  st.sHi = sHi;
  st.poleM = poleM;
  st.poleP = poleP;
  st.ok = true;
  return st;
}

// Cap ring-count from a cap's polar half-angle + kCapSagitta (shared by the S5-i builders).
int cylSphereCapRings(const CylSphere2Setup& s, const std::vector<math::Point3>& ring,
                      const math::Point3& apex) {
  const math::Vec3 aDir{apex.x - s.C.x, apex.y - s.C.y, apex.z - s.C.z};
  double theta = 0.0;
  for (const auto& p : ring) {
    const math::Vec3 sDir{p.x - s.C.x, p.y - s.C.y, p.z - s.C.z};
    const double den = std::max(math::norm(aDir) * math::norm(sDir), 1e-12);
    theta = std::max(theta, std::acos(std::clamp(math::dot(aDir, sDir) / den, -1.0, 1.0)));
  }
  return std::clamp(
      static_cast<int>(std::ceil(std::max(theta, 1e-6) * std::sqrt(s.Rs / (2.0 * kCapSagitta)))), 4,
      48);
}

// buildCylSphere2Common(A,B) = COMMON of the two-circle coaxial cylinder∩sphere: sphere lower cap
// + cylinder segment band + sphere upper cap, welded along the two analytic seam rings (both of
// radius Rc). The min-radius profile of revolution.
topo::Shape buildCylSphere2Common(const CurvedSolid& A, const CurvedSolid& B,
                                  const std::vector<Seam>& seams) {
  const CylSphere2Setup s = cylSphere2Setup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const Seam seamLo = s.seamRing(s.Rc, s.sLo);
  const Seam seamHi = s.seamRing(s.Rc, s.sHi);
  const std::vector<math::Point3> ringLo = seamLo.pts;
  const std::vector<math::Point3> ringHi = seamHi.pts;
  appendSphereCap(*s.sph, s.axisPtM(), seamLo, cylSphereCapRings(s, ringLo, s.axisPtM()), pool,
                  faces, /*outer=*/false, /*reversed=*/false);
  appendRevolvedBand(ringLo, ringHi, s.O, s.zc, pool, faces);  // cylinder segment (straight ruling)
  appendSphereCap(*s.sph, s.axisPtP(), seamHi, cylSphereCapRings(s, ringHi, s.axisPtP()), pool,
                  faces, /*outer=*/false, /*reversed=*/false);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildCylSphere2Fuse(A,B) = A ∪ B: cylinder wall (cylEnd→seamLo) + sphere ZONE bulge (seamLo→
// seamHi, the mid-band where the sphere is wider) + cylinder wall (seamHi→cylEnd) + two cylinder
// terminal discs. A GROW. V = V(cyl)+V(sph)−V(COMMON).
topo::Shape buildCylSphere2Fuse(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  const CylSphere2Setup s = cylSphere2Setup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ring0 = s.ring(s.Rc, s.cylS0);
  const std::vector<math::Point3> ringLo = s.ring(s.Rc, s.sLo);
  const std::vector<math::Point3> ringHi = s.ring(s.Rc, s.sHi);
  const std::vector<math::Point3> ring1 = s.ring(s.Rc, s.cylS1);
  appendDiskCap(*s.cyl, s.cylS0, ring0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ring0, ringLo, s.O, s.zc, pool, faces);      // cylinder wall below seamLo
  appendSphereZone(s.C, s.Rs, ringLo, ringHi, pool, faces);       // sphere bulge (mid-band)
  appendRevolvedBand(ringHi, ring1, s.O, s.zc, pool, faces);      // cylinder wall above seamHi
  appendDiskCap(*s.cyl, s.cylS1, ring1, s.zc, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildCylSphere2Cut(A,B) = A − B (cylinder MINUEND). The sphere fully engulfs the cylinder
// cross-section on the mid-band, so the result PINCHES into TWO disconnected components welded
// into one shell: a lower cylinder-end piece (cylNear→seamLo, its top scooped by the sphere lower
// cap reversed) + an upper piece (seamHi→cylFar, scooped by the sphere upper cap reversed). A
// SHRINK. V = V(cyl)−V(COMMON). A sphere-minuend (sphere−cyl) declines → OCCT.
topo::Shape buildCylSphere2Cut(const CurvedSolid& A, const CurvedSolid& B,
                               const std::vector<Seam>& seams) {
  const CylSphere2Setup s = cylSphere2Setup(A, B, seams);
  if (!s.ok) return {};
  if (&A != s.cyl) return {};  // A must be the cylinder minuend; sphere−cyl → OCCT
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const Seam seamLo = s.seamRing(s.Rc, s.sLo);
  const Seam seamHi = s.seamRing(s.Rc, s.sHi);
  const std::vector<math::Point3> ringLo = seamLo.pts;
  const std::vector<math::Point3> ringHi = seamHi.pts;
  const std::vector<math::Point3> ring0 = s.ring(s.Rc, s.cylS0);
  const std::vector<math::Point3> ring1 = s.ring(s.Rc, s.cylS1);
  // Lower component: cylinder terminal disc + cylinder wall (cylNear→seamLo) + sphere lower cap
  // REVERSED (the dimple scooping from above, apex = poleM).
  appendDiskCap(*s.cyl, s.cylS0, ring0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ring0, ringLo, s.O, s.zc, pool, faces);
  appendSphereCap(*s.sph, s.axisPtM(), seamLo, cylSphereCapRings(s, ringLo, s.axisPtM()), pool,
                  faces, /*outer=*/false, /*reversed=*/true);
  // Upper component: sphere upper cap REVERSED (dimple, apex = poleP) + cylinder wall (seamHi→
  // cylFar) + cylinder terminal disc.
  appendSphereCap(*s.sph, s.axisPtP(), seamHi, cylSphereCapRings(s, ringHi, s.axisPtP()), pool,
                  faces, /*outer=*/false, /*reversed=*/true);
  appendRevolvedBand(ringHi, ring1, s.O, s.zc, pool, faces);
  appendDiskCap(*s.cyl, s.cylS1, ring1, s.zc, pool, faces);
  if (faces.size() < 8) return {};  // two components → ≥8 faces
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── The COMMON of a THROUGH-DRILL transversal pair (design.md §1-4): one operand (the
// PIERCED wall, whose two seams are full-circle rim loops) is drilled clean through by
// the other (the PIERCING wall, whose two seams are local patches). The common region
// is the piercing TUBE inside the pierced solid, capped at each end by the pierced
// wall's drill-mouth patch. Its boundary is therefore:
//   * the piercing-wall TUBE BAND between the two full-circle rim seams (appendTubeBand
//     — a quad strip that shares the traced seam nodes with the caps),
//   * TWO pierced-wall DRILL-MOUTH PATCHES, each the SMALL region bounded by one seam
//     loop (seamWire → ear-clipped mouth patch, NOT the whole wall).
// All faces draw their seam vertices from ONE shared VertexPool, so the tube band and
// caps weld along the two seams. Both walls carry their EXACT analytic surface (no
// faceting); the host oracle is the mesh volume vs the analytic value (engine
// self-verify), which also owns the watertight gate + OCCT fallback.
//
// Requires EXACTLY TWO closed seams, one operand full-circle on BOTH (the tube), the
// other local on BOTH (the mouths), and the tube band's interior sample INSIDE the
// pierced solid. Any other configuration → NULL (declined → OCCT), never faked.
//
// The seam-assembly geometry (periodic rim alignment, cap region selection) is isolated
// in appendTubeBand / seamWire (systems-band, flagged there); this driver is a short
// linear composition per the complexity policy.
topo::Shape buildCommon(const CurvedSolid& A, const CurvedSolid& B,
                        const std::vector<Seam>& seams) {
  if (seams.size() != 2) return {};
  for (const Seam& s : seams)
    if (!s.closed || s.pts.size() < 4) return {};

  // Decide which operand is the piercing TUBE (full-circle on both seams) and which is
  // the PIERCED wall (local patch on both seams).
  const bool aTube = isFullCircle(seams[0].uvA) && isFullCircle(seams[1].uvA);
  const bool bTube = isFullCircle(seams[0].uvB) && isFullCircle(seams[1].uvB);
  const bool aPierced = !isFullCircle(seams[0].uvA) && !isFullCircle(seams[1].uvA);
  const bool bPierced = !isFullCircle(seams[0].uvB) && !isFullCircle(seams[1].uvB);
  if (!((aTube && bPierced) || (bTube && aPierced))) return {};  // not a clean through-drill

  const CurvedSolid& tube = aTube ? A : B;
  const CurvedSolid& pierced = aTube ? B : A;
  auto tubeUV = [&](int i) -> const std::vector<std::pair<double, double>>& {
    return aTube ? seams[i].uvA : seams[i].uvB;
  };
  auto piercedUV = [&](int i) -> const std::vector<std::pair<double, double>>& {
    return aTube ? seams[i].uvB : seams[i].uvA;
  };

  // The tube band between the two rim seams must be INSIDE the pierced solid (the
  // common survival rule: keep the inside-the-other fragment). Sample its mid station.
  const double tubeMidV = 0.5 * (meanV(tubeUV(0)) + meanV(tubeUV(1)));
  const double tubeMidU = meanU(tubeUV(0));
  if (classifyPoint(pierced, tube.point(tubeMidU, tubeMidV), kSsiTol) != 1) return {};

  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendTubeBand(tube, seams, tubeUV(0), tubeUV(1), pool, faces);

  // The two drill-mouth CAPS: each the SMALL patch of the pierced wall bounded by one
  // seam loop. Radially refined into planar facets that follow the pierced cylinder's
  // bulge across the mouth (appendMouthCap) — a flat ear-clip would chord that bulge and
  // remove ≈2.5× the true mouth cavity. The refinement count is derived from the mouth's
  // angular span on the pierced wall and its curvature-per-facet target (NOT hand-tuned):
  // rings ≈ arc / step so each facet's sagitta on the pierced cylinder is bounded.
  for (int i = 0; i < 2; ++i) {
    const double uSpanMouth = uSpan(piercedUV(i));           // mouth angular span (rad)
    const double sagStep = std::sqrt(8.0 * kCapSagitta / std::max(pierced.radius, 1e-9));
    const int rings = std::clamp(static_cast<int>(std::ceil(uSpanMouth / sagStep)), 2, 64);
    appendMouthCap(pierced, seams[i], piercedUV(i), rings, pool, faces);
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// S5-d — the BRANCHED-TRACE Steinmetz assembler (branch-point self-crossing).
//
// Two EQUAL-radius cylinders whose axes cross ORTHOGONALLY (the Steinmetz config)
// intersect in TWO ellipses that cross at TWO branch points. The S4-d marcher
// (MarchOptions.enableBranchPoints = true) localizes the two branch points and routes
// the four branch-to-branch arcs (WLine status BranchArc). This assembler consumes
// that branched TraceSet and builds the boolean directly from the four arcs.
//
// TOPOLOGY (verified against the live trace, R=1, axes Z & X through the origin):
//   * branchPoints == 2 at (0, ±R, 0); all four arms meet at BOTH branch nodes.
//   * four BranchArc arms, each running pole-to-pole (full y ∈ [−R, R]); each carries
//     (u1,v1) on cyl-A and (u2,v2) on cyl-B per node.
//   * on EACH cylinder the region INSIDE the other is TWO lune patches; a lune is
//     bounded by the two arcs that lie on the SAME half of that cylinder (same folded
//     mean u). The four lunes (two per cylinder) tile the COMMON boundary, sharing the
//     four arcs (each arc is the crease between one cyl-A lune and one cyl-B lune) and
//     the two branch-point poles (all four lunes taper to both poles).
//
// The COMMON is the four lunes welded on the four arcs + the two poles. FUSE / CUT
// re-select which wall fragments survive (the outside-the-other regions), keeping the
// four arcs as the shared seams. Every seam-adjacent facet is a PLANAR triangle through
// SHARED pooled nodes (the S5-a discipline — the analytic surface face's structured-grid
// mesh would inject interior u-lines and open a T-junction; planar facets carry none),
// the two branch-point vertices are pooled ONCE, and the shell welds watertight.
// Anything not a recognised equal-R orthogonal Steinmetz branched pair → NULL → OCCT.
// ─────────────────────────────────────────────────────────────────────────────

// A branch arc: the shared 3D polyline + its (u,v) track on EACH cylinder, RESAMPLED
// so it runs monotone from pole0 → pole1 (aligned to the two branch-point poles). The
// endpoints are snapped to the exact pole points so all four arcs share the same two
// pole vertices when pooled.
struct BranchArcData {
  std::vector<math::Point3> pts;                  ///< shared 3D nodes, pole0 → pole1
  std::vector<std::pair<double, double>> uvA;     ///< (u,v) on cyl-A per node
  std::vector<std::pair<double, double>> uvB;     ///< (u,v) on cyl-B per node
};

// Which cylinder a lune sits on, and the accessor that returns that cylinder's (u,v)
// track for an arc.
enum class LuneCyl { A, B };

// Orient one arc pole0 → pole1: its nodes are the traced arc; if its first node is
// nearer pole1 than pole0, reverse it (keeping 3D and both (u,v) tracks in lockstep),
// then snap the two endpoints to the exact pole points.
BranchArcData orientArc(const Seam& s, const math::Point3& pole0, const math::Point3& pole1) {
  BranchArcData a{s.pts, s.uvA, s.uvB};
  if (a.pts.size() < 2) return a;
  const double d0 = math::norm(a.pts.front() - pole0);
  const double d1 = math::norm(a.pts.front() - pole1);
  if (d1 < d0) {
    std::reverse(a.pts.begin(), a.pts.end());
    std::reverse(a.uvA.begin(), a.uvA.end());
    std::reverse(a.uvB.begin(), a.uvB.end());
  }
  a.pts.front() = pole0;
  a.pts.back() = pole1;
  return a;
}

// Resample an arc onto a COMMON pole-axis grid: `tvals[k]` is the fractional distance
// along the pole0→pole1 axis (the y direction: t = (P−pole0)·axis / |pole1−pole0|). Both
// arcs of a lune are resampled onto the SAME tvals so lo[k] and hi[k] sit at the same y —
// the lune strip is then a clean pole-axis-perpendicular ribbon (no matched-index skew,
// which would let the ribbon self-cross and double-cover). Linear interp between the
// bracketing traced nodes, in 3D and BOTH (u,v) tracks; endpoints stay the exact poles.
BranchArcData resampleArcByAxis(const BranchArcData& a, const math::Vec3& axisU,
                                double axisLen, const std::vector<double>& tvals,
                                const math::Point3& pole0) {
  BranchArcData out;
  const int m = static_cast<int>(a.pts.size());
  std::vector<double> at(m);
  for (int i = 0; i < m; ++i)
    at[i] = math::dot(a.pts[i] - pole0, axisU) / std::max(axisLen, 1e-12);
  auto lerpPair = [](std::pair<double, double> p, std::pair<double, double> q, double f) {
    return std::make_pair(p.first + (q.first - p.first) * f, p.second + (q.second - p.second) * f);
  };
  for (double t : tvals) {
    int i = 0;
    while (i + 1 < m && at[i + 1] < t) ++i;
    const int j = std::min(i + 1, m - 1);
    const double span = at[j] - at[i];
    const double f = (std::fabs(span) < 1e-12) ? 0.0 : std::clamp((t - at[i]) / span, 0.0, 1.0);
    out.pts.push_back(math::Point3{a.pts[i].x + (a.pts[j].x - a.pts[i].x) * f,
                                   a.pts[i].y + (a.pts[j].y - a.pts[i].y) * f,
                                   a.pts[i].z + (a.pts[j].z - a.pts[i].z) * f});
    out.uvA.push_back(lerpPair(a.uvA[i], a.uvA[j], f));
    out.uvB.push_back(lerpPair(a.uvB[i], a.uvB[j], f));
  }
  return out;
}

// Mean folded u of an arc on cylinder `which` — decides which HALF of that cylinder the
// arc lies on (two arcs on the same half bound one lune). Folded contiguous about the
// first node's u so the periodic wrap does not corrupt the mean.
double arcMeanU(const BranchArcData& a, LuneCyl which) {
  const auto& uv = (which == LuneCyl::A) ? a.uvA : a.uvB;
  const double u0 = uv.front().first;
  double sum = 0.0;
  for (const auto& p : uv) sum += nearU(u0, p.first);
  return sum / static_cast<double>(uv.size());
}

// Build ONE lune patch on cylinder `cs` (kind `which`) bounded by arcs `lo` and `hi`
// (both oriented pole0 → pole1). Walk the two arcs in lockstep by index fraction; at
// each step interior-sample the cylinder in u between the two arcs' u-values (folded
// contiguous) so each planar facet's bow off the true wall is bounded, and emit a strip
// of planar triangles. `refInside` = the other solid — a lune facet's outward normal is
// the cylinder's radial-out (COMMON keeps the inside-the-other patch, whose outward
// normal is the wall's outward radial). The two poles are pooled ONCE (shared by all
// four lunes), and each arc's nodes are pooled so both owning lunes share them → weld.
//
// systems-band (~18 — dual-arc lockstep walk + interior u-fold sampling); flagged.
void appendLunePatch(const CurvedSolid& cs, LuneCyl which, const BranchArcData& lo,
                     const BranchArcData& hi, int uSamples, double outwardSign, VertexPool& pool,
                     std::vector<topo::Shape>& faces) {
  const int n = std::min(static_cast<int>(lo.pts.size()), static_cast<int>(hi.pts.size()));
  if (n < 3) return;
  auto uvOf = [&](const BranchArcData& a, int k) {
    return (which == LuneCyl::A) ? a.uvA[k] : a.uvB[k];
  };
  // Fold BOTH arcs' u-tracks contiguous, PER-STEP, anchored to the lo-arc node at THIS k
  // (never a global base — the lune spans ~π of u pole-to-pole, so no single window keeps
  // it contiguous). Interior column points are sampled on the cylinder across the lune
  // WIDTH; because the two bounding arcs share the same y (hence ~the same u) at matched k,
  // that width is essentially a v-ruling (a cylinder is straight in v), so the interior
  // samples add negligible bow but keep every facet's chord on the true wall.
  const math::Point3 axisPt = cs.frame.origin;
  const math::Dir3 axisDir = cs.frame.z;
  auto tri = [&](const math::Point3& p, const math::Point3& q, const math::Point3& s) {
    const math::Point3 ctr{(p.x + q.x + s.x) / 3, (p.y + q.y + s.y) / 3, (p.z + q.z + s.z) / 3};
    const math::Vec3 w = ctr - axisPt;
    const math::Vec3 radial = w - axisDir.vec() * math::dot(w, axisDir.vec());
    pushPlanarTri(p, q, s, math::Vec3{radial.x * outwardSign, radial.y * outwardSign,
                                      radial.z * outwardSign}, pool, faces);
  };
  // Column j (0..uSamples): a chain of world points across the lune width at arc-step k. j=0
  // is the `lo` arc node (shared, pooled), j=uSamples is the `hi` arc node (shared) — the
  // exact traced nodes so the seam welds against the owning lune; interior j on the cylinder.
  auto colPt = [&](int k, int j) -> math::Point3 {
    if (j == 0) return lo.pts[k];
    if (j == uSamples) return hi.pts[k];
    // At a POLE station the two bounding arcs collapse to the SAME (shared) pole point;
    // the whole width column is that exact pole. Interpolating (u,v) there and re-evaluating
    // cs.point would drift ~1e-4 off the pole and spawn near-degenerate slivers the mesher
    // splits inconsistently (a T-junction → not watertight). Return the exact pole instead.
    if (math::norm(lo.pts[k] - hi.pts[k]) < kSsiTol) return lo.pts[k];
    const auto uvLo = uvOf(lo, k);
    const auto uvHi = uvOf(hi, k);
    const double t = static_cast<double>(j) / uSamples;
    const double u = uvLo.first + (nearU(uvLo.first, uvHi.first) - uvLo.first) * t;
    const double v = uvLo.second + (uvHi.second - uvLo.second) * t;
    return cs.point(u, v);
  };
  for (int k = 0; k + 1 < n; ++k)
    for (int j = 0; j < uSamples; ++j) {
      const math::Point3 a00 = colPt(k, j), a01 = colPt(k, j + 1);
      const math::Point3 a10 = colPt(k + 1, j), a11 = colPt(k + 1, j + 1);
      tri(a00, a10, a11);
      tri(a00, a11, a01);
    }
}

// Recognise the Steinmetz-family branched TraceSet: nearTangentGaps == 0, exactly two
// branch nodes, exactly four BranchArc arms all on both cylinders (≤ onSurfTol), and
// both branch nodes connecting all four arms. Returns the two pole points + the four
// arcs (as Seams) on success; nullopt → decline → OCCT.
struct SteinmetzTrace {
  math::Point3 pole0, pole1;
  std::vector<Seam> arcs;  ///< the four branch arcs (uvA on cyl-A, uvB on cyl-B)
};
std::optional<SteinmetzTrace> recogniseSteinmetzTrace(const ssi::TraceSet& t) {
  if (t.nearTangentGaps > 0) return std::nullopt;
  if (t.branchPoints != 2 || t.branchNodes.size() != 2) return std::nullopt;
  if (t.lines.size() != 4) return std::nullopt;
  for (const ssi::WLine& w : t.lines) {
    if (w.status != ssi::TraceStatus::BranchArc) return std::nullopt;
    if (w.points.size() < 3) return std::nullopt;
    if (w.onSurfResidual > 10.0 * kSsiTol) return std::nullopt;  // must sit on both walls
  }
  // Both branch nodes must connect all four arms (each arm's branchId listed).
  for (const ssi::BranchNode& bn : t.branchNodes)
    if (bn.armLineIds.size() < 4) return std::nullopt;
  SteinmetzTrace out;
  out.pole0 = t.branchNodes[0].point;
  out.pole1 = t.branchNodes[1].point;
  if (math::norm(out.pole0 - out.pole1) < kSsiTol) return std::nullopt;  // coincident poles
  out.arcs.reserve(4);
  for (const ssi::WLine& w : t.lines) out.arcs.push_back(toSeam(w));
  return out;
}

// Group the four arcs into the two lunes on cylinder `which` (each lune = the two arcs on
// the same half of that cylinder, by folded mean u). Returns false if the split is not a
// clean 2+2 (→ decline). Each pair is returned as an (lo,hi) ordered by mean v so the lune
// strip walks from the low-v boundary to the high-v boundary.
bool groupLunes(const std::vector<BranchArcData>& arcs, LuneCyl which,
                std::array<std::pair<int, int>, 2>& lunesOut) {
  // Cluster the four arcs by mean u into two groups (nearest-of-two by wrapped distance).
  std::array<double, 4> mu{};
  for (int i = 0; i < 4; ++i) mu[i] = arcMeanU(arcs[i], which);
  // Seed group centres at the two most-separated arcs.
  int seedA = 0, seedB = 1;
  double best = -1.0;
  for (int i = 0; i < 4; ++i)
    for (int j = i + 1; j < 4; ++j) {
      const double d = wrapDiff(mu[i], mu[j]);
      if (d > best) { best = d; seedA = i; seedB = j; }
    }
  std::vector<int> gA, gB;
  for (int i = 0; i < 4; ++i)
    (wrapDiff(mu[i], mu[seedA]) <= wrapDiff(mu[i], mu[seedB]) ? gA : gB).push_back(i);
  if (gA.size() != 2 || gB.size() != 2) return false;  // not a clean 2+2 → decline
  auto meanV = [&](int i) {
    const auto& uv = (which == LuneCyl::A) ? arcs[i].uvA : arcs[i].uvB;
    double s = 0.0;
    for (const auto& p : uv) s += p.second;
    return s / static_cast<double>(uv.size());
  };
  auto order = [&](std::vector<int>& g) -> std::pair<int, int> {
    return (meanV(g[0]) <= meanV(g[1])) ? std::make_pair(g[0], g[1])
                                        : std::make_pair(g[1], g[0]);
  };
  lunesOut[0] = order(gA);
  lunesOut[1] = order(gB);
  return true;
}

// Facet-refinement count across a lune's width: keep each planar facet's chord bow off the
// cylinder within kCapSagitta (the S5-a discipline). The width is the max folded-u span of
// the lune's two bounding arcs; uSamples ≈ span / sqrt(8·kCapSagitta/R). Bounded [2,64].
int luneUSamples(const CurvedSolid& cs, const BranchArcData& lo, const BranchArcData& hi,
                 LuneCyl which) {
  double span = 0.0;
  const int n = std::min(static_cast<int>(lo.pts.size()), static_cast<int>(hi.pts.size()));
  for (int k = 0; k < n; ++k) {
    const auto a = (which == LuneCyl::A) ? lo.uvA[k] : lo.uvB[k];
    const auto b = (which == LuneCyl::A) ? hi.uvA[k] : hi.uvB[k];
    span = std::max(span, std::fabs(nearU(a.first, b.first) - a.first));
  }
  const double step = std::sqrt(8.0 * kCapSagitta / std::max(cs.radius, 1e-9));
  return std::clamp(static_cast<int>(std::ceil(span / std::max(step, 1e-6))), 2, 64);
}

// Orient + resample the four Steinmetz arcs onto ONE common pole-axis grid (shared by ALL
// three builders so their weld nodes are byte-identical). All four arcs are oriented
// pole0 → pole1, then RESAMPLED onto the same cosine-clustered tvals. Two lunes/strips
// sharing an arc then reference IDENTICAL 3D nodes at each grid station → the seam welds;
// and lo[k], hi[k] of every lune sit at the SAME pole-axis station (matched y) so a ribbon
// is a clean pole-perpendicular strip that cannot self-cross / double-cover. Returns the
// four resampled arcs (empty on a degenerate pole axis → the caller declines → OCCT).
//
// Factored verbatim out of buildSteinmetzCommon so COMMON stays byte-identical: the arc
// count, cosine grid and pole snap are UNCHANGED — FUSE/CUT merely re-select fragments.
std::vector<BranchArcData> orientResampleArcs(const CurvedSolid& refR, const SteinmetzTrace& st) {
  const math::Vec3 axisVec = st.pole1 - st.pole0;
  const double axisLen = math::norm(axisVec);
  if (axisLen < kSsiTol) return {};
  const math::Vec3 axisU{axisVec.x / axisLen, axisVec.y / axisLen, axisVec.z / axisLen};
  // Node count along the arc from a chord-sagitta bound (the arc is ~a quarter ellipse of
  // extent ~πR/2): nn ≈ arcLen / sqrt(8·kCapSagitta·R). Bounded [24,180]. Grid stations are
  // COSINE-clustered toward the two poles (where all four lunes taper) so the tapering tip
  // facets stay well-shaped.
  const double arcLen = 0.5 * kSsiPi * std::max(refR.radius, 1e-9);
  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * refR.radius, 1e-12));
  const int nn = std::clamp(static_cast<int>(std::ceil(arcLen / std::max(chord, 1e-9))), 24, 180);
  std::vector<double> tvals(nn);
  for (int k = 0; k < nn; ++k)
    tvals[k] = 0.5 * (1.0 - std::cos(kSsiPi * k / (nn - 1)));  // 0..1, clustered at the poles
  std::vector<BranchArcData> arcs;
  arcs.reserve(4);
  for (const Seam& s : st.arcs)
    arcs.push_back(resampleArcByAxis(orientArc(s, st.pole0, st.pole1), axisU, axisLen, tvals,
                                     st.pole0));
  // Snap the two grid endpoints to the exact shared poles (all arcs → the same two nodes).
  for (auto& a : arcs) { a.pts.front() = st.pole0; a.pts.back() = st.pole1; }
  return arcs;
}

// buildSteinmetzCommon(A,B) = the bicylinder: the four inside-the-other lune patches
// (two on cyl-A, two on cyl-B) welded on the four shared arcs + the two shared poles.
// Each lune is kept iff its centroid classifies INSIDE the other solid (the COMMON
// survival rule); an ON verdict → NULL → OCCT (never faked). One VertexPool welds the
// whole shell: the two poles are pooled ONCE and each arc's nodes are shared by its two
// owning lunes (cyl-A side and cyl-B side).
topo::Shape buildSteinmetzCommon(const CurvedSolid& A, const CurvedSolid& B,
                                 const SteinmetzTrace& st) {
  const std::vector<BranchArcData> arcs = orientResampleArcs(A, st);
  if (arcs.size() != 4) return {};

  VertexPool pool;
  std::vector<topo::Shape> faces;
  // Two lunes on cyl-A (inside B) and two on cyl-B (inside A).
  struct LuneSpec { const CurvedSolid& cs; LuneCyl which; const CurvedSolid& other; };
  const std::array<LuneSpec, 2> specs{LuneSpec{A, LuneCyl::A, B}, LuneSpec{B, LuneCyl::B, A}};
  for (const LuneSpec& sp : specs) {
    std::array<std::pair<int, int>, 2> lunes;
    if (!groupLunes(arcs, sp.which, lunes)) return {};
    for (const auto& [iLo, iHi] : lunes) {
      const BranchArcData& lo = arcs[iLo];
      const BranchArcData& hi = arcs[iHi];
      // Keep the lune iff its centroid is INSIDE the other solid (COMMON survival rule).
      const int mid = static_cast<int>(lo.pts.size()) / 2;
      const auto uvLo = (sp.which == LuneCyl::A) ? lo.uvA[mid] : lo.uvB[mid];
      const auto uvHi = (sp.which == LuneCyl::A) ? hi.uvA[mid] : hi.uvB[mid];
      const double uMid = 0.5 * (uvLo.first + nearU(uvLo.first, uvHi.first));
      const double vMid = 0.5 * (uvLo.second + uvHi.second);
      const math::Point3 centroid = sp.cs.point(uMid, vMid);
      const int cls = classifyPoint(sp.other, centroid, kSsiTol);
      if (cls == 0) return {};        // ON the other wall → tangent → OCCT
      if (cls != 1) continue;         // OUTSIDE → not a COMMON lune (should not happen)
      const int uSamp = luneUSamples(sp.cs, lo, hi, sp.which);
      appendLunePatch(sp.cs, sp.which, lo, hi, uSamp, /*outwardSign=*/1.0, pool, faces);
    }
  }
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// S5-d FUSE / CUT — the SAME branched trace, different fragment selection + caps.
//
//  * FUSE = A ∪ B: the OUTSIDE-the-other wall region of BOTH cylinders (each cylinder's
//    full wall with its two INSIDE-the-other lunes cut out as mouths) + all four original
//    disc end caps, welded along the four shared arcs.
//  * CUT(A,B) = A − B: A's OUTSIDE wall (its two inside-A lunes cut out) + A's two disc
//    caps + B's two INSIDE-A lunes emitted REVERSED (inward normal) — the wall of the
//    channel B carves through A. A is the order-honored minuend.
//
// The OUTSIDE wall is NOT a simple two-arc strip (it wraps around and reaches the caps at
// v=vLo/vHi), so — exactly like the through-drill fat wall — it is a STRUCTURED (u,v) grid
// of the full cylinder with the two inside-lune regions REMOVED as mouths, ribbon-stitched
// to the arc seams (appendHoledWall). Each inside lune is a CLOSED (u,v) loop bounded by its
// two arcs meeting at the two poles, so it is a valid "mouth"; the two lunes on one cylinder
// are the two mouths. The mouth 3D nodes are the SAME resampled arc nodes the reversed B
// lunes / the other cylinder's mouth use → the whole shell welds through one VertexPool.
// ─────────────────────────────────────────────────────────────────────────────

// The two INSIDE-the-other lunes on cylinder `which` as (lo,hi) arc-index pairs, ordered by
// mean v — the SAME clean 2+2 clustering COMMON uses (groupLunes). false → decline.
bool insideLunesOn(const std::vector<BranchArcData>& arcs, LuneCyl which,
                   std::array<std::pair<int, int>, 2>& lunesOut) {
  return groupLunes(arcs, which, lunesOut);
}

// Build a CLOSED mouth-loop Seam for ONE inside lune on cylinder `which`: walk arc `lo`
// pole0 → pole1, then arc `hi` pole1 → pole0, so the concatenated (u,v) track forms a single
// closed loop bounding the lune. The 3D nodes are the shared resampled arc nodes (so the
// mouth welds against the reversed lune / the other cylinder's fragment that owns the same
// arcs). The pole endpoints are shared, so drop the duplicate at each junction.
Seam luneMouthSeam(const BranchArcData& lo, const BranchArcData& hi, LuneCyl which) {
  Seam m;
  (void)which;  // the loop tracks BOTH cylinders' (u,v); the caller selects which for the mouth
  const int n = std::min(static_cast<int>(lo.pts.size()), static_cast<int>(hi.pts.size()));
  auto push = [&](const BranchArcData& a, int k) {
    m.pts.push_back(a.pts[k]);
    m.uvA.push_back(a.uvA[k]);
    m.uvB.push_back(a.uvB[k]);
  };
  for (int k = 0; k < n; ++k) push(lo, k);                 // lo: pole0 → pole1
  for (int k = n - 2; k >= 1; --k) push(hi, k);            // hi: pole1 → pole0 (skip shared poles)
  return m;
}

// v-cell / u-cell resolution for a Steinmetz outer wall — mirrors wallSteps: u finely
// sampled around the full circle (curvature), v modest but fine enough that a mouth spans
// several cells. The mouth v-span is the two arcs' v-extent on `which`.
void steinmetzWallSteps(const CurvedSolid& cs, const std::vector<BranchArcData>& arcs,
                        LuneCyl which, int& uCells, int& vCells) {
  const double sagStep = std::sqrt(8.0 * kCapSagitta / std::max(cs.radius, 1e-9));
  uCells = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi / std::max(sagStep, 1e-6))), 16, 512);
  double vLoM = 1e300, vHiM = -1e300;
  for (const BranchArcData& a : arcs)
    for (const auto& uv : (which == LuneCyl::A ? a.uvA : a.uvB)) {
      vLoM = std::min(vLoM, uv.second);
      vHiM = std::max(vHiM, uv.second);
    }
  const double mouthVspan = std::max(vHiM - vLoM, 1e-6);
  const double vCell = std::max(mouthVspan / 6.0, 1e-6);
  vCells = std::clamp(static_cast<int>(std::ceil((cs.vHi - cs.vLo) / vCell)), 4, 256);
}

// Emit the OUTSIDE-the-other wall of cylinder `cs` (full wall minus its two inside lunes)
// + its two disc end caps into (pool, faces). The two inside lunes become the two mouths of
// appendHoledWall. Declines (false → NULL → OCCT) if the lune split is not a clean 2+2, the
// wall stitch fails, or a cap plane clips the seam band (a short cylinder whose |vCap| ≤ the
// mouth v-extent — the disc cap would cross the seam and the mouth is not interior).
bool appendSteinmetzOuterWall(const CurvedSolid& cs, const std::vector<BranchArcData>& arcs,
                              LuneCyl which, VertexPool& pool, std::vector<topo::Shape>& faces) {
  std::array<std::pair<int, int>, 2> lunes;
  if (!insideLunesOn(arcs, which, lunes)) return false;
  // SHORT-CYLINDER decline: the two disc caps sit at v=vLo/vHi; the inside-lune mouths must
  // be strictly interior (else appendHoledWall's mouthBlockRange rejects the v-edge touch).
  // Guard it up front so the decline is an explicit honest gate, never a faked cap.
  double vLoM = 1e300, vHiM = -1e300;
  for (const BranchArcData& a : arcs)
    for (const auto& uv : (which == LuneCyl::A ? a.uvA : a.uvB)) {
      vLoM = std::min(vLoM, uv.second);
      vHiM = std::max(vHiM, uv.second);
    }
  if (!(vLoM > cs.vLo + kSsiTol) || !(vHiM < cs.vHi - kSsiTol)) return false;  // cap clips seam

  const Seam m0 = luneMouthSeam(arcs[lunes[0].first], arcs[lunes[0].second], which);
  const Seam m1 = luneMouthSeam(arcs[lunes[1].first], arcs[lunes[1].second], which);
  const auto& uv0 = (which == LuneCyl::A) ? m0.uvA : m0.uvB;
  const auto& uv1 = (which == LuneCyl::A) ? m1.uvA : m1.uvB;
  int uSteps, vSteps;
  steinmetzWallSteps(cs, arcs, which, uSteps, vSteps);
  std::vector<math::Point3> rimLo, rimHi;
  if (!appendHoledWall(cs, m0, m1, uv0, uv1, uSteps, vSteps, pool, faces, rimLo, rimHi))
    return false;
  appendDiskCap(cs, cs.vLo, rimLo, math::Vec3{-cs.frame.z.vec().x, -cs.frame.z.vec().y,
                -cs.frame.z.vec().z}, pool, faces);
  appendDiskCap(cs, cs.vHi, rimHi, cs.frame.z.vec(), pool, faces);
  return true;
}

// buildSteinmetzFuse(A,B) = A ∪ B: both cylinders' OUTSIDE walls (each with its two inside
// lunes cut out) + all four disc caps, welded along the four shared arcs.
topo::Shape buildSteinmetzFuse(const CurvedSolid& A, const CurvedSolid& B,
                               const SteinmetzTrace& st) {
  const std::vector<BranchArcData> arcs = orientResampleArcs(A, st);
  if (arcs.size() != 4) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  if (!appendSteinmetzOuterWall(A, arcs, LuneCyl::A, pool, faces)) return {};
  if (!appendSteinmetzOuterWall(B, arcs, LuneCyl::B, pool, faces)) return {};
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildSteinmetzCut(A,B) = A − B: A's OUTSIDE wall + A's caps + B's two INSIDE-A lunes
// emitted REVERSED (inward normal — the wall of the channel B carves through A). B's inside
// lunes are kept iff their centroid classifies INSIDE A (the same survival sample COMMON
// uses); an ON verdict → NULL → OCCT (never faked). A is the order-honored minuend.
topo::Shape buildSteinmetzCut(const CurvedSolid& A, const CurvedSolid& B,
                              const SteinmetzTrace& st) {
  const std::vector<BranchArcData> arcs = orientResampleArcs(A, st);
  if (arcs.size() != 4) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // A's OUTSIDE wall + A's two disc caps.
  if (!appendSteinmetzOuterWall(A, arcs, LuneCyl::A, pool, faces)) return {};
  // B's two INSIDE-A lunes, REVERSED (outwardSign = −1) → the carved-channel wall.
  std::array<std::pair<int, int>, 2> lunesB;
  if (!insideLunesOn(arcs, LuneCyl::B, lunesB)) return {};
  for (const auto& [iLo, iHi] : lunesB) {
    const BranchArcData& lo = arcs[iLo];
    const BranchArcData& hi = arcs[iHi];
    const int mid = static_cast<int>(lo.pts.size()) / 2;
    const auto uvLo = lo.uvB[mid];
    const auto uvHi = hi.uvB[mid];
    const double uMid = 0.5 * (uvLo.first + nearU(uvLo.first, uvHi.first));
    const double vMid = 0.5 * (uvLo.second + uvHi.second);
    const math::Point3 centroid = B.point(uMid, vMid);
    const int cls = classifyPoint(A, centroid, kSsiTol);
    if (cls == 0) return {};   // ON A's wall → tangent → OCCT
    if (cls != 1) continue;    // OUTSIDE A → not an inside-A lune (should not happen)
    const int uSamp = luneUSamples(B, lo, hi, LuneCyl::B);
    appendLunePatch(B, LuneCyl::B, lo, hi, uSamp, /*outwardSign=*/-1.0, pool, faces);
  }
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// Cheap pre-gate for the Steinmetz family, checked ONLY on the decline edge (before the
// expensive branch-enabled re-trace): both operands cylinders, near-equal radius, axis
// directions orthogonal, axis lines crossing (min-distance ≈ 0). Anything else → no
// re-trace (the branched path never fires for a non-Steinmetz pair).
bool steinmetzPreGate(const CurvedSolid& A, const CurvedSolid& B) {
  if (A.kind != CurvedKind::Cylinder || B.kind != CurvedKind::Cylinder) return false;
  const double rMax = std::max(A.radius, B.radius);
  if (std::fabs(A.radius - B.radius) > kSsiTol * std::max(rMax, 1.0)) return false;
  const math::Vec3 za = A.frame.z.vec(), zb = B.frame.z.vec();
  if (std::fabs(math::dot(za, zb)) > 1e-4) return false;  // axes not orthogonal
  // Axis-line crossing distance: |(oB−oA)·(za×zb)| / |za×zb| ≈ 0 (skew lines meet).
  const math::Vec3 n = math::cross(za, zb);
  const double nn = math::norm(n);
  if (nn < 1e-9) return false;  // parallel axes → not a crossing pair
  const math::Vec3 d = B.frame.origin - A.frame.origin;
  const double cross = std::fabs(math::dot(d, n)) / nn;
  return cross <= kSsiTol * std::max(rMax, 1.0);
}

// S5-d branched dispatch: on the decline edge (default trace near-tangent), if the pair is
// a Steinmetz-family equal-R orthogonal cylinder pair, RE-TRACE with branch points enabled
// and route the four arcs. COMMON is the guaranteed slice; FUSE/CUT are deferred (NULL →
// OCCT). Returns NULL for any non-Steinmetz branched pair or unresolved branched trace.
topo::Shape tryBranchedSteinmetz(const CurvedSolid& A, const CurvedSolid& B,
                                 const ssi::SurfaceAdapter& adA, const ssi::SurfaceAdapter& adB,
                                 Op op) {
  if (!steinmetzPreGate(A, B)) return {};
  ssi::MarchOptions bopts;
  bopts.enableBranchPoints = true;
  const ssi::TraceSet bt = ssi::trace_intersection(adA, adB, {}, bopts);
  const std::optional<SteinmetzTrace> st = recogniseSteinmetzTrace(bt);
  if (!st) return {};
  switch (op) {
    case Op::Common: return buildSteinmetzCommon(A, B, *st);
    case Op::Fuse:   return buildSteinmetzFuse(A, B, *st);
    case Op::Cut:    return buildSteinmetzCut(A, B, *st);
  }
  return {};
}

}  // namespace

// ── The S5-a driver (design.md §Pipeline). Short, linear; the geometry lives in the
// detail helpers + buildCommon. Returns a candidate Solid or NULL; the ENGINE
// self-verifies (watertight + set-algebra volume) and owns the OCCT fallback. ──────
topo::Shape ssi_boolean_solid(const topo::Shape& a, const topo::Shape& b, Op op) {
  // 0. GATE: both operands must be recognised elementary curved solids.
  const std::optional<CurvedSolid> csA = recogniseCurvedSolid(a);
  const std::optional<CurvedSolid> csB = recogniseCurvedSolid(b);
  if (!csA || !csB) return {};  // a box / freeform / torus → other paths / OCCT

  // 0. TRACE: the S3 TraceSet for the two curved walls.
  const ssi::SurfaceAdapter adA = csA->adapter();
  const ssi::SurfaceAdapter adB = csB->adapter();
  const ssi::TraceSet trace = ssi::trace_intersection(adA, adB);

  // Gate on full transversality — the honest S4 boundary. The DEFAULT (unbranched)
  // trace never routes branch arms, so a self-crossing pair (Steinmetz) shows up here as
  // nearTangentGaps > 0 (the sine→0 dip at the branch points). That is exactly the
  // DECLINE edge on which the S5-d branched assembler engages: on this edge ONLY, and
  // ONLY when the cheap steinmetzPreGate matches, RE-TRACE with branch points enabled and
  // route the four arcs to the branched builders. Every single-seam S5-a/b/c pass keeps
  // its DEFAULT trace unchanged (the pre-gate requires equal-R orthogonal cylinders,
  // which those pairs are not) — no re-trace, no regression.
  if (trace.nearTangentGaps > 0)                    // a branch traced up to a tangent → S4
    return tryBranchedSteinmetz(*csA, *csB, adA, adB, op);  // Steinmetz S5-d, else NULL → OCCT
  if (trace.branchPoints > 0) return {};            // S4-d self-crossing (multi-arm) → out of scope for S5 single-seam booleans → OCCT
  if (trace.lines.empty()) return {};               // no seam → OCCT
  for (const ssi::WLine& w : trace.lines)
    if (w.status == ssi::TraceStatus::NearTangent || w.status == ssi::TraceStatus::Failed ||
        w.status == ssi::TraceStatus::BranchArc)
      return {};                                    // out of scope → OCCT

  std::vector<Seam> seams;
  seams.reserve(trace.lines.size());
  for (const ssi::WLine& w : trace.lines) seams.push_back(toSeam(w));

  // 1-4. SPLIT + CLASSIFY + SELECT + WELD, dispatched on seam-count + operand-kinds + op:
  //   * THROUGH-DRILL cyl∩cyl (two rim seams, one operand full-circle on both, the other
  //     local on both) — ALL THREE ops: COMMON (S5-a) buildCommon; FUSE / CUT (S5-b) select
  //     the OUTSIDE fat wall (mouths cut out) + faceted caps + the reversed tunnel band (cut)
  //     or the protruding end tubes (fuse).
  //   * sphere∩sphere COMMON (S5-c) — one closed seam, both operands Sphere: buildLensCommon
  //     welds the two inside-the-other spherical caps along the single seam.
  // Every seam-adjacent fragment is a shared-pool planar facet so the shell welds watertight
  // (assembler-side, tessellator untouched). Anything else (sphere fuse/cut, tangent/coincident,
  // oblique/multi-tube cyl∩cyl, other curved-curved families) still returns NULL → OCCT.
  switch (op) {
    case Op::Common: {
      // Through-drill (two rim seams) → buildCommon; single-seam sphere∩sphere lens
      // (S5-c) → buildLensCommon. buildCommon is untouched (returns {} for one seam).
      const topo::Shape drill = buildCommon(*csA, *csB, seams);
      if (!drill.isNull()) return drill;
      const topo::Shape lens = buildLensCommon(*csA, *csB, seams);
      if (!lens.isNull()) return lens;
      // S5-e: coaxial cone(frustum)∩cylinder COMMON (single analytic circle seam).
      const topo::Shape coneCyl = buildConeCylCommon(*csA, *csB, seams);
      if (!coneCyl.isNull()) return coneCyl;
      // S5-f: coaxial cone(frustum)∩sphere COMMON (cone band + sphere inner cap).
      const topo::Shape coneSph = buildConeSphereCommon(*csA, *csB, seams);
      if (!coneSph.isNull()) return coneSph;
      // S5-h: TWO-CIRCLE coaxial cone∩sphere COMMON (sphere lower cap + cone band + sphere upper cap).
      const topo::Shape coneSph2 = buildConeSphere2Common(*csA, *csB, seams);
      if (!coneSph2.isNull()) return coneSph2;
      // S5-i: TWO-CIRCLE coaxial cylinder∩sphere COMMON (sphere lower cap + cyl band + sphere upper cap).
      const topo::Shape cylSph2 = buildCylSphere2Common(*csA, *csB, seams);
      if (!cylSph2.isNull()) return cylSph2;
      // S5-j: HOURGLASS (apex-to-apex) coaxial cone∩cone COMMON (bicone; apex-terminated min profile).
      const topo::Shape hgCommon = buildHourglassConeConeCommon(*csA, *csB, seams);
      if (!hgCommon.isNull()) return hgCommon;
      // S5-g: coaxial cone(frustum)∩cone(frustum) COMMON (min-radius profile of revolution).
      return buildConeConeCommon(*csA, *csB, seams);
    }
    case Op::Fuse: {
      // Through-drill (two rim seams) → buildFuse; single-seam sphere∩sphere lens (S5-c
      // fuse) → buildLensFuse (two OUTER caps). buildFuse declines the single seam.
      const topo::Shape drill = buildFuse(*csA, *csB, seams);
      if (!drill.isNull()) return drill;
      const topo::Shape lens = buildLensFuse(*csA, *csB, seams);
      if (!lens.isNull()) return lens;
      // S5-e: coaxial cone(frustum)∩cylinder FUSE (outer wall regions + caps).
      const topo::Shape coneCyl = buildConeCylFuse(*csA, *csB, seams);
      if (!coneCyl.isNull()) return coneCyl;
      // S5-f: coaxial cone(frustum)∩sphere FUSE (sphere outer cap + cone outer wall + disc).
      const topo::Shape coneSph = buildConeSphereFuse(*csA, *csB, seams);
      if (!coneSph.isNull()) return coneSph;
      // S5-h: TWO-CIRCLE coaxial cone∩sphere FUSE (cone walls + sphere zone bulge + discs).
      const topo::Shape coneSph2 = buildConeSphere2Fuse(*csA, *csB, seams);
      if (!coneSph2.isNull()) return coneSph2;
      // S5-i: TWO-CIRCLE coaxial cylinder∩sphere FUSE (cyl walls + sphere zone bulge + discs).
      const topo::Shape cylSph2 = buildCylSphere2Fuse(*csA, *csB, seams);
      if (!cylSph2.isNull()) return cylSph2;
      // S5-g: coaxial cone(frustum)∩cone(frustum) FUSE (max-radius profile of revolution).
      return buildConeConeFuse(*csA, *csB, seams);
    }
    case Op::Cut: {
      // Through-drill → buildCut; single-seam sphere∩sphere lens (S5-c cut) → buildLensCut
      // (outer-cap-of-A + reversed inner-cap-of-B). buildCut declines the single seam.
      const topo::Shape drill = buildCut(*csA, *csB, seams);
      if (!drill.isNull()) return drill;
      const topo::Shape lens = buildLensCut(*csA, *csB, seams);
      if (!lens.isNull()) return lens;
      // S5-e: coaxial cone(frustum)∩cylinder CUT (A outer wall + A caps + reversed B band).
      const topo::Shape coneCyl = buildConeCylCut(*csA, *csB, seams);
      if (!coneCyl.isNull()) return coneCyl;
      // S5-f: coaxial cone(frustum)∩sphere CUT (cone outer wall + disc + reversed sphere dimple).
      const topo::Shape coneSph = buildConeSphereCut(*csA, *csB, seams);
      if (!coneSph.isNull()) return coneSph;
      // S5-h: TWO-CIRCLE coaxial cone∩sphere CUT (two disconnected cone-tip/end dimpled pieces).
      const topo::Shape coneSph2 = buildConeSphere2Cut(*csA, *csB, seams);
      if (!coneSph2.isNull()) return coneSph2;
      // S5-i: TWO-CIRCLE coaxial cylinder∩sphere CUT (two disconnected cyl-end dimpled pieces).
      const topo::Shape cylSph2 = buildCylSphere2Cut(*csA, *csB, seams);
      if (!cylSph2.isNull()) return cylSph2;
      // S5-j: HOURGLASS (apex-to-apex) coaxial cone∩cone CUT (conical shell to a full A-end disc).
      const topo::Shape hgCut = buildHourglassConeConeCut(*csA, *csB, seams);
      if (!hgCut.isNull()) return hgCut;
      // S5-g: coaxial cone(frustum)∩cone(frustum) CUT (A washer + reversed B wall + A-only slice).
      return buildConeConeCut(*csA, *csB, seams);
    }
  }
  return {};
}

}  // namespace cybercad::native::boolean

#else  // !CYBERCAD_HAS_NUMSCI — substrate absent: stub that always declines.

namespace cybercad::native::boolean {
topo::Shape ssi_boolean_solid(const topo::Shape&, const topo::Shape&, Op) { return {}; }
}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_HAS_NUMSCI
