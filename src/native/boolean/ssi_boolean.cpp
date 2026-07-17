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
  s.minorRadius = cs.minorRadius;
  switch (cs.kind) {
    case CurvedKind::Cylinder: s.kind = topo::FaceSurface::Kind::Cylinder; break;
    case CurvedKind::Sphere:   s.kind = topo::FaceSurface::Kind::Sphere; break;
    case CurvedKind::Cone:     s.kind = topo::FaceSurface::Kind::Cone; break;
    case CurvedKind::Torus:    s.kind = topo::FaceSurface::Kind::Torus; break;
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

// A flat disc cap fanning from an EXPLICIT axis point to a rim ring, planar facets, normal `capN`.
// (appendDiskCap keys the axis point off a CurvedSolid frame + v, which is wrong for a cone slant-v
// or a torus z-frame station; this variant takes the axis point directly.)
void appendAxisDiscCap(const math::Point3& axisPt, const std::vector<math::Point3>& rim,
                       const math::Vec3& capN, VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int n = static_cast<int>(rim.size());
  if (n < 3) return;
  for (int k = 0; k < n; ++k)
    pushPlanarTri(axisPt, rim[k], rim[(k + 1) % n], capN, pool, faces);
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
                      std::vector<topo::Shape>& faces, double outwardSign = 1.0) {
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
      const math::Vec3 ref{(ctr.x - sphCentre.x) * outwardSign, (ctr.y - sphCentre.y) * outwardSign,
                           (ctr.z - sphCentre.z) * outwardSign};  // ±radial (outward / reversed)
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
// non-coaxial (transversal) pair all decline → OCCT. The sphere-minuend CUT (sphere−cyl) LANDS via
// buildSphereCyl2Cut (the tunnel solid: sphere belt + reversed bore) — order-sensitive but tractable.
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

// buildSphereCyl2Cut(A,B) = A − B with the SPHERE the minuend (the ORDER-SENSITIVE reverse of
// buildCylSphere2Cut): the sphere with a coaxial cylindrical TUNNEL drilled through it — a single
// annular solid of revolution (topologically a tube), NOT the cylinder-minuend's two-piece dimpled
// stack. Boundary: the sphere OUTER ZONE between the two analytic seam latitudes (seamLo→seamHi, the
// equatorial belt at ρ ≥ Rc, outward +1) + the cylinder BORE band between the same two seams,
// REVERSED (inward normal, bounding the drilled tunnel). The two seam rings (ρ = Rc, station sLo/sHi)
// are shared so the belt welds to the bore. Gated on the SAME clean two-circle poke-through the S5-i
// COMMON needs (the cylinder pierces both sphere poles → the two seams bound the belt + tunnel). The
// sphere polar caps (ρ < Rc, near the poles) are entirely inside the drill → removed. V = V_sph −
// V_common. The engine's watertight + two-sided volume self-verify is the safety net; a pose that
// cannot weld robustly HONEST-DECLINES → OCCT.
topo::Shape buildSphereCyl2Cut(const CurvedSolid& A, const CurvedSolid& B,
                               const std::vector<Seam>& seams) {
  const CylSphere2Setup s = cylSphere2Setup(A, B, seams);
  if (!s.ok) return {};
  if (&A != s.sph) return {};  // A must be the sphere minuend; cyl−sphere is buildCylSphere2Cut
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ringLo = s.ring(s.Rc, s.sLo);
  const std::vector<math::Point3> ringHi = s.ring(s.Rc, s.sHi);
  // Sphere OUTER equatorial zone between the two seams (the belt the FUSE uses), outward +1.
  appendSphereZone(s.C, s.Rs, ringLo, ringHi, pool, faces, /*outwardSign=*/1.0);
  // Cylinder BORE band between the two seams, REVERSED (inward) — the drilled tunnel wall.
  appendRevolvedBand(ringLo, ringHi, s.O, s.zc, pool, faces, /*outwardSign=*/-1.0);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── S5-k — TRANSVERSAL (NON-COAXIAL) CYLINDER∩SPHERE COMMON / CUT / FUSE ─────────────
// The FIRST transversal (non-coaxial) curved-boolean slice. Where S5-i handles the COAXIAL
// finite-cylinder∩sphere pose (sphere centre ON the cylinder axis → two ANALYTIC circle
// seams, planar rings), S5-k handles the OFFSET pose: the cylinder axis is PARALLEL to but
// DISPLACED from the sphere centre (perpendicular offset), so the two seams are NON-PLANAR
// closed space curves (generalised Viviani curves) — no analytic circle exists. The seam is
// consumed DIRECTLY from the S3 TraceSet (the general SSI machinery the roadmap names), not
// re-derived from a closed form.
//
// SCOPE (the clean, robustly-handleable transversal pose): a THIN cylinder (radius Rc <
// sphere radius Rs) whose axis is offset from the sphere centre but still passes fully
// THROUGH the sphere — both cylinder ends poke out beyond the sphere, so the cylinder wall
// crosses the sphere at exactly TWO disjoint closed loops (a lower loop + an upper loop
// along the cylinder axis), both fully transversal (nearTangentGaps==0, branchPoints==0,
// both Closed). The COMMON is then the same TOPOLOGY as the coaxial S5-i COMMON — a cylinder
// mid-band capped by two spherical caps — but every ring is the traced NON-PLANAR seam:
//   COMMON = sphere lower cap (inside the cylinder) + cylinder band (seamLo→seamHi, inside
//     the sphere) + sphere upper cap (inside the cylinder). r ≤ min profile, generalised to
//     the offset pose. Its boundary rings are the two traced seams (shared VertexPool weld).
//     This is the LANDED transversal slice — every fragment is seam-driven and verified.
//
// CUT / FUSE both additionally need the sphere OUTER SHELL (the sphere ZONE between the two
// NON-PLANAR seams, the long way round outside the bore); welding that zone as a shared-pool
// planar-facet shell is the UNRESOLVED transversal residual (no revolved-band / far-pole
// meridian tiles a two-non-planar-seam zone watertight), so CUT/FUSE HONEST-DECLINE → OCCT.
//
// REDUCTION: as the offset → 0 the pose becomes coaxial and S5-i's `cylSphere2Setup` claims
// it FIRST in the dispatch (it runs before S5-k), reproducing the landed coaxial result
// BYTE-for-byte; S5-k therefore gates on a STRICTLY-POSITIVE offset (else → decline, letting
// S5-i own it). Anything else — a cylinder that does not pierce both poles (a single-loop /
// tangent / grazing pose → NearTangent → OCCT), a non-parallel (skew) axis, a fat cylinder
// (Rc ≥ Rs) — declines → OCCT. The engine owns the watertight + volume gate + OCCT fallback.
// Nothing here is faked: any pose that cannot weld robustly returns NULL.
struct TransCylSphereSetup {
  bool ok = false;
  const CurvedSolid* cyl = nullptr;
  const CurvedSolid* sph = nullptr;
  math::Point3 O;            ///< cylinder origin
  math::Vec3 zc;             ///< cylinder axis (unit)
  math::Point3 C;            ///< sphere centre
  double Rc = 0.0, Rs = 0.0;
  double offset = 0.0;       ///< perpendicular distance of the sphere centre from the cyl axis
  int N = 0;                 ///< common azimuth sample count (seam-chord bounded)
  Seam seamLo{}, seamHi{};   ///< the two traced seams, resampled + ordered lo/hi along zc
  double cylLo = 0.0, cylHi = 0.0;  ///< cylinder axial extent (v param) ordered along +zc
  // The two-cap+outer-zone topology (COMMON caps = the sphere's two polar caps INSIDE the
  // cylinder; the CUT/FUSE outer zone = the rest of the sphere OUTSIDE the cylinder, the long
  // way round) is only valid when BOTH sphere poles (along ±zc from the centre) sit strictly
  // INSIDE the cylinder. When a pole falls outside (a very thin cylinder grazing the pole
  // region) the sphere-outside-cylinder set is no longer two-cap-complementary and the outer
  // zone cannot be tiled by a single monotone φ-sweep → decline (kept honest). Gated in setup.
  bool polesInsideCyl = false;
  // A point on the cylinder axis far beyond the lower / upper seam (selects each cap's apex).
  math::Point3 axisFarM() const {
    return math::Point3{O.x - zc.x * 1e4, O.y - zc.y * 1e4, O.z - zc.z * 1e4};
  }
  math::Point3 axisFarP() const {
    return math::Point3{O.x + zc.x * 1e4, O.y + zc.y * 1e4, O.z + zc.z * 1e4};
  }
  // Sphere pole along −zc / +zc (the two cap apices; the caps' furthest-from-equator points).
  math::Point3 poleM() const {
    return math::Point3{C.x - zc.x * Rs, C.y - zc.y * Rs, C.z - zc.z * Rs};
  }
  math::Point3 poleP() const {
    return math::Point3{C.x + zc.x * Rs, C.y + zc.y * Rs, C.z + zc.z * Rs};
  }
  // A cylinder wall ring at axial station v, sampled at the SAME N azimuths as seamLo/seamHi
  // (index i ↔ seam index i), so an end disc / wall band welds ring-to-ring through the pool.
  // The seam azimuth is read from the cylinder (u,v) track (uvB if the cylinder is operand B).
  std::vector<math::Point3> cylRing(double v, bool cylIsB) const {
    std::vector<math::Point3> r(seamLo.pts.size());
    for (size_t i = 0; i < seamLo.pts.size(); ++i) {
      const double u = cylIsB ? seamLo.uvB[i].first : seamLo.uvA[i].first;
      r[i] = cyl->point(u, v);
    }
    return r;
  }
};

// Resample a traced closed seam onto a COMMON cylinder-azimuth grid of N nodes: for each
// target azimuth u = 2πk/N, pick the seam node whose cylinder u-param (uvB, folded to
// [0,2π)) is nearest u. This aligns BOTH seams index-wise (same azimuth per index) so the
// cylinder band welds ring-to-ring cleanly, and keeps the seam's EXACT 3D points (so a cap
// built on the shared pool welds along the true traced curve). `cylIsB` picks which (u,v)
// track is the cylinder's.
Seam resampleByAzimuth(const Seam& seam, bool cylIsB, int N) {
  Seam out;
  out.closed = true;
  out.pts.resize(N);
  out.uvA.resize(N);
  out.uvB.resize(N);
  const int m = static_cast<int>(seam.pts.size());
  for (int k = 0; k < N; ++k) {
    const double target = kSsiTwoPi * k / N;
    double best = 1e300;
    int bi = 0;
    for (int i = 0; i < m; ++i) {
      double u = cylIsB ? seam.uvB[i].first : seam.uvA[i].first;
      u = std::fmod(u, kSsiTwoPi);
      if (u < 0.0) u += kSsiTwoPi;
      double du = std::fabs(u - target);
      du = std::min(du, kSsiTwoPi - du);
      if (du < best) { best = du; bi = i; }
    }
    out.pts[k] = seam.pts[bi];
    out.uvA[k] = seam.uvA[bi];
    out.uvB[k] = seam.uvB[bi];
  }
  return out;
}

// Mean axial projection (onto the cylinder axis from the cylinder origin) of a seam's nodes.
double seamAxialMean(const TransCylSphereSetup& s, const Seam& seam) {
  double acc = 0.0;
  for (const auto& p : seam.pts)
    acc += math::dot(math::Vec3{p.x - s.O.x, p.y - s.O.y, p.z - s.O.z}, s.zc);
  return acc / static_cast<double>(std::max<size_t>(1, seam.pts.size()));
}

// Recognise the transversal (offset) cylinder∩sphere pierce-both-poles pose from the two
// traced seams. Declines (→ ok=false) for the coaxial pose (offset ≤ tol, owned by S5-i), a
// skew/non-parallel axis, a fat cylinder, ≠2 closed seams, or a pose whose caps/band fail
// the inside-the-other survival sample. Mirrors the other setup prologues (coneCylSetup /
// cylSphere2Setup): systems-band (a linear pose-recognition + resample + survival gate);
// isolated + documented per the complexity policy.
TransCylSphereSetup transCylSphereSetup(const CurvedSolid& A, const CurvedSolid& B,
                                        const std::vector<Seam>& seams) {
  TransCylSphereSetup st;
  if (seams.size() != 2) return st;                       // pierce-both-poles → exactly two loops
  for (const Seam& s : seams)
    if (!s.closed || s.pts.size() < 8) return st;

  const CurvedSolid* cylPtr = nullptr;
  const CurvedSolid* sphPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Cylinder) cylPtr = s;
    else if (s->kind == CurvedKind::Sphere) sphPtr = s;
  }
  if (!cylPtr || !sphPtr) return st;
  const CurvedSolid& cyl = *cylPtr;
  const CurvedSolid& sph = *sphPtr;
  const bool cylIsB = (&B == cylPtr);

  const math::Vec3 zc = cyl.frame.z.vec();
  const math::Point3 O = cyl.frame.origin;
  const math::Point3 C = sph.frame.origin;
  const math::Vec3 d{C.x - O.x, C.y - O.y, C.z - O.z};
  const double sc = math::dot(d, zc);
  const double offset = math::norm(d - zc * sc);          // perpendicular sphere-centre offset
  if (offset <= 1e-4) return st;                           // coaxial → S5-i owns it → decline
  const double Rc = cyl.radius, Rs = sph.radius;
  if (!(Rc > 1e-9) || !(Rs > 1e-9)) return st;
  if (!(Rs > Rc + 1e-6)) return st;                        // fat cylinder → not this pierce pose
  // The whole cylinder cross-section must fit inside the sphere at the sphere's equator so
  // the cylinder pierces both poles: the farthest cyl-wall point from C in the equatorial
  // plane is offset+Rc; require offset+Rc < Rs (strictly) so both seam loops are proper.
  if (!(offset + Rc < Rs - 1e-6)) return st;

  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * Rc, 1e-12));
  const int N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * Rc / chord)), 24, 180);

  Seam s0 = resampleByAzimuth(seams[0], cylIsB, N);
  Seam s1 = resampleByAzimuth(seams[1], cylIsB, N);

  st.cyl = cylPtr; st.sph = sphPtr; st.O = O; st.zc = zc; st.C = C;
  st.Rc = Rc; st.Rs = Rs; st.offset = offset; st.N = N;
  // Cylinder axial extent, ordered so cylLo ≤ cylHi along +zc (vLo/vHi already are, but the
  // recognised axis may be flipped relative to zc; keep the ordered pair for the end discs).
  st.cylLo = std::min(cyl.vLo, cyl.vHi);
  st.cylHi = std::max(cyl.vLo, cyl.vHi);
  const double m0 = seamAxialMean(st, s0), m1 = seamAxialMean(st, s1);
  if (std::fabs(m0 - m1) < 1e-4) return st;                // both loops at one station → not two poles
  st.seamLo = (m0 <= m1) ? s0 : s1;
  st.seamHi = (m0 <= m1) ? s1 : s0;

  // SURVIVAL samples (the honest inside-the-other gate, never faked):
  //  * the cylinder band midpoint (a wall point half-way between the two seams) must be
  //    INSIDE the sphere (the band is a COMMON boundary);
  //  * each cap apex direction (the sphere point nearest the axis-far point) must be INSIDE
  //    the cylinder (the caps close the COMMON).
  const double axLo = seamAxialMean(st, st.seamLo), axHi = seamAxialMean(st, st.seamHi);
  const math::Point3 bandMid = cyl.point(0.0, 0.5 * (axLo + axHi));
  if (classifyPoint(sph, bandMid, kSsiTol) != 1) return st;
  auto capApex = [&](const math::Point3& far) {
    const math::Vec3 t{far.x - C.x, far.y - C.y, far.z - C.z};
    const double L = std::max(math::norm(t), 1e-12);
    return math::Point3{C.x + t.x / L * Rs, C.y + t.y / L * Rs, C.z + t.z / L * Rs};
  };
  if (classifyPoint(cyl, capApex(st.axisFarM()), kSsiTol) != 1) return st;
  if (classifyPoint(cyl, capApex(st.axisFarP()), kSsiTol) != 1) return st;

  // OUTER-ZONE gate (used by CUT/FUSE, not COMMON): both sphere poles (±zc from the centre)
  // must be strictly INSIDE the cylinder. Then the sphere splits cleanly into the two polar
  // caps (inside the cyl → the COMMON caps) and one connected outer zone (outside the cyl →
  // the CUT/FUSE band, the long way round), tileable by a monotone equator-crossing φ-sweep.
  st.polesInsideCyl = classifyPoint(cyl, st.poleM(), kSsiTol) == 1 &&
                      classifyPoint(cyl, st.poleP(), kSsiTol) == 1;

  st.ok = true;
  return st;
}

// Ring count for a transversal sphere cap: the cap's polar half-angle (apex→farthest seam
// node, measured from the sphere centre) × sqrt(Rs/(2·kCapSagitta)), bounded [4,48] — the
// S5-i cap-refinement discipline, generalised to a non-planar seam.
int transCapRings(const TransCylSphereSetup& s, const Seam& seam, const math::Point3& apex) {
  const math::Vec3 aDir{apex.x - s.C.x, apex.y - s.C.y, apex.z - s.C.z};
  double theta = 0.0;
  for (const auto& p : seam.pts) {
    const math::Vec3 sDir{p.x - s.C.x, p.y - s.C.y, p.z - s.C.z};
    const double den = std::max(math::norm(aDir) * math::norm(sDir), 1e-12);
    theta = std::max(theta, std::acos(std::clamp(math::dot(aDir, sDir) / den, -1.0, 1.0)));
  }
  return std::clamp(
      static_cast<int>(std::ceil(std::max(theta, 1e-6) * std::sqrt(s.Rs / (2.0 * kCapSagitta)))), 4,
      48);
}

// ── SPHERE OUTER ZONE between two NON-PLANAR seams (the transversal seam-band primitive) ──
// The band of the sphere surface that lies OUTSIDE the cylinder — the sphere ZONE between the
// two traced seams taken the LONG way round (away from the bore). This is the primitive the
// transversal CUT / FUSE need and that the coaxial `appendSphereZone` cannot supply: the two
// seams are NON-PLANAR (generalised Viviani) curves that do NOT lie on a common latitude, and
// the two seam nodes at a shared cylinder azimuth are (near-)antipodal on the sphere, so a raw
// 3-D great-circle slerp between them is ambiguous and picks the SHORT arc — which cuts THROUGH
// the bore (through the two polar caps that belong to the COMMON), not around the outside.
//
// The fix uses the sphere's OWN spherical coordinates about the CYLINDER AXIS zc (as the polar
// axis, at the sphere centre): every seam node has a polar angle φ (from zc) and an azimuth θ
// (around zc). Because both seams pass through the same cylinder-wall azimuth per index, the
// two nodes at index i share θ; seamHi sits near the +zc pole (small φ) and seamLo near the −zc
// pole (large φ). The outer zone is then swept at CONSTANT θ by interpolating φ LINEARLY from
// φ_hi(i) UP to φ_lo(i) — i.e. crossing the EQUATOR (the far side, away from the bore) — never
// approaching either pole. Every interior row point is placed EXACTLY on the sphere (centre +
// unit(dir)·Rs), so it is on-surface to machine precision; the outer rows ARE the pooled seam
// nodes, so the zone welds ring-to-ring to whatever bounds the seams (the reversed cylinder
// tunnel for CUT sphere−cyl, the cylinder end stubs for FUSE). Outward radial normal by
// default; `outwardSign=-1` for a reversed (inward) orientation. Row count from the zone's
// polar span + kCapSagitta (the appendSphereZone discipline). Pre: both poles inside the cyl
// (the setup's polesInsideCyl gate) so the sweep is a clean monotone equator crossing.
//
// systems-band (~16 — θ/φ decode + monotone φ-sweep + quad emit); flagged per the policy.
void appendSphereOuterZoneBetweenSeams(const math::Point3& C, double Rs, const math::Vec3& zc,
                                       const Seam& seamHi, const Seam& seamLo, VertexPool& pool,
                                       std::vector<topo::Shape>& faces, double outwardSign = 1.0) {
  const int n = static_cast<int>(seamHi.pts.size());
  if (n < 3 || static_cast<int>(seamLo.pts.size()) != n) return;
  // Orthonormal (ex,ey) spanning the plane ⟂ zc, at C, for the azimuth θ about zc.
  const math::Vec3 seed =
      std::fabs(zc.x) < 0.9 ? math::Vec3{1, 0, 0} : math::Vec3{0, 1, 0};
  math::Vec3 ex = seed - zc * math::dot(seed, zc);
  ex = ex * (1.0 / std::max(math::norm(ex), 1e-12));
  const math::Vec3 ey = math::cross(zc, ex);
  auto decode = [&](const math::Point3& p) {
    math::Vec3 d{p.x - C.x, p.y - C.y, p.z - C.z};
    d = d * (1.0 / std::max(math::norm(d), 1e-12));
    const double phi = std::acos(std::clamp(math::dot(d, zc), -1.0, 1.0));  // from +zc pole
    const double th = std::atan2(math::dot(d, ey), math::dot(d, ex));
    return std::pair<double, double>{th, phi};
  };
  auto dirOf = [&](double th, double phi) {
    const double sp = std::sin(phi), cp = std::cos(phi);
    return math::Vec3{sp * (std::cos(th) * ex.x + std::sin(th) * ey.x) + cp * zc.x,
                      sp * (std::cos(th) * ex.y + std::sin(th) * ey.y) + cp * zc.y,
                      sp * (std::cos(th) * ex.z + std::sin(th) * ey.z) + cp * zc.z};
  };
  // Per-index (θ,φ) of the two seams; φ_hi (small) → φ_lo (large) is the equator-crossing span.
  std::vector<double> thHi(n), phHi(n), thLo(n), phLo(n);
  double spanMax = 0.0;
  for (int i = 0; i < n; ++i) {
    auto h = decode(seamHi.pts[i]);
    auto l = decode(seamLo.pts[i]);
    thHi[i] = h.first; phHi[i] = h.second;
    thLo[i] = l.first; phLo[i] = l.second;
    // Unwrap θ_lo onto θ_hi's branch (shared azimuth per index; guard the ±π seam).
    double dth = thLo[i] - thHi[i];
    if (dth > kSsiPi) thLo[i] -= kSsiTwoPi;
    else if (dth < -kSsiPi) thLo[i] += kSsiTwoPi;
    spanMax = std::max(spanMax, std::fabs(phLo[i] - phHi[i]));
  }
  const int rows = std::clamp(
      static_cast<int>(std::ceil(std::max(spanMax, 1e-6) * std::sqrt(Rs / (2.0 * kCapSagitta)))), 2,
      64);
  auto rowPt = [&](int r, int i) -> math::Point3 {
    if (r == 0) return seamHi.pts[i];       // top ring = pooled upper-seam node
    if (r == rows) return seamLo.pts[i];    // bottom ring = pooled lower-seam node
    const double t = static_cast<double>(r) / rows;
    const math::Vec3 d = dirOf(thHi[i] + (thLo[i] - thHi[i]) * t, phHi[i] + (phLo[i] - phHi[i]) * t);
    return math::Point3{C.x + d.x * Rs, C.y + d.y * Rs, C.z + d.z * Rs};
  };
  for (int r = 0; r < rows; ++r)
    for (int i = 0; i < n; ++i) {
      const int j = (i + 1) % n;
      const math::Point3 a = rowPt(r, i), b = rowPt(r, j);
      const math::Point3 c = rowPt(r + 1, j), dd = rowPt(r + 1, i);
      const math::Point3 ctr{(a.x + b.x + c.x + dd.x) / 4, (a.y + b.y + c.y + dd.y) / 4,
                             (a.z + b.z + c.z + dd.z) / 4};
      const math::Vec3 ref{(ctr.x - C.x) * outwardSign, (ctr.y - C.y) * outwardSign,
                           (ctr.z - C.z) * outwardSign};  // ±sphere radial
      pushPlanarTri(a, b, c, ref, pool, faces);
      pushPlanarTri(a, c, dd, ref, pool, faces);
    }
}

// buildTransCylSphereCommon(A,B) = COMMON of the TRANSVERSAL (offset) cylinder∩sphere: the
// sphere lower cap (apex toward −zc, inside the cylinder) + the cylinder band between the two
// traced seams (inside the sphere) + the sphere upper cap (apex toward +zc). All three share
// the two traced seam rings through one VertexPool → watertight. The first transversal slice.
topo::Shape buildTransCylSphereCommon(const CurvedSolid& A, const CurvedSolid& B,
                                      const std::vector<Seam>& seams) {
  const TransCylSphereSetup s = transCylSphereSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const math::Point3 apexM = s.axisFarM(), apexP = s.axisFarP();
  appendSphereCap(*s.sph, apexM, s.seamLo, transCapRings(s, s.seamLo, apexM), pool, faces,
                  /*outer=*/false, /*reversed=*/false);
  appendRevolvedBand(s.seamLo.pts, s.seamHi.pts, s.O, s.zc, pool, faces);  // cylinder band
  appendSphereCap(*s.sph, apexP, s.seamHi, transCapRings(s, s.seamHi, apexP), pool, faces,
                  /*outer=*/false, /*reversed=*/false);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTransCylSphereCut / buildTransCylSphereFuse — the transversal CUT / FUSE of the offset
// cylinder∩sphere, LANDED via the seam-band primitive `appendSphereOuterZoneBetweenSeams` (the
// sphere ZONE OUTSIDE the cylinder, the long way round between the two NON-PLANAR seams). This
// resolves the former transversal residual: the earlier claim that no parametrisation tiles the
// two-non-planar-seam zone watertight was wrong — the sphere's OWN spherical coordinates about
// the CYLINDER axis (a constant-θ, equator-crossing φ-sweep) tile it exactly on-surface. Every
// zone row point is placed EXACTLY on the sphere, and the outer rows ARE the pooled seam nodes
// so the zone welds byte-clean to the cylinder tunnel (CUT) or the cylinder end stubs (FUSE).
//
// Both need the two-cap+outer-zone topology, valid only when both sphere poles are INSIDE the
// cylinder (`polesInsideCyl`); otherwise the sphere-outside-cyl set is not two-cap-complementary
// and we HONEST-DECLINE → OCCT. Nothing is faked: any pose that cannot weld robustly returns NULL.
//
// buildTransCylSphereCut(A,B) = A − B:
//   * SPHERE − CYLINDER (A = sphere): the sphere with the cylinder rod bored straight through.
//     Boundary = sphere OUTER ZONE (outward normal) + the cylinder tunnel wall between the two
//     seams (INWARD normal — the reversed bore) — one shell, watertight along both seams. The
//     two sphere polar caps (inside the cyl) are removed. V = V(sph) − V(COMMON).
//   * CYLINDER − SPHERE (A = cylinder): the cylinder rod with the sphere-shaped bite scooped out
//     of its middle → TWO disconnected end pieces (like the coaxial S5-i cut) — a lower cylinder
//     stub (cylLo end disc + wall + sphere LOWER cap REVERSED) + an upper stub (sphere UPPER cap
//     REVERSED + wall + cylHi end disc). The caps are the two COMMON polar caps reversed (the
//     dimples). V = V(cyl) − V(COMMON).
topo::Shape buildTransCylSphereCut(const CurvedSolid& A, const CurvedSolid& B,
                                   const std::vector<Seam>& seams) {
  const TransCylSphereSetup s = transCylSphereSetup(A, B, seams);
  if (!s.ok || !s.polesInsideCyl) return {};
  const bool cylIsB = (&B == s.cyl);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  if (&A == s.sph) {
    // SPHERE − CYLINDER: outer sphere zone + reversed cylinder tunnel band (the bore).
    appendSphereOuterZoneBetweenSeams(s.C, s.Rs, s.zc, s.seamHi, s.seamLo, pool, faces,
                                      /*outwardSign=*/1.0);
    // Cylinder tunnel wall (seamLo→seamHi), INWARD normal (material is OUTSIDE the tunnel).
    appendRevolvedBand(s.seamLo.pts, s.seamHi.pts, s.O, s.zc, pool, faces, /*outwardSign=*/-1.0);
    if (faces.size() < 4) return {};
  } else {
    // CYLINDER − SPHERE: two disconnected cylinder stubs, each dimpled by a reversed COMMON cap.
    const math::Point3 apexM = s.axisFarM(), apexP = s.axisFarP();
    const std::vector<math::Point3> ringLo = s.cylRing(s.cylLo, cylIsB);
    const std::vector<math::Point3> ringHi = s.cylRing(s.cylHi, cylIsB);
    // Lower stub: end disc at cylLo + wall (cylLo→seamLo) + sphere lower cap REVERSED (dimple).
    appendDiskCap(*s.cyl, s.cylLo, ringLo, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
    appendRevolvedBand(ringLo, s.seamLo.pts, s.O, s.zc, pool, faces);
    appendSphereCap(*s.sph, apexM, s.seamLo, transCapRings(s, s.seamLo, apexM), pool, faces,
                    /*outer=*/false, /*reversed=*/true);
    // Upper stub: sphere upper cap REVERSED (dimple) + wall (seamHi→cylHi) + end disc at cylHi.
    appendSphereCap(*s.sph, apexP, s.seamHi, transCapRings(s, s.seamHi, apexP), pool, faces,
                    /*outer=*/false, /*reversed=*/true);
    appendRevolvedBand(s.seamHi.pts, ringHi, s.O, s.zc, pool, faces);
    appendDiskCap(*s.cyl, s.cylHi, ringHi, s.zc, pool, faces);
    if (faces.size() < 8) return {};  // two components → ≥8 faces
  }
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTransCylSphereFuse(A,B) = A ∪ B: the union outer envelope. Boundary = sphere OUTER ZONE
// (the sphere surface outside the cylinder) + the two cylinder end stubs beyond the seams
// (cylLo→seamLo and seamHi→cylHi walls) + the two cylinder end discs. The two sphere polar caps
// (inside the cyl) and the cylinder mid-band (inside the sphere) are interior → dropped. All
// four fragments weld through the pooled seam rings + shared cylinder rings. V = V(A)+V(B)−V(∩).
topo::Shape buildTransCylSphereFuse(const CurvedSolid& A, const CurvedSolid& B,
                                    const std::vector<Seam>& seams) {
  const TransCylSphereSetup s = transCylSphereSetup(A, B, seams);
  if (!s.ok || !s.polesInsideCyl) return {};
  const bool cylIsB = (&B == s.cyl);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ringLo = s.cylRing(s.cylLo, cylIsB);
  const std::vector<math::Point3> ringHi = s.cylRing(s.cylHi, cylIsB);
  // Cylinder lower end disc + lower stub wall (cylLo→seamLo).
  appendDiskCap(*s.cyl, s.cylLo, ringLo, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ringLo, s.seamLo.pts, s.O, s.zc, pool, faces);
  // Sphere outer zone (the bulge, outward normal), the long way round between the two seams.
  appendSphereOuterZoneBetweenSeams(s.C, s.Rs, s.zc, s.seamHi, s.seamLo, pool, faces,
                                    /*outwardSign=*/1.0);
  // Upper stub wall (seamHi→cylHi) + cylinder upper end disc.
  appendRevolvedBand(s.seamHi.pts, ringHi, s.O, s.zc, pool, faces);
  appendDiskCap(*s.cyl, s.cylHi, ringHi, s.zc, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ═══ S5-p — TRANSVERSAL (NON-COAXIAL) TORUS ∩ CYLINDER COMMON (CUT/FUSE decline) ══════
// The SECOND transversal (non-coaxial) curved-boolean slice (after S5-k offset cyl∩sphere),
// and the FIRST transversal TORUS pair. Where S5-l handles the COAXIAL torus∩cylinder pose
// (cylinder axis colinear with the torus axis → two ANALYTIC circle seams, a Pappus solid of
// revolution), S5-p handles the OFFSET pose: the cylinder axis is PARALLEL to the torus axis
// but DISPLACED perpendicular from it, sitting over the tube's rim so a thin cylinder pierces
// the tube like a vertical rod through the ring. Because the axes are non-coaxial the two
// seams are NON-PLANAR closed space curves (the cylinder∩torus quartic locus — no analytic
// circle exists), consumed DIRECTLY from the S3 TraceSet (the general SSI machinery), not a
// closed form. This is the natural torus sibling of the S5-k Viviani slice.
//
// SCOPE (the clean, robustly-handleable transversal pose): a THIN cylinder whose axis is
// parallel to the torus axis, offset perpendicular so the cylinder crosses ONE side of the
// ring, and long enough (axially) that it pokes fully THROUGH the tube — entering the tube's
// lower sheet and exiting its upper sheet — so the cylinder wall crosses the tube at exactly
// TWO disjoint closed loops (a lower loop + an upper loop along the cylinder axis), both fully
// transversal (nearTangentGaps==0, branchPoints==0, both Closed). The COMMON is then the same
// TOPOLOGY as the coaxial S5-l COMMON — a cylinder mid-band capped by two TUBE-surface caps —
// but every ring is the traced NON-PLANAR seam:
//   COMMON = torus lower cap (the tube sheet inside the cylinder) + cylinder band (seamLo→
//     seamHi, inside the tube) + torus upper cap (the tube sheet inside the cylinder). Its
//     boundary rings are the two traced seams (shared VertexPool weld). Every fragment is
//     seam-driven + verified. This is the landed transversal-torus slice.
//
// Each torus cap is a radial fan from the tube-surface CENTRE point of the cap patch (evaluated
// ON the torus at the mean seam (u,v)) out through concentric rings to the exact traced seam
// nodes, each interior ring node placed ON the torus surface by lerping the torus (u,v) from the
// centre to the boundary node (the S5-a appendMouthCap discipline generalised to the torus, and
// oriented by the TRUE tube-outward normal — not axis-radial — so the sheet welds correctly on
// both tube halves). The cap follows the true tube bulge to O(1/rings²).
//
// CUT / FUSE both additionally need the TORUS OUTER SHELL (the tube surface OUTSIDE the bore,
// the long way round between the two non-planar seams) — the same unresolved two-non-planar-seam
// zone weld that made S5-k's CUT/FUSE decline. So CUT/FUSE HONEST-DECLINE → OCCT; COMMON (which
// needs only the two INNER tube caps + the cylinder band) is the landed slice. Nothing is faked.
//
// REDUCTION: as the offset → 0 the pose becomes coaxial and S5-l's `torusCylSetup` claims it
// FIRST in the dispatch (it runs before S5-p), reproducing the landed coaxial result; S5-p
// therefore gates on a STRICTLY-POSITIVE offset (else → decline, letting S5-l own it). A skew
// (non-parallel) cylinder axis, ≠2 closed seams, or a pose failing the inside-the-other survival
// samples all decline → OCCT. The engine owns the watertight + volume gate + OCCT fallback.
struct TransTorusCylSetup {
  bool ok = false;
  const CurvedSolid* tor = nullptr;
  const CurvedSolid* cyl = nullptr;
  bool torIsA = true;         ///< which operand is the torus (picks the seam (u,v) track)
  bool cylIsB = true;         ///< which operand is the cylinder (picks the seam (u,v) track)
  math::Point3 O;             ///< cylinder origin
  math::Vec3 zc;              ///< cylinder axis (unit) — parallel to the torus axis
  double Rc = 0.0;            ///< cylinder radius
  double offset = 0.0;        ///< perpendicular distance of the cyl axis from the torus axis
  int N = 0;                  ///< common azimuth sample count (seam-chord bounded)
  Seam seamLo{}, seamHi{};    ///< the two traced seams, resampled + ordered lo/hi along zc
  double cylLo = 0.0, cylHi = 0.0;  ///< cylinder axial extent (v param) ordered along +zc
  // The two-cap+tube-outer-zone topology (COMMON caps = the tube patch INSIDE the cylinder;
  // the CUT/FUSE outer zone = the rest of the tube surface OUTSIDE the cylinder, swept the
  // long way round the MINOR angle) is only valid when the cylinder pokes fully THROUGH the
  // tube — both cylinder end discs sit clear of the tube (outside), so the two seams bound a
  // clean bore and the complementary tube band is one connected zone. Gated by CUT/FUSE.
  bool cylPierces = false;
  // A cylinder wall ring at axial station v, sampled at the SAME N azimuths as seamLo/seamHi
  // (index i ↔ seam index i) so an end disc / wall band welds ring-to-ring through the pool.
  std::vector<math::Point3> cylRing(double v) const {
    std::vector<math::Point3> r(seamLo.pts.size());
    for (size_t i = 0; i < seamLo.pts.size(); ++i) {
      const double u = cylIsB ? seamLo.uvB[i].first : seamLo.uvA[i].first;
      r[i] = cyl->point(u, v);
    }
    return r;
  }
};

// Mean cylinder-axial projection (onto the cyl axis from the cyl origin) of a seam's nodes.
double transTorusSeamAxialMean(const TransTorusCylSetup& s, const Seam& seam) {
  double acc = 0.0;
  for (const auto& p : seam.pts)
    acc += math::dot(math::Vec3{p.x - s.O.x, p.y - s.O.y, p.z - s.O.z}, s.zc);
  return acc / static_cast<double>(std::max<size_t>(1, seam.pts.size()));
}

// The tube-outward reference at a world point near the torus surface: (p − nearest tube-centre
// point). The tube centre lies on the major circle (radius R about the torus axis) in the plane
// through p's projection onto that axis. Correct on BOTH tube halves (unlike an axis-radial
// reference, which inverts on the inner half) — mirrors S5-l tubeCentre's rationale.
math::Vec3 tubeOutwardAt(const CurvedSolid& tor, const math::Point3& p) {
  const math::Vec3 zc = tor.frame.z.vec();
  const math::Vec3 w{p.x - tor.frame.origin.x, p.y - tor.frame.origin.y, p.z - tor.frame.origin.z};
  const math::Vec3 radial = w - zc * math::dot(w, zc);
  const double rho = math::norm(radial);
  if (rho < 1e-12) return w;  // on the axis (degenerate) → any reference
  const math::Vec3 tubeCtr{tor.frame.origin.x + radial.x / rho * tor.radius,
                           tor.frame.origin.y + radial.y / rho * tor.radius,
                           tor.frame.origin.z + radial.z / rho * tor.radius};
  return math::Vec3{p.x - tubeCtr.x, p.y - tubeCtr.y, p.z - tubeCtr.z};
}

// A torus-surface CAP bounded by one traced seam loop: a radial fan from the cap's tube-surface
// CENTRE (evaluated ON the torus at the mean seam (u,v)) through `rings` concentric rings out to
// the exact traced seam nodes. Every interior node sits ON the torus (lerp the (u,v) centre→
// boundary, evaluate); the OUTER ring is the shared pooled seam nodes so cap↔band welds. Facets
// are oriented by the TRUE tube-outward normal at the centroid. `torUv` is the seam's torus
// (u,v) track (uvA or uvB per operand order). Mirrors appendMouthCap (torus normal, not axis).
void appendTransTorusCap(const CurvedSolid& tor, const Seam& seam,
                         const std::vector<std::pair<double, double>>& torUv, int rings,
                         VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int n = static_cast<int>(seam.pts.size());
  if (n < 3 || rings < 1 || static_cast<int>(torUv.size()) != n) return;
  // Cap centre in (u,v): mean of the boundary track, u unwrapped contiguously around the first
  // node (torus u is the periodic MAJOR angle) and v around the first node (minor angle).
  const double u0 = torUv.front().first, v0 = torUv.front().second;
  double uSum = 0.0, vSum = 0.0;
  for (const auto& p : torUv) { uSum += nearU(u0, p.first); vSum += nearU(v0, p.second); }
  const double uc = uSum / n, vc = vSum / n;
  auto ringPt = [&](int r, int k) -> math::Point3 {
    if (r == rings) return seam.pts[k];  // outer ring = exact traced seam node
    const double t = static_cast<double>(r) / rings;
    const double u = uc + (nearU(u0, torUv[k].first) - uc) * t;
    const double v = vc + (nearU(v0, torUv[k].second) - vc) * t;
    return tor.point(u, v);
  };
  const math::Point3 centre = tor.point(uc, vc);
  auto tri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    const math::Point3 ctr{(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
    pushPlanarTri(a, b, c, tubeOutwardAt(tor, ctr), pool, faces);
  };
  for (int k = 0; k < n; ++k) tri(centre, ringPt(1, k), ringPt(1, (k + 1) % n));
  for (int r = 2; r <= rings; ++r)
    for (int k = 0; k < n; ++k) {
      const int kn = (k + 1) % n;
      tri(ringPt(r - 1, k), ringPt(r, k), ringPt(r, kn));
      tri(ringPt(r - 1, k), ringPt(r, kn), ringPt(r - 1, kn));
    }
}

// Recognise the transversal (offset, axis-parallel) torus∩cylinder pierce-through pose from the
// two traced seams. Declines (→ ok=false) for the coaxial pose (offset ≤ tol, owned by S5-l), a
// skew (non-parallel) axis, ≠2 closed seams, or a pose failing the inside-the-other survival
// gate. Mirrors transCylSphereSetup: a linear pose-recognition + resample + survival gate.
TransTorusCylSetup transTorusCylSetup(const CurvedSolid& A, const CurvedSolid& B,
                                      const std::vector<Seam>& seams) {
  TransTorusCylSetup st;
  if (seams.size() != 2) return st;                       // pierce-through → exactly two loops
  for (const Seam& s : seams)
    if (!s.closed || s.pts.size() < 8) return st;

  const CurvedSolid* torPtr = nullptr;
  const CurvedSolid* cylPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Torus) torPtr = s;
    else if (s->kind == CurvedKind::Cylinder) cylPtr = s;
  }
  if (!torPtr || !cylPtr) return st;
  const CurvedSolid& tor = *torPtr;
  const CurvedSolid& cyl = *cylPtr;
  const bool torIsA = (&A == torPtr);
  const bool cylIsB = (&B == cylPtr);

  const math::Vec3 zt = tor.frame.z.vec();
  const math::Vec3 zc = cyl.frame.z.vec();
  // Cylinder axis must be PARALLEL to the torus axis (axis-parallel offset pose). A skew /
  // perpendicular cylinder is a different, harder locus → decline → OCCT.
  if (math::norm(math::cross(zt, zc)) > 1e-6) return st;
  const math::Point3 O = cyl.frame.origin;
  const math::Vec3 d{O.x - tor.frame.origin.x, O.y - tor.frame.origin.y, O.z - tor.frame.origin.z};
  const double offset = math::norm(d - zt * math::dot(d, zt));  // perpendicular cyl-axis offset
  if (offset <= 1e-4) return st;                          // coaxial → S5-l owns it → decline

  const double R = tor.radius, r = tor.minorRadius, Rc = cyl.radius;
  if (!(r > 1e-9) || !(R > r + 1e-9) || !(Rc > 1e-9)) return st;

  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * Rc, 1e-12));
  const int N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * Rc / chord)), 24, 200);

  Seam s0 = resampleByAzimuth(seams[0], cylIsB, N);
  Seam s1 = resampleByAzimuth(seams[1], cylIsB, N);

  st.tor = torPtr; st.cyl = cylPtr; st.torIsA = torIsA; st.cylIsB = cylIsB; st.O = O; st.zc = zc;
  st.Rc = Rc; st.offset = offset; st.N = N;
  // Cylinder axial extent along +zc (v param), ordered lo≤hi so end discs / stubs are consistent.
  st.cylLo = std::min(cyl.vLo, cyl.vHi);
  st.cylHi = std::max(cyl.vLo, cyl.vHi);
  const double m0 = transTorusSeamAxialMean(st, s0), m1 = transTorusSeamAxialMean(st, s1);
  if (std::fabs(m0 - m1) < 1e-4) return st;               // both loops at one station → not two sheets
  st.seamLo = (m0 <= m1) ? s0 : s1;
  st.seamHi = (m0 <= m1) ? s1 : s0;

  // SURVIVAL samples (the honest inside-the-other gate, never faked):
  //  * the cylinder band midpoint (on the cyl axis, half-way between the two seams) must be
  //    INSIDE the torus tube (the band is a COMMON boundary);
  //  * each cap centre (the tube-surface point at the mean seam (u,v)) must be INSIDE the
  //    cylinder (the caps close the COMMON).
  const double axLo = transTorusSeamAxialMean(st, st.seamLo);
  const double axHi = transTorusSeamAxialMean(st, st.seamHi);
  const double axMid = 0.5 * (axLo + axHi);
  const math::Point3 bandMid{O.x + zc.x * axMid, O.y + zc.y * axMid, O.z + zc.z * axMid};
  if (classifyPoint(tor, bandMid, kSsiTol) != 1) return st;
  auto capCentre = [&](const Seam& seam) {
    const auto& uv = torIsA ? seam.uvA : seam.uvB;
    const double u0 = uv.front().first, v0 = uv.front().second;
    double uSum = 0.0, vSum = 0.0;
    for (const auto& p : uv) { uSum += nearU(u0, p.first); vSum += nearU(v0, p.second); }
    return tor.point(uSum / uv.size(), vSum / uv.size());
  };
  if (classifyPoint(cyl, capCentre(st.seamLo), kSsiTol) != 1) return st;
  if (classifyPoint(cyl, capCentre(st.seamHi), kSsiTol) != 1) return st;

  // CUT/FUSE tube-outer-zone gate: the cylinder must poke fully THROUGH the tube, so BOTH end
  // discs (their whole rim + axis centre) sit strictly OUTSIDE the tube. Then the two seams
  // bound a clean bore and the complementary tube band (swept the long way round the minor
  // angle) is one connected zone. If an end disc lands inside the tube (a stub buried in the
  // tube) the band is not two-seam-complementary → CUT/FUSE decline (COMMON still lands).
  auto endDiscOutside = [&](double v) -> bool {
    if (classifyPoint(tor, math::Point3{O.x + zc.x * v, O.y + zc.y * v, O.z + zc.z * v},
                      kSsiTol) != -1)
      return false;
    for (int k = 0; k < 16; ++k)
      if (classifyPoint(tor, cyl.point(kSsiTwoPi * k / 16.0, v), kSsiTol) != -1) return false;
    return true;
  };
  st.cylPierces = endDiscOutside(st.cylLo) && endDiscOutside(st.cylHi);

  st.ok = true;
  return st;
}

// Ring count for a transversal torus cap: the cap's angular span (max seam-node distance from the
// cap centre, over the tube minor radius) × sqrt(r/(2·kCapSagitta)), bounded [4,48] — the S5-k
// cap-refinement discipline generalised to the tube surface.
int transTorusCapRings(const TransTorusCylSetup& s, const Seam& seam) {
  const auto& uv = s.torIsA ? seam.uvA : seam.uvB;
  const double u0 = uv.front().first, v0 = uv.front().second;
  double uSum = 0.0, vSum = 0.0;
  for (const auto& p : uv) { uSum += nearU(u0, p.first); vSum += nearU(v0, p.second); }
  const double uc = uSum / uv.size(), vc = vSum / uv.size();
  const math::Point3 centre = s.tor->point(uc, vc);
  double span = 0.0;
  for (const auto& p : seam.pts)
    span = std::max(span, math::norm(math::Vec3{p.x - centre.x, p.y - centre.y, p.z - centre.z}));
  return std::clamp(
      static_cast<int>(std::ceil(std::max(span, 1e-6) * std::sqrt(1.0 / (2.0 * kCapSagitta)))), 4,
      48);
}

// buildTransTorusCylCommon(A,B) = COMMON of the TRANSVERSAL (offset) torus∩cylinder: the torus
// lower cap (the tube sheet inside the cylinder, toward −zc) + the cylinder band between the two
// traced seams (inside the tube) + the torus upper cap (toward +zc). All three share the two
// traced seam rings through one VertexPool → watertight. The first transversal-torus slice.
topo::Shape buildTransTorusCylCommon(const CurvedSolid& A, const CurvedSolid& B,
                                     const std::vector<Seam>& seams) {
  const TransTorusCylSetup s = transTorusCylSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const auto& uvLo = s.torIsA ? s.seamLo.uvA : s.seamLo.uvB;
  const auto& uvHi = s.torIsA ? s.seamHi.uvA : s.seamHi.uvB;
  appendTransTorusCap(*s.tor, s.seamLo, uvLo, transTorusCapRings(s, s.seamLo), pool, faces);
  appendRevolvedBand(s.seamLo.pts, s.seamHi.pts, s.O, s.zc, pool, faces, 1.0);  // cylinder band
  appendTransTorusCap(*s.tor, s.seamHi, uvHi, transTorusCapRings(s, s.seamHi), pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── TORUS TUBE OUTER ZONE = the FULL tube surface MINUS the two seam-bounded cap patches ──
// The transversal torus∩cylinder outer zone (the tube surface OUTSIDE the cylinder bore) is NOT a
// between-two-seams band the way the SPHERE outer zone is (appendSphereOuterZoneBetweenSeams). The
// sphere version works because a cylinder pierces the sphere pole-to-pole, so its two seams are
// LATITUDE-like loops each ENCIRCLING the cylinder axis (they span the full azimuth θ), and the
// sphere surface between them is a clean equatorial belt swept at constant θ. The offset torus∩cyl
// geometry is genuinely different: a thin axis-parallel cylinder pierces ONE side of the tube, so
// its two seams are LOCALIZED closed loops in the torus (u,v) plane (MEASURED: torus major u ∈
// [−0.2, 0.2], NOT the full [0,2π]). The tube surface outside the bore is therefore the ENTIRE
// torus tube MINUS the two small cap patches — a doubly-holed torus surface, not a v-sweep band.
//
// The exact-on-surface tiling: mesh the full torus tube on a (Nu × Nv) (u,v) grid (every node ON
// the torus via tor.point), DROP every quad whose (u,v) centre falls inside either seam loop (the
// cap patches, already tiled by the COMMON caps), and STITCH each grid hole to the exact seam loop
// with a LOOP ZIPPER (a watertight triangle strip between two closed loops of possibly-different
// node counts). The seam nodes are the shared pool vertices the cylinder tunnel / stub bands weld
// to, so the whole CUT/FUSE shell closes along the exact traced seams. Facets are oriented by the
// TRUE tube-outward reference (tubeOutwardAt), scaled by outwardSign (+1 outward / −1 reversed).
//
// systems-band (~30 — grid + point-in-loop + loop-zipper stitch); isolated + flagged per policy.

// A watertight LOOP ZIPPER between an OUTER closed loop `outer` (the grid-rectangle ring, Po nodes)
// and an INNER closed loop `innerIn` (the exact seam, Pi nodes). The inner loop is first ALIGNED to
// the outer: reversed if its winding (about the shared centroid, projected on the local tube-normal
// plane) is opposite, and rotated so its node 0 is the nearest to outer[0]. Then it advances
// whichever loop is "behind" in fractional arc position, emitting one triangle per step → a closed
// triangle strip sharing every node of both loops. Facets oriented by tubeOutwardAt × outwardSign.
void zipLoops(const CurvedSolid& tor, const std::vector<math::Point3>& outer,
              const std::vector<math::Point3>& innerIn, double outwardSign, VertexPool& pool,
              std::vector<topo::Shape>& faces) {
  const int Po = static_cast<int>(outer.size()), Pi = static_cast<int>(innerIn.size());
  if (Po < 3 || Pi < 3) return;
  // Shared centroid + a plane normal (tube-outward at the centroid) for a consistent winding sign.
  math::Point3 ctrO{0, 0, 0};
  for (const auto& p : outer) { ctrO.x += p.x; ctrO.y += p.y; ctrO.z += p.z; }
  ctrO.x /= Po; ctrO.y /= Po; ctrO.z /= Po;
  const math::Vec3 nrm = tubeOutwardAt(tor, ctrO);
  auto signedTurn = [&](const std::vector<math::Point3>& L) {
    double acc = 0.0;
    const int n = static_cast<int>(L.size());
    for (int i = 0; i < n; ++i) {
      const math::Vec3 a{L[i].x - ctrO.x, L[i].y - ctrO.y, L[i].z - ctrO.z};
      const math::Vec3 b{L[(i + 1) % n].x - ctrO.x, L[(i + 1) % n].y - ctrO.y,
                         L[(i + 1) % n].z - ctrO.z};
      acc += math::dot(math::cross(a, b), nrm);
    }
    return acc;
  };
  std::vector<math::Point3> inner = innerIn;
  if (signedTurn(outer) * signedTurn(inner) < 0.0) std::reverse(inner.begin(), inner.end());
  // Rotate inner so inner[0] is nearest outer[0].
  int best = 0; double bd = 1e300;
  for (int i = 0; i < Pi; ++i) {
    const math::Vec3 d{inner[i].x - outer[0].x, inner[i].y - outer[0].y, inner[i].z - outer[0].z};
    const double dd = math::dot(d, d);
    if (dd < bd) { bd = dd; best = i; }
  }
  std::rotate(inner.begin(), inner.begin() + best, inner.end());

  auto emit = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    const math::Point3 ctr{(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
    const math::Vec3 t = tubeOutwardAt(tor, ctr);
    pushPlanarTri(a, b, c, math::Vec3{t.x * outwardSign, t.y * outwardSign, t.z * outwardSign},
                  pool, faces);
  };
  int io = 0, ii = 0;
  while (io < Po || ii < Pi) {
    const double fo = static_cast<double>(io) / Po, fi = static_cast<double>(ii) / Pi;
    if (io < Po && (ii >= Pi || fo <= fi)) {            // advance the OUTER loop
      emit(outer[io % Po], outer[(io + 1) % Po], inner[ii % Pi]);
      ++io;
    } else {                                            // advance the INNER loop
      emit(outer[io % Po], inner[(ii + 1) % Pi], inner[ii % Pi]);
      ++ii;
    }
  }
}

// The tube OUTER zone = full torus tube grid, minus a small grid RECTANGLE around each seam,
// with each rectangle ring zipped to the exact seam loop. Returns false (appending nothing on a
// clean-fail path) if either seam is not localizable inside a clean grid rectangle whose boundary
// is entirely OUTSIDE the seam (→ the caller HONEST-DECLINES). Every emitted node is ON the torus
// (grid via tor.point, ring via tor.point, seam = exact traced nodes) so the zone is on-surface to
// machine precision; the seam nodes are shared pool vertices the bore/stub bands weld to.
bool appendTorusTubeOuterZone(const CurvedSolid& tor, const Seam& seamHi, const Seam& seamLo,
                              bool torIsA, VertexPool& pool, std::vector<topo::Shape>& faces,
                              double outwardSign) {
  const auto& uvHi = torIsA ? seamHi.uvA : seamHi.uvB;
  const auto& uvLo = torIsA ? seamLo.uvA : seamLo.uvB;
  if (uvHi.size() < 3 || uvLo.size() < 3) return false;
  // Grid resolution bounded by the S5-l full-grid facet-sagitta convention (kFacetSag=0.004, the
  // mesh deflection) rather than the far-finer kCapSagitta — the facet chord error then rides at
  // the mesh deflection, well inside the 1% curved-parity bar, while keeping the total facet count
  // (≈ Nu·Nv) tractable for the mesher (the whole tube is a full N×M grid, not a few bands).
  const double kFacetSag = 0.004;
  const double chordU = std::sqrt(std::max(8.0 * kFacetSag * tor.radius, 1e-12));
  const double chordV = std::sqrt(std::max(8.0 * kFacetSag * tor.minorRadius, 1e-12));
  const int Nu = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * tor.radius / chordU)), 32, 200);
  const int Nv = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * tor.minorRadius / chordV)), 24, 160);
  auto uAt = [&](int i) { return kSsiTwoPi * (((i % Nu) + Nu) % Nu) / Nu; };
  auto vAt = [&](int j) { return kSsiTwoPi * (((j % Nv) + Nv) % Nv) / Nv; };

  // Per-seam grid RECTANGLE [i0,i1]×[j0,j1] (grid-index space, i on u, j on v) strictly containing
  // the seam bbox with one cell of margin. Cells inside the rectangle are OMITTED from the grid;
  // the rectangle ring is zipped to the seam. Requires: the seam localized (span < ~a third of the
  // grid in each param, so the rectangle does not wrap) — else decline.
  struct Rect { int i0, i1, j0, j1; };
  auto rectOf = [&](const std::vector<std::pair<double, double>>& uv, Rect& R) -> bool {
    const double u0 = uv.front().first, v0 = uv.front().second;
    double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
    for (const auto& p : uv) {
      const double u = nearU(u0, p.first), v = nearU(v0, p.second);
      umin = std::min(umin, u); umax = std::max(umax, u);
      vmin = std::min(vmin, v); vmax = std::max(vmax, v);
    }
    R.i0 = static_cast<int>(std::floor(umin / kSsiTwoPi * Nu)) - 1;
    R.i1 = static_cast<int>(std::ceil(umax / kSsiTwoPi * Nu)) + 1;
    R.j0 = static_cast<int>(std::floor(vmin / kSsiTwoPi * Nv)) - 1;
    R.j1 = static_cast<int>(std::ceil(vmax / kSsiTwoPi * Nv)) + 1;
    // Localized: the rectangle must be well inside the grid (no wrap-overlap) in BOTH params.
    return (R.i1 - R.i0) >= 2 && (R.i1 - R.i0) <= Nu - 2 &&
           (R.j1 - R.j0) >= 2 && (R.j1 - R.j0) <= Nv - 2;
  };
  Rect RH{}, RL{};
  if (!rectOf(uvHi, RH) || !rectOf(uvLo, RL)) return false;
  // The two rectangles must be DISJOINT in grid-index space (mod Nu/Nv) so cells are omitted once.
  auto inRect = [&](int i, int j, const Rect& R) {
    for (int di = R.i0; di < R.i1; ++di) {
      const int ii = ((di % Nu) + Nu) % Nu;
      if (ii != (((i % Nu) + Nu) % Nu)) continue;
      if (j >= R.j0 && j < R.j1) return true;
    }
    return false;
  };
  // Emit every grid quad NOT inside either seam rectangle.
  for (int i = 0; i < Nu; ++i)
    for (int j = 0; j < Nv; ++j) {
      if (inRect(i, j, RH) || inRect(i, j, RL)) continue;
      const math::Point3 a = tor.point(uAt(i), vAt(j)), b = tor.point(uAt(i + 1), vAt(j));
      const math::Point3 c = tor.point(uAt(i + 1), vAt(j + 1)), dd = tor.point(uAt(i), vAt(j + 1));
      const math::Point3 ctr{(a.x + b.x + c.x + dd.x) / 4, (a.y + b.y + c.y + dd.y) / 4,
                             (a.z + b.z + c.z + dd.z) / 4};
      const math::Vec3 t = tubeOutwardAt(tor, ctr);
      const math::Vec3 ref{t.x * outwardSign, t.y * outwardSign, t.z * outwardSign};
      pushPlanarTri(a, b, c, ref, pool, faces);
      pushPlanarTri(a, c, dd, ref, pool, faces);
    }
  // Build each rectangle RING (the closed loop of grid nodes bounding the omitted block, ordered
  // consistently with the seam) and zip it to the exact seam. The ring nodes are the SAME grid
  // nodes the surrounding kept quads use → the ring welds to the grid; the zipper welds ring→seam.
  auto ringOf = [&](const Rect& R) {
    std::vector<math::Point3> ring;
    for (int i = R.i0; i < R.i1; ++i) ring.push_back(tor.point(uAt(i), vAt(R.j0)));      // bottom
    for (int j = R.j0; j < R.j1; ++j) ring.push_back(tor.point(uAt(R.i1), vAt(j)));      // right
    for (int i = R.i1; i > R.i0; --i) ring.push_back(tor.point(uAt(i), vAt(R.j1)));      // top
    for (int j = R.j1; j > R.j0; --j) ring.push_back(tor.point(uAt(R.i0), vAt(j)));      // left
    return ring;
  };
  zipLoops(tor, ringOf(RH), seamHi.pts, outwardSign, pool, faces);
  zipLoops(tor, ringOf(RL), seamLo.pts, outwardSign, pool, faces);
  return true;
}

// buildTransTorusCylCut / buildTransTorusCylFuse — the transversal CUT / FUSE of the offset
// torus∩cylinder, LANDED via the tube OUTER zone (`appendTorusTubeOuterZone`): the FULL torus tube
// grid MINUS the two localized seam cap patches (MEASURED: the seams are localized in torus u,
// NOT azimuth-wrapping, so the outer zone is a doubly-holed torus surface, NOT a between-two-seams
// band like the SPHERE family — the sphere primitive does NOT apply here). Each grid hole is a
// small rectangle zipped to the exact traced seam, so the shell is watertight on-surface and welds
// to the reversed bore (CUT) / cylinder stubs (FUSE) along the shared seam nodes.
//
// Gated on `cylPierces` (both cylinder end discs outside the tube → the two seams bound a clean
// bore) AND on each seam being localizable inside a clean grid rectangle (appendTorusTubeOuterZone
// returns false otherwise). Any pose that cannot weld robustly HONEST-DECLINES → NULL → OCCT.
//
// buildTransTorusCylCut(A,B) = A − B (TORUS minuend only; cyl − torus declines, order-sensitive):
//   the tube OUTER zone (outward) + the cylinder tunnel wall between the two seams (INWARD normal
//   — the reversed bore). V = V(torus) − V(COMMON).
topo::Shape buildTransTorusCylCut(const CurvedSolid& A, const CurvedSolid& B,
                                  const std::vector<Seam>& seams) {
  const TransTorusCylSetup s = transTorusCylSetup(A, B, seams);
  if (!s.ok || !s.cylPierces) return {};
  if (A.kind != CurvedKind::Torus) return {};  // order-sensitive: torus must be the minuend
  VertexPool pool;
  std::vector<topo::Shape> faces;
  if (!appendTorusTubeOuterZone(*s.tor, s.seamHi, s.seamLo, s.torIsA, pool, faces, 1.0)) return {};
  appendRevolvedBand(s.seamLo.pts, s.seamHi.pts, s.O, s.zc, pool, faces, /*outwardSign=*/-1.0);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTransTorusCylFuse(A,B) = A ∪ B: the tube OUTER zone + the two cylinder end stubs beyond the
// seams (cylLo→seamLo, seamHi→cylHi walls) + the two cylinder end discs. V = V(A)+V(B)−V(∩).
topo::Shape buildTransTorusCylFuse(const CurvedSolid& A, const CurvedSolid& B,
                                   const std::vector<Seam>& seams) {
  const TransTorusCylSetup s = transTorusCylSetup(A, B, seams);
  if (!s.ok || !s.cylPierces) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ringLo = s.cylRing(s.cylLo);
  const std::vector<math::Point3> ringHi = s.cylRing(s.cylHi);
  appendDiskCap(*s.cyl, s.cylLo, ringLo, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ringLo, s.seamLo.pts, s.O, s.zc, pool, faces, 1.0);
  if (!appendTorusTubeOuterZone(*s.tor, s.seamHi, s.seamLo, s.torIsA, pool, faces, 1.0)) return {};
  appendRevolvedBand(s.seamHi.pts, ringHi, s.O, s.zc, pool, faces, 1.0);
  appendDiskCap(*s.cyl, s.cylHi, ringHi, s.zc, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ═══ S5-q — TRANSVERSAL (NON-COAXIAL) CONE ∩ SPHERE COMMON (CUT/FUSE decline) ═════════
// The THIRD transversal (non-coaxial) curved-boolean slice, after S5-k (offset cyl∩sphere)
// and S5-p (offset torus∩cyl), and the FIRST transversal CONE pair. Where S5-f/S5-h handle
// the COAXIAL cone(frustum)∩sphere pose (sphere centre ON the cone axis → ANALYTIC circle
// seams, planar rings), S5-q handles the OFFSET pose: the cone axis is DISPLACED
// (perpendicular offset) from the sphere centre, so the two seams are NON-PLANAR closed space
// curves (a cone∩sphere quartic locus — no analytic circle exists). The seam is consumed
// DIRECTLY from the S3 TraceSet (the general SSI machinery), not re-derived from a closed form.
//
// SCOPE (the clean, robustly-handleable transversal pose): a THIN cone whose axis is offset
// from the sphere centre but still passes fully THROUGH the sphere — both cone ends poke out
// beyond the sphere, so the cone wall crosses the sphere at exactly TWO disjoint closed loops
// (a lower loop + an upper loop along the cone axis), both fully transversal (nearTangentGaps
// ==0, branchPoints==0, both Closed). The COMMON is then the SAME TOPOLOGY as the transversal
// cyl∩sphere S5-k COMMON — a cone mid-band capped by two spherical caps — but every ring is the
// traced NON-PLANAR seam:
//   COMMON = sphere lower cap (inside the cone) + cone band (seamLo→seamHi, inside the sphere)
//     + sphere upper cap (inside the cone). Its boundary rings are the two traced seams (shared
//     VertexPool weld). Every fragment is seam-driven and verified. The cone band uses the SAME
//     appendRevolvedBand as S5-k (the cone-axis radial reference orients each facet outward for
//     a widening wall exactly as for a cylinder wall), only the axis carries the cone slant.
//
// CUT / FUSE both additionally need the sphere OUTER SHELL (the sphere ZONE between the two
// NON-PLANAR seams, the long way round outside the cone); welding that zone as a shared-pool
// planar-facet shell is the SAME UNRESOLVED transversal residual that made S5-k / S5-p decline
// (no revolved-band / far-pole meridian tiles a two-non-planar-seam zone watertight), so
// CUT/FUSE HONEST-DECLINE → OCCT.
//
// REDUCTION: as the offset → 0 the pose becomes coaxial and S5-f/S5-h's `coneSphere*Setup`
// claims it FIRST in the dispatch (they run before S5-q), reproducing the landed coaxial result;
// S5-q therefore gates on a STRICTLY-POSITIVE offset (else → decline). Anything else — a cone
// that does not pierce both poles (a single-loop / tangent pose → OCCT), a fat cone (a wall that
// does not thread through), a near-cylindrical cone (tanα≈0 → S5-k territory), ≠2 closed seams —
// declines → OCCT. The engine owns the watertight + volume gate + OCCT fallback. Nothing here is
// faked: any pose that cannot weld robustly returns NULL.
struct TransConeSphereSetup {
  bool ok = false;
  const CurvedSolid* cone = nullptr;
  const CurvedSolid* sph = nullptr;
  math::Point3 O;            ///< cone origin (apex-frame origin: r_cone(s)=R0+s·tanα)
  math::Vec3 zc;             ///< cone axis (unit)
  math::Point3 C;            ///< sphere centre
  double Rs = 0.0;           ///< sphere radius
  double offset = 0.0;       ///< perpendicular distance of the sphere centre from the cone axis
  bool coneIsB = false;      ///< which operand is the cone (picks the seam (u,v) track)
  int N = 0;                 ///< common azimuth sample count (seam-chord bounded)
  Seam seamLo{}, seamHi{};   ///< the two traced seams, resampled + ordered lo/hi along zc
  double coneLo = 0.0, coneHi = 0.0;  ///< cone axial extent (v param) ordered along +zc
  // The two-cap+outer-zone topology (same class as S5-k) is only valid when BOTH sphere poles
  // (±zc from the centre) sit strictly INSIDE the cone. Gated by CUT/FUSE, not COMMON.
  bool polesInsideCone = false;
  // A point on the cone axis far beyond the lower / upper seam (selects each cap's apex).
  math::Point3 axisFarM() const {
    return math::Point3{O.x - zc.x * 1e4, O.y - zc.y * 1e4, O.z - zc.z * 1e4};
  }
  math::Point3 axisFarP() const {
    return math::Point3{O.x + zc.x * 1e4, O.y + zc.y * 1e4, O.z + zc.z * 1e4};
  }
  math::Point3 poleM() const {
    return math::Point3{C.x - zc.x * Rs, C.y - zc.y * Rs, C.z - zc.z * Rs};
  }
  math::Point3 poleP() const {
    return math::Point3{C.x + zc.x * Rs, C.y + zc.y * Rs, C.z + zc.z * Rs};
  }
  // A cone wall ring at AXIAL station s (measured along zc from the cone origin), sampled at the
  // SAME azimuths as the seams (index i ↔ seam index i) so an end disc / wall band welds
  // ring-to-ring through the pool. Placed via the cone frame at radius rCone(s)=radius+s·tanα so
  // the ring's axial station matches appendDiskCap's axis point (origin+zc·s) EXACTLY — the cone
  // surface's own (u,v) `point` uses the SLANT v, which is axially short and would tilt the disc.
  std::vector<math::Point3> coneRingAxial(double s) const {
    const math::Vec3 X = cone->frame.x.vec(), Y = cone->frame.y.vec();
    const double rC = cone->radius + s * std::tan(cone->semiAngle);
    std::vector<math::Point3> r(seamLo.pts.size());
    for (size_t i = 0; i < seamLo.pts.size(); ++i) {
      const double u = coneIsB ? seamLo.uvB[i].first : seamLo.uvA[i].first;
      const double cx = rC * std::cos(u), cy = rC * std::sin(u);
      r[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                          O.y + X.y * cx + Y.y * cy + zc.y * s,
                          O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return r;
  }
};

// Mean axial projection (onto the cone axis from the cone origin) of a seam's nodes.
double transConeSeamAxialMean(const TransConeSphereSetup& s, const Seam& seam) {
  double acc = 0.0;
  for (const auto& p : seam.pts)
    acc += math::dot(math::Vec3{p.x - s.O.x, p.y - s.O.y, p.z - s.O.z}, s.zc);
  return acc / static_cast<double>(std::max<size_t>(1, seam.pts.size()));
}

// Recognise the transversal (offset) cone∩sphere pierce-both-ends pose from the two traced
// seams. Declines (→ ok=false) for the coaxial pose (offset ≤ tol, owned by S5-f/S5-h), a
// near-cylindrical cone (tanα≈0, S5-k territory), ≠2 closed seams, or a pose whose caps/band
// fail the inside-the-other survival sample. Mirrors transCylSphereSetup: a linear pose-
// recognition + resample + survival gate; isolated + documented per the complexity policy.
TransConeSphereSetup transConeSphereSetup(const CurvedSolid& A, const CurvedSolid& B,
                                          const std::vector<Seam>& seams) {
  TransConeSphereSetup st;
  if (seams.size() != 2) return st;                       // pierce-both-ends → exactly two loops
  for (const Seam& s : seams)
    if (!s.closed || s.pts.size() < 8) return st;

  const CurvedSolid* conePtr = nullptr;
  const CurvedSolid* sphPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Cone) conePtr = s;
    else if (s->kind == CurvedKind::Sphere) sphPtr = s;
  }
  if (!conePtr || !sphPtr) return st;
  const CurvedSolid& cone = *conePtr;
  const CurvedSolid& sph = *sphPtr;
  const bool coneIsB = (&B == conePtr);

  const double tanA = std::tan(cone.semiAngle);
  if (std::fabs(tanA) < 1e-6) return st;                  // near-cylindrical → S5-k territory
  const math::Vec3 zc = cone.frame.z.vec();
  const math::Point3 O = cone.frame.origin;
  const math::Point3 C = sph.frame.origin;
  const math::Vec3 d{C.x - O.x, C.y - O.y, C.z - O.z};
  const double sc = math::dot(d, zc);
  const double offset = math::norm(d - zc * sc);          // perpendicular sphere-centre offset
  if (offset <= 1e-4) return st;                          // coaxial → S5-f/S5-h own it → decline
  const double Rs = sph.radius;
  if (!(Rs > 1e-9)) return st;

  // Azimuthal resolution from the sphere-radius chord bound (the cone wall radius varies with
  // station; the sphere radius is the stable curvature scale for both caps + band).
  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * Rs, 1e-12));
  const int N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * Rs / chord)), 24, 200);

  Seam s0 = resampleByAzimuth(seams[0], coneIsB, N);
  Seam s1 = resampleByAzimuth(seams[1], coneIsB, N);

  st.cone = conePtr; st.sph = sphPtr; st.O = O; st.zc = zc; st.C = C;
  st.Rs = Rs; st.offset = offset; st.coneIsB = coneIsB; st.N = N;
  st.coneLo = std::min(cone.vLo, cone.vHi);
  st.coneHi = std::max(cone.vLo, cone.vHi);
  const double m0 = transConeSeamAxialMean(st, s0), m1 = transConeSeamAxialMean(st, s1);
  if (std::fabs(m0 - m1) < 1e-4) return st;                // both loops at one station → not two ends
  st.seamLo = (m0 <= m1) ? s0 : s1;
  st.seamHi = (m0 <= m1) ? s1 : s0;

  // SURVIVAL samples (the honest inside-the-other gate, never faked):
  //  * a cone-wall point half-way between the two seams (the band midpoint, on the cone wall at
  //    azimuth 0) must be INSIDE the sphere (the band is a COMMON boundary);
  //  * each cap apex direction (the sphere point nearest the axis-far point) must be INSIDE the
  //    cone (the caps close the COMMON).
  const double axLo = transConeSeamAxialMean(st, st.seamLo), axHi = transConeSeamAxialMean(st, st.seamHi);
  const double sMid = 0.5 * (axLo + axHi);
  const double rMid = cone.radius + sMid * tanA;
  const math::Vec3 Xc = cone.frame.x.vec();
  const math::Point3 bandMid{O.x + Xc.x * rMid + zc.x * sMid, O.y + Xc.y * rMid + zc.y * sMid,
                             O.z + Xc.z * rMid + zc.z * sMid};
  if (classifyPoint(sph, bandMid, kSsiTol) != 1) return st;
  auto capApex = [&](const math::Point3& far) {
    const math::Vec3 t{far.x - C.x, far.y - C.y, far.z - C.z};
    const double L = std::max(math::norm(t), 1e-12);
    return math::Point3{C.x + t.x / L * Rs, C.y + t.y / L * Rs, C.z + t.z / L * Rs};
  };
  if (classifyPoint(cone, capApex(st.axisFarM()), kSsiTol) != 1) return st;
  if (classifyPoint(cone, capApex(st.axisFarP()), kSsiTol) != 1) return st;

  // OUTER-ZONE gate (CUT/FUSE): both sphere poles (±zc from the centre) strictly INSIDE the cone.
  st.polesInsideCone = classifyPoint(cone, st.poleM(), kSsiTol) == 1 &&
                       classifyPoint(cone, st.poleP(), kSsiTol) == 1;

  st.ok = true;
  return st;
}

// Ring count for a transversal cone∩sphere sphere cap: the cap's polar half-angle (apex→farthest
// seam node, from the sphere centre) × sqrt(Rs/(2·kCapSagitta)), bounded [4,48] — the S5-k
// transCapRings discipline (a non-planar seam), reused here identically.
int transConeCapRings(const TransConeSphereSetup& s, const Seam& seam, const math::Point3& apex) {
  const math::Vec3 aDir{apex.x - s.C.x, apex.y - s.C.y, apex.z - s.C.z};
  double theta = 0.0;
  for (const auto& p : seam.pts) {
    const math::Vec3 sDir{p.x - s.C.x, p.y - s.C.y, p.z - s.C.z};
    const double den = std::max(math::norm(aDir) * math::norm(sDir), 1e-12);
    theta = std::max(theta, std::acos(std::clamp(math::dot(aDir, sDir) / den, -1.0, 1.0)));
  }
  return std::clamp(
      static_cast<int>(std::ceil(std::max(theta, 1e-6) * std::sqrt(s.Rs / (2.0 * kCapSagitta)))), 4,
      48);
}

// buildTransConeSphereCommon(A,B) = COMMON of the TRANSVERSAL (offset) cone∩sphere: the sphere
// lower cap (apex toward −zc, inside the cone) + the cone band between the two traced seams
// (inside the sphere) + the sphere upper cap (apex toward +zc). All three share the two traced
// seam rings through one VertexPool → watertight. The first transversal-cone slice.
topo::Shape buildTransConeSphereCommon(const CurvedSolid& A, const CurvedSolid& B,
                                       const std::vector<Seam>& seams) {
  const TransConeSphereSetup s = transConeSphereSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const math::Point3 apexM = s.axisFarM(), apexP = s.axisFarP();
  appendSphereCap(*s.sph, apexM, s.seamLo, transConeCapRings(s, s.seamLo, apexM), pool, faces,
                  /*outer=*/false, /*reversed=*/false);
  appendRevolvedBand(s.seamLo.pts, s.seamHi.pts, s.O, s.zc, pool, faces);  // cone band
  appendSphereCap(*s.sph, apexP, s.seamHi, transConeCapRings(s, s.seamHi, apexP), pool, faces,
                  /*outer=*/false, /*reversed=*/false);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTransConeSphereCut / buildTransConeSphereFuse — the transversal CUT / FUSE of the offset
// cone∩sphere, LANDED by the SAME seam-band primitive as S5-k (`appendSphereOuterZoneBetweenSeams`
// about the CONE axis). The sphere OUTER ZONE (the long way round outside the cone) is tiled
// on-surface by the cone-axis φ-sweep; the cone side is exact via straight-ruling revolved bands
// (a cone generatrix is straight) + cone end discs. Gated on `polesInsideCone`; otherwise the
// two-cap+outer-zone topology fails and we HONEST-DECLINE → OCCT. Nothing faked.
//   * SPHERE − CONE: sphere outer zone + reversed cone tunnel band.
//   * CONE − SPHERE: two cone stubs each dimpled by a reversed COMMON sphere cap + cone end discs.
//   * FUSE: sphere outer zone + two cone end stubs + cone end discs.
topo::Shape buildTransConeSphereCut(const CurvedSolid& A, const CurvedSolid& B,
                                    const std::vector<Seam>& seams) {
  const TransConeSphereSetup s = transConeSphereSetup(A, B, seams);
  if (!s.ok || !s.polesInsideCone) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  if (&A == s.sph) {
    // SPHERE − CONE: outer sphere zone + reversed cone tunnel band (the bore).
    appendSphereOuterZoneBetweenSeams(s.C, s.Rs, s.zc, s.seamHi, s.seamLo, pool, faces, 1.0);
    appendRevolvedBand(s.seamLo.pts, s.seamHi.pts, s.O, s.zc, pool, faces, /*outwardSign=*/-1.0);
    if (faces.size() < 4) return {};
  } else {
    // CONE − SPHERE: two disconnected cone stubs, each dimpled by a reversed COMMON cap.
    const math::Point3 apexM = s.axisFarM(), apexP = s.axisFarP();
    const std::vector<math::Point3> ringLo = s.coneRingAxial(s.coneLo);
    const std::vector<math::Point3> ringHi = s.coneRingAxial(s.coneHi);
    appendDiskCap(*s.cone, s.coneLo, ringLo, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
    appendRevolvedBand(ringLo, s.seamLo.pts, s.O, s.zc, pool, faces);
    appendSphereCap(*s.sph, apexM, s.seamLo, transConeCapRings(s, s.seamLo, apexM), pool, faces,
                    /*outer=*/false, /*reversed=*/true);
    appendSphereCap(*s.sph, apexP, s.seamHi, transConeCapRings(s, s.seamHi, apexP), pool, faces,
                    /*outer=*/false, /*reversed=*/true);
    appendRevolvedBand(s.seamHi.pts, ringHi, s.O, s.zc, pool, faces);
    appendDiskCap(*s.cone, s.coneHi, ringHi, s.zc, pool, faces);
    if (faces.size() < 8) return {};
  }
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

topo::Shape buildTransConeSphereFuse(const CurvedSolid& A, const CurvedSolid& B,
                                     const std::vector<Seam>& seams) {
  const TransConeSphereSetup s = transConeSphereSetup(A, B, seams);
  if (!s.ok || !s.polesInsideCone) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ringLo = s.coneRingAxial(s.coneLo);
  const std::vector<math::Point3> ringHi = s.coneRingAxial(s.coneHi);
  appendDiskCap(*s.cone, s.coneLo, ringLo, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ringLo, s.seamLo.pts, s.O, s.zc, pool, faces);        // lower cone stub
  appendSphereOuterZoneBetweenSeams(s.C, s.Rs, s.zc, s.seamHi, s.seamLo, pool, faces, 1.0);
  appendRevolvedBand(s.seamHi.pts, ringHi, s.O, s.zc, pool, faces);        // upper cone stub
  appendDiskCap(*s.cone, s.coneHi, ringHi, s.zc, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ═══ S5-s — TRANSVERSAL (NON-COAXIAL) CONE ∩ CYLINDER COMMON (CUT/FUSE decline) ═══════
// The FOURTH transversal (non-coaxial) curved-boolean slice, after S5-k (offset cyl∩sphere),
// S5-p (offset torus∩cyl) and S5-q (offset cone∩sphere), and the FIRST transversal cone∩cyl pair.
// Where S5-e handles the COAXIAL cone(frustum)∩cylinder pose (cone axis colinear with the cylinder
// axis → a single ANALYTIC circle seam, a solid of revolution), S5-s handles the OFFSET pose: the
// cylinder axis is PARALLEL to the cone axis but DISPLACED perpendicular from it, so a thin
// cylinder crosses the SLANTED cone wall in ONE NON-PLANAR closed space curve (the cone∩cylinder
// quartic locus — no analytic circle exists), consumed DIRECTLY from the S3 TraceSet.
//
// SINGLE-SEAM TOPOLOGY (why this is NOT the S5-k/p/q two-loop pose). A cylinder whose axis is
// PARALLEL to the cone axis meets the cone in a SINGLE closed loop, not two: the cone wall radius
// r(y)=R0+y·tanα is MONOTONIC in the axial coordinate, so a parallel-offset cylinder crosses it
// exactly once along its length — the cylinder cross-section is fully OUTSIDE the cone at the
// narrow end and fully INSIDE at the wide end (measured + verified: for every parallel-axis pose
// the trace returns ONE loop; a fat cylinder straddling the axis returns at most an end-clipped
// open arc, never two closed poke-through loops). So the S5-k/p/q "cone band capped by two caps"
// machinery does NOT apply here; S5-s is a distinct SINGLE-SEAM assembler.
//
// SCOPE (the clean, robustly-handleable single-crossing pose): a THIN cylinder whose axis is
// parallel to the cone axis, offset perpendicular, whose whole cross-section is OUTSIDE the cone at
// its NARROW-end disc and fully INSIDE the cone at its WIDE-end disc (a clean single wall crossing
// → exactly ONE fully-transversal closed seam, nearTangentGaps==0, branchPoints==0, Closed). The
// COMMON is then the part of the cylinder inside the cone, bounded by three seam-driven fragments:
//   COMMON = cone-wall CAP (the cone-surface sheet inside the cylinder, bounded by the seam) +
//     cylinder-wall BAND (from the seam up to the cylinder's INSIDE-end rim) + cylinder INSIDE-END
//     DISC (a full disc at the wide end, entirely inside the cone). The cone cap + cyl band share
//     the traced seam ring, and the cyl band + end disc share the inside-end rim ring, all through
//     one VertexPool → watertight. Every fragment is seam- or analytic-rim-driven and verified.
//     This is the landed transversal cone∩cyl slice.
//
// The cone cap is a radial fan from the cap's cone-surface CENTRE (evaluated ON the cone at the
// mean seam (u,v)) through concentric rings out to the exact traced seam nodes, each interior node
// placed ON the cone (lerp the (u,v) centre→boundary, evaluate), oriented by the TRUE cone-outward
// normal (radial·cosα − axial·sinα). The cylinder band is appendRevolvedBand between the seam ring
// and the inside-end rim (both on a common azimuth grid → welds ring-to-ring); the end disc is a
// full appendDiskCap fan on the cylinder axis.
//
// CUT / FUSE both additionally need the cone OUTER SHELL / the cylinder OUTER stub welded across
// the non-planar seam (the outer-zone weld that made S5-k / S5-p / S5-q decline). So CUT/FUSE
// HONEST-DECLINE → OCCT; COMMON is the landed slice. Nothing is faked.
//
// REDUCTION: as the offset → 0 the pose becomes coaxial and S5-e's `coneCylSetup` claims it FIRST
// in the dispatch (it runs before S5-s), reproducing the landed coaxial result; S5-s therefore
// gates on a STRICTLY-POSITIVE offset (else → decline, letting S5-e own it). A skew (non-parallel)
// cylinder axis, ≠1 closed seam, or a pose failing the clean-single-crossing survival samples all
// decline → OCCT. The engine owns the watertight + volume gate + OCCT fallback. Nothing is faked.
struct TransConeCylSetup {
  bool ok = false;
  const CurvedSolid* cone = nullptr;
  const CurvedSolid* cyl = nullptr;
  bool coneIsA = true;        ///< which operand is the cone (picks the seam (u,v) track)
  bool cylIsB = true;         ///< which operand is the cylinder (picks the seam (u,v) track)
  math::Point3 O;             ///< cylinder frame origin
  math::Vec3 zc;              ///< cylinder axis (unit) — parallel to the cone axis
  double Rc = 0.0;            ///< cylinder radius
  double offset = 0.0;        ///< perpendicular distance of the cyl axis from the cone axis
  double vInside = 0.0;       ///< cylinder frame-v of the WIDE (inside-the-cone) end disc
  double vOutside = 0.0;      ///< cylinder frame-v of the NARROW (outside-the-cone) end disc
  double coneLo = 0.0, coneHi = 0.0;  ///< cone wall slant-v extent, ordered lo≤hi
  int N = 0;                  ///< common azimuth sample count (seam-chord bounded)
  Seam seam{};                ///< the single traced seam, resampled onto the azimuth grid

  /// The cone wall ring at slant station v (N azimuth samples on the seam grid) — the disc rim.
  std::vector<math::Point3> coneRing(double v) const {
    std::vector<math::Point3> r(N);
    for (int i = 0; i < N; ++i) r[i] = cone->point(kSsiTwoPi * i / N, v);
    return r;
  }
  /// The cone axis point at AXIAL station s (disc centre): origin + s·axis. Matches the AXIAL wall
  /// parametrization used by appendConeWallOuterZone (CurvedSolid.vLo/vHi are axial stations).
  math::Point3 coneAxisPt(double s) const {
    return math::Point3{cone->frame.origin.x + cone->frame.z.vec().x * s,
                        cone->frame.origin.y + cone->frame.z.vec().y * s,
                        cone->frame.origin.z + cone->frame.z.vec().z * s};
  }
  /// The cylinder wall ring at frame-v station v (N azimuth samples on the seam grid).
  std::vector<math::Point3> cylRing(double v) const {
    std::vector<math::Point3> r(N);
    for (int i = 0; i < N; ++i) r[i] = cyl->point(kSsiTwoPi * i / N, v);
    return r;
  }
};

// The cone-outward reference at a cone (u,v): radial·cosα − axial·sinα (the true outward wall
// normal of a cone that WIDENS toward +axis). Correct regardless of station (unlike a plain
// axis-radial reference, which ignores the slant).
math::Vec3 coneOutwardAt(const CurvedSolid& cone, double u) {
  const math::Vec3 X = cone.frame.x.vec(), Y = cone.frame.y.vec(), Z = cone.frame.z.vec();
  const double ca = std::cos(cone.semiAngle), sa = std::sin(cone.semiAngle);
  const math::Vec3 radial{X.x * std::cos(u) + Y.x * std::sin(u),
                          X.y * std::cos(u) + Y.y * std::sin(u),
                          X.z * std::cos(u) + Y.z * std::sin(u)};
  return math::Vec3{radial.x * ca - Z.x * sa, radial.y * ca - Z.y * sa, radial.z * ca - Z.z * sa};
}

// A cone-surface CAP bounded by one traced seam loop: a radial fan from the cap's cone-surface
// CENTRE (evaluated ON the cone at the mean seam (u,v)) through `rings` concentric rings out to the
// exact traced seam nodes. Every interior node sits ON the cone (lerp the (u,v) centre→boundary,
// evaluate); the OUTER ring is the shared pooled seam nodes so cap↔band welds. Facets are oriented
// by the TRUE cone-outward normal at the cap centre. `coneUv` is the seam's cone (u,v) track (uvA
// or uvB per operand order). Mirrors appendTransTorusCap for the ruled cone surface.
void appendTransConeCap(const CurvedSolid& cone, const Seam& seam,
                        const std::vector<std::pair<double, double>>& coneUv, int rings,
                        VertexPool& pool, std::vector<topo::Shape>& faces, bool reversed = false) {
  const int n = static_cast<int>(seam.pts.size());
  if (n < 3 || rings < 1 || static_cast<int>(coneUv.size()) != n) return;
  const double u0 = coneUv.front().first;
  double uSum = 0.0, vSum = 0.0;
  for (const auto& p : coneUv) { uSum += nearU(u0, p.first); vSum += p.second; }
  const double uc = uSum / n, vc = vSum / n;
  auto ringPt = [&](int r, int k) -> math::Point3 {
    if (r == rings) return seam.pts[k];  // outer ring = exact traced seam node
    const double t = static_cast<double>(r) / rings;
    const double u = uc + (nearU(u0, coneUv[k].first) - uc) * t;
    const double v = vc + (coneUv[k].second - vc) * t;
    return cone.point(u, v);
  };
  const math::Point3 centre = cone.point(uc, vc);
  const math::Vec3 co = coneOutwardAt(cone, uc);
  const math::Vec3 outRef = reversed ? math::Vec3{-co.x, -co.y, -co.z} : co;
  auto tri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    pushPlanarTri(a, b, c, outRef, pool, faces);
  };
  for (int k = 0; k < n; ++k) tri(centre, ringPt(1, k), ringPt(1, (k + 1) % n));
  for (int r = 2; r <= rings; ++r)
    for (int k = 0; k < n; ++k) {
      const int kn = (k + 1) % n;
      tri(ringPt(r - 1, k), ringPt(r, k), ringPt(r, kn));
      tri(ringPt(r - 1, k), ringPt(r, kn), ringPt(r - 1, kn));
    }
}

// Recognise the transversal (offset, axis-parallel) cone∩cylinder single-crossing pose from the
// one traced seam. Declines (→ ok=false) for the coaxial pose (offset ≤ tol, owned by S5-e), a skew
// (non-parallel) axis, ≠1 closed seam, or a pose whose two end discs are not (outside / inside) the
// cone respectively — the clean single-crossing gate. Mirrors the other transversal setups: a
// linear pose-recognition + resample + survival gate.
TransConeCylSetup transConeCylSetup(const CurvedSolid& A, const CurvedSolid& B,
                                    const std::vector<Seam>& seams) {
  TransConeCylSetup st;
  if (seams.size() != 1) return st;                       // single wall crossing → exactly one loop
  if (!seams[0].closed || seams[0].pts.size() < 8) return st;

  const CurvedSolid* conePtr = nullptr;
  const CurvedSolid* cylPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Cone) conePtr = s;
    else if (s->kind == CurvedKind::Cylinder) cylPtr = s;
  }
  if (!conePtr || !cylPtr) return st;
  const CurvedSolid& cone = *conePtr;
  const CurvedSolid& cyl = *cylPtr;
  const bool coneIsA = (&A == conePtr);
  const bool cylIsB = (&B == cylPtr);

  const double tanA = std::tan(cone.semiAngle);
  if (std::fabs(tanA) < 1e-6) return st;                  // near-cylindrical → S5-a territory
  const math::Vec3 zt = cone.frame.z.vec();
  const math::Vec3 zc = cyl.frame.z.vec();
  // Cylinder axis must be PARALLEL to the cone axis (axis-parallel offset pose). A skew /
  // perpendicular cylinder is a different, harder locus → decline → OCCT.
  if (math::norm(math::cross(zt, zc)) > 1e-6) return st;
  const math::Point3 O = cyl.frame.origin;
  const math::Vec3 d{O.x - cone.frame.origin.x, O.y - cone.frame.origin.y, O.z - cone.frame.origin.z};
  const double offset = math::norm(d - zt * math::dot(d, zt));  // perpendicular cyl-axis offset
  if (offset <= 1e-4) return st;                          // coaxial → S5-e owns it → decline
  const double Rc = cyl.radius;
  if (!(Rc > 1e-9)) return st;

  const double chord = std::sqrt(std::max(8.0 * kCapSagitta * Rc, 1e-12));
  const int N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * Rc / chord)), 24, 200);
  st.cone = conePtr; st.cyl = cylPtr; st.coneIsA = coneIsA; st.cylIsB = cylIsB;
  st.O = O; st.zc = zc; st.Rc = Rc; st.offset = offset; st.N = N;
  st.seam = resampleByAzimuth(seams[0], cylIsB, N);

  // The cylinder AXIS point at frame-v (O + v·zc) — the disc CENTRE (cyl.point(u,v) is a WALL point).
  auto axisAt = [&](double v) {
    return math::Point3{O.x + zc.x * v, O.y + zc.y * v, O.z + zc.z * v};
  };
  // CLEAN SINGLE-CROSSING gate (the honest inside-the-other test, never faked): the cylinder's two
  // end discs bracket the crossing — one end disc must be fully OUTSIDE the cone and the other fully
  // INSIDE. We test each end disc's axis centre + full rim against the cone. `vInside` is the
  // fully-inside end's frame-v.
  auto endDiscInside = [&](double v) -> int {
    const int cc = classifyPoint(cone, axisAt(v), kSsiTol);
    bool allIn = (cc == 1), allOut = (cc == -1);
    for (int k = 0; k < 16; ++k) {
      const double u = kSsiTwoPi * k / 16.0;
      const int r = classifyPoint(cone, cyl.point(u, v), kSsiTol);
      allIn = allIn && (r == 1);
      allOut = allOut && (r == -1);
    }
    return allIn ? 1 : (allOut ? -1 : 0);
  };
  const int loEnd = endDiscInside(cyl.vLo);
  const int hiEnd = endDiscInside(cyl.vHi);
  if (loEnd == -1 && hiEnd == 1) { st.vInside = cyl.vHi; st.vOutside = cyl.vLo; }
  else if (loEnd == 1 && hiEnd == -1) { st.vInside = cyl.vLo; st.vOutside = cyl.vHi; }
  else return st;                                          // not a clean single crossing → decline
  st.coneLo = std::min(cone.vLo, cone.vHi);
  st.coneHi = std::max(cone.vLo, cone.vHi);

  // The cylinder band midpoint (on the AXIS, between the seam mean and the inside end) must be
  // INSIDE the cone (the band is a COMMON boundary).
  double seamMeanV = 0.0;
  for (const auto& p : st.seam.pts)
    seamMeanV += math::dot(math::Vec3{p.x - O.x, p.y - O.y, p.z - O.z}, zc);
  seamMeanV /= static_cast<double>(st.seam.pts.size());
  const double vMid = 0.5 * (seamMeanV + st.vInside);
  if (classifyPoint(cone, axisAt(vMid), kSsiTol) != 1) return st;

  st.ok = true;
  return st;
}

// Ring count for the transversal cone cap: the cap's angular span (max seam-node distance from the
// cap centre) × sqrt(1/(2·kCapSagitta)), bounded [4,48].
int transConeCylCapRings(const TransConeCylSetup& s) {
  const auto& uv = s.coneIsA ? s.seam.uvA : s.seam.uvB;
  const double u0 = uv.front().first;
  double uSum = 0.0, vSum = 0.0;
  for (const auto& p : uv) { uSum += nearU(u0, p.first); vSum += p.second; }
  const math::Point3 centre = s.cone->point(uSum / uv.size(), vSum / uv.size());
  double span = 0.0;
  for (const auto& p : s.seam.pts)
    span = std::max(span, math::norm(math::Vec3{p.x - centre.x, p.y - centre.y, p.z - centre.z}));
  return std::clamp(
      static_cast<int>(std::ceil(std::max(span, 1e-6) * std::sqrt(1.0 / (2.0 * kCapSagitta)))), 4,
      48);
}

// buildTransConeCylCommon(A,B) = COMMON of the TRANSVERSAL (offset) cone∩cylinder single-crossing
// pose: the cone-wall cap (bounded by the traced seam) + the cylinder wall band (seam → inside-end
// rim) + the cylinder inside-end disc. All fragments share their boundary rings through one
// VertexPool → watertight. The first transversal cone∩cyl slice.
topo::Shape buildTransConeCylCommon(const CurvedSolid& A, const CurvedSolid& B,
                                    const std::vector<Seam>& seams) {
  const TransConeCylSetup s = transConeCylSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // The inside-end rim: a full azimuth ring on the cylinder at vInside, on the SAME grid as the
  // seam (u = 2πk/N) so the band welds ring-to-ring.
  std::vector<math::Point3> insideRim(s.N);
  for (int k = 0; k < s.N; ++k) insideRim[k] = s.cyl->point(kSsiTwoPi * k / s.N, s.vInside);
  const auto& coneUv = s.coneIsA ? s.seam.uvA : s.seam.uvB;
  // Cone-wall cap bounded by the seam (the cone sheet inside the cylinder).
  appendTransConeCap(*s.cone, s.seam, coneUv, transConeCylCapRings(s), pool, faces);
  // Cylinder wall band from the seam ring to the inside-end rim.
  appendRevolvedBand(s.seam.pts, insideRim, s.O, s.zc, pool, faces, 1.0);
  // Cylinder inside-end disc (full fan on the cyl axis). The inside end is either vHi (outward
  // along +zc) or vLo (outward along −zc).
  const bool endIsHi = (s.vInside > 0.5 * (s.cyl->vLo + s.cyl->vHi));
  const math::Vec3 discOut = endIsHi ? s.zc : math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z};
  appendDiskCap(*s.cyl, s.vInside, insideRim, discOut, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTransConeCylCut / buildTransConeCylFuse — the transversal CUT / FUSE of the offset
// cone∩cylinder. S5-s is a SINGLE-SEAM pose (the parallel-offset cylinder crosses the monotone
// cone wall exactly once), so — unlike the two-loop S5-k/S5-q — its CUT/FUSE need a SINGLE-seam
// outer weld. Two shapes arise, and BOTH now land:
//   * the CYLINDER outer stub (the cyl part OUTSIDE the cone) is a clean seam-driven solid — no
//     holed surface: cyl narrow-end disc + cyl wall band (narrow rim → seam) + the COMMON cone cap
//     REVERSED (the dimple). This is buildCut(cyl − cone).
//   * the CONE with a cylindrical bite (cone − cyl, and the FUSE envelope) needs the FULL cone wall
//     MINUS the single seam cap patch — a HOLED cone surface — welded to the cone end discs + the
//     reversed cyl band. Built by `appendConeWallOuterZone` (the SAME grid + loop-zipper scheme as
//     the S5-p torus tube outer zone, on the cone wall with ONE localized hole).
// Every fragment is seam-/rim-driven and on-surface; any pose that cannot localize the seam hole
// inside a clean grid rectangle HONEST-DECLINES → OCCT. Nothing is faked.

// Winding-align + rotate `innerIn` to `outer` (about the shared centroid, using `outRef` as the
// orientation normal) and zip: advance whichever loop is behind in fractional arc, one triangle
// per step, oriented by `outRef` — a closed watertight strip sharing every node of both loops.
void zipToSeam(const std::vector<math::Point3>& outer, const std::vector<math::Point3>& innerIn,
               const math::Vec3& outRef, VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int Po = static_cast<int>(outer.size()), Pi = static_cast<int>(innerIn.size());
  if (Po < 3 || Pi < 3) return;
  std::vector<math::Point3> inner = innerIn;
  math::Point3 c{0, 0, 0};
  for (const auto& p : outer) { c.x += p.x; c.y += p.y; c.z += p.z; }
  c.x /= Po; c.y /= Po; c.z /= Po;
  auto turn = [&](const std::vector<math::Point3>& L) {
    double acc = 0.0; const int n = static_cast<int>(L.size());
    for (int i = 0; i < n; ++i) {
      const math::Vec3 a{L[i].x - c.x, L[i].y - c.y, L[i].z - c.z};
      const math::Vec3 b{L[(i + 1) % n].x - c.x, L[(i + 1) % n].y - c.y, L[(i + 1) % n].z - c.z};
      acc += math::dot(math::cross(a, b), outRef);
    }
    return acc;
  };
  if (turn(outer) * turn(inner) < 0.0) std::reverse(inner.begin(), inner.end());
  int best = 0; double bd = 1e300;
  for (int i = 0; i < Pi; ++i) {
    const math::Vec3 d{inner[i].x - outer[0].x, inner[i].y - outer[0].y, inner[i].z - outer[0].z};
    const double dd = math::dot(d, d);
    if (dd < bd) { bd = dd; best = i; }
  }
  std::rotate(inner.begin(), inner.begin() + best, inner.end());
  int io = 0, ii = 0;
  while (io < Po || ii < Pi) {
    const double fo = static_cast<double>(io) / Po, fi = static_cast<double>(ii) / Pi;
    if (io < Po && (ii >= Pi || fo <= fi)) {
      pushPlanarTri(outer[io % Po], outer[(io + 1) % Po], inner[ii % Pi], outRef, pool, faces);
      ++io;
    } else {
      pushPlanarTri(outer[io % Po], inner[(ii + 1) % Pi], inner[ii % Pi], outRef, pool, faces);
      ++ii;
    }
  }
}

// Point-in-loop test in a param (u,v) plane; `uv` unwrapped about its own front, query shifted to
// the same u-branch. Bbox-rejects a query off the loop's branch (returns false → outside).
bool pointInUvLoop(double qu, double qv, const std::vector<std::pair<double, double>>& uv) {
  const int n = static_cast<int>(uv.size());
  if (n < 3) return false;
  const double u0 = uv.front().first;
  std::vector<double> lu(n), lv(n);
  double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
  for (int i = 0; i < n; ++i) {
    lu[i] = nearU(u0, uv[i].first); lv[i] = uv[i].second;
    umin = std::min(umin, lu[i]); umax = std::max(umax, lu[i]);
    vmin = std::min(vmin, lv[i]); vmax = std::max(vmax, lv[i]);
  }
  const double u = nearU(u0, qu);
  if (u < umin || u > umax || qv < vmin || qv > vmax) return false;
  bool in = false;
  for (int i = 0, j = n - 1; i < n; j = i++)
    if (((lv[i] > qv) != (lv[j] > qv)) &&
        (u < (lu[j] - lu[i]) * (qv - lv[i]) / (lv[j] - lv[i] + 1e-300) + lu[i]))
      in = !in;
  return in;
}

// The cone WALL OUTER zone = the full cone wall grid (u∈[0,2π), v∈[coneLo,coneHi]) MINUS the cells
// whose centre lies inside the single seam loop (a TIGHT jagged hole hugging the seam within one
// cell), the jagged hole boundary chained into an ordered loop and ZIPPED to the exact seam. The
// grid v-direction is NOT periodic: the coneLo / coneHi wall edges are the cone's rim rings the end
// discs close. Returns false (appending nothing) if the seam is not localized clear of both rims or
// its hole boundary is not a single clean loop (→ caller HONEST-DECLINES). `capRingLo/Hi` receive
// the rim rings so the caller welds the end discs to the SAME nodes.
bool appendConeWallOuterZone(const CurvedSolid& cone, double coneLo, double coneHi, const Seam& seam,
                             bool coneIsA, VertexPool& pool, std::vector<topo::Shape>& faces,
                             std::vector<math::Point3>& capRingLo,
                             std::vector<math::Point3>& capRingHi) {
  const auto& uv = coneIsA ? seam.uvA : seam.uvB;
  if (uv.size() < 3) return false;
  const double rHi = cone.radius + coneHi * std::sin(cone.semiAngle);  // widest wall radius
  const double kFacetSag = 0.0005;  // finer than the coaxial 0.004 — the cone wall carries the bulk
  const double chordU = std::sqrt(std::max(8.0 * kFacetSag * std::max(rHi, 1e-6), 1e-12));
  const int Nu = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rHi / chordU)), 48, 300);
  const int Nv = std::clamp(static_cast<int>(std::ceil((coneHi - coneLo) / chordU)), 16, 200);
  // The cone wall grid + end discs are built in AXIAL coordinates (station s along zc from the cone
  // frame origin), matching the codebase convention that CurvedSolid.vLo/vHi are AXIAL stations
  // (see S5-q coneRingAxial): a wall ring at axial s has radius r(s)=radius+s·tanα, placed at
  // origin+zc·s, so the disc at s (centre origin+zc·s, same radius) welds flush — unlike
  // cone.point(u,v) whose v is the SLANT parameter (v·cosα short of the axial station). `coneLo/Hi`
  // are those axial stations. The seam's cone (u,v) v is SLANT, so convert to axial (y=v·cosα) for
  // the hole cell test.
  const math::Vec3 CX = cone.frame.x.vec(), CY = cone.frame.y.vec(), CZ = cone.frame.z.vec();
  const double tanA = std::tan(cone.semiAngle), cosA = std::cos(cone.semiAngle);
  auto uAt = [&](int i) { return kSsiTwoPi * (((i % Nu) + Nu) % Nu) / Nu; };
  auto vAt = [&](int j) { return coneLo + (coneHi - coneLo) * j / Nv; };  // AXIAL station s
  auto wallPt = [&](double u, double s) {
    const double r = cone.radius + s * tanA;
    return math::Point3{cone.frame.origin.x + CX.x * r * std::cos(u) + CY.x * r * std::sin(u) + CZ.x * s,
                        cone.frame.origin.y + CX.y * r * std::cos(u) + CY.y * r * std::sin(u) + CZ.y * s,
                        cone.frame.origin.z + CX.z * r * std::cos(u) + CY.z * r * std::sin(u) + CZ.z * s};
  };
  // Seam (u,v) track in AXIAL coordinates (its v is SLANT → axial = v·cosα) for the hole cell test.
  std::vector<std::pair<double, double>> uvAxial;
  uvAxial.reserve(uv.size());
  for (const auto& p : uv) uvAxial.emplace_back(p.first, p.second * cosA);
  // A cell (i,j) is a HOLE cell iff its (u, axial-s) centre lies inside the seam loop. The seam must
  // sit clear of both rims (no hole cell in row 0 or row Nv-1) and clear of the u-wrap.
  auto cellCentreInSeam = [&](int i, int j) {
    return pointInUvLoop(uAt(i) + kSsiPi / Nu, vAt(j) + (coneHi - coneLo) / (2.0 * Nv), uvAxial);
  };
  std::vector<std::pair<int, int>> holeCells;
  for (int i = 0; i < Nu; ++i)
    for (int j = 0; j < Nv; ++j)
      if (cellCentreInSeam(i, j)) {
        if (j == 0 || j == Nv - 1) return false;   // seam touches a rim → decline (rim weld out of scope)
        holeCells.emplace_back(i, j);
      }
  if (holeCells.size() < 3) return false;           // seam not resolved by the grid → decline
  auto isHole = [&](int i, int j) {
    const int iu = ((i % Nu) + Nu) % Nu;
    if (j < 0 || j >= Nv) return false;
    for (const auto& c : holeCells)
      if (c.first == iu && c.second == j) return true;
    return false;
  };
  // Emit every KEPT wall quad (grid corner at (i,j)→(i+1,j+1)); the four grid nodes are shared pool
  // vertices so kept quads weld to each other and to the hole-boundary ring.
  for (int i = 0; i < Nu; ++i)
    for (int j = 0; j < Nv; ++j) {
      if (isHole(i, j)) continue;
      const math::Point3 a = wallPt(uAt(i), vAt(j)), b = wallPt(uAt(i + 1), vAt(j));
      const math::Point3 c = wallPt(uAt(i + 1), vAt(j + 1)), dd = wallPt(uAt(i), vAt(j + 1));
      pushPlanarTri(a, b, c, coneOutwardAt(cone, uAt(i)), pool, faces);
      pushPlanarTri(a, c, dd, coneOutwardAt(cone, uAt(i)), pool, faces);
    }
  // Chain the hole-boundary GRID EDGES into an ordered loop of grid NODES. A boundary edge of the
  // hole is a grid edge with a hole cell on ONE side and a kept cell on the other. Collect the
  // directed boundary edges (node→node) so the hole is on the LEFT (kept on the right), then walk
  // them tip-to-tail into one loop. Node key = (i in [0,Nu), j in [0,Nv]).
  auto nodeKey = [&](int i, int j) { return (((i % Nu) + Nu) % Nu) * (Nv + 1) + j; };
  std::unordered_map<int, int> nextOf;   // directed edge node→node (keyed by from-node)
  auto addEdge = [&](int fi, int fj, int ti, int tj) {
    nextOf[nodeKey(fi, fj)] = nodeKey(ti, tj);
  };
  for (const auto& c : holeCells) {
    const int i = c.first, j = c.second;
    // For each of the 4 sides of the hole cell, if the neighbour cell is KEPT, add the side as a
    // directed boundary edge wound so the hole is enclosed CCW in (i,j) space.
    if (!isHole(i, j - 1)) addEdge(i + 1, j, i, j);       // bottom side (neighbour below kept)
    if (!isHole(i + 1, j)) addEdge(i + 1, j + 1, i + 1, j); // right side
    if (!isHole(i, j + 1)) addEdge(i, j + 1, i + 1, j + 1); // top side
    if (!isHole(i - 1, j)) addEdge(i, j, i, j + 1);         // left side
  }
  if (nextOf.empty()) return false;
  // Walk the chain into an ordered node loop; must return to start and consume every edge once.
  std::vector<int> loopKeys;
  const int startK = nextOf.begin()->first;
  int cur = startK;
  for (size_t guard = 0; guard <= nextOf.size(); ++guard) {
    loopKeys.push_back(cur);
    auto it = nextOf.find(cur);
    if (it == nextOf.end()) return false;                 // broken chain → decline
    cur = it->second;
    if (cur == startK) break;
  }
  if (loopKeys.size() != nextOf.size()) return false;      // not a SINGLE simple loop → decline
  // Materialize the ring on the cone (each node key → (i,j) → cone.point).
  std::vector<math::Point3> ring;
  ring.reserve(loopKeys.size());
  for (int k : loopKeys) {
    const int i = k / (Nv + 1), j = k % (Nv + 1);
    ring.push_back(wallPt(uAt(i), vAt(j)));
  }
  const math::Vec3 outRef = coneOutwardAt(cone, uAt(holeCells.front().first));
  zipToSeam(ring, seam.pts, outRef, pool, faces);
  capRingLo.clear(); capRingHi.clear();
  for (int i = 0; i < Nu; ++i) capRingLo.push_back(wallPt(uAt(i), vAt(0)));
  for (int i = 0; i < Nu; ++i) capRingHi.push_back(wallPt(uAt(i), vAt(Nv)));
  return true;
}

// buildTransConeCylCut(A,B) = A − B:
//   * CYLINDER − CONE (A = cyl): the cyl part OUTSIDE the cone — a clean seam-driven solid (cyl
//     narrow-end disc + cyl wall band narrow-rim→seam + the COMMON cone cap REVERSED). No hole.
//   * CONE − CYLINDER (A = cone): the cone with a cylindrical bite — the HOLED cone wall (full wall
//     minus the seam cap) + cone end discs + reversed cyl band + cyl inside-end disc reversed.
topo::Shape buildTransConeCylCut(const CurvedSolid& A, const CurvedSolid& B,
                                 const std::vector<Seam>& seams) {
  const TransConeCylSetup s = transConeCylSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const auto& coneUv = s.coneIsA ? s.seam.uvA : s.seam.uvB;
  if (&A == s.cyl) {
    // CYLINDER − CONE: the cylinder stub OUTSIDE the cone.
    const std::vector<math::Point3> rimOut = s.cylRing(s.vOutside);
    const bool outIsHi = (s.vOutside > 0.5 * (s.cyl->vLo + s.cyl->vHi));
    appendDiskCap(*s.cyl, s.vOutside, rimOut, outIsHi ? s.zc : math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z},
                  pool, faces);
    appendRevolvedBand(rimOut, s.seam.pts, s.O, s.zc, pool, faces, 1.0);       // cyl wall (outside)
    // The cone cap (the cone-wall patch bounding the stub top), REVERSED — its outward normal
    // points OUT of the stub (into the cone interior), opposite the COMMON cap's cone-outward.
    appendTransConeCap(*s.cone, s.seam, coneUv, transConeCylCapRings(s), pool, faces,
                       /*reversed=*/true);
    if (faces.size() < 4) return {};
  } else {
    // CONE − CYLINDER: the holed cone wall + cone end discs + reversed cyl band + reversed inside disc.
    std::vector<math::Point3> capLo, capHi;
    if (!appendConeWallOuterZone(*s.cone, s.coneLo, s.coneHi, s.seam, s.coneIsA, pool, faces, capLo,
                                 capHi))
      return {};
    appendAxisDiscCap(s.coneAxisPt(s.coneLo), capLo, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
    appendAxisDiscCap(s.coneAxisPt(s.coneHi), capHi, s.zc, pool, faces);
    // The removed cylinder's inner surface: reversed cyl band (seam→inside rim) + reversed inside disc.
    std::vector<math::Point3> insideRim = s.cylRing(s.vInside);
    appendRevolvedBand(s.seam.pts, insideRim, s.O, s.zc, pool, faces, /*outwardSign=*/-1.0);
    const bool endIsHi = (s.vInside > 0.5 * (s.cyl->vLo + s.cyl->vHi));
    appendDiskCap(*s.cyl, s.vInside, insideRim,
                  endIsHi ? math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z} : s.zc, pool, faces);
    if (faces.size() < 6) return {};
  }
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTransConeCylFuse(A,B) = A ∪ B: the HOLED cone wall (full wall minus the seam cap) + cone end
// discs + the cylinder stub OUTSIDE the cone (narrow-end disc + cyl wall band narrow-rim→seam). The
// cone cap patch and the cyl band inside the cone are interior → dropped. V = V(A)+V(B)−V(∩).
topo::Shape buildTransConeCylFuse(const CurvedSolid& A, const CurvedSolid& B,
                                  const std::vector<Seam>& seams) {
  const TransConeCylSetup s = transConeCylSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  std::vector<math::Point3> capLo, capHi;
  if (!appendConeWallOuterZone(*s.cone, s.coneLo, s.coneHi, s.seam, s.coneIsA, pool, faces, capLo,
                               capHi))
    return {};
  appendAxisDiscCap(s.coneAxisPt(s.coneLo), capLo, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendAxisDiscCap(s.coneAxisPt(s.coneHi), capHi, s.zc, pool, faces);
  // Cylinder stub outside the cone: narrow-end disc + cyl wall band (narrow rim → seam).
  const std::vector<math::Point3> rimOut = s.cylRing(s.vOutside);
  const bool outIsHi = (s.vOutside > 0.5 * (s.cyl->vLo + s.cyl->vHi));
  appendDiskCap(*s.cyl, s.vOutside, rimOut, outIsHi ? s.zc : math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z},
                pool, faces);
  appendRevolvedBand(rimOut, s.seam.pts, s.O, s.zc, pool, faces, 1.0);
  if (faces.size() < 6) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ═══ S5-l — COAXIAL TORUS ∩ CYLINDER (COMMON / FUSE / CUT) ══════════════════════
// The TORUS surface family opened. A ring torus (major R, minor r, axis = frame Z) and a
// coaxial cylinder (radius Rc, same axis) whose wall crosses the torus TUBE at TWO
// latitudes → TWO analytic circle seams. In the meridian (ρ,z) plane the tube is the disk
// of radius r centred at (R, 0); the cylinder is the vertical chord ρ = Rc. The chord cuts
// the tube iff |Rc − R| < r, giving cos v0 = (Rc − R)/r and the two seam circles at axial
// stations z = ±z0, z0 = √(r² − (Rc − R)²), both of radius Rc. Every boolean here is a
// SOLID OF REVOLUTION of the corresponding (ρ,z) region, welded from revolved tube-arc
// bands + the cylinder chord band + flat disc/annulus caps — the S5-e…j machinery reused.
//
// CLEANEST-ORACLE choice (SSI-ROADMAP §S5): all volumes are Pappus-exact closed forms.
//   V_torus       = 2π² R r²                              (Pappus, full tube)
//   V_common      = 2π·(R·A_seg + M)                      (revolve the ρ ≤ Rc segment)
//        where, with d = Rc − R and the disk radius r,
//        A_cap(ρ>Rc) = r²·acos(d/r) − d·√(r²−d²)         (the OUTER circular segment area)
//        A_seg(ρ≤Rc) = π r² − A_cap                       (the INNER segment area)
//        M           = −(2/3)(r² − d²)^{3/2}              (its first moment about ρ=R)
//   V_cut(T−C)    = V_torus − V_common
//   V_fuse(T∪C)   = V_torus + V_cyl − V_common
// The generic booleanResultVerified drives FUSE/CUT off the native COMMON as V(A∩B); the
// engine's ssiCurvedBooleanVerified S5-l arm additionally checks COMMON against the Pappus
// closed form directly.
struct TorusCylSetup {
  bool ok = false;
  const CurvedSolid* tor = nullptr;
  const CurvedSolid* cyl = nullptr;
  math::Point3 O;          ///< TORUS centre (on the shared axis) — the canonical origin
  math::Vec3 X, Y, zc;     ///< torus frame basis (zc = shared axis)
  double R = 0.0, r = 0.0;  ///< torus major / minor radius
  double Rc = 0.0;          ///< cylinder radius
  double v0 = 0.0;          ///< tube minor angle of the +z seam (cos v0 = (Rc−R)/r)
  double z0 = 0.0;          ///< +z seam axial station (= r·sin v0)
  double cylS0 = 0.0, cylS1 = 0.0;  ///< cylinder axial extent in the torus z-frame
  int N = 0;                ///< azimuth sample count (seam-chord bounded)
  int M = 0;                ///< tube-arc subdivision (minor-angle chord bounded)

  /// A closed ring of N azimuth samples at (radius ρ, axial station s) in the torus frame.
  std::vector<math::Point3> ring(double rho, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = rho * std::cos(u), cy = rho * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  /// (ρ, z) on the tube at minor angle v: ρ = R + r·cos v, z = r·sin v.
  std::pair<double, double> tube(double v) const {
    return {R + r * std::cos(v), r * std::sin(v)};
  }
  math::Point3 axisPt(double s) const {
    return math::Point3{O.x + zc.x * s, O.y + zc.y * s, O.z + zc.z * s};
  }
};

TorusCylSetup torusCylSetup(const CurvedSolid& A, const CurvedSolid& B,
                            const std::vector<Seam>& seams) {
  TorusCylSetup st;
  if (seams.empty()) return st;  // need ≥1 traced seam to cross-check

  const CurvedSolid* torPtr = nullptr;
  const CurvedSolid* cylPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Torus) torPtr = s;
    else if (s->kind == CurvedKind::Cylinder) cylPtr = s;
  }
  if (!torPtr || !cylPtr) return st;
  const CurvedSolid& tor = *torPtr;
  const CurvedSolid& cyl = *cylPtr;

  const math::Vec3 zc = tor.frame.z.vec();
  const math::Point3 O = tor.frame.origin;
  // Cylinder must be COAXIAL: parallel axis + colinear origin.
  if (math::norm(math::cross(zc, cyl.frame.z.vec())) > 1e-6) return st;
  const math::Vec3 d{cyl.frame.origin.x - O.x, cyl.frame.origin.y - O.y, cyl.frame.origin.z - O.z};
  if (math::norm(d - zc * math::dot(d, zc)) > 1e-6) return st;  // cyl axis not colinear → OCCT

  const double R = tor.radius, r = tor.minorRadius, Rc = cyl.radius;
  if (!(r > 1e-9) || !(R > r + 1e-9) || !(Rc > 1e-9)) return st;
  // The cylinder wall must cross the TUBE at two latitudes: |Rc − R| < r strictly (a proper
  // two-circle poke-through). |Rc − R| ≥ r → tangent / clear / inside the hole → OCCT.
  const double dChord = Rc - R;
  if (!(std::fabs(dChord) < r - 1e-6)) return st;

  const double cosv = dChord / r;
  const double v0 = std::acos(std::clamp(cosv, -1.0, 1.0));  // ∈ (0, π)
  const double z0 = r * std::sin(v0);                        // = √(r² − dChord²)
  if (!(z0 > 1e-6)) return st;

  // Cylinder axial extent in the torus z-frame; the tube spans z ∈ [−r, r], both seams at
  // ±z0 must lie strictly inside the cylinder so the two circles are genuine (the cylinder
  // fully spans the tube axially).
  const double base = math::dot(d, zc);
  const double sgn = math::dot(cyl.frame.z.vec(), zc) >= 0.0 ? 1.0 : -1.0;
  double cylS0 = base + sgn * cyl.vLo, cylS1 = base + sgn * cyl.vHi;
  if (cylS0 > cylS1) std::swap(cylS0, cylS1);
  if (!(cylS0 < -z0 - 1e-6) || !(cylS1 > z0 + 1e-6)) return st;  // seams not both interior → OCCT

  // Cross-check EVERY traced seam against ONE of the two analytic circles (station ±z0,
  // radius Rc) — never trust a missing / mis-placed loop.
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
    const bool matchLo = std::fabs(sTr + z0) < 1e-3 && std::fabs(rhoTr - Rc) < 1e-3;
    const bool matchHi = std::fabs(sTr - z0) < 1e-3 && std::fabs(rhoTr - Rc) < 1e-3;
    if (!matchLo && !matchHi) return st;  // traced seam matches neither analytic circle → OCCT
  }

  // Azimuth (N) + tube-arc (M) resolution, chord-bounded so a planar facet's sagitta off
  // the true surface stays under the mesh deflection (0.005). Because the torus wall is a
  // FULL N×M facet grid (not a few bands like the cyl/sphere families), we bound by the
  // tessellation deflection rather than the far-finer kCapSagitta — the facet chord error
  // then rides at the mesh deflection, well inside the 1% curved-parity bar, while keeping
  // the total facet count (≈ N·M) tractable for the mesher.
  const double kFacetSag = 0.004;
  const double chordA = std::sqrt(std::max(8.0 * kFacetSag * Rc, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * Rc / chordA)), 32, 200);
  const double chordM = std::sqrt(std::max(8.0 * kFacetSag * r, 1e-12));
  st.M = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * r / chordM)), 24, 160);
  st.tor = torPtr;
  st.cyl = cylPtr;
  st.O = O;
  st.X = tor.frame.x.vec();
  st.Y = tor.frame.y.vec();
  st.zc = zc;
  st.R = R;
  st.r = r;
  st.Rc = Rc;
  st.v0 = v0;
  st.z0 = z0;
  st.cylS0 = cylS0;
  st.cylS1 = cylS1;
  st.ok = true;
  return st;
}

// The world point on the TUBE-CENTRE circle at azimuth index i (radius R about the axis) —
// the point the tube's outward normal radiates FROM. Unlike the axis-radial reference, this
// is correct on the tube's INNER half (v near π), where the outward tube normal points TOWARD
// the axis — so a plain appendRevolvedBand (axis-radial orientation) would invert it.
math::Point3 tubeCentre(const TorusCylSetup& s, int i) {
  const double u = kSsiTwoPi * i / s.N;
  const double cx = s.R * std::cos(u), cy = s.R * std::sin(u);
  return math::Point3{s.O.x + s.X.x * cx + s.Y.x * cy, s.O.y + s.X.y * cx + s.Y.y * cy,
                      s.O.z + s.X.z * cx + s.Y.z * cy};
}

// Append the revolved TUBE ARC between minor angles vA→vB (M subdivisions). Each facet is
// oriented by the TRUE tube-outward reference (radiating from the tube-centre circle),
// scaled by `outwardSign` (+1 outward tube wall / −1 an inward reversed bore). Rings at vA
// and vB reuse the shared pool so they weld to the neighbouring seam/cap rings.
void appendTubeArc(const TorusCylSetup& s, double vA, double vB, double outwardSign,
                   VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int m = s.M;
  std::vector<math::Point3> prev;
  for (int k = 0; k <= m; ++k) {
    const double v = vA + (vB - vA) * (static_cast<double>(k) / m);
    const auto [rho, z] = s.tube(v);
    std::vector<math::Point3> cur = s.ring(rho, z);
    if (k > 0) {
      const int n = s.N;
      for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        // Outward tube reference at the quad's centre = midpoint minus the tube-centre point.
        const math::Point3 tc = tubeCentre(s, i);
        const math::Point3 mid{(prev[i].x + prev[j].x + cur[j].x + cur[i].x) / 4,
                               (prev[i].y + prev[j].y + cur[j].y + cur[i].y) / 4,
                               (prev[i].z + prev[j].z + cur[j].z + cur[i].z) / 4};
        const math::Vec3 out{(mid.x - tc.x) * outwardSign, (mid.y - tc.y) * outwardSign,
                             (mid.z - tc.z) * outwardSign};
        pushPlanarTri(prev[i], prev[j], cur[j], out, pool, faces);
        pushPlanarTri(prev[i], cur[j], cur[i], out, pool, faces);
      }
    }
    prev = std::move(cur);
  }
}

// buildTorusCylCommon(A,B) = COMMON of the coaxial torus∩cylinder: the ρ ≤ Rc solid of
// revolution — the INNER tube arc (v ∈ [v0, 2π−v0], through the inner equator ρ = R−r) +
// the cylinder chord band ρ = Rc between the two seam rings (z ∈ [−z0, z0]). Watertight,
// closed surface of revolution.
topo::Shape buildTorusCylCommon(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  const TorusCylSetup s = torusCylSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // Inner tube arc from the +z seam (v0) THROUGH the inner equator (π) to the −z seam
  // (2π − v0); its endpoints are the two seam rings at ρ = Rc, z = ±z0.
  appendTubeArc(s, s.v0, kSsiTwoPi - s.v0, /*outwardSign=*/1.0, pool, faces);
  // Cylinder chord band ρ = Rc from the −z seam ring up to the +z seam ring (its outward
  // normal points radially OUT of the material, i.e. +radial).
  const std::vector<math::Point3> ringLo = s.ring(s.Rc, -s.z0);
  const std::vector<math::Point3> ringHi = s.ring(s.Rc, s.z0);
  appendRevolvedBand(ringLo, ringHi, s.O, s.zc, pool, faces, 1.0);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusCylCut(A,B) = A − B with TORUS the minuend: the ρ > Rc solid of revolution —
// the OUTER tube arc (v ∈ [−v0, v0], through the outer equator ρ = R+r) + the cylinder
// chord band REVERSED (inward normal, bounding the carved bore). A SHRINK; a single closed
// ring-of-revolution component. The cylinder-minuend (cyl − torus) LANDS via buildCylTorusCut (the
// grooved-cylinder solid) — order-sensitive but tractable.
topo::Shape buildTorusCylCut(const CurvedSolid& A, const CurvedSolid& B,
                             const std::vector<Seam>& seams) {
  const TorusCylSetup s = torusCylSetup(A, B, seams);
  if (!s.ok) return {};
  if (A.kind != CurvedKind::Torus) return {};  // order-sensitive: torus must be the minuend
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // Outer tube arc from the −z seam (−v0) THROUGH the outer equator (0) to the +z seam (v0).
  appendTubeArc(s, -s.v0, s.v0, /*outwardSign=*/1.0, pool, faces);
  // Cylinder chord band REVERSED (inward): from the +z seam ring down to the −z seam ring,
  // outward normal pointing radially IN toward the axis (−radial).
  const std::vector<math::Point3> ringLo = s.ring(s.Rc, -s.z0);
  const std::vector<math::Point3> ringHi = s.ring(s.Rc, s.z0);
  appendRevolvedBand(ringLo, ringHi, s.O, s.zc, pool, faces, -1.0);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusCylFuse(A,B) = A ∪ B: the union (ρ,z) profile — the OUTER tube arc bulge
// (v ∈ [−v0, v0], ρ > Rc) + the cylinder wall ρ = Rc OUTSIDE the tube (z ∈ [cylS0, −z0] and
// [z0, cylS1]) + the two cylinder terminal disc caps. The cylinder fills the donut hole, so
// the union is simply connected (no inner hole). A GROW. V = V_torus + V_cyl − V_common.
topo::Shape buildTorusCylFuse(const CurvedSolid& A, const CurvedSolid& B,
                              const std::vector<Seam>& seams) {
  const TorusCylSetup s = torusCylSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ring0 = s.ring(s.Rc, s.cylS0);   // bottom cap rim
  const std::vector<math::Point3> ringLo = s.ring(s.Rc, -s.z0);    // −z seam
  const std::vector<math::Point3> ringHi = s.ring(s.Rc, s.z0);     // +z seam
  const std::vector<math::Point3> ring1 = s.ring(s.Rc, s.cylS1);   // top cap rim
  // Bottom disc cap (outward normal −z), cylinder wall up to the −z seam.
  appendAxisDiscCap(s.axisPt(s.cylS0), ring0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ring0, ringLo, s.O, s.zc, pool, faces, 1.0);
  // Outer tube-arc bulge (−v0 → 0 → v0), sharing the two seam rings.
  appendTubeArc(s, -s.v0, s.v0, /*outwardSign=*/1.0, pool, faces);
  // Cylinder wall above the +z seam, top disc cap (outward normal +z).
  appendRevolvedBand(ringHi, ring1, s.O, s.zc, pool, faces, 1.0);
  appendAxisDiscCap(s.axisPt(s.cylS1), ring1, s.zc, pool, faces);
  if (faces.size() < 6) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildCylTorusCut(A,B) = A − B with the CYLINDER the minuend (the ORDER-SENSITIVE reverse of
// buildTorusCylCut): the finite cylinder MINUS the part of the torus tube inside it — a solid
// cylinder with a concave toroidal GROOVE carved into its lateral wall over z ∈ [−z0, z0].
// It is a single closed solid of revolution, NOT the torus-minuend's outer ring; landed here as
// the tractable reverse (previously honest-declined). The (ρ,z) profile is the cylinder rectangle
// [0, Rc] × [cylS0, cylS1] minus the inner tube segment {(ρ−R)²+z² ≤ r², ρ ≤ Rc}. Its boundary:
//   * bottom disc cap (z = cylS0, normal −z) + top disc cap (z = cylS1, normal +z);
//   * cylinder wall ρ = Rc OUTSIDE the groove: z ∈ [cylS0, −z0] and [z0, cylS1] (outward +radial);
//   * the INNER tube arc (v ∈ [v0, 2π−v0], through the inner equator ρ = R−r) as the GROOVE wall,
//     REVERSED (outwardSign = −1): the remaining cylinder material sits on the axis side of the
//     arc, so its outward normal points AWAY from the axis, into the removed tube region.
// The two seam rings (ρ = Rc, z = ±z0) are shared so the groove welds to the wall stubs. Gated on
// the cylinder fully spanning the tube axially (the S5-l setup's cylS0 < −z0, cylS1 > z0 guarantee,
// so both wall stubs are non-degenerate). V = V_cyl − V_common. The engine's watertight + two-sided
// volume self-verify is the safety net; a pose that cannot weld robustly HONEST-DECLINES → OCCT.
topo::Shape buildCylTorusCut(const CurvedSolid& A, const CurvedSolid& B,
                             const std::vector<Seam>& seams) {
  const TorusCylSetup s = torusCylSetup(A, B, seams);
  if (!s.ok) return {};
  if (A.kind != CurvedKind::Cylinder) return {};  // order-sensitive: cylinder must be the minuend
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const std::vector<math::Point3> ring0 = s.ring(s.Rc, s.cylS0);   // bottom cap rim
  const std::vector<math::Point3> ringLo = s.ring(s.Rc, -s.z0);    // −z seam
  const std::vector<math::Point3> ringHi = s.ring(s.Rc, s.z0);     // +z seam
  const std::vector<math::Point3> ring1 = s.ring(s.Rc, s.cylS1);   // top cap rim
  // Bottom disc cap (normal −z), cylinder wall stub up to the −z seam (outward +radial).
  appendAxisDiscCap(s.axisPt(s.cylS0), ring0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ring0, ringLo, s.O, s.zc, pool, faces, 1.0);
  // The INNER tube arc as the concave groove, REVERSED (material on the axis side of the arc).
  // Walk from the −z seam (v = 2π − v0) through the inner equator (π) to the +z seam (v = v0).
  appendTubeArc(s, kSsiTwoPi - s.v0, s.v0, /*outwardSign=*/-1.0, pool, faces);
  // Cylinder wall stub above the +z seam, top disc cap (normal +z).
  appendRevolvedBand(ringHi, ring1, s.O, s.zc, pool, faces, 1.0);
  appendAxisDiscCap(s.axisPt(s.cylS1), ring1, s.zc, pool, faces);
  if (faces.size() < 6) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ═══ S5-r — COAXIAL TORUS ∩ HALF-SPACE (PLANE ⟂ AXIS) COMMON / FUSE / CUT ════════
// A ring torus (major R, minor r, axis = frame Z, centre O) cut by a PLANAR HALF-SPACE
// whose bounding plane is PERPENDICULAR to the torus axis (a horizontal slice at axial
// station z = h in the torus z-frame). The half-space keeps the torus material on ONE
// side of z = h. When the plane crosses the TUBE (|h| < r strictly) the section is an
// ANNULUS (a washer between ρ = R − c and ρ = R + c, c = √(r² − h²)), so the result is a
// SOLID OF REVOLUTION welded from a revolved tube arc + a flat annulus cap — the SAME
// S5-l machinery (appendTubeArc, ring, appendAnnulusCap). The plane-parallel-to-axis
// (Villarceau-like) orientation and every degenerate pose (plane tangent to the tube,
// plane clear of the torus, plane through the hole) HONEST-DECLINE → NULL → OCCT.
//
// CLEANEST-ORACLE (SSI-ROADMAP §S5): the plane-cut torus-segment volume is a Pappus-exact
// closed form; the host test cross-checks it against a 200³ numeric-integration oracle.
// Keep the material where z ≤ h (the far side is the complement). The horizontal cut is
// SYMMETRIC in ρ−R, so the segment's first moment about ρ = R VANISHES and Pappus collapses
// to 2π·R·(segment area):
//   V_torus       = 2π² R r²
//   A_cap(z>h)    = r²·acos(h/r) − h·√(r²−h²)          (the removed circular cap area)
//   A_low(z≤h)    = π r² − A_cap
//   V_common(z≤h) = 2π·R·A_low                         (Pappus about the torus axis)
// FUSE/CUT are the complement/whole relations the generic verifier already drives.
//
// The half-space operand is a PLANE-ONLY solid (a box/slab); it is NOT a CurvedSolid, so
// this family is dispatched by the dedicated tryTorusHalfspace entry (below ssi_boolean_solid)
// BEFORE the both-operands-curved gate. Nothing here is faked: a plane that is not axis-
// perpendicular, does not cut the tube, or leaves the result full/empty → NULL → OCCT.
struct TorusPlaneSetup {
  bool ok = false;
  const CurvedSolid* tor = nullptr;
  math::Point3 O;           ///< torus centre (on the axis)
  math::Vec3 X, Y, zc;      ///< torus frame basis (zc = axis)
  double R = 0.0, r = 0.0;  ///< torus major / minor radius
  double h = 0.0;           ///< cut plane axial station in the torus z-frame
  bool keepLow = true;      ///< true → keep z ≤ h (the material below the plane); else z ≥ h
  double v0 = 0.0;          ///< minor angle of the +ρ seam on the KEPT side (see builders)
  int N = 0;                ///< azimuth sample count
  int M = 0;                ///< tube-arc subdivision

  std::vector<math::Point3> ring(double rho, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = rho * std::cos(u), cy = rho * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  std::pair<double, double> tube(double v) const {
    return {R + r * std::cos(v), r * std::sin(v)};
  }
};

// The world point on the tube-CENTRE circle at azimuth index i (for the true tube-outward
// normal — mirrors S5-l tubeCentre so the inner-half arc is oriented correctly).
math::Point3 torusPlaneTubeCentre(const TorusPlaneSetup& s, int i) {
  const double u = kSsiTwoPi * i / s.N;
  const double cx = s.R * std::cos(u), cy = s.R * std::sin(u);
  return math::Point3{s.O.x + s.X.x * cx + s.Y.x * cy, s.O.y + s.X.y * cx + s.Y.y * cy,
                      s.O.z + s.X.z * cx + s.Y.z * cy};
}

// Append the revolved tube arc vA→vB (M subdivisions), oriented by the true tube-outward
// reference scaled by outwardSign. Identical machinery to S5-l appendTubeArc, keyed off
// TorusPlaneSetup.
void appendTorusPlaneTubeArc(const TorusPlaneSetup& s, double vA, double vB, double outwardSign,
                             VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int m = s.M;
  std::vector<math::Point3> prev;
  for (int k = 0; k <= m; ++k) {
    const double v = vA + (vB - vA) * (static_cast<double>(k) / m);
    const auto [rho, z] = s.tube(v);
    std::vector<math::Point3> cur = s.ring(rho, z);
    if (k > 0) {
      const int n = s.N;
      for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        const math::Point3 tc = torusPlaneTubeCentre(s, i);
        const math::Point3 mid{(prev[i].x + prev[j].x + cur[j].x + cur[i].x) / 4,
                               (prev[i].y + prev[j].y + cur[j].y + cur[i].y) / 4,
                               (prev[i].z + prev[j].z + cur[j].z + cur[i].z) / 4};
        const math::Vec3 out{(mid.x - tc.x) * outwardSign, (mid.y - tc.y) * outwardSign,
                             (mid.z - tc.z) * outwardSign};
        pushPlanarTri(prev[i], prev[j], cur[j], out, pool, faces);
        pushPlanarTri(prev[i], cur[j], cur[i], out, pool, faces);
      }
    }
    prev = std::move(cur);
  }
}

// A PLANAR half-space bounding one operand, recovered from a plane-only solid (box / slab):
// its outward normal (out of the material) + a point on the plane.
struct PlaneHalfspace {
  bool ok = false;
  math::Vec3 n;       ///< outward normal (points OUT of the kept material)
  math::Point3 o;     ///< a point on the bounding plane
};

// Recognise `s` as a planar half-space RELATIVE to a torus `tor`: a solid whose faces are
// ALL planar, EXACTLY ONE of which cuts the torus tube while every other plane leaves the
// whole torus strictly on its interior side (a box/slab acting as one half-space through
// the torus). Returns the cutting plane oriented outward. nullopt for a curved operand, a
// plane that misses the torus, or ≠1 cutting plane (→ decline → OCCT).
std::optional<PlaneHalfspace> recognisePlaneHalfspace(const CurvedSolid& tor,
                                                      const topo::Shape& s) {
  if (s.isNull()) return std::nullopt;
  // Interior reference: the torus tube-centre point at u=0 (definitely inside the torus, and
  // — for a box that fully brackets the torus save one cut — inside the box material too).
  const math::Point3 torRef{tor.frame.origin.x + tor.frame.x.vec().x * tor.radius,
                            tor.frame.origin.y + tor.frame.x.vec().y * tor.radius,
                            tor.frame.origin.z + tor.frame.x.vec().z * tor.radius};
  std::vector<PlaneHalfspace> cutting;
  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) {
    const auto surf = topo::surfaceOf(ex.current());
    if (!surf) return std::nullopt;
    if (surf->surface->kind != topo::FaceSurface::Kind::Plane) return std::nullopt;  // curved → not this path
    const math::Ax3 fr = ssidetail::worldFrame(*surf, ex.current());
    math::Vec3 n = fr.z.vec();
    if (math::dot(fr.origin - torRef, n) < 0.0) n = math::Vec3{-n.x, -n.y, -n.z};  // outward
    // Does this plane cut the torus? The torus point nearest the plane on the OUTWARD side is
    // at signed distance (centre→plane along −n) + (R + r) at most; it cuts iff the torus
    // straddles it. Compute the torus's max/min signed distance to the plane.
    const math::Vec3 zc = tor.frame.z.vec();
    const double axialComp = math::dot(zc, n);                       // n·ẑ
    const double radialComp = std::sqrt(std::max(1.0 - axialComp * axialComp, 0.0));
    // Farthest torus surface extent along n: centre-plane offset ± (R·radialComp + r).
    const double dCentre = math::dot(tor.frame.origin - fr.origin, n);
    const double reach = tor.radius * radialComp + tor.minorRadius;
    const double dMax = dCentre + reach, dMin = dCentre - reach;
    if (dMax > 1e-6 && dMin < -1e-6) cutting.push_back(PlaneHalfspace{true, n, fr.origin});
    else if (dMax > 1e-6) return std::nullopt;  // torus pokes OUT past a non-cutting plane → box does not bracket it
  }
  if (cutting.size() != 1) return std::nullopt;  // must be exactly one clean cut
  return cutting.front();
}

// Assemble the torus∩half-space setup for a PLANE ⟂ AXIS cut. hs.n must be (anti)parallel
// to the torus axis; the plane must cut the TUBE (|h| < r strictly). keepLow = keep z ≤ h.
TorusPlaneSetup torusPlaneSetup(const CurvedSolid& tor, const PlaneHalfspace& hs) {
  TorusPlaneSetup st;
  if (tor.kind != CurvedKind::Torus) return st;
  const math::Vec3 zc = tor.frame.z.vec();
  // Plane normal must be parallel to the axis (the ⟂-axis, horizontal-slice orientation).
  // A plane PARALLEL to the axis (Villarceau-like) or any oblique plane declines here.
  const math::Vec3 nn = hs.n;
  if (math::norm(math::cross(zc, nn)) > 1e-6) return st;  // not axis-perpendicular → OCCT
  const double axialSign = math::dot(zc, nn) >= 0.0 ? 1.0 : -1.0;  // +1 ⇒ outward n along +ẑ
  const double h = math::dot(hs.o - tor.frame.origin, zc);         // cut station on the axis
  const double R = tor.radius, r = tor.minorRadius;
  if (!(r > 1e-9) || !(R > r + 1e-9)) return st;
  if (!(std::fabs(h) < r - 1e-6)) return st;  // plane tangent / clear of the tube → OCCT
  // Outward n along +ẑ ⇒ material is on the z ≤ h side (keepLow); along −ẑ ⇒ z ≥ h side.
  st.keepLow = (axialSign > 0.0);
  st.tor = &tor;
  st.O = tor.frame.origin;
  st.X = tor.frame.x.vec();
  st.Y = tor.frame.y.vec();
  st.zc = zc;
  st.R = R;
  st.r = r;
  st.h = h;
  // Minor angle of the tube crossing: sin v = h/r. The two seam circles are at
  // ρ = R + r·cos v (outer) and ρ = R − r·cos v (inner). v0 = asin(h/r) ∈ (−π/2, π/2).
  st.v0 = std::asin(std::clamp(h / r, -1.0, 1.0));
  // Azimuth (N) + tube-arc (M) chord resolution — matched to S5-l torusCyl (kFacetSag).
  const double kFacetSag = 0.004;
  const double rhoMax = R + r;
  const double chordA = std::sqrt(std::max(8.0 * kFacetSag * rhoMax, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rhoMax / chordA)), 32, 200);
  const double chordM = std::sqrt(std::max(8.0 * kFacetSag * r, 1e-12));
  st.M = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * r / chordM)), 24, 160);
  st.ok = true;
  return st;
}

// The seam-annulus inner / outer radii + the tube-arc minor-angle span on the KEPT side.
// KEPT = z ≤ h (keepLow): the arc runs from the upper-INNER seam (v = π − v0) THROUGH the
// inner equator (π), the bottom (3π/2) and the outer equator (2π) to the upper-OUTER seam
// (2π + v0). KEPT = z ≥ h: the complementary short cap arc v ∈ [v0, π − v0] over the top.
struct TorusPlaneArc {
  double vA, vB;        ///< tube-arc minor-angle span for the kept side
  double rhoInner, rhoOuter;  ///< the annulus inner/outer radii at z = h
};
TorusPlaneArc torusPlaneArc(const TorusPlaneSetup& s) {
  const double c = s.r * std::cos(s.v0);            // = √(r² − h²) ≥ 0
  const double rhoOuter = s.R + c, rhoInner = s.R - c;
  if (s.keepLow)  // long way round the bottom
    return TorusPlaneArc{kSsiPi - s.v0, kSsiTwoPi + s.v0, rhoInner, rhoOuter};
  // keep z ≥ h: the short cap arc over the top (v0 → π − v0)
  return TorusPlaneArc{s.v0, kSsiPi - s.v0, rhoInner, rhoOuter};
}

// buildTorusPlaneCommon = torus ∩ half-space (plane ⟂ axis): the tube arc on the kept side
// + a flat ANNULUS cap at z = h between the inner/outer seam circles. Closed watertight
// solid of revolution.
topo::Shape buildTorusPlaneCommon(const TorusPlaneSetup& s) {
  if (!s.ok) return {};
  const TorusPlaneArc a = torusPlaneArc(s);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendTorusPlaneTubeArc(s, a.vA, a.vB, /*outwardSign=*/1.0, pool, faces);
  // Annulus cap at z = h. Its outward normal points AWAY from the kept material: +ẑ if we
  // keep z ≤ h, −ẑ if we keep z ≥ h. inner ring first, outer ring second (both at s = h).
  const std::vector<math::Point3> inner = s.ring(a.rhoInner, s.h);
  const std::vector<math::Point3> outer = s.ring(a.rhoOuter, s.h);
  const math::Vec3 capN = s.keepLow ? s.zc : math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z};
  appendAnnulusCap(inner, outer, capN, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusPlaneCut = torus − half-space: the torus material on the COMPLEMENT side, i.e.
// the OTHER tube arc + the annulus cap with the REVERSED normal. When the torus is the
// minuend the CUT keeps the far side of the plane; it is the same revolution assembly with
// the kept side flipped. (Order-sensitive: only a torus-minuend CUT lands here.)
topo::Shape buildTorusPlaneCut(const TorusPlaneSetup& s) {
  if (!s.ok) return {};
  TorusPlaneSetup flip = s;
  flip.keepLow = !s.keepLow;  // CUT removes the half-space material → keep the complement
  return buildTorusPlaneCommon(flip);
}

// buildTorusPlaneFuse = torus ∪ half-space. The half-space is a genuine (unbounded/large)
// solid, so the union is NOT a closed torus-segment revolution — it needs the half-space's
// OTHER bounding faces (the rest of the box), which are outside this family's revolution
// assembly. HONEST-DECLINE → NULL → OCCT (never faked). COMMON + CUT are the landed slices.
topo::Shape buildTorusPlaneFuse(const TorusPlaneSetup& s) {
  if (!s.ok) return {};
  return {};  // half-space-union needs the box's remaining faces → OCCT
}

// ═══ S5-m — COAXIAL TORUS ∩ SPHERE (COMMON / FUSE / CUT) ═════════════════════════
// The second TORUS-family pair (after the S5-l torus∩cylinder). A ring torus (major R,
// minor r, axis = frame Z, centre O) and a sphere (radius Rs) whose centre sits ON the
// torus axis AT THE TORUS CENTRE (the CLEANEST-oracle coaxial pose — a fully SYMMETRIC
// two-circle poke-through). In the meridian (ρ,z) plane the tube is the disk of radius r
// centred at (R,0); the sphere is the circle ρ²+z²=Rs². The two curves meet at the SAME
// radius ρ* = (R²−r²+Rs²)/(2R) and z = ±z0, z0 = √(r²−(ρ*−R)²) — TWO seam circles of equal
// radius ρ* at axial stations z=±z0 (exactly like the S5-l cylinder chord, but the seam
// radius is derived from the sphere, not a cylinder). For |z| ≤ z0 the sphere arc lies
// INSIDE the tube walls; for |z| > z0 the tube slice is entirely OUTSIDE the ball.
//
// Every boolean is a SOLID OF REVOLUTION welded from the S5-e…l machinery through one
// VertexPool — the revolved tube arcs (appendTubeArc, S5-l), the sphere ZONE between the
// two seam rings (appendSphereZone, S5-h/i), and the two sphere polar CAPS beyond the
// seams (appendSphereCap, S5-c). The tube-arc topology is IDENTICAL to S5-l torus∩cylinder
// (COMMON = inner arc; CUT = outer arc); only the CYLINDER chord band becomes the SPHERE
// zone/caps (a straight ρ=Rc ruling → the bulging sphere arc).
//
// CLEANEST-ORACLE choice (SSI-ROADMAP §S5): all volumes are AIRTIGHT closed forms —
//   V_torus  = 2π² R r²                                     (Pappus, full tube)
//   V_common = 2π·[ (Rs²−R²−r²)·z0 + R·(z0·√(r²−z0²) + r²·asin(z0/r)) ]
//              (Pappus of the ρ ≤ rho_s(z) tube segment; d²=r²−z², the −z²/+z² terms cancel)
//   V_cut(T−S)  = V_torus − V_common                        (a SHRINK)
//   V_fuse(T∪S) = V_torus + V_sphere − V_common             (a GROW; hole filled by the ball)
// The generic booleanResultVerified drives FUSE/CUT off the native COMMON as V(A∩B); the
// engine's ssiCurvedBooleanVerified S5-m arm additionally checks COMMON against this closed
// form directly.
//
// HONEST SCOPE. Ring torus only (R > r > 0; a spindle R ≤ r declines at recognition). The
// sphere centre must sit ON the torus axis AND at the torus centre (sc = 0 — the symmetric
// pose whose two seams share one radius); an off-centre coaxial sphere (sc ≠ 0, unequal
// seam radii — the general spiric section) and a non-coaxial / off-axis sphere both decline
// → OCCT. A proper two-circle crossing requires |ρ*−R| < r strictly (a tangent / clear /
// engulfing sphere declines). A sphere-minuend (sphere − torus) CUT declines → OCCT.
struct TorusSphereSetup {
  bool ok = false;
  const CurvedSolid* tor = nullptr;
  const CurvedSolid* sph = nullptr;
  math::Point3 O;          ///< TORUS centre (= sphere centre) on the shared axis
  math::Vec3 X, Y, zc;     ///< torus frame basis (zc = shared axis)
  double R = 0.0, r = 0.0;  ///< torus major / minor radius
  double Rs = 0.0;          ///< sphere radius
  double rhoStar = 0.0;     ///< seam circle radius (= (R²−r²+Rs²)/(2R))
  double v0 = 0.0;          ///< tube minor angle of the +z seam (cos v0 = (ρ*−R)/r)
  double z0 = 0.0;          ///< +z seam axial station (= √(r²−(ρ*−R)²))
  int N = 0;                ///< azimuth sample count (seam-chord bounded)
  int M = 0;                ///< tube-arc subdivision (minor-angle chord bounded)

  std::vector<math::Point3> ring(double rho, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = rho * std::cos(u), cy = rho * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  std::pair<double, double> tube(double v) const {
    return {R + r * std::cos(v), r * std::sin(v)};
  }
  math::Point3 axisPt(double s) const {
    return math::Point3{O.x + zc.x * s, O.y + zc.y * s, O.z + zc.z * s};
  }
  Seam seamRing(double rho, double s) const {
    Seam out;
    out.closed = true;
    out.pts = ring(rho, s);
    return out;
  }
};

TorusSphereSetup torusSphereSetup(const CurvedSolid& A, const CurvedSolid& B,
                                  const std::vector<Seam>& seams) {
  TorusSphereSetup st;
  if (seams.empty()) return st;  // need ≥1 traced seam to cross-check

  const CurvedSolid* torPtr = nullptr;
  const CurvedSolid* sphPtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Torus) torPtr = s;
    else if (s->kind == CurvedKind::Sphere) sphPtr = s;
  }
  if (!torPtr || !sphPtr) return st;
  const CurvedSolid& tor = *torPtr;
  const CurvedSolid& sph = *sphPtr;

  const math::Vec3 zc = tor.frame.z.vec();
  const math::Point3 O = tor.frame.origin;
  // Sphere centre must be ON the torus axis AND at the torus centre (the symmetric pose):
  // the offset vector from the torus centre to the sphere centre must be ~zero.
  const math::Vec3 d{sph.frame.origin.x - O.x, sph.frame.origin.y - O.y, sph.frame.origin.z - O.z};
  if (math::norm(d) > 1e-6) return st;  // off-centre coaxial / off-axis → OCCT (never faked)

  const double R = tor.radius, r = tor.minorRadius, Rs = sph.radius;
  if (!(r > 1e-9) || !(R > r + 1e-9) || !(Rs > 1e-9)) return st;
  const double rhoStar = (R * R - r * r + Rs * Rs) / (2.0 * R);
  if (!(rhoStar > 1e-9)) return st;  // seam radius must be positive
  const double dChord = rhoStar - R;
  // Proper two-circle poke-through: the seam radius sits strictly between the tube walls.
  if (!(std::fabs(dChord) < r - 1e-6)) return st;  // tangent / clear / engulfing → OCCT
  const double cosv = dChord / r;
  const double v0 = std::acos(std::clamp(cosv, -1.0, 1.0));  // ∈ (0, π)
  const double z0 = r * std::sin(v0);                        // = √(r² − dChord²)
  if (!(z0 > 1e-6)) return st;

  // Cross-check EVERY traced seam against ONE of the two analytic circles (station ±z0,
  // radius ρ*) — never trust a missing / mis-placed loop.
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
    const bool matchLo = std::fabs(sTr + z0) < 1e-3 && std::fabs(rhoTr - rhoStar) < 1e-3;
    const bool matchHi = std::fabs(sTr - z0) < 1e-3 && std::fabs(rhoTr - rhoStar) < 1e-3;
    if (!matchLo && !matchHi) return st;  // traced seam matches neither analytic circle → OCCT
  }

  // Azimuth (N) + tube-arc (M) resolution, chord-bounded so a planar facet's sagitta off
  // the true surface stays under the mesh deflection (matches the S5-l torus∩cylinder
  // discretisation exactly).
  const double kFacetSag = 0.004;
  const double chordA = std::sqrt(std::max(8.0 * kFacetSag * rhoStar, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rhoStar / chordA)), 32, 200);
  const double chordM = std::sqrt(std::max(8.0 * kFacetSag * r, 1e-12));
  st.M = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * r / chordM)), 24, 160);
  st.tor = torPtr;
  st.sph = sphPtr;
  st.O = O;
  st.X = tor.frame.x.vec();
  st.Y = tor.frame.y.vec();
  st.zc = zc;
  st.R = R;
  st.r = r;
  st.Rs = Rs;
  st.rhoStar = rhoStar;
  st.v0 = v0;
  st.z0 = z0;
  st.ok = true;
  return st;
}

// The revolved tube arc between minor angles vA→vB for a TorusSphereSetup (the S5-l
// appendTubeArc keys off TorusCylSetup; the geometry is identical — a revolved tube band
// oriented by the true tube-outward reference radiating from the tube-centre circle).
void appendTorusSphereTubeArc(const TorusSphereSetup& s, double vA, double vB,
                              VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int m = s.M;
  auto tubeCentreAt = [&](int i) {
    const double u = kSsiTwoPi * i / s.N;
    const double cx = s.R * std::cos(u), cy = s.R * std::sin(u);
    return math::Point3{s.O.x + s.X.x * cx + s.Y.x * cy, s.O.y + s.X.y * cx + s.Y.y * cy,
                        s.O.z + s.X.z * cx + s.Y.z * cy};
  };
  std::vector<math::Point3> prev;
  for (int k = 0; k <= m; ++k) {
    const double v = vA + (vB - vA) * (static_cast<double>(k) / m);
    const auto [rho, z] = s.tube(v);
    std::vector<math::Point3> cur = s.ring(rho, z);
    if (k > 0) {
      const int n = s.N;
      for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        const math::Point3 tc = tubeCentreAt(i);
        const math::Point3 mid{(prev[i].x + prev[j].x + cur[j].x + cur[i].x) / 4,
                               (prev[i].y + prev[j].y + cur[j].y + cur[i].y) / 4,
                               (prev[i].z + prev[j].z + cur[j].z + cur[i].z) / 4};
        const math::Vec3 out{mid.x - tc.x, mid.y - tc.y, mid.z - tc.z};
        pushPlanarTri(prev[i], prev[j], cur[j], out, pool, faces);
        pushPlanarTri(prev[i], cur[j], cur[i], out, pool, faces);
      }
    }
    prev = std::move(cur);
  }
}

// Cap ring-count for a torus∩sphere polar cap (mirrors cylSphereCapRings: polar half-angle
// at the sphere centre × sqrt(Rs/(2·kCapSagitta))).
int torusSphereCapRings(const TorusSphereSetup& s, const std::vector<math::Point3>& ring,
                        const math::Point3& apex) {
  const math::Vec3 aDir{apex.x - s.O.x, apex.y - s.O.y, apex.z - s.O.z};
  double theta = 0.0;
  for (const auto& p : ring) {
    const math::Vec3 sDir{p.x - s.O.x, p.y - s.O.y, p.z - s.O.z};
    const double den = std::max(math::norm(aDir) * math::norm(sDir), 1e-12);
    theta = std::max(theta, std::acos(std::clamp(math::dot(aDir, sDir) / den, -1.0, 1.0)));
  }
  return std::clamp(
      static_cast<int>(std::ceil(std::max(theta, 1e-6) * std::sqrt(s.Rs / (2.0 * kCapSagitta)))), 4,
      48);
}

// buildTorusSphereCommon(A,B) = COMMON of the coaxial torus∩sphere: the tube ∩ ball — the
// INNER tube arc (v ∈ [v0, 2π−v0], through the inner equator ρ = R−r) + the SPHERE ZONE
// between the two seam rings (the bulging sphere arc ρ = √(Rs²−z²), replacing the S5-l
// cylinder chord band). Watertight closed surface of revolution.
topo::Shape buildTorusSphereCommon(const CurvedSolid& A, const CurvedSolid& B,
                                   const std::vector<Seam>& seams) {
  const TorusSphereSetup s = torusSphereSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // Inner tube arc from the +z seam (v0) through the inner equator (π) to the −z seam.
  appendTorusSphereTubeArc(s, s.v0, kSsiTwoPi - s.v0, pool, faces);
  // Sphere zone ρ = √(Rs²−z²) from the −z seam ring up to the +z seam ring (outward radial).
  const std::vector<math::Point3> ringLo = s.ring(s.rhoStar, -s.z0);
  const std::vector<math::Point3> ringHi = s.ring(s.rhoStar, s.z0);
  appendSphereZone(s.O, s.Rs, ringLo, ringHi, pool, faces, /*outwardSign=*/1.0);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusSphereCut(A,B) = A − B with TORUS the minuend: the tube ∖ ball — the OUTER tube
// arc (v ∈ [−v0, v0], through the outer equator ρ = R+r, covering the pole-region tube where
// |z| > z0) + the SPHERE ZONE REVERSED (inward normal, bounding the carved cavity). A SHRINK,
// one closed ring-of-revolution component. A sphere-minuend (sphere − torus) declines → OCCT.
topo::Shape buildTorusSphereCut(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  const TorusSphereSetup s = torusSphereSetup(A, B, seams);
  if (!s.ok) return {};
  if (A.kind != CurvedKind::Torus) return {};  // order-sensitive: torus must be the minuend
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // Outer tube arc from the −z seam (−v0) through the outer equator (0) to the +z seam (v0).
  appendTorusSphereTubeArc(s, -s.v0, s.v0, pool, faces);
  // Sphere zone REVERSED (inward radial) between the two seam rings: the scooped cavity wall.
  const std::vector<math::Point3> ringLo = s.ring(s.rhoStar, -s.z0);
  const std::vector<math::Point3> ringHi = s.ring(s.rhoStar, s.z0);
  appendSphereZone(s.O, s.Rs, ringLo, ringHi, pool, faces, /*outwardSign=*/-1.0);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusSphereFuse(A,B) = A ∪ B: the OUTER tube arc bulge (v ∈ [−v0, v0], ρ > ρ*, outside
// the ball) + the TWO SPHERE POLAR CAPS (each seam ring out to its pole, the sphere surface
// OUTSIDE the tube). The ball fills the donut hole and the mid-band, so the union is simply
// connected (no inner hole). A GROW. V = V_torus + V_sphere − V_common.
topo::Shape buildTorusSphereFuse(const CurvedSolid& A, const CurvedSolid& B,
                                 const std::vector<Seam>& seams) {
  const TorusSphereSetup s = torusSphereSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  // Outer tube-arc bulge (−v0 → 0 → v0), sharing the two seam rings.
  appendTorusSphereTubeArc(s, -s.v0, s.v0, pool, faces);
  // Two sphere polar caps beyond the seams: the +z0 seam ring out to the +Rs pole, the −z0
  // seam ring out to the −Rs pole (apexes on the axis, outward radial normal).
  const Seam seamLo = s.seamRing(s.rhoStar, -s.z0);
  const Seam seamHi = s.seamRing(s.rhoStar, s.z0);
  const math::Point3 poleP = s.axisPt(s.Rs);
  const math::Point3 poleM = s.axisPt(-s.Rs);
  // apex = the sphere pole; appendSphereCap picks the surface point nearest otherCentre
  // (outer=false). Pass otherCentre = the pole itself so the apex resolves to that pole.
  appendSphereCap(*s.sph, poleP, seamHi, torusSphereCapRings(s, seamHi.pts, poleP), pool, faces,
                  /*outer=*/false, /*reversed=*/false);
  appendSphereCap(*s.sph, poleM, seamLo, torusSphereCapRings(s, seamLo.pts, poleM), pool, faces,
                  /*outer=*/false, /*reversed=*/false);
  if (faces.size() < 6) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ═══ S5-n — COAXIAL TORUS ∩ CONE (COMMON / FUSE / CUT) ═══════════════════════════
// The THIRD torus-family pair (after S5-l torus∩cylinder and S5-m torus∩sphere). A ring
// torus (major R, minor r, axis = frame Z, centre O) and a COAXIAL cone (apex/axis on the
// shared axis, half-angle α) whose OBLIQUE wall crosses the torus TUBE at TWO latitudes →
// TWO analytic circle seams. This GENERALISES S5-l: where the cylinder is a VERTICAL chord
// ρ = Rc in the meridian (ρ,z) plane, the cone is a SLANTED chord ρ = a + b·z (b = ±tanα),
// so the two seam circles sit at DIFFERENT radii AND different axial stations — but they are
// still analytic circles (two surfaces of revolution about one axis meet in circles), and the
// COMMON is still a Pappus-exact solid of revolution.
//
// In the meridian plane the tube is the disk of radius r centred at (R,0); the cone wall is
// the line ρ = a + b·z. The line cuts the tube-boundary circle (ρ−R)²+z²=r² where
//   (1+b²)z² + 2b(a−R)z + (a−R)²−r² = 0,  roots z1<z2, seam radii ρ_i = a + b·z_i.
// The COMMON is the tube part on the CONE-INTERIOR side (ρ ≤ a+b·z) — the INNER tube arc
// (through the inner equator ρ=R−r) closed by the cone chord band between the two seam rings;
// a closed meridian loop → a watertight closed surface of revolution (no caps), EXACTLY the
// S5-l COMMON topology with the vertical chord band replaced by a slanted (still exact-on-cone)
// revolved band. CUT (torus−cone) revolves the OUTER tube arc + the reversed cone chord band;
// FUSE adds the cone wall OUTSIDE the tube + the two cone terminal disc caps (the cone frustum
// fills the donut hole → simply connected).
//
// CLEANEST-ORACLE choice (SSI-ROADMAP §S5): the COMMON volume is an AIRTIGHT closed form that
// REDUCES to the S5-l torus∩cylinder formula at b=0. Working about the tube centre (ρ'=ρ−R),
// the cone chord is (ρ',z)·m̂ = t0 with m̂ = (1,−b)/√(1+b²) (unit normal into the DISCARDED
// ρ>line region) and signed offset t0 = (a−R)/√(1+b²). The discarded circular segment has
//   area  A_d(t0) = r²·acos(t0/r) − t0·√(r²−t0²)                       (valid ∀ t0∈[−r,r])
//   ρ'-moment  ∫∫_discard ρ' dA = (1/√(1+b²))·(2/3)(r²−t0²)^{3/2}      (the standard segment moment)
// so the KEPT (COMMON) segment has area A_seg = πr² − A_d and ρ'-moment M = −(1/√(1+b²))·(2/3)(r²−t0²)^{3/2},
// and by Pappus
//   V_common = 2π·(R·A_seg + M).
// At b=0: m̂=(1,0), t0=a−R=Rc−R=d ⇒ A_d = r²acos(d/r)−d√(r²−d²) (the S5-l cap), M = −(2/3)(r²−d²)^{3/2}
// — byte-identical to the S5-l closed form. The generic booleanResultVerified drives FUSE/CUT off
// the native COMMON as V(A∩B); the engine's ssiCurvedBooleanVerified S5-n arm checks COMMON
// against this closed form directly.
//
// HONEST SCOPE. Ring torus only (R > r > 0; a spindle R ≤ r declines at recognition). The cone
// must be COAXIAL (axis parallel + origin colinear with the torus axis) and its wall must cross
// the tube in a proper TWO-CIRCLE poke-through (two distinct real roots strictly inside the tube),
// with the cone axially spanning past both seams. A degenerate near-cylindrical cone (tanα≈0)
// declines (the S5-l cylinder path owns that); a cone tangent to / clear of the tube, an
// apex-crossing pose whose chord does not cut the tube in two circles, and a non-coaxial /
// off-axis / skew cone all decline → NULL → OCCT (never faked). A cone-minuend (cone − torus)
// CUT declines → OCCT.
struct TorusConeSetup {
  bool ok = false;
  const CurvedSolid* tor = nullptr;
  const CurvedSolid* cone = nullptr;
  math::Point3 O;          ///< TORUS centre (on the shared axis) — the canonical origin
  math::Vec3 X, Y, zc;     ///< torus frame basis (zc = shared axis)
  double R = 0.0, r = 0.0;  ///< torus major / minor radius
  double a = 0.0, b = 0.0;  ///< cone meridian line ρ = a + b·s in the TORUS s-frame (b = ±tanα)
  double z1 = 0.0, z2 = 0.0;    ///< the two seam axial stations (z1 < z2)
  double rho1 = 0.0, rho2 = 0.0;  ///< the two seam radii (= a + b·z_i)
  double v1 = 0.0, v2 = 0.0;    ///< tube minor angle of the z1 / z2 seam
  bool innerFwd = true;         ///< the inner (ρ≤line) tube arc runs v1→v2 (true) or v2→v1 (false)
  double coneS0 = 0.0, coneS1 = 0.0;  ///< cone axial extent in the torus s-frame (coneS0 < coneS1)
  int N = 0;                ///< azimuth sample count (seam-chord bounded)
  int M = 0;                ///< tube-arc subdivision (minor-angle chord bounded)

  std::vector<math::Point3> ring(double rho, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = rho * std::cos(u), cy = rho * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  std::pair<double, double> tube(double v) const {
    return {R + r * std::cos(v), r * std::sin(v)};
  }
  double rCone(double s) const { return a + b * s; }
  math::Point3 axisPt(double s) const {
    return math::Point3{O.x + zc.x * s, O.y + zc.y * s, O.z + zc.z * s};
  }
};

TorusConeSetup torusConeSetup(const CurvedSolid& A, const CurvedSolid& B,
                              const std::vector<Seam>& seams) {
  TorusConeSetup st;
  if (seams.empty()) return st;  // need ≥1 traced seam to cross-check

  const CurvedSolid* torPtr = nullptr;
  const CurvedSolid* conePtr = nullptr;
  for (const CurvedSolid* s : {&A, &B}) {
    if (s->kind == CurvedKind::Torus) torPtr = s;
    else if (s->kind == CurvedKind::Cone) conePtr = s;
  }
  if (!torPtr || !conePtr) return st;
  const CurvedSolid& tor = *torPtr;
  const CurvedSolid& cone = *conePtr;

  const math::Vec3 zc = tor.frame.z.vec();
  const math::Point3 O = tor.frame.origin;
  // Cone must be COAXIAL: parallel axis + colinear origin.
  if (math::norm(math::cross(zc, cone.frame.z.vec())) > 1e-6) return st;
  const math::Vec3 d{cone.frame.origin.x - O.x, cone.frame.origin.y - O.y, cone.frame.origin.z - O.z};
  if (math::norm(d - zc * math::dot(d, zc)) > 1e-6) return st;  // cone axis not colinear → OCCT

  const double R = tor.radius, r = tor.minorRadius;
  if (!(r > 1e-9) || !(R > r + 1e-9)) return st;
  const double tanA = std::tan(cone.semiAngle);
  if (std::fabs(tanA) < 1e-6) return st;  // near-cylindrical cone → the S5-l cylinder path owns it

  // Cone meridian line ρ = a + b·s in the TORUS s-frame (s = axial projection from O).
  const double base = math::dot(d, zc);
  const double sgn = math::dot(cone.frame.z.vec(), zc) >= 0.0 ? 1.0 : -1.0;
  const double b = sgn * tanA;
  const double a = cone.radius - sgn * base * tanA;

  // Seam quadratic (1+b²)z² + 2b(a−R)z + (a−R)²−r² = 0 → two distinct real roots strictly
  // inside the tube for a proper two-circle poke-through.
  const double Aq = 1.0 + b * b;
  const double Bq = 2.0 * b * (a - R);
  const double Cq = (a - R) * (a - R) - r * r;
  const double disc = Bq * Bq - 4.0 * Aq * Cq;
  if (!(disc > 1e-9)) return st;  // tangent / clear → no proper two-circle crossing → OCCT
  const double sq = std::sqrt(disc);
  double z1 = (-Bq - sq) / (2.0 * Aq), z2 = (-Bq + sq) / (2.0 * Aq);
  if (z1 > z2) std::swap(z1, z2);
  const double rho1 = a + b * z1, rho2 = a + b * z2;
  if (!(rho1 > 1e-9) || !(rho2 > 1e-9)) return st;  // a seam ring collapses on the axis → OCCT
  // Both seams must lie strictly on the tube (|ρ_i−R| < r), guaranteed by the roots, and off
  // the tube extremes so the two minor-angle arcs are non-degenerate.
  if (!(z2 - z1 > 1e-6)) return st;

  // Cone axial extent in the torus s-frame; the cone must span PAST both seams so its wall
  // crosses the tube in two full circles (both seams interior to the cone frustum).
  double coneS0 = base + sgn * cone.vLo, coneS1 = base + sgn * cone.vHi;
  if (coneS0 > coneS1) std::swap(coneS0, coneS1);
  if (!(coneS0 < z1 - 1e-6) || !(coneS1 > z2 + 1e-6)) return st;  // seams not both interior → OCCT

  // Tube minor angles of the two seams and which tube arc is the CONE-INTERIOR (ρ≤line) side.
  const double v1 = std::atan2(z1 / r, (rho1 - R) / r);
  const double v2 = std::atan2(z2 / r, (rho2 - R) / r);
  // Midpoint of the v1→v2 forward arc; if its tube point is inside the cone (ρ ≤ a+b·z) that
  // forward arc is the inner (COMMON) arc, else the v2→v1 arc is.
  auto vmidFwd = [&](double va, double vb) {
    double d2 = vb - va;
    while (d2 < 0.0) d2 += kSsiTwoPi;
    return va + 0.5 * d2;
  };
  const double vm = vmidFwd(v1, v2);
  const double rhoM = R + r * std::cos(vm), zM = r * std::sin(vm);
  const bool innerFwd = rhoM <= a + b * zM;

  // Cross-check EVERY traced seam against ONE of the two analytic circles (station z_i,
  // radius ρ_i) — never trust a missing / mis-placed loop.
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
    const bool m1 = std::fabs(sTr - z1) < 1e-3 && std::fabs(rhoTr - rho1) < 1e-3;
    const bool m2 = std::fabs(sTr - z2) < 1e-3 && std::fabs(rhoTr - rho2) < 1e-3;
    if (!m1 && !m2) return st;  // traced seam matches neither analytic circle → OCCT
  }

  // Azimuth (N) + tube-arc (M) resolution, chord-bounded (matches the S5-l discretisation).
  const double kFacetSag = 0.004;
  const double rMax = std::max(rho1, rho2);
  const double chordA = std::sqrt(std::max(8.0 * kFacetSag * rMax, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rMax / chordA)), 32, 200);
  const double chordM = std::sqrt(std::max(8.0 * kFacetSag * r, 1e-12));
  st.M = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * r / chordM)), 24, 160);
  st.tor = torPtr;
  st.cone = conePtr;
  st.O = O;
  st.X = tor.frame.x.vec();
  st.Y = tor.frame.y.vec();
  st.zc = zc;
  st.R = R;
  st.r = r;
  st.a = a;
  st.b = b;
  st.z1 = z1;
  st.z2 = z2;
  st.rho1 = rho1;
  st.rho2 = rho2;
  st.v1 = v1;
  st.v2 = v2;
  st.innerFwd = innerFwd;
  st.coneS0 = coneS0;
  st.coneS1 = coneS1;
  st.ok = true;
  return st;
}

// The revolved tube arc between minor angles vA→vB for a TorusConeSetup (identical geometry to
// the S5-l appendTubeArc — a revolved tube band oriented by the true tube-outward reference
// radiating from the tube-centre circle).
void appendTorusConeTubeArc(const TorusConeSetup& s, double vA, double vB,
                            VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int m = s.M;
  auto tubeCentreAt = [&](int i) {
    const double u = kSsiTwoPi * i / s.N;
    const double cx = s.R * std::cos(u), cy = s.R * std::sin(u);
    return math::Point3{s.O.x + s.X.x * cx + s.Y.x * cy, s.O.y + s.X.y * cx + s.Y.y * cy,
                        s.O.z + s.X.z * cx + s.Y.z * cy};
  };
  std::vector<math::Point3> prev;
  for (int k = 0; k <= m; ++k) {
    const double v = vA + (vB - vA) * (static_cast<double>(k) / m);
    const auto [rho, z] = s.tube(v);
    std::vector<math::Point3> cur = s.ring(rho, z);
    if (k > 0) {
      const int n = s.N;
      for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        const math::Point3 tc = tubeCentreAt(i);
        const math::Point3 mid{(prev[i].x + prev[j].x + cur[j].x + cur[i].x) / 4,
                               (prev[i].y + prev[j].y + cur[j].y + cur[i].y) / 4,
                               (prev[i].z + prev[j].z + cur[j].z + cur[i].z) / 4};
        const math::Vec3 out{mid.x - tc.x, mid.y - tc.y, mid.z - tc.z};
        pushPlanarTri(prev[i], prev[j], cur[j], out, pool, faces);
        pushPlanarTri(prev[i], cur[j], cur[i], out, pool, faces);
      }
    }
    prev = std::move(cur);
  }
}

// The inner (COMMON) / outer (CUT) tube arcs by minor-angle endpoints, honouring innerFwd.
// The inner arc goes through the inner equator (ρ=R−r, v=π); the outer through ρ=R+r (v=0).
void appendTorusConeInnerArc(const TorusConeSetup& s, VertexPool& pool,
                             std::vector<topo::Shape>& faces) {
  double va = s.innerFwd ? s.v1 : s.v2;
  double vb = s.innerFwd ? s.v2 : s.v1;
  while (vb < va) vb += kSsiTwoPi;   // forward sweep va→vb (through the inner equator)
  appendTorusConeTubeArc(s, va, vb, pool, faces);
}
void appendTorusConeOuterArc(const TorusConeSetup& s, VertexPool& pool,
                             std::vector<topo::Shape>& faces) {
  double va = s.innerFwd ? s.v2 : s.v1;
  double vb = s.innerFwd ? s.v1 : s.v2;
  while (vb < va) vb += kSsiTwoPi;   // the complementary (outer) sweep
  appendTorusConeTubeArc(s, va, vb, pool, faces);
}

// buildTorusConeCommon(A,B) = COMMON of the coaxial torus∩cone: the tube ∩ cone-solid — the
// INNER tube arc (through the inner equator) closed by the cone chord band between the two seam
// rings (slanted, exact on the cone). A closed watertight surface of revolution (no caps).
topo::Shape buildTorusConeCommon(const CurvedSolid& A, const CurvedSolid& B,
                                 const std::vector<Seam>& seams) {
  const TorusConeSetup s = torusConeSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendTorusConeInnerArc(s, pool, faces);
  // Cone chord band from the z1 seam ring to the z2 seam ring (outward = +radial; the cone
  // interior is on the −radial side of its wall for this poke-through). A straight ruling is
  // EXACT on the cone wall.
  const std::vector<math::Point3> ring1 = s.ring(s.rho1, s.z1);
  const std::vector<math::Point3> ring2 = s.ring(s.rho2, s.z2);
  appendRevolvedBand(ring1, ring2, s.O, s.zc, pool, faces, 1.0);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusConeCut(A,B) = A − B with TORUS the minuend: the tube ∖ cone — the OUTER tube arc
// (through the outer equator ρ=R+r) + the cone chord band REVERSED (inward normal, bounding the
// carved cavity). A SHRINK, one closed ring-of-revolution component. A cone-minuend (cone −
// torus) declines → OCCT.
topo::Shape buildTorusConeCut(const CurvedSolid& A, const CurvedSolid& B,
                              const std::vector<Seam>& seams) {
  const TorusConeSetup s = torusConeSetup(A, B, seams);
  if (!s.ok) return {};
  if (A.kind != CurvedKind::Torus) return {};  // order-sensitive: torus must be the minuend
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendTorusConeOuterArc(s, pool, faces);
  const std::vector<math::Point3> ring1 = s.ring(s.rho1, s.z1);
  const std::vector<math::Point3> ring2 = s.ring(s.rho2, s.z2);
  appendRevolvedBand(ring1, ring2, s.O, s.zc, pool, faces, -1.0);  // reversed (inward) cone wall
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusConeFuse(A,B) = A ∪ B: the union profile — the cone frustum fills the donut hole,
// so the union is simply connected. The OUTER boundary walks bottom→top: the cone terminal
// disc at coneS0, the cone wall from coneS0 up to the z1 seam, the OUTER tube-arc bulge between
// the two seams (ρ > line, outside the cone), the cone wall from the z2 seam up to coneS1, and
// the cone terminal disc at coneS1. A GROW. V = V_torus + V_cone − V_common.
topo::Shape buildTorusConeFuse(const CurvedSolid& A, const CurvedSolid& B,
                               const std::vector<Seam>& seams) {
  const TorusConeSetup s = torusConeSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  const double r0 = s.rCone(s.coneS0), r1 = s.rCone(s.coneS1);
  if (!(r0 > 1e-9) || !(r1 > 1e-9)) return {};  // a terminal disc collapses on the axis → decline
  const std::vector<math::Point3> ring0 = s.ring(r0, s.coneS0);        // bottom cap rim
  const std::vector<math::Point3> ringS1 = s.ring(s.rho1, s.z1);       // z1 seam
  const std::vector<math::Point3> ringS2 = s.ring(s.rho2, s.z2);       // z2 seam
  const std::vector<math::Point3> ring1 = s.ring(r1, s.coneS1);        // top cap rim
  // Bottom disc cap (outward normal −z), cone wall up to the z1 seam.
  appendAxisDiscCap(s.axisPt(s.coneS0), ring0, math::Vec3{-s.zc.x, -s.zc.y, -s.zc.z}, pool, faces);
  appendRevolvedBand(ring0, ringS1, s.O, s.zc, pool, faces, 1.0);
  // Outer tube-arc bulge between the two seams (ρ > line, outside the cone).
  appendTorusConeOuterArc(s, pool, faces);
  // Cone wall above the z2 seam, top disc cap (outward normal +z).
  appendRevolvedBand(ringS2, ring1, s.O, s.zc, pool, faces, 1.0);
  appendAxisDiscCap(s.axisPt(s.coneS1), ring1, s.zc, pool, faces);
  if (faces.size() < 6) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ═══ S5-o — COAXIAL TORUS ∩ TORUS (COMMON / FUSE / CUT) ══════════════════════════
// The FOURTH torus-family pair (after S5-l torus∩cylinder, S5-m torus∩sphere, S5-n
// torus∩cone) and the FIRST curved∩curved slice where BOTH operands are tori. Two COAXIAL
// ring tori sharing an axis: torus A (major R1, minor r1, tube-centre circle at station zA)
// and torus B (major R2, minor r2, tube-centre at zB), zA/zB measured along the shared axis
// from the frame origin. In the meridian (ρ,z) plane BOTH tubes are DISKS — tube A the disk
// of radius r1 centred at (R1,zA), tube B the disk of radius r2 centred at (R2,zB). Two
// circles in a plane meet in up to TWO points; when they cross properly (|r1−r2| < D < r1+r2,
// D = centre distance) the two intersection points give TWO analytic seam circles at DIFFERENT
// radii AND stations. Unlike S5-l/m/n (one tube-arc wall + a flat/sphere-zone wall from the
// second primitive), here BOTH boundary walls are TUBE ARCS, so every op reuses appendTubeArc
// on BOTH tori — no flat chord band, no caps.
//
// Seam solve (two circles in the meridian plane). Centre A = (R1,zA), centre B = (R2,zB),
// d = B−A, D = |d|, unit û = d/D. The chord line sits at signed offset a = (D²+r1²−r2²)/(2D)
// from A along û; the half-chord h = √(r1²−a²) = √(r2²−(D−a)²). The two seam meridian points
// are  M ± h·û⊥  with M = A + a·û. Each seam point (ρ*,z*) with ρ*>0 → a seam circle radius ρ*
// at station z*. A proper two-circle crossing needs |r1−r2| < D < r1+r2 and both ρ*>0.
//
// Topology (all revolved arcs; the meridian region revolved about the axis):
//   * COMMON (A∩B) — the LENS (disk A ∩ disk B): the arc of tube A INSIDE tube B + the arc of
//     tube B INSIDE tube A, both outward (+1). A closed watertight ring of revolution.
//   * FUSE (A∪B) — the UNION (disk A ∪ disk B): the arc of tube A OUTSIDE B + the arc of tube B
//     OUTSIDE A (the complementary outer arcs), both outward. A GROW; one ring of revolution.
//   * CUT (A−B, torus-A minuend) — disk A ∖ lens: the OUTER arc of tube A + the INNER arc of
//     tube B REVERSED (inward normal, bounding the carved bite). A SHRINK. B−A declines (built
//     by swapping operands — COMMON/FUSE are symmetric, CUT is order-sensitive).
//
// CLEANEST-ORACLE (SSI-ROADMAP §S5). The COMMON is the revolved planar lens, so by Pappus its
// volume is 2π times the first moment of the lens area about the axis. The lens splits into the
// two circular segments beyond the chord; the segment first-moment about the axis for a disk of
// radius rr centred (Rc,·) cut at signed offset t (kept side |·|≥t toward the other centre) is
// Rc·A_seg + n̂ρ·(2/3)(rr²−t²)^{3/2}, A_seg = rr²·acos(t/rr) − t·√(rr²−t²). Summed over the two
// tori the equal-h moment terms CANCEL (n̂ρ of B = −n̂ρ of A on the shared chord), leaving the
// airtight closed form  V_common = 2π·(R1·A_segA + R2·A_segB),  A_segA = r1²·acos(a/r1) − a·h,
// A_segB = r2²·acos((D−a)/r2) − (D−a)·h. FUSE/CUT follow by inclusion–exclusion off V_torus_i =
// 2π²R_i r_i² (the generic booleanResultVerified path); the engine's ssiCurvedBooleanVerified
// S5-o arm checks COMMON against the closed form directly.
//
// HONEST SCOPE. Ring tori only (R_i > r_i > 0; a spindle declines at recognition). The two tori
// must be COAXIAL (parallel + colinear axes) and their tubes must cross in a proper TWO-CIRCLE
// poke-through (|r1−r2| < D < r1+r2, both seam radii > 0). Concentric-coaxial tubes (D≈0), a
// contained tube (D ≤ |r1−r2|), tangent/clear tubes (D ≥ r1+r2), a seam ring collapsing on the
// axis, and non-coaxial / off-axis / skew tori all decline → NULL → OCCT (never faked).
struct TorusTorusSetup {
  bool ok = false;
  const CurvedSolid* A = nullptr;   ///< torus A (the CUT minuend when order-sensitive)
  const CurvedSolid* B = nullptr;   ///< torus B
  math::Point3 O;          ///< torus-A centre projected onto the axis — canonical origin
  math::Vec3 X, Y, zc;     ///< torus-A frame basis (zc = shared axis)
  double R1 = 0.0, r1 = 0.0, zA = 0.0;  ///< torus A: major / minor / tube-centre station
  double R2 = 0.0, r2 = 0.0, zB = 0.0;  ///< torus B: major / minor / tube-centre station
  double rho1 = 0.0, z1 = 0.0;   ///< seam 1 (ρ,station)
  double rho2 = 0.0, z2 = 0.0;   ///< seam 2 (ρ,station)
  // Minor angles of the two seams on each tube, and which forward arc is the INNER (∩-side).
  double vA1 = 0.0, vA2 = 0.0;   ///< seam1/seam2 minor angle on tube A
  double vB1 = 0.0, vB2 = 0.0;   ///< seam1/seam2 minor angle on tube B
  bool innerFwdA = true;         ///< tube-A inner arc runs vA1→vA2 (true) or vA2→vA1 (false)
  bool innerFwdB = true;         ///< tube-B inner arc runs vB1→vB2 (true) or vB2→vB1 (false)
  int N = 0;                ///< azimuth sample count (seam-chord bounded)
  int M = 0;                ///< tube-arc subdivision (minor-angle chord bounded)

  std::vector<math::Point3> ring(double rho, double s) const {
    std::vector<math::Point3> out(N);
    for (int i = 0; i < N; ++i) {
      const double u = kSsiTwoPi * i / N;
      const double cx = rho * std::cos(u), cy = rho * std::sin(u);
      out[i] = math::Point3{O.x + X.x * cx + Y.x * cy + zc.x * s,
                            O.y + X.y * cx + Y.y * cy + zc.y * s,
                            O.z + X.z * cx + Y.z * cy + zc.z * s};
    }
    return out;
  }
  /// (ρ,z) on tube A / B at minor angle v.
  std::pair<double, double> tubeA(double v) const {
    return {R1 + r1 * std::cos(v), zA + r1 * std::sin(v)};
  }
  std::pair<double, double> tubeB(double v) const {
    return {R2 + r2 * std::cos(v), zB + r2 * std::sin(v)};
  }
};

TorusTorusSetup torusTorusSetup(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  TorusTorusSetup st;
  if (seams.empty()) return st;                        // need ≥1 traced seam to cross-check
  if (A.kind != CurvedKind::Torus || B.kind != CurvedKind::Torus) return st;

  const math::Vec3 zc = A.frame.z.vec();
  const math::Point3 Oa = A.frame.origin;
  // Coaxial: parallel axes + torus-B centre colinear with torus-A axis.
  if (math::norm(math::cross(zc, B.frame.z.vec())) > 1e-6) return st;
  const math::Vec3 dOB{B.frame.origin.x - Oa.x, B.frame.origin.y - Oa.y, B.frame.origin.z - Oa.z};
  if (math::norm(dOB - zc * math::dot(dOB, zc)) > 1e-6) return st;  // B axis not colinear → OCCT

  const double R1 = A.radius, r1 = A.minorRadius, R2 = B.radius, r2 = B.minorRadius;
  if (!(r1 > 1e-9) || !(R1 > r1 + 1e-9)) return st;
  if (!(r2 > 1e-9) || !(R2 > r2 + 1e-9)) return st;
  // Canonical origin = torus-A centre projected onto the axis (O = Oa; both centres lie on it).
  const math::Point3 O = Oa;
  const double zA = 0.0;                       // torus-A centre station (origin at Oa)
  const double zB = math::dot(dOB, zc);        // torus-B centre station along the shared axis

  // Seam solve — two circles in the meridian plane: centre A=(R1,zA), B=(R2,zB).
  const double cr = R2 - R1, cz = zB - zA;
  const double D = std::hypot(cr, cz);
  if (!(D > 1e-9)) return st;                   // concentric tubes (same centre) → OCCT
  if (!(D < r1 + r2 - 1e-6)) return st;         // clear / tangent → no crossing → OCCT
  if (!(D > std::fabs(r1 - r2) + 1e-6)) return st;  // one tube inside the other → OCCT
  const double urho = cr / D, uz = cz / D;      // unit vector A→B in the meridian plane
  const double a = (D * D + r1 * r1 - r2 * r2) / (2.0 * D);   // chord offset from A along û
  const double h2 = r1 * r1 - a * a;
  if (!(h2 > 1e-12)) return st;                 // degenerate (tangent) → OCCT
  const double h = std::sqrt(h2);
  // Chord midpoint M = A + a·û; the two seam points = M ± h·û⊥ (û⊥ = (−uz, urho)).
  const double mrho = R1 + a * urho, mz = zA + a * uz;
  const double p1rho = mrho - h * uz, p1z = mz + h * urho;
  const double p2rho = mrho + h * uz, p2z = mz - h * urho;
  if (!(p1rho > 1e-9) || !(p2rho > 1e-9)) return st;  // a seam ring collapses on the axis → OCCT

  // Minor angles of each seam on each tube.
  const double vA1 = std::atan2((p1z - zA) / r1, (p1rho - R1) / r1);
  const double vA2 = std::atan2((p2z - zA) / r1, (p2rho - R1) / r1);
  const double vB1 = std::atan2((p1z - zB) / r2, (p1rho - R2) / r2);
  const double vB2 = std::atan2((p2z - zB) / r2, (p2rho - R2) / r2);
  auto vmidFwd = [&](double va, double vb) {
    double d2 = vb - va;
    while (d2 < 0.0) d2 += kSsiTwoPi;
    return va + 0.5 * d2;
  };
  // Tube A: the INNER (∩-side) arc is the one whose midpoint lies inside disk B.
  const double vmA = vmidFwd(vA1, vA2);
  const double rhoMA = R1 + r1 * std::cos(vmA), zMA = zA + r1 * std::sin(vmA);
  const bool innerFwdA = (rhoMA - R2) * (rhoMA - R2) + (zMA - zB) * (zMA - zB) <= r2 * r2;
  // Tube B: the INNER arc is the one whose midpoint lies inside disk A.
  const double vmB = vmidFwd(vB1, vB2);
  const double rhoMB = R2 + r2 * std::cos(vmB), zMB = zB + r2 * std::sin(vmB);
  const bool innerFwdB = (rhoMB - R1) * (rhoMB - R1) + (zMB - zA) * (zMB - zA) <= r1 * r1;

  // Cross-check EVERY traced seam against ONE of the two analytic circles (station + radius) —
  // never trust a missing / mis-placed co-resident loop.
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
    const bool m1 = std::fabs(sTr - p1z) < 1e-3 && std::fabs(rhoTr - p1rho) < 1e-3;
    const bool m2 = std::fabs(sTr - p2z) < 1e-3 && std::fabs(rhoTr - p2rho) < 1e-3;
    if (!m1 && !m2) return st;                  // traced seam matches neither analytic circle → OCCT
  }

  // Azimuth (N) + tube-arc (M) resolution, chord-bounded (matches the S5-l/n discretisation).
  const double kFacetSag = 0.004;
  const double rMax = std::max({p1rho, p2rho, R1 + r1, R2 + r2});
  const double chordA = std::sqrt(std::max(8.0 * kFacetSag * rMax, 1e-12));
  st.N = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rMax / chordA)), 32, 200);
  const double rMin = std::min(r1, r2);
  const double chordM = std::sqrt(std::max(8.0 * kFacetSag * rMin, 1e-12));
  st.M = std::clamp(static_cast<int>(std::ceil(kSsiTwoPi * rMin / chordM)), 24, 160);
  st.A = &A;
  st.B = &B;
  st.O = O;
  st.X = A.frame.x.vec();
  st.Y = A.frame.y.vec();
  st.zc = zc;
  st.R1 = R1; st.r1 = r1; st.zA = zA;
  st.R2 = R2; st.r2 = r2; st.zB = zB;
  st.rho1 = p1rho; st.z1 = p1z;
  st.rho2 = p2rho; st.z2 = p2z;
  st.vA1 = vA1; st.vA2 = vA2;
  st.vB1 = vB1; st.vB2 = vB2;
  st.innerFwdA = innerFwdA;
  st.innerFwdB = innerFwdB;
  st.ok = true;
  return st;
}

// A revolved tube arc between minor angles vA→vB on tube A / tube B (identical geometry to the
// S5-l appendTubeArc — each facet oriented by the TRUE tube-outward reference radiating from the
// tube-centre circle, scaled by outwardSign). `useTubeB` picks tube B's (R2,zB,r2), else tube A.
void appendTorusTorusTubeArc(const TorusTorusSetup& s, bool useTubeB, double vA, double vB,
                             double outwardSign, VertexPool& pool,
                             std::vector<topo::Shape>& faces) {
  const int m = s.M;
  const double Rmaj = useTubeB ? s.R2 : s.R1;
  const double zctr = useTubeB ? s.zB : s.zA;
  auto tubeCentreAt = [&](int i) {
    const double u = kSsiTwoPi * i / s.N;
    const double cx = Rmaj * std::cos(u), cy = Rmaj * std::sin(u);
    return math::Point3{s.O.x + s.X.x * cx + s.Y.x * cy + s.zc.x * zctr,
                        s.O.y + s.X.y * cx + s.Y.y * cy + s.zc.y * zctr,
                        s.O.z + s.X.z * cx + s.Y.z * cy + s.zc.z * zctr};
  };
  std::vector<math::Point3> prev;
  for (int k = 0; k <= m; ++k) {
    const double v = vA + (vB - vA) * (static_cast<double>(k) / m);
    const auto [rho, z] = useTubeB ? s.tubeB(v) : s.tubeA(v);
    std::vector<math::Point3> cur = s.ring(rho, z);
    if (k > 0) {
      const int n = s.N;
      for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        const math::Point3 tc = tubeCentreAt(i);
        const math::Point3 mid{(prev[i].x + prev[j].x + cur[j].x + cur[i].x) / 4,
                               (prev[i].y + prev[j].y + cur[j].y + cur[i].y) / 4,
                               (prev[i].z + prev[j].z + cur[j].z + cur[i].z) / 4};
        const math::Vec3 out{(mid.x - tc.x) * outwardSign, (mid.y - tc.y) * outwardSign,
                             (mid.z - tc.z) * outwardSign};
        pushPlanarTri(prev[i], prev[j], cur[j], out, pool, faces);
        pushPlanarTri(prev[i], cur[j], cur[i], out, pool, faces);
      }
    }
    prev = std::move(cur);
  }
}

// The INNER (∩-side) / OUTER (∪-side) tube arcs on tube A / B, honouring innerFwd, welded to the
// shared seam rings. seam1 is at minor angle v?1, seam2 at v?2; the forward sweep runs low→high.
void appendTorusTorusArc(const TorusTorusSetup& s, bool useTubeB, bool inner, double outwardSign,
                         VertexPool& pool, std::vector<topo::Shape>& faces) {
  const double v1 = useTubeB ? s.vB1 : s.vA1;
  const double v2 = useTubeB ? s.vB2 : s.vA2;
  const bool innerFwd = useTubeB ? s.innerFwdB : s.innerFwdA;
  // The INNER arc runs (innerFwd ? v1→v2 : v2→v1); the OUTER arc is the complement.
  double va, vb;
  if (inner) { va = innerFwd ? v1 : v2; vb = innerFwd ? v2 : v1; }
  else       { va = innerFwd ? v2 : v1; vb = innerFwd ? v1 : v2; }
  while (vb < va) vb += kSsiTwoPi;
  appendTorusTorusTubeArc(s, useTubeB, va, vb, outwardSign, pool, faces);
}

// buildTorusTorusCommon(A,B) = COMMON of two coaxial tori: the revolved LENS (disk A ∩ disk B) —
// the INNER arc of tube A (inside B) + the INNER arc of tube B (inside A), both outward. A closed
// watertight ring of revolution (no caps).
topo::Shape buildTorusTorusCommon(const CurvedSolid& A, const CurvedSolid& B,
                                  const std::vector<Seam>& seams) {
  const TorusTorusSetup s = torusTorusSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendTorusTorusArc(s, /*useTubeB=*/false, /*inner=*/true, /*outwardSign=*/1.0, pool, faces);
  appendTorusTorusArc(s, /*useTubeB=*/true, /*inner=*/true, /*outwardSign=*/1.0, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusTorusFuse(A,B) = A ∪ B: the revolved UNION (disk A ∪ disk B) — the OUTER arc of
// tube A (outside B) + the OUTER arc of tube B (outside A), both outward. A GROW; one ring of
// revolution. V = V_torusA + V_torusB − V_common.
topo::Shape buildTorusTorusFuse(const CurvedSolid& A, const CurvedSolid& B,
                                const std::vector<Seam>& seams) {
  const TorusTorusSetup s = torusTorusSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendTorusTorusArc(s, /*useTubeB=*/false, /*inner=*/false, /*outwardSign=*/1.0, pool, faces);
  appendTorusTorusArc(s, /*useTubeB=*/true, /*inner=*/false, /*outwardSign=*/1.0, pool, faces);
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// buildTorusTorusCut(A,B) = A − B with TORUS-A the minuend: disk A ∖ lens — the OUTER arc of
// tube A + the INNER arc of tube B REVERSED (inward normal, bounding the carved bite). A SHRINK,
// one closed ring of revolution. V = V_torusA − V_common. (B − A is built by swapping operands.)
topo::Shape buildTorusTorusCut(const CurvedSolid& A, const CurvedSolid& B,
                               const std::vector<Seam>& seams) {
  const TorusTorusSetup s = torusTorusSetup(A, B, seams);
  if (!s.ok) return {};
  VertexPool pool;
  std::vector<topo::Shape> faces;
  appendTorusTorusArc(s, /*useTubeB=*/false, /*inner=*/false, /*outwardSign=*/1.0, pool, faces);
  appendTorusTorusArc(s, /*useTubeB=*/true, /*inner=*/true, /*outwardSign=*/-1.0, pool, faces);
  if (faces.size() < 4) return {};
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
  // A TORUS is never a through-drill piercer/pierced wall — a coaxial torus∩{cyl,sphere}
  // two-circle poke-through has its OWN dedicated S5-l/m assembler. The sphere's polar (u,v)
  // parametrisation can make one of the two torus-latitude seam circles read NON-full on the
  // sphere (a tilted loop), spuriously matching the "one operand pierced" through-drill gate.
  // Decline any torus pair here so the S5-l/m builders (dispatched below) own it.
  if (A.kind == CurvedKind::Torus || B.kind == CurvedKind::Torus) return {};
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

// S5-r dispatch: exactly one operand a ring TORUS, the other a PLANE-ONLY half-space (a
// box/slab) whose single cutting plane is PERPENDICULAR to the torus axis. COMMON + CUT
// (torus minuend) land as a revolution; FUSE and every non-perpendicular / degenerate pose
// HONEST-DECLINE → NULL → OCCT. Returns NULL when the pair is not this family.
topo::Shape tryTorusHalfspace(const topo::Shape& a, const topo::Shape& b, Op op) {
  const std::optional<CurvedSolid> csA = recogniseCurvedSolid(a);
  const std::optional<CurvedSolid> csB = recogniseCurvedSolid(b);
  // Exactly one operand must be a recognised ring torus; the other must NOT be curved.
  const bool aTor = csA && csA->kind == CurvedKind::Torus;
  const bool bTor = csB && csB->kind == CurvedKind::Torus;
  if (aTor == bTor) return {};                 // need exactly one torus operand
  if (aTor && csB) return {};                  // the other is a curved solid → not a half-space
  if (bTor && csA) return {};
  const CurvedSolid& tor = aTor ? *csA : *csB;
  const topo::Shape& plane = aTor ? b : a;
  const std::optional<PlaneHalfspace> hs = recognisePlaneHalfspace(tor, plane);
  if (!hs) return {};
  const TorusPlaneSetup st = torusPlaneSetup(tor, *hs);
  if (!st.ok) return {};
  switch (op) {
    case Op::Common: return buildTorusPlaneCommon(st);
    case Op::Fuse:   return buildTorusPlaneFuse(st);
    case Op::Cut: {
      // CUT is order-sensitive: torus − half-space keeps the torus complement. A
      // half-space − torus minuend is out of this family's revolution scope → OCCT.
      if (!aTor) return {};  // torus must be operand A (the minuend)
      return buildTorusPlaneCut(st);
    }
  }
  return {};
}

}  // namespace

// ── The S5-a driver (design.md §Pipeline). Short, linear; the geometry lives in the
// detail helpers + buildCommon. Returns a candidate Solid or NULL; the ENGINE
// self-verifies (watertight + set-algebra volume) and owns the OCCT fallback. ──────
topo::Shape ssi_boolean_solid(const topo::Shape& a, const topo::Shape& b, Op op) {
  // 0. TORUS ∩ HALF-SPACE (S5-r): a plane-only operand is not a CurvedSolid, so it is
  // dispatched BEFORE the both-operands-curved gate. NULL when the pair is not this family.
  const topo::Shape torPlane = tryTorusHalfspace(a, b, op);
  if (!torPlane.isNull()) return torPlane;

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
      // S5-q: TRANSVERSAL (offset/non-coaxial) cone∩sphere COMMON (traced non-planar seams).
      const topo::Shape transConeSph = buildTransConeSphereCommon(*csA, *csB, seams);
      if (!transConeSph.isNull()) return transConeSph;
      // S5-s: TRANSVERSAL (offset/non-coaxial) cone∩cylinder COMMON (traced non-planar seams).
      const topo::Shape transConeCyl = buildTransConeCylCommon(*csA, *csB, seams);
      if (!transConeCyl.isNull()) return transConeCyl;
      // S5-i: TWO-CIRCLE coaxial cylinder∩sphere COMMON (sphere lower cap + cyl band + sphere upper cap).
      const topo::Shape cylSph2 = buildCylSphere2Common(*csA, *csB, seams);
      if (!cylSph2.isNull()) return cylSph2;
      // S5-k: TRANSVERSAL (offset/non-coaxial) cylinder∩sphere COMMON (traced non-planar seams).
      const topo::Shape transCS = buildTransCylSphereCommon(*csA, *csB, seams);
      if (!transCS.isNull()) return transCS;
      // S5-p: TRANSVERSAL (offset/non-coaxial) torus∩cylinder COMMON (traced non-planar seams).
      const topo::Shape transTC = buildTransTorusCylCommon(*csA, *csB, seams);
      if (!transTC.isNull()) return transTC;
      // S5-l: COAXIAL torus∩cylinder COMMON (inner tube arc + cylinder chord band).
      const topo::Shape torCyl = buildTorusCylCommon(*csA, *csB, seams);
      if (!torCyl.isNull()) return torCyl;
      // S5-m: COAXIAL torus∩sphere COMMON (inner tube arc + sphere zone between the seams).
      const topo::Shape torSph = buildTorusSphereCommon(*csA, *csB, seams);
      if (!torSph.isNull()) return torSph;
      // S5-n: COAXIAL torus∩cone COMMON (inner tube arc + slanted cone chord band).
      const topo::Shape torCone = buildTorusConeCommon(*csA, *csB, seams);
      if (!torCone.isNull()) return torCone;
      // S5-o: COAXIAL torus∩torus COMMON (revolved lens — inner arc of A + inner arc of B).
      const topo::Shape torTor = buildTorusTorusCommon(*csA, *csB, seams);
      if (!torTor.isNull()) return torTor;
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
      // S5-q: TRANSVERSAL cone∩sphere FUSE (honest-decline — sphere-outer-zone residual → OCCT).
      const topo::Shape transConeSph = buildTransConeSphereFuse(*csA, *csB, seams);
      if (!transConeSph.isNull()) return transConeSph;
      // S5-s: TRANSVERSAL cone∩cylinder FUSE (honest-decline — cone-outer-zone residual → OCCT).
      const topo::Shape transConeCyl = buildTransConeCylFuse(*csA, *csB, seams);
      if (!transConeCyl.isNull()) return transConeCyl;
      // S5-i: TWO-CIRCLE coaxial cylinder∩sphere FUSE (cyl walls + sphere zone bulge + discs).
      const topo::Shape cylSph2 = buildCylSphere2Fuse(*csA, *csB, seams);
      if (!cylSph2.isNull()) return cylSph2;
      // S5-k: TRANSVERSAL (offset) cylinder∩sphere FUSE (sphere outer shell + two cyl end stubs).
      const topo::Shape transCS = buildTransCylSphereFuse(*csA, *csB, seams);
      if (!transCS.isNull()) return transCS;
      // S5-p: TRANSVERSAL torus∩cylinder FUSE (honest-decline — torus-outer-zone residual → OCCT).
      const topo::Shape transTC = buildTransTorusCylFuse(*csA, *csB, seams);
      if (!transTC.isNull()) return transTC;
      // S5-l: COAXIAL torus∩cylinder FUSE (outer tube bulge + cyl wall outside the tube + discs).
      const topo::Shape torCyl = buildTorusCylFuse(*csA, *csB, seams);
      if (!torCyl.isNull()) return torCyl;
      // S5-m: COAXIAL torus∩sphere FUSE (outer tube bulge + two sphere polar caps).
      const topo::Shape torSph = buildTorusSphereFuse(*csA, *csB, seams);
      if (!torSph.isNull()) return torSph;
      // S5-n: COAXIAL torus∩cone FUSE (cone frustum fills the hole + outer tube bulge + cone discs).
      const topo::Shape torCone = buildTorusConeFuse(*csA, *csB, seams);
      if (!torCone.isNull()) return torCone;
      // S5-o: COAXIAL torus∩torus FUSE (revolved union — outer arc of A + outer arc of B).
      const topo::Shape torTor = buildTorusTorusFuse(*csA, *csB, seams);
      if (!torTor.isNull()) return torTor;
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
      // S5-q: TRANSVERSAL cone∩sphere CUT (honest-decline — sphere-outer-zone residual → OCCT).
      const topo::Shape transConeSph = buildTransConeSphereCut(*csA, *csB, seams);
      if (!transConeSph.isNull()) return transConeSph;
      // S5-s: TRANSVERSAL cone∩cylinder CUT (honest-decline — cone-outer-zone residual → OCCT).
      const topo::Shape transConeCyl = buildTransConeCylCut(*csA, *csB, seams);
      if (!transConeCyl.isNull()) return transConeCyl;
      // S5-i: TWO-CIRCLE coaxial cylinder∩sphere CUT (two disconnected cyl-end dimpled pieces).
      const topo::Shape cylSph2 = buildCylSphere2Cut(*csA, *csB, seams);
      if (!cylSph2.isNull()) return cylSph2;
      // S5-i reverse: coaxial sphere∩cylinder CUT (sphere − cyl: sphere belt + reversed bore tunnel).
      const topo::Shape sphCyl2 = buildSphereCyl2Cut(*csA, *csB, seams);
      if (!sphCyl2.isNull()) return sphCyl2;
      // S5-k: TRANSVERSAL (offset) cylinder∩sphere CUT (sphere − cyl: sphere shell + reversed bore).
      const topo::Shape transCS = buildTransCylSphereCut(*csA, *csB, seams);
      if (!transCS.isNull()) return transCS;
      // S5-p: TRANSVERSAL torus∩cylinder CUT (honest-decline — torus-outer-zone residual → OCCT).
      const topo::Shape transTC = buildTransTorusCylCut(*csA, *csB, seams);
      if (!transTC.isNull()) return transTC;
      // S5-l: COAXIAL torus∩cylinder CUT (torus − cyl: outer tube arc + reversed cylinder bore).
      const topo::Shape torCyl = buildTorusCylCut(*csA, *csB, seams);
      if (!torCyl.isNull()) return torCyl;
      // S5-l reverse: COAXIAL cylinder∩torus CUT (cyl − torus: cylinder with a toroidal groove).
      const topo::Shape cylTor = buildCylTorusCut(*csA, *csB, seams);
      if (!cylTor.isNull()) return cylTor;
      // S5-m: COAXIAL torus∩sphere CUT (torus − sphere: outer tube arc + reversed sphere zone).
      const topo::Shape torSph = buildTorusSphereCut(*csA, *csB, seams);
      if (!torSph.isNull()) return torSph;
      // S5-n: COAXIAL torus∩cone CUT (torus − cone: outer tube arc + reversed cone chord band).
      const topo::Shape torCone = buildTorusConeCut(*csA, *csB, seams);
      if (!torCone.isNull()) return torCone;
      // S5-o: COAXIAL torus∩torus CUT (A − B: outer arc of A + reversed inner arc of B).
      const topo::Shape torTor = buildTorusTorusCut(*csA, *csB, seams);
      if (!torTor.isNull()) return torTor;
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
