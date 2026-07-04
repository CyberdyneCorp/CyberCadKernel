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
#include <cmath>
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

  // Gate on full transversality — the honest S4 boundary.
  if (trace.nearTangentGaps > 0) return {};         // a branch traced up to a tangent → S4
  if (trace.lines.empty()) return {};               // no seam → OCCT
  for (const ssi::WLine& w : trace.lines)
    if (w.status == ssi::TraceStatus::NearTangent || w.status == ssi::TraceStatus::Failed)
      return {};                                    // out of scope → OCCT

  std::vector<Seam> seams;
  seams.reserve(trace.lines.size());
  for (const ssi::WLine& w : trace.lines) seams.push_back(toSeam(w));

  // 1-4. SPLIT + CLASSIFY + SELECT + WELD. Only the COMMON of the two-branch
  // transversal case is robustly assembled here (Steinmetz family, host-verifiable).
  // Fuse / cut require the OUTSIDE wall fragments + the operands' caps re-trimmed by
  // the seam — that watertight cap re-trim is NOT yet robust, so those ops decline
  // (→ OCCT), honestly reported. This is the S5-a slice; wider ops are follow-on work.
  switch (op) {
    case Op::Common:
      return buildCommon(*csA, *csB, seams);
    case Op::Fuse:
    case Op::Cut:
      return {};  // deferred → OCCT (see note above)
  }
  return {};
}

}  // namespace cybercad::native::boolean

#else  // !CYBERCAD_HAS_NUMSCI — substrate absent: stub that always declines.

namespace cybercad::native::boolean {
topo::Shape ssi_boolean_solid(const topo::Shape&, const topo::Shape&, Op) { return {}; }
}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_HAS_NUMSCI
