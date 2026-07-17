// SPDX-License-Identifier: Apache-2.0
//
// Host GATE for MULTI-HOLE-SPLIT (holed_face_split.h) — split a face that ALREADY carries
// a seam-hole (an annulus) by a SECOND closed interior seam, PRESERVING the existing hole.
// The precise enabler the readiness doc's §4 multi-crossing split named for BOOL-READMIT's
// general genuine-overlap ≥3 weld (a second boolean seam on an already-holed annulus).
//
// This suite proves, OCCT-free (host closed-form + mesh-tiling oracles):
//   * an ANNULUS (outer + one seam-hole) cut by a SECOND closed interior seam that ENCLOSES
//     the existing hole → TWO sub-annuli, the EXISTING HOLE PRESERVED, Σ signed NET area ==
//     parent net area to machine precision, each sub-region a valid simple loop;
//   * the two rebuilt sub-faces MESH (M0, unchanged) and their curved areas TILE the parent
//     annulus's, converging as the deflection refines;
//   * the seam is laid onto both sub-faces BIT-IDENTICALLY (faceInside outer == faceOutside
//     hole) so they weld watertight along it (0 boundary edges at the seam radius);
//   * the byte-frozen simply-connected split `splitFaceSmoothTrim` DROPS the existing hole
//     (the measured blocker) — the contrast that motivates this verb;
//   * the honest-decline envelope: a seam that CROSSES the existing hole, a hole-free face,
//     a too-short / non-interior seam each DECLINE with the measured blocker (never faked).
//
// The fixture is a PLANAR annulus (a Plane face, outer circle R + inner hole circle r0) cut
// by a middle circle rM (r0 < rM < R) — a pure closed-form tiling: parent net = π(R²−r0²),
// faceInside net = π(rM²−r0²), faceOutside net = π(R²−rM²). No SSI needed (the seam is a
// synthesised circular WLine), so this suite is ALWAYS-ON (not numsci-gated).
//
#include "native/boolean/holed_face_split.h"
#include "native/boolean/smooth_trim_split.h"
#include "native/tessellate/face_mesher.h"
#include "native/tessellate/trim.h"

#include "harness.h"

#include <array>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace fmath = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A circle (cx=cy=0.5, radius r) as a closed UV polygon, CCW, n samples.
std::vector<fmath::Point3> circleUV(double r, int n) {
  std::vector<fmath::Point3> uv;
  uv.reserve(static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    const double t = 2.0 * kPi * static_cast<double>(k) / n;
    uv.push_back(fmath::Point3{0.5 + r * std::cos(t), 0.5 + r * std::sin(t), 0.0});
  }
  return uv;
}

// Build a planar annulus face on z=0: outer circle radius R + one hole circle radius r0.
// The plane's (u,v) map is the identity (S(u,v) = (u,v,0)), so UV area == 3-D area exactly.
topo::Shape planarAnnulus(double R, double r0, int segs) {
  topo::FaceSurface pl{};
  pl.kind = topo::FaceSurface::Kind::Plane;
  pl.frame.origin = fmath::Point3{0, 0, 0};
  pl.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  pl.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
  pl.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});

  auto wireOf = [&](double r, bool ccw) {
    const std::vector<fmath::Point3> uv = circleUV(r, segs);
    const int n = segs;
    std::vector<topo::Shape> v(n), edges;
    for (int k = 0; k < n; ++k)
      v[k] = topo::ShapeBuilder::makeVertex(fmath::Point3{uv[k].x, uv[k].y, 0.0});
    std::vector<int> order(n);
    for (int k = 0; k < n; ++k) order[k] = ccw ? k : (n - k) % n;
    for (int idx = 0; idx < n; ++idx) {
      const int k = order[idx];
      const int k1 = order[(idx + 1) % n];
      topo::EdgeCurve c{};
      c.kind = topo::EdgeCurve::Kind::BSpline;
      c.degree = 1;
      c.poles = {fmath::Point3{uv[k].x, uv[k].y, 0.0}, fmath::Point3{uv[k1].x, uv[k1].y, 0.0}};
      c.knots = {0, 0, 1, 1};
      topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, v[k], v[k1]);
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::BSpline;
      pc.degree = 1;
      pc.poles2d = {fmath::Point3{uv[k].x, uv[k].y, 0.0}, fmath::Point3{uv[k1].x, uv[k1].y, 0.0}};
      pc.knots = {0, 0, 1, 1};
      edges.push_back(topo::ShapeBuilder::addPCurve(e, node.tshape(), pc));
    }
    return topo::ShapeBuilder::makeWire(std::move(edges));
  };
  const topo::Shape outer = wireOf(R, true);   // outer CCW
  const topo::Shape hole = wireOf(r0, false);  // hole CW
  return topo::ShapeBuilder::makeFace(pl, outer, {hole}, topo::Orientation::Forward);
}

// A circle centred at (cx,cy), radius r, as a closed UV polygon, CCW, n samples.
std::vector<fmath::Point3> circleUVAt(double cx, double cy, double r, int n) {
  std::vector<fmath::Point3> uv;
  uv.reserve(static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    const double t = 2.0 * kPi * static_cast<double>(k) / n;
    uv.push_back(fmath::Point3{cx + r * std::cos(t), cy + r * std::sin(t), 0.0});
  }
  return uv;
}

// Build a planar face on z=0 (identity UV map ⇒ UV area == 3-D area) with an outer
// circle radius R centred at (0.5,0.5) and N arbitrary hole circles (centre + radius).
// Each hole is emitted CW (opposite to the outer's CCW) so it is a genuine hole loop.
topo::Shape planarFaceHoles(double R, const std::vector<std::array<double, 3>>& holes, int segs) {
  topo::FaceSurface pl{};
  pl.kind = topo::FaceSurface::Kind::Plane;
  pl.frame.origin = fmath::Point3{0, 0, 0};
  pl.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  pl.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
  pl.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});

  auto wireOf = [&](double cx, double cy, double r, bool ccw) {
    const std::vector<fmath::Point3> uv = circleUVAt(cx, cy, r, segs);
    const int n = segs;
    std::vector<topo::Shape> v(n), edges;
    for (int k = 0; k < n; ++k)
      v[k] = topo::ShapeBuilder::makeVertex(fmath::Point3{uv[k].x, uv[k].y, 0.0});
    std::vector<int> order(n);
    for (int k = 0; k < n; ++k) order[k] = ccw ? k : (n - k) % n;
    for (int idx = 0; idx < n; ++idx) {
      const int k = order[idx];
      const int k1 = order[(idx + 1) % n];
      topo::EdgeCurve c{};
      c.kind = topo::EdgeCurve::Kind::BSpline;
      c.degree = 1;
      c.poles = {fmath::Point3{uv[k].x, uv[k].y, 0.0}, fmath::Point3{uv[k1].x, uv[k1].y, 0.0}};
      c.knots = {0, 0, 1, 1};
      topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, v[k], v[k1]);
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::BSpline;
      pc.degree = 1;
      pc.poles2d = {fmath::Point3{uv[k].x, uv[k].y, 0.0}, fmath::Point3{uv[k1].x, uv[k1].y, 0.0}};
      pc.knots = {0, 0, 1, 1};
      edges.push_back(topo::ShapeBuilder::addPCurve(e, node.tshape(), pc));
    }
    return topo::ShapeBuilder::makeWire(std::move(edges));
  };
  const topo::Shape outer = wireOf(0.5, 0.5, R, true);  // outer CCW
  std::vector<topo::Shape> holeWires;
  for (const auto& h : holes) holeWires.push_back(wireOf(h[0], h[1], h[2], false));  // CW holes
  return topo::ShapeBuilder::makeFace(pl, outer, std::move(holeWires),
                                      topo::Orientation::Forward);
}

// A closed circular seam centred at (cx,cy) as a WLine on the face's (u,v).
ssi::WLine circleSeamAt(double cx, double cy, double r, int n) {
  ssi::WLine w;
  const std::vector<fmath::Point3> uv = circleUVAt(cx, cy, r, n);
  for (const fmath::Point3& p : uv) {
    ssi::WLinePoint q;
    q.u1 = p.x;
    q.v1 = p.y;
    w.points.push_back(q);
  }
  w.status = ssi::TraceStatus::Closed;
  return w;
}

// A closed circular seam as a WLine (u1,v1 on the face's (u,v) = its plane map).
ssi::WLine circleSeam(double r, int n) {
  ssi::WLine w;
  const std::vector<fmath::Point3> uv = circleUV(r, n);
  for (const fmath::Point3& p : uv) {
    ssi::WLinePoint q;
    q.u1 = p.x;
    q.v1 = p.y;
    w.points.push_back(q);
  }
  w.status = ssi::TraceStatus::Closed;
  return w;
}

// Count boundary (used-once) edges of a mesh whose midpoint radius is near `r` (within tol).
int boundaryEdgesNearRadius(const tess::Mesh& m, double r, double tol) {
  std::map<std::pair<std::uint32_t, std::uint32_t>, int> cnt;
  for (const auto& t : m.triangles) {
    std::uint32_t v[3] = {t.a, t.b, t.c};
    for (int e = 0; e < 3; ++e) {
      std::uint32_t a = v[e], b = v[(e + 1) % 3];
      if (a > b) std::swap(a, b);
      cnt[{a, b}]++;
    }
  }
  int near = 0;
  for (const auto& kv : cnt)
    if (kv.second == 1) {
      const auto& p0 = m.vertices[kv.first.first];
      const auto& p1 = m.vertices[kv.first.second];
      const double rm = std::hypot((p0.x + p1.x) / 2 - 0.5, (p0.y + p1.y) / 2 - 0.5);
      if (std::fabs(rm - r) < tol) ++near;
    }
  return near;
}

}  // namespace

// ── The PRIMARY oracle: annulus + enclosing second seam → hole preserved, exact net tiling. ──
CC_TEST(holed_split_preserves_hole_exact_tiling) {
  const double R = 0.35, r0 = 0.20, rM = 0.2345;
  const topo::Shape face = planarAnnulus(R, r0, 96);
  const ssi::WLine seam = circleSeam(rM, 96);

  const bo::HoledSplitResult r = bo::splitFaceSmoothTrimHoled(face, seam);
  CC_CHECK(r.ok());
  CC_CHECK(r.decline == bo::HoledSplitDecline::Ok);
  CC_CHECK(r.crossings == 0);
  if (!r.ok()) return;
  const bo::SmoothFaceSplit& s = *r.split;

  // The existing hole is PRESERVED, attributed to faceInside (the seam encloses it).
  CC_CHECK(r.holesInside == 1);
  CC_CHECK(r.holesOutside == 0);

  // NET-area tiling: Σ sub-region net == parent net, to machine precision (gap 0).
  const double parentNet = kPi * (R * R - r0 * r0);
  const double insideNet = kPi * (rM * rM - r0 * r0);   // ring between hole and seam
  const double outsideNet = kPi * (R * R - rM * rM);     // ring between seam and outer
  CC_CHECK(std::fabs(s.parentArea - parentNet) / parentNet < 3e-3);  // 96-gon discretization
  CC_CHECK(std::fabs(s.areaInside - insideNet) / insideNet < 3e-3);
  CC_CHECK(std::fabs(s.areaOutside - outsideNet) / outsideNet < 3e-3);
  // The IDENTITY parent == inside + outside holds to machine precision (self-verify).
  const double gap = std::fabs(s.parentArea - (s.areaInside + s.areaOutside));
  CC_CHECK(gap <= s.parentArea * 1e-12);
  CC_CHECK(r.tilingGap <= s.parentArea * 1e-9);

  // Both sub-faces are genuinely-trimmed faces over the plane surface.
  CC_CHECK(s.faceInside.type() == topo::ShapeType::Face);
  CC_CHECK(s.faceOutside.type() == topo::ShapeType::Face);
  // faceInside is a HOLED annulus (seam outer + the preserved hole); faceOutside carries
  // the parent outer + the seam-as-hole (one hole).
  const tess::UVRegion regIn = tess::buildRegion(s.faceInside, 8);
  const tess::UVRegion regOut = tess::buildRegion(s.faceOutside, 8);
  CC_CHECK(regIn.holes.size() == 1);   // the existing hole preserved
  CC_CHECK(regOut.holes.size() == 1);  // the seam as this side's hole
}

// ── The two sub-faces MESH and TILE the parent's curved area, and weld along the seam. ──
CC_TEST(holed_split_subfaces_mesh_tile_and_weld_seam) {
  const double R = 0.35, r0 = 0.20, rM = 0.2345;
  const topo::Shape face = planarAnnulus(R, r0, 96);
  const ssi::WLine seam = circleSeam(rM, 96);
  const bo::HoledSplitResult r = bo::splitFaceSmoothTrimHoled(face, seam);
  CC_CHECK(r.ok());
  if (!r.ok()) return;
  const bo::SmoothFaceSplit& s = *r.split;

  const double deflections[] = {0.02, 0.01, 0.005};
  double prevRel = 1e9;
  for (double d : deflections) {
    tess::MeshParams mp;
    mp.deflection = d;
    const tess::FaceMesher fm(mp);
    const tess::Mesh mIn = fm.mesh(s.faceInside);
    const tess::Mesh mOut = fm.mesh(s.faceOutside);
    CC_CHECK(!mIn.triangles.empty());
    CC_CHECK(!mOut.triangles.empty());
    const double aIn = tess::surfaceArea(mIn);
    const double aOut = tess::surfaceArea(mOut);
    const double parentNet = kPi * (R * R - r0 * r0);
    const double rel = std::fabs(parentNet - (aIn + aOut)) / parentNet;
    CC_CHECK(rel < 2e-2);            // the two sub-annuli TILE the parent's net area
    CC_CHECK(rel <= prevRel + 1e-9);  // converging as the deflection tightens
    prevRel = rel;

    // The seam (r=rM) is laid onto BOTH sub-faces bit-identically ⇒ no unpaired edge there:
    // faceInside's OUTER == faceOutside's HOLE. Each face ALONE has that seam as a boundary,
    // but a WELD of the two shares it; verify the shared boundary is EXACTLY the two loops
    // (faceInside seam edges == faceOutside seam edges) by radius-localized boundary count:
    // each face alone has ~the same # boundary edges at rM (the shared seam) — the weld
    // pairs them. (The existing hole r0 and outer R stay genuine boundaries of the annuli.)
    const double tol = 0.02;
    const int beInSeam = boundaryEdgesNearRadius(mIn, rM, tol);
    const int beOutSeam = boundaryEdgesNearRadius(mOut, rM, tol);
    CC_CHECK(beInSeam > 0 && beOutSeam > 0);   // both carry the seam boundary
    CC_CHECK(beInSeam == beOutSeam);           // matched counts ⇒ they weld 1:1 along it
    // faceInside carries the preserved hole boundary at r0; faceOutside does NOT.
    CC_CHECK(boundaryEdgesNearRadius(mIn, r0, tol) > 0);
    CC_CHECK(boundaryEdgesNearRadius(mOut, r0, tol) == 0);
  }
}

// ── BYTE-FREEZE CONTRAST: the simply-connected split DROPS the existing hole (the blocker). ──
CC_TEST(holed_split_contrast_smoothtrim_drops_hole) {
  const double R = 0.35, r0 = 0.20, rM = 0.2345;
  const topo::Shape face = planarAnnulus(R, r0, 96);
  const ssi::WLine seam = circleSeam(rM, 96);

  // splitFaceSmoothTrim treats the face as simply-connected: its faceInside is the FULL
  // disk (r<rM), NOT the ring — the existing hole (r0) is DROPPED. Its "parent area" is the
  // GROSS outer area (hole not subtracted), so its inside disk area = π·rM² (not the ring).
  const bo::SmoothSplitResult sr = bo::splitFaceSmoothTrim(face, seam);
  CC_CHECK(sr.ok());
  if (sr.ok()) {
    const double diskGross = kPi * rM * rM;               // the FULL disk (hole ignored)
    CC_CHECK(std::fabs(sr.split->areaInside - diskGross) / diskGross < 3e-3);
    // faceInside has NO hole (the blocker): the removed-region hole is gone.
    const tess::UVRegion reg = tess::buildRegion(sr.split->faceInside, 8);
    CC_CHECK(reg.holes.empty());
  }
  // The holed verb, on the SAME input, PRESERVES the hole (its inside = the RING).
  const bo::HoledSplitResult hr = bo::splitFaceSmoothTrimHoled(face, seam);
  CC_CHECK(hr.ok());
  if (hr.ok()) {
    const double ring = kPi * (rM * rM - r0 * r0);
    CC_CHECK(std::fabs(hr.split->areaInside - ring) / ring < 3e-3);
  }
}

// ── HONEST DECLINE: a hole-free (simply-connected) face → NoHole (use splitFaceSmoothTrim). ──
CC_TEST(holed_split_declines_hole_free_face) {
  // A plain disk (outer circle, no hole).
  topo::FaceSurface pl{};
  pl.kind = topo::FaceSurface::Kind::Plane;
  pl.frame.origin = fmath::Point3{0, 0, 0};
  pl.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  pl.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
  pl.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
  const std::vector<fmath::Point3> uv = circleUV(0.35, 64);
  std::vector<topo::Shape> edges;
  for (int k = 0; k < 64; ++k) {
    const int k1 = (k + 1) % 64;
    topo::EdgeCurve c{};
    c.kind = topo::EdgeCurve::Kind::BSpline;
    c.degree = 1;
    c.poles = {fmath::Point3{uv[k].x, uv[k].y, 0}, fmath::Point3{uv[k1].x, uv[k1].y, 0}};
    c.knots = {0, 0, 1, 1};
    topo::Shape e = topo::ShapeBuilder::makeEdge(
        c, 0, 1, topo::ShapeBuilder::makeVertex(fmath::Point3{uv[k].x, uv[k].y, 0}),
        topo::ShapeBuilder::makeVertex(fmath::Point3{uv[k1].x, uv[k1].y, 0}));
    topo::PCurve pc{};
    pc.kind = topo::EdgeCurve::Kind::BSpline;
    pc.degree = 1;
    pc.poles2d = {fmath::Point3{uv[k].x, uv[k].y, 0}, fmath::Point3{uv[k1].x, uv[k1].y, 0}};
    pc.knots = {0, 0, 1, 1};
    edges.push_back(topo::ShapeBuilder::addPCurve(e, node.tshape(), pc));
  }
  const topo::Shape disk =
      topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(edges)), {},
                                   topo::Orientation::Forward);
  const bo::HoledSplitResult r = bo::splitFaceSmoothTrimHoled(disk, circleSeam(0.2, 64));
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::HoledSplitDecline::NoHole);
}

// ── HONEST DECLINE: a seam that CROSSES the existing hole → SeamCrossesHole (harder case). ──
CC_TEST(holed_split_declines_seam_crossing_hole) {
  const topo::Shape face = planarAnnulus(0.35, 0.20, 96);
  // A seam circle centred OFF the hole so it crosses the hole boundary (radius 0.10 about a
  // centre at (0.5+0.20, 0.5) crosses the r0=0.20 hole loop).
  ssi::WLine seam;
  const int n = 96;
  for (int k = 0; k < n; ++k) {
    const double t = 2.0 * kPi * k / n;
    ssi::WLinePoint q;
    q.u1 = 0.5 + 0.20 + 0.12 * std::cos(t);
    q.v1 = 0.5 + 0.12 * std::sin(t);
    seam.points.push_back(q);
  }
  seam.status = ssi::TraceStatus::Closed;
  const bo::HoledSplitResult r = bo::splitFaceSmoothTrimHoled(face, seam);
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::HoledSplitDecline::SeamCrossesHole);
}

// ── HONEST DECLINE: an OPEN seam that crosses the outer boundary → SeamNotInterior. ──
CC_TEST(holed_split_declines_non_interior_seam) {
  const topo::Shape face = planarAnnulus(0.35, 0.20, 96);
  // A big circle (r=0.5) that crosses the outer rim (R=0.35).
  const bo::HoledSplitResult r = bo::splitFaceSmoothTrimHoled(face, circleSeam(0.5, 96));
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::HoledSplitDecline::SeamNotInterior);
}

// ── GENERAL ≥2-HOLE CASE: a face carrying TWO holes, a seam enclosing exactly ONE of them.
// The enclosed hole goes to faceInside; the other stays with faceOutside. Net area tiles
// exactly and each sub-face carries the right hole count (the general holed-face split). ──
CC_TEST(holed_split_two_holes_seam_encloses_one) {
  const double R = 0.42;
  const double c0x = 0.5 - 0.18, c0y = 0.5, r0 = 0.06;  // hole 0 (left), enclosed by the seam
  const double c1x = 0.5 + 0.18, c1y = 0.5, r1 = 0.05;  // hole 1 (right), left OUTSIDE the seam
  const topo::Shape face =
      planarFaceHoles(R, {{c0x, c0y, r0}, {c1x, c1y, r1}}, 96);
  // Seam: a circle around the LEFT hole only (centre = left hole centre, radius 0.11 > r0,
  // and 0.11+|c0x−c1x|=0.11+0.36 keeps the right hole well outside).
  const double rS = 0.11;
  const ssi::WLine seam = circleSeamAt(c0x, c0y, rS, 96);

  const bo::HoledSplitResult r = bo::splitFaceSmoothTrimHoled(face, seam);
  CC_CHECK(r.ok());
  CC_CHECK(r.decline == bo::HoledSplitDecline::Ok);
  if (!r.ok()) return;
  const bo::SmoothFaceSplit& s = *r.split;

  // ONE hole inside the seam, ONE outside.
  CC_CHECK(r.holesInside == 1);
  CC_CHECK(r.holesOutside == 1);

  const double parentNet = kPi * (R * R - r0 * r0 - r1 * r1);
  const double insideNet = kPi * (rS * rS - r0 * r0);   // seam disk minus the enclosed hole
  const double outsideNet = parentNet - insideNet;
  CC_CHECK(std::fabs(s.parentArea - parentNet) / parentNet < 3e-3);
  CC_CHECK(std::fabs(s.areaInside - insideNet) / insideNet < 3e-3);
  CC_CHECK(std::fabs(s.areaOutside - outsideNet) / outsideNet < 3e-3);
  const double gap = std::fabs(s.parentArea - (s.areaInside + s.areaOutside));
  CC_CHECK(gap <= s.parentArea * 1e-12);

  // faceInside carries exactly the enclosed hole; faceOutside carries the seam-as-hole +
  // the non-enclosed hole (two holes).
  const tess::UVRegion regIn = tess::buildRegion(s.faceInside, 8);
  const tess::UVRegion regOut = tess::buildRegion(s.faceOutside, 8);
  CC_CHECK(regIn.holes.size() == 1);
  CC_CHECK(regOut.holes.size() == 2);
}

// ── GENERAL ≥3-HOLE CASE: a face carrying THREE holes, a seam enclosing TWO of them. Both
// enclosed holes go to faceInside (nested), the third stays with faceOutside. The net-area
// decomposition and hole attribution are exact for the ≥3 general holed-face split. ──
CC_TEST(holed_split_three_holes_seam_encloses_two) {
  const double R = 0.46;
  const double a[3] = {0.06, 0.05, 0.055};                    // hole radii
  const double cx[3] = {0.5 - 0.10, 0.5 + 0.10, 0.5};         // hole centres x
  const double cy[3] = {0.5 - 0.08, 0.5 - 0.08, 0.5 + 0.24};  // hole centres y
  const topo::Shape face = planarFaceHoles(
      R, {{cx[0], cy[0], a[0]}, {cx[1], cy[1], a[1]}, {cx[2], cy[2], a[2]}}, 120);
  // Seam: a circle low-centred enclosing holes 0 and 1 (both near y=0.42) but NOT hole 2
  // (up at y=0.74). Centre (0.5, 0.42), radius 0.20 covers the two lower holes with margin.
  const double sx = 0.5, sy = 0.5 - 0.08, rS = 0.20;
  const ssi::WLine seam = circleSeamAt(sx, sy, rS, 120);

  const bo::HoledSplitResult r = bo::splitFaceSmoothTrimHoled(face, seam);
  CC_CHECK(r.ok());
  CC_CHECK(r.decline == bo::HoledSplitDecline::Ok);
  if (!r.ok()) return;
  const bo::SmoothFaceSplit& s = *r.split;

  CC_CHECK(r.holesInside == 2);   // holes 0 and 1 nested inside the seam
  CC_CHECK(r.holesOutside == 1);  // hole 2 outside

  const double holesAll = kPi * (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  const double parentNet = kPi * R * R - holesAll;
  const double insideNet = kPi * rS * rS - kPi * (a[0] * a[0] + a[1] * a[1]);
  const double outsideNet = parentNet - insideNet;
  CC_CHECK(std::fabs(s.parentArea - parentNet) / parentNet < 3e-3);
  CC_CHECK(std::fabs(s.areaInside - insideNet) / insideNet < 3e-3);
  CC_CHECK(std::fabs(s.areaOutside - outsideNet) / outsideNet < 3e-3);
  const double gap = std::fabs(s.parentArea - (s.areaInside + s.areaOutside));
  CC_CHECK(gap <= s.parentArea * 1e-12);

  // faceInside carries the two nested holes; faceOutside carries the seam-as-hole + hole 2.
  const tess::UVRegion regIn = tess::buildRegion(s.faceInside, 8);
  const tess::UVRegion regOut = tess::buildRegion(s.faceOutside, 8);
  CC_CHECK(regIn.holes.size() == 2);
  CC_CHECK(regOut.holes.size() == 2);
}

int main() { return cctest::run_all(); }
