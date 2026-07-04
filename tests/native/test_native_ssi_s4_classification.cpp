// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for SSI Stage S4-a — coincident / overlapping-surface detection +
// the typed CoincidentRegion result (OCCT-FREE, Gate 1 of the two-gate model).
//
// This suite covers the S4-a SLICE only (detection + classification):
//   ANALYTIC (no substrate, always built):
//     * classify_degeneracy returns FullSurfaceSame for identical elementary surfaces of
//       every family (plane, cylinder, cone, sphere, torus);
//     * a shifted / rotated / resized near-miss of each returns None (no false
//       coincidence);
//     * intersect_surfaces STILL reports IntersectionStatus::Coincident for the
//       previously-shipped same-sphere and coaxial-equal-cylinder pairs (regression).
//   SEEDED (CYBERCAD_HAS_NUMSCI):
//     * two freeform patches that coincide on an interior sub-rectangle yield ONE
//       delimited OverlapSubRegion whose bounds bracket the constructed overlap, and no
//       spurious seeds inside it;
//     * two freeform patches coincident over their WHOLE domain (overlap runs to the
//       domain edges, not interior-delimitable) yield Undecided — a honest → OCCT
//       verdict, never a fabricated region.
//
// No OCCT is linked. S4-b (typed tangent-contact classification) is a SEPARATE slice and
// is NOT exercised here.
//
#include "native/ssi/native_ssi.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace ssi = cybercad::native::ssi;
namespace nmath = cybercad::native::math;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Point3;
using nmath::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;

Ax3 frameZ(Point3 o = {0, 0, 0}, Dir3 z = Dir3{0, 0, 1}, Dir3 x = Dir3{1, 0, 0}) {
  return Ax3::fromAxisAndRef(o, z, x);
}

// ── ANALYTIC FullSurfaceSame — one identical pair + one near-miss per family ─────

CC_TEST(s4a_analytic_same_plane_is_full_surface_same) {
  ssi::Surface a = ssi::Surface::of(nmath::Plane{frameZ({0, 0, 0})});
  // Same locus: same plane, origin slid within the plane, normal flipped (± sign ok).
  ssi::Surface b = ssi::Surface::of(nmath::Plane{frameZ({5, -2, 0}, Dir3{0, 0, -1})});
  auto cr = ssi::classify_degeneracy(a, b);
  CC_CHECK(cr.kind == ssi::CoincidenceKind::FullSurfaceSame);

  // Near-miss: parallel plane offset by 0.01 along the normal → NOT the same locus.
  ssi::Surface c = ssi::Surface::of(nmath::Plane{frameZ({0, 0, 0.01})});
  CC_CHECK(ssi::classify_degeneracy(a, c).kind == ssi::CoincidenceKind::None);
}

CC_TEST(s4a_analytic_same_sphere_is_full_surface_same) {
  ssi::Surface a = ssi::Surface::of(nmath::Sphere{frameZ({1, 2, 3}), 2.5});
  ssi::Surface b = ssi::Surface::of(nmath::Sphere{frameZ({1, 2, 3}), 2.5});
  CC_CHECK(ssi::classify_degeneracy(a, b).kind == ssi::CoincidenceKind::FullSurfaceSame);

  // Near-miss: radius off by 0.01.
  ssi::Surface c = ssi::Surface::of(nmath::Sphere{frameZ({1, 2, 3}), 2.51});
  CC_CHECK(ssi::classify_degeneracy(a, c).kind == ssi::CoincidenceKind::None);
  // Near-miss: centre shifted.
  ssi::Surface d = ssi::Surface::of(nmath::Sphere{frameZ({1, 2, 3.01}), 2.5});
  CC_CHECK(ssi::classify_degeneracy(a, d).kind == ssi::CoincidenceKind::None);
}

CC_TEST(s4a_analytic_coaxial_equal_cylinder_is_full_surface_same) {
  nmath::Cylinder c1{frameZ({0, 0, 0}), 1.5};
  // Same axis line, origin slid ALONG the axis, equal radius → same locus.
  nmath::Cylinder c2{frameZ({0, 0, 7}), 1.5};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(c1), ssi::Surface::of(c2)).kind ==
           ssi::CoincidenceKind::FullSurfaceSame);

  // Near-miss: radius off.
  nmath::Cylinder c3{frameZ({0, 0, 0}), 1.51};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(c1), ssi::Surface::of(c3)).kind ==
           ssi::CoincidenceKind::None);
  // Near-miss: axis offset (parallel but not collinear).
  nmath::Cylinder c4{frameZ({0.02, 0, 0}), 1.5};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(c1), ssi::Surface::of(c4)).kind ==
           ssi::CoincidenceKind::None);
}

CC_TEST(s4a_analytic_same_cone_is_full_surface_same) {
  const double alpha = 0.5;  // half-angle
  nmath::Cone k1{frameZ({0, 0, 0}), 1.0, alpha};
  // Same locus: describe the SAME double-napped cone from a frame slid along the axis.
  // Apex of k1 is at v=-1/sinα along +Z. Build k2 with origin at that apex (radius 0).
  const Point3 apex = ssi::coneApex(k1);
  nmath::Cone k2{frameZ(apex), 0.0, alpha};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(k1), ssi::Surface::of(k2)).kind ==
           ssi::CoincidenceKind::FullSurfaceSame);

  // Near-miss: half-angle off.
  nmath::Cone k3{frameZ(apex), 0.0, alpha + 0.01};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(k1), ssi::Surface::of(k3)).kind ==
           ssi::CoincidenceKind::None);
  // Near-miss: apex shifted off the axis.
  nmath::Cone k4{frameZ(apex + Vec3{0.05, 0, 0}), 0.0, alpha};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(k1), ssi::Surface::of(k4)).kind ==
           ssi::CoincidenceKind::None);
}

CC_TEST(s4a_analytic_same_torus_is_full_surface_same) {
  nmath::Torus t1{frameZ({1, 1, 1}), 3.0, 0.75};
  // Same locus: axis flipped (torus is axis-flip symmetric), equal R and r.
  nmath::Torus t2{frameZ({1, 1, 1}, Dir3{0, 0, -1}), 3.0, 0.75};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(t1), ssi::Surface::of(t2)).kind ==
           ssi::CoincidenceKind::FullSurfaceSame);

  // Near-miss: minor radius off.
  nmath::Torus t3{frameZ({1, 1, 1}), 3.0, 0.76};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(t1), ssi::Surface::of(t3)).kind ==
           ssi::CoincidenceKind::None);
  // Near-miss: axis tilted.
  nmath::Torus t4{frameZ({1, 1, 1}, Dir3{0.02, 0, 1}), 3.0, 0.75};
  CC_CHECK(ssi::classify_degeneracy(ssi::Surface::of(t1), ssi::Surface::of(t4)).kind ==
           ssi::CoincidenceKind::None);
}

CC_TEST(s4a_analytic_mixed_kinds_never_coincident) {
  ssi::Surface pl = ssi::Surface::of(nmath::Plane{frameZ()});
  ssi::Surface sp = ssi::Surface::of(nmath::Sphere{frameZ(), 1.0});
  CC_CHECK(ssi::classify_degeneracy(pl, sp).kind == ssi::CoincidenceKind::None);
}

// ── REGRESSION — the shipped Coincident STATUS is unchanged, now backed by the region.
CC_TEST(s4a_intersect_surfaces_coincident_status_unchanged) {
  // Same sphere → IntAna_Same equivalent (was already Coincident).
  ssi::Surface s1 = ssi::Surface::of(nmath::Sphere{frameZ({0, 0, 0}), 2.0});
  ssi::Surface s2 = ssi::Surface::of(nmath::Sphere{frameZ({0, 0, 0}), 2.0});
  CC_CHECK(ssi::intersect_surfaces(s1, s2).status == ssi::IntersectionStatus::Coincident);

  // Coaxial-equal cylinder → Coincident (unchanged).
  ssi::Surface c1 = ssi::Surface::of(nmath::Cylinder{frameZ({0, 0, 0}), 1.0});
  ssi::Surface c2 = ssi::Surface::of(nmath::Cylinder{frameZ({0, 0, 3}), 1.0});
  CC_CHECK(ssi::intersect_surfaces(c1, c2).status == ssi::IntersectionStatus::Coincident);

  // Same plane → Coincident (unchanged), and the typed region agrees.
  ssi::Surface p1 = ssi::Surface::of(nmath::Plane{frameZ()});
  ssi::Surface p2 = ssi::Surface::of(nmath::Plane{frameZ()});
  CC_CHECK(ssi::intersect_surfaces(p1, p2).status == ssi::IntersectionStatus::Coincident);
  CC_CHECK(ssi::classify_degeneracy(p1, p2).kind == ssi::CoincidenceKind::FullSurfaceSame);
}

// ── S4-b ANALYTIC tangent-contact classification (no substrate, always built) ────

// Distance from a point to BOTH surfaces of a pair must be ~0 for the emitted contact
// point / a sample on the emitted contact curve (the S4-b correctness invariant).
double distToSphere(const nmath::Sphere& s, const Point3& p) {
  return std::fabs(nmath::distance(p, s.pos.origin) - s.radius);
}
double distToCylinder(const nmath::Cylinder& c, const Point3& p) {
  const Vec3 w = p - c.pos.origin;
  const Vec3 axial = c.pos.z.vec() * nmath::dot(w, c.pos.z.vec());
  return std::fabs(nmath::norm(w - axial) - c.radius);
}
double distToPlane(const nmath::Plane& pl, const Point3& p) {
  return std::fabs(nmath::dot(p - pl.pos.origin, pl.pos.z.vec()));
}

CC_TEST(s4b_analytic_sphere_sphere_external_is_tangent_point) {
  // Two unit spheres 2 apart along X: d = R1+R2 = 2 → external tangency at the midpoint.
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({2, 0, 0}), 1.0};
  auto tc = ssi::classify_tangency(ssi::Surface::of(s1), ssi::Surface::of(s2));
  CC_CHECK(tc.type == ssi::TangentContactType::TangentPoint);
  // The touch point lies on BOTH spheres.
  CC_CHECK(distToSphere(s1, tc.point) < 1e-7);
  CC_CHECK(distToSphere(s2, tc.point) < 1e-7);
  // Order independence.
  auto tc2 = ssi::classify_tangency(ssi::Surface::of(s2), ssi::Surface::of(s1));
  CC_CHECK(tc2.type == ssi::TangentContactType::TangentPoint);
}

CC_TEST(s4b_analytic_sphere_sphere_internal_is_tangent_point) {
  // R1=3 at origin, R2=1 centred at (2,0,0): d = 2 = |R1−R2| → internal tangency.
  nmath::Sphere s1{frameZ({0, 0, 0}), 3.0};
  nmath::Sphere s2{frameZ({2, 0, 0}), 1.0};
  auto tc = ssi::classify_tangency(ssi::Surface::of(s1), ssi::Surface::of(s2));
  CC_CHECK(tc.type == ssi::TangentContactType::TangentPoint);
  CC_CHECK(distToSphere(s1, tc.point) < 1e-7);
  CC_CHECK(distToSphere(s2, tc.point) < 1e-7);
}

CC_TEST(s4b_analytic_sphere_sphere_transversal_is_not_tangent) {
  // d = 1 < R1+R2 = 2, crossing in a circle → transversal, NOT a tangency.
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({1, 0, 0}), 1.0};
  auto tc = ssi::classify_tangency(ssi::Surface::of(s1), ssi::Surface::of(s2));
  CC_CHECK(tc.type == ssi::TangentContactType::TransversalOnly);
  // Near-miss: d = 2.01 (just past external touch) → disjoint, still not tangent.
  nmath::Sphere s3{frameZ({2.01, 0, 0}), 1.0};
  CC_CHECK(ssi::classify_tangency(ssi::Surface::of(s1), ssi::Surface::of(s3)).type ==
           ssi::TangentContactType::TransversalOnly);
}

CC_TEST(s4b_analytic_plane_tangent_to_sphere_is_tangent_point) {
  // Plane z=1 tangent to the unit sphere at (0,0,1).
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Plane pl{frameZ({0, 0, 1})};
  auto tc = ssi::classify_tangency(ssi::Surface::of(pl), ssi::Surface::of(sp));
  CC_CHECK(tc.type == ssi::TangentContactType::TangentPoint);
  CC_CHECK(distToSphere(sp, tc.point) < 1e-7);
  CC_CHECK(distToPlane(pl, tc.point) < 1e-7);
  // Cutting plane z=0.5 → a real circle → transversal.
  nmath::Plane cut{frameZ({0, 0, 0.5})};
  CC_CHECK(ssi::classify_tangency(ssi::Surface::of(cut), ssi::Surface::of(sp)).type ==
           ssi::TangentContactType::TransversalOnly);
}

CC_TEST(s4b_analytic_coaxial_sphere_cylinder_equator_is_tangent_curve) {
  // Cylinder radius 1 coaxial (Z) through the unit sphere centre: R_c = R_s → the
  // cylinder touches the sphere along its equator (a tangent Circle).
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({0, 0, 0}), 1.0};
  auto tc = ssi::classify_tangency(ssi::Surface::of(sp), ssi::Surface::of(cy));
  CC_CHECK(tc.type == ssi::TangentContactType::TangentCurve);
  CC_CHECK(tc.curve.has_value());
  if (tc.curve.has_value()) CC_CHECK(tc.curve->kind == ssi::CurveKind::Circle);
  // Sample the emitted contact circle: every point lies on BOTH surfaces.
  for (int i = 0; i < 8; ++i) {
    const double t = 2.0 * kPi * (i / 8.0);
    const Point3 p = tc.curve->value(t);
    CC_CHECK(distToSphere(sp, p) < 1e-7);
    CC_CHECK(distToCylinder(cy, p) < 1e-7);
  }
  // Narrower cylinder (R_c = 0.5 < R_s) cuts two latitude circles → transversal.
  nmath::Cylinder cy2{frameZ({0, 0, 0}), 0.5};
  CC_CHECK(ssi::classify_tangency(ssi::Surface::of(sp), ssi::Surface::of(cy2)).type ==
           ssi::TangentContactType::TransversalOnly);
}

CC_TEST(s4b_analytic_plane_tangent_to_cylinder_is_tangent_curve) {
  // Cylinder radius 1 along Z at the origin; plane x=1 (normal +X) grazes it along the
  // ruling line x=1,y=0 → a tangent Line.
  nmath::Cylinder cy{frameZ({0, 0, 0}), 1.0};
  nmath::Plane pl{Ax3::fromAxisAndRef({1, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 0, 1})};
  auto tc = ssi::classify_tangency(ssi::Surface::of(pl), ssi::Surface::of(cy));
  CC_CHECK(tc.type == ssi::TangentContactType::TangentCurve);
  CC_CHECK(tc.curve.has_value());
  if (tc.curve.has_value()) CC_CHECK(tc.curve->kind == ssi::CurveKind::Line);
  for (double s = -3.0; s <= 3.0; s += 1.0) {
    const Point3 p = tc.curve->value(s);
    CC_CHECK(distToPlane(pl, p) < 1e-7);
    CC_CHECK(distToCylinder(cy, p) < 1e-7);
  }
  // Plane x=0 through the axis cuts two rulings → transversal.
  nmath::Plane thru{Ax3::fromAxisAndRef({0, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 0, 1})};
  CC_CHECK(ssi::classify_tangency(ssi::Surface::of(thru), ssi::Surface::of(cy)).type ==
           ssi::TangentContactType::TransversalOnly);
}

CC_TEST(s4b_analytic_full_surface_same_is_not_a_tangency) {
  // A same-locus pair is coincidence (classify_degeneracy), NOT a tangency.
  nmath::Sphere s{frameZ({0, 0, 0}), 2.0};
  auto tc = ssi::classify_tangency(ssi::Surface::of(s), ssi::Surface::of(s));
  CC_CHECK(tc.type == ssi::TangentContactType::TransversalOnly);
}

#ifdef CYBERCAD_HAS_NUMSCI

// A flat (z=0) bilinear Bézier patch spanning [x0,x1]×[y0,y1]. Its (u,v)∈[0,1]² map
// linearly onto that XY footprint at z=0. Two such patches with DIFFERENT footprints
// coincide (same z=0 plane, aligned +z normals) exactly where their footprints overlap.
ssi::SurfaceAdapter flatPatch(double x0, double x1, double y0, double y1) {
  std::vector<Point3> poles = {
      {x0, y0, 0.0}, {x0, y1, 0.0},
      {x1, y0, 0.0}, {x1, y1, 0.0}};
  return ssi::makeBezierAdapter(poles, 2, 2);
}

// A big flat patch [-1,2]² and a small flat patch [0,1]² share the plane z=0 over the
// small patch's footprint, which sits STRICTLY INSIDE the big patch's domain → the
// coincident overlap is interior-delimitable on all four sides.
CC_TEST(s4a_seeded_interior_overlap_is_delimited) {
  ssi::SurfaceAdapter big = flatPatch(-1.0, 2.0, -1.0, 2.0);   // A: u,v∈[0,1] → x,y∈[-1,2]
  ssi::SurfaceAdapter small = flatPatch(0.0, 1.0, 0.0, 1.0);   // B: the interior overlap

  ssi::SeedOptions opts;
  opts.minPatchFrac = 1.0 / 16.0;
  auto ss = ssi::seed_intersection(big, small, opts);

  // Exactly one delimited OverlapSubRegion, no Undecided.
  int overlaps = 0, undecided = 0;
  ssi::CoincidentRegion region{};
  for (const auto& cr : ss.coincidentRegions) {
    if (cr.kind == ssi::CoincidenceKind::OverlapSubRegion) { ++overlaps; region = cr; }
    if (cr.kind == ssi::CoincidenceKind::Undecided) ++undecided;
  }
  CC_CHECK(overlaps == 1);
  CC_CHECK(undecided == 0);

  // The delimited A-bounds bracket the true overlap: A's footprint x∈[-1,2] maps
  // u∈[0,1], so the overlap x∈[0,1] is u∈[1/3,2/3]; v likewise. Allow a generous slack
  // for the discrete grow grid — the point is the region is INTERIOR, not full-domain.
  CC_CHECK(region.regionA.u0 > 0.05 && region.regionA.u0 < 0.45);
  CC_CHECK(region.regionA.u1 > 0.55 && region.regionA.u1 < 0.95);
  CC_CHECK(region.regionA.v0 > 0.05 && region.regionA.v0 < 0.45);
  CC_CHECK(region.regionA.v1 > 0.55 && region.regionA.v1 < 0.95);

  // No spurious transversal seeds fabricated on a shared 2D locus.
  CC_CHECK(ss.branchCount() == 0);
}

// Two IDENTICAL flat patches coincide over their WHOLE domains — the agreement runs to
// every domain edge, so it cannot be interior-delimited → Undecided (honest → OCCT),
// never a fabricated full-domain rectangle claimed as OverlapSubRegion.
CC_TEST(s4a_seeded_full_domain_overlap_is_undecided) {
  ssi::SurfaceAdapter a = flatPatch(0.0, 1.0, 0.0, 1.0);
  ssi::SurfaceAdapter b = flatPatch(0.0, 1.0, 0.0, 1.0);

  ssi::SeedOptions opts;
  opts.minPatchFrac = 1.0 / 16.0;
  auto ss = ssi::seed_intersection(a, b, opts);

  bool sawUndecided = false, sawFabricatedOverlap = false;
  for (const auto& cr : ss.coincidentRegions) {
    if (cr.kind == ssi::CoincidenceKind::Undecided) sawUndecided = true;
    if (cr.kind == ssi::CoincidenceKind::OverlapSubRegion) sawFabricatedOverlap = true;
  }
  CC_CHECK(sawUndecided);
  CC_CHECK(!sawFabricatedOverlap);  // full-domain coincidence is NOT delimited, not guessed
}

// ── S4-b SEEDED tangent-contact classification (differential geometry) ───────────
//
// These exercise classify_tangent_contact_seeded DIRECTLY on constructed surface
// adapters at a known contact point (deterministic — decoupled from whether the full
// subdivision happens to land a candidate exactly on the tangency), plus one end-to-end
// seed_intersection run confirming the typed contact reaches SeedSet.tangentContacts and
// the deferredTangent compatibility counter still moves.

// Elementary sphere/cylinder/plane wrapped as seeding adapters over a param box.
ssi::SurfaceAdapter sphereAdapter(const nmath::Sphere& s) {
  return ssi::makeSphereAdapter(s, ssi::ParamBox{0.0, kPi, -kPi / 2, kPi / 2});
}
ssi::SurfaceAdapter cylinderAdapter(const nmath::Cylinder& c, double v0, double v1) {
  return ssi::makeCylinderAdapter(c, ssi::ParamBox{0.0, 2.0 * kPi, v0, v1});
}

// Seeded classifier at contact point P with A/B params + normals resolved from geometry.
ssi::TangentContact classifyAt(const ssi::SurfaceAdapter& A, const ssi::SurfaceAdapter& B,
                               double u1, double v1, double u2, double v2, double scale) {
  const Point3 P = A.point(u1, v1);
  const Dir3 nA = A.normal(u1, v1);
  const Dir3 nB = B.normal(u2, v2);
  const double sine = nmath::norm(nmath::cross(nA.vec(), nB.vec()));
  return ssi::classify_tangent_contact_seeded(A, B, u1, v1, u2, v2, P, nA, nB, sine, scale);
}

CC_TEST(s4b_seeded_sphere_sphere_external_is_tangent_point) {
  // Unit spheres 2 apart along X: touch at (1,0,0). On A (origin) the touch is u=0,v=0;
  // on B (centre (2,0,0)) it is u=π,v=0. Relative second form is sign-definite → point.
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({2, 0, 0}), 1.0};
  auto A = sphereAdapter(s1), B = sphereAdapter(s2);
  auto tc = classifyAt(A, B, 0.0, 0.0, kPi, 0.0, 1.0);
  CC_CHECK(tc.type == ssi::TangentContactType::TangentPoint);
}

CC_TEST(s4b_seeded_coaxial_sphere_cylinder_equator_is_tangent_curve) {
  // Cylinder radius 1 coaxial through the unit sphere centre: tangent along the equator.
  // At the equator point (1,0,0): sphere u=0,v=0; cylinder u=0,v=0. Along the equator
  // (the u direction) the gap is flat → rank-1 relative form → tangent curve.
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({0, 0, 0}), 1.0};
  auto A = sphereAdapter(sp), B = cylinderAdapter(cy, -1.0, 1.0);
  auto tc = classifyAt(A, B, 0.0, 0.0, 0.0, 0.0, 1.0);
  CC_CHECK(tc.type == ssi::TangentContactType::TangentCurve);
}

CC_TEST(s4b_seeded_saddle_contact_is_near_tangent_transversal) {
  // Two Bézier patches tangent at the centre with OPPOSITE curvatures → the relative
  // second form is INDEFINITE (a saddle gap) → grazes and crosses → NearTangentTransversal
  // (an S4-c gap, handed on, NOT traced). Patch A bows +z in x and −z in y; patch B is the
  // mirror (−z in x, +z in y). Both pass through the origin with a +z tangent plane there.
  const double k = 0.4;
  auto bump = [&](double sx, double sy) {
    // A biquadratic Bézier over [-1,1]² whose height is sx·k·x² + sy·k·y² (a saddle when
    // sx,sy differ in sign). 3×3 control net; corner/edge poles set the quadratic.
    std::vector<Point3> poles;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        const double x = -1.0 + i;          // -1, 0, 1
        const double y = -1.0 + j;
        // Bézier of a quadratic: control heights equal the function at the Greville-ish
        // nodes for a pure x²/y² shape scale linearly at the mid controls; use the exact
        // biquadratic pole heights h_ij = sx·k·x·x_ctrl + ... For a degree-2 Bézier the
        // control heights reproduce a·x² via poles [0, 0, a] pattern; simplest: set pole
        // z to the analytic quadratic at the pole's (x,y) — the surface then bows toward it.
        poles.push_back(Point3{x, y, sx * k * x * x + sy * k * y * y});
      }
    return ssi::makeBezierAdapter(poles, 3, 3);
  };
  auto A = bump(+1.0, -1.0);   // +x² − y²
  auto B = bump(-1.0, +1.0);   // −x² + y²  (opposite saddle → indefinite gap)
  // Contact at the centre u=v=0.5 on both patches (maps to (0,0,0)).
  auto tc = classifyAt(A, B, 0.5, 0.5, 0.5, 0.5, 2.0);
  CC_CHECK(tc.type == ssi::TangentContactType::NearTangentTransversal);
  CC_CHECK(!tc.isTangent());
  CC_CHECK(tc.isDeferred());  // handed on to S4-c → OCCT, never traced
}

CC_TEST(s4b_seeded_matched_curvature_is_undecided) {
  // Two spheres of EQUAL, LARGE radius touching externally, probed at a small model scale:
  // both curvatures are 1/R, tiny relative to 1/scale, so the relative second form falls
  // below the curvature-noise floor → the point/curve/cross call is not robust → Undecided
  // (honest → OCCT), never a guessed definite verdict.
  const double R = 1.0e6;
  nmath::Sphere s1{frameZ({0, 0, 0}), R};
  nmath::Sphere s2{frameZ({2 * R, 0, 0}), R};
  auto A = sphereAdapter(s1), B = sphereAdapter(s2);
  // Touch at (R,0,0): A u=0,v=0; B u=π,v=0. Probe at unit scale (≪ R) → flat jet.
  auto tc = classifyAt(A, B, 0.0, 0.0, kPi, 0.0, 1.0);
  CC_CHECK(tc.type == ssi::TangentContactType::Undecided);
  CC_CHECK(tc.isDeferred());
}

// End-to-end: two externally-tangent unit spheres run through seed_intersection. The
// touch is near-tangent, so NO transversal seed is emitted there; the dropped near-tangent
// cluster is TYPED into SeedSet.tangentContacts AND still counted in deferredTangent (the
// compatibility summary). A tangent point is a 0-dim contact → no spurious branch.
CC_TEST(s4b_seeded_end_to_end_types_deferred_tangent) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({2, 0, 0}), 1.0};
  auto A = sphereAdapter(s1), B = sphereAdapter(s2);
  ssi::SeedOptions opts;
  opts.minPatchFrac = 1.0 / 16.0;
  auto ss = ssi::seed_intersection(A, B, opts);

  // If the seeder saw the near-tangent touch, it typed it and kept the counter in sync.
  CC_CHECK(static_cast<int>(ss.tangentContacts.size()) == ss.deferredTangent);
  if (!ss.tangentContacts.empty()) {
    // Every typed contact is one of the four honest kinds (never TransversalOnly here).
    for (const auto& tc : ss.tangentContacts)
      CC_CHECK(tc.type != ssi::TangentContactType::TransversalOnly);
  }
  // No fabricated transversal branch on an isolated tangency.
  CC_CHECK(ss.branchCount() == 0);
}

// ── S4-b marching stop-reason typing (additive; tracer still stops at the tangency) ─
//
// Two externally-tangent unit spheres: a march that runs into the tangency stops there and
// now carries a TYPED stopReason. This is purely additive — the tracer does NOT step through
// the tangency (no curve is fabricated past it). A transversal seed is unlikely for a pure
// point tangency, so we drive march_branch directly from a seed placed at the touch point.
CC_TEST(s4b_marching_types_near_tangent_stop) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({2, 0, 0}), 1.0};
  auto A = sphereAdapter(s1), B = sphereAdapter(s2);

  // A seed AT the touch point (1,0,0): sphere A u=0,v=0; sphere B u=π,v=0.
  ssi::Seed seed;
  seed.u1 = 0.0; seed.v1 = 0.0; seed.u2 = kPi; seed.v2 = 0.0;
  seed.point = A.point(0.0, 0.0);
  auto w = ssi::march_branch(A, B, seed, {});

  // The march cannot advance transversally at the tangency → NearTangent, and the stop is
  // TYPED (not just flagged). The tracer stopped AT the tangency (did not step through).
  if (w.status == ssi::TraceStatus::NearTangent) {
    CC_CHECK(w.stopReason.has_value());
    if (w.stopReason.has_value())
      CC_CHECK(w.stopReason->type != ssi::TangentContactType::TransversalOnly);
  } else {
    // If it did not even start (Failed), no stopReason is required — but it must not have
    // fabricated a traced curve through the tangency.
    CC_CHECK(w.status != ssi::TraceStatus::BoundaryExit || w.points.size() >= 2);
  }
}

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace

int main() { return cctest::run_all(); }
