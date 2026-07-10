// SPDX-License-Identifier: Apache-2.0
//
// curved_offset.h — MOAT M3 CURVED offset-face: offset the CYLINDRICAL LATERAL face of a
// capped cylinder RADIALLY (grow/shrink the tube radius Rc → Rc+d), the curved analogue of
// the planar `offset_face` (offset_face.h). The offset of a cylinder surface is a coaxial
// cylinder, so the operation is exactly analytic: the wall moves to radius Rc+d and the two
// coaxial planar caps grow/shrink to match, keeping the axial extent.
//
// ── WHY A CURVED ARM ────────────────────────────────────────────────────────────────
// The planar `offset_face` needs an ALL-PLANAR solid (PlanarModel), so a capped cylinder —
// which carries a Cylinder lateral face — declines to OCCT even when the picked face is the
// cylinder wall (the exact "offset a curved face" the app hits, readiness M3 @10 sites). A
// cylinder wall's outward normal is radial, so offsetting it by d is a pure radius change;
// the caps (perpendicular discs) simply widen/narrow to the new radius. No adjacent face
// tilts — the operation is a clean coaxial re-radius.
//
// ── REBUILD (planar-facet weld, tessellator-pristine) ───────────────────────────────
// The capped cylinder is rebuilt at the new radius Rc' = Rc + d as one deflection-bounded
// planar-facet soup sharing N angular samples across every seam (a wall band of N quads +
// two N-gon disc caps), welded watertight through the existing `nb::assembleSolid` — NO
// tessellator change. The engine self-verify (watertight + the correctly-signed volume
// change: grow d>0 → Vr>Vo, shrink d<0 → 0<Vr<Vo) then accepts it, else → OCCT.
//
// ── F3 CURVED FAMILIES (cone-frustum + sphere) ───────────────────────────────────────
// The same self-verified re-radius builder extends per-family to the other two analytic
// curved walls the app @10 offset_face hits:
//   * CONE-FRUSTUM wall — the offset of a cone surface is a COAXIAL cone of the SAME
//     semi-angle σ; the wall normal is radial tilted by σ, so radius at every height shifts
//     by d/cosσ, i.e. Rref → Rref + d/cosσ with the cap heights fixed. Both cap radii must
//     stay positive (a shrink cannot invert a cap; a full cone through the apex declines).
//   * SPHERE wall (sphere-cap dome) — the offset of a sphere is a CONCENTRIC sphere R → R+d;
//     the single cap plane stays fixed and the rim radius follows to √((R+d)²−a²). A shrink
//     cannot pass the cap plane (|capA| < R+d). Dome volume = π(2R³/3 − R²a + a³/3) at the
//     cap axial coord a — grow d>0 raises it, shrink d<0 lowers it.
//
// ── SCOPE (honest) ──────────────────────────────────────────────────────────────────
// Native only when the picked face is a CYLINDER / CONE lateral face of a PURE capped body
// (every face a coaxial cylinder/cone of the SAME axis + radius/σ, plus an axis-normal disc
// plane at exactly two heights) OR a SPHERE wall of a PURE sphere-cap dome (coaxial spheres
// of the same centre/radius + EXACTLY ONE axis-normal cap that cuts the ball), with the
// offset staying non-degenerate (Rc+d>0; both cone caps positive; |capA|<R+d). A stepped
// shaft / multi-radius / spherical zone (two caps) / off-centre sphere / tilted cap / a
// picked PLANAR face (owned by the planar arm) / a freeform solid → NULL → OCCT. Every
// family has a closed-form volume; the engine gates it with the two-sided volume +
// orientation self-verify. NEVER a wrong/leaky solid.
//
// CLEAN-ROOM. Reuses src/native/math + topology + boolean(assembleSolid) + blend_geom +
// curved_fillet(cylinderInfo/ringPoint/sagittaSteps). OCCT-FREE. clang++ -std=c++20.
// Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CURVED_OFFSET_H
#define CYBERCAD_NATIVE_BLEND_CURVED_OFFSET_H

#include "native/blend/blend_geom.h"
#include "native/blend/curved_fillet.h"  // detail::cylinderInfo / ringPoint / sagittaSteps

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// The pure capped cylinder resolved about a picked Cylinder lateral face: the frame, the
// radius, and the two cap axial coords. Recognised WHOLESALE (a full-turn cylinder is one
// face here, but validating the whole body guarantees the analytic rebuild == the input, so
// the volume self-verify cannot wrongly accept a differently-shaped rebuild).
struct CappedCylGeom {
  math::Ax3 axis;    // cylinder frame (z = axis)
  double radius = 0.0;
  double hLo = 0.0;  // lower cap axial coord (along z from frame origin)
  double hHi = 0.0;  // upper cap axial coord
};

inline std::optional<CappedCylGeom> cappedCylGeom(const topo::Shape& solid, int faceId) {
  const auto info = cylinderInfo(solid, faceId);
  if (!info) return std::nullopt;  // picked face is not a Cylinder → planar arm / OCCT
  const math::Ax3 ax = info->frame;
  const math::Vec3 az = ax.z.vec();
  const double Rc = info->radius;
  if (!(Rc > kBlendEps)) return std::nullopt;

  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  std::vector<double> capH;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) return std::nullopt;
    if (surf->surface->kind == topo::FaceSurface::Kind::Cylinder) {
      // Every cylinder face must be THIS coaxial cylinder at radius Rc (a full wall may be
      // one face; a segmented revolve splits it into sectors that still share the frame).
      const auto ci = cylinderInfo(solid, static_cast<int>(fi));
      if (!ci) return std::nullopt;
      if (std::fabs(std::fabs(math::dot(ci->frame.z.vec(), az)) - 1.0) > 1e-6) return std::nullopt;
      const math::Vec3 d = ci->frame.origin - ax.origin;
      if (math::norm(d - az * math::dot(d, az)) > 1e-6) return std::nullopt;  // not coaxial
      if (std::fabs(ci->radius - Rc) > 1e-6) return std::nullopt;             // a different R
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Plane) {
      const auto pl = facePlane(solid, static_cast<int>(fi));
      if (!pl) return std::nullopt;
      if (std::fabs(std::fabs(math::dot(pl->normal, az)) - 1.0) > 1e-6) return std::nullopt;  // ⟂ axis
      const double h = (pl->w - math::dot(pl->normal, ax.origin.asVec())) / math::dot(pl->normal, az);
      bool found = false;
      for (double e : capH)
        if (std::fabs(e - h) < 1e-6) { found = true; break; }
      if (!found) capH.push_back(h);
    } else {
      return std::nullopt;  // cone / sphere / freeform → not a pure capped cylinder
    }
  }
  if (capH.size() != 2) return std::nullopt;  // exactly two disc caps
  CappedCylGeom g;
  g.axis = ax;
  g.radius = Rc;
  g.hLo = std::min(capH[0], capH[1]);
  g.hHi = std::max(capH[0], capH[1]);
  if (!(g.hHi - g.hLo > kBlendEps)) return std::nullopt;
  return g;
}

// Rebuild the capped cylinder at radius `newR` as a planar-facet soup (empty on degeneracy).
// Wall band (N quads, each split into two planar triangles) + two disc caps; all share the
// SAME N angular samples so the wall→cap seams weld. Outward normals: wall radial, caps ±z.
inline std::vector<nb::Polygon> buildCappedCyl(const CappedCylGeom& g, double newR, double defl) {
  if (!(newR > kBlendEps)) return {};
  const math::Ax3& ax = g.axis;
  const double hLo = g.hLo, hHi = g.hHi;
  const int N = sagittaSteps(newR, kTwoPi, defl, 8, 256);

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * 3 + 4);

  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  };
  auto emitTri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, b, c}, nrm);
  };

  // Wall band: hLo → hHi, N quads (two triangles each, kept exactly planar).
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N, um = 0.5 * (u0 + u1);
    const math::Point3 a0 = ringPoint(ax, newR, u0, hLo);
    const math::Point3 a1 = ringPoint(ax, newR, u1, hLo);
    const math::Point3 b1 = ringPoint(ax, newR, u1, hHi);
    const math::Point3 b0 = ringPoint(ax, newR, u0, hHi);
    const math::Vec3 outN = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    emitTri(a0, a1, b1, outN);
    emitTri(a0, b1, b0, outN);
  }
  // Lower cap disk (outward = −z) and upper cap disk (outward = +z).
  {
    std::vector<math::Point3> lo, hi;
    lo.reserve(static_cast<std::size_t>(N));
    hi.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
      lo.push_back(ringPoint(ax, newR, kTwoPi * i / N, hLo));
      hi.push_back(ringPoint(ax, newR, kTwoPi * i / N, hHi));
    }
    emit(std::move(lo), ax.z.vec() * -1.0);
    emit(std::move(hi), ax.z.vec());
  }
  return polys;
}

// ── F3 CONE-FRUSTUM wall offset ─────────────────────────────────────────────────────
// The pure capped cone frustum resolved about a picked Cone lateral face: the frame, the
// signed semi-angle σ (radius = Rref + h·tanσ along +z), the cone reference radius at the
// frame origin, and the two axis-normal cap heights. Offsetting the cone wall by distance d
// along its outward (tilted) normal produces a COAXIAL cone of the SAME σ whose radius at
// EVERY height shifts by d/cosσ (the normal is radial-tilted by σ), so the whole capped
// frustum re-radiuses by Rref → Rref + d/cosσ with the cap heights fixed. Recognised
// WHOLESALE so the volume self-verify cannot wrongly accept a differently-shaped rebuild.
struct CappedConeGeom {
  math::Ax3 axis;         // cone frame (z = axis)
  double semiAngle = 0.0; // σ (radius grows with +z when σ>0)
  double Rref = 0.0;      // cone radius at the frame origin (axial coord 0)
  double hLo = 0.0;       // lower cap axial coord
  double hHi = 0.0;       // upper cap axial coord
};

inline std::optional<CappedConeGeom> cappedConeGeom(const topo::Shape& solid, int faceId) {
  const auto info = coneInfo(solid, faceId);
  if (!info) return std::nullopt;  // picked face is not a Cone → cylinder/planar arm / OCCT
  const math::Ax3 ax = info->frame;
  const math::Vec3 az = ax.z.vec();

  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  std::vector<double> capH;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) return std::nullopt;
    if (surf->surface->kind == topo::FaceSurface::Kind::Cone) {
      // Every cone face must be THIS coaxial cone (same σ / Rref); a stepped or multi-cone
      // shaft splits into different cones and declines.
      const auto ci = coneInfo(solid, static_cast<int>(fi));
      if (!ci) return std::nullopt;
      if (std::fabs(std::fabs(math::dot(ci->frame.z.vec(), az)) - 1.0) > 1e-6) return std::nullopt;
      const math::Vec3 d = ci->frame.origin - ax.origin;
      if (math::norm(d - az * math::dot(d, az)) > 1e-6) return std::nullopt;  // not coaxial
      if (std::fabs(ci->radius - info->radius) > 1e-5) return std::nullopt;
      if (std::fabs(ci->semiAngle - info->semiAngle) > 1e-6) return std::nullopt;
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Plane) {
      const auto pl = facePlane(solid, static_cast<int>(fi));
      if (!pl) return std::nullopt;
      if (std::fabs(std::fabs(math::dot(pl->normal, az)) - 1.0) > 1e-6) return std::nullopt;  // ⟂ axis
      const double h = (pl->w - math::dot(pl->normal, ax.origin.asVec())) / math::dot(pl->normal, az);
      bool found = false;
      for (double e : capH)
        if (std::fabs(e - h) < 1e-6) { found = true; break; }
      if (!found) capH.push_back(h);
    } else {
      return std::nullopt;  // cylinder / sphere / freeform → not a pure capped frustum
    }
  }
  if (capH.size() != 2) return std::nullopt;  // exactly two disc caps
  CappedConeGeom g;
  g.axis = ax;
  g.semiAngle = info->semiAngle;
  g.Rref = info->radius;
  g.hLo = std::min(capH[0], capH[1]);
  g.hHi = std::max(capH[0], capH[1]);
  if (!(g.hHi - g.hLo > kBlendEps)) return std::nullopt;
  // Both cap radii of the ORIGINAL frustum must be positive (a true frustum, not a full
  // cone through the apex — the apex disc would collapse and there would be no oracle disc).
  const double rLo = g.Rref + g.hLo * std::tan(g.semiAngle);
  const double rHi = g.Rref + g.hHi * std::tan(g.semiAngle);
  if (!(rLo > kBlendEps) || !(rHi > kBlendEps)) return std::nullopt;
  return g;
}

// Rebuild the capped cone frustum at the new reference radius `newRref` (same σ, same cap
// heights) as a planar-facet soup. Wall band (N quads) + two disc caps at radii
// newRref + h·tanσ, all sharing the SAME N angular samples. Empty on degeneracy.
inline std::vector<nb::Polygon> buildCappedCone(const CappedConeGeom& g, double newRref,
                                                double defl) {
  const math::Ax3& ax = g.axis;
  const double tanS = std::tan(g.semiAngle);
  const double hLo = g.hLo, hHi = g.hHi;
  const double rLo = newRref + hLo * tanS;
  const double rHi = newRref + hHi * tanS;
  if (!(rLo > kBlendEps) || !(rHi > kBlendEps)) return {};  // a shrink cannot invert either cap
  const int N = sagittaSteps(std::max(rLo, rHi), kTwoPi, defl, 8, 256);

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * 3 + 4);

  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  };
  auto emitTri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, b, c}, nrm);
  };

  // Wall band: hLo (radius rLo) → hHi (radius rHi). Outward normal is radial tilted by −σ
  // toward the axis of the narrowing end; the emitTri sign-fix corrects any residual flip,
  // and (cos u, sin u, 0) radial is a safe outward-pointing reference for the disambiguation.
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N, um = 0.5 * (u0 + u1);
    const math::Point3 a0 = ringPoint(ax, rLo, u0, hLo);
    const math::Point3 a1 = ringPoint(ax, rLo, u1, hLo);
    const math::Point3 b1 = ringPoint(ax, rHi, u1, hHi);
    const math::Point3 b0 = ringPoint(ax, rHi, u0, hHi);
    const math::Vec3 radial = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    const math::Vec3 outN = radial - ax.z.vec() * tanS;  // wall outward (radial, −tanσ axial)
    emitTri(a0, a1, b1, outN);
    emitTri(a0, b1, b0, outN);
  }
  // Lower cap disk (outward = −z) and upper cap disk (outward = +z).
  {
    std::vector<math::Point3> lo, hi;
    lo.reserve(static_cast<std::size_t>(N));
    hi.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
      lo.push_back(ringPoint(ax, rLo, kTwoPi * i / N, hLo));
      hi.push_back(ringPoint(ax, rHi, kTwoPi * i / N, hHi));
    }
    emit(std::move(lo), ax.z.vec() * -1.0);
    emit(std::move(hi), ax.z.vec());
  }
  return polys;
}

// ── F3 SPHERE wall offset (sphere-cap dome) ─────────────────────────────────────────
// The pure sphere-cap dome resolved about a picked Sphere lateral face: the centre, the
// outer radius, the cap-plane axial coord (from the centre along the dome axis toward the
// pole), and the pole direction. Offsetting the sphere wall by distance d gives a CONCENTRIC
// sphere R+d (the offset of a sphere is a concentric sphere); the SAME cap plane stays fixed
// and the rim radius follows to √((R+d)²−a²). Recognised WHOLESALE: every face a coaxial
// sphere (same centre / radius) or EXACTLY ONE axis-normal planar cap that actually cuts the
// ball, the sphere closed at the pole opposite the cap. A spherical zone (two caps), an
// off-centre / multi-radius sphere, a cylinder / cone / freeform body → nullopt.
struct SphereDomeGeom {
  math::Point3 centre;   // sphere centre
  math::Dir3 pole;       // unit axis from the centre toward the closed pole (away from cap)
  double R = 0.0;        // outer sphere radius
  double capA = 0.0;     // cap-plane axial coord along `pole` from the centre (dome = axial ≥ capA)
};

inline std::optional<SphereDomeGeom> sphereDomeGeom(const topo::Shape& solid, int faceId) {
  const auto info = sphereInfo(solid, faceId);
  if (!info) return std::nullopt;  // picked face is not a Sphere → other arm / OCCT
  const math::Point3 centre = info->frame.origin;
  const double R = info->radius;
  if (!(R > kBlendEps)) return std::nullopt;

  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  // A full revolve fragments the sphere wall AND the disc cap into angular sectors, so match
  // by GEOMETRY: every sphere face is THIS ball (same centre / radius); every planar face is
  // an axis-normal cap, collected as a DISTINCT plane (sectors of one disc share a plane).
  std::optional<nb::Plane> cap;
  int distinctCaps = 0;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) return std::nullopt;
    if (surf->surface->kind == topo::FaceSurface::Kind::Sphere) {
      const auto si = sphereInfo(solid, static_cast<int>(fi));
      if (!si) return std::nullopt;
      if (math::norm(si->frame.origin - centre) > 1e-6) return std::nullopt;  // off-centre
      if (std::fabs(si->radius - R) > 1e-6) return std::nullopt;              // a different R
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Plane) {
      const auto pl = facePlane(solid, static_cast<int>(fi));
      if (!pl) return std::nullopt;
      if (!cap) {
        cap = pl;
        ++distinctCaps;
      } else {
        // Sectors of one disc share the SAME facePlane (identical outward normal + w). A
        // plane with a different normal direction or offset is a SECOND cap → a spherical
        // zone, which declines.
        const bool same = std::fabs(math::dot(pl->normal, cap->normal) - 1.0) < 1e-6 &&
                          std::fabs(pl->w - cap->w) < 1e-6;
        if (!same) ++distinctCaps;
      }
    } else {
      return std::nullopt;  // cylinder / cone / freeform → not a pure sphere-cap dome
    }
  }
  if (distinctCaps != 1 || !cap) return std::nullopt;  // exactly ONE cap plane (a zone declines)

  // The cap plane normal is the dome axis. The cap's outward normal points AWAY from the
  // dome material, so the enclosed pole is on the −normal side: pole = −cap.normal. (For a
  // through-centre hemisphere the choice is still consistent since the material fills the
  // −normal hemisphere.) Axial coord of the cap plane along +pole from the centre:
  //   dot(n, centre + a·pole) = w, with n = cap.normal and pole = −n ⇒ a = dot(n,centre) − w.
  const math::Dir3 pole{cap->normal * -1.0};
  if (!pole.valid()) return std::nullopt;
  const double aCap = math::dot(cap->normal, centre.asVec()) - cap->w;
  if (!(std::fabs(aCap) < R - 1e-9)) return std::nullopt;  // cap must actually cut the sphere

  SphereDomeGeom g;
  g.centre = centre;
  g.pole = pole;
  g.R = R;
  g.capA = aCap;
  return g;
}

// Rebuild the sphere-cap dome at the new outer radius `newR` (concentric, same cap plane) as
// a planar-facet soup: sphere wall from the pole down to the cap plane (latitude bands) + an
// axis-normal disc cap at axial coord capA (radius √(newR²−capA²)). Shares N angular samples
// across the rim seam. Empty on degeneracy.
inline std::vector<nb::Polygon> buildSphereDome(const SphereDomeGeom& g, double newR,
                                                double defl) {
  const double capA = g.capA;
  if (!(newR > kBlendEps) || !(std::fabs(capA) < newR - 1e-9)) return {};
  const math::Vec3 pole = g.pole.vec();
  // Build an orthonormal frame (ex, ey) spanning the plane ⟂ pole.
  math::Vec3 seed = std::fabs(pole.x) < 0.9 ? math::Vec3{1, 0, 0} : math::Vec3{0, 1, 0};
  math::Vec3 exV = seed - pole * math::dot(seed, pole);
  const double exn = math::norm(exV);
  if (!(exn > kBlendEps)) return {};
  exV = exV * (1.0 / exn);
  const math::Vec3 eyV = math::cross(pole, exV);

  // A point on the concentric sphere at longitude u and polar-axial coord a (∈[capA, newR]).
  auto spherePt = [&](double u, double a) -> math::Point3 {
    const double rho = std::sqrt(std::max(0.0, newR * newR - a * a));  // radius off the axis
    const math::Vec3 p = g.centre.asVec() + pole * a +
                         (exV * std::cos(u) + eyV * std::sin(u)) * rho;
    return math::Point3{p.x, p.y, p.z};
  };
  auto sphereNormal = [&](double u, double a) -> math::Vec3 {
    const double rho = std::sqrt(std::max(0.0, newR * newR - a * a));
    return (pole * a + (exV * std::cos(u) + eyV * std::sin(u)) * rho) * (1.0 / newR);  // radial out
  };

  const int N = sagittaSteps(newR, kTwoPi, defl, 8, 256);
  // Polar bands from the cap latitude (a=capA) up to the pole (a=newR); the polar span is
  // the arc from asin(capA/newR) to π/2.
  const double latCap = std::asin(std::clamp(capA / newR, -1.0, 1.0));
  const double polarSpan = (kTwoPi / 4.0) - latCap;  // π/2 − latCap
  const int M = sagittaSteps(newR, std::fabs(polarSpan), defl, 3, 128);

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * (M + 1) + 2);

  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  };
  auto emitTri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, b, c}, nrm);
  };

  // Latitude-to-axial map: at polar band j (0..M), the polar angle φ from the equator runs
  // latCap → π/2; the axial coord a = newR·sin(φ), off-axis radius follows in spherePt.
  auto aAt = [&](int j) -> double {
    const double phi = latCap + (kTwoPi / 4.0 - latCap) * j / M;
    return newR * std::sin(phi);
  };

  // Sphere wall: M latitude bands × N longitude quads (two triangles each). Top band closes
  // to the pole as triangles (rho→0).
  for (int j = 0; j < M; ++j) {
    const double a0 = aAt(j), a1 = aAt(j + 1);
    for (int i = 0; i < N; ++i) {
      const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N, um = 0.5 * (u0 + u1);
      const math::Point3 p00 = spherePt(u0, a0);
      const math::Point3 p10 = spherePt(u1, a0);
      const math::Point3 p11 = spherePt(u1, a1);
      const math::Point3 p01 = spherePt(u0, a1);
      const math::Vec3 outN = sphereNormal(um, 0.5 * (a0 + a1));
      emitTri(p00, p10, p11, outN);
      emitTri(p00, p11, p01, outN);
    }
  }
  // Cap disc at axial coord capA (outward = −pole; the dome material is on the +pole side).
  {
    std::vector<math::Point3> rim;
    rim.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) rim.push_back(spherePt(kTwoPi * i / N, capA));
    emit(std::move(rim), pole * -1.0);
  }
  return polys;
}

}  // namespace detail

// Offset the CYLINDRICAL LATERAL face `faceId` (1-based, mapShapes order) of a capped
// cylinder RADIALLY by `distance` (grow d>0 → radius Rc+d, shrink d<0 → Rc+d, must stay
// > 0). Returns the re-radiused capped cylinder (deflection-bounded planar-facet soup,
// watertight) or a NULL Shape (→ OCCT) when the picked face is not a cylinder wall of a
// pure capped cylinder, when Rc + distance ≤ 0, or on any degeneracy. A picked PLANAR face
// is left to the planar `offset_face` arm; a cone / sphere / stepped / multi-cylinder body
// → NULL.
inline topo::Shape curved_offset_face(const topo::Shape& solid, int faceId, double distance,
                                      double deflection = 0.01) {
  if (std::fabs(distance) < kBlendEps) return {};

  // 1. CYLINDER lateral wall of a pure capped cylinder → coaxial re-radius Rc → Rc+d.
  if (const auto g = detail::cappedCylGeom(solid, faceId)) {
    const double newR = g->radius + distance;
    if (!(newR > kBlendEps)) return {};  // a shrink cannot invert the tube
    std::vector<nb::Polygon> polys = detail::buildCappedCyl(*g, newR, deflection);
    if (polys.size() < 4) return {};
    return nb::assembleSolid(polys);
  }

  // 2. CONE lateral wall of a pure capped frustum → coaxial cone of the SAME semi-angle σ
  //    whose radius at every height shifts by d/cosσ (the wall normal is radial-tilted by σ),
  //    i.e. Rref → Rref + d/cosσ, cap heights fixed. Both cap radii must stay positive.
  if (const auto g = detail::cappedConeGeom(solid, faceId)) {
    const double newRref = g->Rref + distance / std::cos(g->semiAngle);
    std::vector<nb::Polygon> polys = detail::buildCappedCone(*g, newRref, deflection);
    if (polys.size() < 4) return {};  // empty ⇒ a shrink inverted a cap → decline
    return nb::assembleSolid(polys);
  }

  // 3. SPHERE wall of a pure sphere-cap dome → concentric sphere R → R+d, SAME cap plane
  //    (rim radius follows). A shrink cannot pass the cap plane (|capA| < R+d required).
  if (const auto g = detail::sphereDomeGeom(solid, faceId)) {
    const double newR = g->R + distance;
    if (!(newR > kBlendEps) || !(std::fabs(g->capA) < newR - 1e-9)) return {};
    std::vector<nb::Polygon> polys = detail::buildSphereDome(*g, newR, deflection);
    if (polys.size() < 4) return {};
    return nb::assembleSolid(polys);
  }

  return {};  // not a cylinder / cone / sphere wall of a recognised body → OCCT
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CURVED_OFFSET_H
