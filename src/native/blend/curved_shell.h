// SPDX-License-Identifier: Apache-2.0
//
// curved_shell.h — native uniform-thickness hollow of a CURVED-wall body (MOAT M3,
// second curved-blend slice, `moat-m3cs-curved-shell`). Extends the planar shell
// (shell.h, convex-planar only) to the first CURVED substrate: a capped CYLINDER or a
// capped CONE FRUSTUM hollowed to a constant wall thickness with one planar cap left
// OPEN — the tractable body-of-revolution case the app hits when it shells a turned
// part (a cup / sleeve / tapered bushing).
//
// ── APPROACH (analytic inward offset of the curved wall, tessellator-pristine) ──────
// A uniform-thickness hollow of a solid S removing face(s) F is
//     result = S − cavity,
// where the cavity is S inset inward by `t` on every wall that STAYS and left flush with
// the outer surface on every REMOVED face. For a capped cylinder / cone frustum the
// curved wall offsets inward ANALYTICALLY:
//   * CYLINDER radius Rc  → inner coaxial cylinder radius Ri = Rc − t.
//   * CONE half-angle σ, wall radius Rw(h) → inner coaxial cone shifted inward by the
//     PERPENDICULAR distance t: the inner radius at height h is Rw(h) − t/cosσ, i.e. the
//     same apex-tilt cone whose reference radius drops by t/cosσ.
// The kept planar cap offsets inward by `t` along its inward normal (a smaller coaxial
// disk). The removed cap stays flush (the opening). Rather than run a curved BSP-CSG cut
// (the native boolean is planar-faced only), we REBUILD the hollow tube directly as one
// deflection-bounded planar-facet soup that shares the SAME N angular samples across
// every seam, so it welds watertight through the SAME assembleSolid path a native
// prism/boolean uses — NO tessellator change. The engine self-verify (watertight +
// SHRINK: 0 < Vr < Vo) then accepts it, else → OCCT.
//
// The boundary of the hollowed tube (kept end at hKept, open end at hOpen, s = axial sign
// from kept toward open, wall height H = |hOpen − hKept|):
//   1. OUTER curved wall (Rc / cone) over the full height hKept → hOpen;
//   2. KEPT cap OUTER disk (radius Rc / Rw(hKept)) at hKept;
//   3. INNER curved wall (Ri / inner cone) from the kept inner face (hKept + s·t) → hOpen,
//      normal pointing toward the AXIS (into the cavity);
//   4. KEPT cap INNER disk (radius Ri_kept) at hKept + s·t, normal toward the open side;
//   5. OPEN-END rim ANNULUS (Ri_open → outer radius) at hOpen, the wall-thickness ring.
//
// ── F2 SPHERE-CAP extension (curved-face breadth) ─────────────────────────────────────
// A third analytic curved substrate: a SPHERE-CAP dome — one coaxial Sphere wall closed at
// the pole and cut by EXACTLY ONE axis-normal planar cap (a hemisphere is capH = 0; any
// shallower/deeper spherical cap is the same family). Hollowing it opens the cap; the inner
// surface is the CONCENTRIC sphere Ri = Ro − t (the offset of a sphere is a concentric
// sphere), running pole → the SAME cap plane (a flush opening) with an open-rim ANNULUS at
// the cap. Closed-form wall volume = outer spherical segment − inner spherical segment
// (segment above the plane at axial a: π(2R³/3 − R²a + a³/3)). recogniseShellSphere +
// removedSphereCap + buildSphereShell, welded through the SAME assembleSolid — no tessellator
// change. This lands the app's turned-dome / bowl shell on the sphere substrate.
//
// ── SCOPE (honest) ──────────────────────────────────────────────────────────────────
// Native only for a body that is EXACTLY a capped cylinder / capped cone frustum (one
// coaxial Cylinder/Cone + two axis-normal planar caps) removing ONE cap, OR a sphere-cap
// dome (one coaxial Sphere + EXACTLY ONE axis-normal planar cap) opening that cap, with
// t < the smallest inner dimension. Everything else → NULL → OCCT
// (BRepOffsetAPI_MakeThickSolid):
//   * removing the curved wall or BOTH caps, or removing zero faces;
//   * a stepped shaft / multi-cylinder / multi-cone body, a freeform wall;
//   * a spherical ZONE (two cap planes) or an off-centre / multi-radius sphere;
//   * a MIXED planar-and-curved (box-with-a-curved-face) or NON-CONVEX planar body — the
//     per-face inward offset self-intersects at reflex edges (no analytic re-weld here);
//   * a tilted (non-perpendicular) cap; a thickness that collapses the cavity;
//   * a cone whose inner offset crosses the axis before the far end.
//
// CLEAN-ROOM. Reuses only src/native/math + topology + boolean (Polygon/assembleSolid)
// and the curved_fillet.h face recognisers. No OCCT. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CURVED_SHELL_H
#define CYBERCAD_NATIVE_BLEND_CURVED_SHELL_H

#include "native/blend/blend_geom.h"
#include "native/blend/curved_fillet.h"  // detail::CylInfo/cylinderInfo, ConeInfo/coneInfo
#include "native/math/elementary.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// ── shared facet-soup emit helpers (identical idiom to curved_fillet.h) ─────────────
// A ring point in a coaxial frame at radius `rad`, azimuth u, axial coord h.
inline math::Point3 shellRingPoint(const math::Ax3& ax, double rad, double u, double h) {
  return math::frameCombine(ax, rad * std::cos(u), rad * std::sin(u), h);
}

// Collect every axis-normal planar-cap axial height (dot with the axis) of `solid`, and
// every distinct coaxial cylinder / cone face; reject any other face kind. Used to
// validate the body is EXACTLY a capped cylinder or capped frustum before rebuilding.
struct ShellBody {
  bool isCone = false;
  math::Ax3 axis;           // wall frame (z = axis)
  double cylRadius = 0.0;   // cylinder radius (isCone==false)
  double coneRef = 0.0;     // cone reference radius at frame origin (isCone==true)
  double coneSemi = 0.0;    // cone half-angle
  double capLo = 0.0;       // the lower planar-cap axial coord (capLo < capHi)
  double capHi = 0.0;       // the upper planar-cap axial coord
};

// Wall radius at axial coord h (measured from the frame origin along +axis).
inline double shellWallRadius(const ShellBody& b, double h) {
  return b.isCone ? (b.coneRef + h * std::tan(b.coneSemi)) : b.cylRadius;
}

// Recognise `solid` as a pure capped cylinder or capped cone frustum. The wall face may
// be split into angular sectors by a full revolve, so we validate WHOLESALE: every face
// is a coaxial Cylinder (same radius) OR a coaxial Cone (same σ/reference) OR an
// axis-normal Plane at one of exactly TWO heights. Returns nullopt otherwise.
inline std::optional<ShellBody> recogniseShellBody(const topo::Shape& solid) {
  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  if (fmap.size() < 3) return std::nullopt;

  std::optional<math::Ax3> axis;
  bool isCone = false;
  double cylR = 0.0, coneRef = 0.0, coneSemi = 0.0;

  auto sameAxis = [](const math::Ax3& a, const math::Vec3& z, const math::Point3& o) {
    if (std::fabs(std::fabs(math::dot(a.z.vec(), z)) - 1.0) > 1e-6) return false;
    const math::Vec3 d = o - a.origin;
    return math::norm(d - a.z.vec() * math::dot(d, a.z.vec())) <= 1e-6;
  };

  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) return std::nullopt;
    const int id = static_cast<int>(fi);
    if (surf->surface->kind == topo::FaceSurface::Kind::Cylinder) {
      const auto info = cylinderInfo(solid, id);
      if (!info) return std::nullopt;
      if (!axis) { axis = info->frame; cylR = info->radius; isCone = false; }
      else {
        if (isCone) return std::nullopt;  // mixed cone + cylinder
        if (!sameAxis(*axis, info->frame.z.vec(), info->frame.origin)) return std::nullopt;
        if (std::fabs(info->radius - cylR) > 1e-6) return std::nullopt;  // multi-cylinder
      }
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Cone) {
      const auto info = coneInfo(solid, id);
      if (!info) return std::nullopt;
      if (!axis) { axis = info->frame; coneRef = info->radius; coneSemi = info->semiAngle; isCone = true; }
      else {
        if (!isCone) return std::nullopt;  // mixed cylinder + cone
        if (!sameAxis(*axis, info->frame.z.vec(), info->frame.origin)) return std::nullopt;
        // Same cone (fold the reference radius to the shared origin's axial 0 by frame).
        if (std::fabs(info->radius - coneRef) > 1e-5 ||
            std::fabs(info->semiAngle - coneSemi) > 1e-6)
          return std::nullopt;  // multi-frustum
      }
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Plane) {
      const auto pl = facePlane(solid, id);
      if (!pl) return std::nullopt;
      // Cap heights are resolved in the SECOND pass below (once the wall axis is known,
      // so perpendicularity and axial coordinate are well-defined).
    } else {
      return std::nullopt;  // sphere / freeform → defer
    }
  }
  if (!axis) return std::nullopt;

  // Second pass: resolve every plane cap height against the now-known axis; every cap
  // must be axis-normal, and there must be exactly TWO distinct cap heights.
  const math::Vec3 az = axis->z.vec();
  std::vector<double> heights;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf || surf->surface->kind != topo::FaceSurface::Kind::Plane) continue;
    const auto pl = facePlane(solid, static_cast<int>(fi));
    if (!pl) return std::nullopt;
    const double nDotAz = math::dot(pl->normal, az);
    if (std::fabs(std::fabs(nDotAz) - 1.0) > 1e-6) return std::nullopt;  // tilted cap
    const double h = (pl->w - math::dot(pl->normal, axis->origin.asVec())) / nDotAz;
    bool found = false;
    for (double e : heights) if (std::fabs(e - h) < 1e-6) { found = true; break; }
    if (!found) heights.push_back(h);
  }
  if (heights.size() != 2) return std::nullopt;

  ShellBody b;
  b.isCone = isCone;
  b.axis = *axis;
  b.cylRadius = cylR;
  b.coneRef = coneRef;
  b.coneSemi = coneSemi;
  b.capLo = std::min(heights[0], heights[1]);
  b.capHi = std::max(heights[0], heights[1]);
  return b;
}

// Which of the two caps is being removed (opened)? Resolve the picked face ids to the
// axial cap heights and require EXACTLY ONE distinct cap (the other stays). Returns the
// removed cap's axial coord, or nullopt if the pick is not exactly one cap.
inline std::optional<double> removedCapHeight(const topo::Shape& solid, const ShellBody& b,
                                              const int* faceIds, int faceCount) {
  if (faceIds == nullptr || faceCount < 1) return std::nullopt;
  const math::Vec3 az = b.axis.z.vec();
  std::optional<double> removed;
  for (int i = 0; i < faceCount; ++i) {
    const auto pl = facePlane(solid, faceIds[i]);
    if (!pl) return std::nullopt;  // a picked non-planar (curved wall) face → decline
    const double nDotAz = math::dot(pl->normal, az);
    if (std::fabs(std::fabs(nDotAz) - 1.0) > 1e-6) return std::nullopt;
    const double h = (pl->w - math::dot(pl->normal, b.axis.origin.asVec())) / nDotAz;
    // Must be one of the two known caps.
    const bool isLo = std::fabs(h - b.capLo) < 1e-6;
    const bool isHi = std::fabs(h - b.capHi) < 1e-6;
    if (!isLo && !isHi) return std::nullopt;
    const double hh = isLo ? b.capLo : b.capHi;
    if (!removed) removed = hh;
    else if (std::fabs(*removed - hh) > 1e-6) return std::nullopt;  // both caps picked → decline
  }
  return removed;
}

// Build the hollow tube as a planar-facet soup (empty on any degeneracy → NULL → OCCT).
inline std::vector<nb::Polygon> buildCurvedShell(const ShellBody& b, double hOpen, double t,
                                                 double defl) {
  const math::Ax3& ax = b.axis;
  const double hKept = (std::fabs(hOpen - b.capLo) < 1e-6) ? b.capHi : b.capLo;
  const double s = (hOpen >= hKept) ? 1.0 : -1.0;   // axial sign kept → open
  const double H = std::fabs(hOpen - hKept);
  if (!(H > t + 1e-9)) return {};                    // cavity must have positive height

  // Outer/inner wall radii at the kept and open ends.
  const double RwKept = shellWallRadius(b, hKept);
  const double RwOpen = shellWallRadius(b, hOpen);
  if (!(RwKept > 1e-9) || !(RwOpen > 1e-9)) return {};

  // Inner-wall reference: cylinder → Rc − t; cone → reference radius dropped by t/cosσ
  // (perpendicular inward offset of the tilted wall).
  const double dRadInset = b.isCone ? (t / std::cos(b.coneSemi)) : t;
  auto innerRadius = [&](double h) -> double { return shellWallRadius(b, h) - dRadInset; };
  const double hKeptInner = hKept + s * t;           // inner face of the kept cap
  const double RiKept = innerRadius(hKeptInner);      // inner-wall radius at the kept inner face
  const double RiOpen = innerRadius(hOpen);            // inner-wall radius at the open end
  if (!(RiKept > 1e-9) || !(RiOpen > 1e-9)) return {}; // wall thicker than the body → decline

  // Sampling: angular from the max radius; the wall is straight in z (1 quad tall) but a
  // cone needs the inner/outer walls tiled in one step (they are ruled straight lines).
  const int N = sagittaSteps(std::max(RwKept, RwOpen), kTwoPi, defl, 8, 256);

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * 4 + 8);

  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  };
  auto emitTri = [&](const math::Point3& a, const math::Point3& c1, const math::Point3& c2,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(c1 - a, c2 - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, c1, c2}, nrm);
  };
  auto emitQuad = [&](const math::Point3& p00, const math::Point3& p10, const math::Point3& p11,
                      const math::Point3& p01, const math::Vec3& outward) {
    emitTri(p00, p10, p11, outward);
    emitTri(p00, p11, p01, outward);
  };
  auto uAt = [&](int i) { return kTwoPi * i / N; };

  // Outer wall outward normal in the axial cross-section: radial for a cylinder; for a
  // cone tilt by the slope dR/d(+axis) = tanσ (outward normal (1,−s·tanσ)/|..|, +radial).
  const double dRdz = b.isCone ? std::tan(b.coneSemi) : 0.0;
  const double wn = std::sqrt(1.0 + dRdz * dRdz);
  auto outerWallNormal = [&](double u) -> math::Vec3 {
    const math::Vec3 radial = ax.x.vec() * std::cos(u) + ax.y.vec() * std::sin(u);
    return radial * (1.0 / wn) + ax.z.vec() * (s * (-dRdz / wn));
  };
  // Inner wall outward normal points toward the AXIS (into the cavity): negate radial;
  // its axial component mirrors the outer wall's (parallel offset surface).
  auto innerWallNormal = [&](double u) -> math::Vec3 {
    const math::Vec3 radial = ax.x.vec() * std::cos(u) + ax.y.vec() * std::sin(u);
    return radial * (-1.0 / wn) + ax.z.vec() * (s * (dRdz / wn));
  };

  const math::Vec3 keptOut = ax.z.vec() * (-s);   // kept cap outer normal (away from open)
  const math::Vec3 keptInnerOut = ax.z.vec() * s; // kept-cap inner face normal (toward open)
  const math::Vec3 openOut = ax.z.vec() * s;      // open-end rim annulus normal (toward open)

  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);

    // 1. Outer wall: hKept → hOpen (radius follows the wall — cone tapers, cylinder const).
    emitQuad(shellRingPoint(ax, RwKept, u0, hKept), shellRingPoint(ax, RwKept, u1, hKept),
             shellRingPoint(ax, RwOpen, u1, hOpen), shellRingPoint(ax, RwOpen, u0, hOpen),
             outerWallNormal(um));

    // 3. Inner wall: hKeptInner → hOpen (radius follows the inner offset wall).
    emitQuad(shellRingPoint(ax, RiKept, u0, hKeptInner), shellRingPoint(ax, RiKept, u1, hKeptInner),
             shellRingPoint(ax, RiOpen, u1, hOpen), shellRingPoint(ax, RiOpen, u0, hOpen),
             innerWallNormal(um));

    // 5. Open-end rim annulus: RiOpen → RwOpen at hOpen.
    emitQuad(shellRingPoint(ax, RiOpen, u0, hOpen), shellRingPoint(ax, RiOpen, u1, hOpen),
             shellRingPoint(ax, RwOpen, u1, hOpen), shellRingPoint(ax, RwOpen, u0, hOpen),
             openOut);
  }

  // 2. Kept cap OUTER disk: radius RwKept at hKept.
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(shellRingPoint(ax, RwKept, uAt(i), hKept));
    emit(std::move(ring), keptOut);
  }
  // 4. Kept cap INNER disk: radius RiKept at hKeptInner (bottom of the cavity).
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(shellRingPoint(ax, RiKept, uAt(i), hKeptInner));
    emit(std::move(ring), keptInnerOut);
  }
  return polys;
}

// ── SPHERE-CAP dome shell (F2 curved-face extension) ─────────────────────────────────
// The third analytic curved substrate: a SPHERE-CAP dome — a body whose faces are one
// coaxial Sphere wall (radius Ro, centre C) closed at the pole and cut by EXACTLY ONE
// axis-normal planar cap. A hemisphere (cap through the centre, capH = 0) and any
// shallower/deeper spherical cap (0 < capH < Ro) are the same family. Hollowing it to a
// uniform wall `t` opens the planar cap: the inner surface is the CONCENTRIC sphere
// Ri = Ro − t (the offset of a sphere is a concentric sphere), running from the pole down
// to the SAME cap plane (a flush opening). The wall volume is the outer spherical segment
// (axial coord toward the pole ≥ capH) minus the inner segment — a closed form.
struct ShellSphere {
  math::Ax3 axis;        // sphere frame; origin at the centre, z toward the POLE (away from cap)
  double R = 0.0;        // outer sphere radius
  double capH = 0.0;     // axial coord of the single cap plane from the centre (0 ≤ capH < R)
};

// Recognise `solid` as a pure sphere-cap dome: every face a coaxial Sphere (same centre /
// radius) OR an axis-normal Plane at EXACTLY ONE height (the cap). The sphere is closed at
// the pole opposite the cap. Returns nullopt for anything else (cylinder/cone body already
// handled by recogniseShellBody; a spherical ZONE with two caps; a tilted cap; freeform).
inline std::optional<ShellSphere> recogniseShellSphere(const topo::Shape& solid) {
  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  if (fmap.size() < 2) return std::nullopt;

  std::optional<math::Point3> centre;
  double R = 0.0;
  math::Ax3 sphAxis;
  bool haveSphere = false;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) return std::nullopt;
    if (surf->surface->kind == topo::FaceSurface::Kind::Sphere) {
      const auto si = sphereInfo(solid, static_cast<int>(fi));
      if (!si) return std::nullopt;
      if (!haveSphere) { centre = si->frame.origin; R = si->radius; sphAxis = si->frame; haveSphere = true; }
      else {
        if (math::norm(si->frame.origin - *centre) > 1e-6) return std::nullopt;  // off-centre
        if (std::fabs(si->radius - R) > 1e-6) return std::nullopt;               // multi-radius
      }
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Plane) {
      // Deferred to the second pass (needs the sphere axis to test perpendicularity).
    } else {
      return std::nullopt;  // cylinder / cone / freeform → not a pure sphere-cap dome
    }
  }
  if (!haveSphere || !(R > kBlendEps)) return std::nullopt;

  // The cap axis is the sphere-cap normal. All caps must be axis-normal at exactly ONE
  // height; the axis direction is that shared cap normal.
  std::optional<math::Vec3> capN;
  std::vector<double> capOffsets;  // signed plane offset w along the cap normal
  bool anyPlane = false;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf || surf->surface->kind != topo::FaceSurface::Kind::Plane) continue;
    anyPlane = true;
    const auto pl = facePlane(solid, static_cast<int>(fi));
    if (!pl) return std::nullopt;
    if (!capN) capN = pl->normal;
    else if (std::fabs(std::fabs(math::dot(pl->normal, *capN)) - 1.0) > 1e-6) return std::nullopt;
    const double h = signedDist(*pl, *centre);  // signed distance centre → plane along +normal
    // Fold every cap plane's height to the FIRST cap normal so ± sectors compare equal.
    const double sgn = math::dot(pl->normal, *capN) >= 0.0 ? 1.0 : -1.0;
    const double hh = -h * sgn;  // axial coord of the plane from the centre along +capN
    bool found = false;
    for (double e : capOffsets) if (std::fabs(e - hh) < 1e-6) { found = true; break; }
    if (!found) capOffsets.push_back(hh);
  }
  if (!anyPlane || capOffsets.size() != 1) return std::nullopt;  // must be exactly ONE cap

  const double capH = capOffsets.front();  // along +capN from the centre
  if (!(std::fabs(capH) < R - 1e-9)) return std::nullopt;  // cap must actually cut the sphere

  // Build the dome frame: z toward the POLE (away from the cap). The cap plane sits at axial
  // coord capH along +capN; the pole is on the OPPOSITE side of the material from the cap, so
  // z = −capN (the cap normal points OUT of the material, i.e. away from the pole).
  const math::Dir3 zPole{*capN * -1.0};
  if (!zPole.valid()) return std::nullopt;
  ShellSphere b;
  b.R = R;
  b.capH = -capH;  // axial coord of the cap along +zPole (pole side positive): capH along +capN → −capH along +zPole
  // Orthonormal frame at the centre with z = zPole.
  math::Vec3 zv = zPole.vec();
  math::Vec3 ref = std::fabs(zv.x) < 0.9 ? math::Vec3{1, 0, 0} : math::Vec3{0, 1, 0};
  math::Dir3 xd{ref - zv * math::dot(ref, zv)};
  if (!xd.valid()) return std::nullopt;
  b.axis = math::Ax3{*centre, xd, math::Dir3{math::cross(zv, xd.vec())}, zPole};
  return b;
}

// The picked face(s) must be EXACTLY the single cap plane of the sphere-cap dome (opening
// it), else nullopt. Returns the cap's axial coord along +axis (== b.capH) on success.
inline std::optional<double> removedSphereCap(const topo::Shape& solid, const ShellSphere& b,
                                              const int* faceIds, int faceCount) {
  if (faceIds == nullptr || faceCount < 1) return std::nullopt;
  const math::Vec3 az = b.axis.z.vec();
  for (int i = 0; i < faceCount; ++i) {
    const auto pl = facePlane(solid, faceIds[i]);
    if (!pl) return std::nullopt;  // picked a curved (sphere) wall → decline
    if (std::fabs(std::fabs(math::dot(pl->normal, az)) - 1.0) > 1e-6) return std::nullopt;
    const double nDotAz = math::dot(pl->normal, az);
    const double h = (pl->w - math::dot(pl->normal, b.axis.origin.asVec())) / nDotAz;
    if (std::fabs(h - b.capH) > 1e-6) return std::nullopt;  // not the one cap
  }
  return b.capH;
}

// Build the hollow sphere-cap dome (a bowl) as a planar-facet soup. Outer sphere wall +
// concentric inner sphere wall (Ri = R − t), both from the pole down to the cap plane, and
// the open-rim ANNULUS at the cap plane (innerRim → outerRim). Empty on any degeneracy.
inline std::vector<nb::Polygon> buildSphereShell(const ShellSphere& b, double t, double defl) {
  const math::Ax3& ax = b.axis;
  const double Ro = b.R, Ri = Ro - t, capH = b.capH;
  if (!(Ri > kBlendEps)) return {};                      // wall thicker than the ball
  // Rim radii where each sphere meets the cap plane (axial coord capH from the centre).
  if (!(capH < Ri - 1e-9)) return {};                    // inner sphere must still cross the cap
  const double rimOut = std::sqrt(std::max(0.0, Ro * Ro - capH * capH));
  const double rimIn = std::sqrt(std::max(0.0, Ri * Ri - capH * capH));
  if (!(rimOut > rimIn + kBlendEps)) return {};

  // Sphere latitude (from the equator) of the cap plane on each sphere: sin(lat) = capH/R.
  const double latCapOut = std::asin(std::clamp(capH / Ro, -1.0, 1.0));
  const double latCapIn = std::asin(std::clamp(capH / Ri, -1.0, 1.0));
  const double latPole = kTwoPi / 4.0;                   // +π/2 at the pole (+axis)

  const int N = sagittaSteps(Ro, kTwoPi, defl, 8, 256);                 // angular
  const int Ko = sagittaSteps(Ro, latPole - latCapOut, defl, 4, 128);  // outer wall latitude
  const int Ki = sagittaSteps(Ri, latPole - latCapIn, defl, 4, 128);   // inner wall latitude

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * (Ko + Ki + 1) + 4);

  auto uAt = [&](int i) { return kTwoPi * i / N; };
  auto spherePt = [&](double rad, double u, double lat) -> math::Point3 {
    return ringPoint(ax, rad * std::cos(lat), u, rad * std::sin(lat));
  };
  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  };
  auto emitTri = [&](const math::Point3& a, const math::Point3& c1, const math::Point3& c2,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(c1 - a, c2 - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, c1, c2}, nrm);
  };
  auto emitQuad = [&](const math::Point3& p00, const math::Point3& p10, const math::Point3& p11,
                      const math::Point3& p01, const math::Vec3& outward) {
    emitTri(p00, p10, p11, outward);
    emitTri(p00, p11, p01, outward);
  };

  // 1. OUTER sphere wall: latCapOut → pole, N·Ko quads, outward normal radial-from-centre.
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    for (int k = 0; k < Ko; ++k) {
      const double l0 = latCapOut + (latPole - latCapOut) * k / Ko;
      const double l1 = latCapOut + (latPole - latCapOut) * (k + 1) / Ko;
      const double lm = 0.5 * (l0 + l1);
      const math::Vec3 radial = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
      const math::Vec3 outN = radial * std::cos(lm) + ax.z.vec() * std::sin(lm);
      emitQuad(spherePt(Ro, u0, l0), spherePt(Ro, u1, l0), spherePt(Ro, u1, l1),
               spherePt(Ro, u0, l1), outN);
    }
  }
  // 2. INNER sphere wall: latCapIn → pole, N·Ki quads, normal points TOWARD the centre
  //    (into the cavity → out of the wall material).
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    for (int k = 0; k < Ki; ++k) {
      const double l0 = latCapIn + (latPole - latCapIn) * k / Ki;
      const double l1 = latCapIn + (latPole - latCapIn) * (k + 1) / Ki;
      const double lm = 0.5 * (l0 + l1);
      const math::Vec3 radial = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
      const math::Vec3 inN = (radial * std::cos(lm) + ax.z.vec() * std::sin(lm)) * -1.0;
      emitQuad(spherePt(Ri, u0, l0), spherePt(Ri, u1, l0), spherePt(Ri, u1, l1),
               spherePt(Ri, u0, l1), inN);
    }
  }
  // 3. OPEN-rim ANNULUS at the cap plane (axial capH): rimIn → rimOut, normal = +capN = −axis
  //    (the cap opening faces OUT of the material along the cap normal).
  const math::Vec3 rimOut3 = ax.z.vec() * -1.0;
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1);
    emitQuad(ringPoint(ax, rimIn, u0, capH), ringPoint(ax, rimIn, u1, capH),
             ringPoint(ax, rimOut, u1, capH), ringPoint(ax, rimOut, u0, capH), rimOut3);
  }
  return polys;
}

}  // namespace detail

// Hollow `solid` to a uniform wall `thickness`, opening the picked cap face(s) `faceIds`
// (1-based, mapShapes order), for a body that is EXACTLY a capped CYLINDER or capped CONE
// FRUSTUM with one cap removed. Returns the hollow tube (deflection-bounded planar-facet
// soup, watertight) or a NULL Shape (→ OCCT BRepOffsetAPI_MakeThickSolid) when the body is
// not a pure capped cylinder/frustum, when the removed pick is not exactly one planar cap,
// or when the thickness collapses the cavity (Ri ≤ 0 or H ≤ t). Multiple distinct caps or
// a picked curved wall → NULL.
inline topo::Shape curved_shell(const topo::Shape& solid, const int* faceIds, int faceCount,
                                double thickness, double deflection = 0.01) {
  if (!(thickness > kBlendEps)) return {};
  // 1. Capped CYLINDER / CONE FRUSTUM — one cap open.
  if (const auto body = detail::recogniseShellBody(solid)) {
    const auto hOpen = detail::removedCapHeight(solid, *body, faceIds, faceCount);
    if (hOpen) {
      std::vector<nb::Polygon> polys =
          detail::buildCurvedShell(*body, *hOpen, thickness, deflection);
      if (polys.size() >= 4) return nb::assembleSolid(polys);
    }
    return {};
  }
  // 2. SPHERE-CAP dome (hemisphere / spherical cap) — the single planar cap open, the
  //    concentric inner sphere Ri = Ro − t is the cavity (F2 curved-face extension).
  if (const auto ball = detail::recogniseShellSphere(solid)) {
    const auto cap = detail::removedSphereCap(solid, *ball, faceIds, faceCount);
    if (cap) {
      std::vector<nb::Polygon> polys = detail::buildSphereShell(*ball, thickness, deflection);
      if (polys.size() >= 4) return nb::assembleSolid(polys);
    }
  }
  return {};
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CURVED_SHELL_H
