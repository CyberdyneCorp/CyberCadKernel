// SPDX-License-Identifier: Apache-2.0
//
// surface_eval.h — one uniform (u,v) evaluator over the native FaceSurface
// variants, wrapping the src/native/math surface routines.
//
// A face's surface is a topology::FaceSurface (analytic Plane/Cylinder/Cone/
// Sphere, or a free-form BSpline/Bezier grid). The mesher needs, at any (u,v):
//   * the 3D point            S(u,v)                 — to place a vertex,
//   * the first derivatives   ∂S/∂u, ∂S/∂v           — for the tangent frame,
//   * the unit normal         n = (Sᵤ×Sᵥ)/‖·‖        — for per-vertex normals,
//   * the natural (u,v) bounds of the parametrization — the sampling box,
//   * a curvature magnitude   ‖∂²S/∂u²‖, ‖∂²S/∂v²‖    — for the deflection step.
// This header maps each variant onto the right math evaluator and applies the
// face's Location so results are WORLD-placed (like BRep_Tool bakes the location
// into the returned geometry). It is the ONLY place the FaceSurface variant is
// switched, keeping the mesher surface-agnostic (Visitor-style dispatch, so the
// mesher's cognitive complexity stays low).
//
// fp64 CPU path is the SOURCE OF TRUTH. (The GPU fp32 sampler in gpu_sample.h
// may substitute point evaluation for eligible faces; correctness holds here.)
//
// OCCT-FREE. Uses src/native/math + src/native/topology. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_SURFACE_EVAL_H
#define CYBERCAD_NATIVE_TESSELLATE_SURFACE_EVAL_H

#include "native/math/native_math.h"
#include "native/topology/shape.h"

#include <array>
#include <cmath>

namespace cybercad::native::tessellate {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

/// A first-order surface sample at (u,v), world-placed.
struct SurfaceSample {
  math::Point3 point;   ///< S(u,v)
  math::Vec3 du;        ///< ∂S/∂u
  math::Vec3 dv;        ///< ∂S/∂v
  math::Dir3 normal;    ///< unit (Sᵤ×Sᵥ); .valid()==false on a degeneracy
};

/// Natural parametric bounds of a surface. Periodic directions (cylinder/cone U,
/// sphere U) span a full turn; the caller clamps/overrides with the face's real
/// trimmed range where available.
struct UVBounds {
  double uMin = 0.0;
  double uMax = 1.0;
  double vMin = 0.0;
  double vMax = 1.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// SurfaceEvaluator — binds a FaceSurface + its Location, exposes value/d1/normal
// /curvature at (u,v). Built once per face; cheap to copy.
// ─────────────────────────────────────────────────────────────────────────────
class SurfaceEvaluator {
 public:
  SurfaceEvaluator(const topo::FaceSurface& s, const topo::Location& loc) noexcept
      : s_(s), loc_(loc) {}

  const topo::FaceSurface& surface() const noexcept { return s_; }

  /// World-placed point S(u,v).
  math::Point3 value(double u, double v) const noexcept {
    return place(localValue(u, v));
  }

  /// First-order sample: point + ∂u + ∂v + unit normal, world-placed. Directions
  /// are mapped through the linear part of the location (translation is dropped
  /// for vectors, matching Transform::applyToVector).
  SurfaceSample d1(double u, double v) const noexcept {
    LocalD1 d = localD1(u, v);
    SurfaceSample out;
    out.point = place(d.point);
    out.du = placeVec(d.du);
    out.dv = placeVec(d.dv);
    out.normal = math::Dir3{math::cross(out.du, out.dv)};
    return out;
  }

  /// Natural (u,v) bounds for the parametrization. Free-form surfaces use their
  /// knot/parameter span (0..1 for Bézier); analytic periodic directions use a
  /// full turn.
  UVBounds bounds() const noexcept;

  /// A conservative per-direction second-derivative magnitude used to size the
  /// sampling step from a deflection bound (see face_mesher chordDeviationStep).
  /// For analytic surfaces this is exact at (u,v); for free-form it is a finite
  /// difference of the first derivative. Returns {‖Sᵤᵤ‖, ‖Sᵥᵥ‖, ‖Sᵤᵥ‖} — the
  /// MIXED term ‖Sᵤᵥ‖ (the twist) is what makes a ruled/bilinear saddle patch
  /// (Sᵤᵤ = Sᵥᵥ = 0 but Sᵤᵥ ≠ 0) subdivide; without it a twisted loft side face
  /// would mesh as a single flat quad and its enclosed volume would be wrong.
  std::array<double, 3> curvatureMagnitude(double u, double v) const noexcept;

 private:
  struct LocalD1 {
    math::Point3 point;
    math::Vec3 du;
    math::Vec3 dv;
  };

  // ── World placement of a local result ──────────────────────────────────────
  math::Point3 place(const math::Point3& p) const noexcept {
    return loc_.isIdentity() ? p : loc_.transform().applyToPoint(p);
  }
  math::Vec3 placeVec(const math::Vec3& v) const noexcept {
    return loc_.isIdentity() ? v : loc_.transform().applyToVector(v);
  }

  // ── Local (un-placed) evaluation dispatched by surface kind ─────────────────
  math::Point3 localValue(double u, double v) const noexcept;
  LocalD1 localD1(double u, double v) const noexcept;

  const topo::FaceSurface& s_;
  topo::Location loc_;
};

// ── Analytic helpers: build the math surface object from the payload ──────────
namespace detail {

inline math::Plane asPlane(const topo::FaceSurface& s) noexcept { return math::Plane{s.frame}; }
inline math::Cylinder asCylinder(const topo::FaceSurface& s) noexcept {
  return math::Cylinder{s.frame, s.radius};
}
inline math::Cone asCone(const topo::FaceSurface& s) noexcept {
  return math::Cone{s.frame, s.radius, s.semiAngle};
}
inline math::Sphere asSphere(const topo::FaceSurface& s) noexcept {
  return math::Sphere{s.frame, s.radius};
}

// Free-form (B-spline) surface grid view onto the payload poles.
inline math::SurfaceGrid gridOf(const topo::FaceSurface& s) noexcept {
  return math::SurfaceGrid{std::span<const math::Point3>(s.poles.data(), s.poles.size()),
                           s.nPolesU, s.nPolesV};
}

}  // namespace detail

// ── localValue ────────────────────────────────────────────────────────────────
inline math::Point3 SurfaceEvaluator::localValue(double u, double v) const noexcept {
  using K = topo::FaceSurface::Kind;
  switch (s_.kind) {
    case K::Plane:    return detail::asPlane(s_).value(u, v);
    case K::Cylinder: return detail::asCylinder(s_).value(u, v);
    case K::Cone:     return detail::asCone(s_).value(u, v);
    case K::Sphere:   return detail::asSphere(s_).value(u, v);
    case K::Bezier:
      return s_.weights.empty()
                 ? math::bezierSurfacePoint({s_.poles.data(), s_.poles.size()}, s_.nPolesU,
                                            s_.nPolesV, u, v)
                 : math::rationalBezierSurfacePoint({s_.poles.data(), s_.poles.size()},
                                                    {s_.weights.data(), s_.weights.size()},
                                                    s_.nPolesU, s_.nPolesV, u, v);
    case K::BSpline:
    default:
      return s_.weights.empty()
                 ? math::surfacePoint(s_.degreeU, s_.degreeV, detail::gridOf(s_),
                                      {s_.knotsU.data(), s_.knotsU.size()},
                                      {s_.knotsV.data(), s_.knotsV.size()}, u, v)
                 : math::nurbsSurfacePoint(s_.degreeU, s_.degreeV, detail::gridOf(s_),
                                           {s_.weights.data(), s_.weights.size()},
                                           {s_.knotsU.data(), s_.knotsU.size()},
                                           {s_.knotsV.data(), s_.knotsV.size()}, u, v);
  }
}

// ── localD1 ─────────────────────────────────────────────────────────────────--
inline SurfaceEvaluator::LocalD1 SurfaceEvaluator::localD1(double u, double v) const noexcept {
  using K = topo::FaceSurface::Kind;
  switch (s_.kind) {
    case K::Plane: {
      auto s = detail::asPlane(s_);
      return {s.value(u, v), s.dU(u, v), s.dV(u, v)};
    }
    case K::Cylinder: {
      auto s = detail::asCylinder(s_);
      return {s.value(u, v), s.dU(u, v), s.dV(u, v)};
    }
    case K::Cone: {
      auto s = detail::asCone(s_);
      return {s.value(u, v), s.dU(u, v), s.dV(u, v)};
    }
    case K::Sphere: {
      auto s = detail::asSphere(s_);
      return {s.value(u, v), s.dU(u, v), s.dV(u, v)};
    }
    case K::Bezier: {
      // Bézier D1 (non-rational) is available directly; for rational we fall to
      // a central difference of the point (rare — kept correct, not fastest).
      if (s_.weights.empty()) {
        auto d = math::bezierSurfaceD1({s_.poles.data(), s_.poles.size()}, s_.nPolesU, s_.nPolesV,
                                       u, v);
        return {d.point, d.du, d.dv};
      }
      break;
    }
    case K::BSpline: {
      std::array<math::Vec3, 9> out{};  // (maxDeriv+1)²=9 for maxDeriv=2
      if (s_.weights.empty())
        math::surfaceDerivs(s_.degreeU, s_.degreeV, detail::gridOf(s_),
                            {s_.knotsU.data(), s_.knotsU.size()},
                            {s_.knotsV.data(), s_.knotsV.size()}, u, v, /*maxDeriv=*/2, out);
      else
        math::nurbsSurfaceDerivs(s_.degreeU, s_.degreeV, detail::gridOf(s_),
                                 {s_.weights.data(), s_.weights.size()},
                                 {s_.knotsU.data(), s_.knotsU.size()},
                                 {s_.knotsV.data(), s_.knotsV.size()}, u, v, 2, out);
      // Layout row-major (maxDeriv+1): out[k*3+l] = ∂^(k+l)/∂u^k∂v^l.
      const math::Vec3 s00 = out[0];  // point
      const math::Vec3 s10 = out[3];  // ∂u
      const math::Vec3 s01 = out[1];  // ∂v
      return {math::Point3{s00.x, s00.y, s00.z}, s10, s01};
    }
    default:
      break;
  }
  // Fallback (rational Bézier or unmodelled): central difference for derivatives.
  const double h = 1e-6;
  const math::Point3 p = localValue(u, v);
  const math::Vec3 du = (localValue(u + h, v) - localValue(u - h, v)) / (2 * h);
  const math::Vec3 dv = (localValue(u, v + h) - localValue(u, v - h)) / (2 * h);
  return {p, du, dv};
}

// ── bounds ────────────────────────────────────────────────────────────────────
inline UVBounds SurfaceEvaluator::bounds() const noexcept {
  using K = topo::FaceSurface::Kind;
  constexpr double kTwoPi = 6.28318530717958647692;
  switch (s_.kind) {
    case K::Cylinder: return {0.0, kTwoPi, 0.0, 1.0};   // v overridden by trim range
    case K::Cone:     return {0.0, kTwoPi, 0.0, 1.0};
    case K::Sphere:   return {0.0, kTwoPi, -kTwoPi / 4.0, kTwoPi / 4.0};  // v∈[-π/2,π/2]
    case K::Bezier:   return {0.0, 1.0, 0.0, 1.0};
    case K::BSpline:  {
      UVBounds b;
      if (!s_.knotsU.empty()) { b.uMin = s_.knotsU.front(); b.uMax = s_.knotsU.back(); }
      if (!s_.knotsV.empty()) { b.vMin = s_.knotsV.front(); b.vMax = s_.knotsV.back(); }
      return b;
    }
    case K::Plane:
    default:          return {0.0, 1.0, 0.0, 1.0};      // overridden by trim range
  }
}

// ── curvatureMagnitude ──────────────────────────────────────────────────────--
inline std::array<double, 3> SurfaceEvaluator::curvatureMagnitude(double u,
                                                                  double v) const noexcept {
  // Central second difference of the LOCAL point (curvature is location-invariant
  // for rigid motions; a non-uniform scale only rescales it, which is acceptable
  // for a conservative step estimate).
  const UVBounds b = bounds();
  const double hu = std::max((b.uMax - b.uMin) * 1e-4, 1e-7);
  const double hv = std::max((b.vMax - b.vMin) * 1e-4, 1e-7);
  const math::Point3 c = localValue(u, v);
  const math::Vec3 suu =
      (localValue(u + hu, v).asVec() - 2.0 * c.asVec() + localValue(u - hu, v).asVec()) / (hu * hu);
  const math::Vec3 svv =
      (localValue(u, v + hv).asVec() - 2.0 * c.asVec() + localValue(u, v - hv).asVec()) / (hv * hv);
  // Mixed derivative Sᵤᵥ via the central cross difference
  // [f(+,+) − f(+,−) − f(−,+) + f(−,−)] / (4·hu·hv). Nonzero for a twisted
  // (saddle / ruled) patch even when suu = svv = 0.
  const math::Vec3 suv =
      (localValue(u + hu, v + hv).asVec() - localValue(u + hu, v - hv).asVec() -
       localValue(u - hu, v + hv).asVec() + localValue(u - hu, v - hv).asVec()) /
      (4.0 * hu * hv);
  return {math::norm(suu), math::norm(svv), math::norm(suv)};
}

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_SURFACE_EVAL_H
