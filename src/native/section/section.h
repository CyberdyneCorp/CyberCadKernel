// SPDX-License-Identifier: Apache-2.0
//
// section.h — native PLANAR SECTION CURVES of a solid (MOAT M-GS GS2).
//
// `sectionByPlane(solid, cutPlane)` returns the SECTION CURVES a cut plane carves
// out of a B-rep solid — NOT the cut solid. For every face of the solid the cut
// plane is intersected with the face's surface (closed-form conics for the
// elementary Plane/Cylinder/Cone/Sphere faces, via the landed SSI Stage-S1
// intersector, consumed READ-ONLY), the resulting section edges are CLIPPED to the
// finite face, and the clipped edges are ASSEMBLED into closed loops ordered by
// shared endpoints. An optional planar cap area (the region the loops enclose on
// the cut plane) is returned as `totalArea`.
//
// TWO closed-form gates back this module:
//   (a) HOST ANALYTIC (no OCCT): the loops lie on the cut plane AND on the solid's
//       faces, close, and enclose an area that matches a closed-form value
//       (box section = rectangle, cylinder cross-section = circle, cylinder axial
//       section = rectangle, sphere = great/small circle).
//   (b) SIM native-vs-OCCT: match BRepAlgoAPI_Section (edge length + loop count +
//       closed-ness) and the capped area vs BRepGProp.
//
// The OBLIQUE cut of a cylindrical face is now COMPUTED: the landed SSI
// `intersectPlaneCylinder` returns the correct oblique ellipse (semi-major R/|cosθ|,
// semi-minor R; see plane_conics.h, regression test_native_ssi::plane_cylinder(3b)),
// so GS2 assembles it into a closed Ellipse loop exactly like the circle/great-circle
// conic cases. The perpendicular (circle), oblique (ellipse), and parallel (ruling)
// cylinder cuts are all in scope.
//
// HONEST DECLINE is first-class. This slice NEVER emits a wrong or open section: it
// returns `Declined` (with a measured reason) for configurations it does not
// robustly handle — the plane tangent to a curved face (section does not enclose
// area), a section that does not close, a curved-face conic trimmed by the finite
// face boundary (arc-trim, e.g. an oblique cut that runs off the cylinder's finite
// axial band), and a freeform face (deferred to the marcher, not in this slice).
//
// Header-only, OCCT-FREE. clang++ -std=c++20. Consumes src/native/{math,topology,
// ssi} read-only; adds NOTHING to boolean/ or ssi/.
//
#ifndef CYBERCAD_NATIVE_SECTION_SECTION_H
#define CYBERCAD_NATIVE_SECTION_SECTION_H

#include "native/math/elementary.h"
#include "native/math/vec.h"
#include "native/ssi/dispatch.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace cybercad::native::section {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;

using math::Dir3;
using math::Point3;
using math::Vec3;

inline constexpr double kPi = 3.14159265358979323846;

// Tolerances. Section geometry is exact fp64 for the elementary targets; these are
// the "same point / on plane / on surface" thresholds the self-verify gate uses.
inline constexpr double kWeldTol = 1e-7;   ///< endpoints within this distance are the same vertex
inline constexpr double kOnTol = 1e-7;     ///< on-plane / on-surface residual bound

// ─────────────────────────────────────────────────────────────────────────────
// Result types.
// ─────────────────────────────────────────────────────────────────────────────

/// The analytic shape of one closed section loop (drives closed-form length/area).
enum class LoopShape {
  Polygon,  ///< straight-segment loop (box, cylinder axial rectangle)
  Circle,   ///< a full circle (cylinder cross-section, sphere great circle, cone base-parallel)
  Ellipse,  ///< a full ellipse (oblique cut of a cylindrical face)
};

/// One closed section loop. `points` is an ordered, densely-sampled polyline (the
/// last point is NOT a duplicate of the first) usable for rendering / OCCT parity;
/// the analytic fields give the exact closed-form length + enclosed area.
struct SectionLoop {
  LoopShape shape = LoopShape::Polygon;
  std::vector<Point3> points;  ///< ordered loop vertices / samples
  bool closed = true;

  // Analytic descriptor (Circle/Ellipse). Unused for Polygon.
  math::Ax3 frame{};      ///< conic centre + plane frame
  double radius = 0.0;    ///< Circle radius
  double a = 0.0, b = 0.0;  ///< Ellipse semi-axes

  /// Exact closed-form perimeter length.
  double length() const noexcept {
    if (shape == LoopShape::Circle) return 2.0 * kPi * radius;
    if (shape == LoopShape::Ellipse) {
      // Ramanujan II — relative error ≪1e-4 at the eccentricities an oblique
      // cylinder cut produces; within the OCCT parity harness's edge-length tol.
      const double h = ((a - b) * (a - b)) / ((a + b) * (a + b));
      return kPi * (a + b) * (1.0 + 3.0 * h / (10.0 + std::sqrt(4.0 - 3.0 * h)));
    }
    double L = 0.0;
    const std::size_t n = points.size();
    for (std::size_t i = 0; i < n; ++i)
      L += math::distance(points[i], points[(i + 1) % n]);
    return L;
  }

  /// Exact closed-form area enclosed on the cut plane. For a polygon this is the
  /// shoelace area in the cut-plane 2D frame (`px`,`py` = the plane's X,Y axes).
  double area(const Dir3& px, const Dir3& py, const Point3& po) const noexcept {
    if (shape == LoopShape::Circle) return kPi * radius * radius;
    if (shape == LoopShape::Ellipse) return kPi * a * b;
    double s = 0.0;
    const std::size_t n = points.size();
    for (std::size_t i = 0; i < n; ++i) {
      const Vec3 wi = points[i] - po, wj = points[(i + 1) % n] - po;
      const double ui = math::dot(wi, px.vec()), vi = math::dot(wi, py.vec());
      const double uj = math::dot(wj, px.vec()), vj = math::dot(wj, py.vec());
      s += ui * vj - uj * vi;
    }
    return std::fabs(s) * 0.5;
  }
};

enum class SectionStatus {
  Ok,        ///< ≥1 closed section loop produced
  Empty,     ///< the plane provably misses the solid (no section)
  Declined,  ///< a configuration this slice does not robustly handle (see `reason`)
};

struct SectionResult {
  SectionStatus status = SectionStatus::Declined;
  std::string reason;                 ///< decline / empty explanation (measured gap)
  std::vector<SectionLoop> loops;     ///< closed section loops
  math::Ax3 planeFrame{};             ///< the cut plane's frame (for area projection)

  int loopCount() const noexcept { return static_cast<int>(loops.size()); }
  double totalLength() const noexcept {
    double L = 0.0;
    for (const auto& lp : loops) L += lp.length();
    return L;
  }
  /// Capped planar section area (sum of loop areas on the cut plane).
  double totalArea() const noexcept {
    double A = 0.0;
    for (const auto& lp : loops) A += lp.area(planeFrame.x, planeFrame.y, planeFrame.origin);
    return A;
  }
  bool ok() const noexcept { return status == SectionStatus::Ok; }

  static SectionResult declined(std::string why) {
    SectionResult r; r.status = SectionStatus::Declined; r.reason = std::move(why); return r;
  }
  static SectionResult empty(std::string why) {
    SectionResult r; r.status = SectionStatus::Empty; r.reason = std::move(why); return r;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// detail — read-only geometry consumption.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

/// A tiny linear epsilon for "direction parallel / degenerate" guards.
inline constexpr double kTinyEps = 1e-12;

/// Place a local Ax3 frame into world coordinates by a topology Location.
inline math::Ax3 placeFrame(const math::Ax3& f, const topo::Location& loc) {
  if (loc.isIdentity()) return f;
  const math::Transform& t = loc.transform();
  return math::Ax3{t.applyToPoint(f.origin), t.applyToDir(f.x), t.applyToDir(f.y),
                   t.applyToDir(f.z)};
}

/// The elementary surface kinds this slice handles analytically.
inline bool isElementary(topo::FaceSurface::Kind k) {
  using K = topo::FaceSurface::Kind;
  return k == K::Plane || k == K::Cylinder || k == K::Cone || k == K::Sphere;
}

/// Build the ssi::Surface for an (analytic) face, world-placed. nullopt for
/// freeform (BSpline/Bezier/Torus) — deferred to the marcher, out of this slice.
inline std::optional<ssi::Surface> toSsiSurface(const topo::FaceSurface& s,
                                                const topo::Location& loc) {
  using K = topo::FaceSurface::Kind;
  const math::Ax3 fr = placeFrame(s.frame, loc);
  switch (s.kind) {
    case K::Plane:    return ssi::Surface::of(math::Plane{fr});
    case K::Cylinder: return ssi::Surface::of(math::Cylinder{fr, s.radius});
    case K::Cone:     return ssi::Surface::of(math::Cone{fr, s.radius, s.semiAngle});
    case K::Sphere:   return ssi::Surface::of(math::Sphere{fr, s.radius});
    default:          return std::nullopt;  // Torus / BSpline / Bezier
  }
}

/// World-placed sample of an analytic edge curve at parameter t. Only Line/Circle
/// are needed for the elementary-solid boundary edges this slice reads; other
/// kinds return nullopt (their host faces are handled by the closed-conic path,
/// which needs no boundary crossing).
inline std::optional<Point3> edgePoint(const topo::EdgeCurve& c, const topo::Location& loc,
                                       double t) {
  const math::Ax3 fr = placeFrame(c.frame, loc);
  using EK = topo::EdgeCurve::Kind;
  if (c.kind == EK::Line) return fr.origin + fr.x.vec() * t;
  if (c.kind == EK::Circle)
    return fr.origin + fr.x.vec() * (c.radius * std::cos(t)) +
           fr.y.vec() * (c.radius * std::sin(t));
  if (c.kind == EK::Ellipse)
    return fr.origin + fr.x.vec() * (c.radius * std::cos(t)) +
           fr.y.vec() * (c.minorRadius * std::sin(t));
  return std::nullopt;
}

/// Signed distance of a point to the cut plane.
inline double planeResidual(const math::Plane& pl, const Point3& p) {
  return math::dot(p - pl.pos.origin, pl.pos.z.vec());
}

/// All crossing points of the cut plane with ONE boundary edge (within its param
/// range). Line → 0/1 point; Circle/Ellipse → 0..2 points. Unsupported edge kinds
/// contribute nothing (safe: the closed-conic path does not depend on them).
inline void crossPlaneEdge(const topo::EdgeCurveResult& e, const math::Plane& pl,
                           std::vector<Point3>& out) {
  const topo::EdgeCurve& c = *e.curve;
  const math::Ax3 fr = placeFrame(c.frame, e.location);
  const Vec3 n = pl.pos.z.vec();
  using EK = topo::EdgeCurve::Kind;

  if (c.kind == EK::Line) {
    const double dn = math::dot(fr.x.vec(), n);
    if (std::fabs(dn) <= kTinyEps) return;  // parallel — no isolated crossing
    const double t = -math::dot(fr.origin - pl.pos.origin, n) / dn;
    if (t >= e.first - kWeldTol && t <= e.last + kWeldTol)
      out.push_back(fr.origin + fr.x.vec() * t);
    return;
  }
  if (c.kind == EK::Circle || c.kind == EK::Ellipse) {
    const double ra = c.radius;
    const double rb = (c.kind == EK::Circle) ? c.radius : c.minorRadius;
    // A cos t + B sin t = -K, with A = n·(ra·X), B = n·(rb·Y), K = n·(centre - O).
    const double A = ra * math::dot(fr.x.vec(), n);
    const double B = rb * math::dot(fr.y.vec(), n);
    const double K = math::dot(fr.origin - pl.pos.origin, n);
    const double M = std::hypot(A, B);
    if (M <= kTinyEps) return;               // edge plane ∥ cut plane in-band
    const double rhs = -K / M;
    if (rhs < -1.0 - 1e-12 || rhs > 1.0 + 1e-12) return;  // no crossing
    const double base = std::atan2(B, A);
    const double d = std::acos(std::max(-1.0, std::min(1.0, rhs)));
    for (double t : {base + d, base - d}) {
      // Normalise into the edge's [first,last] range (mod 2π) before the range test.
      double tt = t;
      while (tt < e.first - 1e-12) tt += 2.0 * kPi;
      while (tt > e.last + 1e-12) tt -= 2.0 * kPi;
      if (tt >= e.first - kWeldTol && tt <= e.last + kWeldTol) {
        if (auto p = edgePoint(c, e.location, tt)) out.push_back(*p);
      }
    }
    return;
  }
  // BSpline/Bezier boundary edges: not needed by this slice's closed-conic path.
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// An intermediate (pre-assembly) section edge: a clipped, ordered polyline, plus
// an analytic tag for closed conics (a face that yields a complete interior loop).
// ─────────────────────────────────────────────────────────────────────────────
struct SectionEdge {
  std::vector<Point3> points;  ///< ordered, ≥2 points
  bool closedLoop = false;     ///< true ⇒ this edge is ALREADY a full closed loop
  LoopShape shape = LoopShape::Polygon;
  math::Ax3 frame{};
  double radius = 0.0, a = 0.0, b = 0.0;
};

namespace detail {

/// Distance of a point to a Line intersection curve.
inline double lineResidual(const ssi::IntersectionCurve& L, const Point3& p) {
  return math::norm(math::cross((p - L.frame.origin), L.frame.x.vec()));
}

/// A dense polyline sampling of a full closed conic (Circle/Ellipse), n samples,
/// last point NOT duplicating the first.
inline std::vector<Point3> sampleClosedConic(const ssi::IntersectionCurve& c, int n) {
  std::vector<Point3> pts;
  pts.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) pts.push_back(c.value(2.0 * kPi * i / n));
  return pts;
}

/// Axial extent [min,max] of a face along `axis` from `axisO`, taken over its
/// boundary vertices AND densely-sampled boundary edge curves (so a rim CIRCLE
/// edge that carries no explicit vertices still bounds the band). nullopt if the
/// face exposes no readable boundary geometry.
inline std::optional<std::pair<double, double>> faceAxialExtent(const topo::Shape& face,
                                                                const Vec3& axis,
                                                                const Point3& axisO) {
  double lo = 1e300, hi = -1e300;
  bool any = false;
  auto acc = [&](const Point3& p) {
    const double a = math::dot(p - axisO, axis);
    lo = std::min(lo, a); hi = std::max(hi, a); any = true;
  };
  for (topo::Explorer v(face, topo::ShapeType::Vertex); v.more(); v.next())
    if (auto p = topo::pointOf(v.current())) acc(*p);
  for (topo::Explorer e(face, topo::ShapeType::Edge); e.more(); e.next()) {
    if (auto ec = topo::curveOf(e.current())) {
      constexpr int kN = 8;
      for (int i = 0; i <= kN; ++i) {
        const double t = ec->first + (ec->last - ec->first) * i / kN;
        if (auto p = edgePoint(*ec->curve, ec->location, t)) acc(*p);
      }
    }
  }
  if (!any) return std::nullopt;
  return std::make_pair(lo, hi);
}

/// Collect (deduped by weld) all crossing points of `pl` with `face`'s boundary edges.
inline std::vector<Point3> faceBoundaryCrossings(const topo::Shape& face, const math::Plane& pl) {
  std::vector<Point3> xs;
  for (topo::Explorer e(face, topo::ShapeType::Edge); e.more(); e.next()) {
    if (auto ec = topo::curveOf(e.current())) crossPlaneEdge(*ec, pl, xs);
  }
  // Dedup coincident crossings (shared corner reported by two edges).
  std::vector<Point3> uniq;
  for (const Point3& p : xs) {
    bool seen = false;
    for (const Point3& q : uniq)
      if (math::distance(p, q) <= kWeldTol) { seen = true; break; }
    if (!seen) uniq.push_back(p);
  }
  return uniq;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// Per-face section: turn one face's intersection conics into clipped SectionEdges.
// Returns false (via `ok`) when the face's cut is outside this slice's envelope —
// the caller then DECLINES the whole section (never a partial / wrong result).
// ─────────────────────────────────────────────────────────────────────────────
inline bool sectionOneFace(const topo::Shape& face, const topo::FaceSurface& fs,
                           const topo::Location& floc, const math::Plane& cut,
                           const ssi::IntersectionResult& xr, std::vector<SectionEdge>& out,
                           std::string& why) {
  const std::vector<Point3> crossings = detail::faceBoundaryCrossings(face, cut);

  for (const ssi::IntersectionCurve& c : xr.curves) {
    const bool closedConic =
        c.kind == ssi::CurveKind::Circle || c.kind == ssi::CurveKind::Ellipse;

    if (c.kind == ssi::CurveKind::Line) {
      // Open segment: keep the two boundary crossings that lie ON this line.
      std::vector<Point3> ends;
      for (const Point3& p : crossings)
        if (detail::lineResidual(c, p) <= kOnTol) ends.push_back(p);
      // A cut line that misses this finite planar face (no boundary crossings)
      // contributes no edge — that is not an error (e.g. the oblique cut plane
      // meets a cap's infinite plane in a Line that misses the finite cap disk).
      if (ends.empty()) continue;
      if (ends.size() != 2) {
        why = "section: a planar/ruled face contributes an open edge with " +
              std::to_string(ends.size()) + " boundary crossings (expected 2) — declined";
        return false;
      }
      SectionEdge se;
      se.points = {ends[0], ends[1]};
      out.push_back(std::move(se));
      continue;
    }

    if (closedConic) {
      // A full-revolution curved face (the elementary cylinder/cone/sphere lateral
      // surface) yields a COMPLETE conic loop iff the whole conic lies within the
      // face's FINITE axial band. We test the entire sampled conic (not just its
      // centre) so a tilted conic that runs off the finite patch is caught. This is
      // robust to the parametric SEAM edge (a false "boundary" that does not trim
      // the loop), so — unlike the open path — it does NOT consult boundary
      // crossings. (Partial-revolution curved faces are outside this slice.)
      std::vector<Point3> samples = detail::sampleClosedConic(c, 180);
      using K = topo::FaceSurface::Kind;
      if (fs.kind == K::Cylinder || fs.kind == K::Cone) {
        const math::Ax3 sf = detail::placeFrame(fs.frame, floc);
        const auto ext = detail::faceAxialExtent(face, sf.z.vec(), sf.origin);
        if (!ext) {
          why = "section: curved face has no readable extent (declined)";
          return false;
        }
        double amin = 1e300, amax = -1e300;
        for (const Point3& p : samples) {
          const double a = math::dot(p - sf.origin, sf.z.vec());
          amin = std::min(amin, a); amax = std::max(amax, a);
        }
        const bool below = amax < ext->first - kOnTol;
        const bool above = amin > ext->second + kOnTol;
        if (below || above) continue;  // plane misses the finite face → no loop
        const bool inside = amin >= ext->first - kOnTol && amax <= ext->second + kOnTol;
        if (!inside) {
          why = "section: curved-face conic is trimmed by the finite face boundary — "
                "arc-trim not supported in this slice (declined)";
          return false;
        }
      }
      SectionEdge se;
      se.closedLoop = true;
      se.shape = (c.kind == ssi::CurveKind::Circle) ? LoopShape::Circle : LoopShape::Ellipse;
      se.frame = c.frame;
      se.radius = c.radius;
      se.a = c.a; se.b = c.b;
      se.points = std::move(samples);
      out.push_back(std::move(se));
      continue;
    }

    // Parabola / Hyperbola (open cone sections that meet the base) — the arc-trim
    // path is out of this first slice.
    why = "section: open conic (parabola/hyperbola) arc-trim not supported this slice (declined)";
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Assemble open SectionEdges into closed polygon loops by shared endpoints.
// Returns nullopt if any chain fails to close (honest — the section is open).
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<std::vector<SectionLoop>> assembleOpenLoops(
    const std::vector<SectionEdge>& edges) {
  std::vector<SectionLoop> loops;
  std::vector<bool> used(edges.size(), false);

  for (std::size_t i = 0; i < edges.size(); ++i) {
    if (used[i]) continue;
    std::vector<Point3> loop = edges[i].points;
    used[i] = true;
    const Point3 start = loop.front();
    Point3 end = loop.back();

    bool closed = false;
    for (std::size_t guard = 0; guard < edges.size() + 1; ++guard) {
      if (math::distance(end, start) <= kWeldTol && loop.size() >= 3) { closed = true; break; }
      bool extended = false;
      for (std::size_t j = 0; j < edges.size(); ++j) {
        if (used[j]) continue;
        const Point3 a = edges[j].points.front();
        const Point3 b = edges[j].points.back();
        const bool matchA = math::distance(a, end) <= kWeldTol;
        const bool matchB = math::distance(b, end) <= kWeldTol;
        if (!matchA && !matchB) continue;
        // Append the far endpoint's points (skip the shared vertex).
        const auto& pts = edges[j].points;
        if (matchA)
          for (std::size_t k = 1; k < pts.size(); ++k) loop.push_back(pts[k]);
        else
          for (std::size_t k = pts.size(); k-- > 1;) loop.push_back(pts[k - 1]);
        end = loop.back();
        used[j] = true; extended = true; break;
      }
      if (!extended) break;
    }
    if (!closed) return std::nullopt;
    // Drop the closing duplicate vertex (loop.back ≈ loop.front).
    if (loop.size() >= 2 && math::distance(loop.front(), loop.back()) <= kWeldTol)
      loop.pop_back();
    SectionLoop sl;
    sl.shape = LoopShape::Polygon;
    sl.points = std::move(loop);
    sl.closed = true;
    loops.push_back(std::move(sl));
  }
  return loops;
}

// ─────────────────────────────────────────────────────────────────────────────
// sectionByPlane — THE entry point. Intersect every face of `solid` with `cut`,
// clip + assemble into closed loops, and SELF-VERIFY (closed + on the cut plane).
// Returns Declined (measured reason) rather than a wrong / open section.
// ─────────────────────────────────────────────────────────────────────────────
inline SectionResult sectionByPlane(const topo::Shape& solid, const math::Plane& cut) {
  SectionResult result;
  result.planeFrame = cut.pos;
  const ssi::Surface cutSurf = ssi::Surface::of(cut);

  std::vector<SectionEdge> openEdges;
  std::vector<SectionLoop> closedLoops;

  for (topo::Explorer fx(solid, topo::ShapeType::Face); fx.more(); fx.next()) {
    const topo::Shape& face = fx.current();
    const auto fsr = topo::surfaceOf(face);
    if (!fsr || fsr->surface == nullptr)
      return SectionResult::declined("section: a face carries no surface (declined)");
    const topo::FaceSurface& fs = *fsr->surface;

    if (!detail::isElementary(fs.kind))
      return SectionResult::declined(
          "section: freeform/torus face — deferred to the SSI marcher, not in this slice (declined)");

    const auto surf = detail::toSsiSurface(fs, fsr->location);
    if (!surf) return SectionResult::declined("section: unsupported face surface (declined)");

    const ssi::IntersectionResult xr = ssi::intersect_surfaces(cutSurf, *surf);
    if (xr.status == ssi::IntersectionStatus::NoIntersection) continue;
    if (xr.status == ssi::IntersectionStatus::Coincident)
      return SectionResult::declined(
          "section: cut plane coincident with a planar face — degenerate (declined)");
    if (xr.status != ssi::IntersectionStatus::Ok)
      return SectionResult::declined("section: face intersection not analytic (declined)");

    std::vector<SectionEdge> faceEdges;
    std::string why;
    if (!sectionOneFace(face, fs, fsr->location, cut, xr, faceEdges, why))
      return SectionResult::declined(why);

    for (SectionEdge& se : faceEdges) {
      if (se.closedLoop) {
        SectionLoop sl;
        sl.shape = se.shape; sl.points = std::move(se.points); sl.closed = true;
        sl.frame = se.frame; sl.radius = se.radius; sl.a = se.a; sl.b = se.b;
        closedLoops.push_back(std::move(sl));
      } else {
        openEdges.push_back(std::move(se));
      }
    }
  }

  // Assemble the open (straight/ruled) edges into polygon loops.
  std::vector<SectionLoop> loops = std::move(closedLoops);
  if (!openEdges.empty()) {
    auto assembled = assembleOpenLoops(openEdges);
    if (!assembled)
      return SectionResult::declined("section: open edges do not close into a loop (declined)");
    for (auto& lp : *assembled) loops.push_back(std::move(lp));
  }

  if (loops.empty())
    return SectionResult::empty("section: the cut plane misses the solid (no section)");

  // SELF-VERIFY: every loop closed + every sample on the cut plane.
  for (const SectionLoop& lp : loops) {
    if (lp.points.size() < 3)
      return SectionResult::declined("section: degenerate loop (< 3 points) (declined)");
    for (const Point3& p : lp.points) {
      if (std::fabs(detail::planeResidual(cut, p)) > kOnTol)
        return SectionResult::declined("section: a loop point is off the cut plane (declined)");
    }
  }

  result.status = SectionStatus::Ok;
  result.loops = std::move(loops);
  return result;
}

}  // namespace cybercad::native::section

#endif  // CYBERCAD_NATIVE_SECTION_SECTION_H
