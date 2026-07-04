// SPDX-License-Identifier: Apache-2.0
//
// ssi_boolean.h — SSI Stage S5-a: the GENERAL, SSI-curve-driven curved boolean for
// TRANSVERSAL ELEMENTARY curved face pairs (openspec change
// add-native-ssi-curved-boolean; SSI-ROADMAP.md S5).
//
// ── WHAT THIS IS (and is NOT) ─────────────────────────────────────────────────
// The analytic curved.h path is PATTERN-MATCHED: it recognises a box/cylinder
// configuration and builds the answer per-primitive; it never reads an SSI curve.
// S5-a is the SIBLING, GENERAL path: it drives the boolean from the S3 `TraceSet`
// (src/native/ssi/marching.h) — the traced intersection curves (WLines), each node
// carrying its (u,v) on BOTH surfaces and a fitted B-spline seam. Because the split
// is driven by the traced curve rather than a hand-matched primitive, the SAME code
// generalises across the transversal elementary family (cylinder / sphere / cone /
// plane pairs) instead of enumerating each primitive by hand.
//
// It is HONESTLY NARROW. It consumes ONLY a fully-transversal trace
// (`nearTangentGaps == 0`, every consumed WLine Closed or BoundaryExit). Near-tangent
// / coincident / branch-point / freeform pairs are DEFERRED to S4 and fall back to
// OCCT — this library returns a NULL Shape and the ENGINE (native_engine.cpp) owns
// the OCCT fallthrough plus the mandatory watertight + correct-volume self-verify.
// Nothing here is faked or hand-tuned: any stage that cannot proceed robustly returns
// NULL, and the honest gap is what the roadmap calls the S4 seam.
//
// ── PIPELINE (design.md §Pipeline) ────────────────────────────────────────────
//   0. GATE + TRACE. Recover each operand's elementary curved wall from its native
//      B-rep, build the two ssi::SurfaceAdapters, and get the ssi::TraceSet. Proceed
//      ONLY if fully transversal (nearTangentGaps == 0, all WLines Closed/BoundaryExit,
//      ≥ 1 seam); else NULL → OCCT.
//   1. SPLIT. Cut each curved wall along its seam WLine's per-node (u,v) track — the
//      curved analogue of bsp.h splitPolygon — into an INSIDE-the-other piece and an
//      OUTSIDE piece. Each piece keeps its parent's EXACT surface kind (no faceting);
//      the fitted seam B-spline is the shared boundary.
//   2. CLASSIFY. Tag each fragment INSIDE / OUTSIDE / ON the OTHER solid with a curved
//      point-in-solid test at an interior UV sample (the bsp.h side-of-boundary idea
//      generalised to curved half-spaces). An ON sample → out of scope → NULL → OCCT.
//   3. SELECT. Same set algebra as planar booleanPolygons: fuse = out∪out,
//      cut = out(A)∪in(B)-reversed, common = in∩in.
//   4. WELD. Sew the surviving wall fragments + the shared seam into one Solid,
//      welding coincident seam vertices so the two faces meet watertight along the
//      curved seam.
//
// ── COMPLEXITY ────────────────────────────────────────────────────────────────
// The irreducible seam-split geometry is isolated in ssidetail:: and flagged; the
// driver ssi_boolean_solid is a short linear composition.
//
// SUBSTRATE GUARD. The TraceSet comes from S3, whose corrector (least_squares) and
// B-spline fit (lstsq) are native-numerics, so this whole path is compiled only under
// CYBERCAD_HAS_NUMSCI, like native-ssi. Without the substrate ssi_boolean_solid is a
// stub returning NULL (declared always; defined here), so the dispatcher compiles
// either way.
//
// CLEAN-ROOM. Uses src/native/ssi + math + topology + boolean/assemble. No OCCT.
// clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_SSI_BOOLEAN_H
#define CYBERCAD_NATIVE_BOOLEAN_SSI_BOOLEAN_H

#include "native/boolean/native_boolean_fwd.h"   // Op (shared with native_boolean.h)
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <optional>
#include <vector>

#if defined(CYBERCAD_HAS_NUMSCI)
#include "native/math/elementary.h"     // Cylinder / Sphere / Cone / Plane
#include "native/ssi/marching.h"        // TraceSet / WLine (the S5 input contract)
#include "native/ssi/seeding.h"         // makeCylinderAdapter etc. + SurfaceAdapter

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#endif

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace math = cybercad::native::math;

// ─────────────────────────────────────────────────────────────────────────────
// ssi_boolean_solid — the S5-a entry the boolean_solid dispatcher calls when the
// analytic curved.h path declined and at least one operand carries an elementary
// curved face. Returns the assembled native Solid, or a NULL Shape whenever the pair
// is not a robustly-handleable transversal elementary pair (→ engine falls back to
// OCCT). The engine ALWAYS re-verifies (watertight + correct volume) before shipping.
//
// The DECLARATION is always visible; the DEFINITION is compiled only under
// CYBERCAD_HAS_NUMSCI (it consumes the S3 tracer). A build without the substrate
// links a stub that returns NULL, so the dispatcher compiles either way.
// ─────────────────────────────────────────────────────────────────────────────
topo::Shape ssi_boolean_solid(const topo::Shape& a, const topo::Shape& b, Op op);

#if defined(CYBERCAD_HAS_NUMSCI)

namespace ssi = cybercad::native::ssi;

namespace ssidetail {

inline constexpr double kSsiPi = 3.14159265358979323846;
inline constexpr double kSsiTwoPi = 6.28318530717958647692;
inline constexpr double kSsiTol = 1e-6;

// ── An elementary curved solid, recovered from its native B-rep faces ──────────
// S5-a's transversal-elementary domain is a solid whose curved boundary is ONE
// analytic elementary surface (Cylinder / Sphere / Cone) — the shape native construct
// emits and exactly what S3 can trace transversally. We fold the faces into ONE
// analytic surface + its axial extent + the planar cap half-spaces, so we can (a)
// build the SurfaceAdapter S3 needs and (b) run the curved point-in-solid test.
// Anything richer (two distinct curved surfaces, freeform, torus) → nullopt →
// declined.
enum class CurvedKind { Cylinder, Sphere, Cone };

struct CurvedSolid {
  CurvedKind kind = CurvedKind::Cylinder;
  math::Ax3 frame{};        ///< analytic surface placement (world)
  double radius = 0.0;      ///< cylinder/sphere radius, cone reference radius
  double semiAngle = 0.0;   ///< cone half-angle
  double vLo = 0.0;         ///< axial/latitude extent of the wall (surface v param)
  double vHi = 0.0;
  std::vector<std::pair<math::Vec3, math::Point3>> capPlanes;  ///< outward-normal caps

  math::Point3 point(double u, double v) const {
    switch (kind) {
      case CurvedKind::Cylinder: return math::Cylinder{frame, radius}.value(u, v);
      case CurvedKind::Sphere:   return math::Sphere{frame, radius}.value(u, v);
      case CurvedKind::Cone:     return math::Cone{frame, radius, semiAngle}.value(u, v);
    }
    return {};
  }
  ssi::SurfaceAdapter adapter() const {
    const ssi::ParamBox dom{0.0, kSsiTwoPi, vLo, vHi};
    switch (kind) {
      case CurvedKind::Cylinder: return ssi::makeCylinderAdapter(math::Cylinder{frame, radius}, dom);
      case CurvedKind::Sphere:   return ssi::makeSphereAdapter(math::Sphere{frame, radius}, dom);
      case CurvedKind::Cone:     return ssi::makeConeAdapter(math::Cone{frame, radius, semiAngle}, dom);
    }
    return {};
  }
};

// World-place a face's analytic surface frame (fold surface + face location).
inline math::Ax3 worldFrame(const topo::FaceSurfaceResult& surf, const topo::Shape& face) {
  math::Ax3 f = surf.surface->frame;
  auto apply = [](math::Ax3& fr, const topo::Location& loc) {
    if (loc.isIdentity()) return;
    fr.origin = loc.transform().applyToPoint(fr.origin);
    fr.x = math::Dir3{loc.transform().applyToVector(fr.x.vec())};
    fr.y = math::Dir3{loc.transform().applyToVector(fr.y.vec())};
    fr.z = math::Dir3{loc.transform().applyToVector(fr.z.vec())};
  };
  apply(f, surf.location);
  apply(f, face.location());
  return f;
}

// Two analytic frames describe the SAME surface axis (parallel + colinear axes) —
// used to fold a revolve's several curved patches into one analytic surface.
inline bool sameAxis(const math::Ax3& a, const math::Ax3& b, double tol) {
  if (math::norm(math::cross(a.z.vec(), b.z.vec())) > tol) return false;
  const math::Vec3 d = b.origin - a.origin;
  const math::Vec3 perp = d - a.z.vec() * math::dot(d, a.z.vec());
  return math::norm(perp) <= tol;
}

// Recognise a native solid as ONE elementary curved solid (cylinder / sphere / cone
// wall + planar caps). nullopt for a box (no curved face → the planar/analytic paths
// own it), a multi-curved solid, freeform, or torus — all declined → OCCT.
//
// systems-band (~20 — a per-face fold with kind/axis agreement + extent/caps);
// isolated + documented per the complexity policy.
inline std::optional<CurvedSolid> recogniseCurvedSolid(const topo::Shape& s) {
  if (s.isNull()) return std::nullopt;
  std::optional<CurvedSolid> cs;
  std::vector<topo::Shape> planeFaces;

  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) {
    const auto surf = topo::surfaceOf(ex.current());
    if (!surf) return std::nullopt;
    const auto k = surf->surface->kind;
    if (k == topo::FaceSurface::Kind::Plane) { planeFaces.push_back(ex.current()); continue; }
    CurvedKind ck;
    switch (k) {
      case topo::FaceSurface::Kind::Cylinder: ck = CurvedKind::Cylinder; break;
      case topo::FaceSurface::Kind::Sphere:   ck = CurvedKind::Sphere; break;
      case topo::FaceSurface::Kind::Cone:     ck = CurvedKind::Cone; break;
      default: return std::nullopt;  // BSpline / Bezier → freeform → OCCT
    }
    const math::Ax3 fr = worldFrame(*surf, ex.current());
    if (!cs) {
      cs = CurvedSolid{ck, fr, surf->surface->radius, surf->surface->semiAngle, 0, 0, {}};
    } else {
      if (cs->kind != ck) return std::nullopt;
      if (std::fabs(cs->radius - surf->surface->radius) > kSsiTol) return std::nullopt;
      if (!sameAxis(cs->frame, fr, kSsiTol)) return std::nullopt;
    }
  }
  if (!cs) return std::nullopt;  // no curved face → not this path

  // Axial (v) extent from the solid vertices projected into the surface's v param.
  double vLo = std::numeric_limits<double>::infinity(), vHi = -vLo;
  for (topo::Explorer ex(s, topo::ShapeType::Vertex); ex.more(); ex.next()) {
    const auto p = topo::pointOf(ex.current());
    if (!p) continue;
    const math::Vec3 w = *p - cs->frame.origin;
    double v;
    if (cs->kind == CurvedKind::Sphere) {
      const double z = math::dot(w, cs->frame.z.vec());
      v = std::asin(std::clamp(z / std::max(cs->radius, 1e-12), -1.0, 1.0));
    } else {
      v = math::dot(w, cs->frame.z.vec());
    }
    vLo = std::min(vLo, v); vHi = std::max(vHi, v);
  }
  if (!(vHi - vLo > 1e-9)) return std::nullopt;
  cs->vLo = vLo; cs->vHi = vHi;

  for (const topo::Shape& f : planeFaces) {
    const auto surf = topo::surfaceOf(f);
    const math::Ax3 fr = worldFrame(*surf, f);
    math::Vec3 n = fr.z.vec();
    if (f.orientation() == topo::Orientation::Reversed) n = -n;
    cs->capPlanes.emplace_back(n, fr.origin);
  }
  return cs;
}

// ── Curved point-in-solid test (design.md §Curved point-in-solid) ──────────────
// The bsp.h idea generalised: a point is INSIDE the elementary solid iff it lies in
// the curved wall's radial/spherical half-space AND every planar cap half-space. The
// driving quantity is the MOST-POSITIVE signed distance to any bounding half-space
// (>0 = outside that half-space). Returns +1 inside, -1 outside, 0 ON (within tol of a
// boundary). An ON verdict aborts the native path (coincident / tangent → OCCT).
inline int classifyPoint(const CurvedSolid& cs, const math::Point3& p, double tol) {
  const math::Vec3 w = p - cs.frame.origin;
  double wall;  // signed distance to the curved wall (>0 = outside the material)
  switch (cs.kind) {
    case CurvedKind::Cylinder: {
      const double axial = math::dot(w, cs.frame.z.vec());
      wall = math::norm(w - cs.frame.z.vec() * axial) - cs.radius;
      break;
    }
    case CurvedKind::Sphere:
      wall = math::norm(w) - cs.radius;
      break;
    case CurvedKind::Cone: {
      const double axial = math::dot(w, cs.frame.z.vec());
      const double rAt = cs.radius + axial * std::tan(cs.semiAngle);
      wall = (math::norm(w - cs.frame.z.vec() * axial) - rAt) * std::cos(cs.semiAngle);
      break;
    }
  }
  double worst = wall;
  for (const auto& [n, o] : cs.capPlanes)
    worst = std::max(worst, math::dot(p - o, n));
  if (std::fabs(worst) <= tol) return 0;
  return worst < 0.0 ? 1 : -1;
}

// ── SPLIT band (design.md §1) ──────────────────────────────────────────────────
// A seam WLine, projected onto ONE operand's wall (its (u,v) track), splits the wall
// into a v < seam piece and a v > seam piece — the curved analogue of bsp.h routing a
// polygon to the FRONT/BACK of a split. We return the seam's v-band on this wall plus
// an interior sample strictly on each side (never on the seam), for the classifier.
// The seam must sit STRICTLY inside the wall v-extent (a transversal cut); a seam
// pinned to a wall edge is boundary-coincident → not transversal → decline.
struct WallSplit {
  double seamV = 0.0;      ///< representative seam v (median band centre)
  double belowSample = 0.0;  ///< interior v strictly below the seam
  double aboveSample = 0.0;  ///< interior v strictly above the seam
  double uSample = 0.0;      ///< a u the seam covers (for the sample point)
  bool ok = false;
};
inline WallSplit seamBand(const CurvedSolid& wall, const ssi::WLine& seam, bool paramIsA) {
  WallSplit r;
  if (seam.points.size() < 3) return r;
  double vLo = std::numeric_limits<double>::infinity(), vHi = -vLo, uSum = 0.0;
  for (const auto& n : seam.points) {
    const double v = paramIsA ? n.v1 : n.v2;
    const double u = paramIsA ? n.u1 : n.u2;
    vLo = std::min(vLo, v); vHi = std::max(vHi, v);
    uSum += u;
  }
  const double mid = 0.5 * (vLo + vHi);
  if (!(mid - wall.vLo > 1e-6) || !(wall.vHi - mid > 1e-6)) return r;  // seam at a wall edge
  r.seamV = mid;
  r.belowSample = 0.5 * (wall.vLo + vLo);
  r.aboveSample = 0.5 * (vHi + wall.vHi);
  r.uSample = uSum / static_cast<double>(seam.points.size());
  r.ok = true;
  return r;
}

}  // namespace ssidetail

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_SSI_BOOLEAN_H
