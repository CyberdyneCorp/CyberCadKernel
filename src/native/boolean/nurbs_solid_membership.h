// SPDX-License-Identifier: Apache-2.0
//
// nurbs_solid_membership.h — NURBS roadmap LAYER 3, STAGE 4: general
// point-in-NURBS-SOLID membership across MULTIPLE trimmed NURBS faces (the
// Stage-4 region classifier the general two-solid boolean needs).
//
// ── WHAT THIS IS (and is NOT) ─────────────────────────────────────────────────
// `ssi_boolean.h` `classifyPoint(const CurvedSolid&, …)` answers point-in-solid
// ONLY for a solid whose curved wall folds to ONE analytic ELEMENTARY surface
// (cylinder / sphere / cone half-space + planar caps): the moment an operand
// carries a `Kind::BSpline`/`Kind::Bezier` face, `recogniseCurvedSolid` returns
// `nullopt` and there is NO membership test — the Stage-4 gap the readiness doc
// (L3-EXACT-NURBS-BOOLEAN-READINESS §2 Stage 4) measured as MISSING.
//
// `freeform_membership.h` (`classifyPointInMesh`) is the sibling MESH-driven
// classifier — but it needs a WATERTIGHT TRIANGLE MESH the M0 tessellator built,
// so its verdict is only as faithful as the mesh deflection (a wrong-side sliver
// within ~2×deflection resolves to `On`, never a crisp In/Out).
//
// This header is the EXACT-GEOMETRY path: `solid` is a set of TRIMMED NURBS faces
// (each a `topology::TrimmedNurbsFace` = surface S(u,v) + trim loops) forming a
// closed shell, and membership is answered by a ray-cast that intersects the ray
// with each face's TRUE surface via the H1 exact intersector
// `math::intersectCurveSurface`, then tests each hit's (u,v) against the face's
// trim loops via `topology::classify`. No mesh, no faceting — the crossing count
// is exact on the true NURBS boundary.
//
// ── THE ALGORITHM (readiness doc §2 Stage-4 / §4 tail-gap G4-general) ──────────
//   1. RAY-CAST. Shoot a ray from `point` in a generic direction, modelled as a
//      degree-1 NURBS segment (a line = the degenerate `cc`/`cs` curve) long
//      enough to clear the solid. For each trimmed face:
//        a. intersect the ray-segment ∩ the face surface (H1 intersectCurveSurface),
//        b. for each FORWARD hit (paramT ∈ (eps, 1)) test its (u,v) against the
//           face's trim loops (`classify`): a `Containment::In` hit is a REAL
//           transversal crossing of the solid boundary.
//      Odd total crossing count ⇒ the point is INSIDE.
//   2. ROBUSTNESS. A hit is AMBIGUOUS when it lands ON a trim edge
//      (`Containment::OnBoundary` / `Unknown`), when the curve↔surface pair is
//      TANGENTIAL, or when the surface intersector honest-declines (Coincident).
//      An ambiguous ray is DISCARDED and RE-CAST in a different generic direction
//      rather than miscounted. If K directions all stay ambiguous the verdict is
//      an honest `Membership::Unknown` — never a guessed In/Out.
//   3. FRAGMENT CLASSIFIER. `classifyFragmentVsSolid` samples an interior
//      representative of a face fragment (respecting its holes) and VOTES the
//      fragment's membership against the OTHER solid — the batch Stage-4 region
//      classifier the general boolean's keep/discard select consumes.
//
// ── HONESTY CONTRACT (hard invariant) ─────────────────────────────────────────
// NEVER widen a tolerance to manufacture a crisp verdict; NEVER count an ambiguous
// (on-edge / tangent / declined) hit as a crossing. A point the ray-cast cannot
// resolve after K directions is `Unknown` (honest decline) — the same discipline
// as `classifyPoint` / `classifyPointInMesh` / `intersectCurveSurface`. This
// mirrors the kernel-wide honest-decline seam.
//
// ── SUBSTRATE ─────────────────────────────────────────────────────────────────
// OCCT-FREE. Depends only on `src/native/math` (vec.h + the numsci-gated
// bspline_intersect.h) and `src/native/topology` (trimmed_nurbs.h). Header-only,
// `clang++ -std=c++20`. NO `cc_*` ABI change. The surface intersector is
// numsci-gated (it lives in the numsci-gated NURBS layer), so the ray-cast body
// is compiled only under CYBERCAD_HAS_NUMSCI; the declarations stay visible so a
// build without the substrate still compiles (the entry points return `Unknown`,
// an honest decline, never a fabricated verdict).
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_MEMBERSHIP_H
#define CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_MEMBERSHIP_H

#include "native/math/native_math.h"
#include "native/topology/trimmed_nurbs.h"

#include <cstddef>
#include <vector>

#if defined(CYBERCAD_HAS_NUMSCI)
#include "native/math/bspline_intersect.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#endif

namespace cybercad::native::boolean {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

// ─────────────────────────────────────────────────────────────────────────────
// A NURBS solid = the set of TRIMMED NURBS faces forming a closed shell. Each
// face is a topology::TrimmedNurbsFace (surface + location + outer/hole loops); a
// well-formed solid's faces together bound a watertight region. The membership
// test does not re-derive topology — it consumes the trimmed faces directly.
// ─────────────────────────────────────────────────────────────────────────────
using NurbsSolid = std::vector<topo::TrimmedNurbsFace>;

// ─────────────────────────────────────────────────────────────────────────────
// The four possible verdicts, mirroring freeform_membership::Membership. `On` =
// the point lies (within tol) ON the solid boundary; `Unknown` = an honest
// decline (the ray-cast stayed ambiguous after every direction) — never a silent
// wrong crisp verdict.
// ─────────────────────────────────────────────────────────────────────────────
enum class Membership { In, Out, On, Unknown };

// ─────────────────────────────────────────────────────────────────────────────
// Tolerances / thresholds. NONE weakens a bound: the surface-intersection tol is
// the intersector's own gap tol, the on-edge band is `classify`'s scale-relative
// OnBoundary tol, and the direction count only ever turns a risky ray into an
// honest decline.
// ─────────────────────────────────────────────────────────────────────────────
struct SolidMembershipTol {
  double surfaceTol = 1e-9;   ///< intersectCurveSurface gap tolerance
  double onEdgeTol = 1e-7;    ///< classify OnBoundary band (× loop UV extent)
  int flattenSegments = 64;   ///< classify polyline samples per pcurve segment
  double onBoundaryDist = 1e-9;  ///< |query − a real crossing point| under this ⇒ On the boundary
  double rayLengthFactor = 4.0;  ///< ray span = this × the solid's bbox diagonal (clears the solid)
  double paramEps = 1e-9;     ///< forward-hit floor on the ray parameter t ∈ (eps, 1−eps)
};

// ─────────────────────────────────────────────────────────────────────────────
// pointInNurbsSolid — the Stage-4 point-in-SOLID membership test.
//
// Returns In / Out / On / Unknown for `point` against the trimmed-NURBS `solid`,
// by an exact ray-cast over the true face surfaces (see the file header). A
// point the ray-cast cannot resolve after every generic direction is `Unknown`
// (honest decline), never a guessed verdict.
//
// The DECLARATION is always visible; the DEFINITION is compiled only under
// CYBERCAD_HAS_NUMSCI (it consumes the numsci-gated surface intersector). Without
// the substrate the entry returns `Unknown` — an honest decline, not a fabricated
// verdict — so every consumer compiles either way.
// ─────────────────────────────────────────────────────────────────────────────
Membership pointInNurbsSolid(const math::Point3& point, const NurbsSolid& solid,
                             const SolidMembershipTol& tol = {});

// ─────────────────────────────────────────────────────────────────────────────
// A face fragment = a trimmed NURBS face whose trim loops bound the region to
// classify (an interior representative is sampled from it). The batch classifier
// `classifyFragmentVsSolid` votes the fragment's membership against `otherSolid`
// — the Stage-4 region classifier the general boolean's keep/discard consumes.
// ─────────────────────────────────────────────────────────────────────────────
using FaceFragment = topo::TrimmedNurbsFace;

// ─────────────────────────────────────────────────────────────────────────────
// classifyFragmentVsSolid — is a face fragment INSIDE / OUTSIDE the other solid?
//
// Samples up to `maxSamples` INTERIOR representatives of `fragment` (points that
// classify `In` in the fragment's own trim region, i.e. respecting holes),
// evaluates each on the fragment surface, and calls pointInNurbsSolid against
// `otherSolid`. The fragment's verdict is the MAJORITY of the crisp votes:
//   * every sample In        ⇒ In        (fragment lies inside the other solid)
//   * every sample Out       ⇒ Out       (fragment lies outside)
//   * a MIX (straddling)     ⇒ the interior-rep vote is well-defined: the
//                              majority verdict, reported alongside the split (a
//                              caller that needs a clean fragment must sub-split).
//   * no crisp votes / a tie ⇒ Unknown   (honest decline)
// A fragment with no valid interior sample ⇒ Unknown.
// ─────────────────────────────────────────────────────────────────────────────
struct FragmentClassification {
  Membership verdict = Membership::Unknown;  ///< the fragment's majority membership
  int inVotes = 0;     ///< interior samples classified In the other solid
  int outVotes = 0;    ///< interior samples classified Out
  int onVotes = 0;     ///< interior samples classified On the boundary
  int unknownVotes = 0;///< interior samples the ray-cast could not resolve
  int samples = 0;     ///< interior representatives actually tested
  bool straddles = false;  ///< true ⇔ both In and Out votes present (the fragment crosses)
};

struct FragmentOptions {
  int gridU = 9;          ///< interior-sample grid density in u over the fragment domain
  int gridV = 9;          ///< interior-sample grid density in v
  int maxSamples = 64;    ///< cap on interior representatives tested
  SolidMembershipTol tol{};  ///< membership tolerances forwarded to pointInNurbsSolid
};

FragmentClassification classifyFragmentVsSolid(const FaceFragment& fragment,
                                               const NurbsSolid& otherSolid,
                                               const FragmentOptions& opts = {});

#if defined(CYBERCAD_HAS_NUMSCI)

namespace nsmdetail {

// A FIXED set of generic, mutually non-parallel ray directions (no two are scalar
// multiples; none lies in a coordinate plane; none axis-aligned). Deterministic —
// the classifier is a pure function of its inputs (no RNG). The redundancy exists
// so an ambiguous (on-edge / tangent / declined) ray is RE-CAST in a fresh
// direction rather than miscounted (the readiness §2 Stage-4 "re-cast on a
// near-boundary/tangent hit" requirement).
inline const std::array<math::Vec3, 8>& rayDirections() noexcept {
  static const std::array<math::Vec3, 8> kDirs = [] {
    const double raw[8][3] = {{1, 2, 3},   {2, -3, 1},  {-3, 1, 2}, {1, -2, 4},
                              {3, 4, -2},  {-2, 3, -4}, {4, -1, 2}, {-1, 4, -3}};
    std::array<math::Vec3, 8> d{};
    for (int i = 0; i < 8; ++i) {
      const math::Vec3 v{raw[i][0], raw[i][1], raw[i][2]};
      d[i] = v / math::norm(v);
    }
    return d;
  }();
  return kDirs;
}

// ── World-evaluate a trimmed face's surface at (u,v) ───────────────────────────
// The surface is stored local; `location` places it in world. We build a plain
// point evaluator that respects the FaceSurface kind for the two kinds this Stage-4
// slice supports as EXACT ray-cast targets: BSpline/Bezier (the freeform walls) and
// Plane (the flat caps). Analytic curved walls (Cylinder/Sphere/Cone) already have
// the closed-form classifyPoint path (ssi_boolean.h); a solid mixing them is out of
// this slice's exact-ray envelope and is declined honestly upstream.
inline math::Point3 evalFaceSurface(const topo::FaceSurface& s, double u, double v) {
  switch (s.kind) {
    case topo::FaceSurface::Kind::Plane:
      return math::Plane{s.frame}.value(u, v);
    case topo::FaceSurface::Kind::BSpline:
    case topo::FaceSurface::Kind::Bezier: {
      const math::SurfaceGrid grid{s.poles, s.nPolesU, s.nPolesV};
      if (!s.weights.empty())
        return math::nurbsSurfacePoint(s.degreeU, s.degreeV, grid, s.weights, s.knotsU, s.knotsV,
                                       u, v);
      return math::surfacePoint(s.degreeU, s.degreeV, grid, s.knotsU, s.knotsV, u, v);
    }
    default:
      return {};  // Cylinder/Sphere/Cone/Torus — not this slice's exact ray-cast target
  }
}

// Apply a Location (world placement) to a local point.
inline math::Point3 worldPoint(const topo::Location& loc, const math::Point3& p) {
  if (loc.isIdentity()) return p;
  return loc.transform().applyToPoint(p);
}

// ── A SurfaceView over a trimmed face for the H1 intersector ───────────────────
// BSpline/Bezier faces map their pole grid directly; a Plane face is expanded into
// a degree-(1,1) bilinear 2×2 patch over the (u,v) domain the trim loop spans (a
// plane IS an exact bilinear NURBS). The view's poles/knots are owned by the
// returned holder so the spans stay valid for the intersect call.
struct SurfaceViewHolder {
  std::vector<math::Point3> poles;
  std::vector<double> weights;
  std::vector<double> knotsU, knotsV;
  int degreeU = 1, degreeV = 1, nRows = 0, nCols = 0;
  bool ok = false;
  math::SurfaceView view() const {
    return math::SurfaceView{degreeU, degreeV, poles, weights, nRows, nCols, knotsU, knotsV};
  }
};

// Build a SurfaceView holder for a trimmed face. `uLo..vHi` bound the plane domain
// (ignored for BSpline/Bezier — their knots carry the domain). Points are placed
// in WORLD via the face location so the ray-cast is in world coordinates.
inline SurfaceViewHolder makeFaceView(const topo::TrimmedNurbsFace& f, double uLo, double uHi,
                                      double vLo, double vHi) {
  SurfaceViewHolder h;
  const topo::FaceSurface& s = f.surface;
  switch (s.kind) {
    case topo::FaceSurface::Kind::BSpline:
    case topo::FaceSurface::Kind::Bezier: {
      if (s.nPolesU < 2 || s.nPolesV < 2) return h;
      h.degreeU = s.degreeU;
      h.degreeV = s.degreeV;
      h.nRows = s.nPolesU;
      h.nCols = s.nPolesV;
      h.poles.reserve(s.poles.size());
      for (const math::Point3& p : s.poles) h.poles.push_back(worldPoint(f.location, p));
      h.weights = s.weights;
      // A Bezier stored without knots gets a clamped Bézier knot vector per axis.
      h.knotsU = s.knotsU.empty()
                     ? std::vector<double>{}
                     : s.knotsU;
      h.knotsV = s.knotsV.empty() ? std::vector<double>{} : s.knotsV;
      if (h.knotsU.empty()) {
        h.knotsU.assign(static_cast<std::size_t>(s.nPolesU + s.degreeU + 1), 0.0);
        for (std::size_t i = 0; i < h.knotsU.size(); ++i)
          h.knotsU[i] = i < static_cast<std::size_t>(s.degreeU + 1) ? 0.0 : 1.0;
      }
      if (h.knotsV.empty()) {
        h.knotsV.assign(static_cast<std::size_t>(s.nPolesV + s.degreeV + 1), 0.0);
        for (std::size_t i = 0; i < h.knotsV.size(); ++i)
          h.knotsV[i] = i < static_cast<std::size_t>(s.degreeV + 1) ? 0.0 : 1.0;
      }
      h.ok = true;
      return h;
    }
    case topo::FaceSurface::Kind::Plane: {
      // Bilinear 2×2 patch over [uLo,uHi]×[vLo,vHi] on the plane. Corner (u,v).
      const math::Plane pl{s.frame};
      auto corner = [&](double u, double v) {
        return worldPoint(f.location, pl.value(u, v));
      };
      h.degreeU = 1;
      h.degreeV = 1;
      h.nRows = 2;
      h.nCols = 2;
      // Row-major (U outer, V inner): (uLo,vLo),(uLo,vHi),(uHi,vLo),(uHi,vHi).
      h.poles = {corner(uLo, vLo), corner(uLo, vHi), corner(uHi, vLo), corner(uHi, vHi)};
      h.knotsU = {uLo, uLo, uHi, uHi};
      h.knotsV = {vLo, vLo, vHi, vHi};
      h.ok = true;
      return h;
    }
    default:
      return h;  // curved analytic — not this slice's exact ray-cast target
  }
}

// The (u,v) trim-domain extent of a face's outer loop (for the plane patch span and
// for interior sampling). Flattens the outer loop and takes its UV bounding box.
struct UvExtent {
  double uLo = 0.0, uHi = 1.0, vLo = 0.0, vHi = 1.0;
  bool ok = false;
};
inline UvExtent outerUvExtent(const topo::TrimmedNurbsFace& f) {
  UvExtent e;
  if (!f.hasOuter()) return e;
  const std::vector<topo::ParamPoint> poly = topo::flattenTrimLoop(f.outer, 48);
  if (poly.size() < 3) return e;
  e.uLo = e.uHi = poly.front().u;
  e.vLo = e.vHi = poly.front().v;
  for (const topo::ParamPoint& p : poly) {
    e.uLo = std::min(e.uLo, p.u); e.uHi = std::max(e.uHi, p.u);
    e.vLo = std::min(e.vLo, p.v); e.vHi = std::max(e.vHi, p.v);
  }
  // A hair of padding so the bilinear plane patch strictly contains the trim rim
  // (a hit on the domain boundary is inside the intersector's [u0,u1] closed box).
  const double du = std::max(e.uHi - e.uLo, 1e-9) * 0.05;
  const double dv = std::max(e.vHi - e.vLo, 1e-9) * 0.05;
  e.uLo -= du; e.uHi += du; e.vLo -= dv; e.vHi += dv;
  e.ok = true;
  return e;
}

// The world-space AABB diagonal of the solid (from every face's outer-loop UV grid,
// world-evaluated) — sizes the ray span. Robust to any face kind we can evaluate.
inline double solidDiagonal(const NurbsSolid& solid) {
  math::Point3 lo{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity()};
  math::Point3 hi{-lo.x, -lo.y, -lo.z};
  bool any = false;
  for (const topo::TrimmedNurbsFace& f : solid) {
    const UvExtent e = outerUvExtent(f);
    if (!e.ok) continue;
    for (int i = 0; i <= 3; ++i)
      for (int j = 0; j <= 3; ++j) {
        const double u = e.uLo + (e.uHi - e.uLo) * i / 3.0;
        const double v = e.vLo + (e.vHi - e.vLo) * j / 3.0;
        const math::Point3 p = worldPoint(f.location, evalFaceSurface(f.surface, u, v));
        lo.x = std::min(lo.x, p.x); hi.x = std::max(hi.x, p.x);
        lo.y = std::min(lo.y, p.y); hi.y = std::max(hi.y, p.y);
        lo.z = std::min(lo.z, p.z); hi.z = std::max(hi.z, p.z);
        any = true;
      }
  }
  if (!any) return 1.0;
  return std::max(math::norm(hi - lo), 1.0);
}

// The result of casting ONE ray: whether it was usable (no ambiguous hit) and, if
// usable, whether the odd/even real-crossing parity puts the point inside, plus the
// nearest real-crossing distance (for the On-boundary band).
struct RayVote {
  bool usable = false;
  bool inside = false;
  double nearestCrossing = std::numeric_limits<double>::infinity();
};

// Cast one ray (as a degree-1 segment from `point` along `dir`, length `span`) and
// count REAL boundary crossings. A crossing is real iff the surface hit's (u,v)
// classifies `In` the face's trim region. The ray is UNUSABLE (parity discarded, →
// re-cast) if any candidate hit is ambiguous: a TANGENTIAL surface hit, a
// Coincident (curve-on-surface) decline, or a hit whose (u,v) is OnBoundary/Unknown
// in the trim region (on a trim edge). An unusable ray is never miscounted.
inline RayVote castRay(const math::Point3& point, const math::Vec3& dir, double span,
                       const NurbsSolid& solid, const SolidMembershipTol& tol) {
  RayVote vote;
  const math::Point3 pEnd{point.x + dir.x * span, point.y + dir.y * span, point.z + dir.z * span};
  const std::array<math::Point3, 2> segPoles{point, pEnd};
  const std::array<double, 4> segKnots{0.0, 0.0, 1.0, 1.0};
  const math::CurveView ray{1, segPoles, {}, segKnots};

  topo::ClassifyOptions copts;
  copts.flattenSegments = tol.flattenSegments;
  copts.onEdgeTol = tol.onEdgeTol;

  int crossings = 0;
  for (const topo::TrimmedNurbsFace& f : solid) {
    const UvExtent e = outerUvExtent(f);
    if (!e.ok) return vote;  // a face we cannot bound ⇒ ambiguous ray
    const SurfaceViewHolder h = makeFaceView(f, e.uLo, e.uHi, e.vLo, e.vHi);
    if (!h.ok) return vote;  // a face kind outside the exact ray-cast envelope
    const math::CurveSurfaceResult r =
        math::intersectCurveSurface(ray, h.view(), tol.surfaceTol);
    if (r.status == math::IntersectStatus::Coincident) return vote;  // ray on face ⇒ ambiguous

    for (const math::CurveSurfaceHit& hit : r.hits) {
      if (hit.paramT <= tol.paramEps || hit.paramT >= 1.0 - tol.paramEps) continue;  // not forward
      if (hit.type == math::IntersectionType::Tangential) return vote;  // graze ⇒ ambiguous ray
      const topo::Containment c = topo::classify(f, {hit.paramU, hit.paramV}, copts);
      if (c == topo::Containment::OnBoundary || c == topo::Containment::Unknown)
        return vote;  // hit on a trim edge ⇒ ambiguous ray, re-cast
      if (c != topo::Containment::In) continue;  // Out of the trim region ⇒ not a real crossing
      // A real crossing. Track its distance for the On-boundary band.
      const double d = hit.paramT * span;
      vote.nearestCrossing = std::min(vote.nearestCrossing, d);
      ++crossings;
    }
  }
  vote.usable = true;
  vote.inside = (crossings & 1) != 0;
  return vote;
}

}  // namespace nsmdetail

inline Membership pointInNurbsSolid(const math::Point3& point, const NurbsSolid& solid,
                                    const SolidMembershipTol& tol) {
  if (solid.empty()) return Membership::Unknown;
  const double span = tol.rayLengthFactor * nsmdetail::solidDiagonal(solid);

  // Cast each generic direction until one is usable, re-casting on ambiguity.
  double nearestCrossing = std::numeric_limits<double>::infinity();
  for (const math::Vec3& dir : nsmdetail::rayDirections()) {
    const nsmdetail::RayVote v = nsmdetail::castRay(point, dir, span, solid, tol);
    if (!v.usable) continue;  // ambiguous ray discarded → try the next direction
    // A usable ray resolves membership. But first: if the very first real crossing
    // sits essentially AT the query point, the point is ON the boundary.
    if (v.nearestCrossing <= tol.onBoundaryDist) return Membership::On;
    return v.inside ? Membership::In : Membership::Out;
  }
  (void)nearestCrossing;
  return Membership::Unknown;  // every generic direction stayed ambiguous → honest decline
}

inline FragmentClassification classifyFragmentVsSolid(const FaceFragment& fragment,
                                                      const NurbsSolid& otherSolid,
                                                      const FragmentOptions& opts) {
  FragmentClassification out;
  const nsmdetail::UvExtent e = nsmdetail::outerUvExtent(fragment);
  if (!e.ok) return out;  // no trim region ⇒ Unknown

  topo::ClassifyOptions copts;
  copts.flattenSegments = opts.tol.flattenSegments;
  copts.onEdgeTol = opts.tol.onEdgeTol;

  // Collect interior representatives: grid samples that classify In the fragment's
  // OWN trim region (respecting holes), evaluated on the fragment surface.
  for (int i = 1; i < opts.gridU && out.samples < opts.maxSamples; ++i) {
    for (int j = 1; j < opts.gridV && out.samples < opts.maxSamples; ++j) {
      const double u = e.uLo + (e.uHi - e.uLo) * i / static_cast<double>(opts.gridU);
      const double v = e.vLo + (e.vHi - e.vLo) * j / static_cast<double>(opts.gridV);
      if (topo::classify(fragment, {u, v}, copts) != topo::Containment::In) continue;
      const math::Point3 world =
          nsmdetail::worldPoint(fragment.location, nsmdetail::evalFaceSurface(fragment.surface, u, v));
      ++out.samples;
      switch (pointInNurbsSolid(world, otherSolid, opts.tol)) {
        case Membership::In:      ++out.inVotes; break;
        case Membership::Out:     ++out.outVotes; break;
        case Membership::On:      ++out.onVotes; break;
        case Membership::Unknown: ++out.unknownVotes; break;
      }
    }
  }

  out.straddles = out.inVotes > 0 && out.outVotes > 0;
  const int crisp = out.inVotes + out.outVotes;
  if (crisp == 0) {
    out.verdict = Membership::Unknown;  // no crisp interior vote → honest decline
    return out;
  }
  if (out.inVotes > out.outVotes) out.verdict = Membership::In;
  else if (out.outVotes > out.inVotes) out.verdict = Membership::Out;
  else out.verdict = Membership::Unknown;  // an exact tie is ambiguous → decline
  return out;
}

#else  // !CYBERCAD_HAS_NUMSCI — honest declines (never a fabricated verdict).

inline Membership pointInNurbsSolid(const math::Point3&, const NurbsSolid&,
                                    const SolidMembershipTol&) {
  return Membership::Unknown;
}
inline FragmentClassification classifyFragmentVsSolid(const FaceFragment&, const NurbsSolid&,
                                                      const FragmentOptions&) {
  return FragmentClassification{};
}

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_MEMBERSHIP_H
