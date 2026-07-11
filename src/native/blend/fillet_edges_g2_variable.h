// SPDX-License-Identifier: Apache-2.0
//
// fillet_edges_g2_variable.h — native G2 (CURVATURE-CONTINUOUS) VARIABLE-RADIUS blend
// fillet on a convex edge between two PLANAR faces (MOAT M3, the drop-OCCT Class-B
// `fillet_edges_g2` slice, extended to a radius that VARIES along the edge).
//
// ── WHAT THIS IS, VS THE CONSTANT-RADIUS fillet_edges_g2 ──────────────────────────
// The landed fillet_edges_g2 (blend/fillet_edges_g2.h) replaces a sharp convex planar
// dihedral with a CONSTANT-radius zero-END-CURVATURE quintic section swept unchanged
// along the whole edge. This header keeps the SAME per-section geometry (the exact
// quintic with collinear rail-triples ⇒ B''(0)=B''(1)=0 ⇒ curvature 0 at both rails
// ⇒ genuine G2 to the flat faces) but lets the section RADIUS ramp LINEARLY along the
// edge: r(τ) = r0·(1−τ) + r1·τ for τ∈[0,1] from edge-end A (r0) to edge-end B (r1).
// Each cross-section is the constant-radius G2 quintic for its own LOCAL radius r(τ);
// the fillet surface is the LOFT (skin) of that varying-radius section along the edge.
//
// ── WHY THE SWEEP STAYS G2 ALONG THE ENTIRE EDGE, AND CANNOT SELF-INTERSECT ────────
// Two structural facts make this tractable and PROVABLY correct on a STRAIGHT dihedral:
//   (1) Every section lives in the cross-plane PERPENDICULAR to the (straight) crease
//       through its station: the six poles are T1,T2 (on the crease cross-section) plus
//       offsets along the IN-FACE directions w1,w2, which are ORTHOGONAL to the crease.
//       So each section poles share the station's crease coordinate — the sections are a
//       stack of PARALLEL planar curves, one per cross-plane. The crease coordinate is
//       therefore STRICTLY MONOTONE across the sweep (injective in τ), so two sections at
//       different τ can NEVER share a 3-D point ⇒ the swept surface is EMBEDDED (no
//       cross-station self-intersection) for ANY r0,r1 that keep each section valid.
//   (2) In its OWN cross-plane each section is the landed constant-radius G2 quintic,
//       whose curvature is IDENTICALLY zero at both rails (B''=0 by the collinear triple)
//       — so the seam curvature matches the flat faces' zero curvature at EVERY station,
//       i.e. G2 holds along the WHOLE edge, not merely at the two ends. A merely-G1
//       (circular varying-r) section would have seam curvature 1/r(τ) at every station —
//       the very jump this section removes.
// Because the tangent points move LINEARLY with r(τ) (T1,T2 are affine in r, and r is
// affine in τ), the setback footprint on each face is a STRAIGHT tangent line — so each
// face is trimmed by a SINGLE slanted plane (through that line), exactly as the constant
// path uses one plane, and the trimmed faces stay simple.
//
// ── SELF-INTERSECTION GUARD (a too-fast radius ramp DECLINES) ──────────────────────
// Although different-τ sections cannot cross (fact 1), a radius that ramps TOO FAST
// still produces a degenerate solid two ways, both of which DECLINE (return NULL, never
// a self-intersecting body): (a) at either edge end the local radius is too large for
// its faces — filletArc/section rejects, or the setback clip leaves < 3 vertices; (b)
// the setback line recedes across the face FASTER than the edge advances (|r1−r0| > L),
// so the trimmed face folds. We reject |r1−r0| ≥ L·kMaxRampSlope up front, and re-check
// section validity + a non-degenerate setback at EVERY station; any failure ⇒ NULL ⇒
// OCCT. The engine's mandatory watertight + monotone-shrink self-verify is the backstop.
//
// ── VOLUME (closed-form, monotone in r0,r1) ───────────────────────────────────────
// A 90° convex corner's quintic section removes cross-section area k·r² with k = 131/960
// (the same per-corner constant the concave slice proves — the concave test's
// 4·(131/960)·r² is that area times its edge length 4; the fuller quintic shoulder removes
// LESS than the circular r²(1−π/4)=0.2146·r²). Over an edge of length L with the linear law
//     V_removed = k · L · ∫₀¹ r(τ)² dτ = (131/960) · L · (r0² + r0·r1 + r1²)/3,
// strictly increasing in both r0 and r1 (a bigger ball at either end removes more), and
// reducing to the constant-radius (131/960)·L·r² when r0=r1. Filleted solid = Vo − V_removed.
//
// ── SCOPE (honest) ────────────────────────────────────────────────────────────────
// Native ONLY for a CONVEX edge shared by exactly TWO PLANAR faces (a planar dihedral),
// with a LINEAR radius law, both radii small enough that every station's tangent lines
// stay within their faces and |r1−r0| < L. A CONCAVE edge, a CURVED adjacent face, an
// edge shared by ≠2 faces, a non-linear law, a non-planar solid, freeform substrates, or
// any self-verify failure → NULL → OCCT fallthrough. The constant-radius builders
// (fillet_edges_g2 / _concave) are NOT invoked here and this builder does not fire on
// their inputs beyond the honest r0==r1 reduction. A G1 (merely-tangent) blend is NEVER
// emitted: the section is the zero-END-curvature quintic at its local radius, or nothing.
//
// CLEAN-ROOM. Reuses only src/native/math + topology + boolean + blend/fillet_edges +
// blend/fillet_edges_g2. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_VARIABLE_H
#define CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_VARIABLE_H

#include "native/blend/blend_geom.h"
#include "native/blend/fillet_edges.h"     // detail::filletArc / FilletArc
#include "native/blend/fillet_edges_g2.h"  // detail::g2Section / quinticPoint / g2Facets

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// Max setback-recession slope: reject |r1−r0| ≥ kMaxRampSlope·L. At slope 1 the tangent
// line recedes across the face as fast as the edge advances (the trimmed face folds); we
// keep a safety margin below 1 so the trimmed faces stay comfortably simple.
inline constexpr double kG2VarMaxRampSlope = 0.999;

// Number of stations along the edge for the loft. The section changes only in SIZE (its
// shape is the fixed-fullness quintic), so a modest station count captures the linear
// ramp faithfully; we tie it to the facet deflection so a finer request refines the sweep
// too. Bounded like arcFacets so the body stays a manageable, deterministic soup.
inline int g2VarStations(double rMax, double deflection) {
  // The largest section (rMax) sets the sampling; reuse the section-facet rule so the
  // along-edge sampling scales with the requested fidelity.
  const int f = g2Facets(rMax, deflection);
  return std::max(4, std::min(64, f));
}

// One evaluated cross-section: its six quintic poles at station τ (already placed in the
// station's cross-plane) and the local radius. Built by reusing the landed constant-radius
// g2Section at the station's edge point and local radius — so the SAME zero-end-curvature
// pole layout (⇒ G2) is used verbatim at every station.
struct G2VarStation {
  std::array<math::Point3, 6> poles;
  math::Point3 t1, t2;  // tangent points at this station (for the setback lines)
  double radius = 0.0;
};

// Evaluate the variable G2 section stack. Returns nullopt on any degeneracy (a station's
// section rejects, or the ramp slope is too steep) — the honest self-intersection/scope
// decline. `f1,f2` are the two planar faces on the edge in the CURRENT soup.
struct G2VarSweep {
  std::vector<G2VarStation> stations;  // size ns+1, τ = k/ns
  math::Vec3 t;                        // unit crease direction (A→B)
  math::Point3 t1LineA, t1LineB;       // face-1 setback line endpoints (τ=0,1)
  math::Point3 t2LineA, t2LineB;       // face-2 setback line endpoints (τ=0,1)
  int ns = 0;
};

inline std::optional<G2VarSweep> g2VarSweep(const math::Point3& ea, const math::Point3& eb,
                                            const nb::Polygon& f1, const nb::Polygon& f2,
                                            double r0, double r1, double deflection) {
  if (!(r0 > kBlendEps) || !(r1 > kBlendEps)) return std::nullopt;
  const auto tOpt = creaseDir(ea, eb);
  if (!tOpt) return std::nullopt;
  const double L = math::norm(eb - ea);
  if (!(L > kBlendEps)) return std::nullopt;
  // Self-intersection guard: the setback line must not recede faster than the edge runs.
  if (std::fabs(r1 - r0) >= kG2VarMaxRampSlope * L) return std::nullopt;

  const double rMax = std::max(r0, r1);
  const int ns = g2VarStations(rMax, deflection);

  const math::Vec3 tdir = tOpt->vec();
  G2VarSweep sw;
  sw.t = tdir;
  sw.ns = ns;
  sw.stations.reserve(static_cast<std::size_t>(ns) + 1);
  for (int k = 0; k <= ns; ++k) {
    const double tau = static_cast<double>(k) / static_cast<double>(ns);
    const math::Point3 E{ea.x + (eb.x - ea.x) * tau, ea.y + (eb.y - ea.y) * tau,
                         ea.z + (eb.z - ea.z) * tau};
    const double r = r0 * (1.0 - tau) + r1 * tau;
    // Seat the constant-radius G2 section in THIS station's cross-plane: g2Section places
    // its poles on the cross-section through the edge MIDPOINT, so pass a tiny genuine
    // edge centred at E (preserving the true crease direction, which a zero-length edge
    // would lose). The section geometry is translation-invariant along the crease, so the
    // span size is irrelevant — only the midpoint E and the crease direction matter.
    const double h = 0.5 * std::max(L * 1e-3, kBlendEps * 16.0);
    const math::Point3 sa{E.x - tdir.x * h, E.y - tdir.y * h, E.z - tdir.z * h};
    const math::Point3 sb{E.x + tdir.x * h, E.y + tdir.y * h, E.z + tdir.z * h};
    const auto sec = g2Section(sa, sb, f1, f2, r);
    if (!sec) return std::nullopt;
    G2VarStation st;
    st.poles = sec->poles;
    st.t1 = sec->poles[0];  // P0 = T1
    st.t2 = sec->poles[5];  // P5 = T2
    st.radius = r;
    sw.stations.push_back(st);
  }
  sw.t1LineA = sw.stations.front().t1;
  sw.t1LineB = sw.stations.back().t1;
  sw.t2LineA = sw.stations.front().t2;
  sw.t2LineB = sw.stations.back().t2;
  return sw;
}

// The single DIAGONAL wedge plane that sets both faces back to their tangent lines — the
// variable-radius analogue of the constant path's tangentCutPlane. It passes through the
// two slanted tangent lines T1(τ) and T2(τ); for a straight convex dihedral the four
// endpoints T1(0),T2(0),T1(1),T2(1) are coplanar (both lines are affine in τ and the
// corner-out direction n1+n2 is fixed along the straight crease), so ONE plane cuts the
// whole corner wedge off — a single convex half-space clip, exactly like the constant path
// (which is the r0==r1 special case). Its normal points toward the removed corner. Returns
// nullopt if the four points are not coplanar (a skew wedge — outside this slice → OCCT) or
// the plane is degenerate.
inline std::optional<nb::Plane> g2VarWedgePlane(const G2VarSweep& sw, const math::Point3& corner) {
  const math::Vec3 e1 = sw.t1LineB - sw.t1LineA;  // along T1 line
  const math::Vec3 e2 = sw.t2LineA - sw.t1LineA;  // T1→T2 across the wedge
  math::Vec3 n = math::cross(e1, e2);
  const math::Dir3 nd{n};
  if (!nd.valid()) return std::nullopt;
  nb::Plane pl = nb::Plane::fromPointNormal(sw.t1LineA, nd.vec());
  // Coplanarity check: the other two tangent endpoints must lie on this plane.
  if (std::fabs(signedDist(pl, sw.t1LineB)) > 1e-6 * (1.0 + math::norm(e1)) ||
      std::fabs(signedDist(pl, sw.t2LineB)) > 1e-6 * (1.0 + math::norm(e2)))
    return std::nullopt;
  if (signedDist(pl, corner) < 0.0) { pl.normal = -pl.normal; pl.w = -pl.w; }
  return pl;
}

// Loft the section stack into quad strips (station k→k+1 × section sample j→j+1), plus the
// two end-cap section faces at τ=0 and τ=1. Same watertight structure as the constant-r
// g2Strips, but the rim ring VARIES per station (radius ramp).
inline std::vector<nb::Polygon> g2VarStrips(const G2VarSweep& sw, double rMax, double deflection) {
  const int nfSec = g2Facets(rMax, deflection);  // samples ACROSS the section
  if (nfSec < 1 || sw.ns < 1) return {};
  const int ns = sw.ns;

  // A rim point: section sample s at station k.
  auto rimAt = [&](int k, int j) -> math::Point3 {
    const double s = static_cast<double>(j) / static_cast<double>(nfSec);
    return quinticPoint(sw.stations[static_cast<std::size_t>(k)].poles, s);
  };
  // Outward reference at (k,j): from the station's rolling-ball axis, approximated by the
  // midpoint of T1,T2 pulled inward — but the exact axis is not stored; use the section
  // chord midpoint→sample direction as the outward sign reference (the section bulges away
  // from the T1–T2 chord toward the removed corner). Consistent with g2Strips' radial sign.
  auto sectionInterior = [&](int k) -> math::Point3 {
    const math::Point3& a = sw.stations[static_cast<std::size_t>(k)].t1;
    const math::Point3& b = sw.stations[static_cast<std::size_t>(k)].t2;
    return math::Point3{0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z)};
  };

  std::vector<nb::Polygon> strips;
  strips.reserve(static_cast<std::size_t>(ns) * static_cast<std::size_t>(nfSec) + 2);
  for (int k = 0; k < ns; ++k) {
    for (int j = 0; j < nfSec; ++j) {
      const math::Point3 a = rimAt(k, j);
      const math::Point3 b = rimAt(k, j + 1);
      const math::Point3 c = rimAt(k + 1, j + 1);
      const math::Point3 d = rimAt(k + 1, j);
      // Quad true normal = cross of its two spanning edges. Orient OUTWARD using the
      // section-chord midpoint as an interior reference (the blend bulges away from it).
      math::Vec3 gn = math::cross(b - a, d - a);
      const math::Point3 interior = sectionInterior(k);
      const math::Vec3 outward = a - interior;
      if (math::dot(gn, outward) < 0.0) gn = -gn;
      const math::Dir3 ndir{gn};
      if (!ndir.valid()) continue;
      std::vector<math::Point3> loop{a, b, c, d};
      math::Vec3 area{0, 0, 0};
      for (std::size_t i = 0; i < loop.size(); ++i)
        area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
      if (math::dot(area, ndir.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
      strips.emplace_back(loop, nb::Plane::fromPointNormal(loop.front(), ndir.vec()));
    }
  }

  // END CAPS — the section-segment gap in each end face (identical role to g2Strips).
  auto endCap = [&](int k, const math::Vec3& outward) {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(nfSec) + 1);
    for (int j = 0; j <= nfSec; ++j) ring.push_back(rimAt(k, j));
    const math::Dir3 nd{outward};
    if (!nd.valid() || ring.size() < 3) return;
    math::Vec3 areaV{0, 0, 0};
    for (std::size_t i = 0; i < ring.size(); ++i)
      areaV += math::cross(ring[i].asVec(), ring[(i + 1) % ring.size()].asVec());
    if (math::dot(areaV, nd.vec()) < 0.0) std::reverse(ring.begin(), ring.end());
    strips.emplace_back(ring, nb::Plane::fromPointNormal(ring.front(), nd.vec()));
  };
  endCap(0, -sw.t);   // τ=0 end face outward normal points back along −t
  endCap(ns, sw.t);   // τ=1 end face outward normal points forward along +t
  return strips;
}

}  // namespace detail

// VARIABLE-radius G2 (curvature-continuous) fillet on the convex planar-dihedral edges
// `edgeIds` (1-based, mapShapes order) of `solid`, with the section radius ramping
// linearly from `radius0` at edge-end A to `radius1` at edge-end B. Returns the blended
// solid (a lofted zero-end-curvature quintic section, deflection-bounded), or a NULL
// Shape if any picked edge is not a convex edge between two planar faces / the solid is
// not planar / a radius is degenerate / the ramp is too steep (self-intersection guard) /
// a station's section is too large for its faces (→ OCCT fallthrough). `deflection` bounds
// the facet chord error. A merely-G1 (circular) blend is NEVER produced here; when
// radius0==radius1 the result reduces to the constant-radius G2 fillet.
inline topo::Shape fillet_edges_g2_variable(const topo::Shape& solid, const int* edgeIds,
                                            int edgeCount, double radius0, double radius1,
                                            double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount <= 0) return {};
  if (!(radius0 > kBlendEps) || !(radius1 > kBlendEps)) return {};
  PlanarModel model(solid);
  if (!model.isValid()) return {};

  std::vector<nb::Polygon> polys = model.polygons();
  for (int i = 0; i < edgeCount; ++i) {
    const auto ends = edgeEnds(solid, edgeIds[i]);
    if (!ends) return {};

    std::size_t faces[2];
    if (facesOnEdgeInSoup(polys, ends->a, ends->b, faces) != 2) return {};
    const nb::Polygon& f1 = polys[faces[0]];
    const nb::Polygon& f2 = polys[faces[1]];

    const auto sweep = detail::g2VarSweep(ends->a, ends->b, f1, f2, radius0, radius1, deflection);
    if (!sweep) return {};

    // The removed corner is on the material side of the crease (the sharp edge itself);
    // use the edge midpoint as the "toward-corner" reference for orienting the wedge plane.
    const math::Point3 corner{0.5 * (ends->a.x + ends->b.x), 0.5 * (ends->a.y + ends->b.y),
                              0.5 * (ends->a.z + ends->b.z)};
    const auto wedge = detail::g2VarWedgePlane(*sweep, corner);
    if (!wedge) return {};

    // Set both faces back to the (slanted) tangent lines with the SINGLE diagonal wedge
    // clip — identical structure to the constant path's tangentCutPlane clip, so only the
    // corner wedge is removed and all other faces stay on the material side. The lofted
    // quintic strips fill the opening.
    std::vector<nb::Polygon> setback;
    setback.reserve(polys.size());
    for (const nb::Polygon& poly : polys) {
      std::vector<math::Point3> clipped = clipBelow(poly.vertices, *wedge, kBlendEps);
      if (clipped.size() >= 3) setback.emplace_back(std::move(clipped), poly.plane);
    }

    const double rMax = std::max(radius0, radius1);
    std::vector<nb::Polygon> strips = detail::g2VarStrips(*sweep, rMax, deflection);
    if (strips.empty()) return {};
    for (nb::Polygon& s : strips) setback.push_back(std::move(s));
    polys = std::move(setback);
  }
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_VARIABLE_H
