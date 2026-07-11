// SPDX-License-Identifier: Apache-2.0
//
// fillet_edges_g2.h — native G2 (CURVATURE-CONTINUOUS) blend fillet on a convex edge
// between two PLANAR faces (MOAT M3, the drop-OCCT Class-B `fillet_edges_g2` slice).
//
// ── WHAT THIS IS, VS THE G1 fillet_edges ──────────────────────────────────────────
// The stock native fillet_edges (blend/fillet_edges.h) rolls a ball of radius r into a
// convex planar dihedral and replaces the sharp edge with a CIRCULAR-ARC cross section:
// constant curvature 1/r, so the blend is only G1 (tangent-continuous) at the two
// seams — its curvature JUMPS from 1/r on the blend to 0 on the flat neighbours. A G2
// blend instead uses a cross-section curve whose curvature is ZERO at both rails, so
// curvature is continuous across the seam (matching the flat faces' zero normal
// curvature). Everything else — where the rails land, the setback clip, the deflection-
// bounded facet tiling, and the watertight assembleSolid weld — is IDENTICAL to the G1
// path; ONLY the section curve changes (circular arc → G2 quintic).
//
// ── THE G2 SECTION CURVE (closed-form curvature-continuous, PROVEN) ────────────────
// In the crease cross-section plane the G1 fillet seats the rolling-ball cylinder axis
// at C and touches the two faces at the tangent points T1 = C + r·n1, T2 = C + r·n2
// (blend/fillet_edges.h::filletArc). The blend must (a) pass through T1 and T2 (G0),
// (b) be tangent to face i at Ti (G1 — its section tangent lies IN face i there), and
// (c) have zero curvature at T1 and T2 (G2 — the flat faces have zero normal
// curvature). We take a QUINTIC Bézier B(s), s∈[0,1], poles P0..P5, with
//     P0 = T1,                     P5 = T2,
//     P1 = T1 + w1·a,  P2 = T1 + w1·2a,      (poles 0,1,2 COLLINEAR along w1)
//     P4 = T2 + w2·a,  P3 = T2 + w2·2a,      (poles 5,4,3 COLLINEAR along w2)
// where wi is the UNIT in-face direction at Ti pointing away from the face interior
// (toward the removed corner) — the section leaves each rail travelling along the flat
// face — and a = kSectionSpacing·r sets the fullness. EXACT G2 proof (per rail):
//   * B'(0) = 5(P1−P0) = 5a·w1  ⇒ tangent lies in face 1 ⇒ G1 tangent at T1;
//   * B''(0) = 20(P0 − 2P1 + P2) = 20(T1 − 2(T1+a·w1) + (T1+2a·w1)) = 0 ⇒ the section
//     ACCELERATION is exactly zero at s=0, so its curvature κ(0)=|B'×B''|/|B'|³ = 0.
//   The symmetric algebra at s=1 gives B''(1)=20(P5 − 2P4 + P3)=0 ⇒ κ(1)=0.
// Both end-curvatures are IDENTICALLY zero by construction (the collinear-triple
// condition), matching the flat neighbours ⇒ genuine G2 (curvature) continuity at both
// seams — the property the stock circular arc (constant curvature 1/r) cannot have.
// This mirrors the OCCT-adapter reference construction (src/engine/occt/occt_g2_fillet.cpp,
// which likewise notes OCCT's BRepFilletAPI is G1/circular-only and builds the same
// quintic-with-collinear-rail-triples cross section), but here entirely OCCT-FREE and in
// the native planar-polygon-edit path.
//
// ── IMPLEMENTATION (planar-polygon edit; reuses the G1 machinery) ─────────────────
// Per picked convex edge, EXACTLY as blend/fillet_edges.h:
//   1. filletArc → the rolling-ball tangent points T1, T2 and crease direction t;
//   2. clip both adjacent faces back to the tangent-line CHAMFER plane through T1,T2;
//   3. tile the G2 QUINTIC section T1→…→T2 into deflection-bounded facet strips
//      (chord sagitta bounded like the arc), each strip a quad extruded across the edge
//      length; add the two end-cap segment faces;
//   4. re-weld through assembleSolid (open-seam weld, T-junction repair, triangulate).
// The engine's mandatory self-verify (watertight + a SHRINK volume 0 < Vr < Vo — a
// convex blend removes material) accepts it, else falls through to OCCT.
//
// ── SCOPE (honest) ────────────────────────────────────────────────────────────────
// Native ONLY for a CONVEX edge shared by exactly TWO PLANAR faces (a planar dihedral),
// radius small enough that both tangent lines stay within their faces. A CONCAVE edge,
// a CURVED adjacent face, an edge shared by ≠2 faces, multi-edge interference, a
// non-planar solid, or any self-verify failure → NULL → OCCT (occt_g2_fillet.cpp)
// fallthrough. The DEEP residual — arbitrary G2 on freeform / curved substrates — is
// NOT handled here (the real moat) and stays an honest decline. A G1 (merely-tangent)
// blend is NEVER emitted from this entry: the section is the zero-end-curvature quintic
// or nothing.
//
// CLEAN-ROOM. Reuses only src/native/math + topology + boolean + blend/fillet_edges +
// blend/chamfer_edges. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_H
#define CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_H

#include "native/blend/blend_geom.h"
#include "native/blend/chamfer_edges.h"  // detail::faceInward / centroidOf
#include "native/blend/fillet_edges.h"   // detail::filletArc / tangentCutPlane

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// Fullness of the G2 quintic section: the collinear-triple spacing a = kG2Spacing·r.
// 0.35 matches the OCCT-adapter reference (occt_g2_fillet.cpp) so the two paths build
// the same-shape section; small enough that the section stays a clean simple arc from
// T1 to T2 for a convex dihedral.
inline constexpr double kG2Spacing = 0.35;

// A G2 (curvature-continuous) quintic section for one convex edge, built from the
// rolling-ball geometry the G1 path already computes. The six poles are laid out so
// poles {P0,P1,P2} are collinear along the face-1 tangent and {P5,P4,P3} collinear
// along the face-2 tangent → B''(0)=B''(1)=0 → zero end-curvature (G2). Returns nullopt
// on the same degeneracies filletArc rejects.
struct G2Section {
  std::array<math::Point3, 6> poles;  // quintic Bézier poles P0..P5, P0=T1, P5=T2
  math::Vec3 t;                       // unit crease direction
  math::Point3 axis;                  // rolling-ball axis C (for outward-normal sign)
};

inline std::optional<G2Section> g2Section(const math::Point3& ea, const math::Point3& eb,
                                          const nb::Polygon& f1, const nb::Polygon& f2, double r) {
  const auto arc = filletArc(ea, eb, f1, f2, r);
  if (!arc) return std::nullopt;

  const math::Point3 mid{0.5 * (ea.x + eb.x), 0.5 * (ea.y + eb.y), 0.5 * (ea.z + eb.z)};
  // In-face inward directions (toward each face interior, away from the edge).
  const math::Vec3 in1 = faceInward(arc->t, f1.plane.normal, mid, centroidOf(f1.vertices));
  const math::Vec3 in2 = faceInward(arc->t, f2.plane.normal, mid, centroidOf(f2.vertices));
  if (math::isNull(in1) || math::isNull(in2)) return std::nullopt;
  // The section leaves each rail travelling ALONG the flat face toward the removed
  // corner (away from the face interior) → tangent direction wi = -in_i. This is what
  // makes the section tangent to face i at Ti (G1) while the collinear triple gives G2.
  const math::Vec3 w1 = -in1;
  const math::Vec3 w2 = -in2;

  const double a = kG2Spacing * r;
  G2Section sec;
  sec.t = arc->t;
  sec.axis = arc->axis;
  sec.poles[0] = arc->t1;              // P0 = T1
  sec.poles[1] = arc->t1 + w1 * a;     // P1  (P0,P1,P2 collinear along w1)
  sec.poles[2] = arc->t1 + w1 * (2.0 * a);
  sec.poles[3] = arc->t2 + w2 * (2.0 * a);
  sec.poles[4] = arc->t2 + w2 * a;
  sec.poles[5] = arc->t2;              // P5 = T2  (P5,P4,P3 collinear along w2)
  return sec;
}

// de Casteljau evaluation of the quintic section at s∈[0,1]. Trivial (6 poles), kept
// local so this header carries the SAME dependency footprint as fillet_edges.h.
inline math::Point3 quinticPoint(const std::array<math::Point3, 6>& poles, double s) {
  std::array<math::Point3, 6> p = poles;
  for (int k = 1; k < 6; ++k)
    for (int i = 0; i < 6 - k; ++i)
      p[i] = math::Point3{p[i].x + (p[i + 1].x - p[i].x) * s, p[i].y + (p[i + 1].y - p[i].y) * s,
                          p[i].z + (p[i + 1].z - p[i].z) * s};
  return p[0];
}

// Facet count for the G2 section so the chord sagitta is bounded like the G1 arc. The
// section spans roughly the same corner as the r-arc; use the arc's facet rule at a
// conservative full sweep (≤ π/2 for a convex dihedral) so refinement matches.
inline int g2Facets(double radius, double deflection) {
  return arcFacets(radius, M_PI / 2.0, deflection);
}

// Tile the G2 section T1→T2 into facet strips extruded across the edge (ea..eb), plus
// the two end-cap segment faces — the SAME structure as arcStrips but sampling the
// quintic instead of the circular arc. The outward normal of each strip is taken from
// the section's local outward direction (away from the rolling-ball axis C), so the
// strip winds outward like the G1 cylinder facets.
inline std::vector<nb::Polygon> g2Strips(const G2Section& sec, const math::Point3& ea,
                                         const math::Point3& eb, double radius,
                                         double deflection) {
  const int nf = g2Facets(radius, deflection);
  if (nf < 1) return {};

  const math::Point3 mid{0.5 * (ea.x + eb.x), 0.5 * (ea.y + eb.y), 0.5 * (ea.z + eb.z)};
  const double offA = math::dot(ea - mid, sec.t);
  const double offB = math::dot(eb - mid, sec.t);

  // A rim point at parameter step k (s=k/nf), translated along the crease by `off`.
  auto rimAt = [&](int k, double off) -> math::Point3 {
    const double s = static_cast<double>(k) / static_cast<double>(nf);
    const math::Point3 p = quinticPoint(sec.poles, s);
    return p + sec.t * off;
  };

  std::vector<nb::Polygon> strips;
  strips.reserve(static_cast<std::size_t>(nf) + 2);
  for (int k = 0; k < nf; ++k) {
    const math::Point3 aK = rimAt(k, offA);
    const math::Point3 aK1 = rimAt(k + 1, offA);
    const math::Point3 bK1 = rimAt(k + 1, offB);
    const math::Point3 bK = rimAt(k, offB);
    // The strip is a quad extruded across the edge, so its true plane normal is the
    // cross of its two spanning edges (the section chord × the crease) — NOT the radial
    // rimAt−C direction, which for the quintic (not a circle about C) is only
    // approximately normal and would make the assigned plane disagree with the quad
    // geometry (that mismatch is what breaks the weld). We take the geometric normal and
    // orient it OUTWARD using the radial rimAt−C purely as a sign reference (the section
    // bulges away from the rolling-ball axis).
    const math::Vec3 chord = (rimAt(k + 1, 0.0) - rimAt(k, 0.0));  // section step
    math::Vec3 gn = math::cross(chord, sec.t);                     // quad plane normal
    const math::Vec3 radial = (rimAt(k, 0.0) - sec.axis) + (rimAt(k + 1, 0.0) - sec.axis);
    if (math::dot(gn, radial) < 0.0) gn = -gn;
    const math::Dir3 nd{gn};
    if (!nd.valid()) continue;
    std::vector<math::Point3> loop{aK, aK1, bK1, bK};
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    strips.emplace_back(loop, nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  }

  // END CAPS — the section-segment gap in each end face (identical role to arcStrips).
  auto endCap = [&](double off, const math::Vec3& outward) {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(nf) + 1);
    for (int k = 0; k <= nf; ++k) ring.push_back(rimAt(k, off));
    const math::Dir3 nd{outward};
    if (!nd.valid() || ring.size() < 3) return;
    math::Vec3 areaV{0, 0, 0};
    for (std::size_t i = 0; i < ring.size(); ++i)
      areaV += math::cross(ring[i].asVec(), ring[(i + 1) % ring.size()].asVec());
    if (math::dot(areaV, nd.vec()) < 0.0) std::reverse(ring.begin(), ring.end());
    strips.emplace_back(ring, nb::Plane::fromPointNormal(ring.front(), nd.vec()));
  };
  endCap(offA, sec.t * (offA >= offB ? 1.0 : -1.0));
  endCap(offB, sec.t * (offB >= offA ? 1.0 : -1.0));
  return strips;
}

}  // namespace detail

// G2 (curvature-continuous) fillet on the convex planar-dihedral edges `edgeIds`
// (1-based, mapShapes order) of `solid` at nominal radius `r`. Returns the blended
// solid (a faceted zero-end-curvature quintic section, deflection-bounded), or a NULL
// Shape if any picked edge is not a convex edge between two planar faces / the solid is
// not planar / r is degenerate (→ OCCT occt_g2_fillet.cpp fallthrough). `deflection`
// bounds the facet chord error. A merely-G1 (circular) blend is NEVER produced here.
inline topo::Shape fillet_edges_g2(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                   double radius, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount <= 0 || !(radius > kBlendEps)) return {};
  PlanarModel model(solid);
  if (!model.isValid()) return {};

  std::vector<nb::Polygon> polys = model.polygons();
  for (int i = 0; i < edgeCount; ++i) {
    const auto ends = edgeEnds(solid, edgeIds[i]);
    if (!ends) return {};
    const math::Point3 mid{0.5 * (ends->a.x + ends->b.x), 0.5 * (ends->a.y + ends->b.y),
                           0.5 * (ends->a.z + ends->b.z)};

    std::size_t faces[2];
    if (facesOnEdgeInSoup(polys, ends->a, ends->b, faces) != 2) return {};

    const auto sec = detail::g2Section(ends->a, ends->b, polys[faces[0]], polys[faces[1]], radius);
    if (!sec) return {};
    // Set both faces back to the tangent lines T1,T2 (same chamfer clip as the G1
    // fillet); the G2 section strips fill the opening.
    const auto arc = detail::filletArc(ends->a, ends->b, polys[faces[0]], polys[faces[1]], radius);
    if (!arc) return {};
    const auto cut = detail::tangentCutPlane(*arc, mid);
    if (!cut) return {};

    std::vector<nb::Polygon> setback;
    setback.reserve(polys.size());
    for (const nb::Polygon& poly : polys) {
      std::vector<math::Point3> clipped = clipBelow(poly.vertices, *cut, kBlendEps);
      if (clipped.size() >= 3) setback.emplace_back(std::move(clipped), poly.plane);
    }
    std::vector<nb::Polygon> strips = detail::g2Strips(*sec, ends->a, ends->b, radius, deflection);
    if (strips.empty()) return {};
    for (nb::Polygon& s : strips) setback.push_back(std::move(s));
    polys = std::move(setback);
  }
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_H
