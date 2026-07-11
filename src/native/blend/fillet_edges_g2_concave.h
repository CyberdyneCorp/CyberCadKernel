// SPDX-License-Identifier: Apache-2.0
//
// fillet_edges_g2_concave.h — native G2 (CURVATURE-CONTINUOUS) blend fillet on a
// CONCAVE (reflex) edge between two PLANAR faces (MOAT M3, the second scoped slice of
// the drop-OCCT Class-B `fillet_edges_g2` after the convex planar dihedral).
//
// ── WHAT THIS IS, VS THE CONVEX G2 SLICE ──────────────────────────────────────────
// The convex G2 fillet (blend/fillet_edges_g2.h) rolls a ball into a CONVEX dihedral
// (n1+n2 points OUT of the material), seats the ball axis on the MATERIAL side, and
// REMOVES the corner wedge, replacing it with a zero-end-curvature QUINTIC section →
// a genuine G2 (κ=0 at both rails) blend that SHRINKS the solid.
//
// A CONCAVE (reflex) planar dihedral is the mirror image: the two faces meet at a
// valley where n1+n2 points INTO the void (away from the material — the inner corner
// of an L-shaped prism). A ball rolled into that valley seats on the VOID side and the
// fillet ADDS material, rounding the sharp inner corner. EXACTLY the convex machinery
// — the same quintic-with-collinear-rail-triples section (so B''(0)=B''(1)=0 → κ=0 at
// both seams, the identical G2 identity), the same setback clip, the same deflection-
// bounded facet tiling, the same watertight assembleSolid weld — is reused; ONLY three
// things flip sign for the concave side:
//   (1) BALL CENTRE / TANGENT POINTS — the rolling-ball axis is placed on the VOID side
//       (C = E + r/(1+n1·n2)·(n1+n2), the +sign vs the convex −sign) and each tangent
//       point is the foot of the perpendicular toward the plane, Ti = C − r·ni (vs the
//       convex Ti = C + r·ni). Both solve n_i·(C−E)=+r (tangent from the void) instead
//       of the convex −r (tangent from the material).
//   (2) STRIP OUTWARD NORMAL — the concave blend surface faces TOWARD the axis C (its
//       outward normal points into the void, toward the ball centre), so the outward
//       sign reference is (C − rim) — the NEGATION of the convex (rim − C).
//   (3) VOLUME SELF-VERIFY — a concave fillet ADDS material (0 < Vo < Vr), so the
//       engine's sane-volume guard expects a GROW, not the convex SHRINK.
//   (4) ASSEMBLY — the convex path REMOVES a wedge, so it globally clips every polygon to
//       the tangent-cut half-space. A concave fillet ADDS a sliver, so a global clip
//       would slice the whole solid; instead the soup is edited surgically: the two
//       adjacent faces are trimmed back to their tangent lines (clipped, and ONLY those
//       two), every OTHER face carrying the reflex-corner vertex (the prism end caps) has
//       that vertex REPLACED by the quintic rim so the cap grows to the fillet boundary,
//       and the quintic ROOF strips bridge T1→T2 into the void. No separate end-cap
//       faces — the corner splice closes the section on the cap planes.
// The section-tangent direction wi = −in_i (leave each rail along the flat face toward
// the corner being filled) is IDENTICAL to the convex path.
//
// ── THE G2 SECTION CURVE (closed-form curvature-continuous, PROVEN — same as convex) ─
// In the crease cross-section the section is the QUINTIC Bézier B(s), s∈[0,1], poles
//     P0 = T1,                  P5 = T2,
//     P1 = T1 + w1·a, P2 = T1 + w1·2a,   (poles 0,1,2 COLLINEAR along w1)
//     P4 = T2 + w2·a, P3 = T2 + w2·2a,   (poles 5,4,3 COLLINEAR along w2)
// with wi the unit in-face direction at Ti toward the filled corner and a=kG2Spacing·r.
// EXACT G2 proof (per rail), unchanged from the convex slice:
//   * B'(0)  = 5(P1−P0) = 5a·w1              ⇒ tangent lies in face 1 ⇒ G1 at T1;
//   * B''(0) = 20(P0 − 2P1 + P2) = 0         ⇒ zero acceleration ⇒ κ(0)=0 (G2);
//   * B''(1) = 20(P5 − 2P4 + P3) = 0         ⇒ κ(1)=0 (G2), by symmetry.
// Both end-curvatures are IDENTICALLY zero (the collinear-triple condition), matching
// the flat neighbours' zero normal curvature ⇒ genuine G2 at both seams — the property
// the stock circular arc (constant curvature 1/r) cannot have. The concavity changes
// WHERE the section sits and which way it bulges, NOT the curvature-continuity identity.
//
// ── SCOPE (honest) ────────────────────────────────────────────────────────────────
// Native ONLY for a CONCAVE edge shared by exactly TWO PLANAR faces (a reflex planar
// dihedral), radius small enough that both tangent lines stay within their faces. A
// CONVEX edge (→ fillet_edges_g2.h), a CURVED adjacent face, an edge shared by ≠2
// faces, a non-planar solid, or any self-verify failure → NULL → OCCT fallthrough. The
// DEEP residual — G2 on freeform / curved substrates — is NOT handled here and stays an
// honest decline. A merely-G1 (circular) blend is NEVER emitted from this entry.
//
// CLEAN-ROOM. Reuses only src/native/math + topology + boolean + blend/blend_geom +
// blend/chamfer_edges + blend/fillet_edges_g2. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_CONCAVE_H
#define CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_CONCAVE_H

#include "native/blend/blend_geom.h"
#include "native/blend/chamfer_edges.h"    // detail::faceInward / centroidOf
#include "native/blend/fillet_edges_g2.h"  // detail::G2Section / quinticPoint / g2Facets / kG2Spacing

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// The rolling-ball cylinder for a CONCAVE (reflex) edge (ea,eb) between planar faces
// f1,f2 at radius r — the sign-flipped twin of filletArc. The ball seats on the VOID
// side of the valley (C = E + r/(1+c)·(n1+n2)) and touches each face at Ti = C − r·ni.
// Returns nullopt unless this is a clean CONCAVE planar dihedral (the convex guard is
// inverted: the corner-outward direction n1+n2 must point INTO the void, i.e. AGAINST
// each in-face inward direction pointing back at the corner).
inline std::optional<FilletArc> concaveFilletArc(const math::Point3& ea, const math::Point3& eb,
                                                 const nb::Polygon& f1, const nb::Polygon& f2,
                                                 double r) {
  const auto tOpt = creaseDir(ea, eb);
  if (!tOpt) return std::nullopt;
  const math::Vec3 t = tOpt->vec();
  const math::Point3 mid{0.5 * (ea.x + eb.x), 0.5 * (ea.y + eb.y), 0.5 * (ea.z + eb.z)};

  const math::Vec3 n1 = f1.plane.normal;
  const math::Vec3 n2 = f2.plane.normal;

  const math::Vec3 in1 = faceInward(t, n1, mid, centroidOf(f1.vertices));
  const math::Vec3 in2 = faceInward(t, n2, mid, centroidOf(f2.vertices));
  if (math::isNull(in1) || math::isNull(in2)) return std::nullopt;

  // CONCAVITY guard — the exact inversion of the convex guard in filletArc. For a
  // convex edge n1+n2 (corner-outward) OPPOSES each inward dir (dot < −eps). For a
  // reflex edge the material fills MORE than the dihedral half-spaces, so n1+n2 points
  // into the void and AGREES with each inward dir (dot > +eps). Require both > +eps so
  // a convex or near-flat edge is declined (→ the convex builder / OCCT owns it).
  const math::Vec3 cornerOut = n1 + n2;
  if (math::dot(cornerOut, in1) < kBlendEps || math::dot(cornerOut, in2) < kBlendEps)
    return std::nullopt;

  const double c = math::dot(n1, n2);
  if (c <= -1.0 + kBlendEps) return std::nullopt;  // planes anti-parallel (no corner)
  // Ball centre on the VOID side: n_i·(C−E) = +r for both i ⇒ C−E = r/(1+c)·(n1+n2).
  const math::Vec3 x = (n1 + n2) * (r / (1.0 + c));  // C − E  (opposite sign to convex)
  const math::Point3 C = mid + x;
  const math::Point3 T1 = C - n1 * r;  // foot of perpendicular toward plane 1 (concave)
  const math::Point3 T2 = C - n2 * r;
  return FilletArc{C, T1, T2, t, r};
}

// A G2 (curvature-continuous) quintic section for one CONCAVE edge. Identical pole
// layout to the convex g2Section (collinear rail-triples ⇒ B''(0)=B''(1)=0 ⇒ κ=0 at
// both rails); it differs ONLY in that the tangent points come from concaveFilletArc
// (the ball seats in the void). Returns nullopt on the same degeneracies.
inline std::optional<G2Section> g2SectionConcave(const math::Point3& ea, const math::Point3& eb,
                                                 const nb::Polygon& f1, const nb::Polygon& f2,
                                                 double r) {
  const auto arc = concaveFilletArc(ea, eb, f1, f2, r);
  if (!arc) return std::nullopt;

  const math::Point3 mid{0.5 * (ea.x + eb.x), 0.5 * (ea.y + eb.y), 0.5 * (ea.z + eb.z)};
  const math::Vec3 in1 = faceInward(arc->t, f1.plane.normal, mid, centroidOf(f1.vertices));
  const math::Vec3 in2 = faceInward(arc->t, f2.plane.normal, mid, centroidOf(f2.vertices));
  if (math::isNull(in1) || math::isNull(in2)) return std::nullopt;
  // The section leaves each rail travelling ALONG the flat face toward the FILLED corner
  // (away from the face interior) → wi = −in_i. Same as convex: this is what makes the
  // section tangent to face i at Ti (G1) while the collinear triple gives G2.
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

// The tangent-cut plane through the two concave tangent lines (T1,T2 ∥ crease). Same
// construction as tangentCutPlane, oriented so the faces keep the side AWAY from the
// reflex corner (the corner tip edgeMid on the +normal side); clipBelow then trims each
// face back to its tangent line and the concave quintic strips fill the corner.
inline std::optional<nb::Plane> concaveTangentCutPlane(const FilletArc& fa,
                                                       const math::Point3& edgeMid) {
  math::Vec3 n = math::cross(fa.t, fa.t2 - fa.t1);
  const math::Dir3 nd{n};
  if (!nd.valid()) return std::nullopt;
  nb::Plane pl = nb::Plane::fromPointNormal(fa.t1, nd.vec());
  if (signedDist(pl, edgeMid) < 0.0) { pl.normal = -pl.normal; pl.w = -pl.w; }
  return pl;
}

// Tile the CONCAVE G2 section T1→T2 into the ROOF facet strips extruded across the edge
// (ea..eb). Unlike the convex g2Strips this returns ONLY the curved roof — the two end
// caps are NOT separate faces here: a concave fillet ADDS material, so the reflex-corner
// vertex of each adjacent end face is spliced out and REPLACED by the quintic rim (see
// spliceCornerCap / the entry), which closes the section on those planes. The roof's
// outward normal points AWAY from the filled sliver (into the void, away from the reflex
// corner), so the sign reference is (rim − corner) — the concave roof bulges out of the
// notch. The true plane normal is the geometric quad normal (section-chord × crease);
// (rim − corner) only fixes its sign.
inline std::vector<nb::Polygon> g2RoofStripsConcave(const G2Section& sec, const math::Point3& ea,
                                                    const math::Point3& eb, double radius,
                                                    double deflection) {
  const int nf = g2Facets(radius, deflection);
  if (nf < 1) return {};

  const math::Point3 mid{0.5 * (ea.x + eb.x), 0.5 * (ea.y + eb.y), 0.5 * (ea.z + eb.z)};
  const double offA = math::dot(ea - mid, sec.t);
  const double offB = math::dot(eb - mid, sec.t);

  auto rimAt = [&](int k, double off) -> math::Point3 {
    const double s = static_cast<double>(k) / static_cast<double>(nf);
    const math::Point3 p = quinticPoint(sec.poles, s);
    return p + sec.t * off;
  };

  std::vector<nb::Polygon> strips;
  strips.reserve(static_cast<std::size_t>(nf));
  for (int k = 0; k < nf; ++k) {
    const math::Point3 aK = rimAt(k, offA);
    const math::Point3 aK1 = rimAt(k + 1, offA);
    const math::Point3 bK1 = rimAt(k + 1, offB);
    const math::Point3 bK = rimAt(k, offB);
    const math::Vec3 chord = (rimAt(k + 1, 0.0) - rimAt(k, 0.0));  // section step
    math::Vec3 gn = math::cross(chord, sec.t);                     // quad plane normal
    // CONCAVE sign reference: outward points AWAY from the reflex corner (into the void),
    // so use (rim − corner) with corner = the crease point at the section (mid).
    const math::Vec3 radialOut = (rimAt(k, 0.0) - mid) + (rimAt(k + 1, 0.0) - mid);
    if (math::dot(gn, radialOut) < 0.0) gn = -gn;
    const math::Dir3 nd{gn};
    if (!nd.valid()) continue;
    std::vector<math::Point3> loop{aK, aK1, bK1, bK};
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    strips.emplace_back(loop, nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  }
  return strips;
}

// The concave quintic rim point k∈[0,nf], sampled on the section built through the edge
// midpoint and translated along the crease onto the cross-section plane through `corner`
// (an edge endpoint). rim(0)=T1, rim(nf)=T2, at the corner's crease coordinate.
inline math::Point3 concaveRimAtCorner(const G2Section& sec, const math::Point3& corner, int nf,
                                       int k) {
  const double s = static_cast<double>(k) / static_cast<double>(nf);
  const math::Point3 p = quinticPoint(sec.poles, s);
  // Shift the whole section from the midpoint plane to the corner plane along the crease.
  const double off = math::dot(corner - sec.poles[0], sec.t);
  return p + sec.t * off;
}

// Splice the concave quintic rim into an END face `poly` that carries the reflex-corner
// vertex `corner` (an endpoint of the picked edge): replace that single vertex by the
// ordered quintic rim T1→…→T2 (on the corner's plane), so the cap face GROWS to the
// fillet boundary (rounding the reflex vertex) rather than keeping the sharp corner. The
// insertion order is chosen so the rim's endpoints join the corner's two polygon
// neighbours (loop stays simple, winding intact). Returns nullopt if `poly` does not
// contain `corner` (caller keeps it unchanged).
inline std::optional<nb::Polygon> spliceCornerCap(const nb::Polygon& poly, const G2Section& sec,
                                                  const math::Point3& corner, int nf) {
  const auto& vs = poly.vertices;
  int ci = -1;
  for (std::size_t j = 0; j < vs.size(); ++j)
    if (math::distance(vs[j], corner) < 1e-6) { ci = static_cast<int>(j); break; }
  if (ci < 0) return std::nullopt;

  const math::Point3 t1 = concaveRimAtCorner(sec, corner, nf, 0);
  const math::Point3 t2 = concaveRimAtCorner(sec, corner, nf, nf);
  const math::Point3 nextV = vs[(ci + 1) % vs.size()];
  const bool fwd = math::distance(t2, nextV) < math::distance(t1, nextV);

  std::vector<math::Point3> np;
  np.reserve(vs.size() + static_cast<std::size_t>(nf));
  for (std::size_t j = 0; j < vs.size(); ++j) {
    if (static_cast<int>(j) == ci) {
      for (int k = 0; k <= nf; ++k)
        np.push_back(concaveRimAtCorner(sec, corner, nf, fwd ? k : nf - k));
    } else {
      np.push_back(vs[j]);
    }
  }
  return nb::Polygon(np, poly.plane);
}

}  // namespace detail

// G2 (curvature-continuous) fillet on the CONCAVE planar-dihedral edges `edgeIds`
// (1-based, mapShapes order) of `solid` at nominal radius `r`. Returns the blended solid
// (a faceted zero-end-curvature quintic section that ADDS material into the reflex
// corner, deflection-bounded), or a NULL Shape if any picked edge is not a concave edge
// between two planar faces / the solid is not planar / r is degenerate (→ OCCT
// fallthrough). `deflection` bounds the facet chord error. A merely-G1 (circular) blend
// is NEVER produced here.
inline topo::Shape fillet_edges_g2_concave(const topo::Shape& solid, const int* edgeIds,
                                           int edgeCount, double radius, double deflection = 0.01) {
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

    const auto sec =
        detail::g2SectionConcave(ends->a, ends->b, polys[faces[0]], polys[faces[1]], radius);
    if (!sec) return {};
    const auto arc =
        detail::concaveFilletArc(ends->a, ends->b, polys[faces[0]], polys[faces[1]], radius);
    if (!arc) return {};
    const auto cut = detail::concaveTangentCutPlane(*arc, mid);
    if (!cut) return {};

    const int nf = detail::g2Facets(radius, deflection);
    if (nf < 1) return {};

    // A concave fillet ADDS material into the reflex corner. Unlike the convex path (which
    // globally clips the wedge off), here we edit the soup surgically so the added sliver
    // exactly fills the notch:
    //   (a) the two ADJACENT faces are trimmed back to their tangent lines T1,T2 (clip
    //       each — and ONLY those two — to keep the side away from the reflex corner);
    //   (b) every OTHER face carrying the reflex-corner vertex (the end caps of the
    //       prism) has that vertex REPLACED by the quintic rim (spliceCornerCap), so the
    //       cap grows to the fillet boundary and closes the section on its plane;
    //   (c) the curved quintic ROOF strips bridge T1→T2 into the void.
    std::vector<nb::Polygon> next;
    next.reserve(polys.size() + static_cast<std::size_t>(nf));
    for (std::size_t k = 0; k < polys.size(); ++k) {
      if (k == faces[0] || k == faces[1]) {
        std::vector<math::Point3> clipped = clipBelow(polys[k].vertices, *cut, kBlendEps);
        if (clipped.size() >= 3) next.emplace_back(std::move(clipped), polys[k].plane);
        continue;
      }
      auto spliced = detail::spliceCornerCap(polys[k], *sec, ends->a, nf);
      if (!spliced) spliced = detail::spliceCornerCap(polys[k], *sec, ends->b, nf);
      if (spliced) {
        next.push_back(std::move(*spliced));
      } else {
        next.push_back(polys[k]);
      }
    }
    std::vector<nb::Polygon> roof =
        detail::g2RoofStripsConcave(*sec, ends->a, ends->b, radius, deflection);
    if (roof.empty()) return {};
    for (nb::Polygon& s : roof) next.push_back(std::move(s));
    polys = std::move(next);
  }
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_FILLET_EDGES_G2_CONCAVE_H
