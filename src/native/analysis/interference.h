// SPDX-License-Identifier: Apache-2.0
//
// interference.h — MOAT M-GS GS7 CLASH / INTERFERENCE detection between two solids
// (assemblies + mates value). Given the two solids' M0 boundary meshes it decides,
// OCCT-FREE, whether the pair CLASHES (interiors overlap with positive volume),
// merely TOUCHES (zero-volume boundary contact), or is CLEAR (a positive clearance
// gap), and reports a WITNESS (the overlap AABB + a representative interior point)
// and the minimum boundary clearance.
//
// STATE MACHINE (the quantity the app's assembly mates care about):
//   * CLASH    — the two solids' INTERIORS intersect over a set of positive volume.
//                Detected robustly + COPLANAR-SAFELY at the mesh level via TWO
//                complementary signatures. (i) ENCLOSURE: either operand has a boundary
//                VERTEX, or a boundary TRIANGLE CENTROID, that classifies strictly
//                INSIDE the other. A shared face reads On (the B3 ON-band absorbs the
//                seam), never In, so a flush TOUCH never fires; a face poking through the
//                other's wall has an interior centroid → CLASH. (ii) PASS-THROUGH: an
//                EDGE of one solid pierces a FACE of the other TRANSVERSALLY through its
//                interior — the signal for a bar poking CLEAN THROUGH a slab (or vice
//                versa), a positive-volume overlap where NEITHER solid has a
//                vertex/centroid inside the other (bar ends stick out both slab faces;
//                the slab is wider than the bar) so enclosure alone MISSES it. A
//                coplanar/endpoint contact never pierces an interior → TOUCHING is
//                unaffected. (A raw Möller tri–tri crossing was rejected for enclosure:
//                it over-reports at a shared seam where edge-adjacent triangles of two
//                flush solids "cross" though nothing interpenetrates; the pass-through
//                pierce is seam-safe by requiring a STRICTLY interior crossing.)
//   * TOUCHING  — no interior overlap, but the boundaries make CONTACT: the minimum
//                triangle–triangle distance is within the mesh fidelity band (≈0) and
//                no penetration signature fired. (Two boxes sharing a face; a shaft
//                seated in a bore at nominal fit.)
//   * CLEAR     — the minimum boundary distance exceeds the contact band: a genuine
//                clearance gap. The min distance IS the clearance.
//
// WITNESS. On CLASH the service returns the AABB of the penetration evidence (the
// inside-classified boundary vertices ∪ triangle centroids) and a representative
// interior point (the first boundary vertex / centroid confirmed strictly inside the
// other solid; the engine sharpens it to the COMMON centroid). The overlap VOLUME is NOT
// computed here — it is the native boolean COMMON, which the ENGINE composes over
// this classification (interference.h is mesh-math only and OCCT-free / boolean-free);
// the engine's two-sided volume self-verify guards it (design §self-verify).
//
// SELF-VERIFY / HONEST DECLINE (Membership::Unknown → CLASH_UNKNOWN): the classifier
// only asserts CLASH / TOUCHING / CLEAR when the mesh evidence is UNAMBIGUOUS. A
// non-watertight operand mesh, or a boundary point the B3 ray-parity classifier
// declines (grazing / disagreeing rays), yields an UNKNOWN verdict — the engine then
// falls through to the OCCT oracle rather than emit a wrong clash flag. A wrong
// overlap is NEVER returned.
//
// OCCT is the ORACLE only (the sim gate: BRepAlgoAPI_Common volume + BRepGProp for
// the overlap, BRepExtrema_DistShapeShape for touching/clearance); no OCCT header is
// included here. Header-only, mesh-math only (reuses the SAME freeform_membership B3
// classifier + point-triangle distance as the landed analysis/boolean layers).
// clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_ANALYSIS_INTERFERENCE_H
#define CYBERCAD_NATIVE_ANALYSIS_INTERFERENCE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "native/boolean/freeform_membership.h" // classifyPointInMesh / Membership (B3)
#include "native/math/vec.h"
#include "native/tessellate/mesh.h"

namespace cybercad::native::analysis {

namespace math = cybercad::native::math;
namespace tessellate = cybercad::native::tessellate;
namespace nbool = cybercad::native::boolean;

/// The three interference states plus an honest decline.
enum class ClashState : std::uint8_t {
  Clear,     ///< positive clearance gap (min distance > contact band)
  Touching,  ///< boundary contact, no interior overlap (distance ≈ 0)
  Clash,     ///< interiors overlap over a set of positive volume
  Unknown    ///< the mesh evidence is ambiguous → engine falls through to OCCT
};

inline const char* clashStateName(ClashState s) noexcept {
  switch (s) {
    case ClashState::Clear: return "Clear";
    case ClashState::Touching: return "Touching";
    case ClashState::Clash: return "Clash";
    case ClashState::Unknown: return "Unknown";
  }
  return "?";
}

/// The mesh-level interference verdict of two solids A and B. `overlapVolume` is
/// filled by the ENGINE (the native boolean COMMON); this mesh classifier fills the
/// state, the clearance, and the witness (present only when state == Clash).
struct InterferenceResult {
  ClashState state = ClashState::Unknown;
  double minDistance = 0.0;   ///< min boundary clearance (meaningful for Clear/Touching)
  double overlapVolume = 0.0; ///< COMMON volume (engine-filled; 0 unless Clash)

  bool hasWitness = false;    ///< true ⇒ the fields below are the penetration evidence
  math::Point3 witnessLo{};   ///< overlap-evidence AABB min corner
  math::Point3 witnessHi{};   ///< overlap-evidence AABB max corner
  math::Point3 witnessPoint{};///< a representative interior point of the overlap

  bool clash() const noexcept { return state == ClashState::Clash; }
};

namespace idetail {

/// Accumulate a point into an (lo,hi) running AABB. `any` gates the first point.
inline void growBox(const math::Point3& p, math::Point3& lo, math::Point3& hi, bool& any) {
  if (!any) { lo = hi = p; any = true; return; }
  lo.x = std::min(lo.x, p.x); hi.x = std::max(hi.x, p.x);
  lo.y = std::min(lo.y, p.y); hi.y = std::max(hi.y, p.y);
  lo.z = std::min(lo.z, p.z); hi.z = std::max(hi.z, p.z);
}

/// AABB of one mesh triangle by vertex index.
inline void triBox(const tessellate::Mesh& m, const tessellate::Triangle& t,
                   math::Point3& lo, math::Point3& hi) {
  const math::Point3& a = m.vertices[t.a];
  const math::Point3& b = m.vertices[t.b];
  const math::Point3& c = m.vertices[t.c];
  lo = {std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::min({a.z, b.z, c.z})};
  hi = {std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::max({a.z, b.z, c.z})};
}

inline bool boxDisjoint(const math::Point3& plo, const math::Point3& phi,
                        const math::Point3& qlo, const math::Point3& qhi, double margin) {
  return phi.x + margin < qlo.x || qhi.x + margin < plo.x ||
         phi.y + margin < qlo.y || qhi.y + margin < plo.y ||
         phi.z + margin < qlo.z || qhi.z + margin < plo.z;
}

/// Centroid of a triangle (a witness interior seed on a crossing pair).
inline math::Point3 triCentroid(const tessellate::Mesh& m, const tessellate::Triangle& t) {
  const math::Vec3 s = m.vertices[t.a].asVec() + m.vertices[t.b].asVec() + m.vertices[t.c].asVec();
  return math::Point3{s.x / 3.0, s.y / 3.0, s.z / 3.0};
}

/// The overall AABB of a mesh's vertices (for the model scale + the boolean B3 bbox).
inline void meshBounds(const tessellate::Mesh& m, math::Point3& lo, math::Point3& hi) {
  bool any = false;
  for (const math::Point3& v : m.vertices) growBox(v, lo, hi, any);
}

/// Closed-form minimum distance between two line SEGMENTS [p1,q1] and [p2,q2]
/// (Ericson, Real-Time Collision Detection §5.1.9). Clamps the barycentric
/// parameters (s,t) to [0,1] so the closest points stay on the segments, and
/// handles the parallel / degenerate (zero-length) cases via the near-zero
/// denominator guard. This is the edge–edge term the tri–tri distance needs:
/// for two disjoint convex triangles the true minimum distance is attained at
/// either a vertex–face pair OR an edge–edge pair, and the vertex–face tests
/// alone miss the coplanar-overlap pose where two faces cross with no mutually
/// contained vertex. Pure math — OCCT-free.
inline double segmentSegmentDistance(const math::Point3& p1, const math::Point3& q1,
                                     const math::Point3& p2, const math::Point3& q2) noexcept {
  const math::Vec3 d1 = q1 - p1;  // direction of segment 1
  const math::Vec3 d2 = q2 - p2;  // direction of segment 2
  const math::Vec3 r = p1 - p2;
  const double a = math::dot(d1, d1);  // squared length of segment 1
  const double e = math::dot(d2, d2);  // squared length of segment 2
  const double f = math::dot(d2, r);

  constexpr double eps = 1e-24;  // squared-length degeneracy guard (fp64 scale)
  double s = 0.0, t = 0.0;

  if (a <= eps && e <= eps) {
    // Both segments degenerate to points.
    return math::norm(p1 - p2);
  }
  if (a <= eps) {
    // Segment 1 is a point; clamp t of the projection onto segment 2.
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = math::dot(d1, r);
    if (e <= eps) {
      // Segment 2 is a point; clamp s of the projection onto segment 1.
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      // General non-degenerate case.
      const double b = math::dot(d1, d2);
      const double denom = a * e - b * b;  // ≥ 0; == 0 ⇒ parallel segments
      if (denom > eps) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      } else {
        s = 0.0;  // parallel: pick an arbitrary point on segment 1, clamp t below
      }
      t = (b * s + f) / e;
      // If t is out of [0,1], clamp it and recompute s for the clamped t.
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  const math::Point3 c1 = p1 + d1 * s;
  const math::Point3 c2 = p2 + d2 * t;
  return math::norm(c1 - c2);
}

/// Minimum distance between the three edges of triangle (a1,b1,c1) and the three
/// edges of triangle (a2,b2,c2) — the 9 edge–edge sub-tests. Combined with the
/// six vertex–face tests this gives the exact tri–tri minimum for disjoint convex
/// triangles.
inline double triEdgeEdgeDistance(const math::Point3& a1, const math::Point3& b1,
                                  const math::Point3& c1, const math::Point3& a2,
                                  const math::Point3& b2, const math::Point3& c2) noexcept {
  const math::Point3 e1p[3] = {a1, b1, c1};
  const math::Point3 e1q[3] = {b1, c1, a1};
  const math::Point3 e2p[3] = {a2, b2, c2};
  const math::Point3 e2q[3] = {b2, c2, a2};
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      best = std::min(best, segmentSegmentDistance(e1p[i], e1q[i], e2p[j], e2q[j]));
  return best;
}

/// Does the SEGMENT [p,q] pierce the triangle (a,b,c) TRANSVERSALLY through its
/// interior — i.e. cross the face plane at a point strictly inside both the segment
/// and the triangle? This is the pass-through penetration signal: a bar poking
/// CLEAN THROUGH a slab has its side edges pierce the slab's top/bottom faces even
/// though no vertex or centroid of either solid is mutually contained (the missed
/// CLASH signature). Returns the pierce point when a transversal interior crossing
/// exists, else nullopt.
///
/// ROBUSTNESS / TOUCH-SAFETY (never over-reports a flush TOUCH):
///   * A segment COPLANAR with the face (a shared-seam edge, or the coplanar
///     plus-cross edges) makes the ray-plane determinant ~0 → Möller declines →
///     nullopt. A coplanar overlap is a zero-volume TOUCH, not a pass-through.
///   * The crossing parameter t must be STRICTLY interior to the segment (an
///     endpoint touching the face — a shaft seated flush against a bore wall — is
///     contact, not a through-crossing), by a relative margin `segEps`.
///   * The barycentric hit (u,v) must be STRICTLY interior to the triangle by a
///     margin `baryEps` (a pierce exactly on a triangle EDGE is a seam artifact,
///     not an interior transversal). Only a hit comfortably inside the face fires.
/// Pure math (reuses the landed Möller–Trumbore ray-triangle kernel) — OCCT-free.
inline std::optional<math::Point3> segmentPiercesTriangleInterior(
    const math::Point3& p, const math::Point3& q, const math::Point3& a,
    const math::Point3& b, const math::Point3& c) noexcept {
  const math::Vec3 seg = q - p;
  const double len = math::norm(seg);
  if (len <= 1e-15) return std::nullopt;  // degenerate edge → no crossing
  const math::Vec3 dir = seg / len;       // unit direction; Möller reports t in model units
  // Möller only reports FORWARD crossings (t > eps); a through-edge that pierces
  // "behind" p is caught when the incident face is visited from the other operand,
  // so a single forward test per (directed) edge suffices for the pass-through OR.
  const std::optional<nbool::RayTriHit> hit = nbool::mollerTrumbore(p, dir, a, b, c);
  if (!hit) return std::nullopt;  // parallel / coplanar → no transversal crossing
  // Strictly interior to the segment: not at either endpoint (a flush seat touches).
  const double segEps = 1e-9 * len;
  if (hit->t <= segEps || hit->t >= len - segEps) return std::nullopt;
  // Strictly interior to the triangle: not on an edge/vertex (a seam grazing).
  const double baryEps = 1e-9;
  if (hit->u <= baryEps || hit->v <= baryEps || hit->u + hit->v >= 1.0 - baryEps)
    return std::nullopt;
  return math::Point3{p.x + dir.x * hit->t, p.y + dir.y * hit->t, p.z + dir.z * hit->t};
}

}  // namespace idetail

// ─────────────────────────────────────────────────────────────────────────────
// meshInterference — classify the interference of two solids by their WATERTIGHT
// M0 boundary meshes. `deflection` is the deflection the meshes were built at (it
// scales the B3 ON-band and the contact band). The overlap VOLUME is left 0 (the
// engine fills it with the native COMMON); everything else is decided here.
//
// ALGORITHM (all evidence is at the mesh level, no boolean needed):
//   1. Preconditions — both meshes must be watertight; else UNKNOWN (decline).
//   2. Penetration signature — CLASH via either of two complementary signals:
//      (2a/2b) ENCLOSURE — a boundary VERTEX or a boundary TRIANGLE CENTROID of one
//         solid classifies strictly IN the other (B3 multi-ray parity). A shared face
//         reads On (absorbed by the B3 ON-band), never In, so a face-flush TOUCH never
//         fires; a face poking through the other's wall has an interior centroid → CLASH.
//         A lone Membership::Unknown (ambiguous ray) with no confirmed inside point
//         declines (UNKNOWN) rather than guess.
//      (2c) PASS-THROUGH — an EDGE of one solid pierces a FACE of the other
//         TRANSVERSALLY through its interior (segment–triangle interior crossing). This
//         catches a bar poking CLEAN THROUGH a slab, a positive-volume overlap where NO
//         vertex/centroid is mutually contained (enclosure misses it). Seam-safe: a
//         coplanar or endpoint contact never pierces an interior, so TOUCHING/CLEAR are
//         unaffected. Only evaluated when 2a/2b found no enclosed point.
//   3. Witness — on CLASH, the AABB of the inside points + a representative interior
//      point (the engine sharpens this to the COMMON solid's box + centroid).
//   4. No penetration — compute the min triangle–triangle distance (min over the
//      six vertex–face AND the nine edge–edge sub-tests, the exact tri–tri minimum
//      for disjoint convex triangles); TOUCHING within the contact band, else CLEAR
//      (the min distance is the clearance). The edge–edge term is required for the
//      coplanar-overlap pose (two faces crossing with no mutually contained vertex).
// ─────────────────────────────────────────────────────────────────────────────
inline InterferenceResult meshInterference(const tessellate::Mesh& a, const tessellate::Mesh& b,
                                           double deflection) {
  InterferenceResult r;

  // ── (1) Preconditions: ray parity + enclosed classification need watertight input.
  if (!tessellate::isWatertight(a) || !tessellate::isWatertight(b)) {
    r.state = ClashState::Unknown;
    return r;
  }

  const nbool::Aabb bbA = nbool::meshAabb(a);
  const nbool::Aabb bbB = nbool::meshAabb(b);
  math::Point3 loA{}, hiA{}, loB{}, hiB{};
  idetail::meshBounds(a, loA, hiA);
  idetail::meshBounds(b, loB, hiB);
  const double scale = std::max({hiA.x - loA.x, hiA.y - loA.y, hiA.z - loA.z,
                                 hiB.x - loB.x, hiB.y - loB.y, hiB.z - loB.z, 1e-12});

  // Whole-body AABB reject: if the two solids' bounding boxes are separated by more
  // than a contact band they cannot clash OR touch → CLEAR fast-path (still measure
  // the real min distance below when boxes are close). Only skip the O(n·m) work when
  // provably clear by a comfortable margin.
  const double contactBand = std::max(1e-9 * scale, 2.0 * deflection);

  // ── (2) Penetration signature ───────────────────────────────────────────────
  bool anyInside = false;       // a confirmed strictly-inside boundary vertex
  bool anyUnknown = false;      // an ambiguous ray verdict (would decline if alone)
  bool anyCross = false;        // a transversal triangle crossing (unambiguous clash)
  math::Point3 wLo{}, wHi{};
  bool wAny = false;
  math::Point3 wPoint{};
  bool wPointSet = false;

  // A Membership::Unknown (grazing / disagreeing rays) only VETOES the verdict when
  // the point could plausibly be MASKING an interior overlap — i.e. it lies strictly
  // INSIDE the target's axis-aligned bounding box (the only region an interior point
  // can occupy) AND beyond the contact band of the target's boundary. A point OUTSIDE
  // the target's AABB cannot be interior no matter what the rays did (a spurious
  // coplanar-grazing Unknown on a far face), and an Unknown AT the contact seam is an
  // expected artifact of a legitimate touch — neither forces a decline. Only a
  // genuinely-ambiguous point in the enclosable interior region is an honest decline.
  auto insideBox = [](const nbool::Aabb& box, const math::Point3& p, double m) {
    return p.x > box.lo.x - m && p.x < box.hi.x + m && p.y > box.lo.y - m &&
           p.y < box.hi.y + m && p.z > box.lo.z - m && p.z < box.hi.z + m;
  };
  auto noteUnknown = [&](const tessellate::Mesh& dst, const nbool::Aabb& dstBox,
                         const math::Point3& p) {
    if (insideBox(dstBox, p, contactBand) && nbool::minDistanceToMesh(dst, p) > contactBand)
      anyUnknown = true;
  };

  // (2a) Boundary vertices of A inside B, and of B inside A. A single confirmed
  // inside vertex is a witness interior point.
  auto classifyVerts = [&](const tessellate::Mesh& src, const tessellate::Mesh& dst,
                           const nbool::Aabb& dstBox) {
    for (const math::Point3& p : src.vertices) {
      const nbool::Membership m = nbool::classifyPointInMesh(dst, dstBox, deflection, p);
      if (m == nbool::Membership::In) {
        anyInside = true;
        idetail::growBox(p, wLo, wHi, wAny);
        if (!wPointSet) { wPoint = p; wPointSet = true; }
      } else if (m == nbool::Membership::Unknown) {
        noteUnknown(dst, dstBox, p);
      }
    }
  };
  classifyVerts(a, b, bbB);
  classifyVerts(b, a, bbA);

  // (2b) Triangle-CENTROID membership — the coplanar-safe penetration signal.
  // A boundary VERTEX (2a) can sit exactly on a shared corner/edge and read On even
  // under real penetration, and a raw Möller tri–tri crossing over-reports at a
  // shared seam (edge-adjacent triangles of two flush solids "cross" along the seam
  // though nothing interpenetrates). The reliable, coplanar-safe signal is: the
  // CENTROID of a boundary triangle of one solid lies strictly INSIDE the other.
  // For two solids that merely TOUCH, every shared-seam triangle centroid classifies
  // On (never In) — the B3 ON-band absorbs it — so a touch never fires; for genuine
  // interpenetration a face poking through the other's wall has its centroid strictly
  // interior. This reuses the SAME B3 classifier and needs no tri–tri crossing test.
  const std::size_t nb = b.triangles.size();
  std::vector<math::Point3> bLo(nb), bHi(nb);  // B triangle boxes (reused by step 4)
  for (std::size_t j = 0; j < nb; ++j) idetail::triBox(b, b.triangles[j], bLo[j], bHi[j]);

  auto centroidsInside = [&](const tessellate::Mesh& src, const tessellate::Mesh& dst,
                             const nbool::Aabb& dstBox) {
    for (const tessellate::Triangle& t : src.triangles) {
      const math::Point3 c = idetail::triCentroid(src, t);
      const nbool::Membership m = nbool::classifyPointInMesh(dst, dstBox, deflection, c);
      if (m == nbool::Membership::In) {
        anyCross = true;
        idetail::growBox(c, wLo, wHi, wAny);
        if (!wPointSet) { wPoint = c; wPointSet = true; }
      } else if (m == nbool::Membership::Unknown) {
        noteUnknown(dst, dstBox, c);
      }
    }
  };
  if (!anyInside) {
    centroidsInside(a, b, bbB);
    centroidsInside(b, a, bbA);
  }

  // (2c) PASS-THROUGH penetration signature — the missed CLASH case this fix adds.
  // The vertex/centroid signatures (2a/2b) catch a solid that ENCLOSES a point of the
  // other. But a bar poking CLEAN THROUGH a slab (or a slab pierced by a thin bar) can
  // overlap with POSITIVE volume while NEITHER solid has a vertex or a triangle centroid
  // inside the other — the bar's ends stick out both sides of the slab (its vertices are
  // outside the slab), the slab is wider than the bar (its vertices are outside the bar),
  // and every triangle centroid lands outside. The interiors still overlap. The robust
  // mesh-level signal is a transversal PIERCE: an EDGE of one solid crosses a FACE of the
  // other through its interior (segment–triangle interior crossing). A single interior
  // pierce means the edge passes from outside to inside the other solid's wall, so the
  // interiors overlap → CLASH. TOUCHING (coplanar/endpoint contact) never pierces the
  // interior (see segmentPiercesTriangleInterior touch-safety), so this leaves TOUCHING
  // and CLEAR unaffected. Runs only when 2a/2b found no enclosed point (the common,
  // already-decided cases short-circuit before this O(edges·faces) scan).
  auto edgePierces = [&](const tessellate::Mesh& src, const tessellate::Mesh& dst) {
    for (const tessellate::Triangle& s : src.triangles) {
      const math::Point3 sv[3] = {src.vertices[s.a], src.vertices[s.b], src.vertices[s.c]};
      for (int ei = 0; ei < 3; ++ei) {
        const math::Point3& ep = sv[ei];
        const math::Point3& eq = sv[(ei + 1) % 3];
        for (const tessellate::Triangle& u : dst.triangles) {
          const std::optional<math::Point3> hit = idetail::segmentPiercesTriangleInterior(
              ep, eq, dst.vertices[u.a], dst.vertices[u.b], dst.vertices[u.c]);
          if (hit) {
            anyCross = true;
            idetail::growBox(*hit, wLo, wHi, wAny);
            if (!wPointSet) { wPoint = *hit; wPointSet = true; }
            return;  // one interior pierce is a conclusive CLASH; stop this direction
          }
        }
      }
    }
  };
  if (!anyInside && !anyCross) {
    edgePierces(a, b);
    if (!anyCross) edgePierces(b, a);
  }

  if (anyInside || anyCross) {
    r.state = ClashState::Clash;
    r.hasWitness = wAny;
    r.witnessLo = wLo; r.witnessHi = wHi; r.witnessPoint = wPoint;
    return r;  // overlapVolume filled by the engine's native COMMON
  }

  // No penetration signature, but an ambiguous ray with no crossing → honest decline.
  if (anyUnknown) { r.state = ClashState::Unknown; return r; }

  // ── (4) No overlap: min boundary distance → TOUCHING vs CLEAR ────────────────
  // The min boundary clearance is the minimum triangle–triangle distance. For two
  // disjoint convex triangles the true minimum is attained at either a VERTEX–FACE
  // pair OR an EDGE–EDGE pair, so the tri–tri distance is the min over BOTH the six
  // vertex-vs-face sub-tests AND the nine edge–edge sub-tests. The vertex-face
  // tests alone MISS the coplanar-overlap pose where two faces cross in a plus/cross
  // with no mutually contained vertex (a real flush TOUCH the closed-form edge–edge
  // term catches at distance 0). A per-pair AABB-gap prune with the running `best`
  // keeps it fast when the bodies are far apart (every B box is farther than `best`
  // ⇒ skipped).
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < a.triangles.size(); ++i) {
    const tessellate::Triangle& s = a.triangles[i];
    math::Point3 slo, shi; idetail::triBox(a, s, slo, shi);
    for (std::size_t j = 0; j < nb; ++j) {
      if (idetail::boxDisjoint(slo, shi, bLo[j], bHi[j], best)) continue;
      const tessellate::Triangle& u = b.triangles[j];
      const math::Point3& sa = a.vertices[s.a]; const math::Point3& sb = a.vertices[s.b];
      const math::Point3& sc = a.vertices[s.c];
      const math::Point3& ua = b.vertices[u.a]; const math::Point3& ub = b.vertices[u.b];
      const math::Point3& uc = b.vertices[u.c];
      double dd = std::numeric_limits<double>::infinity();
      // Six vertex-vs-face sub-tests (exact for the vertex-in-Voronoi-face cases).
      dd = std::min(dd, nbool::pointTriangleDistance(sa, ua, ub, uc));
      dd = std::min(dd, nbool::pointTriangleDistance(sb, ua, ub, uc));
      dd = std::min(dd, nbool::pointTriangleDistance(sc, ua, ub, uc));
      dd = std::min(dd, nbool::pointTriangleDistance(ua, sa, sb, sc));
      dd = std::min(dd, nbool::pointTriangleDistance(ub, sa, sb, sc));
      dd = std::min(dd, nbool::pointTriangleDistance(uc, sa, sb, sc));
      // Nine edge–edge sub-tests (the coplanar-cross / skew-edge minimum).
      dd = std::min(dd, idetail::triEdgeEdgeDistance(sa, sb, sc, ua, ub, uc));
      best = std::min(best, dd);
    }
  }

  r.minDistance = best;
  r.state = (best <= contactBand) ? ClashState::Touching : ClashState::Clear;
  return r;
}

}  // namespace cybercad::native::analysis

#endif  // CYBERCAD_NATIVE_ANALYSIS_INTERFERENCE_H
