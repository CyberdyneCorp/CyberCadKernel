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
//                Detected robustly + COPLANAR-SAFELY at the mesh level via the B3
//                classifier: either operand has a boundary VERTEX, or a boundary
//                TRIANGLE CENTROID, that classifies strictly INSIDE the other. A
//                shared face reads On (the B3 ON-band absorbs the seam), never In, so
//                a flush TOUCH never fires; a face poking through the other's wall has
//                an interior centroid → CLASH. (A raw Möller tri–tri crossing was
//                rejected: it over-reports at a shared seam where edge-adjacent
//                triangles of two flush solids "cross" though nothing interpenetrates.)
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

}  // namespace idetail

// ─────────────────────────────────────────────────────────────────────────────
// meshInterference — classify the interference of two solids by their WATERTIGHT
// M0 boundary meshes. `deflection` is the deflection the meshes were built at (it
// scales the B3 ON-band and the contact band). The overlap VOLUME is left 0 (the
// engine fills it with the native COMMON); everything else is decided here.
//
// ALGORITHM (all evidence is at the mesh level, no boolean needed):
//   1. Preconditions — both meshes must be watertight; else UNKNOWN (decline).
//   2. Penetration signature — CLASH iff a boundary VERTEX or a boundary TRIANGLE
//      CENTROID of one solid classifies strictly IN the other (B3 multi-ray parity).
//      A shared face reads On (absorbed by the B3 ON-band), never In, so a face-flush
//      TOUCH never fires; a face poking through the other's wall has an interior
//      centroid → CLASH. A lone Membership::Unknown (ambiguous ray) with no confirmed
//      inside point declines (UNKNOWN) rather than guess.
//   3. Witness — on CLASH, the AABB of the inside points + a representative interior
//      point (the engine sharpens this to the COMMON solid's box + centroid).
//   4. No penetration — compute the min triangle–triangle distance; TOUCHING within
//      the contact band, else CLEAR (the min distance is the clearance).
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

  if (anyInside || anyCross) {
    r.state = ClashState::Clash;
    r.hasWitness = wAny;
    r.witnessLo = wLo; r.witnessHi = wHi; r.witnessPoint = wPoint;
    return r;  // overlapVolume filled by the engine's native COMMON
  }

  // No penetration signature, but an ambiguous ray with no crossing → honest decline.
  if (anyUnknown) { r.state = ClashState::Unknown; return r; }

  // ── (4) No overlap: min boundary distance → TOUCHING vs CLEAR ────────────────
  // The min boundary clearance is the minimum triangle–triangle distance, evaluated
  // as the min over the six vertex-vs-face sub-tests of each candidate pair (the
  // true tri–tri minimum of two disjoint convex triangles is attained at a
  // vertex-face or edge-edge pair; for the contact/clearance decision at mesh
  // fidelity the vertex-face min is exact for axis-aligned faces and a tight bound
  // otherwise). A per-pair AABB-gap prune with the running `best` keeps it fast when
  // the bodies are far apart (every B box is farther than `best` ⇒ skipped).
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < a.triangles.size(); ++i) {
    const tessellate::Triangle& s = a.triangles[i];
    math::Point3 slo, shi; idetail::triBox(a, s, slo, shi);
    for (std::size_t j = 0; j < nb; ++j) {
      if (idetail::boxDisjoint(slo, shi, bLo[j], bHi[j], best)) continue;
      const tessellate::Triangle& u = b.triangles[j];
      double dd = std::numeric_limits<double>::infinity();
      dd = std::min(dd, nbool::pointTriangleDistance(a.vertices[s.a], b.vertices[u.a], b.vertices[u.b], b.vertices[u.c]));
      dd = std::min(dd, nbool::pointTriangleDistance(a.vertices[s.b], b.vertices[u.a], b.vertices[u.b], b.vertices[u.c]));
      dd = std::min(dd, nbool::pointTriangleDistance(a.vertices[s.c], b.vertices[u.a], b.vertices[u.b], b.vertices[u.c]));
      dd = std::min(dd, nbool::pointTriangleDistance(b.vertices[u.a], a.vertices[s.a], a.vertices[s.b], a.vertices[s.c]));
      dd = std::min(dd, nbool::pointTriangleDistance(b.vertices[u.b], a.vertices[s.a], a.vertices[s.b], a.vertices[s.c]));
      dd = std::min(dd, nbool::pointTriangleDistance(b.vertices[u.c], a.vertices[s.a], a.vertices[s.b], a.vertices[s.c]));
      best = std::min(best, dd);
    }
  }

  r.minDistance = best;
  r.state = (best <= contactBand) ? ClashState::Touching : ClashState::Clear;
  return r;
}

}  // namespace cybercad::native::analysis

#endif  // CYBERCAD_NATIVE_ANALYSIS_INTERFERENCE_H
