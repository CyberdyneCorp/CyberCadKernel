// SPDX-License-Identifier: Apache-2.0
//
// freeform_membership.h — MOAT M2c / B3 (first slice): a native, OCCT-FREE
// POINT-IN-FREEFORM-SOLID classifier.
//
// ── WHAT THIS IS (and is NOT) ─────────────────────────────────────────────────
// `ssi_boolean.h` `classifyPoint(const CurvedSolid&, …)` answers point-in-solid
// ONLY for a solid whose curved wall folds to ONE elementary surface
// (cylinder / sphere / cone half-space + planar caps). The moment an operand
// carries a `Kind::BSpline`/`Kind::Bezier` face, `recogniseCurvedSolid` returns
// `nullopt` and there is no membership test — the B3 gap.
//
// This header is the SIBLING, mesh-driven path: given the WATERTIGHT boundary
// triangle `Mesh` the landed MOAT M0 tessellator
// (`tessellate::SolidMesher::mesh`, `face_mesher.h trimmedFreeformMesh`) produces
// for a genuinely-trimmed freeform-walled solid, it classifies an arbitrary query
// point IN / OUT / ON / UNKNOWN by ray-cast odd/even crossing parity, with a
// principled ON-boundary tolerance band. It is a PURE READ-ONLY CONSUMER of the
// mesh — the tessellator is neither modified nor re-implemented — and it touches
// neither `classifyPoint` nor `recogniseCurvedSolid`.
//
// ── HONESTY CONTRACT ──────────────────────────────────────────────────────────
// A meshed boundary is only within `meshDeflection` of the true surface, so a
// query point within the mesh's own fidelity of the boundary CANNOT be crisply
// classified: it resolves to `On` (min point-to-triangle distance inside the
// band) rather than a guessed IN/OUT. A ray that grazes a shared edge/vertex or
// runs nearly tangent to a face is UNUSABLE (discarded, never miscounted); if the
// usable rays fail to reach a quorum, or disagree, the verdict is `Unknown`
// (an honest decline) — NEVER a silent wrong crisp verdict. Robust ON-boundary /
// near-tangent classification is the asymptotic tail and is intentionally
// `On`/`Unknown` in this slice.
//
// ── SUBSTRATE ─────────────────────────────────────────────────────────────────
// OCCT-FREE. Depends only on `src/native/math` and `src/native/tessellate/mesh.h`
// (both OCCT-free). Header-only, `clang++ -std=c++20`. NO `cc_*` ABI change.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_FREEFORM_MEMBERSHIP_H
#define CYBERCAD_NATIVE_BOOLEAN_FREEFORM_MEMBERSHIP_H

#include "native/math/native_math.h"
#include "native/tessellate/mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

namespace cybercad::native::boolean {

namespace math = cybercad::native::math;
namespace tessellate = cybercad::native::tessellate;

// ─────────────────────────────────────────────────────────────────────────────
// Aabb — a minimal axis-aligned bounding box. The classifier needs only the box
// DIAGONAL (to scale the relative ON-band); `meshAabb` derives it from the mesh
// so callers that already have the solid's bbox can pass it, and tests can build
// it straight off the boundary mesh.
// ─────────────────────────────────────────────────────────────────────────────
struct Aabb {
  math::Point3 lo{0.0, 0.0, 0.0};
  math::Point3 hi{0.0, 0.0, 0.0};
  double diagonal() const noexcept { return math::norm(hi - lo); }
};

/// Tight AABB of a mesh's vertices. An empty mesh yields a degenerate zero box
/// (diagonal 0), which drives the ON-band to its absolute floor.
inline Aabb meshAabb(const tessellate::Mesh& m) noexcept {
  if (m.vertices.empty()) return Aabb{};
  math::Point3 lo = m.vertices.front(), hi = m.vertices.front();
  for (const math::Point3& v : m.vertices) {
    lo.x = std::min(lo.x, v.x); hi.x = std::max(hi.x, v.x);
    lo.y = std::min(lo.y, v.y); hi.y = std::max(hi.y, v.y);
    lo.z = std::min(lo.z, v.z); hi.z = std::max(hi.z, v.z);
  }
  return Aabb{lo, hi};
}

// ═════════════════════════════════════════════════════════════════════════════
// §1.1 — Möller–Trumbore ray/triangle intersection (isolated pure kernel).
//
// Returns the FORWARD hit (`t > eps`) of ray `orig + t·dir` against triangle
// (a,b,c), with barycentric weights so the caller can detect edge/vertex grazing.
// The reported point is `(1−u−v)·a + u·b + v·c`. `nullopt` when the ray is
// parallel to the triangle plane, misses the triangle, or hits behind the origin.
// ═════════════════════════════════════════════════════════════════════════════
struct RayTriHit {
  double t = 0.0;  ///< ray parameter (distance along the unit `dir`)
  double u = 0.0;  ///< barycentric weight of vertex b
  double v = 0.0;  ///< barycentric weight of vertex c
};

inline std::optional<RayTriHit> mollerTrumbore(const math::Point3& orig, const math::Vec3& dir,
                                               const math::Point3& a, const math::Point3& b,
                                               const math::Point3& c, double eps = 1e-12) noexcept {
  const math::Vec3 e1 = b - a, e2 = c - a;
  const math::Vec3 pv = math::cross(dir, e2);
  const double det = math::dot(e1, pv);
  if (std::fabs(det) < eps) return std::nullopt;  // ray parallel to the triangle plane
  const double inv = 1.0 / det;
  const math::Vec3 tv = orig - a;
  const double u = math::dot(tv, pv) * inv;
  if (u < 0.0 || u > 1.0) return std::nullopt;
  const math::Vec3 qv = math::cross(tv, e1);
  const double v = math::dot(dir, qv) * inv;
  if (v < 0.0 || u + v > 1.0) return std::nullopt;
  const double t = math::dot(e2, qv) * inv;
  if (t <= eps) return std::nullopt;  // behind or at the origin — not a forward crossing
  return RayTriHit{t, u, v};
}

// ═════════════════════════════════════════════════════════════════════════════
// §1.2 — Exact point-to-triangle distance (Ericson, RTCD §5.1.5) (isolated pure
// kernel). Returns the Euclidean distance from `p` to the CLOSEST point of the
// solid triangle (a,b,c), correctly handling the vertex / edge / face Voronoi
// regions. Used for the ON-band min-distance test.
// ═════════════════════════════════════════════════════════════════════════════
inline double pointTriangleDistance(const math::Point3& p, const math::Point3& a,
                                    const math::Point3& b, const math::Point3& c) noexcept {
  const math::Vec3 ab = b - a, ac = c - a, ap = p - a;
  const double d1 = math::dot(ab, ap), d2 = math::dot(ac, ap);
  if (d1 <= 0.0 && d2 <= 0.0) return math::norm(ap);  // vertex-a region

  const math::Vec3 bp = p - b;
  const double d3 = math::dot(ab, bp), d4 = math::dot(ac, bp);
  if (d3 >= 0.0 && d4 <= d3) return math::norm(bp);  // vertex-b region

  const math::Vec3 cp = p - c;
  const double d5 = math::dot(ab, cp), d6 = math::dot(ac, cp);
  if (d6 >= 0.0 && d5 <= d6) return math::norm(cp);  // vertex-c region

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {  // edge-ab region
    const double s = d1 / (d1 - d3);
    return math::norm(ap - ab * s);
  }
  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {  // edge-ac region
    const double s = d2 / (d2 - d6);
    return math::norm(ap - ac * s);
  }
  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {  // edge-bc region
    const double s = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return math::norm(bp - (c - b) * s);
  }
  // face region — barycentric projection onto the triangle plane.
  const double denom = 1.0 / (va + vb + vc);
  const double s = vb * denom, tt = vc * denom;
  return math::norm(ap - ab * s - ac * tt);
}

/// Minimum distance from `p` to any triangle of the mesh (empty mesh → +inf).
inline double minDistanceToMesh(const tessellate::Mesh& m, const math::Point3& p) noexcept {
  double best = std::numeric_limits<double>::infinity();
  for (const tessellate::Triangle& t : m.triangles)
    best = std::min(best, pointTriangleDistance(p, m.vertices[t.a], m.vertices[t.b], m.vertices[t.c]));
  return best;
}

// ═════════════════════════════════════════════════════════════════════════════
// §2 — the classifier.
// ═════════════════════════════════════════════════════════════════════════════

/// The four possible verdicts. `On` = within the mesh's fidelity of the boundary
/// (honest, not a guessed IN/OUT). `Unknown` = an honest decline (non-watertight
/// input, sub-quorum usable rays, or disagreeing rays) — never a silent wrong.
enum class Membership { In, Out, On, Unknown };

/// Tolerances / thresholds. Defaults are the empirically-validated values from the
/// M2c diagnosis. NONE of these WEAKEN a tolerance — the ON-band is the mesh's own
/// resolution, and the degeneracy thresholds only ever turn a risky ray into an
/// honest decline.
struct MembershipTol {
  double absTol = 1e-9;   ///< absolute ON-band floor (model units)
  double relTol = 1e-6;   ///< relative ON-band term (× bbox diagonal)
  /// The meshed convex wall is inset from the true surface by up to ~2×deflection
  /// (chord-secant offset: the M0 bump-cap fixture meshes ~2.7% under-volume at
  /// deflection 0.02). The wrong-side sliver between mesh and true surface is
  /// therefore within 2×deflection of the mesh; the band must cover it so no such
  /// point is ever crisply mis-sided. Design §D3's `+ meshDeflection` is the floor;
  /// 2× is the measured, conservative (more-honest-ON) value.
  double bandDeflectionFactor = 2.0;
  double edgeEps = 1e-4;      ///< barycentric proximity to an edge/vertex ⇒ ray unusable
  double grazingCos = 1e-3;   ///< |dir·n̂| below this ⇒ ray grazes the face ⇒ unusable
  int quorum = 3;             ///< min usable rays required for a crisp verdict
};

namespace ffdetail {

/// The result of casting ONE ray: whether it was usable (no grazing/edge hit) and,
/// if so, whether the odd/even forward-crossing parity puts `p` inside.
struct RayVote {
  bool usable = false;
  bool inside = false;
};

/// A FIXED set of non-axis-aligned, mutually non-parallel directions (no two are
/// scalar multiples; none lies in a coordinate plane). Deterministic — the
/// classifier is a pure function of its inputs (no RNG). Chosen so that for a
/// point comfortably away from the boundary every ray is clean and they agree
/// unanimously; the redundancy exists so an occasional grazing ray is discarded
/// (→ decline) rather than believed.
inline const std::array<math::Vec3, 7>& rayDirections() noexcept {
  static const std::array<math::Vec3, 7> kDirs = [] {
    const double raw[7][3] = {{1, 2, 3},  {2, -3, 1}, {-3, 1, 2}, {1, -2, 4},
                              {3, 4, -2}, {-2, 3, -4}, {4, -1, 2}};
    std::array<math::Vec3, 7> d{};
    for (int i = 0; i < 7; ++i) {
      math::Vec3 v{raw[i][0], raw[i][1], raw[i][2]};
      d[i] = v / math::norm(v);
    }
    return d;
  }();
  return kDirs;
}

/// Cast one ray and count forward Möller–Trumbore crossings. The ray is declared
/// UNUSABLE (and its parity discarded) if any crossing grazes a shared edge/vertex
/// (a barycentric weight within `edgeEps` of 0) or skims a face nearly tangentially
/// (|dir·n̂| < `grazingCos`) — either would double-count or drop a crossing and
/// corrupt the parity.
inline RayVote castRay(const tessellate::Mesh& m, const math::Point3& p, const math::Vec3& dir,
                       const MembershipTol& tol) noexcept {
  int crossings = 0;
  for (const tessellate::Triangle& t : m.triangles) {
    const math::Point3& a = m.vertices[t.a];
    const math::Point3& b = m.vertices[t.b];
    const math::Point3& c = m.vertices[t.c];
    const std::optional<RayTriHit> hit = mollerTrumbore(p, dir, a, b, c);
    if (!hit) continue;
    const double w = 1.0 - hit->u - hit->v;
    if (hit->u < tol.edgeEps || hit->v < tol.edgeEps || w < tol.edgeEps)
      return RayVote{false, false};  // grazed an edge/vertex → whole ray unusable
    const math::Vec3 n = math::cross(b - a, c - a);
    const double nlen = math::norm(n);
    if (nlen > 0.0 && std::fabs(math::dot(dir, n) / nlen) < tol.grazingCos)
      return RayVote{false, false};  // ray ~tangent to this face → whole ray unusable
    ++crossings;
  }
  return RayVote{true, (crossings & 1) != 0};
}

}  // namespace ffdetail

// ── The public classifier (design §Algorithm) ────────────────────────────────
// `boundary` MUST be a watertight, outward-CCW-wound M0 boundary mesh; `bbox` the
// solid's AABB (for relative-band scaling); `meshDeflection` the deflection the
// mesh was built at. A non-watertight mesh is out of scope → `Unknown` (a decline,
// not a fabricated verdict).
inline Membership classifyPointInMesh(const tessellate::Mesh& boundary, const Aabb& bbox,
                                      double meshDeflection, const math::Point3& p,
                                      const MembershipTol& tol = {}) {
  // Precondition: watertight input (task 2.4). Ray parity is undefined otherwise.
  if (!tessellate::isWatertight(boundary)) return Membership::Unknown;

  // ON-band: within the mesh's own fidelity of the boundary ⇒ honest ON.
  const double band =
      std::max(tol.absTol, tol.relTol * bbox.diagonal()) + tol.bandDeflectionFactor * meshDeflection;
  if (minDistanceToMesh(boundary, p) <= band) return Membership::On;

  // Multi-ray odd/even consensus over the fixed direction set.
  int usable = 0, insideVotes = 0;
  for (const math::Vec3& dir : ffdetail::rayDirections()) {
    const ffdetail::RayVote r = ffdetail::castRay(boundary, p, dir, tol);
    if (!r.usable) continue;  // grazing ray discarded — not miscounted
    ++usable;
    if (r.inside) ++insideVotes;
  }
  if (usable < tol.quorum) return Membership::Unknown;  // too few clean rays → decline
  if (insideVotes == 0) return Membership::Out;         // unanimous outside
  if (insideVotes == usable) return Membership::In;      // unanimous inside
  return Membership::Unknown;                            // rays disagree → honest decline
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_FREEFORM_MEMBERSHIP_H
