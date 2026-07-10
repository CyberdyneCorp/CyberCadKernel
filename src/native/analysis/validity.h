// SPDX-License-Identifier: Apache-2.0
//
// validity.h — GS6 B-rep VALIDITY checker: an OCCT-FREE structural-validity report
// over a solid's M0 triangulation (the native tessellator's boundary mesh), the
// mesh-level analogue of BRepCheck_Analyzer::IsValid.
//
// The report answers, per check, the concrete failure modes a valid solid must
// clear:
//   * finite         — every vertex coordinate is finite (no NaN / Inf).
//   * closed         — closed 2-manifold: every undirected edge is shared by
//                      EXACTLY two triangles (tessellate::isWatertight). Rejects a
//                      hole / open shell (boundary edge) and a non-manifold edge.
//   * oriented       — consistent outward orientation: every DIRECTED half-edge
//                      (a→b) occurs at most once, so the two triangles on a shared
//                      edge traverse it in OPPOSITE directions. A single flipped
//                      face makes one half-edge appear twice → caught even though
//                      watertightness (undirected count) is preserved.
//   * nondegenerate  — no zero-area triangle and no zero-length edge.
//   * noSelfIntersection — no two NON-ADJACENT triangles cross, via Möller
//                      triangle–triangle intersection with an AABB broad-phase
//                      reject, a PARALLEL-PLANE guard (the naïve crossing predicate
//                      gives false positives on coplanar-parallel disjoint faces, so
//                      parallel pairs are excluded from the crossing predicate and
//                      coplanar OVERLAP is reported as out-of-scope, not as a false
//                      "valid"), and a STRADDLE gate: a transversal crossing is
//                      accepted only when EACH triangle pierces to BOTH sides of the
//                      other's plane. This rejects the tangent-seam false positive
//                      where two facets of the SAME tessellated surface (e.g. a
//                      cylinder-bend strip meeting the coarse base face or a wedge
//                      end-cap at a T-junction — coincident along a boundary but with
//                      no shared vertex index) merely touch along a common line; a
//                      genuine facet that passes THROUGH another still straddles and
//                      is still caught (GS6).
//
// SELF-VERIFY / HONEST DECLINE: `selfIntersectionCertified` is false when a
// coplanar-overlap pair is reachable that the transversal predicate cannot decide
// — the checker declares that out of scope rather than assert a false clean
// verdict; `valid()` then does not certify. OCCT is the ORACLE only (the sim gate
// BRepCheck_Analyzer::IsValid); no OCCT header is included here. Header-only,
// math-only (reuses the same vec.h segment/tri primitives as GS3 distance).
//
#ifndef CYBERCAD_NATIVE_ANALYSIS_VALIDITY_H
#define CYBERCAD_NATIVE_ANALYSIS_VALIDITY_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "native/math/vec.h"
#include "native/tessellate/mesh.h"

namespace cybercad::native::analysis {

namespace math = cybercad::native::math;
namespace tessellate = cybercad::native::tessellate;

/// Per-check validity verdict over a boundary mesh. Every field is an independent
/// necessary condition; `valid()` is their conjunction (and requires the
/// self-intersection test to have CERTIFIED, not merely "found nothing").
struct ValidityReport {
  bool finite = false;
  bool closed = false;
  bool oriented = false;
  bool nondegenerate = false;
  bool noSelfIntersection = false;
  bool selfIntersectionCertified = true;  ///< false ⇒ coplanar-overlap out of scope

  bool valid() const {
    return finite && closed && oriented && nondegenerate && noSelfIntersection &&
           selfIntersectionCertified;
  }

  /// First failing check as a short reason (empty when valid()).
  const char* reason() const {
    if (!finite) return "non-finite coordinate";
    if (!closed) return "not a closed 2-manifold (open/non-manifold edge)";
    if (!oriented) return "inconsistent face orientation";
    if (!nondegenerate) return "degenerate (zero-area face or zero-length edge)";
    if (!selfIntersectionCertified) return "self-intersection undecidable (coplanar overlap)";
    if (!noSelfIntersection) return "self-intersecting faces";
    return "";
  }
};

namespace detail {

// ── Triangle–triangle intersection (Möller) with a parallel-plane guard ───────
enum class TriTri { Separate, Cross, Coplanar };

/// Interval of the two triangle vertices on the opposite plane's intersection
/// line, given the signed distances (dp0,dp1,dp2) and the projections (pp0,..).
inline void isectInterval(double vp0, double vp1, double vp2, double d0, double d1,
                          double d2, double d0d1, double d0d2, double& a, double& b) {
  // The odd-one-out vertex (opposite sign) bridges to the other two.
  double A0 = vp0, B0 = vp1, C0 = vp2, X0 = d0, X1 = d1, X2 = d2;
  if (d0d1 > 0.0) { A0 = vp2; B0 = vp0; C0 = vp1; X0 = d2; X1 = d0; X2 = d1; }
  else if (d0d2 > 0.0) { A0 = vp1; B0 = vp0; C0 = vp2; X0 = d1; X1 = d0; X2 = d2; }
  a = B0 + (A0 - B0) * X1 / (X1 - X0);
  b = C0 + (A0 - C0) * X2 / (X2 - X0);
}

/// Positive-area overlap test for two COPLANAR triangles, decided in their shared
/// plane via the 2D separating-axis theorem (SAT) over the six edge normals. Returns
/// true only when the triangles overlap with area beyond `tol` — a bare edge/vertex
/// touch (adjacent same-plane facets of one tessellated surface) separates on some
/// axis and reads as NO overlap. This makes the coplanar case DECIDABLE for the
/// common, benign sub-case (coplanar-disjoint neighbours), reserving the honest
/// decline for a genuine coplanar OVERLAP the transversal predicate cannot resolve.
inline bool coplanarOverlap(const math::Vec3& v0, const math::Vec3& v1, const math::Vec3& v2,
                            const math::Vec3& u0, const math::Vec3& u1, const math::Vec3& u2,
                            double tol) {
  math::Vec3 nrm = math::cross(v1 - v0, v2 - v0);
  const double nlen = math::norm(nrm);
  if (nlen < 1e-30) return false;  // degenerate: no plane to project into
  nrm = {nrm.x / nlen, nrm.y / nlen, nrm.z / nlen};
  math::Vec3 e1 = v1 - v0;
  const double e1len = math::norm(e1);
  if (e1len < 1e-30) return false;
  e1 = {e1.x / e1len, e1.y / e1len, e1.z / e1len};
  const math::Vec3 e2 = math::cross(nrm, e1);  // in-plane basis (e1,e2)
  const math::Vec3 tv[3] = {v0, v1, v2}, tu[3] = {u0, u1, u2};
  double px[3], py[3], qx[3], qy[3];
  for (int i = 0; i < 3; ++i) {
    px[i] = math::dot(tv[i], e1); py[i] = math::dot(tv[i], e2);
    qx[i] = math::dot(tu[i], e1); qy[i] = math::dot(tu[i], e2);
  }
  // SAT: separated on any triangle-edge normal ⇒ no positive-area overlap.
  auto separated = [&](const double* ax, const double* ay) {
    for (int i = 0; i < 3; ++i) {
      const double ex = ax[(i + 1) % 3] - ax[i], ey = ay[(i + 1) % 3] - ay[i];
      double nx = -ey, ny = ex;  // edge normal
      const double nl = std::sqrt(nx * nx + ny * ny);
      if (nl < 1e-30) continue;
      nx /= nl; ny /= nl;
      double pmin = 1e300, pmax = -1e300, qmin = 1e300, qmax = -1e300;
      for (int k = 0; k < 3; ++k) {
        const double dp = px[k] * nx + py[k] * ny, dq = qx[k] * nx + qy[k] * ny;
        pmin = std::min(pmin, dp); pmax = std::max(pmax, dp);
        qmin = std::min(qmin, dq); qmax = std::max(qmax, dq);
      }
      if (pmax < qmin + tol || qmax < pmin + tol) return true;  // gap on this axis
    }
    return false;
  };
  return !separated(px, py) && !separated(qx, qy);
}

/// Möller triangle–triangle test. Returns Cross (transversal intersection),
/// Separate, or Coplanar (a genuine coplanar OVERLAP the caller declines as out of
/// scope — a coplanar-DISJOINT neighbour pair is resolved to Separate here, never a
/// spurious decline). `eps` scales the parallel and on-plane thresholds.
inline TriTri triTri(const math::Vec3& v0, const math::Vec3& v1, const math::Vec3& v2,
                     const math::Vec3& u0, const math::Vec3& u1, const math::Vec3& u2,
                     double eps) {
  const math::Vec3 n2 = math::cross(u1 - u0, u2 - u0);
  const double d2 = -math::dot(n2, u0);
  double dv0 = math::dot(n2, v0) + d2, dv1 = math::dot(n2, v1) + d2, dv2 = math::dot(n2, v2) + d2;
  if (std::fabs(dv0) < eps) dv0 = 0.0;
  if (std::fabs(dv1) < eps) dv1 = 0.0;
  if (std::fabs(dv2) < eps) dv2 = 0.0;
  const double dv0dv1 = dv0 * dv1, dv0dv2 = dv0 * dv2;
  if (dv0dv1 > 0.0 && dv0dv2 > 0.0) return TriTri::Separate;  // tri1 entirely one side

  const math::Vec3 n1 = math::cross(v1 - v0, v2 - v0);
  const double d1 = -math::dot(n1, v0);
  double du0 = math::dot(n1, u0) + d1, du1 = math::dot(n1, u1) + d1, du2 = math::dot(n1, u2) + d1;
  if (std::fabs(du0) < eps) du0 = 0.0;
  if (std::fabs(du1) < eps) du1 = 0.0;
  if (std::fabs(du2) < eps) du2 = 0.0;
  const double du0du1 = du0 * du1, du0du2 = du0 * du2;
  if (du0du1 > 0.0 && du0du2 > 0.0) return TriTri::Separate;

  // PARALLEL-PLANE guard: the crossing line is D = n1×n2; if the planes are
  // parallel the transversal predicate is out of scope. Either the triangles are
  // coplanar (all distances ~0 above ⇒ Coplanar) or they are on disjoint parallel
  // planes (a same-sign distance triple would have returned Separate already).
  const math::Vec3 D = math::cross(n1, n2);
  const double dmax = math::normSquared(D);
  if (dmax < eps * eps) {
    // Parallel planes. All of tri1's distances to tri2's plane are ~0 here (else the
    // Separate early-outs fired), so the pair is coplanar. Decide the benign case:
    // coplanar-DISJOINT neighbours (adjacent same-plane facets of one tessellated
    // surface) do NOT overlap and are Separate; only a true coplanar OVERLAP is left
    // as the honest-decline Coplanar the transversal predicate cannot resolve.
    // Overlap tolerance scaled to the local geometry (a bare seam touch of this size
    // reads as no overlap): a fraction of the mean edge length of the two triangles.
    const double edgeScale =
        (math::norm(v1 - v0) + math::norm(v2 - v1) + math::norm(v0 - v2) +
         math::norm(u1 - u0) + math::norm(u2 - u1) + math::norm(u0 - u2)) /
        6.0;
    const double otol = 1e-7 * (edgeScale + 1.0);
    if (coplanarOverlap(v0, v1, v2, u0, u1, u2, otol)) return TriTri::Coplanar;
    return TriTri::Separate;
  }

  // Project onto the largest component of D and build both intervals on the line.
  int idx = 0;
  double m = std::fabs(D.x);
  if (std::fabs(D.y) > m) { m = std::fabs(D.y); idx = 1; }
  if (std::fabs(D.z) > m) { idx = 2; }
  const double vp0 = v0[idx], vp1 = v1[idx], vp2 = v2[idx];
  const double up0 = u0[idx], up1 = u1[idx], up2 = u2[idx];
  double a1, b1, a2, b2;
  isectInterval(vp0, vp1, vp2, dv0, dv1, dv2, dv0dv1, dv0dv2, a1, b1);
  isectInterval(up0, up1, up2, du0, du1, du2, du0du1, du0du2, a2, b2);
  if (a1 > b1) std::swap(a1, b1);
  if (a2 > b2) std::swap(a2, b2);
  // Overlap of the two on-line intervals ⇒ the triangles cross. A bare touch
  // (shared endpoint) is excluded with a small margin so edge-adjacent faces that
  // slip the shared-vertex filter do not read as a crossing.
  const double tol = 1e-9 * (1.0 + std::fabs(a1) + std::fabs(b1) + std::fabs(a2) + std::fabs(b2));
  if (b1 < a2 + tol || b2 < a1 + tol) return TriTri::Separate;

  // STRADDLE gate — the transversal/two-sidedness invariant. A genuine transversal
  // crossing requires EACH triangle to pierce through the OTHER's plane: it must
  // have a vertex STRICTLY on the positive side AND one STRICTLY on the negative
  // side. The signed distances above are already clamped to exactly 0 within the
  // on-plane band `eps`, so a vertex that merely LIES ON the other plane counts as
  // neither side. Without this gate, two facets of the SAME tessellated surface that
  // meet TANGENTLY along a common seam line (a T-junction where a coarse face abuts
  // fine bend strips, or a wedge end-cap abuts its strips — no shared vertex index,
  // yet coincident along a boundary) yield overlapping on-line intervals and read as
  // a spurious Cross. Requiring true two-sided straddle excludes such seam touches
  // while a real self-intersection — where a facet passes THROUGH another to its far
  // side — still straddles and is still caught (GS6).
  const bool vStraddles = (dv0 > 0.0 || dv1 > 0.0 || dv2 > 0.0) &&
                          (dv0 < 0.0 || dv1 < 0.0 || dv2 < 0.0);
  const bool uStraddles = (du0 > 0.0 || du1 > 0.0 || du2 > 0.0) &&
                          (du0 < 0.0 || du1 < 0.0 || du2 < 0.0);
  if (!vStraddles || !uStraddles) return TriTri::Separate;  // tangent seam touch, not a crossing
  return TriTri::Cross;
}

/// Axis-aligned bounding box of a triangle (broad-phase reject key).
struct Aabb {
  math::Vec3 lo, hi;
};
inline Aabb triAabb(const math::Vec3& a, const math::Vec3& b, const math::Vec3& c) {
  Aabb r;
  r.lo = {std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::min({a.z, b.z, c.z})};
  r.hi = {std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::max({a.z, b.z, c.z})};
  return r;
}
inline bool aabbDisjoint(const Aabb& p, const Aabb& q, double margin) {
  return p.hi.x + margin < q.lo.x || q.hi.x + margin < p.lo.x ||
         p.hi.y + margin < q.lo.y || q.hi.y + margin < p.lo.y ||
         p.hi.z + margin < q.lo.z || q.hi.z + margin < p.lo.z;
}

/// True if two triangles share any vertex index (edge/corner-adjacent — a
/// legitimate touch, excluded from the self-intersection scan).
inline bool shareVertex(const tessellate::Triangle& s, const tessellate::Triangle& t) {
  return s.a == t.a || s.a == t.b || s.a == t.c || s.b == t.a || s.b == t.b ||
         s.b == t.c || s.c == t.a || s.c == t.b || s.c == t.c;
}

}  // namespace detail

/// Build the validity report for the boundary `mesh`. Pure structural checks over
/// the M0 triangulation; never throws, never fabricates a verdict (coplanar-
/// overlap ambiguity flips `selfIntersectionCertified` off instead of asserting
/// clean). The characteristic scale (from the vertex AABB) sets the geometric
/// tolerances so the checks are scale-aware.
inline ValidityReport checkSolidMesh(const tessellate::Mesh& mesh) {
  ValidityReport r;

  // ── finite coordinates + characteristic scale ──────────────────────────────
  r.finite = true;
  math::Vec3 lo{0, 0, 0}, hi{0, 0, 0};
  bool first = true;
  for (const auto& p : mesh.vertices) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) { r.finite = false; break; }
    const math::Vec3 v = p.asVec();
    if (first) { lo = hi = v; first = false; continue; }
    lo = {std::min(lo.x, v.x), std::min(lo.y, v.y), std::min(lo.z, v.z)};
    hi = {std::max(hi.x, v.x), std::max(hi.y, v.y), std::max(hi.z, v.z)};
  }
  if (!r.finite) return r;  // downstream geometry is meaningless with NaN/Inf
  const double scale = std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z, 1e-12});
  const double lenEps = 1e-9 * scale;              // zero-length edge threshold
  const double areaEps = 1e-12 * scale * scale;    // zero-area (2·area) threshold

  // ── closed (undirected) + oriented (directed) via one half-edge pass ────────
  std::unordered_map<std::uint64_t, int> directed;
  directed.reserve(mesh.triangles.size() * 3);
  auto key = [](std::uint32_t u, std::uint32_t v) {
    return (static_cast<std::uint64_t>(u) << 32) | static_cast<std::uint64_t>(v);
  };
  r.oriented = true;
  auto add = [&](std::uint32_t u, std::uint32_t v) {
    if (++directed[key(u, v)] > 1) r.oriented = false;  // half-edge repeated ⇒ flip
  };
  for (const tessellate::Triangle& t : mesh.triangles) {
    add(t.a, t.b); add(t.b, t.c); add(t.c, t.a);
  }
  r.closed = tessellate::isWatertight(mesh);

  // ── nondegenerate: no zero-length edge, no zero-area triangle ───────────────
  r.nondegenerate = !mesh.triangles.empty();
  for (const tessellate::Triangle& t : mesh.triangles) {
    const math::Vec3 a = mesh.vertices[t.a].asVec();
    const math::Vec3 b = mesh.vertices[t.b].asVec();
    const math::Vec3 c = mesh.vertices[t.c].asVec();
    if (math::norm(b - a) < lenEps || math::norm(c - b) < lenEps ||
        math::norm(a - c) < lenEps) { r.nondegenerate = false; break; }
    if (math::norm(math::cross(b - a, c - a)) < areaEps) { r.nondegenerate = false; break; }
  }

  // ── self-intersection: AABB broad-phase + Möller with the parallel guard ────
  const std::size_t n = mesh.triangles.size();
  std::vector<detail::Aabb> boxes(n);
  for (std::size_t i = 0; i < n; ++i) {
    const tessellate::Triangle& t = mesh.triangles[i];
    boxes[i] = detail::triAabb(mesh.vertices[t.a].asVec(), mesh.vertices[t.b].asVec(),
                               mesh.vertices[t.c].asVec());
  }
  const double planeEps = 1e-10 * scale;
  const double margin = 1e-9 * scale;
  r.noSelfIntersection = true;
  for (std::size_t i = 0; i < n && r.noSelfIntersection && r.selfIntersectionCertified; ++i) {
    const tessellate::Triangle& s = mesh.triangles[i];
    const math::Vec3 sa = mesh.vertices[s.a].asVec(), sb = mesh.vertices[s.b].asVec(),
                     sc = mesh.vertices[s.c].asVec();
    for (std::size_t j = i + 1; j < n; ++j) {
      if (detail::shareVertex(s, mesh.triangles[j])) continue;   // adjacent — legit touch
      if (detail::aabbDisjoint(boxes[i], boxes[j], margin)) continue;  // broad-phase reject
      const tessellate::Triangle& u = mesh.triangles[j];
      const detail::TriTri res =
          detail::triTri(sa, sb, sc, mesh.vertices[u.a].asVec(), mesh.vertices[u.b].asVec(),
                         mesh.vertices[u.c].asVec(), planeEps);
      if (res == detail::TriTri::Cross) { r.noSelfIntersection = false; break; }
      if (res == detail::TriTri::Coplanar) { r.selfIntersectionCertified = false; break; }
    }
  }
  return r;
}

}  // namespace cybercad::native::analysis

#endif  // CYBERCAD_NATIVE_ANALYSIS_VALIDITY_H
