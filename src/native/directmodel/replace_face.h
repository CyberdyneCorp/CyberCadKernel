// SPDX-License-Identifier: Apache-2.0
//
// replace_face.h — MOAT M-DM DM2: the native `replace_face_to_plane` (the app's
// push/pull "move a face to a target plane"). Pick ONE planar face `F` of a
// convex planar polyhedron, hand it a target plane `P_t = (tp, n̂)`, and RE-SOLVE
// the adjacent faces so the solid stays watertight — extend/trim each neighbour to
// meet the moved face's new plane.
//
// It re-derives NO geometry: the whole verb is ASSEMBLED from the two already-
// landed, already-gated DM1/boolean verbs plus construct + the mesh audit:
//
//   * PARALLEL TRIM  (n̂ ∥ n̂_F, d̄ < 0)         → one DM1 `splitByPlane` (keep bulk).
//   * PARALLEL GROW  (n̂ ∥ n̂_F, d̄ > 0)         → `boolean_solid(operand, slab, Fuse)`
//                                                 with slab = the picked face loop
//                                                 (`build_prism`) extruded n̂_F·d.
//   * GENERAL RE-SOLVE (tilted / mixed grow-trim) → GROW-THEN-TRIM: fuse a parallel
//     slab that overshoots the target reach, then ONE tilted `splitByPlane` back to
//     the target plane. NOT an N-cut half-space chain — that breaks the watertight
//     self-verify at ~4 cuts (DIAGNOSE finding). One Fuse + one BSP Cut, each
//     individually watertight-gated, so it inherits the DM1 robustness. A pure-trim
//     tilt (the moved face never travels outward) skips the grow (single cut).
//
// ── SELF-VERIFY → HONEST DECLINE (never a leaky / wrong / multi-lump solid) ───────
// The candidate is accepted ONLY when ALL hold (mesh audit, planar meshes exact):
// watertight closed 2-manifold; positive enclosed volume; single lump (Euler χ = 2);
// the moved face reached P_t and nothing pokes past it; the distinct-supporting-plane
// count is preserved (no neighbour removed/added/inverted); and the enclosed volume
// matches the CLOSED-FORM oracle `V' = V₀ + A_F·d̄`. Anything outside the convex-
// planar slice — a curved neighbour (the solid is not all-planar), a non-planar
// picked face, a degenerate / topology-changing target, a non-convex operand —
// returns a NULL Shape with a measured `ReplaceFaceDecline`. The engine then reports
// the honest decline and falls to OCCT; it NEVER hands a native void to OCCT and
// NEVER emits an unverified solid.
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// `splitByPlane` (boolean/split_plane.h), `boolean_solid` (boolean/native_boolean.h),
// `build_prism` (construct/native_construct.h), `PlanarModel` / `facePlane`
// (blend/blend_geom.h), the tessellate mesh audit. Additive sibling — touches none.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_DIRECTMODEL_REPLACE_FACE_H
#define CYBERCAD_NATIVE_DIRECTMODEL_REPLACE_FACE_H

#include "native/blend/blend_geom.h"
#include "native/boolean/native_boolean.h"
#include "native/boolean/split_plane.h"
#include "native/construct/native_construct.h"
#include "native/math/elementary.h"
#include "native/math/native_math.h"
#include "native/math/transform.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <vector>

namespace cybercad::native::directmodel {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace nb   = cybercad::native::boolean;
namespace cst  = cybercad::native::construct;
namespace bl   = cybercad::native::blend;

// Measured reason a native re-solve declined (→ engine reports it, falls to OCCT).
enum class ReplaceFaceDecline {
  Ok = 0,
  NonPlanarOrForeign,  // solid not all-planar (curved neighbour) / mesh-only body /
                       // the picked face is not a planar polygon we can identify
  DegenerateNormal,    // target normal ~ 0
  DegenerateTarget,    // n̂·n̂_F too small (plane parallel to the sweep) or a no-op
  BuildFailed,         // a consumed verb (slab / fuse / cut) returned NULL
  SelfVerifyFailed,    // watertight / volume / lump / plane-count / on-plane gate failed
};

namespace rfdetail {

// Numeric bands. Planar polyhedra mesh EXACTLY, so these are tight — never widened.
inline constexpr double kClassTol   = 1e-6;   // grow/trim/no-op classification
inline constexpr double kParallelTol = 1e-9;  // |n̂·n̂_F − 1| ≤ this ⇒ n̂ ∥ n̂_F
inline constexpr double kMinDenom   = 5e-2;   // reject an extreme tilt (n̂ ⟂-ish n̂_F)
inline constexpr double kOnPlaneTol = 1e-5;   // moved-face-on-P_t / poke-past band
inline constexpr double kPlaneQuant = 1e-6;   // distinct-plane key quantum

// A reference X direction guaranteed not parallel to `z` (for a Gram–Schmidt frame).
inline math::Dir3 refNotParallel(const math::Dir3& z) {
  return std::fabs(z.x()) < 0.9 ? math::Dir3{math::Vec3{1, 0, 0}}
                                : math::Dir3{math::Vec3{0, 1, 0}};
}

// The affine map local (u,v,w) → world = origin + u·x + v·y + w·z of a plane frame.
inline math::Transform frameToWorld(const math::Ax3& fr) {
  const math::Mat3 L(fr.x.x(), fr.y.x(), fr.z.x(),
                     fr.x.y(), fr.y.y(), fr.z.y(),
                     fr.x.z(), fr.y.z(), fr.z.z());
  return math::Transform{L, fr.origin.asVec()};
}

// The picked planar face, flattened: its outward plane, its world boundary loop, a
// frame with z = n̂_F, the loop projected to that frame's (u,v), the true area and
// the true area-centroid (world). δ is affine over F, so d̄ = δ(centroid).
struct PickedFace {
  nb::Plane plane;                 // outward n̂_F + offset w
  math::Ax3 frame;                 // z = n̂_F, origin on the face
  std::vector<double> profile;     // (u,v) pairs of the loop in the frame
  std::vector<math::Point3> loop;  // world boundary loop
  math::Point3 centroid;           // true area-centroid (world)
  double area = 0.0;
  double faceDiag = 0.0;           // in-plane bbox diagonal (for the grow margin)
};

// The picked-face polygon in the model: normal agrees with `plane` and every vertex
// lies ON it (mirrors blend/offset_face's pickFacePolygon). npos if none matches.
inline std::size_t pickFacePolygon(const std::vector<nb::Polygon>& polys,
                                   const nb::Plane& plane) {
  for (std::size_t i = 0; i < polys.size(); ++i) {
    const nb::Polygon& p = polys[i];
    if (math::dot(p.plane.normal, plane.normal) < 0.999) continue;  // normal must agree
    bool allOn = true;
    for (const math::Point3& v : p.vertices)
      if (std::fabs(bl::signedDist(plane, v)) > 1e-5) { allOn = false; break; }
    if (allOn) return i;
  }
  return static_cast<std::size_t>(-1);
}

// Read + flatten the picked face. nullopt if the solid is not all-planar (a curved
// neighbour), the picked face is non-planar, or the loop is degenerate.
inline std::optional<PickedFace> readPickedFace(const topo::Shape& solid, int faceId) {
  bl::PlanarModel model(solid);
  if (!model.isValid()) return std::nullopt;          // curved neighbour / mesh body
  const auto planeOpt = bl::facePlane(solid, faceId);
  if (!planeOpt) return std::nullopt;                 // non-planar picked face
  const std::size_t pick = pickFacePolygon(model.polygons(), *planeOpt);
  if (pick == static_cast<std::size_t>(-1)) return std::nullopt;

  PickedFace pf;
  pf.plane = *planeOpt;
  pf.loop = model.polygons()[pick].vertices;
  if (pf.loop.size() < 3) return std::nullopt;

  const math::Dir3 z{pf.plane.normal};
  pf.frame = math::Ax3::fromAxisAndRef(pf.loop.front(), z, refNotParallel(z));

  // Project the loop to (u,v); accumulate the shoelace area + area-centroid in 2D.
  const std::size_t n = pf.loop.size();
  pf.profile.reserve(2 * n);
  std::vector<double> uu(n), vv(n);
  double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const math::Vec3 d = pf.loop[i] - pf.frame.origin;
    uu[i] = math::dot(d, pf.frame.x.vec());
    vv[i] = math::dot(d, pf.frame.y.vec());
    pf.profile.push_back(uu[i]);
    pf.profile.push_back(vv[i]);
    if (i == 0) { uMin = uMax = uu[0]; vMin = vMax = vv[0]; }
    uMin = std::min(uMin, uu[i]); uMax = std::max(uMax, uu[i]);
    vMin = std::min(vMin, vv[i]); vMax = std::max(vMax, vv[i]);
  }
  double a2 = 0.0, cu = 0.0, cv = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    const double cross = uu[i] * vv[j] - uu[j] * vv[i];
    a2 += cross;
    cu += (uu[i] + uu[j]) * cross;
    cv += (vv[i] + vv[j]) * cross;
  }
  if (std::fabs(a2) < 1e-12) return std::nullopt;     // degenerate loop
  pf.area = std::fabs(a2) * 0.5;
  const double cU = cu / (3.0 * a2), cV = cv / (3.0 * a2);
  pf.centroid = pf.frame.origin + pf.frame.x.vec() * cU + pf.frame.y.vec() * cV;
  pf.faceDiag = std::hypot(uMax - uMin, vMax - vMin);
  return pf;
}

// A parallel slab: the picked-face loop (`profile`) extruded along +n̂_F by `depth`,
// placed so its base lies exactly on the picked face. NULL if build_prism declines.
inline topo::Shape buildSlab(const PickedFace& pf, double depth) {
  const topo::Shape prism =
      cst::build_prism(pf.profile.data(), static_cast<int>(pf.profile.size() / 2), depth);
  if (prism.isNull()) return {};
  return prism.located(topo::Location(frameToWorld(pf.frame)));
}

// ── Self-verify primitives (mesh audit) ──────────────────────────────────────────

// Euler characteristic V − E + F of a watertight (index-welded) triangle mesh. A
// single genus-0 lump ⇒ 2; k disjoint lumps ⇒ 2k. Counts only referenced vertices.
inline long eulerChar(const tess::Mesh& m) {
  std::unordered_set<std::uint32_t> verts;
  for (const tess::Triangle& t : m.triangles) { verts.insert(t.a); verts.insert(t.b); verts.insert(t.c); }
  const auto edges = tess::edgeUseCounts(m);
  return static_cast<long>(verts.size()) - static_cast<long>(edges.size()) +
         static_cast<long>(m.triangles.size());
}

// Distinct supporting planes of a mesh: each non-degenerate triangle's plane,
// canonicalized to a sign-independent (normal, offset) key and quantized. The count
// is the geometric face count (adjacency) the convex re-solve must preserve.
inline std::size_t distinctPlaneCount(const tess::Mesh& m) {
  struct Key {
    std::int64_t n[3], d;
    bool operator==(const Key& o) const noexcept {
      return n[0] == o.n[0] && n[1] == o.n[1] && n[2] == o.n[2] && d == o.d;
    }
  };
  struct Hash {
    std::size_t operator()(const Key& k) const noexcept {
      std::size_t h = 1469598103934665603ull;
      for (std::int64_t v : {k.n[0], k.n[1], k.n[2], k.d})
        h = (h ^ static_cast<std::size_t>(v)) * 1099511628211ull;
      return h;
    }
  };
  auto q = [](double v) { return static_cast<std::int64_t>(std::llround(v / kPlaneQuant)); };
  std::unordered_set<Key, Hash> planes;
  for (const tess::Triangle& t : m.triangles) {
    const math::Vec3 a = m.vertices[t.a].asVec();
    math::Vec3 nrm = math::cross(m.vertices[t.b].asVec() - a, m.vertices[t.c].asVec() - a);
    const double len = math::norm(nrm);
    if (!(len > 1e-12)) continue;
    nrm = nrm / len;
    double off = math::dot(nrm, a);
    // Canonicalize the sign so a plane and its flip hash the same.
    const bool flip = nrm.x < -1e-9 ||
                      (std::fabs(nrm.x) <= 1e-9 && nrm.y < -1e-9) ||
                      (std::fabs(nrm.x) <= 1e-9 && std::fabs(nrm.y) <= 1e-9 && nrm.z < 0.0);
    if (flip) { nrm = -nrm; off = -off; }
    planes.insert(Key{{q(nrm.x), q(nrm.y), q(nrm.z)}, q(off)});
  }
  return planes.size();
}

// The full re-solve gate. `expectedVol = V₀ + A_F·d̄` is the closed-form oracle.
inline bool verifyResolve(const topo::Shape& result, std::size_t origPlanes,
                          const math::Vec3& n, double wt, double expectedVol, double defl) {
  if (result.isNull()) return false;
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(result);
  if (!tess::isWatertight(m)) return false;             // closed 2-manifold
  const double vol = std::fabs(tess::enclosedVolume(m));
  if (!(vol > 0.0)) return false;                       // positive volume
  if (eulerChar(m) != 2) return false;                  // single genus-0 lump
  if (distinctPlaneCount(m) != origPlanes) return false;  // adjacency preserved

  bool anyOn = false;                                   // moved face reached P_t,
  for (const math::Point3& v : m.vertices) {            // and nothing pokes past it
    const double sd = math::dot(n, v.asVec()) - wt;
    if (sd > kOnPlaneTol) return false;
    if (std::fabs(sd) <= kOnPlaneTol) anyOn = true;
  }
  if (!anyOn) return false;

  const double volTol = std::max(1e-6 * std::fabs(expectedVol), 1e-4);
  return std::fabs(vol - expectedVol) <= volTol;
}

}  // namespace rfdetail

// ─────────────────────────────────────────────────────────────────────────────
// The DM2 verb. Retarget the planar face `faceId` (1-based, mapShapes Face order —
// the same ids the facade/OCCT path uses) of `solid` onto the plane through `tp`
// with normal `nRaw`, re-solving the neighbours. Returns the one verified watertight
// solid, or a NULL Shape (with a measured `ReplaceFaceDecline`) for an honest decline.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape replaceFaceToPlane(const topo::Shape& solid, int faceId,
                                      const math::Point3& tp, const math::Vec3& nRaw,
                                      ReplaceFaceDecline* why = nullptr, double defl = 0.005) {
  using namespace rfdetail;
  auto fail = [&](ReplaceFaceDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  const double nlen = math::norm(nRaw);
  if (!(nlen > 1e-9)) return fail(ReplaceFaceDecline::DegenerateNormal);
  const math::Vec3 n = nRaw / nlen;                    // unit target normal
  const double wt = math::dot(n, tp.asVec());          // target plane offset

  const auto pfOpt = readPickedFace(solid, faceId);
  if (!pfOpt) return fail(ReplaceFaceDecline::NonPlanarOrForeign);
  const PickedFace& pf = *pfOpt;

  const math::Vec3 nF = pf.plane.normal;               // unit outward face normal
  const double denom = math::dot(n, nF);
  if (denom < kMinDenom) return fail(ReplaceFaceDecline::DegenerateTarget);

  // Signed displacement δ(p) = ((tp − p)·n)/(n·n̂_F) of a face point onto P_t along n̂_F.
  auto delta = [&](const math::Point3& p) { return math::dot(tp - p, n) / denom; };
  const double dBar = delta(pf.centroid);
  double dMax = delta(pf.loop.front()), dMin = dMax;
  for (const math::Point3& v : pf.loop) {
    const double d = delta(v);
    dMax = std::max(dMax, d);
    dMin = std::min(dMin, d);
  }
  if (std::fabs(dMax) <= kClassTol && std::fabs(dMin) <= kClassTol)
    return fail(ReplaceFaceDecline::DegenerateTarget);  // coincident (no-op)

  // Dispatch — every branch is one or two already-gated verbs, keep the bulk (−n side).
  const bool parallel = std::fabs(denom - 1.0) < kParallelTol;
  topo::Shape result;
  nb::HalfSpaceCutDecline sd = nb::HalfSpaceCutDecline::Ok;
  if (parallel && dBar < 0.0) {
    // (1) pure parallel trim — one DM1 cut.
    result = nb::splitByPlane(solid, tp, n, /*keepPositive=*/false, defl, &sd);
  } else if (parallel) {
    // (2) pure parallel grow — fuse the slab extruded to the target.
    const topo::Shape slab = buildSlab(pf, dBar);
    if (slab.isNull()) return fail(ReplaceFaceDecline::BuildFailed);
    result = nb::boolean_solid(solid, slab, nb::Op::Fuse);
  } else {
    // (3) tilted / mixed re-solve — GROW-then-TRIM (skip the grow for a pure-trim tilt).
    topo::Shape operand = solid;
    if (dMax > kClassTol) {
      const double growTo = dMax + 0.5 * pf.faceDiag + 1e-3;  // overshoot the target reach
      const topo::Shape slab = buildSlab(pf, growTo);
      if (slab.isNull()) return fail(ReplaceFaceDecline::BuildFailed);
      operand = nb::boolean_solid(solid, slab, nb::Op::Fuse);
      if (operand.isNull()) return fail(ReplaceFaceDecline::BuildFailed);
    }
    result = nb::splitByPlane(operand, tp, n, /*keepPositive=*/false, defl, &sd);
  }
  if (result.isNull()) return fail(ReplaceFaceDecline::BuildFailed);

  // Self-verify against the closed-form oracle V₀ + A_F·d̄; discard anything that fails.
  tess::MeshParams mp0; mp0.deflection = defl;
  const tess::Mesh origMesh = tess::SolidMesher(mp0).mesh(solid);
  const double v0 = std::fabs(tess::enclosedVolume(origMesh));
  const std::size_t origPlanes = distinctPlaneCount(origMesh);
  if (!verifyResolve(result, origPlanes, n, wt, v0 + pf.area * dBar, defl))
    return fail(ReplaceFaceDecline::SelfVerifyFailed);

  if (why) *why = ReplaceFaceDecline::Ok;
  return result;
}

}  // namespace cybercad::native::directmodel

#endif  // CYBERCAD_NATIVE_DIRECTMODEL_REPLACE_FACE_H
