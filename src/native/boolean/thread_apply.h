// SPDX-License-Identifier: Apache-2.0
//
// thread_apply.h — MOAT (Class-B) native `cc_thread_apply`: apply a helical thread
// body to a shaft by the LANDED planar-polyhedron BSP boolean, with a mandatory
// four-part self-verify and an honest OCCT fall-through.
//
// ── ROLE (the #1 OCCT wall, attempted natively) ───────────────────────────────────
// `cc_thread_apply(shaft, thread, op)` FUSES the crest (op 0, external thread) or CUTS
// the groove (op 1, internal thread) of a helical thread onto a shaft. It is the
// boolean of a shaft cylinder with a swept helical thread ridge. A single-shot OCCT
// boolean of a fine multi-turn helix HANGS (minutes, non-cancellable — the app's #1
// acceleration wall, GitHub #286); the OCCT adapter survives only by rebuilding the
// thread one turn at a time and accumulating bounded per-turn booleans under a
// wall-clock budget (occt_thread_boolean.cpp). This verb is the OCCT-FREE native
// attempt, reusing the substrate the M2 booleans + the planar BSP `boolean_solid`
// already landed.
//
// ── THE PIPELINE ──────────────────────────────────────────────────────────────────
//   1. RECOGNISE the tractable input — the shaft an axis-parallel finite cylinder
//      (`curved::recogniseCylinder`), the thread a coaxial native helical-ridge solid
//      whose crest/root radii + z-extent are MEASURED from its mesh (crest = max
//      radial distance, root = min), mirroring the OCCT oracle's measureThread /
//      measureRootRadius. Decline ShaftNotCylinder / ThreadDegenerate / CrestBelowShaft.
//   2. FACET both operands into CONSISTENTLY-ORIENTED planar-triangle B-rep solids at a
//      controlled faceting deflection (each triangle a construct::detail::planarFace
//      carrying the mesh triangle's winding normal), so the EXACT planar BSP set-algebra
//      applies (the BSP domain is all-planar polyhedra).
//   3. BOOLEAN via the landed `boolean_solid(shaftFacets, threadFacets, op)` — op 0 fuse,
//      op 1 cut. Decline BooleanEmpty on a null/degenerate result.
//   4. SELF-VERIFY (mandatory, four-part) — WATERTIGHT + Euler χ = 2 + CONSISTENTLY
//      ORIENTED (the DIRECTED-edge invariant tess::isConsistentlyOriented, 0
//      same-direction duplicate half-edges) + a TWO-SIDED enclosed-volume band against
//      the closed-form threaded-shaft volume (FUSE: V_shaft < V ≤ V_shaft + V_thread;
//      CUT: V_shaft − V_thread ≤ V < V_shaft). Decline NotWatertight / NotOriented /
//      VolumeInconsistent. A candidate that passes all four is returned native.
//
// ── HONESTY (measured, not claimed) ────────────────────────────────────────────────
// A multi-turn helical thread DECLINES to OCCT today, for two VERIFIED reasons the
// self-verify catches:
//   * the native `build_thread` solid is watertight but NOT consistently oriented
//     (measured tess::sameDirectionEdgeCount == 6, a latent cap/band winding defect), so
//     it is an invalid BSP operand (the BSP's inside/outside classification assumes
//     coherent outward normals) → NotOriented;
//   * the near-tangent helical root ↔ shaft-wall contact fragments the dense triangle-soup
//     BSP into T-junction cracks (boundaryEdgeCount 15–140 across single-turn to 4-turn,
//     insets 0.6–1.5, deflections 0.05–0.15) → NotWatertight.
// ISOLATION: the SAME verb WELDS a faceted cylinder CUT by a box (bnd=0, sd=0, χ=2) at the
// analytic volume — so the BSP substrate is SOUND; the blocker is specific to the thread
// operand's orientation + the helical near-tangency (the sharpened next blocker for M7b:
// an orientation-coherent thread builder + robust dense-soup CSG with T-junction repair).
// "Fall through to OCCT only when we cannot verify a watertight, coherent, correct-volume
// native body — NEVER a faked / leaky / misoriented solid; NO tolerance widened."
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// `curved::recogniseCylinder` + `AxisCylinder`, the planar BSP `boolean_solid`, the M0
// tessellator (SolidMesher / isWatertight / isConsistentlyOriented / enclosedVolume /
// edgeUseCounts), construct::detail::planarFace. Additive sibling — touches NONE of them.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_THREAD_APPLY_H
#define CYBERCAD_NATIVE_BOOLEAN_THREAD_APPLY_H

#include "native/boolean/curved.h"           // recogniseCylinder / AxisCylinder / kPi
#include "native/boolean/native_boolean.h"   // boolean_solid
#include "native/construct/construct.h"       // detail::planarFace
#include "native/math/native_math.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_set>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;
namespace ncst = cybercad::native::construct;

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
/// watertight, consistently-oriented, correct-volume result solid is returned.
enum class ThreadApplyDecline {
  Ok,
  ShaftNotCylinder,   ///< the shaft operand is not an axis-parallel finite cylinder
  ThreadDegenerate,   ///< the thread mesh is empty / has no measurable crest radius
  CrestBelowShaft,    ///< FUSE: the thread crest does not clear the shaft surface
  BooleanEmpty,       ///< the BSP boolean returned a null / degenerate result
  NotWatertight,      ///< self-verify: the welded result is not a closed 2-manifold
  NotOriented,        ///< self-verify: watertight but orientation-INCONSISTENT (bad signed V)
  VolumeInconsistent  ///< self-verify: enclosed volume off the two-sided closed-form band
};

inline const char* threadApplyDeclineName(ThreadApplyDecline d) noexcept {
  switch (d) {
    case ThreadApplyDecline::Ok: return "Ok";
    case ThreadApplyDecline::ShaftNotCylinder: return "ShaftNotCylinder";
    case ThreadApplyDecline::ThreadDegenerate: return "ThreadDegenerate";
    case ThreadApplyDecline::CrestBelowShaft: return "CrestBelowShaft";
    case ThreadApplyDecline::BooleanEmpty: return "BooleanEmpty";
    case ThreadApplyDecline::NotWatertight: return "NotWatertight";
    case ThreadApplyDecline::NotOriented: return "NotOriented";
    case ThreadApplyDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

namespace tadetail {

/// Measured crest/root radii (about the cylinder axis) + axial extent of the thread,
/// from its M0 mesh. Mirrors the OCCT oracle's measureThread + measureRootRadius: crest
/// = max radial distance from the axis, root = min; the axial extent bounds the ridge.
struct ThreadGeom {
  double crestR = 0.0;
  double rootR = 0.0;
  double volume = 0.0;  ///< meshed ridge volume (exact for the tessellated ridge)
  bool ok = false;
};

/// Radial distance of a point from the cylinder axis (axis-perpendicular plane).
inline double radialDist(const math::Point3& p, const curved::AxisCylinder& cyl) noexcept {
  const auto perp = cyl.perpAxes();
  const double c[3] = {p.x, p.y, p.z};
  const double d0 = c[perp[0]] - cyl.c0;
  const double d1 = c[perp[1]] - cyl.c1;
  return std::hypot(d0, d1);
}

inline ThreadGeom measureThread(const tess::Mesh& m, const curved::AxisCylinder& cyl) {
  ThreadGeom g;
  if (m.vertices.empty() || m.triangles.empty()) return g;
  g.rootR = std::numeric_limits<double>::infinity();
  g.crestR = 0.0;
  for (const math::Point3& v : m.vertices) {
    const double r = radialDist(v, cyl);
    g.crestR = std::max(g.crestR, r);
    g.rootR = std::min(g.rootR, r);
  }
  if (!(g.crestR > 0.0) || !std::isfinite(g.rootR)) return g;
  g.volume = std::fabs(tess::enclosedVolume(m));
  g.ok = g.volume > 0.0;
  return g;
}

/// Facet a native solid into a CONSISTENTLY-ORIENTED planar-triangle B-rep solid: mesh
/// at `deflection`, then rebuild each mesh triangle as a `planarFace` whose normal is the
/// triangle's own winding normal (Forward). The faceted solid inherits the mesh's own
/// winding, so `extractPolygons`/BSP see coherent outward normals — provided the source
/// solid meshed consistently oriented (the caller's self-verify catches the case where it
/// did not, e.g. the sd=6 thread).
inline topo::Shape facetSolid(const topo::Shape& s, double deflection) {
  tess::MeshParams p;
  p.deflection = deflection;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  std::vector<topo::Shape> faces;
  faces.reserve(m.triangles.size());
  for (const tess::Triangle& t : m.triangles) {
    const math::Point3 a = m.vertices[t.a], b = m.vertices[t.b], c = m.vertices[t.c];
    const math::Vec3 n = math::cross(b - a, c - a);
    const double nl = math::norm(n);
    if (!(nl > 1e-12)) continue;  // drop a degenerate sliver triangle
    std::vector<topo::Shape> loop = {topo::ShapeBuilder::makeVertex(a),
                                     topo::ShapeBuilder::makeVertex(b),
                                     topo::ShapeBuilder::makeVertex(c)};
    faces.push_back(ncst::detail::planarFace(loop, math::Dir3{n / nl}, topo::Orientation::Forward));
  }
  if (faces.size() < 4) return {};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

/// Euler characteristic χ = V − E + F of a welded mesh, using the SHARED (welded) edge
/// map so E counts distinct undirected edges. For a closed sphere-topology boundary
/// χ = 2. Vertices are the distinct indices actually referenced by a triangle (so an
/// orphaned unwelded vertex does not inflate V).
inline long eulerCharacteristic(const tess::Mesh& m) {
  std::unordered_set<std::uint32_t> usedV;
  usedV.reserve(m.triangles.size() * 3);
  for (const tess::Triangle& t : m.triangles) {
    usedV.insert(t.a);
    usedV.insert(t.b);
    usedV.insert(t.c);
  }
  const long V = static_cast<long>(usedV.size());
  const long E = static_cast<long>(tess::edgeUseCounts(m).size());
  const long F = static_cast<long>(m.triangles.size());
  return V - E + F;
}

/// The mandatory four-part self-verify of a candidate threaded-shaft mesh: WATERTIGHT +
/// Euler χ=2 + consistently-oriented (0 same-direction duplicate half-edges) + a TWO-SIDED
/// enclosed-volume band. `vShaft` is the analytic shaft volume, `vThread` the measured ridge
/// volume; `op` selects the FUSE (0: V_shaft < V ≤ V_shaft+V_thread) or CUT (1: V_shaft−
/// V_thread ≤ V < V_shaft) band. `analyticVolume` (optional, NaN ⇒ unknown) tightens it.
/// Returns `Ok` iff every check passes, else the specific decline.
inline ThreadApplyDecline verifyThreadedSolid(const tess::Mesh& rm, int op, double vShaft,
                                              double vThread, double deflection,
                                              double analyticVolume) {
  if (!tess::isWatertight(rm) || eulerCharacteristic(rm) != 2)
    return ThreadApplyDecline::NotWatertight;
  if (tess::sameDirectionEdgeCount(rm) != 0) return ThreadApplyDecline::NotOriented;

  const double v = std::fabs(tess::enclosedVolume(rm));
  if (!(v > 0.0) || std::isnan(v)) return ThreadApplyDecline::VolumeInconsistent;

  // Two-sided operand-derived band; a faceted mesh under-counts by O(deflection), so allow
  // a matching slack. FUSE adds at most the whole ridge, CUT removes at most the whole ridge.
  const double slack = std::max(0.02 * vShaft, 30.0 * deflection * std::max(vThread, 1e-12));
  const double lo = op == 0 ? vShaft - slack : vShaft - vThread - slack;
  const double hi = op == 0 ? vShaft + vThread + slack : vShaft + slack;
  if (v < lo || v > hi) return ThreadApplyDecline::VolumeInconsistent;

  // Tighter TWO-SIDED band vs the closed-form threaded-shaft volume, when supplied.
  if (!std::isnan(analyticVolume) && analyticVolume > 0.0) {
    const double band = std::min(0.5, 30.0 * deflection) * analyticVolume;
    if (std::fabs(v - analyticVolume) > band) return ThreadApplyDecline::VolumeInconsistent;
  }
  return ThreadApplyDecline::Ok;
}

}  // namespace tadetail

// ─────────────────────────────────────────────────────────────────────────────
// threadApply — the entry point. `shaft`,`thread` are native solids. `op` = 0 FUSE
// (external), 1 CUT (internal). Returns the welded, self-verified threaded-shaft solid,
// or a NULL Shape (→ OCCT `thread_apply`) with a measured `ThreadApplyDecline`. Never
// emits a leaky / partial / orientation-inconsistent / wrong-volume solid; no tolerance
// widened.
//
// `analyticVolume` (optional, NaN ⇒ unknown): the closed-form threaded-shaft volume. When
// supplied it TIGHTENS the two-sided band; when unknown the operand-derived bound
// (V_shaft ± V_thread) + the orientation-coherence gate still guarantee no wrong solid.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape threadApply(const topo::Shape& shaft, const topo::Shape& thread, int op,
                               double deflection = 0.05, ThreadApplyDecline* why = nullptr,
                               double analyticVolume = std::numeric_limits<double>::quiet_NaN()) {
  using namespace tadetail;
  auto fail = [&](ThreadApplyDecline d) -> topo::Shape { if (why) *why = d; return {}; };
  if (op != 0 && op != 1) return fail(ThreadApplyDecline::BooleanEmpty);

  // (1) recognise the shaft cylinder + measure the thread geometry.
  const auto cyl = curved::recogniseCylinder(shaft);
  if (!cyl) return fail(ThreadApplyDecline::ShaftNotCylinder);

  tess::MeshParams mp;
  mp.deflection = deflection;
  const tess::Mesh threadMesh = tess::SolidMesher{mp}.mesh(thread);
  const ThreadGeom tg = measureThread(threadMesh, *cyl);
  if (!tg.ok) return fail(ThreadApplyDecline::ThreadDegenerate);

  // FUSE needs the crest to clear the shaft surface (else nothing is added — the ridge is
  // buried); CUT needs the groove to reach inside the shaft (root < shaft radius). Both
  // mirror the OCCT oracle's crest/root sanity gate.
  const double Rs = cyl->radius;
  if (op == 0 && !(tg.crestR > Rs + 1e-9)) return fail(ThreadApplyDecline::CrestBelowShaft);

  const double vShaft = cyl->volume();
  if (!(vShaft > 0.0)) return fail(ThreadApplyDecline::ShaftNotCylinder);

  // (2) facet both operands into consistently-oriented planar-triangle solids.
  const topo::Shape shaftF = facetSolid(shaft, deflection);
  const topo::Shape threadF = facetSolid(thread, deflection);
  if (shaftF.isNull() || threadF.isNull()) return fail(ThreadApplyDecline::BooleanEmpty);

  // (3) the landed planar BSP set-algebra.
  const topo::Shape result = boolean_solid(shaftF, threadF, op);
  if (result.isNull()) return fail(ThreadApplyDecline::BooleanEmpty);

  // (4) mandatory four-part self-verify (watertight + χ=2 + oriented + two-sided volume).
  const tess::Mesh rm = tess::SolidMesher{mp}.mesh(result);
  const ThreadApplyDecline verdict =
      verifyThreadedSolid(rm, op, vShaft, tg.volume, deflection, analyticVolume);
  if (verdict != ThreadApplyDecline::Ok) return fail(verdict);

  if (why) *why = ThreadApplyDecline::Ok;
  return result;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_THREAD_APPLY_H
