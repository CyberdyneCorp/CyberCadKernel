// SPDX-License-Identifier: Apache-2.0
//
// split_plane.h — MOAT M-DM DM1: the FIRST native direct-modeling verb, a planar
// half-space SPLIT of a solid, assembled ADDITIVELY from the two already-landed,
// already-gated verbs. It re-derives NO geometry:
//
//   * an operand with ONE freeform (Bezier) wall  → `freeformHalfSpaceCut` on the
//     keep side (the landed M2-assembly B4 cut, which self-verifies watertight);
//   * an all-planar polyhedron                     → `boolean_solid(operand, box, Cut)`
//     with `box` = a bbox-scaled native planar half-space covering the DISCARD side
//     (the landed BSP-CSG cut, capped exactly on the plane).
//
// `splitByPlane` picks the surviving half by `keepPositive` relative to the plane
// NORMAL: keepPositive != 0 keeps the +n half (KeepSide::Above, signed dist ≥ 0);
// 0 keeps the −n half (KeepSide::Below). Two calls (0 then 1) yield the two pieces,
// whose volumes SUM to the whole (partition-closure) because both verbs cap on the
// SAME plane.
//
// ── SELF-VERIFY → HONEST DECLINE (never a leaky piece) ───────────────────────────
// The result is accepted only when it is a closed watertight 2-manifold (mesh-level
// audit, the SAME predicate the two verbs already use). Anything the two verbs cannot
// robustly do — a plane grazing tangent to a curved face, a perpendicular cylinder
// slice (`cyl − box`, which the landed curved slice excludes), a multi-lump split, a
// degenerate/coincident plane, a multi-freeform operand — returns a NULL Shape (a
// measured `HalfSpaceCutDecline`). The engine then reports the honest decline; it
// NEVER hands a native void to OCCT and NEVER emits an unverified solid.
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// `freeformHalfSpaceCut` (half_space_cut.h), `boolean_solid` (native_boolean.h),
// `build_prism` (construct.h). Additive sibling — touches none of them.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_SPLIT_PLANE_H
#define CYBERCAD_NATIVE_BOOLEAN_SPLIT_PLANE_H

#include "native/boolean/half_space_cut.h"
#include "native/boolean/native_boolean.h"
#include "native/construct/native_construct.h"
#include "native/math/native_math.h"
#include "native/math/transform.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace cybercad::native::boolean {

namespace spdetail {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;
namespace cst  = cybercad::native::construct;

// A world axis-aligned bounding box of a native solid, from a coarse watertight mesh
// (planar solids mesh exact; curved solids are within `defl`). `valid` is false when
// the solid cannot be meshed.
struct MeshAabb {
  math::Point3 lo, hi;
  bool valid = false;
};

inline MeshAabb meshBounds(const topo::Shape& s, double defl) {
  MeshAabb bb;
  if (s.isNull()) return bb;
  tess::MeshParams mp;
  mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  if (m.vertices.empty()) return bb;
  const auto& v0 = m.vertices.front();
  double lo[3] = {v0.x, v0.y, v0.z}, hi[3] = {v0.x, v0.y, v0.z};
  for (const auto& p : m.vertices) {
    const double c[3] = {p.x, p.y, p.z};
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], c[k]);
      hi[k] = std::max(hi[k], c[k]);
    }
  }
  bb.lo = math::Point3{lo[0], lo[1], lo[2]};
  bb.hi = math::Point3{hi[0], hi[1], hi[2]};
  bb.valid = true;
  return bb;
}

// A reference X direction guaranteed not parallel to `z` (for the Gram-Schmidt frame).
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

// Build the DISCARD half-space as a native planar box sitting on the plane `P`. The
// box near-face lies exactly on P (local w = 0) and it strictly overspans the operand
// (lateral half-width and depth = 2·(diag+1), centred on the operand-bbox-centre's
// projection onto P), so `operand − box` removes the discard half and caps on P — the
// native mirror of OCCT MakeHalfSpace + Cut. keepPositive keeps the +normal (w ≥ 0)
// half, so the discard box covers w ≤ 0 (offset down by its depth); else it covers
// w ≥ 0.
inline topo::Shape halfSpaceBox(const math::Plane& P, bool keepPositive, const MeshAabb& bb) {
  const double diag = std::max(math::distance(bb.lo, bb.hi), 1e-9);
  const double ext = 2.0 * (diag + 1.0);  // lateral half-width AND depth (L in OCCT)

  // Centre the lateral square at the bbox centre projected onto P's frame.
  const math::Point3 c{0.5 * (bb.lo.x + bb.hi.x), 0.5 * (bb.lo.y + bb.hi.y),
                       0.5 * (bb.lo.z + bb.hi.z)};
  const math::Vec3 d = c - P.pos.origin;
  const double cu = math::dot(d, P.pos.x.vec());
  const double cv = math::dot(d, P.pos.y.vec());

  const double sq[8] = {cu - ext, cv - ext, cu + ext, cv - ext,
                        cu + ext, cv + ext, cu - ext, cv + ext};
  topo::Shape box = cst::build_prism(sq, 4, ext);  // local w ∈ [0, ext] along +z
  if (box.isNull()) return {};

  math::Transform T = frameToWorld(P.pos);
  if (keepPositive)  // discard w ≤ 0 → shift the box down to w ∈ [−ext, 0]
    T = T.composedWith(math::Transform::translationOf(math::Vec3{0, 0, -ext}));
  return box.located(topo::Location(T));
}

// Watertight self-verify at `defl` (planar meshes exact). The freeform verb already
// self-verifies internally; this is the audit for the analytic BSP path.
inline bool watertightAt(const topo::Shape& s, double defl) {
  if (s.isNull()) return false;
  tess::MeshParams mp;
  mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  return tess::isWatertight(m);
}

}  // namespace spdetail

// ─────────────────────────────────────────────────────────────────────────────
// The DM1 verb: split `operand` by the plane through `o` with normal `n`, keeping the
// `keepPositive ? +n : −n` half. Returns the one watertight keep-side Solid, or a NULL
// Shape (with a measured `HalfSpaceCutDecline`) the engine turns into an honest decline.
// `defl` sizes the self-verify mesh (0.008 mirrors the landed freeform weld gate).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape splitByPlane(const topo::Shape& operand, const math::Point3& o,
                                const math::Vec3& n, bool keepPositive, double defl = 0.008,
                                HalfSpaceCutDecline* why = nullptr) {
  using namespace spdetail;
  auto fail = [&](HalfSpaceCutDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  if (operand.isNull()) return fail(HalfSpaceCutDecline::NotAdmitted);
  if (!(math::norm(n) > 1e-9)) return fail(HalfSpaceCutDecline::NotAdmitted);

  const math::Dir3 z{n};  // normalizes
  const math::Ax3 fr = math::Ax3::fromAxisAndRef(o, z, refNotParallel(z));
  const math::Plane P{fr};
  const KeepSide side = keepPositive ? KeepSide::Above : KeepSide::Below;

  // (A) freeform-walled operand → the landed B4 cut (self-verifies watertight).
  HalfSpaceCutDecline wff = HalfSpaceCutDecline::Ok;
  const topo::Shape ff = freeformHalfSpaceCut(operand, P, side, defl, &wff);
  if (!ff.isNull()) { if (why) *why = HalfSpaceCutDecline::Ok; return ff; }

  // (B) all-planar polyhedron → the landed BSP cut against the discard half-space box.
  const MeshAabb bb = meshBounds(operand, defl);
  if (!bb.valid) return fail(wff);
  const topo::Shape box = halfSpaceBox(P, keepPositive, bb);
  if (box.isNull()) return fail(wff);
  const topo::Shape cut = boolean_solid(operand, box, Op::Cut);
  if (cut.isNull()) return fail(wff);  // curved / degenerate → outside the native domain
  if (!watertightAt(cut, defl)) return fail(HalfSpaceCutDecline::NotWatertight);

  if (why) *why = HalfSpaceCutDecline::Ok;
  return cut;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_SPLIT_PLANE_H
