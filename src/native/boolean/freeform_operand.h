// SPDX-License-Identifier: Apache-2.0
//
// freeform_operand.h — MOAT M2 / B1: the freeform operand DESCRIPTOR + its
// `recogniseFreeformSolid` admission gate.
//
// ── WHAT THIS IS (and is NOT) ─────────────────────────────────────────────────
// `ssi_boolean.h` `recogniseCurvedSolid(const Shape&)` folds a solid whose curved
// wall is ONE elementary surface (cylinder / sphere / cone half-space + planar
// caps) into a `CurvedSolid`; the moment a face is `Kind::BSpline`/`Kind::Bezier`
// it returns `nullopt` (`default: return nullopt`) — the analytic path declines a
// genuinely-freeform operand. This header is the strictly ADDITIVE SIBLING for
// that declined family: a value descriptor of a freeform-faced solid plus the gate
// that ADMITS one reachable freeform operand and hands the M2 verbs exactly the
// handles they consume:
//
//   * the freeform `Face`  → B2 `boolean/face_split.h splitFace`
//   * the operand `Shape`  → M0 `tessellate/solid_mesher.h SolidMesher::mesh`
//   * the world `Aabb`     → B3 `boolean/freeform_membership.h classifyPointInMesh`
//
// It does NOT touch `recogniseCurvedSolid` / `classifyPoint`, nor any of the
// consumed M0/M1/B2/B3 subsystems — they stay byte-identical. It carries NO derived
// mesh (M0 is asked on demand) and NO OCCT type.
//
// ── HONESTY CONTRACT ──────────────────────────────────────────────────────────
// `recogniseFreeformSolid` returns `nullopt` with a MEASURED blocker
// (`OperandDecline`) whenever the operand is not admissible — a non-solid, a
// multi-shell solid, a face whose surface is missing, an unsupported surface kind
// (torus/other), a freeform face that is bare (untrimmed / natural-rectangle) or
// holed, an operand with no freeform face at all (the analytic paths own it), or a
// boundary that is not a closed 2-manifold (leaky/open). The reason is what the
// caller logs before falling through to the OCCT oracle. No tolerance is weakened;
// the watertight audit is the same exactly-two-incidences predicate M0's mesh-level
// `isWatertight` uses, lifted to the topology graph. An honest decline is a
// first-class outcome.
//
// ── SUBSTRATE ─────────────────────────────────────────────────────────────────
// OCCT-FREE (0 OCCT includes). Depends only on `src/native/{math,topology,
// tessellate}` and the B3 `Aabb` (all OCCT-free). Header-only, `clang++ -std=c++20`.
// NO `cc_*` ABI change.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_FREEFORM_OPERAND_H
#define CYBERCAD_NATIVE_BOOLEAN_FREEFORM_OPERAND_H

#include "native/boolean/freeform_membership.h"  // Aabb (shared world-box type)
#include "native/math/native_math.h"
#include "native/tessellate/surface_eval.h"      // SurfaceEvaluator (outward normal)
#include "native/tessellate/trim.h"              // buildRegion / UVRegion (trim audit)
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cstddef>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;

// ─────────────────────────────────────────────────────────────────────────────
// The descriptor.
// ─────────────────────────────────────────────────────────────────────────────

/// A boundary face's role in the operand. `Freeform` = a genuinely-trimmed
/// BSpline/Bezier wall (the B2 splitFace target). `AnalyticHalfSpace` = an
/// elementary cap/side (plane / cylinder / sphere / cone).
enum class FaceRole { Freeform, AnalyticHalfSpace };

/// One boundary face, tagged. `surface`+`location` re-evaluate (via
/// SurfaceEvaluator) to the face's own surface — the faithful round-trip. `face`
/// is the topology Face itself (its trimmed EDGE_LOOP carried verbatim), which the
/// verbs consume. `outwardN` is the orientation-resolved surface normal at the
/// face's trim centroid (material-outward when the face is Forward).
struct OperandFace {
  topo::Shape face;
  topo::FaceSurface surface;
  topo::Location location;
  FaceRole role = FaceRole::AnalyticHalfSpace;
  math::Vec3 outwardN{0.0, 0.0, 0.0};
};

/// The admitted freeform operand — exactly what the minimal M2 assembly reads.
struct FreeformOperand {
  topo::Shape solid;                      ///< the operand Shape (→ M0 SolidMesher, → B3)
  std::vector<OperandFace> faces;         ///< every boundary face, role-tagged
  std::vector<std::size_t> freeform;      ///< indices into `faces`: the freeform walls (→ B2)
  std::vector<std::size_t> analytic;      ///< indices into `faces`: the analytic caps/sides
  Aabb bbox;                              ///< world AABB (→ B3 ON-band scale)
  bool watertight = false;                ///< closed 2-manifold audit (always true when admitted)
};

/// The measured admission blocker (the reason logged before OCCT fall-through).
enum class OperandDecline {
  Ok,
  NotSolid,                  ///< null shape or not a Solid
  MultiShell,                ///< not exactly one shell under the solid
  FaceSurfaceMissing,        ///< a face carries no FaceSurface geometry
  UnsupportedSurfaceKind,    ///< a Torus / other surface kind — declined
  BareFreeformFace,          ///< a freeform face is untrimmed (natural-rectangle / no wire)
  HoledFreeformFace,         ///< a freeform face has an inner hole loop
  NoFreeformFace,            ///< no freeform wall at all (the analytic paths own it)
  NotWatertight              ///< boundary is not a closed 2-manifold (open/leaky)
};

inline const char* declineName(OperandDecline d) noexcept {
  switch (d) {
    case OperandDecline::Ok: return "Ok";
    case OperandDecline::NotSolid: return "NotSolid";
    case OperandDecline::MultiShell: return "MultiShell";
    case OperandDecline::FaceSurfaceMissing: return "FaceSurfaceMissing";
    case OperandDecline::UnsupportedSurfaceKind: return "UnsupportedSurfaceKind";
    case OperandDecline::BareFreeformFace: return "BareFreeformFace";
    case OperandDecline::HoledFreeformFace: return "HoledFreeformFace";
    case OperandDecline::NoFreeformFace: return "NoFreeformFace";
    case OperandDecline::NotWatertight: return "NotWatertight";
  }
  return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// Admission helpers (free functions — the gate delegates to these so its own
// cognitive complexity stays in the backend band).
// ─────────────────────────────────────────────────────────────────────────────
namespace fodetail {

/// Per-face role from its surface kind. `nullopt` for Torus / any unmodelled kind
/// (→ the gate declines with UnsupportedSurfaceKind).
inline std::optional<FaceRole> classifyFaceRole(topo::FaceSurface::Kind k) noexcept {
  using K = topo::FaceSurface::Kind;
  switch (k) {
    case K::Plane:
    case K::Cylinder:
    case K::Cone:
    case K::Sphere:
      return FaceRole::AnalyticHalfSpace;
    case K::BSpline:
    case K::Bezier:
      return FaceRole::Freeform;
    case K::Torus:
    default:
      return std::nullopt;
  }
}

/// Trim status of a freeform face. A wall is only reachable if it carries EXACTLY
/// one outer trimming loop (a genuine, non-rectangular silhouette) and no hole
/// loop — that is what B2 splitFace + M0 trimmedFreeformMesh consume.
enum class TrimStatus { Genuine, Bare, Holed };

inline TrimStatus freeformTrimStatus(const topo::Shape& face) {
  int wires = 0;
  for (topo::Explorer wx(face, topo::ShapeType::Wire); wx.more(); wx.next()) ++wires;
  if (wires == 0) return TrimStatus::Bare;   // untrimmed natural rectangle
  if (wires > 1) return TrimStatus::Holed;   // an inner hole loop
  const tess::UVRegion reg = tess::buildRegion(face, 24);
  if (!reg.hasOuter()) return TrimStatus::Bare;
  if (!reg.holes.empty()) return TrimStatus::Holed;
  if (reg.isFullRectangle(1e-6, /*requireCorners=*/false)) return TrimStatus::Bare;
  return TrimStatus::Genuine;
}

/// Orientation-resolved outward surface normal at the face's trim centroid. The
/// centroid of the flattened outer loop is a valid in-domain (u,v) for a convex
/// trim; the geometric normal there, flipped for a Reversed face, is the
/// material-outward normal (makeFace's Forward ⇒ outward convention).
inline math::Vec3 faceOutwardNormal(const topo::Shape& face, const topo::FaceSurface& surface,
                                    const topo::Location& location) {
  double cu = 0.5, cv = 0.5;
  const tess::UVRegion reg = tess::buildRegion(face, 16);
  if (reg.hasOuter()) {
    double su = 0.0, sv = 0.0;
    for (const tess::UV& q : reg.outer) { su += q.u; sv += q.v; }
    const double inv = 1.0 / static_cast<double>(reg.outer.size());
    cu = su * inv;
    cv = sv * inv;
  }
  const tess::SurfaceEvaluator eval(surface, location);
  const tess::SurfaceSample d = eval.d1(cu, cv);
  math::Vec3 n = d.normal.valid() ? d.normal.vec() : math::cross(d.du, d.dv);
  if (face.orientation() == topo::Orientation::Reversed) n = -n;
  return n;
}

/// Closed 2-manifold audit on the topology graph: every undirected edge — keyed by
/// its unordered pair of endpoint-VERTEX identities — must be used by EXACTLY two
/// face incidences. This is the topological lift of M0's mesh-level `isWatertight`
/// (every mesh edge shared by exactly two triangles). It relies on coincident faces
/// sharing vertex nodes (how the substrate builds watertight solids — addPCurve
/// reuses the edge's vertex children); a solid whose coincident boundary does not
/// share vertices honestly fails the audit and is declined (→ OCCT).
/// Undirected key of an edge = the sorted pair of its endpoint-VERTEX identities.
/// A CLOSED edge (a full circle) has both endpoints at the SAME vertex node, which
/// the Explorer deduplicates to one visit — a valid seam-less loop, keyed (id, id).
/// Returns {-1,-1} for an edge with no bounding vertex (degenerate / not closable).
inline std::pair<int, int> edgeVertexKey(const topo::Shape& edge, const topo::ShapeMap& vmap) {
  int lo = 0, hi = 0, count = 0;
  for (topo::Explorer vx(edge, topo::ShapeType::Vertex); vx.more(); vx.next()) {
    const int id = vmap.findIndex(vx.current());
    if (count == 0) { lo = hi = id; }
    else { lo = std::min(lo, id); hi = std::max(hi, id); }
    ++count;
  }
  return count == 0 ? std::pair<int, int>{-1, -1} : std::pair<int, int>{lo, hi};
}

inline bool watertightByEdgeIncidence(const topo::Shape& solid) {
  const topo::ShapeMap vmap = topo::mapShapes(solid, topo::ShapeType::Vertex);
  std::map<std::pair<int, int>, int> incidence;
  for (topo::Explorer fx(solid, topo::ShapeType::Face); fx.more(); fx.next())
    for (topo::Explorer ex(fx.current(), topo::ShapeType::Edge); ex.more(); ex.next()) {
      const std::pair<int, int> key = edgeVertexKey(ex.current(), vmap);
      if (key.first < 0) return false;  // an edge with no bounding vertex — not closable
      ++incidence[key];
    }
  if (incidence.empty()) return false;
  for (const auto& [key, c] : incidence)
    if (c != 2) return false;
  return true;
}

/// Admit ONE boundary face into an `OperandFace`, or set `why` and return nullopt.
/// Isolated so the gate's per-face fold stays a flat loop (keeps the driver's
/// cognitive complexity well within the backend band).
inline std::optional<OperandFace> admitOperandFace(const topo::Shape& f, OperandDecline& why) {
  const auto surf = topo::surfaceOf(f);
  if (!surf || !surf->surface) { why = OperandDecline::FaceSurfaceMissing; return std::nullopt; }
  const auto role = classifyFaceRole(surf->surface->kind);
  if (!role) { why = OperandDecline::UnsupportedSurfaceKind; return std::nullopt; }
  if (*role == FaceRole::Freeform) {
    const TrimStatus ts = freeformTrimStatus(f);
    if (ts == TrimStatus::Bare) { why = OperandDecline::BareFreeformFace; return std::nullopt; }
    if (ts == TrimStatus::Holed) { why = OperandDecline::HoledFreeformFace; return std::nullopt; }
  }
  OperandFace of;
  of.face = f;
  of.surface = *surf->surface;
  of.location = surf->location;
  of.role = *role;
  of.outwardN = faceOutwardNormal(f, of.surface, of.location);
  return of;
}

/// Faithful world AABB. Folded over each face's flattened outer-loop sampled on
/// the surface in 3D (so a solid whose vertices are sparse — e.g. closed circular
/// rims sharing one vertex — still yields the true boundary extent, which B3 needs
/// to scale its ON-band) plus the raw location-placed vertices as a backstop.
inline Aabb foldAabb(const topo::Shape& solid) {
  Aabb bb;
  bool first = true;
  auto expand = [&](const math::Point3& w) {
    if (first) { bb.lo = bb.hi = w; first = false; return; }
    bb.lo.x = std::min(bb.lo.x, w.x); bb.hi.x = std::max(bb.hi.x, w.x);
    bb.lo.y = std::min(bb.lo.y, w.y); bb.hi.y = std::max(bb.hi.y, w.y);
    bb.lo.z = std::min(bb.lo.z, w.z); bb.hi.z = std::max(bb.hi.z, w.z);
  };
  for (topo::Explorer fx(solid, topo::ShapeType::Face); fx.more(); fx.next()) {
    const topo::Shape f = fx.current();
    const auto surf = topo::surfaceOf(f);
    if (!surf || !surf->surface) continue;
    const tess::UVRegion reg = tess::buildRegion(f, 16);
    if (!reg.hasOuter()) continue;
    const tess::SurfaceEvaluator eval(*surf->surface, surf->location);
    for (const tess::UV& q : reg.outer) expand(eval.value(q.u, q.v));
  }
  for (topo::Explorer vx(solid, topo::ShapeType::Vertex); vx.more(); vx.next()) {
    auto p = topo::pointOf(vx.current());
    if (!p) continue;
    math::Point3 w = *p;
    const topo::Location loc = vx.current().location();
    if (!loc.isIdentity()) w = loc.transform().applyToPoint(w);
    expand(w);
  }
  return bb;
}

}  // namespace fodetail

// ─────────────────────────────────────────────────────────────────────────────
// recogniseFreeformSolid — the B1 gate. Admits ONE reachable freeform operand and
// exposes exactly the handles the M2 verbs consume; `nullopt` with a measured
// `OperandDecline` (written through `why` when non-null) otherwise. Additive
// sibling to `recogniseCurvedSolid`; does not edit it.
//
// backend-band: the per-face fold delegates role classification, the trim audit,
// the outward-normal computation, the edge-incidence watertight audit, and the
// AABB fold to `fodetail::` helpers, so the driver itself stays a short linear
// pass with early-return declines.
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<FreeformOperand> recogniseFreeformSolid(const topo::Shape& s,
                                                             OperandDecline* why = nullptr) {
  auto fail = [&](OperandDecline d) -> std::optional<FreeformOperand> {
    if (why) *why = d;
    return std::nullopt;
  };

  if (s.isNull() || s.type() != topo::ShapeType::Solid) return fail(OperandDecline::NotSolid);

  int shells = 0;
  for (topo::Explorer sh(s, topo::ShapeType::Shell); sh.more(); sh.next()) ++shells;
  if (shells != 1) return fail(OperandDecline::MultiShell);

  FreeformOperand op;
  op.solid = s;
  for (topo::Explorer fx(s, topo::ShapeType::Face); fx.more(); fx.next()) {
    OperandDecline faceWhy = OperandDecline::Ok;
    auto of = fodetail::admitOperandFace(fx.current(), faceWhy);
    if (!of) return fail(faceWhy);
    const bool isFreeform = of->role == FaceRole::Freeform;
    const std::size_t idx = op.faces.size();
    op.faces.push_back(std::move(*of));
    (isFreeform ? op.freeform : op.analytic).push_back(idx);
  }

  if (op.freeform.empty()) return fail(OperandDecline::NoFreeformFace);
  if (!fodetail::watertightByEdgeIncidence(s)) return fail(OperandDecline::NotWatertight);

  op.watertight = true;
  op.bbox = fodetail::foldAabb(s);
  if (why) *why = OperandDecline::Ok;
  return op;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_FREEFORM_OPERAND_H
