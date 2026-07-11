// SPDX-License-Identifier: Apache-2.0
//
// native_boolean.h — public aggregate header for the native planar-polyhedron
// boolean (Phase 4, capability #5 `native-booleans`, openspec/NATIVE-REWRITE.md).
//
// Clean-room, OCCT-FREE constructive-solid-geometry (fuse / cut / common) for
// solids whose faces are ALL PLANAR (polyhedra — boxes, prisms, convex or simple-
// concave). It is the RESEARCH-GRADE capability, delivered HONESTLY narrow: a
// general robust B-rep boolean over curved + near-tangent geometry is out of reach
// in one pass, so anything the planar algorithm cannot robustly handle returns a
// NULL Shape and the engine falls through to the OCCT BOPAlgo oracle (labelled,
// verified — never faked).
//
// ── PIPELINE ──────────────────────────────────────────────────────────────────
//   polygon.h   extractPolygons(solid) — flatten each planar face to an oriented
//               world-space Polygon (outward normal = front). isAllPlanar guards
//               the domain.
//   bsp.h       Node — a BSP tree over the polygons; splitPolygon cuts a face along
//               a plane (face-face intersection + fragment split, implicitly), clip
//               drops the inside parts, invert complements the solid. The three
//               booleans are the classic BSP-CSG compositions (Naylor–Amanatides–
//               Thibault, SIGGRAPH 1990) — reference algorithm, not OCCT code.
//   assemble.h  assembleSolid(polys) — weld coincident corners to shared vertices
//               and build faces → shell → Solid.
//
// ── SET ALGEBRA (the op mapping the cc_boolean contract asks for) ─────────────
//   op 0 fuse   = A ∪ B  — outside(A) + outside(B) + coherent coincident faces.
//   op 1 cut    = A − B  — outside(A) + (inside-B-of-A, reversed).
//   op 2 common = A ∩ B  — inside(A) ∩ inside(B).
// Each is expressed as clip/invert/build on the two trees (booleanPolygons below);
// coincident (coplanar) faces of two boxes sharing a wall are resolved by the
// coplanar-front/back split + the invert-clip-invert overlap removal, so a fuse of
// two flush boxes yields ONE merged wall, not a doubled/missing one.
//
// ── SELF-VERIFY IS THE ENGINE'S JOB, NOT THIS LIBRARY'S ───────────────────────
// boolean_solid returns the assembled native Solid (or NULL when the inputs are
// non-planar / degenerate). It does NOT itself decide whether the result is good
// enough to ship — the engine (native_engine.cpp) runs the mandatory watertight +
// correct-volume self-verify and DISCARDS a bad result, falling through to OCCT.
// Keeping the guard in the engine (where the OCCT fallback lives) keeps this library
// OCCT-free and single-purpose.
//
// ── COMPLEXITY ────────────────────────────────────────────────────────────────
// The irreducible geometry (splitPolygon ~18) is isolated in bsp.h and flagged
// there; boolean_solid and the op compositions are short linear drivers (≤ ~8).
//
// clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_H
#define CYBERCAD_NATIVE_BOOLEAN_H

#include "native/boolean/assemble.h"
#include "native/boolean/bsp.h"
#include "native/boolean/curved.h"
#include "native/boolean/native_boolean_fwd.h"  // Op (shared with ssi_boolean.h)
#include "native/boolean/polygon.h"
#include "native/boolean/ssi_boolean.h"          // ssi_boolean_solid (SSI Stage S5-a)
#include "native/topology/native_topology.h"

#include <optional>
#include <vector>

namespace cybercad::native::boolean {

// `Op` is declared in native_boolean_fwd.h so the SSI-driven S5-a path can name it
// without a circular include; it is the same enum the cc_boolean op codes map to.

// ─────────────────────────────────────────────────────────────────────────────
// booleanPolygons — the BSP-CSG compositions. Given the two solids' polygon soups,
// return the polygon soup of the requested op. Each mirrors the canonical csg-on-
// BSP recipe:
//
//   fuse (A ∪ B):
//       a.clipTo(b); b.clipTo(a);
//       b.invert(); b.clipTo(a); b.invert();     // remove B's coincident overlap
//       a.build(b.all());
//   cut (A − B):
//       a.invert(); a.clipTo(b);                 // keep A's inside-of-B, reversed
//       b.clipTo(a); b.invert(); b.clipTo(a); b.invert();
//       a.build(b.all()); a.invert();
//   common (A ∩ B):  A ∩ B = ¬(¬A ∪ ¬B) — the complement of the union of the
//       complements, one invert-fuse-invert.
// ─────────────────────────────────────────────────────────────────────────────
inline std::vector<Polygon> booleanPolygons(std::vector<Polygon> aPolys,
                                            std::vector<Polygon> bPolys, Op op) {
  Node a(std::move(aPolys));
  Node b(std::move(bPolys));

  switch (op) {
    case Op::Fuse: {
      a.clipTo(b);
      b.clipTo(a);
      b.invert();
      b.clipTo(a);
      b.invert();
      a.build(b.allPolygons());
      // Scale-relative, volume-validated cancellation of near-coincident opposite-wall
      // pairs the ABSOLUTE on-plane band (kPlaneEps) left doubled (dense-CSG Band 2).
      // A no-op unless a genuine internal air-slab boundary is present; see bsp.h.
      return cancelNearCoincidentWalls(a.allPolygons());
    }
    case Op::Cut: {
      a.invert();
      a.clipTo(b);
      b.clipTo(a);
      b.invert();
      b.clipTo(a);
      b.invert();
      a.build(b.allPolygons());
      a.invert();
      return cancelNearCoincidentWalls(a.allPolygons());
    }
    case Op::Common: {
      // A ∩ B = ¬(¬A ∪ ¬B).
      a.invert();
      b.clipTo(a);
      b.invert();
      a.clipTo(b);
      b.clipTo(a);
      a.build(b.allPolygons());
      a.invert();
      return cancelNearCoincidentWalls(a.allPolygons());
    }
  }
  return {};
}

// ─────────────────────────────────────────────────────────────────────────────
// boolean_solid — the AGREED entry the engine glue calls. Fuse/cut/common two
// native solids whose faces are all planar. Returns the assembled native Solid, or
// a NULL Shape when:
//   * either input is null / not a solid-like shape with planar faces (curved-face
//     solids → NULL → engine falls through to OCCT), or
//   * the op produces fewer than a closeable set of faces (degenerate — e.g. two
//     disjoint solids under `common`, or a cut that removes everything).
//
// The engine MUST self-verify (watertight + correct-volume) before accepting a
// non-null result; see native_engine.cpp boolean_op.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape boolean_solid(const topo::Shape& a, const topo::Shape& b, Op op) {
  if (a.isNull() || b.isNull()) return {};

  // ── Curved slice: axis-aligned box ⟷ axis-parallel cylinder (analytic) ────────
  // Try the narrow curved-boolean path FIRST (one operand curved). It recognises
  // the box/cylinder configuration analytically and builds a true-B-rep result
  // (Cylinder/Circle/Plane faces). A NULL means "not in the analytic family" and we
  // fall through to the planar path (which itself gates on isAllPlanar). The engine
  // self-verify (watertight + analytic volume) still guards whatever comes back.
  if (topo::Shape curvedResult = curved::tryBoxCylinder(a, b, static_cast<int>(op));
      !curvedResult.isNull())
    return curvedResult;

  // ── SSI Stage S5-a: the GENERAL, SSI-curve-driven curved boolean ──────────────
  // When the analytic box∩cylinder pattern did not match, try the transversal
  // elementary curved path (cyl∩cyl / sphere∩box / cone∩box …). It drives the split
  // from the S3 TraceSet instead of matching a primitive, so it generalises across
  // the elementary family. A NULL means "not a robustly-handleable transversal
  // elementary pair" (near-tangent / coincident / freeform / out-of-scope op) and we
  // fall through to the planar path / OCCT. The engine self-verify still guards it.
  // Substrate-gated: without CYBERCAD_HAS_NUMSCI this links a NULL-returning stub.
  if (topo::Shape ssiResult = ssi_boolean_solid(a, b, op); !ssiResult.isNull())
    return ssiResult;

  // ── Planar polyhedron path (both operands all-planar) ─────────────────────────
  if (!isAllPlanar(a) || !isAllPlanar(b)) return {};  // curved → OCCT fallthrough

  std::vector<Polygon> aPolys = extractPolygons(a);
  std::vector<Polygon> bPolys = extractPolygons(b);
  if (aPolys.size() < 4 || bPolys.size() < 4) return {};

  std::vector<Polygon> result = booleanPolygons(std::move(aPolys), std::move(bPolys), op);
  return assembleSolid(result);
}

/// int-op overload matching the cc_boolean / IEngine::boolean_op contract
/// (0 fuse, 1 cut a−b, 2 common). An out-of-range op returns NULL.
inline topo::Shape boolean_solid(const topo::Shape& a, const topo::Shape& b, int op) {
  if (op < 0 || op > 2) return {};
  return boolean_solid(a, b, static_cast<Op>(op));
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_H
