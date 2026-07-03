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
#include "native/boolean/polygon.h"
#include "native/topology/native_topology.h"

#include <vector>

namespace cybercad::native::boolean {

/// The three set operations, matching the cc_boolean op codes.
enum class Op { Fuse = 0, Cut = 1, Common = 2 };

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
      return a.allPolygons();
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
      return a.allPolygons();
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
      return a.allPolygons();
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
