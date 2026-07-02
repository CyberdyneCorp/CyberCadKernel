// SPDX-License-Identifier: Apache-2.0
//
// native_tessellate.h — public aggregate header for the native tessellator
// (Phase 4, capability #3 `native-tessellation`).
//
// Clean-room, OCCT-FREE triangulation of a native B-rep into a triangle Mesh,
// built on the verified native foundations:
//   * src/native/math      — surface point / dU / dV / normal / curvature
//                            (bezier / bspline / nurbs / elementary).
//   * src/native/topology  — Face → surface + ordered outer/inner wires +
//                            pcurves, Explorer, BRep_Tool accessors.
// The tessellator NEVER sees OCCT; it compiles with `clang++ -std=c++20`. OCCT's
// BRepMesh / IMeshTools are a REFERENCE ORACLE only (see NATIVE-REWRITE.md).
//
// ── API SURFACE ───────────────────────────────────────────────────────────────
//   mesh.h          Mesh (fp64 verts, int tris, optional normals) + properties:
//                   surfaceArea, enclosedVolume, isWatertight, isTwoManifold.
//   surface_eval.h  SurfaceEvaluator — uniform (u,v) value/d1/normal/curvature
//                   over every FaceSurface variant, world-placed.
//   edge_mesher.h   EdgeCache — STAGE 1: shared per-edge 1D discretization. Each
//                   unique topological edge is discretized ONCE into a deflection-
//                   based fraction list; both adjacent faces reuse it (the seam
//                   that makes CURVED shared edges weld watertight).
//   trim.h          UVRegion — face wires flattened to UV polygons (via pcurves)
//                   + point-in-polygon keep test (outer ∧ ¬holes); shared-fraction
//                   wire flattener (appendEdgeSamplesAtFracs) for STAGE 2.
//   uv_triangulate.h Bowyer-Watson Delaunay in (u,v) — triangulates the fixed
//                   boundary samples + interior grid points (systems band).
//   face_mesher.h   FaceMesher — STAGE 2: boundary-conforming triangulation of one
//                   trimmed face (boundary pinned to the shared edge samples;
//                   interior grid deflection-driven; vertices ON the true surface).
//   solid_mesher.h  SolidMesher — mesh every face with ONE shared EdgeCache +
//                   weld/stitch → watertight mesh for a closed solid (incl. curved
//                   shared edges); VertexWelder (spatial hash merge).
//   gpu_sample.h    OPTIONAL fp32 GPU sampling for eligible free-form faces
//                   (Phase-2 evaluator); CPU fp64 is the source of truth.
//   MeshParams      deflection bound + division clamps + wire sampling density.
//
// ── DEFLECTION METRIC (the correctness knob) ─────────────────────────────────
// The chord (sagitta) error of a smooth patch over a parameter step Δ is
//   d ≈ ⅛·‖S″‖·Δ²  ⇒  Δ ≤ √(8·deflection/‖S″‖).
// The face mesher probes ‖Sᵤᵤ‖,‖Sᵥᵥ‖ on a 3×3 UV lattice, takes the worst, and
// sizes the per-direction division count from it (clamped to [minDiv,maxDiv]).
// A plane collapses to the minimum grid (exact); a tight blend subdivides until
// every produced vertex lies within `deflection` of the true surface. Because
// vertices are S(u,v) (never interpolated in 3D), "vertices on the true surface"
// holds exactly up to fp round-off, and area/volume converge as deflection → 0.
//
// ── TRIMMING ─────────────────────────────────────────────────────────────────
// Each face wire is flattened to a UV polygon by sampling the edges' pcurves;
// child 0 is the outer loop, the rest are holes. A grid triangle is kept iff its
// UV centroid is inside the outer polygon and outside every hole polygon
// (even-odd point-in-polygon). Holes are carved; the outer silhouette follows
// the sampled boundary within deflection.
//
// ── STITCHING (watertightness) — TWO-STAGE, curved edges included ────────────
// Faces share edge NODES in the topology. The mesher is a two-stage pipeline that
// mirrors OCCT BRepMesh:
//   STAGE 1 (edge_mesher.h): each UNIQUE edge is discretized ONCE into a shared
//     list of parameter fractions f∈[0,1], sized from the edge's 3D-curvature
//     deflection bound, cached by edge TShape identity.
//   STAGE 2 (face_mesher.h): each face pins its BOUNDARY vertices to those shared
//     fractions (mapped to (u,v) through the face's own pcurve) and fills its
//     interior with the deflection-driven parameter grid, Delaunay-triangulated to
//     the fixed boundary. Because a pcurve satisfies S_face(pcurve(f)) = C_edge(f),
//     the two faces sharing an edge produce the SAME 3D boundary points — so even a
//     CURVED shared edge (cylinder cap↔side circle, fillet-blend seams) coincides.
// The SolidMesher then welds vertices within a weld tolerance (a fraction of the
// deflection) via a spatial hash grid, fusing the shared boundary so every interior
// mesh edge is used by exactly two triangles. A closed solid ⇒ isWatertight() and a
// signed enclosedVolume that converges to the true volume. (The prior independent-
// grid path only welded STRAIGHT shared edges; curved ones were left open. FIXED.)
//
// ── VERIFICATION MODEL (two gates, NATIVE-REWRITE.md) ─────────────────────────
// Tessellation is an APPROXIMATION — the gates assert TOLERANCE-BASED PROPERTIES,
// never triangle-identical output vs OCCT:
//   Gate 1 (host, no OCCT): analytic properties — a plane meshes to its exact
//     area; a box/cylinder/sphere mesh is watertight and its area/volume are
//     within a deflection-derived bound of the closed-form value and converge as
//     deflection shrinks; every vertex lies on the analytic surface; a holed
//     face drops triangles inside the hole. (tests/native/test_native_tessellate.cpp)
//   Gate 2 (sim, OCCT oracle): on identical shapes bridged from OCCT (the same
//     TEST-ONLY OCCT→native bridge pattern as native_topology_parity.mm, kept in
//     the harness — never in src/native), the native mesh satisfies: max vertex
//     distance to the OCCT surface ≤ deflection; area/volume within tolerance of
//     BRepGProp on the OCCT shape; watertight for a closed solid.
//
// ── COMPLEXITY (systems band, flagged) ───────────────────────────────────────
// The irreducible geometry — surface eval, the weld grid, the edge-curve Visitor
// switch (edge_mesher edgeCurveLocal), and the ear-clip driver (uv_triangulate
// earClip, ~11 after extracting isEar) — sits in the 10–15 range and is documented
// in place; the driver mesh()/discretize() functions delegate to helpers and stay
// ≤ ~10 (measured with the cognitive-complexity skill). No function is mangled to
// hit a number.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_H
#define CYBERCAD_NATIVE_TESSELLATE_H

#include "native/tessellate/mesh.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/tessellate/face_mesher.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/gpu_sample.h"

/// The entire native tessellator API lives in this namespace.
namespace cybercad::native::tessellate {}

#endif  // CYBERCAD_NATIVE_TESSELLATE_H
