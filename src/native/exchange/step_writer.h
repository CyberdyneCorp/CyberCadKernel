// SPDX-License-Identifier: Apache-2.0
//
// step_writer.h — native ISO-10303-21 (STEP AP203/AP214) exporter (Phase 4,
// capability #7 `native-data-exchange`, the EXPORT slice).
//
// Clean-room from the STEP standard:
//   * ISO 10303-21 — the exchange-file structure (HEADER + DATA sections, the
//     `#id = ENTITY(args);` instance syntax, and the string/number encoding).
//   * ISO 10303-42 — the geometry/topology entity model (CARTESIAN_POINT,
//     DIRECTION, AXIS2_PLACEMENT_3D, the curves/surfaces, the B-rep topology
//     VERTEX_POINT → EDGE_CURVE → ORIENTED_EDGE → EDGE_LOOP → FACE_BOUND →
//     ADVANCED_FACE → CLOSED_SHELL → MANIFOLD_SOLID_BREP) and the
//     ADVANCED_BREP_SHAPE_REPRESENTATION + PRODUCT/context wrapper (AP203/214).
//
// OCCT `STEPControl_Writer` / `STEPCAFControl` were consulted as a REFERENCE
// ORACLE ONLY (entity naming, the product/context boilerplate a conformant AP203
// file carries, the unit/uncertainty declarations) — nothing is copied verbatim.
//
// SCOPE (honest, narrow — see openspec/NATIVE-REWRITE.md #7). This walks a NATIVE
// topology::Shape (a Solid built by the native construct/boolean/blend libraries)
// and emits a valid AP203 STEP file in true MILLIMETRES. It serialises exactly the
// entity kinds the native kernel produces:
//   vertex                       → VERTEX_POINT
//   Line edge                    → LINE + EDGE_CURVE
//   Circle edge                  → CIRCLE + EDGE_CURVE
//   BSpline edge                 → B_SPLINE_CURVE_WITH_KNOTS + EDGE_CURVE
//   wire (edge loop)             → EDGE_LOOP via ORIENTED_EDGE
//   Plane face                   → PLANE      + ADVANCED_FACE
//   Cylinder face                → CYLINDRICAL_SURFACE + ADVANCED_FACE
//   Cone face                    → CONICAL_SURFACE     + ADVANCED_FACE
//   Sphere face                  → SPHERICAL_SURFACE   + ADVANCED_FACE
//   BSpline face                 → B_SPLINE_SURFACE_WITH_KNOTS + ADVANCED_FACE
//   shell                        → CLOSED_SHELL
//   solid                        → MANIFOLD_SOLID_BREP
//   + ADVANCED_BREP_SHAPE_REPRESENTATION + PRODUCT / PRODUCT_DEFINITION /
//     APPLICATION_CONTEXT boilerplate + SI_UNIT(mm) geometric context.
//
// `canSerialize(solid)` reports whether every face carries one of the surface
// kinds above and every edge one of the curve kinds above; the engine calls it to
// decide native-vs-OCCT-fallback (an Ellipse/Bezier edge, a Bezier surface, or a
// null-geometry face is out of scope → fall back to OCCT, never faked).
//
// The correctness gate (openspec/NATIVE-REWRITE.md): the emitted file MUST re-read
// through OCCT STEPControl_Reader to the SAME solid (volume/bbox/topology within
// tolerance). Verified on the simulator; a solid that cannot be faithfully
// serialised falls through to OCCT rather than emit a wrong/invalid STEP file.
//
// OCCT-FREE. Uses only src/native/{topology,math}. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_EXCHANGE_STEP_WRITER_H
#define CYBERCAD_NATIVE_EXCHANGE_STEP_WRITER_H

#include "native/topology/native_topology.h"

#include <string>

namespace cybercad::native::exchange {

namespace topo = cybercad::native::topology;

/// True iff `solid` is a native B-rep whose every face surface and every edge
/// curve is one of the kinds the writer can serialise (Plane/Cylinder/Cone/
/// Sphere/BSpline surfaces; Line/Circle/BSpline curves). A shape with an
/// unsupported geometry kind (Ellipse/Bezier curve, Bezier surface) or a face/edge
/// with no attached geometry returns false — the engine then falls back to OCCT.
/// A null shape or a shape that is not a Solid/Shell returns false.
bool canSerialize(const topo::Shape& solid);

/// Serialise `solid` to an ISO-10303-21 STEP AP203 file at `path` (true mm).
/// Returns true on success. Returns false (writing nothing) if the shape cannot be
/// serialised (see canSerialize) or the file cannot be opened. `productName` names
/// the PRODUCT entity (defaults to a stable placeholder). This is a pure text emit;
/// it does not modify the topology graph.
bool writeStepFile(const topo::Shape& solid, const std::string& path,
                   const std::string& productName = "CyberCadKernel_part");

/// Serialise `solid` to an in-memory STEP string (same content writeStepFile
/// writes to disk). Empty string if the shape cannot be serialised. Exposed for
/// host unit tests that assert on the entity graph without touching the filesystem.
std::string writeStepString(const topo::Shape& solid,
                            const std::string& productName = "CyberCadKernel_part");

}  // namespace cybercad::native::exchange

#endif  // CYBERCAD_NATIVE_EXCHANGE_STEP_WRITER_H
