// SPDX-License-Identifier: Apache-2.0
//
// step_reader.h — native ISO-10303-21 (STEP AP203) READER: the deterministic
// inverse of step_writer.cpp (the first native STEP IMPORT slice).
//
// SCOPE (honest, narrow — the exact inverse of the writer's alphabet):
//   * A Part-21 tokenizer + entity table over the DATA section: records
//     `#N = NAME(args);` with refs (#M), integers, reals (typed forms 1., 1.E2,
//     -3.5E-07), single-quoted strings ('' un-doubled), enums (.T./.PLANE.),
//     lists (...), $ (null), * (derived), and the combined-instance
//     ( SUB(...) SUB(...) ) unit/context form.
//   * A two-pass AP203 mapper → native B-rep: leaf geometry (CARTESIAN_POINT →
//     Point3, AXIS2_PLACEMENT_3D → Ax3, LINE/CIRCLE/B_SPLINE_CURVE → EdgeCurve,
//     PLANE/CYLINDRICAL/CONICAL/SPHERICAL/B_SPLINE_SURFACE → FaceSurface) then
//     topology (VERTEX_POINT → makeVertex, EDGE_CURVE → makeEdgeWithVertices,
//     ORIENTED_EDGE → oriented edge, EDGE_LOOP → wire, ADVANCED_FACE → face,
//     CLOSED_SHELL → shell, MANIFOLD_SOLID_BREP → solid), reusing the
//     writer-shared EDGE_CURVE/VERTEX_POINT dedup by #id and reconstructing the
//     analytic PCURVEs the tessellator needs (STEP carries no pcurve).
//
// The reconstructed solid is a face-graph that shares vertex+edge NODES by #id
// exactly as the writer shared them, so it re-tessellates WATERTIGHT by the same
// shared-edge weld path a native prism uses — no healing needed for a native
// round-trip. If the shared-node reconstruction still leaves a gap, healShell is
// applied as a fallback (planar-only; a curved solid that fails to reconstruct
// declines to OCCT rather than being planarized).
//
// DECLINE (returns a NULL Shape → engine falls to OCCT, never fabricates
// geometry): any unsupported entity/surface keyword, rational/weighted B-spline,
// assembly / >1 root, non-mm unit context, malformed record, or a reconstruction
// that does not self-verify watertight.
//
// OCCT-FREE. Declaration here; body in step_reader.cpp. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_EXCHANGE_STEP_READER_H
#define CYBERCAD_NATIVE_EXCHANGE_STEP_READER_H

#include "native/topology/shape.h"

#include <string>

namespace cybercad::native::exchange {

namespace topo = cybercad::native::topology;

/// Parse the STEP AP203 text `content` and build a native B-rep Solid, or return a
/// NULL Shape if the file is out of the writer's scope / cannot be reconstructed to
/// a valid watertight solid. Never fabricates geometry.
topo::Shape readStepString(const std::string& content);

/// Read the STEP file at `path` and build a native B-rep Solid, or NULL on any
/// failure (missing file, out of scope, unhealable). The engine self-verifies the
/// result and falls back to OCCT on NULL.
topo::Shape readStepFile(const std::string& path);

}  // namespace cybercad::native::exchange

#endif  // CYBERCAD_NATIVE_EXCHANGE_STEP_READER_H
