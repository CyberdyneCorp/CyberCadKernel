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
//     PLANE/CYLINDRICAL/CONICAL/SPHERICAL/TOROIDAL/B_SPLINE_SURFACE → FaceSurface) then
//     topology (VERTEX_POINT → makeVertex, EDGE_CURVE → makeEdgeWithVertices,
//     ORIENTED_EDGE → oriented edge, EDGE_LOOP → wire, ADVANCED_FACE → face,
//     CLOSED_SHELL → shell, MANIFOLD_SOLID_BREP → solid), reusing the
//     writer-shared EDGE_CURVE/VERTEX_POINT dedup by #id and reconstructing the
//     analytic PCURVEs the tessellator needs (STEP carries no pcurve). A FULL
//     periodic surface OCCT emits with a VERTEX_LOOP bound (a single degenerate
//     pole vertex, NO edges — e.g. a whole SPHERICAL_SURFACE or on-axis-circle
//     SURFACE_OF_REVOLUTION sphere) maps to a native Kind::Sphere face with a NULL
//     outer wire, meshed watertight over its natural (u∈[0,2π], v∈[−π/2,π/2])
//     bounds (seam + both poles welded). A full TORUS (a TOROIDAL_SURFACE face, or an
//     off-axis-circle SURFACE_OF_REVOLUTION) is DOUBLY periodic with NO pole: OCCT bounds
//     it with a FULLY-SEAMED EDGE_LOOP (the equator v-seam + the tube u-seam, each used
//     forward AND reversed). That carries no real trim, so — like the sphere VERTEX_LOOP —
//     it maps to a native Kind::Torus face with a NULL outer wire, meshed watertight over
//     its natural (u,v)∈[0,2π]² bounds (BOTH seams welded, no poles). A VERTEX_LOOP on any
//     non-sphere surface, or a sphere/torus with surviving real trim edges (a partial
//     zone), keeps the honest OCCT deferral.
//
// The reconstructed solid is a face-graph that shares vertex+edge NODES by #id
// exactly as the writer shared them, so it re-tessellates WATERTIGHT by the same
// shared-edge weld path a native prism uses — no healing needed for a native
// round-trip. If the shared-node reconstruction still leaves a gap, healShell is
// applied as a fallback (planar-only; a curved solid that fails to reconstruct
// declines to OCCT rather than being planarized).
//
// ASSEMBLIES: a product placement (CONTEXT_DEPENDENT_SHAPE_REPRESENTATION →
// REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION) reaching a MANIFOLD_SOLID_BREP is
// composed into a placed Compound. The placement may be SINGLE-level OR MULTI-level
// (nested): a component that is itself an assembly is modelled as a parent-edge forest over
// shape-representations (each placing CDSR is one childSr → parentSr edge), and each leaf's
// world placement is the chain product W = T_root ∘ … ∘ T_leaf walked from the leaf's child
// representation up to a UNIQUE root (a length-1 chain reproduces the single-level placement
// exactly). Each level's transform may be RIGID (an ITEM_DEFINED_TRANSFORMATION AXIS2 frame
// pair) or a UNIFORM-SCALE / MIRROR (a CARTESIAN_TRANSFORMATION_OPERATOR_3D — the entity that
// can carry a scale/reflection); the COMPOSED W is re-classified conformal (rigid /
// uniform-scale / mirror), and a mirror's faces are orientation-complemented so it meshes
// outward/watertight. AP242 PMI / GD&T / annotation entities (and their relationship graph
// that does NOT reach a brep) are SKIPPED — the geometry imports, the PMI is dropped.
//
// The nested walk DECLINES → OCCT (never a mis-placed solid) on a CYCLE in the parent-edge
// forest, an AMBIGUOUS child (one shape-representation placed into two distinct parents = a
// shared sub-assembly instanced twice), a DANGLING / unreadable level transform, or a
// NON-CONFORMAL composed W. A Form-B MAPPED_ITEM / REPRESENTATION_MAP is still declined.
//
// DECLINE (returns a NULL Shape → engine falls to OCCT, never fabricates
// geometry): any unsupported entity/surface keyword, rational/weighted B-spline, a
// NON-uniform-scale / shear component transform, a Form-B MAPPED_ITEM or lone
// assembly-usage with no composable placement, non-mm unit context, malformed record,
// or a reconstruction that does not self-verify watertight. A GENERAL revolution whose
// generatrix is an ELLIPSE or a (non-rational) B_SPLINE_CURVE touching the axis at both
// ends now imports NATIVELY: the profile meridian is revolved into the EXACT rational
// tensor-product B-spline (Piegl & Tiller A7.1 — u = the standard rational-quadratic full
// circle; v = the ellipse promoted to two exact rational-quadratic 90° arcs, or the
// B-spline profile used directly) and stored as a native Kind::BSpline face WITH weights,
// bounded by the same VERTEX_LOOP bare-periodic path as the sphere and meshed watertight
// over its natural (u∈[0,2π], v=profile) bounds (u-seam welded, axis poles collapsed). A
// tilted/off-axis ellipse, a rational STEP profile, or a PARTIAL torus (a TOROIDAL_SURFACE
// / off-axis revolution carrying real trim edges) has no faithful native mesh path and
// DECLINES → OCCT (they already import fine there).
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
