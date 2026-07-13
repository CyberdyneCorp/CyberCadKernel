// SPDX-License-Identifier: Apache-2.0
//
// step_brep.h — native, OCCT-FREE ISO-10303-21 (STEP AP214) round-trip for the
// EXACT trimmed-NURBS B-rep (topology::TrimmedNurbsFace).
//
// The engine's existing native STEP writer/reader (step_writer.h / step_reader.h)
// walks a topology::Shape SOLID and is deliberately NON-rational and pcurve-free
// (it reconstructs analytic pcurves the tessellator needs; a weighted/rational
// spline falls back to OCCT). That path cannot serialise the LAYER-8 exact model —
// a set of topology::TrimmedNurbsFace, each a (possibly RATIONAL) B-spline surface
// bounded by trim LOOPS of 2-D PCURVES in the surface's (u,v) plane.
//
// THIS module is the missing exact-NURBS-B-rep exchange: it writes a set of
// TrimmedNurbsFace directly to a valid AP214 Part-21 file and reads it back, EXACT
// for the NURBS data (poles / knots / weights / trims recovered to ≤ 1e-9). It is a
// self-contained Part-21 emitter + parser (pure std::string; no dependency on the
// big writer/reader internals), so the trimmed-NURBS exact kernel becomes
// interoperable without OCCT.
//
// STEP entities emitted (ISO 10303-42 geometry + AP214 topology wrapper):
//   surface (non-rational)  → B_SPLINE_SURFACE_WITH_KNOTS
//   surface (rational)      → ( B_SPLINE_SURFACE B_SPLINE_SURFACE_WITH_KNOTS
//                               BOUNDED_SURFACE GEOMETRIC_REPRESENTATION_ITEM
//                               RATIONAL_B_SPLINE_SURFACE REPRESENTATION_ITEM
//                               SURFACE ) — the AP214 complex (rational) instance
//                               carrying the WEIGHTS grid.
//   analytic surface        → PLANE / CYLINDRICAL_SURFACE / CONICAL_SURFACE /
//                             SPHERICAL_SURFACE / TOROIDAL_SURFACE.
//   pcurve (2-D, in (u,v))  → the same B_SPLINE_CURVE_WITH_KNOTS / rational complex
//                             form (2-D poles as CARTESIAN_POINT((u,v))), or the
//                             analytic LINE / CIRCLE / ELLIPSE, wrapped in a
//                             PCURVE(surface, DEFINITIONAL_REPRESENTATION) — the
//                             STEP entity that pins a 2-D curve to a surface's plane.
//   3-D edge curve          → the SAME kind, its poles = S(pcurve(t)) sampled on the
//                             surface (an EDGE_CURVE needs a 3-D geometry). It is
//                             informational for viewers; the EXACT trim is recovered
//                             from the PCURVE, never from the sampled 3-D curve.
//   loop / face / shell     → ORIENTED_EDGE → EDGE_LOOP → FACE_BOUND / FACE_OUTER_BOUND
//                             → ADVANCED_FACE → (CLOSED|OPEN)_SHELL, plus the
//                             ADVANCED_BREP_SHAPE_REPRESENTATION + PRODUCT / context /
//                             SI_UNIT(mm) boilerplate a conformant AP214 file carries.
//
// HONEST-DECLINE (never emit invalid STEP): a face whose surface or a trim pcurve is
// a kind this exact writer cannot represent (e.g. an empty free-form with no knots)
// makes writeStepBrep return an EMPTY string — the caller declines rather than ships
// a malformed file. The round-trip is EXACT or it declines; a tolerance is NEVER
// widened.
//
// OCCT-FREE. Uses only src/native/{topology,math} + the C++20 stdlib string/parse.
// clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_EXCHANGE_STEP_BREP_H
#define CYBERCAD_NATIVE_EXCHANGE_STEP_BREP_H

#include "native/topology/trimmed_nurbs.h"

#include <string>
#include <vector>

namespace cybercad::native::exchange {

namespace topo = cybercad::native::topology;

/// True iff every surface + every trim pcurve of `face` is a kind the exact writer
/// can serialise (analytic Plane/Cylinder/Cone/Sphere/Torus surface, or a knotted
/// B-spline/rational surface; Line/Circle/Ellipse or knotted B-spline/rational
/// pcurves). A face with an empty free-form surface (no knots) or an unrepresentable
/// pcurve returns false — writeStepBrep then declines (empty string) rather than
/// emit invalid STEP.
bool canWriteStepBrep(const topo::TrimmedNurbsFace& face);

/// Serialise `faces` to a valid ISO-10303-21 STEP AP214 Part-21 string (true mm).
/// Each surface becomes a (rational) B-spline / analytic surface with knots; each
/// trim loop's pcurves become PCURVE-wrapped (rational) B-spline / analytic 2-D
/// curves over 3-D EDGE_CURVEs, wrapped in ADVANCED_FACE / FACE_BOUND / EDGE_LOOP and
/// a shell. Returns an EMPTY string if ANY face is out of scope (honest decline) or
/// the set is empty. Deterministic (stable #id numbering).
std::string writeStepBrep(const std::vector<topo::TrimmedNurbsFace>& faces);

/// Parse a STEP AP214 string produced by writeStepBrep back into a set of
/// TrimmedNurbsFace, EXACT for the NURBS data (poles/knots/weights/trims recovered to
/// ≤ 1e-9). Returns an EMPTY vector if the DATA section is malformed, an entity
/// reference does not resolve, or a required entity kind is absent. Never fabricates
/// geometry.
std::vector<topo::TrimmedNurbsFace> readStepBrep(const std::string& step);

// ─────────────────────────────────────────────────────────────────────────────
// GENERAL (external) AP203/214 IMPORT — readStepBrepExternal.
//
// readStepBrep above recovers ONLY the entity forms OUR writer emits (it keys the
// exact trim off the AP214-extension CC_TRIM records). A STEP file exported by a
// real CAD system uses the FULL AP203/214 entity set — analytic surfaces
// (PLANE / CYLINDRICAL_SURFACE / CONICAL_SURFACE / SPHERICAL_SURFACE /
// TOROIDAL_SURFACE), knotted / rational B-spline surfaces, and a topology graph of
// ADVANCED_FACE → FACE_OUTER_BOUND / FACE_BOUND → EDGE_LOOP → ORIENTED_EDGE →
// EDGE_CURVE(→ LINE / CIRCLE / B_SPLINE_CURVE_WITH_KNOTS) → VERTEX_POINT, with NO
// CC_TRIM and NO PCURVE — the trim is IMPLIED by the 3-D edge geometry lying on the
// surface. readStepBrepExternal imports that general file into TrimmedNurbsFaces:
//   * Surfaces are recovered as the native analytic FaceSurface kinds (exact) or a
//     knotted / rational B-spline surface.
//   * Each trim loop's 2-D pcurve is DERIVED by inverting the 3-D edge curve into the
//     surface's (u,v) parameter plane (closed-form analytic inverse for the analytic
//     surfaces; sampled projection otherwise) and building a 2-D pcurve — so the
//     imported face carries a correct trim region.
//   * Robust to arbitrary entity ORDERING, FORWARD `#id` references (two-pass: build
//     the table, then resolve), comments / whitespace, and unit / context boilerplate
//     (skipped cleanly).
//
// HONEST-DECLINE: a face whose surface or an edge is a kind this importer cannot
// represent is SKIPPED (never emitted as a wrong face); the skipped-face reasons are
// reported (optional out-param) so a caller sees exactly what was declined. A file with
// no importable face returns an EMPTY vector. Geometry is NEVER fabricated.
struct ExternalImportReport {
  int facesSeen = 0;              ///< ADVANCED_FACE records encountered
  int facesImported = 0;         ///< faces successfully converted to a TrimmedNurbsFace
  int facesSkipped = 0;          ///< faces honestly declined
  std::vector<std::string> skipReasons;  ///< one human-readable reason per skipped face
};

/// Import a GENERAL external AP203/214 part21 string into TrimmedNurbsFaces (see above).
/// `report` (optional) receives the per-face import/skip accounting. Returns the imported
/// faces; a face whose surface or edge kind is unrepresentable is skipped (honest decline),
/// not fabricated. An empty / malformed file, or one with no importable face, returns {}.
std::vector<topo::TrimmedNurbsFace> readStepBrepExternal(const std::string& step,
                                                         ExternalImportReport* report = nullptr);

}  // namespace cybercad::native::exchange

#endif  // CYBERCAD_NATIVE_EXCHANGE_STEP_BREP_H
