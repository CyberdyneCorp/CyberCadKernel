// SPDX-License-Identifier: Apache-2.0
//
// native_construct.h — public aggregate header for the native swept-solid
// construction library (Phase 4, capability #4 `native-construction`).
//
// Clean-room, OCCT-FREE builders that assemble a native B-rep
// (src/native/topology) with native-math elementary geometry (src/native/math)
// for the two solid operations the native engine implements today. This header
// is the AGREED API surface between the construction work and the engine glue
// (src/engine/native): the engine calls these free functions and wraps the
// returned topology::Shape into an EngineShape.
//
// ── API SURFACE (all in namespace cybercad::native::construct) ────────────────
//   LineSeg / RevolveAxis            input POD (a straight profile segment;
//                                    the revolution axis point + direction).
//   build_prism(profileXY,count,d)   extrude a CLOSED planar polygon profile
//                                    (x,y pairs on z=0) by +depth into a prism
//                                    SOLID: bottom + top planar caps + one planar
//                                    quad side face per profile edge. Full native
//                                    topology + geometry.
//   build_revolution(segs,axis,ang)  revolve a LINE-SEGMENT profile about an
//                                    in-plane axis. classification helper
//                                    detail::segmentSurface picks the analytic
//                                    surface per segment: axis-parallel → Cylinder,
//                                    perpendicular → planar annulus/disk, oblique →
//                                    Cone. A full turn (≥2π) closes the solid; a
//                                    partial turn additionally caps the two open
//                                    ends with planar profile faces.
//   build_revolution(profileXY,...)  convenience overload taking a raw closed
//                                    polygon point loop (the cc_solid_revolve path).
//
//   ── Tier-A (#4b, profile.h) ──────────────────────────────────────────────────
//   ProfileSegment / CircleHole      typed profile segment (kind 0 line / 1 arc /
//                                    2 full circle / 3 spline) + a circular hole POD.
//   build_prism_with_holes(...)      extrude an OUTER polygon with circular +
//                                    polygon HOLES: both caps carry the hole wires
//                                    and every loop grows its own side ring; circular
//                                    holes keep a TRUE Circle edge + Cylinder wall.
//   build_prism_profile(...)         extrude a TYPED profile (line/arc/full-circle)
//                                    with circular + polygon holes. kind-3 spline →
//                                    NULL Shape (OCCT fallthrough).
//   build_revolution_profile(...)    revolve a TYPED profile: line → Plane/Cylinder/
//                                    Cone, arc centred ON the axis → Sphere band. An
//                                    arc OFF the axis (Torus) / spline → NULL (OCCT).
//
//   ── Tier-B (#4b, loft.h) ─────────────────────────────────────────────────────
//   build_ruled_loft(secA,secB)      skin TWO closed section loops with EQUAL vertex
//                                    counts into a watertight ruled SOLID: one
//                                    BILINEAR (degree-1 Bézier) side face per
//                                    corresponding edge pair + two planar caps.
//                                    Requires both sections PLANAR & non-degenerate;
//                                    mismatched counts / non-planar / point-collapse
//                                    → NULL (OCCT fallthrough). Mirrors ruled
//                                    BRepOffsetAPI_ThruSections.
//   build_loft(botXY,topXY,depth)    entry for cc_solid_loft: bottom profile at z=0,
//                                    top at z=depth, then build_ruled_loft.
//   build_loft_wires(aXYZ,bXYZ)      entry for cc_solid_loft_wires: the two 3D wires
//                                    directly, then build_ruled_loft.
//
//   ── Tier-C (#4b, sweep.h) ────────────────────────────────────────────────────
//   build_sweep(profileXY,pathXYZ)   sweep a CLOSED planar profile along a 3D
//                                    polyline path: profile centred on its centroid,
//                                    placed perpendicular to the START tangent (local
//                                    x = cross(tan,+Y), local y = +Y), transported
//                                    with a rotation-minimizing frame (double
//                                    reflection). NATIVE for a STRAIGHT spine → an
//                                    EXACT directional prism (profile area × path
//                                    length), AND for a SMOOTH CURVED spine → an
//                                    RMF-transported ruled-band tube (deflection-bounded
//                                    vs OCCT, watertight at working deflections). A
//                                    TIGHT-CURVATURE / self-intersecting spine (turning
//                                    radius < profile circumradius, or a too-sharp turn)
//                                    returns NULL → OCCT MakePipe (guarded, not faked).
//                                    Degenerate profile / < 2 path points → NULL.
//   build_twisted_sweep(...,tw,sc)   NATIVE only when it reduces to the plain sweep
//                                    (twist ≈ 0, scale ≈ 1) → forwards to build_sweep
//                                    (straight or smooth curved); any real twist/scale
//                                    → NULL (OCCT). cc_twisted_sweep.
//
// ── SUPPORTED vs DEFERRED (honest — see openspec/NATIVE-REWRITE.md) ───────────
//   SUPPORTED natively:
//     * extrude of a closed polygon profile → prism solid.
//     * revolve of a LINE-SEGMENT profile (segment → plane / cylinder / cone
//       face of revolution), full 360° or partial with planar side caps.
//   NOW NATIVE (Tier-B #4b, loft.h): 2-section RULED loft with EQUAL vertex counts
//   and planar sections (cc_solid_loft / cc_solid_loft_wires) → bilinear side faces
//   + planar caps → watertight solid.
//   DEFERRED (the builder returns a NULL Shape so the engine falls through to
//   OCCT — it NEVER fakes a wrong shape):
//     * loft with MISMATCHED section counts / a NON-PLANAR section / 3+ sections /
//       guided or rail loft (Tier C); guided sweep, loft-along-rail, threads.
//     * a TIGHT-CURVATURE / self-intersecting sweep spine, or a TWISTED/scaled sweep
//       (build_sweep is native for straight + smooth curved; build_twisted_sweep is
//       native only for the plain no-twist sweep).
//     * kind-3 SPLINE profile edges (extrude and revolve).
//     * arc-revolve whose circle centre is OFF the axis (a Torus surface of
//       revolution — no native Torus surface yet).
//     * degenerate input (< 3 pts / zero area / depth ≤ 0 / angle ≤ 0).
//   NOW NATIVE (Tier-A #4b, was deferred at #4): holed extrude (circular + polygon
//   holes), typed line/arc/full-circle profile extrude, and typed-profile revolve
//   for line segments + on-axis arc (Sphere) — see profile.h.
//
// ── VERIFICATION MODEL (two gates, NATIVE-REWRITE.md) ─────────────────────────
//   Gate 1 (host, no OCCT): analytic properties of known solids via the native
//     tessellator — a unit box meshes watertight with volume 2 / area 10; a cone
//     profile revolved 360° has volume 4π; a rectangle revolved 360° is a tube of
//     volume 9π; a partial turn is watertight with the fractional volume.
//     (tests/test_native_engine.cpp)
//   Gate 2 (sim, OCCT oracle): native solid_extrude / solid_revolve compared
//     against BRepPrimAPI_MakePrism / BRepPrimAPI_MakeRevol at sampled inputs
//     (face/edge structure + area/volume within tolerance).
//
// REFERENCE ORACLE ONLY: OCCT BRepPrimAPI_MakePrism / BRepPrimAPI_MakeRevol were
// consulted to confirm the face decomposition and outward-orientation conventions;
// nothing is copied. Surface/curve parametrizations match src/native/math
// (ElSLib-aligned) so the native tessellator and the OCCT parity gate agree.
//
// OCCT-FREE. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_CONSTRUCT_NATIVE_CONSTRUCT_H
#define CYBERCAD_NATIVE_CONSTRUCT_NATIVE_CONSTRUCT_H

#include "native/construct/construct.h"
#include "native/construct/loft.h"
#include "native/construct/profile.h"
#include "native/construct/sweep.h"

#endif  // CYBERCAD_NATIVE_CONSTRUCT_NATIVE_CONSTRUCT_H
