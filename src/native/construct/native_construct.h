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
//   build_torus(R,r,frame|centre,ax) the BARE analytic RING-TORUS primitive: one
//                                    doubly-periodic Kind::Torus face (no seam wire).
//                                    Unlike an off-axis arc revolve (rational B-spline
//                                    bands), the face is a TRUE analytic torus, so the
//                                    boolean's recogniseCurvedSolid admits it and the
//                                    torus∩{cyl/sphere/cone/torus} families run in the
//                                    pure-native path (no OCCT). Ring torus only
//                                    (R > r > 0); spindle/degenerate → NULL. cc_torus.
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
//   build_ruled_loft_sections(secs)  N-SECTION generalisation (N≥2): skin 2..N
//                                    equal-count planar section loops into one solid
//                                    — (N−1) ruled bands (shared internal vertex
//                                    rings) + the first & last planar caps. Each
//                                    section is aligned to its predecessor so the
//                                    1:1 correspondence propagates down the chain.
//                                    build_ruled_loft(A,B) is the N=2 special case.
//                                    Mismatched counts / non-planar / degenerate /
//                                    self-intersecting → NULL (OCCT fallthrough).
//   build_loft_sections(xyz,cnt,ns)  entry for the cc_solid_loft CHAIN / a section
//                                    list: `xyz` packs `ns` flat (x,y,z) section
//                                    loops back to back, `cnt[k]` the vertex count
//                                    of section k; then build_ruled_loft_sections.
//
//   ── Tier-C (#4b, sweep.h) ────────────────────────────────────────────────────
//   build_sweep(profileXY,pathXYZ)   sweep a CLOSED planar profile along a 3D
//                                    polyline path: profile centred on its centroid,
//                                    placed perpendicular to the START tangent (local
//                                    x = cross(tan,+Y), local y = +Y), transported
//                                    with the CONSTANT frame (OCCT MakePipe's planar
//                                    corrected-Frenet law). NATIVE for a STRAIGHT spine
//                                    → an EXACT directional prism (profile area × path
//                                    length), AND for a SMOOTH CURVED but PLANAR spine →
//                                    a constant-frame ruled-band tube (deflection-bounded
//                                    vs OCCT, watertight). A NON-PLANAR curved spine
//                                    (genuine corrected-Frenet) or a TIGHT-CURVATURE /
//                                    self-intersecting spine (turning radius < profile
//                                    circumradius, or a too-sharp turn) returns NULL →
//                                    OCCT MakePipe (guarded, not faked). Degenerate
//                                    profile / < 2 path points → NULL. A rotation-
//                                    minimizing frame (double-reflection RMF, detail::
//                                    rmfFrames) is provided for twist-free non-planar
//                                    transport but is not used by the constant-frame
//                                    MakePipe oracle path.
//   build_twisted_sweep(...,tw,sc)   NATIVE: plain (twist ≈ 0, scale ≈ 1) forwards to
//                                    build_sweep; a REAL twist/scale builds the per-
//                                    station Frenet-framed ruled ThruSections tube
//                                    (matching the OCCT twisted_sweep oracle) when it
//                                    welds watertight and does not self-fold, else NULL
//                                    (OCCT). cc_twisted_sweep.
//   build_guided_sweep(...,guide)    NATIVE. Sweep the profile scaling each station's
//                                    section by the guide splay dist(path,guide)/d0
//                                    (per-station Frenet-framed ruled ThruSections,
//                                    matching the OCCT guided_sweep oracle). NULL on a
//                                    coincident guide start / degenerate input / self-
//                                    fold. cc_guided_sweep.
//   build_loft_along_rail(rail,A,B)  NATIVE for a STRAIGHT rail — a ruled loft between
//                                    the two equal-count sections placed perpendicular
//                                    to the rail tangent (matching MakePipeShell on a
//                                    straight rail). A CURVED / kinked rail (genuine
//                                    pipe-shell morph) or mismatched section counts →
//                                    NULL (OCCT MakePipeShell). cc_loft_along_rail.
//   build_variable_sweep(A,B,spine,  NATIVE variable-section / guide+spine sweep. A section
//                        guide)       that MORPHS from A (spine start) to B (spine end) along
//                                    the spine, each station = interpolate(A,B,f) placed by
//                                    the perpendicular (straight) / RMF (curved) frame,
//                                    OPTIONALLY scaled by the guide splay dist(spine,guide)/d0
//                                    (guide null / count<2 → no guide, reduces to the
//                                    loft_along_rail morph). A circle→circle radius-varying
//                                    morph along a straight spine is a truncated cone. NULL
//                                    (OCCT MakePipeShell multi-section) on mismatched counts,
//                                    a non-planar spine, a coincident guide start, a non-
//                                    positive guide scale, or a self-folding morph.
//                                    cc_variable_sweep.
//
//   ── Tier-D (#4b, thread.h) ────────────────────────────────────────────────────
//   build_tapered_shank(r,fh,th,ppm) GENUINELY NATIVE. Revolve a shank silhouette 360°
//                                    about Z (full radius r over the upper fullHeight,
//                                    tapering to a TRUE point over the lower taperHeight)
//                                    by reusing the native revolve (build_revolution).
//                                    Wide at the head, a point at the tip; watertight,
//                                    exact/deflection-bounded vs BRepPrimAPI_MakeRevol.
//   build_helical_thread(...)        ATTEMPT native, engine-verified. A V/triangular
//   build_tapered_thread(...)        section swept RADIALLY along the pitch-line helix
//                                    via the axis-aux-spine law (mirroring MakePipeShell
//                                    SetMode(axisWire,true)), tiled into bilinear ruled
//                                    bands + planar caps, guarded against self-
//                                    intersection. The engine accepts it as native ONLY
//                                    when it self-verifies robustly watertight; on the
//                                    current tessellator the per-turn seams do not weld
//                                    robustly, so these HONESTLY fall through to OCCT
//                                    (never faked). See thread.h §HONESTY.
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
//     * helical_thread / tapered_thread (Tier D): the native radial-V helical tiling is
//       built + guarded, but its per-turn seams do not weld robustly watertight on the
//       current tessellator, so the engine self-verify defers them to OCCT (honest,
//       verified fall-through, never faked — see thread.h §HONESTY). tapered_shank IS
//       native.
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
#include "native/construct/residuals.h"
#include "native/construct/sweep.h"
#include "native/construct/thread.h"

#endif  // CYBERCAD_NATIVE_CONSTRUCT_NATIVE_CONSTRUCT_H
