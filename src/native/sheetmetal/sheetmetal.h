// SPDX-License-Identifier: Apache-2.0
//
// sheetmetal.h — public aggregate header for the native SHEET-METAL library
// (MOAT M-SM, first slice). Clean-room, OCCT-FREE builders for the three
// constant-thickness sheet-metal primitives the native engine implements today:
//
//   * base_flange(profile2D, thickness)  — the flat sheet solid = the 2D profile
//     extruded by `thickness`. The base of every sheet-metal part. This is a thin
//     wrapper over the landed construct::build_prism (a base flange IS a prism);
//     it exists so the sheet-metal facade has a semantic entry point and so the
//     part's thickness is a first-class, later-consumed quantity.
//
//   * edge_flange(base, edgeId, height, bendRadius, angleDeg) — add a flange off a
//     STRAIGHT edge of the base sheet: a cylindrical BEND (a partial annulus of
//     inner radius `bendRadius`, swept through `angleDeg`) welded to a planar
//     FLANGE WALL of the given `height`, at the base's constant thickness. Fully
//     analytic — the bend is a partial cylinder, the wall is a prism. The whole
//     part (base + bend + wall) is emitted as ONE watertight solid built directly
//     face-by-face in the base's frame (no boolean), so the bend walls are TRUE
//     Cylinder surfaces and the enclosed volume converges to the closed form.
//
//   * unfold(part, kFactor) — the FLAT-PATTERN: unroll the single bend about its
//     neutral fibre (bend allowance BA = angle·(bendRadius + kFactor·thickness))
//     into the developed flat blank (a planar sheet solid of the same thickness).
//     This is the manufacturing payoff — the blank a laser/press starts from.
//
// ── SCOPE (honest — first slice) ──────────────────────────────────────────────
// SUPPORTED natively (constant thickness, planar + single cylindrical bend only):
//   base flange (any simple polygon profile), ONE edge flange off ONE straight
//   edge with ONE cylindrical bend, and the unfold of that single-bend part.
// HONEST-DECLINED (a NULL Shape + a measured SheetMetalDecline — NEVER a wrong or
// self-intersecting solid; there is NO OCCT sheet-metal op to fall back to, so a
// decline here is a clean cc_last_error, not an OCCT forward):
//   * multi-bend parts / more than one flange (bend-bend interference, corner
//     relief, and miter are all out of the first slice);
//   * a NON-STRAIGHT bend line (only a straight base edge can be flanged);
//   * a self-colliding flange (the bend/wall would intersect the base — measured
//     by the composite self-verify, never forced through);
//   * a freeform / non-planar base, or a degenerate parameter (thickness ≤ 0,
//     height < 0, bendRadius < 0, angle outside (0°,180°)).
//
// ── VERIFICATION (two gates; OCCT is NOT an oracle here) ──────────────────────
// OCCT core has no sheet-metal module, so there is NO OCCT sheet-metal oracle. The
// ARBITER is CLOSED FORM:
//   Gate (a) HOST, no OCCT (tests/native/test_native_sheetmetal.cpp): every built
//     solid meshes WATERTIGHT with χ=2 and consistently oriented, at the CLOSED-
//     FORM volume — base = profileArea·thickness; folded = base + bend(partial-
//     annulus × width) + wall prism; and the UNFOLD round-trips: developed area =
//     base area + bend developed length·width + flange area (the bend-allowance
//     k-factor formula), invariant under fold→unfold.
//   Gate (b) SIM (booted simulator, native under its own engine; NO OCCT compared):
//     the built parts pass cc_check_solid (valid closed 2-manifold) and their
//     cc_mass_properties volume matches the host closed form, deterministically.
//
// Self-verify reuses the LANDED mesh audit (tess::isWatertight /
// tess::isConsistentlyOriented / enclosedVolume + dm::rfdetail::eulerChar) — the
// same vocabulary draft_faces / split_plane use. Nothing is widened.
//
// CONSUMES (never rewritten): construct::build_prism (base + wall prisms),
// the topology ShapeBuilder + Explorer, math elementary Cylinder/Ax3, and the
// tessellate mesh audit. Additive sibling module — touches no landed header.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SHEETMETAL_H
#define CYBERCAD_NATIVE_SHEETMETAL_H

#include "native/sheetmetal/base_flange.h"
#include "native/sheetmetal/edge_flange.h"
#include "native/sheetmetal/unfold.h"

#endif  // CYBERCAD_NATIVE_SHEETMETAL_H
