// SPDX-License-Identifier: Apache-2.0
//
// native_heal.h — public aggregate header for the native shape-healing module
// (Phase 4 capability #4 `native-healing`, FIRST SLICE).
//
// Clean-room, OCCT-FREE shape healing: given a topology::Shape face soup (or a
// malformed shell) + a tolerance, stitch it into a connected, consistently-oriented,
// WATERTIGHT solid — or report UNHEALED honestly. Built on the native math +
// topology + tessellate foundation; OCCT (`BRepBuilderAPI_Sewing`, `ShapeFix_Shell`,
// `ShapeFix_Solid`) is the SIM ORACLE only, never included here and never copied.
//
// ── WHAT THIS SLICE HEALS (the tractable, coincident/degenerate/orientation 80%) ──
//   1. TOLERANT SEWING           — a face soup whose shared edges/vertices are
//                                  coincident WITHIN tolerance (but topologically
//                                  independent) → shared edge/vertex nodes.
//   2. VERTEX / TOLERANCE UNIFY  — near-coincident vertices merged onto one node.
//   3. DEGENERATE REMOVAL        — zero-length edges + sliver/zero-area faces dropped.
//   4. ORIENTATION FIX           — flood-fill consistent outward winding.
//
// ── HONEST SCOPE (asymptotic completeness, like SSI S4-f — NOT a guarantee) ──
// This slice heals the coincident-within-tolerance / degenerate / orientation defect
// family EXACTLY (spatial-hash + closed-form geometry, no re-approximation). It does
// NOT heal arbitrary broken industrial B-rep. When a defect is OUT OF SCOPE —
//   * a gap genuinely BEYOND tolerance (a real hole),
//   * a genuinely OPEN shell that cannot close within tolerance,
//   * a MISSING PCURVE needing re-projection,
//   * a SELF-INTERSECTING wire,
//   * freeform-surface re-approximation —
// the healer NEVER fabricates a closure: it returns HealResult{Unhealed, reason,
// shape = INPUT UNCHANGED, metrics.maxResidualGap = <measured residual>}. The
// tolerance is NEVER weakened to force a pass, and no result is ever reported
// watertight unless the candidate actually tessellated closed (self_verify.h).
//
// Reported wins are MEASURED against OCCT on the in-scope fixtures, not a promise on
// every input.
//
// ── ENGINE CONTRACT (the deferral seam) ──
// Healing is INTERNAL — no cc_* entry point (like SSI). The engine's internal
// tryNativeHeal (src/engine/native) runs healShell and:
//   * status == Healed   ⇒ keep the native watertight solid.
//   * status == Unhealed ⇒ fall through to the OCCT adapter (src/engine/occt):
//                          BRepBuilderAPI_Sewing on the soup + ShapeFix_Shell /
//                          ShapeFix_Solid. src/native/** includes NO OCCT header.
//
// OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_HEAL_NATIVE_HEAL_H
#define CYBERCAD_NATIVE_HEAL_NATIVE_HEAL_H

#include "native/heal/heal.h"
#include "native/heal/heal_result.h"

// Sub-operation headers (header-only) — included so a caller can drive an
// individual step or inspect the intermediate types.
#include "native/heal/assemble_shell.h"
#include "native/heal/degenerate.h"
#include "native/heal/face_soup.h"
#include "native/heal/orient.h"
#include "native/heal/self_verify.h"
#include "native/heal/tolerant_sew.h"
#include "native/heal/vertex_unify.h"

#endif  // CYBERCAD_NATIVE_HEAL_NATIVE_HEAL_H
