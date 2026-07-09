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
//   5. BOUNDED GAP BRIDGING      — OPT-IN (HealOptions.gapBridgeBudget > 0): close a
//                                  near-miss seam whose gap sits in the bounded band
//                                  (tolerance, min(budget, ¼·localFeature)] by
//                                  snapping the unpaired corners onto their partner
//                                  and re-sewing. The primary weld tolerance is NEVER
//                                  widened; a gap past the effective bound stays
//                                  Unhealed{GapBeyondBudget} with the measured
//                                  residual. Default budget 0 ⇒ this pass is a no-op.
//   6. SINGLE PLANAR-HOLE CAP    — OPT-IN (HealOptions.capPlanarHoles == true): a shell
//                                  that sews cleanly but is MISSING one face leaves a
//                                  ring of boundary edges. When that boundary is EXACTLY
//                                  ONE simple cycle, coplanar within tolerance, and a
//                                  simple polygon, synthesize ONE cap face on the hole's
//                                  existing shared nodes and re-sew. ≥ 2 holes, a
//                                  non-planar / curved hole, a self-intersecting or
//                                  branching boundary, or a cap that fails self-verify
//                                  stay Unhealed{OpenShell} / {SelfVerifyFailed}, input
//                                  unchanged. Default false ⇒ this pass is a no-op.
//   7. MULTI PLANAR-HOLE CAP     — OPT-IN (HealOptions.capMultiplePlanarHoles == true): the
//                                  strict superset of pass 6 for a shell missing TWO OR MORE
//                                  faces. When EVERY surviving boundary ring is a DISJOINT
//                                  simple cycle, coplanar within tolerance, and a simple
//                                  polygon, synthesize ONE cap per hole on the holes' existing
//                                  shared nodes and re-sew. ALL-OR-NOTHING: any branching /
//                                  non-planar / self-intersecting ring declines the WHOLE set
//                                  (no partial closure) → Unhealed{OpenShell}, input unchanged.
//                                  When false the single-hole pass 6 runs unchanged (every
//                                  existing caller byte-identical). Default false ⇒ no-op.
//   8. SHORT-EDGE COLLAPSE        — OPT-IN (HealOptions.shortEdgeMergeLen > 0): remove a
//                                  REDUNDANT COLLINEAR sub-feature edge a boundary
//                                  vertex-split inserted into an otherwise-straight wire
//                                  run — a tiny NON-zero edge above the weld tolerance but
//                                  below the bounded band (tolerance, min(mergeLen,
//                                  ¼·neighbour)] whose interior vertex the neighbour face
//                                  does not carry (so the sew cannot share the run and the
//                                  shell is left open). Collapsed ONLY when both endpoints
//                                  lie within tolerance of the straight line through the
//                                  wire neighbours, so a short edge that turns a real
//                                  corner is left in place; removing it restores the exact
//                                  straight span the neighbour already carries. The weld
//                                  tolerance is NEVER widened. Default 0 ⇒ this pass is a
//                                  no-op (existing callers byte-identical).
//   9. COLLINEAR-VERTEX REMOVAL   — OPT-IN (HealOptions.removeCollinearVerts == true): drop a
//                                  single REDUNDANT COLLINEAR vertex B a STEP exporter / mesh→
//                                  B-rep conversion dropped onto a face's straight boundary run
//                                  A→C (the classic "T-vertex" / seam-split artifact) — the face
//                                  lists A→B→C (two edges) while the NEIGHBOUR carries the same
//                                  span as ONE straight edge A→C, so the sew cannot share the run
//                                  and the shell is left open. Distinct from pass 8: BOTH incident
//                                  edges may be FULL-LENGTH (no ¼·neighbour micro-edge bound; a
//                                  single corner removed, not two), so short_edge.h cannot reach
//                                  it. B is removed ONLY when its perpendicular distance to A→C is
//                                  ≤ tolerance AND it projects strictly between A and C, restoring
//                                  the exact straight span the neighbour carries; a vertex that
//                                  turns a real corner is left in place. Introduces NO length
//                                  parameter — exact collinearity is the sole criterion — and
//                                  NEVER widens the weld tolerance. Default false ⇒ this pass is a
//                                  no-op (existing callers byte-identical). See collinear_vert.h.
//
// ── HONEST SCOPE (asymptotic completeness, like SSI S4-f — NOT a guarantee) ──
// This slice heals the coincident-within-tolerance / degenerate / orientation defect
// family EXACTLY (spatial-hash + closed-form geometry, no re-approximation). It does
// NOT heal arbitrary broken industrial B-rep. When a defect is OUT OF SCOPE —
//   * a gap genuinely BEYOND tolerance (a real hole),
//   * a genuinely OPEN shell that cannot close within tolerance (EXCEPT the opt-in
//     single simple planar hole — pass 6 above; ≥ 2 holes / non-planar / self-
//     intersecting boundaries still defer),
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
#include "native/heal/cap_hole.h"
#include "native/heal/collinear_vert.h"
#include "native/heal/degenerate.h"
#include "native/heal/face_soup.h"
#include "native/heal/gap_bridge.h"
#include "native/heal/orient.h"
#include "native/heal/self_verify.h"
#include "native/heal/short_edge.h"
#include "native/heal/tolerant_sew.h"
#include "native/heal/vertex_unify.h"

#endif  // CYBERCAD_NATIVE_HEAL_NATIVE_HEAL_H
