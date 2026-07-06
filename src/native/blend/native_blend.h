// SPDX-License-Identifier: Apache-2.0
//
// native_blend.h — public aggregate header for the native blend slice (Phase 4 #6
// `native-blends`, openspec/NATIVE-REWRITE.md). Clean-room, OCCT-FREE fillet /
// chamfer / offset-face / shell for the TRACTABLE planar cases, built on the native
// math + topology + boolean (BSP-CSG) + tessellate foundations.
//
// ── WHAT LANDS NATIVE (each self-verified watertight + sane volume by the engine;
//    anything else returns a NULL Shape → OCCT BRepFilletAPI/BRepOffsetAPI oracle) ──
//   * chamfer_edges  — a PLANAR chamfer on a CONVEX edge between two PLANAR faces:
//                      slice the corner off with the plane through the two setback
//                      lines (a planar cut), re-weld. Likely EXACT vs OCCT for a box.
//   * fillet_edges   — a CONSTANT-radius rolling-ball (tangent-cylinder) fillet on a
//                      CONVEX planar-dihedral edge: axis ∥ crease, radius r, seated
//                      tangent to both planes (the Phase-3 dihedral construction),
//                      tiled into deflection-bounded facets. Deflection-bounded vs
//                      OCCT.
//   * offset_face    — offset a PLANAR face along its outward normal (grow/shrink the
//                      solid by a slab), dragging the adjacent side faces. EXACT for
//                      a prism cap.
//   * shell          — uniform-thickness hollow: remove the selected faces, inset the
//                      remaining shell inward by `thickness` (offset + BSP-CSG cut of
//                      the inward cavity). Convex planar solids.
//
// ── WHAT IS OCCT-FALLTHROUGH (never faked — the builder returns NULL) ─────────────
//   curved-face inputs, CONCAVE edges, variable-radius fillets (fillet_edges_variable),
//   fillet_face, an edge shared by ≠ 2 faces, multi-edge fillet interference, a
//   non-convex shell, or any self-verify failure. The engine (native_engine.cpp)
//   runs the MANDATORY watertight + sane-volume-sign guard and DISCARDS a bad native
//   result, falling through to OCCT rather than emit a wrong/leaky solid.
//
// The four entries are the AGREED API the engine glue (src/engine/native) calls; keep
// the signatures stable. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_H
#define CYBERCAD_NATIVE_BLEND_H

#include "native/blend/blend_geom.h"
#include "native/blend/chamfer_edges.h"
#include "native/blend/curved_chamfer.h"
#include "native/blend/curved_fillet.h"
#include "native/blend/fillet_edges.h"
#include "native/blend/offset_face.h"
#include "native/blend/shell.h"

/// The native blend API lives in this namespace.
namespace cybercad::native::blend {}

#endif  // CYBERCAD_NATIVE_BLEND_H
