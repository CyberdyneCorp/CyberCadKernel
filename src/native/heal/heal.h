// SPDX-License-Identifier: Apache-2.0
//
// heal.h — the top-level healer entry point: healShell(shape, opts).
//
// Runs the four sub-operations in dependency order and self-verifies:
//   (a) extract each face as an ordered world-corner loop           (face_soup.h)
//   (b) drop zero-length edges + sliver/zero-area faces             (degenerate.h)
//   (c) unify near-coincident vertices → shared Vertex nodes        (vertex_unify.h)
//   (d) tolerant sew: share an edge node per coincident side        (tolerant_sew.h)
//   (e) orientation fix: flood-fill consistent + global outward     (orient.h)
//   (f) assemble Shell/Solid on the shared nodes                    (assemble_shell.h)
//   (g) self-verify watertight + enclosed volume > 0                (self_verify.h)
//
// Steps (c) and (d) are fused in sew() (unify feeds the edge keying directly). The
// orchestrator is a flat pipeline; each dense step is isolated in its own header.
//
// OUTCOME. If the assembled solid self-verifies watertight + positive volume with
// no surviving boundary edges, returns HealResult{Healed, shape=solid, metrics...}.
// Otherwise returns HealResult{Unhealed, reason, shape=INPUT UNCHANGED, metrics with
// the measured residual}. The tolerance is NEVER widened to force a pass; a
// beyond-tolerance gap / genuinely open shell is reported honestly and deferred to
// OCCT ShapeFix by the caller (see native_heal.h).
//
// OCCT-FREE. clang++ -std=c++20. Declaration here; body in heal.cpp.
//
#ifndef CYBERCAD_NATIVE_HEAL_HEAL_H
#define CYBERCAD_NATIVE_HEAL_HEAL_H

#include "native/heal/heal_result.h"
#include "native/topology/shape.h"

namespace cybercad::native::heal {

/// Heal a face-soup / malformed shell into a watertight, consistently-oriented
/// solid — or report Unhealed with the measured residual (input returned
/// unchanged). See the file header for the pipeline and the honesty contract.
HealResult healShell(const topo::Shape& shape, const HealOptions& opts);

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_HEAL_H
