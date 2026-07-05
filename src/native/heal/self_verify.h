// SPDX-License-Identifier: Apache-2.0
//
// self_verify.h — the mandatory gate: a healed candidate is kept ONLY if it
// tessellates watertight with a positive enclosed volume.
//
// The healer never trusts its own bookkeeping to declare a closure — it proves it,
// by the SAME instrument the boolean assembler and every native op self-verify
// with: tessellate the candidate and require
//   * tessellate::isWatertight(mesh)   — every undirected mesh edge used by exactly
//                                        two triangles (closed 2-manifold, no
//                                        boundary/no T-junction), AND
//   * tessellate::enclosedVolume > 0    — consistent OUTWARD orientation (a
//                                        consistently inward shell gives a negative
//                                        signed volume; the orientation-fix's global
//                                        sign tie-break flips it, so a genuine solid
//                                        lands positive here).
//
// The candidate is rebuilt with SHARED vertex/edge nodes and straight spans (see
// assemble_shell.h), so it meshes by the identical shared-edge path a native prism
// uses and the default weld is legitimate (it fuses boundary points the two faces
// place identically on a shared side — NOT a soup gap; a soup gap has no shared
// nodes and would leave boundary edges). Verifying across a small deflection ladder
// guards against a weld tolerance that happens to close a near-miss at one
// deflection only.
//
// Returns the signed enclosed volume too, so the orchestrator can read the sign for
// the global-outward tie-break before the final positive-volume assertion.
//
// OCCT-FREE. Uses src/native/tessellate + src/native/topology. clang++ -std=c++20.
// Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_SELF_VERIFY_H
#define CYBERCAD_NATIVE_HEAL_SELF_VERIFY_H

#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <cmath>

namespace cybercad::native::heal {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;

struct VerifyResult {
  bool watertight = false;   ///< closed 2-manifold at every tested deflection
  double signedVolume = 0.0;  ///< enclosed volume at the finest deflection (signed)
};

/// Tessellate `candidate` across a deflection ladder and report watertightness +
/// signed enclosed volume. watertight is true only if EVERY deflection closed.
inline VerifyResult verify(const topo::Shape& candidate) {
  VerifyResult out;
  if (candidate.isNull()) return out;
  bool allClosed = true;
  double vol = 0.0;
  for (const double d : {0.05, 0.02, 0.01}) {
    tess::MeshParams p;
    p.deflection = d;
    const tess::Mesh m = tess::SolidMesher{p}.mesh(candidate);
    if (!tess::isWatertight(m)) {
      allClosed = false;
      break;
    }
    vol = tess::enclosedVolume(m);
  }
  out.watertight = allClosed;
  out.signedVolume = vol;
  return out;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_SELF_VERIFY_H
