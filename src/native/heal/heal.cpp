// SPDX-License-Identifier: Apache-2.0
//
// heal.cpp — healShell orchestration (see heal.h for the pipeline + contract).
//
// OCCT-FREE. Uses only src/native/{math,topology,tessellate,heal}. clang++ -std=c++20.
//
#include "native/heal/heal.h"

#include "native/heal/assemble_shell.h"
#include "native/heal/degenerate.h"
#include "native/heal/face_soup.h"
#include "native/heal/orient.h"
#include "native/heal/self_verify.h"
#include "native/heal/tolerant_sew.h"

#include <cmath>
#include <utility>
#include <vector>

namespace cybercad::native::heal {

namespace {

// Package an Unhealed verdict: input UNCHANGED, measured residual carried, reason
// set. Never claims watertight/valid.
HealResult unhealed(const topo::Shape& input, UnhealedReason reason, double residual,
                    HealMetrics metrics) {
  metrics.watertight = false;
  metrics.valid = false;
  metrics.maxResidualGap = residual;
  HealResult r;
  r.status = HealStatus::Unhealed;
  r.reason = reason;
  r.shape = input;  // pristine soup for the OCCT fallback
  r.metrics = metrics;
  return r;
}

}  // namespace

HealResult healShell(const topo::Shape& shape, const HealOptions& opts) {
  const double tol = opts.tolerance > 0 ? opts.tolerance : 1e-12;
  HealMetrics m;

  // (a) extract the face soup as world-corner loops.
  const std::vector<FaceLoop> raw = extractSoup(shape, tol);
  if (raw.size() < 4)  // fewer than a tetrahedron's faces → cannot be a closed solid
    return unhealed(shape, UnhealedReason::OpenShell, 0.0, m);

  // (b) degenerate removal (zero-length sides + sliver/zero-area faces).
  int dropped = 0;
  const std::vector<FaceLoop> clean = removeDegenerate(raw, tol, dropped);
  m.nDroppedDegenerate = dropped;
  if (clean.size() < 4) return unhealed(shape, UnhealedReason::OpenShell, 0.0, m);

  // (c)+(d) vertex unify + tolerant sew (share vertex + edge nodes).
  SewResult sr = sew(clean, tol);
  m.nMergedVerts = sr.mergedVerts;
  m.nMergedEdges = sr.mergedEdges;
  if (sr.faces.size() < 4) return unhealed(shape, UnhealedReason::OpenShell, sr.maxResidualGap, m);

  // A manifold closed shell has NO boundary edges: every side is shared by exactly
  // two faces. A surviving boundary side means a real hole (missing face) or a
  // beyond-tolerance gap — report honestly, do not fake closure.
  if (sr.boundaryEdges > 0) {
    const UnhealedReason why =
        sr.maxResidualGap > tol ? UnhealedReason::GapBeyondTolerance : UnhealedReason::OpenShell;
    return unhealed(shape, why, sr.maxResidualGap, m);
  }

  // (e) orientation fix: flood-fill consistent winding across shared edges.
  m.nFlipped = makeConsistent(sr.faces);

  // (f) assemble + (g) self-verify (with the global outward sign tie-break).
  topo::Shape solid = assembleSolid(sr.faces);
  VerifyResult v = verify(solid);
  if (v.watertight && v.signedVolume < 0.0) {
    // Consistently oriented but inward → flip the whole shell and re-verify.
    reverseAll(sr.faces);
    m.nFlipped += static_cast<int>(sr.faces.size());
    solid = assembleSolid(sr.faces);
    v = verify(solid);
  }

  if (!v.watertight || !(v.signedVolume > 0.0))
    return unhealed(shape, UnhealedReason::SelfVerifyFailed, sr.maxResidualGap, m);

  // Healed: self-verified watertight + positive enclosed volume, fully closed.
  m.watertight = true;
  m.valid = true;
  m.maxResidualGap = 0.0;
  HealResult r;
  r.status = HealStatus::Healed;
  r.reason = UnhealedReason::None;
  r.shape = std::move(solid);
  r.metrics = m;
  return r;
}

}  // namespace cybercad::native::heal
