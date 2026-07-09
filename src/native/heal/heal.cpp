// SPDX-License-Identifier: Apache-2.0
//
// heal.cpp — healShell orchestration (see heal.h for the pipeline + contract).
//
// OCCT-FREE. Uses only src/native/{math,topology,tessellate,heal}. clang++ -std=c++20.
//
#include "native/heal/heal.h"

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

  // (c)+(d) vertex unify + tolerant sew (share vertex + edge nodes). `work` is the
  // working soup the sew reads; the opt-in bridging / capping passes below rewrite it
  // and re-sew (with both opt-in flags off it stays == `clean` and never changes).
  std::vector<FaceLoop> work = clean;

  // Opt-in bounded SHORT-EDGE collapse (M5 tail): when a caller supplies a merge
  // length, remove any REDUNDANT COLLINEAR sub-feature edge a boundary vertex-split
  // inserted into an otherwise-straight wire run — a tiny NON-zero edge above the weld
  // `tol` but below the bounded band (tol, min(mergeLen, ¼·neighbour)] whose interior
  // vertex the neighbour face does not carry, so the sew cannot share the run and the
  // shell is left open. Collapsing it restores the straight span the neighbour already
  // has, so vertex_unify then shares the corners (short_edge.h). The primary weld `tol`
  // is NEVER widened; only a within-tolerance-collinear short edge is removed, so a
  // short edge that turns a real corner is left in place. With mergeLen == 0 this block
  // is a no-op (dead-guarded) and `work` stays == `clean`, byte-identical to the landed
  // slices. Runs BEFORE the first sew because it rewrites per-face corner loops.
  if (opts.shortEdgeMergeLen > 0.0) {
    const ShortEdgeResult se = collapseShortEdges(work, tol, opts.shortEdgeMergeLen);
    if (se.applied) {
      m.nCollapsedShortEdges = se.nCollapsed;
      m.maxCollapsedShortEdge = se.maxCollapsed;
      work = se.soup;
    }
  }

  // Opt-in REDUNDANT COLLINEAR-VERTEX removal (M5 tail): when a caller enables it, drop a
  // single extra vertex B a STEP exporter / mesh→B-rep conversion dropped onto a face's
  // straight boundary run A→C (so the face lists A→B→C while the NEIGHBOUR carries A→C as
  // one straight edge). B is removed ONLY when it lies within `tol` of the line A→C AND
  // projects strictly between A and C, so a vertex that turns a real corner is left in
  // place; both incident edges may be full-length, which is why short_edge.h (¼·neighbour
  // micro-edge only) and degenerate.h (≤tol only) cannot reach it. Removing B restores the
  // straight span the neighbour already carries, so vertex_unify then shares A and C and
  // the shell closes (collinear_vert.h). Introduces NO length parameter — exact
  // collinearity is the sole criterion — and NEVER widens the weld `tol`. With
  // removeCollinearVerts == false this block is a no-op (dead-guarded) and `work` stays
  // == `clean` (modulo any short-edge collapse), byte-identical to the landed slices. Runs
  // BEFORE the first sew because it rewrites per-face corner loops.
  if (opts.removeCollinearVerts) {
    const CollinearVertResult cv = removeCollinearVertices(work, tol);
    if (cv.applied) {
      m.nRemovedCollinearVerts = cv.nRemoved;
      m.maxCollinearVertDev = cv.maxDeviation;
      work = cv.soup;
    }
  }

  SewResult sr = sew(work, tol);
  m.nMergedVerts = sr.mergedVerts;
  m.nMergedEdges = sr.mergedEdges;
  if (sr.faces.size() < 4) return unhealed(shape, UnhealedReason::OpenShell, sr.maxResidualGap, m);

  // A manifold closed shell has NO boundary edges: every side is shared by exactly
  // two faces. A surviving boundary side means a real hole (missing face) or a
  // beyond-tolerance gap.
  //
  // Opt-in bounded bridging (M5): when a caller supplies a budget, try to close a
  // NEAR-MISS seam whose gap sits in the bounded band (tol, min(budget,
  // ¼·localFeature)] by snapping the unpaired corners onto their partner and
  // re-sewing (gap_bridge.h). The primary weld `tol` is NEVER widened; a gap past
  // the effective bound stays a surviving boundary edge and is reported honestly
  // below. With budget == 0 this block is a no-op (dead-guarded) and the path is
  // byte-identical to the landed slice.
  if (sr.boundaryEdges > 0 && opts.gapBridgeBudget > 0.0) {
    const BridgeResult br = bridgeGaps(work, tol, opts.gapBridgeBudget);
    if (br.applied) {
      m.nBridgedGaps = br.nBridged;
      m.maxBridgedGap = br.maxBridged;
      work = br.soup;
      sr = sew(work, tol);  // re-sew the bridged soup; boundary edges recomputed
      m.nMergedVerts = sr.mergedVerts;
      m.nMergedEdges = sr.mergedEdges;
    }
  }

  // Opt-in bounded MULTI planar-hole capping (M5 tail superset): when a caller enables
  // capMultiplePlanarHoles, a shell that sews cleanly but is MISSING two or more faces
  // leaves two or more disjoint rings of boundary edges. If EVERY ring is a disjoint
  // simple cycle, coplanar within `tol`, and a simple polygon, synthesize ONE cap face
  // per hole on the holes' existing shared nodes and re-sew (cap_hole.h,
  // capAllPlanarHoles). ALL-OR-NOTHING: any branching / non-planar / self-intersecting
  // ring declines the WHOLE set (no partial closure) and the surviving boundary edges
  // are reported honestly below. With capMultiplePlanarHoles == false this block is a
  // no-op (dead-guarded) and the path is byte-identical to the landed slices.
  if (sr.boundaryEdges > 0 && opts.capMultiplePlanarHoles) {
    const MultiCapResult caps = capAllPlanarHoles(sr, tol);
    if (!caps.declined && !caps.caps.empty()) {
      m.nCappedFaces = static_cast<int>(caps.caps.size());
      m.maxCapPlanarityDev = caps.planarityDev;
      for (const FaceLoop& c : caps.caps) work.push_back(c);
      sr = sew(work, tol);  // re-sew with every cap appended; boundary edges recomputed
      m.nMergedVerts = sr.mergedVerts;
      m.nMergedEdges = sr.mergedEdges;
    }
  }

  // Opt-in bounded SINGLE planar-hole capping (M5 tail): when a caller enables it, a shell
  // that sews cleanly but is simply MISSING one face leaves a single ring of boundary
  // edges. If that boundary is exactly one simple cycle, coplanar within `tol`, and a
  // simple polygon, synthesize ONE cap face on the hole's existing shared nodes and
  // re-sew (cap_hole.h). Any hole outside the bound leaves `declined` and the surviving
  // boundary edges are reported honestly below. This landed branch runs ONLY when the
  // multi-hole superset above is NOT enabled, so with capMultiplePlanarHoles == false
  // (every existing caller) the guard is unchanged and the path is byte-identical to the
  // landed slices; with capPlanarHoles == false it is likewise a no-op.
  if (sr.boundaryEdges > 0 && opts.capPlanarHoles && !opts.capMultiplePlanarHoles) {
    const CapResult cap = capPlanarHole(sr, tol);
    if (!cap.declined && cap.cap) {
      m.nCappedFaces = 1;
      m.maxCapPlanarityDev = cap.planarityDev;
      work.push_back(*cap.cap);
      sr = sew(work, tol);  // re-sew with the cap appended; boundary edges recomputed
      m.nMergedVerts = sr.mergedVerts;
      m.nMergedEdges = sr.mergedEdges;
    }
  }

  // Still open after the (optional) bridging / capping → report honestly, do not fake closure.
  if (sr.boundaryEdges > 0) {
    const UnhealedReason why =
        sr.maxResidualGap > tol
            ? (opts.gapBridgeBudget > 0.0 ? UnhealedReason::GapBeyondBudget
                                          : UnhealedReason::GapBeyondTolerance)
            : UnhealedReason::OpenShell;
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
