// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_seeding_parity.mm — SSI Stage S2 (subdivision seeding) native-vs-OCCT
// per-branch RECALL parity harness (iOS simulator). Gate 2 of the two-gate S2 model;
// Gate 1 (host, no OCCT) is tests/native/test_native_ssi_seeding.cpp.
//
// This is the STRICTER companion to native_ssi_seeding_recall.mm. Where that harness
// reports recall from deduped seed COUNTS, this one asserts recall PER BRANCH by
// geometry: it takes OCCT GeomAPI_IntSS as the oracle for the TRUE intersection loci
// (each OCCT Geom_Curve = one branch) and, for EVERY transversal OCCT branch, requires
// that the native seeder produced ≥ 1 seed lying ON THAT OCCT CURVE (nearest point on
// the curve < tol, GeomAPI_ProjectPointOnCurve) AND on BOTH surfaces
// (GeomAPI_ProjectPointOnSurf::LowerDistance < tol). Counting alone cannot catch a
// native seed that landed on the WRONG branch or that OCCT arc-split a loop into
// several curves; matching seeds to curves does.
//
// TANGENTIAL branches are EXCLUDED from the recall denominator (S2 is transversal-only;
// tangent seeding ill-conditions the refine → S4). A branch is classified tangential
// when, sampled along the OCCT curve, ‖n₁ × n₂‖ (the sine of the angle between the two
// surface normals) stays below a tangency threshold everywhere on it. Such branches are
// reported separately (tang=…) and never counted against recall — but the harness also
// never fakes a seed for them.
//
// COVERAGE (freeform + non-closed-form pairs S1 defers as NotAnalytic):
//   * bspline ∩ bspline        — two freeform biquadratic B-spline sheets, one loop.
//   * bspline ∩ plane          — a rippled B-spline sheet cut by a plane → MULTI-LOOP
//                                (the completeness stressor: several small transversal
//                                 loops the subdivision must all reach).
//   * skew cyl ∩ cyl           — orthogonal unequal-radius cylinders → 2 transversal
//                                loops (the skew-quadric quartic S1 defers).
//   * sphere ∩ sphere          — crossing equal spheres → 1 transversal circle.
//
// PASS = recall == 100% of the TRANSVERSAL OCCT branches for the pair AND every emitted
// native seed lies on both surfaces within tol. A missed transversal branch (too-shallow
// subdivision) FAILS honestly and prints the miss; it is not hidden behind a seed count.
//
// SSI is INTERNAL — no cc_* entry point is called or added; asserted at the
// cybercad::native::ssi C++ boundary, exactly like the S1 parity harness.
//
// This TU is OCCT-dependent AND substrate-dependent: it links the OCCT oracle and the
// NumPP/SciPP numsci archive, and compiles src/native/ssi/seeding.cpp +
// src/native/numerics/numerics.cpp under -DCYBERCAD_HAS_NUMSCI. Built ONLY by
// scripts/run-sim-native-ssi-seeding.sh; on the SKIP list of run-sim-suite.sh.
//
// Output: one [NSEED] PASS/FAIL line per pair with per-pair transversal-branch recall,
// tangential-branch count, and the worst seed on-both-surfaces residual, then a final
// "== N passed, M failed ==". Flushes and std::_Exit (OCCT static teardown in the
// trimmed static build is not exit-clean — same rationale as native_ssi_parity).
//
// ── DENOMINATOR REPAIRED 2026-07-19; ONE REAL GAP REMAINS (bspline ∩ plane) ────────
//
// This harness did not compile from the day it was written until 2026-07-19 (gp_Dir has no
// Magnitude(); see the fixed line below), so it had NEVER RUN. Its first execution reported
// 1 passed / 3 failed. TWO of those failures were ARTIFACTS OF THIS HARNESS — it counted
// OCCT arc PIECES as branches — and are fixed by mergeArcsIntoBranches below. Measured:
//
//   * sphere x sphere  was 0.50 (1/2). Two unit spheres meet in EXACTLY ONE circle; OCCT
//     bisects it at the parameter seam into t=[0,pi] and [pi,2pi] on x=0.5, r=0.866, sharing
//     both endpoints. Measured: 2 arcs, 2 nodes, both degree 2, 0 junctions. Now 1 branch,
//     recall 1.00, seed on it to 1.11e-16.
//   * skew cyl unequal was 0.67 (2/3). Measured: 3 arcs, 3 nodes, all degree 2, 0 junctions,
//     2 loops (one closed arc + two arcs meeting end-to-end). Now 2 branches, recall 1.00.
//
// The sibling gate `native_ssi_seeding_recall` always passed these same pairs at 1.00 — an
// in-repo gate already disagreed with this one's model, and it was this one that was wrong.
//
// ── THE REMAINING FAILURE IS REAL AND IS NOT A SEEDING-DENSITY PROBLEM ─────────────
//
//   * bspline x plane  recall=0.03 (1/40). The egg-carton cut at z=0.5 is ONE connected
//     saddle network: 40 arcs over 32 nodes = 16 degree-4 saddles + 16 degree-1 boundary
//     exits (16·4+16·1 = 80 = 2·40 ✓). There is NO degree-2 node, so nothing merges and all
//     40 arcs stay distinct branches. The level set passes exactly through surface saddles,
//     where the normals are parallel — sampled ‖n₁×n₂‖ bottoms out at 0.0000.
//
//     MEASURED, so the next reader does not re-litigate it: raising seeder resolution does
//     NOTHING. Candidate regions 30352 → 164268 across five configurations (8x8 @1/48,
//     8x8 @1/96, 16x16 @1/96, 16x16 @1/192, 32x32 @1/192), and at every one of them
//     seeds=1, refinedAccepted=1, arcs covered = 1/40. The completeness critic and the M1c
//     targeted re-seed change nothing either. Two independent reasons, both confirmed by
//     CYBERCAD_SSI_SEED_DIAG:
//       (a) all candidates fall in ONE cluster (clusters=1), which is topologically CORRECT
//           — the locus really is one connected component — so "≈1 seed per branch" gives 1;
//       (b) the distinct-branch split that could emit more than one seed per cluster is
//           gated on `bothFreeform` (seeding.cpp), and a PLANE is not freeform, so doSplit=0
//           and the cluster collapses to its single tightest seed regardless of resolution.
//
//     It is therefore a MARCHING/branch-point problem, not a seeding one. Measured reach
//     from the one seed: plain S3 stalls at the first saddle (status NearTangent) and covers
//     2/40 arcs; MarchOptions.enableBranchPoints (S4-d) localizes 1 branch point, routes 3
//     arms and reaches 7/40, then stalls again with nearTangentGaps=3 — branch-point
//     localization is applied to the SEED's arm but is not re-applied to the arms it routes.
//     Closing this needs S4-d made ITERATIVE: a work list where every arm ending in a
//     near-tangent stall is re-tested for a branch point and its unvisited arms enqueued,
//     with visited-arc dedup to terminate. On this graph that walks all 16 junctions and
//     covers all 40 arcs from the single seed. See the S4-d work in openspec/MOAT-ROADMAP.md.
//     Until then this pair FAILS HONESTLY at 1/40 rather than being merged away.
//
#include "native/ssi/native_ssi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_ssi_seeding_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif
#if !defined(CYBERCAD_HAS_NUMSCI)
#error "native_ssi_seeding_parity requires -DCYBERCAD_HAS_NUMSCI (the least_squares refine)"
#endif

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax3.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomLProp_SLProps.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

namespace ssi = cybercad::native::ssi;
namespace nm = cybercad::native::math;
using nm::Ax3;
using nm::Dir3;
using nm::Point3;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kSamplesPerCurve = 96;

int g_pass = 0;
int g_fail = 0;

gp_Ax3 toOcctAx3(const Ax3& f) {
  return gp_Ax3(gp_Pnt(f.origin.x, f.origin.y, f.origin.z),
                gp_Dir(f.z.x(), f.z.y(), f.z.z()),
                gp_Dir(f.x.x(), f.x.y(), f.x.z()));
}
gp_Pnt toOcctPnt(const Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

double distToOcctSurface(const Handle(Geom_Surface)& s, const Point3& p) {
  GeomAPI_ProjectPointOnSurf proj(toOcctPnt(p), s);
  return proj.NbPoints() > 0 ? proj.LowerDistance() : 1e30;
}

double distToOcctCurve(const Handle(Geom_Curve)& c, const Point3& p) {
  GeomAPI_ProjectPointOnCurve proj(toOcctPnt(p), c);
  return proj.NbPoints() > 0 ? proj.LowerDistance() : 1e30;
}

// Sine of the angle between the two OCCT surface normals at a 3D point, obtained by
// projecting the point onto each surface, reading the (u,v) foot, and taking the
// GeomLProp_SLProps normal there. This is the transversality witness used to classify
// an OCCT branch as transversal vs tangential — computed on the ORACLE surfaces so the
// classification does not depend on the native side. Returns -1 if a normal is
// undefined (degenerate patch) so the caller can treat it conservatively.
double crossingSineOnOcct(const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                          const gp_Pnt& q) {
  GeomAPI_ProjectPointOnSurf pa(q, sa), pb(q, sb);
  if (pa.NbPoints() == 0 || pb.NbPoints() == 0) return -1.0;
  Standard_Real ua, va, ub, vb;
  pa.LowerDistanceParameters(ua, va);
  pb.LowerDistanceParameters(ub, vb);
  GeomLProp_SLProps la(sa, ua, va, 1, 1e-9), lb(sb, ub, vb, 1, 1e-9);
  if (!la.IsNormalDefined() || !lb.IsNormalDefined()) return -1.0;
  gp_Dir na = la.Normal(), nb = lb.Normal();
  // gp_Dir::Crossed returns a NORMALIZED gp_Dir, so its magnitude is identically 1 — and gp_Dir
  // has no Magnitude() member at all, so this line did NOT COMPILE and this harness has never
  // run since it was written. Go through gp_Vec, as the sibling harnesses already do
  // (native_ssi_marching_parity.mm:180, native_ssi_s4_classification_parity.mm:159).
  const double s = gp_Vec(na).Crossed(gp_Vec(nb)).Magnitude();  // ‖n₁ × n₂‖ for unit normals
  return s;
}

// A classified OCCT branch: its curve handle plus whether it is transversal (part of
// the recall denominator) or tangential (excluded, reported separately, an S4 gap).
struct OcctBranch {
  Handle(Geom_Curve) curve;
  bool transversal = true;
  double maxCrossingSine = 0.0;  // over samples; > tangentSine ⇒ transversal
};

// Sample one OCCT curve over its (clamped) parameter range and decide transversal vs
// tangential from the MAX ‖n₁×n₂‖ along it: a transversal branch crosses somewhere even
// if it grazes locally, so the max being above the tangency threshold means the branch
// is genuinely a crossing locus (S2's target). A branch whose normals stay parallel
// everywhere is tangential (S4).
OcctBranch classifyBranch(const Handle(Geom_Curve)& c,
                          const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                          double tangentSine) {
  OcctBranch b;
  b.curve = c;
  double f = c->FirstParameter(), l = c->LastParameter();
  if (!std::isfinite(f) || f < -1e6) f = -1e6;
  if (!std::isfinite(l) || l > 1e6) l = 1e6;
  double maxSine = 0.0;
  for (int i = 0; i <= kSamplesPerCurve; ++i) {
    const double t = f + (l - f) * (double(i) / kSamplesPerCurve);
    // The guard covers crossingSineOnOcct as well as Value(): GeomAPI_ProjectPointOnSurf and
    // GeomLProp_SLProps both raise on a degenerate patch (a projection that fails to converge,
    // a first-derivative pair that spans no plane), and a throw out of here aborts the whole
    // gate rather than skipping one sample. LATENT, NOT A LIVE FIX: no pair in this corpus
    // throws today — all four run clean with the narrow guard. Note the corpus DOES reach the
    // degenerate regime without tripping it: sampled ‖n₁×n₂‖ bottoms out at 0.0000 on the
    // bspline∩plane arcs (the 16 saddle junctions are exact tangencies), where OCCT still
    // returns defined normals and a finite sine rather than raising. So this widens the guard
    // ahead of a pair whose projection actually fails; it does not repair an observed crash.
    double s = -1.0;
    try {
      const gp_Pnt q = c->Value(t);
      s = crossingSineOnOcct(sa, sb, q);
    } catch (...) { continue; }
    if (s >= 0.0) maxSine = std::max(maxSine, s);
  }
  b.maxCrossingSine = maxSine;
  b.transversal = (maxSine > tangentSine);
  return b;
}

// ── ARC → BRANCH MERGE (the recall denominator) ───────────────────────────────────
//
// GeomAPI_IntSS returns arc PIECES, not branches. It bisects a closed circle at the
// parameter seam and splits a locus at every junction, so `NbLines()` overcounts the
// branches a seeder is expected to cover — that is what made this gate report 0.50 on a
// sphere pair that meets in exactly ONE circle.
//
// The branch a seed is expected to cover is a maximal chain of arcs joined end-to-end
// where the join is UNAMBIGUOUS. Weld arc endpoints into shared nodes, then union two
// arcs iff they meet at a node of degree EXACTLY 2: there the continuation is unique, so
// the marcher runs straight through and one seed anywhere on the chain covers all of it.
// A node of degree > 2 is a genuine JUNCTION where the continuation is NOT unique — the
// marcher must stop and choose, so the arcs on either side stay DISTINCT branches, each
// needing its own seed. Degree-1 nodes are domain-boundary exits and merge nothing.
//
// This is deliberately NOT "connected components". On the bspline∩plane egg-carton the
// whole locus IS one connected component, so a component denominator would read 1, the
// single native seed would score 1/1, and the gate would PASS while the seeder in fact
// covers 1 arc in 40. The degree-2 rule keeps that case at 40 and keeps it failing.
//
// WELD TOLERANCE. Measured on this corpus, arc-endpoint separations that are meant to be
// the same node are either EXACTLY 0 or in [2.07e-7 median, 3.65e-7 max]; the next-larger
// separation between endpoints that are genuinely distinct is above 1e-3. So the verdict
// is identical for any weld tolerance in [4e-7, 1e-3] — checked at 1e-6, 1e-5, 1e-4 and
// 1e-3, all giving 32 nodes / 16 junctions on bspline∩plane — and kWeldTol sits
// mid-plateau, three orders of magnitude clear on both sides.
//
// Which pairs the tolerance actually binds on: the sphere and skew-cylinder joins are
// EXACT (separation 0), because there OCCT splits one analytic curve at a parameter seam
// and both pieces evaluate to the identical point. They stay correct even at a 1e-12
// weld. It is only the bspline∩plane SADDLES that need the 4e-7 floor, because there
// OCCT's marcher halts near — not exactly at — the degeneracy. Dropping the tolerance to
// 1e-12 splits that locus into 39 branches over 6 junctions instead of 40 over 16, which
// the hardcoded 40 in pairBSplinePlane catches.
constexpr double kWeldTol = 1e-5;

// Union-find over arcs.
struct ArcMerge {
  std::vector<int> parent;
  explicit ArcMerge(std::size_t n) : parent(n) {
    for (std::size_t i = 0; i < n; ++i) parent[i] = static_cast<int>(i);
  }
  int find(int a) {
    while (parent[static_cast<std::size_t>(a)] != a) {
      parent[static_cast<std::size_t>(a)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(a)])];
      a = parent[static_cast<std::size_t>(a)];
    }
    return a;
  }
  void unite(int a, int b) { a = find(a); b = find(b); if (a != b) parent[static_cast<std::size_t>(a)] = b; }
};

// Clamp an unbounded/infinite parameter range the way classifyBranch does, so endpoint
// extraction and transversality sampling agree on where an arc starts and ends.
void clampedRange(const Handle(Geom_Curve)& c, double& f, double& l) {
  f = c->FirstParameter();
  l = c->LastParameter();
  if (!std::isfinite(f) || f < -1e6) f = -1e6;
  if (!std::isfinite(l) || l > 1e6) l = 1e6;
}

// Group OCCT arcs into branches by the degree-2 rule above. Returns a group id per arc
// (dense, 0..groupCount-1) and sets `junctions` to the number of degree-(>2) nodes — the
// structural witness the caller reports so a merge that silently over- or under-merges
// is visible in the log rather than hidden inside a passing recall figure.
std::vector<int> mergeArcsIntoBranches(const std::vector<OcctBranch>& branches,
                                       int& groupCount, int& junctions) {
  const std::size_t n = branches.size();
  std::vector<gp_Pnt> nodes;
  std::vector<int> degree;
  // Endpoint → node index, welding onto an existing node within kWeldTol. Linear scan:
  // this corpus tops out at 64 endpoints, so a spatial index would be pure overhead.
  auto nodeOf = [&](const gp_Pnt& p) {
    for (std::size_t i = 0; i < nodes.size(); ++i)
      if (nodes[i].Distance(p) < kWeldTol) return static_cast<int>(i);
    nodes.push_back(p);
    degree.push_back(0);
    return static_cast<int>(nodes.size()) - 1;
  };

  // Pass 1 — build the node set and node degrees.
  std::vector<std::pair<int, int>> ends(n);
  for (std::size_t i = 0; i < n; ++i) {
    double f = 0.0, l = 0.0;
    clampedRange(branches[i].curve, f, l);
    const int a = nodeOf(branches[i].curve->Value(f));
    const int b = nodeOf(branches[i].curve->Value(l));
    // A CLOSED arc welds both ends onto one node, contributing degree 2 there. It then
    // unites with itself below (a no-op) and stays its own branch — which is right: a full
    // closed loop is exactly one branch needing exactly one seed.
    ++degree[static_cast<std::size_t>(a)];
    ++degree[static_cast<std::size_t>(b)];
    ends[i] = {a, b};
  }

  // Pass 2 — union arcs meeting at a degree-2 node (unique continuation).
  ArcMerge merge(n);
  std::vector<int> firstArcAtNode(nodes.size(), -1);
  for (std::size_t i = 0; i < n; ++i)
    for (const int nd : {ends[i].first, ends[i].second}) {
      if (degree[static_cast<std::size_t>(nd)] != 2) continue;
      int& slot = firstArcAtNode[static_cast<std::size_t>(nd)];
      if (slot < 0) slot = static_cast<int>(i); else merge.unite(slot, static_cast<int>(i));
    }

  junctions = 0;
  for (const int d : degree) if (d > 2) ++junctions;

  // Densify the union-find roots into 0..groupCount-1.
  std::vector<int> group(n, -1);
  std::vector<int> rootToGroup(n, -1);
  groupCount = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const int r = merge.find(static_cast<int>(i));
    int& g = rootToGroup[static_cast<std::size_t>(r)];
    if (g < 0) g = groupCount++;
    group[i] = g;
  }
  return group;
}

// Report one pair. Runs native seeding + OCCT GeomAPI_IntSS, classifies OCCT branches,
// and asserts PER-BRANCH recall: every transversal OCCT branch must carry ≥ 1 native
// seed that lies on that OCCT curve (< onCurveTol) AND on both surfaces (< onSurfTol).
//   expectTransversalBranches  analytic truth, cross-checked against the classification.
//   tangentSine                threshold below which an OCCT branch is deemed tangential.
void reportPair(const std::string& pairName,
                const ssi::SurfaceAdapter& A, const ssi::SurfaceAdapter& B,
                const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                int expectTransversalBranches, double onSurfTol, double onCurveTol,
                double tangentSine, const ssi::SeedOptions& opt) {
  const ssi::SeedSet ss = ssi::seed_intersection(A, B, opt);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;

  // Classify every OCCT branch as transversal (recall denominator) or tangential (S4).
  std::vector<OcctBranch> branches;
  for (int i = 1; i <= occtN; ++i) branches.push_back(classifyBranch(iss.Line(i), sa, sb, tangentSine));

  // Collapse OCCT's arc PIECES into the branches a seeder is expected to cover (see
  // mergeArcsIntoBranches). Everything below counts merged branches, never arcs.
  int groupCount = 0, junctions = 0;
  const std::vector<int> group = mergeArcsIntoBranches(branches, groupCount, junctions);

  // A merged branch is transversal if ANY of its arcs is — consistent with the per-arc
  // rule, which already takes the MAX crossing sine along the arc (a branch that crosses
  // anywhere is a crossing locus, even where it grazes).
  std::vector<char> groupTransversal(static_cast<std::size_t>(groupCount), 0);
  for (std::size_t i = 0; i < branches.size(); ++i)
    if (branches[i].transversal) groupTransversal[static_cast<std::size_t>(group[i])] = 1;
  int transversal = 0, tangential = 0;
  for (const char t : groupTransversal) (t ? transversal : tangential)++;

  // Worst on-both-surfaces residual over the emitted seeds (a seed OFF a surface fails).
  double worstOnSurf = 0.0;
  for (const auto& s : ss.seeds)
    worstOnSurf = std::max(worstOnSurf,
                           std::max(distToOcctSurface(sa, s.point), distToOcctSurface(sb, s.point)));

  // PER-BRANCH recall: a transversal branch is COVERED iff some native seed lies on its
  // OCCT curve (< onCurveTol) and on both surfaces (< onSurfTol).
  // A merged transversal branch is COVERED iff some native seed lies on ANY of its arcs
  // (< onCurveTol) and on both surfaces (< onSurfTol). "Any arc" is the whole point of the
  // merge: the arcs of one branch are joined at degree-2 nodes, so a seed on any of them
  // seeds the entire branch.
  std::vector<char> groupCovered(static_cast<std::size_t>(groupCount), 0);
  for (std::size_t i = 0; i < branches.size(); ++i) {
    const auto g = static_cast<std::size_t>(group[i]);
    if (!groupTransversal[g] || groupCovered[g]) continue;
    for (const auto& s : ss.seeds) {
      const bool onCurve = distToOcctCurve(branches[i].curve, s.point) < onCurveTol;
      const bool onBoth = distToOcctSurface(sa, s.point) < onSurfTol &&
                          distToOcctSurface(sb, s.point) < onSurfTol;
      if (onCurve && onBoth) { groupCovered[g] = 1; break; }
    }
  }
  int covered = 0;
  for (const char c : groupCovered) if (c) ++covered;
  const double recall = transversal > 0 ? double(covered) / double(transversal) : 1.0;

  bool ok = true;
  if (worstOnSurf > onSurfTol) ok = false;             // a seed OFF a surface → fail
  if (covered < transversal) ok = false;               // missed a transversal branch → fail
  if (transversal != expectTransversalBranches) ok = false;  // oracle disagreed with analytic truth
  if (ok) ++g_pass; else ++g_fail;

  // `occtArcs` vs `occtBranches` are both printed on purpose: their RATIO is the diagnostic
  // that tells a reader whether a failure is a seeder miss or an arc-splitting artifact,
  // and `junctions` says which of the two the merge could not collapse.
  std::printf("[NSEED] %-4s %-20s recall=%.2f (%d/%d transversal) tang=%d nativeSeeds=%d "
              "occtArcs=%d occtBranches=%d junctions=%d deferredTangent=%d worstOnSurf=%.2e\n",
              ok ? "PASS" : "FAIL", pairName.c_str(), recall, covered, transversal,
              tangential, ss.branchCount(), occtN, groupCount, junctions,
              ss.deferredTangent, worstOnSurf);
  std::fflush(stdout);
}

// ── shared frame helpers ─────────────────────────────────────────────────────────

Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

// Build a clamped uniform knot vector of length nPoles + degree + 1 over [0,1]:
// (degree+1) zeros, interior knots evenly spaced, (degree+1) ones.
std::vector<double> clampedKnots(int degree, int nPoles) {
  const int m = nPoles + degree + 1;
  std::vector<double> k(static_cast<std::size_t>(m), 0.0);
  const int interior = nPoles - degree - 1;  // #interior knots
  for (int i = 0; i < m; ++i) {
    if (i <= degree) k[i] = 0.0;
    else if (i >= nPoles) k[i] = 1.0;
    else k[i] = double(i - degree) / double(interior + 1);
  }
  return k;
}

// OCCT Geom_BSplineSurface from a row-major native pole grid + clamped-uniform knots
// over [0,1]² (identical parametrization to makeBSplineAdapter). `deg` is the degree in
// both directions; `nR`×`nC` poles.
Handle(Geom_BSplineSurface) toOcctBSpline(const std::vector<Point3>& poles, int nR, int nC,
                                          int degU, int degV) {
  TColgp_Array2OfPnt cp(1, nR, 1, nC);
  for (int i = 0; i < nR; ++i)
    for (int j = 0; j < nC; ++j) {
      const Point3& p = poles[static_cast<std::size_t>(i) * nC + j];
      cp.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  // Distinct knots + multiplicities in each direction (clamped-uniform).
  auto distinct = [](int degree, int nPoles,
                     TColStd_Array1OfReal& knots, TColStd_Array1OfInteger& mults) {
    const int interior = nPoles - degree - 1;
    const int nDistinct = interior + 2;
    for (int i = 1; i <= nDistinct; ++i) {
      knots.SetValue(i, (i == 1) ? 0.0 : (i == nDistinct ? 1.0 : double(i - 1) / double(interior + 1)));
      mults.SetValue(i, (i == 1 || i == nDistinct) ? (degree + 1) : 1);
    }
  };
  const int nDistU = (nR - degU - 1) + 2, nDistV = (nC - degV - 1) + 2;
  TColStd_Array1OfReal knU(1, nDistU), knV(1, nDistV);
  TColStd_Array1OfInteger muU(1, nDistU), muV(1, nDistV);
  distinct(degU, nR, knU, muU);
  distinct(degV, nC, knV, muV);
  return new Geom_BSplineSurface(cp, knU, knV, muU, muV, degU, degV);
}

// ── pair builders (native + OCCT, identical geometry) ──────────────────────────────

// bspline ∩ bspline — two biquadratic (deg 2×2, 3×3 poles) freeform sheets: a bump
// opening +Z and a facing dish opening −Z, overlapping in one transversal loop.
void pairBSplineBSpline() {
  std::vector<Point3> bump = {
      {-1, -1, 0.6}, {-1, 0, 1.0}, {-1, 1, 0.6},
      { 0, -1, 1.0}, { 0, 0, 1.6}, { 0, 1, 1.0},
      { 1, -1, 0.6}, { 1, 0, 1.0}, { 1, 1, 0.6}};
  std::vector<Point3> dish = {
      {-1, -1, 1.4}, {-1, 0, 1.0}, {-1, 1, 1.4},
      { 0, -1, 1.0}, { 0, 0, 0.4}, { 0, 1, 1.0},
      { 1, -1, 1.4}, { 1, 0, 1.0}, { 1, 1, 1.4}};
  const auto kU = clampedKnots(2, 3), kV = clampedKnots(2, 3);
  auto A = ssi::makeBSplineAdapter(2, 2, bump, 3, 3, kU, kV);
  auto B = ssi::makeBSplineAdapter(2, 2, dish, 3, 3, kU, kV);
  Handle(Geom_Surface) sa = toOcctBSpline(bump, 3, 3, 2, 2);
  Handle(Geom_Surface) sb = toOcctBSpline(dish, 3, 3, 2, 2);
  ssi::SeedOptions opt; opt.initialGridU = 4; opt.initialGridV = 4;
  reportPair("bspline x bspline", A, B, sa, sb, /*branches=*/1, 1e-6, 1e-5, 1e-2, opt);
}

// bspline ∩ plane (MULTI-LOOP) — a rippled biquartic B-spline sheet (deg 2×2, 5×5
// poles, alternating high/low poles → an egg-carton) cut by the horizontal plane z=0.5;
// each ripple hump that pokes through the plane is its own transversal loop. The
// completeness stressor: the subdivision must reach every small loop.
void pairBSplinePlane() {
  const int n = 5;
  std::vector<Point3> poles;
  poles.reserve(static_cast<std::size_t>(n) * n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const double x = -1.0 + 2.0 * double(i) / (n - 1);
      const double y = -1.0 + 2.0 * double(j) / (n - 1);
      const double z = 0.5 + 0.55 * (((i + j) % 2 == 0) ? 1.0 : -1.0);  // egg-carton
      poles.push_back({x, y, z});
    }
  const auto kU = clampedKnots(2, n), kV = clampedKnots(2, n);
  auto A = ssi::makeBSplineAdapter(2, 2, poles, n, n, kU, kV);

  nm::Plane pl{Ax3{{0, 0, 0.5}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  ssi::ParamBox pd{-1.5, 1.5, -1.5, 1.5};
  auto B = ssi::makePlaneAdapter(pl, pd);

  Handle(Geom_Surface) sa = toOcctBSpline(poles, n, n, 2, 2);
  Handle(Geom_Surface) sb = new Geom_Plane(toOcctAx3(Ax3{{0, 0, 0.5}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));

  // Multi-loop: pre-split finely and subdivide deeper so every small loop is reached.
  ssi::SeedOptions opt; opt.initialGridU = 8; opt.initialGridV = 8; opt.minPatchFrac = 1.0 / 48.0;
  // 40 = the MEASURED branch count of this egg-carton's saddle network, not a guess and not
  // a number this file re-derives from the oracle at run time. The locus is one connected
  // component whose graph has 32 nodes — 16 degree-4 saddles (where the level set z=0.5
  // passes exactly through a saddle of the surface, so the two normals are parallel and
  // ‖n₁×n₂‖ → 0) and 16 degree-1 domain-boundary exits. Handshake: 16·4 + 16·1 = 80 = 2·40
  // edges, so 40 arcs with NO degree-2 node anywhere — nothing merges, and 40 branches each
  // require their own seed.
  //
  // The previous expectation was computed by re-running GeomAPI_IntSS here and counting what
  // the classifier called transversal. That made the `transversal != expect` check in
  // reportPair compare the oracle against itself: it could not fail, whatever the seeder or
  // OCCT did. A hardcoded 40 is what makes that assertion load-bearing — if OCCT's arc
  // splitting or the merge rule shifts, this pair now says so instead of silently absorbing it.
  reportPair("bspline x plane", A, B, sa, sb, /*branches=*/40, 1e-6, 1e-5, 1e-2, opt);
}

// skew cyl ∩ cyl — orthogonal unequal-radius cylinders → 2 transversal loops (the
// skew-quadric quartic S1 defers). OCCT may arc-split the loci; the per-branch match
// still requires a native seed on each transversal curve.
void pairSkewCylinders() {
  nm::Cylinder cz{frameZ(), 1.0};
  const Ax3 fx{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}};  // axis X
  nm::Cylinder cx{fx, 0.7};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -2.0, 2.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);
  Handle(Geom_Surface) sa = new Geom_CylindricalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(fx), 0.7);
  ssi::SeedOptions opt; opt.initialGridU = 4; opt.initialGridV = 3;
  reportPair("skew cyl unequal", A, B, sa, sb, /*branches=*/2, 1e-6, 1e-5, 1e-2, opt);
}

// sphere ∩ sphere — crossing equal-radius spheres → 1 transversal circle. OCCT may
// arc-split the circle; per-branch match requires a native seed on the shared circle.
void pairCrossingSpheres() {
  nm::Sphere s1{frameZ(), 1.0};
  nm::Sphere s2{frameZ({1.0, 0, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(frameZ({1.0, 0, 0})), 1.0);
  ssi::SeedOptions opt; opt.initialGridU = 3; opt.initialGridV = 3;
  reportPair("sphere x sphere", A, B, sa, sb, /*branches=*/1, 1e-6, 1e-5, 1e-2, opt);
}

}  // namespace

int main() {
  std::printf("== SSI Stage S2 subdivision-seeding native-vs-OCCT per-branch RECALL ==\n");
  std::fflush(stdout);

  pairBSplineBSpline();
  pairBSplinePlane();
  pairSkewCylinders();
  pairCrossingSpheres();

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
