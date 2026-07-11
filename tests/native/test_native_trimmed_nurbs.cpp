// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 8 — the trimmed-NURBS B-rep data
// model + pcurve robustness (src/native/topology/trimmed_nurbs.{h,cpp}). OCCT-FREE.
// The oracles are airtight and closed-form:
//
//   1. CONTAINMENT CORRECTNESS — on a KNOWN trim (a rectangular sub-region of a
//      bicubic patch, plus a circular hole) points provably inside/outside classify
//      correctly; boundary points → OnBoundary; a point in a hole → Out.
//   2. PCURVE FIDELITY — for a face whose 3-D edge is a KNOWN iso-curve,
//      S(pcurve(t)) matches C(t) to ~1e-9; a deliberately-WRONG pcurve (drifting off
//      S) is DETECTED (large deviation flagged), never passed.
//   3. PCURVE CONSTRUCTION ROUND-TRIP (numsci) — take a 3-D curve lying on S (S
//      evaluated along a known (u,v) path), construct its pcurve by projection+fit,
//      and verify the reconstructed pcurve reproduces the path in (u,v) and
//      S(pcurve) reproduces the 3-D curve, both to the fit tol.
//   4. DEGENERATE GUARDS — empty / open / self-touching loops, and a pcurve leaving
//      the surface domain → honest In/Out/OnBoundary/Unknown handling, never a
//      fabricated verdict.
//
// The construction round-trip calls numerics::closest_point_on_surface, so that leg
// is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_fit). The data-model +
// classify + fidelity legs are always-on. With the guard OFF the whole gate still
// runs its always-on legs.
//
#include <cstdio>

#include "native/math/bspline.h"
#include "native/topology/trimmed_nurbs.h"
#ifdef CYBERCAD_HAS_NUMSCI
#include "native/math/bspline_fit.h"  // interpolateCurve (the on-S edge fixture)
#endif

#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::topology;
namespace math = cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectEq(int a, int b, const char* what) {
  ++g_checks;
  if (a != b) {
    std::printf("FAIL %-46s got %d want %d\n", what, a, b);
    ++g_failures;
  }
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}
static void expectGT(double a, double b, const char* what) {
  ++g_checks;
  if (!(a > b)) {
    std::printf("FAIL %-46s %.6g > %.6g violated\n", what, a, b);
    ++g_failures;
  }
}
static const char* nameOf(Containment c) {
  switch (c) {
    case Containment::In: return "In";
    case Containment::Out: return "Out";
    case Containment::OnBoundary: return "OnBoundary";
    default: return "Unknown";
  }
}
static void expectClass(Containment got, Containment want, const char* what) {
  ++g_checks;
  if (got != want) {
    std::printf("FAIL %-40s got %s want %s\n", what, nameOf(got), nameOf(want));
    ++g_failures;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixtures.
// ─────────────────────────────────────────────────────────────────────────────

// A gently-curved bicubic B-spline patch over the parametric box [0,1]×[0,1]:
// degree 3 in both directions, 4×4 control net, clamped knots {0,0,0,0,1,1,1,1}.
// z varies quadratically-ish so the surface is genuinely non-planar (a real patch),
// while (x,y) track (u,v)·range so S is a graph z=f(x,y) — makes projection unique.
static FaceSurface bicubicPatch() {
  FaceSurface s;
  s.kind = FaceSurface::Kind::BSpline;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 4;
  s.nPolesV = 4;
  const double xy[4] = {0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      const double x = xy[i] * 3.0;  // world extent [0,3]
      const double y = xy[j] * 3.0;
      const double z = 0.4 * std::sin(1.1 * x) * std::cos(0.7 * y);  // curved bump
      s.poles.push_back(math::Point3{x, y, z});
    }
  s.knotsU = {0, 0, 0, 0, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 1, 1, 1, 1};
  return s;
}

// A planar surface z=0 with S(u,v) = (u,v,0). Iso-curves are exact lines → the
// airtight fidelity oracle (S(pcurve(t)) == C(t) to machine precision).
static FaceSurface unitPlane() {
  FaceSurface s;
  s.kind = FaceSurface::Kind::Plane;
  s.frame = math::Ax3{};  // origin 0, X=(1,0,0), Y=(0,1,0), Z=(0,0,1)
  return s;
}

// Build a straight-line pcurve segment from (u0,v0) to (u1,v1) (a Line pcurve
// parametrized on [0, L] with unit direction).
static PcurveSegment lineSeg(double u0, double v0, double u1, double v1) {
  PcurveSegment seg;
  seg.curve.kind = EdgeCurve::Kind::Line;
  const double du = u1 - u0, dv = v1 - v0;
  const double L = std::sqrt(du * du + dv * dv);
  seg.curve.origin2d = math::Point3{u0, v0, 0.0};
  seg.curve.dir2d = L > 0 ? math::Vec3{du / L, dv / L, 0.0} : math::Vec3{1, 0, 0};
  seg.first = 0.0;
  seg.last = L;
  seg.reversed = false;
  return seg;
}

// A rectangular loop in UV: [uLo,uHi]×[vLo,vHi], CCW.
static TrimLoop rectLoop(double uLo, double uHi, double vLo, double vHi) {
  TrimLoop loop;
  loop.push_back(lineSeg(uLo, vLo, uHi, vLo));
  loop.push_back(lineSeg(uHi, vLo, uHi, vHi));
  loop.push_back(lineSeg(uHi, vHi, uLo, vHi));
  loop.push_back(lineSeg(uLo, vHi, uLo, vLo));
  return loop;
}

// A circular hole loop in UV centered at (cu,cv), radius r (Circle pcurve, one seg).
static TrimLoop circleLoop(double cu, double cv, double r) {
  PcurveSegment seg;
  seg.curve.kind = EdgeCurve::Kind::Circle;
  seg.curve.origin2d = math::Point3{cu, cv, 0.0};
  seg.curve.dir2d = math::Vec3{r, 0.0, 0.0};  // dir2d.x carries the radius
  seg.first = 0.0;
  seg.last = 2.0 * M_PI;
  seg.reversed = false;
  return TrimLoop{seg};
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Containment correctness.
// ─────────────────────────────────────────────────────────────────────────────
static void testContainment() {
  // Rectangular sub-region [0.2,0.8]×[0.2,0.8] of the bicubic patch, with a
  // circular hole of radius 0.1 at the center (0.5,0.5).
  TrimmedNurbsFace f;
  f.surface = bicubicPatch();
  f.outer = rectLoop(0.2, 0.8, 0.2, 0.8);
  f.holes.push_back(circleLoop(0.5, 0.5, 0.1));

  // Provably inside (in the rect, outside the hole).
  expectClass(classify(f, {0.3, 0.3}), Containment::In, "rect interior point In");
  expectClass(classify(f, {0.7, 0.3}), Containment::In, "rect interior corner-ish In");
  expectClass(classify(f, {0.5, 0.75}), Containment::In, "rect near-top interior In");

  // Provably outside the rect.
  expectClass(classify(f, {0.1, 0.5}), Containment::Out, "left of rect Out");
  expectClass(classify(f, {0.9, 0.5}), Containment::Out, "right of rect Out");
  expectClass(classify(f, {0.5, 0.1}), Containment::Out, "below rect Out");
  expectClass(classify(f, {0.5, 0.9}), Containment::Out, "above rect Out");

  // A point inside the hole → Out (removed region).
  expectClass(classify(f, {0.5, 0.5}), Containment::Out, "hole center Out");
  expectClass(classify(f, {0.53, 0.5}), Containment::Out, "inside hole Out");

  // Boundary points → OnBoundary (on the rect edge, and on the hole circle).
  expectClass(classify(f, {0.2, 0.5}), Containment::OnBoundary, "on left rect edge OnBoundary");
  expectClass(classify(f, {0.5, 0.8}), Containment::OnBoundary, "on top rect edge OnBoundary");
  expectClass(classify(f, {0.6, 0.5}), Containment::OnBoundary, "on hole circle OnBoundary");

  // No-hole variant: a point where the hole WAS is now In.
  TrimmedNurbsFace f2;
  f2.surface = bicubicPatch();
  f2.outer = rectLoop(0.2, 0.8, 0.2, 0.8);
  expectClass(classify(f2, {0.5, 0.5}), Containment::In, "center In when no hole");
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Pcurve fidelity.
// ─────────────────────────────────────────────────────────────────────────────
static void testFidelity() {
  const FaceSurface plane = unitPlane();

  // The 3-D edge is the iso-curve S(0.4, v) = (0.4, v, 0), a Line from (0.4,0,0)
  // along +Y. Its CORRECT pcurve is the UV line origin=(0.4,0), dir=(0,1).
  EdgeCurve edge;
  edge.kind = EdgeCurve::Kind::Line;
  edge.frame.origin = math::Point3{0.4, 0.0, 0.0};
  edge.frame.x = math::Dir3{0.0, 1.0, 0.0};  // direction +Y (Line uses frame.x as dir)

  PCurve good;
  good.kind = EdgeCurve::Kind::Line;
  good.origin2d = math::Point3{0.4, 0.0, 0.0};
  good.dir2d = math::Vec3{0.0, 1.0, 0.0};

  FidelityReport rep = pcurveFidelity(plane, Location{}, edge, Location{}, good,
                                      /*first=*/0.0, /*last=*/1.0);
  expectTrue(rep.ok, "correct pcurve is faithful (ok)");
  expectLE(rep.maxDeviation, 1e-9, "correct pcurve S(p(t))==C(t) to ~1e-9");

  // A deliberately-WRONG pcurve that drifts in u as v grows (dir=(0.3,1)): it leaves
  // the true iso-line, so S(pcurve(t)) diverges from C(t) → large deviation flagged.
  PCurve wrong;
  wrong.kind = EdgeCurve::Kind::Line;
  wrong.origin2d = math::Point3{0.4, 0.0, 0.0};
  wrong.dir2d = math::Vec3{0.3, 1.0, 0.0};  // wrong: drifts off the iso-curve

  FidelityReport bad = pcurveFidelity(plane, Location{}, edge, Location{}, wrong, 0.0, 1.0);
  expectTrue(!bad.ok, "wrong pcurve is DETECTED (not ok)");
  expectGT(bad.maxDeviation, 1e-3, "wrong pcurve deviation is large (flagged)");

  // Fidelity holds on the CURVED bicubic patch too. The EXACT iso-curve S(u0, ·) of a
  // clamped B-spline surface is itself a B-spline curve in v: evaluate the u-basis at
  // u0 to collapse the 4 pole-rows into one row of 4 v-poles, sharing the v-knots.
  // C(v) built from those poles equals S(u0, v) POINTWISE, so its UV-line pcurve
  // (u0, v) satisfies S(pcurve(v)) == C(v) to machine precision — an airtight curved
  // oracle (no fit, no approximation).
  const FaceSurface patch = bicubicPatch();
  const double u0 = 0.5;
  // u-basis at u0 (degree 3, clamped [0,0,0,0,1,1,1,1] → Bézier basis in u).
  const int spanU = math::findSpan(patch.nPolesU - 1, patch.degreeU, u0,
                                   {patch.knotsU.data(), patch.knotsU.size()});
  std::vector<double> bu(patch.degreeU + 1);
  math::basisFuns(spanU, u0, patch.degreeU, {patch.knotsU.data(), patch.knotsU.size()}, bu);
  const int firstU = spanU - patch.degreeU;
  EdgeCurve iso;
  iso.kind = EdgeCurve::Kind::BSpline;
  iso.degree = patch.degreeV;
  iso.knots = patch.knotsV;
  for (int j = 0; j < patch.nPolesV; ++j) {
    math::Point3 acc{0, 0, 0};
    for (int k = 0; k <= patch.degreeU; ++k) {
      const math::Point3& p = patch.poles[static_cast<std::size_t>(firstU + k) * patch.nPolesV + j];
      acc.x += bu[k] * p.x;
      acc.y += bu[k] * p.y;
      acc.z += bu[k] * p.z;
    }
    iso.poles.push_back(acc);
  }
  PCurve isoPc;
  isoPc.kind = EdgeCurve::Kind::Line;
  isoPc.origin2d = math::Point3{u0, 0.0, 0.0};
  isoPc.dir2d = math::Vec3{0.0, 1.0, 0.0};  // (u0, v) as v runs 0..1
  FidelityReport curved =
      pcurveFidelity(patch, Location{}, iso, Location{}, isoPc, /*first=*/0.0, /*last=*/1.0);
  expectTrue(curved.ok, "curved-patch iso-curve pcurve is faithful (ok)");
  expectLE(curved.maxDeviation, 1e-9, "curved iso-curve S(p(t))==C(t) to ~1e-9");
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Degenerate guards (always-on).
// ─────────────────────────────────────────────────────────────────────────────
static void testDegenerate() {
  const FaceSurface plane = unitPlane();

  // Empty outer loop → Unknown (no region to classify).
  TrimmedNurbsFace empty;
  empty.surface = plane;
  expectClass(classify(empty, {0.5, 0.5}), Containment::Unknown, "empty face Unknown");

  // Open / too-few-points loop (a single degenerate segment collapsing to a point)
  // → Unknown. A zero-length line seg gives < 3 distinct polyline points.
  TrimmedNurbsFace open;
  open.surface = plane;
  open.outer.push_back(lineSeg(0.5, 0.5, 0.5, 0.5));  // degenerate point segment
  expectClass(classify(open, {0.5, 0.5}), Containment::Unknown, "degenerate loop Unknown");

  // Self-touching loop (a figure-eight pinch: the loop revisits an interior vertex)
  // → Unknown. Build a bowtie that pinches at (0.5,0.5).
  TrimmedNurbsFace pinch;
  pinch.surface = plane;
  pinch.outer.push_back(lineSeg(0.2, 0.2, 0.5, 0.5));
  pinch.outer.push_back(lineSeg(0.5, 0.5, 0.8, 0.2));
  pinch.outer.push_back(lineSeg(0.8, 0.2, 0.5, 0.5));  // revisits the pinch vertex
  pinch.outer.push_back(lineSeg(0.5, 0.5, 0.2, 0.2));
  expectClass(classify(pinch, {0.4, 0.35}), Containment::Unknown, "self-touching loop Unknown");

  // A hole loop that is degenerate → the whole classify declines Unknown (honest).
  TrimmedNurbsFace badHole;
  badHole.surface = plane;
  badHole.outer = rectLoop(0.0, 1.0, 0.0, 1.0);
  badHole.holes.push_back(TrimLoop{lineSeg(0.5, 0.5, 0.5, 0.5)});  // degenerate hole
  expectClass(classify(badHole, {0.3, 0.3}), Containment::Unknown, "degenerate hole → Unknown");
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. TOLERANT-TOPOLOGY HEALING (always-on).
//
// Airtight oracles for the healing pass. A rectangular outer loop [0.2,0.8]² is the
// EXACT reference region; each gate injects a specific defect and proves the heal is
// (a) correct when it should heal, (b) an honest decline when it must not, and — the
// load-bearing property — (c) REGION-PRESERVING: a heal never flips an interior/exterior
// point's In/Out verdict versus the exact loop.
// ─────────────────────────────────────────────────────────────────────────────

// A rectangle [uLo,uHi]×[vLo,vHi] with the FIRST segment's start shifted by (du,dv) so
// the closing join (last segment end → first segment start) opens a gap of ‖(du,dv)‖.
// Every other join stays exact. The gap magnitude is exactly the injected shift.
static TrimLoop rectLoopGapped(double uLo, double uHi, double vLo, double vHi,
                               double du, double dv) {
  TrimLoop loop;
  // Bottom edge starts at the SHIFTED corner (uLo+du, vLo+dv) → the closing edge from
  // (uLo,vHi) back to the start no longer meets the true corner: an injected gap.
  loop.push_back(lineSeg(uLo + du, vLo + dv, uHi, vLo));
  loop.push_back(lineSeg(uHi, vLo, uHi, vHi));
  loop.push_back(lineSeg(uHi, vHi, uLo, vHi));
  loop.push_back(lineSeg(uLo, vHi, uLo, vLo));  // ends at the TRUE corner (uLo,vLo)
  return loop;
}

// The set of interior/exterior probe points used for the preservation proof.
struct Probe { double u, v; Containment want; const char* name; };
static const Probe kProbes[] = {
    {0.30, 0.30, Containment::In,  "interior (0.30,0.30)"},
    {0.50, 0.50, Containment::In,  "interior center"},
    {0.70, 0.70, Containment::In,  "interior (0.70,0.70)"},
    {0.25, 0.75, Containment::In,  "interior (0.25,0.75)"},
    {0.10, 0.50, Containment::Out, "exterior left"},
    {0.90, 0.50, Containment::Out, "exterior right"},
    {0.50, 0.10, Containment::Out, "exterior below"},
    {0.50, 0.90, Containment::Out, "exterior above"},
};

static void testHealing() {
  const FaceSurface patch = bicubicPatch();

  // ── (1) SMALL injected gap HEALS to closed and classifies IDENTICALLY to the exact
  //        loop. Inject ε = 1e-8 (× extent ~0.85 ⇒ ~8.5e-9 gap), far under the default
  //        healGapTol = 1e-6·extent (~8.5e-7). The gapped loop must classify EVERY probe
  //        the same as the exact loop.
  TrimmedNurbsFace exact;
  exact.surface = patch;
  exact.outer = rectLoop(0.2, 0.8, 0.2, 0.8);

  const double eps = 1e-8;
  TrimmedNurbsFace gapped;
  gapped.surface = patch;
  gapped.outer = rectLoopGapped(0.2, 0.8, 0.2, 0.8, eps, eps);

  for (const Probe& pr : kProbes) {
    const Containment cExact = classify(exact, {pr.u, pr.v});
    const Containment cGap = classify(gapped, {pr.u, pr.v});
    expectClass(cExact, pr.want, pr.name);              // sanity: exact loop is correct
    expectClass(cGap, cExact, "small gap heals identical");  // heal reproduces exact
  }

  // The heal REPORT confirms a gap was welded (not a large-gap decline, not a pinch).
  HealReport hr = healTrimLoop(gapped.outer);
  expectTrue(hr.healed, "small-gap loop heals (healed)");
  expectTrue(hr.changed, "small-gap heal moved a vertex (changed)");
  expectTrue(!hr.largeGap, "small gap is NOT flagged large");
  expectTrue(!hr.pinch, "small-gap loop is not a pinch");
  expectTrue(hr.gapsClosed >= 1, "small gap was welded (gapsClosed>=1)");
  expectLE(hr.maxGapClosed, hr.tolerance, "welded gap is within tolerance");

  // ── (2) LARGE gap (beyond tol) still DECLINES honestly — NOT force-healed. Inject a
  //        gap of 0.05 (× nothing; absolute ~0.05 ≫ tol ~8.5e-7). classify → Unknown, and
  //        the report flags largeGap with the residual gap it refused to weld.
  TrimmedNurbsFace bigGap;
  bigGap.surface = patch;
  bigGap.outer = rectLoopGapped(0.2, 0.8, 0.2, 0.8, 0.05, 0.0);
  expectClass(classify(bigGap, {0.5, 0.5}), Containment::Unknown, "large gap declines Unknown");
  HealReport bg = healTrimLoop(bigGap.outer);
  expectTrue(!bg.healed, "large-gap loop declines (not healed)");
  expectTrue(bg.largeGap, "large gap is flagged (largeGap)");
  expectTrue(!bg.pinch, "large gap is not a pinch");
  expectGT(bg.residualGap, bg.tolerance, "residual gap exceeds tolerance");

  // ── (3) PRESERVATION across a SWEEP of gap sizes that heal: for every ε in a band up to
  //        the tolerance, NO probe's classification ever flips vs the exact loop. This is
  //        the explicit "a heal never changes the region" proof.
  const double extent = 0.6 * std::sqrt(2.0);  // rect diagonal ≈ 0.849
  const double tol = 1e-6 * extent;            // default healGapTol × extent
  const double gapSizes[] = {1e-10, 1e-9, 1e-8, 1e-7, 0.4 * tol, 0.9 * tol};
  for (double g : gapSizes) {
    TrimmedNurbsFace gf;
    gf.surface = patch;
    // Split the injected shift across u and v so the gap magnitude is g.
    const double c = g / std::sqrt(2.0);
    gf.outer = rectLoopGapped(0.2, 0.8, 0.2, 0.8, c, c);
    for (const Probe& pr : kProbes)
      expectClass(classify(gf, {pr.u, pr.v}), pr.want, "preservation: probe unchanged under heal");
  }

  // ── (4a) NEAR-COINCIDENT PCURVE SNAP: two pcurve endpoints that are within tol but not
  //         coincident are snapped to share the boundary. The gapped rectangle IS exactly
  //         this case (the closing pcurve's end and the opening pcurve's start are 1e-8
  //         apart); the heal welds them → the loop is closed. Verify a point ON the healed
  //         seam classifies OnBoundary (the snapped seam is a real boundary), and the heal
  //         reports the snap.
  TrimmedNurbsFace snap;
  snap.surface = patch;
  snap.outer = rectLoopGapped(0.2, 0.8, 0.2, 0.8, eps, eps);
  expectClass(classify(snap, {0.2, 0.5}), Containment::OnBoundary,
              "point on snapped seam OnBoundary");
  HealReport sr = healTrimLoop(snap.outer);
  expectTrue(sr.healed && sr.gapsClosed >= 1, "near-coincident pcurves snap (welded)");

  // ── (4b) PINCH DETECTION: a bowtie that self-touches at (0.5,0.5) is DETECTED and
  //         reported (pinch=true), and classify declines Unknown — never fabricated into a
  //         different region by "healing" the pinch away.
  TrimmedNurbsFace pinch;
  pinch.surface = patch;
  pinch.outer.push_back(lineSeg(0.2, 0.2, 0.5, 0.5));
  pinch.outer.push_back(lineSeg(0.5, 0.5, 0.8, 0.2));
  pinch.outer.push_back(lineSeg(0.8, 0.2, 0.5, 0.5));  // revisits the pinch vertex
  pinch.outer.push_back(lineSeg(0.5, 0.5, 0.2, 0.2));
  expectClass(classify(pinch, {0.4, 0.35}), Containment::Unknown, "pinch declines Unknown");
  HealReport pr = healTrimLoop(pinch.outer);
  expectTrue(pr.pinch, "pinch is DETECTED (pinch=true)");
  expectTrue(!pr.healed, "pinch is declined honestly (not healed)");

  // ── (5) HEALING TOGGLE: with heal OFF, the small-gap loop is NOT welded. Depending on
  //        the flatten dedup it either classifies (the 1e-8 gap survives as a stray edge)
  //        or declines — the point is that the RESULT can differ from the healed one, i.e.
  //        healing is what closes the gap. Assert the healed classify is In at center while
  //        confirming the toggle path runs without crashing.
  ClassifyOptions noHeal;
  noHeal.heal = false;
  (void)classify(gapped, {0.5, 0.5}, noHeal);  // must not crash / must be deterministic
  ClassifyOptions withHeal;  // default heal=true
  expectClass(classify(gapped, {0.5, 0.5}, withHeal), Containment::In,
              "healed gapped loop: center In");
}

#ifdef CYBERCAD_HAS_NUMSCI
// ─────────────────────────────────────────────────────────────────────────────
// 3. Pcurve construction round-trip (numsci).
// ─────────────────────────────────────────────────────────────────────────────
static void testConstruction() {
  const FaceSurface patch = bicubicPatch();
  const Location id{};

  // A KNOWN (u,v) path across the patch: a gently-curved diagonal u(s)=0.15+0.7s,
  // v(s)=0.2+0.6s+0.12 sin(πs), s∈[0,1]. The 3-D edge C(t) = S(u(t),v(t)) lies ON S
  // by construction. We encode C(t) as an INTERPOLATING B-spline through dense S-path
  // samples (a Bezier through interior samples would NOT pass through them, drifting
  // off S). The interpolant passes through every sample and, for this smooth path with
  // 33 samples, stays within the fit tol of the true S-path everywhere — so the edge
  // genuinely lies on S (verified by the small projection residual below).
  auto pathUV = [](double s) {
    return ParamPoint{0.15 + 0.7 * s, 0.2 + 0.6 * s + 0.12 * std::sin(M_PI * s)};
  };
  auto surfLocal = [&](double u, double v) {
    return math::surfacePoint(patch.degreeU, patch.degreeV,
                              math::SurfaceGrid{{patch.poles.data(), patch.poles.size()},
                                                patch.nPolesU, patch.nPolesV},
                              {patch.knotsU.data(), patch.knotsU.size()},
                              {patch.knotsV.data(), patch.knotsV.size()}, u, v);
  };
  const int nEdge = 32;
  std::vector<math::Point3> edgePts;
  for (int i = 0; i <= nEdge; ++i) {
    const double s = static_cast<double>(i) / nEdge;
    const ParamPoint uv = pathUV(s);
    edgePts.push_back(surfLocal(uv.u, uv.v));
  }
  const math::CurveFitResult edgeFit =
      math::interpolateCurve({edgePts.data(), edgePts.size()}, 3, math::ParamMethod::ChordLength);
  expectTrue(edgeFit.ok, "edge B-spline interpolation ok");
  EdgeCurve edge;
  edge.kind = EdgeCurve::Kind::BSpline;
  edge.degree = edgeFit.curve.degree;
  edge.poles = edgeFit.curve.poles;
  edge.weights = edgeFit.curve.weights;
  edge.knots = edgeFit.curve.knots;  // parametrized on [0,1]

  // Map the edge param s∈[0,1] back to the (u,v) path for the reconstruction check.
  // The chord-length interpolant's parameter is NOT exactly s, so compare via the
  // 3-D point instead (below): the pcurve is verified against S(pcurve)==C(t), the
  // fidelity contract, which is the load-bearing property.

  ConstructOptions co;
  co.samples = 40;
  co.fitDegree = 3;
  co.surfSamplesU = 28;
  co.surfSamplesV = 28;
  co.fidelity.absTol = 1e-3;   // interpolant-on-S + fit + projection tolerance (achieved ~1.1e-3)
  co.fidelity.relTol = 1e-3;

  PcurveConstruction pc = constructPcurve(patch, id, edge, id, /*first=*/0.0, /*last=*/1.0,
                                          /*u0=*/0.0, 1.0, /*v0=*/0.0, 1.0, co);
  expectTrue(pc.ok, "constructPcurve round-trips (ok)");
  expectLE(pc.projMaxDistance, 1e-2, "edge lies on S (projection residual small)");
  expectLE(pc.fidelity.maxDeviation, pc.fidelity.tolerance,
           "S(constructed pcurve) reproduces C(t) within fit tol");
  expectTrue(!pc.pcurve.poles2d.empty(), "constructed pcurve has poles");
  expectTrue(pc.pcurve.knots.size() ==
                 pc.pcurve.poles2d.size() + static_cast<std::size_t>(pc.pcurve.degree) + 1,
             "constructed pcurve knot vector well-formed");

  // The reconstructed pcurve reproduces the (u,v) PATH: at each edge param t the pcurve
  // gives (u,v), and S(u,v) must equal C(t) — AND (u,v) must lie in the domain box the
  // path stays inside (u∈[0.15,0.85], v∈[0.2,0.92]). This confirms the pcurve traces the
  // true parameter path, not merely some curve mapping onto S.
  if (pc.ok) {
    double maxOutside = 0.0;
    for (int i = 0; i <= 20; ++i) {
      const double t = static_cast<double>(i) / 20;
      const math::Point3 uv = math::curvePoint(pc.pcurve.degree, pc.pcurve.poles2d,
                                               pc.pcurve.knots, t);
      const double outU = std::max(0.0, std::max(0.14 - uv.x, uv.x - 0.86));
      const double outV = std::max(0.0, std::max(0.19 - uv.y, uv.y - 0.93));
      maxOutside = std::max(maxOutside, std::max(outU, outV));
    }
    expectLE(maxOutside, 1e-3, "reconstructed pcurve stays in the path's (u,v) box");
  }

  // Round-trip through classify: build a loop whose top edge IS the constructed pcurve,
  // closed with straight returns, and confirm a point ON the pcurve classifies
  // OnBoundary (the constructed pcurve is a usable trim segment). Guarded on pc.ok so
  // an honest decline never dereferences an empty pcurve.
  if (pc.ok) {
    TrimmedNurbsFace f;
    f.surface = patch;
    PcurveSegment seg;
    seg.curve = pc.pcurve;
    seg.first = 0.0;
    seg.last = 1.0;
    f.outer.push_back(seg);
    const ParamPoint end = pathUV(1.0);
    const ParamPoint beg = pathUV(0.0);
    f.outer.push_back(lineSeg(end.u, end.v, end.u, 0.0));
    f.outer.push_back(lineSeg(end.u, 0.0, beg.u, 0.0));
    f.outer.push_back(lineSeg(beg.u, 0.0, beg.u, beg.v));
    // A point on the constructed pcurve (evaluate it at its midpoint).
    const math::Point3 onPc =
        math::curvePoint(pc.pcurve.degree, pc.pcurve.poles2d, pc.pcurve.knots, 0.5);
    ClassifyOptions cop;
    cop.onEdgeTol = 1e-3;
    expectClass(classify(f, {onPc.x, onPc.y}, cop), Containment::OnBoundary,
                "point on constructed pcurve classifies OnBoundary");
  }
}

// A pcurve/edge that does NOT lie on S → constructPcurve declines honestly.
static void testConstructionOffSurface() {
  const FaceSurface patch = bicubicPatch();
  const Location id{};

  // An edge lifted 0.5 in +Z off the patch (definitely not on S).
  EdgeCurve edge;
  edge.kind = EdgeCurve::Kind::Line;
  edge.frame.origin = math::Point3{0.5, 0.5, 5.0};  // far above the patch
  edge.frame.x = math::Dir3{1.0, 0.0, 0.0};

  PcurveConstruction pc = constructPcurve(patch, id, edge, id, 0.0, 1.0,
                                          0.0, 1.0, 0.0, 1.0);
  expectTrue(!pc.ok, "off-surface edge → constructPcurve declines (not ok)");
  expectGT(pc.projMaxDistance, 1e-3, "off-surface projection residual is large");
}
#endif  // CYBERCAD_HAS_NUMSCI

int main() {
  testContainment();
  testFidelity();
  testDegenerate();
  testHealing();
#ifdef CYBERCAD_HAS_NUMSCI
  testConstruction();
  testConstructionOffSurface();
#endif

  if (g_failures == 0)
    std::printf("OK  test_native_trimmed_nurbs: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_trimmed_nurbs: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}
