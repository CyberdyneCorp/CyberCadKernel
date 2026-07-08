// SPDX-License-Identifier: Apache-2.0
//
// test_native_analysis.cpp — GATE A (HOST ANALYTIC, no OCCT) for the MOAT M-GS
// analysis services: GS3 measurement (distance / angle) + GS4 curvature.
//
// Every assertion is against a HAND-DERIVED CLOSED FORM (do Carmo / analytic
// geometry) or a brute-force reference — OCCT is not linked. The two-gate
// discipline: this is gate (a); the native-vs-OCCT parity is gate (b) on the
// simulator. HONEST DECLINE is a first-class outcome: the decline cases assert a
// std::nullopt, never a compared number.
//
// Built only when CYBERCAD_HAS_NUMSCI=ON (the seed-and-refine distance path uses
// the numerics/closest_point minimizer, whose substrate lives under that flag).
//
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include "native/analysis/angle.h"
#include "native/analysis/curvature.h"
#include "native/analysis/distance.h"
#include "native/math/native_math.h"
#include "native/topology/shape.h"

namespace an = cybercad::native::analysis;
namespace nm = cybercad::native::math;
namespace nt = cybercad::native::topology;

static int g_pass = 0, g_fail = 0;
static void check(const char* name, bool ok, double got = 0, double want = 0) {
  if (ok) { ++g_pass; std::printf("  PASS %-46s\n", name); }
  else    { ++g_fail; std::printf("  FAIL %-46s got=%.10g want=%.10g\n", name, got, want); }
}
static bool close(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

static nm::Ax3 stdFrame(const nm::Point3& o = {0, 0, 0}) {
  return nm::Ax3{o, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
}

// ── GS3 distance: closed-form cells ──────────────────────────────────────────
static void test_distance_closed_form() {
  std::printf("[GS3 distance — closed form]\n");

  // point · infinite line: P=(0,3,4), line = x-axis through origin → d=5.
  auto pl = an::distPointLine({0, 3, 4}, {0, 0, 0}, {1, 0, 0});
  check("point·line perpendicular d=5", close(pl.distance, 5.0), pl.distance, 5.0);
  check("point·line foot on axis", close(pl.p2.x, 0) && close(pl.p2.y, 0) && close(pl.p2.z, 0));

  // point · segment: projection param t=2.5 clamps to the endpoint (2,0,0), so
  // the nearest distance from (5,1,0) is √((5−2)²+1²) = √10.
  auto ps = an::distPointSegment({5, 1, 0}, {0, 0, 0}, {2, 0, 0});
  check("point·segment clamp d=√10", close(ps.distance, std::sqrt(10.0)), ps.distance, std::sqrt(10.0));
  check("point·segment foot at endpoint", close(ps.p2.x, 2.0), ps.p2.x, 2.0);

  // point · plane: z=0 plane, P=(1,2,7) → d=7.
  auto pp = an::distPointPlane({1, 2, 7}, {0, 0, 0}, {0, 0, 1});
  check("point·plane d=7", close(pp.distance, 7.0), pp.distance, 7.0);

  // point · circle: unit circle in z=0, P=(0,0,4) on the axis → d=√(1+16)=√17.
  auto pc = an::distPointCircle({0, 0, 4}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 1.0);
  check("point·circle on-axis d=√17", close(pc.distance, std::sqrt(17.0)), pc.distance, std::sqrt(17.0));
  // point off-axis in plane: P=(5,0,0), R=1 → nearest is (1,0,0), d=4.
  auto pc2 = an::distPointCircle({5, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 1.0);
  check("point·circle in-plane d=4", close(pc2.distance, 4.0), pc2.distance, 4.0);

  // segment · segment SKEW: x-axis seg [(-2,0,0),(2,0,0)] and
  // y-axis seg at z=3 [(0,-2,3),(0,2,3)] → common perpendicular d=3.
  auto ss = an::distSegSeg({-2, 0, 0}, {2, 0, 0}, {0, -2, 3}, {0, 2, 3});
  check("segment·segment skew d=3", close(ss.distance, 3.0), ss.distance, 3.0);
  check("segment·segment skew feet at crossing", close(ss.p1.x, 0) && close(ss.p2.y, 0));

  // segment · segment PARALLEL: two x-parallel segments offset by 4 in y, the
  // overlapping-projection gap is the perpendicular offset 4.
  auto sp = an::distSegSeg({0, 0, 0}, {4, 0, 0}, {1, 4, 0}, {3, 4, 0});
  check("segment·segment parallel d=4", close(sp.distance, 4.0), sp.distance, 4.0);

  // segment · segment PARALLEL non-overlapping: gap is between the near endpoints.
  auto sp2 = an::distSegSeg({0, 0, 0}, {2, 0, 0}, {5, 0, 0}, {7, 0, 0});
  check("segment·segment parallel gap d=3", close(sp2.distance, 3.0), sp2.distance, 3.0);
}

// ── GS3 distance: Entity dispatcher + point·analytic-surface projection ───────
static void test_distance_entities() {
  std::printf("[GS3 distance — entities]\n");

  // vertex · plane face (dispatcher → closed form).
  nt::FaceSurface plane; plane.kind = nt::FaceSurface::Kind::Plane; plane.frame = stdFrame();
  auto vf = an::minDistance(an::Entity::ofVertex({2, 3, 9}),
                            an::Entity::ofFace(plane, -10, 10, -10, 10));
  check("vertex·plane d=9", vf && close(vf->distance, 9.0), vf ? vf->distance : -1, 9.0);

  // vertex · sphere surface (dispatcher → projection). Sphere R=2 at origin,
  // P=(10,0,0) → d = 10 − 2 = 8, witness at (2,0,0).
  nt::FaceSurface sph; sph.kind = nt::FaceSurface::Kind::Sphere; sph.frame = stdFrame(); sph.radius = 2.0;
  auto vs = an::minDistance(an::Entity::ofVertex({10, 0, 0}),
                            an::Entity::ofFace(sph, 0, 2 * M_PI, -M_PI / 2, M_PI / 2));
  check("vertex·sphere d=8", vs && close(vs->distance, 8.0, 1e-6), vs ? vs->distance : -1, 8.0);
  check("vertex·sphere witness at (2,0,0)", vs && close(vs->p2.x, 2.0, 1e-6), vs ? vs->p2.x : -1, 2.0);

  // vertex · circle edge (dispatcher → closed form). Circle R=1 z=0, P=(0,0,3).
  nt::EdgeCurve circ; circ.kind = nt::EdgeCurve::Kind::Circle; circ.frame = stdFrame(); circ.radius = 1.0;
  auto vc = an::minDistance(an::Entity::ofVertex({0, 0, 3}),
                            an::Entity::ofEdge(circ, 0, 2 * M_PI));
  check("vertex·circle d=√10", vc && close(vc->distance, std::sqrt(10.0)), vc ? vc->distance : -1, std::sqrt(10.0));

  // edge(line) · edge(line) skew dispatcher → exact segment-segment.
  nt::EdgeCurve lx; lx.kind = nt::EdgeCurve::Kind::Line;
  lx.frame = nm::Ax3{{-2, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
  nt::EdgeCurve ly; ly.kind = nt::EdgeCurve::Kind::Line;
  ly.frame = nm::Ax3{{0, -2, 3}, nm::Dir3{0, 1, 0}, nm::Dir3{-1, 0, 0}, nm::Dir3{0, 0, 1}};
  auto ee = an::minDistance(an::Entity::ofEdge(lx, 0, 4), an::Entity::ofEdge(ly, 0, 4));
  check("edge·edge skew lines d=3", ee && close(ee->distance, 3.0), ee ? ee->distance : -1, 3.0);
}

// ── GS3 distance: freeform edge·edge seed-and-refine vs brute force ───────────
static void test_distance_freeform() {
  std::printf("[GS3 distance — freeform seed/refine]\n");

  // Two well-separated non-rational Bézier curves. A: parabola near origin.
  // B: a shifted parabola 5 units up in z. Compare minDistance to a dense scan.
  nt::EdgeCurve a; a.kind = nt::EdgeCurve::Kind::Bezier; a.degree = 2;
  a.poles = {{-1, 0, 0}, {0, 1, 0}, {1, 0, 0}};
  nt::EdgeCurve b; b.kind = nt::EdgeCurve::Kind::Bezier; b.degree = 2;
  b.poles = {{-1, 0, 5}, {0, -1, 5}, {1, 0, 5}};

  auto ce = an::makeCurveEval(a, 0, 1); auto de = an::makeCurveEval(b, 0, 1);
  double bruteMin = 1e30;
  for (int i = 0; i <= 400; ++i)
    for (int j = 0; j <= 400; ++j) {
      nm::Point3 pa = ce.fn(i / 400.0), pb = de.fn(j / 400.0);
      bruteMin = std::min(bruteMin, nm::distance(pa, pb));
    }
  auto ff = an::minDistance(an::Entity::ofEdge(a, 0, 1), an::Entity::ofEdge(b, 0, 1));
  check("freeform edge·edge converges", ff.has_value());
  check("freeform edge·edge == brute force", ff && close(ff->distance, bruteMin, 1e-5),
        ff ? ff->distance : -1, bruteMin);
}

// ── GS4 curvature: analytic surfaces ─────────────────────────────────────────
static void test_curvature_analytic() {
  std::printf("[GS4 curvature — analytic]\n");

  nt::FaceSurface plane; plane.kind = nt::FaceSurface::Kind::Plane; plane.frame = stdFrame();
  auto cp = an::surfaceCurvature(plane, 0.3, 0.7);
  check("plane K=0 H=0", cp && close(cp->K, 0) && close(cp->H, 0));

  // sphere R=2: K=1/4, |H|=1/2, |k1|=|k2|=1/2.
  nt::FaceSurface sph; sph.kind = nt::FaceSurface::Kind::Sphere; sph.frame = stdFrame(); sph.radius = 2.0;
  auto cs = an::surfaceCurvature(sph, 1.0, 0.5);
  check("sphere K=1/R²=0.25", cs && close(cs->K, 0.25), cs ? cs->K : 0, 0.25);
  check("sphere |H|=1/R=0.5", cs && close(std::fabs(cs->H), 0.5), cs ? std::fabs(cs->H) : 0, 0.5);

  // cylinder R=3: K=0, |H|=1/6, principal {1/3,0}.
  nt::FaceSurface cyl; cyl.kind = nt::FaceSurface::Kind::Cylinder; cyl.frame = stdFrame(); cyl.radius = 3.0;
  auto cc = an::surfaceCurvature(cyl, 0.4, 1.2);
  check("cylinder K=0", cc && close(cc->K, 0), cc ? cc->K : 0, 0);
  check("cylinder |H|=1/(2R)=1/6", cc && close(std::fabs(cc->H), 1.0 / 6.0), cc ? std::fabs(cc->H) : 0, 1.0 / 6.0);
  check("cylinder principal {1/3,0}", cc && close(std::max(std::fabs(cc->k1), std::fabs(cc->k2)), 1.0 / 3.0) &&
                                      close(std::min(std::fabs(cc->k1), std::fabs(cc->k2)), 0.0));

  // cone α=π/6, reference radius 5 at v=0: ρ=5, kc=cos(π/6)/5, K=0, |H|=kc/2.
  nt::FaceSurface cone; cone.kind = nt::FaceSurface::Kind::Cone; cone.frame = stdFrame();
  cone.radius = 5.0; cone.semiAngle = M_PI / 6.0;
  auto cn = an::surfaceCurvature(cone, 0.7, 0.0);
  const double kc = std::cos(M_PI / 6.0) / 5.0;
  check("cone K=0", cn && close(cn->K, 0), cn ? cn->K : 0, 0);
  check("cone |H|=cosα/(2ρ)", cn && close(std::fabs(cn->H), kc / 2.0), cn ? std::fabs(cn->H) : 0, kc / 2.0);

  // torus R=5,r=2: v=0 → K=1/14, |H|=9/28; v=π/2 → K=0.
  nt::FaceSurface tor; tor.kind = nt::FaceSurface::Kind::Torus; tor.frame = stdFrame();
  tor.radius = 5.0; tor.minorRadius = 2.0;
  auto t0 = an::surfaceCurvature(tor, 1.1, 0.0);
  check("torus v=0 K=1/14", t0 && close(t0->K, 1.0 / 14.0), t0 ? t0->K : 0, 1.0 / 14.0);
  check("torus v=0 |H|=9/28", t0 && close(std::fabs(t0->H), 9.0 / 28.0), t0 ? std::fabs(t0->H) : 0, 9.0 / 28.0);
  auto tt = an::surfaceCurvature(tor, 0.3, M_PI / 2.0);
  check("torus v=π/2 K=0", tt && close(tt->K, 0), tt ? tt->K : 0, 0);
}

// ── GS4 curvature: freeform surfaces (fundamental forms) ──────────────────────
static void test_curvature_freeform_surface() {
  std::printf("[GS4 curvature — freeform surface]\n");

  // Non-rational biquadratic Bézier for S(u,v)=(2u−1, 2v−1, (2u−1)²+(2v−1)²),
  // i.e. the paraboloid z=x²+y². At u=v=0.5 (x=y=0): K=4, |H|=2 (Monge form).
  nt::FaceSurface par; par.kind = nt::FaceSurface::Kind::Bezier;
  par.nPolesU = 3; par.nPolesV = 3;
  const double xi[3] = {-1, 0, 1}, uz[3] = {1, -1, 1}, vz[3] = {1, -1, 1};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      par.poles.push_back({xi[i], xi[j], uz[i] + vz[j]});
  auto pb = an::surfaceCurvature(par, 0.5, 0.5);
  check("paraboloid K=4", pb && close(pb->K, 4.0, 1e-7), pb ? pb->K : 0, 4.0);
  check("paraboloid |H|=2", pb && close(std::fabs(pb->H), 2.0, 1e-7), pb ? std::fabs(pb->H) : 0, 2.0);

  // Rational (NURBS) quarter-cylinder R=3: an exact circular arc (rational
  // quadratic, weights [1,√2/2,1]) extruded in z. K=0, |H|=1/(2R)=1/6.
  nt::FaceSurface qc; qc.kind = nt::FaceSurface::Kind::Bezier;
  qc.nPolesU = 3; qc.nPolesV = 2;  // U = arc (deg 2), V = extrusion (deg 1)
  const double R = 3.0, w = std::sqrt(2.0) / 2.0;
  const nm::Point3 arc[3] = {{R, 0, 0}, {R, R, 0}, {0, R, 0}};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j)
      qc.poles.push_back({arc[i].x, arc[i].y, j == 0 ? 0.0 : 4.0});
  const double wa[3] = {1.0, w, 1.0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j) qc.weights.push_back(wa[i]);
  auto qb = an::surfaceCurvature(qc, 0.5, 0.5);
  check("rational quarter-cylinder K=0", qb && close(qb->K, 0.0, 1e-7), qb ? qb->K : 0, 0.0);
  check("rational quarter-cylinder |H|=1/6", qb && close(std::fabs(qb->H), 1.0 / 6.0, 1e-7),
        qb ? std::fabs(qb->H) : 0, 1.0 / 6.0);
}

// ── GS4 curvature: edges ──────────────────────────────────────────────────────
static void test_curvature_edges() {
  std::printf("[GS4 curvature — edges]\n");

  nt::EdgeCurve line; line.kind = nt::EdgeCurve::Kind::Line; line.frame = stdFrame();
  auto kl = an::edgeCurvature(line, 3.0);
  check("line κ=0", kl && close(*kl, 0), kl ? *kl : -1, 0);

  nt::EdgeCurve circ; circ.kind = nt::EdgeCurve::Kind::Circle; circ.frame = stdFrame(); circ.radius = 4.0;
  auto kc = an::edgeCurvature(circ, 1.0);
  check("circle κ=1/R=0.25", kc && close(*kc, 0.25), kc ? *kc : -1, 0.25);

  nt::EdgeCurve ell; ell.kind = nt::EdgeCurve::Kind::Ellipse; ell.frame = stdFrame();
  ell.radius = 3.0; ell.minorRadius = 2.0;  // a=3,b=2; at t=0 κ = a/b² = 3/4.
  auto ke = an::edgeCurvature(ell, 0.0);
  check("ellipse κ(t=0)=a/b²=0.75", ke && close(*ke, 0.75), ke ? *ke : -1, 0.75);

  // Non-rational Bézier parabola C(t)=(t,t²,0): κ = 2/(1+4t²)^{3/2}; t=0.5 → 1/√2.
  nt::EdgeCurve par; par.kind = nt::EdgeCurve::Kind::Bezier; par.degree = 2;
  par.poles = {{0, 0, 0}, {0.5, 0, 0}, {1, 1, 0}};
  auto kp = an::edgeCurvature(par, 0.5);
  check("bezier parabola κ(0.5)=1/√2", kp && close(*kp, 1.0 / std::sqrt(2.0), 1e-9),
        kp ? *kp : -1, 1.0 / std::sqrt(2.0));
}

// ── GS3 angle ─────────────────────────────────────────────────────────────────
static void test_angle() {
  std::printf("[GS3 angle]\n");

  nt::EdgeCurve lx; lx.kind = nt::EdgeCurve::Kind::Line;
  lx.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
  nt::EdgeCurve l45; l45.kind = nt::EdgeCurve::Kind::Line;
  l45.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{1, 1, 0}, nm::Dir3{-1, 1, 0}, nm::Dir3{0, 0, 1}};
  auto all = an::angle(an::Entity::ofEdge(lx, 0, 1), an::Entity::ofEdge(l45, 0, 1));
  check("line·line θ=45°", all && close(*all, M_PI / 4.0), all ? *all : -1, M_PI / 4.0);

  nt::FaceSurface pz; pz.kind = nt::FaceSurface::Kind::Plane;
  pz.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
  nt::FaceSurface px; px.kind = nt::FaceSurface::Kind::Plane;
  px.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}, nm::Dir3{1, 0, 0}};
  auto app = an::angle(an::Entity::ofFace(pz, 0, 1, 0, 1), an::Entity::ofFace(px, 0, 1, 0, 1));
  check("plane·plane θ=90°", app && close(*app, M_PI / 2.0), app ? *app : -1, M_PI / 2.0);

  // line along z vs plane z=0 (normal z): line ⟂ plane → asin(1)=90°.
  nt::EdgeCurve lz; lz.kind = nt::EdgeCurve::Kind::Line;
  lz.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{0, 0, 1}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}};
  auto alp = an::angle(an::Entity::ofEdge(lz, 0, 1), an::Entity::ofFace(pz, 0, 1, 0, 1));
  check("line·plane θ=90°", alp && close(*alp, M_PI / 2.0), alp ? *alp : -1, M_PI / 2.0);
  // in-plane line (x-axis) vs plane z=0 → asin(0)=0.
  auto alp0 = an::angle(an::Entity::ofEdge(lx, 0, 1), an::Entity::ofFace(pz, 0, 1, 0, 1));
  check("line·plane in-plane θ=0", alp0 && close(*alp0, 0.0), alp0 ? *alp0 : -1, 0.0);
}

// ── HONEST DECLINE cases (assert std::nullopt, never a number) ────────────────
static void test_declines() {
  std::printf("[honest declines]\n");

  // Curvature at the cone apex (ρ=0): analytic decline.
  nt::FaceSurface cone; cone.kind = nt::FaceSurface::Kind::Cone; cone.frame = stdFrame();
  cone.radius = 0.0; cone.semiAngle = M_PI / 6.0;  // apex at v=0
  auto capex = an::surfaceCurvature(cone, 0.5, 0.0);
  check("DECLINE cone apex curvature", !capex.has_value());

  // Freeform parametric singularity: a bilinear Bézier patch whose u=1 edge is
  // collapsed to a point (P10==P11) → S_u=0, EG−F²=0 at u=1 → decline.
  nt::FaceSurface deg; deg.kind = nt::FaceSurface::Kind::Bezier; deg.nPolesU = 2; deg.nPolesV = 2;
  deg.poles = {{0, 0, 0}, {0, 1, 0}, {1, 0.5, 0}, {1, 0.5, 0}};  // row-major U outer
  auto dsing = an::surfaceCurvature(deg, 1.0, 0.5);
  check("DECLINE freeform parametric singularity", !dsing.has_value());
  auto dinterior = an::surfaceCurvature(deg, 0.4, 0.5);  // interior is well-posed
  check("freeform interior curvature well-posed", dinterior.has_value());

  // Edge curvature at a cusp: degree-2 Bézier with P0==P1 → C'(0)=0 → decline.
  nt::EdgeCurve cusp; cusp.kind = nt::EdgeCurve::Kind::Bezier; cusp.degree = 2;
  cusp.poles = {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}};
  auto kcusp = an::edgeCurvature(cusp, 0.0);
  check("DECLINE edge cusp curvature", !kcusp.has_value());

  // Angle for a non-line/plane pair: circle edge vs line edge → decline.
  nt::EdgeCurve circ; circ.kind = nt::EdgeCurve::Kind::Circle; circ.frame = stdFrame(); circ.radius = 1.0;
  nt::EdgeCurve lx; lx.kind = nt::EdgeCurve::Kind::Line; lx.frame = stdFrame();
  auto acl = an::angle(an::Entity::ofEdge(circ, 0, 2 * M_PI), an::Entity::ofEdge(lx, 0, 1));
  check("DECLINE angle circle·line", !acl.has_value());

  // Angle for a curved surface (cylinder) vs plane → decline.
  nt::FaceSurface cyl; cyl.kind = nt::FaceSurface::Kind::Cylinder; cyl.frame = stdFrame(); cyl.radius = 1.0;
  nt::FaceSurface pl; pl.kind = nt::FaceSurface::Kind::Plane; pl.frame = stdFrame();
  auto acyl = an::angle(an::Entity::ofFace(cyl, 0, 6, 0, 1), an::Entity::ofFace(pl, 0, 1, 0, 1));
  check("DECLINE angle cylinder·plane", !acyl.has_value());
}

int main() {
  std::printf("=== test_native_analysis (MOAT M-GS GS3/GS4, host-analytic gate) ===\n");
  test_distance_closed_form();
  test_distance_entities();
  test_distance_freeform();
  test_curvature_analytic();
  test_curvature_freeform_surface();
  test_curvature_edges();
  test_angle();
  test_declines();
  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
