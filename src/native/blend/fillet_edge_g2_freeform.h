// SPDX-License-Identifier: Apache-2.0
//
// fillet_edge_g2_freeform.h — native G2 (CURVATURE-CONTINUOUS) rolling-ball fillet
// where the TWO base faces are GENERAL FREEFORM NURBS surfaces (not analytic
// primitives). This is the Layer-4-NURBS freeform-substrate slice — the deepest
// generalization of the G2 fillet family: the existing G2 builders each read the
// end curvature from a KNOWN analytic form (a plane → 0, a sphere → 1/R, a
// straight-ruled cyl/cone → 0 in the meridian direction); here the end curvature is
// read from the LOCAL SECOND FUNDAMENTAL FORM of each freeform surface at the
// rolling-ball contact point. Everything else — the quintic cross-section with the
// pole rule q=(5/4)·κ·h², the deflection-bounded skin, the honest-decline discipline
// — carries over unchanged.
//
// ── THE THREE PIECES (task) ────────────────────────────────────────────────────
// 1. CONTACT / SPINE MARCHING. A ball of radius r seated in the concave dihedral
//    between faceA and faceB touches faceA at pA and faceB at pB, with the ball
//    centre c = pA + r·n̂A = pB + r·n̂B (n̂ the OUTWARD unit normal of each face,
//    which points from the contact point toward the ball centre for a fillet that
//    ADDS material into the concave pocket, or away for a convex round — the sign is
//    fixed per configuration by seeding, below). We march this tangency condition in
//    each surface's (u,v) parameter domain: given a station on faceA's contact
//    curve, the matching faceB contact point is the one whose centre coincides,
//    solved by a damped Newton step on the 2-D residual r(uB,vB) = ‖cB − cA‖ plus a
//    tangency correction. Sampling the shared spine parameter τ gives a sequence of
//    (pA, n̂A, pB, n̂B) stations.
// 2. G2 QUINTIC CROSS-SECTION. At each station the section lives in the plane through
//    pA and pB spanned by the two contact normals (the rolling-ball section plane).
//    The quintic B(s), s∈[0,1], has P0=pA, P5=pB, end tangents in each face's tangent
//    plane (G1), and END CURVATURE matching each face's NORMAL CURVATURE in the
//    section plane. The pole rule generalises verbatim:
//        q_i = (5/4) · κ_i · h_i²
//    with κ_i the LOCAL normal curvature of face i in the section-plane direction,
//    READ from the freeform second fundamental form:
//        κ_n(d) = (L·du² + 2M·du·dv + N·dv²) / (E·du² + 2F·du·dv + G·dv²),
//    where E,F,G are the first fundamental form (Su·Su, Su·Sv, Sv·Sv), L,M,N the
//    second (Suu·n̂, Suv·n̂, Svv·n̂), and (du,dv) is the parameter-space direction
//    whose surface image is the section-plane tangent at the contact point. All five
//    surface derivatives come from native-math nurbsSurfaceDerivs (order 2). The
//    analytic cases reduce EXACTLY: a plane has L=M=N=0 → κ=0 → q=0 (collinear
//    triple), a sphere is umbilic → κ=1/R in every direction, a cylinder has κ=0
//    along the ruling — so a NURBS representation of any of them reproduces the
//    existing analytic G2 section to rounding (the analytic-reduction oracle).
// 3. SKIN. Consecutive stations' quintic sections are lofted into quad strips
//    (station k→k+1 × section sample j→j+1). The fillet surface welds to nothing
//    here (it is returned as its own band of oriented polygons, the same output idiom
//    as the curved builders); the caller stitches it against the trimmed base faces.
//
// ── HONEST DECLINE (the hard-track discipline) ─────────────────────────────────
// A station is DECLINED (never emitted) when: the tangency Newton does not converge
// (no consistent rolling-ball seat — e.g. r exceeds the local CONCAVE curvature limit
// so the ball will not fit); the section would fold (the two contact normals are
// anti-parallel / the poles cross); or the section-plane curvature read is
// non-finite. A whole call returns EMPTY if ANY requested station declines — the
// caller falls through to OCCT — so a self-intersecting fillet is NEVER produced. The
// per-station decline reason is reported through FreeformFilletReport for the
// residual-map deliverable. NO tolerance is ever widened to force a pass.
//
// CLEAN-ROOM. Uses only src/native/math (bspline surface eval + vec) + the local
// quintic/pole helpers. OCCT-FREE (0 OCCT/Geom/BRep/TK refs). Header-only.
// clang++ -std=c++20. Piegl & Tiller (surface derivatives A3.6/A4.4); the rolling-ball
// / q=(5/4)κh² pole rule generalised to the freeform second fundamental form.
//
#ifndef CYBERCAD_NATIVE_BLEND_FILLET_EDGE_G2_FREEFORM_H
#define CYBERCAD_NATIVE_BLEND_FILLET_EDGE_G2_FREEFORM_H

#include "native/math/bspline.h"
#include "native/math/vec.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace cybercad::native::blend {

namespace ffmath = cybercad::native::math;

namespace ffdetail {

// Tolerances for the freeform fillet. kFfEps is the linear coincidence band; the
// Newton residual must fall below kFfNewtonTol (centre coincidence) for a station to
// seat. Both are TIGHT — the honest-decline discipline forbids widening them.
inline constexpr double kFfEps = 1e-9;
inline constexpr double kFfNewtonTol = 1e-10;
inline constexpr int kFfNewtonMaxIter = 64;

// Fullness of the freeform quintic section: the tangent leg h = kFfSpacing·|pB−pA|
// (a fraction of the contact-to-contact chord). Chord-proportional (like the curved
// sphere/cyl builders) keeps the section a clean SIMPLE arc pA→pB across the useful
// radius range; the skin re-checks non-crossing and declines otherwise.
inline constexpr double kFfSpacing = 0.25;

// ── A freeform NURBS surface, in the native-math evaluation representation. ─────
// Non-rational when `weights` is empty; rational (NURBS) otherwise. This is a VIEW
// (spans borrowed from the caller); the caller keeps the backing storage alive.
struct Surface {
  int degreeU = 0;
  int degreeV = 0;
  ffmath::SurfaceGrid grid;                 // pole grid (row-major U outer, V inner)
  std::span<const double> weights;          // empty → non-rational B-spline
  std::span<const double> knotsU;
  std::span<const double> knotsV;
};

// Evaluate surface point + all order-2 derivatives at (u,v). out layout matches
// nurbsSurfaceDerivs/surfaceDerivs: row-major 3×3, out[k*3+l] = ∂^{k+l}S/∂u^k∂v^l.
inline void surfaceDerivs2(const Surface& s, double u, double v,
                           std::array<ffmath::Vec3, 9>& out) {
  std::span<ffmath::Vec3> span{out.data(), out.size()};
  if (s.weights.empty())
    ffmath::surfaceDerivs(s.degreeU, s.degreeV, s.grid, s.knotsU, s.knotsV, u, v, 2, span);
  else
    ffmath::nurbsSurfaceDerivs(s.degreeU, s.degreeV, s.grid, s.weights, s.knotsU, s.knotsV, u, v,
                               2, span);
}

// Surface point.
inline ffmath::Point3 surfacePoint(const Surface& s, double u, double v) {
  if (s.weights.empty())
    return ffmath::surfacePoint(s.degreeU, s.degreeV, s.grid, s.knotsU, s.knotsV, u, v);
  return ffmath::nurbsSurfacePoint(s.degreeU, s.degreeV, s.grid, s.weights, s.knotsU, s.knotsV, u,
                                   v);
}

// Local differential geometry at (u,v): point, unit normal, Su/Sv/Suu/Suv/Svv, and
// the first/second fundamental form scalars. `ok` is false if the surface is
// degenerate there (Su×Sv null).
struct LocalGeom {
  ffmath::Point3 p;
  ffmath::Vec3 Su, Sv, Suu, Suv, Svv;
  ffmath::Vec3 n;         // UNIT surface normal (Su×Sv normalized)
  double E = 0, F = 0, G = 0;   // first fundamental form
  double L = 0, M = 0, N = 0;   // second fundamental form
  bool ok = false;
};

inline LocalGeom localGeom(const Surface& s, double u, double v) {
  std::array<ffmath::Vec3, 9> d;
  surfaceDerivs2(s, u, v, d);
  LocalGeom g;
  g.p = ffmath::Point3{d[0].x, d[0].y, d[0].z};   // out[0] = S (order 0)
  g.Su = d[1 * 3 + 0];                            // ∂S/∂u
  g.Sv = d[0 * 3 + 1];                            // ∂S/∂v
  g.Suu = d[2 * 3 + 0];                           // ∂²S/∂u²
  g.Suv = d[1 * 3 + 1];                           // ∂²S/∂u∂v
  g.Svv = d[0 * 3 + 2];                           // ∂²S/∂v²
  const ffmath::Vec3 cr = ffmath::cross(g.Su, g.Sv);
  const double cn = ffmath::norm(cr);
  if (!(cn > kFfEps)) return g;                    // degenerate parameterisation
  g.n = cr / cn;
  g.E = ffmath::dot(g.Su, g.Su);
  g.F = ffmath::dot(g.Su, g.Sv);
  g.G = ffmath::dot(g.Sv, g.Sv);
  g.L = ffmath::dot(g.Suu, g.n);
  g.M = ffmath::dot(g.Suv, g.n);
  g.N = ffmath::dot(g.Svv, g.n);
  g.ok = true;
  return g;
}

// Normal curvature of the surface at a point (given its LocalGeom) in the TANGENT
// direction whose image is the unit 3-D vector `t` (t must lie in the tangent plane).
// We first solve for the parameter-space direction (du,dv) with Su·du+Sv·dv = t
// (least-squares via the metric), then apply κ_n = II(d)/I(d). Returns 0 for a
// planar patch (L=M=N=0) — the exact analytic-plane reduction.
inline double normalCurvatureAlong(const LocalGeom& g, const ffmath::Vec3& t) {
  // Solve [E F; F G] (du,dv)^T = (Su·t, Sv·t)^T for the param-direction whose surface
  // image is t (the standard first-fundamental-form projection).
  const double a = ffmath::dot(g.Su, t);
  const double b = ffmath::dot(g.Sv, t);
  const double det = g.E * g.G - g.F * g.F;
  if (!(std::fabs(det) > kFfEps)) return 0.0;
  const double du = (a * g.G - b * g.F) / det;
  const double dv = (b * g.E - a * g.F) / det;
  const double I = g.E * du * du + 2.0 * g.F * du * dv + g.G * dv * dv;
  if (!(std::fabs(I) > kFfEps)) return 0.0;
  const double II = g.L * du * du + 2.0 * g.M * du * dv + g.N * dv * dv;
  return II / I;
}

// ── ROLLING-BALL CONTACT SEATING (the marching primitive) ──────────────────────
// The rolling-ball spine is the CENTRE LOCUS: the set of points equidistant r from
// BOTH freeform faces. Choosing a contact point on ONE face does NOT determine a seat
// on a CURVED face (it over-constrains — a ball touching faceA at a chosen point in
// general does not touch faceB at all). So the free variable is the BALL CENTRE c: for
// a center guess we drop the FOOTPOINT (nearest point) onto each surface, giving the
// contact points pA,pB and signed offset distances dA=|c−pA|, dB=|c−pB| with c−pA ∥ nA
// and c−pB ∥ nB (the footpoint property). We Newton-adjust c so dA=r AND dB=r; the
// resulting c is on the spine and pA,pB are the true tangency contacts. Marching the
// spine = seeding successive centers (warm-started along the previous seat).

// FOOTPOINT: project an external point `q` onto surface `s`, starting from (u0,v0).
// Newton on the orthogonality residual (q−S)·Su = 0, (q−S)·Sv = 0. Returns the
// converged (u,v) + LocalGeom there, or nullopt if it diverges / leaves the domain.
struct Footpoint {
  double u = 0, v = 0;
  LocalGeom g;
};
inline std::optional<Footpoint> footpoint(const Surface& s, const ffmath::Point3& q, double u0,
                                          double v0) {
  double u = u0, v = v0;
  for (int it = 0; it < kFfNewtonMaxIter; ++it) {
    const LocalGeom g = localGeom(s, u, v);
    if (!g.ok) return std::nullopt;
    const ffmath::Vec3 e = q - g.p;                  // q relative to the surface point
    const double f1 = ffmath::dot(e, g.Su);
    const double f2 = ffmath::dot(e, g.Sv);
    // Convergence: gradient of ½|q−S|² is (−f1,−f2); footpoint when both ≈ 0.
    if (std::fabs(f1) < kFfNewtonTol && std::fabs(f2) < kFfNewtonTol) return Footpoint{u, v, g};
    // Newton on the 2×2 system ∇f = 0. Jacobian (Gauss–Newton, dropping the second-
    // derivative term that vanishes at the footpoint): [Su·Su Su·Sv; Sv·Su Sv·Sv].
    const double det = g.E * g.G - g.F * g.F;
    if (!(std::fabs(det) > kFfEps)) return std::nullopt;
    const double du = (f1 * g.G - f2 * g.F) / det;
    const double dv = (f2 * g.E - f1 * g.F) / det;
    u += du;
    v += dv;
    if (!std::isfinite(u) || !std::isfinite(v)) return std::nullopt;
    // Stay in a padded [0,1]² so a wandering step honest-declines rather than reading
    // the clamped-knot extrapolation as a spurious seat.
    if (u < -0.05 || u > 1.05 || v < -0.05 || v > 1.05) return std::nullopt;
  }
  return std::nullopt;
}

struct ContactStation {
  ffmath::Point3 pA, pB;    // contact points on faceA / faceB
  ffmath::Vec3 nA, nB;      // UNIT normals oriented toward the ball centre
  ffmath::Point3 center;    // rolling-ball centre (on the spine)
  double kA = 0.0;          // faceA normal curvature in the section plane
  double kB = 0.0;          // faceB normal curvature in the section plane
  double uA = 0, vA = 0, uB = 0, vB = 0;
};

// Seat a ball of radius r whose centre is near `cGuess`, with footpoint searches warm-
// started at (uA0,vA0)/(uB0,vB0). sA,sB ∈ {+1,−1} pick which side of each surface the
// centre lies on (toward-centre normal orientation). Newton on the centre c drives the
// two footpoint distances to r simultaneously. Returns the seated station or nullopt
// (ball won't fit / no consistent seat / footpoint left the domain).
inline std::optional<ContactStation> seatCenter(const Surface& A, const Surface& B, double r,
                                                const ffmath::Point3& cGuess, double sA, double sB,
                                                double uA0, double vA0, double uB0, double vB0) {
  ffmath::Point3 c = cGuess;
  double uA = uA0, vA = vA0, uB = uB0, vB = vB0;
  for (int it = 0; it < kFfNewtonMaxIter; ++it) {
    const auto fa = footpoint(A, c, uA, vA);
    const auto fb = footpoint(B, c, uB, vB);
    if (!fa || !fb) return std::nullopt;
    uA = fa->u; vA = fa->v; uB = fb->u; vB = fb->v;
    // Signed offset directions: from the contact toward the centre = sX·n̂X (the
    // footpoint guarantees c−p is PARALLEL to the surface normal; sX fixes the sign).
    const ffmath::Vec3 nA = fa->g.n * sA;
    const ffmath::Vec3 nB = fb->g.n * sB;
    const ffmath::Vec3 eA = c - fa->g.p;   // ≈ dA·nA
    const ffmath::Vec3 eB = c - fb->g.p;   // ≈ dB·nB
    const double dA = ffmath::dot(eA, nA); // signed distance along the toward-centre normal
    const double dB = ffmath::dot(eB, nB);
    // Residual: (dA−r, dB−r) along the two normals. Solve for the centre move δc that
    // zeroes both. δc·nA ≈ δdA, δc·nB ≈ δdB (moving the centre along a normal changes
    // that distance ≈1:1 near the seat). Two constraints, three DOF — the third (along
    // the spine tangent nA×nB) is left free (that IS the spine direction), so we take
    // the minimum-norm move in span{nA,nB}: δc = α·nA + β·nB with the 2×2 Gram system.
    const double gAA = 1.0, gBB = 1.0, gAB = ffmath::dot(nA, nB);
    const double rA = -(dA - r), rB = -(dB - r);   // want δdA=rA, δdB=rB
    const double res = std::sqrt(rA * rA + rB * rB);
    if (res < kFfNewtonTol) {
      ContactStation st;
      st.pA = fa->g.p; st.pB = fb->g.p; st.nA = nA; st.nB = nB; st.center = c;
      st.uA = uA; st.vA = vA; st.uB = uB; st.vB = vB;
      const ffmath::Vec3 chord = st.pB - st.pA;
      auto inFaceTangent = [](const ffmath::Vec3& n, const ffmath::Vec3& toward) -> ffmath::Vec3 {
        return toward - n * ffmath::dot(n, toward);   // projection into the tangent plane
      };
      const ffmath::Vec3 tA = inFaceTangent(nA, chord);
      const ffmath::Vec3 tB = inFaceTangent(nB, ffmath::Vec3{-chord.x, -chord.y, -chord.z});
      if (ffmath::isNull(tA) || ffmath::isNull(tB)) return std::nullopt;
      st.kA = normalCurvatureAlong(fa->g, tA);
      st.kB = normalCurvatureAlong(fb->g, tB);
      if (!std::isfinite(st.kA) || !std::isfinite(st.kB)) return std::nullopt;
      return st;
    }
    // Gram solve [1 gAB; gAB 1](α,β) = (rA,rB); δc = α·nA + β·nB.
    const double det = gAA * gBB - gAB * gAB;
    if (!(std::fabs(det) > kFfEps)) return std::nullopt;   // normals (anti)parallel → no seat
    const double alpha = (rA * gBB - rB * gAB) / det;
    const double beta = (rB * gAA - rA * gAB) / det;
    const ffmath::Vec3 dc = nA * alpha + nB * beta;
    c = ffmath::Point3{c.x + dc.x, c.y + dc.y, c.z + dc.z};
    if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z)) return std::nullopt;
  }
  return std::nullopt;   // did not converge → honest decline (ball won't fit here)
}

// ── THE G2 QUINTIC SECTION (freeform end curvatures) ────────────────────────────
// Six 3-D poles P0..P5: P0=pA, P5=pB; end tangents in each face's tangent plane
// (G1); end curvatures matched to the freeform normal curvatures kA,kB via the
// generalised pole rule q_i=(5/4)·κ_i·h². Returns nullopt on degeneracy / fold.
struct FreeformSection {
  std::array<ffmath::Point3, 6> poles;
  ffmath::Vec3 nSection;   // section-plane normal (for the outward orientation)
};

inline std::optional<FreeformSection> freeformSection(const ContactStation& st) {
  const ffmath::Vec3 chord = st.pB - st.pA;
  const double chordLen = ffmath::norm(chord);
  if (!(chordLen > kFfEps)) return std::nullopt;

  // In-face section tangents (unit): the direction the section departs each contact
  // along, lying in that face's tangent plane and pointing toward the other contact.
  auto inFaceTangent = [](const ffmath::Vec3& n, const ffmath::Vec3& toward) -> ffmath::Vec3 {
    const ffmath::Vec3 t = toward - n * ffmath::dot(n, toward);
    return t;
  };
  ffmath::Vec3 tA = inFaceTangent(st.nA, chord);
  ffmath::Vec3 tB = inFaceTangent(st.nB, ffmath::Vec3{-chord.x, -chord.y, -chord.z});
  if (ffmath::isNull(tA) || ffmath::isNull(tB)) return std::nullopt;
  tA = tA / ffmath::norm(tA);
  tB = tB / ffmath::norm(tB);

  // In-section normal at each contact, toward the CENTRE OF CURVATURE. For a fillet
  // that adds material into the concave pocket, the section bows toward the ball
  // centre; the curvature-offset pole moves toward the surface's centre of curvature,
  // which along the section-plane tangent is −n̂ if κ>0 (convex outward) — but the
  // SIGN of the offset must reproduce the substrate's second-order shape. We use the
  // surface's own normal as the reference: the offset q·N̂ with N̂ = −n̂ (toward the
  // ball centre / into the material) so a positive κ curls the section the same way the
  // substrate curves. This reduces EXACTLY to the analytic builders: a plane (κ=0) →
  // q=0 (collinear triple); an umbilic sphere (κ=1/R) → q=(5/4)(1/R)h² toward the
  // sphere centre, matching curved_fillet_g2.
  const ffmath::Vec3 NA = st.nA * -1.0;   // toward the ball centre
  const ffmath::Vec3 NB = st.nB * -1.0;

  const double h = kFfSpacing * chordLen;
  const double qA = 1.25 * st.kA * h * h;   // (5/4)·κA·h²  → κ(0)=κA at faceA
  const double qB = 1.25 * st.kB * h * h;   // (5/4)·κB·h²  → κ(1)=κB at faceB

  FreeformSection sec;
  sec.poles[0] = st.pA;
  sec.poles[1] = st.pA + tA * h;
  sec.poles[2] = st.pA + tA * (2.0 * h) + NA * qA;
  sec.poles[5] = st.pB;
  sec.poles[4] = st.pB + tB * h;
  sec.poles[3] = st.pB + tB * (2.0 * h) + NB * qB;
  ffmath::Vec3 nsec = ffmath::cross(chord, st.nA + st.nB);
  if (ffmath::isNull(nsec)) nsec = st.nA;
  sec.nSection = nsec;
  return sec;
}

// de Casteljau evaluation of the 3-D quintic section at s∈[0,1].
inline ffmath::Point3 quinticPoint(const std::array<ffmath::Point3, 6>& poles, double s) {
  std::array<ffmath::Point3, 6> p = poles;
  for (int k = 1; k < 6; ++k)
    for (int i = 0; i < 6 - k; ++i)
      p[i] = ffmath::Point3{p[i].x + (p[i + 1].x - p[i].x) * s, p[i].y + (p[i + 1].y - p[i].y) * s,
                            p[i].z + (p[i + 1].z - p[i].z) * s};
  return p[0];
}

// Section 3-D curvature at s (the plane-curve curvature of the quintic in space). Used
// by the G2-to-face witness: κ(0) must equal the faceA normal curvature kA, κ(1) the
// faceB one kB. Analytic Bézier derivatives (hodographs), κ = |B'×B''| / |B'|³.
inline double sectionCurvature(const std::array<ffmath::Point3, 6>& poles, double s) {
  std::array<ffmath::Vec3, 5> d1;
  for (int i = 0; i < 5; ++i)
    d1[i] = ffmath::Vec3{5.0 * (poles[i + 1].x - poles[i].x), 5.0 * (poles[i + 1].y - poles[i].y),
                         5.0 * (poles[i + 1].z - poles[i].z)};
  std::array<ffmath::Vec3, 4> d2;
  for (int i = 0; i < 4; ++i)
    d2[i] = ffmath::Vec3{4.0 * (d1[i + 1].x - d1[i].x), 4.0 * (d1[i + 1].y - d1[i].y),
                         4.0 * (d1[i + 1].z - d1[i].z)};
  auto evalQuartic = [&](std::array<ffmath::Vec3, 5> p, double t) {
    for (int k = 1; k < 5; ++k)
      for (int i = 0; i < 5 - k; ++i) p[i] = p[i] + (p[i + 1] - p[i]) * t;
    return p[0];
  };
  auto evalCubic = [&](std::array<ffmath::Vec3, 4> p, double t) {
    for (int k = 1; k < 4; ++k)
      for (int i = 0; i < 4 - k; ++i) p[i] = p[i] + (p[i + 1] - p[i]) * t;
    return p[0];
  };
  const ffmath::Vec3 b1 = evalQuartic(d1, s);
  const ffmath::Vec3 b2 = evalCubic(d2, s);
  const double sp = ffmath::normSquared(b1);
  if (sp < 1e-18) return 0.0;
  return ffmath::norm(ffmath::cross(b1, b2)) / std::pow(sp, 1.5);
}

// Unit tangent of the section at s (for the G1 witness).
inline ffmath::Vec3 sectionTangent(const std::array<ffmath::Point3, 6>& poles, double s) {
  std::array<ffmath::Vec3, 5> d1;
  for (int i = 0; i < 5; ++i)
    d1[i] = ffmath::Vec3{5.0 * (poles[i + 1].x - poles[i].x), 5.0 * (poles[i + 1].y - poles[i].y),
                         5.0 * (poles[i + 1].z - poles[i].z)};
  for (int k = 1; k < 5; ++k)
    for (int i = 0; i < 5 - k; ++i) d1[i] = d1[i] + (d1[i + 1] - d1[i]) * s;
  return d1[0];
}

}  // namespace ffdetail

// ── PUBLIC RESULT TYPES ─────────────────────────────────────────────────────────

// The reason a station or a whole fillet declined (for the residual-map deliverable).
enum class FreeformFilletDecline {
  None,
  DegenerateSurface,     // Su×Sv null at a contact seed
  NewtonDiverged,        // no consistent rolling-ball seat (ball won't fit)
  SectionFold,           // the quintic poles cross (self-intersecting section)
  NonFiniteCurvature,    // the second-form read is not finite
  TooFewStations,        // fewer than 2 seated stations → cannot skin
};

// One evaluated station on the fillet spine.
struct FreeformFilletStation {
  ffdetail::ContactStation contact;
  ffdetail::FreeformSection section;
  bool seated = false;
  FreeformFilletDecline decline = FreeformFilletDecline::None;
};

// The full freeform fillet result: the skinned band (oriented triangle polygons as
// flat point loops), the per-station stations (for continuity witnesses), and the
// overall decline reason (None on success).
struct FreeformFilletResult {
  std::vector<FreeformFilletStation> stations;   // one per requested spine sample
  std::vector<std::array<ffmath::Point3, 3>> triangles;  // skinned fillet triangles
  FreeformFilletDecline decline = FreeformFilletDecline::None;
  bool ok() const { return decline == FreeformFilletDecline::None && !triangles.empty(); }
};

// Seed for the fillet march (CENTRE-LOCUS parametrisation). The caller supplies an
// initial ball-centre guess `center0`, the normal-orientation signs (sA,sB ∈ {+1,−1})
// picking which side of each face the centre lies on, warm-start footpoint params
// (uA0,vA0)/(uB0,vB0), a SPINE-STEP guide direction `spineDir` (an approximate world
// direction the crease runs — the march steps the centre along the true spine tangent
// nA×nB, oriented to agree with spineDir), a step length, and the number of stations.
// This decouples the (hard, configuration-specific) spine SEEDING from the (general)
// per-station centre seat + section + skin. On each station the seat floats the centre
// onto the true spine; the next station steps `stepLen` along the local spine tangent.
struct FreeformFilletSeed {
  ffmath::Point3 center0;               // initial rolling-ball centre guess
  ffmath::Vec3 spineDir{1, 0, 0};       // approximate crease-run direction (world)
  double stepLen = 0.1;                 // centre advance per station along the spine
  int nStations = 8;                    // number of spine stations (≥2)
  double sA = 1.0;
  double sB = 1.0;
  double uA0 = 0.5, vA0 = 0.5;
  double uB0 = 0.5, vB0 = 0.5;
};

// Build the G2 freeform fillet between two NURBS faces at radius r, seeded by `seed`.
// Marches the rolling-ball CENTRE along the spine, seats each station (footpoint on
// both faces + centre Newton to distance r), builds the freeform G2 quintic section,
// and skins consecutive sections into triangles. HONEST-DECLINES the whole result
// (empty triangles + a decline reason) if ANY station fails to seat / the section
// folds — never emits a self-intersecting fillet. `nSectionSamples` is the number of
// samples across each quintic section for the skin.
inline FreeformFilletResult fillet_edge_g2_freeform(const ffdetail::Surface& faceA,
                                                    const ffdetail::Surface& faceB, double r,
                                                    const FreeformFilletSeed& seed,
                                                    int nSectionSamples = 12) {
  FreeformFilletResult out;
  if (!(r > ffdetail::kFfEps) || seed.nStations < 2 || nSectionSamples < 1) {
    out.decline = FreeformFilletDecline::TooFewStations;
    return out;
  }
  out.stations.reserve(static_cast<std::size_t>(seed.nStations));

  ffmath::Point3 c = seed.center0;                       // marched centre
  double uA = seed.uA0, vA = seed.vA0, uB = seed.uB0, vB = seed.vB0;   // warm starts
  for (int k = 0; k < seed.nStations; ++k) {
    FreeformFilletStation stn;
    const auto seat =
        ffdetail::seatCenter(faceA, faceB, r, c, seed.sA, seed.sB, uA, vA, uB, vB);
    if (!seat) {
      stn.decline = FreeformFilletDecline::NewtonDiverged;
      out.stations.push_back(stn);
      out.decline = FreeformFilletDecline::NewtonDiverged;
      return out;   // whole fillet declines — no partial (leaky) emission
    }
    uA = seat->uA; vA = seat->vA; uB = seat->uB; vB = seat->vB;   // warm start next
    const auto sec = ffdetail::freeformSection(*seat);
    if (!sec) {
      stn.decline = FreeformFilletDecline::SectionFold;
      out.stations.push_back(stn);
      out.decline = FreeformFilletDecline::SectionFold;
      return out;
    }
    stn.contact = *seat;
    stn.section = *sec;
    stn.seated = true;
    out.stations.push_back(std::move(stn));

    // Step the centre along the local spine tangent (perpendicular to both contact
    // normals), oriented to run with the caller's spineDir guide. The next station's
    // footpoint/centre Newton refines it back onto the true spine.
    ffmath::Vec3 spineT = ffmath::cross(seat->nA, seat->nB);
    if (ffmath::isNull(spineT)) spineT = seed.spineDir;   // umbilic fallback
    if (ffmath::dot(spineT, seed.spineDir) < 0.0) spineT = spineT * -1.0;
    const double tn = ffmath::norm(spineT);
    if (tn > ffdetail::kFfEps) spineT = spineT / tn;
    c = ffmath::Point3{seat->center.x + spineT.x * seed.stepLen,
                       seat->center.y + spineT.y * seed.stepLen,
                       seat->center.z + spineT.z * seed.stepLen};
  }

  // SIMPLICITY GUARD: each section must be a clean simple arc pA→pB (no pole crossing);
  // a folded section revolves/skins into a self-intersecting band. Check monotone
  // progress of the sampled section (each successive sample farther along the chord).
  for (const auto& stn : out.stations) {
    const auto& poles = stn.section.poles;
    const ffmath::Vec3 chord = poles[5] - poles[0];
    const double cl2 = ffmath::normSquared(chord);
    if (!(cl2 > ffdetail::kFfEps * ffdetail::kFfEps)) {
      out.decline = FreeformFilletDecline::SectionFold;
      out.triangles.clear();
      return out;
    }
    double prev = -1e300;
    for (int j = 0; j <= 16; ++j) {
      const ffmath::Point3 pt = ffdetail::quinticPoint(poles, j / 16.0);
      const double proj = ffmath::dot(pt - poles[0], chord) / cl2;
      if (proj < prev - 1e-9) {   // section reversed along the chord → fold
        out.decline = FreeformFilletDecline::SectionFold;
        out.triangles.clear();
        return out;
      }
      prev = proj;
    }
  }

  // SKIN: loft consecutive stations' sections into quad strips (two triangles each).
  const int ns = static_cast<int>(out.stations.size());
  const int nj = nSectionSamples;
  auto rimAt = [&](int k, int j) -> ffmath::Point3 {
    const double s = static_cast<double>(j) / static_cast<double>(nj);
    return ffdetail::quinticPoint(out.stations[static_cast<std::size_t>(k)].section.poles, s);
  };
  out.triangles.reserve(static_cast<std::size_t>(ns - 1) * nj * 2);
  for (int k = 0; k < ns - 1; ++k)
    for (int j = 0; j < nj; ++j) {
      const ffmath::Point3 a = rimAt(k, j);
      const ffmath::Point3 b = rimAt(k, j + 1);
      const ffmath::Point3 c = rimAt(k + 1, j + 1);
      const ffmath::Point3 d = rimAt(k + 1, j);
      out.triangles.push_back({a, b, c});
      out.triangles.push_back({a, c, d});
    }
  return out;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_FILLET_EDGE_G2_FREEFORM_H
