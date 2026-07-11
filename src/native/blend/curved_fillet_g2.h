// SPDX-License-Identifier: Apache-2.0
//
// curved_fillet_g2.h — native G2 (CURVATURE-CONTINUOUS) fillet on a CURVED substrate:
// the CONVEX circular rim between a SPHERE lateral face and a coaxial planar cap (a
// truncated ball / dome / spherical plug). This is the Layer-4-NURBS curved-substrate
// slice — the generalization of the PLANAR G2 fillet (fillet_edges_g2.h) to a base face
// whose normal curvature at the tangency line is NON-ZERO.
//
// ── WHY THE CURVED CASE IS HARDER (the crux) ──────────────────────────────────────
// The planar G2 fillet uses a quintic section with COLLINEAR rail-triples so that
// B''(0)=B''(1)=0 → section curvature EXACTLY 0 at both rails, matching the flat faces'
// zero normal curvature. On a CURVED substrate the base face has NON-ZERO normal
// curvature at the tangency line, so G2 requires the section curvature at each rail to
// MATCH the substrate's normal curvature there — not zero. The collinear-triple rule
// (κ=0) is the special case of a more general pole placement that hits a PRESCRIBED end
// curvature.
//
// ── CURVATURE-MATCHING POLE PLACEMENT (closed form, PROVEN) ────────────────────────
// For a quintic Bézier B(s), s∈[0,1], poles P0..P5:
//     B'(0)  = 5 (P1 − P0),      B''(0) = 20 (P2 − 2 P1 + P0).
// Lay the near-rail poles in the local (tangent T̂, in-plane normal N̂) frame at P0:
//     P1 = P0 + h·T̂,            P2 = P1 + h·T̂ + q·N̂     (h = tangent leg, q = normal
//                                                          offset of P2 off the tangent)
// Then B'(0)=5h·T̂ and B''(0)=20q·N̂ (the tangent part of P2−2P1+P0 cancels), so
//     κ(0) = |B'(0) × B''(0)| / |B'(0)|³ = (100 h q) / (125 h³) = (4/5) · q / h².
// To MATCH a prescribed end curvature κ_base:   q = (5/4) · κ_base · h².
//   * κ_base = 0  ⇒  q = 0  ⇒  P0,P1,P2 COLLINEAR — exactly the planar collinear-triple.
//   * κ_base ≠ 0  ⇒  q = (5/4)κ_base h²  — P2 offset toward the substrate curvature centre.
// The symmetric algebra at s=1 (P4=P5−h₁T̂₁, P3=P4−h₁T̂₁+q₁N̂₁) gives
//     κ(1) = (4/5) q₁ / h₁²,   q₁ = (5/4) κ_base2 h₁².
// So EACH rail independently matches its own substrate normal curvature.
//
// ── THE SPHERE↔CAP SUBSTRATE (the one curved case landed) ─────────────────────────
// In the meridian half-plane (radial ρ, axial z, origin at the sphere centre) the G1
// rolling-ball fillet (curved_fillet.h::buildFilletedSphere) seats a torus tube (minor
// r) tangent to the sphere at the WALL seam W=(seamRad, seamAx) and to the cap at the
// CAP seam C=(Rmaj, capH). Its section is a CIRCLE of radius r → curvature 1/r at BOTH
// seams — a JUMP against the neighbours (sphere wants 1/R, cap wants 0). The G2 section
// is instead a quintic W→C whose end curvatures MATCH the substrates:
//   * WALL seam: a sphere is umbilic (normal curvature 1/R in every direction, so also in
//     the meridian section direction) ⇒ κ(0) = 1/R (NON-ZERO — the real curvature match);
//   * CAP seam: the plane has zero normal curvature ⇒ κ(1) = 0 (the collinear triple).
// The section leaves W along the sphere tangent (G1) curving toward the sphere centre by
// 1/R, and meets C along the cap tangent (G1) with zero curvature. Revolving that quintic
// about the axis gives the G2 blend surface, welded (SAME N angular samples) to the sphere
// wall band below and the trimmed cap disk above — identical weld idiom to the G1 builder,
// ONLY the meridian section curve changes (circular tube → curvature-matching quintic).
//
// ── SCOPE (honest) ─────────────────────────────────────────────────────────────────
// Native G2 ONLY for a CONVEX circular rim of a PURE truncated ball (one coaxial sphere
// wall + one coaxial axis-normal cap), radius r with the same ring-torus / seam-inside
// guards as the G1 sphere fillet. Everything else → NULL → OCCT:
//   * cylinder / cone / stepped / freeform substrates (the cylinder↔cap meridian normal
//     curvature is ZERO at both seams — that is the PLANAR-style κ=0 quintic, not the
//     genuine curvature-MATCH this slice proves, so it is left to the planar/OCCT paths);
//   * concave sphere rims, two-cap spherical zones, tilted caps, multiple picked edges.
// A merely-G1 (circular tube) blend is NEVER emitted here: the section is the
// curvature-MATCHING quintic (κ(0)=1/R, κ(1)=0) or nothing.
//
// CLEAN-ROOM. Reuses only src/native/blend/curved_fillet (sphere-rim geometry + weld
// helpers) + math + topology + boolean. No OCCT. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CURVED_FILLET_G2_H
#define CYBERCAD_NATIVE_BLEND_CURVED_FILLET_G2_H

#include "native/blend/curved_fillet.h"  // detail::sphereCapGeom / ringPoint / sagittaSteps

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// A point/vector in the meridian half-plane (radial ρ, axial z), measured from the sphere
// centre. Kept local so this header carries the SAME dependency footprint as curved_fillet.h
// (no 2-D vector type in native/math).
struct Mrd {
  double rho = 0.0;  // radial coordinate ρ (distance from the axis)
  double z = 0.0;    // axial coordinate z (along the axis, from the sphere centre)
  constexpr Mrd operator+(const Mrd& o) const noexcept { return {rho + o.rho, z + o.z}; }
  constexpr Mrd operator-(const Mrd& o) const noexcept { return {rho - o.rho, z - o.z}; }
  constexpr Mrd operator*(double s) const noexcept { return {rho * s, z * s}; }
};

// Fullness of the G2 quintic meridian section: the tangent leg h = kG2CurvedSpacing·|C−W|
// (a fraction of the seam-to-seam chord). Chord-proportional (not r-proportional) keeps the
// section a clean SIMPLE monotone arc W→C across the whole useful radius range — the two
// tangent legs (2h out from each rail) then never overshoot the chord and the poles never
// cross, which an r-proportional leg does for small r (a high wall-seam latitude packs the
// two ends close together). 0.25 leaves the section strictly monotone (ρ↓, z↑) and inside
// the sphere/cap bounds for r∈[~0.4, 2] on an R=5 ball; buildG2FilletedSphere re-checks
// monotonicity and declines otherwise.
inline constexpr double kG2CurvedSpacing = 0.25;

// A G2 curvature-MATCHING quintic meridian section for the sphere↔cap rim, in the
// meridian (radial ρ, axial z) plane measured from the sphere centre. Poles are laid so
// κ(0)=1/R at the sphere-wall seam and κ(1)=0 at the planar-cap seam (see the header
// derivation). Returns nullopt on the same degeneracies buildFilletedSphere rejects.
struct G2CurvedSection {
  std::array<Mrd, 6> poles;         // meridian (ρ, z) quintic poles P0..P5, P0=W, P5=C
  double R = 0.0;                   // sphere radius (for the wall-seam curvature witness)
  double r = 0.0;                   // rolling-ball radius (for the G1 control curvature)
  double kWall = 0.0;              // matched end curvature at the wall seam (= 1/R)
  double kCap = 0.0;               // matched end curvature at the cap seam  (= 0)
};

// Build the meridian section poles for the sphere↔cap rim. `g` is the recognised sphere
// cap geometry; `r` the rolling-ball radius. All geometry is in the (ρ, z) meridian
// half-plane; the caller revolves it about the axis.
inline std::optional<G2CurvedSection> g2CurvedSphereSection(const SphereCapGeom& g, double r) {
  const double R = g.R, capH = g.capH;
  if (!(r > kCurveEps) || !(R - r > kCurveEps)) return std::nullopt;

  // Rolling-ball centre circle (identical to buildFilletedSphere): axial Cz = capH − r,
  // radius Rmaj = √((R−r)² − Cz²) on the R−r offset sphere.
  const double Cz = capH - r;
  const double d = R - r;
  const double disc = d * d - Cz * Cz;
  if (!(disc > 1e-12)) return std::nullopt;
  const double Rmaj = std::sqrt(disc);
  if (!(Rmaj >= r - 1e-9)) return std::nullopt;                 // ring-torus guard
  const double vWall = std::atan2(Cz, Rmaj);
  if (!(vWall < kTwoPi / 4.0 - 1e-6)) return std::nullopt;      // seam would meet the cap

  // Seam positions in the meridian plane (same as the G1 builder's seam ring).
  const double seamRad = Rmaj + r * std::cos(vWall);            // ρ at the wall seam (on sphere)
  const double seamAx = Cz + r * std::sin(vWall);              // z at the wall seam
  const Mrd W{seamRad, seamAx};                                 // P0 — wall seam (on the sphere)
  const Mrd C{Rmaj, capH};                                     // P5 — cap seam (on the cap)

  // WALL-seam frame. The seam sits on the sphere at latitude latSeam; the sphere outward
  // normal there is radial-from-centre n̂ = (cos lat, sin lat). The section leaves along
  // the sphere tangent toward the cap (increasing latitude): T̂0 = (−sin lat, cos lat).
  // The curvature centre is the sphere centre, so N̂0 (toward the centre of curvature)
  // is the INWARD radial −n̂ = (−cos lat, −sin lat). κ(0) = 1/R (sphere is umbilic).
  const double latSeam = std::atan2(seamAx, seamRad);
  const Mrd t0{-std::sin(latSeam), std::cos(latSeam)};          // unit tangent at W (toward cap)
  const Mrd n0{-std::cos(latSeam), -std::sin(latSeam)};         // toward the sphere centre
  const double kWall = 1.0 / R;

  // CAP-seam frame. The section arrives at the cap along the cap plane. Going wall→cap the
  // radius shrinks from seamRad to Rmaj, so the tangent points inward: T̂1 = (−1, 0) at C
  // (the section is TANGENT to the cap plane there — G1). κ(1) = 0 (a plane). P4 sits a
  // tangent leg h back along +T̂1's reverse (i.e. P4 = C − h1·T̂1 = C + (h1, 0)).
  const double kCap = 0.0;

  // Tangent legs: h on each rail = kG2CurvedSpacing·|C−W| (a fraction of the seam chord).
  // The near-rail poles are P1=P0+h·T̂0, P2=P1+h·T̂0+q0·N̂0 with q0=(5/4)·kWall·h²;
  // symmetric at the cap (q1=(5/4)·kCap·h²=0 → collinear triple {P5,P4,P3}).
  const double chord = std::sqrt((C.rho - W.rho) * (C.rho - W.rho) + (C.z - W.z) * (C.z - W.z));
  if (!(chord > 1e-9)) return std::nullopt;
  const double h = kG2CurvedSpacing * chord;
  const double q0 = 1.25 * kWall * h * h;   // (5/4)·κ_base·h²  → κ(0)=1/R at the wall seam
  const double q1 = 1.25 * kCap * h * h;    // = 0  → κ(1)=0 at the cap seam (collinear)

  // Cap tangent (unit, pointing FROM the cap seam back INTO the section, i.e. −T̂1 = +ρ).
  const Mrd t1{-1.0, 0.0};                  // section tangent AT C (direction of travel wall→cap)
  const Mrd n1{0.0, 0.0};                   // cap curvature is zero → normal offset unused (q1=0)

  G2CurvedSection sec;
  sec.R = R;
  sec.r = r;
  sec.kWall = kWall;
  sec.kCap = kCap;
  sec.poles[0] = W;                                     // P0 = W (wall seam, on sphere)
  sec.poles[1] = W + t0 * h;                            // P1  (tangent leg along the sphere)
  sec.poles[2] = W + t0 * (2.0 * h) + n0 * q0;          // P2  (curvature-matching offset → 1/R)
  sec.poles[5] = C;                                     // P5 = C (cap seam, on the cap)
  sec.poles[4] = C - t1 * h;                            // P4  (tangent leg along the cap)
  sec.poles[3] = C - t1 * (2.0 * h) + n1 * q1;          // P3  (collinear triple; q1=0)
  return sec;
}

// de Casteljau evaluation of the meridian quintic at s∈[0,1] (six 2-D poles).
inline Mrd quinticMeridian(const std::array<Mrd, 6>& poles, double s) {
  std::array<Mrd, 6> p = poles;
  for (int k = 1; k < 6; ++k)
    for (int i = 0; i < 6 - k; ++i)
      p[i] = Mrd{p[i].rho + (p[i + 1].rho - p[i].rho) * s, p[i].z + (p[i + 1].z - p[i].z) * s};
  return p[0];
}

// Signed in-plane curvature of the meridian quintic at s (κ = (ρ'z''−z'ρ'')/(ρ'²+z'²)^{3/2}),
// via analytic Bézier derivatives. Used by the closed-form end-curvature witness.
inline double meridianCurvature(const std::array<Mrd, 6>& poles, double s) {
  // First/second hodograph of a quintic: B'(s) = 5·(quartic on P_{i+1}−P_i),
  // B''(s) = 20·(cubic on P_{i+2}−2P_{i+1}+P_i). Evaluate by de Casteljau on the
  // difference poles (the constant factors 5·4 fold into ρ'z''−z'ρ'' proportionally and
  // cancel nothing — kept explicit for a correct absolute curvature).
  std::array<Mrd, 5> d1;
  for (int i = 0; i < 5; ++i) d1[i] = Mrd{5.0 * (poles[i + 1].rho - poles[i].rho),
                                          5.0 * (poles[i + 1].z - poles[i].z)};
  std::array<Mrd, 4> d2;
  for (int i = 0; i < 4; ++i) d2[i] = Mrd{4.0 * (d1[i + 1].rho - d1[i].rho),
                                          4.0 * (d1[i + 1].z - d1[i].z)};
  auto evalQuartic = [&](const std::array<Mrd, 5>& p, double t) {
    std::array<Mrd, 5> q = p;
    for (int k = 1; k < 5; ++k)
      for (int i = 0; i < 5 - k; ++i)
        q[i] = Mrd{q[i].rho + (q[i + 1].rho - q[i].rho) * t, q[i].z + (q[i + 1].z - q[i].z) * t};
    return q[0];
  };
  auto evalCubic = [&](const std::array<Mrd, 4>& p, double t) {
    std::array<Mrd, 4> q = p;
    for (int k = 1; k < 4; ++k)
      for (int i = 0; i < 4 - k; ++i)
        q[i] = Mrd{q[i].rho + (q[i + 1].rho - q[i].rho) * t, q[i].z + (q[i + 1].z - q[i].z) * t};
    return q[0];
  };
  const Mrd b1 = evalQuartic(d1, s);
  const Mrd b2 = evalCubic(d2, s);
  const double sp = b1.rho * b1.rho + b1.z * b1.z;
  if (sp < 1e-18) return 0.0;
  return (b1.rho * b2.z - b1.z * b2.rho) / std::pow(sp, 1.5);
}

// Assemble the G2-filleted truncated ball as a planar-facet soup. Same weld idiom as
// buildFilletedSphere (faceted sphere wall + revolved section band + trimmed cap, all
// sharing N angular samples) — ONLY the middle band's meridian curve is the curvature-
// matching quintic instead of the circular torus tube. Empty on any degeneracy.
inline std::vector<nb::Polygon> buildG2FilletedSphere(const SphereCapGeom& g, double r,
                                                      double defl) {
  const auto sec = g2CurvedSphereSection(g, r);
  if (!sec) return {};
  const math::Ax3& ax = g.axis;
  const double R = g.R, capH = g.capH;

  // Meridian seam positions (P0=W wall seam on sphere, P5=C cap seam on the cap).
  const Mrd W = sec->poles[0];
  const Mrd C = sec->poles[5];
  const double seamRad = W.rho, seamAx = W.z;
  const double Rmaj = C.rho;                                     // cap disk radius

  // SIMPLICITY GUARD: the revolved meridian must be a clean SIMPLE arc — ρ monotone
  // decreasing and z monotone increasing wall→cap, staying at/below the cap height. A
  // section that folds (poles crossing) would revolve into a self-intersecting band the
  // weld cannot close watertight. Decline (→ NULL → OCCT) rather than emit a bad solid.
  {
    Mrd prev = quinticMeridian(sec->poles, 0.0);
    for (int i = 1; i <= 48; ++i) {
      const Mrd m = quinticMeridian(sec->poles, static_cast<double>(i) / 48.0);
      if (m.rho > prev.rho + 1e-9 || m.z < prev.z - 1e-9) return {};  // not monotone
      if (m.z > capH + 1e-9) return {};                               // overshoots the cap
      prev = m;
    }
  }
  const double latSeam = std::atan2(seamAx, seamRad);           // sphere latitude of the wall seam
  const double latSouth = -kTwoPi / 4.0;                        // south pole

  const int N = sagittaSteps(std::max(Rmaj, seamRad), kTwoPi, defl, 8, 256);  // angular
  // Section band steps: bound by the largest section curvature (the wall end, 1/R) AND the
  // ball radius r; sample generously so the deflection bound holds on the quintic.
  const int M = sagittaSteps(r, kTwoPi / 4.0, defl, 6, 96);     // meridian section steps
  const int K = sagittaSteps(R, latSeam - latSouth, defl, 4, 128);  // sphere wall latitude

  auto uAt = [&](int i) { return kTwoPi * i / N; };
  auto spherePoint = [&](double u, double lat) -> math::Point3 {
    return ringPoint(ax, R * std::cos(lat), u, R * std::sin(lat));
  };
  // A revolved section point: meridian (ρ, z) from the quintic at parameter s, placed at
  // azimuth u about the axis. This is the G2 blend surface (a surface of revolution).
  auto sectionPoint = [&](double u, double s) -> math::Point3 {
    const Mrd m = quinticMeridian(sec->poles, s);
    return ringPoint(ax, m.rho, u, m.z);
  };
  // SHARED seam rings, computed from ONE expression each so neighbouring bands are
  // BYTE-identical at the seam (a sphere-latitude / cap formula and the quintic endpoint
  // agree only to rounding; on FMA arm64 that gap leaks through the coarse tessellator).
  auto seamPoint = [&](double u) -> math::Point3 { return ringPoint(ax, seamRad, u, seamAx); };
  auto capRingPoint = [&](double u) -> math::Point3 { return ringPoint(ax, Rmaj, u, capH); };

  // Approximate outward normal of the revolved section at azimuth u, meridian param s: the
  // meridian tangent (dρ, dz) rotated −90° in the (ρ,z) plane gives the outward meridian
  // normal (nρ, nz); lift to 3-D via radial(u)·nρ + axis·nz. Sign fixed to agree with the
  // sphere/cap outward (points away from the axis / toward +cap).
  auto sectionNormal = [&](double u, double s) -> math::Vec3 {
    const double ds = 1e-4;
    const Mrd a = quinticMeridian(sec->poles, std::max(0.0, s - ds));
    const Mrd b = quinticMeridian(sec->poles, std::min(1.0, s + ds));
    const Mrd tang{b.rho - a.rho, b.z - a.z};
    // Outward meridian normal: rotate the (wall→cap) tangent by −90° → (dz, −dρ). For our
    // orientation (ρ decreasing, z increasing wall→cap: dρ<0, dz>0) this gives (+ρ, +z),
    // pointing outward (away from the axis / toward the +cap side) as the sphere/cap do.
    Mrd nrm{tang.z, -tang.rho};
    const double ln = std::sqrt(nrm.rho * nrm.rho + nrm.z * nrm.z);
    if (ln < 1e-12) return ax.z.vec();
    nrm = Mrd{nrm.rho / ln, nrm.z / ln};
    const math::Vec3 radial = ax.x.vec() * std::cos(u) + ax.y.vec() * std::sin(u);
    return radial * nrm.rho + ax.z.vec() * nrm.z;
  };

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * (K + M + 1) + 2);

  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  };
  auto emitTri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& cc,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(b - a, cc - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, b, cc}, nrm);
  };
  auto emitQuad = [&](const math::Point3& p00, const math::Point3& p10, const math::Point3& p11,
                      const math::Point3& p01, const math::Vec3& outward) {
    emitTri(p00, p10, p11, outward);
    emitTri(p00, p11, p01, outward);
  };

  // A sphere-wall ring at latitude index k∈[0,K]: the TOP ring (k==K) snaps to the shared
  // wall seam so it coincides with the section band's bottom ring.
  auto wallRingPoint = [&](double u, int k) -> math::Point3 {
    if (k >= K) return seamPoint(u);
    const double lat = latSouth + (latSeam - latSouth) * k / K;
    return spherePoint(u, lat);
  };
  // 1. Sphere wall: latSouth → latSeam, N·K quads (south-pole fan degenerates cleanly).
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    for (int k = 0; k < K; ++k) {
      const double lat0 = latSouth + (latSeam - latSouth) * k / K;
      const double lat1 = latSouth + (latSeam - latSouth) * (k + 1) / K;
      const double latm = 0.5 * (lat0 + lat1);
      const math::Vec3 radial = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
      const math::Vec3 outN = radial * std::cos(latm) + ax.z.vec() * std::sin(latm);
      emitQuad(wallRingPoint(u0, k), wallRingPoint(u1, k), wallRingPoint(u1, k + 1),
               wallRingPoint(u0, k + 1), outN);
    }
  }
  // A section-band ring at meridian index j∈[0,M]: the BOTTOM (j==0, s=0) snaps to the wall
  // seam and the TOP (j==M, s=1) snaps to the cap ring, so the revolved quintic band welds
  // EXACTLY to the sphere wall and the cap disk.
  auto sectionRingPoint = [&](double u, int j) -> math::Point3 {
    if (j <= 0) return seamPoint(u);
    if (j >= M) return capRingPoint(u);
    return sectionPoint(u, static_cast<double>(j) / M);
  };
  // 2. Revolved G2 section band: s ∈ [0,1] (wall→cap) × full turn, N·M quads.
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    for (int j = 0; j < M; ++j) {
      const double sm = (static_cast<double>(j) + 0.5) / M;
      emitQuad(sectionRingPoint(u0, j), sectionRingPoint(u1, j), sectionRingPoint(u1, j + 1),
               sectionRingPoint(u0, j + 1), sectionNormal(um, sm));
    }
  }
  // 3. Trimmed cap: disk radius Rmaj at axial capH, outward = capNormal.
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(capRingPoint(uAt(i)));
    emit(std::move(ring), g.capNormal);
  }
  return polys;
}

// ── G2 curvature-MATCHING fillet on a CYLINDER (and CONE) ↔ coaxial planar cap rim ──
//
// ── DIRECTION-DEPENDENT NORMAL CURVATURE (the derivation the task asks for) ─────────
// A cylinder (radius Rc) is a surface of revolution with a STRAIGHT meridian ruling. Its
// two principal normal curvatures at any wall point are:
//   * hoop (around the axis):   κ_hoop = 1/Rc   (the parallel circle),
//   * meridian (along the axis): κ_axial = 0     (the straight ruling).
// The blend is ALSO a surface of revolution: sweep a meridian section curve (ρ(s), z(s))
// about the axis. Its two principal curvatures at a seam point are
//   * meridian:  κ_m = the plane-curve curvature of the SECTION in the (ρ,z) plane — this
//                is exactly what the quintic controls,
//   * hoop:      κ_h = |n_ρ| / ρ  (radial component of the unit surface normal over ρ) —
//                fixed by the revolution ONCE the seam position + normal are fixed by G1.
// The section plane is the MERIDIAN half-plane, so "the substrate's normal curvature IN
// THE SECTION PLANE at the seam" is the cylinder's MERIDIAN (axial) normal curvature = 0
// (the straight ruling). Hence the genuine G2 match at the cylinder-wall seam is
//   κ_m(wall) = 0   (section-plane curvature; generalises the sphere's 1/R, here ZERO
//                    because the ruling is straight — this is the κ_section the task means),
// while the hoop curvature is matched AUTOMATICALLY by the revolution: G1 forces the blend
// normal radial at the wall seam ⇒ n_ρ=1, ρ=Rc ⇒ κ_h = 1/Rc == the cylinder's hoop 1/Rc.
// At the cap seam the blend normal is axial ⇒ n_ρ=0 ⇒ κ_h=0, and κ_m(cap)=0 — matching the
// FLAT cap's zero curvature in BOTH directions. So the fillet is G2 (both principal
// curvatures continuous) across BOTH circular seams on a genuinely CURVED substrate.
//
// This is NOT "the planar case": the substrate is curved (nonzero hoop / Gaussian
// environment), the seams are CIRCLES, and the blend is a surface of revolution whose hoop
// curvature match is a real second-order condition — it just happens that the SECTION-plane
// (meridian) curvature the quintic must hit is 0 at both straight-ruled seams, so the pole
// rule q=(5/4)·κ_section·h² collapses to the collinear triple (q=0) at each rail, exactly as
// the planar builder and the sphere's CAP end. The G1 torus tube, by contrast, has meridian
// curvature 1/r at BOTH seams — a JUMP against the substrate's 0. The quintic removes it.
//
// ── CONE (σ ≠ 0) ────────────────────────────────────────────────────────────────────
// A truncated cone's wall is ALSO straight-ruled, so κ_m(wall)=0 identically; only the wall
// tangent DIRECTION tilts by the half-angle σ (the section leaves the wall seam along the
// cone ruling, not the vertical). The hoop match is again automatic (blend normal ⟂ cone
// ruling ⇒ κ_h = cosσ/ρ == the cone's hoop curvature). So the SAME κ=0 quintic generalises
// cleanly to the cone: only the wall-seam frame (tangent + seam position) changes. The
// constant-radius fillet on a COAXIAL cone is a surface of revolution (same section at every
// azimuth), so the varying rim radius is NOT a variable-radius blend — it is fully handled.
// (A NON-coaxial / tilted-cap / multi-frustum body still declines through coneCapGeom.)

// Fullness of the G2 quintic section on a straight-ruled (cyl/cone) wall: h = a fraction of
// the seam chord |C−W| (chord-proportional, like the sphere case). Both end curvatures are 0
// so BOTH triples are collinear; 0.25 keeps the section a clean SIMPLE monotone arc W→C.
inline constexpr double kG2CylSpacing = 0.25;

// A G2 curvature-MATCHING quintic meridian section for a straight-ruled wall (cylinder or
// cone) ↔ coaxial planar cap rim, in the (ρ, z) meridian plane (z = axial coord along the
// cap-ward axis from the frame origin). κ(0)=0 at the wall seam (straight ruling) and
// κ(1)=0 at the cap seam — BOTH collinear triples. Poles P0=W (wall seam), P5=C (cap seam).
struct G2StraightSection {
  std::array<Mrd, 6> poles;   // meridian (ρ, z) quintic poles P0..P5, P0=W, P5=C
  double Rc = 0.0;            // wall radius at the seam (for the hoop-curvature witness)
  double r = 0.0;            // rolling-ball radius (for the G1 tube control curvature)
  double kWall = 0.0;        // matched section (meridian) curvature at the wall seam (= 0)
  double kCap = 0.0;         // matched section (meridian) curvature at the cap seam  (= 0)
  double kHoopWall = 0.0;    // hoop curvature the blend carries at the wall seam (= cosσ/Rc)
};

// Cylinder↔cap section. `g` is the recognised cylinder rim; `r` the rolling-ball radius.
// The wall tangent is AXIAL (vertical in (ρ,z)); the two collinear triples put κ=0 at both
// ends, matching the cylinder's straight-ruling meridian curvature (0) and the cap's 0.
inline std::optional<G2StraightSection> g2CurvedCylSection(const RimGeom& g, double r) {
  const double Rc = g.radius;
  const double R = Rc - r;                                    // trimmed-cap radius
  if (!(r > kCurveEps) || !(R > kCurveEps)) return std::nullopt;
  if (!(R >= r - 1e-12)) return std::nullopt;                 // ring-torus guard (Rc ≥ 2r)

  const double s = (g.capH >= g.farH) ? 1.0 : -1.0;          // +z toward the cap
  const double seamAx = g.capH - s * r;                       // wall seam axial (r below cap)
  if (s * (seamAx - g.farH) <= 1e-9) return std::nullopt;     // far end must clear the seam

  const Mrd W{Rc, seamAx};                                    // P0 — wall seam (on the cylinder)
  const Mrd C{R, g.capH};                                    // P5 — cap seam (on the cap)
  const Mrd t0{0.0, s};                                       // wall tangent (axial, toward cap)
  const Mrd t1{-1.0, 0.0};                                    // travel dir AT C (radius shrinks)

  const double chord = std::sqrt((C.rho - W.rho) * (C.rho - W.rho) + (C.z - W.z) * (C.z - W.z));
  if (!(chord > 1e-9)) return std::nullopt;
  const double h = kG2CylSpacing * chord;

  G2StraightSection sec;
  sec.Rc = Rc;
  sec.r = r;
  sec.kWall = 0.0;                                            // straight ruling → meridian κ=0
  sec.kCap = 0.0;                                            // flat cap → κ=0
  sec.kHoopWall = 1.0 / Rc;                                   // σ=0 → cosσ/Rc = 1/Rc
  sec.poles[0] = W;
  sec.poles[1] = W + t0 * h;                                  // collinear triple (q0 = 0)
  sec.poles[2] = W + t0 * (2.0 * h);
  sec.poles[5] = C;
  sec.poles[4] = C - t1 * h;                                  // collinear triple (q1 = 0)
  sec.poles[3] = C - t1 * (2.0 * h);
  return sec;
}

// Cone↔cap section. Same κ=0-at-both-ends quintic; only the wall-seam frame tilts by the
// cone half-angle σ. The seam geometry (centre circle, wall tangent point, seam axial) is
// taken from the SAME rolling-ball construction the G1 cone builder uses.
inline std::optional<G2StraightSection> g2CurvedConeSection(const ConeCapGeom& g, double r) {
  const double s = g.s, sigma = g.semiAngle;
  const double Rrim = g.Rref + g.capH * std::tan(sigma);      // cone radius at the rim
  if (!(Rrim > kCurveEps) || !(r > kCurveEps)) return std::nullopt;

  // r-z cross section (radial, sAxial toward +cap). Wall / cap outward normals; rolling-ball
  // centre C = rim − r·(nW+nC)/(1+c) (the same dihedral offset the G1 cone builder uses).
  const double dRdz = s * std::tan(sigma);
  const double wn = std::sqrt(1.0 + dRdz * dRdz);
  const double nWx = 1.0 / wn, nWy = -dRdz / wn;              // (radial, sAxial) wall outward
  const double nCx = 0.0, nCy = 1.0;                          // (radial, sAxial) cap outward
  const double c = nWx * nCx + nWy * nCy;
  if (c <= -1.0 + kCurveEps) return std::nullopt;
  const double Cr = Rrim - r * (nWx + nCx) / (1.0 + c);
  const double Cz = 0.0 - r * (nWy + nCy) / (1.0 + c);        // sAxial (below the cap)
  const double Rmaj = Cr;
  if (!(Rmaj >= r - 1e-9)) return std::nullopt;               // ring-torus guard
  const double angWall = std::atan2(nWy, nWx);                // wall tangent-point minor angle
  const double TwallR = Cr + r * std::cos(angWall);           // wall seam radius
  const double TwallZ = Cz + r * std::sin(angWall);           // wall seam sAxial (below cap)

  // Absolute (ρ, z) seam positions (z = axial coord along +cap from the frame origin).
  const double hCap = g.capH;
  const Mrd W{TwallR, hCap + s * TwallZ};                     // P0 — wall seam (on the cone)
  const Mrd C{Rmaj, hCap};                                    // P5 — cap seam (on the cap)
  if (!(s * (W.z - g.farH) > 1e-9)) return std::nullopt;      // wall seam stays on the frustum

  // Wall tangent: the cone ruling direction, oriented wall→cap. The cap is on the +cap-axial
  // side (z increases by +s toward it), so the tangent must carry that: along the ruling,
  // dρ/d(+cap-axial) = dRdz, so the wall→cap direction in (ρ, z) is (s·dRdz, s·1) = s·(dRdz,1)
  // — z-component has sign s (toward the cap) for BOTH a narrowing (dRdz<0, ρ shrinks) and a
  // widening (dRdz>0, ρ grows) frustum. The seam is r below the cap along the ruling, so the
  // section rises along the ruling to the cap seam. (For a widening cone ρ initially GROWS
  // along the ruling; the quintic then bends it back to the smaller cap radius Rmaj — a
  // section that is NOT ρ-monotone, so buildG2StraightBand's simplicity guard is ρ-relaxed
  // for cones, see below.)
  const Mrd tRuling{s * dRdz, s};                            // (dρ, dz) wall→cap along ruling
  const double tl = std::sqrt(tRuling.rho * tRuling.rho + tRuling.z * tRuling.z);
  const Mrd t0{tRuling.rho / tl, tRuling.z / tl};            // unit wall tangent toward cap
  const Mrd t1{-1.0, 0.0};                                    // travel dir AT the cap seam

  const double chord = std::sqrt((C.rho - W.rho) * (C.rho - W.rho) + (C.z - W.z) * (C.z - W.z));
  if (!(chord > 1e-9)) return std::nullopt;
  const double h = kG2CylSpacing * chord;

  G2StraightSection sec;
  sec.Rc = TwallR;
  sec.r = r;
  sec.kWall = 0.0;                                            // straight ruling → meridian κ=0
  sec.kCap = 0.0;
  sec.kHoopWall = std::cos(sigma) / TwallR;                   // cone hoop curvature at the seam
  sec.poles[0] = W;
  sec.poles[1] = W + t0 * h;                                  // collinear triple (q0 = 0)
  sec.poles[2] = W + t0 * (2.0 * h);
  sec.poles[5] = C;
  sec.poles[4] = C - t1 * h;                                  // collinear triple (q1 = 0)
  sec.poles[3] = C - t1 * (2.0 * h);
  return sec;
}

// Assemble a G2-filleted capped cylinder as a planar-facet soup. Same weld idiom as
// buildFilletedCylinder (far cap + wall band + revolved section band + trimmed cap, all
// sharing N angular samples); ONLY the middle band's meridian is the κ=0 quintic instead
// of the circular torus tube. `wallTangentIsAxial` selects the wall-band geometry (a
// cylinder wall is at constant radius Rc; a cone wall varies linearly). Empty on any
// degeneracy → NULL → OCCT.
inline std::vector<nb::Polygon> buildG2StraightBand(const math::Ax3& ax, const G2StraightSection& sec,
                                                    double capH, double farH, double wallRadFar,
                                                    const math::Vec3& capNormal, double defl) {
  const Mrd W = sec.poles[0];
  const Mrd C = sec.poles[5];
  const double s = (capH >= farH) ? 1.0 : -1.0;
  const double Rmaj = C.rho;                                  // trimmed-cap radius
  const double seamRad = W.rho, seamAx = W.z;

  // SIMPLICITY GUARD: the revolved meridian must be a clean SIMPLE arc. z STRICTLY monotone
  // toward the cap makes ρ a single-valued graph ρ(z), so the surface of revolution cannot
  // self-intersect (a folded band would need a z reversal). ρ need NOT be monotone — a
  // WIDENING cone's section rises along the (outward-tilted) ruling before bending back to
  // the smaller cap radius, so ρ bulges slightly; that bulge stays INSIDE the sharp cone
  // wall (bounded below), which the caller's wall envelope guarantees. z-monotone + the
  // cap-seam/wall-seam snaps keep the band watertight. Decline on any z reversal.
  {
    Mrd prev = quinticMeridian(sec.poles, 0.0);
    for (int i = 1; i <= 48; ++i) {
      const Mrd m = quinticMeridian(sec.poles, static_cast<double>(i) / 48.0);
      if (s * (m.z - prev.z) < -1e-9) return {};              // z must head toward the cap
      prev = m;
    }
  }

  const int N = sagittaSteps(std::max(seamRad, Rmaj), kTwoPi, defl, 8, 256);   // angular
  const int M = sagittaSteps(sec.r, kTwoPi / 4.0, defl, 6, 96);                // meridian
  auto uAt = [&](int i) { return kTwoPi * i / N; };

  auto sectionPoint = [&](double u, double sp) -> math::Point3 {
    const Mrd m = quinticMeridian(sec.poles, sp);
    return ringPoint(ax, m.rho, u, m.z);
  };
  auto seamPoint = [&](double u) -> math::Point3 { return ringPoint(ax, seamRad, u, seamAx); };
  auto capRingPoint = [&](double u) -> math::Point3 { return ringPoint(ax, Rmaj, u, capH); };
  auto sectionNormal = [&](double u, double sp) -> math::Vec3 {
    const double ds = 1e-4;
    const Mrd a = quinticMeridian(sec.poles, std::max(0.0, sp - ds));
    const Mrd b = quinticMeridian(sec.poles, std::min(1.0, sp + ds));
    const Mrd tang{b.rho - a.rho, b.z - a.z};
    // Outward meridian normal: rotate the (wall→cap) tangent −90° (dz, −dρ) for s=+1; the
    // sign folds through s so the normal points away from the axis / toward the +cap side.
    Mrd nrm{s * tang.z, -s * tang.rho};
    const double ln = std::sqrt(nrm.rho * nrm.rho + nrm.z * nrm.z);
    if (ln < 1e-12) return ax.z.vec();
    nrm = Mrd{nrm.rho / ln, nrm.z / ln};
    const math::Vec3 radial = ax.x.vec() * std::cos(u) + ax.y.vec() * std::sin(u);
    return radial * nrm.rho + ax.z.vec() * nrm.z;
  };

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * (M + 2) + 4);

  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  };
  auto emitTri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& cc,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(b - a, cc - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, b, cc}, nrm);
  };
  auto emitQuad = [&](const math::Point3& p00, const math::Point3& p10, const math::Point3& p11,
                      const math::Point3& p01, const math::Vec3& outward) {
    emitTri(p00, p10, p11, outward);
    emitTri(p00, p11, p01, outward);
  };

  // 1. Far cap: full disk radius wallRadFar at farH, outward = −capNormal.
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, wallRadFar, uAt(i), farH));
    emit(std::move(ring), capNormal * -1.0);
  }
  // 2. Wall band: farH (radius wallRadFar) → wall seam (radius seamRad, axial seamAx). A
  //    cylinder keeps radius constant; a cone tapers linearly — a single quad ring per
  //    sector spans it either way (both ends share N angular samples).
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    // Outward normal of the (possibly tilted) wall: the meridian from (wallRadFar,farH) to
    // (seamRad,seamAx) rotated −90° (folded through s), lifted to 3-D.
    const Mrd wt{seamRad - wallRadFar, seamAx - farH};
    Mrd wn{s * wt.z, -s * wt.rho};
    const double wl = std::sqrt(wn.rho * wn.rho + wn.z * wn.z);
    const math::Vec3 radial = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    const math::Vec3 outN = wl > 1e-12 ? radial * (wn.rho / wl) + ax.z.vec() * (wn.z / wl) : radial;
    emitQuad(ringPoint(ax, wallRadFar, u0, farH), ringPoint(ax, wallRadFar, u1, farH),
             seamPoint(u1), seamPoint(u0), outN);
  }
  // A section-band ring at meridian index j: BOTTOM (j=0) snaps to the wall seam, TOP (j=M)
  // snaps to the cap ring, so the revolved quintic welds EXACTLY to the wall + cap disk.
  auto sectionRingPoint = [&](double u, int j) -> math::Point3 {
    if (j <= 0) return seamPoint(u);
    if (j >= M) return capRingPoint(u);
    return sectionPoint(u, static_cast<double>(j) / M);
  };
  // 3. Revolved G2 section band: s ∈ [0,1] (wall→cap) × full turn, N·M quads.
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    for (int j = 0; j < M; ++j) {
      const double sm = (static_cast<double>(j) + 0.5) / M;
      emitQuad(sectionRingPoint(u0, j), sectionRingPoint(u1, j), sectionRingPoint(u1, j + 1),
               sectionRingPoint(u0, j + 1), sectionNormal(um, sm));
    }
  }
  // 4. Trimmed cap: disk radius Rmaj at axial capH, outward = capNormal.
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(capRingPoint(uAt(i)));
    emit(std::move(ring), capNormal);
  }
  return polys;
}

}  // namespace detail

// G2 (curvature-continuous) fillet on a single CONVEX CIRCULAR crease between a coaxial
// SPHERE lateral face and a coaxial planar cap (a truncated ball / dome) of `solid` with
// constant radius `r`. The blend is a REVOLVED curvature-MATCHING quintic section whose
// meridian curvature MATCHES the sphere's normal curvature 1/R at the wall seam and the
// cap's zero curvature at the cap seam — genuine curvature continuity across BOTH seams
// on a CURVED substrate. Returns the filleted solid (deflection-bounded planar-facet soup,
// watertight) or a NULL Shape (→ OCCT) when the edge is not a convex sphere↔cap circular
// rim, when the ring-torus / seam-inside guards fail, or on any degeneracy. The whole body
// must be a pure truncated ball (same wholesale classification as the G1 sphere_fillet_edge).
// A merely-G1 (circular tube) blend is NEVER produced here. Multiple picked edges → NULL.
inline topo::Shape curved_fillet_edge_g2(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                         double r, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r > kBlendEps)) return {};
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (edgeIds[0] < 1 || static_cast<std::size_t>(edgeIds[0]) > emap.size()) return {};
  const auto ce = topo::curveOf(emap.shape(edgeIds[0]));
  if (!ce || ce->curve->kind != topo::EdgeCurve::Kind::Circle) return {};

  const auto g = detail::sphereCapGeom(solid, edgeIds[0]);
  if (!g) return {};  // not a pure truncated-ball sphere↔cap rim → OCCT
  std::vector<nb::Polygon> polys = detail::buildG2FilletedSphere(*g, r, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

// G2 (curvature-continuous) fillet on a single CONVEX CIRCULAR crease between a coaxial
// CYLINDER lateral face and a coaxial planar cap of `solid` with constant radius `r`. The
// blend is a REVOLVED κ=0 quintic section whose MERIDIAN (section-plane) curvature MATCHES
// the cylinder's straight-ruling meridian normal curvature (0) at the wall seam and the
// flat cap's 0 at the cap seam, while the HOOP curvature is matched automatically by the
// revolution (1/Rc at the wall, 0 at the cap) — genuine G2 in BOTH principal directions
// across BOTH circular seams on a curved substrate. The G1 torus tube's 1/r meridian
// curvature is a JUMP the quintic removes; a merely-G1 blend is NEVER emitted here.
// Returns the filleted solid (deflection-bounded planar-facet soup, watertight) or a NULL
// Shape (→ OCCT) when the edge is not a convex cylinder↔cap circular rim, when Rc < 2r
// (ring-torus guard), when the wall seam would leave the wall, or on any degeneracy. Same
// wholesale cyl↔cap classification as the G1 curved_fillet_edge. Multiple picked edges → NULL.
inline topo::Shape curved_fillet_edge_g2_cyl(const topo::Shape& solid, const int* edgeIds,
                                             int edgeCount, double r, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r > kBlendEps)) return {};
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (edgeIds[0] < 1 || static_cast<std::size_t>(edgeIds[0]) > emap.size()) return {};
  const auto ce = topo::curveOf(emap.shape(edgeIds[0]));
  if (!ce || ce->curve->kind != topo::EdgeCurve::Kind::Circle) return {};

  detail::RimFaces rf;
  if (!detail::facesOnRim(solid, edgeIds[0], rf)) return {};  // not a cyl↔cap rim → OCCT
  const auto cyl = detail::cylinderInfo(solid, rf.cyl);
  if (!cyl) return {};
  const auto cap = facePlane(solid, rf.cap);
  if (!cap) return {};
  const auto g = detail::rimGeom(solid, edgeIds[0], *cyl, *cap);
  if (!g) return {};

  const auto sec = detail::g2CurvedCylSection(*g, r);
  if (!sec) return {};
  const double s = (g->capH >= g->farH) ? 1.0 : -1.0;
  (void)s;
  std::vector<nb::Polygon> polys =
      detail::buildG2StraightBand(g->axis, *sec, g->capH, g->farH, g->radius, g->capNormal, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

// G2 (curvature-continuous) fillet on a single CONVEX CIRCULAR crease between a coaxial
// CONE-FRUSTUM lateral face and a coaxial planar cap of `solid` with constant radius `r`.
// Identical curvature-matching argument to the cylinder arm — the cone wall is ALSO
// straight-ruled, so κ_meridian(wall)=0 and the hoop curvature cosσ/ρ is matched by the
// revolution — only the wall-seam tangent tilts by the half-angle σ (the κ=0 quintic leaves
// the wall along the cone ruling). The constant-radius fillet on a COAXIAL cone is a surface
// of revolution (no variable radius). Returns the filleted solid or a NULL Shape (→ OCCT)
// when the edge is not a convex cone↔cap circular rim, when the ring-torus / seam guards
// fail, or on any degeneracy — same wholesale capped-frustum classification as the G1
// cone_fillet_edge (a cylinder σ=0 is owned by the cyl arm). Multiple picked edges → NULL.
inline topo::Shape curved_fillet_edge_g2_cone(const topo::Shape& solid, const int* edgeIds,
                                              int edgeCount, double r, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r > kBlendEps)) return {};
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (edgeIds[0] < 1 || static_cast<std::size_t>(edgeIds[0]) > emap.size()) return {};
  const auto ce = topo::curveOf(emap.shape(edgeIds[0]));
  if (!ce || ce->curve->kind != topo::EdgeCurve::Kind::Circle) return {};

  const auto g = detail::coneCapGeom(solid, edgeIds[0]);
  if (!g) return {};  // not a pure capped frustum (cylinder / stepped / tilted → OCCT)
  const auto sec = detail::g2CurvedConeSection(*g, r);
  if (!sec) return {};
  // Far-end cone radius (the wall band's far ring): Rref + farH·tanσ.
  const double Rfar = g->Rref + g->farH * std::tan(g->semiAngle);
  std::vector<nb::Polygon> polys =
      detail::buildG2StraightBand(g->axis, *sec, g->capH, g->farH, Rfar, g->capNormal, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CURVED_FILLET_G2_H
