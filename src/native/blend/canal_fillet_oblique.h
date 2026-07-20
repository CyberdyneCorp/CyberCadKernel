// SPDX-License-Identifier: Apache-2.0
//
// canal_fillet_oblique.h — MOAT M3 UNEQUAL-radius NON-ORTHOGONAL CYL↔CYL CANAL fillet:
// a constant-radius rolling-ball fillet along the two crossing creases of two cylinders
// of DISTINCT radii whose axes CROSS at an OBLIQUE angle α ≠ 90° (the thinner cylinder
// poking through the thicker one at a slant — an angled branch pipe of a different
// diameter). This lands the NON-ORTHOGONAL entry of the tail canal_fillet.h and
// canal_fillet_unequal.h explicitly declined:
//   "Unequal radii / non-orthogonal / TORUS / elliptical creases still decline → OCCT."
//   "non-orthogonal / non-crossing axes ... still decline → OCCT."
//
// ── WHY OBLIQUE-UNEQUAL IS STILL THE REGULAR (DISJOINT-LOOP) TOPOLOGY ─────────────────
// Frame: thin cyl A axis ẑ (wall cx²+cy²=Ra²), thick cyl B axis b̂ = sinα·x̂ + cosα·ẑ
// (b̂ lies in the ẑ–x̂ plane by choosing x̂ = the ⊥-projection of the thick axis onto A's
// normal plane), with cosα = b̂·ẑ, sinα = |b̂ − cosα·ẑ|, Ra < Rb. The intersection of the
// two infinite walls, parametrized by A's azimuth u (cx=Ra cos u, cy=Ra sin u), solves the
// quadric  cz²sin²α − 2β cz cosα − β² = Rb²−Ra²  with β = Ra cos u sinα, i.e.
//
//     cz±(u) = [ Ra cos u cosα ± √(Rb² − Ra² sin²u) ] / sinα .
//
// The two branches meet only where √(Rb²−Ra²sin²u)=0 — impossible for Rb>Ra — so the crease
// is TWO DISJOINT CLOSED LOOPS (top/bottom), NOT a pinched pole-sharing figure. At α=90°
// (cosα=0, sinα=1) this is EXACTLY the orthogonal unequal formula (canal_fillet_unequal.h);
// that case routes there, this arm fires only for α clearly ≠ 90° (and clearly ≠ 0/180°).
//
// The rolling ball of radius r seated INSIDE both walls has its centre at distance R0a=Ra−r
// from the thin axis AND R0b=Rb−r from the thick axis — the SAME quadric with Ra,Rb→R0a,R0b:
//     Cz±(u) = [ R0a cos u cosα ± √(R0b² − R0a² sin²u) ] / sinα ,  Cx=R0a cos u, Cy=R0a sin u.
// Each crease loop gets a NON-DEGENERATE closed canal strip, G1-tangent to the thin wall at
// C + r·(cos u, sin u, 0) (|xy|=Ra) and to the thick wall at C + r·n_B (dist Rb from b̂),
// where n_B = (C − (C·b̂)b̂)/R0b is the unit thick-wall radial. ONE canonical slerp between
// the two wall radials is consumed bit-identically by the strip and both walls, so every
// seam welds exactly — the tangency construction is IDENTICAL to the orthogonal siblings;
// ONLY the spine curve and the (non-orthogonal) thick-axis frame change.
//
// ── REBUILD (planar-facet weld, tessellator-pristine) ───────────────────────────────
// The whole filleted COMMON is rebuilt as ONE deflection-bounded planar-facet soup sharing
// vertices along every seam, welded by assembleSolid. Faces (identical family set to the
// orthogonal unequal slice — a thin waist tube + two thick caps + two strips):
//   * two canal STRIPS (top/bottom crease loops), each u∈[0,2π) closed × t∈[0,1];
//   * the thin wall's TUBE band (waist), each azimuth column spanning cz between the two
//     strips' thin-wall seams (a cylinder generator is straight in cz → one quad each);
//   * the thick wall's two CAP patches, each the region enclosed by a strip's thick-wall
//     seam loop, ring-fanned from the loop centroid on the thick wall (parametrized in the
//     thick cylinder's OWN frame (b̂, ey, eB2) so any axis tilt is handled uniformly).
// The engine self-verify (watertight + consistently oriented + 0 < Vr < Vo) then accepts it,
// else → OCCT.
//
// ── SCOPE (honest) ──────────────────────────────────────────────────────────────────
// Native only for a canonical unequal-radius OBLIQUE-crossing bicylinder COMMON: exactly two
// cylinder wall families whose axes CROSS at a clearly non-orthogonal, non-parallel angle,
// with DISTINCT radii Ra≠Rb and thin Ra ≥ 2r (ring-torus guard R0a=Ra−r ≥ r). ORTHOGONAL
// axes route to canal_fillet_unequal.h; EQUAL radii (which pinch at poles at ANY angle) and
// near-parallel / non-crossing axes / a third wall family / r ≤ 0 / a multi-edge pick →
// NULL → OCCT. The idealized perpendicular cross-section differs from OCCT's variable-
// dihedral canal by a small modeling-convention gap (REPORTED, never gated). The engine
// gates the result with the two-sided volume + orientation self-verify; NEVER a wrong or
// leaky solid.
//
// CLEAN-ROOM. Reuses src/native/math + boolean(extractPolygons/assembleSolid) + blend_geom
// + tessellate(self-verify) + canal_fillet.h's generic soup helpers (canalTri /
// orientSoupCoherent). OCCT-FREE. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CANAL_FILLET_OBLIQUE_H
#define CYBERCAD_NATIVE_BLEND_CANAL_FILLET_OBLIQUE_H

#include "native/blend/blend_geom.h"
#include "native/blend/canal_fillet.h"  // detail::canalTri, detail::orientSoupCoherent (generic)
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

inline constexpr double kOblCanalTwoPi = 6.28318530717958647692;
// Non-orthogonality band: |cosα| must clear this margin from 0 (else it is the orthogonal
// case — route to canal_fillet_unequal.h) and stay below the near-parallel cap (else the
// axes barely cross and sinα → 0 would blow up the spine).
inline constexpr double kOblMinCos = 5e-2;   // ≳ ~3° off orthogonal
inline constexpr double kOblMaxCos = 0.97;   // ≲ ~14° off parallel (sinα ≳ 0.24)

// The recognized unequal oblique pair, in a canonical frame — origin at the axis crossing,
// ez = THIN cyl axis (radius Ra), ex = the unit ⊥-projection of the THICK axis onto ez's
// normal plane, ey = ez×ex. The thick axis is b̂ = sinA·ex + cosA·ez. All world points are
// origin + cx·ex + cy·ey + cz·ez.
struct ObliqueFrame {
  math::Point3 origin;
  math::Vec3 ex, ey, ez;  // orthonormal; thin axis = ez
  double Ra = 0.0;        // thin cylinder radius  (axis ez)
  double Rb = 0.0;        // thick cylinder radius (axis b̂), Rb > Ra
  double cosA = 0.0;      // b̂·ez  (may be ±; |cosA| ∈ (kOblMinCos, kOblMaxCos))
  double sinA = 1.0;      // |b̂ − cosA·ez|  (> 0)
};

inline math::Point3 oblFromCanon(const ObliqueFrame& f, double cx, double cy, double cz) {
  return math::Point3{f.origin.x + cx * f.ex.x + cy * f.ey.x + cz * f.ez.x,
                      f.origin.y + cx * f.ex.y + cy * f.ey.y + cz * f.ez.y,
                      f.origin.z + cx * f.ex.z + cy * f.ey.z + cz * f.ez.z};
}
inline math::Vec3 oblDirCanon(const ObliqueFrame& f, double cx, double cy, double cz) {
  return math::Vec3{cx * f.ex.x + cy * f.ey.x + cz * f.ez.x,
                    cx * f.ex.y + cy * f.ey.y + cz * f.ez.y,
                    cx * f.ex.z + cy * f.ey.z + cz * f.ez.z};
}

// Recognise the body as an unequal-radius OBLIQUE-crossing bicylinder COMMON from its planar-
// facet soup. Axis recovery + per-family radius classification mirror the orthogonal unequal
// recognizer; the difference is the axes must be NON-orthogonal (|cosα| ∈ (kOblMinCos,
// kOblMaxCos)) — orthogonal pairs route to canal_fillet_unequal.h. Returns the frame or
// nullopt.
inline std::optional<ObliqueFrame> obliqueCylFrame(const topo::Shape& solid, int edgeId) {
  (void)edgeId;  // the crease is recovered wholesale; any picked crease edge is accepted
  const std::vector<nb::Polygon> polys = nb::extractPolygons(solid);
  if (polys.size() < 8) return std::nullopt;

  struct FN { math::Vec3 n; double area; math::Point3 c; };
  std::vector<FN> fn;
  fn.reserve(polys.size());
  for (const nb::Polygon& p : polys) {
    if (p.vertices.size() < 3) continue;
    math::Vec3 nrm = p.plane.normal;
    const double L = math::norm(nrm);
    if (L < 1e-9) continue;
    nrm = nrm * (1.0 / L);
    math::Vec3 av{0, 0, 0}, ctr{0, 0, 0};
    for (std::size_t i = 0; i < p.vertices.size(); ++i) {
      av += math::cross(p.vertices[i].asVec(), p.vertices[(i + 1) % p.vertices.size()].asVec());
      ctr += p.vertices[i].asVec();
    }
    ctr = ctr * (1.0 / static_cast<double>(p.vertices.size()));
    fn.push_back(FN{nrm, 0.5 * math::norm(av), math::Point3{ctr.x, ctr.y, ctr.z}});
  }
  const int N = static_cast<int>(fn.size());
  if (N < 8) return std::nullopt;

  auto score = [&](const math::Vec3& d) {
    double s = 0.0;
    for (const FN& f : fn)
      if (std::fabs(math::dot(f.n, d)) < 2e-2) s += f.area;
    return s;
  };
  const int stride = std::max(1, N / 64);
  auto bestAxis = [&](const math::Vec3* exclude) -> std::pair<math::Vec3, double> {
    math::Vec3 best{0, 0, 0};
    double bestS = -1.0;
    for (int i = 0; i < N; i += stride)
      for (int j = i + stride; j < N; j += stride) {
        math::Vec3 d = math::cross(fn[static_cast<std::size_t>(i)].n, fn[static_cast<std::size_t>(j)].n);
        const double L = math::norm(d);
        if (L < 0.3) continue;
        d = d * (1.0 / L);
        if (exclude && std::fabs(math::dot(d, *exclude)) > kOblMaxCos) continue;
        const double s = score(d);
        if (s > bestS) { bestS = s; best = d; }
      }
    return {best, bestS};
  };
  auto [ax0, s0] = bestAxis(nullptr);
  if (!(s0 > 0.0)) return std::nullopt;
  auto [ax1, s1] = bestAxis(&ax0);
  if (!(s1 > 0.0)) return std::nullopt;
  const double axDot = std::fabs(math::dot(ax0, ax1));
  if (axDot <= kOblMinCos) return std::nullopt;  // orthogonal → canal_fillet_unequal.h
  if (axDot >= kOblMaxCos) return std::nullopt;  // near-parallel → not a crossing bicylinder

  auto perpDist = [&](const math::Point3& p, const math::Vec3& axis, const math::Point3& on) {
    const math::Vec3 d = p - on;
    return math::norm(d - axis * math::dot(d, axis));
  };
  math::Vec3 csum{0, 0, 0};
  for (const FN& f : fn) csum += f.c.asVec();
  const math::Point3 c0{csum.x / N, csum.y / N, csum.z / N};

  // Per-family radius: bootstrap from facets whose normal is ~⊥ each axis, then CLASSIFY
  // EVERY facet by RADIUS match (a facet on the ax0 cylinder sits at perp-distance R0
  // EXACTLY regardless of its facet-normal tilt — radius, not the normal, is the robust
  // discriminator). A facet matching neither radius → not a pure bicylinder.
  double boot0 = 0.0, boot1 = 0.0;
  int bc0 = 0, bc1 = 0;
  for (const FN& f : fn) {
    if (std::fabs(math::dot(f.n, ax0)) < 2e-2) { boot0 += perpDist(f.c, ax0, c0); ++bc0; }
    else if (std::fabs(math::dot(f.n, ax1)) < 2e-2) { boot1 += perpDist(f.c, ax1, c0); ++bc1; }
  }
  if (bc0 < 3 || bc1 < 3) return std::nullopt;
  const double R0boot = boot0 / bc0, R1boot = boot1 / bc1;
  if (!(R0boot > kBlendEps) || !(R1boot > kBlendEps)) return std::nullopt;
  double sum0 = 0.0, sum1 = 0.0;
  int cnt0 = 0, cnt1 = 0;
  for (const FN& f : fn) {
    const double e0 = std::fabs(perpDist(f.c, ax0, c0) - R0boot);
    const double e1 = std::fabs(perpDist(f.c, ax1, c0) - R1boot);
    const double tol0 = std::max(5e-3, 5e-3 * R0boot), tol1 = std::max(5e-3, 5e-3 * R1boot);
    if (e0 <= tol0 && e0 <= e1) { sum0 += perpDist(f.c, ax0, c0); ++cnt0; }
    else if (e1 <= tol1) { sum1 += perpDist(f.c, ax1, c0); ++cnt1; }
    else return std::nullopt;  // a facet on neither cylinder → not a pure bicylinder
  }
  if (cnt0 < 3 || cnt1 < 3) return std::nullopt;
  const double R0fam = sum0 / cnt0, R1fam = sum1 / cnt1;
  if (!(R0fam > kBlendEps) || !(R1fam > kBlendEps)) return std::nullopt;
  // DISTINCT radii — equal radii pinch at poles (a separate, harder slice) at ANY angle.
  if (std::fabs(R0fam - R1fam) <= std::max(2e-2, 2e-2 * std::max(R0fam, R1fam)))
    return std::nullopt;

  ObliqueFrame f;
  f.origin = c0;
  const bool zeroThin = R0fam < R1fam;
  const math::Vec3 thinAx = zeroThin ? ax0 : ax1;
  const math::Vec3 thickAx = zeroThin ? ax1 : ax0;
  f.Ra = zeroThin ? R0fam : R1fam;
  f.Rb = zeroThin ? R1fam : R0fam;
  f.ez = math::Dir3{thinAx}.vec();
  f.cosA = math::dot(thickAx, f.ez);
  math::Vec3 perp = thickAx - f.ez * f.cosA;  // thick-axis ⊥-projection onto ez's normal plane
  const double np = math::norm(perp);
  if (np < 1e-9) return std::nullopt;
  f.ex = perp * (1.0 / np);
  f.sinA = std::sqrt(std::max(0.0, 1.0 - f.cosA * f.cosA));
  if (!(f.sinA > 1e-6)) return std::nullopt;
  f.ey = math::cross(f.ez, f.ex);
  const double ny = math::norm(f.ey);
  if (ny < 1e-9) return std::nullopt;
  f.ey = f.ey * (1.0 / ny);
  return f;
}

// The rolling-ball spine cz for crease loop sgn (top +1 / bottom −1) at thin-azimuth u.
inline double oblCanalCz(const ObliqueFrame& f, int sgn, double u, double r) {
  const double R0a = f.Ra - r, R0b = f.Rb - r;
  const double disc = R0b * R0b - R0a * R0a * std::sin(u) * std::sin(u);
  return (R0a * std::cos(u) * f.cosA + sgn * std::sqrt(std::max(0.0, disc))) / f.sinA;
}

inline math::Point3 oblCanalCentre(const ObliqueFrame& f, int sgn, double u, double r) {
  const double R0a = f.Ra - r;
  return oblFromCanon(f, R0a * std::cos(u), R0a * std::sin(u), oblCanalCz(f, sgn, u, r));
}

// A canonical strip point: crease loop sgn, thin-azimuth u, minor param t∈[0,1] (t=0 seam
// on thin wall, t=1 seam on thick wall). Slerp of the two unit wall radials about the ball
// centre. Neither radial degenerates (R0a, R0b > 0), so no pole special-case.
inline math::Point3 oblCanalStripPoint(const ObliqueFrame& f, int sgn, double u, double t,
                                       double r) {
  const double R0a = f.Ra - r, R0b = f.Rb - r;
  const double cx = R0a * std::cos(u), cy = R0a * std::sin(u), cz = oblCanalCz(f, sgn, u, r);
  const math::Vec3 rA{cx / R0a, cy / R0a, 0.0};  // unit thin-wall radial (canonical)
  const double cb = cx * f.sinA + cz * f.cosA;   // C·b̂ in canonical coords
  math::Vec3 rB{cx - cb * f.sinA, cy, cz - cb * f.cosA};  // C − (C·b̂)b̂  (|·| = R0b)
  const double nB = math::norm(rB);
  if (nB > 1e-12) rB = rB * (1.0 / nB);
  else rB = math::Vec3{0.0, 1.0, 0.0};
  const double dp = std::max(-1.0, std::min(1.0, rA.x * rB.x + rA.y * rB.y + rA.z * rB.z));
  const double om = std::acos(dp);
  math::Vec3 dir;
  if (om < 1e-9) {
    dir = rA;
  } else {
    const double s = std::sin(om);
    dir = rA * (std::sin((1.0 - t) * om) / s) + rB * (std::sin(t * om) / s);
  }
  const double dn = std::hypot(std::hypot(dir.x, dir.y), dir.z);
  dir = dir * (1.0 / dn);
  return oblFromCanon(f, cx + r * dir.x, cy + r * dir.y, cz + r * dir.z);
}

// Azimuthal (u) sample count from the loop's largest characteristic extent (the z-amplitude
// R0b/sinA can exceed the in-plane radius R0a on a slanted crossing).
inline int oblCanalUSteps(double charR, double defl) {
  if (charR <= kBlendEps) return 96;
  const double dmax = 2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - defl / charR)));
  const int n = dmax > 1e-9 ? static_cast<int>(std::ceil(kOblCanalTwoPi / dmax)) : 96;
  return std::clamp(n, 32, 512);
}

// Build the whole filleted oblique-unequal-bicylinder COMMON as a planar-facet soup (empty
// on degeneracy). All faces share canonical seam samples; the interior origin orients
// coherently.
inline std::vector<nb::Polygon> buildObliqueCanalSoup(const ObliqueFrame& f, double r,
                                                      double deflection) {
  const double R0a = f.Ra - r, R0b = f.Rb - r;
  if (!(R0a >= r - 1e-9)) return {};    // ring-torus guard Ra ≥ 2r
  if (!(R0b > R0a + 1e-9)) return {};   // strict thin/thick separation (disjoint loops)

  const int M = oblCanalUSteps(std::max(R0a, R0b / f.sinA), deflection);
  const int Nt = std::clamp(static_cast<int>(std::ceil(6.0 * r / std::max(R0a, 1e-9))), 3, 24);

  std::vector<nb::Polygon> soup;
  soup.reserve(static_cast<std::size_t>(M) * (2 * Nt + 8) + 32);

  // ── two canal strips (top/bottom crease loops), periodic in u ──
  for (int sgn : {+1, -1}) {
    for (int i = 0; i < M; ++i) {
      const double u0 = kOblCanalTwoPi * i / M;
      const double u1 = kOblCanalTwoPi * (i + 1) / M;  // wraps to u0 at i=M-1 (2π ≡ 0)
      for (int j = 0; j < Nt; ++j) {
        const double t0 = static_cast<double>(j) / Nt, t1 = static_cast<double>(j + 1) / Nt;
        const math::Point3 p00 = oblCanalStripPoint(f, sgn, u0, t0, r);
        const math::Point3 p01 = oblCanalStripPoint(f, sgn, u0, t1, r);
        const math::Point3 p10 = oblCanalStripPoint(f, sgn, u1, t0, r);
        const math::Point3 p11 = oblCanalStripPoint(f, sgn, u1, t1, r);
        const math::Point3 c0 = oblCanalCentre(f, sgn, u0, r);
        const math::Point3 c1 = oblCanalCentre(f, sgn, u1, r);
        const math::Vec3 out0{p00.x - c0.x, p00.y - c0.y, p00.z - c0.z};
        const math::Vec3 out1{p11.x - c1.x, p11.y - c1.y, p11.z - c1.z};
        const math::Vec3 out{out0.x + out1.x, out0.y + out1.y, out0.z + out1.z};
        canalTri(soup, p00, p10, p11, out);
        canalTri(soup, p00, p11, p01, out);
      }
    }
  }

  // ── thin wall TUBE band (waist): each azimuth column spans cz between the two strips'
  //    thin-wall seams (t=0). A cylinder generator is straight in cz → one quad per column. ──
  for (int i = 0; i < M; ++i) {
    const double u0 = kOblCanalTwoPi * i / M;
    const double u1 = kOblCanalTwoPi * (i + 1) / M;
    const math::Point3 top0 = oblCanalStripPoint(f, +1, u0, 0.0, r);
    const math::Point3 top1 = oblCanalStripPoint(f, +1, u1, 0.0, r);
    const math::Point3 bot0 = oblCanalStripPoint(f, -1, u0, 0.0, r);
    const math::Point3 bot1 = oblCanalStripPoint(f, -1, u1, 0.0, r);
    const math::Vec3 out = oblDirCanon(f, std::cos(0.5 * (u0 + u1)), std::sin(0.5 * (u0 + u1)), 0.0);
    canalTri(soup, bot0, bot1, top1, out);
    canalTri(soup, bot0, top1, top0, out);
  }

  // ── thick wall CAP patches (top/bottom), parametrized in the THICK cylinder's own frame
  //    (axis b̂, ⊥ dirs eB1=ey and eB2=b̂×ey). A point on the thick wall is
  //    origin + s·b̂ + Rb(cos w·eB1 + sin w·eB2). The strip t=1 seam point has axial s = C·b̂
  //    and perp = Rb·rB, so w=atan2 reproduces the strip's t=1 samples EXACTLY → the cap
  //    welds watertight to the strip. Interior rings interpolate (s,w) and re-project. ──
  const math::Vec3 bhat = oblDirCanon(f, f.sinA, 0.0, f.cosA);   // thick axis (world)
  const math::Vec3 eB1 = f.ey;                                   // ⊥ b̂ (b̂ has no ey comp)
  const math::Vec3 eB2 = oblDirCanon(f, -f.cosA, 0.0, f.sinA);   // = b̂×ey (world, unit)
  auto thickWallPoint = [&](double s, double w) {
    const math::Vec3 d = bhat * s + eB1 * (f.Rb * std::cos(w)) + eB2 * (f.Rb * std::sin(w));
    return math::Point3{f.origin.x + d.x, f.origin.y + d.y, f.origin.z + d.z};
  };
  for (int sgn : {+1, -1}) {
    std::vector<double> ps(static_cast<std::size_t>(M)), pw(static_cast<std::size_t>(M));
    double gs = 0.0, gw = 0.0;
    for (int i = 0; i < M; ++i) {
      const double u = kOblCanalTwoPi * i / M;
      const math::Point3 P = oblCanalStripPoint(f, sgn, u, 1.0, r);  // thick-wall seam point
      const math::Vec3 d{P.x - f.origin.x, P.y - f.origin.y, P.z - f.origin.z};
      const double s = math::dot(d, bhat);
      const math::Vec3 pr = d - bhat * s;  // perp component (|·| = Rb)
      ps[static_cast<std::size_t>(i)] = s;
      pw[static_cast<std::size_t>(i)] = std::atan2(math::dot(pr, eB2), math::dot(pr, eB1));
      gs += s;
      gw += pw[static_cast<std::size_t>(i)];
    }
    gs /= M;
    gw /= M;
    double wspread = 0.0;
    for (int i = 0; i < M; ++i)
      wspread = std::max(wspread, std::fabs(pw[static_cast<std::size_t>(i)] - gw));
    const double dmax =
        2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - deflection / std::max(f.Rb, 1e-9))));
    const int Nr = std::clamp(dmax > 1e-9 ? static_cast<int>(std::ceil(wspread / dmax)) : 8, 2, 64);
    const math::Point3 G = thickWallPoint(gs, gw);
    auto ringP = [&](int k, int i) -> math::Point3 {
      if (k == 0) return G;
      const double a = static_cast<double>(k) / Nr;
      return thickWallPoint(gs + (ps[static_cast<std::size_t>(i)] - gs) * a,
                            gw + (pw[static_cast<std::size_t>(i)] - gw) * a);
    };
    auto outAt = [&](const math::Point3& p0, const math::Point3& p1, const math::Point3& p2) {
      const math::Point3 tc{(p0.x + p1.x + p2.x) / 3.0, (p0.y + p1.y + p2.y) / 3.0,
                            (p0.z + p1.z + p2.z) / 3.0};
      const math::Vec3 d{tc.x - f.origin.x, tc.y - f.origin.y, tc.z - f.origin.z};
      const double s = math::dot(d, bhat);
      return math::Vec3{d.x - bhat.x * s, d.y - bhat.y * s, d.z - bhat.z * s};  // thick-wall radial
    };
    for (int k = 0; k < Nr; ++k) {
      for (int i = 0; i < M; ++i) {
        const int in = (i + 1) % M;
        const math::Point3 a = ringP(k, i);
        const math::Point3 b = ringP(k, in);
        const math::Point3 cN = ringP(k + 1, in);
        const math::Point3 d = ringP(k + 1, i);
        if (k == 0) {
          canalTri(soup, G, d, cN, outAt(G, d, cN));
        } else {
          canalTri(soup, a, b, cN, outAt(a, b, cN));
          canalTri(soup, a, cN, d, outAt(a, cN, d));
        }
      }
    }
  }

  detail::orientSoupCoherent(soup, f.origin);
  return soup;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// oblique_canal_fillet_edge — round the two crossing creases of an unequal-radius
// OBLIQUE (α ≠ 90°) bicylinder COMMON (thin cyl through thick cyl at a slant) at
// constant `radius`. Returns the filleted solid, or a NULL Shape (→ OCCT) when the
// body is not a recognizable unequal oblique bicylinder, axes are orthogonal (→
// canal_fillet_unequal.h) / near-parallel, radii are equal, Ra < 2·radius, on a
// multi-edge pick, or on any degeneracy. `deflection` bounds the facet chord error.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape oblique_canal_fillet_edge(const topo::Shape& solid, const int* edgeIds,
                                             int edgeCount, double r, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r > kBlendEps)) return {};
  const auto frameOpt = detail::obliqueCylFrame(solid, edgeIds[0]);
  if (!frameOpt) return {};
  const std::vector<nb::Polygon> soup = detail::buildObliqueCanalSoup(*frameOpt, r, deflection);
  if (soup.size() < 4) return {};
  const topo::Shape result = nb::assembleSolid(soup);
  if (result.isNull()) return {};

  // ── MANDATORY internal self-verify (never a wrong/folded solid) ──────────────────────
  // A large radius could fold a strip/cap into a self-intersecting-but-watertight shell
  // enclosing a grossly wrong volume. Guard with (a) consistent orientation, (b) a removed-
  // volume upper bound (the fillet removes at most a tube of radius r over the two crease
  // loops, whose length grows with 1/sinA on a slant), and (c) it keeps the large majority
  // of the body. A fold violates (b)+(c) → DECLINE → OCCT. The engine SHRINK gate re-confirms.
  namespace tess = cybercad::native::tessellate;
  tess::MeshParams mp;
  mp.deflection = std::min(deflection, 0.01);
  const tess::Mesh mR = tess::SolidMesher(mp).mesh(result);
  if (!tess::isConsistentlyOriented(mR)) return {};
  const double Vr = std::fabs(tess::enclosedVolume(mR));
  const tess::Mesh mS = tess::SolidMesher(mp).mesh(solid);
  if (!tess::isWatertight(mS)) return {};
  const double Vsharp = std::fabs(tess::enclosedVolume(mS));
  const double removed = Vsharp - Vr;
  const double maxRemoved =
      30.0 * r * r * (frameOpt->Ra + frameOpt->Rb) / frameOpt->sinA + 5e-2 * Vsharp;
  if (!(removed > -5e-2 * Vsharp) || !(removed < maxRemoved)) return {};
  if (!(Vr > 0.5 * Vsharp)) return {};
  return result;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CANAL_FILLET_OBLIQUE_H
