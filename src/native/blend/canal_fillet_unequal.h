// SPDX-License-Identifier: Apache-2.0
//
// canal_fillet_unequal.h — MOAT M3 UNEQUAL-radius CYL↔CYL CANAL fillet: a
// constant-radius rolling-ball fillet along the two crossing creases of two
// ORTHOGONAL-axis cylinders of DISTINCT radii (the thinner cylinder poking fully
// through the thicker one). This lands the FIRST entry of the tail the equal-radius
// Steinmetz slice (canal_fillet.h) explicitly declined:
//   "Unequal radii / non-orthogonal / TORUS / elliptical creases still decline → OCCT."
//
// ── WHY UNEQUAL IS A DISTINCT (AND MORE REGULAR) TOPOLOGY ────────────────────────────
// Two EQUAL-radius orthogonal cylinders intersect along two ellipse-like arcs that MEET
// at two poles (a degenerate pinch) — the Steinmetz case canal_fillet.h handles. When the
// radii DIFFER, the geometry splits cleanly instead of pinching:
//
//   * Frame: thin cyl A axis ẑ (wall cx²+cy²=Ra²), thick cyl B axis x̂ (wall cy²+cz²=Rb²),
//     Ra < Rb. The intersection curve of the two infinite walls, parametrized by the thin
//     cylinder's azimuth u (cx=Ra cos u, cy=Ra sin u), has cz=±√(Rb²−Ra² sin²u). Because
//     Rb > Ra the radicand ≥ Rb²−Ra² > 0 for EVERY u — cz NEVER reaches 0. So the crease is
//     TWO DISJOINT CLOSED LOOPS (top cz>0, bottom cz<0), NOT a pinched figure that shares
//     poles. The thin cylinder passes fully through the thick one; the common's boundary is
//     the thin wall's full tube (waist between the two loops) capped top & bottom by the
//     thick wall.
//   * The rolling ball of radius r seated INSIDE both walls has its centre at distance
//     R0a=Ra−r from the thin axis AND R0b=Rb−r from the thick axis: cx=R0a cos u, cy=R0a
//     sin u, cz=±√(R0b²−R0a² sin²u) — again never a pole (R0b>R0a). Each crease loop gets a
//     NON-DEGENERATE canal strip (cross-section width bounded away from zero everywhere),
//     G1-tangent to the thin wall at C+r·(cx,cy,0)/R0a (|xy|=Ra) and to the thick wall at
//     C+r·(0,cy,cz)/R0b (|yz|=Rb). ONE canonical slerp between the two wall radials is
//     consumed bit-identically by the strip and both walls, so every seam welds exactly.
//
// ── REBUILD (planar-facet weld, tessellator-pristine) ───────────────────────────────
// The whole filleted COMMON is rebuilt as ONE deflection-bounded planar-facet soup sharing
// vertices along every seam, welded by assembleSolid. Faces:
//   * two canal STRIPS (top/bottom crease loops), each u∈[0,2π) closed × t∈[0,1];
//   * the thin wall's TUBE band (waist), each azimuth column spanning z between the two
//     strips' thin-wall seams (a cylinder generator is straight in z → one z-column each);
//   * the thick wall's two CAP patches (top/bottom), each the region enclosed by a strip's
//     thick-wall seam loop, ring-fanned from the loop centroid on the thick wall.
// There are NO poles and NO caps (the two thick-wall patches + thin tube fully bound the
// lens on a body long enough that its disc caps miss the fillet band). The engine self-
// verify (watertight + consistently oriented + 0 < Vr < Vo) then accepts it, else → OCCT.
//
// ── SCOPE (honest) ──────────────────────────────────────────────────────────────────
// Native only for a canonical unequal-radius orthogonal-crossing bicylinder COMMON:
// exactly two cylinder wall families whose axes are ORTHOGONAL and CROSS, with DISTINCT
// radii Ra≠Rb, and the thin radius Ra ≥ 2r (ring-torus guard R0a=Ra−r ≥ r). EQUAL radii
// route to canal_fillet.h; non-orthogonal / non-crossing axes, a third wall family, r ≤ 0,
// or a multi-edge pick → NULL → OCCT. The idealized perpendicular cross-section differs
// from OCCT's variable-dihedral canal by a small modeling-convention gap (REPORTED, never
// gated). The engine gates the result with the two-sided volume + orientation self-verify;
// NEVER a wrong or leaky solid.
//
// CLEAN-ROOM. Reuses src/native/math + boolean(extractPolygons/assembleSolid) + blend_geom
// + tessellate(self-verify) + canal_fillet.h's generic soup helpers (canalTri /
// orientSoupCoherent). OCCT-FREE. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CANAL_FILLET_UNEQUAL_H
#define CYBERCAD_NATIVE_BLEND_CANAL_FILLET_UNEQUAL_H

#include "native/blend/blend_geom.h"
#include "native/blend/canal_fillet.h"  // detail::canalTri, detail::orientSoupCoherent (generic)
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

inline constexpr double kUCanalTwoPi = 6.28318530717958647692;

// The recognized unequal orthogonal pair, resolved into a canonical world frame — origin
// at the axis crossing point, ez = THIN cyl axis (radius Ra), ex = THICK cyl axis
// (radius Rb), ey = ez×ex. All world points are origin + cx·ex + cy·ey + cz·ez.
struct UnequalFrame {
  math::Point3 origin;
  math::Vec3 ex, ey, ez;  // orthonormal; thin axis = ez, thick axis = ex
  double Ra = 0.0;        // thin cylinder radius  (axis ez)
  double Rb = 0.0;        // thick cylinder radius (axis ex), Rb > Ra
};

inline math::Point3 uFromCanon(const UnequalFrame& f, double cx, double cy, double cz) {
  return math::Point3{f.origin.x + cx * f.ex.x + cy * f.ey.x + cz * f.ez.x,
                      f.origin.y + cx * f.ex.y + cy * f.ey.y + cz * f.ez.y,
                      f.origin.z + cx * f.ex.z + cy * f.ey.z + cz * f.ez.z};
}
inline math::Vec3 uDirCanon(const UnequalFrame& f, double cx, double cy, double cz) {
  return math::Vec3{cx * f.ex.x + cy * f.ey.x + cz * f.ez.x,
                    cx * f.ex.y + cy * f.ey.y + cz * f.ez.y,
                    cx * f.ex.z + cy * f.ey.z + cz * f.ez.z};
}

// Recognise the body as an unequal-radius orthogonal bicylinder COMMON from its planar-
// facet soup (the native SSI boolean bakes both cylinder walls into planar triangles). The
// axis-recovery mirrors the Steinmetz recognizer (axes = cross-products of facet-normal
// pairs, area-scored perpendicular families); the RADII are recovered PER FAMILY and must
// be DISTINCT (equal radii route to canal_fillet.h). Returns the frame or nullopt.
inline std::optional<UnequalFrame> unequalCylFrame(const topo::Shape& solid, int edgeId) {
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
        if (exclude && std::fabs(math::dot(d, *exclude)) > 5e-2) continue;
        const double s = score(d);
        if (s > bestS) { bestS = s; best = d; }
      }
    return {best, bestS};
  };
  auto [ax0, s0] = bestAxis(nullptr);
  if (!(s0 > 0.0)) return std::nullopt;
  auto [ax1, s1] = bestAxis(&ax0);
  if (!(s1 > 0.0)) return std::nullopt;
  if (std::fabs(math::dot(ax0, ax1)) > 3e-2) return std::nullopt;  // orthogonal axes
  {  // Gram-Schmidt ax1 against ax0 so the canonical frame is EXACTLY orthonormal.
    math::Vec3 b = ax1 - ax0 * math::dot(ax0, ax1);
    const double nb2 = math::norm(b);
    if (nb2 < 1e-6) return std::nullopt;
    ax1 = b * (1.0 / nb2);
  }

  auto perpDist = [&](const math::Point3& p, const math::Vec3& axis, const math::Point3& on) {
    const math::Vec3 d = p - on;
    return math::norm(d - axis * math::dot(d, axis));
  };
  math::Vec3 csum{0, 0, 0};
  for (const FN& f : fn) csum += f.c.asVec();
  const math::Point3 c0{csum.x / N, csum.y / N, csum.z / N};

  // Per-family radius. Bootstrap each family's radius from the facets whose normal is
  // ~perpendicular to that axis, then CLASSIFY EVERY facet by RADIUS match (a facet on the
  // ax0 cylinder has perpDist(ax0)=R0 EXACTLY, regardless of its planar-facet normal tilt —
  // so radius, not the normal threshold, is the robust discriminator). A facet matching
  // neither radius is a genuine stray → not a pure bicylinder. This tolerates the small
  // axial tilt a coarse cap triangle's normal carries (which a hard normal reject cannot).
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
  // DISTINCT radii — equal radii are the Steinmetz case (canal_fillet.h). A gap under this
  // band would make the two crease loops nearly touch (near-pole) and route ambiguously.
  if (std::fabs(R0fam - R1fam) <= std::max(2e-2, 2e-2 * std::max(R0fam, R1fam)))
    return std::nullopt;

  UnequalFrame f;
  f.origin = c0;
  // Assign THIN family → ez, THICK family → ex.
  const bool zeroThin = R0fam < R1fam;
  const math::Vec3 thinAx = zeroThin ? ax0 : ax1;
  const math::Vec3 thickAx = zeroThin ? ax1 : ax0;
  f.Ra = zeroThin ? R0fam : R1fam;
  f.Rb = zeroThin ? R1fam : R0fam;
  f.ez = math::Dir3{thinAx}.vec();
  f.ex = math::Dir3{thickAx}.vec();
  f.ey = math::cross(f.ez, f.ex);
  const double ny = math::norm(f.ey);
  if (ny < 1e-9) return std::nullopt;
  f.ey = f.ey * (1.0 / ny);
  return f;
}

// Rolling-ball centre on the crease loop sgn (top +1 / bottom −1) at thin-azimuth u.
inline math::Point3 uCanalCentre(const UnequalFrame& f, int sgn, double u, double r) {
  const double R0a = f.Ra - r, R0b = f.Rb - r;
  const double cx = R0a * std::cos(u), cy = R0a * std::sin(u);
  const double cz2 = R0b * R0b - R0a * R0a * std::sin(u) * std::sin(u);
  const double cz = sgn * std::sqrt(std::max(0.0, cz2));
  return uFromCanon(f, cx, cy, cz);
}

// A canonical strip point: crease loop sgn, thin-azimuth u, minor param t∈[0,1] (t=0 seam
// on thin wall, t=1 seam on thick wall). Slerp of the two unit wall radials about the ball
// centre. Neither radial degenerates (R0a, R0b > 0), so no pole special-case.
inline math::Point3 uCanalStripPoint(const UnequalFrame& f, int sgn, double u, double t,
                                     double r) {
  const double R0a = f.Ra - r, R0b = f.Rb - r;
  const double cx = R0a * std::cos(u), cy = R0a * std::sin(u);
  const double cz = sgn * std::sqrt(std::max(0.0, R0b * R0b - R0a * R0a * std::sin(u) * std::sin(u)));
  const math::Vec3 rA{cx / R0a, cy / R0a, 0.0};                 // unit thin-wall radial
  const double nB = std::hypot(cy, cz);
  const math::Vec3 rB = nB > 1e-12 ? math::Vec3{0.0, cy / nB, cz / nB} : math::Vec3{0.0, 0.0, 1.0};
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
  return uFromCanon(f, cx + r * dir.x, cy + r * dir.y, cz + r * dir.z);
}

// Azimuthal (u) sample count from the spine loop curvature (radius ~R0a over 2π).
inline int uCanalUSteps(double R0a, double defl) {
  if (R0a <= kBlendEps) return 48;
  const double dmax = 2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - defl / R0a)));
  const int n = dmax > 1e-9 ? static_cast<int>(std::ceil(kUCanalTwoPi / dmax)) : 96;
  return std::clamp(n, 24, 400);
}

// Build the whole filleted unequal-bicylinder COMMON as a planar-facet soup (empty on
// degeneracy). All faces share canonical seam samples; the interior origin orients coherently.
inline std::vector<nb::Polygon> buildUnequalCanalSoup(const UnequalFrame& f, double r,
                                                      double deflection) {
  const double R0a = f.Ra - r, R0b = f.Rb - r;
  if (!(R0a >= r - 1e-9)) return {};       // ring-torus guard Ra ≥ 2r
  if (!(R0b > R0a + 1e-9)) return {};      // strict thin/thick separation (disjoint loops)

  const int M = uCanalUSteps(R0a, deflection);                       // azimuth samples (closed)
  const int Nt = std::clamp(static_cast<int>(std::ceil(6.0 * r / std::max(R0a, 1e-9))), 3, 24);

  std::vector<nb::Polygon> soup;
  soup.reserve(static_cast<std::size_t>(M) * (2 * Nt + 8) + 32);

  // ── two canal strips (top/bottom crease loops), periodic in u ──
  for (int sgn : {+1, -1}) {
    for (int i = 0; i < M; ++i) {
      const double u0 = kUCanalTwoPi * i / M;
      const double u1 = kUCanalTwoPi * (i + 1) / M;  // wraps to u0 at i=M-1 (2π ≡ 0)
      for (int j = 0; j < Nt; ++j) {
        const double t0 = static_cast<double>(j) / Nt, t1 = static_cast<double>(j + 1) / Nt;
        const math::Point3 p00 = uCanalStripPoint(f, sgn, u0, t0, r);
        const math::Point3 p01 = uCanalStripPoint(f, sgn, u0, t1, r);
        const math::Point3 p10 = uCanalStripPoint(f, sgn, u1, t0, r);
        const math::Point3 p11 = uCanalStripPoint(f, sgn, u1, t1, r);
        const math::Point3 c0 = uCanalCentre(f, sgn, u0, r);
        const math::Point3 c1 = uCanalCentre(f, sgn, u1, r);
        const math::Vec3 out0{p00.x - c0.x, p00.y - c0.y, p00.z - c0.z};
        const math::Vec3 out1{p11.x - c1.x, p11.y - c1.y, p11.z - c1.z};
        const math::Vec3 out{out0.x + out1.x, out0.y + out1.y, out0.z + out1.z};
        canalTri(soup, p00, p10, p11, out);
        canalTri(soup, p00, p11, p01, out);
      }
    }
  }

  // ── thin wall TUBE band (waist): each azimuth column spans z between the two strips'
  //    thin-wall seams (t=0). A cylinder generator is straight in z → one quad per column. ──
  for (int i = 0; i < M; ++i) {
    const double u0 = kUCanalTwoPi * i / M;
    const double u1 = kUCanalTwoPi * (i + 1) / M;
    const math::Point3 top0 = uCanalStripPoint(f, +1, u0, 0.0, r);
    const math::Point3 top1 = uCanalStripPoint(f, +1, u1, 0.0, r);
    const math::Point3 bot0 = uCanalStripPoint(f, -1, u0, 0.0, r);
    const math::Point3 bot1 = uCanalStripPoint(f, -1, u1, 0.0, r);
    const math::Vec3 out = uDirCanon(f, std::cos(0.5 * (u0 + u1)), std::sin(0.5 * (u0 + u1)), 0.0);
    canalTri(soup, bot0, bot1, top1, out);
    canalTri(soup, bot0, top1, top0, out);
  }

  // ── thick wall CAP patches (top/bottom): the region enclosed by a strip's thick-wall
  //    seam loop (t=1), ring-fanned from the loop centroid on the thick wall. The seam
  //    ring (k=Nr) reproduces the strip's t=1 samples EXACTLY (same closed-form), so the
  //    cap welds watertight to the strip. Interior rings interpolate (x,w) params and
  //    project to the thick wall (cy²+cz²=Rb²), bounding chord error. ──
  auto thickWallPoint = [&](double x, double w) {
    return uFromCanon(f, x, f.Rb * std::cos(w), f.Rb * std::sin(w));
  };
  for (int sgn : {+1, -1}) {
    // Collect seam params (x=cx, w=atan2(cz,cy)) per azimuth; the strip t=1 point equals
    // thickWallPoint(x,w) exactly.
    std::vector<double> px(static_cast<std::size_t>(M)), pw(static_cast<std::size_t>(M));
    double gx = 0.0, gw = 0.0;
    for (int i = 0; i < M; ++i) {
      const double u = kUCanalTwoPi * i / M;
      const double cx = R0a * std::cos(u), cy = R0a * std::sin(u);
      const double cz = sgn * std::sqrt(std::max(0.0, R0b * R0b - R0a * R0a * std::sin(u) * std::sin(u)));
      px[static_cast<std::size_t>(i)] = cx;
      pw[static_cast<std::size_t>(i)] = std::atan2(cz, cy);  // sgn keeps w in (0,π) top / (−π,0) bottom
      gx += cx;
      gw += pw[static_cast<std::size_t>(i)];
    }
    gx /= M;
    gw /= M;
    // Ring count from the cap's angular spread on the thick wall (radius Rb).
    double wspread = 0.0;
    for (int i = 0; i < M; ++i) wspread = std::max(wspread, std::fabs(pw[static_cast<std::size_t>(i)] - gw));
    const double dmax = 2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - deflection / std::max(f.Rb, 1e-9))));
    const int Nr = std::clamp(dmax > 1e-9 ? static_cast<int>(std::ceil(wspread / dmax)) : 8, 2, 64);
    const math::Point3 G = thickWallPoint(gx, gw);
    // Ring points, k=0(centre G)..Nr(seam). k=Nr reproduces the strip's t=1 samples exactly.
    auto ringP = [&](int k, int i) -> math::Point3 {
      if (k == 0) return G;
      const double s = static_cast<double>(k) / Nr;
      return thickWallPoint(gx + (px[static_cast<std::size_t>(i)] - gx) * s,
                            gw + (pw[static_cast<std::size_t>(i)] - gw) * s);
    };
    // Outward = thick-wall radial at the triangle centroid (canon (·,cy,cz) plane).
    auto outAt = [&](const math::Point3& p0, const math::Point3& p1, const math::Point3& p2) {
      const math::Point3 tc{(p0.x + p1.x + p2.x) / 3.0, (p0.y + p1.y + p2.y) / 3.0,
                            (p0.z + p1.z + p2.z) / 3.0};
      const math::Vec3 d = tc - f.origin;
      return uDirCanon(f, 0.0, math::dot(d, f.ey), math::dot(d, f.ez));
    };
    for (int k = 0; k < Nr; ++k) {
      for (int i = 0; i < M; ++i) {
        const int in = (i + 1) % M;
        const math::Point3 a = ringP(k, i);
        const math::Point3 b = ringP(k, in);
        const math::Point3 cN = ringP(k + 1, in);
        const math::Point3 d = ringP(k + 1, i);
        if (k == 0) {  // fan from the centre point G to the innermost ring
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
// unequal_canal_fillet_edge — round the two crossing creases of an unequal-radius
// orthogonal bicylinder COMMON (thin cyl through thick cyl) at constant `radius`.
// Returns the filleted solid, or a NULL Shape (→ OCCT) when the body is not a
// recognizable unequal orthogonal bicylinder, radii are equal (→ canal_fillet.h),
// Ra < 2·radius, on a multi-edge pick, or on any degeneracy. `deflection` bounds
// the facet chord error.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape unequal_canal_fillet_edge(const topo::Shape& solid, const int* edgeIds,
                                             int edgeCount, double r, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r > kBlendEps)) return {};
  const auto frameOpt = detail::unequalCylFrame(solid, edgeIds[0]);
  if (!frameOpt) return {};
  const std::vector<nb::Polygon> soup = detail::buildUnequalCanalSoup(*frameOpt, r, deflection);
  if (soup.size() < 4) return {};
  const topo::Shape result = nb::assembleSolid(soup);
  if (result.isNull()) return {};

  // ── MANDATORY internal self-verify (never a wrong/folded solid) ──────────────────────
  // As with the Steinmetz canal, a large radius could fold a strip/cap into a self-
  // intersecting-but-watertight shell enclosing a grossly wrong volume. Guard with
  // (a) consistent orientation, (b) a removed-volume upper bound (the fillet removes at
  // most a tube of radius r over the two crease loops ~2π·(Ra+Rb)), and (c) it keeps the
  // large majority of the body. A fold violates (b)+(c) → DECLINE → OCCT. The engine's
  // SHRINK gate then re-confirms 0 < Vr < Vo.
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
  const double maxRemoved = 30.0 * r * r * (frameOpt->Ra + frameOpt->Rb) + 5e-2 * Vsharp;
  if (!(removed > -5e-2 * Vsharp) || !(removed < maxRemoved)) return {};
  if (!(Vr > 0.5 * Vsharp)) return {};
  return result;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CANAL_FILLET_UNEQUAL_H
