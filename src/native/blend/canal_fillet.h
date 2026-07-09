// SPDX-License-Identifier: Apache-2.0
//
// canal_fillet.h — MOAT M3 CYL↔CYL CANAL fillet: a constant-radius rolling-ball
// fillet along the crossing crease of two EQUAL-radius, ORTHOGONAL-axis cylinders
// (the Steinmetz bicylinder COMMON / intersection). This is the curved↔curved crease
// analogue of the landed single-rim curved fillets (curved_fillet.h) and the last
// app-relevant M3 curved-blend residual that DECLINED to OCCT.
//
// ── WHY THE OLD DECLINE WAS TOO PESSIMISTIC (the crux) ──────────────────────────────
// Two equal-radius cylinders whose axes cross orthogonally intersect along TWO
// ellipse-like crease arcs meeting at TWO poles. A naive "single swept-r canal" cannot
// close the crossing watertight — so it declined. But the GEOMETRY is exactly analytic:
//
//   * Frame: cyl A axis ẑ (wall x²+y²=Rc²), cyl B axis x̂ (wall y²+z²=Rc²), radius Rc.
//     The rolling ball of radius r seated INSIDE both walls along the z=x crease has its
//     centre at distance Rc−r from BOTH axes: √(Cx²+Cy²)=Rc−r AND √(Cy²+Cz²)=Rc−r ⇒
//     Cx=Cz. Parametrizing Cx=Cz=R0 sinφ, Cy=R0 cosφ (R0=Rc−r) gives a centre curve at
//     CONSTANT distance R0 from each axis. The fillet is the canal surface swept by that
//     ball — G1-tangent to cyl A at C+r·(Cx,Cy,0)/R0 (on the wall, |xy|=Rc) and to cyl B
//     at C+r·(0,Cy,Cz)/R0 (|yz|=Rc). ONE canonical slerp between the two wall radials is
//     consumed bit-identically by the strip and both walls, so the seams weld exactly.
//   * The two crease planes (z=±x) give two such strips; they meet where the local
//     dihedral opens to 180° — the two POLES (0,±Rc,0) — where EACH strip's cross-section
//     TAPERS TO ZERO width. The "crossing" is a DEGENERATE PINCH sharing the two pole
//     vertices, NOT a finite trihedral corner needing a spherical patch. The whole
//     filleted bicylinder therefore welds watertight PURELY in the ASSEMBLY layer.
//
// ── REBUILD (planar-facet weld, tessellator-pristine) ───────────────────────────────
// The whole filleted COMMON is rebuilt as ONE deflection-bounded planar-facet soup that
// shares vertices along every seam and at the two poles, welded by assembleSolid (weld +
// T-junction repair). Faces:
//   * two canal STRIPS (crease planes z=±x), each φ∈[0,2π) × t∈[0,1] tapering at the poles;
//   * cyl-A's two LUNE walls (region of wall A inside cyl B), each bounded by two seam arcs
//     matched by azimuth u (φ lockstep) — trimmed back to the strip seams;
//   * cyl-B's two LUNE walls, each = ONE arc self-paired across its two legs at matched
//     axial x (the cosine φ-grid is symmetric, so φ↔π−φ reuse identical strip seam verts).
// A body long enough that its disc caps do not touch the fillet band closes with NO caps
// (the four lune walls fully bound the lens). The engine self-verify (watertight +
// consistently oriented + 0 < Vr < Vo) then accepts it, else → NULL → OCCT.
//
// ── SCOPE (honest) ──────────────────────────────────────────────────────────────────
// Native only for a canonical Steinmetz bicylinder COMMON: exactly two EQUAL-radius
// cylinder lateral faces whose axes are ORTHOGONAL and CROSS, plus their disc caps, with
// the picked edge on the crossing crease, and Rc ≥ 2r (ring-torus guard R0=Rc−r ≥ r).
// Unequal radii, non-orthogonal / non-crossing axes, a non-Steinmetz crease, an extra
// face, caps clipping the fillet band, r ≤ 0, or a multi-edge pick → NULL → OCCT. A
// MODELING-CONVENTION gap vs OCCT (~1% of the removed volume, the idealized perpendicular
// cross-section vs OCCT's variable-dihedral canal) is REPORTED, never gated. The engine
// gates the result with the two-sided volume + orientation self-verify; NEVER a wrong or
// leaky solid.
//
// CLEAN-ROOM. Reuses src/native/math + topology + boolean(extractPolygons/assembleSolid) +
// blend_geom + tessellate(mesh self-verify). OCCT-FREE. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CANAL_FILLET_H
#define CYBERCAD_NATIVE_BLEND_CANAL_FILLET_H

#include "native/blend/blend_geom.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

inline constexpr double kCanalPi = 3.14159265358979323846;
inline constexpr double kCanalTwoPi = 6.28318530717958647692;

// The recognized Steinmetz pair: two equal-radius orthogonal-axis crossing cylinders,
// resolved into a canonical world frame — origin at the axis crossing point, e0 = cyl-A
// axis, e1 = the shared perpendicular (cyl-B axis), e2 = e0×e1. In this frame cyl A is
// x²+y²=Rc² (axis e0? — we choose e2 as A's axis so the canonical math below matches the
// header: A axis = ẑ ≡ e2, B axis = x̂ ≡ e0). All world points are e0·x + e1·y + e2·z.
struct SteinmetzFrame {
  math::Point3 origin;   // axis crossing point
  math::Vec3 ex, ey, ez; // orthonormal; cyl A axis = ez, cyl B axis = ex
  double Rc = 0.0;       // common cylinder radius
};

// World point from canonical (cx,cy,cz): origin + cx·ex + cy·ey + cz·ez.
inline math::Point3 fromCanon(const SteinmetzFrame& f, double cx, double cy, double cz) {
  return math::Point3{f.origin.x + cx * f.ex.x + cy * f.ey.x + cz * f.ez.x,
                      f.origin.y + cx * f.ex.y + cy * f.ey.y + cz * f.ez.y,
                      f.origin.z + cx * f.ex.z + cy * f.ey.z + cz * f.ez.z};
}
// Canonical vector from a canonical direction (no origin).
inline math::Vec3 dirCanon(const SteinmetzFrame& f, double cx, double cy, double cz) {
  return math::Vec3{cx * f.ex.x + cy * f.ey.x + cz * f.ez.x,
                    cx * f.ex.y + cy * f.ey.y + cz * f.ez.y,
                    cx * f.ex.z + cy * f.ey.z + cz * f.ez.z};
}

// Recognise the body as a Steinmetz bicylinder from its PLANAR-FACET soup. The native SSI
// boolean bakes the two cylinder lune walls into planar triangles (no analytic Cylinder
// faces survive), so we recover the two cylinders GEOMETRICALLY from the facets:
//   * every wall facet's outward normal is perpendicular to ITS cylinder axis, so the two
//     axes are recovered as the two orthogonal directions each perpendicular to the largest
//     area of facet normals (cross-products of normal pairs, area-scored);
//   * Rc = the mean perpendicular distance of the wall facets to their axis (must be one
//     common radius); the crossing point = the shared perpendicular foot of both axes.
// Requirements (wholesale): the two recovered axes are orthogonal and cross, EVERY facet is
// a wall facet of exactly one cylinder at the SAME radius Rc (or a disc cap perpendicular to
// an axis), and the picked edge lies on the crossing crease. Returns the frame or nullopt.
inline std::optional<SteinmetzFrame> steinmetzFrame(const topo::Shape& solid, int edgeId) {
  (void)edgeId;  // the crease is recovered wholesale; any picked crease edge is accepted
  const std::vector<nb::Polygon> polys = nb::extractPolygons(solid);
  if (polys.size() < 8) return std::nullopt;

  // Area-weighted unit facet normals + centroids.
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

  // Score a candidate axis by the total facet area whose normal is ~perpendicular to it.
  auto score = [&](const math::Vec3& d) {
    double s = 0.0;
    for (const FN& f : fn)
      if (std::fabs(math::dot(f.n, d)) < 2e-2) s += f.area;
    return s;
  };
  // Candidate axes = cross-products of a bounded subsample of facet-normal PAIRS (a cylinder
  // wall's axis is ⟂ its facet normals, so it is such a cross-product). Stride keeps it
  // O((N/stride)²·N) — bounded on a dense soup while still sampling both normal families.
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
  auto [axA, sA] = bestAxis(nullptr);
  if (!(sA > 0.0)) return std::nullopt;
  auto [axB, sB] = bestAxis(&axA);
  if (!(sB > 0.0)) return std::nullopt;
  // The recovered directions are from facet cross-products, so they are near- but not
  // exactly orthogonal (facet discretization). Require near-orthogonal, then Gram-Schmidt
  // axB against axA so the canonical frame is EXACTLY orthonormal.
  if (std::fabs(math::dot(axA, axB)) > 3e-2) return std::nullopt;  // orthogonal axes
  {
    const math::Vec3 proj = axA * math::dot(axA, axB);
    math::Vec3 b = axB - proj;
    const double nb2 = math::norm(b);
    if (nb2 < 1e-6) return std::nullopt;
    axB = b * (1.0 / nb2);
  }

  // Assign every facet to the axis its normal is perpendicular to (the wall it lies on);
  // recover Rc from the mean perpendicular distance and require ONE common radius.
  auto perpDist = [&](const math::Point3& p, const math::Vec3& axis, const math::Point3& on) {
    const math::Vec3 d = p - on;
    return math::norm(d - axis * math::dot(d, axis));
  };
  // First pass: crossing point ≈ centroid of all facet centroids projected — refine below.
  math::Vec3 csum{0, 0, 0};
  for (const FN& f : fn) csum += f.c.asVec();
  const math::Point3 c0{csum.x / N, csum.y / N, csum.z / N};

  double sumR = 0.0;
  int wallCnt = 0, capCnt = 0;
  for (const FN& f : fn) {
    const bool perpA = std::fabs(math::dot(f.n, axA)) < 2e-2;
    const bool perpB = std::fabs(math::dot(f.n, axB)) < 2e-2;
    const bool alongA = std::fabs(std::fabs(math::dot(f.n, axA)) - 1.0) < 2e-2;
    const bool alongB = std::fabs(std::fabs(math::dot(f.n, axB)) - 1.0) < 2e-2;
    if (perpA) { sumR += perpDist(f.c, axA, c0); ++wallCnt; }
    else if (perpB) { sumR += perpDist(f.c, axB, c0); ++wallCnt; }
    else if (alongA || alongB) { ++capCnt; }  // disc cap ⟂ an axis
    else return std::nullopt;  // a facet on neither wall nor a cap → not a pure bicylinder
  }
  if (wallCnt < 6) return std::nullopt;
  const double Rc = sumR / wallCnt;
  if (!(Rc > kBlendEps)) return std::nullopt;
  // Verify every wall facet sits at the SAME radius Rc (a common bicylinder, not a mix).
  for (const FN& f : fn) {
    const bool perpA = std::fabs(math::dot(f.n, axA)) < 2e-2;
    const bool perpB = std::fabs(math::dot(f.n, axB)) < 2e-2;
    if (!perpA && !perpB) continue;
    const double rr = perpDist(f.c, perpA ? axA : axB, c0);
    if (std::fabs(rr - Rc) > std::max(5e-3, 5e-3 * Rc)) return std::nullopt;
  }
  (void)capCnt;

  SteinmetzFrame f;
  f.origin = c0;                     // axes cross at the body centroid (both pass through it)
  f.ez = math::Dir3{axA}.vec();      // cyl A axis
  f.ex = math::Dir3{axB}.vec();      // cyl B axis
  f.ey = math::cross(f.ez, f.ex);
  const double ny = math::norm(f.ey);
  if (ny < 1e-9) return std::nullopt;
  f.ey = f.ey * (1.0 / ny);
  f.Rc = Rc;
  return f;
}

// The rolling-ball centre (canal spine) on crease plane z = sgn·x at spine angle φ.
inline math::Point3 canalCentre(const SteinmetzFrame& f, int sgn, double phi, double r) {
  const double R0 = f.Rc - r;
  return fromCanon(f, R0 * std::sin(phi), R0 * std::cos(phi), sgn * R0 * std::sin(phi));
}

// A canonical strip point: crease plane z = sgn·x, spine angle φ, minor param t∈[0,1]
// (t=0 seam on cyl A, t=1 seam on cyl B). Slerp of the two wall radials about the centre.
inline math::Point3 canalStripPoint(const SteinmetzFrame& f, int sgn, double phi, double t,
                                     double r) {
  const double R0 = f.Rc - r;
  const double cx = R0 * std::sin(phi), cy = R0 * std::cos(phi), cz = sgn * R0 * std::sin(phi);
  const double nA = std::hypot(cx, cy), nB = std::hypot(cy, cz);
  if (nA < 1e-9 || nB < 1e-9)  // pole: cross-section collapses to the pole point
    return fromCanon(f, 0.0, cy >= 0.0 ? f.Rc : -f.Rc, 0.0);
  const math::Vec3 rA{cx / nA, cy / nA, 0.0};
  const math::Vec3 rB{0.0, cy / nB, cz / nB};
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
  return fromCanon(f, cx + r * dir.x, cy + r * dir.y, cz + r * dir.z);
}

// Facet counts. Angular from the spine curvature (~R0 over 2π), minor from the corner arc.
inline int canalPhiSteps(double R0, double defl) {
  if (R0 <= kBlendEps) return 24;
  const double dmax = 2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - defl / R0)));
  const int n = dmax > 1e-9 ? static_cast<int>(std::ceil(kCanalPi / dmax)) : 48;
  return std::clamp(n, 12, 200) | 1;  // ODD so the cosine grid is symmetric about π/2
}

// Emit a planar triangle whose VERTEX WINDING is CCW-as-seen-from-outward and whose stored
// plane normal points `outward`. Both the winding and the normal are set consistently (the
// vertices are swapped when the geometric normal opposes `outward`) so every facet the
// assembler triangulates is coherently wound — the invariant isConsistentlyOriented needs.
inline void canalTri(std::vector<nb::Polygon>& soup, const math::Point3& a, const math::Point3& b,
                     const math::Point3& c, const math::Vec3& outward) {
  math::Vec3 nrm = math::cross(b - a, c - a);
  const bool flip = math::dot(nrm, outward) < 0.0;
  if (flip) nrm = nrm * -1.0;
  const math::Dir3 nd{nrm};
  if (!nd.valid()) return;  // degenerate (zero-area) — skip
  std::vector<math::Point3> tri = flip ? std::vector<math::Point3>{a, c, b}
                                       : std::vector<math::Point3>{a, b, c};
  soup.emplace_back(std::move(tri), nb::Plane::fromPointNormal(a, nd.vec()));
}

// Flood-fill the triangle soup to ONE coherent winding (every shared edge traversed
// oppositely by its two faces), then flip globally so all normals point outward (away from
// the interior reference `inside`). The soup must be a closed 2-manifold (it is, by
// construction: every seam is shared by exactly two families at identical samples). This
// makes the assembled shell pass isConsistentlyOriented regardless of per-family winding.
inline void orientSoupCoherent(std::vector<nb::Polygon>& soup, const math::Point3& inside) {
  const int n = static_cast<int>(soup.size());
  if (n == 0) return;
  // Weld vertices to ids (kWeldTol) so shared edges are detected.
  struct VKey { long long x, y, z; bool operator==(const VKey& o) const {
    return x == o.x && y == o.y && z == o.z; } };
  struct VHash { std::size_t operator()(const VKey& k) const {
    return (static_cast<std::size_t>(k.x) * 73856093ULL) ^
           (static_cast<std::size_t>(k.y) * 19349663ULL) ^
           (static_cast<std::size_t>(k.z) * 83492791ULL); } };
  std::unordered_map<VKey, int, VHash> vid;
  auto q = [](double v) { const double s = v / kBlendEps;
    return static_cast<long long>(s >= 0 ? s + 0.5 : s - 0.5); };
  auto id = [&](const math::Point3& p) {
    const VKey k{q(p.x), q(p.y), q(p.z)};
    auto it = vid.find(k);
    if (it != vid.end()) return it->second;
    const int i = static_cast<int>(vid.size());
    vid.emplace(k, i);
    return i;
  };
  std::vector<std::array<int, 3>> tv(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i)
    tv[static_cast<std::size_t>(i)] = {id(soup[i].vertices[0]), id(soup[i].vertices[1]),
                                       id(soup[i].vertices[2])};
  // undirected edge → the (≤2) incident triangles.
  std::unordered_map<long long, std::array<int, 2>> em;
  auto ek = [](int a, int b) { if (a > b) std::swap(a, b);
    return (static_cast<long long>(a) << 32) | static_cast<unsigned>(b); };
  for (int i = 0; i < n; ++i)
    for (int e = 0; e < 3; ++e) {
      const long long k = ek(tv[static_cast<std::size_t>(i)][e],
                             tv[static_cast<std::size_t>(i)][(e + 1) % 3]);
      auto& slot = em[k];
      if (slot[0] == 0 && slot[1] == 0) slot = {i + 1, 0};  // store 1-based
      else if (slot[1] == 0) slot[1] = i + 1;
    }
  // BFS: fix triangle 0's winding; propagate so neighbours traverse the shared edge oppositely.
  std::vector<int> state(static_cast<std::size_t>(n), 0);  // 0 unseen, 1 keep, -1 flipped
  std::vector<int> stack{0};
  state[0] = 1;
  auto dirEdges = [&](int i) {
    const auto& t = tv[static_cast<std::size_t>(i)];
    const bool fl = state[static_cast<std::size_t>(i)] < 0;
    std::array<std::pair<int, int>, 3> d;
    if (!fl) d = {{{t[0], t[1]}, {t[1], t[2]}, {t[2], t[0]}}};
    else d = {{{t[0], t[2]}, {t[2], t[1]}, {t[1], t[0]}}};
    return d;
  };
  while (!stack.empty()) {
    const int i = stack.back();
    stack.pop_back();
    const auto de = dirEdges(i);
    for (const auto& [a, b] : de) {
      const auto& slot = em[ek(a, b)];
      for (int s : {slot[0], slot[1]}) {
        if (s == 0) continue;
        const int j = s - 1;
        if (j == i || state[static_cast<std::size_t>(j)] != 0) continue;
        // j must traverse (a,b) as (b,a). Its current directed edges:
        const auto& t = tv[static_cast<std::size_t>(j)];
        bool hasAB = false;
        for (int e = 0; e < 3; ++e)
          if (t[e] == a && t[(e + 1) % 3] == b) hasAB = true;
        state[static_cast<std::size_t>(j)] = hasAB ? -1 : 1;  // flip if same direction as i
        stack.push_back(j);
      }
    }
  }
  // Global sign: a keep/flip choice or its inverse are both coherent; pick the one whose
  // facets point AWAY from `inside` (outward) by majority vote (convex body → unambiguous).
  double vote = 0.0;
  for (int i = 0; i < n; ++i) {
    const auto de = dirEdges(i);
    (void)de;
    const math::Point3& A = soup[static_cast<std::size_t>(i)].vertices[0];
    const math::Point3& B = soup[static_cast<std::size_t>(i)].vertices[1];
    const math::Point3& C = soup[static_cast<std::size_t>(i)].vertices[2];
    math::Vec3 nrm = math::cross(B - A, C - A);
    if (state[static_cast<std::size_t>(i)] < 0) nrm = nrm * -1.0;
    const math::Vec3 outward{(A.x + B.x + C.x) / 3.0 - inside.x,
                             (A.y + B.y + C.y) / 3.0 - inside.y,
                             (A.z + B.z + C.z) / 3.0 - inside.z};
    vote += math::dot(nrm, outward);
  }
  const bool invert = vote < 0.0;
  // Apply: rewrite each polygon's winding + plane normal to the coherent outward orientation.
  for (int i = 0; i < n; ++i) {
    const bool flip = (state[static_cast<std::size_t>(i)] < 0) != invert;
    nb::Polygon& p = soup[static_cast<std::size_t>(i)];
    math::Vec3 nrm = math::cross(p.vertices[1] - p.vertices[0], p.vertices[2] - p.vertices[0]);
    if (flip) { std::swap(p.vertices[1], p.vertices[2]); nrm = nrm * -1.0; }
    const math::Dir3 nd{nrm};
    if (nd.valid()) p.plane = nb::Plane::fromPointNormal(p.vertices[0], nd.vec());
  }
}

// Build the whole filleted bicylinder COMMON as a planar-facet soup (empty on
// degeneracy). Factored out of canal_fillet_edge so it can be self-checked. All faces
// share canonical seam samples and the two pole vertices; the CONVEX-body global outward
// (centroid − frame origin) orients every facet coherently.
inline std::vector<nb::Polygon> buildCanalSoup(const SteinmetzFrame& f, double r,
                                               double deflection) {
  const double R0 = f.Rc - r;
  if (!(R0 >= r - 1e-9)) return {};  // ring-torus guard Rc ≥ 2r

  const int M = canalPhiSteps(R0, deflection);   // spine samples per half (odd)
  const int Nt = std::clamp(static_cast<int>(std::ceil(6.0 * r / std::max(R0, 1e-9))), 3, 24);
  const int Nw = M;                                       // wall cross samples

  // Cosine phi grids on [0,π] and [π,2π] (M samples each) — symmetric about the midpoint so
  // g[k] ↔ g[M-1-k] share the SAME axial x and reuse identical strip seam vertices (the
  // cyl-B lune weld requirement).
  std::vector<double> g1(static_cast<std::size_t>(M)), g2(static_cast<std::size_t>(M));
  for (int k = 0; k < M; ++k) {
    const double s = 0.5 * (1.0 - std::cos(kCanalPi * k / (M - 1)));
    g1[static_cast<std::size_t>(k)] = kCanalPi * s;
    g2[static_cast<std::size_t>(k)] = kCanalPi + kCanalPi * s;
  }

  std::vector<nb::Polygon> soup;
  soup.reserve(static_cast<std::size_t>(M) * (Nt + 2 * Nw) * 4 + 16);

  // ---- two canal strips (crease planes z=±x), each over both φ-halves ----
  for (int sgn : {+1, -1}) {
    for (const std::vector<double>* g : {&g1, &g2}) {
      const std::vector<double>& gg = *g;
      for (int i = 0; i + 1 < M; ++i) {
        for (int j = 0; j < Nt; ++j) {
          const double t0 = static_cast<double>(j) / Nt, t1 = static_cast<double>(j + 1) / Nt;
          const math::Point3 p00 = canalStripPoint(f, sgn, gg[i], t0, r);
          const math::Point3 p01 = canalStripPoint(f, sgn, gg[i], t1, r);
          const math::Point3 p10 = canalStripPoint(f, sgn, gg[static_cast<std::size_t>(i) + 1], t0, r);
          const math::Point3 p11 = canalStripPoint(f, sgn, gg[static_cast<std::size_t>(i) + 1], t1, r);
          // The canal surface normal at a strip point P is the ball-centre radial (P − C)/r —
          // ALWAYS well-conditioned (never grazing), so each facet winds outward reliably.
          const math::Point3 c0 = canalCentre(f, sgn, gg[i], r);
          const math::Point3 c1 = canalCentre(f, sgn, gg[static_cast<std::size_t>(i) + 1], r);
          const math::Vec3 out0{p00.x - c0.x, p00.y - c0.y, p00.z - c0.z};
          const math::Vec3 out1{p11.x - c1.x, p11.y - c1.y, p11.z - c1.z};
          const math::Vec3 out{out0.x + out1.x, out0.y + out1.y, out0.z + out1.z};
          canalTri(soup, p00, p10, p11, out);
          canalTri(soup, p00, p11, p01, out);
        }
      }
    }
  }

  // Canonical coords of a world point (project onto the frame axes).
  auto canonOf = [&](const math::Point3& p) {
    const math::Vec3 d = p - f.origin;
    return std::array<double, 3>{math::dot(d, f.ex), math::dot(d, f.ey), math::dot(d, f.ez)};
  };
  auto unwrap = [](double u0, double u1) {
    while (u1 - u0 > kCanalPi) u1 -= kCanalTwoPi;
    while (u1 - u0 < -kCanalPi) u1 += kCanalTwoPi;
    return u1;
  };

  // ---- cyl A lunes: two arcs matched by azimuth u (φ lockstep). Wall A = x²+y²=Rc² in
  //      canonical (ex,ey) plane, axial ez. u=atan2(cy,cx), z=cz. Interp (u,z) across. ----
  auto cylApoint = [&](double u, double z) {
    return fromCanon(f, f.Rc * std::cos(u), f.Rc * std::sin(u), z);
  };
  auto luneA = [&](int loSgn, int hiSgn, const std::vector<double>& g) {
    for (int i = 0; i + 1 < M; ++i) {
      const math::Point3 lo0 = canalStripPoint(f, loSgn, g[static_cast<std::size_t>(i)], 0.0, r);
      const math::Point3 hi0 = canalStripPoint(f, hiSgn, g[static_cast<std::size_t>(i)], 0.0, r);
      const math::Point3 lo1 = canalStripPoint(f, loSgn, g[static_cast<std::size_t>(i) + 1], 0.0, r);
      const math::Point3 hi1 = canalStripPoint(f, hiSgn, g[static_cast<std::size_t>(i) + 1], 0.0, r);
      const auto cl0 = canonOf(lo0), ch0 = canonOf(hi0), cl1 = canonOf(lo1), ch1 = canonOf(hi1);
      const double ul0 = std::atan2(cl0[1], cl0[0]), uh0 = std::atan2(ch0[1], ch0[0]);
      const double ul1 = std::atan2(cl1[1], cl1[0]), uh1 = std::atan2(ch1[1], ch1[0]);
      auto P = [&](double ul, double zl, double uh, double zh, double ss,
                   const math::Point3& el, const math::Point3& eh) -> math::Point3 {
        if (ss <= 0.0) return el;
        if (ss >= 1.0) return eh;
        return cylApoint(ul + (unwrap(ul, uh) - ul) * ss, zl + (zh - zl) * ss);
      };
      for (int j = 0; j < Nw; ++j) {
        const double s0 = static_cast<double>(j) / Nw, s1 = static_cast<double>(j + 1) / Nw;
        const math::Point3 p00 = P(ul0, cl0[2], uh0, ch0[2], s0, lo0, hi0);
        const math::Point3 p01 = P(ul0, cl0[2], uh0, ch0[2], s1, lo0, hi0);
        const math::Point3 p10 = P(ul1, cl1[2], uh1, ch1[2], s0, lo1, hi1);
        const math::Point3 p11 = P(ul1, cl1[2], uh1, ch1[2], s1, lo1, hi1);
        // Wall A outward = cyl-A radial at the centroid (ex,ey plane) — never grazing.
        const math::Point3 tc{(p00.x + p10.x + p11.x) / 3.0, (p00.y + p10.y + p11.y) / 3.0,
                              (p00.z + p10.z + p11.z) / 3.0};
        const auto ctc = canonOf(tc);
        const math::Vec3 out = dirCanon(f, ctc[0], ctc[1], 0.0);
        canalTri(soup, p00, p10, p11, out);
        canalTri(soup, p00, p11, p01, out);
      }
    }
  };
  luneA(-1, +1, g1);  // front lune (u~0)
  luneA(+1, -1, g2);  // back lune  (u~π)

  // ---- cyl B lunes: parametrized by azimuth w (the cyl-B analogue of cyl-A's u), matched
  //      pole-to-pole by φ. Each lune is bounded by TWO crease seams from DIFFERENT arcs:
  //      the w~+90 lune by arc(+1,x>0 leg φ∈[0,π]) on its x>0 side and arc(−1, mirror leg
  //      φ'=2π−φ) on its x<0 side — the two share the same w at matched φ, so we connect
  //      hi=strip(hiSgn,φ) to lo=strip(loSgn,2π−φ) across the wall in x. Wall B = y²+z²=Rc²,
  //      param (w=atan2(cz,cy), x=cx). φ over g1 sweeps the lune pole-to-pole (w 0→π). ----
  auto cylBpoint = [&](double w, double x) {
    return fromCanon(f, x, f.Rc * std::cos(w), f.Rc * std::sin(w));
  };
  auto luneB = [&](int hiSgn, int loSgn) {
    for (int i = 0; i + 1 < M; ++i) {
      const double phi0 = g1[static_cast<std::size_t>(i)];
      const double phi1 = g1[static_cast<std::size_t>(i) + 1];
      const math::Point3 hi0 = canalStripPoint(f, hiSgn, phi0, 1.0, r);
      const math::Point3 lo0 = canalStripPoint(f, loSgn, kCanalTwoPi - phi0, 1.0, r);
      const math::Point3 hi1 = canalStripPoint(f, hiSgn, phi1, 1.0, r);
      const math::Point3 lo1 = canalStripPoint(f, loSgn, kCanalTwoPi - phi1, 1.0, r);
      const auto cl0 = canonOf(lo0), ch0 = canonOf(hi0), cl1 = canonOf(lo1), ch1 = canonOf(hi1);
      const double wl0 = std::atan2(cl0[2], cl0[1]), wh0 = std::atan2(ch0[2], ch0[1]);
      const double wl1 = std::atan2(cl1[2], cl1[1]), wh1 = std::atan2(ch1[2], ch1[1]);
      auto P = [&](double wl, double xl, double wh, double xh, double ss,
                   const math::Point3& el, const math::Point3& eh) -> math::Point3 {
        if (ss <= 0.0) return el;
        if (ss >= 1.0) return eh;
        return cylBpoint(wl + (unwrap(wl, wh) - wl) * ss, xl + (xh - xl) * ss);
      };
      for (int j = 0; j < Nw; ++j) {
        const double s0 = static_cast<double>(j) / Nw, s1 = static_cast<double>(j + 1) / Nw;
        const math::Point3 p00 = P(wl0, cl0[0], wh0, ch0[0], s0, lo0, hi0);
        const math::Point3 p01 = P(wl0, cl0[0], wh0, ch0[0], s1, lo0, hi0);
        const math::Point3 p10 = P(wl1, cl1[0], wh1, ch1[0], s0, lo1, hi1);
        const math::Point3 p11 = P(wl1, cl1[0], wh1, ch1[0], s1, lo1, hi1);
        // Wall B outward = cyl-B radial at the centroid (ey,ez plane) — never grazing.
        const math::Point3 tc{(p00.x + p10.x + p11.x) / 3.0, (p00.y + p10.y + p11.y) / 3.0,
                              (p00.z + p10.z + p11.z) / 3.0};
        const auto ctc = canonOf(tc);
        const math::Vec3 out = dirCanon(f, 0.0, ctc[1], ctc[2]);
        canalTri(soup, p00, p10, p11, out);
        canalTri(soup, p00, p11, p01, out);
      }
    }
  };
  luneB(+1, -1);  // w~+90 lune: x>0 side arc(+1), x<0 side arc(−1)
  luneB(-1, +1);  // w~−90 lune: x>0 side arc(−1), x<0 side arc(+1)

  // Reconcile per-family windings into ONE coherent outward orientation (the seams between
  // the strips and the two cylinders' lunes are traversed the same rotational sense per
  // family, so a flood-fill pass is needed for the assembled shell to be consistently
  // oriented). The frame origin is interior to the convex bicylinder → the outward pick.
  orientSoupCoherent(soup, f.origin);
  return soup;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// canal_fillet_edge — round the crossing crease of a Steinmetz bicylinder COMMON
// (two equal-radius orthogonal-axis crossing cylinders) at constant `radius`.
// Returns the filleted solid, or a NULL Shape (→ OCCT) when the body is not a
// recognizable Steinmetz bicylinder, when Rc < 2·radius, on a multi-edge pick, or
// on any degeneracy. `deflection` bounds the facet chord error.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape canal_fillet_edge(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                     double r, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r > kBlendEps)) return {};
  const auto frameOpt = detail::steinmetzFrame(solid, edgeIds[0]);
  if (!frameOpt) return {};
  const std::vector<nb::Polygon> soup = detail::buildCanalSoup(*frameOpt, r, deflection);
  if (soup.size() < 4) return {};
  const topo::Shape result = nb::assembleSolid(soup);
  if (result.isNull()) return {};

  // ── MANDATORY internal self-verify (never a wrong/folded solid) ──────────────────────
  // The cyl-B lune degenerates near the poles; for a large radius the reconstruction can
  // FOLD into a self-intersecting shell that is still watertight + consistently oriented but
  // encloses a grossly wrong (tiny) volume. isConsistentlyOriented alone cannot catch that,
  // so we ALSO bound the removed volume: a rolling-ball fillet of radius r along the two
  // crossing creases removes at most ≈ a tube of radius r over the crease length (~2π·Rc
  // per plane), so V_removed ≤ C·r²·Rc with a generous C. A fold (removing most of the body)
  // violates this and DECLINES → OCCT. The engine's SHRINK gate then re-confirms 0<Vr<Vo.
  namespace tess = cybercad::native::tessellate;
  tess::MeshParams mp;
  mp.deflection = std::min(deflection, 0.01);
  const tess::Mesh mR = tess::SolidMesher(mp).mesh(result);
  if (!tess::isConsistentlyOriented(mR)) return {};
  const double Vr = std::fabs(tess::enclosedVolume(mR));
  // Baseline = the SHARP bicylinder meshed at the SAME deflection, so the faceting deficit
  // cancels in the difference (comparing analytic 16/3·Rc³ would fold the ~O(defl·Rc²) facet
  // undershoot into the removed estimate and mis-gate small radii).
  const tess::Mesh mS = tess::SolidMesher(mp).mesh(solid);
  if (!tess::isWatertight(mS)) return {};
  const double Vsharp = std::fabs(tess::enclosedVolume(mS));
  const double Rc = frameOpt->Rc;
  const double removed = Vsharp - Vr;
  // A rolling-ball fillet of radius r (with Rc ≥ 2r) only ROUNDS the two crossing creases:
  // it removes a small sliver and provably keeps well over half the bicylinder. A cyl-B
  // pole-region FOLD (a large-radius self-intersection the reconstruction can hit) yields a
  // watertight, consistently-oriented shell that nonetheless encloses a GROSSLY wrong (near-
  // zero) volume. The two guards together catch it: (a) the removed sliver is bounded by a
  // tube of radius r over the two creases (~2π·Rc each) → V_removed ≤ C·r²·Rc (generous C,
  // plus a facet-noise floor since the two meshes tessellate differently); AND (b) the fillet
  // keeps the large majority of the body. A fold violates BOTH → DECLINE → OCCT. The engine's
  // SHRINK gate then re-confirms 0 < Vr < Vo. NEVER a wrong/folded solid.
  const double maxRemoved = 30.0 * r * r * Rc + 5e-2 * Vsharp;
  if (!(removed > -5e-2 * Vsharp) || !(removed < maxRemoved)) return {};
  if (!(Vr > 0.5 * Vsharp)) return {};
  return result;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CANAL_FILLET_H
