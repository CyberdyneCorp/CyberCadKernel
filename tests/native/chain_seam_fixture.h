// SPDX-License-Identifier: Apache-2.0
//
// chain_seam_fixture.h — the reachable proof fixture for MOAT M2 blocker #4 (≥3-seam):
// the EDGE-straddling box `B` whose THREE consecutive faces slice `A`'s Bézier wall (a
// three-arc, two-junction seam CHAIN), plus the closed-form STRIP-clip volume oracle.
// Shared by the host analytic gate (and later the sim native-vs-OCCT gate). OCCT-FREE.
//
// `A` = the bowl-lidded convex-quad prism (`first_freeform_boolean_fixture`), footprint
// quad `Q` in world (x,y) ≈ [−0.35, 0.35]², bowl integrand `H0 + a·(x²+y²)`.
// `B` = the axis-aligned box `x ∈ [−0.15, 0.15], y ∈ [0.0, 0.8], z ∈ [−0.6, 0.2]`,
// straddling one EDGE of `A`, so its `x = −0.15`, `x = +0.15` and `y = 0` faces each
// slice `A`'s wall (the three cutting faces) and the other three faces contain `A`. `B`
// removes exactly the STRIP `A ∩ {−0.15 ≤ x ≤ 0.15, y ≥ 0}`.
//
// The oracle integrates the bowl integrand over `Q` clipped to the strip, so
// `V(A ∩ B)`, `V(A − B)`, `V(A ∪ B)` are mesh-free / OCCT-free (for the future weld).
//
#ifndef CYBERCAD_TESTS_NATIVE_CHAIN_SEAM_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_CHAIN_SEAM_FIXTURE_H

#include "native/first_freeform_boolean_fixture.h"
#include "native/multi_seam_fixture.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace chain_seam_fixture {

namespace topo = cybercad::native::topology;
namespace fmath = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;
namespace msx = multi_seam_fixture;

// The three cutting-face box planes in world coordinates.
inline constexpr double kX0 = -0.15, kX1 = 0.15;  // the two parallel end faces (iso-u arcs)
inline constexpr double kY0 = 0.0, kY1 = 0.8;      // middle face at y=kY0 (iso-v arc)
inline constexpr double kZ0 = -0.6, kZ1 = 0.2;

/// The edge-straddling box `B` (six single-quad Plane faces, outward normals). Reuses
/// the byte-identical `multi_seam_fixture::buildCornerBox` builder.
inline topo::Shape edgeBox() { return msx::buildCornerBox(kX0, kX1, kY0, kY1, kZ0, kZ1); }
inline constexpr double kBoxVolume = (kX1 - kX0) * (kY1 - kY0) * (kZ1 - kZ0);  // 0.30*0.80*0.80

// ── closed-form strip-clip volume oracle (mesh-free, no OCCT) ─────────────────────
using ffx::P2;
using msx::clipHalf;

/// Clip a convex polygon to the strip {kX0 ≤ x ≤ kX1, y ≥ kY0} (shifting the plane to
/// the origin per clip, reusing `clipHalf`). Returns the clipped polygon.
inline std::vector<P2> clipStrip(const std::vector<P2>& in) {
  auto shift = [](std::vector<P2> p, int axis, double v) {
    for (P2& q : p) { if (axis == 0) q.x -= v; else q.y -= v; }
    return p;
  };
  auto unshift = [](std::vector<P2> p, int axis, double v) {
    for (P2& q : p) { if (axis == 0) q.x += v; else q.y += v; }
    return p;
  };
  // x ≥ kX0
  std::vector<P2> c = unshift(clipHalf(shift(in, 0, kX0), 0, true), 0, kX0);
  // x ≤ kX1
  c = unshift(clipHalf(shift(c, 0, kX1), 0, false), 0, kX1);
  // y ≥ kY0
  c = unshift(clipHalf(shift(c, 1, kY0), 1, true), 1, kY0);
  return c;
}

inline double volFull() { return ffx::polyVolume(ffx::quadXY()); }
/// V(A ∩ B): the bowl integrand over `Q ∩ strip` (the removed strip).
inline double volCommon() {
  const std::vector<P2> q = clipStrip(ffx::quadXY());
  return q.size() >= 3 ? ffx::polyVolume(q) : 0.0;
}
inline double volCut() { return volFull() - volCommon(); }
inline double volUnion() { return kBoxVolume + volCut(); }

// ── closed-form UV strip-area oracle (the removed strip's wall projection) ────────
// The bowl wall is parameterised over UV = [0,1]², trimmed to Q(uv). The removed strip
// `A ∩ {kX0 ≤ x ≤ kX1, y ≥ kY0}` maps to `Q(uv) ∩ {u0 ≤ u ≤ u1, v ≥ v0}` with
// u = x + ½, v = y + ½, so u0 = kX0+½, u1 = kX1+½, v0 = kY0+½. Its UV-domain area is
// the shoelace of that clipped polygon — the ground truth for the strip sub-face UV
// area. Mesh-free, OCCT-free.
inline double uvStripArea() {
  struct Q2 { double u, v; };
  std::vector<Q2> poly;
  for (const auto& c : ffx::fx::quadUV()) poly.push_back({c.x, c.y});
  auto clip = [](std::vector<Q2> in, int axis, double val, bool keepGreater) {
    std::vector<Q2> out;
    const int n = static_cast<int>(in.size());
    auto coord = [&](const Q2& q) { return axis == 0 ? q.u : q.v; };
    for (int i = 0; i < n; ++i) {
      const Q2 a = in[i], b = in[(i + 1) % n];
      const bool ai = keepGreater ? coord(a) >= val : coord(a) <= val;
      const bool bi = keepGreater ? coord(b) >= val : coord(b) <= val;
      if (ai) out.push_back(a);
      if (ai != bi) {
        const double ca = coord(a), cb = coord(b), t = (val - ca) / (cb - ca);
        out.push_back({a.u + t * (b.u - a.u), a.v + t * (b.v - a.v)});
      }
    }
    return out;
  };
  std::vector<Q2> c = clip(poly, 0, kX0 + 0.5, true);    // u ≥ kX0+½
  c = clip(c, 0, kX1 + 0.5, false);                       // u ≤ kX1+½
  c = clip(c, 1, kY0 + 0.5, true);                        // v ≥ kY0+½
  double A = 0.0;
  const int n = static_cast<int>(c.size());
  for (int i = 0, j = n - 1; i < n; j = i++) A += c[j].u * c[i].v - c[i].u * c[j].v;
  return std::fabs(0.5 * A);
}

}  // namespace chain_seam_fixture

#endif  // CYBERCAD_TESTS_NATIVE_CHAIN_SEAM_FIXTURE_H
