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

}  // namespace chain_seam_fixture

#endif  // CYBERCAD_TESTS_NATIVE_CHAIN_SEAM_FIXTURE_H
