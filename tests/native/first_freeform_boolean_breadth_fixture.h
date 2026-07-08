// SPDX-License-Identifier: Apache-2.0
//
// first_freeform_boolean_breadth_fixture.h — MOAT M2-breadth (COMMON): the
// COMPLEMENTARY keep-side oracle for the bowl-lidded convex-quad prism.
//
// The M2-assembly slice landed the CUT keep-side (A ∩ {x ≤ 0}) with the closed-form
// oracle ∫∫_{Q ∩ {x ≤ 0}} (H0 + a(x²+y²)) dA. COMMON is the OTHER side of the same
// cut plane — A ∩ {x ≥ 0} — computed by the SAME landed verb at KeepSide::Above. This
// header ADDS only the complementary clip / volume and the partition-closure helpers,
// reusing the M2-assembly fixture (buildOperand / polyVolume / quadXY / cutPlane /
// clipXle0 / fullVolume / cutVolume) BYTE-IDENTICALLY. OCCT-FREE.
//
#ifndef CYBERCAD_TESTS_NATIVE_FIRST_FREEFORM_BOOLEAN_BREADTH_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_FIRST_FREEFORM_BOOLEAN_BREADTH_FIXTURE_H

#include "native/first_freeform_boolean_fixture.h"

#include <cmath>
#include <vector>

namespace first_freeform_boolean_breadth_fixture {

namespace ffx = first_freeform_boolean_fixture;
using P2 = ffx::P2;

// Sutherland–Hodgman clip keeping x ≥ 0 — the exact complement of the M2-assembly
// clipXle0 (which keeps x ≤ 0). Together they partition the quad Q along the chord x = 0.
inline std::vector<P2> clipXge0(const std::vector<P2>& in) {
  std::vector<P2> out;
  const int n = static_cast<int>(in.size());
  for (int i = 0; i < n; ++i) {
    const P2 a = in[i], b = in[(i + 1) % n];
    const bool ai = a.x >= 0, bi = b.x >= 0;
    if (ai) out.push_back(a);
    if (ai != bi) { const double t = (0.0 - a.x) / (b.x - a.x); out.push_back({0.0, a.y + t * (b.y - a.y)}); }
  }
  return out;
}

// The COMMON (complementary) closed-form volume ∫∫_{Q ∩ {x ≥ 0}} (H0 + a(x²+y²)) dA.
inline double commonVolume() { return ffx::polyVolume(clipXge0(ffx::quadXY())); }

// |signed area| of a convex polygon (for the clip-complement area identity).
inline double polyArea(const std::vector<P2>& p) {
  double a = 0.0;
  const int n = static_cast<int>(p.size());
  for (int i = 0, j = n - 1; i < n; j = i++) a += p[j].x * p[i].y - p[i].x * p[j].y;
  return std::fabs(0.5 * a);
}

}  // namespace first_freeform_boolean_breadth_fixture

#endif  // CYBERCAD_TESTS_NATIVE_FIRST_FREEFORM_BOOLEAN_BREADTH_FIXTURE_H
