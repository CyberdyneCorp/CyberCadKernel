// SPDX-License-Identifier: Apache-2.0
//
// smooth_trim_split_fixture.h — the reachable proof fixture for MOAT M2b / B2
// SMOOTH-TRIM (closed / circular seam) generalisation. OCCT-FREE; requires
// CYBERCAD_HAS_NUMSCI for the M1 seam trace.
//
// Parent: the SAME quad-trimmed Bézier bowl the byte-frozen B2 fixture uses
// (`face_split_fixture::parentFace`) — z = a·((u−½)²+(v−½)²), x = u−½, y = v−½.
//
// Cutter: the HORIZONTAL plane z = c (with c < the quad's interior bowl height). The
// bowl ∩ {z=c} is the CIRCLE (u−½)²+(v−½)² = c/a of radius ρ = √(c/a) centred at
// (½,½) in the bowl's own (u,v) — a CLOSED smooth seam ENTIRELY INTERIOR to the quad
// (crossings == 0). This is exactly the seam byte-frozen B2 `splitFace` DECLINES.
//
// The seam is the REAL M1 WLine from ssi::trace_intersection(bowl, plane z=c) — the
// tracer is CONSUMED, not synthesised. Closed-form disk area = π·ρ² = π·c/a.
//
#ifndef CYBERCAD_TESTS_NATIVE_SMOOTH_TRIM_SPLIT_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_SMOOTH_TRIM_SPLIT_FIXTURE_H

#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"

#include "native/face_split_fixture.h"

namespace smooth_trim_split_fixture {

namespace ssi = cybercad::native::ssi;
namespace fmath = cybercad::native::math;
namespace fx = face_split_fixture;

inline constexpr double kBowlA = fx::kBowlA;  // 0.4 — the shared bowl amplitude
inline constexpr double kRho = 0.20;          // target circular-seam radius in (u,v)
inline constexpr double kCutZ = kBowlA * kRho * kRho;  // c = a·ρ² → z = c cuts at radius ρ

// The closed-form UV area the circular seam encloses: π·ρ² = π·c/a.
inline double closedFormDiskArea() { return 3.14159265358979323846 * kRho * kRho; }

// The horizontal cutter plane z = kCutZ as a SurfaceAdapter (frame Z = +z, so its
// param (u,v) → (u, v, c) and the intersection with the bowl is the circle above).
inline ssi::SurfaceAdapter cutterAdapter() {
  fmath::Plane pl{};
  pl.pos.origin = fmath::Point3{0.0, 0.0, kCutZ};  // default Ax3 has z = {0,0,1}
  return ssi::makePlaneAdapter(pl, ssi::ParamBox{-0.6, 0.6, -0.6, 0.6});
}

// The real M1 seam: trace bowl ∩ {z=c} and return the single CLOSED WLine.
// Requires CYBERCAD_HAS_NUMSCI. Returns an empty WLine on failure.
inline ssi::WLine closedSeamWLine() {
  const ssi::SurfaceAdapter A = fx::bowlAdapter();
  const ssi::SurfaceAdapter B = cutterAdapter();
  const ssi::TraceSet tr = ssi::trace_intersection(A, B);
  // Prefer a Closed loop; fall back to the longest branch.
  const ssi::WLine* best = nullptr;
  for (const ssi::WLine& w : tr.lines) {
    if (w.points.size() < 3) continue;
    if (w.isClosed()) return w;
    if (!best || w.points.size() > best->points.size()) best = &w;
  }
  return best ? *best : ssi::WLine{};
}

}  // namespace smooth_trim_split_fixture

#endif  // CYBERCAD_TESTS_NATIVE_SMOOTH_TRIM_SPLIT_FIXTURE_H
