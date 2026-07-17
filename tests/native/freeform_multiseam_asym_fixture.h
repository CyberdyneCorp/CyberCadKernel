// SPDX-License-Identifier: Apache-2.0
//
// freeform_multiseam_asym_fixture.h — the ASYMMETRIC (curvature-MISMATCHED) multi-seam
// freeform↔freeform pose, the honest frontier past the z-mirror-symmetric fixture in
// freeform_freeform_multiseam_fixture.h.
//
// ── WHY THIS FIXTURE (measurement-first) ─────────────────────────────────────────
// The symmetric fixture (`freeform_freeform_multiseam_fixture.h`) is the FRIENDLIEST
// multi-seam pose: B is A mirrored about z = H/2, so the two curved annuli sharing every
// seam have IDENTICAL curvature magnitude, the M0 mesher's curvature-driven boundary
// refinement MATCHES on both sides, and the annulus↔annulus sew welds watertight. That
// fixture's own header names the untested harder case (lines 18-21): "A degree-MISMATCHED
// pair — e.g. a degree-4 valley ∩ a degree-2 dome — would leave a small T-junction
// residual at the higher-curvature seam, which the verb HONEST-DECLINES, never leaks."
// This fixture MEASURES exactly that: a curvature-mismatched (but same-degree) pair, so
// the two walls meet the shared seam at DIFFERENT slopes/curvatures — the general
// (non-mirror) multi-seam assembly the BOOL-MULTISEAM track targets.
//
// ── The two operands (both degree-4 freeform cups, DIFFERENT amplitudes) ──────────
//   * A = valley cup, z_A = a·(r² − ρ₀²)²  (amplitude a = 4).
//   * B = dome  cup, z_B = H − b·(r² − ρ₀²)²  (amplitude b = 6 ≠ a).
//   Because b ≠ a the two walls are NOT mirror images: at each shared seam the valley and
//   the dome have different curvature, so the M0 mesher refines the two annuli's near-seam
//   boundaries DIFFERENTLY — the curvature-parity assumption the symmetric weld relies on
//   is broken. Both walls are exact degree-4 tensor-Bézier (5×5 poles), z = amplitude·(a
//   fixed degree-4 polynomial), so the z-poles scale LINEARLY with the amplitude (verified
//   against the symmetric fixture's a=4 grid).
//
// ── The TWO shared closed curved seams ───────────────────────────────────────────
// z_A = z_B ⟹ (a+b)(r²−ρ₀²)² = H ⟹ r² = ρ₀² ± √(H/(a+b)) — TWO CONCENTRIC CIRCLES at
// r₁ ≈ 0.154, r₂ ≈ 0.365, both interior to the rim R = 0.45, both at height
// z* = a·H/(a+b) = 0.012 (NOT H/2 — the asymmetry). The SSI trace returns TWO closed loops.
//
// ── Closed-form volume oracles (no OCCT) ─────────────────────────────────────────
// V(A) = 0.029166, V(B) = 0.043749, V(A∩B) = 0.006883 (the annular lens between the seams,
// where the dome is above the valley), V(A−B) = V(A)−lens, V(A∪B) = V(A)+V(B)−lens.
//
#ifndef CYBERCAD_TESTS_NATIVE_FREEFORM_MULTISEAM_ASYM_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_FREEFORM_MULTISEAM_ASYM_FIXTURE_H

#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "freeform_freeform_multiseam_fixture.h"  // reuse buildCup / bezierSurface / rimUV

#include <cmath>
#include <vector>

namespace freeform_multiseam_asym_fixture {

namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;
namespace sym = freeform_freeform_multiseam_fixture;

inline constexpr double kAvalley = 4.0;   // A valley amplitude (== the symmetric fixture)
inline constexpr double kBdome = 6.0;     // B dome amplitude (≠ A ⇒ curvature mismatch)
inline constexpr double kRho0 = sym::kRho0;
inline constexpr double kH = sym::kH;     // dome/valley separation (lens max height)
inline constexpr double kR = sym::kR;     // rim radius
inline constexpr double kPi = sym::kPi;

inline double rho2() { return kRho0 * kRho0; }
inline double zA(double r) { const double s = r * r - rho2(); return kAvalley * s * s; }
inline double zBdome(double r) { const double s = r * r - rho2(); return kH - kBdome * s * s; }

// seam radii: r² = ρ₀² ± √(H/(a+b)).
inline double seamDisc() { return std::sqrt(kH / (kAvalley + kBdome)); }
inline double seamR1() { return std::sqrt(rho2() - seamDisc()); }  // ≈ 0.154
inline double seamR2() { return std::sqrt(rho2() + seamDisc()); }  // ≈ 0.365

inline double lidA() { return zA(kR); }        // A's flat top lid (its wall max)
inline double botB() { return zBdome(kR); }    // B's flat bottom lid (its wall min)

// ── closed-form volume oracles (no OCCT) ─────────────────────────────────────────
inline double volA() {
  const double R2 = kR * kR, p = rho2();
  const double intzA = kAvalley * (R2 * R2 * R2 / 3.0 - p * R2 * R2 + p * p * R2);
  return kPi * (lidA() * R2 - intzA);
}
inline double volB() {
  const double R2 = kR * kR, p = rho2();
  const double intzB = kH * R2 - kBdome * (R2 * R2 * R2 / 3.0 - p * R2 * R2 + p * p * R2);
  return kPi * (intzB - botB() * R2);
}
inline double volCommon() {
  const double p = rho2(), disc = seamDisc();
  const double s1 = p - disc, s2 = p + disc;
  const double ab = kAvalley + kBdome;
  auto antid = [&](double s) { return kH * s - ab * (s * s * s / 3.0 - p * s * s + p * p * s); };
  return kPi * (antid(s2) - antid(s1));
}
inline double volCut() { return volA() - volCommon(); }

// ── A's degree-4 VALLEY Bézier (5×5 poles): reuse the symmetric fixture's exact a=4 grid.
inline std::vector<fmath::Point3> valleyPoles() { return sym::valleyPoles(); }

// ── B's DOME Bézier: z = H − b·(r²−ρ₀²)². The z-poles of b·(r²−ρ₀²)² scale LINEARLY from
// the a=4 grid by (b/4); then z ↦ H − z (the dome mirror). ─────────────────────────
inline std::vector<fmath::Point3> domePoles() {
  std::vector<fmath::Point3> poles = sym::valleyPoles();  // these are the a=4 z-poles
  const double scale = kBdome / kAvalley;
  for (fmath::Point3& p : poles) p.z = kH - scale * p.z;
  return poles;
}

inline ssi::SurfaceAdapter valleyAdapter() { return ssi::makeBezierAdapter(valleyPoles(), 5, 5); }
inline ssi::SurfaceAdapter domeAdapter() { return ssi::makeBezierAdapter(domePoles(), 5, 5); }

// The real M1 seams: trace A's valley ∩ B's dome → the TWO closed WLines (circles r₁,r₂).
// Cached (the degree-4↔degree-4 trace is expensive) — a pure function of the fixed poles.
inline const std::vector<ssi::WLine>& closedSeams() {
  static const std::vector<ssi::WLine> cached = [] {
    const ssi::TraceSet tr = ssi::trace_intersection(valleyAdapter(), domeAdapter());
    std::vector<ssi::WLine> out;
    for (const ssi::WLine& w : tr.lines)
      if (w.points.size() >= 3 && w.isClosed()) out.push_back(w);
    return out;
  }();
  return cached;
}

// A = the valley cup (top lid at z=z_A(R)); B = the dome cup (bottom lid at z=z_B(R)).
inline topo::Shape buildA() { return sym::buildCup(valleyPoles(), lidA()); }
inline topo::Shape buildB() { return sym::buildCup(domePoles(), botB()); }

}  // namespace freeform_multiseam_asym_fixture

#endif  // CYBERCAD_TESTS_NATIVE_FREEFORM_MULTISEAM_ASYM_FIXTURE_H
