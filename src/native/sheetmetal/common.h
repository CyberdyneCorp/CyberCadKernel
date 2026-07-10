// SPDX-License-Identifier: Apache-2.0
//
// common.h — shared POD types, the measured-decline enum, tolerances and the
// self-verify primitive for the native sheet-metal library (MOAT M-SM).
//
// OCCT-FREE. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SHEETMETAL_COMMON_H
#define CYBERCAD_NATIVE_SHEETMETAL_COMMON_H

#include "native/directmodel/replace_face.h"  // dm::rfdetail::eulerChar (mesh χ)
#include "native/math/native_math.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdio>

namespace cybercad::native::sheetmetal {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace dm   = cybercad::native::directmodel;

inline constexpr double kPi = 3.14159265358979323846;

// Local tolerances (mm-scale sheet-metal features; looser than the raw fp floor so
// a hand-authored profile is judged degenerate sensibly, tighter than a widened
// fudge — never relaxed to force a marginal case through).
inline constexpr double kTol = 1e-7;       // coincident-point / zero-length
inline constexpr double kMinThick = 1e-6;  // minimum sheet thickness
inline constexpr double kMinRadius = 0.0;  // a sharp (0-radius) bend is allowed
inline constexpr double kAngleFloor = 1e-6;  // radians; below → a no-op fold

// Measured reason a native sheet-metal op declined. The engine reports it; because
// OCCT has no sheet-metal module there is NO forward — a decline is a clean error.
enum class SheetMetalDecline {
  Ok = 0,
  BadProfile,          // base profile null / < 3 pts / zero-area
  BadThickness,        // thickness ≤ 0
  BadParam,            // height < 0 / bendRadius < 0 / angle outside (0,π)
  EdgeNotFound,        // edgeId out of range on the base
  EdgeNotStraight,     // the picked bend line is not a straight (Line) edge
  NotSingleBendPart,   // unfold: the part is not a recognised base+one-bend part
  SelfCollision,       // the fold would self-intersect (measured by the audit)
  VerifyFailed,        // the composite failed watertight / χ=2 / oriented / volume
};

// ── The one self-verify gate (mirrors draft_faces / verifyResolve) ────────────
// A built sheet-metal solid must mesh WATERTIGHT (closed 2-manifold), CONSISTENTLY
// ORIENTED, be a single genus-0 lump (χ=2), and enclose a POSITIVE volume within a
// relative band of the closed-form `expectedVol`. Any miss → decline (never a wrong
// solid, never a widened tolerance). Returns true iff every gate passes.
inline bool verifySolid(const topo::Shape& s, double expectedVol, double defl = 0.005) {
  if (s.isNull()) return false;
  tess::MeshParams mp;
  mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
#ifdef CYBERCAD_SM_DEBUG
  std::fprintf(stderr, "[sm verify] wt=%d ori=%d chi=%ld vol=%.5f exp=%.5f\n",
               (int)tess::isWatertight(m), (int)tess::isConsistentlyOriented(m),
               dm::rfdetail::eulerChar(m), std::fabs(tess::enclosedVolume(m)), expectedVol);
#endif
  if (!tess::isWatertight(m)) return false;              // closed 2-manifold
  if (!tess::isConsistentlyOriented(m)) return false;    // outward-consistent
  if (dm::rfdetail::eulerChar(m) != 2) return false;     // single genus-0 lump
  const double vol = std::fabs(tess::enclosedVolume(m));
  if (!(vol > 0.0)) return false;                        // positive volume
  // A curved (cylindrical) bend meshes to `defl`, so its volume converges from
  // below; the band is set by the deflection, NOT widened arbitrarily. 1.5% is a
  // conservative bound for a bend meshed at defl=0.005 on mm-scale radii; a flat
  // part (no curved face) lands well inside it.
  const double band = std::max(0.015 * expectedVol, 1e-6);
  return std::fabs(vol - expectedVol) <= band;
}

// ── Fold metadata carried alongside a folded native body ──────────────────────
// A base+edge-flange part is a closed-form parametric object: the engine records
// these recovered parameters on the produced native body so cc_sheet_unfold(body,
// kFactor) can develop it EXACTLY (no fragile mesh reverse-engineering). Additive,
// process-internal — it does not cross the cc_* ABI. `valid` is false for a body
// that is not a recognised single-bend fold (unfold then honest-declines).
struct FoldRecord {
  bool valid = false;
  double baseRunL = 0.0;   // base run in the flanged direction
  double width = 0.0;      // extent along the bend line
  double thickness = 0.0;  // sheet gauge
  double bendRadius = 0.0; // inner bend radius
  double angleRad = 0.0;   // bend angle
  double flangeHeight = 0.0;
};

// Signed area (shoelace) of a raw (x,y) point loop; |area| is the profile area.
inline double signedArea(const double* xy, int n) noexcept {
  double a2 = 0.0;
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    a2 += xy[i * 2] * xy[j * 2 + 1] - xy[j * 2] * xy[i * 2 + 1];
  }
  return 0.5 * a2;
}

}  // namespace cybercad::native::sheetmetal

#endif  // CYBERCAD_NATIVE_SHEETMETAL_COMMON_H
