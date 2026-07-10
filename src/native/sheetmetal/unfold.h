// SPDX-License-Identifier: Apache-2.0
//
// unfold.h — the FLAT-PATTERN (MOAT M-SM). Unroll the single cylindrical bend of a
// base+edge-flange part about its NEUTRAL FIBRE into the developed flat blank — the
// planar sheet a laser/press starts from. This is the manufacturing payoff of the
// sheet-metal slice.
//
// ── BEND ALLOWANCE (the k-factor formula) ─────────────────────────────────────
// When a strip of thickness t is bent through angle θ to inner radius r, the fibre
// that neither stretches nor compresses (the NEUTRAL fibre) sits at radius
// r + k·t, where k∈[0,1] is the k-factor (material/process constant, typ. 0.3–0.5).
// Its developed (unrolled flat) length is the BEND ALLOWANCE
//     BA = θ · (r + k·t).
// So the flat blank's run in the flanged direction is
//     Ldev = L (base run) + BA (developed bend) + height (flange wall),
// and the blank is that run × the width W × the thickness t.
//
// ── ROUND-TRIP INVARIANT (Gate a arbiter) ────────────────────────────────────
// The developed FOOTPRINT AREA of the flat blank is
//     A_dev = Ldev · W = (L + BA + height) · W
//           = baseArea + BA·W + flangeArea.
// Folding then unfolding is AREA-INVARIANT: the same closed form gives A_dev from
// either the folded part (via its recovered L, W, t, r, θ, height) or directly from
// the blank. The test asserts A_dev is invariant under fold→unfold.
//
// The unfold is emitted as a base flange (construct::build_prism) of the developed
// rectangle, so the result is itself a valid, watertight, self-verified sheet solid.
//
// ── FIRST-SLICE SCOPE ─────────────────────────────────────────────────────────
// Unfold serves the recognised single-bend part (base + ONE edge flange + ONE
// cylindrical bend). A part that is not that recognised topology → NotSingleBendPart
// (honest decline; no OCCT sheet-metal oracle to forward to).
//
// OCCT-FREE. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SHEETMETAL_UNFOLD_H
#define CYBERCAD_NATIVE_SHEETMETAL_UNFOLD_H

#include "native/construct/native_construct.h"
#include "native/sheetmetal/common.h"

#include <cmath>

namespace cybercad::native::sheetmetal {

namespace cst2 = cybercad::native::construct;

// The developed dimensions of a single-bend part. `devLength` is the total flat run
// in the flanged direction (base + bend allowance + flange wall); `area` is the
// developed footprint area = devLength · width.
struct FlatPattern {
  double devLength = 0.0;
  double width = 0.0;
  double thickness = 0.0;
  double bendAllowance = 0.0;
  double area = 0.0;
};

// Compute the flat pattern of a single-bend part from its recovered parameters and
// the k-factor. Pure closed form — the arbiter both the fold and the unfold agree on.
inline FlatPattern flatPattern(double baseRunL, double width, double thickness, double bendRadius,
                               double angleRad, double flangeHeight, double kFactor) {
  FlatPattern fp;
  fp.width = width;
  fp.thickness = thickness;
  fp.bendAllowance = angleRad * (bendRadius + kFactor * thickness);
  fp.devLength = baseRunL + fp.bendAllowance + flangeHeight;
  fp.area = fp.devLength * width;
  return fp;
}

// Unfold: given the SAME parameters used to build the edge flange plus the k-factor,
// emit the developed flat blank (a planar sheet solid [0,Ldev]×[0,W] × t) and return
// it. `out` (optional) receives the FlatPattern (for the area-invariant gate).
// Returns a NULL Shape with a measured decline on bad input.
inline topo::Shape unfold(double baseRunL, double width, double thickness, double bendRadius,
                          double angleRad, double flangeHeight, double kFactor,
                          FlatPattern* out = nullptr, SheetMetalDecline* why = nullptr) {
  auto fail = [&](SheetMetalDecline d) -> topo::Shape {
    if (why) *why = d;
    return {};
  };
  if (!(baseRunL > kTol) || !(width > kTol) || !(thickness > kMinThick))
    return fail(SheetMetalDecline::NotSingleBendPart);
  if (!(bendRadius >= kMinRadius) || !(angleRad > kAngleFloor) || !(angleRad < kPi) ||
      !(flangeHeight >= 0.0) || !(kFactor >= 0.0) || !(kFactor <= 1.0))
    return fail(SheetMetalDecline::BadParam);

  const FlatPattern fp =
      flatPattern(baseRunL, width, thickness, bendRadius, angleRad, flangeHeight, kFactor);
  if (out) *out = fp;

  // Developed rectangle footprint on z=0, extruded by thickness.
  const double rect[8] = {0.0, 0.0, fp.devLength, 0.0, fp.devLength, width, 0.0, width};
  const topo::Shape blank = cst2::build_prism(rect, 4, thickness);
  if (blank.isNull()) return fail(SheetMetalDecline::VerifyFailed);

  // Self-verify at the closed-form blank volume = area · thickness (planar ⇒ exact).
  if (!verifySolid(blank, fp.area * thickness)) return fail(SheetMetalDecline::VerifyFailed);

  if (why) *why = SheetMetalDecline::Ok;
  return blank;
}

// Unfold a recognised folded body from its recorded FoldRecord (the engine path:
// cc_sheet_unfold(body, kFactor)). Declines if the body carries no valid fold record.
inline topo::Shape unfold(const FoldRecord& fr, double kFactor, FlatPattern* out = nullptr,
                          SheetMetalDecline* why = nullptr) {
  if (!fr.valid) {
    if (why) *why = SheetMetalDecline::NotSingleBendPart;
    return {};
  }
  return unfold(fr.baseRunL, fr.width, fr.thickness, fr.bendRadius, fr.angleRad, fr.flangeHeight,
                kFactor, out, why);
}

}  // namespace cybercad::native::sheetmetal

#endif  // CYBERCAD_NATIVE_SHEETMETAL_UNFOLD_H
