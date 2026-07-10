// SPDX-License-Identifier: Apache-2.0
//
// base_flange.h — the BASE FLANGE: a flat sheet solid = the closed 2D profile
// extruded by `thickness` along +Z. Every sheet-metal part starts as a base
// flange. This is a thin, honest wrapper over the landed construct::build_prism (a
// base flange IS a prism): the sheet lies in z∈[0,thickness], its footprint is the
// profile on z=0, and its constant thickness is the sheet gauge every later bend
// preserves.
//
// Closed-form volume (Gate a arbiter): |profileArea| · thickness.
//
// OCCT-FREE. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SHEETMETAL_BASE_FLANGE_H
#define CYBERCAD_NATIVE_SHEETMETAL_BASE_FLANGE_H

#include "native/construct/native_construct.h"
#include "native/sheetmetal/common.h"

#include <cmath>

namespace cybercad::native::sheetmetal {

namespace cst = cybercad::native::construct;

// Build the base flange: extrude the closed polygon `profileXY` (x,y pairs on z=0,
// `pointCount` points) by `thickness` along +Z. Returns the flat sheet SOLID, or a
// NULL Shape with a measured decline (never a wrong solid).
inline topo::Shape baseFlange(const double* profileXY, int pointCount, double thickness,
                              SheetMetalDecline* why = nullptr) {
  auto fail = [&](SheetMetalDecline d) -> topo::Shape {
    if (why) *why = d;
    return {};
  };
  if (profileXY == nullptr || pointCount < 3) return fail(SheetMetalDecline::BadProfile);
  if (!(thickness > kMinThick)) return fail(SheetMetalDecline::BadThickness);
  if (std::fabs(signedArea(profileXY, pointCount)) < kTol) return fail(SheetMetalDecline::BadProfile);

  const topo::Shape solid = cst::build_prism(profileXY, pointCount, thickness);
  if (solid.isNull()) return fail(SheetMetalDecline::BadProfile);

  // Self-verify at the closed-form volume (planar ⇒ mesh is exact).
  const double expected = std::fabs(signedArea(profileXY, pointCount)) * thickness;
  if (!verifySolid(solid, expected)) return fail(SheetMetalDecline::VerifyFailed);

  if (why) *why = SheetMetalDecline::Ok;
  return solid;
}

}  // namespace cybercad::native::sheetmetal

#endif  // CYBERCAD_NATIVE_SHEETMETAL_BASE_FLANGE_H
