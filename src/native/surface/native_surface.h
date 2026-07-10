// SPDX-License-Identifier: Apache-2.0
//
// native_surface.h — public aggregate header for the native SURFACE module: the
// bounded N-sided fill / surface patch (MOAT surfacing slice).
//
// SCOPE BOUND (non-negotiable): the boundary loop is 3–6 analytic/polyline sides
// (straight segments + circular arcs); the fill patch is a Coons/Gregory transfinite
// interpolant EVALUATED to a TESSELLATED triangle mesh — NOT a general trimmed-NURBS
// surface. No general NURBS surface representation or evaluator is added. Cases needing
// true NURBS surfacing are honest-declined → OCCT BRepFill_Filling.
//
//   * ngon_fill.h  — surface::fillNGon(boundary, opts, &decline) → tessellated patch mesh.
//   * fill_solid.h — surface::fillHoleSolid(openShell, opts) → welded watertight solid.
//
// OCCT-FREE. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_SURFACE_H
#define CYBERCAD_NATIVE_SURFACE_H

#include "native/surface/fill_solid.h"
#include "native/surface/ngon_fill.h"

namespace cybercad::native::surface {}

#endif  // CYBERCAD_NATIVE_SURFACE_H
