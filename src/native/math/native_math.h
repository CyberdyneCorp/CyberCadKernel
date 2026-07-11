// SPDX-License-Identifier: Apache-2.0
//
// native_math.h — public aggregate header for the native math/geometry library.
//
// This is the clean-room, OCCT-FREE math foundation for the native kernel
// rewrite (Phase 4, capability #1 `native-math`). It provides:
//   * vec.h        — Vec3, Point3, Dir3 (unit) and vector algebra.
//   * transform.h  — Mat3 and affine Transform (compose / inverse / apply).
//   * bspline.h    — B-spline & rational NURBS curves and surfaces (points,
//                    derivatives, surface normal); FindSpan / BasisFuns / de Boor.
//   * bspline_ops.h— exact-NURBS geometry kernel (NURBS-SCOPE Layer 1): knot
//                    insert/refine/remove, degree elevate/reduce, split, Bézier
//                    decompose, reparam — curves & tensor surfaces, rational-aware.
//   * bspline_fit.h— NURBS fitting / approximation (NURBS-SCOPE Layer 7): points →
//                    B-spline curve/surface interpolation + least-squares approx.
//                    (numsci-gated; the solve-bearing routines need CYBERCAD_HAS_NUMSCI.)
//   * bspline_skin.h—NURBS skinning / lofting (NURBS-SCOPE Layer 6): section curves →
//                    tensor-product surface containing every section (compat + A10.3).
//                    (numsci-gated; the V-interpolation solve needs CYBERCAD_HAS_NUMSCI.)
//   * bspline_sweep.h—NURBS swept surfaces (NURBS-SCOPE Layer 6): a section swept along a
//                    trajectory — translational (exact tensor product) + general
//                    (transform-then-skin along a rotation-minimizing frame, §10.4).
//                    (numsci-gated; the general sweep composes skinSurface.)
//   * bspline_offset.h—NURBS surface offset (NURBS-SCOPE Layer 5): S + d·N sampled on
//                    an adaptive grid + fitted to a NURBS surface (Layer-7), with a
//                    curvature-radius self-intersection guard. (numsci-gated; the fit +
//                    projection metric need CYBERCAD_HAS_NUMSCI.)
//
// NURBS solid thicken / shell (NURBS-SCOPE Layer 5, `bspline_thicken.h`) — an open
// surface + its offset panel + ruled side walls sewn into a CLOSED watertight triangle
// shell — is a HIGHER-level module: it depends on `native/tessellate/mesh.h` (the
// closed-shell carrier), which itself includes THIS aggregate, so `bspline_thicken.h` is
// deliberately NOT re-exported here to avoid a circular include. Consumers include it
// directly, exactly as consumers of the tessellator do.
//   * bezier.h     — Bézier curves & surfaces via de Casteljau (+ rational).
//   * elementary.h — analytic plane / cylinder / cone / sphere.
//   * torus.h      — analytic torus (surface of revolution of an off-axis circle).
//
// Conventions mirror OCCT gp_*/BSplCLib/BSplSLib/ElSLib so results can be
// verified against the OCCT oracle on the simulator; but no OCCT header is
// included here or anywhere under src/native/math/. Everything compiles with:
//   clang++ -std=c++20
//
// All numerics are fp64 and deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_H
#define CYBERCAD_NATIVE_MATH_H

#include "vec.h"
#include "transform.h"
#include "bspline.h"
#include "bspline_ops.h"
#include "bspline_fit.h"
#include "bspline_skin.h"
#include "bspline_sweep.h"
#include "bspline_offset.h"
#include "bezier.h"
#include "elementary.h"
#include "torus.h"

/// The entire native math API lives in this namespace.
namespace cybercad::native::math {}

#endif  // CYBERCAD_NATIVE_MATH_H
