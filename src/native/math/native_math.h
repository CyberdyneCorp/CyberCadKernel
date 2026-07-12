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
//   * bspline_gordon.h—NURBS Gordon / network surface (NURBS-SCOPE Layer 6): a surface
//                    through a NETWORK of u- and v-curves via the boolean sum
//                    G = S_u ⊕ S_v ⊖ T (§10.5); interpolates every network curve.
//                    (numsci-gated; composes skinSurface + surface interpolation.)
//   * bspline_coons.h—NURBS boundary-filled Coons patch (NURBS-SCOPE Layer 6): fill a
//                    CLOSED four-sided boundary with the bilinearly-blended boolean sum
//                    Coons = L_u ⊕ L_v ⊖ B; interpolates all four boundary curves.
//                    (numsci-gated for Layer-6 family uniformity; exact Layer-1 ops only.)
//   * bspline_nsided.h—NURBS N-SIDED boundary-filled surface (NURBS-SCOPE Layer 6): fill a
//                    CLOSED N-gon (N ≠ 4) by MIDPOINT subdivision into N Coons sub-patches
//                    meeting at the centroid; the union interpolates all N boundary curves.
//                    (numsci-gated; composes splitCurve + bspline_coons.) C0 at the spokes.
//   * bspline_nsided_g1.h—G1 (tangent-plane continuous) N-SIDED fill (NURBS-SCOPE Layer 6):
//                    the Gregory/Chiyokura-Kimura upgrade of the C0 N-sided fill — N bicubic
//                    "pie slices" meeting the boundary + each other G1 (unit normal continuous
//                    across every spoke, machine-exact for smooth/planar boundaries) via shared
//                    spoke columns + a shared cross-spoke rib; declines a creased 3-D corner
//                    honestly. (numsci-gated; additive to the C0 API.)
//   * bspline_nsided_g2.h—G2 (curvature-continuous) N-SIDED fill (NURBS-SCOPE Layer 6):
//                    the G2 Gregory/Chiyokura-Kimura upgrade of the G1 N-sided fill — N
//                    QUINTIC-in-v "pie slices" meeting the boundary + each other G2 (unit
//                    normal AND normal curvature continuous across every spoke) via shared
//                    seam-adjacent u-columns (position + 1st-inward rib + 2nd-inward rib);
//                    declines a creased corner / corner-incompatible curvature honestly.
//                    (numsci-gated; additive to the C0 + G1 APIs.)
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
#include "bspline_gordon.h"
#include "bspline_coons.h"
#include "bspline_nsided.h"
#include "bspline_nsided_g1.h"
#include "bspline_nsided_g2.h"
#include "bspline_offset.h"
#include "bezier.h"
#include "elementary.h"
#include "torus.h"

/// The entire native math API lives in this namespace.
namespace cybercad::native::math {}

#endif  // CYBERCAD_NATIVE_MATH_H
