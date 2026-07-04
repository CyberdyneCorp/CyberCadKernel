// SPDX-License-Identifier: Apache-2.0
//
// native_numerics.h — public aggregate header for the native numeric module
// (Phase-4 #2 numeric-foundations).
//
// This is the OCCT-FREE numeric substrate + closest-point layer for the native
// kernel rewrite. It adopts NumPP (/Users/leonardoaraujo/work/NumPP) and the
// SciPP optimize/linalg subset (/Users/leonardoaraujo/work/SciPP) as the
// point-solver / dense-linalg engine — referenced by absolute path exactly like
// OCCT, NOT vendored — and exposes:
//
//   * numerics.h       — the thin, substrate-free facade: scalar roots
//                        (brentq / newton), nonlinear system solve (fsolve),
//                        minimize (BFGS) / nonlinear least_squares (LM), dense
//                        solve / lstsq, and the evaluator-based closest-point.
//   * closest_point.h  — TYPED native closest-point / projection (the OCCT
//                        `Extrema` on-ramp): project a 3D point onto any native
//                        Bézier / B-spline / NURBS curve & surface + elementary
//                        plane / cylinder / cone / sphere / torus, with
//                        multi-start grid seeding, endpoint / boundary reporting,
//                        and the full set of local minima.
//
// GUARD — the whole module compiles only when CYBERCAD_HAS_NUMSCI is defined
// (CMake option). Nothing OUTSIDE src/native/numerics includes these headers, so
// the rest of src/native builds and tests without NumPP / SciPP linked. The only
// TU that touches the substrate is numerics.cpp; the closest-point layer is
// ordinary header code that calls the facade's `minimize`.
//
// See docs/EVAL-numpp-scipp.md (verdict GO-WITH-HARDENING) and
// openspec/NATIVE-REWRITE.md #2. SSI / curved booleans are NOT here — they are #5.
//
// clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_NUMERICS_H_AGG
#define CYBERCAD_NATIVE_NUMERICS_H_AGG

#include "native/numerics/numerics.h"
#include "native/numerics/closest_point.h"

/// The entire native numerics API lives in this namespace.
namespace cybercad::native::numerics {}

#endif  // CYBERCAD_NATIVE_NUMERICS_H_AGG
