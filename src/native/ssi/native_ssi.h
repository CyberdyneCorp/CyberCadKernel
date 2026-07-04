// SPDX-License-Identifier: Apache-2.0
//
// native_ssi.h — public aggregate header for the native Surface-Surface Intersection
// module, Stage S1 (ANALYTIC SSI — closed-form elementary-surface pairs).
//
// This is the clean-room, OCCT-FREE analytic SSI layer for the native kernel
// (openspec/SSI-ROADMAP.md, capability #5 stage S1). It computes CLOSED-FORM
// intersection curves — native Line / Circle / Ellipse / Parabola / Hyperbola — that
// PROVABLY lie on BOTH input surfaces (the S1 correctness invariant), for the conic
// family of elementary-surface pairs. Anything outside that family returns
// NotAnalytic and MUST be deferred to S2/S3 marching or OCCT — S1 never fakes a
// curve.
//
// SSI is an INTERNAL kernel capability: there is NO cc_* facade entry point and NO
// ABI change. It is verified at the SSI-function level (native curves vs OCCT
// IntAna_QuadQuadGeo / GeomAPI_IntSS on the simulator; sampled-points-on-both-
// surfaces on the host), exactly like native-math / topology parity.
//
// Modules:
//   * curve.h         — IntersectionCurve (native conic + evaluator) and the
//                       multi-branch IntersectionResult { Ok | NoIntersection |
//                       Coincident | NotAnalytic }.
//   * tolerance.h     — shared angular/linear tolerances + geometric predicates.
//   * plane_conics.h  — plane∩{plane,sphere,cylinder,cone} closed-form conics.
//   * plane_torus.h   — plane∩torus for the ⟂-axis and axis-containing orientations
//                       (oblique = planar quartic, deferred per S1 scope).
//   * quadric_pairs.h — sphere∩sphere; coaxial sphere∩cylinder, sphere∩cone,
//                       cylinder∩cone; coaxial/parallel cylinder∩cylinder.
//   * dispatch.h      — Surface variant + order-independent intersect_surfaces(A,B).
//
// SUPPORTED PAIRS (S1 closed-form):
//   plane∩plane, plane∩sphere, plane∩cylinder, plane∩cone, plane∩torus (⟂/axis),
//   sphere∩sphere, coaxial sphere∩cylinder, coaxial sphere∩cone,
//   coaxial|parallel cylinder∩cylinder, coaxial cylinder∩cone.
//
// DEFERRED (NotAnalytic → S2/S3 / OCCT, never faked):
//   skew cylinder∩cylinder (quartic), general cone∩cone, torus∩curved, sphere∩torus,
//   cylinder∩torus, oblique plane∩torus, ANY NURBS/freeform, near-tangent/coincident
//   freeform.
//
// SUBSTRATE: every S1 handler is pure closed-form geometry — the reductions used
// (plane-plane 2×2 solve, sphere-cone/sphere-cone-coaxial QUADRATIC in v) stay at
// degree ≤ 2 and are solved in-line, so NO NumPP/SciPP polynomial solver is
// required for the pairs that ship here. (The only case that would need the
// substrate's polynomial roots — the general oblique plane∩torus QUARTIC — is
// deferred, so the module has no build-time dependency on CYBERCAD_HAS_NUMSCI.)
//
// clang++ -std=c++20. fp64, deterministic. Uses src/native/math only.
//
#ifndef CYBERCAD_NATIVE_SSI_H
#define CYBERCAD_NATIVE_SSI_H

#include "native/ssi/curve.h"
#include "native/ssi/tolerance.h"
#include "native/ssi/plane_conics.h"
#include "native/ssi/plane_torus.h"
#include "native/ssi/quadric_pairs.h"
#include "native/ssi/dispatch.h"

/// The entire native SSI (Stage S1) API lives in this namespace.
namespace cybercad::native::ssi {}

#endif  // CYBERCAD_NATIVE_SSI_H
