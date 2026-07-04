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
//   * dispatch.h      — Surface variant + order-independent intersect_surfaces(A,B) +
//                       classify_degeneracy(A,B) (S4-a analytic coincidence classifier).
//
// STAGE S4-a — COINCIDENT / OVERLAPPING-SURFACE DETECTION (typed region):
//   * coincidence.h   — CoincidentRegion { None | FullSurfaceSame | OverlapSubRegion
//                       {ParamBox regionA, regionB} | Undecided } — the typed shared-locus
//                       descriptor downstream booleans/queries consume instead of a flag.
//   * same_surface.h  — closed-form "same locus?" predicates per elementary family (same
//                       plane / coaxial-equal cyl / same cone / same sphere / same torus),
//                       backing classify_degeneracy's FullSurfaceSame verdict. The shipped
//                       IntersectionStatus::Coincident results are UNCHANGED (additive).
//   * seeding.cpp     — the SEEDED coincident-patch detector (under CYBERCAD_HAS_NUMSCI):
//                       a candidate cluster whose interior grid coincides (point-on-both +
//                       aligned normals) is grown to its agreement boundary and returned as
//                       an OverlapSubRegion on SeedSet.coincidentRegions (seeds/march inside
//                       SUPPRESSED); an undelimitable overlap (runs to a domain edge / fuzzy
//                       boundary) is Undecided → OCCT, NEVER a fabricated region.
//
// UNDECIDED → OCCT (honest). src/native never links OCCT; the S4-a detector returns
// Undecided/None on any non-robust decision and the ENGINE owns the OCCT fallback +
// self-verify. A correct "Undecided → OCCT" is first-class; a fabricated region is a bug.
//
// STAGE S4-b — TYPED TANGENT-CONTACT CLASSIFICATION:
//   * tangent_contact.h  — TangentContact { TransversalOnly | TangentPoint {point} |
//                        TangentCurve {curve} | NearTangentTransversal | Undecided } — the
//                        typed degeneracy descriptor replacing the blunt deferredTangent flag.
//                        Built on the EXISTING CurveKind::Point / IntersectionCurve seams.
//   * tangent_analytic.h — closed-form analytic tangent classifiers (sphere∩sphere at
//                        d=R₁+R₂ → TangentPoint; coaxial sphere∩cyl equator, plane∩cyl
//                        ruling → TangentCurve; …). Exact and decidable — never
//                        NearTangentTransversal / Undecided. Wired via classify_tangency(A,B)
//                        in dispatch.h (sibling to intersect_surfaces / classify_degeneracy).
//   * tangent_seeded.h   — the seeded classifier (under CYBERCAD_HAS_NUMSCI): at a near-tangent
//                        refined solution it types the contact by the RELATIVE SECOND
//                        FUNDAMENTAL FORM H = II_A − II_B in the shared tangent plane
//                        (definite → TangentPoint, rank-1 → TangentCurve, indefinite →
//                        NearTangentTransversal, within noise → Undecided). Populates
//                        SeedSet.tangentContacts (deferredTangent kept as a compatibility
//                        count) and types the S3 marcher's NearTangent stop (WLine.stopReason).
//                        NearTangentTransversal is handed on to S4-c → OCCT, NEVER traced.
//
// STAGE S2 — SUBDIVISION SEEDING (freeform + non-closed-form quadric pairs):
//   * seed.h          — Seed { (u1,v1),(u2,v2), point, onSurfResidual, branchId } +
//                       SeedSet { seeds; candidateRegions; refinedAccepted;
//                       deferredTangent } + RecallReport (sim gate).
//   * patch_bounds.h  — ParamBox / Aabb, control-net-convex-hull bound (freeform) +
//                       sampled-with-Lipschitz-margin bound (elementary/torus) +
//                       disjoint-AABB prune test; SurfaceAdapter (one subdivision
//                       path for every surface kind).
//   * seeding.h/.cpp  — seed_intersection(A,B): recursive param-box subdivision +
//                       AABB-overlap prune → least_squares refine (native-numerics,
//                       drives A.point−B.point→0, clamped) → 3D-proximity dedup →
//                       ≈1 seed per transversal branch. Refine (hence a useful
//                       seeder) is under CYBERCAD_HAS_NUMSCI.
//
// STAGE S3 — MARCHING-LINE TRACER (WLine):
//   * marching.h/.cpp — march_branch / trace_intersection: from each S2 seed, a
//                       PREDICTOR-CORRECTOR walk (tangent = normalize(nA×nB); step
//                       P+h·t; re-project onto BOTH surfaces via least_squares;
//                       deflection-adaptive h; terminate on loop-closure / boundary /
//                       near-tangent) yielding one WLine per branch + a fitted
//                       B-spline (lstsq). TRANSVERSAL only — a march into a
//                       near-tangent region is truncated and flagged
//                       NearTangentTruncated (S4 gap), never faked. Entry points are
//                       under CYBERCAD_HAS_NUMSCI (corrector + fit call the substrate).
//
// S2 IS TRANSVERSAL-ONLY (honest). Near-tangent / coincident / degenerate seeding
// ill-conditions the refine and is DEFERRED to S4 — counted in SeedSet.deferredTangent,
// never faked. `deferredTangent > 0` is a first-class "seen but not safely seeded"
// signal (S4 seam), like S1's NotAnalytic. Completeness is a MEASURED branch-recall
// figure (RecallReport) vs OCCT GeomAPI_IntSS, not a blind 100% claim: too-shallow
// subdivision can miss a small loop (the acknowledged failure mode; deeper maxDepth
// recovers it). The SeedSet is the INPUT CONTRACT for S3 marching (one WLine per seed).
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
#include "native/ssi/coincidence.h"
#include "native/ssi/same_surface.h"
#include "native/ssi/tangent_contact.h"
#include "native/ssi/tangent_analytic.h"
#include "native/ssi/tangent_seeded.h"
#include "native/ssi/dispatch.h"

// Stage S2 — subdivision seeding. seed.h / patch_bounds.h are OCCT-free and
// substrate-free (always available); seeding.h's seed_intersection entry point is
// compiled only under CYBERCAD_HAS_NUMSCI (the least_squares refine).
#include "native/ssi/seed.h"
#include "native/ssi/patch_bounds.h"
#include "native/ssi/seeding.h"

// Stage S3 — marching-line tracer. Result types (WLine / TraceResult) are OCCT-free
// and substrate-free; the marching/fit ENTRY POINTS (march_branch /
// trace_intersection) are compiled only under CYBERCAD_HAS_NUMSCI (the corrector +
// B-spline fit call native-numerics).
#include "native/ssi/marching.h"

/// The entire native SSI (Stage S1) API lives in this namespace.
namespace cybercad::native::ssi {}

#endif  // CYBERCAD_NATIVE_SSI_H
