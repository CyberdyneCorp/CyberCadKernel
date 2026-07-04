// SPDX-License-Identifier: Apache-2.0
//
// tangent_seeded.h — the SEEDED (differential-geometry) tangent-contact classifier
// (SSI Stage S4-b). Declaration only; the definition lives in seeding.cpp and is
// compiled under CYBERCAD_HAS_NUMSCI (it projects onto both surfaces via native-numerics
// closest_point_on_surface). The declaration stays visible for callers (seeding's refine
// loop and the S3 marcher's stop-reason typing) exactly like seed_intersection / march_branch.
//
// At a near-tangent solution (‖n_A × n_B‖ < tangentSinTol) on two general surfaces, this
// classifies the contact by the RELATIVE SECOND FUNDAMENTAL FORM H = II_A − II_B in the
// shared tangent plane: sign-definite → TangentPoint, rank-1 → TangentCurve, indefinite →
// NearTangentTransversal (S4-c gap, handed on, never traced), within the curvature-noise
// band → Undecided (→ OCCT). See seeding.cpp for the derivation. Never fabricates a verdict.
//
// OCCT-FREE. Uses src/native/ssi (SurfaceAdapter, TangentContact) only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_TANGENT_SEEDED_H
#define CYBERCAD_NATIVE_SSI_TANGENT_SEEDED_H

#include "native/math/vec.h"
#include "native/ssi/patch_bounds.h"   // SurfaceAdapter
#include "native/ssi/tangent_contact.h"

namespace cybercad::native::ssi {

/// Classify the near-tangent contact of surfaces A, B at a refined solution given by the
/// params (u1,v1) on A / (u2,v2) on B, its base point `P`, the two surface normals there
/// (`nA`,`nB`, ~parallel), the measured crossing sine ‖n_A×n_B‖, and the model `scale`
/// (sets the finite-difference step + curvature-noise floor). Returns the typed
/// TangentContact. Requires CYBERCAD_HAS_NUMSCI to LINK (projection uses the substrate).
TangentContact classify_tangent_contact_seeded(
    const SurfaceAdapter& A, const SurfaceAdapter& B,
    double u1, double v1, double u2, double v2,
    const math::Point3& P, const math::Dir3& nA, const math::Dir3& nB,
    double crossingSine, double scale);

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_TANGENT_SEEDED_H
