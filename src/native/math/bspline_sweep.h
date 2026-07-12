// SPDX-License-Identifier: Apache-2.0
//
// bspline_sweep.h — NURBS roadmap Layer 6: SWEPT surfaces (a section curve swept
// along a trajectory).
//
// Given a SECTION (profile) B-spline curve and a way to move it through space,
// construct a single tensor-product B-spline SURFACE that sweeps the section along
// the path. This is the core freeform-surfacing primitive behind pipes, extrusions,
// mouldings and swept solids (docs/NURBS-SCOPE.md §2/§4 Layer 6). It sits beside
// skinning: skinning interpolates a surface THROUGH given sections, sweeping GENERATES
// the sections by moving one profile along a spine and then skins them.
//
// It sits above the evaluators in bspline.{h,cpp} and COMPOSES lower layers:
//   * Layer 1 (bspline_ops.h) — the shared BsplineCurveData / BsplineSurfaceData types.
//   * Layer 6 (bspline_skin.h) — the general sweep transforms the section to K stations
//     along the trajectory and SKINS them (skinSurface) into one tensor-product surface.
//   * transform.h — the affine moving frame that positions/orients the section at each
//     station.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 10:
//   §10.4 Swept Surfaces. Two constructions are provided:
//
//   1. TRANSLATIONAL sweep (the exact closed-form case) — sweep the section along a
//      DIRECTION vector (a straight path). The result is the tensor product of the
//      section curve (U) with a degree-1 two-pole path (V): every iso-curve S(·, v) is
//      exactly the section translated by v·(the sweep vector). This is EXACT — no
//      fitting, no solve — and is the airtight base case. §10.4, the extrusion/tabulated-
//      cylinder special case (a straight trajectory, no orientation change).
//
//   2. GENERAL sweep along a TRAJECTORY curve — place the section at K stations sampled
//      along the trajectory (spine) B-spline, each transformed by a MOVING FRAME that
//      translates the section to the station point and rotates it from the section's
//      reference normal to the trajectory tangent, then SKIN the K transformed sections
//      via skinSurface. §10.4 Algorithm A10.1 in spirit (station placement + skin), but
//      using a ROTATION-MINIMIZING frame for the orientation instead of a Frenet frame,
//      so the swept profile does not spin about the spine where the trajectory's
//      curvature/torsion would twist a Frenet frame.
//
// FRAME CHOICE (documented, to avoid twist): the general sweep orients the section with a
// ROTATION-MINIMIZING FRAME (RMF) computed by the DOUBLE-REFLECTION method (Wang, Jüttler,
// Zheng & Liu, "Computation of Rotation Minimizing Frames", ACM TOG 2008). Starting from a
// reference frame at station 0 (the section's plane normal aligned to the trajectory's
// start tangent), each subsequent station's frame is propagated by two reflections so the
// frame rotates by the minimum amount consistent with following the tangent — it has NO
// torsion-driven spin about the spine (unlike a Frenet frame, which is undefined at
// inflections and spins with torsion). This is the standard anti-twist choice for pipe/
// tube sweeps. The section is assumed to lie in a plane with a well-defined normal; that
// normal is the reference the frame carries.
//
// SCOPE — NON-RATIONAL section/trajectory (`sweepTranslational` / `sweepAlongTrajectory`),
// PLUS the RATIONAL (weighted-section) case: `sweepRationalTranslational` is the EXACT rational
// tensor product of a rational section (U) with a degree-1 path (V) — the weights are constant
// in V so every iso-curve is the rational section translated, machine-exact (this is the oracle
// that turns a rational CIRCLE into an exact rational CYLINDER); `sweepRationalAlongTrajectory`
// places the rational section at K stations by RIGID transforms (which preserve the weights
// exactly) and RATIONAL-SKINS them via `skinRationalSurface`. A ROTATIONAL sweep (a profile
// revolved — a distinct exact-rational construction), exact GeomFill/BRepFill-class sweeps (an
// analytically exact swept surface rather than a skinned approximation of stations), and a
// VARIABLE section (a section that morphs along the spine) are documented residuals for later
// slices — this module never fakes them. See docs/NURBS-SCOPE.md Layer-6 row.
//
// GUARD — the general sweep composes skinSurface, which solves linear systems through the
// numsci facade, so the whole implementation TU is under CYBERCAD_HAS_NUMSCI (exactly like
// bspline_skin.cpp / bspline_fit.cpp). With the guard OFF the TU is inert and the functions
// are absent; the declarations remain visible for documentation. (The translational sweep
// is itself solve-free — pure tensor product — but is kept in the same guarded TU for a
// single clean compilation unit.)
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_SWEEP_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_SWEEP_H

#include "bspline_ops.h"  // BsplineCurveData / BsplineSurfaceData (Layer-1 data types)
#include "vec.h"          // Vec3 / Dir3 (sweep direction, reference normal)

#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Result of a sweep operation.
// ─────────────────────────────────────────────────────────────────────────────

struct SweepResult {
  bool ok = false;             ///< true ⇔ the sweep succeeded
  BsplineSurfaceData surface;  ///< the swept non-rational tensor-product surface
  /// The V parameters at which the surface reproduces a placed section. For a
  /// translational sweep these are {0, 1} (the two path endpoints); for a general
  /// sweep they are the K station parameters the skin interpolates at.
  std::vector<double> vParams;
};

// ─────────────────────────────────────────────────────────────────────────────
// Translational sweep (EXACT — the closed-form base case).
// ─────────────────────────────────────────────────────────────────────────────

/// Sweep `section` along the straight vector `sweep` (§10.4, extrusion / tabulated
/// cylinder). Returns the EXACT tensor-product surface whose U direction is the
/// section (degree p, the section's knots, its N poles) and whose V direction is a
/// degree-1 two-pole line from 0 to `sweep`. By construction the surface net is
/// pole(i,0) = section.poles[i], pole(i,1) = section.poles[i] + sweep, so the iso-curve
/// S(·, v) is EXACTLY the section translated by v·sweep — no fitting, machine-exact.
///
/// vParams = {0, 1} (the surface reproduces the section at v=0 and the section+sweep at
/// v=1, and every iso in between is the section translated proportionally).
///
/// Declines (`ok=false`, no crash) on a rational section (non-empty weights), an empty /
/// malformed section (bad knot vector, degree < 1), or a null sweep vector (|sweep| below
/// tolerance — a zero-length extrusion has no surface).
SweepResult sweepTranslational(const BsplineCurveData& section, const Vec3& sweep);

// ─────────────────────────────────────────────────────────────────────────────
// General sweep along a trajectory curve (transform-then-skin).
// ─────────────────────────────────────────────────────────────────────────────

/// Sweep `section` along the `trajectory` (spine) B-spline (§10.4). Samples the
/// trajectory at `stations` parameter values (evenly in the trajectory's knot domain),
/// builds a ROTATION-MINIMIZING moving frame along it (double-reflection method — see the
/// file header for the anti-twist rationale), transforms a copy of `section` to each
/// station (translate to the trajectory point, rotate the section's reference normal onto
/// the trajectory tangent within the rotation-minimizing frame), and SKINS the `stations`
/// transformed sections into one tensor-product surface via skinSurface.
///
/// `sectionNormal` is the section's plane normal (the reference the frame carries). It
/// need not be exact; the frame only needs a consistent reference to avoid twist.
/// `degreeV` is the skin's across-stations degree (clamped to [1, stations−1] by the skin).
///
/// GUARANTEE (the containment oracle, inherited from skinSurface): the swept surface
/// contains each TRANSFORMED section at its station parameter to skinning tolerance
/// (~1e-8). The transformed sections themselves are exact rigid placements of the section.
///
/// Declines (`ok=false`, no crash) on: fewer than 2 stations; a rational section or
/// trajectory (non-empty weights); an empty / malformed section or trajectory; a
/// degenerate trajectory whose sampled station points are all coincident (no path to
/// sweep along); or a downstream skin failure. NON-RATIONAL.
SweepResult sweepAlongTrajectory(const BsplineCurveData& section,
                                 const BsplineCurveData& trajectory,
                                 const Dir3& sectionNormal, int stations = 16,
                                 int degreeV = 3);

// ─────────────────────────────────────────────────────────────────────────────
// Rational swept surfaces (weighted section).
// ─────────────────────────────────────────────────────────────────────────────

/// EXACT rational translational sweep — the rational analogue of sweepTranslational.
/// `section` MUST be rational (non-empty, strictly-positive weights, one per pole). Returns
/// the EXACT rational tensor-product surface: U = the rational section (degree p, the
/// section's knots/poles/WEIGHTS), V = a degree-1 two-pole line from 0 to `sweep`. The
/// weight net is constant in V — weight(i,0) = weight(i,1) = section.weights[i] — and the
/// pole net is pole(i,0) = section.poles[i], pole(i,1) = section.poles[i] + sweep, so the
/// rational iso-curve S(·, v) is EXACTLY the rational section translated by v·sweep. No
/// fitting, machine-exact. This is the strongest rational oracle: a rational quadratic
/// CIRCLE (cos-half-angle middle weights) swept translationally is an EXACT rational
/// CYLINDER, matching the analytic cylinder pointwise.
///
/// vParams = {0, 1}. Declines (`ok=false`, no crash) on a NON-rational section (empty
/// weights — use sweepTranslational), a non-positive/mismatched weight, an empty/malformed
/// section, or a null sweep vector.
SweepResult sweepRationalTranslational(const BsplineCurveData& section, const Vec3& sweep);

/// General rational sweep along a `trajectory` — the rational analogue of
/// sweepAlongTrajectory. `section` MUST be rational (strictly-positive weights); the
/// trajectory is NON-rational (it only provides the spine point/tangent for the moving
/// frame). Places the rational section at `stations` along the trajectory with the SAME
/// rotation-minimizing moving frame (double-reflection) — a RIGID transform, which
/// preserves the section's weights EXACTLY — then RATIONAL-SKINS the placed rational
/// sections via skinRationalSurface.
///
/// GUARANTEE (the rational containment oracle, inherited from skinRationalSurface): the
/// swept rational surface contains each transformed rational section at its station
/// parameter to skinning tolerance. Declines (`ok=false`) on: fewer than 2 stations; a
/// NON-rational or non-positive-weight section; a rational trajectory; an empty/malformed
/// section or trajectory; a degenerate (coincident-station) trajectory; or a downstream
/// rational-skin failure.
SweepResult sweepRationalAlongTrajectory(const BsplineCurveData& section,
                                         const BsplineCurveData& trajectory,
                                         const Dir3& sectionNormal, int stations = 16,
                                         int degreeV = 3);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_SWEEP_H
