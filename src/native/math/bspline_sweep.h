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
// exactly) and RATIONAL-SKINS them via `skinRationalSurface`. PLUS the ROTATIONAL (revolved)
// sweep: `sweepRotational` revolves a section PROFILE about an AXIS by an angle — an EXACT
// RATIONAL surface of revolution (the standard rational-arc construction, *The NURBS Book*
// Algorithm A7.1): the revolve direction (V) is a degree-2 rational arc built from piecewise
// ≤90° segments with cos-half-angle middle weights, so the surface matches the ANALYTIC surface
// of revolution pointwise (a straight offset segment → exact CYLINDER, a tilted segment → exact
// CONE, a semicircle → exact SPHERE). Exact GeomFill/BRepFill-class sweeps (an analytically exact
// swept surface rather than a skinned approximation of stations) and a VARIABLE section (a
// section that morphs along the spine) are documented residuals for later slices — this module
// never fakes them. See docs/NURBS-SCOPE.md Layer-6 row.
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

// ─────────────────────────────────────────────────────────────────────────────
// Rotational (revolved) sweep — EXACT rational surface of revolution.
// ─────────────────────────────────────────────────────────────────────────────

/// EXACT rotational sweep (*The NURBS Book* §8.5 / Algorithm A7.1 — MakeRevolvedSurface):
/// revolve the section PROFILE `section` about the axis (`axisPoint`, `axisDir`) through the
/// signed `angle` radians. The result is an EXACT RATIONAL NURBS surface of revolution whose
/// U direction is the profile (degree p, the profile's knots/poles, and — if the profile is
/// rational — its WEIGHTS carried through) and whose V direction is the standard degree-2
/// rational CIRCULAR ARC: the arc is split into `narcs = ceil(|angle| / 90°)` segments each
/// spanning ≤ 90°, giving `2·narcs + 1` V-poles over a clamped degree-2 knot vector with
/// interior multiplicity 2. Each profile point Pₖ contributes, per segment, an ON-arc pole
/// (weight 1) and a BETWEEN pole at radius rₖ / cos(Δθ/2) with weight cos(Δθ/2), where rₖ is
/// Pₖ's distance from the axis. The homogeneous product of the profile weight and the arc
/// weight is the surface weight — so the revolved surface reproduces the ANALYTIC surface of
/// revolution POINTWISE (not a facet):
///   * a straight segment offset from and PARALLEL to the axis, revolved 360° → an EXACT CYLINDER;
///   * a straight segment TILTED to the axis, revolved → an EXACT CONE / frustum;
///   * a semicircle whose diameter lies on the axis, revolved 360° → an EXACT SPHERE.
///
/// The finished surface CONTAINS the profile at V = 0 (`S(·, 0)` equals `section`) and the
/// profile rotated by `angle` at V = 1. `vParams = {0, 1}` (the domain endpoints; the arc in
/// between is the exact rational sweep of the profile through the swept angle).
///
/// Declines (`ok=false`, no crash) on: a malformed profile (bad knot vector, degree < 1, or —
/// if rational — a non-positive / mismatched weight); a non-unit / null `axisDir`; a
/// (near-)zero `|angle|` (no surface is swept); or a DEGENERATE placement where the entire
/// profile lies ON the axis (every rₖ ≈ 0 — the revolve collapses to the axis line, no
/// surface). A profile that only PARTLY touches the axis (some rₖ = 0, e.g. a sphere's pole)
/// is fine — those poles stay on the axis and the surface degenerates smoothly there.
SweepResult sweepRotational(const BsplineCurveData& section, const Point3& axisPoint,
                            const Dir3& axisDir, double angle);

// ─────────────────────────────────────────────────────────────────────────────
// Variable-section sweep — the section SCALES and/or TWISTS as it rides the spine.
// ─────────────────────────────────────────────────────────────────────────────

/// VARIABLE-SECTION sweep (§10.4, generalized): sweep `section` along `trajectory` while
/// applying a station-dependent SCALE and TWIST to the rotation-minimizing-frame–transported
/// section, then SKIN the placed sections into one tensor-product surface (exactly like
/// sweepAlongTrajectory, but the placement composes an extra scale+twist in the section's own
/// plane before the rigid frame). This is the pipe-with-varying-radius / tapered-moulding /
/// twisted-extrusion primitive.
///
/// The variation is supplied as SAMPLED FIELDS, one value per station (analytic laws are
/// sampled by the caller — a linear taper is `scales[k] = s0 + (s1-s0)*k/(K-1)`):
///   * `scales[k]` — uniform scale of the section about its ORIGIN at station k (must be > 0).
///   * `twists[k]` — rotation (radians) of the section about its LOCAL SWEEP AXIS (the spine
///     tangent) at station k, applied in the section plane before the rigid frame.
/// Both fields must have exactly `stations` entries. Passing an all-ones `scales` and an
/// all-zero `twists` reproduces `sweepAlongTrajectory` EXACTLY (the constant-section oracle).
///
/// GUARANTEE (containment, inherited from skinSurface): the surface contains each SCALED+
/// TWISTED+placed section at its station parameter to skinning tolerance (~1e-8). ANALYTIC
/// oracle: a circular section swept along a straight spine with a linear scale is an exact
/// cone frustum (the placed sections are concentric scaled circles → the skin reproduces the
/// frustum). `degreeV` is the across-stations skin degree.
///
/// Declines (`ok=false`, no crash) on: fewer than 2 stations; a rational section or trajectory
/// (use the rational variant); an empty/malformed section or trajectory; a field size !=
/// `stations`; a non-positive scale; a degenerate (coincident-station) trajectory; or a
/// downstream skin failure. NON-RATIONAL.
SweepResult sweepVariable(const BsplineCurveData& section, const BsplineCurveData& trajectory,
                          const Dir3& sectionNormal, const std::vector<double>& scales,
                          const std::vector<double>& twists, int stations = 16, int degreeV = 3);

/// RATIONAL analogue of sweepVariable — `section` MUST be rational (strictly-positive weights),
/// the trajectory is NON-rational. The scale+twist about the section origin is a uniform
/// similarity (it multiplies every pole's position by the scale and rotates it), which PRESERVES
/// the section's weights EXACTLY (weights are invariant under a similarity), then RATIONAL-SKINS
/// the placed rational sections. So a TRUE rational circle scaled linearly along a straight spine
/// is an EXACT rational cone frustum. Declines mirror sweepVariable (plus non-rational/
/// non-positive-weight section -> decline; rational trajectory -> decline).
SweepResult sweepRationalVariable(const BsplineCurveData& section,
                                  const BsplineCurveData& trajectory, const Dir3& sectionNormal,
                                  const std::vector<double>& scales,
                                  const std::vector<double>& twists, int stations = 16,
                                  int degreeV = 3);

// ─────────────────────────────────────────────────────────────────────────────
// Two-rail sweep — the section is fit between two rail curves at every station.
// ─────────────────────────────────────────────────────────────────────────────

/// TWO-RAIL sweep (the classic "sweep between two guide rails"): the section carries two ANCHOR
/// points `poles[anchor0]` and `poles[anchor1]`; at each station parameter `t` the section is
/// positioned/scaled/oriented so that anchor0 rides `rail0(t)` and anchor1 rides `rail1(t)`.
/// At station k (t sampled evenly over the COMMON rail domain):
///   * SCALE `s(t) = |rail1(t) - rail0(t)| / |anchor1 - anchor0|` (the section stretches so its
///     two anchors span the current rail-to-rail chord);
///   * the section's anchor chord direction is aligned to the rail chord `rail1(t)-rail0(t)`,
///     and the remaining orientation is carried by a ROTATION-MINIMIZING frame along the
///     rail-midpoint spine (anti-twist), so the section does not spin between the rails;
///   * TRANSLATE so anchor0 maps onto `rail0(t)`.
/// The placed sections are then SKINNED into one tensor-product surface. This is the standard
/// two-rail construction (BRepFill-class): the profile is anchored to both rails at every
/// station, exactly.
///
/// GUARANTEE: the two anchor iso-curves of the surface LIE ON rail0 and rail1 respectively at
/// every station (<= ~1e-8), and the surface contains each placed section (skin containment).
/// ANALYTIC oracles: two parallel straight rails + a straight segment section give a planar/
/// ruled strip; two rails diverging linearly give the exact tapered surface, with the section
/// endpoints ON the rails at every station.
///
/// `anchor0` / `anchor1` are the section pole INDICES that ride the rails (0-based, distinct,
/// in range). `sectionNormal` is the section plane normal (the RMF reference). `degreeV` is the
/// across-stations skin degree.
///
/// Declines (`ok=false`, no crash) on: fewer than 2 stations; a rational section or either
/// rational rail (use the rational variant); an empty/malformed section or rail; anchor indices
/// out of range / equal; coincident section anchors (`|anchor1-anchor0| ~= 0` — no chord to
/// scale from); DEGENERATE RAILS (a station where the rail chord `|rail1(t)-rail0(t)| ~= 0`, i.e.
/// the rails cross or touch — the scale/orientation is undefined, honest-decline); a degenerate
/// (coincident-midpoint) rail spine; or a downstream skin failure. NON-RATIONAL.
SweepResult sweepTwoRail(const BsplineCurveData& section, const BsplineCurveData& rail0,
                         const BsplineCurveData& rail1, const Dir3& sectionNormal, int anchor0,
                         int anchor1, int stations = 16, int degreeV = 3);

/// RATIONAL analogue of sweepTwoRail — `section` MUST be rational (strictly-positive weights);
/// both rails are NON-rational (they only provide the rail points that the anchors ride). The
/// per-station placement is a SIMILARITY (uniform scale + rotation + translation), which
/// preserves the section's weights EXACTLY, then RATIONAL-SKINS the placed sections. Declines
/// mirror sweepTwoRail (plus non-rational/non-positive-weight section -> decline; rational rail
/// -> decline).
SweepResult sweepRationalTwoRail(const BsplineCurveData& section, const BsplineCurveData& rail0,
                                 const BsplineCurveData& rail1, const Dir3& sectionNormal,
                                 int anchor0, int anchor1, int stations = 16, int degreeV = 3);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_SWEEP_H
