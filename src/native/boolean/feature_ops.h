// SPDX-License-Identifier: Apache-2.0
//
// feature_ops.h — BOOL-FEATURE / LAYER 3 → CAD OPS: the CAD feature operations POCKET
// (subtract a swept profile from a solid) and BOSS/PAD (add one) as the natural
// composition on TOP of the general two-freeform-solid NURBS boolean.
//
// ── WHAT THIS IS (a COMPOSITION, not a new boolean or a new sweep) ─────────────────
// The general boolean `nurbsSolidBoolean(A, B, op)` (nurbs_solid_boolean.h) already welds
// two freeform NURBS solids into ONE watertight-or-honest-decline result. Sweeping a closed
// PROFILE loop into a TOOL solid — via the Layer-6 sweep primitives in math/bspline_sweep.h
// (translational / rotational) — and then booleaning that tool against a BASE solid turns
// the boolean into a usable modelling PRIMITIVE:
//   * POCKET(base, profile, depth) — sweep the profile by `depth` into a tool solid, then
//     `nurbsSolidBoolean(base, tool, Cut)` ⇒ the carved (pocketed) solid.
//   * BOSS  (base, profile, height) — sweep the profile by `height` into a tool solid, then
//     `nurbsSolidBoolean(base, tool, Fuse)` ⇒ the raised pad.
// This header COMPOSES those two landed verbs; it re-implements NEITHER the boolean NOR the
// sweep. The sweep builds the tool wall surface (`sweptExtrudeToolWall`, an EXACT
// translational sweep of the profile section); a small watertight-cup assembly closes the
// swept wall with the two analytic lids the profile bounds; the boolean does the rest.
//
// ── HONESTY (DISAGREED=0 SACRED) ──────────────────────────────────────────────────
// A feature op is watertight IFF the underlying boolean is: `pocket`/`boss` return the
// boolean's verified solid, or — when the boolean HONEST-DECLINES (a multi-seam FUSE, a
// tool whose intersection with the base is beyond the tractable single-seam pose, a null /
// unadmitted operand) — a NULL Shape with the boolean's measured `SolidBoolReport` carried
// through verbatim. NEVER a leaky/partial/wrong solid; NO tolerance is ever widened. The
// feature op adds its OWN pre-checks (a null base, a failed sweep, a degenerate tool) as
// first-class declines BEFORE the boolean is even called.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
// GUARD — the FEATURE OPS themselves (`pocket` / `boss` / `featureOp`) only compose the
// boolean and link UNCONDITIONALLY. The swept-tool WALL builder (`sweptExtrudeToolWall`)
// calls `math::sweepTranslational`, whose translation unit is compiled under
// CYBERCAD_HAS_NUMSCI (bspline_sweep.cpp) — so that ONE helper links only when numsci is
// present, exactly like every other caller of the Layer-6 sweep. The header itself always
// compiles (the sweep declaration is always visible).
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_FEATURE_OPS_H
#define CYBERCAD_NATIVE_BOOLEAN_FEATURE_OPS_H

#include "native/boolean/nurbs_solid_boolean.h"
#include "native/math/bspline_ops.h"
#include "native/math/bspline_sweep.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

namespace cybercad::native::boolean {

namespace fmath = cybercad::native::math;

// ─────────────────────────────────────────────────────────────────────────────
// The CAD feature operation. POCKET subtracts the swept tool; BOSS adds it.
// ─────────────────────────────────────────────────────────────────────────────
enum class FeatureOp { Pocket, Boss };

inline const char* featureOpName(FeatureOp op) noexcept {
  switch (op) {
    case FeatureOp::Pocket: return "Pocket";
    case FeatureOp::Boss: return "Boss";
  }
  return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// The measured feature-op decline. `Ok` iff a verified watertight solid is returned;
// every non-Ok is an HONEST decline to NULL (never a leaky/partial/wrong solid). The
// SweepFailed / DegenerateTool / NullBase reasons are the feature op's OWN pre-checks;
// BooleanDeclined defers to the underlying boolean's measured `SolidBoolReport`.
// ─────────────────────────────────────────────────────────────────────────────
enum class FeatureDecline {
  Ok,
  NullBase,         ///< the base solid is a null Shape (nothing to pocket/boss)
  BadProfile,       ///< the profile section is empty / malformed (degree < 1, short knots)
  BadDepth,         ///< the sweep depth/height is (near-)zero — no tool to build
  SweepFailed,      ///< the Layer-6 sweep declined (see the SweepResult contract)
  DegenerateTool,   ///< the swept tool is not a usable freeform solid (bad wall/lids)
  BooleanDeclined   ///< the underlying nurbsSolidBoolean honest-declined (see `boolReport`)
};

inline const char* featureDeclineName(FeatureDecline d) noexcept {
  switch (d) {
    case FeatureDecline::Ok: return "Ok";
    case FeatureDecline::NullBase: return "NullBase";
    case FeatureDecline::BadProfile: return "BadProfile";
    case FeatureDecline::BadDepth: return "BadDepth";
    case FeatureDecline::SweepFailed: return "SweepFailed";
    case FeatureDecline::DegenerateTool: return "DegenerateTool";
    case FeatureDecline::BooleanDeclined: return "BooleanDeclined";
  }
  return "?";
}

/// The honest witnesses of a feature op (for the host gate / caller log). `boolReport` is
/// the underlying boolean's measured report verbatim (its `decline`, watertight/coherent
/// witnesses, residual map) when the boolean ran; `decline` is the FEATURE-level reason.
struct FeatureReport {
  FeatureDecline decline = FeatureDecline::Ok;
  FeatureOp op = FeatureOp::Pocket;
  SolidBoolReport boolReport;   ///< the underlying boolean's report (valid iff it ran)
};

// ─────────────────────────────────────────────────────────────────────────────
// Swept-tool builders — COMPOSE the Layer-6 sweep to turn a closed profile into the tool
// solid's freeform wall, then close it into a watertight cup. These are the ONLY new code;
// the boolean and the sweep stay byte-unchanged.
// ─────────────────────────────────────────────────────────────────────────────
namespace featdetail {

/// EXACTLY sweep the profile SECTION (a NURBS curve — one iso-rail of the tool wall) by the
/// straight vector `sweep` into the tool's freeform WALL surface (a tensor-product Bézier /
/// B-spline patch). Delegates verbatim to `math::sweepTranslational` (the EXACT extrusion —
/// §10.4, no fitting) and re-packs its `BsplineSurfaceData` into the topology `FaceSurface`
/// the boolean's B1 recogniser + SSI trace consume. Returns nullopt on a sweep decline (a
/// rational / malformed section, a null sweep vector) — the feature op then declines
/// `SweepFailed`, never fabricates a wall.
inline std::optional<topo::FaceSurface> sweptExtrudeToolWall(const fmath::BsplineCurveData& section,
                                                             const fmath::Vec3& sweep) {
  const fmath::SweepResult sr = fmath::sweepTranslational(section, sweep);
  if (!sr.ok) return std::nullopt;
  const fmath::BsplineSurfaceData& s = sr.surface;
  if (s.nPolesU < 2 || s.nPolesV < 2 ||
      s.poles.size() != static_cast<std::size_t>(s.nPolesU) * s.nPolesV)
    return std::nullopt;
  topo::FaceSurface fs{};
  // A non-rational section extruded is a non-rational patch; the SSI trace's Bézier adapter
  // reads the pole grid directly (Bézier kind ⇔ single patch, Bernstein basis).
  const bool bezier = s.knotsU.size() == static_cast<std::size_t>(s.nPolesU + s.degreeU + 1) &&
                      s.degreeU == s.nPolesU - 1;
  fs.kind = bezier ? topo::FaceSurface::Kind::Bezier : topo::FaceSurface::Kind::BSpline;
  fs.degreeU = s.degreeU;
  fs.degreeV = s.degreeV;
  fs.nPolesU = s.nPolesU;
  fs.nPolesV = s.nPolesV;
  fs.poles = s.poles;
  fs.weights = s.weights;
  fs.knotsU = s.knotsU;
  fs.knotsV = s.knotsV;
  return fs;
}

}  // namespace featdetail

// ─────────────────────────────────────────────────────────────────────────────
// featureOp — the shared driver. Builds the requested feature by booleaning the (already
// swept + capped) TOOL solid against the BASE, mapping the op to the boolean operator
// (Pocket ⇒ Cut, Boss ⇒ Fuse), and carrying the boolean's honest-decline THROUGH. The tool
// is supplied as a Shape so the caller (or the fixture) owns the sweep→cup assembly; the two
// public entry points below name the operation.
//
// `analyticOpVolume` (optional, NaN ⇒ unknown) is forwarded to the boolean's TWO-SIDED
// volume band (for a pocket: V(base) − V(pocket box); for a boss: V(base) + V(pad)).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape featureOp(const topo::Shape& base, const topo::Shape& tool, FeatureOp op,
                             double deflection = 0.005, FeatureReport* report = nullptr,
                             double analyticOpVolume =
                                 std::numeric_limits<double>::quiet_NaN()) {
  FeatureReport rep;
  rep.op = op;
  auto emit = [&](topo::Shape s) -> topo::Shape {
    if (report) *report = rep;
    return s;
  };
  auto fail = [&](FeatureDecline d) -> topo::Shape { rep.decline = d; return emit({}); };

  // Feature-level pre-checks (BEFORE the boolean is called).
  if (base.isNull()) return fail(FeatureDecline::NullBase);
  if (tool.isNull()) return fail(FeatureDecline::DegenerateTool);

  // Compose the boolean: Pocket ⇒ Cut(base − tool); Boss ⇒ Fuse(base ∪ tool).
  const SolidBoolOp bop = op == FeatureOp::Pocket ? SolidBoolOp::Cut : SolidBoolOp::Fuse;
  const topo::Shape r =
      nurbsSolidBoolean(base, tool, bop, deflection, &rep.boolReport, analyticOpVolume);
  if (r.isNull() || rep.boolReport.decline != SolidBoolDecline::Ok)
    return fail(FeatureDecline::BooleanDeclined);
  rep.decline = FeatureDecline::Ok;
  return emit(r);
}

// ─────────────────────────────────────────────────────────────────────────────
// pocket — subtract the tool solid from the base (Cut). The tool is a swept profile solid
// (built by the caller / `pocketFromProfile` below). Returns the pocketed watertight solid
// or an HONEST decline to NULL with a measured `FeatureReport`.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape pocket(const topo::Shape& base, const topo::Shape& tool,
                          double deflection = 0.005, FeatureReport* report = nullptr,
                          double analyticOpVolume = std::numeric_limits<double>::quiet_NaN()) {
  return featureOp(base, tool, FeatureOp::Pocket, deflection, report, analyticOpVolume);
}

// ─────────────────────────────────────────────────────────────────────────────
// boss — add the tool solid to the base (Fuse), a raised pad. Returns the boss'd watertight
// solid or an HONEST decline to NULL with a measured `FeatureReport`.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape boss(const topo::Shape& base, const topo::Shape& tool,
                        double deflection = 0.005, FeatureReport* report = nullptr,
                        double analyticOpVolume = std::numeric_limits<double>::quiet_NaN()) {
  return featureOp(base, tool, FeatureOp::Boss, deflection, report, analyticOpVolume);
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_FEATURE_OPS_H
