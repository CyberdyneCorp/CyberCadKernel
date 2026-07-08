// SPDX-License-Identifier: Apache-2.0
//
// orthographic_hlr.h — orthographic (parallel) hidden-line removal over a
//                      POLYHEDRAL occluder.
//
// This is the provable core of the native drafting service (MOAT M-GS GS1). It
// consumes a triangle occluder mesh (the M0 boundary tessellation produced by
// src/native/tessellate/solid_mesher.h) plus the set of straight polyline edges
// to draw, projects each edge orthographically onto the drawing plane, and
// classifies every projected sample VISIBLE or HIDDEN by ray-casting toward the
// (parallel) viewpoint against the occluder. Edges are SPLIT at visibility
// transitions so the visible and hidden segment sets are disjoint and exact.
//
// Scope (this slice): straight (linear) edges over a triangle occluder — i.e.
// the ANALYTIC + POLYHEDRAL core (box / prism, and the faceted occluder of any
// tessellated solid). Curved silhouette tracing (cylinder/cone/sphere outline
// ellipses) and freeform faces are DECLINED here and handled by the
// topology-driven adapter in a later slice; this header never emits a wrong
// visible/hidden classification for the cases it does accept.
//
// Verification: Gate (a) HOST ANALYTIC — a box from an isometric corner yields
//   exactly 9 visible + 3 hidden segments (tests/native/test_native_drafting.cpp).
//   Gate (b) SIM native-vs-OCCT HLRBRep_Algo is a separate harness.
//
// OCCT-FREE: includes only src/native/math. Header-only, clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_DRAFTING_ORTHOGRAPHIC_HLR_H
#define CYBERCAD_NATIVE_DRAFTING_ORTHOGRAPHIC_HLR_H

#include "native/math/vec.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace cybercad::native::drafting {

namespace math = cybercad::native::math;

// ─────────────────────────────────────────────────────────────────────────────
// Inputs / outputs.
// ─────────────────────────────────────────────────────────────────────────────

/// A triangle occluder: shared vertex array + zero-based index triples. This is
/// exactly the shape of tessellate::Mesh (positions + Triangle indices), so a
/// solid's boundary tessellation feeds straight in.
struct Occluder {
  const std::vector<math::Point3>* vertices = nullptr;
  const std::vector<std::array<std::uint32_t, 3>>* triangles = nullptr;
};

/// One straight edge to draw, as two zero-based indices into the edge-vertex
/// array (which may differ from the occluder's vertex array).
using EdgeIndices = std::array<std::uint32_t, 2>;

/// A 2D segment on the drawing plane (drawing-plane coordinates, mm).
struct Segment2D {
  double ax = 0.0, ay = 0.0;  ///< endpoint A (u, w)
  double bx = 0.0, by = 0.0;  ///< endpoint B (u, w)
};

/// Result of an HLR pass: disjoint visible + hidden projected segment sets.
struct HlrResult {
  std::vector<Segment2D> visible;
  std::vector<Segment2D> hidden;
};

/// Orthographic view: projection direction (into the scene, unit) + an up hint.
/// The drawing-plane basis is right = normalize(viewDir × up), trueUp =
/// right × viewDir; a world point projects to (P·right, P·trueUp). Depth toward
/// the camera increases along −viewDir.
struct OrthographicView {
  math::Dir3 viewDir{0, 0, -1};  ///< camera looks along +viewDir
  math::Dir3 up{0, 1, 0};        ///< up hint (must not be parallel to viewDir)
};

/// Tunables. Defaults are calibrated for mm-scale analytic/polyhedral solids;
/// they are never silently weakened at call sites.
struct HlrParams {
  int samplesPerEdge = 64;         ///< classification samples along each edge
  double surfaceOffset = 1e-6;     ///< push a sample toward the camera before casting
  double rayEps = 1e-7;            ///< min hit distance counted as "in front"
  double transitionTol = 1e-9;     ///< bisection tolerance on the split parameter
  double dropSegmentLen = 1e-9;    ///< discard degenerate (zero-length) segments
};

// ─────────────────────────────────────────────────────────────────────────────
// Internal geometry helpers (small, single-purpose).
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

/// Orthonormal drawing-plane basis derived from the view.
struct PlaneBasis {
  math::Vec3 right, up, viewDir;
};

inline PlaneBasis makeBasis(const OrthographicView& v) noexcept {
  const math::Vec3 vd = v.viewDir.vec();
  const math::Dir3 right{cross(vd, v.up.vec())};
  const math::Vec3 r = right.vec();
  return PlaneBasis{r, cross(r, vd), vd};
}

/// Project a world point to drawing-plane (u, w).
inline std::pair<double, double> project(const PlaneBasis& b, const math::Point3& p) noexcept {
  const math::Vec3 pv = p.asVec();
  return {dot(pv, b.right), dot(pv, b.up)};
}

/// Möller–Trumbore ray/triangle intersection. Returns true and sets `t` (signed
/// distance along the unit `dir`) when the ray hits the triangle interior.
inline bool rayTriangle(const math::Point3& orig, const math::Vec3& dir, const math::Point3& v0,
                        const math::Point3& v1, const math::Point3& v2, double& t) noexcept {
  constexpr double kEps = 1e-12;
  const math::Vec3 e1 = v1 - v0, e2 = v2 - v0;
  const math::Vec3 p = cross(dir, e2);
  const double det = dot(e1, p);
  if (std::fabs(det) < kEps) return false;  // ray parallel to triangle plane
  const double inv = 1.0 / det;
  const math::Vec3 s = orig - v0;
  const double u = dot(s, p) * inv;
  if (u < -kEps || u > 1.0 + kEps) return false;
  const math::Vec3 q = cross(s, e1);
  const double w = dot(dir, q) * inv;
  if (w < -kEps || u + w > 1.0 + kEps) return false;
  t = dot(e2, q) * inv;
  return true;
}

/// A world sample is HIDDEN when, after nudging it toward the camera, a ray to
/// the camera crosses any occluder triangle at a strictly positive distance —
/// i.e. some surface lies nearer the camera along this line of sight.
inline bool occluded(const math::Point3& world, const math::Vec3& viewDir, const Occluder& occ,
                     const HlrParams& prm) noexcept {
  const math::Vec3 toCam = -viewDir;                       // unit (viewDir is unit)
  const math::Point3 orig = world + toCam * prm.surfaceOffset;
  const auto& V = *occ.vertices;
  double t = 0.0;
  for (const auto& tri : *occ.triangles) {
    if (rayTriangle(orig, toCam, V[tri[0]], V[tri[1]], V[tri[2]], t) && t > prm.rayEps)
      return true;
  }
  return false;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// projectOrthographic — the HLR pass over straight edges + a triangle occluder.
// ─────────────────────────────────────────────────────────────────────────────

/// Classify + split every straight edge against the occluder and return the
/// disjoint visible/hidden projected-segment sets. `edgeVertices[e]` are the two
/// world endpoints of edge `e` (indexed by `edges`).
inline HlrResult projectOrthographic(const std::vector<math::Point3>& edgeVertices,
                                     const std::vector<EdgeIndices>& edges, const Occluder& occ,
                                     const OrthographicView& view, const HlrParams& prm = {}) {
  const detail::PlaneBasis basis = detail::makeBasis(view);
  HlrResult out;
  const int n = prm.samplesPerEdge < 1 ? 1 : prm.samplesPerEdge;

  auto pointAt = [&](const math::Point3& a, const math::Point3& b, double s) {
    return math::Point3{a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s};
  };
  auto hiddenAt = [&](const math::Point3& a, const math::Point3& b, double s) {
    return detail::occluded(pointAt(a, b, s), basis.viewDir, occ, prm);
  };
  // Refine the visibility-transition parameter in (lo, hi) to `transitionTol`.
  auto refine = [&](const math::Point3& a, const math::Point3& b, double lo, double hi) {
    const bool hiddenLo = hiddenAt(a, b, lo);
    while (hi - lo > prm.transitionTol) {
      const double mid = 0.5 * (lo + hi);
      if (hiddenAt(a, b, mid) == hiddenLo)
        lo = mid;
      else
        hi = mid;
    }
    return 0.5 * (lo + hi);
  };
  auto emit = [&](const math::Point3& a, const math::Point3& b, double s0, double s1, bool hidden) {
    const auto [ax, ay] = detail::project(basis, pointAt(a, b, s0));
    const auto [bx, by] = detail::project(basis, pointAt(a, b, s1));
    if (std::hypot(bx - ax, by - ay) < prm.dropSegmentLen) return;
    (hidden ? out.hidden : out.visible).push_back(Segment2D{ax, ay, bx, by});
  };

  // Classify at CELL MIDPOINTS (strict edge interior), never at the exact
  // endpoints: an edge's endpoints coincide with vertices shared by other edges
  // and, on a silhouette, sit on the outline where the visible/hidden status is
  // ambiguous. Sampling interiors makes the classification well-defined and the
  // emitted segments still span the full edge to its endpoints.
  for (const EdgeIndices& e : edges) {
    const math::Point3 a = edgeVertices[e[0]];
    const math::Point3 b = edgeVertices[e[1]];
    double runStart = 0.0;
    double prevMid = 0.5 / n;
    bool runHidden = hiddenAt(a, b, prevMid);
    for (int i = 1; i < n; ++i) {
      const double sMid = (static_cast<double>(i) + 0.5) / n;
      const bool h = hiddenAt(a, b, sMid);
      if (h != runHidden) {
        const double sSplit = refine(a, b, prevMid, sMid);
        emit(a, b, runStart, sSplit, runHidden);
        runStart = sSplit;
        runHidden = h;
      }
      prevMid = sMid;
    }
    emit(a, b, runStart, 1.0, runHidden);
  }
  return out;
}

}  // namespace cybercad::native::drafting

#endif  // CYBERCAD_NATIVE_DRAFTING_ORTHOGRAPHIC_HLR_H
