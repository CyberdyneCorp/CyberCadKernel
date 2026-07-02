// SPDX-License-Identifier: Apache-2.0
//
// gpu_sample.h — OPTIONAL fp32 GPU sampling backend for eligible free-form faces.
//
// The Phase-2 GPU surface evaluator (src/compute/metal/gpu_surface_eval.h,
// `cyber::metal::evaluateSurfaceGrid`) fills an fp32 grid of surface points +
// normals for a tensor-product NURBS/B-spline/Bézier patch on the Metal compute
// backend. A face is ELIGIBLE for it when:
//   * its surface is free-form (BSpline/Bezier) — analytic faces are trivial on
//     the CPU and not worth a GPU dispatch;
//   * it has a single outer wire and no holes (a rectangular UV grid maps
//     directly to the untrimmed patch domain — no per-sample trim needed on the
//     GPU; trimming still happens on the CPU for holed faces);
//   * its degree is within the GPU kernel's cap (kMaxSurfaceDegree).
// For every other face — and whenever no Metal backend is active — the CPU fp64
// path (surface_eval.h) is used. Correctness is defined on the CPU fp64 path;
// the GPU only substitutes fp32 point/normal sampling within the documented
// fp32 parity tolerance, which is far under a typical meshing deflection.
//
// This header is HOST-BUILDABLE and OCCT-FREE: without CYBERCAD_HAS_METAL it
// compiles to a no-op stub (eligible()==false, sampleGrid returns nullopt), so
// src/native/tessellate always builds with `clang++ -std=c++20` and no Metal.
// The Metal include is pulled in ONLY under the guard.
//
// clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_GPU_SAMPLE_H
#define CYBERCAD_NATIVE_TESSELLATE_GPU_SAMPLE_H

#include "native/math/native_math.h"
#include "native/topology/shape.h"

#include <optional>
#include <vector>

#if defined(CYBERCAD_HAS_METAL)
#include "compute/metal/gpu_surface_eval.h"
#endif

namespace cybercad::native::tessellate {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

/// An fp64 grid of sampled points + normals (row-major, U outer), the CPU-side
/// product of a GPU sample after promotion back to fp64 for the mesh.
struct SampledGrid {
  int nU = 0;
  int nV = 0;
  std::vector<math::Point3> points;
  std::vector<math::Dir3> normals;
};

/// A face is GPU-eligible if it is a free-form patch with a single outer wire and
/// no holes, and within the degree cap. Always false without Metal.
inline bool gpuEligible(const topo::FaceSurface& s, const topo::Shape& face) noexcept {
#if defined(CYBERCAD_HAS_METAL)
  using K = topo::FaceSurface::Kind;
  if (s.kind != K::BSpline && s.kind != K::Bezier) return false;
  if (face.isNull() || face.tshape()->children().size() != 1) return false;  // one outer, no holes
  if (s.degreeU > cyber::metal::kMaxSurfaceDegree ||
      s.degreeV > cyber::metal::kMaxSurfaceDegree)
    return false;
  return s.nPolesU > 0 && s.nPolesV > 0;
#else
  (void)s;
  (void)face;
  return false;
#endif
}

#if defined(CYBERCAD_HAS_METAL)

namespace detail {

// Convert a native free-form FaceSurface into the Metal SurfaceDef (fp32). A
// Bézier patch is expressed as a clamped B-spline of degree = nPoles-1.
inline cyber::metal::SurfaceDef toSurfaceDef(const topo::FaceSurface& s) {
  cyber::metal::SurfaceDef def;
  const bool bezier = s.kind == topo::FaceSurface::Kind::Bezier;
  def.degreeU = bezier ? s.nPolesU - 1 : s.degreeU;
  def.degreeV = bezier ? s.nPolesV - 1 : s.degreeV;
  def.numU = s.nPolesU;
  def.numV = s.nPolesV;
  def.control.reserve(static_cast<std::size_t>(s.nPolesU) * s.nPolesV);
  for (std::size_t i = 0; i < s.poles.size(); ++i) {
    const float w = s.weights.empty() ? 1.0f : static_cast<float>(s.weights[i]);
    def.control.push_back(cyber::metal::ControlPoint{
        static_cast<float>(s.poles[i].x), static_cast<float>(s.poles[i].y),
        static_cast<float>(s.poles[i].z), w});
  }
  if (bezier || s.knotsU.empty())
    def.knotsU = cyber::metal::makeClampedKnots(def.numU, def.degreeU);
  else
    for (double k : s.knotsU) def.knotsU.push_back(static_cast<float>(k));
  if (bezier || s.knotsV.empty())
    def.knotsV = cyber::metal::makeClampedKnots(def.numV, def.degreeV);
  else
    for (double k : s.knotsV) def.knotsV.push_back(static_cast<float>(k));
  return def;
}

}  // namespace detail

/// Sample an eligible face's patch on a (nU+1)×(nV+1) grid via the GPU (or the
/// Metal-module CPU reference when `backend` is null), applying the face's
/// Location to place results in world space. Returns nullopt if the surface is
/// not eligible or the evaluator errors — the caller then falls back to the fp64
/// CPU path.
inline std::optional<SampledGrid> sampleGrid(cyber::metal::MetalBackend* backend,
                                             const topo::FaceSurface& s,
                                             const topo::Location& loc, int nU, int nV) {
  cyber::metal::GridRequest req;
  req.gridU = nU + 1;
  req.gridV = nV + 1;
  req.computeNormals = true;
  auto res = cyber::metal::evaluateSurfaceGrid(backend, detail::toSurfaceDef(s), req);
  if (!res.ok()) return std::nullopt;
  const cyber::metal::SurfaceGrid& g = res.value();

  SampledGrid out;
  out.nU = g.gridU - 1;
  out.nV = g.gridV - 1;
  out.points.reserve(g.points.size());
  out.normals.reserve(g.points.size());
  const bool haveN = !g.normals.empty();
  for (std::size_t k = 0; k < g.points.size(); ++k) {
    math::Point3 p{g.points[k].x, g.points[k].y, g.points[k].z};
    if (!loc.isIdentity()) p = loc.transform().applyToPoint(p);
    out.points.push_back(p);
    if (haveN) {
      math::Vec3 n{g.normals[k].x, g.normals[k].y, g.normals[k].z};
      if (!loc.isIdentity()) n = loc.transform().applyToVector(n);
      out.normals.push_back(math::Dir3{n});
    } else {
      out.normals.push_back(math::Dir3{});
    }
  }
  return out;
}

#endif  // CYBERCAD_HAS_METAL

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_GPU_SAMPLE_H
