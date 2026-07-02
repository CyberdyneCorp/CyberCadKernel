#ifndef CYBERCADKERNEL_COMPUTE_METAL_GPU_SURFACE_EVAL_H
#define CYBERCADKERNEL_COMPUTE_METAL_GPU_SURFACE_EVAL_H

// GPU surface-grid evaluation (Phase 2, capability `gpu-tessellation`).
//
// Evaluates a tensor-product NURBS / B-spline / Bézier surface on a regular
// parameter grid `(u_i, v_j)` — producing an fp32 grid of surface POINTS and,
// optionally, an fp32 grid of unit NORMALS — on the Metal compute backend
// (`ComputeKind::SurfaceEval`). One GPU thread evaluates one grid sample via the
// Cox-de Boor basis functions and their analytic first derivatives; the normal is
// `normalize(dS/du x dS/dv)` from the same evaluation (no finite differencing).
//
// This is the pure NUMERIC contribution of the GPU: it fills per-sample fields
// that a CPU triangulator then consumes. All TOPOLOGY — trimming, grid-to-triangle
// connectivity, cross-face stitching — stays on the CPU (the source of truth). The
// exact fp64 surface definition also stays on the CPU; the GPU works in fp32 only.
//
// Determinism: the grid is decomposed row-major (u-major) and each sample is
// written by exactly one thread with no shared-memory scatter, so repeated GPU
// runs on the same input are bit-identical (see `evaluateSurfaceGridGPU`).
//
// The Cox-de Boor evaluator (CPU reference here, MSL kernel in the .mm) is a
// standard, irreducible NURBS algorithm (Piegl & Tiller, "The NURBS Book",
// A2.2/A2.3/A4.4); it is isolated and documented rather than split to hit a
// complexity number.
//
// This header compiles as plain C++20 (Metal types stay in the .mm). The
// implementation lives in gpu_surface_eval.mm and is only built when
// CYBERCAD_HAS_METAL is set. `evaluateSurfaceGridCPU` is the parity oracle and the
// CPU fallback used when no Metal backend is available.

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "compute/metal/metal_backend.h"
#include "core/result.h"

namespace cyber::metal {

// Largest surface degree (in either parameter direction) the GPU kernel supports.
// The MSL evaluator uses fixed-size per-thread scratch sized for this cap; a
// higher degree is rejected by the input validation rather than truncated. Covers
// all practical CAD surfaces (cubic B-splines, high-order Bézier patches).
inline constexpr int kMaxSurfaceDegree = 10;

// Documented fp32 parity tolerance for a GPU grid vs the CPU reference, applied as
// `|a-b| <= kAbs + kRel*|b|` per component. Kept well within a typical meshing
// deflection so fp32 sampling never coarsens the visible mesh. Positions are in
// model units (mm); normals are unit vectors.
inline constexpr float kSurfaceEvalAbsTol = 1e-4f;
inline constexpr float kSurfaceEvalRelTol = 1e-4f;

// Tightly packed fp32 3-vector. Layout matches MSL `packed_float3` (12 bytes, no
// padding) so a grid buffer round-trips between CPU and GPU without repacking.
struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// A homogeneous control point: position (x, y, z) plus weight w. w == 1 for a
// non-rational (polynomial) B-spline / Bézier surface; w != 1 makes the patch
// rational (NURBS). Layout matches MSL `float4` (16 bytes, 16-byte aligned).
struct ControlPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

// A tensor-product NURBS / B-spline / Bézier surface.
//
// Control points are row-major with u as the major axis:
//   control[i * numV + j]  is the (i, j) control point,
//   i in [0, numU) indexing the u direction, j in [0, numV) indexing v.
//
// Knot vectors are clamped and non-decreasing:
//   knotsU.size() == numU + degreeU + 1,  knotsV.size() == numV + degreeV + 1.
// A Bézier patch is the special case numU == degreeU+1 with knots [0..0, 1..1]
// (use makeClampedKnots()).
struct SurfaceDef {
    int degreeU = 0;
    int degreeV = 0;
    int numU = 0;  // control points in u; must be >= degreeU + 1
    int numV = 0;  // control points in v; must be >= degreeV + 1
    std::vector<ControlPoint> control;  // numU * numV, row-major (u-major)
    std::vector<float> knotsU;          // numU + degreeU + 1, clamped
    std::vector<float> knotsV;          // numV + degreeV + 1, clamped
};

// What to sample. Grid counts are per direction (>= 1). If paramsU / paramsV are
// provided they are used verbatim (must have gridU / gridV entries and lie in the
// knot domain); otherwise the sampler uses `gridN` values spread uniformly across
// the clamped domain [knots[degree], knots[num]].
struct GridRequest {
    int gridU = 2;
    int gridV = 2;
    bool computeNormals = true;
    std::vector<float> paramsU;  // empty => uniform over the u domain
    std::vector<float> paramsV;  // empty => uniform over the v domain
};

// The evaluated grid. `points` (and `normals`, when requested) are row-major with
// u as the major axis, matching the control net: index = i * gridV + j.
struct SurfaceGrid {
    int gridU = 0;
    int gridV = 0;
    std::vector<Vec3f> points;   // gridU * gridV
    std::vector<Vec3f> normals;  // gridU * gridV, or empty when normals not requested
};

// Build a clamped, uniformly-spaced knot vector on [0, 1] for `numCtrl` control
// points of the given `degree` (size numCtrl + degree + 1). With
// numCtrl == degree + 1 this yields the Bézier knot vector [0..0, 1..1].
std::vector<float> makeClampedKnots(int numCtrl, int degree);

// GPU evaluation on the given Metal backend. Uploads the control net, knots, and
// sample parameters into shared buffers, dispatches the Cox-de Boor kernel once
// over the flattened grid (`ComputeKind::SurfaceEval`), and reads back the point
// (and optional normal) grid. Deterministic: one thread per sample, no scatter.
// Returns an Error for invalid input or a Metal failure (never crashes).
Result<SurfaceGrid> evaluateSurfaceGridGPU(MetalBackend& backend,
                                           const SurfaceDef& surf,
                                           const GridRequest& req);

// CPU reference evaluation using the identical fp32 algorithm — the parity oracle
// for the GPU kernel and the fallback when no Metal backend is available. Always
// available (plain C++, no Metal), so this compiles and runs on the host too.
Result<SurfaceGrid> evaluateSurfaceGridCPU(const SurfaceDef& surf,
                                           const GridRequest& req);

// Routing + fallback: evaluate on `backend` when it is non-null (GPU path),
// otherwise on the CPU reference. Lets callers express "GPU when a backend is
// active, CPU otherwise" without branching on backend internals.
Result<SurfaceGrid> evaluateSurfaceGrid(MetalBackend* backend,
                                        const SurfaceDef& surf,
                                        const GridRequest& req);

// Component-wise fp32 parity test using the documented tolerances above. Handy for
// GPU-vs-CPU parity assertions in the test suite.
inline bool approxEqual(const Vec3f& a, const Vec3f& b) {
    auto ok = [](float x, float y) {
        return std::fabs(x - y) <= kSurfaceEvalAbsTol + kSurfaceEvalRelTol * std::fabs(y);
    };
    return ok(a.x, b.x) && ok(a.y, b.y) && ok(a.z, b.z);
}

}  // namespace cyber::metal

#endif  // CYBERCADKERNEL_COMPUTE_METAL_GPU_SURFACE_EVAL_H
