// GPU surface-grid evaluation implementation (Objective-C++, iOS-only).
//
// Built only when CYBERCAD_HAS_METAL is set (see CMakeLists.txt), with ARC and
// -framework Metal -framework Foundation. Objective-C / Metal types stay in this
// translation unit; the header (gpu_surface_eval.h) trades only plain C++.
//
// Two evaluators live here and MUST stay algorithmically identical so GPU output
// matches the CPU oracle within fp32 tolerance:
//   * the MSL kernel `cc_surface_eval` (runs on the "Apple iOS simulator GPU"),
//   * `evaluateSurfaceGridCPU` (plain C++ float, the parity oracle + fallback).
// Both implement Cox-de Boor basis evaluation with first derivatives (Piegl &
// Tiller, "The NURBS Book", FindSpan A2.1, DersBasisFuns A2.3, rational surface
// point/derivative A4.4). The rational (NURBS) case carries weights homogeneously;
// the non-rational case is w == 1 and reduces exactly.
//
// See gpu_surface_eval.h for the API contract, layouts, tolerance, and the
// determinism guarantee (one thread per sample, no scatter).

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "compute/metal/gpu_surface_eval.h"
#include "compute/metal/metal_backend.h"
#include "core/result.h"

namespace cyber::metal {
namespace {

// Scratch stride for the fixed-size per-thread basis tables: kMaxSurfaceDegree+1.
constexpr int kS = kMaxSurfaceDegree + 1;

// Parameters passed to the kernel as a single `constant` buffer. Field order and
// types MUST match `struct EvalParams` in the MSL source below.
struct EvalParams {
    std::int32_t degreeU;
    std::int32_t degreeV;
    std::int32_t numU;
    std::int32_t numV;
    std::int32_t gridU;
    std::int32_t gridV;
    std::int32_t computeNormals;
};

// ── MSL kernel ────────────────────────────────────────────────────────────────
// One thread per flattened grid sample (row-major, u-major). Fully independent
// writes => deterministic and reproducible run to run. kMaxDeg / kS below mirror
// the C++ kMaxSurfaceDegree / kS; keep them in sync.
constexpr const char* kSurfaceEvalKernel = R"MSL(
#include <metal_stdlib>
using namespace metal;

constant constexpr int kMaxDeg = 10;   // == cyber::metal::kMaxSurfaceDegree
constant constexpr int kS = 11;        // == kMaxDeg + 1

struct EvalParams {
    int degreeU; int degreeV; int numU; int numV; int gridU; int gridV; int computeNormals;
};

// Knot span index for parameter u over a clamped knot vector U with n+1 control
// points (n = num-1). Binary search per "The NURBS Book" A2.1.
static int findSpan(int n, int p, float u, device const float* U) {
    if (u >= U[n + 1]) return n;
    if (u <= U[p]) return p;
    int low = p, high = n + 1, mid = (low + high) / 2;
    while (u < U[mid] || u >= U[mid + 1]) {
        if (u < U[mid]) high = mid; else low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

// Nonzero basis functions N[0..p] and their first derivatives dN[0..p] at u.
// Cox-de Boor triangle (A2.2) + first-derivative pass (A2.3, n=1). Returns span.
static int basisDers(int num, int p, float u, device const float* U,
                     thread float* N, thread float* dN) {
    int span = findSpan(num - 1, p, u, U);
    float ndu[kS * kS];
    float left[kS];
    float right[kS];
    ndu[0] = 1.0f;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - U[span + 1 - j];
        right[j] = U[span + j] - u;
        float saved = 0.0f;
        for (int r = 0; r < j; ++r) {
            ndu[j * kS + r] = right[r + 1] + left[j - r];        // knot difference
            float temp = ndu[r * kS + (j - 1)] / ndu[j * kS + r];
            ndu[r * kS + j] = saved + right[r + 1] * temp;       // basis value
            saved = left[j - r] * temp;
        }
        ndu[j * kS + j] = saved;
    }
    for (int j = 0; j <= p; ++j) N[j] = ndu[j * kS + p];
    // First derivatives (A2.3 specialised to n = 1): the middle summation range is
    // empty at order 1, leaving the two boundary terms below.
    for (int r = 0; r <= p; ++r) {
        float d = 0.0f;
        int rk = r - 1, pk = p - 1;
        if (r >= 1) {
            float a0 = 1.0f / ndu[(pk + 1) * kS + rk];
            d += a0 * ndu[rk * kS + pk];
        }
        if (r <= pk) {
            float ak = -1.0f / ndu[(pk + 1) * kS + r];
            d += ak * ndu[r * kS + pk];
        }
        dN[r] = d * (float)p;   // factorial factor for the first derivative
    }
    return span;
}

kernel void cc_surface_eval(device const float4*  control [[buffer(0)]],
                            device const float*   knotsU  [[buffer(1)]],
                            device const float*   knotsV  [[buffer(2)]],
                            device const float*   paramsU [[buffer(3)]],
                            device const float*   paramsV [[buffer(4)]],
                            constant EvalParams&  P       [[buffer(5)]],
                            device packed_float3* outPts  [[buffer(6)]],
                            device packed_float3* outNrm  [[buffer(7)]],
                            uint gid [[thread_position_in_grid]]) {
    int total = P.gridU * P.gridV;
    if ((int)gid >= total) return;
    int i = (int)gid / P.gridV;   // u index (major)
    int j = (int)gid % P.gridV;   // v index
    float u = paramsU[i];
    float v = paramsV[j];

    float NU[kS], dNU[kS], NV[kS], dNV[kS];
    int su = basisDers(P.numU, P.degreeU, u, knotsU, NU, dNU) - P.degreeU;
    int sv = basisDers(P.numV, P.degreeV, v, knotsV, NV, dNV) - P.degreeV;

    // Accumulate the homogeneous point A and its u/v partials over the local
    // (degreeU+1)x(degreeV+1) control-point window in fixed order (determinism).
    float4 A = float4(0.0f), Au = float4(0.0f), Av = float4(0.0f);
    for (int a = 0; a <= P.degreeU; ++a) {
        for (int b = 0; b <= P.degreeV; ++b) {
            float4 cp = control[(su + a) * P.numV + (sv + b)];
            float4 hw = float4(cp.xyz * cp.w, cp.w);   // weighted homogeneous
            float nu = NU[a], nv = NV[b], dnu = dNU[a], dnv = dNV[b];
            A  += (nu * nv) * hw;
            Au += (dnu * nv) * hw;
            Av += (nu * dnv) * hw;
        }
    }
    float w = A.w;
    float3 S = A.xyz / w;
    outPts[gid] = packed_float3(S);
    if (P.computeNormals != 0) {
        // Quotient rule for the rational surface partials (A4.4), then the normal.
        float3 dSdu = (Au.xyz - S * Au.w) / w;
        float3 dSdv = (Av.xyz - S * Av.w) / w;
        float3 nrm = cross(dSdu, dSdv);
        float len = length(nrm);
        outNrm[gid] = packed_float3(len > 1e-12f ? nrm / len : float3(0.0f));
    }
}
)MSL";

// ── input validation ────────────────────────────────────────────────────────
Result<void> validate(const SurfaceDef& s, const GridRequest& r) {
    if (s.degreeU < 0 || s.degreeV < 0) {
        return make_error("surface_eval: negative degree");
    }
    if (s.degreeU > kMaxSurfaceDegree || s.degreeV > kMaxSurfaceDegree) {
        return make_error("surface_eval: degree exceeds kMaxSurfaceDegree");
    }
    if (s.numU < s.degreeU + 1 || s.numV < s.degreeV + 1) {
        return make_error("surface_eval: control points fewer than degree+1");
    }
    if (static_cast<int>(s.control.size()) != s.numU * s.numV) {
        return make_error("surface_eval: control.size() != numU*numV");
    }
    if (static_cast<int>(s.knotsU.size()) != s.numU + s.degreeU + 1 ||
        static_cast<int>(s.knotsV.size()) != s.numV + s.degreeV + 1) {
        return make_error("surface_eval: knot vector size mismatch");
    }
    if (r.gridU < 1 || r.gridV < 1) {
        return make_error("surface_eval: grid counts must be >= 1");
    }
    if ((!r.paramsU.empty() && static_cast<int>(r.paramsU.size()) != r.gridU) ||
        (!r.paramsV.empty() && static_cast<int>(r.paramsV.size()) != r.gridV)) {
        return make_error("surface_eval: explicit params size != grid count");
    }
    return Result<void>::ok();
}

// Sample parameters: explicit if supplied, else `gridN` values uniform over the
// clamped domain [knots[degree], knots[num]] with an exact end sample.
std::vector<float> buildParams(const std::vector<float>& provided,
                               const std::vector<float>& knots, int degree,
                               int num, int gridN) {
    if (!provided.empty()) {
        return provided;
    }
    const float lo = knots[degree];
    const float hi = knots[num];
    std::vector<float> p(static_cast<std::size_t>(gridN), lo);
    if (gridN > 1) {
        for (int i = 0; i < gridN; ++i) {
            p[i] = lo + (hi - lo) * (static_cast<float>(i) / static_cast<float>(gridN - 1));
        }
        p[gridN - 1] = hi;
    }
    return p;
}

// ── CPU reference: mirror of the MSL kernel in float ──────────────────────────
using Basis = std::array<float, kS>;

int cpuFindSpan(int n, int p, float u, const float* U) {
    if (u >= U[n + 1]) return n;
    if (u <= U[p]) return p;
    int low = p, high = n + 1, mid = (low + high) / 2;
    while (u < U[mid] || u >= U[mid + 1]) {
        if (u < U[mid]) high = mid; else low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

int cpuBasisDers(int num, int p, float u, const float* U, Basis& N, Basis& dN) {
    const int span = cpuFindSpan(num - 1, p, u, U);
    std::array<float, kS * kS> ndu{};
    std::array<float, kS> left{};
    std::array<float, kS> right{};
    ndu[0] = 1.0f;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - U[span + 1 - j];
        right[j] = U[span + j] - u;
        float saved = 0.0f;
        for (int r = 0; r < j; ++r) {
            ndu[j * kS + r] = right[r + 1] + left[j - r];
            const float temp = ndu[r * kS + (j - 1)] / ndu[j * kS + r];
            ndu[r * kS + j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j * kS + j] = saved;
    }
    for (int j = 0; j <= p; ++j) N[j] = ndu[j * kS + p];
    for (int r = 0; r <= p; ++r) {
        float d = 0.0f;
        const int rk = r - 1, pk = p - 1;
        if (r >= 1) d += (1.0f / ndu[(pk + 1) * kS + rk]) * ndu[rk * kS + pk];
        if (r <= pk) d += (-1.0f / ndu[(pk + 1) * kS + r]) * ndu[r * kS + pk];
        dN[r] = d * static_cast<float>(p);
    }
    return span;
}

// One rational surface sample: fills point (and normal when requested), mirroring
// the kernel's accumulation and quotient-rule normal exactly.
void cpuSample(const SurfaceDef& s, float u, float v, bool wantNormal,
               Vec3f& outPt, Vec3f& outNrm) {
    Basis NU{}, dNU{}, NV{}, dNV{};
    const int su = cpuBasisDers(s.numU, s.degreeU, u, s.knotsU.data(), NU, dNU) - s.degreeU;
    const int sv = cpuBasisDers(s.numV, s.degreeV, v, s.knotsV.data(), NV, dNV) - s.degreeV;

    float A[4] = {0, 0, 0, 0}, Au[4] = {0, 0, 0, 0}, Av[4] = {0, 0, 0, 0};
    for (int a = 0; a <= s.degreeU; ++a) {
        for (int b = 0; b <= s.degreeV; ++b) {
            const ControlPoint& cp = s.control[(su + a) * s.numV + (sv + b)];
            const float hw[4] = {cp.x * cp.w, cp.y * cp.w, cp.z * cp.w, cp.w};
            const float nu = NU[a], nv = NV[b], dnu = dNU[a], dnv = dNV[b];
            for (int c = 0; c < 4; ++c) {
                A[c] += (nu * nv) * hw[c];
                Au[c] += (dnu * nv) * hw[c];
                Av[c] += (nu * dnv) * hw[c];
            }
        }
    }
    const float w = A[3];
    const float sx = A[0] / w, sy = A[1] / w, sz = A[2] / w;
    outPt = {sx, sy, sz};
    if (!wantNormal) {
        return;
    }
    const float dux = (Au[0] - sx * Au[3]) / w, duy = (Au[1] - sy * Au[3]) / w,
                duz = (Au[2] - sz * Au[3]) / w;
    const float dvx = (Av[0] - sx * Av[3]) / w, dvy = (Av[1] - sy * Av[3]) / w,
                dvz = (Av[2] - sz * Av[3]) / w;
    const float nx = duy * dvz - duz * dvy;
    const float ny = duz * dvx - dux * dvz;
    const float nz = dux * dvy - duy * dvx;
    const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    outNrm = (len > 1e-12f) ? Vec3f{nx / len, ny / len, nz / len} : Vec3f{0, 0, 0};
}

// ── GPU dispatch helper ───────────────────────────────────────────────────────
// Allocates the shared buffers, compiles + dispatches the kernel once, and copies
// the results back. Split out so evaluateSurfaceGridGPU stays readable.
Result<SurfaceGrid> runGpu(MetalBackend& backend, const SurfaceDef& s,
                           const GridRequest& r, const std::vector<float>& pu,
                           const std::vector<float>& pv) {
    const int n = r.gridU * r.gridV;
    EvalParams params{s.degreeU, s.degreeV, s.numU, s.numV,
                      r.gridU,   r.gridV,   r.computeNormals ? 1 : 0};

    BufferHandle hCtrl = backend.makeSharedBuffer(s.control.data(),
                                                  s.control.size() * sizeof(ControlPoint));
    BufferHandle hKU = backend.makeSharedBuffer(s.knotsU.data(), s.knotsU.size() * sizeof(float));
    BufferHandle hKV = backend.makeSharedBuffer(s.knotsV.data(), s.knotsV.size() * sizeof(float));
    BufferHandle hPU = backend.makeSharedBuffer(pu.data(), pu.size() * sizeof(float));
    BufferHandle hPV = backend.makeSharedBuffer(pv.data(), pv.size() * sizeof(float));
    BufferHandle hP = backend.makeSharedBuffer(&params, sizeof(params));
    BufferHandle hPts = backend.makeSharedBuffer(static_cast<std::size_t>(n) * sizeof(Vec3f));
    BufferHandle hNrm = backend.makeSharedBuffer(static_cast<std::size_t>(n) * sizeof(Vec3f));
    if (!hCtrl || !hKU || !hKV || !hPU || !hPV || !hP || !hPts || !hNrm) {
        return make_error("surface_eval: shared buffer allocation failed");
    }

    auto pso = backend.compilePipeline(kSurfaceEvalKernel, "cc_surface_eval");
    if (!pso) {
        return pso.error();
    }
    auto disp = backend.dispatch(pso.value(), {hCtrl, hKU, hKV, hPU, hPV, hP, hPts, hNrm},
                                 static_cast<std::size_t>(n));
    if (!disp) {
        return disp.error();
    }

    SurfaceGrid grid;
    grid.gridU = r.gridU;
    grid.gridV = r.gridV;
    grid.points.resize(static_cast<std::size_t>(n));
    const auto* pts = static_cast<const Vec3f*>(backend.bufferContents(hPts));
    if (pts == nullptr) {
        return make_error("surface_eval: point buffer readback failed");
    }
    for (int i = 0; i < n; ++i) grid.points[i] = pts[i];
    if (r.computeNormals) {
        grid.normals.resize(static_cast<std::size_t>(n));
        const auto* nrm = static_cast<const Vec3f*>(backend.bufferContents(hNrm));
        if (nrm == nullptr) {
            return make_error("surface_eval: normal buffer readback failed");
        }
        for (int i = 0; i < n; ++i) grid.normals[i] = nrm[i];
    }
    return grid;
}

}  // namespace

std::vector<float> makeClampedKnots(int numCtrl, int degree) {
    std::vector<float> k;
    if (numCtrl < degree + 1 || degree < 0) {
        return k;  // invalid; caller's validate() reports it
    }
    const int m = numCtrl + degree + 1;
    k.resize(static_cast<std::size_t>(m));
    const int interior = numCtrl - degree - 1;  // internal knots
    for (int i = 0; i <= degree; ++i) k[i] = 0.0f;
    for (int i = 1; i <= interior; ++i) {
        k[degree + i] = static_cast<float>(i) / static_cast<float>(interior + 1);
    }
    for (int i = 0; i <= degree; ++i) k[m - 1 - i] = 1.0f;
    return k;
}

Result<SurfaceGrid> evaluateSurfaceGridCPU(const SurfaceDef& surf, const GridRequest& req) {
    if (auto v = validate(surf, req); !v) {
        return v.error();
    }
    const std::vector<float> pu =
        buildParams(req.paramsU, surf.knotsU, surf.degreeU, surf.numU, req.gridU);
    const std::vector<float> pv =
        buildParams(req.paramsV, surf.knotsV, surf.degreeV, surf.numV, req.gridV);

    SurfaceGrid grid;
    grid.gridU = req.gridU;
    grid.gridV = req.gridV;
    const int n = req.gridU * req.gridV;
    grid.points.resize(static_cast<std::size_t>(n));
    if (req.computeNormals) {
        grid.normals.resize(static_cast<std::size_t>(n));
    }
    for (int i = 0; i < req.gridU; ++i) {
        for (int j = 0; j < req.gridV; ++j) {
            const int idx = i * req.gridV + j;  // row-major, u-major (matches GPU)
            Vec3f nrm{};
            cpuSample(surf, pu[i], pv[j], req.computeNormals, grid.points[idx], nrm);
            if (req.computeNormals) {
                grid.normals[idx] = nrm;
            }
        }
    }
    return grid;
}

Result<SurfaceGrid> evaluateSurfaceGridGPU(MetalBackend& backend, const SurfaceDef& surf,
                                           const GridRequest& req) {
    if (auto v = validate(surf, req); !v) {
        return v.error();
    }
    const std::vector<float> pu =
        buildParams(req.paramsU, surf.knotsU, surf.degreeU, surf.numU, req.gridU);
    const std::vector<float> pv =
        buildParams(req.paramsV, surf.knotsV, surf.degreeV, surf.numV, req.gridV);
    return runGpu(backend, surf, req, pu, pv);
}

Result<SurfaceGrid> evaluateSurfaceGrid(MetalBackend* backend, const SurfaceDef& surf,
                                        const GridRequest& req) {
    if (backend != nullptr) {
        return evaluateSurfaceGridGPU(*backend, surf, req);
    }
    return evaluateSurfaceGridCPU(surf, req);
}

}  // namespace cyber::metal
