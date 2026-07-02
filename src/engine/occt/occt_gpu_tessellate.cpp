// OCCT engine adapter — GPU tessellation path (per-face eligibility + grid mesh).
//
// Wires the EXISTING GPU surface-evaluation module (src/compute/metal/
// gpu_surface_eval.*) into cc_tessellate / cc_face_meshes. Two concerns live here:
//
//   1. Per-thread diagnostics (gpu_tess_stats.h) — ALWAYS compiled (whenever the
//      OCCT adapter is built), so a test linking the OCCT slice can read how many
//      faces took each path even on a non-Metal build (where gpuFaces stays 0).
//
//   2. The GPU face-eval path (tryTessellateFaceGPU) — compiled ONLY under
//      CYBERCAD_HAS_METAL. It classifies a face for GPU-eligibility, converts an
//      eligible face's surface to a cyber::metal::SurfaceDef, evaluates a (u,v)
//      grid via evaluateSurfaceGrid (Metal device when present, CPU reference
//      otherwise — both fp32), and triangulates the regular grid on the CPU.
//
// TOPOLOGY STAYS ON THE CPU: eligibility, trimming rejection, grid connectivity
// and winding are all decided here; the GPU only fills fp32 sample positions. The
// exact fp64 modeling core is untouched — these fp32 samples feed the display
// mesh only.

#include "engine/occt/occt_gpu_tessellate.h"
#include "engine/occt/gpu_tess_stats.h"

#include <atomic>

namespace cyber {
namespace occt {

// ── Diagnostics (always compiled) ─────────────────────────────────────────────
// Process-wide counters: a tessellation runs on a scheduler worker thread and the
// tally is read on the calling thread after the op completes (which synchronizes
// the worker's writes), so per-thread storage would be invisible to the caller.
namespace {
std::atomic<int> g_gpuFaces{0};
std::atomic<int> g_fallbackFaces{0};
}  // namespace

GpuTessStats gpuTessStats() {
    GpuTessStats s;
    s.gpuFaces = g_gpuFaces.load(std::memory_order_relaxed);
    s.fallbackFaces = g_fallbackFaces.load(std::memory_order_relaxed);
    return s;
}

void resetGpuTessStats() {
    g_gpuFaces.store(0, std::memory_order_relaxed);
    g_fallbackFaces.store(0, std::memory_order_relaxed);
}

void recordGpuTessFace(bool viaGpu) {
    if (viaGpu) {
        g_gpuFaces.fetch_add(1, std::memory_order_relaxed);
    } else {
        g_fallbackFaces.fetch_add(1, std::memory_order_relaxed);
    }
}

}  // namespace occt
}  // namespace cyber

// ── GPU face-eval path (Metal builds only) ────────────────────────────────────
#ifdef CYBERCAD_HAS_METAL

#include <algorithm>
#include <cmath>
#include <vector>

#include "compute/metal/gpu_surface_eval.h"
#include "compute/metal/metal_backend.h"

// OCCT geometry headers this path needs on top of the adapter core (occt_engine.h).
#include <BRepTools.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_Surface.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt2d.hxx>

namespace cyber {
namespace occt {
namespace {

using cyber::metal::ControlPoint;
using cyber::metal::GridRequest;
using cyber::metal::SurfaceDef;
using cyber::metal::SurfaceGrid;

constexpr double kPi = 3.14159265358979323846;

// Grid segment-count clamps. A floor keeps even a flat quad smooth enough to
// stitch; a ceiling bounds the per-face vertex budget for the display mesh.
constexpr int kMinSeg = 8;
constexpr int kMaxSeg = 256;

// The single Metal backend used by GPU tessellation, created lazily. Null when no
// Metal device is available (e.g. a headless CI slice) — evaluateSurfaceGrid then
// runs the identical fp32 CPU reference, so the mesh is unchanged either way.
cyber::metal::MetalBackend* gpuBackend() {
    static std::shared_ptr<cyber::metal::MetalBackend> backend =
        cyber::metal::MetalBackend::create();
    return backend.get();
}

// Derive a segment count for one parametric direction from the control-net
// polygon length and the requested deflection. Approximates the net length as a
// circular arc of radius r ≈ len/(2π) and picks a chord whose sagitta ≈ deflection
// (c ≈ 2√(2·r·d)); segments = len/c, clamped. This is a display-mesh heuristic —
// exact geometry never depends on it — and is monotonic in deflection.
int gridSegments(double netLength, double deflection) {
    if (netLength <= 0.0 || deflection <= 0.0) {
        return kMinSeg;
    }
    const double r = netLength / (2.0 * kPi);
    const double chord = 2.0 * std::sqrt(std::max(2.0 * r * deflection, 1e-12));
    const int segs = static_cast<int>(std::ceil(netLength / std::max(chord, 1e-9)));
    return std::clamp(segs, kMinSeg, kMaxSeg);
}

// Distance between two control-point positions (weights ignored — these are the
// geometric poles).
double poleDistance(const ControlPoint& a, const ControlPoint& b) {
    const double dx = static_cast<double>(a.x) - b.x;
    const double dy = static_cast<double>(a.y) - b.y;
    const double dz = static_cast<double>(a.z) - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ── eligibility: untrimmed rectangular patch ──────────────────────────────────
//
// A face is GPU-eligible only when its (single) boundary wire coincides with its
// own UVBounds rectangle: exactly one wire, and every boundary edge's 2D pcurve
// lies on one of the four rectangle sides (u==u0|u1 or v==v0|v1) within tolerance.
// A hole (inner wire) fails the wire count; a trimmed boundary (circular/diagonal)
// has pcurve points interior to the UV bbox and fails the on-boundary test. A null
// pcurve (cannot be verified) also fails — when in doubt, fall back.
bool isUntrimmedRectangularFace(const TopoDS_Face& face, double u0, double u1, double v0,
                                double v1) {
    // Exactly one wire → single outer wire, no inner wires (holes).
    int wireCount = 0;
    for (TopExp_Explorer wexp(face, TopAbs_WIRE); wexp.More(); wexp.Next()) {
        ++wireCount;
    }
    if (wireCount != 1) {
        return false;
    }

    const double uSpan = u1 - u0;
    const double vSpan = v1 - v0;
    if (uSpan <= 0.0 || vSpan <= 0.0) {
        return false;
    }
    // On-boundary tolerance: a small fraction of the larger UV span. Real trims
    // deviate by a large fraction of the span, so this rejects them while
    // tolerating pcurve approximation on a genuine isoparametric boundary.
    const double uTol = 1e-3 * uSpan;
    const double vTol = 1e-3 * vSpan;

    constexpr int kSamples = 8;  // interior samples per edge (endpoints included)
    for (TopExp_Explorer eexp(face, TopAbs_EDGE); eexp.More(); eexp.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(eexp.Current());
        Standard_Real first = 0.0, last = 0.0;
        Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);
        if (pcurve.IsNull() || !(last > first)) {
            return false;  // cannot verify this edge lies on the rectangle → fall back
        }
        for (int k = 0; k <= kSamples; ++k) {
            const Standard_Real t = first + (last - first) * (static_cast<double>(k) / kSamples);
            const gp_Pnt2d p = pcurve->Value(t);
            const double du = std::min(std::fabs(p.X() - u0), std::fabs(p.X() - u1));
            const double dv = std::min(std::fabs(p.Y() - v0), std::fabs(p.Y() - v1));
            // The point must sit on a u-isoline OR a v-isoline of the rectangle.
            if (du > uTol && dv > vTol) {
                return false;
            }
        }
    }
    return true;
}

// ── surface conversion: Geom_Surface -> cyber::metal::SurfaceDef ──────────────
//
// Trim the (possibly unbounded) face surface to its UVBounds rectangle and convert
// to a clamped B-spline; copy poles (transformed by the face location into world
// coordinates), homogeneous weights, and the flat clamped knot sequences into a
// SurfaceDef. Returns false for a null/unconvertible surface or a degree above
// kMaxSurfaceDegree. Any OCCT Standard_Failure is caught by the caller.
bool buildSurfaceDef(const TopoDS_Face& face, double u0, double u1, double v0, double v1,
                     SurfaceDef& out) {
    TopLoc_Location loc;
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face, loc);
    if (surf.IsNull()) {
        return false;
    }
    Handle(Geom_RectangularTrimmedSurface) trimmed =
        new Geom_RectangularTrimmedSurface(surf, u0, u1, v0, v1);
    Handle(Geom_BSplineSurface) bs = GeomConvert::SurfaceToBSplineSurface(trimmed);
    if (bs.IsNull()) {
        return false;
    }

    const int degreeU = bs->UDegree();
    const int degreeV = bs->VDegree();
    if (degreeU < 1 || degreeV < 1 || degreeU > cyber::metal::kMaxSurfaceDegree ||
        degreeV > cyber::metal::kMaxSurfaceDegree) {
        return false;
    }
    const int numU = bs->NbUPoles();
    const int numV = bs->NbVPoles();

    const gp_Trsf& trsf = loc.Transformation();
    out.degreeU = degreeU;
    out.degreeV = degreeV;
    out.numU = numU;
    out.numV = numV;
    out.control.resize(static_cast<std::size_t>(numU) * numV);
    for (int i = 1; i <= numU; ++i) {
        for (int j = 1; j <= numV; ++j) {
            const gp_Pnt p = bs->Pole(i, j).Transformed(trsf);
            ControlPoint& cp = out.control[static_cast<std::size_t>(i - 1) * numV + (j - 1)];
            cp.x = static_cast<float>(p.X());
            cp.y = static_cast<float>(p.Y());
            cp.z = static_cast<float>(p.Z());
            cp.w = static_cast<float>(bs->Weight(i, j));  // 1.0 for non-rational
        }
    }

    // Flat (expanded) clamped knot sequences: size == num + degree + 1, exactly
    // what SurfaceDef / the Cox-de Boor evaluator expect.
    const TColStd_Array1OfReal& uk = bs->UKnotSequence();
    const TColStd_Array1OfReal& vk = bs->VKnotSequence();
    out.knotsU.clear();
    out.knotsU.reserve(static_cast<std::size_t>(uk.Length()));
    for (int i = uk.Lower(); i <= uk.Upper(); ++i) {
        out.knotsU.push_back(static_cast<float>(uk.Value(i)));
    }
    out.knotsV.clear();
    out.knotsV.reserve(static_cast<std::size_t>(vk.Length()));
    for (int i = vk.Lower(); i <= vk.Upper(); ++i) {
        out.knotsV.push_back(static_cast<float>(vk.Value(i)));
    }
    return true;
}

// Append the regular-grid triangulation of `grid` to the target buffers with the
// given base-vertex offset, matching appendFaceTriangulation's winding: forward
// winding follows +u×+v (outward for a FORWARD face); a REVERSED face flips each
// triangle's last two indices so triangles still face outward.
void appendGridTriangulation(const SurfaceGrid& grid, bool reversed,
                             std::vector<double>& vertices, std::vector<int>& triangles,
                             int baseVertex) {
    const int gu = grid.gridU;
    const int gv = grid.gridV;
    vertices.reserve(vertices.size() + static_cast<std::size_t>(gu) * gv * 3);
    for (const cyber::metal::Vec3f& p : grid.points) {
        vertices.push_back(p.x);
        vertices.push_back(p.y);
        vertices.push_back(p.z);
    }
    const auto pushTri = [&](int a, int b, int c) {
        if (reversed) {
            std::swap(b, c);
        }
        triangles.push_back(a);
        triangles.push_back(b);
        triangles.push_back(c);
    };
    triangles.reserve(triangles.size() + static_cast<std::size_t>(gu - 1) * (gv - 1) * 6);
    for (int i = 0; i < gu - 1; ++i) {
        for (int j = 0; j < gv - 1; ++j) {
            const int p00 = baseVertex + i * gv + j;
            const int p10 = baseVertex + (i + 1) * gv + j;
            const int p11 = baseVertex + (i + 1) * gv + (j + 1);
            const int p01 = baseVertex + i * gv + (j + 1);
            pushTri(p00, p10, p11);
            pushTri(p00, p11, p01);
        }
    }
}

}  // namespace

bool tryTessellateFaceGPU(const TopoDS_Face& face, double deflection,
                          std::vector<double>& vertices, std::vector<int>& triangles,
                          int baseVertex) {
    // Any OCCT Standard_Failure during classification/conversion means this face
    // is not safely GPU-eligible — swallow it and fall back to OCCT for the face
    // (never let it abort the whole-body tessellation).
    try {
        double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
        BRepTools::UVBounds(face, u0, u1, v0, v1);
        if (!isUntrimmedRectangularFace(face, u0, u1, v0, v1)) {
            return false;
        }
        SurfaceDef surf;
        if (!buildSurfaceDef(face, u0, u1, v0, v1, surf)) {
            return false;
        }

        // Grid density from the control-net extent + deflection (topology on CPU).
        double uLen = 0.0;
        for (int i = 0; i + 1 < surf.numU; ++i) {
            double best = 0.0;
            for (int j = 0; j < surf.numV; ++j) {
                best = std::max(best,
                                poleDistance(surf.control[static_cast<std::size_t>(i) * surf.numV + j],
                                             surf.control[static_cast<std::size_t>(i + 1) * surf.numV + j]));
            }
            uLen += best;
        }
        double vLen = 0.0;
        for (int j = 0; j + 1 < surf.numV; ++j) {
            double best = 0.0;
            for (int i = 0; i < surf.numU; ++i) {
                best = std::max(best,
                                poleDistance(surf.control[static_cast<std::size_t>(i) * surf.numV + j],
                                             surf.control[static_cast<std::size_t>(i) * surf.numV + (j + 1)]));
            }
            vLen += best;
        }

        GridRequest req;
        req.gridU = gridSegments(uLen, deflection) + 1;  // samples = segments + 1
        req.gridV = gridSegments(vLen, deflection) + 1;
        req.computeNormals = false;  // CCMesh carries no normals; positions suffice

        auto res = cyber::metal::evaluateSurfaceGrid(gpuBackend(), surf, req);
        if (!res) {
            return false;  // unrepresentable (validation) or GPU failure → fall back
        }
        const SurfaceGrid& grid = res.value();
        if (grid.gridU < 2 || grid.gridV < 2 ||
            static_cast<int>(grid.points.size()) != grid.gridU * grid.gridV) {
            return false;
        }

        const bool reversed = (face.Orientation() == TopAbs_REVERSED);
        appendGridTriangulation(grid, reversed, vertices, triangles, baseVertex);
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

}  // namespace occt
}  // namespace cyber

#endif  // CYBERCAD_HAS_METAL
