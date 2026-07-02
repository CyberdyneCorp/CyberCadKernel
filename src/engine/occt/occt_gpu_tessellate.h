#ifndef CYBERCADKERNEL_ENGINE_OCCT_OCCT_GPU_TESSELLATE_H
#define CYBERCADKERNEL_ENGINE_OCCT_OCCT_GPU_TESSELLATE_H

// GPU-backed tessellation for the OCCT adapter — per-face GPU-eligibility
// classification + regular-grid triangulation, wired into cc_tessellate /
// cc_face_meshes (occt_tessellate.cpp).
//
// This is an OCCT-INTERNAL header (it names TopoDS_Face), so it is included only
// by the OCCT adapter TUs. The GPU face-eval entry point tryTessellateFaceGPU()
// is compiled ONLY under CYBERCAD_HAS_METAL: it converts a face's surface to a
// cyber::metal::SurfaceDef and evaluates a (u,v) grid through the
// gpu_surface_eval module (Metal when a device is present, CPU reference
// otherwise). Without Metal the whole GPU path is absent and occt_tessellate.cpp
// meshes every face with OCCT exactly as before.
//
// TOPOLOGY STAYS ON THE CPU: this module decides eligibility, trimming rejection,
// grid-to-triangle connectivity and orientation; the GPU only supplies fp32
// sample points. The exact fp64 modeling core is never touched by this path.

#include <vector>

#include "engine/IEngine.h"          // MeshData
#include "engine/occt/occt_engine.h"  // OCCT core (TopoDS_Face, gp_*, BRep_Tool)

namespace cyber {
namespace occt {

#ifdef CYBERCAD_HAS_METAL

// Attempt to mesh one face on the GPU surface-evaluation grid path.
//
// A face is GPU-ELIGIBLE only when it is provably equivalent to a GPU-evaluated
// regular grid: exactly one wire (a single outer wire, no inner wires/holes), a
// 2D boundary that equals the BRepTools::UVBounds rectangle within tolerance (an
// untrimmed rectangular patch), and a surface convertible to a supported
// cyber::metal::SurfaceDef (via Geom_RectangularTrimmedSurface +
// GeomConvert::SurfaceToBSplineSurface, degree <= kMaxSurfaceDegree).
//
// On eligibility, evaluates a UxV grid (density derived from `deflection`) and
// appends its regular-grid triangulation to `vertices` / `triangles` with the
// given `baseVertex` offset, matching the OCCT winding convention (winding is
// flipped for a REVERSED face), then returns true.
//
// Returns false and appends NOTHING for any non-eligible face (holes, trimmed,
// unsupported/unconvertible surface, degree too high) or on any conversion/eval
// failure — the caller MUST fall back to OCCT BRepMesh for that face. When in
// doubt, this returns false (fall back).
bool tryTessellateFaceGPU(const TopoDS_Face& face, double deflection,
                          std::vector<double>& vertices, std::vector<int>& triangles,
                          int baseVertex);

#endif  // CYBERCAD_HAS_METAL

}  // namespace occt
}  // namespace cyber

#endif  // CYBERCADKERNEL_ENGINE_OCCT_OCCT_GPU_TESSELLATE_H
