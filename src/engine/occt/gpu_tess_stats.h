#ifndef CYBERCADKERNEL_ENGINE_OCCT_GPU_TESS_STATS_H
#define CYBERCADKERNEL_ENGINE_OCCT_GPU_TESS_STATS_H

// GPU-tessellation diagnostics (NOT part of the public C ABI).
//
// The OCCT tessellate / face_meshes paths record, per call, how many faces were
// meshed on the GPU surface-evaluation grid path versus the OCCT BRepMesh
// fallback. A GPU parity test can read the last call's tally to assert that the
// GPU path was actually exercised (gpuFaces > 0) rather than silently falling
// back to OCCT for everything.
//
// The tally is PROCESS-WIDE (atomic counters), not per-thread, because a
// tessellation runs on an operation-scheduler worker thread while the tally is
// read back on the calling thread after the op completes; the op's completion
// synchronizes the worker's writes with the caller's read. It reflects the most
// recent tessellation and is not meaningful under concurrent tessellations.
//
// This header is deliberately OCCT-FREE and Metal-FREE: it carries only a POD
// tally and three plain-C++ accessors, so a test that links the OCCT slice can
// include it without pulling in any OpenCASCADE header. The definitions live in
// occt_gpu_tessellate.cpp (compiled whenever CYBERCAD_HAS_OCCT is set); when the
// build has no Metal backend the counters simply stay at gpuFaces == 0.

namespace cyber {
namespace occt {

// Faces routed to each meshing path in the most recent tessellate()/face_meshes()
// on the calling thread. gpuFaces is always 0 when GPU tessellation is OFF or on
// a non-Metal build.
struct GpuTessStats {
    int gpuFaces = 0;       // faces meshed from a GPU-evaluated (u,v) grid
    int fallbackFaces = 0;  // faces meshed by OCCT BRepMesh_IncrementalMesh
};

// The most recent tessellation's tally.
GpuTessStats gpuTessStats();

// Clear the tally (called at the start of a tessellation).
void resetGpuTessStats();

// Record one face's routing decision into the tally.
void recordGpuTessFace(bool viaGpu);

}  // namespace occt
}  // namespace cyber

#endif  // CYBERCADKERNEL_ENGINE_OCCT_GPU_TESS_STATS_H
