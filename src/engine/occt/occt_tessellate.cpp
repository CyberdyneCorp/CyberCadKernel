// OCCT engine adapter — TESSELLATE capability group.
//
// Defines the OcctEngine display/pick meshing methods declared in occt_engine.h:
//   - tessellate      -> one merged MeshData for the whole body (viewport display)
//   - face_meshes     -> one FaceMeshData per face, tagged with the stable face id
//   - edge_polylines  -> one EdgePolylineData per edge, tagged with the stable edge id
//
// These are behaviour-preserving ports of the CYBERCAD_HAS_OCCT bodies of
// cc_tessellate / cc_face_meshes / cc_edge_polylines from the app's KernelBridge.mm.
// They produce ONLY the internal POD buffers; the facade converts them to the C
// structs and owns cc_mesh_free / cc_face_meshes_free / cc_edge_polylines_free.
//
// This is an OCCT-only TU: every OpenCASCADE include stays inside this file (the
// shared core comes from occt_engine.h). It is not host-buildable — it compiles
// only for iOS where the trimmed OCCT static libs are linked (CYBERCAD_HAS_OCCT).
//
// Degenerate-input handling is preserved exactly from the source: an unknown /
// null body yields an EMPTY result (not an error, so cc_last_error is untouched),
// a face with no triangulation or an empty/degenerate edge yields an empty slot
// tagged with its id, and the deflection is clamped to 0.1 when non-positive.

#include "engine/occt/occt_engine.h"
#include "engine/occt/gpu_tess_stats.h"
#include "engine/occt/occt_gpu_tessellate.h"
#include "engine/occt/parallel_policy.h"

#include <utility>
#include <vector>

// ── OCCT headers (adapter TU only) ────────────────────────────────────────────
// The shared meshing / triangulation / topology-map core is pulled in through
// occt_engine.h (including BRepMesh_IncrementalMesh); IMeshTools_Parameters lets
// us drive the mesher with an explicit InParallel flag, and edge discretization
// needs the two extra curve builders.
#include <IMeshTools_Parameters.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_TangentialDeflection.hxx>

namespace cyber {

namespace {
// The app's bridge clamps a non-positive requested deflection to 0.1 mm before
// meshing so a caller passing 0 still gets a usable display mesh.
double clampDeflection(double deflection) {
    return deflection > 0 ? deflection : 0.1;
}

// Parallel per-face meshing knobs. `InParallel` spreads the body's faces across
// CPU cores; every other value is left at exactly what the serial mesher used
// (absolute linear deflection, 0.5 rad angular, MinSize/interior tolerances
// derived by BRepMesh_IncrementalMesh from the deflection). Because each face is
// triangulated independently of the others, the per-face result does not depend
// on how many workers run — so the mesh is identical to the serial mesher for a
// given deflection; only the wall-clock changes. This is the InParallel form of
// the positional (…, Standard_True) ctor the app's bridge used byte-for-byte.
IMeshTools_Parameters parallelMeshParams(double deflection) {
    // Per-face parallel meshing is governed by the global `parallel` toggle
    // (default ON, opt-out) and, when on, bounded by the host worker cap over
    // OCCT's OSD_ThreadPool. The per-face triangulation is independent of the
    // worker count, so the mesh is identical to serial for a given deflection.
    const bool inParallel = occt::ParallelPolicy::instance().parallelFor();
    if (inParallel) {
        occt::applyOcctWorkerCap();
    }
    IMeshTools_Parameters params;       // OCCT defaults for the interior knobs
    params.Deflection = deflection;     // linear deflection (mm)
    params.Angle = 0.5;                 // angular deflection (rad)
    params.Relative = Standard_False;   // absolute, not edge-relative
    params.InParallel = inParallel ? Standard_True : Standard_False;  // ← multi-core meshing
    return params;
}

// Read one meshed face's triangulation into flat vertex/triangle buffers,
// transforming nodes by the face location and flipping winding for reversed
// faces so triangles face outward. `baseVertex` is the running vertex count of
// the target buffer, so tessellate() can merge every face into one mesh (0 for
// the face-local buffers face_meshes hands back per face). A face without a
// usable triangulation appends nothing, leaving an empty slot — the exact
// degenerate-face handling of the serial path.
void appendFaceTriangulation(const TopoDS_Face& face, std::vector<double>& vertices,
                             std::vector<int>& triangles, int baseVertex) {
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull()) {
        return;
    }
    const int nv = tri->NbNodes();
    const int nt = tri->NbTriangles();
    if (nv <= 0 || nt <= 0) {
        return;
    }
    const gp_Trsf& trsf = loc.Transformation();
    vertices.reserve(vertices.size() + static_cast<std::size_t>(nv) * 3);
    for (int k = 1; k <= nv; ++k) {
        const gp_Pnt p = tri->Node(k).Transformed(trsf);
        vertices.push_back(p.X());
        vertices.push_back(p.Y());
        vertices.push_back(p.Z());
    }
    const bool reversed = (face.Orientation() == TopAbs_REVERSED);
    triangles.reserve(triangles.size() + static_cast<std::size_t>(nt) * 3);
    for (int k = 1; k <= nt; ++k) {
        Standard_Integer a, b, c;
        tri->Triangle(k).Get(a, b, c);
        if (reversed) {
            std::swap(b, c);
        }
        triangles.push_back(baseVertex + a - 1);
        triangles.push_back(baseVertex + b - 1);
        triangles.push_back(baseVertex + c - 1);
    }
}

// Mesh one face into (vertices, triangles) at `baseVertex`, honouring the GPU
// tessellation toggle. When `gpuOn` and the face is provably a GPU-eligible
// untrimmed rectangular patch, its triangles come from the GPU-evaluated (u,v)
// grid; otherwise the face uses the OCCT triangulation already computed by the
// whole-body BRepMesh pass — byte-for-byte the serial/OCCT path. With `gpuOn`
// false (default) this is EXACTLY the previous behaviour and the GPU code is not
// referenced. Per-face routing is recorded for diagnostics only when the GPU
// toggle is active (the OFF path stays side-effect-free on the mesh).
void appendFaceMesh(const TopoDS_Face& face, double deflection, bool gpuOn,
                    std::vector<double>& vertices, std::vector<int>& triangles, int baseVertex) {
#ifdef CYBERCAD_HAS_METAL
    if (gpuOn) {
        if (occt::tryTessellateFaceGPU(face, deflection, vertices, triangles, baseVertex)) {
            occt::recordGpuTessFace(true);
            return;
        }
        occt::recordGpuTessFace(false);  // not eligible / eval failed → OCCT fallback
    }
#else
    (void)deflection;
    (void)gpuOn;
#endif
    appendFaceTriangulation(face, vertices, triangles, baseVertex);
}
}  // namespace

// ── tessellate ────────────────────────────────────────────────────────────────
Result<MeshData> OcctEngine::tessellate(EngineShape body, double deflection) {
    const TopoDS_Shape* shape = occt::unwrap(body);
    if (shape == nullptr) {
        return MeshData{};  // unknown body → empty mesh, matching the source
    }
    // GPU tessellation toggle, read on the caller thread and captured for the
    // worker. OFF (default) → the OCCT-only path below, unchanged.
    const bool gpuOn = gpu_tessellation_enabled();
    // Route the long mesh off the caller's inline path through the operation-
    // scheduler with the cancellation-safe boundary (spec §"Cancellable
    // accelerated operations"); `body` is captured to keep the shape alive on the
    // worker thread.
    return occt::runScheduled([body, deflection, gpuOn](OperationContext& ctx) -> Result<MeshData> {
        return occt::occtGuard([&]() -> Result<MeshData> {
            const TopoDS_Shape& s = *occt::unwrap(body);
            occt::resetGpuTessStats();
            ctx.report(0.1, "mesh: triangulate");
            // Parallel per-face meshing behind cc_tessellate: one incremental mesh
            // over the whole body with InParallel on, then merge every face into a
            // single buffer. The whole-body mesh is always built so every OCCT
            // fallback face has a triangulation; when the GPU toggle is on, a
            // GPU-eligible face is instead meshed from its (u,v) grid (its OCCT
            // triangulation is simply unused). Face traversal order and the
            // per-face merge are unchanged, so with the toggle OFF the merged mesh
            // is byte-identical to the serial path for a given deflection.
            BRepMesh_IncrementalMesh mesher(s, parallelMeshParams(clampDeflection(deflection)));
            (void)mesher;
            MeshData mesh;
            for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) {
                const int base = static_cast<int>(mesh.vertices.size() / 3);
                appendFaceMesh(TopoDS::Face(ex.Current()), clampDeflection(deflection), gpuOn,
                               mesh.vertices, mesh.triangles, base);
            }
            ctx.report(1.0, "mesh: done");
            return mesh;
        });
    });
}

// ── face_meshes ───────────────────────────────────────────────────────────────
Result<std::vector<FaceMeshData>> OcctEngine::face_meshes(EngineShape body, double deflection) {
    const TopoDS_Shape* shape = occt::unwrap(body);
    if (shape == nullptr) {
        return std::vector<FaceMeshData>{};
    }
    const bool gpuOn = gpu_tessellation_enabled();
    return occt::runScheduled([body, deflection, gpuOn](OperationContext& ctx)
                                  -> Result<std::vector<FaceMeshData>> {
        return occt::occtGuard([&]() -> Result<std::vector<FaceMeshData>> {
            const TopoDS_Shape& s = *occt::unwrap(body);
            occt::resetGpuTessStats();
            // Iterate the FACE index map so faceId matches cc_subshape_ids/cc_shell.
            const TopTools_IndexedMapOfShape map = occt::mapFaces(s);
            const int count = map.Extent();
            std::vector<FaceMeshData> faces;
            if (count <= 0) {
                return faces;
            }
            ctx.report(0.1, "mesh: triangulate");
            // One parallel incremental mesh for the whole body (same knobs as the
            // display mesh), then read each face's mesh back out with face-local
            // 0-based indices (baseVertex 0). Exactly one CCFaceMesh slot per face
            // id, in map order — unchanged from the OCCT-only path; only an
            // eligible face's triangles change source (GPU grid) when the toggle
            // is on.
            BRepMesh_IncrementalMesh mesher(s, parallelMeshParams(clampDeflection(deflection)));
            (void)mesher;
            faces.resize(static_cast<std::size_t>(count));
            for (int i = 1; i <= count; ++i) {
                FaceMeshData& fm = faces[static_cast<std::size_t>(i - 1)];
                fm.faceId = i;  // empty slot unless the face has a usable triangulation
                appendFaceMesh(TopoDS::Face(map.FindKey(i)), clampDeflection(deflection), gpuOn,
                               fm.vertices, fm.triangles, 0);
            }
            ctx.report(1.0, "mesh: done");
            return faces;
        });
    });
}

// ── edge_polylines ────────────────────────────────────────────────────────────
Result<std::vector<EdgePolylineData>> OcctEngine::edge_polylines(EngineShape body) {
    const TopoDS_Shape* shape = occt::unwrap(body);
    if (shape == nullptr) {
        return std::vector<EdgePolylineData>{};
    }
    return occt::occtGuard([&]() -> Result<std::vector<EdgePolylineData>> {
        // 1-based edge ids match cc_subshape_ids/cc_fillet_edges.
        const TopTools_IndexedMapOfShape map = occt::mapEdges(*shape);
        const int count = map.Extent();
        std::vector<EdgePolylineData> edges;
        if (count <= 0) {
            return edges;
        }
        edges.resize(static_cast<std::size_t>(count));
        for (int i = 1; i <= count; ++i) {
            EdgePolylineData& e = edges[static_cast<std::size_t>(i - 1)];
            e.edgeId = i;  // empty slot for seam/degenerate/too-short edges
            const TopoDS_Edge edge = TopoDS::Edge(map.FindKey(i));
            if (BRep_Tool::Degenerated(edge)) {
                continue;  // seam/degenerate → no 3D polyline
            }
            BRepAdaptor_Curve curve(edge);
            // Tangential-deflection discretization: a line yields 2 points, a curve
            // as many as its curvature needs — the whole arc as ONE polyline.
            GCPnts_TangentialDeflection disc(curve, 0.2, 0.2);
            const int n = disc.NbPoints();
            if (n < 2) {
                continue;
            }
            e.points.reserve(static_cast<std::size_t>(n) * 3);
            for (int k = 1; k <= n; ++k) {
                const gp_Pnt p = disc.Value(k);
                e.points.push_back(p.X());
                e.points.push_back(p.Y());
                e.points.push_back(p.Z());
            }
        }
        return edges;
    });
}

}  // namespace cyber
