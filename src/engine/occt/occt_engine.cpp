// OCCT engine adapter — SPINE translation unit.
//
// This file (with occt_engine.h) is the shared spine the capability groups plug
// into. It is one of the OCCT-only TUs: it may include OpenCASCADE headers, but
// no OCCT type escapes into any public or shared header. It is NOT host-buildable
// (there is no host OCCT); it compiles only for iOS where the trimmed OCCT static
// libs are linked (CYBERCAD_HAS_OCCT=ON).
//
// Responsibilities kept here (cross-cutting; the per-capability bodies live in the
// sibling occt_*.cpp TUs — see the file map in occt_engine.h):
//   - engine registration/selection: create_default_engine() returns OcctEngine,
//     making it the active engine (and cc_brep_available() -> 1) whenever this TU
//     is linked;
//   - the type-erased shape store (wrap / unwrap over EngineShape);
//   - the BRepCheck_Analyzer::IsValid validation gate (isValid / addIfValid);
//   - the Poly_Triangulation -> MeshData converter (appendTriangulation);
//   - the stable TopExp sub-shape id maps (mapEdges/Faces/Vertices, edgesByIds,
//     facesByIds).
// cc_last_error integration and cc_shape_release are handled facade-side: occtGuard
// (in occt_engine.h) rethrows Standard_Failure as std::runtime_error so the
// facade's guard() records the message; cc_shape_release frees the registry handle.

#include "engine/occt/occt_engine.h"
#include "engine/occt/parallel_policy.h"

#include "core/guard.h"             // set_last_error — record the cross-engine decline
#include "core/shape_provenance.h"  // is_native_shape — refuse a foreign native body

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace cyber {
namespace occt {

// ── Type-erased shape store ───────────────────────────────────────────────────

EngineShape wrap(const TopoDS_Shape& shape) {
    auto holder = std::make_shared<OcctShape>();
    holder->shape = shape;
    return std::static_pointer_cast<void>(holder);
}

EngineShape wrapThread(const TopoDS_Shape& shape, const ThreadTag& tag) {
    auto holder = std::make_shared<OcctShape>();
    holder->shape = shape;
    holder->thread = tag;
    return std::static_pointer_cast<void>(holder);
}

// CROSS-ENGINE GUARD. The void behind an EngineShape is a bare shared_ptr<void>
// with no RTTI, so this static_pointer_cast<OcctShape> is UNCHECKED: handed a body
// built by the native engine (a NativeShape holder), it would reinterpret those
// bytes as an OcctShape and read a garbage TopoDS_Shape — OCCT then dereferences a
// bad handle and SIGSEGVs (the reported crash: native build → cc_set_engine(0) →
// cc_step_export / cc_mass_properties on the native handle). The header contract is
// explicit that a native body must NEVER be handed to OCCT, so refuse it HERE, at
// the single chokepoint every OCCT op funnels through: return nullptr (an honest
// decline every caller already handles as "unknown body") and record why. Guarding
// unwrap protects ALL body-consuming OCCT ops — step_export, mass_properties, and
// every other — not just the two that surfaced the bug.
bool isForeignNativeBody(const EngineShape& handle) {
    if (is_native_shape(handle.get())) {
        set_last_error(
            "shape was built under a different engine (native); cannot operate across "
            "engines — rebuild it under the active engine or switch back with "
            "cc_set_engine before this call");
        return true;
    }
    return false;
}

const TopoDS_Shape* unwrap(const EngineShape& handle) {
    if (!handle || isForeignNativeBody(handle)) {
        return nullptr;
    }
    return &std::static_pointer_cast<OcctShape>(handle)->shape;
}

const ThreadTag* threadTagOf(const EngineShape& handle) {
    if (!handle || isForeignNativeBody(handle)) {
        return nullptr;
    }
    return &std::static_pointer_cast<OcctShape>(handle)->thread;
}

ShapeResult tagAsThread(ShapeResult result, double turns, double pitchMM) {
    if (!result) {
        return result;  // build already failed → pass the Error through untouched
    }
    const TopoDS_Shape* shape = unwrap(result.value());
    if (shape == nullptr) {
        return result;
    }
    ThreadTag tag;
    tag.present = true;
    tag.turns = turns;
    tag.pitchMM = pitchMM;
    return wrapThread(*shape, tag);
}

ShapeResult tagAsThreadedBody(ShapeResult result) {
    if (!result) {
        return result;  // thread_apply already failed → pass the Error through untouched
    }
    const TopoDS_Shape* shape = unwrap(result.value());
    if (shape == nullptr) {
        return result;
    }
    ThreadTag tag;
    tag.threadedBody = true;  // provenance only; `present` stays false (not a raw thread ridge)
    return wrapThread(*shape, tag);
}

// True when either boolean operand is a threaded body (the result of thread_apply). Such a
// body has near-tangent helical faces baked in that a subsequent OCCT fuse/cut cannot
// robustly consume, so the caller uses this to emit an accurate ordering-constraint decline.
bool anyThreadedBodyOperand(const EngineShape& a, const EngineShape& b) {
    for (const ThreadTag* tag : {threadTagOf(a), threadTagOf(b)}) {
        if (tag != nullptr && tag->threadedBody) {
            return true;
        }
    }
    return false;
}

// ── Fine-thread boolean gate glue ─────────────────────────────────────────────

bool checkFineThreadGate(const EngineShape& a, const EngineShape& b, int op,
                         ShapeResult& outGated) {
    if (op != 0 && op != 1) {
        return false;  // only fuse/cut of a thread into a shaft is gated
    }
    ParallelPolicy& policy = ParallelPolicy::instance();
    for (const ThreadTag* tag : {threadTagOf(a), threadTagOf(b)}) {
        if (tag == nullptr || !tag->present) {
            continue;
        }
        const GateDecision decision = policy.evaluateGate({tag->turns, tag->pitchMM});
        policy.recordGateDecision(decision);  // surface to the host either way
        if (decision.gated) {
            outGated = make_error(decision.reason);
            return true;
        }
    }
    return false;
}

// ── Validation gate ───────────────────────────────────────────────────────────

bool isValid(const TopoDS_Shape& shape) {
    if (shape.IsNull()) {
        return false;
    }
    return BRepCheck_Analyzer(shape).IsValid();
}

ShapeResult addIfValid(const TopoDS_Shape& shape, const char* invalidMessage) {
    if (!isValid(shape)) {
        return make_error(invalidMessage ? invalidMessage : "invalid shape");
    }
    return wrap(shape);
}

// ── Tessellation (Poly_Triangulation -> MeshData) ─────────────────────────────

void appendTriangulation(const TopoDS_Shape& shape, double deflection, MeshData& mesh) {
    // Relative deflection off, angular deflection 0.5. Per-face parallel meshing is
    // governed by the policy toggle (default ON) and bounded by the worker cap; the
    // triangulation itself is a deterministic per-face decomposition, so the result
    // matches the serial mesher byte-for-byte (parallel-acceleration §Parallel meshing).
    const bool inParallel = ParallelPolicy::instance().parallelFor();
    if (inParallel) {
        applyOcctWorkerCap();
    }
    BRepMesh_IncrementalMesh mesher(shape, deflection, Standard_False, 0.5,
                                    inParallel ? Standard_True : Standard_False);
    (void)mesher;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) {
            continue;
        }
        const gp_Trsf& trsf = loc.Transformation();
        const int base = static_cast<int>(mesh.vertices.size() / 3);
        for (Standard_Integer i = 1; i <= tri->NbNodes(); ++i) {
            const gp_Pnt p = tri->Node(i).Transformed(trsf);
            mesh.vertices.push_back(p.X());
            mesh.vertices.push_back(p.Y());
            mesh.vertices.push_back(p.Z());
        }
        const bool reversed = (face.Orientation() == TopAbs_REVERSED);
        for (Standard_Integer i = 1; i <= tri->NbTriangles(); ++i) {
            Standard_Integer a, b, c;
            tri->Triangle(i).Get(a, b, c);
            if (reversed) {
                std::swap(b, c);
            }
            mesh.triangles.push_back(base + a - 1);
            mesh.triangles.push_back(base + b - 1);
            mesh.triangles.push_back(base + c - 1);
        }
    }
}

MeshData tessellateShape(const TopoDS_Shape& shape, double deflection) {
    MeshData mesh;
    appendTriangulation(shape, deflection, mesh);
    return mesh;
}

// ── Stable TopExp sub-shape id maps ───────────────────────────────────────────

namespace {
TopTools_IndexedMapOfShape mapOf(const TopoDS_Shape& shape, TopAbs_ShapeEnum kind) {
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(shape, kind, map);
    return map;
}
}  // namespace

TopTools_IndexedMapOfShape mapEdges(const TopoDS_Shape& shape) {
    return mapOf(shape, TopAbs_EDGE);
}

TopTools_IndexedMapOfShape mapFaces(const TopoDS_Shape& shape) {
    return mapOf(shape, TopAbs_FACE);
}

TopTools_IndexedMapOfShape mapVertices(const TopoDS_Shape& shape) {
    return mapOf(shape, TopAbs_VERTEX);
}

std::vector<TopoDS_Edge> edgesByIds(const TopoDS_Shape& shape, const int* ids, int count) {
    std::vector<TopoDS_Edge> out;
    if (ids == nullptr || count <= 0) {
        return out;
    }
    const TopTools_IndexedMapOfShape map = mapEdges(shape);
    for (int i = 0; i < count; ++i) {
        const int id = ids[i];
        if (id >= 1 && id <= map.Extent()) {
            out.push_back(TopoDS::Edge(map.FindKey(id)));
        }
    }
    return out;
}

std::vector<TopoDS_Face> facesByIds(const TopoDS_Shape& shape, const int* ids, int count) {
    std::vector<TopoDS_Face> out;
    if (ids == nullptr || count <= 0) {
        return out;
    }
    const TopTools_IndexedMapOfShape map = mapFaces(shape);
    for (int i = 0; i < count; ++i) {
        const int id = ids[i];
        if (id >= 1 && id <= map.Extent()) {
            out.push_back(TopoDS::Face(map.FindKey(id)));
        }
    }
    return out;
}

}  // namespace occt

// ── Identity / availability ───────────────────────────────────────────────────

std::string OcctEngine::name() const {
    return "occt";
}

bool OcctEngine::available() const {
    return true;  // cc_brep_available() -> 1 when the OCCT adapter is active.
}

// ── Parallel control ──────────────────────────────────────────────────────────
// Route the facade toggle to the process-wide ParallelPolicy the boolean/mesh
// paths consult (occt::ParallelPolicy::parallelFor / applyOcctWorkerCap).
void OcctEngine::set_parallel(bool enabled) {
    occt::ParallelPolicy::instance().setEnabled(enabled);
}

bool OcctEngine::parallel_enabled() const {
    return occt::ParallelPolicy::instance().enabled();
}

// ── GPU tessellation control ──────────────────────────────────────────────────
// The toggle only latches ON in a Metal build; without CYBERCAD_HAS_METAL there
// is no GPU surface-eval module linked, so the flag stays OFF and cc_tessellate
// runs the OCCT-only path (cc_gpu_tessellation_enabled() -> 0).
void OcctEngine::set_gpu_tessellation(bool enabled) {
#ifdef CYBERCAD_HAS_METAL
    gpuTessellation_.store(enabled, std::memory_order_relaxed);
#else
    (void)enabled;  // no Metal backend in this build: GPU tessellation unavailable
#endif
}

bool OcctEngine::gpu_tessellation_enabled() const {
#ifdef CYBERCAD_HAS_METAL
    return gpuTessellation_.load(std::memory_order_relaxed);
#else
    return false;
#endif
}

// ── Engine registration / selection ───────────────────────────────────────────
// Defined here (not in the stub) so that whenever the OCCT adapter TUs are linked
// (CYBERCAD_HAS_OCCT=ON), OcctEngine is the build's default/active engine. The
// stub only provides create_default_engine() in the no-OCCT host build.
std::shared_ptr<IEngine> create_default_engine() {
    return std::make_shared<OcctEngine>();
}

}  // namespace cyber
