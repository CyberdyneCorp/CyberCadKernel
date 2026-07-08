#ifndef CYBERCADKERNEL_ENGINE_OCCT_OCCT_ENGINE_H
#define CYBERCADKERNEL_ENGINE_OCCT_OCCT_ENGINE_H

// OCCT engine adapter — the shared spine the capability groups plug into.
//
// This is an OCCT-INTERNAL implementation header: it includes OpenCASCADE
// headers, so it MUST NOT be pulled into any public or shared translation unit.
// Only the OCCT adapter TUs (src/engine/occt/*.cpp) include it. It compiles only
// for iOS where the trimmed OCCT static libs are linked (CYBERCAD_HAS_OCCT); there
// is no host OCCT, so this header is never seen by the host build.
//
// The single OcctEngine class is DECLARED here in full — every capability-group
// method (construct / feature / boolean / transform / tessellate / query /
// exchange) is an override on the class — but the methods are DEFINED across
// sibling TUs so the port can proceed group by group:
//
//     construct           -> occt_construct.cpp
//     feature             -> occt_feature.cpp
//     boolean + transform -> occt_booltransform.cpp
//     tessellate          -> occt_tessellate.cpp
//     query               -> occt_query.cpp
//     exchange            -> occt_exchange.cpp
//   Phase-3 features, each owned by its own TU so they land in disjoint files:
//     wrap_emboss (robust) -> occt_wrap_emboss.cpp
//     reference geometry   -> occt_reference_geometry.cpp
//     thread apply         -> occt_thread_boolean.cpp
//     full-round fillet    -> occt_full_round_fillet.cpp
//     G2 blend fillet      -> occt_g2_fillet.cpp
//     spine (this class's name()/available(), helpers, registration)
//                         -> occt_engine.cpp
//
// The cross-cutting pieces every group depends on live in namespace cyber::occt:
// the type-erased shape store (wrap/unwrap), the occtGuard (translates OCCT's
// Standard_Failure — which does NOT derive from std::exception — so the facade's
// guard records it into cc_last_error), the BRepCheck_Analyzer::IsValid gate
// (isValid / addIfValid), the Poly_Triangulation -> MeshData converter, and the
// stable TopExp sub-shape id maps used for edge/face picking.

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "engine/IEngine.h"

// ── OCCT headers (adapter TUs only) ───────────────────────────────────────────
// The common core every capability group needs; groups include their own
// specialized builders on top of this.
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRep_Tool.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>

namespace cyber {

// ── Shared OCCT adapter spine (namespace occt) ────────────────────────────────
namespace occt {

// Optional provenance the fine-thread gate needs: when a shape was built by
// cc_helical_thread / cc_tapered_thread we remember its turns/pitch so a later
// boolean can predict a runaway fuse/cut and gate it (see checkFineThreadGate).
// `present` is false for every non-thread shape, so the gate is a no-op for them.
struct ThreadTag {
    bool present = false;
    double turns = 0.0;
    double pitchMM = 0.0;
};

// Type-erased shape store. The adapter wraps a TopoDS_Shape into the registry's
// opaque EngineShape (std::shared_ptr<void>); the facade/registry only ever see
// the void handle, so no OCCT type crosses into public/shared code.
struct OcctShape {
    TopoDS_Shape shape;
    ThreadTag thread;  // default: not a thread — gate ignores it
};

// Wrap a TopoDS_Shape into an EngineShape for the registry.
EngineShape wrap(const TopoDS_Shape& shape);

// Wrap with thread provenance so the fine-thread gate can evaluate a later boolean.
EngineShape wrapThread(const TopoDS_Shape& shape, const ThreadTag& tag);

// Re-wrap a just-built thread ShapeResult carrying its turns/pitch. Passes a
// failed/empty result through unchanged. Used by cc_helical_thread / cc_tapered_thread.
ShapeResult tagAsThread(ShapeResult result, double turns, double pitchMM);

// Thread provenance behind an EngineShape, or nullptr if the handle is empty.
const ThreadTag* threadTagOf(const EngineShape& handle);

// Recover the TopoDS_Shape behind an EngineShape, or nullptr if the handle is
// empty (an unknown/invalid CCShapeId resolves to an empty EngineShape).
const TopoDS_Shape* unwrap(const EngineShape& handle);

// Guard used by EVERY group body: OCCT signals invalid geometry / degenerate
// input via Standard_Failure, which does NOT derive from std::exception and must
// never cross the C ABI. This catches it and rethrows a std::runtime_error so the
// facade's outer guard() records the message into the per-thread cc_last_error.
// Inline template so each group TU can wrap its OCCT calls: occtGuard([&]{ ... }).
template <class Fn>
auto occtGuard(Fn&& fn) -> decltype(fn()) {
    try {
        return std::forward<Fn>(fn)();
    } catch (const Standard_Failure& e) {
        const char* m = e.GetMessageString();
        throw std::runtime_error(std::string("OCCT: ") + ((m && *m) ? m : "operation failed"));
    }
}

// BRepCheck_Analyzer::IsValid gate: a built shape is accepted only if it is
// non-null and topologically valid, so the facade never returns a malformed body.
bool isValid(const TopoDS_Shape& shape);

// Validate a freshly built shape and, if good, wrap it into a ShapeResult; on a
// null/invalid result return an Error tagged with `op`. Every construct / feature
// / boolean / transform body ends with `return addIfValid(built, "op: invalid");`.
ShapeResult addIfValid(const TopoDS_Shape& shape, const char* invalidMessage);

// Mesh the shape at `deflection` (mm) and append every face's triangles to `mesh`,
// transforming nodes by each face's location and flipping winding for reversed
// faces so all triangles face outward. Shared Poly_Triangulation -> MeshData path.
void appendTriangulation(const TopoDS_Shape& shape, double deflection, MeshData& mesh);

// Convenience: appendTriangulation into a fresh MeshData.
MeshData tessellateShape(const TopoDS_Shape& shape, double deflection);

// Stable TopExp sub-shape id maps. Ids are 1-based indices into these maps and are
// the SAME ids cc_subshape_ids / cc_fillet_edges / cc_shell exchange with the app,
// so a UI selection round-trips deterministically.
TopTools_IndexedMapOfShape mapEdges(const TopoDS_Shape& shape);
TopTools_IndexedMapOfShape mapFaces(const TopoDS_Shape& shape);
TopTools_IndexedMapOfShape mapVertices(const TopoDS_Shape& shape);

// Resolve `count` 1-based ids (from cc_subshape_ids) into edges / faces, skipping
// out-of-range ids so a stale selection degrades gracefully instead of throwing.
std::vector<TopoDS_Edge> edgesByIds(const TopoDS_Shape& shape, const int* ids, int count);
std::vector<TopoDS_Face> facesByIds(const TopoDS_Shape& shape, const int* ids, int count);

// Fine-thread boolean gate glue. For a fuse (op 0) or cut (op 1) whose operand `a`
// or `b` carries thread provenance, ask ParallelPolicy whether the op is too
// expensive; if so, write a gated ShapeResult (an Error the facade collapses to a
// nil handle, keeping the thread and shaft as SEPARATE bodies) into `outGated`,
// record the decision for the host, and return true. Returns false (leaving
// `outGated` untouched) when the op should proceed. `common` (op 2) is never gated.
bool checkFineThreadGate(const EngineShape& a, const EngineShape& b, int op,
                         ShapeResult& outGated);

// ── shape-healing oracle (Phase 4 #4 native-healing fallback) ─────────────────
// The OCCT side of the engine-internal native-heal hook (see
// src/engine/native/native_heal_hook.h): sew a face soup + heal the shell/solid via
// BRepBuilderAPI_Sewing → ShapeFix_Shell → ShapeFix_Solid. Confined to this OCCT
// adapter; the native module never includes an OCCT header. Also the ORACLE the sim
// parity harness compares the native healer against.
struct SewFixResult {
    TopoDS_Shape shape;         // the sewn/fixed shell or solid (may be null on failure)
    bool watertight = false;    // the sewn shell is closed (BRep_Tool / free-edge count == 0)
    bool validSolid = false;    // a valid Solid was formed (ShapeFix_Solid + IsValid)
    double volume = 0.0;        // enclosed volume of the fixed solid (BRepGProp), 0 if none
};

// Sew `faceSoup` (a TopoDS_Compound of independent faces) at `tolerance` and heal
// with ShapeFix. Reports watertight/valid/volume so the parity harness can compare
// against the native HealResult. Never widens the tolerance.
SewFixResult sewAndFix(const TopoDS_Shape& faceSoup, double tolerance);

}  // namespace occt

// ── OcctEngine ────────────────────────────────────────────────────────────────
// One class, methods DEFINED across the sibling group TUs (see the file map at the
// top). All 49 IEngine geometry methods are overridden so the facade routes every
// geometric cc_* here; buffer-free / availability / error cc_* stay facade-side.
class OcctEngine final : public IEngine {
public:
    // Identity / availability (occt_engine.cpp). available() == true drives
    // cc_brep_available() -> 1 whenever the OCCT adapter is the active engine.
    std::string name() const override;
    bool available() const override;

    // ── parallel control ──────────────────────────────────────────────────────
    // Drive cc_set_parallel / cc_parallel_enabled through occt::ParallelPolicy's
    // process-wide toggle (setEnabled/enabled), which the boolean/mesh paths read.
    void set_parallel(bool enabled) override;
    bool parallel_enabled() const override;

    // ── GPU tessellation control ──────────────────────────────────────────────
    // Hold the process-visible GPU-tessellation toggle the tessellate/face_meshes
    // paths consult. The setter only latches ON when the build was compiled with
    // CYBERCAD_HAS_METAL; otherwise it stays OFF so cc_gpu_tessellation_enabled()
    // reports 0 and cc_tessellate runs the OCCT-only path unchanged.
    void set_gpu_tessellation(bool enabled) override;
    bool gpu_tessellation_enabled() const override;

    // ── construct (occt_construct.cpp) ────────────────────────────────────────
    Result<MeshData> extrude_mesh(const double* profileXY, int pointCount, double depth) override;
    ShapeResult solid_extrude(const double* profileXY, int pointCount, double depth) override;
    ShapeResult solid_revolve(const double* profileXY, int pointCount, double angleRadians) override;
    ShapeResult solid_loft(const double* bottomXY, int bottomCount, const double* topXY,
                           int topCount, double depth) override;
    ShapeResult solid_loft_wires(const double* aXYZ, int aCount, const double* bXYZ,
                                 int bCount) override;
    ShapeResult solid_loft_sections(const double* sectionsXYZ, const int* counts,
                                    int sectionCount) override;
    ShapeResult solid_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                            int pathCount) override;
    ShapeResult twisted_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                              int pathCount, double twistRadians, double scaleEnd) override;
    ShapeResult loft_along_rail(const double* railXYZ, int railCount, const double* profileA_XY,
                                int aCount, const double* profileB_XY, int bCount) override;
    ShapeResult guided_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                             int pathCount, const double* guideXYZ, int guideCount) override;
    ShapeResult guided_orient_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                                    int pathCount, const double* guideXYZ,
                                    int guideCount) override;
    // wrap_emboss is DEFINED in its own TU (occt_wrap_emboss.cpp) so the robust
    // sewn-pad rework (Phase-3) can be owned there without touching occt_construct.cpp.
    ShapeResult wrap_emboss(EngineShape body, int faceId, const double* profileXY, int count,
                            double depth, int boss) override;
    ShapeResult helical_thread(double majorRadiusMM, double pitchMM, double turns, double depthMM,
                               double flankAngleDeg, double pointsPerMM,
                               int samplesPerTurn) override;
    ShapeResult tapered_thread(double topRadiusMM, double tipRadiusMM, double pitchMM, double turns,
                               double depthMM, double flankAngleDeg, double pointsPerMM,
                               int samplesPerTurn) override;
    ShapeResult tapered_shank(double radiusMM, double fullHeightMM, double taperHeightMM,
                              double pointsPerMM) override;
    ShapeResult solid_extrude_holes(const double* outerXY, int outerCount,
                                    const double* holesCenterRadius, int holeCount,
                                    double depth) override;
    ShapeResult solid_extrude_polyholes(const double* outerXY, int outerCount,
                                        const double* holesXY, const int* holeCounts, int holeCount,
                                        double depth) override;
    ShapeResult solid_extrude_profile(const ProfileSeg* segs, int segCount,
                                      const double* holesCenterRadius, int holeCount,
                                      const double* splineXY, int splineXYCount,
                                      double depth) override;
    ShapeResult solid_extrude_profile_polyholes(const ProfileSeg* segs, int segCount,
                                                const double* holesCenterRadius, int circleCount,
                                                const double* polyXY, const int* polyCounts,
                                                int polyCount, const double* splineXY,
                                                int splineXYCount, double depth) override;
    ShapeResult solid_revolve_profile(const ProfileSeg* segs, int segCount, double ax, double ay,
                                      double adx, double ady, const double* splineXY,
                                      int splineXYCount, double angleRadians) override;

    // ── feature (occt_feature.cpp) ────────────────────────────────────────────
    ShapeResult fillet_edges(EngineShape body, const int* edgeIds, int edgeCount,
                             double radius) override;
    ShapeResult fillet_edges_variable(EngineShape body, const int* edgeIds, int edgeCount,
                                      double radius1, double radius2) override;
    ShapeResult chamfer_edges(EngineShape body, const int* edgeIds, int edgeCount,
                              double distance) override;
    ShapeResult chamfer_edges_asym(EngineShape body, const int* edgeIds, int edgeCount,
                                   double distance1, double distance2) override;
    ShapeResult shell(EngineShape body, const int* faceIds, int faceCount,
                      double thickness) override;
    ShapeResult offset_face(EngineShape body, int faceId, double distance) override;
    ShapeResult replace_face(EngineShape body, int faceId, double offset, double tiltDeg) override;
    ShapeResult replace_face_to_plane(EngineShape body, int faceId, double px, double py, double pz,
                                      double nx, double ny, double nz) override;
    ShapeResult fillet_face(EngineShape body, int faceId, double radius) override;
    ShapeResult split_plane(EngineShape body, double ox, double oy, double oz, double nx, double ny,
                            double nz, int keepPositive) override;

    // ── Phase-3 feature: full-round fillet (occt_full_round_fillet.cpp) ────────
    ShapeResult full_round_fillet(EngineShape body, int faceId) override;
    ShapeResult full_round_fillet_faces(EngineShape body, int leftFaceId, int middleFaceId,
                                        int rightFaceId) override;

    // ── Phase-3 feature: G2 blend fillet (occt_g2_fillet.cpp) ──────────────────
    ShapeResult fillet_edges_g2(EngineShape body, const int* edgeIds, int edgeCount,
                                double radius) override;

    // ── boolean (occt_booltransform.cpp) ──────────────────────────────────────
    ShapeResult boolean_op(EngineShape a, EngineShape b, int op) override;

    // ── Phase-3 boolean: robust thread apply (occt_thread_boolean.cpp) ─────────
    ShapeResult thread_apply(EngineShape shaft, EngineShape thread, int op) override;

    // ── tessellate (occt_tessellate.cpp) ──────────────────────────────────────
    Result<MeshData> tessellate(EngineShape body, double deflection) override;
    Result<std::vector<FaceMeshData>> face_meshes(EngineShape body, double deflection) override;
    Result<std::vector<EdgePolylineData>> edge_polylines(EngineShape body) override;

    // ── drafting: orthographic HLR oracle (occt_drafting.cpp) ─────────────────
    // The HLRBRep_Algo / HLRBRep_HLRToShape oracle the native polyhedral HLR core
    // is verified against (GATE b). Projects onto the SAME drawing-plane basis the
    // native path uses (right = normalize(viewDir × up), trueUp = right × viewDir).
    Result<DrawingData> hlr_project(EngineShape body, const double viewDir[3], const double up[3],
                                    HlrOptionsData opts) override;

    // ── query (occt_query.cpp) ────────────────────────────────────────────────
    Result<MassData> mass_properties(EngineShape body) override;
    Result<std::vector<double>> principal_moments(EngineShape body) override;
    Result<ValidityData> check_solid(EngineShape body) override;  // MOAT M-GS GS6
    Result<std::vector<double>> bounding_box(EngineShape body) override;
    Result<std::vector<double>> face_axis(EngineShape body, int faceId) override;
    Result<std::vector<int>> subshape_ids(EngineShape body, int kind) override;
    Result<std::vector<int>> tangent_chain(EngineShape body, const int* edgeIds,
                                           int edgeCount) override;
    Result<std::vector<int>> outer_rim_chain(EngineShape body, const int* edgeIds,
                                             int edgeCount) override;
    Result<std::vector<double>> offset_face_boundary(EngineShape body, int faceId,
                                                     double distance) override;

    // ── Phase-3 reference geometry (occt_reference_geometry.cpp) ───────────────
    // Derived datums that read an existing body's geometry (the point-only trio is
    // pure fp64 math and lives facade-side). Each fills 6 values.
    Result<std::vector<double>> ref_plane_from_face(EngineShape body, int faceId) override;
    Result<std::vector<double>> ref_axis_from_edge(EngineShape body, int edgeId) override;
    Result<std::vector<double>> ref_axis_from_face(EngineShape body, int faceId) override;

    // ── transform (occt_booltransform.cpp) ────────────────────────────────────
    ShapeResult scale_shape(EngineShape body, double factor) override;
    ShapeResult scale_shape_about(EngineShape body, double cx, double cy, double cz,
                                  double factor) override;
    ShapeResult rotate_shape_about(EngineShape body, double cx, double cy, double cz, double ax,
                                   double ay, double az, double angleRadians) override;
    ShapeResult mirror_shape(EngineShape body, double px, double py, double pz, double nx,
                             double ny, double nz) override;
    ShapeResult translate_shape(EngineShape body, double tx, double ty, double tz) override;
    ShapeResult place_on_frame(EngineShape body, double ox, double oy, double oz, double ux,
                               double uy, double uz, double vx, double vy, double vz) override;

    // ── exchange (occt_exchange.cpp) ──────────────────────────────────────────
    Result<void> step_export(EngineShape body, const char* path) override;
    ShapeResult step_import(const char* path) override;
    Result<void> iges_export(EngineShape body, const char* path) override;
    ShapeResult iges_import(const char* path) override;

private:
    // Process-visible GPU-tessellation toggle (default OFF). Read by the
    // tessellate / face_meshes bodies in occt_tessellate.cpp to route per-face
    // meshing to the GPU grid path (occt_gpu_tessellate.cpp) when ON.
    std::atomic<bool> gpuTessellation_{false};
};

}  // namespace cyber

#endif  // CYBERCADKERNEL_ENGINE_OCCT_OCCT_ENGINE_H
