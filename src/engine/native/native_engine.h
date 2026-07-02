#ifndef CYBERCADKERNEL_ENGINE_NATIVE_NATIVE_ENGINE_H
#define CYBERCADKERNEL_ENGINE_NATIVE_NATIVE_ENGINE_H

// NativeEngine — the clean-room, OCCT-free geometry engine (Phase 4 endgame,
// openspec/NATIVE-REWRITE.md). It implements the capabilities migrated to native
// C++20 so far and FALLS THROUGH to a fallback engine (the OCCT adapter under
// CYBERCAD_HAS_OCCT, else the stub) for every capability not yet native — the
// coexistence the IEngine boundary was built for. The facade selects it at runtime
// via cc_set_engine(1); DEFAULT stays the OCCT/stub engine so every existing suite
// is unchanged.
//
//   cc_* facade → active engine
//                   ├─ NativeEngine ── native solid_extrude / solid_revolve
//                   │                   (+ native tessellate/query on its own
//                   │                    native bodies)
//                   │        └─ falls through to ↓ for everything else
//                   └─ fallback (OcctEngine on iOS/macOS, StubEngine on host)
//
// NATIVE SCOPE (honest — see construct.h):
//   * solid_extrude  — closed polygon profile → prism solid (native topology).
//   * solid_revolve  — LINE-SEGMENT profile → surfaces of revolution (native).
//   Everything else (loft/sweep/threads/holed-or-typed extrude, arc/spline
//   revolve, fillet/shell/boolean/transform/exchange/reference-geometry) FALLS
//   THROUGH to the fallback engine — the native path never fakes them.
//
// SHAPE COEXISTENCE. The facade owns ONE ShapeRegistry mapping CCShapeId ->
// EngineShape (std::shared_ptr<void>). The OCCT adapter type-erases an OcctShape
// behind that void; NativeEngine type-erases a NativeShape (a native
// topology::Shape). A void built by one engine MUST NOT be read by the other
// (the casts are unchecked). NativeEngine therefore tracks the raw pointers it
// created (isNative) and:
//   * serves body-consuming ops it CAN do natively (tessellate / face_meshes /
//     mass_properties / bounding_box) for its OWN native bodies, and
//   * forwards a body it did NOT create to the fallback engine (an OCCT body),
//   * returns a clean "unsupported for native body" Error for a native body given
//     to an op with no native implementation (never hands a native void to OCCT).
//
// This header is public-safe: it includes only IEngine.h and STL — NO OCCT and NO
// native-geometry headers — so it can be included anywhere. The heavy native
// geometry + the shape holder live entirely in native_engine.cpp.

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "engine/IEngine.h"

namespace cyber {

// Build the fallback engine for the current build: an OcctEngine when compiled
// with CYBERCAD_HAS_OCCT, else the StubEngine. Defined in native_engine.cpp.
std::shared_ptr<IEngine> make_native_fallback_engine();

class NativeEngine final : public IEngine {
public:
    // `fallback` receives every capability NativeEngine does not implement, and
    // every body-consuming op on a shape NativeEngine did not build. If null, the
    // build's fallback (OCCT or stub) is created lazily.
    explicit NativeEngine(std::shared_ptr<IEngine> fallback = nullptr);

    std::string name() const override;
    bool available() const override;  // true — the native engine is always "linked"

    // Parallel / GPU toggles: forwarded to the fallback so cc_set_parallel /
    // cc_set_gpu_tessellation keep driving the same process-wide policy the OCCT
    // paths read, whichever engine is active.
    void set_parallel(bool enabled) override;
    bool parallel_enabled() const override;
    void set_gpu_tessellation(bool enabled) override;
    bool gpu_tessellation_enabled() const override;

    // ── construct: NATIVE (fall through to fallback when the input is a case the
    //    native builders defer, e.g. a degenerate/holed profile) ────────────────
    ShapeResult solid_extrude(const double* profileXY, int pointCount, double depth) override;
    ShapeResult solid_revolve(const double* profileXY, int pointCount, double angleRadians) override;

    // ── body-consuming ops: native for a native body, else fallback ────────────
    Result<MeshData> tessellate(EngineShape body, double deflection) override;
    Result<std::vector<FaceMeshData>> face_meshes(EngineShape body, double deflection) override;
    Result<MassData> mass_properties(EngineShape body) override;
    Result<std::vector<double>> bounding_box(EngineShape body) override;
    // Topology query native for a native body (Vertex/Edge/Face counts via the
    // native Explorer); an OCCT body forwards to the fallback.
    Result<std::vector<int>> subshape_ids(EngineShape body, int kind) override;

    // ── everything else falls through to the fallback engine ────────────────────
    Result<MeshData> extrude_mesh(const double* profileXY, int pointCount, double depth) override;
    ShapeResult solid_loft(const double* bottomXY, int bottomCount, const double* topXY,
                           int topCount, double depth) override;
    ShapeResult solid_loft_wires(const double* aXYZ, int aCount, const double* bXYZ,
                                 int bCount) override;
    ShapeResult solid_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                            int pathCount) override;
    ShapeResult twisted_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                              int pathCount, double twistRadians, double scaleEnd) override;
    ShapeResult loft_along_rail(const double* railXYZ, int railCount, const double* profileA_XY,
                                int aCount, const double* profileB_XY, int bCount) override;
    ShapeResult guided_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                             int pathCount, const double* guideXYZ, int guideCount) override;
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

    ShapeResult fillet_edges(EngineShape body, const int* edgeIds, int edgeCount,
                             double radius) override;
    ShapeResult fillet_edges_variable(EngineShape body, const int* edgeIds, int edgeCount,
                                      double radius1, double radius2) override;
    ShapeResult chamfer_edges(EngineShape body, const int* edgeIds, int edgeCount,
                              double distance) override;
    ShapeResult shell(EngineShape body, const int* faceIds, int faceCount,
                      double thickness) override;
    ShapeResult offset_face(EngineShape body, int faceId, double distance) override;
    ShapeResult replace_face(EngineShape body, int faceId, double offset, double tiltDeg) override;
    ShapeResult replace_face_to_plane(EngineShape body, int faceId, double px, double py, double pz,
                                      double nx, double ny, double nz) override;
    ShapeResult fillet_face(EngineShape body, int faceId, double radius) override;
    ShapeResult split_plane(EngineShape body, double ox, double oy, double oz, double nx, double ny,
                            double nz, int keepPositive) override;
    ShapeResult full_round_fillet(EngineShape body, int faceId) override;
    ShapeResult full_round_fillet_faces(EngineShape body, int leftFaceId, int middleFaceId,
                                        int rightFaceId) override;
    ShapeResult fillet_edges_g2(EngineShape body, const int* edgeIds, int edgeCount,
                                double radius) override;

    ShapeResult boolean_op(EngineShape a, EngineShape b, int op) override;
    ShapeResult thread_apply(EngineShape shaft, EngineShape thread, int op) override;

    Result<std::vector<EdgePolylineData>> edge_polylines(EngineShape body) override;
    Result<std::vector<double>> principal_moments(EngineShape body) override;
    Result<std::vector<double>> face_axis(EngineShape body, int faceId) override;
    Result<std::vector<double>> ref_plane_from_face(EngineShape body, int faceId) override;
    Result<std::vector<double>> ref_axis_from_edge(EngineShape body, int edgeId) override;
    Result<std::vector<double>> ref_axis_from_face(EngineShape body, int faceId) override;
    Result<std::vector<int>> tangent_chain(EngineShape body, const int* edgeIds,
                                           int edgeCount) override;
    Result<std::vector<int>> outer_rim_chain(EngineShape body, const int* edgeIds,
                                             int edgeCount) override;
    Result<std::vector<double>> offset_face_boundary(EngineShape body, int faceId,
                                                     double distance) override;

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

    Result<void> step_export(EngineShape body, const char* path) override;
    ShapeResult step_import(const char* path) override;
    Result<void> iges_export(EngineShape body, const char* path) override;
    ShapeResult iges_import(const char* path) override;

private:
    // Lazily create and return the fallback engine.
    IEngine& fallback() const;

    // True iff `handle` wraps a NativeShape this engine built (tracked pointer set).
    bool isNative(const EngineShape& handle) const;

    // Register a freshly built native EngineShape so isNative() recognises it.
    EngineShape track(EngineShape handle) const;

    mutable std::mutex mutex_;
    mutable std::shared_ptr<IEngine> fallback_;
    // Raw pointers of NativeShape holders this engine created. Membership test
    // only — the shared_ptr keeping each alive lives in the facade's registry.
    mutable std::unordered_set<const void*> nativeShapes_;
};

}  // namespace cyber

#endif  // CYBERCADKERNEL_ENGINE_NATIVE_NATIVE_ENGINE_H
