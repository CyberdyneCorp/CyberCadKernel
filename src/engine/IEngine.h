#ifndef CYBERCADKERNEL_ENGINE_IENGINE_H
#define CYBERCADKERNEL_ENGINE_IENGINE_H

// Internal geometry-engine abstraction. The facade delegates every geometric
// cc_* call to the active IEngine. Methods take POD inputs (raw C pointers +
// counts, matching the ABI) and opaque EngineShape handles, and return
// EngineShape / POD results wrapped in Result<T> — NO OCCT (or any engine) type
// appears here, so the OCCT adapter, a future native adapter, and the stub can
// all implement the same interface and coexist behind one facade call.
//
// A shape is carried across the boundary as EngineShape (std::shared_ptr<void>);
// each adapter type-erases its own shape holder into it (the OCCT adapter wraps a
// TopoDS_Shape). The facade owns the ShapeRegistry that maps CCShapeId <->
// EngineShape, so the engine never sees a handle id.

#include <memory>
#include <string>
#include <vector>

#include "core/result.h"
#include "core/shape_registry.h"  // EngineShape
#include "cybercadkernel/cc_kernel.h"

namespace cyber {

// ── Internal POD results (converted to CCMesh / CCMassProps / ... by facade) ──

struct MeshData {
    std::vector<double> vertices;  // x,y,z triplets
    std::vector<int> triangles;    // i,j,k triplets
};

struct MassData {
    double volume = 0.0;
    double area = 0.0;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    bool valid = false;
};

struct EdgePolylineData {
    int edgeId = 0;
    std::vector<double> points;  // x,y,z triplets
};

struct FaceMeshData {
    int faceId = 0;
    std::vector<double> vertices;  // x,y,z triplets
    std::vector<int> triangles;    // i,j,k triplets (face-local)
};

// A profile segment mirrored from CCProfileSeg but kept in engine-internal form.
struct ProfileSeg {
    int kind = 0;
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    double cx = 0, cy = 0, r = 0;
    double a0 = 0, a1 = 0;
    int ptOffset = 0, ptCount = 0;
};

// Convenience alias for "returns a built/edited shape".
using ShapeResult = Result<EngineShape>;

// The unsupported sentinel every default method returns; a native adapter can
// implement a subset and leave the rest falling through to this (or to OCCT).
inline Error engine_unsupported(const char* op) {
    return make_error(std::string("operation not supported by active engine: ") + op);
}

// ── Engine interface, grouped by capability ───────────────────────────────────

class IEngine {
public:
    virtual ~IEngine() = default;

    // Identity / availability.
    virtual std::string name() const = 0;
    virtual bool available() const = 0;  // maps to cc_brep_available()

    // ── parallel control (ADDITIVE) ─────────────────────────────────────────────
    // Toggle multi-core execution of boolean/mesh so the facade's cc_set_parallel
    // / cc_parallel_enabled can drive a serial-vs-parallel A/B (Phase 1 audit).
    // Default no-op: the stub (and any engine without a parallel path) inherits a
    // disabled toggle; the OCCT adapter overrides both to route through
    // occt::ParallelPolicy.
    virtual void set_parallel(bool /*enabled*/) {}
    virtual bool parallel_enabled() const { return false; }

    // ── GPU tessellation control (ADDITIVE) ─────────────────────────────────────
    // Toggle the GPU surface-evaluation tessellation path so the facade's
    // cc_set_gpu_tessellation / cc_gpu_tessellation_enabled can opt into GPU
    // meshing. Default no-op / disabled: the stub and any engine without a GPU
    // path inherit "off" (so a non-Metal build reports 0). The OCCT adapter
    // overrides these; even there the flag only takes effect when the build was
    // compiled with CYBERCAD_HAS_METAL.
    virtual void set_gpu_tessellation(bool /*enabled*/) {}
    virtual bool gpu_tessellation_enabled() const { return false; }

    // ── construct ─────────────────────────────────────────────────────────────
    virtual Result<MeshData> extrude_mesh(const double* profileXY, int pointCount, double depth) {
        (void)profileXY; (void)pointCount; (void)depth;
        return engine_unsupported("extrude_mesh");
    }
    virtual ShapeResult solid_extrude(const double* profileXY, int pointCount, double depth) {
        (void)profileXY; (void)pointCount; (void)depth;
        return engine_unsupported("solid_extrude");
    }
    virtual ShapeResult solid_revolve(const double* profileXY, int pointCount, double angleRadians) {
        (void)profileXY; (void)pointCount; (void)angleRadians;
        return engine_unsupported("solid_revolve");
    }
    virtual ShapeResult solid_loft(const double* bottomXY, int bottomCount, const double* topXY,
                                   int topCount, double depth) {
        (void)bottomXY; (void)bottomCount; (void)topXY; (void)topCount; (void)depth;
        return engine_unsupported("solid_loft");
    }
    virtual ShapeResult solid_loft_wires(const double* aXYZ, int aCount, const double* bXYZ,
                                         int bCount) {
        (void)aXYZ; (void)aCount; (void)bXYZ; (void)bCount;
        return engine_unsupported("solid_loft_wires");
    }
    virtual ShapeResult solid_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                                    int pathCount) {
        (void)profileXY; (void)profileCount; (void)pathXYZ; (void)pathCount;
        return engine_unsupported("solid_sweep");
    }
    virtual ShapeResult twisted_sweep(const double* profileXY, int profileCount,
                                      const double* pathXYZ, int pathCount, double twistRadians,
                                      double scaleEnd) {
        (void)profileXY; (void)profileCount; (void)pathXYZ; (void)pathCount; (void)twistRadians;
        (void)scaleEnd;
        return engine_unsupported("twisted_sweep");
    }
    virtual ShapeResult loft_along_rail(const double* railXYZ, int railCount,
                                        const double* profileA_XY, int aCount,
                                        const double* profileB_XY, int bCount) {
        (void)railXYZ; (void)railCount; (void)profileA_XY; (void)aCount; (void)profileB_XY;
        (void)bCount;
        return engine_unsupported("loft_along_rail");
    }
    virtual ShapeResult guided_sweep(const double* profileXY, int profileCount,
                                     const double* pathXYZ, int pathCount, const double* guideXYZ,
                                     int guideCount) {
        (void)profileXY; (void)profileCount; (void)pathXYZ; (void)pathCount; (void)guideXYZ;
        (void)guideCount;
        return engine_unsupported("guided_sweep");
    }
    virtual ShapeResult wrap_emboss(EngineShape body, int faceId, const double* profileXY, int count,
                                    double depth, int boss) {
        (void)body; (void)faceId; (void)profileXY; (void)count; (void)depth; (void)boss;
        return engine_unsupported("wrap_emboss");
    }
    virtual ShapeResult helical_thread(double majorRadiusMM, double pitchMM, double turns,
                                       double depthMM, double flankAngleDeg, double pointsPerMM,
                                       int samplesPerTurn) {
        (void)majorRadiusMM; (void)pitchMM; (void)turns; (void)depthMM; (void)flankAngleDeg;
        (void)pointsPerMM; (void)samplesPerTurn;
        return engine_unsupported("helical_thread");
    }
    virtual ShapeResult tapered_thread(double topRadiusMM, double tipRadiusMM, double pitchMM,
                                       double turns, double depthMM, double flankAngleDeg,
                                       double pointsPerMM, int samplesPerTurn) {
        (void)topRadiusMM; (void)tipRadiusMM; (void)pitchMM; (void)turns; (void)depthMM;
        (void)flankAngleDeg; (void)pointsPerMM; (void)samplesPerTurn;
        return engine_unsupported("tapered_thread");
    }
    virtual ShapeResult tapered_shank(double radiusMM, double fullHeightMM, double taperHeightMM,
                                      double pointsPerMM) {
        (void)radiusMM; (void)fullHeightMM; (void)taperHeightMM; (void)pointsPerMM;
        return engine_unsupported("tapered_shank");
    }
    virtual ShapeResult solid_extrude_holes(const double* outerXY, int outerCount,
                                            const double* holesCenterRadius, int holeCount,
                                            double depth) {
        (void)outerXY; (void)outerCount; (void)holesCenterRadius; (void)holeCount; (void)depth;
        return engine_unsupported("solid_extrude_holes");
    }
    virtual ShapeResult solid_extrude_polyholes(const double* outerXY, int outerCount,
                                                const double* holesXY, const int* holeCounts,
                                                int holeCount, double depth) {
        (void)outerXY; (void)outerCount; (void)holesXY; (void)holeCounts; (void)holeCount;
        (void)depth;
        return engine_unsupported("solid_extrude_polyholes");
    }
    virtual ShapeResult solid_extrude_profile(const ProfileSeg* segs, int segCount,
                                              const double* holesCenterRadius, int holeCount,
                                              const double* splineXY, int splineXYCount,
                                              double depth) {
        (void)segs; (void)segCount; (void)holesCenterRadius; (void)holeCount; (void)splineXY;
        (void)splineXYCount; (void)depth;
        return engine_unsupported("solid_extrude_profile");
    }
    virtual ShapeResult solid_extrude_profile_polyholes(
        const ProfileSeg* segs, int segCount, const double* holesCenterRadius, int circleCount,
        const double* polyXY, const int* polyCounts, int polyCount, const double* splineXY,
        int splineXYCount, double depth) {
        (void)segs; (void)segCount; (void)holesCenterRadius; (void)circleCount; (void)polyXY;
        (void)polyCounts; (void)polyCount; (void)splineXY; (void)splineXYCount; (void)depth;
        return engine_unsupported("solid_extrude_profile_polyholes");
    }
    virtual ShapeResult solid_revolve_profile(const ProfileSeg* segs, int segCount, double ax,
                                              double ay, double adx, double ady,
                                              const double* splineXY, int splineXYCount,
                                              double angleRadians) {
        (void)segs; (void)segCount; (void)ax; (void)ay; (void)adx; (void)ady; (void)splineXY;
        (void)splineXYCount; (void)angleRadians;
        return engine_unsupported("solid_revolve_profile");
    }

    // ── feature ───────────────────────────────────────────────────────────────
    virtual ShapeResult fillet_edges(EngineShape body, const int* edgeIds, int edgeCount,
                                     double radius) {
        (void)body; (void)edgeIds; (void)edgeCount; (void)radius;
        return engine_unsupported("fillet_edges");
    }
    virtual ShapeResult fillet_edges_variable(EngineShape body, const int* edgeIds, int edgeCount,
                                              double radius1, double radius2) {
        (void)body; (void)edgeIds; (void)edgeCount; (void)radius1; (void)radius2;
        return engine_unsupported("fillet_edges_variable");
    }
    virtual ShapeResult chamfer_edges(EngineShape body, const int* edgeIds, int edgeCount,
                                      double distance) {
        (void)body; (void)edgeIds; (void)edgeCount; (void)distance;
        return engine_unsupported("chamfer_edges");
    }
    virtual ShapeResult shell(EngineShape body, const int* faceIds, int faceCount,
                              double thickness) {
        (void)body; (void)faceIds; (void)faceCount; (void)thickness;
        return engine_unsupported("shell");
    }
    virtual ShapeResult offset_face(EngineShape body, int faceId, double distance) {
        (void)body; (void)faceId; (void)distance;
        return engine_unsupported("offset_face");
    }
    virtual ShapeResult replace_face(EngineShape body, int faceId, double offset, double tiltDeg) {
        (void)body; (void)faceId; (void)offset; (void)tiltDeg;
        return engine_unsupported("replace_face");
    }
    virtual ShapeResult replace_face_to_plane(EngineShape body, int faceId, double px, double py,
                                              double pz, double nx, double ny, double nz) {
        (void)body; (void)faceId; (void)px; (void)py; (void)pz; (void)nx; (void)ny; (void)nz;
        return engine_unsupported("replace_face_to_plane");
    }
    virtual ShapeResult fillet_face(EngineShape body, int faceId, double radius) {
        (void)body; (void)faceId; (void)radius;
        return engine_unsupported("fillet_face");
    }
    virtual ShapeResult split_plane(EngineShape body, double ox, double oy, double oz, double nx,
                                    double ny, double nz, int keepPositive) {
        (void)body; (void)ox; (void)oy; (void)oz; (void)nx; (void)ny; (void)nz; (void)keepPositive;
        return engine_unsupported("split_plane");
    }

    // ── Phase-3 additive feature: full-round fillet (face-consuming blend) ──────
    // Replace the middle face with a rolling-ball blend tangent to its two opposite
    // neighbours. full_round_fillet auto-detects the neighbours of faceId;
    // full_round_fillet_faces takes them explicitly. Stub inherits unsupported.
    virtual ShapeResult full_round_fillet(EngineShape body, int faceId) {
        (void)body; (void)faceId;
        return engine_unsupported("full_round_fillet");
    }
    virtual ShapeResult full_round_fillet_faces(EngineShape body, int leftFaceId, int middleFaceId,
                                                int rightFaceId) {
        (void)body; (void)leftFaceId; (void)middleFaceId; (void)rightFaceId;
        return engine_unsupported("full_round_fillet_faces");
    }

    // ── Phase-3 additive feature: G2 (curvature-continuous) blend fillet ────────
    // Curvature-continuous blend along the given edges at the nominal radius; the
    // stock fillet_edges (G1) is unchanged. Stub inherits unsupported.
    virtual ShapeResult fillet_edges_g2(EngineShape body, const int* edgeIds, int edgeCount,
                                        double radius) {
        (void)body; (void)edgeIds; (void)edgeCount; (void)radius;
        return engine_unsupported("fillet_edges_g2");
    }

    // ── boolean ───────────────────────────────────────────────────────────────
    virtual ShapeResult boolean_op(EngineShape a, EngineShape b, int op) {
        (void)a; (void)b; (void)op;
        return engine_unsupported("boolean_op");
    }

    // ── Phase-3 additive boolean: robust segmented thread apply ─────────────────
    // Apply a helical thread body to a shaft by a segmented / feature-based boolean
    // under the operation scheduler's time budget (op 0 fuse / op 1 cut; any other
    // op -> failure). Stub inherits unsupported.
    virtual ShapeResult thread_apply(EngineShape shaft, EngineShape thread, int op) {
        (void)shaft; (void)thread; (void)op;
        return engine_unsupported("thread_apply");
    }

    // ── tessellate ────────────────────────────────────────────────────────────
    virtual Result<MeshData> tessellate(EngineShape body, double deflection) {
        (void)body; (void)deflection;
        return engine_unsupported("tessellate");
    }
    virtual Result<std::vector<FaceMeshData>> face_meshes(EngineShape body, double deflection) {
        (void)body; (void)deflection;
        return engine_unsupported("face_meshes");
    }
    virtual Result<std::vector<EdgePolylineData>> edge_polylines(EngineShape body) {
        (void)body;
        return engine_unsupported("edge_polylines");
    }

    // ── query ─────────────────────────────────────────────────────────────────
    virtual Result<MassData> mass_properties(EngineShape body) {
        (void)body;
        return engine_unsupported("mass_properties");
    }
    virtual Result<std::vector<double>> principal_moments(EngineShape body) {
        (void)body;
        return engine_unsupported("principal_moments");  // expects 3 values
    }
    virtual Result<std::vector<double>> bounding_box(EngineShape body) {
        (void)body;
        return engine_unsupported("bounding_box");  // expects 6 values
    }
    virtual Result<std::vector<double>> face_axis(EngineShape body, int faceId) {
        (void)body; (void)faceId;
        return engine_unsupported("face_axis");  // expects 6 values
    }

    // ── reference geometry (Phase-3 additive; derived datum from geometry) ──────
    // The three point-only reference constructors (plane-from-3-points, offset
    // plane, axis-from-2-points) are exact fp64 math done facade-side and do NOT
    // touch the engine, so they work in every build. Only these three DERIVED
    // datums (which read an existing body's surface/curve) route through the
    // engine; the stub inherits the unsupported default -> facade returns 0.
    // Each returns [ox,oy,oz, nx|dx, ny|dy, nz|dz] (6 values).
    virtual Result<std::vector<double>> ref_plane_from_face(EngineShape body, int faceId) {
        (void)body; (void)faceId;
        return engine_unsupported("ref_plane_from_face");  // expects 6 values
    }
    virtual Result<std::vector<double>> ref_axis_from_edge(EngineShape body, int edgeId) {
        (void)body; (void)edgeId;
        return engine_unsupported("ref_axis_from_edge");  // expects 6 values
    }
    virtual Result<std::vector<double>> ref_axis_from_face(EngineShape body, int faceId) {
        (void)body; (void)faceId;
        return engine_unsupported("ref_axis_from_face");  // expects 6 values
    }
    virtual Result<std::vector<int>> subshape_ids(EngineShape body, int kind) {
        (void)body; (void)kind;
        return engine_unsupported("subshape_ids");
    }
    virtual Result<std::vector<int>> tangent_chain(EngineShape body, const int* edgeIds,
                                                   int edgeCount) {
        (void)body; (void)edgeIds; (void)edgeCount;
        return engine_unsupported("tangent_chain");
    }
    virtual Result<std::vector<int>> outer_rim_chain(EngineShape body, const int* edgeIds,
                                                     int edgeCount) {
        (void)body; (void)edgeIds; (void)edgeCount;
        return engine_unsupported("outer_rim_chain");
    }
    virtual Result<std::vector<double>> offset_face_boundary(EngineShape body, int faceId,
                                                             double distance) {
        (void)body; (void)faceId; (void)distance;
        return engine_unsupported("offset_face_boundary");  // x,y,z triplets
    }

    // ── transform ─────────────────────────────────────────────────────────────
    virtual ShapeResult scale_shape(EngineShape body, double factor) {
        (void)body; (void)factor;
        return engine_unsupported("scale_shape");
    }
    virtual ShapeResult scale_shape_about(EngineShape body, double cx, double cy, double cz,
                                          double factor) {
        (void)body; (void)cx; (void)cy; (void)cz; (void)factor;
        return engine_unsupported("scale_shape_about");
    }
    virtual ShapeResult rotate_shape_about(EngineShape body, double cx, double cy, double cz,
                                           double ax, double ay, double az, double angleRadians) {
        (void)body; (void)cx; (void)cy; (void)cz; (void)ax; (void)ay; (void)az; (void)angleRadians;
        return engine_unsupported("rotate_shape_about");
    }
    virtual ShapeResult mirror_shape(EngineShape body, double px, double py, double pz, double nx,
                                     double ny, double nz) {
        (void)body; (void)px; (void)py; (void)pz; (void)nx; (void)ny; (void)nz;
        return engine_unsupported("mirror_shape");
    }
    virtual ShapeResult translate_shape(EngineShape body, double tx, double ty, double tz) {
        (void)body; (void)tx; (void)ty; (void)tz;
        return engine_unsupported("translate_shape");
    }
    virtual ShapeResult place_on_frame(EngineShape body, double ox, double oy, double oz, double ux,
                                       double uy, double uz, double vx, double vy, double vz) {
        (void)body; (void)ox; (void)oy; (void)oz; (void)ux; (void)uy; (void)uz; (void)vx; (void)vy;
        (void)vz;
        return engine_unsupported("place_on_frame");
    }

    // ── exchange ──────────────────────────────────────────────────────────────
    virtual Result<void> step_export(EngineShape body, const char* path) {
        (void)body; (void)path;
        return engine_unsupported("step_export");
    }
    virtual ShapeResult step_import(const char* path) {
        (void)path;
        return engine_unsupported("step_import");
    }
    virtual Result<void> iges_export(EngineShape body, const char* path) {
        (void)body; (void)path;
        return engine_unsupported("iges_export");
    }
    virtual ShapeResult iges_import(const char* path) {
        (void)path;
        return engine_unsupported("iges_import");
    }
};

// ── Active-engine selector ────────────────────────────────────────────────────
//
// The facade calls active_engine() for every geometric op. OCCT and native
// adapters can coexist: each build registers its engines and one is made active;
// set_active_engine() swaps at runtime for A/B comparison during migration.
// create_default_engine() is defined by whichever engine TU is linked (the stub
// in the no-OCCT host build; the OCCT adapter otherwise).

std::shared_ptr<IEngine> create_default_engine();

std::shared_ptr<IEngine> active_engine();
void set_active_engine(std::shared_ptr<IEngine> engine);

}  // namespace cyber

#endif  // CYBERCADKERNEL_ENGINE_IENGINE_H
