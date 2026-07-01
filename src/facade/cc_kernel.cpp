// Public C facade: every cc_* entry point is a thin, guarded delegation to the
// active engine. The facade owns the process-wide ShapeRegistry (CCShapeId <->
// EngineShape) and every buffer allocate/free helper the ABI exposes. It never
// leaks a C++ or engine type across the boundary: engine Result<T> outcomes are
// collapsed to 0/nil + cc_last_error, and internal POD results are copied into
// C-owned malloc buffers.

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "core/guard.h"
#include "core/result.h"
#include "core/shape_registry.h"
#include "cybercadkernel/cc_kernel.h"
#include "engine/IEngine.h"

namespace {

using cyber::active_engine;
using cyber::EdgePolylineData;
using cyber::EngineShape;
using cyber::FaceMeshData;
using cyber::MassData;
using cyber::MeshData;
using cyber::ProfileSeg;
using cyber::Result;
using cyber::set_last_error;
using cyber::ShapeRegistry;
using cyber::ShapeResult;

// Process-wide shape registry backing every CCShapeId.
ShapeRegistry& registry() {
    static ShapeRegistry reg;
    return reg;
}

EngineShape resolve(CCShapeId id) {
    return registry().get(id);
}

// ── buffer helpers (C-owned; freed by the matching cc_*_free) ─────────────────

CCMesh empty_mesh() {
    CCMesh m;
    m.vertices = nullptr;
    m.vertexCount = 0;
    m.triangles = nullptr;
    m.triangleCount = 0;
    return m;
}

double* copy_doubles(const std::vector<double>& src) {
    if (src.empty()) {
        return nullptr;
    }
    auto* out = static_cast<double*>(std::malloc(src.size() * sizeof(double)));
    if (out) {
        std::memcpy(out, src.data(), src.size() * sizeof(double));
    }
    return out;
}

int* copy_ints(const std::vector<int>& src) {
    if (src.empty()) {
        return nullptr;
    }
    auto* out = static_cast<int*>(std::malloc(src.size() * sizeof(int)));
    if (out) {
        std::memcpy(out, src.data(), src.size() * sizeof(int));
    }
    return out;
}

CCMesh alloc_mesh(const MeshData& data) {
    CCMesh out = empty_mesh();
    out.vertices = copy_doubles(data.vertices);
    out.vertexCount = static_cast<int>(data.vertices.size() / 3);
    out.triangles = copy_ints(data.triangles);
    out.triangleCount = static_cast<int>(data.triangles.size() / 3);
    return out;
}

// Collapse a Result<EngineShape> to a CCShapeId (0 on failure, recording why).
CCShapeId finish_shape(ShapeResult& r) {
    if (!r) {
        set_last_error(r.error().message);
        return 0;
    }
    EngineShape shape = std::move(r.value());
    if (!shape) {
        set_last_error("engine produced a null shape");
        return 0;
    }
    return registry().add(std::move(shape));
}

CCMesh finish_mesh(Result<MeshData>& r) {
    if (!r) {
        set_last_error(r.error().message);
        return empty_mesh();
    }
    return alloc_mesh(r.value());
}

// Fill a fixed-size caller buffer from an N-value engine query; returns 1/0.
int finish_fixed(Result<std::vector<double>>& r, double* out, std::size_t n) {
    if (!r) {
        set_last_error(r.error().message);
        return 0;
    }
    if (r.value().size() < n) {
        set_last_error("engine returned too few values");
        return 0;
    }
    if (out) {
        std::copy_n(r.value().data(), n, out);
    }
    return 1;
}

// Emit an int-id vector through an int** out-param; returns the id count.
int finish_ints(Result<std::vector<int>>& r, int** outIds) {
    if (outIds) {
        *outIds = nullptr;
    }
    if (!r) {
        set_last_error(r.error().message);
        return 0;
    }
    const std::vector<int>& ids = r.value();
    if (outIds) {
        *outIds = copy_ints(ids);
    }
    return static_cast<int>(ids.size());
}

std::vector<ProfileSeg> to_profile_segs(const CCProfileSeg* segs, int segCount) {
    std::vector<ProfileSeg> out;
    if (!segs || segCount <= 0) {
        return out;
    }
    out.reserve(static_cast<std::size_t>(segCount));
    for (int i = 0; i < segCount; ++i) {
        const CCProfileSeg& s = segs[i];
        ProfileSeg p;
        p.kind = s.kind;
        p.x0 = s.x0; p.y0 = s.y0; p.x1 = s.x1; p.y1 = s.y1;
        p.cx = s.cx; p.cy = s.cy; p.r = s.r;
        p.a0 = s.a0; p.a1 = s.a1;
        p.ptOffset = s.ptOffset; p.ptCount = s.ptCount;
        out.push_back(p);
    }
    return out;
}

}  // namespace

extern "C" {

// ── legacy mesh extrude ───────────────────────────────────────────────────────

CCMesh cc_extrude(const double* profileXY, int pointCount, double depth) {
    return cyber::guard(
        [&]() -> CCMesh {
            auto r = active_engine()->extrude_mesh(profileXY, pointCount, depth);
            return finish_mesh(r);
        },
        empty_mesh());
}

void cc_mesh_free(CCMesh mesh) {
    std::free(mesh.vertices);
    std::free(mesh.triangles);
}

// ── availability + error ──────────────────────────────────────────────────────

int cc_brep_available(void) {
    return cyber::guard([]() -> int { return active_engine()->available() ? 1 : 0; }, 0);
}

const char* cc_last_error(void) {
    // Must not clear: return the current thread's last recorded message.
    return cyber::last_error_cstr();
}

// ── construction ──────────────────────────────────────────────────────────────

CCShapeId cc_solid_extrude(const double* profileXY, int pointCount, double depth) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->solid_extrude(profileXY, pointCount, depth);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_revolve(const double* profileXY, int pointCount, double angleRadians) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->solid_revolve(profileXY, pointCount, angleRadians);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_loft(const double* bottomXY, int bottomCount, const double* topXY, int topCount,
                        double depth) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->solid_loft(bottomXY, bottomCount, topXY, topCount, depth);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_loft_wires(const double* aXYZ, int aCount, const double* bXYZ, int bCount) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->solid_loft_wires(aXYZ, aCount, bXYZ, bCount);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                         int pathCount) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->solid_sweep(profileXY, profileCount, pathXYZ, pathCount);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_twisted_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                           int pathCount, double twistRadians, double scaleEnd) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->twisted_sweep(profileXY, profileCount, pathXYZ, pathCount,
                                                    twistRadians, scaleEnd);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_loft_along_rail(const double* railXYZ, int railCount, const double* profileA_XY,
                             int aCount, const double* profileB_XY, int bCount) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->loft_along_rail(railXYZ, railCount, profileA_XY, aCount,
                                                      profileB_XY, bCount);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_guided_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                          int pathCount, const double* guideXYZ, int guideCount) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->guided_sweep(profileXY, profileCount, pathXYZ, pathCount,
                                                   guideXYZ, guideCount);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_wrap_emboss(CCShapeId body, int faceId, const double* profileXY, int count,
                         double depth, int boss) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->wrap_emboss(resolve(body), faceId, profileXY, count, depth,
                                                  boss);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_helical_thread(double majorRadiusMM, double pitchMM, double turns, double depthMM,
                            double flankAngleDeg, double pointsPerMM, int samplesPerTurn) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->helical_thread(majorRadiusMM, pitchMM, turns, depthMM,
                                                     flankAngleDeg, pointsPerMM, samplesPerTurn);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_tapered_thread(double topRadiusMM, double tipRadiusMM, double pitchMM, double turns,
                            double depthMM, double flankAngleDeg, double pointsPerMM,
                            int samplesPerTurn) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->tapered_thread(topRadiusMM, tipRadiusMM, pitchMM, turns,
                                                     depthMM, flankAngleDeg, pointsPerMM,
                                                     samplesPerTurn);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_tapered_shank(double radiusMM, double fullHeightMM, double taperHeightMM,
                           double pointsPerMM) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->tapered_shank(radiusMM, fullHeightMM, taperHeightMM,
                                                    pointsPerMM);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_extrude_holes(const double* outerXY, int outerCount,
                                 const double* holesCenterRadius, int holeCount, double depth) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->solid_extrude_holes(outerXY, outerCount, holesCenterRadius,
                                                          holeCount, depth);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_extrude_polyholes(const double* outerXY, int outerCount, const double* holesXY,
                                     const int* holeCounts, int holeCount, double depth) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->solid_extrude_polyholes(outerXY, outerCount, holesXY,
                                                              holeCounts, holeCount, depth);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_extrude_profile(const CCProfileSeg* segs, int segCount,
                                   const double* holesCenterRadius, int holeCount,
                                   const double* splineXY, int splineXYCount, double depth) {
    return cyber::guard(
        [&]() -> CCShapeId {
            std::vector<ProfileSeg> s = to_profile_segs(segs, segCount);
            auto r = active_engine()->solid_extrude_profile(s.data(), static_cast<int>(s.size()),
                                                            holesCenterRadius, holeCount, splineXY,
                                                            splineXYCount, depth);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_extrude_profile_polyholes(const CCProfileSeg* segs, int segCount,
                                             const double* holesCenterRadius, int circleCount,
                                             const double* polyXY, const int* polyCounts,
                                             int polyCount, const double* splineXY,
                                             int splineXYCount, double depth) {
    return cyber::guard(
        [&]() -> CCShapeId {
            std::vector<ProfileSeg> s = to_profile_segs(segs, segCount);
            auto r = active_engine()->solid_extrude_profile_polyholes(
                s.data(), static_cast<int>(s.size()), holesCenterRadius, circleCount, polyXY,
                polyCounts, polyCount, splineXY, splineXYCount, depth);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_solid_revolve_profile(const CCProfileSeg* segs, int segCount, double ax, double ay,
                                   double adx, double ady, const double* splineXY, int splineXYCount,
                                   double angleRadians) {
    return cyber::guard(
        [&]() -> CCShapeId {
            std::vector<ProfileSeg> s = to_profile_segs(segs, segCount);
            auto r = active_engine()->solid_revolve_profile(s.data(), static_cast<int>(s.size()),
                                                            ax, ay, adx, ady, splineXY,
                                                            splineXYCount, angleRadians);
            return finish_shape(r);
        },
        CCShapeId{0});
}

// ── feature edits ─────────────────────────────────────────────────────────────

CCShapeId cc_fillet_edges(CCShapeId body, const int* edgeIds, int edgeCount, double radius) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->fillet_edges(resolve(body), edgeIds, edgeCount, radius);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_fillet_edges_variable(CCShapeId body, const int* edgeIds, int edgeCount,
                                   double radius1, double radius2) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->fillet_edges_variable(resolve(body), edgeIds, edgeCount,
                                                            radius1, radius2);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_chamfer_edges(CCShapeId body, const int* edgeIds, int edgeCount, double distance) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->chamfer_edges(resolve(body), edgeIds, edgeCount, distance);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_shell(CCShapeId body, const int* faceIds, int faceCount, double thickness) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->shell(resolve(body), faceIds, faceCount, thickness);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_offset_face(CCShapeId body, int faceId, double distance) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->offset_face(resolve(body), faceId, distance);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_replace_face(CCShapeId body, int faceId, double offset, double tiltDeg) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->replace_face(resolve(body), faceId, offset, tiltDeg);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_replace_face_to_plane(CCShapeId body, int faceId, double px, double py, double pz,
                                   double nx, double ny, double nz) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->replace_face_to_plane(resolve(body), faceId, px, py, pz, nx,
                                                            ny, nz);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_fillet_face(CCShapeId body, int faceId, double radius) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->fillet_face(resolve(body), faceId, radius);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_split_plane(CCShapeId body, double ox, double oy, double oz, double nx, double ny,
                         double nz, int keepPositive) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->split_plane(resolve(body), ox, oy, oz, nx, ny, nz,
                                                  keepPositive);
            return finish_shape(r);
        },
        CCShapeId{0});
}

// ── booleans ──────────────────────────────────────────────────────────────────

CCShapeId cc_boolean(CCShapeId a, CCShapeId b, int op) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->boolean_op(resolve(a), resolve(b), op);
            return finish_shape(r);
        },
        CCShapeId{0});
}

// ── tessellation ──────────────────────────────────────────────────────────────

CCMesh cc_tessellate(CCShapeId body, double deflection) {
    return cyber::guard(
        [&]() -> CCMesh {
            auto r = active_engine()->tessellate(resolve(body), deflection);
            return finish_mesh(r);
        },
        empty_mesh());
}

// ── queries ───────────────────────────────────────────────────────────────────

CCMassProps cc_mass_properties(CCShapeId body) {
    CCMassProps invalid{};  // all zero, valid == 0
    return cyber::guard(
        [&]() -> CCMassProps {
            auto r = active_engine()->mass_properties(resolve(body));
            if (!r) {
                set_last_error(r.error().message);
                return invalid;
            }
            const MassData& m = r.value();
            CCMassProps out{};
            out.volume = m.volume;
            out.area = m.area;
            out.cx = m.cx;
            out.cy = m.cy;
            out.cz = m.cz;
            out.valid = m.valid ? 1 : 0;
            return out;
        },
        invalid);
}

int cc_principal_moments(CCShapeId body, double* out3) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->principal_moments(resolve(body));
            return finish_fixed(r, out3, 3);
        },
        0);
}

int cc_bounding_box(CCShapeId body, double* out6) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->bounding_box(resolve(body));
            return finish_fixed(r, out6, 6);
        },
        0);
}

int cc_face_axis(CCShapeId body, int faceId, double* out6) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->face_axis(resolve(body), faceId);
            return finish_fixed(r, out6, 6);
        },
        0);
}

int cc_subshape_ids(CCShapeId body, int kind, int** outIds) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->subshape_ids(resolve(body), kind);
            return finish_ints(r, outIds);
        },
        0);
}

void cc_ints_free(int* ids) {
    std::free(ids);
}

int cc_tangent_chain(CCShapeId body, const int* edgeIds, int edgeCount, int** outIds) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->tangent_chain(resolve(body), edgeIds, edgeCount);
            return finish_ints(r, outIds);
        },
        0);
}

int cc_outer_rim_chain(CCShapeId body, const int* edgeIds, int edgeCount, int** outIds) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->outer_rim_chain(resolve(body), edgeIds, edgeCount);
            return finish_ints(r, outIds);
        },
        0);
}

int cc_edge_polylines(CCShapeId body, CCEdgePolyline** outEdges) {
    return cyber::guard(
        [&]() -> int {
            if (outEdges) {
                *outEdges = nullptr;
            }
            auto r = active_engine()->edge_polylines(resolve(body));
            if (!r) {
                set_last_error(r.error().message);
                return 0;
            }
            const std::vector<EdgePolylineData>& edges = r.value();
            if (edges.empty() || !outEdges) {
                return static_cast<int>(edges.size());
            }
            auto* out = static_cast<CCEdgePolyline*>(
                std::malloc(edges.size() * sizeof(CCEdgePolyline)));
            if (!out) {
                set_last_error("out of memory");
                return 0;
            }
            for (std::size_t i = 0; i < edges.size(); ++i) {
                out[i].edgeId = edges[i].edgeId;
                out[i].points = copy_doubles(edges[i].points);
                out[i].pointCount = static_cast<int>(edges[i].points.size() / 3);
            }
            *outEdges = out;
            return static_cast<int>(edges.size());
        },
        0);
}

void cc_edge_polylines_free(CCEdgePolyline* edges, int count) {
    if (!edges) {
        return;
    }
    for (int i = 0; i < count; ++i) {
        std::free(edges[i].points);
    }
    std::free(edges);
}

int cc_offset_face_boundary(CCShapeId body, int faceId, double distance, double** outXYZ) {
    return cyber::guard(
        [&]() -> int {
            if (outXYZ) {
                *outXYZ = nullptr;
            }
            auto r = active_engine()->offset_face_boundary(resolve(body), faceId, distance);
            if (!r) {
                set_last_error(r.error().message);
                return 0;
            }
            const std::vector<double>& pts = r.value();
            if (outXYZ) {
                *outXYZ = copy_doubles(pts);
            }
            return static_cast<int>(pts.size() / 3);
        },
        0);
}

void cc_points_free(double* p) {
    std::free(p);
}

int cc_face_meshes(CCShapeId body, double deflection, CCFaceMesh** outFaces) {
    return cyber::guard(
        [&]() -> int {
            if (outFaces) {
                *outFaces = nullptr;
            }
            auto r = active_engine()->face_meshes(resolve(body), deflection);
            if (!r) {
                set_last_error(r.error().message);
                return 0;
            }
            const std::vector<FaceMeshData>& faces = r.value();
            if (faces.empty() || !outFaces) {
                return static_cast<int>(faces.size());
            }
            auto* out =
                static_cast<CCFaceMesh*>(std::malloc(faces.size() * sizeof(CCFaceMesh)));
            if (!out) {
                set_last_error("out of memory");
                return 0;
            }
            for (std::size_t i = 0; i < faces.size(); ++i) {
                out[i].faceId = faces[i].faceId;
                out[i].vertices = copy_doubles(faces[i].vertices);
                out[i].vertexCount = static_cast<int>(faces[i].vertices.size() / 3);
                out[i].triangles = copy_ints(faces[i].triangles);
                out[i].triangleCount = static_cast<int>(faces[i].triangles.size() / 3);
            }
            *outFaces = out;
            return static_cast<int>(faces.size());
        },
        0);
}

void cc_face_meshes_free(CCFaceMesh* faces, int count) {
    if (!faces) {
        return;
    }
    for (int i = 0; i < count; ++i) {
        std::free(faces[i].vertices);
        std::free(faces[i].triangles);
    }
    std::free(faces);
}

// ── data exchange ─────────────────────────────────────────────────────────────

int cc_step_export(CCShapeId body, const char* path) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->step_export(resolve(body), path);
            if (!r) {
                set_last_error(r.error().message);
                return 0;
            }
            return 1;
        },
        0);
}

CCShapeId cc_step_import(const char* path) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->step_import(path);
            return finish_shape(r);
        },
        CCShapeId{0});
}

int cc_iges_export(CCShapeId body, const char* path) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->iges_export(resolve(body), path);
            if (!r) {
                set_last_error(r.error().message);
                return 0;
            }
            return 1;
        },
        0);
}

CCShapeId cc_iges_import(const char* path) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->iges_import(path);
            return finish_shape(r);
        },
        CCShapeId{0});
}

// ── transforms ────────────────────────────────────────────────────────────────

CCShapeId cc_scale_shape(CCShapeId body, double factor) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->scale_shape(resolve(body), factor);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_scale_shape_about(CCShapeId body, double cx, double cy, double cz, double factor) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->scale_shape_about(resolve(body), cx, cy, cz, factor);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_rotate_shape_about(CCShapeId body, double cx, double cy, double cz, double ax,
                                double ay, double az, double angleRadians) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->rotate_shape_about(resolve(body), cx, cy, cz, ax, ay, az,
                                                         angleRadians);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_mirror_shape(CCShapeId body, double px, double py, double pz, double nx, double ny,
                          double nz) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->mirror_shape(resolve(body), px, py, pz, nx, ny, nz);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_translate_shape(CCShapeId body, double tx, double ty, double tz) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->translate_shape(resolve(body), tx, ty, tz);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_place_on_frame(CCShapeId body, double ox, double oy, double oz, double ux, double uy,
                            double uz, double vx, double vy, double vz) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->place_on_frame(resolve(body), ox, oy, oz, ux, uy, uz, vx, vy,
                                                     vz);
            return finish_shape(r);
        },
        CCShapeId{0});
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void cc_shape_release(CCShapeId body) {
    cyber::guard_void([&]() { registry().release(body); });
}

}  // extern "C"
