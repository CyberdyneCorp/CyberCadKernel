// Public C facade: every cc_* entry point is a thin, guarded delegation to the
// active engine. The facade owns the process-wide ShapeRegistry (CCShapeId <->
// EngineShape) and every buffer allocate/free helper the ABI exposes. It never
// leaks a C++ or engine type across the boundary: engine Result<T> outcomes are
// collapsed to 0/nil + cc_last_error, and internal POD results are copied into
// C-owned malloc buffers.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "core/guard.h"
#include "core/result.h"
#include "core/shape_registry.h"
#include "cybercadkernel/cc_kernel.h"
#include "engine/IEngine.h"
#include "engine/native/native_engine.h"
#include "native/exchange/native_exchange.h"
#include "native/mesh/native_mesh.h"

namespace {

using cyber::active_engine;
using cyber::EdgePolylineData;
using cyber::EngineShape;
using cyber::FaceMeshData;
using cyber::MassData;
using cyber::MeshData;
using cyber::ProfileSeg;
using cyber::ProjectionData;
using cyber::Result;
using cyber::set_last_error;
using cyber::ShapeRegistry;
using cyber::ValidityData;
using cyber::ShapeResult;

// Process-wide shape registry backing every CCShapeId. Intentionally leaked:
// its destructor would free the stored TopoDS_Shapes during static destruction,
// which races OCCT's own static teardown and crashes (SIGSEGV at process exit).
// The library never "exits"; the OS reclaims the memory at process end.
ShapeRegistry& registry() {
    static ShapeRegistry* reg = new ShapeRegistry();
    return *reg;
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

CCTetMesh empty_tet_mesh() {
    CCTetMesh m;
    m.nodes = nullptr;
    m.nodeCount = 0;
    m.elements = nullptr;
    m.elementCount = 0;
    m.nodesPerElement = 0;
    m.order = 0;
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

// Shared body of cc_tet_mesh / cc_tet_mesh_surface: fill a closed triangle surface
// with tetrahedra via the optional AGPL TetGen backend. When the backend is not
// compiled in (default MIT build), records a clean "unavailable" error and returns
// an empty mesh — never links AGPL code, never crashes.
CCTetMesh tet_mesh_from_surface(const std::vector<double>& verts,
                                const std::vector<int>& tris,
                                CCVolumeMeshOptions opts) {
#ifdef CYBERCAD_HAS_TETGEN
    namespace mesh = cybercad::native::mesh;
    mesh::VolumeMeshOptions mopts;
    mopts.order = (opts.order == 4) ? mesh::MeshOrder::Linear : mesh::MeshOrder::Quadratic;
    mopts.target_element_size = opts.target_element_size;
    mopts.grading = opts.grading;
    if (opts.grading >= 1.0) {
        mopts.radius_edge_ratio = opts.grading;
    }
    const mesh::TetMeshResult res = mesh::tetrahedralize_surface(verts, tris, mopts);
    if (!res.ok) {
        set_last_error(res.message);
        return empty_tet_mesh();
    }
    CCTetMesh out = empty_tet_mesh();
    out.nodes = copy_doubles(res.mesh.nodes);
    out.nodeCount = res.mesh.node_count;
    out.elements = copy_ints(res.mesh.connectivity);
    out.elementCount = res.mesh.element_count;
    out.nodesPerElement = res.mesh.nodes_per_elem;
    out.order = res.mesh.order;
    return out;
#else
    (void)verts;
    (void)tris;
    (void)opts;
    set_last_error(
        "tet meshing unavailable (build with CYBERCAD_HAS_TETGEN=ON — optional, "
        "external AGPL TetGen backend)");
    return empty_tet_mesh();
#endif
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

// ── Phase-3 point-only reference geometry (exact fp64, engine-independent) ─────
// These three constructors are pure double-precision vector math, so they work in
// EVERY build (including the no-OCCT host stub) without touching active_engine().
// A single absolute tolerance on the resulting vector magnitude gates degeneracy
// (colinear/coincident points -> zero cross product/difference; zero-length
// normal), matching the reference-geometry spec's 1e-9 unit-length guarantee.
constexpr double kRefEps = 1e-9;

// Write origin + normalize(v) into out6 = [ox,oy,oz, ux,uy,uz]; return 1, or 0 if
// v is shorter than kRefEps (degenerate) or out6 is null.
int ref_emit(const double origin[3], double vx, double vy, double vz, double* out6) {
    const double mag = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (mag < kRefEps || out6 == nullptr) {
        return 0;
    }
    out6[0] = origin[0];
    out6[1] = origin[1];
    out6[2] = origin[2];
    out6[3] = vx / mag;
    out6[4] = vy / mag;
    out6[5] = vz / mag;
    return 1;
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

// ── parallel control ──────────────────────────────────────────────────────────

void cc_set_parallel(int enabled) {
    cyber::guard_void([&]() { active_engine()->set_parallel(enabled != 0); });
}

int cc_parallel_enabled(void) {
    return cyber::guard([]() -> int { return active_engine()->parallel_enabled() ? 1 : 0; }, 0);
}

// ── GPU tessellation control ────────────────────────────────────────────────────

void cc_set_gpu_tessellation(int enabled) {
    cyber::guard_void([&]() { active_engine()->set_gpu_tessellation(enabled != 0); });
}

int cc_gpu_tessellation_enabled(void) {
    return cyber::guard([]() -> int { return active_engine()->gpu_tessellation_enabled() ? 1 : 0; },
                        0);
}

// ── active engine selection ───────────────────────────────────────────────────

void cc_set_engine(int native) {
    cyber::guard_void([&]() {
        if (native != 0) {
            // Idempotent: already native → keep the SAME NativeEngine. Rebuilding it
            // would be needless churn and, historically, discarded per-instance shape
            // bookkeeping (now process-wide, but keeping the instance is still the
            // correct no-op for a redundant toggle).
            if (active_engine()->name() == "native") return;
            // Opt in to the native engine. It falls through to the build's default
            // engine (OCCT/stub) for capabilities it does not implement natively,
            // AND inherits the current parallel/GPU toggles so behaviour is
            // continuous across the swap.
            auto engine = std::make_shared<cyber::NativeEngine>();
            engine->set_parallel(active_engine()->parallel_enabled());
            engine->set_gpu_tessellation(active_engine()->gpu_tessellation_enabled());
            cyber::set_active_engine(std::move(engine));
        } else {
            // Restore the build's DEFAULT engine (OCCT where linked, else stub).
            cyber::set_active_engine(cyber::create_default_engine());
        }
    });
}

int cc_active_engine(void) {
    return cyber::guard([]() -> int { return active_engine()->name() == "native" ? 1 : 0; }, 0);
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

CCShapeId cc_solid_loft_sections(const double* sectionsXYZ, const int* counts, int sectionCount) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->solid_loft_sections(sectionsXYZ, counts, sectionCount);
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

CCShapeId cc_loft_circles(const double* c1, const double* n1, double r1, const double* c2,
                          const double* n2, double r2) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->loft_circles(c1, n1, r1, c2, n2, r2);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_loft_circle_wire(const double* cc, const double* cn, double cr, const double* wXYZ,
                              int wCount) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->loft_circle_wire(cc, cn, cr, wXYZ, wCount);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_loft_along_rails(const double* railXYZ, int railCount, const double* guideXYZ,
                              int guideCount, const double* profileA_XY, int aCount,
                              const double* profileB_XY, int bCount) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->loft_along_rails(railXYZ, railCount, guideXYZ, guideCount,
                                                       profileA_XY, aCount, profileB_XY, bCount);
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

CCShapeId cc_guided_orient_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                                 int pathCount, const double* guideXYZ, int guideCount) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->guided_orient_sweep(profileXY, profileCount, pathXYZ,
                                                          pathCount, guideXYZ, guideCount);
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

CCShapeId cc_loft_typed(const CCProfileSeg* segsA, int countA, const double* splineA,
                        int splineACount, const double* frameA, const CCProfileSeg* segsB,
                        int countB, const double* splineB, int splineBCount, const double* frameB) {
    return cyber::guard(
        [&]() -> CCShapeId {
            std::vector<ProfileSeg> a = to_profile_segs(segsA, countA);
            std::vector<ProfileSeg> b = to_profile_segs(segsB, countB);
            auto r = active_engine()->loft_typed(a.data(), static_cast<int>(a.size()), splineA,
                                                 splineACount, frameA, b.data(),
                                                 static_cast<int>(b.size()), splineB, splineBCount,
                                                 frameB);
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

CCShapeId cc_chamfer_edges_asym(CCShapeId body, const int* edgeIds, int edgeCount,
                                double distance1, double distance2) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->chamfer_edges_asym(resolve(body), edgeIds, edgeCount,
                                                         distance1, distance2);
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

CCProjection cc_project_point_on_face(CCShapeId body, int faceId, double px, double py, double pz) {
    CCProjection invalid{};  // all zero, valid == 0
    return cyber::guard(
        [&]() -> CCProjection {
            auto r = active_engine()->project_point_on_face(resolve(body), faceId, px, py, pz);
            if (!r) {
                set_last_error(r.error().message);
                return invalid;
            }
            const ProjectionData& d = r.value();
            CCProjection out{};
            out.footX = d.footX;
            out.footY = d.footY;
            out.footZ = d.footZ;
            out.distance = d.distance;
            out.valid = 1;
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

// First-failing (or undecidable) check, in the same precedence the native
// ValidityReport::reason() uses — the honest "why is this not valid" code.
static int cc_first_failure(const ValidityData& v) {
    if (!v.finite) return CC_VALID_NONFINITE;
    if (!v.closed) return CC_VALID_NOT_CLOSED;
    if (!v.oriented) return CC_VALID_BAD_ORIENTATION;
    if (!v.nondegenerate) return CC_VALID_DEGENERATE;
    if (!v.certified) return CC_VALID_SELF_INTERSECT_UNDECIDABLE;
    if (!v.noSelfIntersection) return CC_VALID_SELF_INTERSECT;
    return CC_VALID_OK;
}

int cc_check_solid(CCShapeId body, CCValidityReport* out) {
    return cyber::guard(
        [&]() -> int {
            CCValidityReport zero{};
            if (out) *out = zero;
            auto r = active_engine()->check_solid(resolve(body));
            if (!r) {
                set_last_error(r.error().message);
                return 0;  // unknown body / no engine — out already zeroed
            }
            const ValidityData& v = r.value();
            CCValidityReport rep{};
            rep.decided = v.certified ? 1 : 0;  // undecidable self-X => not decided
            rep.valid = (v.valid && v.certified) ? 1 : 0;
            rep.finite = v.finite ? 1 : 0;
            rep.closed_manifold = v.closed ? 1 : 0;
            rep.consistent_orientation = v.oriented ? 1 : 0;
            rep.no_degenerate = v.nondegenerate ? 1 : 0;
            rep.no_self_intersection = v.noSelfIntersection ? 1 : 0;
            rep.first_failure = cc_first_failure(v);
            if (out) *out = rep;
            if (!rep.decided) {  // HONEST DECLINE: never a definite verdict here
                set_last_error("cc_check_solid: self-intersection undecidable (coplanar "
                               "overlap) — verdict declined");
                return 0;
            }
            return 1;
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

int cc_shape_solid_count(CCShapeId body) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->shape_solid_count(resolve(body));
            if (!r) {
                set_last_error(r.error().message);
                return 0;
            }
            return r.value();
        },
        0);
}

CCShapeId cc_shape_solid_at(CCShapeId body, int index) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->shape_solid_at(resolve(body), index);
            return finish_shape(r);
        },
        CCShapeId{0});
}

int cc_measure_distance(CCShapeId body, int subKindA, int subIdA, int subKindB, int subIdB,
                        double* out7) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->measure_distance(resolve(body), subKindA, subIdA,
                                                       subKindB, subIdB);
            return finish_fixed(r, out7, 7);
        },
        0);
}

int cc_measure_angle(CCShapeId body, int subKindA, int subIdA, int subKindB, int subIdB,
                     double* outRadians) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->measure_angle(resolve(body), subKindA, subIdA,
                                                    subKindB, subIdB);
            return finish_fixed(r, outRadians, 1);
        },
        0);
}

int cc_surface_curvature(CCShapeId body, int faceId, double u, double v, double* out4) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->surface_curvature(resolve(body), faceId, u, v);
            return finish_fixed(r, out4, 4);
        },
        0);
}

int cc_edge_curvature(CCShapeId body, int edgeId, double t, double* outKappa) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->edge_curvature(resolve(body), edgeId, t);
            return finish_fixed(r, outKappa, 1);
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

// ── drafting: orthographic hidden-line removal (MOAT GS1, ADDITIVE) ─────────────

namespace {
CCDrawing empty_drawing() {
    CCDrawing d;
    d.visible = nullptr;
    d.visibleCount = 0;
    d.hidden = nullptr;
    d.hiddenCount = 0;
    return d;
}

// Copy an engine DrawingSegmentData vector into a C-owned CCDrawingSegment array.
CCDrawingSegment* copy_segments(const std::vector<cyber::DrawingSegmentData>& src) {
    if (src.empty()) {
        return nullptr;
    }
    auto* out = static_cast<CCDrawingSegment*>(std::malloc(src.size() * sizeof(CCDrawingSegment)));
    if (!out) {
        return nullptr;
    }
    for (std::size_t i = 0; i < src.size(); ++i) {
        out[i].ax = src[i].ax;
        out[i].ay = src[i].ay;
        out[i].bx = src[i].bx;
        out[i].by = src[i].by;
    }
    return out;
}
}  // namespace

CCDrawing cc_hlr_project(CCShapeId body, const double viewDir[3], const double up[3],
                         CCHlrOptions opts) {
    return cyber::guard(
        [&]() -> CCDrawing {
            if (!viewDir || !up) {
                set_last_error("cc_hlr_project: null viewDir/up");
                return empty_drawing();
            }
            cyber::HlrOptionsData o;
            o.deflection = opts.deflection;
            o.samplesPerEdge = opts.samplesPerEdge;
            o.surfaceOffset = opts.surfaceOffset;
            auto r = active_engine()->hlr_project(resolve(body), viewDir, up, o);
            if (!r) {
                set_last_error(r.error().message);
                return empty_drawing();
            }
            const cyber::DrawingData& d = r.value();
            CCDrawing out = empty_drawing();
            out.visible = copy_segments(d.visible);
            out.visibleCount = static_cast<int>(d.visible.size());
            out.hidden = copy_segments(d.hidden);
            out.hiddenCount = static_cast<int>(d.hidden.size());
            return out;
        },
        empty_drawing());
}

void cc_drawing_free(CCDrawing drawing) {
    std::free(drawing.visible);
    std::free(drawing.hidden);
}

// ── drafting: planar section curves (MOAT GS2, ADDITIVE) ────────────────────────

namespace {
CCSection empty_section() {
    CCSection s;
    s.loops = nullptr;
    s.loopCount = 0;
    s.totalLength = 0.0;
    s.totalArea = 0.0;
    return s;
}
}  // namespace

CCSection cc_section_plane(CCShapeId body, const double origin[3], const double normal[3]) {
    return cyber::guard(
        [&]() -> CCSection {
            if (!origin || !normal) {
                set_last_error("cc_section_plane: null origin/normal");
                return empty_section();
            }
            auto r = active_engine()->section_plane(resolve(body), origin, normal);
            if (!r) {
                set_last_error(r.error().message);
                return empty_section();
            }
            const cyber::SectionData& d = r.value();
            CCSection out = empty_section();
            out.totalLength = d.totalLength;
            out.totalArea = d.totalArea;
            out.loopCount = static_cast<int>(d.loops.size());
            if (!d.loops.empty()) {
                out.loops = static_cast<CCSectionLoop*>(
                    std::malloc(d.loops.size() * sizeof(CCSectionLoop)));
                if (!out.loops) {
                    set_last_error("cc_section_plane: out of memory");
                    return empty_section();
                }
                for (std::size_t i = 0; i < d.loops.size(); ++i) {
                    const cyber::SectionLoopData& src = d.loops[i];
                    CCSectionLoop& dst = out.loops[i];
                    dst.shape = src.shape;
                    dst.length = src.length;
                    dst.area = src.area;
                    dst.pointCount = static_cast<int>(src.pointsXYZ.size() / 3);
                    dst.pointsXYZ = nullptr;
                    if (!src.pointsXYZ.empty()) {
                        dst.pointsXYZ = static_cast<double*>(
                            std::malloc(src.pointsXYZ.size() * sizeof(double)));
                        if (dst.pointsXYZ)
                            std::copy(src.pointsXYZ.begin(), src.pointsXYZ.end(), dst.pointsXYZ);
                        else
                            dst.pointCount = 0;
                    }
                }
            }
            return out;
        },
        empty_section());
}

void cc_section_free(CCSection section) {
    if (section.loops) {
        for (int i = 0; i < section.loopCount; ++i) std::free(section.loops[i].pointsXYZ);
        std::free(section.loops);
    }
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

int cc_step_pmi_scan(const char* path, CCPmiSummary* out) {
    return cyber::guard(
        [&]() -> int {
            if (!out) {
                set_last_error("cc_step_pmi_scan: null out");
                return 0;
            }
            auto r = active_engine()->pmi_scan(path);
            if (!r) {
                set_last_error(r.error().message);
                return 0;
            }
            const auto& s = r.value();
            out->dimensions = s.dimensions;
            out->tolerances = s.tolerances;
            out->datums = s.datums;
            out->datum_targets = s.datumTargets;
            out->notes = s.notes;
            out->annotation_geometry = s.annotationGeometry;
            out->unknown = s.unknown;
            out->total = s.total;
            return 1;
        },
        0);
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

int cc_stl_export(CCShapeId body, const char* path, double deflection, int binary) {
    return cyber::guard(
        [&]() -> int {
            // Reuse the neutral tessellation path (no duplicated meshing), then hand
            // the flat POD to the OCCT-free native STL writer.
            auto r = active_engine()->tessellate(resolve(body), deflection);
            if (!r) {
                set_last_error(r.error().message);
                return 0;
            }
            const MeshData& m = r.value();
            if (!cybercad::native::exchange::stl_export_mesh(m.vertices, m.triangles,
                                                            path ? path : "", binary != 0)) {
                set_last_error("STL export failed to write file");
                return 0;
            }
            return 1;
        },
        0);
}

CCShapeId cc_stl_import(const char* path) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->stl_import(path);
            return finish_shape(r);
        },
        CCShapeId{0});
}

// ── tetrahedral volume meshing (Phase-4 additive; TetGen backend optional) ─────

CCTetMesh cc_tet_mesh(CCShapeId body, double deflection, CCVolumeMeshOptions opts) {
    return cyber::guard(
        [&]() -> CCTetMesh {
            // Reuse the neutral tessellation path (no duplicated meshing), then fill
            // the resulting closed triangle surface with tetrahedra.
            auto r = active_engine()->tessellate(resolve(body), deflection);
            if (!r) {
                set_last_error(r.error().message);
                return empty_tet_mesh();
            }
            const MeshData& m = r.value();
            return tet_mesh_from_surface(m.vertices, m.triangles, opts);
        },
        empty_tet_mesh());
}

CCTetMesh cc_tet_mesh_surface(const double* verticesXYZ, int vertexCount,
                              const int* trianglesIJK, int triangleCount,
                              CCVolumeMeshOptions opts) {
    return cyber::guard(
        [&]() -> CCTetMesh {
            if (verticesXYZ == nullptr || trianglesIJK == nullptr || vertexCount <= 0 ||
                triangleCount <= 0) {
                set_last_error("cc_tet_mesh_surface: null or empty surface input");
                return empty_tet_mesh();
            }
            std::vector<double> verts(verticesXYZ,
                                      verticesXYZ + static_cast<long>(vertexCount) * 3);
            std::vector<int> tris(trianglesIJK,
                                  trianglesIJK + static_cast<long>(triangleCount) * 3);
            return tet_mesh_from_surface(verts, tris, opts);
        },
        empty_tet_mesh());
}

void cc_tet_mesh_free(CCTetMesh mesh) {
    std::free(mesh.nodes);
    std::free(mesh.elements);
}

CCQualityReport cc_mesh_quality(CCTetMesh mesh, double min_scaled_jacobian) {
    return cyber::guard(
        [&]() -> CCQualityReport {
            const auto rep = cybercad::native::mesh::quality(
                mesh.nodes, mesh.nodeCount, mesh.elements, mesh.elementCount,
                mesh.nodesPerElement, min_scaled_jacobian);
            CCQualityReport out;
            out.min_dihedral_angle = rep.minDihedral;
            out.max_dihedral_angle = rep.maxDihedral;
            out.min_scaled_jacobian = rep.minScaledJacobian;
            out.mean_scaled_jacobian = rep.meanScaledJacobian;
            out.max_aspect_ratio = rep.maxAspectRatio;
            out.elements_below_threshold = static_cast<int>(rep.flagged.size());
            out.flagged_elements = copy_ints(rep.flagged);
            out.valid = rep.valid ? 1 : 0;
            if (!rep.valid) {
                set_last_error("mesh quality: empty or degenerate mesh");
            }
            return out;
        },
        CCQualityReport{});
}

void cc_quality_report_free(CCQualityReport report) {
    std::free(report.flagged_elements);
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

// ── Phase-3: robust thread boolean ────────────────────────────────────────────

CCShapeId cc_thread_apply(CCShapeId shaft, CCShapeId thread, int op) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->thread_apply(resolve(shaft), resolve(thread), op);
            return finish_shape(r);
        },
        CCShapeId{0});
}

// ── Phase-3: reference geometry (datum planes / axes) ─────────────────────────

int cc_ref_plane_from_points(const double p0[3], const double p1[3], const double p2[3],
                             double out6[6]) {
    return cyber::guard(
        [&]() -> int {
            if (p0 == nullptr || p1 == nullptr || p2 == nullptr) {
                return 0;
            }
            const double ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
            const double vx = p2[0] - p0[0], vy = p2[1] - p0[1], vz = p2[2] - p0[2];
            // normal = (p1-p0) x (p2-p0); |normal| ∝ triangle area -> colinear fails.
            const double nx = uy * vz - uz * vy;
            const double ny = uz * vx - ux * vz;
            const double nz = ux * vy - uy * vx;
            return ref_emit(p0, nx, ny, nz, out6);
        },
        0);
}

int cc_ref_plane_offset(const double origin[3], const double normal[3], double dist,
                        double out6[6]) {
    return cyber::guard(
        [&]() -> int {
            if (origin == nullptr || normal == nullptr || out6 == nullptr) {
                return 0;
            }
            const double mag =
                std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
            if (mag < kRefEps) {
                return 0;
            }
            const double nx = normal[0] / mag, ny = normal[1] / mag, nz = normal[2] / mag;
            const double moved[3] = {origin[0] + dist * nx, origin[1] + dist * ny,
                                     origin[2] + dist * nz};
            return ref_emit(moved, nx, ny, nz, out6);
        },
        0);
}

int cc_ref_axis_from_points(const double a[3], const double b[3], double out6[6]) {
    return cyber::guard(
        [&]() -> int {
            if (a == nullptr || b == nullptr) {
                return 0;
            }
            return ref_emit(a, b[0] - a[0], b[1] - a[1], b[2] - a[2], out6);
        },
        0);
}

int cc_ref_plane_from_face(CCShapeId body, int faceId, double out6[6]) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->ref_plane_from_face(resolve(body), faceId);
            return finish_fixed(r, out6, 6);
        },
        0);
}

int cc_ref_axis_from_edge(CCShapeId body, int edgeId, double out6[6]) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->ref_axis_from_edge(resolve(body), edgeId);
            return finish_fixed(r, out6, 6);
        },
        0);
}

int cc_ref_axis_from_face(CCShapeId body, int faceId, double out6[6]) {
    return cyber::guard(
        [&]() -> int {
            auto r = active_engine()->ref_axis_from_face(resolve(body), faceId);
            return finish_fixed(r, out6, 6);
        },
        0);
}

// ── Phase-3: full-round fillet + G2 blend ─────────────────────────────────────

CCShapeId cc_full_round_fillet(CCShapeId body, int faceId) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->full_round_fillet(resolve(body), faceId);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_full_round_fillet_faces(CCShapeId body, int leftFaceId, int middleFaceId,
                                     int rightFaceId) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->full_round_fillet_faces(resolve(body), leftFaceId,
                                                              middleFaceId, rightFaceId);
            return finish_shape(r);
        },
        CCShapeId{0});
}

CCShapeId cc_fillet_edges_g2(CCShapeId body, const int* edgeIds, int edgeCount, double radius) {
    return cyber::guard(
        [&]() -> CCShapeId {
            auto r = active_engine()->fillet_edges_g2(resolve(body), edgeIds, edgeCount, radius);
            return finish_shape(r);
        },
        CCShapeId{0});
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void cc_shape_release(CCShapeId body) {
    cyber::guard_void([&]() { registry().release(body); });
}

}  // extern "C"
