// NativeEngine implementation — the clean-room native construct ops + fallthrough.
//
// This TU pulls in the heavy native geometry (src/native/construct + tessellate +
// topology + math), all OCCT-FREE and host-buildable. The OCCT engine is referenced
// ONLY inside CYBERCAD_HAS_OCCT (for the fallthrough target); without OCCT the
// fallthrough target is the always-compiled stub. No OCCT type appears in this
// file outside that guard, so it compiles on the host.
//
// See native_engine.h for the coexistence / shape-ownership contract.

#include "engine/native/native_engine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"

#ifdef CYBERCAD_HAS_OCCT
#include "engine/occt/occt_engine.h"
#endif

namespace cyber {

namespace {

namespace ntopo = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;

// ── Native shape holder type-erased behind the registry's EngineShape ──────────
// Distinct from occt::OcctShape; NativeEngine tracks the raw pointer so it never
// hands one of these to the OCCT adapter (whose unwrap would misread it).
struct NativeShape {
    ntopo::Shape shape;
};

EngineShape wrapNative(ntopo::Shape shape) {
    auto holder = std::make_shared<NativeShape>();
    holder->shape = std::move(shape);
    return std::static_pointer_cast<void>(holder);
}

// The revolution axis the cc_solid_revolve contract implies: the Y axis through
// the origin, in the profile plane (z=0). Mirrors the OCCT adapter's solid_revolve
// (BRepPrimAPI_MakeRevol about gp_Ax1(origin, {0,1,0})).
constexpr cybercad::native::construct::RevolveAxis kRevolveYAxis{0.0, 0.0, 0.0, 1.0};

// Default tessellation deflection when a body-consuming op does not carry one
// (mass_properties / bounding_box derive from a mesh). A tight value keeps the
// area/volume within the convergence bound the native tessellator guarantees.
constexpr double kPropertyDeflection = 0.05;

// Convert a native mesh into the ABI MeshData (flat vertex/triangle arrays).
MeshData toMeshData(const ntess::Mesh& m) {
    MeshData out;
    out.vertices.reserve(m.vertices.size() * 3);
    for (const auto& p : m.vertices) {
        out.vertices.push_back(p.x);
        out.vertices.push_back(p.y);
        out.vertices.push_back(p.z);
    }
    out.triangles.reserve(m.triangles.size() * 3);
    for (const auto& t : m.triangles) {
        out.triangles.push_back(static_cast<int>(t.a));
        out.triangles.push_back(static_cast<int>(t.b));
        out.triangles.push_back(static_cast<int>(t.c));
    }
    return out;
}

}  // namespace

// ── Fallback engine factory (build-specific) ───────────────────────────────────
// The always-compiled stub provides make_stub_engine(); the OCCT adapter provides
// OcctEngine only under CYBERCAD_HAS_OCCT.
std::shared_ptr<IEngine> make_stub_engine();

std::shared_ptr<IEngine> make_native_fallback_engine() {
#ifdef CYBERCAD_HAS_OCCT
    return std::make_shared<OcctEngine>();
#else
    return make_stub_engine();
#endif
}

// ── construction / lifecycle ────────────────────────────────────────────────────

NativeEngine::NativeEngine(std::shared_ptr<IEngine> fallback) : fallback_(std::move(fallback)) {}

IEngine& NativeEngine::fallback() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fallback_) fallback_ = make_native_fallback_engine();
    return *fallback_;
}

std::string NativeEngine::name() const { return "native"; }
bool NativeEngine::available() const { return true; }

void NativeEngine::set_parallel(bool enabled) { fallback().set_parallel(enabled); }
bool NativeEngine::parallel_enabled() const { return fallback().parallel_enabled(); }
void NativeEngine::set_gpu_tessellation(bool enabled) { fallback().set_gpu_tessellation(enabled); }
bool NativeEngine::gpu_tessellation_enabled() const { return fallback().gpu_tessellation_enabled(); }

bool NativeEngine::isNative(const EngineShape& handle) const {
    if (!handle) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    return nativeShapes_.find(handle.get()) != nativeShapes_.end();
}

EngineShape NativeEngine::track(EngineShape handle) const {
    if (handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        nativeShapes_.insert(handle.get());
    }
    return handle;
}

// ── NATIVE construct ops (fall through to OCCT for deferred cases) ──────────────

ShapeResult NativeEngine::solid_extrude(const double* profileXY, int pointCount, double depth) {
    ntopo::Shape solid = cybercad::native::construct::build_prism(profileXY, pointCount, depth);
    if (solid.isNull()) {
        // Degenerate input or a case the native builder defers: hand the SAME
        // arguments to the fallback engine (honest coexistence — no faking).
        return fallback().solid_extrude(profileXY, pointCount, depth);
    }
    return track(wrapNative(std::move(solid)));
}

ShapeResult NativeEngine::solid_revolve(const double* profileXY, int pointCount,
                                        double angleRadians) {
    ntopo::Shape solid = cybercad::native::construct::build_revolution(profileXY, pointCount,
                                                                       kRevolveYAxis, angleRadians);
    if (solid.isNull()) {
        return fallback().solid_revolve(profileXY, pointCount, angleRadians);
    }
    return track(wrapNative(std::move(solid)));
}

// ── body-consuming ops: native for a native body, else fallback ─────────────────

Result<MeshData> NativeEngine::tessellate(EngineShape body, double deflection) {
    if (!isNative(body)) return fallback().tessellate(body, deflection);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    ntess::MeshParams params;
    if (deflection > 0.0) params.deflection = deflection;
    ntess::SolidMesher mesher(params);
    return toMeshData(mesher.mesh(holder->shape));
}

Result<std::vector<FaceMeshData>> NativeEngine::face_meshes(EngineShape body, double deflection) {
    if (!isNative(body)) return fallback().face_meshes(body, deflection);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    ntess::MeshParams params;
    if (deflection > 0.0) params.deflection = deflection;
    ntess::FaceMesher fm(params);

    std::vector<FaceMeshData> out;
    int faceId = 0;
    for (ntopo::Explorer ex(holder->shape, ntopo::ShapeType::Face); ex.more(); ex.next()) {
        ++faceId;  // 1-based, matching the subshape-id convention
        const ntess::Mesh m = fm.mesh(ex.current());
        FaceMeshData fmd;
        fmd.faceId = faceId;
        fmd.vertices.reserve(m.vertices.size() * 3);
        for (const auto& p : m.vertices) {
            fmd.vertices.push_back(p.x);
            fmd.vertices.push_back(p.y);
            fmd.vertices.push_back(p.z);
        }
        fmd.triangles.reserve(m.triangles.size() * 3);
        for (const auto& t : m.triangles) {
            fmd.triangles.push_back(static_cast<int>(t.a));
            fmd.triangles.push_back(static_cast<int>(t.b));
            fmd.triangles.push_back(static_cast<int>(t.c));
        }
        out.push_back(std::move(fmd));
    }
    return out;
}

Result<MassData> NativeEngine::mass_properties(EngineShape body) {
    if (!isNative(body)) return fallback().mass_properties(body);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    ntess::MeshParams params;
    params.deflection = kPropertyDeflection;
    const ntess::Mesh mesh = ntess::SolidMesher(params).mesh(holder->shape);

    MassData out;
    out.area = ntess::surfaceArea(mesh);
    // enclosedVolume is signed with +Z-outward CCW winding; a well-formed native
    // solid meshes outward, so take the magnitude for the reported (positive) volume.
    out.volume = std::fabs(ntess::enclosedVolume(mesh));
    // Centroid from the same signed-tetra decomposition (divergence theorem):
    // C = (1/V) · ⅛ Σ (aᵢ+bᵢ+cᵢ scaled)… computed here as the volume-weighted
    // tetra centroid sum over the origin fan.
    double cx = 0.0, cy = 0.0, cz = 0.0, vol6 = 0.0;
    for (const auto& t : mesh.triangles) {
        const auto& A = mesh.vertices[t.a];
        const auto& B = mesh.vertices[t.b];
        const auto& C = mesh.vertices[t.c];
        const double v = A.x * (B.y * C.z - B.z * C.y) - A.y * (B.x * C.z - B.z * C.x) +
                         A.z * (B.x * C.y - B.y * C.x);  // 6·(signed tetra volume)
        vol6 += v;
        cx += v * (A.x + B.x + C.x);
        cy += v * (A.y + B.y + C.y);
        cz += v * (A.z + B.z + C.z);
    }
    if (std::fabs(vol6) > 1e-12) {
        const double inv = 1.0 / (4.0 * vol6);  // ¼ from the tetra centroid, /vol6
        out.cx = cx * inv;
        out.cy = cy * inv;
        out.cz = cz * inv;
    }
    out.valid = ntess::isWatertight(mesh) && out.volume > 0.0;
    return out;
}

Result<std::vector<double>> NativeEngine::bounding_box(EngineShape body) {
    if (!isNative(body)) return fallback().bounding_box(body);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    // Bound the TESSELLATED body, not the raw B-rep vertices. A revolved solid's
    // topological vertices sit only at the angular STATIONS (e.g. θ = 0/120/240 for
    // a full turn), so a vertex-only AABB misses the circular extremes between them
    // (a full-radius error on a tube). Meshing at a tight deflection samples every
    // curved face densely, so the mesh AABB is within `deflection` of the exact
    // B-rep box — matching OCCT's BRepBndLib to the same bound the parity gate uses.
    // For planar prisms the mesh vertices are exact, so this is exact there too.
    ntess::MeshParams params;
    params.deflection = kPropertyDeflection;
    const ntess::Mesh mesh = ntess::SolidMesher(params).mesh(holder->shape);
    if (mesh.vertices.empty()) return make_error("bounding_box: native body meshed empty");
    const auto& v0 = mesh.vertices.front();
    double lo[3] = {v0.x, v0.y, v0.z}, hi[3] = {v0.x, v0.y, v0.z};
    for (const auto& p : mesh.vertices) {
        const double c[3] = {p.x, p.y, p.z};
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], c[k]);
            hi[k] = std::max(hi[k], c[k]);
        }
    }
    return std::vector<double>{lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]};
}

// Native topology query: Vertex/Edge/Face counts via the native Explorer, matching
// OCCT's TopExp::MapShapes + 1-based ids. kind 0 = vertex, 1 = edge, else face
// (the SAME selector the OCCT adapter's subshapeKind uses). An OCCT body forwards.
Result<std::vector<int>> NativeEngine::subshape_ids(EngineShape body, int kind) {
    if (!isNative(body)) return fallback().subshape_ids(body, kind);
    const auto* holder = static_cast<const NativeShape*>(body.get());
    const ntopo::ShapeType type = kind == 0   ? ntopo::ShapeType::Vertex
                                  : kind == 1 ? ntopo::ShapeType::Edge
                                              : ntopo::ShapeType::Face;
    // MapShapes dedups shared sub-shapes by isSame (a shared vertex/edge is counted
    // once), so the count equals OCCT's indexed-map extent; ids are 1..count.
    const ntopo::ShapeMap map = ntopo::mapShapes(holder->shape, type);
    std::vector<int> ids(map.size());
    for (std::size_t i = 0; i < map.size(); ++i) ids[i] = static_cast<int>(i) + 1;
    return ids;
}

// ── Fallthrough helper for body-consuming ops with NO native implementation ─────
// A native body handed to such an op cannot be served (and must NOT be forwarded
// to OCCT, which would misread the void). Return a clean, honest Error; an OCCT
// body forwards normally.
#define CC_NATIVE_BODY_UNSUPPORTED(op, body)                                       \
    do {                                                                           \
        if (isNative(body))                                                        \
            return make_error("operation not supported on a native body yet: " op \
                              " (native scope: extrude/revolve + tessellate/mass/bbox)"); \
    } while (0)

// ── construct fallthrough ───────────────────────────────────────────────────────

Result<MeshData> NativeEngine::extrude_mesh(const double* p, int n, double d) {
    return fallback().extrude_mesh(p, n, d);
}
ShapeResult NativeEngine::solid_loft(const double* b, int bc, const double* t, int tc, double d) {
    return fallback().solid_loft(b, bc, t, tc, d);
}
ShapeResult NativeEngine::solid_loft_wires(const double* a, int ac, const double* b, int bc) {
    return fallback().solid_loft_wires(a, ac, b, bc);
}
ShapeResult NativeEngine::solid_sweep(const double* p, int pc, const double* path, int pathc) {
    return fallback().solid_sweep(p, pc, path, pathc);
}
ShapeResult NativeEngine::twisted_sweep(const double* p, int pc, const double* path, int pathc,
                                        double tw, double se) {
    return fallback().twisted_sweep(p, pc, path, pathc, tw, se);
}
ShapeResult NativeEngine::loft_along_rail(const double* r, int rc, const double* a, int ac,
                                          const double* b, int bc) {
    return fallback().loft_along_rail(r, rc, a, ac, b, bc);
}
ShapeResult NativeEngine::guided_sweep(const double* p, int pc, const double* path, int pathc,
                                       const double* g, int gc) {
    return fallback().guided_sweep(p, pc, path, pathc, g, gc);
}
ShapeResult NativeEngine::wrap_emboss(EngineShape body, int faceId, const double* p, int c, double d,
                                      int boss) {
    CC_NATIVE_BODY_UNSUPPORTED("wrap_emboss", body);
    return fallback().wrap_emboss(body, faceId, p, c, d, boss);
}
ShapeResult NativeEngine::helical_thread(double mr, double pi, double tu, double de, double fa,
                                         double pp, int sp) {
    return fallback().helical_thread(mr, pi, tu, de, fa, pp, sp);
}
ShapeResult NativeEngine::tapered_thread(double tr, double tip, double pi, double tu, double de,
                                         double fa, double pp, int sp) {
    return fallback().tapered_thread(tr, tip, pi, tu, de, fa, pp, sp);
}
ShapeResult NativeEngine::tapered_shank(double r, double fh, double th, double pp) {
    return fallback().tapered_shank(r, fh, th, pp);
}
ShapeResult NativeEngine::solid_extrude_holes(const double* o, int oc, const double* h, int hc,
                                              double d) {
    return fallback().solid_extrude_holes(o, oc, h, hc, d);
}
ShapeResult NativeEngine::solid_extrude_polyholes(const double* o, int oc, const double* h,
                                                  const int* hcs, int hc, double d) {
    return fallback().solid_extrude_polyholes(o, oc, h, hcs, hc, d);
}
ShapeResult NativeEngine::solid_extrude_profile(const ProfileSeg* s, int sc, const double* h, int hc,
                                                const double* sx, int sxc, double d) {
    return fallback().solid_extrude_profile(s, sc, h, hc, sx, sxc, d);
}
ShapeResult NativeEngine::solid_extrude_profile_polyholes(const ProfileSeg* s, int sc,
                                                          const double* h, int cc, const double* px,
                                                          const int* pcs, int pc, const double* sx,
                                                          int sxc, double d) {
    return fallback().solid_extrude_profile_polyholes(s, sc, h, cc, px, pcs, pc, sx, sxc, d);
}
ShapeResult NativeEngine::solid_revolve_profile(const ProfileSeg* s, int sc, double ax, double ay,
                                                double adx, double ady, const double* sx, int sxc,
                                                double a) {
    return fallback().solid_revolve_profile(s, sc, ax, ay, adx, ady, sx, sxc, a);
}

// ── feature fallthrough ─────────────────────────────────────────────────────────

ShapeResult NativeEngine::fillet_edges(EngineShape body, const int* e, int ec, double r) {
    CC_NATIVE_BODY_UNSUPPORTED("fillet_edges", body);
    return fallback().fillet_edges(body, e, ec, r);
}
ShapeResult NativeEngine::fillet_edges_variable(EngineShape body, const int* e, int ec, double r1,
                                                double r2) {
    CC_NATIVE_BODY_UNSUPPORTED("fillet_edges_variable", body);
    return fallback().fillet_edges_variable(body, e, ec, r1, r2);
}
ShapeResult NativeEngine::chamfer_edges(EngineShape body, const int* e, int ec, double d) {
    CC_NATIVE_BODY_UNSUPPORTED("chamfer_edges", body);
    return fallback().chamfer_edges(body, e, ec, d);
}
ShapeResult NativeEngine::shell(EngineShape body, const int* f, int fc, double t) {
    CC_NATIVE_BODY_UNSUPPORTED("shell", body);
    return fallback().shell(body, f, fc, t);
}
ShapeResult NativeEngine::offset_face(EngineShape body, int f, double d) {
    CC_NATIVE_BODY_UNSUPPORTED("offset_face", body);
    return fallback().offset_face(body, f, d);
}
ShapeResult NativeEngine::replace_face(EngineShape body, int f, double o, double t) {
    CC_NATIVE_BODY_UNSUPPORTED("replace_face", body);
    return fallback().replace_face(body, f, o, t);
}
ShapeResult NativeEngine::replace_face_to_plane(EngineShape body, int f, double px, double py,
                                                double pz, double nx, double ny, double nz) {
    CC_NATIVE_BODY_UNSUPPORTED("replace_face_to_plane", body);
    return fallback().replace_face_to_plane(body, f, px, py, pz, nx, ny, nz);
}
ShapeResult NativeEngine::fillet_face(EngineShape body, int f, double r) {
    CC_NATIVE_BODY_UNSUPPORTED("fillet_face", body);
    return fallback().fillet_face(body, f, r);
}
ShapeResult NativeEngine::split_plane(EngineShape body, double ox, double oy, double oz, double nx,
                                      double ny, double nz, int keep) {
    CC_NATIVE_BODY_UNSUPPORTED("split_plane", body);
    return fallback().split_plane(body, ox, oy, oz, nx, ny, nz, keep);
}
ShapeResult NativeEngine::full_round_fillet(EngineShape body, int f) {
    CC_NATIVE_BODY_UNSUPPORTED("full_round_fillet", body);
    return fallback().full_round_fillet(body, f);
}
ShapeResult NativeEngine::full_round_fillet_faces(EngineShape body, int l, int m, int r) {
    CC_NATIVE_BODY_UNSUPPORTED("full_round_fillet_faces", body);
    return fallback().full_round_fillet_faces(body, l, m, r);
}
ShapeResult NativeEngine::fillet_edges_g2(EngineShape body, const int* e, int ec, double r) {
    CC_NATIVE_BODY_UNSUPPORTED("fillet_edges_g2", body);
    return fallback().fillet_edges_g2(body, e, ec, r);
}

// ── boolean fallthrough ─────────────────────────────────────────────────────────

ShapeResult NativeEngine::boolean_op(EngineShape a, EngineShape b, int op) {
    CC_NATIVE_BODY_UNSUPPORTED("boolean_op", a);
    CC_NATIVE_BODY_UNSUPPORTED("boolean_op", b);
    return fallback().boolean_op(a, b, op);
}
ShapeResult NativeEngine::thread_apply(EngineShape shaft, EngineShape thread, int op) {
    CC_NATIVE_BODY_UNSUPPORTED("thread_apply", shaft);
    CC_NATIVE_BODY_UNSUPPORTED("thread_apply", thread);
    return fallback().thread_apply(shaft, thread, op);
}

// ── query / reference fallthrough ────────────────────────────────────────────────

Result<std::vector<EdgePolylineData>> NativeEngine::edge_polylines(EngineShape body) {
    CC_NATIVE_BODY_UNSUPPORTED("edge_polylines", body);
    return fallback().edge_polylines(body);
}
Result<std::vector<double>> NativeEngine::principal_moments(EngineShape body) {
    CC_NATIVE_BODY_UNSUPPORTED("principal_moments", body);
    return fallback().principal_moments(body);
}
Result<std::vector<double>> NativeEngine::face_axis(EngineShape body, int f) {
    CC_NATIVE_BODY_UNSUPPORTED("face_axis", body);
    return fallback().face_axis(body, f);
}
Result<std::vector<double>> NativeEngine::ref_plane_from_face(EngineShape body, int f) {
    CC_NATIVE_BODY_UNSUPPORTED("ref_plane_from_face", body);
    return fallback().ref_plane_from_face(body, f);
}
Result<std::vector<double>> NativeEngine::ref_axis_from_edge(EngineShape body, int e) {
    CC_NATIVE_BODY_UNSUPPORTED("ref_axis_from_edge", body);
    return fallback().ref_axis_from_edge(body, e);
}
Result<std::vector<double>> NativeEngine::ref_axis_from_face(EngineShape body, int f) {
    CC_NATIVE_BODY_UNSUPPORTED("ref_axis_from_face", body);
    return fallback().ref_axis_from_face(body, f);
}
Result<std::vector<int>> NativeEngine::tangent_chain(EngineShape body, const int* e, int ec) {
    CC_NATIVE_BODY_UNSUPPORTED("tangent_chain", body);
    return fallback().tangent_chain(body, e, ec);
}
Result<std::vector<int>> NativeEngine::outer_rim_chain(EngineShape body, const int* e, int ec) {
    CC_NATIVE_BODY_UNSUPPORTED("outer_rim_chain", body);
    return fallback().outer_rim_chain(body, e, ec);
}
Result<std::vector<double>> NativeEngine::offset_face_boundary(EngineShape body, int f, double d) {
    CC_NATIVE_BODY_UNSUPPORTED("offset_face_boundary", body);
    return fallback().offset_face_boundary(body, f, d);
}

// ── transform fallthrough ─────────────────────────────────────────────────────────

ShapeResult NativeEngine::scale_shape(EngineShape body, double f) {
    CC_NATIVE_BODY_UNSUPPORTED("scale_shape", body);
    return fallback().scale_shape(body, f);
}
ShapeResult NativeEngine::scale_shape_about(EngineShape body, double cx, double cy, double cz,
                                            double f) {
    CC_NATIVE_BODY_UNSUPPORTED("scale_shape_about", body);
    return fallback().scale_shape_about(body, cx, cy, cz, f);
}
ShapeResult NativeEngine::rotate_shape_about(EngineShape body, double cx, double cy, double cz,
                                             double ax, double ay, double az, double a) {
    CC_NATIVE_BODY_UNSUPPORTED("rotate_shape_about", body);
    return fallback().rotate_shape_about(body, cx, cy, cz, ax, ay, az, a);
}
ShapeResult NativeEngine::mirror_shape(EngineShape body, double px, double py, double pz, double nx,
                                       double ny, double nz) {
    CC_NATIVE_BODY_UNSUPPORTED("mirror_shape", body);
    return fallback().mirror_shape(body, px, py, pz, nx, ny, nz);
}
ShapeResult NativeEngine::translate_shape(EngineShape body, double tx, double ty, double tz) {
    CC_NATIVE_BODY_UNSUPPORTED("translate_shape", body);
    return fallback().translate_shape(body, tx, ty, tz);
}
ShapeResult NativeEngine::place_on_frame(EngineShape body, double ox, double oy, double oz,
                                         double ux, double uy, double uz, double vx, double vy,
                                         double vz) {
    CC_NATIVE_BODY_UNSUPPORTED("place_on_frame", body);
    return fallback().place_on_frame(body, ox, oy, oz, ux, uy, uz, vx, vy, vz);
}

// ── exchange fallthrough ────────────────────────────────────────────────────────

Result<void> NativeEngine::step_export(EngineShape body, const char* path) {
    CC_NATIVE_BODY_UNSUPPORTED("step_export", body);
    return fallback().step_export(body, path);
}
ShapeResult NativeEngine::step_import(const char* path) { return fallback().step_import(path); }
Result<void> NativeEngine::iges_export(EngineShape body, const char* path) {
    CC_NATIVE_BODY_UNSUPPORTED("iges_export", body);
    return fallback().iges_export(body, path);
}
ShapeResult NativeEngine::iges_import(const char* path) { return fallback().iges_import(path); }

}  // namespace cyber
