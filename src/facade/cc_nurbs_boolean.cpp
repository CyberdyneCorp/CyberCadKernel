// SPDX-License-Identifier: Apache-2.0
//
// cc_nurbs_boolean.cpp — Wave-J track J7: the public C facade for the OCCT-free native
// LAYER-3 general two-freeform-solid NURBS boolean ORCHESTRATOR
// (src/native/boolean/nurbs_solid_boolean.h — `nurbsSolidBoolean(A, B, op)`). This TU is
// J7's alone (the J-track waves are file-disjoint so they never collide). It bridges:
//
//   src/native/boolean/nurbs_solid_boolean.h  → cc_nurbs_solid_boolean
//
// HANDLE MODEL — this TU does NOT touch J1's internal registry (that lives in an
// anonymous namespace in cc_kernel_nurbs.cpp). It reads a wall cc_surface back through
// J1's PUBLIC accessors (cc_surface_info / cc_surface_knots_u/_v / cc_surface_poles) into
// a native FaceSurface, builds the bowl-cup SOLID exactly like the native proof fixture's
// `buildCup` (freeform wall + circular UV rim trim + a flat lid sharing that rim), drives
// the native orchestrator, and copies the WATERTIGHT result mesh into a caller-owned
// CCMesh. So the boundary stays POD and J1 owns the registry.
//
// OPERAND MODEL — the boolean consumes a trimmed-face SOLID (topo::Shape), not a lone
// surface, so the wrapper takes the minimal additional trim data per operand: a rim-circle
// radius (in the wall's (u,v)) and a flat lid z-height. The wall cc_surface MUST be a
// single-patch Bézier (clamped knots, no interior knots) — the L3 seam trace treats the
// wall poles as ONE Bézier patch (makeBezierAdapter). A wall with interior knots is
// honest-declined by the orchestrator's operand gate / seam trace, never mis-welded.
//
// HONEST DECLINE — the wrapper returns 0 (and zeroes *out) + cc_last_error on any genuine
// failure: an unknown/ill-formed wall handle, a non-admissible operand, or the MULTI-SEAM /
// non-watertight annulus↔annulus pose whose sew the orchestrator declines (the L3-d frozen-
// mesher gap). It NEVER emits a leaky/partial mesh and NEVER widens a tolerance.
//
// BUILD GATE — the native boolean composes the numsci-backed SSI seam trace, so it is only
// compiled/linked under CYBERCAD_HAS_NUMSCI. With that macro OFF this wrapper honest-
// declines rather than call it (the declaration stays visible; the ABI symbol is present).

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "core/guard.h"
#include "cybercadkernel/cc_kernel.h"

#ifdef CYBERCAD_HAS_NUMSCI
#include "native/boolean/nurbs_solid_boolean.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"
#endif

namespace {

using cyber::guard;
using cyber::set_last_error;

#ifdef CYBERCAD_HAS_NUMSCI

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace bo = cybercad::native::boolean;
namespace nmath = cybercad::native::math;

// ── read a wall cc_surface back into a native Bézier FaceSurface (J1 PUBLIC accessors) ──
//
// The wall MUST be a single-patch Bézier: nPolesU == degreeU+1 (and likewise V), no
// interior knots. We tag the FaceSurface as Kind::Bezier so the L3 orchestrator's seam
// trace (makeBezierAdapter over poles) is geometrically exact. Returns true iff the handle
// is live, every accessor filled cleanly, and the single-patch invariant holds.
bool readWallBezier(cc_surface h, topo::FaceSurface& out) {
    CCSurfaceInfo info;
    if (cc_surface_info(h, &info) != 1) {
        return false;
    }
    if (info.n_ctrl_u != info.degree_u + 1 || info.n_ctrl_v != info.degree_v + 1) {
        return false;  // not a single Bézier patch (interior knots) — declined
    }
    const int total = info.n_ctrl_u * info.n_ctrl_v;
    std::vector<double> xyzw(static_cast<std::size_t>(total) * 4);
    if (cc_surface_poles(h, xyzw.data(), total * 4) != total * 4) {
        return false;
    }
    out = topo::FaceSurface{};
    out.kind = topo::FaceSurface::Kind::Bezier;
    out.degreeU = info.degree_u;
    out.degreeV = info.degree_v;
    out.nPolesU = info.n_ctrl_u;
    out.nPolesV = info.n_ctrl_v;
    out.poles.resize(static_cast<std::size_t>(total));
    const bool rational = info.rational != 0;
    if (rational) {
        out.weights.resize(static_cast<std::size_t>(total));
    }
    for (int i = 0; i < total; ++i) {
        out.poles[static_cast<std::size_t>(i)] =
            nmath::Point3{xyzw[4 * i + 0], xyzw[4 * i + 1], xyzw[4 * i + 2]};
        if (rational) {
            out.weights[static_cast<std::size_t>(i)] = xyzw[4 * i + 3];
        }
    }
    return true;
}

// The rim circle in the wall's (u,v), centred at the domain midpoint (Bézier domain is
// [0,1]²): (½ + rim·cosθ, ½ + rim·sinθ), CCW, `segs` samples. Mirrors the fixtures' rimUV.
std::vector<nmath::Point3> rimUV(double rim, int segs) {
    constexpr double kPi = 3.14159265358979323846;
    std::vector<nmath::Point3> uv;
    uv.reserve(static_cast<std::size_t>(segs));
    for (int k = 0; k < segs; ++k) {
        const double th = 2.0 * kPi * static_cast<double>(k) / segs;
        uv.push_back(nmath::Point3{0.5 + rim * std::cos(th), 0.5 + rim * std::sin(th), 0.0});
    }
    return uv;
}

// Build a bowl-cup solid from a freeform Bézier `wall` (circular UV trim of radius `rim`)
// closed by a flat LID plane at z = lid. A faithful port of the native proof fixture's
// `buildCup`: the wall face (Reversed ⇒ outward normal away from the interior) and the lid
// (Forward, reversed rim winding) SHARE the same degree-2 Bézier rim edges, so the sewn cup
// is watertight by construction. Returns a null Shape if the rim is degenerate.
topo::Shape buildCup(const topo::FaceSurface& wall, double rim, double lid, int segs) {
    if (!(rim > 0.0) || segs < 8) {
        return topo::Shape{};
    }
    tess::SurfaceEvaluator eval(wall, topo::Location{});
    const std::vector<nmath::Point3> uv = rimUV(rim, segs);
    const int n = static_cast<int>(uv.size());

    std::vector<nmath::Point3> rim3d(static_cast<std::size_t>(n));
    std::vector<topo::Shape> vRim(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        rim3d[static_cast<std::size_t>(k)] =
            eval.value(uv[static_cast<std::size_t>(k)].x, uv[static_cast<std::size_t>(k)].y);
        vRim[static_cast<std::size_t>(k)] = topo::ShapeBuilder::makeVertex(rim3d[static_cast<std::size_t>(k)]);
    }

    // rim edges as degree-2 Bézier 3-D curves (the wall edge over a straight UV chord is
    // exactly degree-2), built ONCE and shared by the wall face + the lid.
    std::vector<topo::Shape> rimEdges(static_cast<std::size_t>(n));
    std::vector<nmath::Point3> rimCtrl(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) {
        const int k1 = (k + 1) % n;
        const nmath::Point3 mid{(uv[static_cast<std::size_t>(k)].x + uv[static_cast<std::size_t>(k1)].x) * 0.5,
                                (uv[static_cast<std::size_t>(k)].y + uv[static_cast<std::size_t>(k1)].y) * 0.5,
                                0.0};
        const nmath::Point3 Sm = eval.value(mid.x, mid.y);
        const nmath::Point3& a = rim3d[static_cast<std::size_t>(k)];
        const nmath::Point3& b = rim3d[static_cast<std::size_t>(k1)];
        rimCtrl[static_cast<std::size_t>(k)] = nmath::Point3{2 * Sm.x - 0.5 * (a.x + b.x),
                                                             2 * Sm.y - 0.5 * (a.y + b.y),
                                                             2 * Sm.z - 0.5 * (a.z + b.z)};
        topo::EdgeCurve c{};
        c.kind = topo::EdgeCurve::Kind::Bezier;
        c.degree = 2;
        c.poles = {a, rimCtrl[static_cast<std::size_t>(k)], b};
        rimEdges[static_cast<std::size_t>(k)] =
            topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, vRim[static_cast<std::size_t>(k)],
                                         vRim[static_cast<std::size_t>(k1)]);
    }

    std::vector<topo::Shape> faces;

    // wall (freeform), circular UV trim. Reversed ⇒ outward normal points away from the
    // interior (consistent closed shell — the B1 recogniser audit).
    {
        const topo::Shape node = topo::ShapeBuilder::makeFace(wall, topo::Shape{});
        std::vector<topo::Shape> we;
        we.reserve(static_cast<std::size_t>(n));
        for (int k = 0; k < n; ++k) {
            const int k1 = (k + 1) % n;
            topo::PCurve pc{};
            pc.kind = topo::EdgeCurve::Kind::Line;
            pc.origin2d = nmath::Point3{uv[static_cast<std::size_t>(k)].x, uv[static_cast<std::size_t>(k)].y, 0.0};
            pc.dir2d = nmath::Vec3{uv[static_cast<std::size_t>(k1)].x - uv[static_cast<std::size_t>(k)].x,
                                   uv[static_cast<std::size_t>(k1)].y - uv[static_cast<std::size_t>(k)].y, 0.0};
            we.push_back(topo::ShapeBuilder::addPCurve(rimEdges[static_cast<std::size_t>(k)], node.tshape(), pc));
        }
        faces.push_back(topo::ShapeBuilder::makeFace(
            wall, topo::ShapeBuilder::makeWire(std::move(we)), {}, topo::Orientation::Reversed));
    }

    // lid (plane z = lid). Bounded by the SAME rim edges, reversed order + orientation so
    // the lid loop winds opposite the wall loop ⇒ shared boundary, watertight.
    {
        topo::FaceSurface pl{};
        pl.kind = topo::FaceSurface::Kind::Plane;
        pl.frame.origin = nmath::Point3{0, 0, lid};
        pl.frame.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
        pl.frame.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
        pl.frame.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
        const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
        std::vector<topo::Shape> we;
        we.reserve(static_cast<std::size_t>(n));
        for (int k = n - 1; k >= 0; --k) {
            const int k1 = (k + 1) % n;
            topo::PCurve pc{};
            pc.kind = topo::EdgeCurve::Kind::BSpline;
            pc.degree = 2;
            pc.poles2d = {nmath::Point3{rim3d[static_cast<std::size_t>(k)].x, rim3d[static_cast<std::size_t>(k)].y, 0.0},
                          nmath::Point3{rimCtrl[static_cast<std::size_t>(k)].x, rimCtrl[static_cast<std::size_t>(k)].y, 0.0},
                          nmath::Point3{rim3d[static_cast<std::size_t>(k1)].x, rim3d[static_cast<std::size_t>(k1)].y, 0.0}};
            pc.knots = {0, 0, 0, 1, 1, 1};
            we.push_back(
                topo::ShapeBuilder::addPCurve(rimEdges[static_cast<std::size_t>(k)], node.tshape(), pc)
                    .reversedShape());
        }
        faces.push_back(topo::ShapeBuilder::makeFace(
            pl, topo::ShapeBuilder::makeWire(std::move(we)), {}, topo::Orientation::Forward));
    }

    const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
    return topo::ShapeBuilder::makeSolid({shell});
}

// ── copy a native closed tessellate::Mesh into a caller-owned CCMesh (malloc'd) ──
//
// Mirrors cc_surface_tessellate's CCMesh ownership: the caller frees it with cc_mesh_free.
// Returns false (leaving *out zeroed) on an empty mesh or OOM.
bool meshToCCMesh(const tess::Mesh& m, CCMesh* out) {
    const int nV = static_cast<int>(m.vertices.size());
    const int nT = static_cast<int>(m.triangles.size());
    if (nV <= 0 || nT <= 0) {
        return false;
    }
    auto* verts = static_cast<double*>(std::malloc(static_cast<std::size_t>(nV) * 3 * sizeof(double)));
    auto* tris = static_cast<int*>(std::malloc(static_cast<std::size_t>(nT) * 3 * sizeof(int)));
    if (verts == nullptr || tris == nullptr) {
        std::free(verts);
        std::free(tris);
        return false;
    }
    for (int i = 0; i < nV; ++i) {
        verts[3 * i + 0] = m.vertices[static_cast<std::size_t>(i)].x;
        verts[3 * i + 1] = m.vertices[static_cast<std::size_t>(i)].y;
        verts[3 * i + 2] = m.vertices[static_cast<std::size_t>(i)].z;
    }
    for (int i = 0; i < nT; ++i) {
        tris[3 * i + 0] = static_cast<int>(m.triangles[static_cast<std::size_t>(i)].a);
        tris[3 * i + 1] = static_cast<int>(m.triangles[static_cast<std::size_t>(i)].b);
        tris[3 * i + 2] = static_cast<int>(m.triangles[static_cast<std::size_t>(i)].c);
    }
    out->vertices = verts;
    out->vertexCount = nV;
    out->triangles = tris;
    out->triangleCount = nT;
    return true;
}

bo::SolidBoolOp opFromCC(CCBoolOp op) {
    switch (op) {
        case CC_BOOL_FUSE: return bo::SolidBoolOp::Fuse;
        case CC_BOOL_CUT: return bo::SolidBoolOp::Cut;
        default: return bo::SolidBoolOp::Common;
    }
}

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace

int cc_nurbs_solid_boolean(cc_surface wallA, double rimA, double lidA, cc_surface wallB,
                           double rimB, double lidB, CCBoolOp op, double deflection, CCMesh* out) {
#ifdef CYBERCAD_HAS_NUMSCI
    return guard(
        [&]() -> int {
            if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
            if (out == nullptr) {
                set_last_error("cc_nurbs_solid_boolean: null out");
                return 0;
            }
            if (op != CC_BOOL_FUSE && op != CC_BOOL_CUT && op != CC_BOOL_COMMON) {
                set_last_error("cc_nurbs_solid_boolean: unknown op");
                return 0;
            }
            topo::FaceSurface wa, wb;
            if (!readWallBezier(wallA, wa) || !readWallBezier(wallB, wb)) {
                set_last_error(
                    "cc_nurbs_solid_boolean: unknown wall handle or wall is not a single-patch "
                    "Bezier (interior knots) — declined");
                return 0;
            }
            constexpr int kRimSegs = 96;  // dense enough that the degree-2 rim tracks the wall
            const topo::Shape A = buildCup(wa, rimA, lidA, kRimSegs);
            const topo::Shape B = buildCup(wb, rimB, lidB, kRimSegs);
            if (A.isNull() || B.isNull()) {
                set_last_error("cc_nurbs_solid_boolean: degenerate rim / could not build operand cup");
                return 0;
            }

            const double d = (deflection > 0.0) ? deflection : 0.005;
            bo::SolidBoolReport rep;
            const topo::Shape r = bo::nurbsSolidBoolean(A, B, opFromCC(op), d, &rep,
                                                        std::numeric_limits<double>::quiet_NaN());
            if (r.isNull() || rep.decline != bo::SolidBoolDecline::Ok) {
                set_last_error(std::string("cc_nurbs_solid_boolean: declined (") +
                               bo::solidBoolDeclineName(rep.decline) +
                               ") — no leaky mesh emitted (multi-seam / non-watertight sew)");
                return 0;
            }

            // Mesh the WATERTIGHT result + self-verify before crossing the boundary. NEVER
            // emit a leaky mesh: a non-closed / non-coherent result honest-declines here too.
            tess::MeshParams mp;
            mp.deflection = d;
            const tess::Mesh m = tess::SolidMesher(mp).mesh(r);
            if (!tess::isWatertight(m) || !tess::isConsistentlyOriented(m) ||
                tess::boundaryEdgeCount(m) != 0) {
                set_last_error("cc_nurbs_solid_boolean: result not watertight — declined (no leaky mesh)");
                return 0;
            }
            if (!meshToCCMesh(m, out)) {
                set_last_error("cc_nurbs_solid_boolean: empty result shell / out of memory");
                return 0;
            }
            return 1;
        },
        0);
#else
    (void)wallA;
    (void)rimA;
    (void)lidA;
    (void)wallB;
    (void)rimB;
    (void)lidB;
    (void)op;
    (void)deflection;
    if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
    set_last_error("cc_nurbs_solid_boolean: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return 0;
#endif
}
