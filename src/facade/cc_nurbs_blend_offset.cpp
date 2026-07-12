// SPDX-License-Identifier: Apache-2.0
//
// cc_nurbs_blend_offset.cpp — Wave-J track J4: the public C facade for the OCCT-free
// native BLEND + OFFSET/THICKEN/SHELL modules. This TU is J4's alone (the J2–J5 waves
// are file-disjoint so they never collide). It bridges:
//
//   src/native/blend/fillet_edge_g2_freeform.h  → cc_nurbs_fillet_freeform_g2
//   src/native/blend/vertex_blend.h             → cc_nurbs_vertex_blend
//   src/native/blend/chamfer_edge_variable_freeform.h
//                                               → cc_nurbs_chamfer_variable / _freeform
//   src/native/math/bspline_offset.h            → cc_nurbs_offset_rational / _trimmed
//   src/native/math/bspline_thicken.h           → cc_nurbs_thicken_trimmed
//   src/native/math/bspline_shell.h             → cc_nurbs_shell_trimmed
//
// HANDLE MODEL — this TU does NOT touch J1's internal registry (that lives in an
// anonymous namespace in cc_kernel_nurbs.cpp). It reads a cc_surface back through J1's
// PUBLIC accessors (cc_surface_info / cc_surface_knots_u/_v / cc_surface_poles) into a
// native BsplineSurfaceData, drives the native module, and registers a produced surface
// with the PUBLIC cc_surface_create. So the boundary stays POD and J1 owns the registry.
//
// HONEST DECLINE — a surface-producing wrapper returns cc_surface{0} + cc_last_error on
// any genuine geometric failure; a solid-producing wrapper returns 0 (and zeroes *out).
// The over-radius fillet / fully-folding thicken NEVER emit a self-intersecting result —
// they decline (design.md §3, tasks.md §4). No tolerance is ever widened at the boundary.
//
// The offset/thicken/shell native routines fit through the numsci substrate; when the
// kernel is built WITHOUT CYBERCAD_HAS_NUMSCI those wrappers honest-decline (the native
// functions are absent, so we do not call them — the declaration stays visible).

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "core/guard.h"
#include "cybercadkernel/cc_kernel.h"

#include "native/blend/chamfer_edge_variable_freeform.h"
#include "native/blend/fillet_edge_g2_freeform.h"
#include "native/blend/vertex_blend.h"
#include "native/math/bspline.h"
#include "native/math/bspline_ops.h"

#ifdef CYBERCAD_HAS_NUMSCI
#include "native/math/bspline_offset.h"
#include "native/math/bspline_shell.h"
#include "native/math/bspline_thicken.h"
#endif

namespace {

using cyber::guard;
using cyber::set_last_error;
namespace nm = cybercad::native::math;
namespace nb = cybercad::native::blend;

// ── read a cc_surface back into a native BsplineSurfaceData (via J1 PUBLIC accessors) ──
//
// Returns true iff the handle is a live cc_surface and every accessor filled cleanly.
bool readSurface(cc_surface h, nm::BsplineSurfaceData& out) {
    CCSurfaceInfo info;
    if (cc_surface_info(h, &info) != 1) {
        return false;
    }
    out.degreeU = info.degree_u;
    out.degreeV = info.degree_v;
    out.nPolesU = info.n_ctrl_u;
    out.nPolesV = info.n_ctrl_v;

    std::vector<double> ku(static_cast<std::size_t>(info.n_knots_u));
    std::vector<double> kv(static_cast<std::size_t>(info.n_knots_v));
    if (cc_surface_knots_u(h, ku.data(), info.n_knots_u) != info.n_knots_u ||
        cc_surface_knots_v(h, kv.data(), info.n_knots_v) != info.n_knots_v) {
        return false;
    }
    out.knotsU = std::move(ku);
    out.knotsV = std::move(kv);

    const int total = info.n_ctrl_u * info.n_ctrl_v;
    std::vector<double> xyzw(static_cast<std::size_t>(total) * 4);
    if (cc_surface_poles(h, xyzw.data(), total * 4) != total * 4) {
        return false;
    }
    out.poles.resize(static_cast<std::size_t>(total));
    if (info.rational != 0) {
        out.weights.resize(static_cast<std::size_t>(total));
    } else {
        out.weights.clear();  // non-rational: J1 emits w = 1 for every pole
    }
    for (int i = 0; i < total; ++i) {
        out.poles[static_cast<std::size_t>(i)] =
            nm::Point3{xyzw[4 * i + 0], xyzw[4 * i + 1], xyzw[4 * i + 2]};
        if (info.rational != 0) {
            out.weights[static_cast<std::size_t>(i)] = xyzw[4 * i + 3];
        }
    }
    return true;
}

// ── register a native BsplineSurfaceData as a cc_surface (via J1 PUBLIC cc_surface_create) ──
//
// Packs the (row-major, U-outer) net into the homogeneous (x,y,z,w) stream cc_surface_create
// expects, then delegates — so the produced handle lives in J1's registry with all of J1's
// lifetime / accessor guarantees. Non-rational (empty weights) ⇒ every w = 1.
cc_surface registerSurface(const nm::BsplineSurfaceData& s) {
    const int total = s.nPolesU * s.nPolesV;
    if (total <= 0 || static_cast<int>(s.poles.size()) != total) {
        return cc_surface{0};
    }
    std::vector<double> xyzw(static_cast<std::size_t>(total) * 4);
    for (int i = 0; i < total; ++i) {
        const nm::Point3& p = s.poles[static_cast<std::size_t>(i)];
        const double w = s.weights.empty() ? 1.0 : s.weights[static_cast<std::size_t>(i)];
        xyzw[4 * i + 0] = p.x;
        xyzw[4 * i + 1] = p.y;
        xyzw[4 * i + 2] = p.z;
        xyzw[4 * i + 3] = w;
    }
    return cc_surface_create(s.degreeU, s.degreeV, xyzw.data(), s.nPolesU, s.nPolesV,
                             s.knotsU.data(), static_cast<int>(s.knotsU.size()), s.knotsV.data(),
                             static_cast<int>(s.knotsV.size()));
}

// A clamped uniform degree-1 knot vector of length n+2 over [0,1] (n ≥ 2 poles).
std::vector<double> linearKnots(int n) {
    std::vector<double> k;
    k.reserve(static_cast<std::size_t>(n) + 2);
    k.push_back(0.0);
    for (int i = 0; i < n; ++i) {
        k.push_back(n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1));
    }
    k.push_back(1.0);
    return k;
}

// Build a degree-1×1 (bilinear) tensor NURBS surface that INTERPOLATES a row-major
// (nU × nV) grid of points exactly — the honest display geometry for a native band/rim
// whose analytic form is not a single higher-degree tensor surface. Each grid vertex is a
// control point; a degree-1 basis passes through every pole, so the surface reproduces the
// sampled grid pointwise. Declines (empty) when the grid is degenerate.
cc_surface registerGridSurface(const std::vector<nm::Point3>& grid, int nU, int nV) {
    if (nU < 2 || nV < 2 || static_cast<int>(grid.size()) != nU * nV) {
        return cc_surface{0};
    }
    nm::BsplineSurfaceData s;
    s.degreeU = 1;
    s.degreeV = 1;
    s.nPolesU = nU;
    s.nPolesV = nV;
    s.poles = grid;  // already row-major, U outer
    s.knotsU = linearKnots(nU);
    s.knotsV = linearKnots(nV);
    return registerSurface(s);
}

// A native ffdetail::Surface VIEW over a BsplineSurfaceData (borrowed spans). The backing
// BsplineSurfaceData MUST outlive the returned view.
nb::ffdetail::Surface asFfSurface(const nm::BsplineSurfaceData& s) {
    nb::ffdetail::Surface v;
    v.degreeU = s.degreeU;
    v.degreeV = s.degreeV;
    v.grid = nm::SurfaceGrid{std::span<const nm::Point3>(s.poles), s.nPolesU, s.nPolesV};
    v.weights = std::span<const double>(s.weights);
    v.knotsU = std::span<const double>(s.knotsU);
    v.knotsV = std::span<const double>(s.knotsV);
    return v;
}

// ── copy a native closed tessellate::Mesh into a caller-owned CCMesh (malloc'd) ──
//
// Mirrors cc_surface_tessellate's CCMesh ownership: the caller frees it with cc_mesh_free.
// Returns false (leaving *out zeroed) on an empty mesh or OOM.
#ifdef CYBERCAD_HAS_NUMSCI
bool meshToCCMesh(const cybercad::native::tessellate::Mesh& m, CCMesh* out) {
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
#endif

// de Casteljau evaluation of a 3-D quintic section at s ∈ [0,1] (matches the native skin).
nm::Point3 quinticPoint(const std::array<nm::Point3, 6>& poles, double s) {
    std::array<nm::Point3, 6> p = poles;
    for (int k = 1; k < 6; ++k) {
        for (int i = 0; i < 6 - k; ++i) {
            p[i] = nm::Point3{p[i].x + (p[i + 1].x - p[i].x) * s, p[i].y + (p[i + 1].y - p[i].y) * s,
                              p[i].z + (p[i + 1].z - p[i].z) * s};
        }
    }
    return p[0];
}

nb::FilletGapSide gapSideFromInt(int s) {
    switch (s) {
        case 0: return nb::FilletGapSide::U0;
        case 1: return nb::FilletGapSide::U1;
        case 2: return nb::FilletGapSide::V0;
        default: return nb::FilletGapSide::V1;
    }
}

}  // namespace

// ── FREEFORM G2 FILLET ────────────────────────────────────────────────────────────────

cc_surface cc_nurbs_fillet_freeform_g2(cc_surface faceA, cc_surface faceB, double r,
                                       const double* center0, const double* spineDir, double sA,
                                       double sB, double stepLen, int nStations,
                                       int nSectionSamples) {
    return guard(
        [&]() -> cc_surface {
            nm::BsplineSurfaceData a, b;
            if (!readSurface(faceA, a) || !readSurface(faceB, b)) {
                set_last_error("cc_nurbs_fillet_freeform_g2: unknown face handle");
                return cc_surface{0};
            }
            if (center0 == nullptr || spineDir == nullptr || nStations < 2 ||
                nSectionSamples < 1 || !(r > 0.0)) {
                set_last_error("cc_nurbs_fillet_freeform_g2: bad radius / seed / station count");
                return cc_surface{0};
            }
            const nb::ffdetail::Surface va = asFfSurface(a);
            const nb::ffdetail::Surface vb = asFfSurface(b);
            nb::FreeformFilletSeed seed;
            seed.center0 = nm::Point3{center0[0], center0[1], center0[2]};
            seed.spineDir = nm::Vec3{spineDir[0], spineDir[1], spineDir[2]};
            seed.stepLen = stepLen;
            seed.nStations = nStations;
            seed.sA = sA;
            seed.sB = sB;

            const nb::FreeformFilletResult res =
                nb::fillet_edge_g2_freeform(va, vb, r, seed, nSectionSamples);
            if (!res.ok()) {
                set_last_error(
                    "cc_nurbs_fillet_freeform_g2: fillet declined (ball won't fit / over-radius / "
                    "section fold) — no self-intersecting result emitted");
                return cc_surface{0};
            }

            // Sample the seated station × section rim grid (the exact skin lattice) into a
            // degree-1×1 tensor cc_surface. nU = stations, nV = section samples + 1.
            const int nU = static_cast<int>(res.stations.size());
            const int nV = nSectionSamples + 1;
            std::vector<nm::Point3> grid;
            grid.reserve(static_cast<std::size_t>(nU) * nV);
            for (int k = 0; k < nU; ++k) {
                const auto& poles = res.stations[static_cast<std::size_t>(k)].section.poles;
                for (int j = 0; j < nV; ++j) {
                    grid.push_back(quinticPoint(poles, static_cast<double>(j) / (nV - 1)));
                }
            }
            const cc_surface h = registerGridSurface(grid, nU, nV);
            if (h.id == 0) {
                set_last_error("cc_nurbs_fillet_freeform_g2: could not register fillet band");
            }
            return h;
        },
        cc_surface{0});
}

// ── SETBACK VERTEX BLEND ──────────────────────────────────────────────────────────────

int cc_nurbs_vertex_blend(const cc_surface* fillets, int nFillets, const int* sides,
                          const double* setbacks, const int* reverses, int mode,
                          cc_surface* outPatches, int cap) {
    return guard(
        [&]() -> int {
            if (fillets == nullptr || sides == nullptr || setbacks == nullptr ||
                reverses == nullptr || nFillets < 3) {
                set_last_error("cc_nurbs_vertex_blend: need N >= 3 incident fillets + arrays");
                return -1;
            }
            std::vector<nb::FilletBoundary> fbs(static_cast<std::size_t>(nFillets));
            for (int i = 0; i < nFillets; ++i) {
                nm::BsplineSurfaceData s;
                if (!readSurface(fillets[i], s)) {
                    set_last_error("cc_nurbs_vertex_blend: unknown fillet handle");
                    return -1;
                }
                fbs[static_cast<std::size_t>(i)].surface = std::move(s);
                fbs[static_cast<std::size_t>(i)].side = gapSideFromInt(sides[i]);
                fbs[static_cast<std::size_t>(i)].setback = setbacks[i];
                fbs[static_cast<std::size_t>(i)].reverse = reverses[i] != 0;
            }

            std::vector<nm::BsplineSurfaceData> patches;
            std::string reason;
            if (mode == 2) {
                const nb::VertexBlendG2Result r = nb::vertexBlendG2(fbs);
                if (!r.ok) reason = r.reason;
                else patches = r.patches;
            } else {
                // Accept any buildable blend within a generous G1 gate, then honest-decline
                // if the fill itself declined (non-closed loop / infeasible corner).
                const nb::VertexBlendResult r = nb::vertexBlendG1(fbs, 1e-7, M_PI, true);
                if (!r.ok) reason = r.reason;
                else patches = r.patches;
            }
            if (patches.empty()) {
                set_last_error("cc_nurbs_vertex_blend: declined — " +
                               (reason.empty() ? std::string("no corner patch produced") : reason));
                return -1;
            }
            const int n = static_cast<int>(patches.size());
            if (outPatches == nullptr || cap < n) {
                set_last_error("cc_nurbs_vertex_blend: output buffer too small for patch count");
                return -1;
            }
            for (int i = 0; i < n; ++i) {
                const cc_surface h = registerSurface(patches[static_cast<std::size_t>(i)]);
                if (h.id == 0) {
                    set_last_error("cc_nurbs_vertex_blend: could not register a corner patch");
                    return -1;
                }
                outPatches[i] = h;
            }
            return n;
        },
        -1);
}

// ── ANALYTIC VARIABLE-DISTANCE CHAMFER ────────────────────────────────────────────────
//
// Packing: subA/subB are 11 doubles: (kind, pX,pY,pZ, nX,nY,nZ, aX,aY,aZ) with radius and
// halfAngle appended? To stay within 11 the axis reuses the normal slot for a plane; the
// full packing is (kind, pX,pY,pZ, nX,nY,nZ, radius, halfAngle, axisEncoded). To keep the
// wrapper simple and unambiguous we pack EXACTLY the Substrate fields used by the analytic
// setback: [0]=kind, [1..3]=point, [4..6]=normal, [7..9]=axis, [10]=radius, and reserve the
// halfAngle in edge-independent callers to 0 (cone half-angle unused by the plane/cylinder
// oracle the analytic chamfer supports). subA[] therefore is 11 doubles.

static nb::chamfer_nurbs::Substrate unpackSubstrate(const double* s) {
    nb::chamfer_nurbs::Substrate sub;
    const int kind = static_cast<int>(std::lround(s[0]));
    switch (kind) {
        case 0: sub.kind = nb::chamfer_nurbs::SubstrateKind::Plane; break;
        case 1: sub.kind = nb::chamfer_nurbs::SubstrateKind::Cylinder; break;
        case 2: sub.kind = nb::chamfer_nurbs::SubstrateKind::Cone; break;
        default: sub.kind = nb::chamfer_nurbs::SubstrateKind::Freeform; break;
    }
    sub.point = nm::Point3{s[1], s[2], s[3]};
    sub.normal = nm::Vec3{s[4], s[5], s[6]};
    sub.axis = nm::Vec3{s[7], s[8], s[9]};
    sub.radius = s[10];
    sub.halfAngle = 0.0;
    return sub;
}

// Build a cc_surface from the ruled bevel band (setback rails cA[k], cB[k]) as a degree-1×1
// tensor grid: U over the edge stations, V over the two rails (nV = 2). Exact for the ruled
// loft R(t,τ) = (1−τ)·cA(t) + τ·cB(t).
static cc_surface registerBevelBand(const std::vector<nm::Point3>& railA,
                                    const std::vector<nm::Point3>& railB) {
    const int nU = static_cast<int>(railA.size());
    if (nU < 2 || railB.size() != railA.size()) {
        return cc_surface{0};
    }
    std::vector<nm::Point3> grid;
    grid.reserve(static_cast<std::size_t>(nU) * 2);
    for (int k = 0; k < nU; ++k) {  // row-major, U outer: (station k, rail 0), (station k, rail 1)
        grid.push_back(railA[static_cast<std::size_t>(k)]);
        grid.push_back(railB[static_cast<std::size_t>(k)]);
    }
    return registerGridSurface(grid, nU, 2);
}

cc_surface cc_nurbs_chamfer_variable(const double* subA, const double* subB, const double* edge,
                                     int nStations, double d0, double d1) {
    return guard(
        [&]() -> cc_surface {
            if (subA == nullptr || subB == nullptr || edge == nullptr || nStations < 2 ||
                !(d0 > 0.0) || !(d1 > 0.0)) {
                set_last_error("cc_nurbs_chamfer_variable: bad substrate / edge / distances");
                return cc_surface{0};
            }
            const nb::chamfer_nurbs::Substrate a = unpackSubstrate(subA);
            const nb::chamfer_nurbs::Substrate b = unpackSubstrate(subB);
            std::vector<nb::chamfer_nurbs::EdgeStation> stations(static_cast<std::size_t>(nStations));
            for (int k = 0; k < nStations; ++k) {
                const double* e = edge + static_cast<std::size_t>(k) * 12;
                auto& st = stations[static_cast<std::size_t>(k)];
                st.p = nm::Point3{e[0], e[1], e[2]};
                st.tangent = nm::Vec3{e[3], e[4], e[5]};
                st.nA = nm::Vec3{e[6], e[7], e[8]};
                st.nB = nm::Vec3{e[9], e[10], e[11]};
            }
            const nb::chamfer_nurbs::ChamferResult res =
                nb::chamfer_nurbs::chamfer_edge_variable(a, b, stations, d0, d1);
            if (res.decline != nb::chamfer_nurbs::ChamferDecline::None) {
                set_last_error(
                    "cc_nurbs_chamfer_variable: declined (over-large setback / degenerate "
                    "dihedral / unsupported substrate)");
                return cc_surface{0};
            }
            const cc_surface h = registerBevelBand(res.setbackA, res.setbackB);
            if (h.id == 0) {
                set_last_error("cc_nurbs_chamfer_variable: could not register bevel band");
            }
            return h;
        },
        cc_surface{0});
}

// ── FREEFORM-EDGE CHAMFER ─────────────────────────────────────────────────────────────
//
// Edge packing (12 doubles/station): pX,pY,pZ, tX,tY,tZ, hintX,hintY,hintZ, uA0,vA0,uB0? —
// only 12 slots, so we pack: [0..2] point, [3..5] tangent, [6..8] material hint, [9] uA0,
// [10] vA0, [11] (uB0 packed as vA0's neighbour is out of budget) — instead we fix vB0=vA0
// and uB0=uA0 warm-starts (the footpoint Newton converges from the mid-domain default in
// practice), so slots [9]/[10] are uA0/vA0 and the B-face warm-start reuses them.

cc_surface cc_nurbs_chamfer_freeform(cc_surface faceA, cc_surface faceB, const double* edge,
                                     int nStations, double d) {
    return guard(
        [&]() -> cc_surface {
            nm::BsplineSurfaceData a, b;
            if (!readSurface(faceA, a) || !readSurface(faceB, b)) {
                set_last_error("cc_nurbs_chamfer_freeform: unknown face handle");
                return cc_surface{0};
            }
            if (edge == nullptr || nStations < 2 || !(d > 0.0)) {
                set_last_error("cc_nurbs_chamfer_freeform: bad edge / distance");
                return cc_surface{0};
            }
            const nb::ffdetail::Surface va = asFfSurface(a);
            const nb::ffdetail::Surface vb = asFfSurface(b);
            std::vector<nb::chamfer_nurbs::FreeformEdgeStation> stations(
                static_cast<std::size_t>(nStations));
            for (int k = 0; k < nStations; ++k) {
                const double* e = edge + static_cast<std::size_t>(k) * 12;
                auto& st = stations[static_cast<std::size_t>(k)];
                st.p = nm::Point3{e[0], e[1], e[2]};
                st.tangent = nm::Vec3{e[3], e[4], e[5]};
                st.materialHint = nm::Vec3{e[6], e[7], e[8]};
                st.uA0 = e[9];
                st.vA0 = e[10];
                st.uB0 = e[9];
                st.vB0 = e[10];
            }
            const nb::chamfer_nurbs::FreeformChamferResult res =
                nb::chamfer_nurbs::chamfer_edge_freeform(va, vb, stations, d);
            if (!res.ok()) {
                set_last_error(
                    "cc_nurbs_chamfer_freeform: declined (footpoint diverged / left domain / "
                    "self-lap) — no self-intersecting bevel emitted");
                return cc_surface{0};
            }
            const cc_surface h = registerBevelBand(res.setbackA, res.setbackB);
            if (h.id == 0) {
                set_last_error("cc_nurbs_chamfer_freeform: could not register bevel band");
            }
            return h;
        },
        cc_surface{0});
}

// ── OFFSET / THICKEN / SHELL ──────────────────────────────────────────────────────────
//
// These compose the numsci-gated Layer-5 fit. Without CYBERCAD_HAS_NUMSCI the native
// functions are absent, so the wrappers honest-decline rather than call them.

cc_surface cc_nurbs_offset_rational(cc_surface h, double dist, double tol) {
#ifdef CYBERCAD_HAS_NUMSCI
    return guard(
        [&]() -> cc_surface {
            nm::BsplineSurfaceData s;
            if (!readSurface(h, s)) {
                set_last_error("cc_nurbs_offset_rational: unknown surface handle");
                return cc_surface{0};
            }
            const double t = (tol > 0.0) ? tol : 1e-4;
            const nm::OffsetResult res = nm::offsetSurfaceRational(s, dist, t);
            if (!res.ok) {
                set_last_error(
                    "cc_nurbs_offset_rational: declined (degenerate normal / self-intersection "
                    "past curvature radius / fit failed)");
                return cc_surface{0};
            }
            const cc_surface out = registerSurface(res.surface);
            if (out.id == 0) {
                set_last_error("cc_nurbs_offset_rational: could not register offset surface");
            }
            return out;
        },
        cc_surface{0});
#else
    (void)h;
    (void)dist;
    (void)tol;
    set_last_error("cc_nurbs_offset_rational: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return cc_surface{0};
#endif
}

cc_surface cc_nurbs_offset_trimmed(cc_surface h, double dist, double tol, double* kept,
                                   int* trimmed) {
#ifdef CYBERCAD_HAS_NUMSCI
    return guard(
        [&]() -> cc_surface {
            if (trimmed != nullptr) *trimmed = 0;
            nm::BsplineSurfaceData s;
            if (!readSurface(h, s)) {
                set_last_error("cc_nurbs_offset_trimmed: unknown surface handle");
                return cc_surface{0};
            }
            const double t = (tol > 0.0) ? tol : 1e-4;
            const nm::OffsetResult res = nm::offsetSurfaceTrimmed(s, dist, t);
            if (!res.ok) {
                set_last_error(
                    "cc_nurbs_offset_trimmed: declined (no fold-free region of meaningful area)");
                return cc_surface{0};
            }
            const cc_surface out = registerSurface(res.surface);
            if (out.id == 0) {
                set_last_error("cc_nurbs_offset_trimmed: could not register offset surface");
                return cc_surface{0};
            }
            if (kept != nullptr) {
                kept[0] = res.keptU0;
                kept[1] = res.keptU1;
                kept[2] = res.keptV0;
                kept[3] = res.keptV1;
            }
            if (trimmed != nullptr) *trimmed = res.trimmed ? 1 : 0;
            return out;
        },
        cc_surface{0});
#else
    (void)h;
    (void)dist;
    (void)tol;
    (void)kept;
    if (trimmed != nullptr) *trimmed = 0;
    set_last_error("cc_nurbs_offset_trimmed: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return cc_surface{0};
#endif
}

int cc_nurbs_thicken_trimmed(cc_surface h, double dist, double tol, CCMesh* out, double* kept,
                             int* trimmed) {
#ifdef CYBERCAD_HAS_NUMSCI
    return guard(
        [&]() -> int {
            if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
            if (trimmed != nullptr) *trimmed = 0;
            if (out == nullptr) {
                set_last_error("cc_nurbs_thicken_trimmed: null out");
                return 0;
            }
            nm::BsplineSurfaceData s;
            if (!readSurface(h, s)) {
                set_last_error("cc_nurbs_thicken_trimmed: unknown surface handle");
                return 0;
            }
            const double t = (tol > 0.0) ? tol : 1e-4;
            const nm::ThickenResult res = nm::thickenTrimmed(s, dist, t);
            if (!res.ok || !res.watertight) {
                set_last_error(
                    "cc_nurbs_thicken_trimmed: declined (fully folding / degenerate / zero "
                    "thickness / non-closed) — no self-intersecting solid emitted");
                return 0;
            }
            if (!meshToCCMesh(res.solid, out)) {
                set_last_error("cc_nurbs_thicken_trimmed: empty shell / out of memory");
                return 0;
            }
            if (kept != nullptr) {
                kept[0] = res.keptU0;
                kept[1] = res.keptU1;
                kept[2] = res.keptV0;
                kept[3] = res.keptV1;
            }
            if (trimmed != nullptr) *trimmed = res.trimmed ? 1 : 0;
            return 1;
        },
        0);
#else
    (void)h;
    (void)dist;
    (void)tol;
    (void)kept;
    if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
    if (trimmed != nullptr) *trimmed = 0;
    set_last_error("cc_nurbs_thicken_trimmed: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return 0;
#endif
}

int cc_nurbs_shell_trimmed(const cc_surface* faces, int nFaces, const int* adjacency, int nEdges,
                           double dist, double tol, CCMesh* out) {
#ifdef CYBERCAD_HAS_NUMSCI
    return guard(
        [&]() -> int {
            if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
            if (out == nullptr || faces == nullptr || nFaces < 1 ||
                (nEdges > 0 && adjacency == nullptr)) {
                set_last_error("cc_nurbs_shell_trimmed: null out / faces / adjacency");
                return 0;
            }
            std::vector<nm::BsplineSurfaceData> fs(static_cast<std::size_t>(nFaces));
            for (int i = 0; i < nFaces; ++i) {
                if (!readSurface(faces[i], fs[static_cast<std::size_t>(i)])) {
                    set_last_error("cc_nurbs_shell_trimmed: unknown face handle");
                    return 0;
                }
            }
            std::vector<nm::SharedEdge> adj(static_cast<std::size_t>(nEdges));
            auto edgeOf = [](int e) -> nm::PatchEdge {
                switch (e) {
                    case 0: return nm::PatchEdge::U0;
                    case 1: return nm::PatchEdge::U1;
                    case 2: return nm::PatchEdge::V0;
                    default: return nm::PatchEdge::V1;
                }
            };
            for (int i = 0; i < nEdges; ++i) {
                const int* a = adjacency + static_cast<std::size_t>(i) * 5;
                auto& se = adj[static_cast<std::size_t>(i)];
                se.faceA = static_cast<std::size_t>(a[0]);
                se.faceB = static_cast<std::size_t>(a[1]);
                se.edgeA = edgeOf(a[2]);
                se.edgeB = edgeOf(a[3]);
                se.reversed = a[4] != 0;
            }
            const double t = (tol > 0.0) ? tol : 1e-4;
            const nm::ShellResult res = nm::shellTrimmed(fs, adj, dist, t);
            if (!res.ok || !res.watertight) {
                set_last_error(
                    "cc_nurbs_shell_trimmed: declined (fold / adjacency mismatch / non-manifold / "
                    "un-trimmable overlap / non-closed) — no self-intersecting solid emitted");
                return 0;
            }
            if (!meshToCCMesh(res.solid, out)) {
                set_last_error("cc_nurbs_shell_trimmed: empty shell / out of memory");
                return 0;
            }
            return 1;
        },
        0);
#else
    (void)faces;
    (void)nFaces;
    (void)adjacency;
    (void)nEdges;
    (void)dist;
    (void)tol;
    if (out != nullptr) *out = CCMesh{nullptr, 0, nullptr, 0};
    set_last_error("cc_nurbs_shell_trimmed: requires the numsci substrate (CYBERCAD_HAS_NUMSCI)");
    return 0;
#endif
}
