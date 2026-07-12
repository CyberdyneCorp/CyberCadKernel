// SPDX-License-Identifier: Apache-2.0
//
// Host test for Wave-J track J7: the cc_nurbs_solid_boolean facade wrapper
// (src/facade/cc_nurbs_boolean.cpp) over the native LAYER-3 general two-freeform-solid
// NURBS boolean ORCHESTRATOR. Everything is driven through the public cc_* surface:
// cc_surface_create builds the two freeform Bézier WALLS, the wrapper builds the bowl-cup
// operands (wall + circular UV rim trim + flat lid) and runs the boolean, and the CCMesh
// accessors read the WATERTIGHT result back.
//
// The pose + oracles mirror the native gate's canonical single-seam bowl-cup
// (freeform_freeform_cut_fixture): A = an UP degree-2 Bézier bowl z_A = a(x²+y²) trimmed
// by a rim CIRCLE of radius R, lid at z = a·R²; B = A mirrored about z = H/2 (a DOWN dome),
// lid at z = H − a·R². The two curved walls meet in ONE closed seam circle ⇒ the boolean
// welds watertight. Closed-form volume oracles (no OCCT):
//   V(A) = π·a·R⁴/2,  V(A∩B) = π·H²/(4a),  V(A−B) = V(A) − V(A∩B).
//
//   1. COMMON / CUT / FUSE → a WATERTIGHT CCMesh (χ=2, every edge used exactly twice),
//      whose meshed volume matches the closed-form op-volume within the tessellation band.
//   2. A MULTI-SEAM pose (two degree-4 mirror cups meeting in TWO seams) → COMMON / CUT /
//      FUSE all WELD to a watertight CCMesh (the M0-WELD inner seam + the FUSE outer
//      envelope); a genuinely non-weldable sub-case would honest-decline (never a leaky mesh).
//
// The native boolean composes the numsci SSI seam trace, so the substantive assertions are
// gated on CYBERCAD_HAS_NUMSCI (with the guard OFF the wrapper honest-declines, asserted).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── single-seam fixture constants (mirror freeform_freeform_cut_fixture) ──
constexpr double kA = 2.0;    // bowl amplitude
constexpr double kR = 0.35;   // rim radius (in the wall's u,v)
constexpr double kH = 0.16;   // B's dome apex height (seam ρ = √(H/2a) = 0.2)

double volA() { return kPi * kA * kR * kR * kR * kR / 2.0; }  // π·a·R⁴/2
double volCommon() { return kPi * kH * kH / (4.0 * kA); }     // lens = π·H²/(4a)
double volCut() { return volA() - volCommon(); }             // A − B

// A's UP bowl surface: separable degree-2 Bézier for z = a·(x²+y²), 3x3 poles.
// xc = {-0.5,0,0.5}; zc = {0.25a,-0.25a,0.25a}; z(i,j) = zc[i]+zc[j].
cc_surface makeBowlWall(bool downDome) {
    const double xc[3] = {-0.5, 0.0, 0.5};
    const double zc[3] = {0.25 * kA, -0.25 * kA, 0.25 * kA};
    std::vector<double> polesXYZW;
    polesXYZW.reserve(9 * 4);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double z = zc[i] + zc[j];
            if (downDome) z = kH - z;  // B is A mirrored: z ↦ H − z
            polesXYZW.push_back(xc[i]);
            polesXYZW.push_back(xc[j]);
            polesXYZW.push_back(z);
            polesXYZW.push_back(1.0);
        }
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};  // clamped Bézier (3+2+1)
    return cc_surface_create(2, 2, polesXYZW.data(), 3, 3, knots.data(),
                             static_cast<int>(knots.size()), knots.data(),
                             static_cast<int>(knots.size()));
}

// ── multi-seam fixture: A's degree-4 VALLEY Bézier (5x5), z = a(x²+y²−ρ₀²)² ──
// (poles precomputed for a=4, ρ₀=0.28, x,y ∈ [-0.5,0.5] — copied from the native fixture).
cc_surface makeValleyWall(bool downDome) {
    constexpr double H = 0.03;
    const double xc[5] = {-0.5, -0.25, 0.0, 0.25, 0.5};
    const double Z[5][5] = {
        {0.710986, -0.132214, 0.253386, -0.132214, 0.710986},
        {-0.132214, -0.475414, 0.076853, -0.475414, -0.132214},
        {0.253386, 0.076853, 0.684675, 0.076853, 0.253386},
        {-0.132214, -0.475414, 0.076853, -0.475414, -0.132214},
        {0.710986, -0.132214, 0.253386, -0.132214, 0.710986}};
    std::vector<double> polesXYZW;
    polesXYZW.reserve(25 * 4);
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j) {
            double z = Z[i][j];
            if (downDome) z = H - z;
            polesXYZW.push_back(xc[i]);
            polesXYZW.push_back(xc[j]);
            polesXYZW.push_back(z);
            polesXYZW.push_back(1.0);
        }
    std::vector<double> knots = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1};  // clamped degree-4 (5+4+1)
    return cc_surface_create(4, 4, polesXYZW.data(), 5, 5, knots.data(),
                             static_cast<int>(knots.size()), knots.data(),
                             static_cast<int>(knots.size()));
}

// Watertight check on a CCMesh: every undirected edge is used by exactly two triangles.
bool ccMeshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0 || m.vertexCount <= 0) return false;
    std::unordered_map<uint64_t, int> edge;
    auto key = [](int a, int b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int a = m.triangles[3 * t + 0];
        const int b = m.triangles[3 * t + 1];
        const int c = m.triangles[3 * t + 2];
        edge[key(a, b)]++;
        edge[key(b, c)]++;
        edge[key(c, a)]++;
    }
    for (const auto& e : edge)
        if (e.second != 2) return false;
    return true;
}

// Euler characteristic χ = V − E + F over the mesh's USED vertices/edges/faces. A closed,
// genus-0 shell has χ = 2. (Uses only vertices referenced by a triangle so unused mesh
// slots do not inflate V.)
int ccMeshEuler(const CCMesh& m) {
    std::unordered_set<int> verts;
    std::unordered_set<uint64_t> edges;
    auto key = [](int a, int b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int a = m.triangles[3 * t + 0];
        const int b = m.triangles[3 * t + 1];
        const int c = m.triangles[3 * t + 2];
        verts.insert(a);
        verts.insert(b);
        verts.insert(c);
        edges.insert(key(a, b));
        edges.insert(key(b, c));
        edges.insert(key(c, a));
    }
    return static_cast<int>(verts.size()) - static_cast<int>(edges.size()) + m.triangleCount;
}

// Signed enclosed volume via the divergence theorem (1/6 * sum a . (b x c)).
double ccMeshVolume(const CCMesh& m) {
    double vol = 0.0;
    for (int t = 0; t < m.triangleCount; ++t) {
        const double* a = m.vertices + 3 * m.triangles[3 * t + 0];
        const double* b = m.vertices + 3 * m.triangles[3 * t + 1];
        const double* c = m.vertices + 3 * m.triangles[3 * t + 2];
        const double cx = b[1] * c[2] - b[2] * c[1];
        const double cy = b[2] * c[0] - b[0] * c[2];
        const double cz = b[0] * c[1] - b[1] * c[0];
        vol += (a[0] * cx + a[1] * cy + a[2] * cz);
    }
    return vol / 6.0;
}

}  // namespace

// ── The single-seam ops weld watertight at their closed-form op-volume ────────────────
CC_TEST(cc_solid_boolean_single_seam_watertight_and_volume) {
    cc_surface wallA = makeBowlWall(/*downDome=*/false);
    cc_surface wallB = makeBowlWall(/*downDome=*/true);
    CC_CHECK(wallA.id != 0 && wallB.id != 0);
    const double lidA = kA * kR * kR;       // A's lid at z = a·R²
    const double lidB = kH - kA * kR * kR;  // B's lid at z = H − a·R²
    const double d = 0.005;

#ifdef CYBERCAD_HAS_NUMSCI
    struct Case {
        CCBoolOp op;
        double cf;
        const char* name;
    };
    const double fuseVol = volA() + volA() - volCommon();  // V(A)+V(B)−lens (V(B)=V(A))
    const Case cases[] = {
        {CC_BOOL_COMMON, volCommon(), "common"},
        {CC_BOOL_CUT, volCut(), "cut"},
        {CC_BOOL_FUSE, fuseVol, "fuse"},
    };
    for (const Case& tc : cases) {
        CCMesh out{};
        const int ok = cc_nurbs_solid_boolean(wallA, kR, lidA, wallB, kR, lidB, tc.op, d, &out);
        CC_CHECK(ok == 1);
        if (ok != 1) {
            CC_CHECK(cc_last_error()[0] != '\0');
            continue;
        }
        CC_CHECK(out.vertexCount > 0 && out.triangleCount > 0);
        CC_CHECK(ccMeshWatertight(out));    // closed: every edge used exactly twice
        CC_CHECK(ccMeshEuler(out) == 2);    // χ = 2 (closed, genus-0)
        const double v = std::fabs(ccMeshVolume(out));
        const double err = std::fabs(v - tc.cf) / tc.cf;
        CC_CHECK(err < 30.0 * d);           // within the tessellation band
        cc_mesh_free(out);
    }
#else
    CCMesh out{};
    const int ok = cc_nurbs_solid_boolean(wallA, kR, lidA, wallB, kR, lidB, CC_BOOL_COMMON, d, &out);
    CC_CHECK(ok == 0);                       // gated off ⇒ honest decline
    CC_CHECK(out.vertexCount == 0 && out.triangleCount == 0);
    CC_CHECK(cc_last_error()[0] != '\0');
#endif
    cc_surface_release(wallA);
    cc_surface_release(wallB);
}

// ── Multi-seam pose: after the M0-WELD shared-seam-strip fix and the FUSE outer-envelope
//    compose, ALL THREE ops WELD watertight — COMMON/CUT the annular lens (inner seam closes),
//    FUSE the outer envelope (A∪B, complement of the lens on both walls). No op is ever a
//    leaky mesh: the wrapper returns ok only when its own watertight self-verify passes. ──
CC_TEST(cc_solid_boolean_multi_seam_all_ops_weld) {
    cc_surface wallA = makeValleyWall(/*downDome=*/false);
    cc_surface wallB = makeValleyWall(/*downDome=*/true);
    CC_CHECK(wallA.id != 0 && wallB.id != 0);
    constexpr double H = 0.03;
    // lids at the (flat, radially-symmetric) valley/dome rim height (r = R = 0.45).
    const double rimR = 0.45;
    const double zAatR = 4.0 * (rimR * rimR - 0.28 * 0.28) * (rimR * rimR - 0.28 * 0.28);
    const double lidA = zAatR;       // A's top lid
    const double lidB = H - zAatR;   // B's bottom lid
#ifdef CYBERCAD_HAS_NUMSCI
    // COMMON / CUT / FUSE all weld to a closed 2-manifold — the wrapper returns ok only when
    // its own watertight + coherence self-verify passes, so a returned mesh is never leaky.
    // COMMON/CUT (the annular lens) weld to fine deflection; FUSE (the outer envelope, whose
    // survivors include each wall's rim-bounded background annulus) welds in the working band
    // [0.005, 0.01] and honest-declines the finer-deflection frozen-mesher parity residual.
    struct { CCBoolOp op; double d; } cases[] = {
        {CC_BOOL_COMMON, 0.0025}, {CC_BOOL_CUT, 0.0025}, {CC_BOOL_FUSE, 0.005}};
    for (const auto& c : cases) {
        CCMesh out{};
        const int ok = cc_nurbs_solid_boolean(wallA, rimR, lidA, wallB, rimR, lidB, c.op, c.d, &out);
        CC_CHECK(ok == 1);                                     // multi-seam now welds (incl. FUSE)
        CC_CHECK(out.vertexCount > 0 && out.triangleCount > 0);
        CC_CHECK(ccMeshWatertight(out));                       // closed: every edge used exactly twice
        cc_mesh_free(out);
    }
#else
    // Without the numsci substrate the whole path honest-declines.
    for (CCBoolOp op : {CC_BOOL_COMMON, CC_BOOL_CUT, CC_BOOL_FUSE}) {
        CCMesh out{};
        const int ok = cc_nurbs_solid_boolean(wallA, rimR, lidA, wallB, rimR, lidB, op, 0.0025, &out);
        CC_CHECK(ok == 0);
        CC_CHECK(cc_last_error()[0] != '\0');
    }
#endif
    cc_surface_release(wallA);
    cc_surface_release(wallB);
}

// ── An unknown / non-Bézier wall handle honest-declines (no leak) ─────────────────────
CC_TEST(cc_solid_boolean_unknown_wall_declines) {
    cc_surface bogus{999999};
    cc_surface wallB = makeBowlWall(/*downDome=*/true);
    CCMesh out{};
    const int ok = cc_nurbs_solid_boolean(bogus, kR, 0.0, wallB, kR, 0.0, CC_BOOL_COMMON, 0.005, &out);
    CC_CHECK(ok == 0);
    CC_CHECK(out.vertexCount == 0 && out.triangleCount == 0);
    CC_CHECK(cc_last_error()[0] != '\0');
    cc_surface_release(wallB);
}

int main() { return cctest::run_all(); }
