// SPDX-License-Identifier: Apache-2.0
//
// Host test for BOOL-CC-EXTEND: the cc_nurbs_solid_union_n / _cut_n (N-ary fold),
// cc_nurbs_pocket / _boss (feature ops), and cc_nurbs_step_write / _step_read
// (exact trimmed-NURBS STEP round-trip) facade wrappers
// (src/facade/cc_nurbs_boolean_ext.cpp). Everything is driven through the public
// cc_* surface. The bowl-cup fixture mirrors the J7 test (a single-seam bowl-cup pose).
//
// Oracles:
//   1. union_n / cut_n over TWO operands → a WATERTIGHT CCMesh (χ=2, every edge twice),
//      volume in the closed-form band. A ≥3-operand union HONEST-DECLINES (the measured
//      re-admission boundary) with cc_last_error + a zeroed CCMesh (no leaky mesh).
//   2. pocket / boss over two operands → a WATERTIGHT CCMesh.
//   3. STEP: write two cc_surfaces → read back → the recovered surfaces evaluate ≤ 1e-9
//      of the originals on a (u,v) grid (bit-exact NURBS round-trip). An empty set / a
//      malformed STEP honest-declines.
//
// The N-ary fold + feature ops compose the numsci SSI seam trace, so those assertions
// are gated on CYBERCAD_HAS_NUMSCI (with the guard OFF the wrappers honest-decline,
// asserted). The STEP round-trip is numsci-free and always exercised.

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── single-seam fixture constants (mirror the J7 boolean test) ──
constexpr double kA = 2.0;    // bowl amplitude
constexpr double kR = 0.35;   // rim radius (in the wall's u,v)
constexpr double kH = 0.16;   // B's dome apex height

double volA() { return kPi * kA * kR * kR * kR * kR / 2.0; }
double volCommon() { return kPi * kH * kH / (4.0 * kA); }
double volCut() { return volA() - volCommon(); }

cc_surface makeBowlWall(bool downDome) {
    const double xc[3] = {-0.5, 0.0, 0.5};
    const double zc[3] = {0.25 * kA, -0.25 * kA, 0.25 * kA};
    std::vector<double> polesXYZW;
    polesXYZW.reserve(9 * 4);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double z = zc[i] + zc[j];
            if (downDome) z = kH - z;
            polesXYZW.push_back(xc[i]);
            polesXYZW.push_back(xc[j]);
            polesXYZW.push_back(z);
            polesXYZW.push_back(1.0);
        }
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    return cc_surface_create(2, 2, polesXYZW.data(), 3, 3, knots.data(),
                             static_cast<int>(knots.size()), knots.data(),
                             static_cast<int>(knots.size()));
}

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

// ── N-ary union / cut over TWO operands weld watertight at the closed-form volume ──
CC_TEST(cc_nary_two_operand_watertight_and_volume) {
    cc_surface wallA = makeBowlWall(/*downDome=*/false);
    cc_surface wallB = makeBowlWall(/*downDome=*/true);
    CC_CHECK(wallA.id != 0 && wallB.id != 0);
    const double lidA = kA * kR * kR;
    const double lidB = kH - kA * kR * kR;
    const double d = 0.005;

#ifdef CYBERCAD_HAS_NUMSCI
    const cc_surface walls[2] = {wallA, wallB};
    const double rims[2] = {kR, kR};
    const double lids[2] = {lidA, lidB};

    // union_n([A,B]) → watertight, V = V(A)+V(B)−lens
    {
        CCMesh out{};
        const int ok = cc_nurbs_solid_union_n(walls, rims, lids, 2, d, &out);
        CC_CHECK(ok == 1);
        if (ok == 1) {
            CC_CHECK(ccMeshWatertight(out));
            CC_CHECK(ccMeshEuler(out) == 2);
            const double v = std::fabs(ccMeshVolume(out));
            const double cf = volA() + volA() - volCommon();
            CC_CHECK(std::fabs(v - cf) / cf < 30.0 * d);
            cc_mesh_free(out);
        }
    }
    // cut_n(A, [B]) → watertight, V = V(A) − lens
    {
        const cc_surface tools[1] = {wallB};
        const double trims[1] = {kR};
        const double tlids[1] = {lidB};
        CCMesh out{};
        const int ok = cc_nurbs_solid_cut_n(wallA, kR, lidA, tools, trims, tlids, 1, d, &out);
        CC_CHECK(ok == 1);
        if (ok == 1) {
            CC_CHECK(ccMeshWatertight(out));
            CC_CHECK(ccMeshEuler(out) == 2);
            const double v = std::fabs(ccMeshVolume(out));
            CC_CHECK(std::fabs(v - volCut()) / volCut() < 30.0 * d);
            cc_mesh_free(out);
        }
    }
#else
    CCMesh out{};
    const cc_surface walls[2] = {wallA, wallB};
    const double rims[2] = {kR, kR};
    const double lids[2] = {lidA, lidB};
    CC_CHECK(cc_nurbs_solid_union_n(walls, rims, lids, 2, d, &out) == 0);
    CC_CHECK(out.vertexCount == 0 && out.triangleCount == 0);
    CC_CHECK(cc_last_error()[0] != '\0');
#endif
    cc_surface_release(wallA);
    cc_surface_release(wallB);
}

// ── A >=3-operand union with a REDUNDANT (contained) third operand now WELDS ──
// BOOL-READMIT re-admits the binary boolean output as an N-ary input; the third solid here
// is a duplicate of A, which is contained in A∪B, so the fold short-circuits it to acc EXACTLY
// (no synthesized geometry) and the result equals the 2-operand union V(A)+V(B)−lens.
// (The general genuine-overlap ≥3 weld — a second seam on an already-holed annulus — remains a
// documented multi-hole-split residual; it still honest-declines, never leaky.)
CC_TEST(cc_nary_three_operand_redundant_union_welds) {
    cc_surface wA = makeBowlWall(false);
    cc_surface wB = makeBowlWall(true);
    cc_surface wC = makeBowlWall(false);  // duplicate of A ⇒ redundant (contained in A∪B)
    CC_CHECK(wA.id != 0 && wB.id != 0 && wC.id != 0);
    const double lidA = kA * kR * kR;
    const double lidB = kH - kA * kR * kR;
    const double d = 0.005;
    const cc_surface walls[3] = {wA, wB, wC};
    const double rims[3] = {kR, kR, kR};
    const double lids[3] = {lidA, lidB, lidA};
    CCMesh out{};
    const int ok = cc_nurbs_solid_union_n(walls, rims, lids, 3, d, &out);
#ifdef CYBERCAD_HAS_NUMSCI
    CC_CHECK(ok == 1);  // redundant third operand short-circuits ⇒ welds to A∪B
    if (ok == 1) {
        CC_CHECK(ccMeshWatertight(out));
        CC_CHECK(ccMeshEuler(out) == 2);
        const double v = std::fabs(ccMeshVolume(out));
        const double cf = volA() + volA() - volCommon();  // = V(A∪B)
        CC_CHECK(std::fabs(v - cf) / cf < 30.0 * d);
        cc_mesh_free(out);
    }
#else
    CC_CHECK(ok == 0);  // numsci-absent decline
    CC_CHECK(out.vertexCount == 0 && out.triangleCount == 0);
    CC_CHECK(cc_last_error()[0] != '\0');
#endif
    cc_surface_release(wA);
    cc_surface_release(wB);
    cc_surface_release(wC);
}

// ── pocket / boss over two operands weld watertight ──
CC_TEST(cc_feature_pocket_boss_watertight) {
    cc_surface wallA = makeBowlWall(false);
    cc_surface wallB = makeBowlWall(true);
    CC_CHECK(wallA.id != 0 && wallB.id != 0);
    const double lidA = kA * kR * kR;
    const double lidB = kH - kA * kR * kR;
    const double d = 0.005;
#ifdef CYBERCAD_HAS_NUMSCI
    {
        CCMesh out{};
        const int ok = cc_nurbs_pocket(wallA, kR, lidA, wallB, kR, lidB, d, &out);
        CC_CHECK(ok == 1);
        if (ok == 1) {
            CC_CHECK(ccMeshWatertight(out));
            CC_CHECK(ccMeshEuler(out) == 2);
            cc_mesh_free(out);
        }
    }
    {
        CCMesh out{};
        const int ok = cc_nurbs_boss(wallA, kR, lidA, wallB, kR, lidB, d, &out);
        CC_CHECK(ok == 1);
        if (ok == 1) {
            CC_CHECK(ccMeshWatertight(out));
            CC_CHECK(ccMeshEuler(out) == 2);
            cc_mesh_free(out);
        }
    }
#else
    CCMesh out{};
    CC_CHECK(cc_nurbs_pocket(wallA, kR, lidA, wallB, kR, lidB, d, &out) == 0);
    CC_CHECK(cc_last_error()[0] != '\0');
    CC_CHECK(cc_nurbs_boss(wallA, kR, lidA, wallB, kR, lidB, d, &out) == 0);
#endif
    cc_surface_release(wallA);
    cc_surface_release(wallB);
}

// ── STEP write→read round-trips the NURBS surface data bit-exact (numsci-free) ──
CC_TEST(cc_step_roundtrip_bit_exact) {
    cc_surface a = makeBowlWall(false);
    cc_surface b = makeBowlWall(true);
    CC_CHECK(a.id != 0 && b.id != 0);
    const cc_surface set[2] = {a, b};

    char* step = nullptr;
    const int wok = cc_nurbs_step_write(set, 2, &step);
    CC_CHECK(wok == 1);
    CC_CHECK(step != nullptr && std::strlen(step) > 0);
    // A valid ISO-10303-21 file starts with the Part-21 magic.
    CC_CHECK(std::strncmp(step, "ISO-10303-21;", 13) == 0);

    // Count-query first.
    const int count = cc_nurbs_step_read(step, nullptr, 0);
    CC_CHECK(count == 2);

    cc_surface got[4] = {};
    const int rok = cc_nurbs_step_read(step, got, 4);
    CC_CHECK(rok == 2);
    if (rok == 2) {
        // Evaluate each recovered surface against the original on a (u,v) grid ≤ 1e-9.
        double maxErr = 0.0;
        for (int s = 0; s < 2; ++s) {
            const cc_surface orig = set[s];
            const cc_surface rec = got[s];
            for (int iu = 0; iu <= 4; ++iu)
                for (int iv = 0; iv <= 4; ++iv) {
                    const double u = iu / 4.0;
                    const double v = iv / 4.0;
                    double p0[3] = {}, p1[3] = {};
                    CC_CHECK(cc_surface_eval(orig, u, v, p0) == 1);
                    CC_CHECK(cc_surface_eval(rec, u, v, p1) == 1);
                    for (int k = 0; k < 3; ++k)
                        maxErr = std::max(maxErr, std::fabs(p0[k] - p1[k]));
                }
            cc_surface_release(rec);
        }
        CC_CHECK(maxErr <= 1e-9);
    }
    cc_string_free(step);

    // Empty set + malformed string honest-decline.
    char* nul = nullptr;
    CC_CHECK(cc_nurbs_step_write(set, 0, &nul) == 0);
    CC_CHECK(nul == nullptr);
    CC_CHECK(cc_last_error()[0] != '\0');
    cc_surface bad[2] = {};
    CC_CHECK(cc_nurbs_step_read("not a step file", bad, 2) < 0);
    CC_CHECK(cc_last_error()[0] != '\0');

    cc_surface_release(a);
    cc_surface_release(b);
}

int main() { return cctest::run_all(); }
