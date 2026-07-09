// SPDX-License-Identifier: Apache-2.0
//
// native_abi_parity.mm — SIM Gate (b) for the MOAT ABI-parity track (the 6 functions
// the CyberCad app links but the kernel was missing). On a booted iOS simulator (OCCT
// linked), through the SHIPPING cc_* facade, this harness verifies:
//
//   A) The four app-parity LOFT variants against the OCCT oracle AND their closed form:
//        * cc_loft_circles      — two coaxial circles → a conical frustum: OCCT volume
//          matches the closed form π·h·(r²+rR+R²)/3 (and the cylinder π·r²·h when r==R),
//          watertight, and a SMOOTH B-rep (one circular side face family + circular rims,
//          NOT a 64-gon), i.e. a small face/edge count.
//        * cc_loft_circle_wire  — a true circle → a square polygon: watertight, and the
//          circle rim stays ONE edge (a small edge count, not a sampled polygon).
//        * cc_loft_typed        — two typed full-circle sections on their own frames →
//          the SAME frustum as cc_loft_circles (closed-form volume), watertight.
//        * cc_loft_along_rails  — a straight rail + a straight guide + two square profiles
//          → a straight square prism (the two-rail sweep reduces to a ruled loft): OCCT
//          volume matches the closed form (base area × height), watertight.
//
//   B) Connected-solid ENUMERATION native-vs-OCCT (cc_shape_solid_count / cc_shape_solid_at):
//      an OCCT-built compound of TWO disjoint boxes (cc_boolean fuse of two separated boxes)
//      → count == 2; each extracted solid re-enumerates to 1 and carries its own box volume;
//      the two volumes sum to the whole. Because the compound is an OCCT body, the native
//      engine's shape_solid_count/at FORWARD it to the OCCT fallback, so the native and OCCT
//      answers MUST be identical (the app's real connectedSolids() scenario).
//
//   cc_set_engine(0) → OCCT (the oracle / default);  cc_set_engine(1) → NativeEngine.
//
// Output: [ABI] PASS/FAIL lines, then a summary. On run-sim-suite.sh's SKIP list (own main()).
//
// Build: scripts/run-sim-native-abi.sh — compiles this harness + the whole facade/core/
// engine (NativeEngine + OCCT adapter) + src/native/**, links OCCT, spawns on a simulator.

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

int g_passed = 0;
int g_failed = 0;

void record(bool ok, const std::string& label, const char* detail) {
    if (ok) {
        ++g_passed;
        std::printf("[ABI] PASS  %-32s %s\n", label.c_str(), detail);
    } else {
        ++g_failed;
        std::printf("[ABI] FAIL  %-32s %s\n", label.c_str(), detail);
    }
}

struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

int subCount(CCShapeId id, int kind) {
    int* ids = nullptr;
    const int n = cc_subshape_ids(id, kind, &ids);
    cc_ints_free(ids);
    return n;
}

// ── A) loft variants vs OCCT + closed form ──────────────────────────────────────

// A closed conical frustum, r at z=0 → R at z=h.
double frustumVolume(double r, double R, double h) { return kPi * h * (r * r + r * R + R * R) / 3.0; }

// A B-rep is valid iff the OCCT analyzer (cc_check_solid) says so. The tessellated mesh
// of a smooth circular face is NOT a naive 2-manifold triangle soup (OCCT emits a
// triangle fan sharing a rim apex / degenerate seam), so the mesh-edge counter is the
// WRONG watertight oracle here — the B-rep analyzer is the correct one.
bool brepValid(CCShapeId id) {
    CCValidityReport r{};
    const int decided = cc_check_solid(id, &r);
    return decided == 1 && r.decided == 1 && r.valid == 1;
}

void checkLoftFrustumClosedForm(const std::string& name, CCShapeId id, double expectVol,
                                double deflection, int maxFaces, int maxEdges) {
    char d[512];
    if (id == 0) {
        std::snprintf(d, sizeof d, "build failed: %s", cc_last_error());
        record(false, name, d);
        return;
    }
    const CCMassProps m = cc_mass_properties(id);
    const bool valid = brepValid(id);           // OCCT analyzer = the watertight oracle
    const CCMesh mesh = cc_tessellate(id, deflection);
    const bool meshed = mesh.triangleCount > 0;  // a real display mesh was produced
    const int nf = subCount(id, 2), ne = subCount(id, 1);
    // The B-rep VOLUME is exact (analytic mass_properties, not the mesh), so a true-circle
    // frustum must match the closed form to fp precision — NOT deflection-bounded.
    const double volRel = (m.valid && expectVol > 0) ? std::fabs(m.volume - expectVol) / expectVol : 1.0;
    const bool volOk = m.valid && volRel < 1.0e-6;
    // A true circular rim keeps a SMALL face/edge count (one side face + circular rims), NOT
    // a sampled 64-gon.
    const bool smooth = (maxFaces <= 0 || nf <= maxFaces) && (maxEdges <= 0 || ne <= maxEdges);
    std::snprintf(d, sizeof d, "vol occt=%.6g closed=%.6g rel=%.2e | valid=%d meshed=%d faces=%d edges=%d (smooth<=%d/%d)",
                  m.volume, expectVol, volRel, valid ? 1 : 0, meshed ? 1 : 0, nf, ne, maxFaces, maxEdges);
    record(volOk && valid && meshed && smooth, name, d);
    cc_mesh_free(mesh);
}

// ── B) solid enumeration native-vs-OCCT on an OCCT-built 2-lump compound ─────────

// Build a compound of two DISJOINT boxes via an OCCT boolean fuse (the app's real
// connectedSolids scenario: a fuse of disjoint bodies is an OCCT compound of 2 solids).
// Built with the OCCT engine ACTIVE. Returns 0 on failure. NOTE: the app only ever calls
// connectedSolids() on such a compound; the native engine's shape_solid_count/at recognise
// an OCCT body and FORWARD it to the OCCT fallback, so the answers are identical.
CCShapeId buildTwoLumpCompound(double& vTotal) {
    const double a[] = {0, 0, 4, 0, 4, 4, 0, 4};                            // 4×4 → box vol 64
    const double b[] = {100, 100, 102, 100, 102, 102, 100, 102};            // 2×2 → box vol 8
    const CCShapeId ba = cc_solid_extrude(a, 4, 4.0);
    const CCShapeId bb = cc_solid_extrude(b, 4, 2.0);
    if (ba == 0 || bb == 0) return 0;
    const CCShapeId comp = cc_boolean(ba, bb, 0 /*fuse → disjoint compound*/);
    cc_shape_release(ba);
    cc_shape_release(bb);
    vTotal = 64.0 + 8.0;
    return comp;
}

// Enumerate the SAME (OCCT-built) compound under whichever engine is currently active.
void checkEnumeration(const char* engineName, CCShapeId comp, double vTotal) {
    char d[512];
    if (comp == 0) {
        std::snprintf(d, sizeof d, "compound build failed: %s", cc_last_error());
        record(false, std::string("enum ") + engineName + " build", d);
        return;
    }
    const int n = cc_shape_solid_count(comp);
    std::snprintf(d, sizeof d, "solid_count=%d (expect 2)", n);
    record(n == 2, std::string("enum ") + engineName + " count", d);

    double vSum = 0.0;
    bool eachSingle = true, eachValid = true;
    for (int i = 0; i < n; ++i) {
        const CCShapeId s = cc_shape_solid_at(comp, i);
        if (s == 0) {
            eachValid = false;
            continue;
        }
        if (cc_shape_solid_count(s) != 1) eachSingle = false;
        const CCMassProps mp = cc_mass_properties(s);
        if (mp.valid) vSum += mp.volume;
        cc_shape_release(s);
    }
    const double sumRel = vTotal > 0 ? std::fabs(vSum - vTotal) / vTotal : 1.0;
    std::snprintf(d, sizeof d, "per-solid single=%d valid=%d volΣ=%.6g whole=%.6g rel=%.2e",
                  eachSingle ? 1 : 0, eachValid ? 1 : 0, vSum, vTotal, sumRel);
    record(eachSingle && eachValid && sumRel < 1e-6, std::string("enum ") + engineName + " per-solid", d);

    // Out-of-range / negative index → 0 (no crash).
    const bool oob = cc_shape_solid_at(comp, n) == 0 && cc_shape_solid_at(comp, -1) == 0;
    record(oob, std::string("enum ") + engineName + " oob-zero", oob ? "index guards return 0" : "guard failed");
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at exit

    // ── A) loft variants (OCCT oracle vs closed form) ───────────────────────────
    std::printf("── app-parity LOFT variants vs OCCT oracle + closed form (cc_* facade)\n");
    cc_set_engine(0);  // OCCT is the oracle for the smooth-B-rep loft topology

    // 1) cc_loft_circles: coaxial circles r=5 @z=0 → R=3 @z=10 → frustum.
    {
        const double c1[] = {0, 0, 0}, n1[] = {0, 0, 1};
        const double c2[] = {0, 0, 10}, n2[] = {0, 0, 1};
        const CCShapeId id = cc_loft_circles(c1, n1, 5.0, c2, n2, 3.0);
        // A true-circle frustum: one side face + two circular rims (few faces/edges), not a 64-gon.
        checkLoftFrustumClosedForm("loft_circles frustum", id, frustumVolume(5.0, 3.0, 10.0),
                                   0.02, /*maxFaces*/ 6, /*maxEdges*/ 8);
        cc_shape_release(id);
    }
    // 1b) cc_loft_circles cylinder degenerate (r==R): closed form π r² h.
    {
        const double c1[] = {0, 0, 0}, n1[] = {0, 0, 1};
        const double c2[] = {0, 0, 8}, n2[] = {0, 0, 1};
        const CCShapeId id = cc_loft_circles(c1, n1, 4.0, c2, n2, 4.0);
        checkLoftFrustumClosedForm("loft_circles cylinder", id, kPi * 16.0 * 8.0, 0.02, 6, 8);
        cc_shape_release(id);
    }
    // 2) cc_loft_circle_wire: circle r=5 @z=0 → 4×4 square @z=10. Watertight; circle rim = 1 edge.
    {
        const double c[] = {0, 0, 0}, nrm[] = {0, 0, 1};
        const double sq[] = {-2, -2, 10, 2, -2, 10, 2, 2, 10, -2, 2, 10};
        const CCShapeId id = cc_loft_circle_wire(c, nrm, 5.0, sq, 4);
        char d[256];
        if (id == 0) {
            std::snprintf(d, sizeof d, "build failed: %s", cc_last_error());
            record(false, "loft_circle_wire", d);
        } else {
            const CCMassProps m = cc_mass_properties(id);
            const bool valid = brepValid(id);
            const CCMesh mesh = cc_tessellate(id, 0.02);
            const bool meshed = mesh.triangleCount > 0;
            const int ne = subCount(id, 1);
            // circle rim (1) + 4 square edges + side edges: a true circle keeps the rim ONE edge,
            // so the total edge count is small (< 16) — NOT a sampled 64-gon rim.
            std::snprintf(d, sizeof d, "vol=%.6g valid=%d meshed=%d edges=%d (rim=1 edge, not 64-gon)",
                          m.volume, valid ? 1 : 0, meshed ? 1 : 0, ne);
            record(m.valid && m.volume > 0 && valid && meshed && ne < 16, "loft_circle_wire", d);
            cc_mesh_free(mesh);
            cc_shape_release(id);
        }
    }
    // 3) cc_loft_typed: two full-circle typed sections on their own frames → same frustum as (1).
    {
        CCProfileSeg a{};
        a.kind = 2; a.cx = 0; a.cy = 0; a.r = 5.0;
        CCProfileSeg b{};
        b.kind = 2; b.cx = 0; b.cy = 0; b.r = 3.0;
        const double fa[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};   // XOY plane @z=0
        const double fb[] = {0, 0, 10, 1, 0, 0, 0, 1, 0};  // XOY plane @z=10
        const CCShapeId id = cc_loft_typed(&a, 1, nullptr, 0, fa, &b, 1, nullptr, 0, fb);
        checkLoftFrustumClosedForm("loft_typed circle-frustum", id, frustumVolume(5.0, 3.0, 10.0),
                                   0.02, /*maxFaces*/ 6, /*maxEdges*/ 8);
        cc_shape_release(id);
    }
    // 4) cc_loft_along_rails: straight rail (z 0→10) + straight guide + two 4×4 profiles →
    //    a straight 4×4×10 square prism (base area 16 × height 10 = 1600).
    {
        const double rail[] = {0, 0, 0, 0, 0, 10};
        const double gd[] = {8, 0, 0, 8, 0, 10};
        const double pa[] = {-2, -2, 2, -2, 2, 2, -2, 2};
        const double pb[] = {-2, -2, 2, -2, 2, 2, -2, 2};
        const CCShapeId id = cc_loft_along_rails(rail, 2, gd, 2, pa, 4, pb, 4);
        char d[256];
        if (id == 0) {
            std::snprintf(d, sizeof d, "build failed: %s", cc_last_error());
            record(false, "loft_along_rails prism", d);
        } else {
            const CCMassProps m = cc_mass_properties(id);
            const bool valid = brepValid(id);
            const CCMesh mesh = cc_tessellate(id, 0.05);
            const bool meshed = mesh.triangleCount > 0;
            const double expect = 16.0 * 10.0;
            const double volRel = m.valid ? std::fabs(m.volume - expect) / expect : 1.0;
            std::snprintf(d, sizeof d, "vol=%.6g expect=%.6g rel=%.2e valid=%d meshed=%d", m.volume,
                          expect, volRel, valid ? 1 : 0, meshed ? 1 : 0);
            record(m.valid && volRel < 2e-2 && valid && meshed, "loft_along_rails prism", d);
            cc_mesh_free(mesh);
            cc_shape_release(id);
        }
    }

    // ── B) solid enumeration parity (OCCT-built compound, both engines) ─────────
    std::printf("── connected-solid enumeration native-vs-OCCT (cc_shape_solid_count/at)\n");
    // Build the disjoint-fuse compound ONCE under OCCT (the app's real connectedSolids body).
    cc_set_engine(0);
    double vTotal = 0.0;
    const CCShapeId comp = buildTwoLumpCompound(vTotal);
    // Enumerate the SAME body under each engine. The compound is an OCCT body, so the native
    // engine forwards it to the OCCT fallback → native and OCCT answers MUST be identical.
    checkEnumeration("occt", comp, vTotal);
    cc_set_engine(1);
    checkEnumeration("native", comp, vTotal);
    cc_set_engine(0);
    cc_shape_release(comp);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
