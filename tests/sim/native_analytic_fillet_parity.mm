// SPDX-License-Identifier: Apache-2.0
//
// native_analytic_fillet_parity.mm — native-vs-OCCT ANALYTIC face-fillet parity
//                                     harness, driven THROUGH the cc_* facade (iOS sim).
//
// MOAT M3 (`moat-m3af-analytic-fillet`) — the tractable analytic-solid face fillets:
//   * FULL ROUND (prismatic rib): cc_full_round_fillet[_faces] replaces the narrow
//     top strip of a rib between two PARALLEL planar walls with a tangent half-cylinder
//     cap of radius r = strip-width/2 (the r = w/2 special case of the rolling-ball
//     blend, built on the landed fillet_edges on the two OPPOSITE seam edges). LANDS
//     NATIVE. Compared vs the OCCT full-round oracle on volume / area / watertight /
//     Euler χ=2 / bbox / Hausdorff.
//   * FILLET_FACE (full-face): cc_fillet_face rounds EVERY edge of the picked face. On
//     a planar solid the bounding edges form a corner-sharing loop whose corner-sphere
//     weld gates on M2, so the native op HONESTLY DECLINES (returns 0) and OCCT owns
//     the BRepFilletAPI_MakeFillet reference. Asserted: native id 0, OCCT id ≠ 0.
//
// This harness drives the SHIPPING PATH: the public cc_* facade under BOTH engines
// (cc_set_engine(0)=OCCT oracle, cc_set_engine(1)=NativeEngine) and compares. A rib
// built via cc_solid_extrude_profile (a rectangle of 4 line segments) is a NATIVE
// all-planar body under engine 1.
//
// The native faceted cap differs from OCCT's true cylinder, so vol/area/mesh compare
// with a loose (deflection-bounded) tolerance; the native result MUST be watertight
// and Euler χ = 2.
//
// Output: [NAFILLET] PASS/FAIL lines. Exits std::_Exit(failed?1:0).
//
// Build: scripts/run-sim-native-analytic-fillet.sh (models run-sim-native-curved-fillet.sh).

#include "cybercadkernel/cc_kernel.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

int g_passed = 0;
int g_failed = 0;
void record(bool ok, const std::string& label, const char* detail) {
    if (ok) { ++g_passed; std::printf("[NAFILLET] PASS  %-34s %s\n", label.c_str(), detail); }
    else    { ++g_failed; std::printf("[NAFILLET] FAIL  %-34s %s\n", label.c_str(), detail); }
}

// Watertight over POSITION-WELDED vertices (engine-agnostic: OCCT emits per-face vertex
// copies, native corners may land a hair apart) + Euler χ over the welded mesh.
struct MeshTopo { bool watertight = false; int chi = 0; };
MeshTopo meshTopo(const CCMesh& m) {
    MeshTopo out;
    if (m.triangleCount <= 0) return out;
    constexpr double kWeld = 1e-6;
    std::unordered_map<std::uint64_t, std::vector<int>> cellReps;
    std::vector<int> rep(static_cast<std::size_t>(m.vertexCount));
    auto cellKey = [](long long x, long long y, long long z) -> std::uint64_t {
        std::uint64_t h = static_cast<std::uint64_t>(x) * 73856093u;
        h ^= static_cast<std::uint64_t>(y) * 19349663u;
        h ^= static_cast<std::uint64_t>(z) * 83492791u;
        return h;
    };
    auto q = [](double v) -> long long {
        const double s = v / kWeld;
        return static_cast<long long>(s >= 0 ? s + 0.5 : s - 0.5);
    };
    int uniq = 0;
    for (int v = 0; v < m.vertexCount; ++v) {
        const double* p = &m.vertices[v * 3];
        const long long cx = q(p[0]), cy = q(p[1]), cz = q(p[2]);
        int match = -1;
        for (long long dx = -1; dx <= 1 && match < 0; ++dx)
            for (long long dy = -1; dy <= 1 && match < 0; ++dy)
                for (long long dz = -1; dz <= 1 && match < 0; ++dz) {
                    auto it = cellReps.find(cellKey(cx + dx, cy + dy, cz + dz));
                    if (it == cellReps.end()) continue;
                    for (int rid : it->second) {
                        const double* rp = &m.vertices[rid * 3];
                        if (std::fabs(rp[0] - p[0]) <= kWeld && std::fabs(rp[1] - p[1]) <= kWeld &&
                            std::fabs(rp[2] - p[2]) <= kWeld) { match = rid; break; }
                    }
                }
        if (match >= 0) rep[static_cast<std::size_t>(v)] = match;
        else { rep[static_cast<std::size_t>(v)] = v; cellReps[cellKey(cx, cy, cz)].push_back(v); ++uniq; }
    }
    std::unordered_map<std::uint64_t, int> edgeCount;
    auto key = [](int a, int b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int i = rep[static_cast<std::size_t>(m.triangles[t * 3 + 0])];
        const int j = rep[static_cast<std::size_t>(m.triangles[t * 3 + 1])];
        const int k = rep[static_cast<std::size_t>(m.triangles[t * 3 + 2])];
        ++edgeCount[key(i, j)]; ++edgeCount[key(j, k)]; ++edgeCount[key(k, i)];
    }
    bool wt = true;
    for (const auto& [e, c] : edgeCount) if (c != 2) { wt = false; break; }
    out.watertight = wt;
    // Euler χ = V − E + F over the welded, deduped-edge mesh (E = unique undirected).
    out.chi = uniq - static_cast<int>(edgeCount.size()) + m.triangleCount;
    return out;
}

double meshVolume(const CCMesh& m) {
    double v6 = 0.0;
    for (int t = 0; t < m.triangleCount; ++t) {
        const double* A = &m.vertices[m.triangles[t * 3 + 0] * 3];
        const double* B = &m.vertices[m.triangles[t * 3 + 1] * 3];
        const double* C = &m.vertices[m.triangles[t * 3 + 2] * 3];
        v6 += A[0] * (B[1] * C[2] - B[2] * C[1]) - A[1] * (B[0] * C[2] - B[2] * C[0]) +
              A[2] * (B[0] * C[1] - B[1] * C[0]);
    }
    return std::fabs(v6) / 6.0;
}

// Per-axis bbox of a mesh into out6 = [minx,miny,minz,maxx,maxy,maxz].
void meshBBox(const CCMesh& m, double out6[6]) {
    out6[0] = out6[1] = out6[2] = 1e300;
    out6[3] = out6[4] = out6[5] = -1e300;
    for (int v = 0; v < m.vertexCount; ++v) {
        const double* p = &m.vertices[v * 3];
        for (int a = 0; a < 3; ++a) {
            if (p[a] < out6[a]) out6[a] = p[a];
            if (p[a] > out6[3 + a]) out6[3 + a] = p[a];
        }
    }
}

// One-sided Hausdorff (max over A vertices of min distance to B vertices), a coarse
// O(Na·Nb) probe adequate at these mesh sizes.
double hausdorffOneSided(const CCMesh& a, const CCMesh& b) {
    double worst = 0.0;
    for (int i = 0; i < a.vertexCount; ++i) {
        const double* pa = &a.vertices[i * 3];
        double best = 1e300;
        for (int j = 0; j < b.vertexCount; ++j) {
            const double* pb = &b.vertices[j * 3];
            const double dx = pa[0] - pb[0], dy = pa[1] - pb[1], dz = pa[2] - pb[2];
            const double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < best) best = d2;
        }
        if (best > worst) worst = best;
    }
    return std::sqrt(worst);
}

// A prismatic rib [0,L]×[0,w]×[0,h] as a rectangle profile extruded +z. Built with the
// ACTIVE engine (native under engine 1).
CCShapeId buildRib(double L, double w, double h) {
    CCProfileSeg segs[4]{};
    const double xs[4] = {0, L, L, 0};
    const double ys[4] = {0, 0, w, w};
    for (int i = 0; i < 4; ++i) {
        segs[i].kind = 0;
        segs[i].x0 = xs[i]; segs[i].y0 = ys[i];
        segs[i].x1 = xs[(i + 1) % 4]; segs[i].y1 = ys[(i + 1) % 4];
    }
    return cc_solid_extrude_profile(segs, 4, nullptr, 0, nullptr, 0, h);
}

// Geometric normal + centroid of a face mesh, so we can pick a face by its plane
// (engine-independent). Returns faceId of the face whose geometric outward normal is
// ~parallel to `n` and touches point `on`.
int pickFace(CCShapeId body, const double n[3], const double on[3], double defl) {
    CCFaceMesh* faces = nullptr;
    const int nf = cc_face_meshes(body, defl, &faces);
    int found = 0;
    for (int i = 0; i < nf && found == 0; ++i) {
        const CCFaceMesh& f = faces[i];
        if (f.triangleCount < 1) continue;
        // area-weighted geometric normal + a point on the face
        double nx = 0, ny = 0, nz = 0, px = 0, py = 0, pz = 0;
        for (int t = 0; t < f.triangleCount; ++t) {
            const double* A = &f.vertices[f.triangles[t * 3 + 0] * 3];
            const double* B = &f.vertices[f.triangles[t * 3 + 1] * 3];
            const double* C = &f.vertices[f.triangles[t * 3 + 2] * 3];
            const double ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2];
            const double vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
            nx += uy * vz - uz * vy; ny += uz * vx - ux * vz; nz += ux * vy - uy * vx;
            px = A[0]; py = A[1]; pz = A[2];
        }
        const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-9) continue;
        nx /= len; ny /= len; nz /= len;
        const double dot = nx * n[0] + ny * n[1] + nz * n[2];
        // distance of `on` from the face plane through (px,py,pz)
        const double d = (on[0] - px) * nx + (on[1] - py) * ny + (on[2] - pz) * nz;
        if (dot > 0.999 && std::fabs(d) < 1e-4) found = f.faceId;
    }
    cc_face_meshes_free(faces, nf);
    return found;
}

struct Snapshot {
    CCShapeId id = 0;
    CCMassProps mass{0, 0, 0, 0, 0, 0};
};

bool nearRel(double got, double want, double rel) {
    return std::fabs(got - want) <= rel * std::max(1.0, std::fabs(want));
}

// ── FULL ROUND: prismatic rib top, both engines, compare ─────────────────────────
void runFullRound(double L, double w, double h) {
    char lbl[80];
    std::snprintf(lbl, sizeof lbl, "full-round L=%.0f w=%.1f h=%.1f", L, w, h);
    const std::string base = lbl;

    // Build + round under OCCT (oracle) and under native.
    auto build = [&](int eng) -> Snapshot {
        cc_set_engine(eng);
        const CCShapeId body = buildRib(L, w, h);
        Snapshot s;
        if (body != 0) {
            const double up[3] = {0, 0, 1};
            const double top[3] = {L / 2, w / 2, h};
            const int mid = pickFace(body, up, top, 0.02);
            if (mid != 0) {
                s.id = cc_full_round_fillet(body, mid);
                if (s.id != 0) s.mass = cc_mass_properties(s.id);
            }
            cc_shape_release(body);
        }
        return s;
    };
    const Snapshot oracle = build(0);
    const Snapshot cand = build(1);

    char detail[512];
    if (oracle.id == 0 || !oracle.mass.valid) {
        std::snprintf(detail, sizeof detail, "OCCT oracle failed to build");
        record(false, base + " oracle", detail);
        return;
    }
    if (cand.id == 0) {
        std::snprintf(detail, sizeof detail,
                      "native returned 0 (expected a landed prismatic full round)");
        record(false, base + " native-lands", detail);
        return;
    }

    // Analytic removed volume: (w²/2)(1−π/4)·L.
    const double v0 = L * w * h;
    const double removed = 0.5 * w * w * (1.0 - kPi / 4.0) * L;
    const double vExpect = v0 - removed;

    const double volRel = std::fabs(cand.mass.volume - oracle.mass.volume) /
                          std::max(1.0, oracle.mass.volume);
    const double areaRel = std::fabs(cand.mass.area - oracle.mass.area) /
                           std::max(1.0, oracle.mass.area);
    const double volAnalytic = std::fabs(cand.mass.volume - vExpect) / std::max(1.0, vExpect);

    const CCMesh cM = cc_tessellate(cand.id, 0.02);
    const CCMesh oM = cc_tessellate(oracle.id, 0.02);
    const MeshTopo ct = meshTopo(cM);
    double cB[6], oB[6];
    meshBBox(cM, cB); meshBBox(oM, oB);
    double bboxMax = 0.0;
    for (int i = 0; i < 6; ++i) bboxMax = std::max(bboxMax, std::fabs(cB[i] - oB[i]));
    const double haus = std::max(hausdorffOneSided(cM, oM), hausdorffOneSided(oM, cM));
    const double meshVolRel = std::fabs(meshVolume(cM) - cand.mass.volume) /
                              std::max(1.0, cand.mass.volume);

    // The native cap is a FACETED half-cylinder of radius r = w/2; OCCT's cap is a
    // SMOOTH cylinder. A vertex-to-vertex Hausdorff between the sparse native facet
    // ring and OCCT's dense samples is bounded by the native facet SPACING r·Δθ (the
    // gap from an OCCT sample to the nearest native facet vertex), NOT by geometry
    // error — the surfaces coincide to the deflection sagitta. So the honest, deflection-
    // bounded Hausdorff envelope is that facet spacing (+ the 0.02 tessellation chord).
    // This is why the sibling curved-fillet harness uses a volume-bound only.
    const double r = 0.5 * w;
    const double engDefl = 0.01;  // the engine's default full-round facet deflection
    const double maxStep = 2.0 * std::acos(std::max(-1.0, 1.0 - engDefl / r));
    const int nf = std::max(1, static_cast<int>(std::ceil(kPi / maxStep)));
    const double facetSpacing = r * (kPi / nf);
    const double hausTol = facetSpacing + 0.02 + 1e-6;  // deflection-bounded envelope

    const bool ok = ct.watertight && ct.chi == 2 && volRel < 5e-3 && areaRel < 1e-2 &&
                    volAnalytic < 5e-3 && bboxMax < 5e-2 && haus < hausTol && meshVolRel < 5e-3;
    std::snprintf(detail, sizeof detail,
                  "wt=%d chi=%d vol nat=%.4f occt=%.4f (rel %.2e, analytic %.2e) area rel=%.2e "
                  "bbox=%.2e haus=%.2e (tol %.2e, facet %.3f)",
                  ct.watertight ? 1 : 0, ct.chi, cand.mass.volume, oracle.mass.volume, volRel,
                  volAnalytic, areaRel, bboxMax, haus, hausTol, facetSpacing);
    record(ok, base, detail);

    cc_set_engine(0);
    if (cand.id) cc_shape_release(cand.id);
    if (oracle.id) cc_shape_release(oracle.id);
}

// ── FILLET_FACE: native honestly declines; OCCT owns the reference ───────────────
void runFilletFaceDecline(double a, double r) {
    char lbl[80];
    std::snprintf(lbl, sizeof lbl, "fillet_face decline a=%.0f r=%.1f", a, r);
    const std::string base = lbl;

    // OCCT reference: build + fillet_face the top of a box under engine 0.
    cc_set_engine(0);
    CCShapeId oBody = buildRib(a, a, a);
    const double up[3] = {0, 0, 1};
    const double top[3] = {a / 2, a / 2, a};
    const int oTop = oBody ? pickFace(oBody, up, top, 0.02) : 0;
    const CCShapeId oFillet = oTop ? cc_fillet_face(oBody, oTop, r) : 0;

    // Native: build + fillet_face the top of a box under engine 1 (must DECLINE → 0).
    cc_set_engine(1);
    CCShapeId nBody = buildRib(a, a, a);
    const int nTop = nBody ? pickFace(nBody, up, top, 0.02) : 0;
    const CCShapeId nFillet = nTop ? cc_fillet_face(nBody, nTop, r) : 0;

    char detail[512];
    const bool ok = (oFillet != 0) && (nFillet == 0) && (nTop != 0);
    std::snprintf(detail, sizeof detail,
                  "OCCT ref id=%ld (owns full-face fillet) | native id=%ld (honest decline: "
                  "corner weld gates M2) topPicked=%d",
                  static_cast<long>(oFillet), static_cast<long>(nFillet), nTop);
    record(ok, base, detail);

    cc_set_engine(0);
    if (oFillet) cc_shape_release(oFillet);
    if (oBody) cc_shape_release(oBody);
    if (nFillet) cc_shape_release(nFillet);
    if (nBody) cc_shape_release(nBody);
}

// ── DIHEDRAL full round: non-parallel walls → native declines, OCCT owns ─────────
void runDihedralDecline() {
    const std::string base = "full-round dihedral decline";
    // A trapezoidal-prism rib (non-parallel side walls) — the top strip's two walls
    // meet at an angle → native full round declines (NotParallel), OCCT owns it.
    auto buildTrap = [&](int eng) -> CCShapeId {
        cc_set_engine(eng);
        CCProfileSeg segs[4]{};
        const double xs[4] = {0, 20, 18, 2};
        const double ys[4] = {0, 0, 3, 3};
        for (int i = 0; i < 4; ++i) {
            segs[i].kind = 0;
            segs[i].x0 = xs[i]; segs[i].y0 = ys[i];
            segs[i].x1 = xs[(i + 1) % 4]; segs[i].y1 = ys[(i + 1) % 4];
        }
        return cc_solid_extrude_profile(segs, 4, nullptr, 0, nullptr, 0, 5.0);
    };
    const double up[3] = {0, 0, 1};
    const double top[3] = {10, 1.5, 5};

    cc_set_engine(0);
    CCShapeId oBody = buildTrap(0);
    const int oMid = oBody ? pickFace(oBody, up, top, 0.02) : 0;
    const CCShapeId oRes = oMid ? cc_full_round_fillet(oBody, oMid) : 0;

    cc_set_engine(1);
    CCShapeId nBody = buildTrap(1);
    const int nMid = nBody ? pickFace(nBody, up, top, 0.02) : 0;
    const CCShapeId nRes = nMid ? cc_full_round_fillet(nBody, nMid) : 0;

    char detail[512];
    const bool ok = (nRes == 0) && (nMid != 0);  // native declines the dihedral
    std::snprintf(detail, sizeof detail,
                  "native id=%ld (declines dihedral → M2 valley-solve) OCCT id=%ld midPicked=%d",
                  static_cast<long>(nRes), static_cast<long>(oRes), nMid);
    record(ok, base, detail);

    cc_set_engine(0);
    if (oRes) cc_shape_release(oRes);
    if (oBody) cc_shape_release(oBody);
    if (nRes) cc_shape_release(nRes);
    if (nBody) cc_shape_release(nBody);
}

}  // namespace

int main() {
    std::printf("── native-vs-OCCT ANALYTIC face-fillet parity (MOAT M3) ──\n");
    // Full round on prismatic ribs (lands native, compared to OCCT + analytic).
    runFullRound(20.0, 3.0, 5.0);
    runFullRound(30.0, 4.0, 8.0);
    runFullRound(12.0, 2.0, 6.0);
    // fillet_face: native honestly declines (corner weld gates M2), OCCT owns it.
    runFilletFaceDecline(10.0, 1.5);
    runFilletFaceDecline(20.0, 3.0);
    // Dihedral full round: native declines, OCCT owns.
    runDihedralDecline();

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
