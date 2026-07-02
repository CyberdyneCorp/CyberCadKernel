// GPU-tessellation INTEGRATION suite (OCCT B-rep + Metal surface-eval, run on the
// iOS simulator).
//
// This is the end-to-end wiring test for the GPU tessellation path: it drives the
// PUBLIC C ABI (cc_set_gpu_tessellation / cc_tessellate) exactly as the host app
// would, and reads the per-call GPU-vs-fallback routing tally through the internal
// diagnostics header (engine/occt/gpu_tess_stats.h) to prove which faces actually
// went down the GPU grid path vs the OCCT BRepMesh fallback.
//
// It complements the pure fp32 GPU-vs-CPU parity suite (tests/sim/gpu_suite.mm):
// that one asserts the surface-eval MODULE is numerically correct; THIS one asserts
// the module is correctly STITCHED into cc_tessellate — eligible faces routed to the
// GPU grid, every other face (holes / trims / unsupported surfaces) falling back to
// OCCT, and the merged CCMesh remaining a valid closed display mesh either way.
//
// Two fixtures, both meshed with the GPU path ON and then OFF and cross-checked:
//
//   1. 10x10x10 box — every face is an untrimmed rectangular planar patch, so it is
//      GPU-ELIGIBLE. We assert ALL six faces take the GPU path (fallback count 0),
//      every vertex lands on a box face plane within fp32 tolerance, the bbox is
//      exactly [0,0,0]..[10,10,10], the tri-summed area is 600, and the welded mesh
//      is a closed 2-manifold (every edge shared by exactly two triangles). Because
//      all six GPU face grids share identical uniform edge sampling, welding
//      coincident boundary vertices within tolerance makes the box watertight.
//
//   2. 20x20x10 slab with a round through-hole — the two capped faces carry an inner
//      wire (the hole) and the bore is a cylinder, so those faces MUST fall back to
//      OCCT while the four planar outer walls stay GPU-eligible. We assert the GPU
//      path was used for some faces AND fell back for others (0 < gpuFaces < total),
//      and that the GPU-ON mesh matches the GPU-OFF (pure OCCT) mesh in bbox, area
//      and enclosed volume — proving fallback correctness.
//
// WATERTIGHTNESS / WELDING NOTE (HARD RULE): the GPU face-eval path appends its grid
// as an independent vertex block and does NOT weld against the OCCT triangulation of
// its neighbours. For the all-GPU box every boundary is GPU-vs-GPU with matching
// uniform sampling, so a tolerance weld recovers a strict closed 2-manifold and we
// assert exactly that. For the MIXED (GPU + OCCT) hole slab the GPU grid edges and
// the OCCT edge polygons do not share nodes, so the merged mesh has T-junctions and
// is deliberately NOT asserted to be edge-2-manifold. Instead we assert it is closed
// by COVERAGE: its enclosed (divergence-theorem) volume equals that of the pure-OCCT
// watertight mesh within tolerance, i.e. the surface is fully tiled with no gaps. The
// pure-OCCT mesh (built by one conformal whole-body BRepMesh pass) IS asserted to be
// a strict welded 2-manifold.
//
// fp32: the GPU grid is a display mesh in single precision; tolerances below are
// sized for fp32 sampling, never for the exact fp64 modeling core (untouched here).
//
// Output: one "[ITEG] PASS/FAIL — ..." line per assertion, a routing summary, then a
// final tally. We flush and std::_Exit so the intentionally-leaked engine / shape
// registry are never torn down (their destructors race OCCT static teardown).

#include "cybercadkernel/cc_kernel.h"
#include "engine/occt/gpu_tess_stats.h"  // cyber::occt::gpuTessStats (routing tally)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

// ── tolerances (fp32 display mesh) ────────────────────────────────────────────
constexpr double kBBoxTol = 1e-2;   // mm — vertex bbox vs analytic / vs OCCT
constexpr double kAreaTol = 1e-1;   // mm^2 — planar (box) tri-summed area parity
constexpr double kVolTol = 5e-1;    // mm^3 — planar (box) enclosed volume parity
constexpr double kOnFaceTol = 1e-2; // mm — distance a box vertex may sit off a plane
constexpr double kWeldTol = 1e-3;   // mm — coincident-vertex weld radius (<< spacing)

// The hole slab has ONE curved face (the cylindrical bore). It is either GPU-eligible
// (an untrimmed rectangular periodic patch) or falls back to OCCT; either way the
// planar walls are exact and only that bore can differ. When it IS on the GPU it is
// sampled at a slightly different density than OCCT's BRepMesh, so its faceted area /
// removed volume differ by a small, deflection-bounded amount. These parity tolerances
// absorb that legitimate faceting gap (well under 1% here) while still catching any
// gross stitching error (which would diverge by tens–hundreds). bbox is unaffected by
// faceting (set by the exact outer corners), so it stays at the tight kBBoxTol.
constexpr double kSlabAreaTol = 5.0;  // mm^2 — mixed-mesh area parity (curved faceting)
constexpr double kSlabVolTol = 10.0;  // mm^3 — mixed-mesh volume parity (curved faceting)

// ── result reporting ──────────────────────────────────────────────────────────
struct Report {
    int passed = 0;
    int failed = 0;

    bool check(bool ok, const std::string& what, const std::string& detail = "") {
        std::printf("[ITEG] %s — %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(),
                    detail.empty() ? "" : ": ", detail.c_str());
        if (ok) {
            ++passed;
        } else {
            ++failed;
        }
        std::fflush(stdout);
        return ok;
    }
};

inline bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ── mesh metrics ──────────────────────────────────────────────────────────────

// Enclosed volume of a closed, outward-oriented triangle soup via the divergence
// theorem (Σ v0·(v1×v2)/6). Correct for a fully-tiled surface even in the presence
// of T-junctions, so it doubles as a "closed by coverage" watertightness witness.
double enclosedVolume(const CCMesh& m) {
    double v6 = 0.0;
    for (int t = 0; t < m.triangleCount; ++t) {
        const int ia = m.triangles[t * 3], ib = m.triangles[t * 3 + 1], ic = m.triangles[t * 3 + 2];
        const double* a = &m.vertices[ia * 3];
        const double* b = &m.vertices[ib * 3];
        const double* c = &m.vertices[ic * 3];
        const double cx = b[1] * c[2] - b[2] * c[1];
        const double cy = b[2] * c[0] - b[0] * c[2];
        const double cz = b[0] * c[1] - b[1] * c[0];
        v6 += a[0] * cx + a[1] * cy + a[2] * cz;
    }
    return v6 / 6.0;
}

// Tri-summed surface area (0.5·|(v1-v0)×(v2-v0)|).
double surfaceArea(const CCMesh& m) {
    double area = 0.0;
    for (int t = 0; t < m.triangleCount; ++t) {
        const int ia = m.triangles[t * 3], ib = m.triangles[t * 3 + 1], ic = m.triangles[t * 3 + 2];
        const double* a = &m.vertices[ia * 3];
        const double* b = &m.vertices[ib * 3];
        const double* c = &m.vertices[ic * 3];
        const double e1x = b[0] - a[0], e1y = b[1] - a[1], e1z = b[2] - a[2];
        const double e2x = c[0] - a[0], e2y = c[1] - a[1], e2z = c[2] - a[2];
        const double cx = e1y * e2z - e1z * e2y;
        const double cy = e1z * e2x - e1x * e2z;
        const double cz = e1x * e2y - e1y * e2x;
        area += 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
    }
    return area;
}

// Axis-aligned vertex bounds -> out6 = [minX,minY,minZ,maxX,maxY,maxZ].
void meshBBox(const CCMesh& m, double out6[6]) {
    out6[0] = out6[1] = out6[2] = 1e300;
    out6[3] = out6[4] = out6[5] = -1e300;
    for (int i = 0; i < m.vertexCount; ++i) {
        for (int d = 0; d < 3; ++d) {
            const double x = m.vertices[i * 3 + d];
            out6[d] = std::min(out6[d], x);
            out6[3 + d] = std::max(out6[3 + d], x);
        }
    }
}

bool bboxNear(const double a[6], const double b[6], double tol) {
    for (int i = 0; i < 6; ++i) {
        if (!near(a[i], b[i], tol)) {
            return false;
        }
    }
    return true;
}

// ── tolerance weld + strict 2-manifold watertightness ─────────────────────────
//
// A uniform hash grid (cell = kWeldTol) that maps each raw vertex onto a welded id,
// merging any two vertices within kWeldTol. The 27-neighbour lookup avoids the
// cell-boundary split a bare quantiser would suffer; kWeldTol (1e-3 mm) is far below
// the real inter-sample spacing of every fixture here, so no distinct vertices are
// ever merged and every genuinely-coincident boundary vertex is.
class WeldGrid {
public:
    int insert(double x, double y, double z) {
        const int64_t cx = cell(x), cy = cell(y), cz = cell(z);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto it = cells_.find(key(cx + dx, cy + dy, cz + dz));
                    if (it == cells_.end()) {
                        continue;
                    }
                    for (int id : it->second) {
                        if (near(px_[id], x, kWeldTol) && near(py_[id], y, kWeldTol) &&
                            near(pz_[id], z, kWeldTol)) {
                            return id;
                        }
                    }
                }
            }
        }
        const int id = static_cast<int>(px_.size());
        px_.push_back(x);
        py_.push_back(y);
        pz_.push_back(z);
        cells_[key(cx, cy, cz)].push_back(id);
        return id;
    }

private:
    static int64_t cell(double v) { return static_cast<int64_t>(std::floor(v / kWeldTol)); }
    static uint64_t key(int64_t a, int64_t b, int64_t c) {
        uint64_t h = 1469598103934665603ull;
        for (int64_t v : {a, b, c}) {
            h ^= static_cast<uint64_t>(v);
            h *= 1099511628211ull;
        }
        return h;
    }
    std::vector<double> px_, py_, pz_;
    std::unordered_map<uint64_t, std::vector<int>> cells_;
};

// True iff, after welding coincident vertices within kWeldTol, every undirected edge
// is shared by EXACTLY two triangles — a closed 2-manifold (watertight) mesh.
bool isWeldedManifold(const CCMesh& m) {
    if (m.vertexCount <= 0 || m.triangleCount <= 0) {
        return false;
    }
    WeldGrid grid;
    std::vector<int> weld(static_cast<std::size_t>(m.vertexCount));
    for (int i = 0; i < m.vertexCount; ++i) {
        weld[static_cast<std::size_t>(i)] =
            grid.insert(m.vertices[i * 3], m.vertices[i * 3 + 1], m.vertices[i * 3 + 2]);
    }
    std::unordered_map<uint64_t, int> edges;
    const auto bump = [&](int a, int b) {
        if (a == b) {
            return;  // degenerate edge after weld — ignore
        }
        if (a > b) {
            std::swap(a, b);
        }
        const uint64_t k = (static_cast<uint64_t>(a) << 32) | static_cast<uint32_t>(b);
        ++edges[k];
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int a = weld[static_cast<std::size_t>(m.triangles[t * 3])];
        const int b = weld[static_cast<std::size_t>(m.triangles[t * 3 + 1])];
        const int c = weld[static_cast<std::size_t>(m.triangles[t * 3 + 2])];
        bump(a, b);
        bump(b, c);
        bump(c, a);
    }
    for (const auto& [k, count] : edges) {
        (void)k;
        if (count != 2) {
            return false;
        }
    }
    return !edges.empty();
}

// Every vertex sits on one of the six planes of the [0,lo]..[hi] box (one coordinate
// pinned to 0 or `side` within kOnFaceTol) and stays inside the slab on the others.
bool verticesOnBoxFaces(const CCMesh& m, double side) {
    for (int i = 0; i < m.vertexCount; ++i) {
        const double x = m.vertices[i * 3];
        const double y = m.vertices[i * 3 + 1];
        const double z = m.vertices[i * 3 + 2];
        const auto onPlane = [&](double v) {
            return near(v, 0.0, kOnFaceTol) || near(v, side, kOnFaceTol);
        };
        const auto inSlab = [&](double v) {
            return v >= -kOnFaceTol && v <= side + kOnFaceTol;
        };
        if (!inSlab(x) || !inSlab(y) || !inSlab(z)) {
            return false;
        }
        if (!(onPlane(x) || onPlane(y) || onPlane(z))) {
            return false;
        }
    }
    return true;
}

// ── fixtures ──────────────────────────────────────────────────────────────────

// cc_tessellate + snapshot the GPU/OCCT routing tally recorded for that call.
struct Tessellation {
    CCMesh mesh;
    cyber::occt::GpuTessStats stats;
};

Tessellation tessellate(CCShapeId body, double deflection) {
    Tessellation out;
    out.mesh = cc_tessellate(body, deflection);
    out.stats = cyber::occt::gpuTessStats();
    return out;
}

// ── test 1: GPU-eligible box (all faces on the GPU grid path) ─────────────────
void runBoxTest(Report& r) {
    std::printf("[ITEG] --- fixture: 10x10x10 box (all faces GPU-eligible) ---\n");
    const double square[8] = {0, 0, 10, 0, 10, 10, 0, 10};  // 10x10 CCW profile
    CCShapeId box = cc_solid_extrude(square, 4, 10.0);
    if (!r.check(box != 0, "cc_solid_extrude -> 10x10x10 box")) {
        return;
    }

    // GPU path ON — must be reported enabled on a Metal build.
    cc_set_gpu_tessellation(1);
    r.check(cc_gpu_tessellation_enabled() == 1, "cc_gpu_tessellation_enabled() == 1 (GPU on)");

    Tessellation g = tessellate(box, 0.1);
    const int gpuTotal = g.stats.gpuFaces + g.stats.fallbackFaces;
    r.check(g.mesh.vertexCount > 0 && g.mesh.triangleCount > 0 && g.mesh.vertices && g.mesh.triangles,
            "GPU box mesh non-empty",
            "v=" + std::to_string(g.mesh.vertexCount) + " t=" + std::to_string(g.mesh.triangleCount));

    // Every one of the six faces took the GPU grid path; nothing fell back.
    r.check(gpuTotal == 6 && g.stats.gpuFaces == 6 && g.stats.fallbackFaces == 0,
            "box routed ALL 6 faces to GPU, 0 fallback",
            "gpu=" + std::to_string(g.stats.gpuFaces) +
                " fallback=" + std::to_string(g.stats.fallbackFaces));

    r.check(verticesOnBoxFaces(g.mesh, 10.0), "every GPU vertex lies on a box face plane (fp32 tol)");

    double gbb[6];
    meshBBox(g.mesh, gbb);
    const double wantBB[6] = {0, 0, 0, 10, 10, 10};
    r.check(bboxNear(gbb, wantBB, kBBoxTol), "GPU box bbox == [0,0,0]..[10,10,10]");

    const double gArea = surfaceArea(g.mesh);
    r.check(near(gArea, 600.0, kAreaTol), "GPU box area == 600", "got " + std::to_string(gArea));

    const double gVol = enclosedVolume(g.mesh);
    r.check(near(gVol, 1000.0, kVolTol), "GPU box enclosed volume == 1000 (closed)",
            "got " + std::to_string(gVol));

    // All-GPU boundaries share uniform sampling → a tolerance weld yields a strict
    // closed 2-manifold.
    r.check(isWeldedManifold(g.mesh), "GPU box is watertight (welded 2-manifold, every edge x2 tris)");

    // GPU path OFF — the reference OCCT-only mesh.
    cc_set_gpu_tessellation(0);
    r.check(cc_gpu_tessellation_enabled() == 0, "cc_gpu_tessellation_enabled() == 0 (GPU off)");
    Tessellation o = tessellate(box, 0.1);
    r.check(o.stats.gpuFaces == 0, "OCCT box used 0 GPU faces",
            "gpu=" + std::to_string(o.stats.gpuFaces));

    double obb[6];
    meshBBox(o.mesh, obb);
    const double oArea = surfaceArea(o.mesh);
    r.check(isWeldedManifold(o.mesh), "OCCT box is watertight (welded 2-manifold)");
    r.check(bboxNear(gbb, obb, kBBoxTol), "GPU vs OCCT box bbox match");
    r.check(near(gArea, oArea, kAreaTol), "GPU vs OCCT box area match",
            "gpu=" + std::to_string(gArea) + " occt=" + std::to_string(oArea));

    std::printf("[ITEG] routing summary — box: gpu=%d fallback=%d (expect 6/0)\n",
                g.stats.gpuFaces, g.stats.fallbackFaces);

    cc_mesh_free(g.mesh);
    cc_mesh_free(o.mesh);
    cc_shape_release(box);
}

// ── test 2: hole slab (holes + cylinder force OCCT fallback) ──────────────────
void runFallbackTest(Report& r) {
    std::printf("[ITEG] --- fixture: 20x20x10 slab with round hole (mixed GPU + OCCT) ---\n");
    const double outer[8] = {0, 0, 20, 0, 20, 20, 0, 20};  // 20x20 outer, CCW
    const double hole[3] = {10, 10, 3};                    // centre (10,10), r=3
    CCShapeId slab = cc_solid_extrude_holes(outer, 4, hole, 1, 10.0);
    if (!r.check(slab != 0, "cc_solid_extrude_holes -> 20x20x10 slab with bore")) {
        return;
    }

    // GPU path OFF first: the pure-OCCT reference (a single conformal whole-body
    // mesh → a strict welded 2-manifold).
    cc_set_gpu_tessellation(0);
    Tessellation o = tessellate(slab, 0.1);
    r.check(o.mesh.vertexCount > 0 && o.mesh.triangleCount > 0, "OCCT slab mesh non-empty",
            "v=" + std::to_string(o.mesh.vertexCount) + " t=" + std::to_string(o.mesh.triangleCount));
    r.check(o.stats.gpuFaces == 0, "OCCT slab used 0 GPU faces");
    r.check(isWeldedManifold(o.mesh), "OCCT slab is watertight (welded 2-manifold)");
    double obb[6];
    meshBBox(o.mesh, obb);
    const double oArea = surfaceArea(o.mesh);
    const double oVol = enclosedVolume(o.mesh);

    // GPU path ON: planar outer walls go to the GPU grid, the two holed caps and the
    // cylindrical bore MUST fall back to OCCT.
    cc_set_gpu_tessellation(1);
    Tessellation g = tessellate(slab, 0.1);
    const int gpuTotal = g.stats.gpuFaces + g.stats.fallbackFaces;
    r.check(g.mesh.vertexCount > 0 && g.mesh.triangleCount > 0, "GPU slab mesh non-empty",
            "v=" + std::to_string(g.mesh.vertexCount) + " t=" + std::to_string(g.mesh.triangleCount));

    // Fallback correctness: some faces GPU, some OCCT, never all-GPU.
    r.check(g.stats.fallbackFaces > 0, "holed/curved faces fell back to OCCT",
            "fallback=" + std::to_string(g.stats.fallbackFaces));
    r.check(g.stats.gpuFaces > 0, "planar outer walls used the GPU path",
            "gpu=" + std::to_string(g.stats.gpuFaces));
    r.check(g.stats.gpuFaces < gpuTotal, "GPU face count < total face count (mixed)",
            "gpu=" + std::to_string(g.stats.gpuFaces) + " total=" + std::to_string(gpuTotal));

    double gbb[6];
    meshBBox(g.mesh, gbb);
    const double gArea = surfaceArea(g.mesh);
    const double gVol = enclosedVolume(g.mesh);

    // Parity: the mixed GPU/OCCT mesh matches the pure-OCCT mesh.
    r.check(bboxNear(gbb, obb, kBBoxTol), "GPU vs OCCT slab bbox match");
    r.check(near(gArea, oArea, kSlabAreaTol), "GPU vs OCCT slab area match",
            "gpu=" + std::to_string(gArea) + " occt=" + std::to_string(oArea));
    r.check(near(gVol, oVol, kSlabVolTol), "GPU vs OCCT slab enclosed volume match",
            "gpu=" + std::to_string(gVol) + " occt=" + std::to_string(oVol));

    // Closed by COVERAGE: the mixed mesh encloses the same volume as the pure-OCCT
    // watertight mesh, so its surface is fully tiled (no gaps). It is intentionally
    // NOT asserted to be edge-2-manifold — GPU grid edges and OCCT edge polygons are
    // not vertex-welded at their shared boundary (documented in the file header).
    r.check(near(gVol, oVol, kSlabVolTol),
            "GPU slab watertight by coverage (volume == OCCT watertight mesh)");

    cc_mesh_free(g.mesh);
    cc_mesh_free(o.mesh);
    cc_shape_release(slab);

    std::printf("[ITEG] routing summary — slab: gpu=%d fallback=%d total=%d (expect fallback>0, gpu<total)\n",
                g.stats.gpuFaces, g.stats.fallbackFaces, gpuTotal);
}

}  // namespace

int main() {
    std::printf("[ITEG] GPU-tessellation integration suite (OCCT + Metal, iOS simulator)\n");
    std::printf("[ITEG] brep_available=%d\n", cc_brep_available());
    std::fflush(stdout);

    Report r;
    runBoxTest(r);
    runFallbackTest(r);

    std::printf("[ITEG] == %d passed, %d failed ==\n", r.passed, r.failed);
    std::fflush(stdout);

    // Bypass static destruction: the engine + shape registry are intentionally
    // leaked (their teardown races OCCT's static teardown). Exit hard.
    std::_Exit(r.failed == 0 ? 0 : 1);
}
