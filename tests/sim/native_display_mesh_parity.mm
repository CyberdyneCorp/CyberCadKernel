// SPDX-License-Identifier: Apache-2.0
//
// native_display_mesh_parity.mm — Gate (b) for the render-quality DISPLAY mesh
// (src/native/render/display_mesh.h via the cc_display_mesh facade), iOS sim.
//
// The display mesh is a PURELY ADDITIVE post-process of the correctness
// tessellation (cc_tessellate). This harness drives cc_display_mesh THROUGH THE
// ENGINE on real bodies built via the cc_* facade (a box via cc_solid_extrude, a
// cylinder via cc_solid_extrude_profile true-circle) and re-asserts the SAME
// closed-form-normal oracle used by the host Gate (a) test — a BETTER oracle than
// OCCT for shading:
//   * cylinder lateral wall → smooth normals EXACTLY radial (⊥ axis) to ~1e-6;
//   * cylinder cap↔wall ring → a crease: the ring SPLITS (axial + radial copies);
//   * box → every edge a crease: 24 split corner normals, 6 axis-aligned faces;
// plus the cross-checks the task calls for:
//   * every display POSITION lies on the source solid (== a cc_tessellate vertex);
//   * the base (no-LOD) display mesh folds back (by position) to a WATERTIGHT mesh
//     with the SAME triangle count as cc_tessellate — i.e. it is display-consistent
//     with the correctness mesh;
//   * cc_tessellate is UNCHANGED (byte-identical hash) whether or not
//     cc_display_mesh is called — the byte-frozen-tessellator proof at the ABI.
//   * LOD reduces the triangle count and every survivor stays within the Hausdorff
//     budget of the source solid.
//   * HONEST DECLINE: cc_display_mesh(0,…) → 0 with the out zeroed.
//
// This file has its OWN main() + runner. It is NOT in the CMake CTest list and is
// on the SKIP list of the default host build (it links the cc_* facade over the
// active engine and is compiled ONLY by a dedicated sim script, mirroring the
// other tests/sim/native_*_parity.mm harnesses). SKIP.
//
#include "cybercadkernel/cc_kernel.h"

#include <algorithm>   // std::max — libstdc++ does not provide it transitively
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

int gPass = 0, gFail = 0;
bool check(bool ok, const char* what, const std::string& detail = "") {
  std::printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", what, detail.empty() ? "" : " — ",
              detail.c_str());
  if (ok) ++gPass; else ++gFail;
  std::fflush(stdout);
  return ok;
}

// Exact-quantised position key for folding split display verts back by position.
struct PosKey {
  std::int64_t x, y, z;
  bool operator==(const PosKey& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct PosKeyHash {
  std::size_t operator()(const PosKey& k) const {
    std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
    h ^= static_cast<std::size_t>(k.y) * 19349663u + (h << 6) + (h >> 2);
    h ^= static_cast<std::size_t>(k.z) * 83492791u + (h << 6) + (h >> 2);
    return h;
  }
};
PosKey posKey(double x, double y, double z) {
  auto q = [](double v) { return static_cast<std::int64_t>(std::llround(v * 1e6)); };
  return {q(x), q(y), q(z)};
}

// FNV-1a over a CCMesh's buffers — determinism / byte-identity check.
std::uint64_t hashMesh(const CCMesh& m) {
  std::uint64_t h = 1469598103934665603ull;
  auto mix = [&](const void* p, std::size_t n) {
    const unsigned char* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
  };
  mix(&m.vertexCount, sizeof m.vertexCount);
  mix(&m.triangleCount, sizeof m.triangleCount);
  if (m.vertices) mix(m.vertices, sizeof(double) * static_cast<std::size_t>(m.vertexCount) * 3);
  if (m.triangles) mix(m.triangles, sizeof(int) * static_cast<std::size_t>(m.triangleCount) * 3);
  return h;
}

// Is the folded display mesh watertight (every undirected edge used exactly twice)?
bool foldedWatertight(const CCDisplayMesh& dm, int& foldedTris) {
  std::unordered_map<PosKey, int, PosKeyHash> posToIdx;
  auto idxOf = [&](int v) {
    const PosKey k = posKey(dm.positions[3 * v], dm.positions[3 * v + 1], dm.positions[3 * v + 2]);
    auto it = posToIdx.find(k);
    if (it != posToIdx.end()) return it->second;
    const int n = static_cast<int>(posToIdx.size());
    posToIdx.emplace(k, n);
    return n;
  };
  std::unordered_map<std::uint64_t, int> edge;
  auto bump = [&](int a, int b) {
    const int lo = a < b ? a : b, hi = a < b ? b : a;
    ++edge[(static_cast<std::uint64_t>(lo) << 32) ^ static_cast<std::uint64_t>(hi)];
  };
  foldedTris = 0;
  for (int t = 0; t < dm.triangleCount; ++t) {
    const int a = idxOf(dm.triangles[3 * t]);
    const int b = idxOf(dm.triangles[3 * t + 1]);
    const int c = idxOf(dm.triangles[3 * t + 2]);
    bump(a, b); bump(b, c); bump(c, a);
    ++foldedTris;
  }
  for (const auto& [k, u] : edge)
    if (u != 2) return false;
  return true;
}

const double kSquare10[8] = {0, 0, 10, 0, 10, 10, 0, 10};  // 10×10 CCW

// ── Cylinder wall: smooth normals are radial ─────────────────────────────────
void check_cylinder(void) {
  CCProfileSeg seg{};
  seg.kind = 2;                 // true circle
  seg.cx = 0; seg.cy = 0; seg.r = 5.0;
  const double h = 12.0;
  CCShapeId cyl = cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, h);
  if (!check(cyl != 0, "cc_solid_extrude_profile(circle) -> cylinder id")) return;

  // Byte-frozen tessellator proof: hash cc_tessellate before AND after a
  // cc_display_mesh call — they must be identical.
  const double defl = 0.05;
  CCMesh t0 = cc_tessellate(cyl, defl);
  const std::uint64_t hBefore = hashMesh(t0);

  CCDisplayMesh dm{};
  const int nT = cc_display_mesh(cyl, defl, 30.0, 0, 0, &dm);
  check(nT > 0 && dm.triangleCount == nT && dm.positions && dm.normals,
        "cc_display_mesh(cylinder) -> populated display mesh",
        "tris " + std::to_string(nT));

  CCMesh t1 = cc_tessellate(cyl, defl);
  check(hashMesh(t1) == hBefore, "cc_tessellate byte-identical across cc_display_mesh call");
  cc_mesh_free(t0); cc_mesh_free(t1);

  // Wall copy of each ring vertex (|nz|<0.1) is EXACTLY radial.
  double worst = 0.0; int wall = 0; bool outward = true;
  for (int i = 0; i < dm.vertexCount; ++i) {
    const double x = dm.positions[3 * i], y = dm.positions[3 * i + 1];
    const double nx = dm.normals[3 * i], ny = dm.normals[3 * i + 1], nz = dm.normals[3 * i + 2];
    const double r = std::sqrt(x * x + y * y);
    if (std::fabs(r - seg.r) < 5 * defl && std::fabs(nz) < 0.1) {
      ++wall;
      const double e = std::hypot(nx - x / r, ny - y / r);
      worst = std::max(worst, e);
      if (nx * x + ny * y <= 0.0) outward = false;
    }
  }
  check(wall > 8, "cylinder wall-copy verts found", "n=" + std::to_string(wall));
  check(worst < 1e-6, "cylinder wall smooth normals radial to ~1e-6",
        "max |n-radial|=" + std::to_string(worst));
  check(outward, "cylinder wall normals all outward");

  // Cap↔wall ring point splits (axial + radial copies at (r,0,0)-ish).
  bool axial = false, radial = false;
  for (int i = 0; i < dm.vertexCount; ++i) {
    const double x = dm.positions[3 * i], y = dm.positions[3 * i + 1], z = dm.positions[3 * i + 2];
    const double r = std::sqrt(x * x + y * y);
    if (std::fabs(r - seg.r) < 5 * defl && std::fabs(z) < 5 * defl) {  // bottom ring band
      const double nz = std::fabs(dm.normals[3 * i + 2]);
      if (nz > 0.9) axial = true;
      if (nz < 0.1) radial = true;
    }
  }
  check(axial && radial, "cylinder cap↔wall ring SPLITS (axial + radial normals)");

  // Base display positions lie on the source solid (== a cc_tessellate vertex).
  CCMesh tess = cc_tessellate(cyl, defl);
  std::unordered_map<PosKey, int, PosKeyHash> src;
  for (int i = 0; i < tess.vertexCount; ++i)
    src[posKey(tess.vertices[3 * i], tess.vertices[3 * i + 1], tess.vertices[3 * i + 2])]++;
  bool onSrc = true;
  for (int i = 0; i < dm.vertexCount; ++i)
    if (src.find(posKey(dm.positions[3 * i], dm.positions[3 * i + 1], dm.positions[3 * i + 2])) ==
        src.end()) { onSrc = false; break; }
  check(onSrc, "every base display position is a cc_tessellate vertex (on the solid)");

  // Folds back to a watertight mesh with the SAME triangle count as cc_tessellate.
  int foldedTris = 0;
  const bool wt = foldedWatertight(dm, foldedTris);
  check(wt, "base display mesh folds to a WATERTIGHT surface");
  check(foldedTris == tess.triangleCount,
        "display triangle count == cc_tessellate triangle count",
        "display " + std::to_string(foldedTris) + " vs tess " + std::to_string(tess.triangleCount));
  cc_mesh_free(tess);
  cc_display_mesh_free(&dm);
  cc_shape_release(cyl);
}

// ── Box: every edge a crease → 24 corner splits + 6 face normals ─────────────
void check_box(void) {
  CCShapeId box = cc_solid_extrude(kSquare10, 4, 10.0);
  if (!check(box != 0, "cc_solid_extrude -> 10×10×10 box id")) return;

  CCDisplayMesh dm{};
  const int nT = cc_display_mesh(box, 1.0, 30.0, 0, 1 /*wantUVs*/, &dm);
  check(nT > 0 && dm.uvs != nullptr, "cc_display_mesh(box, wantUVs) -> mesh with UVs");

  // Every normal axis-aligned; count distinct.
  int axisAligned = 0;
  std::vector<std::array<double, 3>> distinct;
  for (int i = 0; i < dm.vertexCount; ++i) {
    const double nx = dm.normals[3 * i], ny = dm.normals[3 * i + 1], nz = dm.normals[3 * i + 2];
    if (std::max({std::fabs(nx), std::fabs(ny), std::fabs(nz)}) > 1.0 - 1e-9) ++axisAligned;
    bool seen = false;
    for (const auto& d : distinct)
      if (std::hypot(std::hypot(d[0] - nx, d[1] - ny), d[2] - nz) < 1e-6) { seen = true; break; }
    if (!seen) distinct.push_back({nx, ny, nz});
  }
  check(axisAligned == dm.vertexCount, "box: every display normal axis-aligned");
  check(static_cast<int>(distinct.size()) == 6, "box: 6 distinct face normals",
        "got " + std::to_string(distinct.size()));

  // 8 corners × 3 copies = 24.
  const double corners[8][3] = {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0},
                                {0, 0, 10}, {10, 0, 10}, {10, 10, 10}, {0, 10, 10}};
  int totalCopies = 0;
  bool eachThree = true;
  for (auto& cor : corners) {
    int copies = 0;
    for (int i = 0; i < dm.vertexCount; ++i)
      if (std::fabs(dm.positions[3 * i] - cor[0]) < 1e-6 &&
          std::fabs(dm.positions[3 * i + 1] - cor[1]) < 1e-6 &&
          std::fabs(dm.positions[3 * i + 2] - cor[2]) < 1e-6) ++copies;
    if (copies != 3) eachThree = false;
    totalCopies += copies;
  }
  check(eachThree && totalCopies == 24, "box: 24 split corner normals (3 per corner)",
        "total " + std::to_string(totalCopies));

  // UVs in [0,1].
  bool uvInRange = true;
  for (int i = 0; i < dm.vertexCount; ++i)
    if (dm.uvs[2 * i] < 0 || dm.uvs[2 * i] > 1 || dm.uvs[2 * i + 1] < 0 || dm.uvs[2 * i + 1] > 1)
      uvInRange = false;
  check(uvInRange, "box: UVs in [0,1]");
  cc_display_mesh_free(&dm);
  cc_shape_release(box);
}

// ── LOD on a cylinder: reduce tri count within the Hausdorff budget ──────────
void check_lod(void) {
  CCProfileSeg seg{};
  seg.kind = 2; seg.cx = 0; seg.cy = 0; seg.r = 5.0;
  CCShapeId cyl = cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, 12.0);
  if (!check(cyl != 0, "cc_solid_extrude_profile(circle) -> cylinder (LOD fixture)")) return;

  const double defl = 0.02;
  CCMesh full = cc_tessellate(cyl, defl);
  const int fullTris = full.triangleCount;
  cc_mesh_free(full);

  CCDisplayMesh dm{};
  const int target = fullTris / 2;
  const int nT = cc_display_mesh(cyl, defl, 30.0, target, 0, &dm);
  check(nT > 0 && nT < fullTris, "LOD reduces triangle count",
        "full " + std::to_string(fullTris) + " -> " + std::to_string(nT) + " (target " +
            std::to_string(target) + ")");

  // Every survivor within budget = 8·defl of the cylinder wall or a cap plane.
  const double budget = 8.0 * defl;
  double worst = 0.0;
  for (int i = 0; i < dm.vertexCount; ++i) {
    const double x = dm.positions[3 * i], y = dm.positions[3 * i + 1], z = dm.positions[3 * i + 2];
    const double r = std::sqrt(x * x + y * y);
    double dWall = (z >= -budget && z <= 12.0 + budget) ? std::fabs(r - seg.r) : 1e9;
    double dCap = (r <= seg.r + budget) ? std::min(std::fabs(z), std::fabs(z - 12.0)) : 1e9;
    worst = std::max(worst, std::min(dWall, dCap));
  }
  check(worst <= budget + 1e-9, "LOD survivors within Hausdorff budget",
        "max " + std::to_string(worst) + " budget " + std::to_string(budget));
  cc_display_mesh_free(&dm);
  cc_shape_release(cyl);
}

// ── Honest decline ────────────────────────────────────────────────────────────
void check_decline(void) {
  CCDisplayMesh dm{};
  const int nT = cc_display_mesh(0 /*invalid id*/, 0.1, 30.0, 0, 0, &dm);
  check(nT == 0 && dm.positions == nullptr && dm.triangleCount == 0,
        "cc_display_mesh(invalid id) -> 0 with out zeroed (HONEST DECLINE)");
}

}  // namespace

int main() {
  std::printf("== cc_display_mesh Gate (b) parity — engine-served display mesh ==\n");
  std::fflush(stdout);
  check(cc_brep_available() == 1, "cc_brep_available()==1 (engine linked)");

  check_cylinder();
  check_box();
  check_lod();
  check_decline();

  std::printf("== %d passed, %d failed ==\n", gPass, gFail);
  std::fflush(stdout);
  return gFail == 0 ? 0 : 1;
}
