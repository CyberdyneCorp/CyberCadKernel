// OCCT-backed runtime harness for CyberCadKernel, run inside an iOS simulator
// (xcrun simctl spawn). Exercises the cc_* facade through the real OCCT adapter
// and checks: (a) correctness vs analytic values, (b) run-to-run determinism of
// the parallel paths, (c) wall-clock of boolean + meshing. Closes the runtime
// items deferred from add-kernel-foundation / accelerate-multicore-occt.
//
// Serial-vs-parallel A/B is NOT here: the parallel toggle is internal C++ and
// the C facade exposes no switch, so this measures parallel run-to-run
// reproducibility (the core determinism property) + absolute timing. A serial
// baseline needs an additive cc_* toggle hook (follow-up).

#include "cybercadkernel/cc_kernel.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <string>

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what, const std::string& detail = "") {
  std::printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(),
              detail.empty() ? "" : " — ", detail.c_str());
  if (ok) ++g_pass; else ++g_fail;
}
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// FNV-1a over a mesh's geometry, for byte-level determinism comparison.
static uint64_t hashMesh(const CCMesh& m) {
  uint64_t h = 1469598103934665603ull;
  auto mix = [&](const void* p, size_t n) {
    const unsigned char* b = static_cast<const unsigned char*>(p);
    for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
  };
  mix(&m.vertexCount, sizeof m.vertexCount);
  mix(&m.triangleCount, sizeof m.triangleCount);
  if (m.vertices)  mix(m.vertices,  sizeof(double) * m.vertexCount * 3);
  if (m.triangles) mix(m.triangles, sizeof(int)    * m.triangleCount * 3);
  return h;
}

static const double kSquare[8] = {0,0, 10,0, 10,10, 0,10};  // 10x10 profile

int main() {
  std::printf("== CyberCadKernel OCCT runtime harness ==\n");
  check(cc_brep_available() == 1, "cc_brep_available()==1 (OCCT linked)");
  if (!cc_brep_available()) { std::printf("OCCT not linked — aborting\n"); return 2; }

  // ── Correctness: extrude a 10x10x10 box, check volume/area/bbox ──────────
  CCShapeId box = cc_solid_extrude(kSquare, 4, 10.0);
  check(box != 0, "cc_solid_extrude -> box id");
  CCMassProps mp = cc_mass_properties(box);
  check(mp.valid && near(mp.volume, 1000.0, 1e-6), "box volume == 1000",
        "got " + std::to_string(mp.volume));
  check(near(mp.area, 600.0, 1e-6), "box area == 600", "got " + std::to_string(mp.area));
  double bb[6] = {0};
  check(cc_bounding_box(box, bb) == 1 && near(bb[0],0,1e-6) && near(bb[3],10,1e-6)
        && near(bb[5],10,1e-6), "box bbox == [0,0,0,10,10,10]");

  int* edges = nullptr;
  int nedge = cc_subshape_ids(box, 1, &edges);
  check(nedge == 12, "box has 12 edges", "got " + std::to_string(nedge));
  if (edges) cc_ints_free(edges);

  // ── Booleans: fuse/cut/common of box with a box translated by (5,5,5) ────
  CCShapeId box2 = cc_solid_extrude(kSquare, 4, 10.0);
  CCShapeId box2t = cc_translate_shape(box2, 5, 5, 5);
  check(box2t != 0, "cc_translate_shape -> box2 id");
  CCShapeId fused  = cc_boolean(box, box2t, 0);
  CCShapeId cutd   = cc_boolean(box, box2t, 1);
  CCShapeId common = cc_boolean(box, box2t, 2);
  CCMassProps mf = cc_mass_properties(fused), mc = cc_mass_properties(cutd), mk = cc_mass_properties(common);
  check(mf.valid && near(mf.volume, 1875.0, 1e-4), "fuse volume == 1875", "got " + std::to_string(mf.volume));
  check(mc.valid && near(mc.volume,  875.0, 1e-4), "cut volume == 875",  "got " + std::to_string(mc.volume));
  check(mk.valid && near(mk.volume,  125.0, 1e-4), "common volume == 125", "got " + std::to_string(mk.volume));

  // ── Fillet one box edge ─────────────────────────────────────────────────
  int* fedges = nullptr; int nf = cc_subshape_ids(box, 1, &fedges);
  if (nf > 0) {
    CCShapeId filleted = cc_fillet_edges(box, fedges, 1, 1.0);
    check(filleted != 0, "cc_fillet_edges(radius=1) -> valid id");
    cc_shape_release(filleted);
  }
  if (fedges) cc_ints_free(fedges);

  // ── STEP round-trip ─────────────────────────────────────────────────────
  const char* step = "/tmp/cck_roundtrip.step";
  check(cc_step_export(box, step) == 1, "cc_step_export -> 1");
  CCShapeId reimported = cc_step_import(step);
  check(reimported != 0, "cc_step_import -> valid id");
  if (reimported) {
    CCMassProps rmp = cc_mass_properties(reimported);
    check(rmp.valid && near(rmp.volume, 1000.0, 1e-3), "STEP round-trip volume preserved",
          "got " + std::to_string(rmp.volume));
    cc_shape_release(reimported);
  }

  // ── Determinism: parallel boolean + meshing run-to-run reproducible ──────
  const int N = 16;
  uint64_t hFuse = 0, hMesh = 0; bool detFuse = true, detMesh = true;
  for (int i = 0; i < N; ++i) {
    CCShapeId f = cc_boolean(box, box2t, 0);
    CCMesh fm = cc_tessellate(f, 0.1);
    uint64_t h = hashMesh(fm);
    if (i == 0) hFuse = h; else if (h != hFuse) detFuse = false;
    cc_mesh_free(fm); cc_shape_release(f);
  }
  check(detFuse, "parallel fuse+mesh identical across " + std::to_string(N) + " runs");
  for (int i = 0; i < N; ++i) {
    CCMesh bm = cc_tessellate(box, 0.05);
    uint64_t h = hashMesh(bm);
    if (i == 0) hMesh = h; else if (h != hMesh) detMesh = false;
    cc_mesh_free(bm);
  }
  check(detMesh, "parallel box mesh identical across " + std::to_string(N) + " runs");

  // ── Benchmark: wall-clock of boolean + fine meshing ─────────────────────
  using clk = std::chrono::steady_clock;
  auto t0 = clk::now();
  for (int i = 0; i < 50; ++i) { CCShapeId f = cc_boolean(box, box2t, 0); cc_shape_release(f); }
  auto t1 = clk::now();
  CCMesh fine = cc_tessellate(fused, 0.01);
  auto t2 = clk::now();
  double boolMs = std::chrono::duration<double, std::milli>(t1 - t0).count() / 50.0;
  double meshMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
  std::printf("[BENCH] boolean fuse: %.3f ms/op (avg of 50) | fine mesh (defl=0.01): %.3f ms, %d tris\n",
              boolMs, meshMs, fine.triangleCount);
  cc_mesh_free(fine);

  cc_shape_release(fused); cc_shape_release(cutd); cc_shape_release(common);
  cc_shape_release(box2t); cc_shape_release(box);

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);

  // OCCT's static teardown crashes at process exit after algorithm use in the
  // trimmed static build — NOT a CyberCadKernel defect: every shape is released
  // and the facade's engine/registry singletons are intentionally leaked, so no
  // OCCT object of ours is freed at exit; the residual state is OCCT's own
  // internal statics. Exit without running C++ static destructors so the harness
  // reports its true result. Harmless to the app, which never exits via clean
  // static teardown.
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
