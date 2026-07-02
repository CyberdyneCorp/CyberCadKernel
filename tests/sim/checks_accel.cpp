// Acceleration module of the CyberCadKernel OCCT runtime suite.
//
// Phase 1 determinism audit (task 5.1) + wall-clock benchmark (task 6.1), driven
// through the ADDITIVE cc_set_parallel(int) toggle. For a set of representative
// bodies (box-box fuse, box-box cut, a revolve, and a multi-face filleted solid)
// this proves two properties and measures one:
//
//   Determinism A/B (5.1) — the SAME operation (boolean/revolve/fillet + display
//     tessellation) run with parallelism OFF (cc_set_parallel(0)) and ON
//     (cc_set_parallel(1)) yields a BIT-IDENTICAL result: same FNV-1a mesh hash,
//     same exact B-rep volume, same triangle count. Determinism is the default
//     contract, so the parallel path must not perturb a single byte.
//
//   Parallel run-to-run stability — repeating the parallel computation 8x yields
//     the identical result every time (no thread-order-dependent output).
//
//   Benchmark (6.1) — N iterations of (boolean + meshing) timed with
//     std::chrono::steady_clock under serial vs parallel; prints per-op serial ms,
//     parallel ms, and the speedup.
//
// Emits a [DET] line and a [BENCH] line per body. Restores cc_set_parallel(1) and
// releases every body/buffer before returning. This TU is OCCT-free (facade only)
// and compiles on the host, but its assertions are meaningful only under the real
// adapter in the iOS simulator.

#include "checks.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace {

using clk = std::chrono::steady_clock;

const double kPi = 3.14159265358979323846;

// 10x10 square profile -> a 10x10x10 box when extruded 10 in +Z.
const double kSquare[8] = {0, 0, 10, 0, 10, 10, 0, 10};

// Rectangle x[2,5], y[0,10]; revolved 2pi about Y gives a tube (vol 210*pi).
const double kRect[8] = {2, 0, 5, 0, 5, 10, 2, 10};

// ── A body under test ─────────────────────────────────────────────────────────
// build() runs the parallel-SENSITIVE operation (boolean / revolve / fillet) and
// returns a FRESH body id (0 on failure); the caller tessellates it at `defl`.
struct Body {
  std::string name;
  std::function<CCShapeId()> build;
  double defl;
};

// ── One computed sample ───────────────────────────────────────────────────────
// The byte-hash of the display mesh, the exact B-rep volume, and the triangle
// count. Two samples are "identical" iff all three match bit-for-bit.
struct Sample {
  uint64_t meshHash = 0;
  double volume = 0.0;
  int triangleCount = 0;
  bool ok = false;
};

// Fold a double's raw bytes into an existing FNV-1a hash (same mixing hashMesh
// uses), so the volume participates in the single 64-bit determinism value.
uint64_t foldBytes(uint64_t h, const void* p, std::size_t n) {
  const unsigned char* b = static_cast<const unsigned char*>(p);
  for (std::size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
  return h;
}

// Build the body, tessellate it, capture hash+volume+tris, free/release both.
Sample computeOnce(const Body& body) {
  Sample s;
  CCShapeId id = body.build();
  if (id == 0) return s;
  CCMassProps mp = cc_mass_properties(id);
  CCMesh mesh = cc_tessellate(id, body.defl);
  const double vol = mp.valid ? mp.volume : 0.0;
  s.meshHash = foldBytes(hashMesh(mesh), &vol, sizeof vol);
  s.volume = vol;
  s.triangleCount = mesh.triangleCount;
  s.ok = mp.valid && mesh.triangleCount > 0;
  cc_mesh_free(mesh);
  cc_shape_release(id);
  return s;
}

// Bit-for-bit sample equality (mesh hash, exact volume, triangle count).
bool sameSample(const Sample& a, const Sample& b) {
  return a.ok && b.ok && a.meshHash == b.meshHash && a.volume == b.volume &&
         a.triangleCount == b.triangleCount;
}

// Total wall-clock ms to run computeOnce N times (boolean/revolve/fillet + mesh).
double timeRuns(const Body& body, int n) {
  auto t0 = clk::now();
  for (int i = 0; i < n; ++i) {
    Sample s = computeOnce(body);
    (void)s;
  }
  auto t1 = clk::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

std::string fmt(double v, int prec = 3) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", prec, v);
  return std::string(buf);
}

std::string hex64(uint64_t h) {
  char buf[24];
  std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(h));
  return std::string(buf);
}

// ── Determinism A/B + run-to-run stability for one body (task 5.1) ─────────────
// Returns the parallel reference sample (for the benchmark's tri count).
Sample runDeterminism(Ctx& ctx, const Body& body) {
  cc_set_parallel(0);
  const Sample serial = computeOnce(body);
  cc_set_parallel(1);
  const Sample par = computeOnce(body);

  if (!ctx.check(serial.ok && par.ok,
                 "accel/" + body.name + ": serial & parallel both built")) {
    return par;
  }

  const bool det = sameSample(serial, par);
  ctx.check(det, "accel/" + body.name + ": serial == parallel (bit-reproducible)",
            "serialHash=" + hex64(serial.meshHash) + " parHash=" + hex64(par.meshHash) +
            " vol=" + fmt(serial.volume, 6));

  bool stable = true;
  for (int i = 0; i < 8 && stable; ++i) {
    if (!sameSample(computeOnce(body), par)) stable = false;
  }
  ctx.check(stable, "accel/" + body.name + ": parallel stable across 8 runs");

  std::printf("[DET] %-13s serial==parallel: %-3s | parallel 8x stable: %-3s | vol=%s tris=%d\n",
              body.name.c_str(), det ? "YES" : "NO", stable ? "YES" : "NO",
              fmt(serial.volume, 4).c_str(), par.triangleCount);
  std::fflush(stdout);
  return par;
}

// ── Serial-vs-parallel wall-clock benchmark for one body (task 6.1) ────────────
void runBenchmark(Ctx& ctx, const Body& body, const Sample& ref) {
  const int N = 20;

  cc_set_parallel(0);
  const double serialMs = timeRuns(body, N);
  cc_set_parallel(1);
  const double parMs = timeRuns(body, N);

  const double speedup = parMs > 0.0 ? serialMs / parMs : 0.0;
  std::printf("[BENCH] %-13s serial=%.3f ms  parallel=%.3f ms  speedup=%.2fx  (N=%d, %d tris)\n",
              body.name.c_str(), serialMs / N, parMs / N, speedup, N, ref.triangleCount);
  std::fflush(stdout);

  ctx.check(serialMs > 0.0 && parMs > 0.0,
            "accel/" + body.name + ": benchmark timed both serial and parallel");
}

}  // namespace

void run_accel_checks(Ctx& ctx) {
  std::printf("-- accel: determinism A/B (cc_set_parallel) + wall-clock benchmark --\n");
  std::fflush(stdout);

  // Operands are deterministic; build once and capture in the per-body lambdas.
  CCShapeId a = cc_solid_extrude(kSquare, 4, 10.0);
  CCShapeId b0 = cc_solid_extrude(kSquare, 4, 10.0);
  CCShapeId b = cc_translate_shape(b0, 5, 5, 5);  // overlaps `a` in a 5^3 cube
  cc_shape_release(b0);

  // Base box + its 12 edge ids, for the multi-face filleted solid.
  CCShapeId base = cc_solid_extrude(kSquare, 4, 10.0);
  int* edges = nullptr;
  const int nedge = cc_subshape_ids(base, 1, &edges);
  std::vector<int> edgeIds(edges, edges + (nedge > 0 ? nedge : 0));
  if (edges) cc_ints_free(edges);

  if (!ctx.check(a != 0 && b != 0 && base != 0 && nedge > 0,
                 "accel setup: fuse/cut operands + base box built")) {
    cc_shape_release(a);
    cc_shape_release(b);
    cc_shape_release(base);
    cc_set_parallel(1);
    return;
  }

  std::vector<Body> bodies;
  bodies.push_back({"box-box fuse", [a, b]() { return cc_boolean(a, b, 0); }, 0.05});
  bodies.push_back({"box-box cut",  [a, b]() { return cc_boolean(a, b, 1); }, 0.05});
  bodies.push_back({"revolve tube", []() { return cc_solid_revolve(kRect, 4, 2.0 * kPi); }, 0.02});
  bodies.push_back({"fillet solid", [base, edgeIds]() {
                      return cc_fillet_edges(base, edgeIds.data(),
                                             static_cast<int>(edgeIds.size()), 1.5);
                    }, 0.02});

  for (const Body& body : bodies) {
    const Sample ref = runDeterminism(ctx, body);
    runBenchmark(ctx, body, ref);
  }

  // Restore the default (parallel ON) and release every operand.
  cc_set_parallel(1);
  cc_shape_release(a);
  cc_shape_release(b);
  cc_shape_release(base);
}
