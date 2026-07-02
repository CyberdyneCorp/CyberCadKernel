// Tiny shared harness for the CyberCadKernel OCCT runtime suite (iOS simulator).
//
// Every module (checks_construct.cpp, checks_feature.cpp, ...) includes this
// header, receives a Ctx&, and records results via ctx.check(...). full_suite.cpp
// owns the single Ctx, runs each module in order, and prints the tally.
//
// The helpers (near / hashMesh) and Ctx::check are inline so this header can be
// included by every translation unit without an ODR/link clash. checks.h must
// compile standalone (it pulls in cc_kernel.h and the std bits it needs).

#ifndef CYBERCADKERNEL_TESTS_SIM_CHECKS_H
#define CYBERCADKERNEL_TESTS_SIM_CHECKS_H

#include "cybercadkernel/cc_kernel.h"

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstddef>
#include <string>

// ── Result context ──────────────────────────────────────────────────────────
// One instance lives in main(); passed by reference into every run_*_checks.
// `passed`/`failed` accumulate across all modules; check() prints a line per
// assertion in the same [PASS]/[FAIL] format parity_bench.cpp uses.
struct Ctx {
  int passed = 0;
  int failed = 0;

  // Record one assertion. `what` is the human-readable claim; `detail` is
  // optional context (e.g. the actual value) appended after an em dash.
  // Returns `ok` so callers can early-out on a failed precondition.
  bool check(bool ok, const std::string& what, const std::string& detail = "") {
    std::printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(),
                detail.empty() ? "" : " — ", detail.c_str());
    if (ok) ++passed; else ++failed;
    std::fflush(stdout);
    return ok;
  }
};

// ── Numeric tolerance helper ────────────────────────────────────────────────
// True when |a - b| <= tol. Use for volumes/areas/coordinates vs analytic values.
inline bool near(double a, double b, double tol) {
  return std::fabs(a - b) <= tol;
}

// ── Mesh hash helper ────────────────────────────────────────────────────────
// FNV-1a over a mesh's counts + raw vertex/triangle buffers, for byte-level
// run-to-run determinism comparison (identical mesh => identical hash).
inline uint64_t hashMesh(const CCMesh& m) {
  uint64_t h = 1469598103934665603ull;
  auto mix = [&](const void* p, std::size_t n) {
    const unsigned char* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
  };
  mix(&m.vertexCount, sizeof m.vertexCount);
  mix(&m.triangleCount, sizeof m.triangleCount);
  if (m.vertices)  mix(m.vertices,  sizeof(double) * static_cast<std::size_t>(m.vertexCount)   * 3);
  if (m.triangles) mix(m.triangles, sizeof(int)    * static_cast<std::size_t>(m.triangleCount) * 3);
  return h;
}

// ── Per-module entry points ─────────────────────────────────────────────────
// Each is defined in its own checks_<module>.cpp translation unit (written by
// other agents). full_suite.cpp calls them in this order. Every cc_* function in
// the facade is exercised by exactly one of these modules; run_accel_checks adds
// the determinism + benchmark pass.
void run_construct_checks(Ctx&);       // cc_solid_extrude/revolve/loft/sweep/thread/... + legacy cc_extrude
void run_feature_checks(Ctx&);         // cc_fillet_*/chamfer/shell/offset_face/replace_face/split_plane
void run_booltransform_checks(Ctx&);   // cc_boolean + scale/rotate/mirror/translate/place_on_frame
void run_tessellate_checks(Ctx&);      // cc_tessellate/cc_face_meshes/cc_edge_polylines
void run_query_checks(Ctx&);           // cc_mass_properties/principal_moments/bounding_box/face_axis/subshape_ids/chains/offset_face_boundary
void run_exchange_checks(Ctx&);        // cc_step_export/import + cc_iges_export/import
void run_accel_checks(Ctx&);           // determinism (parallel run-to-run) + wall-clock benchmark

#endif  // CYBERCADKERNEL_TESTS_SIM_CHECKS_H
