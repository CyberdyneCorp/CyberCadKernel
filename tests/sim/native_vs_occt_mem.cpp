// SPDX-License-Identifier: Apache-2.0
//
// native_vs_occt_mem.cpp — MOAT drop-OCCT PAYOFF harness: native-vs-OCCT
//                          per-operation RUNTIME MEMORY footprint, driven entirely
//                          through the public cc_* facade under BOTH engines.
//
// WHY: the drop-OCCT campaign has measured the latency payoff
// (tests/sim/native_vs_occt_bench.cpp) and the binary-size payoff
// (scripts/bench-binary-size.sh). The missing third leg is RUNTIME RAM — the tightest
// constraint on iPad. This harness produces the MEMORY half: a per-op table of the
// peak resident / footprint high-water mark of each op under native vs OCCT, plus a
// one-shot process-level peak-RSS comparison (native-only vs OCCT build running the
// SAME representative script) that captures OCCT's static/global footprint.
//
// ── MEASUREMENT METHOD (honest; the most reliable on host macOS arm64) ──────────
//   The macOS memory picture is measured three complementary ways, each labelled:
//
//   (1) PROCESS PEAK RSS  — getrusage(RUSAGE_SELF).ru_maxrss. This is a per-process
//       HIGH-WATER MARK that never falls, so it is only meaningful measured ONCE
//       PER PROCESS. The runner therefore invokes this harness in --single mode
//       ONE (op,size,engine) PER PROCESS: each process does a bounded warm-up build
//       then the measured op, so ru_maxrss at exit is a CLEAN peak attributable to
//       that engine doing that op (no high-water contamination from a previous op).
//
//   (2) PHYS_FOOTPRINT   — task_info(TASK_VM_INFO).phys_footprint, the value the OS
//       memory-pressure system charges the process (what Xcode's memory gauge shows).
//       Reported at end-of-process alongside ru_maxrss for the absolute picture.
//
//   (3) PER-OP FOOTPRINT DELTA — phys_footprint sampled just BEFORE and at the PEAK
//       of the measured region (while the op's result is still alive), reported as a
//       delta. This is the cleanest engine-to-engine per-op signal because it
//       subtracts the fixed baseline (process + static OCCT data), isolating the
//       op's own working-set. NOTE on macOS the allocator does not always return
//       freed pages, so the delta is measured at the op's live peak, not after free,
//       and each op runs in its own process so deltas do not accumulate.
//
//   malloc-zone size_in_use / mstats were evaluated and REJECTED: on macOS arm64
//   large allocations are mmap-backed and do NOT appear in malloc_zone_statistics
//   size_in_use (verified empirically), so a zone-delta would under-count geometry
//   working sets. phys_footprint counts them; it is the honest signal.
//
//   Determinism: fixed regular-N-gon prism inputs (SAME as the latency bench for
//   comparability) — no rand(), no wall-clock seeding. Sizes small/medium/large =
//   octagon / 32-gon / 96-gon prism.
//
//   Honest served-labelling (mirrors the latency bench): a native op the NativeEngine
//   FORWARDS to OCCT (curved / out-of-domain) gets NO native number; an op the OCCT
//   adapter does not implement (section) is labelled native-only. The engine that
//   actually serves each op is reported.
//
// ── MODES ───────────────────────────────────────────────────────────────────────
//   --single --op NAME --size LABEL --engine 0|1
//       Run ONE op in isolation in this process; print a single [MEMROW] line with
//       peak RSS, phys_footprint, and the per-op footprint delta. (Clean high-water.)
//   --script --engine 0|1
//       Run the WHOLE representative script (construct + boolean + tessellate + mass
//       + section) once under the given engine; print a [PROCROW] line with the
//       process peak RSS / footprint. This is the one-shot process-level number that
//       captures OCCT's static/global footprint (complements the binary-size doc).
//   --list
//       Print the (op,size,engine,served) matrix the runner should drive, one
//       [PLAN] line each, so the runner needs no hard-coded op list.
//
// Output is machine-readable; scripts/bench-memory-native-vs-occt.sh scrapes it.
// Flushes stdout and std::_Exit (the trimmed static-OCCT teardown is not exit-clean —
// same rationale as the sibling sim harnesses).
//
// Build/run: scripts/bench-memory-native-vs-occt.sh (host, Homebrew OCCT).
// NON-SHIPPING measurement harness — carries its own main(), not in the CTest set.

#include "cybercadkernel/cc_kernel.h"

#include <mach/mach.h>
#include <sys/resource.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── macOS memory probes ─────────────────────────────────────────────────────────
// phys_footprint: bytes the OS charges the process (mmap-backed large allocs counted).
size_t physFootprint() {
  task_vm_info_data_t info;
  mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
  if (task_info(mach_task_self(), TASK_VM_INFO, reinterpret_cast<task_info_t>(&info),
                &count) == KERN_SUCCESS) {
    return static_cast<size_t>(info.phys_footprint);
  }
  return 0;
}
// ru_maxrss: process peak resident set (bytes on macOS). High-water mark, never falls.
size_t peakRss() {
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return static_cast<size_t>(ru.ru_maxrss);
}

// ── Deterministic fixtures (identical to the latency bench for comparability) ────
std::vector<double> ngon(int n, double r) {
  std::vector<double> p;
  p.reserve(2 * n);
  for (int i = 0; i < n; ++i) {
    double a = 2.0 * kPi * i / n;
    p.push_back(r * std::cos(a));
    p.push_back(r * std::sin(a));
  }
  return p;
}

struct Size { const char* label; int n; double depth; };
const Size kSizes[] = {
  {"small",  8,  10.0},
  {"medium", 32, 10.0},
  {"large",  96, 10.0},
};
const Size* findSize(const std::string& label) {
  for (const auto& s : kSizes)
    if (label == s.label) return &s;
  return nullptr;
}

// One measured op. `body` runs the op AND, at its live peak (result still alive),
// samples phys_footprint via the supplied sampler so we get a clean per-op delta.
// Each op keeps its result alive until the sample is taken, then releases.

// The op matrix. `served` is the honest verdict for the NATIVE engine:
//   "native"       native serves it (planar-domain)
//   "native-only"  OCCT adapter declines it (section) — native is the only server
//   "forwarded"    native forwards to OCCT (curved fillet) — no native number
struct OpSpec { const char* op; const char* nativeServed; };
const OpSpec kOps[] = {
  {"boolean_fuse",    "native"},
  {"boolean_cut",     "native"},
  {"boolean_common",  "native"},
  {"tessellate",      "native"},
  {"mass_properties", "native"},
  {"section",         "native-only"},
  {"fillet_edges",    "forwarded"},
};

// Run a single op once, sampling footprint at its live peak. Returns the peak-delta
// in bytes (footprint at live peak minus footprint just before). The op builds its
// own operands under the ACTIVE engine so the measured working set includes
// construction, matching how the app produces the result.
ssize_t runOpMeasured(const std::string& op, const Size& sz) {
  std::vector<double> prof = ngon(sz.n, 10.0);
  const int np = sz.n;
  const double depth = sz.depth;

  auto boolBody = [&](int opcode) -> ssize_t {
    CCShapeId a = cc_solid_extrude(prof.data(), np, depth);
    CCShapeId b0 = cc_solid_extrude(prof.data(), np, depth);
    CCShapeId b = cc_translate_shape(b0, 5, 5, 5);
    size_t before = physFootprint();
    CCShapeId r = cc_boolean(a, b, opcode);
    size_t peak = physFootprint();  // result alive
    if (r) cc_shape_release(r);
    cc_shape_release(b);
    cc_shape_release(a);
    return static_cast<ssize_t>(peak) - static_cast<ssize_t>(before);
  };

  if (op == "boolean_fuse")   return boolBody(0);
  if (op == "boolean_cut")    return boolBody(1);
  if (op == "boolean_common") return boolBody(2);

  if (op == "tessellate") {
    CCShapeId body = cc_solid_extrude(prof.data(), np, depth);
    size_t before = physFootprint();
    CCMesh m = cc_tessellate(body, 0.05);
    size_t peak = physFootprint();
    cc_mesh_free(m);
    cc_shape_release(body);
    return static_cast<ssize_t>(peak) - static_cast<ssize_t>(before);
  }

  if (op == "mass_properties") {
    CCShapeId body = cc_solid_extrude(prof.data(), np, depth);
    size_t before = physFootprint();
    volatile CCMassProps mp = cc_mass_properties(body);
    (void)mp;
    size_t peak = physFootprint();
    cc_shape_release(body);
    return static_cast<ssize_t>(peak) - static_cast<ssize_t>(before);
  }

  if (op == "section") {
    const double origin[3] = {0, 0, 5.0};
    const double normal[3] = {0, 0, 1.0};
    CCShapeId body = cc_solid_extrude(prof.data(), np, depth);
    size_t before = physFootprint();
    CCSection s = cc_section_plane(body, origin, normal);
    size_t peak = physFootprint();
    cc_section_free(s);
    cc_shape_release(body);
    return static_cast<ssize_t>(peak) - static_cast<ssize_t>(before);
  }

  if (op == "fillet_edges") {
    // A box edge fillet (curved face). Size fixtures do not apply; use a 10x10x10 box.
    const double sq[8] = {0, 0, 10, 0, 10, 10, 0, 10};
    CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    int* edges = nullptr;
    int ne = cc_subshape_ids(box, 1, &edges);
    ssize_t delta = 0;
    if (ne > 0) {
      int oneEdge = edges[0];
      size_t before = physFootprint();
      CCShapeId f = cc_fillet_edges(box, &oneEdge, 1, 1.0);
      size_t peak = physFootprint();
      if (f) cc_shape_release(f);
      delta = static_cast<ssize_t>(peak) - static_cast<ssize_t>(before);
    }
    if (edges) cc_ints_free(edges);
    cc_shape_release(box);
    return delta;
  }

  return 0;
}

// Does the OCCT adapter actually serve `op` on this fixture (vs decline)? Only
// section is known to be declined by the OCCT adapter; probe it at runtime.
bool occtServes(const std::string& op, const Size& sz) {
  if (op != "section") return true;
  std::vector<double> prof = ngon(sz.n, 10.0);
  const double origin[3] = {0, 0, 5.0};
  const double normal[3] = {0, 0, 1.0};
  cc_set_engine(0);
  CCShapeId body = cc_solid_extrude(prof.data(), sz.n, sz.depth);
  CCSection s = cc_section_plane(body, origin, normal);
  bool serves = (s.loopCount > 0);
  cc_section_free(s);
  cc_shape_release(body);
  return serves;
}

// Run the whole representative script once under the active engine (bounded work,
// same fixtures) so the process peak RSS reflects the engine's full working set +
// (for OCCT) its static/global data. Order: construct + all three booleans +
// tessellate + mass + section, over all three sizes.
void runScript() {
  for (const auto& sz : kSizes) {
    std::vector<double> prof = ngon(sz.n, 10.0);
    for (int opcode = 0; opcode <= 2; ++opcode) {
      CCShapeId a = cc_solid_extrude(prof.data(), sz.n, sz.depth);
      CCShapeId b0 = cc_solid_extrude(prof.data(), sz.n, sz.depth);
      CCShapeId b = cc_translate_shape(b0, 5, 5, 5);
      CCShapeId r = cc_boolean(a, b, opcode);
      if (r) cc_shape_release(r);
      cc_shape_release(b);
      cc_shape_release(a);
    }
    CCShapeId body = cc_solid_extrude(prof.data(), sz.n, sz.depth);
    CCMesh m = cc_tessellate(body, 0.05);
    cc_mesh_free(m);
    volatile CCMassProps mp = cc_mass_properties(body);
    (void)mp;
    const double origin[3] = {0, 0, 5.0};
    const double normal[3] = {0, 0, 1.0};
    CCSection s = cc_section_plane(body, origin, normal);
    cc_section_free(s);
    cc_shape_release(body);
  }
}

const char* argValue(int argc, char** argv, const char* key) {
  for (int i = 1; i < argc - 1; ++i)
    if (std::strcmp(argv[i], key) == 0) return argv[i + 1];
  return nullptr;
}
bool hasFlag(int argc, char** argv, const char* key) {
  for (int i = 1; i < argc; ++i)
    if (std::strcmp(argv[i], key) == 0) return true;
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (!cc_brep_available()) {
    std::printf("OCCT not linked — aborting\n");
    return 2;
  }

  // ── --list : emit the (op,size,engine,served) plan for the runner ──────────────
  if (hasFlag(argc, argv, "--list")) {
    for (const auto& os : kOps) {
      // fillet_edges has a single "box" fixture, not the size spread.
      if (std::strcmp(os.op, "fillet_edges") == 0) {
        std::printf("[PLAN] op=%s size=box native_served=%s\n", os.op, os.nativeServed);
        continue;
      }
      for (const auto& sz : kSizes)
        std::printf("[PLAN] op=%s size=%s native_served=%s\n", os.op, sz.label, os.nativeServed);
    }
    std::fflush(stdout);
    std::_Exit(0);
  }

  // ── --script : one-shot process-level peak RSS under one engine ────────────────
  if (hasFlag(argc, argv, "--script")) {
    const char* eng = argValue(argc, argv, "--engine");
    int engine = eng ? std::atoi(eng) : 1;
    size_t base = physFootprint();
    cc_set_engine(engine);
    runScript();
    size_t rss = peakRss();
    size_t fp = physFootprint();
    std::printf("[PROCROW] engine=%s baseline_footprint_bytes=%zu peak_rss_bytes=%zu end_footprint_bytes=%zu\n",
                engine == 1 ? "native" : "occt", base, rss, fp);
    std::fflush(stdout);
    std::_Exit(0);
  }

  // ── --single : one (op,size,engine) in isolation → clean high-water ────────────
  if (hasFlag(argc, argv, "--single")) {
    const char* opC = argValue(argc, argv, "--op");
    const char* sizeC = argValue(argc, argv, "--size");
    const char* engC = argValue(argc, argv, "--engine");
    if (!opC || !sizeC || !engC) {
      std::printf("usage: --single --op NAME --size LABEL --engine 0|1\n");
      return 2;
    }
    std::string op = opC, sizeLbl = sizeC;
    int engine = std::atoi(engC);

    // fillet_edges uses the "box" fixture; the size loop label is "box".
    bool isFillet = (op == "fillet_edges");
    Size fixture;
    if (isFillet) {
      fixture = Size{"box", 4, 10.0};
    } else {
      const Size* s = findSize(sizeLbl);
      if (!s) { std::printf("unknown size: %s\n", sizeLbl.c_str()); return 2; }
      fixture = *s;
    }

    // Honest served verdict for this (op, engine).
    std::string served;
    if (engine == 0) {
      served = "occt";
    } else {
      // native side: forwarded (fillet) / native-only (section, if OCCT declines) / native
      if (isFillet) {
        served = "forwarded";
      } else if (op == "section") {
        served = occtServes(op, fixture) ? "native" : "native-only";
      } else {
        served = "native";
      }
    }

    // A forwarded native op has no honest native number (it re-times OCCT). Report it
    // as forwarded with no measurement so the table never fakes a native win.
    if (engine == 1 && served == "forwarded") {
      std::printf("[MEMROW] op=%s size=%s engine=native served=forwarded "
                  "peak_rss_bytes=- footprint_bytes=- op_delta_bytes=-\n",
                  op.c_str(), sizeLbl.c_str());
      std::fflush(stdout);
      std::_Exit(0);
    }
    // An OCCT-declined op (section under OCCT) has no OCCT number.
    if (engine == 0 && op == "section" && !occtServes(op, fixture)) {
      std::printf("[MEMROW] op=%s size=%s engine=occt served=occt-declined "
                  "peak_rss_bytes=- footprint_bytes=- op_delta_bytes=-\n",
                  op.c_str(), sizeLbl.c_str());
      std::fflush(stdout);
      std::_Exit(0);
    }

    cc_set_engine(engine);
    // One bounded warm-up (build+run once, discarded) to fault in code pages so the
    // measured delta reflects the op's DATA working set, not first-touch code paging.
    (void)runOpMeasured(op, fixture);
    ssize_t opDelta = runOpMeasured(op, fixture);
    size_t rss = peakRss();
    size_t fp = physFootprint();
    std::printf("[MEMROW] op=%s size=%s engine=%s served=%s "
                "peak_rss_bytes=%zu footprint_bytes=%zu op_delta_bytes=%zd\n",
                op.c_str(), sizeLbl.c_str(), engine == 1 ? "native" : "occt",
                served.c_str(), rss, fp, opDelta);
    std::fflush(stdout);
    std::_Exit(0);
  }

  std::printf("usage: %s [--list | --script --engine E | --single --op O --size S --engine E]\n",
              argv[0]);
  return 2;
}
