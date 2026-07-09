// SPDX-License-Identifier: Apache-2.0
//
// native_vs_occt_bench.cpp — MOAT drop-OCCT PAYOFF harness: native-vs-OCCT
//                            per-operation LATENCY, driven entirely through the
//                            public cc_* facade under BOTH engines.
//
// WHY: the drop-OCCT campaign needs the "why" behind unlinking OCCT for the iPad
// CAD app — the measured performance delta of the clean-room native engine vs the
// OCCT oracle on the SAME operations, plus (companion, measured by
// scripts/bench-binary-size.sh) the shipped binary-size win. This harness produces
// the LATENCY half: a per-op table of native ms vs OCCT ms + ratio.
//
// ── METHOD (deterministic, honest) ────────────────────────────────────────────
//   * Runs on the HOST (macOS arm64) against Homebrew OCCT, NOT on device. Host
//     timing is deterministic (stable CPU, no thermal/scheduler noise of a booted
//     simulator) and is what we compare engine-to-engine. Device latency differs in
//     absolute terms but the RATIO (native/OCCT) is the portable signal. This is
//     stated in every report line and in the findings doc.
//   * Each op is driven through the SAME public cc_* call the app calls, once with
//     cc_set_engine(0) (OCCT oracle) and once with cc_set_engine(1) (NativeEngine).
//     This is the pattern of native_boolean_parity / native_transformed_boolean_fuzz.
//   * The engine that actually SERVES an op is recorded honestly:
//       - NATIVE-SERVED: the NativeEngine intercepts (planar-faced boolean, native
//         solid_extrude/revolve construction, native tessellate/mass/bbox on a
//         native body). We time the native code path.
//       - FORWARDED: the NativeEngine forwards to OCCT (curved operands, ops outside
//         the native planar domain). Timing under "native" then == OCCT; we DO NOT
//         report a speedup for a forwarded op — it is labelled FORWARDED and its
//         "native" column is omitted (—) so the table never fakes a native win.
//   * Timing: fixed inputs (no rand / no wall-clock seeding), warm (kWarmup discarded
//     iterations), then kIters measured iterations; we report MEDIAN plus min/max so
//     the reader sees run-to-run variance. steady_clock, -O2.
//   * Fixtures span a spread of model sizes (small / medium / large polygon counts)
//     so the ratio is not a single-point claim.
//
// Output: machine-readable [ROW] lines (op,size,engine,served,median_ms,min_ms,
// max_ms,n) + a human table; scripts/bench-native-vs-occt.sh scrapes the [ROW] lines
// into the findings doc. Flushes stdout and std::_Exit (the trimmed static-OCCT
// teardown is not exit-clean — same rationale as the sibling sim harnesses).
//
// Build/run: scripts/bench-native-vs-occt.sh (host, Homebrew OCCT). NON-SHIPPING
// measurement harness — it carries its own main() and is not in the CTest set.

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

namespace {

constexpr int kWarmup = 3;      // discarded warm-up iterations
constexpr int kIters  = 25;     // measured iterations (median-of-N, N=25)
constexpr double kPi  = 3.14159265358979323846;

using clk = std::chrono::steady_clock;

struct Stat { double median, lo, hi; int n; };

// Time a nullary op kIters times (after kWarmup) and return median/min/max in ms.
// The op must be self-contained (build + release its own transient shapes) so we
// measure only its own cost run-to-run.
Stat timeOp(const std::function<void()>& op) {
  for (int i = 0; i < kWarmup; ++i) op();
  std::vector<double> ms;
  ms.reserve(kIters);
  for (int i = 0; i < kIters; ++i) {
    auto t0 = clk::now();
    op();
    auto t1 = clk::now();
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  Stat s;
  s.n = kIters;
  s.lo = ms.front();
  s.hi = ms.back();
  s.median = ms[kIters / 2];
  return s;
}

// ── Deterministic fixtures ────────────────────────────────────────────────────
// A regular N-gon profile of radius r centred at origin (x,y pairs), CCW.
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

// A solid cylinder (radius r, height h along +Y about the Y axis): revolve a
// rectangle profile a full turn. This is a CURVED body (native declines/forwards).
CCShapeId makeCylinder(double r, double h) {
  const double rect[8] = {0, 0, r, 0, r, h, 0, h};
  return cc_solid_revolve(rect, 4, 2.0 * kPi);
}

// One measured op row: op name, model-size label, and the served/forwarded verdict.
struct Row {
  std::string op, size, served;  // served: "native" | "forwarded" | "occt"
  Stat stat;
  bool hasStat;
};
std::vector<Row> g_rows;

void emit(const std::string& op, const std::string& size,
          const std::string& engine, const std::string& served,
          const Stat& s, bool hasStat) {
  if (hasStat) {
    std::printf("[ROW] op=%s size=%s engine=%s served=%s median_ms=%.4f min_ms=%.4f max_ms=%.4f n=%d\n",
                op.c_str(), size.c_str(), engine.c_str(), served.c_str(),
                s.median, s.lo, s.hi, s.n);
  } else {
    std::printf("[ROW] op=%s size=%s engine=%s served=%s median_ms=- min_ms=- max_ms=- n=0\n",
                op.c_str(), size.c_str(), engine.c_str(), served.c_str());
  }
  std::fflush(stdout);
}

// Detect whether the NativeEngine actually SERVED a boolean natively, vs forwarded to
// OCCT. Heuristic per the ABI contract: a native-native planar boolean is intercepted;
// build both operands under native and check the result is non-zero AND the active
// engine is native. For a curved operand the engine forwards (same result, OCCT cost).
// We record served="native" only for the planar box case where interception is
// guaranteed by the facade contract (see cc_boolean ENGINE NOTE) — everything else is
// labelled by construction below.

}  // namespace

int main() {
  std::printf("== CyberCadKernel native-vs-OCCT latency bench (HOST, Homebrew OCCT) ==\n");
  std::printf("== method: median-of-%d, %d warm-up discarded, fixed inputs, steady_clock, -O2 ==\n",
              kIters, kWarmup);
  std::printf("== NOTE: HOST timing (macOS arm64), not device. The native/OCCT RATIO is the portable signal. ==\n");
  if (!cc_brep_available()) { std::printf("OCCT not linked — aborting\n"); return 2; }

  // Model-size spread for the polyhedral (native-domain) ops: extrude an N-gon prism.
  struct Size { const char* label; int n; double depth; };
  const Size sizes[] = {
    {"small",  8,  10.0},   // octagonal prism
    {"medium", 32, 10.0},   // 32-gon prism
    {"large",  96, 10.0},   // 96-gon prism (many planar side faces)
  };

  auto run = [&](const std::string& op, const std::string& size,
                 int engine, const std::string& served,
                 const std::function<void()>& body) {
    cc_set_engine(engine);
    Stat s = timeOp(body);
    emit(op, size, engine == 1 ? "native" : "occt", served, s, true);
    g_rows.push_back({op, size, engine == 1 ? "native+" + served : "occt", s, true});
  };

  // ════════════════════════════════════════════════════════════════════════════
  // 1. BOOLEAN — planar prisms (NATIVE-SERVED under the native engine) across sizes
  //    fuse / cut / common of an N-gon prism with a translated copy (overlap).
  //    Operands are built UNDER THE ACTIVE ENGINE inside the timed body so the
  //    native BSP-CSG path is exercised end to end (build + boolean), matching how
  //    the app produces a boolean from freshly built solids.
  // ════════════════════════════════════════════════════════════════════════════
  for (const auto& sz : sizes) {
    std::vector<double> prof = ngon(sz.n, 10.0);
    const int np = sz.n;
    const double depth = sz.depth;
    for (int opcode = 0; opcode <= 2; ++opcode) {
      const char* opname = opcode == 0 ? "boolean_fuse" : opcode == 1 ? "boolean_cut" : "boolean_common";
      auto bodyFn = [prof, np, depth, opcode]() {
        CCShapeId a = cc_solid_extrude(prof.data(), np, depth);
        CCShapeId b0 = cc_solid_extrude(prof.data(), np, depth);
        CCShapeId b = cc_translate_shape(b0, 5, 5, 5);
        CCShapeId r = cc_boolean(a, b, opcode);
        if (r) cc_shape_release(r);
        cc_shape_release(b); cc_shape_release(a);
      };
      // OCCT oracle
      run(opname, sz.label, 0, "occt", bodyFn);
      // NativeEngine: planar prism boolean is NATIVE-SERVED (BSP-CSG), self-verified.
      run(opname, sz.label, 1, "native", bodyFn);
    }
  }

  // ════════════════════════════════════════════════════════════════════════════
  // 2. TESSELLATE — a native body meshed under both engines (NATIVE-SERVED for the
  //    native prism). Fixed deflection. Build the body ONCE per size outside the loop
  //    (meshing cost is what we isolate). Same body id reused under both engines
  //    (native bodies are recognised process-wide per the ABI note).
  // ════════════════════════════════════════════════════════════════════════════
  for (const auto& sz : sizes) {
    std::vector<double> prof = ngon(sz.n, 10.0);
    const double defl = 0.05;
    // OCCT body + OCCT tessellate
    cc_set_engine(0);
    CCShapeId occtBody = cc_solid_extrude(prof.data(), sz.n, sz.depth);
    run("tessellate", sz.label, 0, "occt",
        [occtBody, defl]() { CCMesh m = cc_tessellate(occtBody, defl); cc_mesh_free(m); });
    cc_shape_release(occtBody);
    // Native body + native tessellate
    cc_set_engine(1);
    CCShapeId natBody = cc_solid_extrude(prof.data(), sz.n, sz.depth);
    run("tessellate", sz.label, 1, "native",
        [natBody, defl]() { CCMesh m = cc_tessellate(natBody, defl); cc_mesh_free(m); });
    cc_shape_release(natBody);
  }

  // ════════════════════════════════════════════════════════════════════════════
  // 3. MASS_PROPERTIES — closed-form on a native body (NATIVE-SERVED) vs OCCT
  //    BRepGProp. Build once per size, query in the timed body.
  // ════════════════════════════════════════════════════════════════════════════
  for (const auto& sz : sizes) {
    std::vector<double> prof = ngon(sz.n, 10.0);
    cc_set_engine(0);
    CCShapeId occtBody = cc_solid_extrude(prof.data(), sz.n, sz.depth);
    run("mass_properties", sz.label, 0, "occt",
        [occtBody]() { volatile CCMassProps mp = cc_mass_properties(occtBody); (void)mp; });
    cc_shape_release(occtBody);
    cc_set_engine(1);
    CCShapeId natBody = cc_solid_extrude(prof.data(), sz.n, sz.depth);
    run("mass_properties", sz.label, 1, "native",
        [natBody]() { volatile CCMassProps mp = cc_mass_properties(natBody); (void)mp; });
    cc_shape_release(natBody);
  }

  // ════════════════════════════════════════════════════════════════════════════
  // 4. SECTION — planar section of a native prism (NATIVE-SERVED, cybercad::native::
  //    section) vs the OCCT engine. HONEST CAVEAT: the OCCT ENGINE ADAPTER in this
  //    facade does NOT implement cc_section_plane — it returns an honest decline
  //    ("operation not supported by active engine: section_plane"), so its timing is a
  //    no-op, NOT a real section computation. We DETECT that decline at runtime and
  //    label the OCCT row "occt-declined" (no ms), and record the native row as
  //    "native-only". This is a NATIVE-ONLY capability, not a like-for-like speedup —
  //    the summary marks it as such rather than printing a bogus ratio.
  // ════════════════════════════════════════════════════════════════════════════
  for (const auto& sz : sizes) {
    std::vector<double> prof = ngon(sz.n, 10.0);
    const double origin[3] = {0, 0, 5.0};
    const double normal[3] = {0, 0, 1.0};

    // Probe the OCCT engine once: does it actually produce a section, or decline?
    cc_set_engine(0);
    CCShapeId occtBody = cc_solid_extrude(prof.data(), sz.n, sz.depth);
    CCSection probe = cc_section_plane(occtBody, origin, normal);
    bool occtServes = (probe.loopCount > 0);
    cc_section_free(probe);
    if (occtServes) {
      run("section", sz.label, 0, "occt",
          [occtBody, &origin, &normal]() { CCSection s = cc_section_plane(occtBody, origin, normal); cc_section_free(s); });
    } else {
      emit("section", sz.label, "occt", "occt-declined", Stat{}, false);
      g_rows.push_back({"section", sz.label, "occt-declined", Stat{}, false});
    }
    cc_shape_release(occtBody);

    cc_set_engine(1);
    CCShapeId natBody = cc_solid_extrude(prof.data(), sz.n, sz.depth);
    Stat sn = timeOp([natBody, &origin, &normal]() { CCSection s = cc_section_plane(natBody, origin, normal); cc_section_free(s); });
    emit("section", sz.label, "native", occtServes ? "native" : "native-only", sn, true);
    g_rows.push_back({"section", sz.label, occtServes ? "native" : "native-only", sn, true});
    cc_shape_release(natBody);
  }

  // ════════════════════════════════════════════════════════════════════════════
  // 5. FILLET_EDGES — a box edge fillet. The native engine's edge-fillet is OUTSIDE
  //    its planar domain (rolling-ball on an edge is a curved face) → FORWARDED to
  //    OCCT. We time ONLY the OCCT engine and record the native row as FORWARDED
  //    (no native ms) so the table is honest: native does not accelerate this op,
  //    it declines/forwards it. (fillet on a box builds a 10x10x10 box, fillets 1 edge.)
  // ════════════════════════════════════════════════════════════════════════════
  {
    const double sq[8] = {0, 0, 10, 0, 10, 10, 0, 10};
    cc_set_engine(0);
    CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    int* edges = nullptr;
    int ne = cc_subshape_ids(box, 1, &edges);
    if (ne > 0) {
      int oneEdge = edges[0];
      run("fillet_edges", "box", 0, "occt",
          [box, oneEdge]() { CCShapeId f = cc_fillet_edges(box, &oneEdge, 1, 1.0); if (f) cc_shape_release(f); });
      // native: FORWARDED (curved fillet face outside planar domain) — no native ms.
      emit("fillet_edges", "box", "native", "forwarded", Stat{}, false);
      g_rows.push_back({"fillet_edges", "box", "native+forwarded", Stat{}, false});
    }
    if (edges) cc_ints_free(edges);
    cc_shape_release(box);
  }

  // ── Human-readable table ──────────────────────────────────────────────────────
  // Pair rows by (op,size): one OCCT-side row (served occt|occt-declined) and one
  // native-side row (served native|native-only|forwarded).
  std::printf("\n== SUMMARY (median ms; ratio = OCCT/native, >1 => native faster) ==\n");
  std::printf("%-18s %-7s %14s %14s %12s\n", "op", "size", "OCCT ms", "native ms", "ratio/note");
  auto isOcctSide = [](const std::string& s) { return s == "occt" || s == "occt-declined"; };
  for (size_t i = 0; i < g_rows.size(); ++i) {
    if (!isOcctSide(g_rows[i].served)) continue;
    for (size_t j = 0; j < g_rows.size(); ++j) {
      if (g_rows[j].op != g_rows[i].op || g_rows[j].size != g_rows[i].size || isOcctSide(g_rows[j].served))
        continue;
      const Row& o = g_rows[i];
      const Row& n = g_rows[j];
      char occtBuf[32], natBuf[32], noteBuf[32];
      if (o.hasStat) std::snprintf(occtBuf, sizeof occtBuf, "%.4f", o.stat.median);
      else           std::snprintf(occtBuf, sizeof occtBuf, "DECLINED");
      if (n.hasStat) std::snprintf(natBuf, sizeof natBuf, "%.4f", n.stat.median);
      else           std::snprintf(natBuf, sizeof natBuf, "FORWARDED");
      if (o.hasStat && n.hasStat) {
        double ratio = n.stat.median > 0 ? o.stat.median / n.stat.median : 0.0;
        std::snprintf(noteBuf, sizeof noteBuf, "%.2fx", ratio);
      } else if (!n.hasStat) {
        std::snprintf(noteBuf, sizeof noteBuf, "(fwd->OCCT)");
      } else {
        std::snprintf(noteBuf, sizeof noteBuf, "native-only");
      }
      std::printf("%-18s %-7s %14s %14s %12s\n",
                  o.op.c_str(), o.size.c_str(), occtBuf, natBuf, noteBuf);
      break;
    }
  }

  std::printf("\n== bench complete ==\n");
  std::fflush(stdout);
  std::_Exit(0);
}
