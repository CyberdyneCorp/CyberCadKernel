// SPDX-License-Identifier: Apache-2.0
//
// native_sheetmetal_selftest.mm — SIM Gate (b) for the MOAT M-SM sheet-metal first
// slice. On a booted iOS simulator, through the SHIPPING cc_* facade under the NATIVE
// engine (cc_set_engine(1)), this harness verifies the three sheet-metal ops.
//
// ── THERE IS NO OCCT SHEET-METAL ORACLE ───────────────────────────────────────
// OCCT core has NO sheet-metal module, so — unlike the other native parity harnesses —
// this one does NOT compare against OCCT and does NOT link OCCT. The ARBITER is CLOSED
// FORM: the built parts must pass cc_check_solid (a valid closed 2-manifold) and their
// cc_mass_properties volume must match the analytic closed form. The harness also
// asserts DETERMINISM (byte-identical volume on repeat) and the unfold AREA INVARIANT.
//
// Checks (native engine):
//   A) base flange (40×20, t=2)      → cc_check_solid valid; volume = 40·20·2 = 1600.
//   B) edge flange (90°, r=3, h=15)  → cc_check_solid valid; volume =
//        base + ½θ((r+t)²−r²)·W + h·t·W (bend meshes to deflection → within 1%).
//   C) unfold (k=0.44)               → cc_check_solid valid; flat blank volume =
//        (baseRun + BA + h)·W·t with BA = θ(r + k·t); AREA-INVARIANT vs the fold.
//   D) determinism                   → building the same fold twice gives the same volume.
//
// Output: [SM] PASS/FAIL lines, then a summary. On run-sim-suite.sh's SKIP list (own
// main()); its dedicated runner is scripts/run-sim-native-sheetmetal.sh. Because the
// ops are native-only and OCCT-free, the runner links NO OCCT toolkit.

#include "cybercadkernel/cc_kernel.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

constexpr double kPi = 3.14159265358979323846;
int g_pass = 0, g_fail = 0;

void check(bool ok, const char* what, const char* detail = "") {
  std::printf("[SM] %s %s%s%s\n", ok ? "PASS" : "FAIL", what, detail[0] ? " — " : "", detail);
  if (ok) ++g_pass; else ++g_fail;
}

}  // namespace

int main() {
  cc_set_engine(1);  // NativeEngine — sheet metal is native-only.

  // ── A) BASE FLANGE ────────────────────────────────────────────────────────────
  const double L = 40, W = 20, t = 2;
  const double rect[8] = {0, 0, L, 0, L, W, 0, W};
  CCShapeId base = cc_sheet_base_flange(rect, 4, t);
  check(base != 0, "base_flange builds", base ? "" : cc_last_error());
  if (base) {
    CCValidityReport vr{};
    const int ok = cc_check_solid(base, &vr);
    check(ok == 1 && vr.valid == 1, "base_flange cc_check_solid valid");
    CCMassProps mp = cc_mass_properties(base);
    const double expected = L * W * t;
    check(mp.valid && std::fabs(mp.volume - expected) < 1e-6, "base_flange volume = area·t");
  }

  // ── B) EDGE FLANGE ──────────────────────────────────────────────────────────--
  // Probe the subshape edge ids for the flangeable rim: the first id that yields a
  // valid 90° flange is the +X rim (a stable, deterministic id under the native engine).
  const double r = 3, h = 15, thDeg = 90.0, th = kPi / 2;
  CCShapeId folded = 0;
  int rimId = -1;
  for (int id = 1; id <= 32 && folded == 0; ++id) {
    CCShapeId f = cc_sheet_edge_flange(base, id, h, r, thDeg);
    if (f != 0) { folded = f; rimId = id; }
  }
  check(folded != 0, "edge_flange builds off a straight rim",
        folded ? "" : cc_last_error());
  double foldedVol = 0.0;
  if (folded) {
    std::printf("[SM] .... rim edge id = %d\n", rimId);
    CCValidityReport vr{};
    const int ok = cc_check_solid(folded, &vr);
    check(ok == 1 && vr.valid == 1, "edge_flange cc_check_solid valid");
    CCMassProps mp = cc_mass_properties(folded);
    const double ro = r + t;
    const double expected = L * W * t + 0.5 * th * (ro * ro - r * r) * W + h * t * W;
    foldedVol = mp.volume;
    check(mp.valid && mp.volume <= expected + 1e-6 &&
              std::fabs(mp.volume - expected) < 0.01 * expected,
          "edge_flange volume = closed form (converges from below)");
  }

  // ── C) UNFOLD ───────────────────────────────────────────────────────────────--
  if (folded) {
    const double k = 0.44;
    CCShapeId blank = cc_sheet_unfold(folded, k);
    check(blank != 0, "unfold builds the flat blank", blank ? "" : cc_last_error());
    if (blank) {
      CCValidityReport vr{};
      const int ok = cc_check_solid(blank, &vr);
      check(ok == 1 && vr.valid == 1, "unfold cc_check_solid valid");
      CCMassProps mp = cc_mass_properties(blank);
      const double BA = th * (r + k * t);
      const double devArea = L * W + BA * W + h * W;  // base + bend developed × W + flange
      check(mp.valid && std::fabs(mp.volume - devArea * t) < 1e-6,
            "unfold flat-blank volume = developed area · t (area invariant)");
    }
  }

  // ── D) DETERMINISM ───────────────────────────────────────────────────────────
  if (folded && rimId > 0) {
    CCShapeId again = cc_sheet_edge_flange(base, rimId, h, r, thDeg);
    if (again) {
      CCMassProps mp = cc_mass_properties(again);
      check(std::fabs(mp.volume - foldedVol) < 1e-12, "edge_flange is deterministic");
    } else {
      check(false, "edge_flange re-build (determinism)");
    }
  }

  // ── E) HONEST DECLINE (native-only, no OCCT forward) ──────────────────────────
  {
    CCShapeId d = cc_sheet_edge_flange(base, rimId > 0 ? rimId : 1, h, r, 0.0);  // angle 0
    check(d == 0, "edge_flange honest-declines a degenerate angle (no OCCT forward)");
  }

  std::printf("[SM] --- %d passed, %d failed ---\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
