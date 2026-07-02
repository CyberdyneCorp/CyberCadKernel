// Full OCCT runtime suite for CyberCadKernel, run inside an iOS simulator
// (xcrun simctl spawn). Drives the cc_* facade through the real OCCT adapter and
// exercises ALL 57 cc_* entry points across six correctness modules plus a
// determinism/benchmark pass. Each module lives in its own checks_<module>.cpp
// and records results into the shared Ctx defined in checks.h.
//
// This file owns the single Ctx, runs the modules in order, prints the tally,
// and exits via std::_Exit: OCCT's trimmed static build crashes during static
// teardown after algorithm use (an OCCT-upstream quirk, harmless post-main — see
// docs/STATUS-phase-0-1.md), so we bypass C++ static destructors and report the
// suite's true pass/fail result.

#include "checks.h"

#include <cstdio>
#include <cstdlib>

int main() {
  Ctx ctx;

  std::printf("== CyberCadKernel OCCT full runtime suite ==\n");
  std::fflush(stdout);

  ctx.check(cc_brep_available() == 1, "cc_brep_available()==1 (OCCT linked)");
  if (!cc_brep_available()) {
    std::printf("OCCT not linked — aborting\n");
    std::fflush(stdout);
    std::_Exit(2);
  }

  run_construct_checks(ctx);
  run_feature_checks(ctx);
  run_booltransform_checks(ctx);
  run_tessellate_checks(ctx);
  run_query_checks(ctx);
  run_exchange_checks(ctx);
  run_accel_checks(ctx);

  std::printf("== %d passed, %d failed ==\n", ctx.passed, ctx.failed);
  std::fflush(stdout);

  // OCCT's own static teardown crashes at process exit after algorithm use in the
  // trimmed static build (not a CyberCadKernel defect — the facade's singletons
  // are intentionally leaked and every shape is released). Exit without running
  // C++ static destructors so the suite reports its true result.
  std::_Exit(ctx.failed == 0 ? 0 : 1);
}
