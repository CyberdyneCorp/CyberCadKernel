// Phase-3 feature suite for CyberCadKernel, run inside an iOS simulator
// (xcrun simctl spawn). Drives the five Phase-3 cc_* features through the real
// OCCT adapter. Each feature lives in its own checks_<feature>.cpp module and
// records into the shared Ctx defined in phase3_checks.h.
//
// This file owns the single Ctx, runs the modules in order, prints the tally as
// "== P passed, F failed, D deferred ==", and exits via std::_Exit: OCCT's trimmed
// static build crashes during static teardown after algorithm use (an OCCT-upstream
// quirk, harmless post-main — see docs/STATUS-phase-0-1.md), so we bypass C++
// static destructors and report the suite's true result. Exit code is 0 iff no
// hard failures (deferred cases do NOT fail the suite — they are honest fallbacks).

#include "phase3_checks.h"

#include <cstdio>
#include <cstdlib>

int main() {
  Ctx ctx;

  std::printf("== CyberCadKernel Phase-3 feature suite ==\n");
  std::fflush(stdout);

  ctx.check(cc_brep_available() == 1, "cc_brep_available()==1 (OCCT linked)");
  if (!cc_brep_available()) {
    std::printf("OCCT not linked — aborting\n");
    std::fflush(stdout);
    std::_Exit(2);
  }

  run_reference_geometry_checks(ctx);
  run_wrap_emboss_checks(ctx);
  run_thread_boolean_checks(ctx);
  run_full_round_fillet_checks(ctx);
  run_g2_fillet_checks(ctx);

  std::printf("== %d passed, %d failed, %d deferred ==\n", ctx.passed, ctx.failed, ctx.deferred);
  std::fflush(stdout);

  // See full_suite.cpp: exit without running C++ static destructors so OCCT's
  // trimmed-static teardown quirk does not mask the suite's true result.
  std::_Exit(ctx.failed == 0 ? 0 : 1);
}
