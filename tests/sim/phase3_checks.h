// Shared harness for the CyberCadKernel Phase-3 feature suite (iOS simulator).
//
// Phase-3 adds five NATIVE features behind the cc_* facade — reference geometry,
// robust wrap-emboss, robust thread boolean, full-round fillet, and G2 blend
// fillet. Each is exercised by its own checks_<feature>.cpp module against the real
// OCCT adapter; phase3_suite.cpp owns the single Ctx, runs the modules, prints the
// tally, and exits.
//
// This harness is SEPARATE from tests/sim/checks.h on purpose: Phase-3 tracks a
// DEFERRED count alongside passed/failed, because several features are research-
// grade and are allowed to fall back to a valid lower-fidelity result and record
// the case as deferred (with the measured reason) rather than fake a pass. The
// honesty rule is enforced here: defer() is the ONLY way to acknowledge an
// unmet-but-not-failed bar; check() must assert a REAL geometric property.
//
// Everything is inline/header-only so every module TU can include it without an
// ODR/link clash. It must compile standalone (pulls in cc_kernel.h + std bits).

#ifndef CYBERCADKERNEL_TESTS_SIM_PHASE3_CHECKS_H
#define CYBERCADKERNEL_TESTS_SIM_PHASE3_CHECKS_H

#include "cybercadkernel/cc_kernel.h"

#include <cmath>
#include <cstdio>
#include <string>

// ── Result context ──────────────────────────────────────────────────────────
// One instance lives in main(); passed/failed/deferred accumulate across every
// module. check() records a hard assertion of a REAL property; defer() records a
// bar that was NOT met but fell back to a valid result — it is neither a pass nor
// a failure, and MUST carry the measured reason. Never call check(true, ...) to
// paper over an unmet bar; use defer().
struct Ctx {
  int passed = 0;
  int failed = 0;
  int deferred = 0;

  // Assert a real property. `what` is the human-readable claim; `detail` is
  // optional measured context appended after an em dash. Returns `ok` so callers
  // can early-out on a failed precondition.
  bool check(bool ok, const std::string& what, const std::string& detail = "") {
    std::printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(),
                detail.empty() ? "" : " — ", detail.c_str());
    if (ok) ++passed; else ++failed;
    std::fflush(stdout);
    return ok;
  }

  // Record a deferred case: the full-fidelity bar was not achieved, a valid
  // fallback (or no result) was returned, and `reason` is the MEASURED reason
  // (e.g. the tangency/curvature gap, elapsed time, or validity). This is NOT a
  // pass and NOT a failure — it is an honest "deferred, not faked".
  void defer(const std::string& what, const std::string& reason) {
    std::printf("[DEFER] %s — %s\n", what.c_str(), reason.c_str());
    ++deferred;
    std::fflush(stdout);
  }
};

// ── Numeric tolerance helper ────────────────────────────────────────────────
// True when |a - b| <= tol. Use for volumes/areas/coordinates vs analytic values.
inline bool near(double a, double b, double tol) {
  return std::fabs(a - b) <= tol;
}

// ── Per-feature entry points ─────────────────────────────────────────────────
// Each is defined in its own checks_<feature>.cpp (owned by that feature's agent).
// phase3_suite.cpp calls them in this order.
void run_reference_geometry_checks(Ctx&);  // cc_ref_plane_*/cc_ref_axis_*
void run_wrap_emboss_checks(Ctx&);         // cc_wrap_emboss (robust sewn pad)
void run_thread_boolean_checks(Ctx&);      // cc_thread_apply
void run_full_round_fillet_checks(Ctx&);   // cc_full_round_fillet(_faces)
void run_g2_fillet_checks(Ctx&);           // cc_fillet_edges_g2

#endif  // CYBERCADKERNEL_TESTS_SIM_PHASE3_CHECKS_H
