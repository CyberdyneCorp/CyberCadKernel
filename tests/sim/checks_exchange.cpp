// Data-exchange correctness module for the CyberCadKernel OCCT runtime suite.
//
// Exercises the STEP + IGES round-trip facade — cc_step_export/cc_step_import
// and cc_iges_export/cc_iges_import — against a body with known analytic
// properties (a 10x10x10 box: volume 1000 mm^3, surface area 600 mm^2, bounding
// box [0,0,0 .. 10,10,10]). For each format we:
//
//   1. export to /tmp and assert the writer reports success (return 1) AND the
//      file actually landed on disk non-empty,
//   2. re-import and assert the geometry survived the round-trip (STEP keeps the
//      solid, so volume+area+bbox are all checked; IGES BRep mode may downgrade a
//      solid to a shell, so bbox + surface area — both robust to that downgrade —
//      are the concrete invariant, and volume is reported as an informational
//      NOTE rather than a hard assertion),
//   3. exercise the guard paths (invalid body id, null path, missing file) and
//      assert each returns the documented failure value (0).
//
// Every CCShapeId created here is released; the export files are left in /tmp
// (harmless scratch) but truncated/removed before each write so a stale file
// can't mask a writer failure.

#include "checks.h"

#include <cstdio>
#include <string>
#include <sys/stat.h>

namespace {

// 10x10 square profile → extruded 10 deep gives a unit-known box.
const double kSquare[8] = {0, 0, 10, 0, 10, 10, 0, 10};

// File size in bytes, or -1 if the path does not exist / cannot be stat'd.
long fileSize(const char* path) {
  struct stat st{};
  if (::stat(path, &st) != 0) { return -1; }
  return static_cast<long>(st.st_size);
}

// True when a bbox out6 equals the source box [0,0,0 .. 10,10,10] within tol.
bool bboxIsUnitBox(const double* bb, double tol) {
  return near(bb[0], 0, tol) && near(bb[1], 0, tol) && near(bb[2], 0, tol) &&
         near(bb[3], 10, tol) && near(bb[4], 10, tol) && near(bb[5], 10, tol);
}

}  // namespace

void run_exchange_checks(Ctx& ctx) {
  // ── Source body: analytic 10x10x10 box ─────────────────────────────────────
  CCShapeId box = cc_solid_extrude(kSquare, 4, 10.0);
  if (!ctx.check(box != 0, "exchange: cc_solid_extrude -> box id")) { return; }

  CCMassProps src = cc_mass_properties(box);
  ctx.check(src.valid && near(src.volume, 1000.0, 1e-6) && near(src.area, 600.0, 1e-6),
            "exchange: source box volume==1000, area==600",
            "vol=" + std::to_string(src.volume) + " area=" + std::to_string(src.area));

  // ── STEP export ────────────────────────────────────────────────────────────
  const char* stepPath = "/tmp/cck_exchange.step";
  std::remove(stepPath);
  ctx.check(cc_step_export(box, stepPath) == 1, "cc_step_export -> 1");
  ctx.check(fileSize(stepPath) > 0, "STEP file written non-empty",
            "bytes=" + std::to_string(fileSize(stepPath)));

  // ── STEP import round-trip: solid survives, so all three are concrete ───────
  CCShapeId stepBody = cc_step_import(stepPath);
  ctx.check(stepBody != 0, "cc_step_import -> valid id");
  if (stepBody != 0) {
    CCMassProps mp = cc_mass_properties(stepBody);
    ctx.check(mp.valid && near(mp.volume, 1000.0, 1e-3),
              "STEP round-trip volume preserved (==1000)",
              "got " + std::to_string(mp.volume));
    ctx.check(mp.valid && near(mp.area, 600.0, 1e-3),
              "STEP round-trip area preserved (==600)",
              "got " + std::to_string(mp.area));
    double bb[6] = {0};
    ctx.check(cc_bounding_box(stepBody, bb) == 1 && bboxIsUnitBox(bb, 1e-6),
              "STEP round-trip bbox preserved (==[0,0,0..10,10,10])");
    cc_shape_release(stepBody);
  }

  // ── STEP guard paths ───────────────────────────────────────────────────────
  ctx.check(cc_step_export(0, stepPath) == 0, "cc_step_export(invalid body) -> 0");
  ctx.check(cc_step_export(box, nullptr) == 0, "cc_step_export(null path) -> 0");
  ctx.check(cc_step_import(nullptr) == 0, "cc_step_import(null path) -> 0");
  ctx.check(cc_step_import("/tmp/cck_exchange_missing.step") == 0,
            "cc_step_import(missing file) -> 0");

  // ── IGES export ────────────────────────────────────────────────────────────
  const char* igesPath = "/tmp/cck_exchange.iges";
  std::remove(igesPath);
  ctx.check(cc_iges_export(box, igesPath) == 1, "cc_iges_export -> 1");
  ctx.check(fileSize(igesPath) > 0, "IGES file written non-empty",
            "bytes=" + std::to_string(fileSize(igesPath)));

  // ── IGES import round-trip ─────────────────────────────────────────────────
  // IGES BRep mode may degrade a solid to a shell (documented in KernelBridge),
  // so volume is not a reliable invariant. The bounding box and total surface
  // area survive that degradation and are asserted as the concrete check; the
  // volume is logged as a NOTE so a downgrade is visible without failing.
  CCShapeId igesBody = cc_iges_import(igesPath);
  ctx.check(igesBody != 0, "cc_iges_import -> valid id");
  if (igesBody != 0) {
    double bb[6] = {0};
    ctx.check(cc_bounding_box(igesBody, bb) == 1 && bboxIsUnitBox(bb, 1e-4),
              "IGES round-trip bbox preserved (==[0,0,0..10,10,10])");
    CCMassProps mp = cc_mass_properties(igesBody);
    ctx.check(mp.valid && near(mp.area, 600.0, 1e-2),
              "IGES round-trip surface area preserved (==600)",
              "got " + std::to_string(mp.area));
    std::printf("[NOTE] IGES round-trip volume = %.6f (1000 if solid preserved, "
                "0/degraded if downgraded to a shell)\n", mp.volume);
    std::fflush(stdout);
    cc_shape_release(igesBody);
  }

  // ── IGES guard paths ───────────────────────────────────────────────────────
  ctx.check(cc_iges_export(0, igesPath) == 0, "cc_iges_export(invalid body) -> 0");
  ctx.check(cc_iges_export(box, nullptr) == 0, "cc_iges_export(null path) -> 0");
  ctx.check(cc_iges_import(nullptr) == 0, "cc_iges_import(null path) -> 0");
  ctx.check(cc_iges_import("/tmp/cck_exchange_missing.iges") == 0,
            "cc_iges_import(missing file) -> 0");

  cc_shape_release(box);
}
