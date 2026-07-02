// Booleans + rigid/affine transforms module of the OCCT runtime suite.
//
// Exercises the cc_* boolean and placement facade against ANALYTIC invariants:
//   cc_boolean            — fuse/cut/common volumes equal the set-algebra result
//   cc_scale_shape        — uniform scale multiplies volume by factor^3, moves bbox
//   cc_scale_shape_about  — same, but the scale centre is a fixed point
//   cc_rotate_shape_about — rigid rotation preserves volume, rotates the bbox
//   cc_mirror_shape       — reflection preserves volume, mirrors the bbox
//   cc_translate_shape    — translation preserves volume, shifts bbox + centroid
//   cc_place_on_frame     — rigid frame placement preserves volume, relocates bbox
//
// Every function additionally asserts its guard: a deliberately degenerate input
// (unknown id / non-positive factor / zero axis / zero normal / degenerate frame)
// must return 0. Every CCShapeId built here is released before returning.

#include "checks.h"

#include <string>

namespace {

// 10x10 square profile at the origin; extruded 10 in +Z gives a 10^3 box.
constexpr double kSquare[8] = {0, 0, 10, 0, 10, 10, 0, 10};

std::string s(double v) { return std::to_string(v); }

// Build the canonical 10x10x10 axis-aligned box (volume 1000, occupies [0,10]^3).
CCShapeId makeBox() { return cc_solid_extrude(kSquare, 4, 10.0); }

// Volume of a body, or -1 when the id is invalid / mass props unavailable.
double volumeOf(CCShapeId id) {
  CCMassProps mp = cc_mass_properties(id);
  return mp.valid ? mp.volume : -1.0;
}

// Assert a body's exact B-rep bbox equals [x0,y0,z0,x1,y1,z1] (tol 1e-6).
bool bboxEquals(CCShapeId id, double x0, double y0, double z0,
                double x1, double y1, double z1) {
  double b[6] = {0};
  if (cc_bounding_box(id, b) != 1) return false;
  const double t = 1e-6;
  return near(b[0], x0, t) && near(b[1], y0, t) && near(b[2], z0, t) &&
         near(b[3], x1, t) && near(b[4], y1, t) && near(b[5], z1, t);
}

std::string bboxStr(CCShapeId id) {
  double b[6] = {0};
  if (cc_bounding_box(id, b) != 1) return "<no bbox>";
  return "[" + s(b[0]) + "," + s(b[1]) + "," + s(b[2]) + " .. " + s(b[3]) + "," +
         s(b[4]) + "," + s(b[5]) + "]";
}

// ── cc_boolean ─────────────────────────────────────────────────────────────
// Two unit-1000 boxes, the second shifted by (5,5,5): they overlap in a 5^3 cube
// (volume 125). Set algebra: fuse = 1000+1000-125 = 1875, cut = 1000-125 = 875,
// common = 125. Also assert the guard: an unknown operand id yields 0.
void checkBoolean(Ctx& ctx) {
  CCShapeId a = makeBox();
  CCShapeId b0 = makeBox();
  CCShapeId b = cc_translate_shape(b0, 5, 5, 5);
  cc_shape_release(b0);
  if (!ctx.check(a != 0 && b != 0, "cc_boolean setup: two boxes built")) {
    cc_shape_release(a);
    cc_shape_release(b);
    return;
  }

  CCShapeId fuse = cc_boolean(a, b, 0);
  CCShapeId cut = cc_boolean(a, b, 1);
  CCShapeId common = cc_boolean(a, b, 2);

  ctx.check(near(volumeOf(fuse), 1875.0, 1e-4),
            "cc_boolean fuse volume == 1875 (1000+1000-125)", s(volumeOf(fuse)));
  ctx.check(near(volumeOf(cut), 875.0, 1e-4),
            "cc_boolean cut volume == 875 (1000-125)", s(volumeOf(cut)));
  ctx.check(near(volumeOf(common), 125.0, 1e-4),
            "cc_boolean common volume == 125 (5^3 overlap)", s(volumeOf(common)));

  // Guard: an unknown operand id must return 0 (no shape produced).
  ctx.check(cc_boolean(a, 999999, 0) == 0,
            "cc_boolean(unknown id) == 0 (guard)");

  cc_shape_release(fuse);
  cc_shape_release(cut);
  cc_shape_release(common);
  cc_shape_release(a);
  cc_shape_release(b);
}

// ── cc_scale_shape ─────────────────────────────────────────────────────────
// Scale the [0,10]^3 box by 2 about the origin: volume 1000 -> 8000 (2^3), bbox
// [0,0,0 .. 20,20,20]. Guard: factor <= 0 must return 0.
void checkScale(Ctx& ctx) {
  CCShapeId box = makeBox();
  CCShapeId sc = cc_scale_shape(box, 2.0);
  ctx.check(sc != 0, "cc_scale_shape -> valid id");
  ctx.check(near(volumeOf(sc), 8000.0, 1e-3),
            "cc_scale_shape(x2) volume == 8000 (1000*2^3)", s(volumeOf(sc)));
  ctx.check(bboxEquals(sc, 0, 0, 0, 20, 20, 20),
            "cc_scale_shape(x2) bbox == [0..20]^3", bboxStr(sc));

  ctx.check(cc_scale_shape(box, 0.0) == 0,
            "cc_scale_shape(factor<=0) == 0 (guard)");

  cc_shape_release(sc);
  cc_shape_release(box);
}

// ── cc_scale_shape_about ───────────────────────────────────────────────────
// Scale by 2 about the box centre (5,5,5): volume -> 8000, centroid unchanged at
// (5,5,5), and each corner moves twice as far from (5,5,5) -> bbox [-5..15]^3.
// Guard: factor <= 0 must return 0.
void checkScaleAbout(Ctx& ctx) {
  CCShapeId box = makeBox();
  CCShapeId sc = cc_scale_shape_about(box, 5, 5, 5, 2.0);
  ctx.check(sc != 0, "cc_scale_shape_about -> valid id");
  ctx.check(near(volumeOf(sc), 8000.0, 1e-3),
            "cc_scale_shape_about(x2) volume == 8000", s(volumeOf(sc)));
  ctx.check(bboxEquals(sc, -5, -5, -5, 15, 15, 15),
            "cc_scale_shape_about centre (5,5,5) bbox == [-5..15]^3", bboxStr(sc));
  CCMassProps mp = cc_mass_properties(sc);
  ctx.check(mp.valid && near(mp.cx, 5, 1e-6) && near(mp.cy, 5, 1e-6) &&
                near(mp.cz, 5, 1e-6),
            "cc_scale_shape_about centroid fixed at (5,5,5)");

  ctx.check(cc_scale_shape_about(box, 5, 5, 5, -1.0) == 0,
            "cc_scale_shape_about(factor<=0) == 0 (guard)");

  cc_shape_release(sc);
  cc_shape_release(box);
}

// ── cc_rotate_shape_about ──────────────────────────────────────────────────
// Rotate the [0,10]^3 box 90 deg about the +Z axis through the origin: a rigid
// motion preserves volume (1000). Under (x,y)->(-y,x) the corners span x in
// [-10,0], y in [0,10], z unchanged -> bbox [-10,0,0 .. 0,10,10]. Guard: a zero
// axis must return 0.
void checkRotate(Ctx& ctx) {
  CCShapeId box = makeBox();
  const double kHalfPi = 1.57079632679489661923;
  CCShapeId r = cc_rotate_shape_about(box, 0, 0, 0, 0, 0, 1, kHalfPi);
  ctx.check(r != 0, "cc_rotate_shape_about -> valid id");
  ctx.check(near(volumeOf(r), 1000.0, 1e-4),
            "cc_rotate_shape_about preserves volume == 1000", s(volumeOf(r)));
  ctx.check(bboxEquals(r, -10, 0, 0, 0, 10, 10),
            "cc_rotate_shape_about(90 deg Z@origin) bbox == [-10,0,0 .. 0,10,10]",
            bboxStr(r));

  ctx.check(cc_rotate_shape_about(box, 0, 0, 0, 0, 0, 0, kHalfPi) == 0,
            "cc_rotate_shape_about(zero axis) == 0 (guard)");

  cc_shape_release(r);
  cc_shape_release(box);
}

// ── cc_mirror_shape ────────────────────────────────────────────────────────
// Mirror the [0,10]^3 box across the plane x=0 (point origin, normal +X): a
// reflection preserves volume (1000) and maps x in [0,10] to [-10,0] ->
// bbox [-10,0,0 .. 0,10,10]. Guard: a zero normal must return 0.
void checkMirror(Ctx& ctx) {
  CCShapeId box = makeBox();
  CCShapeId m = cc_mirror_shape(box, 0, 0, 0, 1, 0, 0);
  ctx.check(m != 0, "cc_mirror_shape -> valid id");
  ctx.check(near(volumeOf(m), 1000.0, 1e-4),
            "cc_mirror_shape preserves volume == 1000", s(volumeOf(m)));
  ctx.check(bboxEquals(m, -10, 0, 0, 0, 10, 10),
            "cc_mirror_shape(plane x=0) bbox == [-10,0,0 .. 0,10,10]", bboxStr(m));

  ctx.check(cc_mirror_shape(box, 0, 0, 0, 0, 0, 0) == 0,
            "cc_mirror_shape(zero normal) == 0 (guard)");

  cc_shape_release(m);
  cc_shape_release(box);
}

// ── cc_translate_shape ─────────────────────────────────────────────────────
// Translate the [0,10]^3 box by (5,5,5): volume preserved (1000), bbox shifts to
// [5..15]^3, centroid moves from (5,5,5) to (10,10,10). Guard: unknown id -> 0.
void checkTranslate(Ctx& ctx) {
  CCShapeId box = makeBox();
  CCShapeId t = cc_translate_shape(box, 5, 5, 5);
  ctx.check(t != 0, "cc_translate_shape -> valid id");
  ctx.check(near(volumeOf(t), 1000.0, 1e-4),
            "cc_translate_shape preserves volume == 1000", s(volumeOf(t)));
  ctx.check(bboxEquals(t, 5, 5, 5, 15, 15, 15),
            "cc_translate_shape(+5,+5,+5) bbox == [5..15]^3", bboxStr(t));
  CCMassProps mp = cc_mass_properties(t);
  ctx.check(mp.valid && near(mp.cx, 10, 1e-6) && near(mp.cy, 10, 1e-6) &&
                near(mp.cz, 10, 1e-6),
            "cc_translate_shape centroid == (10,10,10)");

  ctx.check(cc_translate_shape(999999, 1, 2, 3) == 0,
            "cc_translate_shape(unknown id) == 0 (guard)");

  cc_shape_release(t);
  cc_shape_release(box);
}

// ── cc_place_on_frame ──────────────────────────────────────────────────────
// Place the [0,10]^3 box onto the frame at origin (10,0,0) with u=+Y, v=+Z. The
// rigid motion maps the global XOY frame (x-dir=u, normal=u×v=+X, y-dir=v) so a
// local point (x,y,z) -> (10+z, x, y). Hence the box maps to X in [10,20],
// Y in [0,10], Z in [0,10] with volume preserved (1000). Guard: u parallel to v
// (u×v = 0) must return 0.
void checkPlaceOnFrame(Ctx& ctx) {
  CCShapeId box = makeBox();
  CCShapeId p = cc_place_on_frame(box, 10, 0, 0, 0, 1, 0, 0, 0, 1);
  ctx.check(p != 0, "cc_place_on_frame -> valid id");
  ctx.check(near(volumeOf(p), 1000.0, 1e-4),
            "cc_place_on_frame preserves volume == 1000", s(volumeOf(p)));
  ctx.check(bboxEquals(p, 10, 0, 0, 20, 10, 10),
            "cc_place_on_frame(o=(10,0,0),u=+Y,v=+Z) bbox == [10,0,0 .. 20,10,10]",
            bboxStr(p));

  ctx.check(cc_place_on_frame(box, 0, 0, 0, 0, 1, 0, 0, 1, 0) == 0,
            "cc_place_on_frame(u parallel v) == 0 (guard)");

  cc_shape_release(p);
  cc_shape_release(box);
}

}  // namespace

void run_booltransform_checks(Ctx& ctx) {
  std::printf("-- booltransform: cc_boolean + scale/rotate/mirror/translate/place_on_frame --\n");
  std::fflush(stdout);
  checkBoolean(ctx);
  checkScale(ctx);
  checkScaleAbout(ctx);
  checkRotate(ctx);
  checkMirror(ctx);
  checkTranslate(ctx);
  checkPlaceOnFrame(ctx);
}
