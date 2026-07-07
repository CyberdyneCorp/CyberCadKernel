// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2b / B2 (freeform face-split) — the ANALYTIC TILING proof,
// OCCT-FREE. On the ONE reachable fixture (a convex-quad-trimmed Bézier bowl cut by
// the plane x=0, seam = the real M1 WLine) we assert the "TILE the original" contract:
//   * exactly one clean entry + one clean exit crossing (crossings == 2);
//   * the two sub-loops' UV areas SUM to the parent's within machine precision;
//   * the seam is the EXACT (bit-identical, opposite-order) shared boundary of both;
//   * two genuinely-trimmed sub-faces are emitted (non-null Faces over the bowl).
// And the honest-decline envelope:
//   * a seam that misses / over-crosses the loop returns NULL with a measured blocker;
//   * an empty seam declines.
// Requires CYBERCAD_HAS_NUMSCI (the fixture's seam is the real S3 trace).
//
#include "native/boolean/face_split.h"
#include "native/tessellate/face_mesher.h"

#include "native/face_split_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace fx = face_split_fixture;
namespace ssi = cybercad::native::ssi;

namespace {

// Shoelace |area| of a UV loop (independent re-derivation of the gate's own area).
double loopArea(const bo::UVPolygon& p) {
  double a = 0.0;
  const std::size_t n = p.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) a += p[j].u * p[i].v - p[i].u * p[j].v;
  return std::fabs(0.5 * a);
}

}  // namespace

// ── The primary tiling gate on the real M1 seam ───────────────────────────────
CC_TEST(face_split_tiles_parent_on_real_seam) {
  const cybercad::native::topology::Shape face = fx::parentFace();
  const ssi::WLine seam = fx::seamWLine();
  CC_CHECK(seam.points.size() >= 2);           // the real S3 trace produced a seam
  CC_CHECK(seam.status == ssi::TraceStatus::BoundaryExit);

  const bo::SplitResult r = bo::splitFace(face, seam);
  CC_CHECK(r.ok());
  CC_CHECK(r.decline == bo::SplitDecline::Ok);
  CC_CHECK(r.crossings == 2);                    // one clean entry + one clean exit
  if (!r.ok()) return;

  const bo::FaceSplit& s = *r.split;

  // Tiling identity to machine precision (scale-relative), independently re-derived.
  const double aIn = loopArea(s.loopIn);
  const double aOut = loopArea(s.loopOut);
  const double gap = std::fabs(s.parentArea - (aIn + aOut));
  CC_CHECK(gap <= s.parentArea * 1e-12);         // areas sum to the parent
  CC_CHECK(std::fabs(aIn - s.areaIn) <= s.parentArea * 1e-12);
  CC_CHECK(std::fabs(aOut - s.areaOut) <= s.parentArea * 1e-12);
  CC_CHECK(s.areaIn > 0.0 && s.areaOut > 0.0);
  CC_CHECK(r.tilingGap <= s.parentArea * 1e-12);

  // Exact shared seam: every seam-chord vertex appears BIT-IDENTICALLY in both
  // sub-loops (the seam is their common boundary). Independent membership re-check.
  const std::size_t m = s.seam.size();
  CC_CHECK(m >= 2);
  auto contains = [](const bo::UVPolygon& loop, const bo::UV& q) {
    for (const bo::UV& p : loop)
      if (p.u == q.u && p.v == q.v) return true;
    return false;
  };
  bool seamShared = (m >= 2);
  for (const bo::UV& q : s.seam)
    if (!contains(s.loopIn, q) || !contains(s.loopOut, q)) seamShared = false;
  CC_CHECK(seamShared);

  // Two genuinely-trimmed sub-faces exist over the bowl surface.
  CC_CHECK(!s.faceIn.isNull() && !s.faceOut.isNull());
  CC_CHECK(s.faceIn.type() == cybercad::native::topology::ShapeType::Face);
  CC_CHECK(s.faceOut.type() == cybercad::native::topology::ShapeType::Face);
}

// ── The two sub-faces MESH via M0 and their meshed areas TILE the parent ──────
// A native-only (no OCCT) pre-image of the sim gate: each rebuilt sub-face routes
// through the M0 trimmedFreeform path and produces a non-empty mesh, and the two
// meshed surface areas sum to the parent's meshed surface area (the curved bowl
// area over the quad, > the flat UV area) within the deflection tolerance.
CC_TEST(face_split_subfaces_mesh_and_tile) {
  namespace tess = cybercad::native::tessellate;
  const cybercad::native::topology::Shape face = fx::parentFace();
  const ssi::WLine seam = fx::seamWLine();
  const bo::SplitResult r = bo::splitFace(face, seam);
  CC_CHECK(r.ok());
  if (!r.ok()) return;
  const bo::FaceSplit& s = *r.split;

  tess::MeshParams mp;
  mp.deflection = 1e-3;
  const tess::FaceMesher fm(mp);
  const tess::Mesh mParent = fm.mesh(face);
  const tess::Mesh mIn = fm.mesh(s.faceIn);
  const tess::Mesh mOut = fm.mesh(s.faceOut);
  CC_CHECK(!mIn.triangles.empty());       // each sub-face genuinely meshes (M0, unchanged)
  CC_CHECK(!mOut.triangles.empty());

  const double aParent = tess::surfaceArea(mParent);
  const double aIn = tess::surfaceArea(mIn);
  const double aOut = tess::surfaceArea(mOut);
  CC_CHECK(aParent > 0.0);
  const double rel = std::fabs(aParent - (aIn + aOut)) / aParent;
  CC_CHECK(rel < 1e-3);                    // the mesh tiling holds (measured ≈ 2.5e-5)
}

// ── Honest decline: a seam that MISSES the trimmed loop (0 crossings) → NULL ───
CC_TEST(face_split_declines_when_seam_misses) {
  const cybercad::native::topology::Shape face = fx::parentFace();
  ssi::WLine seam;                                 // synthetic seam far outside the quad
  for (int i = 0; i <= 10; ++i) {
    ssi::WLinePoint p;
    p.u1 = 1.6;                                     // u well beyond the [0.15,0.85] quad span
    p.v1 = 0.05 + 0.09 * i;
    seam.points.push_back(p);
  }
  const bo::SplitResult r = bo::splitFace(face, seam);
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::SplitDecline::CrossingsNot2);
  CC_CHECK(r.crossings == 0);                       // measured blocker
}

// ── Honest decline: a seam that OVER-crosses (enters/exits twice) → NULL ───────
CC_TEST(face_split_declines_on_reentry) {
  const cybercad::native::topology::Shape face = fx::parentFace();
  // A path that crosses the convex quad FOUR times: up the left third (pierces
  // bottom + top), over the top outside, down the right third (pierces top +
  // bottom). Two entries + two exits ⇒ beyond the first slice ⇒ decline.
  ssi::WLine seam;
  auto add = [&](double u, double v) {
    ssi::WLinePoint p; p.u1 = u; p.v1 = v; seam.points.push_back(p);
  };
  add(0.35, 0.05); add(0.35, 0.55); add(0.35, 0.95);   // left leg: crosses bottom + top
  add(0.50, 0.98);                                      // over the top (outside)
  add(0.65, 0.95); add(0.65, 0.55); add(0.65, 0.05);   // right leg: crosses top + bottom
  const bo::SplitResult r = bo::splitFace(face, seam);
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::SplitDecline::CrossingsNot2);
  CC_CHECK(r.crossings == 4);                           // measured blocker
}

// ── Honest decline: an empty seam → NULL ──────────────────────────────────────
CC_TEST(face_split_declines_on_empty_seam) {
  const cybercad::native::topology::Shape face = fx::parentFace();
  const bo::SplitResult r = bo::splitFace(face, ssi::WLine{});
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::SplitDecline::EmptySeam);
}

int main() { return cctest::run_all(); }
