// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2b / B2 SMOOTH-TRIM (closed / circular seam) generalisation
// — the ANALYTIC TILING proof, OCCT-FREE. On the reachable fixture (a quad-trimmed
// Bézier bowl cut by the HORIZONTAL plane z = c, seam = the real M1 WLine, a CLOSED
// CIRCLE interior to the quad) we assert the "TILE the parent" contract:
//   * the seam is a CLOSED interior loop (crossings == 0), every node on the circle
//     of radius ρ = √(c/a) (closed-form geometry, tracer-tolerance);
//   * splitFaceSmoothTrim → the enclosed disk + the annulus (the seam as a HOLE);
//   * the two sub-loops' UV areas SUM to the parent's to machine precision;
//   * the disk area equals the closed-form π·ρ² (inscribed-polygon band);
//   * two genuinely-trimmed sub-faces mesh (M0, unchanged) and their meshed areas TILE
//     the parent's at MULTIPLE deflections.
// And the byte-freeze contrast + the honest-decline envelope:
//   * byte-frozen B2 `splitFace` STILL DECLINES the same closed seam (CrossingsNot2,
//     crossings == 0) — the generalisation is strictly additive;
//   * an OPEN chord (crosses the boundary), a self-intersecting loop, and a too-short
//     seam each DECLINE with the measured blocker.
// Requires CYBERCAD_HAS_NUMSCI (the fixture's seam is the real S3 trace).
//
#include "native/boolean/face_split.h"
#include "native/boolean/smooth_trim_split.h"
#include "native/tessellate/face_mesher.h"

#include "native/smooth_trim_split_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace fx = face_split_fixture;
namespace stx = smooth_trim_split_fixture;
namespace ssi = cybercad::native::ssi;
namespace topo = cybercad::native::topology;

namespace {

double loopArea(const bo::UVPolygon& p) {
  double a = 0.0;
  const std::size_t n = p.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) a += p[j].u * p[i].v - p[i].u * p[j].v;
  return std::fabs(0.5 * a);
}

}  // namespace

// ── The seam really is a CLOSED CIRCULAR interior loop (closed-form geometry) ──
CC_TEST(smooth_trim_seam_is_closed_circle_on_surface) {
  const ssi::WLine seam = stx::closedSeamWLine();
  CC_CHECK(seam.points.size() >= 8);            // a resolved circle, not a stub
  CC_CHECK(seam.status == ssi::TraceStatus::Closed);
  // Every node lies on the circle of radius ρ centred at (½,½) in the bowl's (u,v).
  double maxRadErr = 0.0;
  for (const ssi::WLinePoint& p : seam.points) {
    const double r = std::hypot(p.u1 - 0.5, p.v1 - 0.5);
    maxRadErr = std::max(maxRadErr, std::fabs(r - stx::kRho));
  }
  CC_CHECK(maxRadErr < 1e-3);                   // on the closed-form circle (tracer tol)
}

// ── Byte-freeze contrast: B2 splitFace STILL declines the closed seam ─────────
CC_TEST(smooth_trim_b2_splitface_still_declines_closed_seam) {
  const topo::Shape face = fx::parentFace();
  const ssi::WLine seam = stx::closedSeamWLine();
  const bo::SplitResult r = bo::splitFace(face, seam);
  CC_CHECK(!r.ok());                            // the byte-frozen convex-chord path is unchanged
  CC_CHECK(r.decline == bo::SplitDecline::CrossingsNot2);
  CC_CHECK(r.crossings == 0);                   // a closed interior loop has no boundary crossing
}

// ── The primary tiling gate: closed seam → disk + annulus, TILE to machine ε ──
CC_TEST(smooth_trim_tiles_parent_on_real_closed_seam) {
  const topo::Shape face = fx::parentFace();
  const ssi::WLine seam = stx::closedSeamWLine();

  const bo::SmoothSplitResult r = bo::splitFaceSmoothTrim(face, seam);
  CC_CHECK(r.ok());
  CC_CHECK(r.decline == bo::SmoothSplitDecline::Ok);
  CC_CHECK(r.crossings == 0);
  if (!r.ok()) return;
  const bo::SmoothFaceSplit& s = *r.split;

  // Exact tiling identity (independently re-derived from the two sub-loops).
  const double aIn = loopArea(s.loopInside);
  const double gap = std::fabs(s.parentArea - (s.areaInside + s.areaOutside));
  CC_CHECK(gap <= s.parentArea * 1e-12);
  CC_CHECK(std::fabs(aIn - s.areaInside) <= s.parentArea * 1e-12);
  CC_CHECK(s.areaInside > 0.0 && s.areaOutside > 0.0);
  CC_CHECK(r.tilingGap <= s.parentArea * 1e-9);         // rebuild-derived tiling residual
  CC_CHECK(s.rebuildResidual <= s.parentArea * 1e-6);   // same strict rtol B2 uses

  // Closed-form: the enclosed disk equals π·ρ² up to the inscribed-polygon error.
  const double cf = stx::closedFormDiskArea();
  CC_CHECK(s.areaInside <= cf * (1.0 + 1e-9));          // an inscribed polygon underestimates
  CC_CHECK(std::fabs(s.areaInside - cf) / cf < 0.05);   // within the discretization band

  // Two genuinely-trimmed sub-faces over the bowl surface.
  CC_CHECK(!s.faceInside.isNull() && !s.faceOutside.isNull());
  CC_CHECK(s.faceInside.type() == topo::ShapeType::Face);
  CC_CHECK(s.faceOutside.type() == topo::ShapeType::Face);
}

// ── The two sub-faces MESH via M0 and their meshed areas TILE the parent, at
// MULTIPLE deflections (a native-only pre-image of the watertight sim gate). ───
CC_TEST(smooth_trim_subfaces_mesh_and_tile_multi_deflection) {
  namespace tess = cybercad::native::tessellate;
  const topo::Shape face = fx::parentFace();
  const ssi::WLine seam = stx::closedSeamWLine();
  const bo::SmoothSplitResult r = bo::splitFaceSmoothTrim(face, seam);
  CC_CHECK(r.ok());
  if (!r.ok()) return;
  const bo::SmoothFaceSplit& s = *r.split;

  // Converged reference: the true curved area of the bowl over the quad, from a
  // FINELY-meshed parent (matches the analytic ∫∫√(1+4a²(x²+y²))dA ≈ 0.38120). The
  // sub-faces resolve the curved SEAM boundary far more densely than a coarse parent,
  // so the honest tiling gate compares their SUM to this converged truth (not to an
  // under-resolved coarse parent).
  tess::MeshParams fine;
  fine.deflection = 5e-4;
  const double aTrue = tess::surfaceArea(tess::FaceMesher(fine).mesh(face));
  CC_CHECK(aTrue > 0.0);

  const double deflections[] = {0.02, 0.01, 0.005};
  double prevRel = 1e9;
  for (double d : deflections) {
    tess::MeshParams mp;
    mp.deflection = d;
    const tess::FaceMesher fm(mp);
    const tess::Mesh mIn = fm.mesh(s.faceInside);
    const tess::Mesh mOut = fm.mesh(s.faceOutside);
    CC_CHECK(!mIn.triangles.empty());       // the enclosed disk genuinely meshes
    CC_CHECK(!mOut.triangles.empty());      // the annulus (hole) genuinely meshes

    const double aIn = tess::surfaceArea(mIn);
    const double aOut = tess::surfaceArea(mOut);
    const double rel = std::fabs(aTrue - (aIn + aOut)) / aTrue;
    CC_CHECK(rel < 1.5e-2);                  // the two sub-faces TILE the parent's curved area
    CC_CHECK(rel <= prevRel + 1e-9);         // and CONVERGE monotonically as deflection tightens
    prevRel = rel;

    // The enclosed disk meshes to the closed-form circle area π·ρ² (the split does
    // not lose or duplicate the enclosed region).
    const double cf = stx::closedFormDiskArea();
    CC_CHECK(std::fabs(aIn - cf) / cf < 1e-2);
  }
}

// ── Honest decline: an OPEN chord that CROSSES the boundary → SeamNotInterior ──
CC_TEST(smooth_trim_declines_open_chord) {
  const topo::Shape face = fx::parentFace();
  const ssi::WLine open = fx::seamWLine();           // the vertical-plane open chord (B2's case)
  CC_CHECK(open.points.size() >= 2);
  const bo::SmoothSplitResult r = bo::splitFaceSmoothTrim(face, open);
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::SmoothSplitDecline::SeamNotInterior);
  CC_CHECK(r.crossings != 0);                        // measured blocker
}

// ── Honest decline: a SELF-INTERSECTING closed loop (bowtie) → SelfIntersecting ─
CC_TEST(smooth_trim_declines_self_intersecting_loop) {
  const topo::Shape face = fx::parentFace();
  // A figure-eight entirely interior to the quad (crosses the boundary 0 times, all
  // nodes inside, but the loop is not simple).
  ssi::WLine bowtie;
  auto add = [&](double u, double v) {
    ssi::WLinePoint p; p.u1 = u; p.v1 = v; bowtie.points.push_back(p);
  };
  add(0.40, 0.40); add(0.60, 0.60); add(0.40, 0.60); add(0.60, 0.40);
  const bo::SmoothSplitResult r = bo::splitFaceSmoothTrim(face, bowtie);
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::SmoothSplitDecline::SelfIntersecting);
}

// ── Honest decline: a too-short seam (< 3 nodes) → SeamTooShort ───────────────
CC_TEST(smooth_trim_declines_short_seam) {
  const topo::Shape face = fx::parentFace();
  ssi::WLine two;
  ssi::WLinePoint a; a.u1 = 0.45; a.v1 = 0.5; two.points.push_back(a);
  ssi::WLinePoint b; b.u1 = 0.55; b.v1 = 0.5; two.points.push_back(b);
  const bo::SmoothSplitResult r = bo::splitFaceSmoothTrim(face, two);
  CC_CHECK(!r.ok());
  CC_CHECK(r.decline == bo::SmoothSplitDecline::SeamTooShort);
}

int main() { return cctest::run_all(); }
