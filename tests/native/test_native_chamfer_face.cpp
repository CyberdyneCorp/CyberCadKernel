// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the MOAT M2 FULL-FACE chamfer weld
// (`src/native/blend/chamfer_face.h`, verb `chamfer_face`). OCCT-FREE — Gate (a) of
// the two-gate model: the verb compiles and unit-tests with clang++ -std=c++20, no
// OCCT, no simulator, no cc_* facade.
//
// `chamfer_face` chamfers EVERY convex planar-dihedral edge bounding a picked planar
// face at constant setback. The byte-frozen SEQUENTIAL `chamfer_edges` DECLINES such a
// corner-sharing edge loop (the first set-back removes the shared corner, losing the
// next edge). The landed convex-CORNER weld `chamfer_corner` — which `chamfer_face`
// assembles on top of — resolves all chamfer planes up front and welds the loop
// watertight, the corner facets forming from the exposed rings.
//
// A single face's edge loop is a set of 2-edge DIHEDRAL corners (never a triple — the
// third edge through each corner vertex runs off the OTHER, unpicked faces), so the
// removed volume has an EXACT closed form (inclusion-exclusion of the per-edge setback
// prisms minus the per-corner dihedral overlaps), asserted to machine ε. For a box face
// [edge length L on all four sides, setback d each face]:
//   * 4 edges, 4 dihedral corners: V_removed = 4·(d²L/2) − 4·(d³/3) = 2d²L − 4d³/3
// (each edge removes a d²L/2 prism; each 2-edge orthogonal corner double-counts a d³/3
// tetrahedral wedge — the same d/3 term the corner weld's 2-edge closed form carries).
// The 2-edge dihedral corner is a union of two half-space prisms OCCT's MakeChamfer
// reproduces EXACTLY, so the sim gate measures native == OCCT to fp64 on the full loop.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_chamfer_face.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o /tmp/test_native_chamfer_face && /tmp/test_native_chamfer_face
//
#include "native/blend/native_blend.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace blend = cybercad::native::blend;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;

namespace {

// A box [0,sx]×[0,sy]×[0,sz] as a native prism (base at z=0).
topo::Shape box(double sx, double sy, double sz) {
  const double p[] = {0, 0, sx, 0, sx, sy, 0, sy};
  return cst::build_prism(p, 4, sz);
}

double vol(const topo::Shape& s, bool& wt) {
  if (s.isNull()) { wt = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = 0.004;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

bool nearAbs(double got, double want, double abs = 1e-4) {
  return std::fabs(got - want) <= abs;
}

// The face id (1-based mapShapes(Face) order) whose outward plane has normal `n` and
// passes through offset `w` (i.e. n·p = w). Returns -1 if not found / non-planar.
int faceWithNormal(const topo::Shape& s, double nx, double ny, double nz, double w) {
  const topo::ShapeMap fmap = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto pl = blend::facePlane(s, static_cast<int>(fi));
    if (!pl) continue;
    if (std::fabs(pl->normal.x - nx) < 1e-6 && std::fabs(pl->normal.y - ny) < 1e-6 &&
        std::fabs(pl->normal.z - nz) < 1e-6 && std::fabs(pl->w - w) < 1e-6)
      return static_cast<int>(fi);
  }
  return -1;
}

// A capped solid cylinder for the curved-decline test.
topo::Shape cappedCylinder(double Rc, double h) {
  cst::ProfileSegment seg;
  seg.kind = 2;  // full circle
  seg.cx = 0; seg.cy = 0; seg.r = Rc;
  return cst::build_prism_profile({seg}, {}, {}, h);
}

// Closed-form removed volume of a full-face box chamfer (4 edges, setback d, side L).
double remFace(double d, double L) { return 2.0 * d * d * L - 4.0 * d * d * d / 3.0; }

}  // namespace

// ── the full-face weld: all four edges of a box face land, EXACT closed-form volume ──

CC_TEST(chamfer_face_top_face_exact) {
  const double L = 10.0, d = 1.5;
  topo::Shape b = box(L, L, L);
  const int top = faceWithNormal(b, 0, 0, 1, L);  // +z face at z=L
  CC_CHECK(top > 0);
  bool wt = false;
  blend::ChamferFaceDecline why = blend::ChamferFaceDecline::BadInput;
  const double v = vol(blend::chamfer_face(b, top, d, &why), wt);
  CC_CHECK(why == blend::ChamferFaceDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(nearAbs(v, L * L * L - remFace(d, L)));  // 959.5
}

CC_TEST(chamfer_face_every_face_welds_exact) {
  // Every one of the six faces of a cube chamfers watertight at the identical
  // closed-form volume (a full-face loop is four 2-edge dihedral corners on any face).
  const double L = 10.0, d = 1.0;
  const double want = L * L * L - remFace(d, L);  // 981.333…
  const struct { double nx, ny, nz, w; } faces[6] = {
      {0, 0, 1, L}, {0, 0, -1, 0}, {1, 0, 0, L}, {-1, 0, 0, 0}, {0, 1, 0, L}, {0, -1, 0, 0}};
  for (const auto& f : faces) {
    topo::Shape b = box(L, L, L);
    const int fid = faceWithNormal(b, f.nx, f.ny, f.nz, f.w);
    CC_CHECK(fid > 0);
    bool wt = false;
    blend::ChamferFaceDecline why = blend::ChamferFaceDecline::BadInput;
    const double v = vol(blend::chamfer_face(b, fid, d, &why), wt);
    CC_CHECK(why == blend::ChamferFaceDecline::Ok);
    CC_CHECK(wt);
    CC_CHECK(nearAbs(v, want, 1e-3));
  }
}

CC_TEST(chamfer_face_setback_sweep_matches_closed_form) {
  // Sweep the setback on the top face; the full-face weld tracks the closed form at
  // every size (each corner stays a 2-edge dihedral → EXACT vs OCCT).
  const double L = 10.0;
  for (double d : {0.5, 1.0, 2.0, 3.0}) {
    topo::Shape b = box(L, L, L);
    const int top = faceWithNormal(b, 0, 0, 1, L);
    CC_CHECK(top > 0);
    bool wt = false;
    blend::ChamferFaceDecline why = blend::ChamferFaceDecline::BadInput;
    const double v = vol(blend::chamfer_face(b, top, d, &why), wt);
    CC_CHECK(why == blend::ChamferFaceDecline::Ok);
    CC_CHECK(wt);
    CC_CHECK(nearAbs(v, L * L * L - remFace(d, L), 2e-3));
  }
}

CC_TEST(chamfer_face_non_orthogonal_prism_top_welds) {
  // A non-orthogonal face loop: the top face of an equilateral-triangle prism (three
  // 60° corners). Each corner is a 2-edge dihedral, so the weld lands watertight and
  // removes material (its closed form depends on the corner angles; we assert the
  // watertight + shrink invariant the engine self-verify enforces).
  const double s = 10.0, h = 8.0, hh = s * std::sqrt(3.0) / 2.0;
  const double tri[] = {0, 0, s, 0, s / 2.0, hh};
  topo::Shape p = cst::build_prism(tri, 3, h);
  const int top = faceWithNormal(p, 0, 0, 1, h);  // +z top at z=h
  CC_CHECK(top > 0);
  bool wt = false;
  blend::ChamferFaceDecline why = blend::ChamferFaceDecline::BadInput;
  const double v = vol(blend::chamfer_face(p, top, 0.8, &why), wt);
  CC_CHECK(why == blend::ChamferFaceDecline::Ok);
  CC_CHECK(wt);
  const double v0 = (s * hh / 2.0) * h;  // triangular prism volume
  CC_CHECK(v > 0.0 && v < v0);           // material removed
}

// ── honest declines (NULL → OCCT), each with a measured reason ────────────────────

CC_TEST(chamfer_face_declines_curved_bad_input_oversized) {
  topo::Shape b = box(10, 10, 10);
  const int top = faceWithNormal(b, 0, 0, 1, 10);
  CC_CHECK(top > 0);
  blend::ChamferFaceDecline why = blend::ChamferFaceDecline::Ok;

  // Curved solid → NonPlanarSolid.
  topo::Shape cyl = cappedCylinder(5.0, 10.0);
  CC_CHECK(blend::chamfer_face(cyl, 1, 1.0, &why).isNull());
  CC_CHECK(why == blend::ChamferFaceDecline::NonPlanarSolid);

  // Non-positive distance / null → BadInput.
  CC_CHECK(blend::chamfer_face(b, top, 0.0, &why).isNull());
  CC_CHECK(why == blend::ChamferFaceDecline::BadInput);

  // Out-of-range face id → no planar face → NonPlanarFace.
  CC_CHECK(blend::chamfer_face(b, 999, 1.0, &why).isNull());
  CC_CHECK(why == blend::ChamferFaceDecline::NonPlanarFace);

  // Oversized setback (>= half the face) → the fit guard rejects the edges (NoConvexEdges)
  // or the corner weld's clip removes everything (WeldFailed) — a measured decline, never
  // a wrong solid.
  CC_CHECK(blend::chamfer_face(b, top, 20.0, &why).isNull());
  CC_CHECK(why == blend::ChamferFaceDecline::NoConvexEdges ||
           why == blend::ChamferFaceDecline::WeldFailed);
}

CC_RUN_ALL()
