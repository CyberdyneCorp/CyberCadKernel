// SPDX-License-Identifier: Apache-2.0
//
// native_replace_face_parity.mm — MOAT M-DM DM2 SIM GATE (b): the native
// `replace_face_to_plane` (the app's push/pull "move a planar face to a target plane")
// native-vs-OCCT on a booted iOS simulator.
//
// The native re-solve (src/native/directmodel/replace_face.h, OCCT-FREE) composes the
// landed DM1 splitByPlane + boolean_solid(Fuse) + build_prism: a parallel pull is one
// DM1 cut, a parallel push is a slab fused on, a tilted/mixed move is grow-then-trim
// (one Fuse + one tilted cut). This harness proves, against the OCCT ORACLE, that the
// native moved solid matches the OCCT move-face reference on:
//   * VOLUME     — native enclosedVolume vs BRepGProp::VolumeProperties (rel ≤ 2e-2);
//   * AREA       — native surfaceArea vs BRepGProp::SurfaceProperties (rel ≤ 2e-2);
//   * WATERTIGHT — the native result mesh is a closed 2-manifold;
//   * TOPOLOGY   — native Euler χ = 2 (single closed genus-0 solid) and OCCT one solid;
//   * BBOX       — native vertex min/max vs BRepBndLib per-axis (abs ≤ 1.5·deflection).
//
// OCCT ORACLE. The shipped OcctEngine::replace_face_to_plane is a half-space CUT (it can
// only TRIM). To also cover the PUSH (grow) and the tilted grow-then-trim, the reference
// is the OCCT plane-cut-and-extend: build the box with the moved face pushed OUT past the
// target, then Cut by the target half-space keeping the bulk (the −n side). This keeps
// the five untouched faces on their original planes and replaces the moved face with the
// target plane — a legitimate OCCT move-face oracle (BRepPrimAPI_MakeBox +
// BRepPrimAPI_MakeHalfSpace + BRepAlgoAPI_Cut, sized L=2·(diag+1), matching the shipped
// cut). For a pure trim it reduces to the shipped cut oracle exactly.
//
// Fixtures (the reachable DM2 slice on a convex planar polyhedron [0,10]³):
//   1. +X face parallel PULL  x=10 → x=7   (ΔV = −A·3, trim);
//   2. +X face parallel PUSH  x=10 → x=13  (ΔV = +A·3, grow);
//   3. +Z face tilted PURE-TRIM  (tp=(5,5,8),  n=(0,sinθ,cosθ), d̄=−2, single cut);
//   4. +Z face tilted MIXED grow-trim (tp=(5,5,10), n=(0,sinθ,cosθ), d̄=0, grow-then-trim).
// OCCT is the ORACLE ONLY, never linked into src/native. Build: run-sim-native-replace-face.sh.
//
#include "native/blend/blend_geom.h"
#include "native/construct/native_construct.h"
#include "native/directmodel/replace_face.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_replace_face_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pln.hxx>
#include <gp_Dir.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

namespace dm = cybercad::native::directmodel;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace ncst = cybercad::native::construct;
namespace nbl = cybercad::native::blend;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[DM2] %-22s %-14s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

// The OCCT move-face oracle — plane-cut-and-extend (see header). `overTall` is the box
// grown so the moved face lies past the target; Cut by the target half-space keeps the
// bulk (−n side). Mirrors OcctEngine::replace_face_to_plane's sizing (L=2·(diag+1)).
static TopoDS_Shape occtMoveFace(const TopoDS_Shape& overTall, const nm::Point3& o,
                                 const nm::Vec3& n) {
  const double nlen = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
  const gp_Dir nd(n.x / nlen, n.y / nlen, n.z / nlen);
  const gp_Pnt op(o.x, o.y, o.z);
  const gp_Pln pln(op, nd);
  Bnd_Box bb; BRepBndLib::Add(overTall, bb);
  Standard_Real xm, ym, zm, xM, yM, zM;
  bb.Get(xm, ym, zm, xM, yM, zM);
  const double L = 2.0 * (gp_Pnt(xm, ym, zm).Distance(gp_Pnt(xM, yM, zM)) + 1.0);
  TopoDS_Face face = BRepBuilderAPI_MakeFace(pln, -L, L, -L, L).Face();
  // Remove the +n half (ref inside +n), keep the bulk on −n.
  const gp_Pnt ref(op.X() + nd.X() * L * 0.25, op.Y() + nd.Y() * L * 0.25,
                   op.Z() + nd.Z() * L * 0.25);
  BRepPrimAPI_MakeHalfSpace mkHS(face, ref);
  BRepAlgoAPI_Cut cut(overTall, mkHS.Solid());
  cut.Build();
  return cut.Shape();
}

static double occtVolume(const TopoDS_Shape& s) {
  GProp_GProps g; BRepGProp::VolumeProperties(s, g); return std::fabs(g.Mass());
}
static double occtArea(const TopoDS_Shape& s) {
  GProp_GProps g; BRepGProp::SurfaceProperties(s, g); return g.Mass();
}
static int occtSolidCount(const TopoDS_Shape& s) {
  int n = 0;
  for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) ++n;
  return n;
}

static long eulerChar(const ntess::Mesh& m) {
  return static_cast<long>(m.vertices.size()) - static_cast<long>(ntess::edgeUseCounts(m).size()) +
         static_cast<long>(m.triangles.size());
}
struct BBox { double lo[3], hi[3]; };
static BBox meshBBox(const ntess::Mesh& m) {
  BBox b{{1e30, 1e30, 1e30}, {-1e30, -1e30, -1e30}};
  for (const nm::Point3& v : m.vertices) {
    const double c[3] = {v.x, v.y, v.z};
    for (int i = 0; i < 3; ++i) { b.lo[i] = std::min(b.lo[i], c[i]); b.hi[i] = std::max(b.hi[i], c[i]); }
  }
  return b;
}

static int faceByNormal(const nt::Shape& s, const nm::Vec3& dir) {
  const nt::ShapeMap map = nt::mapShapes(s, nt::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto pl = nbl::facePlane(s, static_cast<int>(i));
    if (pl && nm::dot(pl->normal, dir) > 0.999) return static_cast<int>(i);
  }
  return -1;
}

static nt::Shape boxPrism(double sx, double sy, double sz) {
  const double p[8] = {0, 0, sx, 0, sx, sy, 0, sy};
  return ncst::build_prism(p, 4, sz);
}

// One fixture: native move-face vs OCCT plane-cut-and-extend oracle on every metric.
static void compareOne(const char* tag, const nt::Shape& box, const nm::Vec3& faceDir,
                       const nm::Point3& tp, const nm::Vec3& n, const TopoDS_Shape& occtOverTall,
                       double defl, double relTol, double spatialTol) {
  const int fid = faceByNormal(box, faceDir);
  report(tag, "face-id", fid > 0, fid > 0 ? "found" : "missing");
  if (fid <= 0) return;

  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const nt::Shape result = dm::replaceFaceToPlane(box, fid, tp, n, &why, defl);
  {
    char buf[48]; std::snprintf(buf, sizeof(buf), "decline=%d", static_cast<int>(why));
    report(tag, "native-resolve", !result.isNull(), buf);
  }
  if (result.isNull()) return;

  ntess::MeshParams mp; mp.deflection = defl;
  const ntess::Mesh m = ntess::SolidMesher(mp).mesh(result);
  report(tag, "watertight", ntess::isWatertight(m), "closed 2-manifold");
  {
    const long chi = eulerChar(m);
    char buf[40]; std::snprintf(buf, sizeof(buf), "euler=%ld", chi);
    report(tag, "topology", chi == 2, buf);
  }
  const double vNat = std::fabs(ntess::enclosedVolume(m));
  const double aNat = ntess::surfaceArea(m);
  const BBox bNat = meshBBox(m);

  const TopoDS_Shape occtPiece = occtMoveFace(occtOverTall, tp, n);
  report(tag, "occt-single", occtSolidCount(occtPiece) == 1, "one closed solid");
  const double vOcct = occtVolume(occtPiece);
  const double aOcct = occtArea(occtPiece);
  {
    const double rel = std::fabs(vNat - vOcct) / vOcct;
    char buf[96]; std::snprintf(buf, sizeof(buf), "nat=%.5f occt=%.5f rel=%.3e", vNat, vOcct, rel);
    report(tag, "volume", rel <= relTol, buf);
  }
  {
    const double rel = std::fabs(aNat - aOcct) / aOcct;
    char buf[96]; std::snprintf(buf, sizeof(buf), "nat=%.5f occt=%.5f rel=%.3e", aNat, aOcct, rel);
    report(tag, "area", rel <= relTol, buf);
  }
  {
    Bnd_Box bb; BRepBndLib::AddOptimal(occtPiece, bb, Standard_True, Standard_False);
    double xo[3], xh[3];
    bb.Get(xo[0], xo[1], xo[2], xh[0], xh[1], xh[2]);
    double worst = 0.0;
    for (int i = 0; i < 3; ++i) {
      worst = std::max(worst, std::fabs(bNat.lo[i] - xo[i]));
      worst = std::max(worst, std::fabs(bNat.hi[i] - xh[i]));
    }
    char buf[80]; std::snprintf(buf, sizeof(buf), "worst=%.3e tol=%.3e", worst, spatialTol);
    report(tag, "bbox", worst <= spatialTol, buf);
  }
}

int main() {
  std::printf("== MOAT M-DM DM2 native cc_replace_face_to_plane: native-vs-OCCT parity ==\n");
  std::fflush(stdout);
  const double relTol = 0.02, defl = 0.005, spatial = std::max(1.5 * defl, 1e-6);

  // 1. +X face parallel PULL x=10 → x=7. Oracle: [0,20]×[0,10]×[0,10] cut at x=7.
  {
    const nt::Shape box = boxPrism(10, 10, 10);
    const TopoDS_Shape over = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(20, 10, 10)).Solid();
    compareOne("box/+x/pull-7", box, nm::Vec3{1, 0, 0}, nm::Point3{7, 0, 0}, nm::Vec3{1, 0, 0},
               over, defl, relTol, spatial);
  }
  // 2. +X face parallel PUSH x=10 → x=13. Oracle: [0,20]×[0,10]×[0,10] cut at x=13.
  {
    const nt::Shape box = boxPrism(10, 10, 10);
    const TopoDS_Shape over = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(20, 10, 10)).Solid();
    compareOne("box/+x/push-13", box, nm::Vec3{1, 0, 0}, nm::Point3{13, 0, 0}, nm::Vec3{1, 0, 0},
               over, defl, relTol, spatial);
  }
  // 3. +Z face tilted PURE-TRIM tp=(5,5,8), n=(0,sinθ,cosθ). Oracle: [0,10]²×[0,20] cut.
  {
    const double th = 0.15;
    const nm::Vec3 n{0, std::sin(th), std::cos(th)};
    const nt::Shape box = boxPrism(10, 10, 10);
    const TopoDS_Shape over = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(10, 10, 20)).Solid();
    compareOne("box/+z/tilt-trim", box, nm::Vec3{0, 0, 1}, nm::Point3{5, 5, 8}, n,
               over, defl, relTol, spatial);
  }
  // 4. +Z face tilted MIXED grow-trim tp=(5,5,10), n=(0,sinθ,cosθ). Oracle: cut over-tall.
  {
    const double th = 0.15;
    const nm::Vec3 n{0, std::sin(th), std::cos(th)};
    const nt::Shape box = boxPrism(10, 10, 10);
    const TopoDS_Shape over = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(10, 10, 20)).Solid();
    compareOne("box/+z/tilt-mixed", box, nm::Vec3{0, 0, 1}, nm::Point3{5, 5, 10}, n,
               over, defl, relTol, spatial);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
