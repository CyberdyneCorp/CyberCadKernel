// SPDX-License-Identifier: Apache-2.0
//
// native_dm3_dm4_parity.mm — MOAT M-DM DM3 + DM4 SIM GATE (b): the native GENERAL
// `replace_face` (offset/tilt retarget) and `project_point_on_face` (point onto an
// analytic surface), native-vs-OCCT on a booted iOS simulator.
//
// ── DM3 (src/native/directmodel/replace_face_general.h, OCCT-FREE) ────────────────
// `replaceFaceOffsetTilt` derives the target plane from the picked planar face's own
// plane (offset along the outward normal; the native slice serves tilt = 0) and
// re-solves via the landed DM2 verb. This harness proves the native moved solid matches
// the OCCT move-face reference (plane-cut-and-extend, the SAME oracle DM2 uses) on
// VOLUME / AREA / WATERTIGHT / TOPOLOGY (Euler χ) / BBOX for:
//   1. +X face parallel PUSH  offset +3  (grow);
//   2. +X face parallel PULL  offset −3  (trim);
//   3. an OFF-AXIS face (box rotated 30° about Z) parallel PUSH offset +2 — the DM3
//      breadth beyond DM2's axis-aligned fixtures; the OCCT oracle is the same cut
//      rigidly rotated by the same transform.
//
// ── DM4 (src/native/directmodel/project.h, OCCT-FREE) ─────────────────────────────
// `projectPointOnFace` drops a point onto a plane / cylinder / sphere face's analytic
// surface. This harness proves the native foot + distance match the OCCT ORACLE
// GeomAPI_ProjectPointOnSurf on the SAME untrimmed analytic surface (Geom_Plane /
// Geom_CylindricalSurface / Geom_SphericalSurface) to machine precision.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build: run-sim-native-dm3-dm4.sh.
//
#include "native/blend/blend_geom.h"
#include "native/construct/native_construct.h"
#include "native/directmodel/project.h"
#include "native/directmodel/replace_face_general.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdio>
#include <optional>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_dm3_dm4_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

namespace dm = cybercad::native::directmodel;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace ncst = cybercad::native::construct;
namespace nbl = cybercad::native::blend;

using nt::EdgeCurve;
using nt::FaceSurface;
using nt::Shape;
using nt::ShapeBuilder;

static int g_pass = 0, g_fail = 0;
static void report(const char* tag, const char* check, bool ok, const char* detail) {
  std::printf("[DM3/4] %-20s %-14s %s  (%s)\n", tag, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

// ── OCCT metrics ─────────────────────────────────────────────────────────────────
static double occtVolume(const TopoDS_Shape& s) {
  GProp_GProps g; BRepGProp::VolumeProperties(s, g); return std::fabs(g.Mass());
}
static double occtArea(const TopoDS_Shape& s) {
  GProp_GProps g; BRepGProp::SurfaceProperties(s, g); return g.Mass();
}
static int occtSolidCount(const TopoDS_Shape& s) {
  int n = 0; for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) ++n; return n;
}

// The OCCT move-face oracle — plane-cut-and-extend (mirrors DM2). Optionally rigidly
// transformed by `xf` (identity by default) to cover the off-axis fixture.
static TopoDS_Shape occtMoveFace(const TopoDS_Shape& overTall, const nm::Point3& o,
                                 const nm::Vec3& n, const gp_Trsf& xf) {
  const double nlen = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
  const gp_Dir nd(n.x / nlen, n.y / nlen, n.z / nlen);
  const gp_Pnt op(o.x, o.y, o.z);
  const gp_Pln pln(op, nd);
  Bnd_Box bb; BRepBndLib::Add(overTall, bb);
  Standard_Real xm, ym, zm, xM, yM, zM; bb.Get(xm, ym, zm, xM, yM, zM);
  const double L = 2.0 * (gp_Pnt(xm, ym, zm).Distance(gp_Pnt(xM, yM, zM)) + 1.0);
  TopoDS_Face face = BRepBuilderAPI_MakeFace(pln, -L, L, -L, L).Face();
  const gp_Pnt ref(op.X() + nd.X() * L * 0.25, op.Y() + nd.Y() * L * 0.25, op.Z() + nd.Z() * L * 0.25);
  BRepPrimAPI_MakeHalfSpace mkHS(face, ref);
  BRepAlgoAPI_Cut cut(overTall, mkHS.Solid());
  cut.Build();
  return BRepBuilderAPI_Transform(cut.Shape(), xf, Standard_True).Shape();
}

// ── native mesh metrics ──────────────────────────────────────────────────────────
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

// ── DM3: one fixture, native replaceFaceOffsetTilt vs OCCT move-face oracle ────────
static void dm3One(const char* tag, const nt::Shape& box, const nm::Vec3& faceDir, double offset,
                   const nm::Point3& targetPt, const nm::Vec3& targetN,
                   const TopoDS_Shape& occtOverTall, const gp_Trsf& xf, double defl, double relTol,
                   double spatialTol) {
  const int fid = faceByNormal(box, faceDir);
  report(tag, "face-id", fid > 0, fid > 0 ? "found" : "missing");
  if (fid <= 0) return;
  dm::ReplaceFaceGeneralDecline why = dm::ReplaceFaceGeneralDecline::Ok;
  const nt::Shape result = dm::replaceFaceOffsetTilt(box, fid, offset, /*tilt=*/0.0, &why, defl);
  {
    char buf[48]; std::snprintf(buf, sizeof(buf), "decline=%d", static_cast<int>(why));
    report(tag, "native-resolve", !result.isNull(), buf);
  }
  if (result.isNull()) return;

  ntess::MeshParams mp; mp.deflection = defl;
  const ntess::Mesh m = ntess::SolidMesher(mp).mesh(result);
  report(tag, "watertight", ntess::isWatertight(m), "closed 2-manifold");
  { const long chi = eulerChar(m); char b[40]; std::snprintf(b, sizeof(b), "euler=%ld", chi);
    report(tag, "topology", chi == 2, b); }
  const double vNat = std::fabs(ntess::enclosedVolume(m));
  const double aNat = ntess::surfaceArea(m);
  const BBox bNat = meshBBox(m);

  const TopoDS_Shape occtPiece = occtMoveFace(occtOverTall, targetPt, targetN, xf);
  report(tag, "occt-single", occtSolidCount(occtPiece) == 1, "one closed solid");
  { const double rel = std::fabs(vNat - occtVolume(occtPiece)) / occtVolume(occtPiece);
    char b[96]; std::snprintf(b, sizeof(b), "nat=%.5f occt=%.5f rel=%.3e", vNat, occtVolume(occtPiece), rel);
    report(tag, "volume", rel <= relTol, b); }
  { const double rel = std::fabs(aNat - occtArea(occtPiece)) / occtArea(occtPiece);
    char b[96]; std::snprintf(b, sizeof(b), "nat=%.5f occt=%.5f rel=%.3e", aNat, occtArea(occtPiece), rel);
    report(tag, "area", rel <= relTol, b); }
  { Bnd_Box bb; BRepBndLib::AddOptimal(occtPiece, bb, Standard_True, Standard_False);
    double xo[3], xh[3]; bb.Get(xo[0], xo[1], xo[2], xh[0], xh[1], xh[2]);
    double worst = 0.0;
    for (int i = 0; i < 3; ++i) {
      worst = std::max(worst, std::fabs(bNat.lo[i] - xo[i]));
      worst = std::max(worst, std::fabs(bNat.hi[i] - xh[i]));
    }
    char b[80]; std::snprintf(b, sizeof(b), "worst=%.3e tol=%.3e", worst, spatialTol);
    report(tag, "bbox", worst <= spatialTol, b); }
}

// ── DM4 fixtures: native cylinder + sphere builders ───────────────────────────────
static Shape circleEdge(const nm::Point3& c, double r, const nm::Dir3& nrm, const nm::Dir3& xr) {
  EdgeCurve e; e.kind = EdgeCurve::Kind::Circle;
  e.frame = nm::Ax3::fromAxisAndRef(c, nrm, xr); e.radius = r;
  return ShapeBuilder::makeEdge(e, 0.0, 2.0 * 3.14159265358979323846, Shape{}, Shape{});
}
static Shape cylinderZ(double R, double H) {  // lateral cylinder = face 3
  const nm::Dir3 zp{0, 0, 1}, xp{1, 0, 0};
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = nm::Ax3::fromAxisAndRef(nm::Point3{0, 0, 0}, nm::Dir3{0, 0, -1}, xp);
  Shape bottom = ShapeBuilder::makeFace(bs, ShapeBuilder::makeWire({circleEdge({0, 0, 0}, R, zp, xp)}));
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = nm::Ax3::fromAxisAndRef(nm::Point3{0, 0, H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(ts, ShapeBuilder::makeWire({circleEdge({0, 0, H}, R, zp, xp)}));
  FaceSurface ls; ls.kind = FaceSurface::Kind::Cylinder;
  ls.frame = nm::Ax3::fromAxisAndRef(nm::Point3{0, 0, 0}, zp, xp); ls.radius = R;
  Shape lat = ShapeBuilder::makeFace(
      ls, ShapeBuilder::makeWire({circleEdge({0, 0, 0}, R, zp, xp), circleEdge({0, 0, H}, R, zp, xp)}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lat})});
}
static Shape sphereAt(double R) {  // single sphere face = face 1
  FaceSurface s; s.kind = FaceSurface::Kind::Sphere;
  s.frame = nm::Ax3::fromAxisAndRef(nm::Point3{0, 0, 0}, nm::Dir3{0, 0, 1}, nm::Dir3{1, 0, 0});
  s.radius = R;
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({ShapeBuilder::makeFace(s, ShapeBuilder::makeWire({}))})});
}

// ── DM4: one fixture, native projectPointOnFace vs OCCT GeomAPI_ProjectPointOnSurf ──
static void dm4One(const char* tag, const nt::Shape& solid, int faceId, const nm::Point3& p,
                   const Handle(Geom_Surface) & occtSurf, double tol) {
  dm::ProjectDecline why = dm::ProjectDecline::Ok;
  const auto r = dm::projectPointOnFace(solid, faceId, p, &why);
  report(tag, "native-foot", r.has_value(), r.has_value() ? "ok" : "declined");
  if (!r) return;
  GeomAPI_ProjectPointOnSurf proj(gp_Pnt(p.x, p.y, p.z), occtSurf);
  const bool ok = proj.IsDone() && proj.NbPoints() >= 1;
  report(tag, "occt-foot", ok, ok ? "ok" : "failed");
  if (!ok) return;
  const gp_Pnt f = proj.NearestPoint();
  const double dFoot = std::sqrt((r->foot.x - f.X()) * (r->foot.x - f.X()) +
                                 (r->foot.y - f.Y()) * (r->foot.y - f.Y()) +
                                 (r->foot.z - f.Z()) * (r->foot.z - f.Z()));
  { char b[112]; std::snprintf(b, sizeof(b), "nat=(%.4f,%.4f,%.4f) occt=(%.4f,%.4f,%.4f) d=%.3e",
      r->foot.x, r->foot.y, r->foot.z, f.X(), f.Y(), f.Z(), dFoot);
    report(tag, "foot-coord", dFoot <= tol, b); }
  { const double dd = std::fabs(r->distance - proj.LowerDistance());
    char b[80]; std::snprintf(b, sizeof(b), "nat=%.5f occt=%.5f |Δ|=%.3e", r->distance, proj.LowerDistance(), dd);
    report(tag, "distance", dd <= tol, b); }
}

int main() {
  std::printf("== MOAT M-DM DM3 replace_face + DM4 project: native-vs-OCCT parity ==\n");
  std::fflush(stdout);
  const double relTol = 0.02, defl = 0.005, spatial = std::max(1.5 * defl, 1e-6);
  gp_Trsf identity;

  // ── DM3 ──
  // 1. +X face PUSH offset +3 (grow) → oracle box [0,20]³-ish cut at x=13.
  { const nt::Shape box = boxPrism(10, 10, 10);
    const TopoDS_Shape over = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(20, 10, 10)).Solid();
    dm3One("box/+x/push+3", box, nm::Vec3{1, 0, 0}, 3.0, nm::Point3{13, 5, 5}, nm::Vec3{1, 0, 0},
           over, identity, defl, relTol, spatial); }
  // 2. +X face PULL offset −3 (trim) → oracle box cut at x=7.
  { const nt::Shape box = boxPrism(10, 10, 10);
    const TopoDS_Shape over = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(20, 10, 10)).Solid();
    dm3One("box/+x/pull-3", box, nm::Vec3{1, 0, 0}, -3.0, nm::Point3{7, 5, 5}, nm::Vec3{1, 0, 0},
           over, identity, defl, relTol, spatial); }
  // 3. OFF-AXIS: box rotated 30° about Z, its former +X face PUSHed +2 along the rotated
  //    normal. Oracle = the axis-aligned cut rigidly rotated by the same 30°.
  { const double th = 30.0 * 3.14159265358979323846 / 180.0;
    nt::Shape box = boxPrism(10, 10, 10);
    box = box.located(nt::Location(nm::Transform::rotationOf(nm::Point3{0, 0, 0}, nm::Dir3{0, 0, 1}, th)));
    gp_Trsf rot; rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), th);
    const TopoDS_Shape over = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(20, 10, 10)).Solid();
    const nm::Vec3 nOff{std::cos(th), std::sin(th), 0};
    dm3One("box/offaxis/push+2", box, nOff, 2.0, nm::Point3{12, 5, 5}, nm::Vec3{1, 0, 0},
           over, rot, defl, relTol, spatial); }

  // ── DM4 ──
  const double footTol = 1e-6;
  // Plane: box +X face at x=10; project (15,3,4) → (10,3,4), d=5.
  { const nt::Shape box = boxPrism(10, 10, 10);
    const int fid = faceByNormal(box, nm::Vec3{1, 0, 0});
    Handle(Geom_Plane) pl = new Geom_Plane(gp_Pnt(10, 0, 0), gp_Dir(1, 0, 0));
    dm4One("plane", box, fid, nm::Point3{15, 3, 4}, pl, footTol); }
  // Cylinder R=5 axis Z; lateral face id 3; project (8,0,12) → (5,0,12), d=3.
  { const nt::Shape cyl = cylinderZ(5, 20);
    Handle(Geom_CylindricalSurface) cs =
        new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), 5.0);
    dm4One("cylinder", cyl, 3, nm::Point3{8, 0, 12}, cs, footTol); }
  // Sphere R=5 at origin; single face id 1; project (6,8,0) → (3,4,0), d=5.
  { const nt::Shape sph = sphereAt(5);
    Handle(Geom_SphericalSurface) ss =
        new Geom_SphericalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), 5.0);
    dm4One("sphere", sph, 1, nm::Point3{6, 8, 0}, ss, footTol); }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
