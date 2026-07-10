// SPDX-License-Identifier: Apache-2.0
//
// native_draft_faces_parity.mm — MOAT feature DRAFT ANGLE SIM GATE (b): the native
// draft (taper planar side faces about a neutral plane) native-vs-OCCT on a booted iOS
// simulator.
//
// ── native (src/native/feature/draft_faces.h, OCCT-FREE) ──────────────────────────
// `feature::draftFaces` derives each drafted plane from the ORIGINAL face geometry
// (pivot on its trace with the neutral plane) and applies it as an inward split-plane
// trim, then re-audits the composite (watertight / χ=2 / oriented / shrink). This
// harness proves the native drafted solid matches the OCCT ORACLE
// BRepOffsetAPI_DraftAngle (+ BRepGProp for volume/area) on VOLUME / AREA / WATERTIGHT /
// TOPOLOGY (Euler χ) / BBOX / one-sided HAUSDORFF for:
//   1. a box +X side face drafted 8° about the base plane (pull +Z) — one wedge;
//   2. all FOUR side faces of a box drafted 5° about the base plane — a frustum;
//   3. an OFF-AXIS box (rotated 25° about Z) side face drafted 8° — rigid-motion
//      invariance of the wedge (the OCCT oracle is the same draft on the rotated solid).
// and the HONEST-DECLINE envelope (native NULL → the engine falls to OCCT):
//   4. a CAP face (normal ∥ pull) declines — no trace line to pivot about.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build: run-sim-native-draft-faces.sh.
//
#include "native/blend/blend_geom.h"
#include "native/construct/native_construct.h"
#include "native/feature/draft_faces.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdio>
#include <optional>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_draft_faces_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>

namespace df = cybercad::native::feature;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace ncst = cybercad::native::construct;
namespace nbl = cybercad::native::blend;

static int g_pass = 0, g_fail = 0;
static void report(const char* tag, const char* check, bool ok, const char* detail) {
  std::printf("[DRAFT] %-22s %-14s %s  (%s)\n", tag, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

static constexpr double kPi = 3.14159265358979323846;

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

// The OCCT draft oracle: BRepOffsetAPI_DraftAngle on the picked planar faces (chosen by
// their outward normal ~ `dirs[i]`), optionally rigidly transformed by `xf`.
static TopoDS_Shape occtDraft(const TopoDS_Shape& box, const nm::Vec3* dirs, int nDirs,
                              const gp_Dir& pull, const gp_Pln& neutral, double angleRad,
                              const gp_Trsf& xf) {
  BRepOffsetAPI_DraftAngle maker(box);
  for (int i = 0; i < nDirs; ++i) {
    for (TopExp_Explorer ex(box, TopAbs_FACE); ex.More(); ex.Next()) {
      const TopoDS_Face f = TopoDS::Face(ex.Current());
      BRepAdaptor_Surface surf(f);
      if (surf.GetType() != GeomAbs_Plane) continue;
      gp_Dir fn = surf.Plane().Axis().Direction();
      if (f.Orientation() == TopAbs_REVERSED) fn.Reverse();
      if (fn.Dot(gp_Dir(dirs[i].x, dirs[i].y, dirs[i].z)) > 0.999) {
        maker.Add(f, pull, angleRad, neutral);
        break;
      }
    }
  }
  maker.Build();
  return BRepBuilderAPI_Transform(maker.Shape(), xf, Standard_True).Shape();
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
// One-sided Hausdorff: worst distance of a native mesh vertex to the OCCT drafted solid.
static double hausdorffNativeToOcct(const ntess::Mesh& m, const TopoDS_Shape& occt) {
  double worst = 0.0;
  for (std::size_t i = 0; i < m.vertices.size(); i += 7) {  // stride: bounded sample
    const nm::Point3& v = m.vertices[i];
    BRepExtrema_DistShapeShape d(BRepBuilderAPI_MakeVertex(gp_Pnt(v.x, v.y, v.z)).Vertex(), occt);
    if (d.IsDone()) worst = std::max(worst, d.Value());
  }
  return worst;
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

// ── one drafted fixture: native feature::draftFaces vs the OCCT draft oracle ───────
static void draftOne(const char* tag, const nt::Shape& box, const nm::Vec3* dirs, int nDirs,
                     double angleRad, const TopoDS_Shape& occtBox, const nm::Vec3* occtDirs,
                     const gp_Trsf& xf, double defl, double relTol, double spatialTol) {
  int ids[8];
  bool allFound = true;
  for (int i = 0; i < nDirs; ++i) {
    ids[i] = faceByNormal(box, dirs[i]);
    if (ids[i] <= 0) allFound = false;
  }
  report(tag, "face-ids", allFound, allFound ? "found" : "missing");
  if (!allFound) return;

  df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
  const nt::Shape result = df::draftFaces(box, ids, nDirs, angleRad, nm::Point3{0, 0, 0},
                                          nm::Vec3{0, 0, 1}, &why, defl);
  { char b[48]; std::snprintf(b, sizeof(b), "decline=%d", static_cast<int>(why));
    report(tag, "native-draft", !result.isNull(), b); }
  if (result.isNull()) return;

  ntess::MeshParams mp; mp.deflection = defl;
  const ntess::Mesh m = ntess::SolidMesher(mp).mesh(result);
  report(tag, "watertight", ntess::isWatertight(m), "closed 2-manifold");
  report(tag, "oriented", ntess::isConsistentlyOriented(m), "consistent winding");
  { const long chi = eulerChar(m); char b[40]; std::snprintf(b, sizeof(b), "euler=%ld", chi);
    report(tag, "topology", chi == 2, b); }
  const double vNat = std::fabs(ntess::enclosedVolume(m));
  const double aNat = ntess::surfaceArea(m);
  const BBox bNat = meshBBox(m);

  const gp_Dir pull(0, 0, 1);
  const gp_Pln neutral(gp_Pnt(0, 0, 0), pull);
  const TopoDS_Shape occt = occtDraft(occtBox, occtDirs, nDirs, pull, neutral, angleRad, xf);
  report(tag, "occt-single", occtSolidCount(occt) == 1, "one closed solid");
  const double vO = occtVolume(occt), aO = occtArea(occt);
  { const double rel = std::fabs(vNat - vO) / vO;
    char b[96]; std::snprintf(b, sizeof(b), "nat=%.5f occt=%.5f rel=%.3e", vNat, vO, rel);
    report(tag, "volume", rel <= relTol, b); }
  { const double rel = std::fabs(aNat - aO) / aO;
    char b[96]; std::snprintf(b, sizeof(b), "nat=%.5f occt=%.5f rel=%.3e", aNat, aO, rel);
    report(tag, "area", rel <= relTol, b); }
  { Bnd_Box bb; BRepBndLib::AddOptimal(occt, bb, Standard_True, Standard_False);
    double xo[3], xh[3]; bb.Get(xo[0], xo[1], xo[2], xh[0], xh[1], xh[2]);
    double worst = 0.0;
    for (int i = 0; i < 3; ++i) {
      worst = std::max(worst, std::fabs(bNat.lo[i] - xo[i]));
      worst = std::max(worst, std::fabs(bNat.hi[i] - xh[i]));
    }
    char b[80]; std::snprintf(b, sizeof(b), "worst=%.3e tol=%.3e", worst, spatialTol);
    report(tag, "bbox", worst <= spatialTol, b); }
  { const double h = hausdorffNativeToOcct(m, occt);
    char b[64]; std::snprintf(b, sizeof(b), "haus=%.3e tol=%.3e", h, spatialTol);
    report(tag, "hausdorff", h <= spatialTol, b); }
}

int main() {
  std::printf("== MOAT feature draft angle: native-vs-OCCT parity ==\n");
  std::fflush(stdout);
  const double relTol = 0.02, defl = 0.005, spatial = std::max(3.0 * defl, 1e-4);
  gp_Trsf identity;

  // 1. box +X side face, 8° about the base plane (pull +Z).
  { const nt::Shape box = boxPrism(10, 10, 10);
    const TopoDS_Shape occtBox = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(10, 10, 10)).Solid();
    const nm::Vec3 dirs[1] = {{1, 0, 0}};
    draftOne("box/+x/8deg", box, dirs, 1, 8.0 * kPi / 180.0, occtBox, dirs, identity, defl, relTol, spatial); }

  // 2. all four side faces, 5° about the base plane → a frustum.
  { const nt::Shape box = boxPrism(10, 10, 10);
    const TopoDS_Shape occtBox = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(10, 10, 10)).Solid();
    const nm::Vec3 dirs[4] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
    draftOne("box/4-side/5deg", box, dirs, 4, 5.0 * kPi / 180.0, occtBox, dirs, identity, defl, relTol, spatial); }

  // 3. OFF-AXIS: rotate the box 25° about Z, draft its former +X face 8°.
  { nt::Shape box = boxPrism(10, 10, 10);
    const double a = 25.0 * kPi / 180.0;
    box = box.located(nt::Location(nm::Transform::rotationOf(nm::Point3{0, 0, 0}, nm::Dir3{0, 0, 1}, a)));
    const TopoDS_Shape occtBox0 = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(10, 10, 10)).Solid();
    gp_Trsf rot; rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), a);
    const nm::Vec3 dirs[1] = {{std::cos(a), std::sin(a), 0}};
    // OCCT oracle: draft the AXIS-ALIGNED +x face of the un-rotated box, then rotate.
    const nm::Vec3 occtDirs[1] = {{1, 0, 0}};
    draftOne("box/offaxis/8deg", box, dirs, 1, 8.0 * kPi / 180.0, occtBox0, occtDirs, rot, defl, relTol, spatial); }

  // 4. HONEST DECLINE: a cap face (+Z, normal ∥ pull) has no trace line → native NULL.
  { const nt::Shape box = boxPrism(10, 10, 10);
    const int fid = faceByNormal(box, nm::Vec3{0, 0, 1});
    const int ids[1] = {fid};
    df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
    const nt::Shape r = df::draftFaces(box, ids, 1, 5.0 * kPi / 180.0, nm::Point3{0, 0, 0},
                                       nm::Vec3{0, 0, 1}, &why, defl);
    const bool declined = r.isNull() && why == df::DraftFacesDecline::FaceParallelToPull;
    char b[40]; std::snprintf(b, sizeof(b), "decline=%d", static_cast<int>(why));
    report("box/cap/decline", "honest-decline", declined, b); }

  std::printf("== draft parity: %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail == 0 ? 0 : 1;
}
