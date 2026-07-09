// SPDX-License-Identifier: Apache-2.0
//
// native_chamfer_face_parity.mm — MOAT M2 FULL-FACE chamfer weld (`chamfer_face`)
// SIM GATE (b): native-vs-OCCT parity on a booted iOS simulator (OCCT linked).
//
// `chamfer_face` (src/native/blend/chamfer_face.h) chamfers EVERY convex planar-dihedral
// edge bounding a picked planar face at constant setback, by assembling the landed M2
// convex-CORNER weld (`chamfer_corner`) over the face's bounding-edge loop. A single
// face's loop is a set of 2-edge DIHEDRAL corners (never a triple), so it welds
// watertight AND matches OCCT `BRepFilletAPI_MakeChamfer` — which, for a 2-edge dihedral
// corner (a union of two setback half-space prisms), builds the IDENTICAL solid. The
// chamfer is EXACT planar geometry, so native == OCCT to fp64 (not merely
// deflection-bounded).
//
// This harness builds the SAME box two ways — natively (native construct API →
// `nblend::chamfer_face` → native tessellate volume) and under OCCT (`BRepPrimAPI` box →
// `BRepFilletAPI_MakeChamfer` adding every edge of the matching face → `BRepGProp`
// volume) — and asserts native volume == OCCT volume == the closed-form
// inclusion-exclusion value, plus native watertight. Every one of the six cube faces is
// exercised, and a setback sweep on the top face. It also confirms the honest declines
// (curved solid, oversized setback) return a NULL native result (→ OCCT owns it).
//
// Build/run: scripts/run-sim-native-chamfer-face.sh (models run-sim-native-blend.sh —
// links the whole kernel + OCCT; native headers are on the -I src path).

#include "native/blend/native_blend.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

// ── OCCT oracle ────────────────────────────────────────────────────────────────
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>

namespace topo = cybercad::native::topology;
namespace blend = cybercad::native::blend;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;

namespace {

int g_pass = 0, g_fail = 0;
void record(bool ok, const std::string& label, const char* detail) {
  std::printf("[CF] %-4s %-40s %s\n", ok ? "PASS" : "FAIL", label.c_str(), detail);
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

// ── native box + full-face chamfer volume ────────────────────────────────────────
topo::Shape nativeBox(double L) {
  const double p[] = {0, 0, L, 0, L, L, 0, L};
  return cst::build_prism(p, 4, L);
}

// The native face id (1-based mapShapes(Face)) with outward plane normal (nx,ny,nz) at
// offset w (n·p = w), or -1.
int nativeFace(const topo::Shape& s, double nx, double ny, double nz, double w) {
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

struct NativeCF { bool ok = false; bool watertight = false; double vol = 0.0; };
NativeCF nativeChamferFace(const topo::Shape& box, int faceId, double d) {
  NativeCF out;
  blend::ChamferFaceDecline why = blend::ChamferFaceDecline::BadInput;
  const topo::Shape r = blend::chamfer_face(box, faceId, d, &why);
  if (r.isNull()) return out;  // declined → out.ok stays false
  tess::MeshParams mp;
  mp.deflection = 0.004;
  const tess::Mesh m = tess::SolidMesher{mp}.mesh(r);
  out.ok = true;
  out.watertight = tess::isWatertight(m);
  out.vol = std::fabs(tess::enclosedVolume(m));
  return out;
}

// ── OCCT box + chamfer every edge of the face whose outward normal ≈ (nx,ny,nz) ────
double occtVolume(const TopoDS_Shape& s) {
  if (s.IsNull()) return 0.0;
  GProp_GProps props;
  BRepGProp::VolumeProperties(s, props);
  return std::fabs(props.Mass());
}

// OCCT: chamfer (symmetric distance d) every edge bounding the face whose outward normal
// is ≈ (nx,ny,nz). Returns the resulting solid, or a null shape.
TopoDS_Shape occtChamferFace(double L, double d, double nx, double ny, double nz) {
  BRepPrimAPI_MakeBox mk(L, L, L);
  const TopoDS_Shape box = mk.Shape();
  // Find the target face by its outward normal.
  TopoDS_Face target;
  for (TopExp_Explorer ex(box, TopAbs_FACE); ex.More(); ex.Next()) {
    const TopoDS_Face f = TopoDS::Face(ex.Current());
    BRepAdaptor_Surface surf(f);
    if (surf.GetType() != GeomAbs_Plane) continue;
    gp_Dir n = surf.Plane().Axis().Direction();
    if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
    if (std::fabs(n.X() - nx) < 1e-6 && std::fabs(n.Y() - ny) < 1e-6 &&
        std::fabs(n.Z() - nz) < 1e-6) { target = f; break; }
  }
  if (target.IsNull()) return {};
  BRepFilletAPI_MakeChamfer ch(box);
  for (TopExp_Explorer ex(target, TopAbs_EDGE); ex.More(); ex.Next())
    ch.Add(d, TopoDS::Edge(ex.Current()));
  ch.Build();
  if (!ch.IsDone()) return {};
  return ch.Shape();
}

bool nearAbs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

double remFace(double d, double L) { return 2.0 * d * d * L - 4.0 * d * d * d / 3.0; }

// One face of the cube: native chamfer_face == OCCT chamfer-all-face-edges == closed form.
void runFace(const char* label, double L, double d, double nx, double ny, double nz, double w) {
  const topo::Shape box = nativeBox(L);
  const int fid = nativeFace(box, nx, ny, nz, w);
  const NativeCF nat = nativeChamferFace(box, fid, d);
  const TopoDS_Shape occt = occtChamferFace(L, d, nx, ny, nz);
  const double vOcct = occtVolume(occt);
  const double vForm = L * L * L - remFace(d, L);

  char detail[256];
  const bool ok = (fid > 0) && nat.ok && nat.watertight &&
                  nearAbs(nat.vol, vOcct, 1e-6) && nearAbs(vOcct, vForm, 1e-6);
  std::snprintf(detail, sizeof detail,
                "faceId=%d wt=%d  native=%.9f  OCCT=%.9f  closed-form=%.9f",
                fid, nat.watertight ? 1 : 0, nat.vol, vOcct, vForm);
  record(ok, label, detail);
}

}  // namespace

int main() {
  std::printf("== MOAT M2 full-face chamfer weld (chamfer_face): native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  const double L = 10.0;

  // ── every cube face: native == OCCT == closed form (EXACT planar chamfer) ────────
  runFace("chamfer-face +Z d=1.5", L, 1.5, 0, 0, 1, L);
  runFace("chamfer-face -Z d=1.5", L, 1.5, 0, 0, -1, 0);
  runFace("chamfer-face +X d=1.5", L, 1.5, 1, 0, 0, L);
  runFace("chamfer-face -X d=1.5", L, 1.5, -1, 0, 0, 0);
  runFace("chamfer-face +Y d=1.5", L, 1.5, 0, 1, 0, L);
  runFace("chamfer-face -Y d=1.5", L, 1.5, 0, -1, 0, 0);

  // ── setback sweep on the top face: tracks OCCT + the closed form at every size ───
  for (double d : {0.5, 1.0, 2.0, 3.0}) {
    char lab[48];
    std::snprintf(lab, sizeof lab, "chamfer-face +Z d=%.1f", d);
    runFace(lab, L, d, 0, 0, 1, L);
  }

  // ── honest declines: NULL native result → OCCT owns it (never a wrong solid) ─────
  {
    // Oversized setback (>= half the face): the fit guard / weld self-verify declines.
    const topo::Shape box = nativeBox(L);
    const int top = nativeFace(box, 0, 0, 1, L);
    blend::ChamferFaceDecline why = blend::ChamferFaceDecline::Ok;
    const topo::Shape r = blend::chamfer_face(box, top, 20.0, &why);
    char detail[160];
    const bool ok = r.isNull() && (why == blend::ChamferFaceDecline::NoConvexEdges ||
                                   why == blend::ChamferFaceDecline::WeldFailed);
    std::snprintf(detail, sizeof detail, "native NULL=%d why=%s (→ OCCT)",
                  r.isNull() ? 1 : 0, blend::chamferFaceDeclineName(why));
    record(ok, "chamfer-face oversized declines", detail);
  }

  std::printf("[CF] SUMMARY %d passed / %d failed\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail ? 1 : 0);
}
