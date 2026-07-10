// SPDX-License-Identifier: Apache-2.0
//
// native_split_plane_parity.mm — MOAT M-DM DM1 SIM GATE (b): the FIRST native
// direct-modeling verb (planar half-space SPLIT) native-vs-OCCT on a booted iOS
// simulator.
//
// The native split (src/native/boolean/split_plane.h, OCCT-FREE) composes the two
// landed, already-gated verbs — freeformHalfSpaceCut for a freeform-walled operand,
// else boolean_solid against a discard half-space box — into the two pieces of a plane
// cut, picking the surviving half by `keepPositive`. This harness proves, against the
// OCCT ORACLE, that each native piece matches the OCCT piece from the SAME oracle the
// facade's cc_split_plane uses (BRepPrimAPI_MakeHalfSpace + BRepAlgoAPI_Cut, sized
// L=2·(diag+1) — verbatim OcctEngine::split_plane), on:
//   * VOLUME     — native enclosedVolume vs BRepGProp::VolumeProperties (rel ≤ 2e-2);
//   * AREA       — native surfaceArea vs BRepGProp::SurfaceProperties (rel ≤ 2e-2);
//   * WATERTIGHT — the native piece mesh is a closed 2-manifold;
//   * TOPOLOGY   — the native piece Euler χ = 2 (single closed genus-0 solid) and OCCT
//                  reports one solid;
//   * BBOX       — native vertex min/max vs BRepBndLib per-axis (abs ≤ 1.5·deflection);
// and PARTITION-CLOSURE on the native side: V(below)+V(above)=V(whole).
//
// Fixtures (the two reachable DM1 domains): an axis-aligned BOX [0,10]³ cut by x=5, and
// the bowl-lidded convex-quad PRISM cut by x=0 — each for BOTH keep sides. OCCT is the
// ORACLE ONLY, never linked into src/native. Build: run-sim-native-split-plane.sh.
//
#include "native/boolean/split_plane.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/first_freeform_boolean_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_split_plane_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pln.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

namespace bo = cybercad::native::boolean;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace ncst = cybercad::native::construct;
namespace ffx = first_freeform_boolean_fixture;
namespace fx = face_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[DM1] %-18s %-16s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT oracle: reconstruct the bowl-lidded convex-quad prism (mirrors the fixture) ──
static TopoDS_Face buildOcctBowlTop() {
  const std::vector<nm::Point3> poles = fx::bowlPoles();
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);
  const auto& q = fx::quadUV();
  BRepBuilderAPI_MakeWire mkWire;
  for (int k = 0; k < 4; ++k) {
    const nm::Point3 a = q[k], b = q[(k + 1) % 4];
    const gp_Pnt2d p0(a.x, a.y), p1(b.x, b.y);
    const double len = p0.Distance(p1);
    gp_Dir2d dir(p1.X() - p0.X(), p1.Y() - p0.Y());
    Handle(Geom2d_Line) line = new Geom2d_Line(p0, dir);
    Handle(Geom2d_TrimmedCurve) seg = new Geom2d_TrimmedCurve(line, 0.0, len);
    mkWire.Add(BRepBuilderAPI_MakeEdge(seg, surf, 0.0, len).Edge());
  }
  TopoDS_Face face = BRepBuilderAPI_MakeFace(surf, mkWire.Wire(), Standard_True).Face();
  BRepLib::BuildCurves3d(face);
  return face;
}

static TopoDS_Shape buildOcctPrism() {
  const nt::FaceSurface bowl = fx::bowlSurface();
  ntess::SurfaceEvaluator eval(bowl, nt::Location{});
  const auto& q = fx::quadUV();
  std::array<nm::Point3, 4> T, B, ctrl;
  for (int k = 0; k < 4; ++k) {
    T[k] = eval.value(q[k].x, q[k].y);
    B[k] = nm::Point3{T[k].x, T[k].y, -ffx::kH0};
  }
  for (int k = 0; k < 4; ++k) {
    const int k1 = (k + 1) % 4;
    const nm::Point3 m{(q[k].x + q[k1].x) * 0.5, (q[k].y + q[k1].y) * 0.5, 0.0};
    const nm::Point3 S0 = T[k], S1 = T[k1], Sm = eval.value(m.x, m.y);
    ctrl[k] = nm::Point3{2 * Sm.x - 0.5 * (S0.x + S1.x), 2 * Sm.y - 0.5 * (S0.y + S1.y),
                         2 * Sm.z - 0.5 * (S0.z + S1.z)};
  }
  BRepBuilderAPI_Sewing sew(1e-6);
  sew.Add(buildOcctBowlTop());
  for (int k = 0; k < 4; ++k) {
    const int k1 = (k + 1) % 4;
    TColgp_Array1OfPnt bp(1, 3);
    bp.SetValue(1, P(T[k1])); bp.SetValue(2, P(ctrl[k])); bp.SetValue(3, P(T[k]));
    Handle(Geom_BezierCurve) top = new Geom_BezierCurve(bp);
    BRepBuilderAPI_MakeWire w;
    w.Add(BRepBuilderAPI_MakeEdge(P(B[k]), P(B[k1])).Edge());
    w.Add(BRepBuilderAPI_MakeEdge(P(B[k1]), P(T[k1])).Edge());
    w.Add(BRepBuilderAPI_MakeEdge(top).Edge());
    w.Add(BRepBuilderAPI_MakeEdge(P(T[k]), P(B[k])).Edge());
    sew.Add(BRepBuilderAPI_MakeFace(w.Wire(), Standard_True).Face());
  }
  {
    BRepBuilderAPI_MakeWire w;
    for (int k = 0; k < 4; ++k)
      w.Add(BRepBuilderAPI_MakeEdge(P(B[k]), P(B[(k + 1) % 4])).Edge());
    sew.Add(BRepBuilderAPI_MakeFace(w.Wire(), Standard_True).Face());
  }
  sew.Perform();
  BRepBuilderAPI_MakeSolid mk;
  for (TopExp_Explorer ex(sew.SewedShape(), TopAbs_SHELL); ex.More(); ex.Next())
    mk.Add(TopoDS::Shell(ex.Current()));
  return mk.Solid();
}

// The OCCT split_plane oracle — VERBATIM OcctEngine::split_plane (the fallback the
// facade's cc_split_plane uses): plane face sized L=2·(diag+1), MakeHalfSpace on the
// DISCARD side, Cut.
static TopoDS_Shape occtSplitPlane(const TopoDS_Shape& body, const nm::Point3& o,
                                   const nm::Vec3& n, bool keepPositive) {
  const double nlen = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
  const gp_Dir nd(n.x / nlen, n.y / nlen, n.z / nlen);
  const gp_Pnt op(o.x, o.y, o.z);
  const gp_Pln pln(op, nd);
  Bnd_Box bb; BRepBndLib::Add(body, bb);
  Standard_Real xm, ym, zm, xM, yM, zM;
  bb.Get(xm, ym, zm, xM, yM, zM);
  const double L = 2.0 * (gp_Pnt(xm, ym, zm).Distance(gp_Pnt(xM, yM, zM)) + 1.0);
  TopoDS_Face face = BRepBuilderAPI_MakeFace(pln, -L, L, -L, L).Face();
  const double s = keepPositive ? -1.0 : 1.0;
  const gp_Pnt ref(op.X() + nd.X() * s * L * 0.25, op.Y() + nd.Y() * s * L * 0.25,
                   op.Z() + nd.Z() * s * L * 0.25);
  BRepPrimAPI_MakeHalfSpace mkHS(face, ref);
  BRepAlgoAPI_Cut cut(body, mkHS.Solid());
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

// One reachable case × one keep side: native split vs OCCT oracle on every metric.
// Returns the native piece volume (for the partition-closure cross-check), or -1 on a
// native failure (which is a BLOCKER on a reachable case).
static double compareOne(const char* tag, const nt::Shape& nativeOperand,
                         const TopoDS_Shape& occtOperand, const nm::Point3& o, const nm::Vec3& n,
                         bool keepPositive, double defl, double relTol, double spatialTol) {
  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
  const nt::Shape piece = bo::splitByPlane(nativeOperand, o, n, keepPositive, defl, &why);
  {
    char buf[64]; std::snprintf(buf, sizeof(buf), "decline=%s", bo::declineName(why));
    report(tag, "native-split", !piece.isNull(), buf);
  }
  if (piece.isNull()) return -1.0;

  ntess::MeshParams mp; mp.deflection = defl;
  const ntess::Mesh m = ntess::SolidMesher(mp).mesh(piece);
  report(tag, "watertight", ntess::isWatertight(m), "closed 2-manifold");
  {
    const long chi = eulerChar(m);
    char buf[40]; std::snprintf(buf, sizeof(buf), "euler=%ld", chi);
    report(tag, "topology", chi == 2, buf);
  }
  const double vNat = std::fabs(ntess::enclosedVolume(m));
  const double aNat = ntess::surfaceArea(m);
  const BBox bNat = meshBBox(m);

  const TopoDS_Shape occtPiece = occtSplitPlane(occtOperand, o, n, keepPositive);
  report(tag, "occt-single", occtSolidCount(occtPiece) == 1, "one closed solid");
  const double vOcct = occtVolume(occtPiece);
  const double aOcct = occtArea(occtPiece);
  {
    const double rel = std::fabs(vNat - vOcct) / vOcct;
    char buf[96]; std::snprintf(buf, sizeof(buf), "nat=%.6f occt=%.6f rel=%.3e", vNat, vOcct, rel);
    report(tag, "volume", rel <= relTol, buf);
  }
  {
    const double rel = std::fabs(aNat - aOcct) / aOcct;
    char buf[96]; std::snprintf(buf, sizeof(buf), "nat=%.6f occt=%.6f rel=%.3e", aNat, aOcct, rel);
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
  return vNat;
}

static void closure(const char* tag, double vBelow, double vAbove, double vFull, double relTol) {
  if (vBelow < 0 || vAbove < 0) { report(tag, "closure", false, "a piece failed"); return; }
  const double rel = std::fabs((vBelow + vAbove) - vFull) / vFull;
  char buf[96];
  std::snprintf(buf, sizeof(buf), "b+a=%.6f full=%.6f rel=%.3e", vBelow + vAbove, vFull, rel);
  report(tag, "closure", rel <= relTol, buf);
}

int main() {
  std::printf("== MOAT M-DM DM1 native cc_split_plane: native-vs-OCCT parity ==\n");
  std::fflush(stdout);
  const double relTol = 0.02;

  // ── Fixture 1: axis-aligned BOX [0,10]³, plane x=5, both keep sides ────────────
  {
    const double defl = 0.005, spatial = std::max(1.5 * defl, 1e-6);
    const nm::Point3 o{5, 0, 0}; const nm::Vec3 n{1, 0, 0};
    const double boxProfile[8] = {0, 0, 10, 0, 10, 10, 0, 10};
    const nt::Shape box = ncst::build_prism(boxProfile, 4, 10.0);
    const TopoDS_Shape occtBox = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(10, 10, 10)).Solid();
    const double vB = compareOne("box/below", box, occtBox, o, n, false, defl, relTol, spatial);
    const double vA = compareOne("box/above", box, occtBox, o, n, true, defl, relTol, spatial);
    closure("box", vB, vA, 1000.0, 1e-3);
  }

  // ── Fixture 2: bowl-lidded PRISM, plane x=0, both keep sides ───────────────────
  {
    const double defl = 0.008, spatial = 1.5 * defl;
    const nm::Point3 o{0, 0, 0}; const nm::Vec3 n{1, 0, 0};
    const nt::Shape prism = ffx::buildOperand();
    const TopoDS_Shape occtPrism = buildOcctPrism();
    report("prism/operand", "occt-solid", occtSolidCount(occtPrism) == 1, "sewn 6-face solid");
    const double vB = compareOne("prism/below", prism, occtPrism, o, n, false, defl, relTol, spatial);
    const double vA = compareOne("prism/above", prism, occtPrism, o, n, true, defl, relTol, spatial);
    closure("prism", vB, vA, ffx::fullVolume(), 0.04);
  }

  // ── Fixture 3 (F4): bowl-lidded PRISM, OFF-CENTRE plane x=0.10, both keep sides ──
  // The frozen keep-face was volume-exact only for the symmetric-centre cut (~29% over at
  // x=0.10). With the cross-section cap oriented by the mesher's real +fr.z convention
  // (planarFaceFromLoopByNormal), the OFF-CENTRE cut is consistently oriented and its
  // volume matches OCCT's BRepAlgoAPI_Cut + BRepGProp at the same 2% band as the centre.
  {
    const double defl = 0.008, spatial = 1.5 * defl;
    const nm::Point3 o{0.10, 0, 0}; const nm::Vec3 n{1, 0, 0};
    const nt::Shape prism = ffx::buildOperand();
    const TopoDS_Shape occtPrism = buildOcctPrism();
    const double vB = compareOne("prism-off/below", prism, occtPrism, o, n, false, defl, relTol, spatial);
    const double vA = compareOne("prism-off/above", prism, occtPrism, o, n, true, defl, relTol, spatial);
    closure("prism-off", vB, vA, ffx::fullVolume(), 0.04);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
