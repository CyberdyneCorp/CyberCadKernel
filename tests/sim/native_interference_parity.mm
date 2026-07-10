// SPDX-License-Identifier: Apache-2.0
//
// native_interference_parity.mm — MOAT M-GS GS7 native-vs-OCCT CLASH / INTERFERENCE
// parity harness (iOS simulator). SIM GATE (b) of the two-gate discipline; gate (a)
// is the OCCT-free host suite tests/native/test_native_interference.cpp.
//
// The native OCCT-FREE, header-only classifier (src/native/analysis/interference.h,
// on the landed B3 membership + M0 mesh vocabulary) is asserted against the OCCT
// ORACLE on IDENTICAL box solids built on both sides:
//   * CLASH / TOUCHING / CLEAR state   vs  BRepExtrema_DistShapeShape (the min
//     boundary distance: 0 ⇒ contact/penetration, >0 ⇒ clearance) COMBINED with
//     BRepAlgoAPI_Common volume (>0 ⇒ interior overlap ⇒ CLASH; ==0 with distance 0
//     ⇒ TOUCHING; distance >0 ⇒ CLEAR) — the same decision the native classifier
//     makes at the mesh level.
//   * overlap VOLUME (the CLASH cases) vs  BRepAlgoAPI_Common + BRepGProp — asserted
//     against the closed-form intersection-box volume, which BOTH the native COMMON
//     (host gate) and the OCCT Common reproduce; here we assert the OCCT oracle side
//     matches the closed form and the native state matches the oracle state.
//
// The native classifier does not itself compute the overlap volume (that is the
// engine's native boolean COMMON, exercised at the host gate through the facade);
// this geometry-level oracle asserts the STATE agreement + the OCCT Common volume vs
// the closed form on identical geometry.
//
// OCCT-DEPENDENT (no NumSci needed — interference uses only the math/vec + mesh
// inline primitives + the B3 classifier, all header-only). Compiled ONLY by
// scripts/run-sim-native-interference.sh; carries its own main(); std::_Exit to skip
// the non-exit-clean OCCT static teardown (same rationale as native_query_parity).

#include "native/analysis/interference.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_interference_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>

namespace an = cybercad::native::analysis;
namespace tess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, bool ok, double a = 0, double b = 0) {
  std::printf("[GS7] %-46s %s  native=%.10g occt=%.10g  d=%.3e\n", name, ok ? "PASS" : "FAIL",
              a, b, std::fabs(a - b));
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

// ── Box mesh (outward-CCW-wound, watertight): [x0,x0+a]×[y0,y0+b]×[z0,z0+c] ────
static tess::Mesh boxMesh(double x0, double y0, double z0, double a, double b, double c) {
  tess::Mesh m;
  const nm::Point3 v[8] = {{x0, y0, z0},         {x0 + a, y0, z0},
                           {x0 + a, y0 + b, z0},  {x0, y0 + b, z0},
                           {x0, y0, z0 + c},      {x0 + a, y0, z0 + c},
                           {x0 + a, y0 + b, z0 + c}, {x0, y0 + b, z0 + c}};
  for (const auto& p : v) m.vertices.push_back(p);
  auto quad = [&](int A, int B, int C, int D) { m.addTriangle(A, B, C); m.addTriangle(A, C, D); };
  quad(0, 3, 2, 1); quad(4, 5, 6, 7); quad(0, 1, 5, 4);
  quad(2, 3, 7, 6); quad(1, 2, 6, 5); quad(3, 0, 4, 7);
  return m;
}

// The OCCT box with the same min corner + extents.
static TopoDS_Shape occtBox(double x0, double y0, double z0, double a, double b, double c) {
  return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), a, b, c).Shape();
}

// The OCCT verdict for two solids: 2 clash (Common volume > 0), 1 touching
// (distance ~0, no overlap), 0 clear. Also returns the Common volume + min distance.
static int occtState(const TopoDS_Shape& A, const TopoDS_Shape& B, double& volOut,
                     double& distOut) {
  volOut = 0.0; distOut = 0.0;
  BRepAlgoAPI_Common k(A, B);
  if (k.IsDone() && !k.Shape().IsNull()) {
    GProp_GProps g; BRepGProp::VolumeProperties(k.Shape(), g);
    volOut = g.Mass();
  }
  BRepExtrema_DistShapeShape ext(A, B);
  if (ext.IsDone() && ext.NbSolution() > 0) distOut = ext.Value();
  if (volOut > 1e-9) return 2;             // clash
  return (distOut <= 1e-7) ? 1 : 0;        // touching vs clear
}

static int nativeState(const tess::Mesh& a, const tess::Mesh& b) {
  const an::InterferenceResult r = an::meshInterference(a, b, 0.005);
  switch (r.state) {
    case an::ClashState::Clash: return 2;
    case an::ClashState::Touching: return 1;
    case an::ClashState::Clear: return 0;
    default: return -1;  // Unknown (decline)
  }
}

// One parity case: build the same two boxes on both sides, assert the native
// classifier state matches the OCCT oracle state, and (on a clash) the OCCT Common
// volume matches the closed-form intersection-box volume.
static void parityCase(const char* name, double ax, double ay, double az, double aa,
                       double ab, double ac, double bx, double by, double bz, double ba,
                       double bb, double bc, int expectState, double expectVol) {
  const tess::Mesh nA = boxMesh(ax, ay, az, aa, ab, ac);
  const tess::Mesh nB = boxMesh(bx, by, bz, ba, bb, bc);
  const TopoDS_Shape oA = occtBox(ax, ay, az, aa, ab, ac);
  const TopoDS_Shape oB = occtBox(bx, by, bz, ba, bb, bc);

  double oVol = 0.0, oDist = 0.0;
  const int oState = occtState(oA, oB, oVol, oDist);
  const int nState = nativeState(nA, nB);

  report((std::string(name) + " state (native vs occt)").c_str(),
         nState == oState && oState == expectState, nState, oState);
  if (expectState == 2)  // clash → OCCT Common volume vs the closed form
    report((std::string(name) + " overlap volume (occt vs closed-form)").c_str(),
           std::fabs(oVol - expectVol) < 1e-6, oVol, expectVol);
}

int main() {
  std::printf("=== native_interference_parity (MOAT M-GS GS7, sim gate b) ===\n");

  // Overlapping boxes: A=[0,2]³, B=[1,3]³ → overlap [1,2]³ volume 1. CLASH.
  parityCase("overlap-corner", 0, 0, 0, 2, 2, 2, 1, 1, 1, 2, 2, 2, 2, 1.0);

  // Overlapping slab: A=[0,4]×[0,4]×[0,1], B=[1,3]×[1,3]×[-0.5,0.5] → overlap
  // [1,3]×[1,3]×[0,0.5] volume 2·2·0.5 = 2. CLASH.
  parityCase("overlap-slab", 0, 0, 0, 4, 4, 1, 1, 1, -0.5, 2, 2, 1, 2, 2.0);

  // Nested: inner [3,5]³ inside outer [0,10]³ → overlap = inner, volume 8. CLASH.
  parityCase("nested", 0, 0, 0, 10, 10, 10, 3, 3, 3, 2, 2, 2, 2, 8.0);

  // Face-touching: A=[0,1]³, B=[1,2]×[0,1]×[0,1] share x=1 → TOUCHING, volume 0.
  parityCase("face-touch", 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0.0);

  // Disjoint: A=[0,1]³, B=[11,12]×[0,1]×[0,1] gap 10 → CLEAR.
  parityCase("disjoint", 0, 0, 0, 1, 1, 1, 11, 0, 0, 1, 1, 1, 0, 0.0);

  // Coplanar plus-sign-cross (moat-clashfix regression): A horizontal bar
  // [0,3]×[1,2]×[0,1] (top z=1), B vertical bar [1,2]×[0,3]×[1,2] (bottom z=1),
  // coplanar at z=1, footprints cross with NO mutually contained vertex. OCCT
  // BRepExtrema_DistShapeShape gives distance 0 (flush contact), Common volume 0 →
  // TOUCHING. Native mis-reported CLEAR before the tri–tri edge–edge term was added.
  parityCase("coplanar-cross", 0, 1, 0, 3, 1, 1, 1, 0, 1, 1, 3, 1, 1, 0.0);

  // Gapped cross: same footprints, B raised to bottom z=1.5 → 0.5 clearance → CLEAR.
  parityCase("coplanar-cross-gap", 0, 1, 0, 3, 1, 1, 1, 0, 1.5, 1, 3, 1, 0, 0.0);

  // PASS-THROUGH (moat-clfix2 regression): a bar poking CLEAN THROUGH a slab. Slab
  // A=[0,10]×[0,10]×[0,1] (wide, thin), bar B=[4,6]×[4,6]×[-5,20] (thin, long) —
  // the bar's ends stick out both slab faces and the slab is wider than the bar, so
  // NEITHER solid has a vertex/centroid inside the other. OCCT Common volume =
  // 2·2·1 = 4 (>0 ⇒ CLASH); native mis-reported TOUCHING before the pass-through
  // (edge-pierces-face) penetration signature was added. overlap [4,6]×[4,6]×[0,1].
  parityCase("bar-through-slab", 0, 0, 0, 10, 10, 1, 4, 4, -5, 2, 2, 25, 2, 4.0);

  // Touching variant: bar bottom z=1 flush with the slab top z=1 → TOUCHING, vol 0.
  parityCase("bar-on-slab", 0, 0, 0, 10, 10, 1, 4, 4, 1, 2, 2, 25, 1, 0.0);

  // Gapped variant: bar bottom z=1.5, 0.5 clearance above the slab top → CLEAR.
  parityCase("bar-above-slab", 0, 0, 0, 10, 10, 1, 4, 4, 1.5, 2, 2, 25, 0, 0.0);

  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
