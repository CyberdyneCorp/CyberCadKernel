// SPDX-License-Identifier: Apache-2.0
//
// native_dense_csg_tangent_sim.mm — a MINIMAL near-tangent / near-coincident DENSE-CSG
// differential leg for the native PLANAR boolean vs the OCCT oracle (iOS simulator).
//
// This is the SHORT sim complement to the primary HOST closed-form battery
// (tests/native/test_native_dense_csg_stress.cpp). The host oracle is authoritative;
// this leg only cross-checks a COUPLE of the mapped near-tangent cases against a real
// OCCT BRepAlgoAPI boolean, confirming the same DISAGREED==0 invariant holds when the
// ground truth is OCCT rather than a closed form. It is deliberately TINY (a fixed
// handful of dense-soup pairs, NO fuzz batch) because the simulator is shared.
//
// Each case builds a DENSE TRIANGLE SOUP operand (facetSolidLocal, mirroring
// thread_apply::facetSolid) so the native BSP hits its dense-soup path — the exact
// regime the thread boolean declines in — and compares native vs OCCT:
//   AGREED            — native watertight + volume within tol of OCCT.
//   HONESTLY-DECLINED — native NULL / non-watertight → OCCT fallback ships (oracle valid).
//   DISAGREED         — native watertight but volume OUTSIDE tol → a SILENT WRONG RESULT.
// Process exits 0 IFF DISAGREED == 0. The agreement tolerance is FIXED (relTol 2e-2,
// native-mesh vs OCCT-exact) and NEVER widened.
//
// src/native stays OCCT-FREE — this harness is additive test/sim code only. It links only
// native math + OCCT (the planar boolean is header-only and does NOT need the numsci/SSI
// substrate), so it is a LIGHTER slice than native_boolean_fuzz.mm. Built + run by
// scripts/run-sim-native-dense-csg-tangent.sh; on run-sim-suite.sh's SKIP list (own main(),
// OCCT slice, std::_Exit).
//
#include "native/boolean/native_boolean.h"       // nb::boolean_solid / Op (header-only)
#include "native/construct/construct.h"           // detail::planarFace
#include "native/construct/native_construct.h"    // build_prism
#include "native/tessellate/native_tessellate.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_dense_csg_tangent_sim requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif

#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

namespace nb = cybercad::native::boolean;
namespace ncst = cybercad::native::construct;
namespace ntess = cybercad::native::tessellate;
namespace ntopo = cybercad::native::topology;
namespace nmath = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeflection = 0.001;  // fine native mesh; OCCT uses exact mass props
constexpr double kRelTol = 2e-2;       // fixed; never widened

ntopo::Shape boxAt(double x0, double y0, double z0, double sx, double sy, double sz) {
  const double p[] = {x0, y0, x0 + sx, y0, x0 + sx, y0 + sy, x0, y0 + sy};
  ntopo::Shape s = ncst::build_prism(p, 4, sz);
  if (z0 != 0.0 && !s.isNull())
    s = s.located(ntopo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, z0})));
  return s;
}

// Dense triangle-soup faceter (mirror of thread_apply::facetSolid).
ntopo::Shape facetSolidLocal(const ntopo::Shape& s, double deflection) {
  ntess::MeshParams p;
  p.deflection = deflection;
  const ntess::Mesh m = ntess::SolidMesher{p}.mesh(s);
  std::vector<ntopo::Shape> faces;
  faces.reserve(m.triangles.size());
  for (const ntess::Triangle& t : m.triangles) {
    const nmath::Point3 a = m.vertices[t.a], b = m.vertices[t.b], c = m.vertices[t.c];
    const nmath::Vec3 n = nmath::cross(b - a, c - a);
    const double nl = nmath::norm(n);
    if (!(nl > 1e-12)) continue;
    std::vector<ntopo::Shape> loop = {ntopo::ShapeBuilder::makeVertex(a),
                                      ntopo::ShapeBuilder::makeVertex(b),
                                      ntopo::ShapeBuilder::makeVertex(c)};
    faces.push_back(
        ncst::detail::planarFace(loop, nmath::Dir3{n / nl}, ntopo::Orientation::Forward));
  }
  if (faces.size() < 4) return {};
  return ntopo::ShapeBuilder::makeSolid({ntopo::ShapeBuilder::makeShell(std::move(faces))});
}

struct NativeMeasure { bool present = false, watertight = false; double volume = 0; };
NativeMeasure measureNative(const ntopo::Shape& s) {
  NativeMeasure m;
  if (s.isNull()) return m;
  m.present = true;
  ntess::MeshParams p; p.deflection = kDeflection;
  const ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(s);
  m.watertight = ntess::isWatertight(mesh);
  m.volume = std::fabs(ntess::enclosedVolume(mesh));
  return m;
}

struct OcctMeasure { bool valid = false; double volume = 0; };
OcctMeasure measureOcct(const TopoDS_Shape& s) {
  OcctMeasure m;
  if (s.IsNull()) return m;
  BRepCheck_Analyzer an(s);
  m.valid = an.IsValid();
  GProp_GProps vg; BRepGProp::VolumeProperties(s, vg); m.volume = std::fabs(vg.Mass());
  return m;
}

TopoDS_Shape occtBox(double x0, double y0, double z0, double sx, double sy, double sz) {
  return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x0 + sx, y0 + sy, z0 + sz)).Shape();
}
TopoDS_Shape occtTiltedBox(double thetaRad) {
  // Upper box [0,10]²×[10,20] rotated by theta about the y-axis line through (0,0,10).
  TopoDS_Shape b = occtBox(0, 0, 10, 10, 10, 10);
  gp_Trsf tr; tr.SetRotation(gp_Ax1(gp_Pnt(0, 0, 10), gp_Dir(0, 1, 0)), thetaRad);
  return BRepBuilderAPI_Transform(b, tr, true).Shape();
}

int g_agreed = 0, g_declined = 0, g_disagreed = 0;

// Classify one native-vs-OCCT fuse and tally.
void checkFuse(const char* tag, const ntopo::Shape& natA, const ntopo::Shape& natB,
               const TopoDS_Shape& occA, const TopoDS_Shape& occB) {
  const ntopo::Shape natRes = nb::boolean_solid(natA, natB, nb::Op::Fuse);
  const NativeMeasure nm = measureNative(natRes);
  const TopoDS_Shape occRes = BRepAlgoAPI_Fuse(occA, occB).Shape();
  const OcctMeasure om = measureOcct(occRes);

  if (!om.valid || om.volume <= 1e-9) {
    std::printf("[dense-tangent-sim] %-24s FALLBACK_ORACLE_INVALID (skipped)\n", tag);
    return;
  }
  const bool nativeUsable = nm.present && nm.watertight;
  if (!nativeUsable) {
    ++g_declined;
    std::printf("[dense-tangent-sim] %-24s HONESTLY-DECLINED (native wt=%d) OCCTvol=%.5f\n",
                tag, (int)nm.watertight, om.volume);
    return;
  }
  const double relErr = std::fabs(nm.volume - om.volume) / om.volume;
  if (relErr <= kRelTol) {
    ++g_agreed;
    std::printf("[dense-tangent-sim] %-24s AGREED  natVol=%.5f OCCTvol=%.5f relErr=%.2e\n",
                tag, nm.volume, om.volume, relErr);
  } else {
    ++g_disagreed;
    std::printf("[dense-tangent-sim] %-24s DISAGREED natVol=%.5f OCCTvol=%.5f relErr=%.2e\n",
                tag, nm.volume, om.volume, relErr);
  }
}

}  // namespace

int main() {
  std::printf("== near-tangent dense-CSG sim leg: native planar boolean vs OCCT (fuse) ==\n");
  std::printf("== relTol=2e-2 deflection=0.001 (minimal; host closed-form oracle is primary) ==\n");
  std::fflush(stdout);

  // Case 1 — CONTROL: dense-soup diagonal overlap (offset 5,5,5). Native lands; AGREE.
  checkFuse("control_diag_5_5_5",
            facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4),
            facetSolidLocal(boxAt(5, 5, 5, 10, 10, 10), 0.4),
            occtBox(0, 0, 0, 10, 10, 10), occtBox(5, 5, 5, 10, 10, 10));

  // Case 2 — BAND 1 near-tangent tilt θ=0.15° (host: cracks → DECLINE). OCCT fuses it.
  {
    ntopo::Shape natA = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4);
    ntopo::Shape upper = boxAt(0, 0, 10, 10, 10, 10);
    upper = upper.located(ntopo::Location(nmath::Transform::rotationOf(
        nmath::Point3{0, 0, 10}, nmath::Dir3{0, 1, 0}, 0.15 * kPi / 180.0)));
    checkFuse("band1_tilt_0.15deg", natA, facetSolidLocal(upper, 0.4),
              occtBox(0, 0, 0, 10, 10, 10), occtTiltedBox(0.15 * kPi / 180.0));
  }

  // Case 3 — BAND 3 near-coincident overlap ε=1e-3 (host: watertight but wrong-vol → the
  // engine self-verify declines it). The sim leg cross-checks that native never presents a
  // volume OUTSIDE tol of OCCT as valid: it must land AGREED or (if the raw mesh is not
  // watertight) DECLINED — never DISAGREED.
  checkFuse("band3_overlap_eps_1e-3",
            facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4),
            facetSolidLocal(boxAt(0, 0, 10 - 1e-3, 10, 10, 10), 0.4),
            occtBox(0, 0, 0, 10, 10, 10), occtBox(0, 0, 10 - 1e-3, 10, 10, 10));

  std::printf("== AGREED=%d HONESTLY-DECLINED=%d DISAGREED=%d ==\n",
              g_agreed, g_declined, g_disagreed);
  std::fflush(stdout);
  const int rc = g_disagreed == 0 ? 0 : 1;
  std::_Exit(rc);  // OCCT static teardown in the trimmed build is not exit-clean.
}
