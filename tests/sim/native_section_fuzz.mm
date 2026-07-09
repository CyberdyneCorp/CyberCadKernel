// SPDX-License-Identifier: Apache-2.0
//
// native_section_fuzz.mm — MOAT M6-breadth-12 (the COMPLETENESS BAR, TWELFTH domain):
// a SECTION-CURVE differential-fuzzing harness (iOS simulator) for the native planar
// section service (cybercad::native::section::sectionByPlane, src/native/section/
// section.h) the CyberCad app's drawing / section-view path reads through the additive
// cc_section_plane facade.
//
// This extends the eleven landed M6 differential fuzzers — native_boolean_fuzz.mm
// (curved booleans), native_step_import_fuzz.mm (STEP round-trip), native_construct_fuzz
// .mm (loft/sweep), native_blend_fuzz.mm (fillet/chamfer/offset/shell), native_wrap_emboss
// _fuzz.mm, native_mass_props_fuzz.mm, native_geometry_services_fuzz.mm,
// native_transform_fuzz.mm, native_reference_geometry_fuzz.mm, native_directmodel_fuzz.mm
// and native_transformed_boolean_fuzz.mm — to a TWELFTH independent native domain: the
// closed-form planar SECTION-CURVE extractor. Like its siblings it is INFRASTRUCTURE (a
// seeded harness, not a geometry capability): OCCT BRepAlgoAPI_Section is the ORACLE, a
// CLOSED-FORM analytic conic is the PRIMARY arbiter, the bar is ZERO silent wrong sections
// over a seeded batch, and an HONEST DECLINE (native Empty/Declined → OCCT) is first-class.
//
// ── WHY SECTION IS A DISTINCT, UN-FUZZED, STABLE DOMAIN ──────────────────────────────
// There is a native_section_parity.mm — but it is a small FIXED-fixture gate (a handful of
// box/cylinder/sphere cuts). This is the first SEEDED, N≥60/seed DIFFERENTIAL FUZZER of the
// section service: random primitive dimensions AND random cut planes (axis-aligned +
// OBLIQUE) per family, arbitrated by a closed-form conic that is EXACT for the elementary
// targets. The section service lives in src/native/section/ — a directory the concurrent
// M3 workflow (blend/feature/boolean) does NOT touch, so it is STABLE and not-overlapping.
// It is OCCT-FREE and needs NO numsci substrate (like native_section_parity), so this
// harness compiles only the OCCT-free native math TUs alongside the header-only section /
// topology / ssi-S1 headers, and links the OCCT oracle toolkits.
//
// ── THE NATIVE SECTION PATH, DRIVEN AT ITS C++ BOUNDARY (the system under test) ───────
// sectionByPlane intersects every face of the solid with the cut plane (closed-form conics
// via the landed SSI Stage-S1 intersector, consumed READ-ONLY), clips edges to the finite
// face, and ASSEMBLES closed loops. It returns exact analytic loops (Circle/Ellipse/Polygon)
// with a closed-form length() and area(); a config it does not robustly handle (plane
// coincident/tangent to a face, a section that does not close, a curved-face conic trimmed
// by the finite face) is HONESTLY DECLINED — never a wrong or open section. We drive it at
// the cybercad::native::section boundary (like native_section_parity), building the native
// solids with the same ShapeBuilder fixtures and the OCCT solids independently with
// BRepPrimAPI_Make{Box,Cylinder,Sphere} to the SAME dimensions.
//
// ── THE ARBITER: A CLOSED-FORM CONIC IS PRIMARY, OCCT IS THE ORACLE ──────────────────
// For every AGREE family the section is an EXACT elementary conic whose perimeter and
// enclosed area have a closed form in plain fp64:
//   BOX (axis cut)        rectangle   L = 2(w+h)          A = w·h
//   CYL perpendicular     circle      L = 2πR             A = πR²
//   CYL axial (thru axis) rectangle   L = 2(2R+H)         A = 2R·H
//   CYL oblique (in band) ellipse     L = Ramanujan-II    A = πab   (a=R/|cosθ|, b=R)
//   SPHERE                circle      L = 2πr             A = πr²   (r=√(R²−d²))
// The native loop's length()/area() ARE these closed forms (Circle/Ellipse analytic
// fields; Polygon shoelace), so native-vs-analytic is EXACT to machine epsilon and the
// analytic value is the PRIMARY correctness signal. OCCT BRepAlgoAPI_Section is the
// SECONDARY oracle: edge length via GCPnts_AbscissaPoint::Length (the dedicated adaptive
// arc-length integrator native_section_parity proved converges to the true perimeter —
// BRepGProp::LinearProperties under-resolves an analytic Ellipse by ~1e-4), loop count via
// ShapeAnalysis_FreeBounds wire recovery, capped area via BRepGProp::SurfaceProperties on
// the section face(s). The classifier attributes any native-vs-OCCT gap with the analytic
// truth: native matching exact math while OCCT does not is an ORACLE limitation (native
// vindicated), NOT a native fault.
//
// ── TOLERANCE (deflection-matched, NEVER widened) ────────────────────────────────────
// Section geometry is EXACT analytic (no tessellation / deflection), so the straight and
// circular cases are held TIGHT: native-vs-analytic 1e-9 relative. The ELLIPSE perimeter is
// the ONLY approximated quantity — native uses the Ramanujan-II series (relative error
// ≪1e-4 at the eccentricities an oblique cylinder cut produces) and OCCT's arc-length
// integrator converges to the true value, so the ellipse-length tolerance is 1e-4 relative
// (the SAME bound native_section_parity proved and NEVER widened here). Area is exact
// closed-form for every family (πab is exact) held to 1e-6 relative against OCCT's
// SurfaceProperties. The tight analytic bound is the certifying arbiter; the OCCT bound is
// matched to OCCT's own integrator accuracy and is not loosened to launder a disagreement.
//
// ── DETERMINISM ──────────────────────────────────────────────────────────────────────
// The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand():
// a splitmix64-seeded xoshiro256** stream, same seed → byte-identical batch on any machine.
//
// src/native / src/engine / include / the cc_* ABI stay BYTE-UNCHANGED; this harness DRIVES
// the native section boundary rather than modifying it. On run-sim-suite.sh's SKIP list
// (own main(), OCCT slice). Built ONLY by scripts/run-sim-native-section-fuzz.sh.
//
// Usage: scripts/run-sim-native-section-fuzz.sh [SEED] [N]
//   SEED  explicit uint64 RNG seed (default 0x5EC7104FEED). Also honoured via FUZZ_SEED env.
//   N     number of generated cases (default 96).           Also honoured via FUZZ_N env.
//
#include "native/section/native_section.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <GProp_GProps.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace sec = cybercad::native::section;
namespace topo = cybercad::native::topology;
namespace nmath = cybercad::native::math;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Point3;
using nmath::Vec3;
using topo::EdgeCurve;
using topo::FaceSurface;
using topo::Shape;
using topo::ShapeBuilder;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream (verbatim discipline of
//    the sibling native_*_fuzz harnesses). Keyed ONLY by an explicit uint64 seed. No
//    clock, no rand(): same seed → byte-identical batch. ─────────────────────────────
struct Rng {
  uint64_t s[4];
  static uint64_t splitmix64(uint64_t& x) {
    uint64_t z = (x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  explicit Rng(uint64_t seed) { for (auto& v : s) v = splitmix64(seed); }
  static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  uint64_t next() {
    const uint64_t r = rotl(s[1] * 5, 7) * 9;
    const uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return r;
  }
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }  // [0,1)
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
};

// ── families ─────────────────────────────────────────────────────────────────────────
enum Family { F_BOX, F_CYL_PERP, F_CYL_AXIAL, F_CYL_OBL, F_SPHERE, F_DECLINE, F_COUNT };
const char* famName(int f) {
  switch (f) {
    case F_BOX:       return "box(rect polygon)";
    case F_CYL_PERP:  return "cyl-perp(circle)";
    case F_CYL_AXIAL: return "cyl-axial(rect polygon)";
    case F_CYL_OBL:   return "cyl-oblique(ellipse)";
    case F_SPHERE:    return "sphere(circle)";
    case F_DECLINE:   return "decline-exerciser";
  }
  return "?";
}

enum Verdict { AGREED, HONESTLY_DECLINED, DISAGREED, ORACLE_UNRELIABLE, ORACLE_BAD };

// ── native fixture builders (mirror the host GATE-a / parity suites) ─────────────────
Shape lineEdge(const Point3& a, const Point3& b) {
  const Vec3 d = b - a;
  EdgeCurve c; c.kind = EdgeCurve::Kind::Line;
  c.frame = Ax3{a, Dir3{d}, Dir3{}, Dir3{}};
  return ShapeBuilder::makeEdge(c, 0.0, nmath::norm(d), ShapeBuilder::makeVertex(a),
                                ShapeBuilder::makeVertex(b));
}
Shape circleEdge(const Point3& ctr, double r, const Dir3& n, const Dir3& x) {
  EdgeCurve c; c.kind = EdgeCurve::Kind::Circle;
  c.frame = Ax3::fromAxisAndRef(ctr, n, x); c.radius = r;
  return ShapeBuilder::makeEdge(c, 0.0, 2.0 * kPi, Shape{}, Shape{});
}
Shape planarQuad(const Point3& p0, const Point3& p1, const Point3& p2, const Point3& p3,
                 const Dir3& n) {
  FaceSurface s; s.kind = FaceSurface::Kind::Plane;
  s.frame = Ax3::fromAxisAndRef(p0, n, Dir3{p1 - p0});
  return ShapeBuilder::makeFace(
      s, ShapeBuilder::makeWire({lineEdge(p0, p1), lineEdge(p1, p2), lineEdge(p2, p3),
                                 lineEdge(p3, p0)}));
}
// Box with one corner at origin, extents (lx,ly,lz). Matches BRepPrimAPI_MakeBox.
Shape makeBox(double lx, double ly, double lz) {
  const Point3 a{0,0,0}, b{lx,0,0}, c{lx,ly,0}, d{0,ly,0};
  const Point3 e{0,0,lz}, f{lx,0,lz}, g{lx,ly,lz}, h{0,ly,lz};
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({
      planarQuad(a,d,c,b, Dir3{0,0,-1}), planarQuad(e,f,g,h, Dir3{0,0,1}),
      planarQuad(a,b,f,e, Dir3{0,-1,0}), planarQuad(d,h,g,c, Dir3{0,1,0}),
      planarQuad(a,e,h,d, Dir3{-1,0,0}), planarQuad(b,c,g,f, Dir3{1,0,0})})});
}
// Cylinder radius R, height H, axis +Z from origin. Matches BRepPrimAPI_MakeCylinder.
Shape makeCylinder(double R, double H) {
  const Dir3 zp{0,0,1}, xp{1,0,0};
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, Dir3{0,0,-1}, xp);
  Shape bottom = ShapeBuilder::makeFace(bs, ShapeBuilder::makeWire({circleEdge(Point3{0,0,0}, R, zp, xp)}));
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0,0,H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(ts, ShapeBuilder::makeWire({circleEdge(Point3{0,0,H}, R, zp, xp)}));
  FaceSurface ls; ls.kind = FaceSurface::Kind::Cylinder;
  ls.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, zp, xp); ls.radius = R;
  Shape lat = ShapeBuilder::makeFace(ls, ShapeBuilder::makeWire({
      circleEdge(Point3{0,0,0}, R, zp, xp), circleEdge(Point3{0,0,H}, R, zp, xp)}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lat})});
}
// Sphere radius R centred at origin. Matches BRepPrimAPI_MakeSphere.
Shape makeSphere(double R) {
  FaceSurface s; s.kind = FaceSurface::Kind::Sphere;
  s.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, Dir3{0,0,1}, Dir3{1,0,0}); s.radius = R;
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({
      ShapeBuilder::makeFace(s, ShapeBuilder::makeWire({}))})});
}
Dir3 orthoX(const Dir3& n) {
  const double ax=std::fabs(n.x()), ay=std::fabs(n.y()), az=std::fabs(n.z());
  Vec3 pick = (ax<=ay && ax<=az) ? Vec3{1,0,0} : (ay<=az) ? Vec3{0,1,0} : Vec3{0,0,1};
  return Dir3{nmath::cross(n.vec(), pick)};
}
nmath::Plane cutPlane(const Point3& o, const Dir3& n) {
  return nmath::Plane{Ax3::fromAxisAndRef(o, n, orthoX(n))};
}

// ── OCCT oracle (verbatim discipline of native_section_parity's occtSection) ──────────
struct OcctSection { int wireCount = 0; double length = 0.0; double area = 0.0; bool allClosed = true; bool ok = false; };

OcctSection occtSection(const TopoDS_Shape& solid, const gp_Pnt& o, const gp_Dir& n) {
  OcctSection r;
  const gp_Pln pln(o, n);
  BRepAlgoAPI_Section sect(solid, pln, Standard_False);
  sect.ComputePCurveOn1(Standard_False);
  sect.Approximation(Standard_False);
  sect.Build();
  if (!sect.IsDone()) return r;
  const TopoDS_Shape edges = sect.Shape();

  Handle(TopTools_HSequenceOfShape) edgeSeq = new TopTools_HSequenceOfShape();
  for (TopExp_Explorer ex(edges, TopAbs_EDGE); ex.More(); ex.Next()) {
    edgeSeq->Append(ex.Current());
    BRepAdaptor_Curve ac(TopoDS::Edge(ex.Current()));
    r.length += GCPnts_AbscissaPoint::Length(ac, 1e-10);
  }
  if (edgeSeq->IsEmpty()) return r;  // OCCT found no section edges (plane misses / tangent)

  Handle(TopTools_HSequenceOfShape) wires;
  ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edgeSeq, 1e-6, Standard_False, wires);
  r.wireCount = wires ? wires->Length() : 0;

  for (Standard_Integer i = 1; i <= r.wireCount; ++i) {
    const TopoDS_Wire w = TopoDS::Wire(wires->Value(i));
    if (!w.Closed()) r.allClosed = false;
    BRepBuilderAPI_MakeFace mf(pln, w, /*OnlyPlane=*/Standard_True);
    if (mf.IsDone()) {
      GProp_GProps sp;
      BRepGProp::SurfaceProperties(mf.Face(), sp);
      r.area += std::fabs(sp.Mass());
    }
  }
  r.ok = true;
  return r;
}

// relative tolerance comparator (scale-robust, matches parity's rel()).
bool rel(double a, double b, double tol) {
  const double d = std::fabs(a - b);
  return d <= tol * std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
}
double relDiff(double a, double b) {
  return std::fabs(a - b) / std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
}

// ── one generated trial ────────────────────────────────────────────────────────────
struct GenCase {
  int family = F_BOX;
  Shape nativeSolid;
  TopoDS_Shape occtSolid;
  Point3 origin{};
  Dir3 normal{Vec3{0,0,1}};
  // closed-form analytic ground truth (loops / length / area). loops<0 ⇒ not asserted
  // (the decline exerciser has no analytic section).
  int aLoops = 1;
  double aLength = 0.0;
  double aArea = 0.0;
  bool expectDecline = false;   // native is EXPECTED to Empty/Decline (honest scope)
  std::string desc;
};

GenCase genCase(Rng& rng, int idx) {
  GenCase c;
  // Force each of the five AGREE families in the first F_DECLINE slots so coverage is
  // guaranteed; then sample uniformly across all six (incl. the decline exerciser).
  int fam = (idx < F_DECLINE) ? idx : static_cast<int>(rng.below(F_COUNT));
  c.family = fam;

  switch (fam) {
    case F_BOX: {
      const double lx = rng.range(4.0, 20.0), ly = rng.range(4.0, 20.0), lz = rng.range(4.0, 20.0);
      c.nativeSolid = makeBox(lx, ly, lz);
      c.occtSolid = BRepPrimAPI_MakeBox(lx, ly, lz).Shape();
      // Axis-aligned interior cut on a random axis at a random interior offset →
      // a rectangle. (An oblique box cut can produce a triangle/pentagon whose polygon
      // is still exact, but its closed-form is not a simple rectangle; keep the AGREE
      // family a clean rectangle and defer oblique polyhedral cuts as future breadth.)
      const int axis = static_cast<int>(rng.below(3));
      if (axis == 0) {  // x = const → rectangle ly × lz
        const double x = rng.range(0.15 * lx, 0.85 * lx);
        c.origin = Point3{x, 0, 0}; c.normal = Dir3{Vec3{1,0,0}};
        c.aLength = 2.0 * (ly + lz); c.aArea = ly * lz;
        c.desc = "box x=" + std::to_string(x);
      } else if (axis == 1) {  // y = const → rectangle lx × lz
        const double y = rng.range(0.15 * ly, 0.85 * ly);
        c.origin = Point3{0, y, 0}; c.normal = Dir3{Vec3{0,1,0}};
        c.aLength = 2.0 * (lx + lz); c.aArea = lx * lz;
        c.desc = "box y=" + std::to_string(y);
      } else {  // z = const → rectangle lx × ly
        const double z = rng.range(0.15 * lz, 0.85 * lz);
        c.origin = Point3{0, 0, z}; c.normal = Dir3{Vec3{0,0,1}};
        c.aLength = 2.0 * (lx + ly); c.aArea = lx * ly;
        c.desc = "box z=" + std::to_string(z);
      }
      c.aLoops = 1;
      break;
    }
    case F_CYL_PERP: {
      const double R = rng.range(2.0, 10.0), H = rng.range(6.0, 24.0);
      c.nativeSolid = makeCylinder(R, H);
      c.occtSolid = BRepPrimAPI_MakeCylinder(R, H).Shape();
      const double z = rng.range(0.15 * H, 0.85 * H);   // interior perpendicular cut
      c.origin = Point3{0, 0, z}; c.normal = Dir3{Vec3{0,0,1}};
      c.aLoops = 1; c.aLength = 2.0 * kPi * R; c.aArea = kPi * R * R;
      c.desc = "cyl R=" + std::to_string(R) + " H=" + std::to_string(H) + " z=" + std::to_string(z);
      break;
    }
    case F_CYL_AXIAL: {
      const double R = rng.range(2.0, 10.0), H = rng.range(6.0, 24.0);
      c.nativeSolid = makeCylinder(R, H);
      c.occtSolid = BRepPrimAPI_MakeCylinder(R, H).Shape();
      // Plane through the axis (normal ⟂ axis, passing the origin on axis) → the axial
      // rectangle 2R × H. Random azimuth of the cut normal in the xy-plane.
      const double az = rng.range(0.0, 2.0 * kPi);
      c.origin = Point3{0, 0, 0}; c.normal = Dir3{Vec3{std::cos(az), std::sin(az), 0}};
      c.aLoops = 1; c.aLength = 2.0 * (2.0 * R + H); c.aArea = 2.0 * R * H;
      c.desc = "cyl-axial R=" + std::to_string(R) + " H=" + std::to_string(H) +
               " az=" + std::to_string(az);
      break;
    }
    case F_CYL_OBL: {
      const double R = rng.range(2.0, 8.0);
      // Oblique cut of angle θ from the perpendicular; the ellipse major = R/|cosθ| must
      // fit inside the finite axial band (no arc-trim) — choose H tall enough that the
      // ellipse's axial extent 2R·tanθ plus margin stays inside [0,H] around z=H/2.
      const double theta = rng.range(0.12, 0.62);  // ~7°..36°, safely inside the band
      const double axialSpan = 2.0 * R * std::tan(theta);
      const double H = axialSpan + rng.range(6.0, 14.0);  // ample band
      c.nativeSolid = makeCylinder(R, H);
      c.occtSolid = BRepPrimAPI_MakeCylinder(R, H).Shape();
      // normal tilted from +Z toward +Y by theta, through the mid-height point on axis.
      c.origin = Point3{0, 0, H * 0.5};
      c.normal = Dir3{Vec3{0, std::sin(theta), std::cos(theta)}};
      const double a = R / std::cos(theta), b = R;             // ellipse semi-axes
      const double hh = ((a - b) * (a - b)) / ((a + b) * (a + b));
      c.aLength = kPi * (a + b) * (1.0 + 3.0 * hh / (10.0 + std::sqrt(4.0 - 3.0 * hh)));  // Ramanujan-II
      c.aArea = kPi * a * b;
      c.aLoops = 1;
      c.desc = "cyl-oblique R=" + std::to_string(R) + " H=" + std::to_string(H) +
               " theta=" + std::to_string(theta);
      break;
    }
    case F_SPHERE: {
      const double R = rng.range(3.0, 12.0);
      c.nativeSolid = makeSphere(R);
      c.occtSolid = BRepPrimAPI_MakeSphere(R).Shape();
      // Cut at signed offset d along a random unit normal through the sphere centre-plane
      // family. |d| < R (interior) → circle of radius r = √(R²−d²).
      const double d = rng.range(-0.8 * R, 0.8 * R);
      // random unit normal
      const double u = rng.range(-1.0, 1.0), phi = rng.range(0.0, 2.0 * kPi);
      const double sq = std::sqrt(std::max(0.0, 1.0 - u * u));
      const Vec3 nv{sq * std::cos(phi), sq * std::sin(phi), u};
      c.normal = Dir3{nv};
      c.origin = Point3{nv.x * d, nv.y * d, nv.z * d};   // point on the plane at offset d
      const double r = std::sqrt(std::max(0.0, R * R - d * d));
      c.aLoops = 1; c.aLength = 2.0 * kPi * r; c.aArea = kPi * r * r;
      c.desc = "sphere R=" + std::to_string(R) + " d=" + std::to_string(d);
      break;
    }
    default: {  // F_DECLINE — a config the native service HONESTLY declines or reports empty
      // NOTE: we deliberately use only UNAMBIGUOUS declines (plane clearly MISSING the
      // solid, plane COINCIDENT with a planar face). We do NOT force an EXACT-tangency
      // plane (a plane grazing a sphere pole at d==R): that is a measure-zero knife-edge
      // where floating-point rounding lands the plane a sub-nanometre INSIDE the sphere,
      // and native then correctly returns the true sub-micron circle (verified: at d==R
      // exactly it declines; at d=R−1e-9 it returns the mathematically-correct tiny loop;
      // OCCT rounds that to empty). Asserting a decline on that knife-edge would flag a
      // CORRECT native section as wrong — the opposite of the bar's intent. Robust
      // near-tangent-but-clearly-outside declines are exercised instead.
      const int kind = static_cast<int>(rng.below(3));
      if (kind == 0) {  // plane MISSES the box entirely → native Empty, OCCT empty
        const double lx = rng.range(4.0, 12.0), ly = rng.range(4.0, 12.0), lz = rng.range(4.0, 12.0);
        c.nativeSolid = makeBox(lx, ly, lz);
        c.occtSolid = BRepPrimAPI_MakeBox(lx, ly, lz).Shape();
        c.origin = Point3{0, 0, lz + rng.range(2.0, 8.0)};   // clearly above the box
        c.normal = Dir3{Vec3{0,0,1}};
        c.desc = "MISS box (plane above)";
      } else if (kind == 1) {  // plane COINCIDENT with a box face → native declines
        const double lx = rng.range(4.0, 12.0), ly = rng.range(4.0, 12.0), lz = rng.range(4.0, 12.0);
        c.nativeSolid = makeBox(lx, ly, lz);
        c.occtSolid = BRepPrimAPI_MakeBox(lx, ly, lz).Shape();
        c.origin = Point3{0, 0, 0};  // z=0 is the bottom face plane (coincident)
        c.normal = Dir3{Vec3{0,0,1}};
        c.desc = "COINCIDENT box bottom face";
      } else {  // plane clearly OUTSIDE a sphere (well beyond the pole) → native Empty
        const double R = rng.range(3.0, 10.0);
        c.nativeSolid = makeSphere(R);
        c.occtSolid = BRepPrimAPI_MakeSphere(R).Shape();
        c.origin = Point3{0, 0, R + rng.range(0.5, 4.0)};   // clearly beyond the sphere
        c.normal = Dir3{Vec3{0,0,1}};
        c.desc = "MISS sphere (plane beyond pole)";
      }
      c.expectDecline = true;
      c.aLoops = -1;  // no analytic section asserted
      break;
    }
  }
  return c;
}

// coverage tallies
int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleUnrel = 0, g_oracleBad = 0;
int g_famAgreed[F_COUNT] = {}, g_famDeclined[F_COUNT] = {}, g_famDisagreed[F_COUNT] = {};
int g_famOracleUnrel[F_COUNT] = {}, g_famOracleBad[F_COUNT] = {};

// tolerances (deflection-matched; section is EXACT analytic so straight/circular are
// tight; only the ellipse perimeter is approximated — held to the parity-proven 1e-4).
constexpr double kTightRel = 1e-9;   // native-vs-analytic straight/circular (exact fp64)
constexpr double kEllipseRel = 1e-4; // ellipse perimeter (Ramanujan-II + OCCT integrator)
constexpr double kAreaRel = 1e-6;    // capped area vs OCCT SurfaceProperties (exact form)

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x5EC7104FEEDull;
  int N = 96;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 96;
  if (N < F_COUNT) N = F_COUNT;   // guarantee each forced family runs at least once

  std::printf("== M6-breadth-12 differential-fuzz: native SECTION-CURVES vs OCCT BRepAlgoAPI_Section ==\n");
  std::printf("== seed=0x%llx N=%d  tightRel=%.0e ellipseRel=%.0e areaRel=%.0e ==\n",
              static_cast<unsigned long long>(seed), N, kTightRel, kEllipseRel, kAreaRel);
  std::fflush(stdout);

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    const GenCase c = genCase(rng, i);

    // (1) NATIVE section at the C++ boundary (OCCT-free system under test).
    const sec::SectionResult nr = sec::sectionByPlane(c.nativeSolid, cutPlane(c.origin, c.normal));

    // (2) OCCT oracle section of the matched solid by the same plane.
    const OcctSection orr = occtSection(c.occtSolid, gp_Pnt(c.origin.x, c.origin.y, c.origin.z),
                                        gp_Dir(c.normal.x(), c.normal.y(), c.normal.z()));

    Verdict v;

    if (c.expectDecline) {
      // Decline exerciser: native MUST report Empty/Declined (never a wrong section).
      // A native Ok on a config we expect it to refuse is a DISAGREE (silent wrong section).
      if (!nr.ok()) {
        v = HONESTLY_DECLINED;
      } else {
        v = DISAGREED;
      }
    } else {
      // AGREE family: native should produce a section; classify by the closed-form arbiter.
      if (!nr.ok()) {
        // Native honestly declined a config inside an AGREE family (e.g. a numerically
        // marginal plane it self-verify-rejected). OCCT ships → HONESTLY-DECLINED.
        v = HONESTLY_DECLINED;
      } else if (!orr.ok) {
        // OCCT produced no usable section on a config the native + closed-form both cover:
        // the oracle is unreliable here (gated off, native NOT faulted).
        v = ORACLE_BAD;
      } else {
        const double lenTol = (c.family == F_CYL_OBL) ? kEllipseRel : kTightRel;
        const bool loopsOk = (nr.loopCount() == c.aLoops) && (orr.wireCount == c.aLoops);
        // native vs CLOSED-FORM analytic truth (PRIMARY, exact)
        const bool natLenA = rel(nr.totalLength(), c.aLength, lenTol);
        const bool natAreaA = rel(nr.totalArea(), c.aArea, kTightRel);
        // OCCT vs CLOSED-FORM analytic truth (oracle trust)
        const bool occLenA = rel(orr.length, c.aLength, lenTol);
        const bool occAreaA = rel(orr.area, c.aArea, kAreaRel);
        // native vs OCCT (SECONDARY engine agreement)
        const bool natOccLen = rel(nr.totalLength(), orr.length, lenTol);
        const bool natOccArea = rel(nr.totalArea(), orr.area, kAreaRel);

        const bool natMatchesAnalytic = loopsOk && natLenA && natAreaA;
        const bool occMatchesAnalytic = (orr.wireCount == c.aLoops) && occLenA && occAreaA && orr.allClosed;

        if (natMatchesAnalytic && natOccLen && natOccArea && occMatchesAnalytic) {
          v = AGREED;    // native = analytic = OCCT
        } else if (natMatchesAnalytic && !occMatchesAnalytic) {
          v = ORACLE_UNRELIABLE;  // native VINDICATED by exact math; OCCT the outlier
        } else {
          v = DISAGREED; // native fails the analytic ground truth → silent wrong section
        }
      }
    }

    switch (v) {
      case AGREED:            ++g_agreed;      ++g_famAgreed[c.family];      break;
      case HONESTLY_DECLINED: ++g_declined;    ++g_famDeclined[c.family];    break;
      case DISAGREED:         ++g_disagreed;   ++g_famDisagreed[c.family];   break;
      case ORACLE_UNRELIABLE: ++g_oracleUnrel; ++g_famOracleUnrel[c.family]; break;
      case ORACLE_BAD:        ++g_oracleBad;   ++g_famOracleBad[c.family];   break;
    }

    if (v == AGREED) {
      std::printf("[FUZZ] AGREED    case=%d %-24s loops=%d/%d lenN=%.5g lenO=%.5g aLen=%.5g dLen=%.2e areaN=%.5g areaO=%.5g aArea=%.5g\n",
                  i, famName(c.family), nr.loopCount(), orr.wireCount,
                  nr.totalLength(), orr.length, c.aLength, relDiff(nr.totalLength(), orr.length),
                  nr.totalArea(), orr.area, c.aArea);
    } else if (v == HONESTLY_DECLINED) {
      std::printf("[FUZZ] DECLINED  case=%d %-24s native=%s reason=\"%s\" occtWires=%d  %s\n",
                  i, famName(c.family), nr.ok() ? "Ok?!" : (nr.status == sec::SectionStatus::Empty ? "Empty" : "Declined"),
                  nr.reason.c_str(), orr.wireCount, c.desc.c_str());
    } else if (v == ORACLE_BAD) {
      std::printf("[FUZZ] ORACLE_UNRELIABLE case=%d %-24s OCCT produced no usable section while native+analytic cover it\n"
                  "       NOTE seed=0x%llx index=%d %s\n",
                  i, famName(c.family), static_cast<unsigned long long>(seed), i, c.desc.c_str());
    } else if (v == ORACLE_UNRELIABLE) {
      std::printf("[FUZZ] ORACLE_UNRELIABLE case=%d %-24s native MATCHES analytic, OCCT does NOT "
                  "lenN=%.6g lenO=%.6g aLen=%.6g  areaN=%.6g areaO=%.6g aArea=%.6g loops N/O=%d/%d\n"
                  "       NOTE seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nr.totalLength(), orr.length, c.aLength,
                  nr.totalArea(), orr.area, c.aArea, nr.loopCount(), orr.wireCount,
                  static_cast<unsigned long long>(seed), i, c.desc.c_str());
    } else {  // DISAGREED
      std::printf("[FUZZ] DISAGREED case=%d %-24s SILENT-WRONG-SECTION "
                  "nativeOk=%d loops N/O/a=%d/%d/%d lenN=%.6g lenO=%.6g aLen=%.6g dLen(a)=%.3e "
                  "areaN=%.6g areaO=%.6g aArea=%.6g dArea(a)=%.3e reason=\"%s\"\n"
                  "       REPRO seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nr.ok() ? 1 : 0, nr.loopCount(), orr.wireCount, c.aLoops,
                  nr.totalLength(), orr.length, c.aLength, relDiff(nr.totalLength(), c.aLength),
                  nr.totalArea(), orr.area, c.aArea, relDiff(nr.totalArea(), c.aArea),
                  nr.reason.c_str(), static_cast<unsigned long long>(seed), i, c.desc.c_str());
    }
    std::fflush(stdout);
  }

  // ── coverage summary ────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE_UNRELIABLE=%d  ORACLE_BAD=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleUnrel, g_oracleBad);
  std::printf("   per-family [agreed/declined/DISAGREED/oracle-unreliable/oracle-bad]:\n");
  for (int f = 0; f < F_COUNT; ++f) {
    std::printf("     %-26s %d/%d/%d/%d/%d\n", famName(f),
                g_famAgreed[f], g_famDeclined[f], g_famDisagreed[f], g_famOracleUnrel[f], g_famOracleBad[f]);
  }

  // Real coverage: each AGREE family (BOX / CYL_PERP / CYL_AXIAL / CYL_OBL / SPHERE) must
  // have ≥1 AGREED trial; the decline exerciser must have ≥1 HONESTLY-DECLINED trial.
  bool coverage = true;
  for (int f = 0; f < F_DECLINE; ++f) if (g_famAgreed[f] < 1) coverage = false;
  if (g_famDeclined[F_DECLINE] < 1) coverage = false;

  const bool bar = (g_disagreed == 0 && g_oracleUnrel == 0 && g_oracleBad == 0 && coverage);
  if (!coverage)
    std::printf("   COVERAGE INCOMPLETE — an AGREE family had 0 AGREED (or the decline exerciser 0 declines); raise N\n");
  if (g_oracleUnrel) std::printf("   ORACLE_UNRELIABLE=%d (native VINDICATED by exact math vs OCCT — oracle-side limitation, logged, NOT a native fault)\n", g_oracleUnrel);
  if (g_oracleBad)   std::printf("   ORACLE_BAD=%d (OCCT produced no usable section where native+analytic cover it — investigate oracle)\n", g_oracleBad);
  std::printf("== M6-breadth-12 BAR: %s (DISAGREED=%d must be 0) ==\n",
              bar ? "PASS — zero silent wrong sections" : "FAIL", g_disagreed);
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
