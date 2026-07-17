// SPDX-License-Identifier: Apache-2.0
//
// step_brep.cpp — implementation of the exact trimmed-NURBS B-rep STEP AP214
// round-trip. See step_brep.h for the contract.
//
// Two self-contained halves:
//   * A Part-21 EMITTER (Emitter + Serializer) that walks a set of TrimmedNurbsFace
//     and prints the HEADER + DATA section (`#N = ENTITY(args);`). Rational surfaces
//     / pcurves use the AP214 complex-instance (RATIONAL_B_SPLINE_*) form so weights
//     are carried EXACTLY.
//   * A Part-21 PARSER (a tokenizer + entity table) that reads the DATA section back,
//     resolves refs, and rebuilds each TrimmedNurbsFace with poles / knots / weights /
//     trims EXACT (the reals are printed with 17 significant digits — round-trip-exact
//     for fp64).
//
// OCCT-FREE. Uses only src/native/{topology,math}. clang++ -std=c++20.
//
#include "native/exchange/step_brep.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cybercad::native::exchange {
namespace {

namespace math = cybercad::native::math;
using topo::EdgeCurve;
using topo::FaceSurface;
using topo::Location;
using topo::ParamPoint;
using topo::PcurveSegment;
using topo::PCurve;
using topo::TrimLoop;
using topo::TrimmedNurbsFace;

// ─────────────────────────────────────────────────────────────────────────────
// Real formatting: 17 significant digits round-trips an fp64 exactly, so the
// parse recovers poles/knots/weights bit-for-bit (well within ≤ 1e-9).
// ─────────────────────────────────────────────────────────────────────────────
std::string real(double x) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", x);
  std::string s(buf);
  // STEP reals must contain a '.'; %g may emit "1" or "1e+20" — normalise.
  if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
      s.find('E') == std::string::npos && s.find("inf") == std::string::npos &&
      s.find("nan") == std::string::npos) {
    s += ".";
  }
  return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Flat knot vector → (distinct knots, multiplicities) for the *_WITH_KNOTS form.
// ─────────────────────────────────────────────────────────────────────────────
struct KnotData {
  std::vector<double> values;
  std::vector<int> mults;
};
KnotData compressKnots(const std::vector<double>& flat) {
  KnotData out;
  for (double u : flat) {
    if (!out.values.empty() && std::abs(u - out.values.back()) <= 1e-12) {
      ++out.mults.back();
    } else {
      out.values.push_back(u);
      out.mults.push_back(1);
    }
  }
  return out;
}
std::vector<double> expandKnots(const std::vector<double>& values, const std::vector<int>& mults) {
  std::vector<double> flat;
  for (std::size_t i = 0; i < values.size() && i < mults.size(); ++i)
    for (int m = 0; m < mults[i]; ++m) flat.push_back(values[i]);
  return flat;
}

std::string ids(const std::vector<int>& v) {
  std::string s = "(";
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) s += ",";
    s += "#" + std::to_string(v[i]);
  }
  return s + ")";
}
std::string intList(const std::vector<int>& v) {
  std::string s = "(";
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) s += ",";
    s += std::to_string(v[i]);
  }
  return s + ")";
}
std::string realList(const std::vector<double>& v) {
  std::string s = "(";
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) s += ",";
    s += real(v[i]);
  }
  return s + ")";
}

// ─────────────────────────────────────────────────────────────────────────────
// EMITTER — accumulates `#id = TEXT;` lines and hands out fresh ids.
// ─────────────────────────────────────────────────────────────────────────────
class Emitter {
 public:
  int emit(const std::string& body) {
    const int id = next_++;
    lines_.push_back("#" + std::to_string(id) + "=" + body + ";");
    return id;
  }
  std::string data() const {
    std::string s;
    for (const std::string& l : lines_) {
      s += l;
      s += "\n";
    }
    return s;
  }
  int point3(const math::Point3& p) {
    return emit("CARTESIAN_POINT('',(" + real(p.x) + "," + real(p.y) + "," + real(p.z) + "))");
  }
  int point2(double u, double v) {
    return emit("CARTESIAN_POINT('',(" + real(u) + "," + real(v) + "))");
  }
  int direction(const math::Vec3& d) {
    return emit("DIRECTION('',(" + real(d.x) + "," + real(d.y) + "," + real(d.z) + "))");
  }
  int placement(const math::Ax3& f) {
    const int o = point3(f.origin);
    const int z = direction(f.z.vec());
    const int x = direction(f.x.vec());
    return emit("AXIS2_PLACEMENT_3D('',#" + std::to_string(o) + ",#" + std::to_string(z) + ",#" +
                std::to_string(x) + ")");
  }

 private:
  int next_ = 1;
  std::vector<std::string> lines_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Scope check for one surface / one pcurve kind.
// ─────────────────────────────────────────────────────────────────────────────
bool surfaceInScope(const FaceSurface& s) {
  using K = FaceSurface::Kind;
  switch (s.kind) {
    case K::Plane:
    case K::Cylinder:
    case K::Cone:
    case K::Sphere:
    case K::Torus:
      return true;
    case K::BSpline:
      // Needs a real knotted grid (a Bezier-as-BSpline with no knots is out of scope).
      return !s.poles.empty() && !s.knotsU.empty() && !s.knotsV.empty() && s.nPolesU > 0 &&
             s.nPolesV > 0;
    case K::Bezier:
    default:
      return false;
  }
}
bool pcurveInScope(const PCurve& c) {
  using K = EdgeCurve::Kind;
  switch (c.kind) {
    case K::Line:
    case K::Circle:
    case K::Ellipse:
      return true;
    case K::BSpline:
      return !c.poles2d.empty() && !c.knots.empty() && c.degree >= 1;
    case K::Bezier:
    default:
      return false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Local evaluators (duplicated small helpers so this TU is self-contained, matching
// the trimmed_nurbs.cpp evaluators — used only to sample S(pcurve(t)) for the
// informational 3-D EDGE_CURVE geometry).
// ─────────────────────────────────────────────────────────────────────────────
math::SurfaceGrid gridOf(const FaceSurface& s) {
  return math::SurfaceGrid{{s.poles.data(), s.poles.size()}, s.nPolesU, s.nPolesV};
}
math::Point3 surfaceLocal(const FaceSurface& s, double u, double v) {
  using K = FaceSurface::Kind;
  switch (s.kind) {
    case K::Plane:    return math::Plane{s.frame}.value(u, v);
    case K::Cylinder: return math::Cylinder{s.frame, s.radius}.value(u, v);
    case K::Cone:     return math::Cone{s.frame, s.radius, s.semiAngle}.value(u, v);
    case K::Sphere:   return math::Sphere{s.frame, s.radius}.value(u, v);
    case K::Torus:    return math::Torus{s.frame, s.radius, s.minorRadius}.value(u, v);
    case K::BSpline:
    default:
      if (s.poles.empty()) return s.frame.origin;
      return s.weights.empty()
                 ? math::surfacePoint(s.degreeU, s.degreeV, gridOf(s),
                                      {s.knotsU.data(), s.knotsU.size()},
                                      {s.knotsV.data(), s.knotsV.size()}, u, v)
                 : math::nurbsSurfacePoint(s.degreeU, s.degreeV, gridOf(s),
                                           {s.weights.data(), s.weights.size()},
                                           {s.knotsU.data(), s.knotsU.size()},
                                           {s.knotsV.data(), s.knotsV.size()}, u, v);
  }
}
ParamPoint pcurveValueRaw(const PCurve& c, double t) {
  using K = EdgeCurve::Kind;
  switch (c.kind) {
    case K::Line:
      return {c.origin2d.x + c.dir2d.x * t, c.origin2d.y + c.dir2d.y * t};
    case K::Circle: {
      const double r = c.dir2d.x;
      return {c.origin2d.x + r * std::cos(t), c.origin2d.y + r * std::sin(t)};
    }
    case K::Ellipse: {
      const double a = c.dir2d.x, b = c.dir2d.y;
      return {c.origin2d.x + a * std::cos(t), c.origin2d.y + b * std::sin(t)};
    }
    case K::BSpline:
    default: {
      const std::size_t np = c.poles2d.size();
      const math::Point3 p =
          c.weights.empty() ? math::curvePoint(c.degree, {c.poles2d.data(), np},
                                               {c.knots.data(), c.knots.size()}, t)
                            : math::nurbsCurvePoint(c.degree, {c.poles2d.data(), np},
                                                    {c.weights.data(), c.weights.size()},
                                                    {c.knots.data(), c.knots.size()}, t);
      return {p.x, p.y};
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// SERIALIZER — the walk that turns TrimmedNurbsFaces into the DATA section.
// ─────────────────────────────────────────────────────────────────────────────
class Serializer {
 public:
  explicit Serializer(Emitter& e) : e_(e) {}

  // Returns the ADVANCED_FACE #id, or 0 on out-of-scope (caller declines).
  int face(const TrimmedNurbsFace& f) {
    if (!surfaceInScope(f.surface)) return 0;
    const int surf = surface(f.surface);
    if (surf == 0) return 0;

    std::vector<int> bounds;
    if (f.hasOuter()) {
      const int b = faceBound(f.surface, surf, f.outer, /*outer=*/true);
      if (b == 0) return 0;
      bounds.push_back(b);
    }
    for (const TrimLoop& hole : f.holes) {
      const int b = faceBound(f.surface, surf, hole, /*outer=*/false);
      if (b == 0) return 0;
      bounds.push_back(b);
    }
    return e_.emit("ADVANCED_FACE(''," + ids(bounds) + ",#" + std::to_string(surf) + ",.T.)");
  }

 private:
  Emitter& e_;

  // ── Surface ────────────────────────────────────────────────────────────────
  int surface(const FaceSurface& s) {
    using K = FaceSurface::Kind;
    switch (s.kind) {
      case K::Plane:    return e_.emit("PLANE('',#" + std::to_string(e_.placement(s.frame)) + ")");
      case K::Cylinder:
        return e_.emit("CYLINDRICAL_SURFACE('',#" + std::to_string(e_.placement(s.frame)) + "," +
                       real(s.radius) + ")");
      case K::Cone:
        return e_.emit("CONICAL_SURFACE('',#" + std::to_string(e_.placement(s.frame)) + "," +
                       real(s.radius) + "," + real(s.semiAngle) + ")");
      case K::Sphere:
        return e_.emit("SPHERICAL_SURFACE('',#" + std::to_string(e_.placement(s.frame)) + "," +
                       real(s.radius) + ")");
      case K::Torus:
        return e_.emit("TOROIDAL_SURFACE('',#" + std::to_string(e_.placement(s.frame)) + "," +
                       real(s.radius) + "," + real(s.minorRadius) + ")");
      case K::BSpline:
      default:
        return bsplineSurface(s);
    }
  }

  int bsplineSurface(const FaceSurface& s) {
    // Control-point net, row-major (U outer, V inner) — STEP's ordering.
    std::string grid = "(";
    for (int i = 0; i < s.nPolesU; ++i) {
      if (i) grid += ",";
      std::vector<int> row;
      for (int j = 0; j < s.nPolesV; ++j) {
        const std::size_t idx = static_cast<std::size_t>(i) * s.nPolesV + j;
        row.push_back(e_.point3(s.poles[idx]));
      }
      grid += ids(row);
    }
    grid += ")";
    const KnotData ku = compressKnots(s.knotsU);
    const KnotData kv = compressKnots(s.knotsV);
    const std::string common =
        "'',"+ std::to_string(s.degreeU) + "," + std::to_string(s.degreeV) + "," + grid +
        ",.UNSPECIFIED.,.F.,.F.,.F.," + intList(ku.mults) + "," + intList(kv.mults) + "," +
        realList(ku.values) + "," + realList(kv.values) + ",.UNSPECIFIED.";
    if (s.weights.empty()) {
      return e_.emit("B_SPLINE_SURFACE_WITH_KNOTS(" + common + ")");
    }
    // Rational: AP214 complex-instance carrying the weights grid (U outer, V inner).
    std::string wgrid = "(";
    for (int i = 0; i < s.nPolesU; ++i) {
      if (i) wgrid += ",";
      std::vector<double> row;
      for (int j = 0; j < s.nPolesV; ++j)
        row.push_back(s.weights[static_cast<std::size_t>(i) * s.nPolesV + j]);
      wgrid += realList(row);
    }
    wgrid += ")";
    return e_.emit(
        "(B_SPLINE_SURFACE(" + std::to_string(s.degreeU) + "," + std::to_string(s.degreeV) + "," +
        grid + ",.UNSPECIFIED.,.F.,.F.,.F.)" + "B_SPLINE_SURFACE_WITH_KNOTS(" + intList(ku.mults) +
        "," + intList(kv.mults) + "," + realList(ku.values) + "," + realList(kv.values) +
        ",.UNSPECIFIED.)" + "BOUNDED_SURFACE()GEOMETRIC_REPRESENTATION_ITEM()" +
        "RATIONAL_B_SPLINE_SURFACE(" + wgrid + ")REPRESENTATION_ITEM('')SURFACE())");
  }

  // ── Loop / bound ─────────────────────────────────────────────────────────────
  int faceBound(const FaceSurface& s, int surf, const TrimLoop& loop, bool outer) {
    std::vector<int> orientedEdges;
    for (const PcurveSegment& seg : loop) {
      const int oe = orientedEdge(s, surf, seg);
      if (oe == 0) return 0;
      orientedEdges.push_back(oe);
    }
    if (orientedEdges.empty()) return 0;
    const int loopId = e_.emit("EDGE_LOOP(''," + ids(orientedEdges) + ")");
    const char* kw = outer ? "FACE_OUTER_BOUND" : "FACE_BOUND";
    return e_.emit(std::string(kw) + "('',#" + std::to_string(loopId) + ",.T.)");
  }

  int orientedEdge(const FaceSurface& s, int surf, const PcurveSegment& seg) {
    if (!pcurveInScope(seg.curve)) return 0;
    const int pc = pcurve(surf, seg);  // the exact 2-D pcurve wrapped in a PCURVE
    if (pc == 0) return 0;

    // The 3-D edge geometry: sample S(pcurve(t)) at the two endpoints for the
    // EDGE_CURVE vertices, and build a B_SPLINE (informational) 3-D curve.
    const ParamPoint uv0 = pcurveValueRaw(seg.curve, seg.first);
    const ParamPoint uv1 = pcurveValueRaw(seg.curve, seg.last);
    const math::Point3 p0 = surfaceLocal(s, uv0.u, uv0.v);
    const math::Point3 p1 = surfaceLocal(s, uv1.u, uv1.v);
    const int v0 = e_.emit("VERTEX_POINT('',#" + std::to_string(e_.point3(p0)) + ")");
    const int v1 = e_.emit("VERTEX_POINT('',#" + std::to_string(e_.point3(p1)) + ")");
    const int curve3d = edgeCurve3d(s, seg);
    const int ec = e_.emit("EDGE_CURVE('',#" + std::to_string(v0) + ",#" + std::to_string(v1) +
                           ",#" + std::to_string(curve3d) + ",.T.)");
    const char* orient = seg.reversed ? ".F." : ".T.";
    const int oe = e_.emit("ORIENTED_EDGE('',*,*,#" + std::to_string(ec) + "," + orient + ")");
    // CC_TRIM(#orientedEdge, #pcurve, first, last, reversed) — the AP214-extension
    // record that pins the EXACT trim to THIS oriented edge (order-independent match).
    e_.emit("CC_TRIM(#" + std::to_string(oe) + ",#" + std::to_string(pc) + "," + real(seg.first) +
            "," + real(seg.last) + "," + (seg.reversed ? ".T." : ".F.") + ")");
    return oe;
  }

  // Emit the 2-D pcurve (Line/Circle/Ellipse/B-spline) wrapped in a PCURVE on the
  // surface. Returns the PCURVE #id.
  int pcurve(int surf, const PcurveSegment& seg) {
    const int curve2d = curve2dEntity(seg.curve);
    if (curve2d == 0) return 0;
    const int drep = e_.emit("DEFINITIONAL_REPRESENTATION('',(#" + std::to_string(curve2d) +
                             "),$)");  // param-space context left null ($)
    return e_.emit("PCURVE('',#" + std::to_string(surf) + ",#" + std::to_string(drep) + ")");
  }

  int curve2dEntity(const PCurve& c) {
    using K = EdgeCurve::Kind;
    switch (c.kind) {
      case K::Line: {
        const int o = e_.point2(c.origin2d.x, c.origin2d.y);
        const int d = e_.emit("DIRECTION('',(" + real(c.dir2d.x) + "," + real(c.dir2d.y) + "))");
        const int vec = e_.emit("VECTOR('',#" + std::to_string(d) + ",1.)");
        return e_.emit("LINE('',#" + std::to_string(o) + ",#" + std::to_string(vec) + ")");
      }
      case K::Circle: {
        const int plc = placement2d(c.origin2d.x, c.origin2d.y);
        return e_.emit("CIRCLE('',#" + std::to_string(plc) + "," + real(c.dir2d.x) + ")");
      }
      case K::Ellipse: {
        const int plc = placement2d(c.origin2d.x, c.origin2d.y);
        return e_.emit("ELLIPSE('',#" + std::to_string(plc) + "," + real(c.dir2d.x) + "," +
                       real(c.dir2d.y) + ")");
      }
      case K::BSpline:
      default:
        return bspline2d(c);
    }
  }

  int placement2d(double u, double v) {
    const int o = e_.point2(u, v);
    const int ax = e_.emit("DIRECTION('',(1.,0.))");
    return e_.emit("AXIS2_PLACEMENT_2D('',#" + std::to_string(o) + ",#" + std::to_string(ax) + ")");
  }

  int bspline2d(const PCurve& c) {
    std::vector<int> poleIds;
    for (const math::Point3& p : c.poles2d) poleIds.push_back(e_.point2(p.x, p.y));
    const KnotData kd = compressKnots(c.knots);
    const std::string knotPart = intList(kd.mults) + "," + realList(kd.values) + ",.UNSPECIFIED.";
    if (c.weights.empty()) {
      return e_.emit("B_SPLINE_CURVE_WITH_KNOTS(''," + std::to_string(c.degree) + "," +
                     ids(poleIds) + ",.UNSPECIFIED.,.F.,.F.," + knotPart + ")");
    }
    return e_.emit(
        "(B_SPLINE_CURVE(" + std::to_string(c.degree) + "," + ids(poleIds) +
        ",.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS(" + knotPart +
        ")BOUNDED_CURVE()CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(" +
        realList(c.weights) + ")REPRESENTATION_ITEM(''))");
  }

  // The 3-D edge curve: sample S(pcurve(t)) and store as a plain B_SPLINE_CURVE poly
  // (informational; the exact trim comes from the pcurve). Degree-1 interpolating
  // poly over the sampled feet — a valid, resolvable 3-D geometry for the EDGE_CURVE.
  int edgeCurve3d(const FaceSurface& s, const PcurveSegment& seg) {
    const int kSamples = 8;
    std::vector<int> poleIds;
    for (int i = 0; i < kSamples; ++i) {
      const double t = seg.first + (seg.last - seg.first) * (double(i) / (kSamples - 1));
      const ParamPoint uv = pcurveValueRaw(seg.curve, t);
      poleIds.push_back(e_.point3(surfaceLocal(s, uv.u, uv.v)));
    }
    // Uniform clamped degree-1 knot vector.
    std::vector<int> mults(kSamples, 1);
    mults.front() = 2;
    mults.back() = 2;
    std::vector<double> vals;
    for (int i = 0; i < kSamples; ++i) vals.push_back(double(i));
    return e_.emit("B_SPLINE_CURVE_WITH_KNOTS('',1," + ids(poleIds) +
                   ",.UNSPECIFIED.,.F.,.F.," + intList(mults) + "," + realList(vals) +
                   ",.UNSPECIFIED.)");
  }
};

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// Public: canWriteStepBrep / writeStepBrep
// ═════════════════════════════════════════════════════════════════════════════
bool canWriteStepBrep(const TrimmedNurbsFace& face) {
  if (!surfaceInScope(face.surface)) return false;
  auto loopOk = [](const TrimLoop& loop) {
    for (const PcurveSegment& seg : loop)
      if (!pcurveInScope(seg.curve)) return false;
    return true;
  };
  if (!loopOk(face.outer)) return false;
  for (const TrimLoop& h : face.holes)
    if (!loopOk(h)) return false;
  return true;
}

std::string writeStepBrep(const std::vector<TrimmedNurbsFace>& faces) {
  if (faces.empty()) return "";
  for (const TrimmedNurbsFace& f : faces)
    if (!canWriteStepBrep(f)) return "";  // honest decline — no partial/invalid file

  Emitter e;
  Serializer ser(e);
  std::vector<int> faceIds;
  for (const TrimmedNurbsFace& f : faces) {
    const int id = ser.face(f);
    if (id == 0) return "";
    faceIds.push_back(id);
  }
  const int shell = e.emit("CLOSED_SHELL(''," + ids(faceIds) + ")");
  const int brep = e.emit("MANIFOLD_SOLID_BREP('',#" + std::to_string(shell) + ")");
  (void)brep;

  std::string out;
  out += "ISO-10303-21;\n";
  out += "HEADER;\n";
  out += "FILE_DESCRIPTION(('CyberCadKernel exact trimmed-NURBS B-rep'),'2;1');\n";
  out += "FILE_NAME('','',(''),(''),'CyberCadKernel','step_brep','');\n";
  out += "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n";
  out += "ENDSEC;\n";
  out += "DATA;\n";
  out += e.data();
  out += "ENDSEC;\n";
  out += "END-ISO-10303-21;\n";
  return out;
}

// ═════════════════════════════════════════════════════════════════════════════
// PARSER
// ═════════════════════════════════════════════════════════════════════════════
namespace {

// One parsed argument token: a ref (#N), a real/int, a string, an enum (.T.), a list,
// or a nested/complex sub-entity ( NAME(...) NAME(...) ).
struct Value;
using ValueList = std::vector<Value>;

struct Value {
  enum class Kind { Ref, Number, String, Enum, List, Entity, Null } kind = Kind::Null;
  long ref = 0;
  double number = 0.0;
  std::string text;      // string / enum text (without quotes/dots) OR entity name
  ValueList list;        // List elements OR Entity args
  std::vector<std::pair<std::string, ValueList>> complex;  // ( SUB(...) SUB(...) )
};

struct Record {
  std::string name;              // simple-instance keyword (empty for a complex instance)
  ValueList args;                // simple-instance args
  std::vector<std::pair<std::string, ValueList>> complex;  // complex-instance sub-records
  bool isComplex = false;
};

// ── Tokenizer over the DATA section, keyed by #id ────────────────────────────
class Parser {
 public:
  explicit Parser(const std::string& src) : s_(src) {}

  // Parse all `#N = ...;` records in the DATA section into the table. Returns false
  // on a malformed record.
  bool parseTable() {
    const std::size_t dataStart = s_.find("DATA;");
    if (dataStart == std::string::npos) return false;
    pos_ = dataStart + 5;
    const std::size_t dataEnd = s_.find("ENDSEC;", pos_);
    end_ = dataEnd == std::string::npos ? s_.size() : dataEnd;

    while (true) {
      skipWs();
      if (pos_ >= end_) break;
      if (s_[pos_] != '#') break;
      ++pos_;  // '#'
      const long id = readInt();
      skipWs();
      if (pos_ >= end_ || s_[pos_] != '=') return false;
      ++pos_;  // '='
      skipWs();
      Record r;
      if (!readRecord(r)) return false;
      skipWs();
      if (pos_ >= end_ || s_[pos_] != ';') return false;
      ++pos_;  // ';'
      table_.emplace(id, std::move(r));
    }
    return true;
  }

  const std::map<long, Record>& table() const { return table_; }

 private:
  const std::string& s_;
  std::size_t pos_ = 0;
  std::size_t end_ = 0;
  std::map<long, Record> table_;

  void skipWs() {
    while (pos_ < end_) {
      const char c = s_[pos_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++pos_;
      } else if (c == '/' && pos_ + 1 < end_ && s_[pos_ + 1] == '*') {
        pos_ += 2;
        while (pos_ + 1 < end_ && !(s_[pos_] == '*' && s_[pos_ + 1] == '/')) ++pos_;
        pos_ += 2;
      } else {
        break;
      }
    }
  }
  long readInt() {
    long sign = 1;
    if (pos_ < end_ && (s_[pos_] == '+' || s_[pos_] == '-')) {
      if (s_[pos_] == '-') sign = -1;
      ++pos_;
    }
    long v = 0;
    while (pos_ < end_ && std::isdigit(static_cast<unsigned char>(s_[pos_]))) {
      v = v * 10 + (s_[pos_] - '0');
      ++pos_;
    }
    return v * sign;
  }

  bool readIdentifier(std::string& out) {
    skipWs();
    std::size_t start = pos_;
    while (pos_ < end_ &&
           (std::isalnum(static_cast<unsigned char>(s_[pos_])) || s_[pos_] == '_')) ++pos_;
    if (pos_ == start) return false;
    out = s_.substr(start, pos_ - start);
    return true;
  }

  // A record is either NAME(args) (simple) or ( NAME(args) NAME(args) ) (complex).
  bool readRecord(Record& r) {
    skipWs();
    if (pos_ < end_ && s_[pos_] == '(') {
      // Complex instance.
      r.isComplex = true;
      ++pos_;  // '('
      while (true) {
        skipWs();
        if (pos_ < end_ && s_[pos_] == ')') { ++pos_; break; }
        std::string name;
        if (!readIdentifier(name)) return false;
        skipWs();
        ValueList args;
        if (pos_ < end_ && s_[pos_] == '(') {
          if (!readArgList(args)) return false;
        }
        r.complex.emplace_back(name, std::move(args));
      }
      return true;
    }
    // Simple instance.
    if (!readIdentifier(r.name)) return false;
    return readArgList(r.args);
  }

  // Read a parenthesised, comma-separated argument list into `out`.
  bool readArgList(ValueList& out) {
    skipWs();
    if (pos_ >= end_ || s_[pos_] != '(') return false;
    ++pos_;  // '('
    while (true) {
      skipWs();
      if (pos_ < end_ && s_[pos_] == ')') { ++pos_; break; }
      Value v;
      if (!readValue(v)) return false;
      out.push_back(std::move(v));
      skipWs();
      if (pos_ < end_ && s_[pos_] == ',') { ++pos_; continue; }
      if (pos_ < end_ && s_[pos_] == ')') { ++pos_; break; }
      return false;
    }
    return true;
  }

  bool readValue(Value& v) {
    skipWs();
    if (pos_ >= end_) return false;
    const char c = s_[pos_];
    if (c == '#') {
      ++pos_;
      v.kind = Value::Kind::Ref;
      v.ref = readInt();
      return true;
    }
    if (c == '\'') {
      ++pos_;
      std::string t;
      while (pos_ < end_) {
        if (s_[pos_] == '\'') {
          if (pos_ + 1 < end_ && s_[pos_ + 1] == '\'') { t += '\''; pos_ += 2; continue; }
          ++pos_;
          break;
        }
        t += s_[pos_];
        ++pos_;
      }
      v.kind = Value::Kind::String;
      v.text = std::move(t);
      return true;
    }
    if (c == '.') {
      // Enum .T. / .PLANE. etc.
      ++pos_;
      std::string t;
      while (pos_ < end_ && s_[pos_] != '.') { t += s_[pos_]; ++pos_; }
      if (pos_ < end_ && s_[pos_] == '.') ++pos_;
      v.kind = Value::Kind::Enum;
      v.text = std::move(t);
      return true;
    }
    if (c == '(') {
      // Could be a plain list OR an inline complex/nested entity. We only need lists
      // for our own emission → treat as a list of values.
      v.kind = Value::Kind::List;
      return readArgList(v.list);
    }
    if (c == '$') { ++pos_; v.kind = Value::Kind::Null; return true; }
    if (c == '*') { ++pos_; v.kind = Value::Kind::Null; return true; }
    if (c == '#') { return false; }
    if (c == '+' || c == '-' || c == '.' || std::isdigit(static_cast<unsigned char>(c))) {
      return readNumber(v);
    }
    if (std::isalpha(static_cast<unsigned char>(c))) {
      // A named inline entity, e.g. inside a nested arg — read NAME(...) as Entity.
      std::string name;
      if (!readIdentifier(name)) return false;
      v.kind = Value::Kind::Entity;
      v.text = name;
      skipWs();
      if (pos_ < end_ && s_[pos_] == '(') return readArgList(v.list);
      return true;
    }
    return false;
  }

  bool readNumber(Value& v) {
    std::size_t start = pos_;
    if (pos_ < end_ && (s_[pos_] == '+' || s_[pos_] == '-')) ++pos_;
    while (pos_ < end_ && (std::isdigit(static_cast<unsigned char>(s_[pos_])) || s_[pos_] == '.' ||
                           s_[pos_] == 'e' || s_[pos_] == 'E' || s_[pos_] == '+' ||
                           s_[pos_] == '-')) {
      // Stop a run-away on the sign that begins the exponent only; general parse is fine
      // because a number token here is always followed by ',' or ')'.
      ++pos_;
    }
    const std::string tok = s_.substr(start, pos_ - start);
    v.kind = Value::Kind::Number;
    v.number = std::strtod(tok.c_str(), nullptr);
    return true;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Mapper — rebuild TrimmedNurbsFaces from the parsed record table.
// ─────────────────────────────────────────────────────────────────────────────
class Mapper {
 public:
  explicit Mapper(const std::map<long, Record>& t) : t_(t) {
    // Index every CC_TRIM by the ORIENTED_EDGE #id it pins (its first arg).
    for (const auto& [id, rec] : t_) {
      if (!rec.isComplex && rec.name == "CC_TRIM" && rec.args.size() >= 5 &&
          rec.args[0].kind == Value::Kind::Ref) {
        trimByEdge_.emplace(rec.args[0].ref, &rec);
      }
    }
  }

  std::vector<TrimmedNurbsFace> build() {
    std::vector<TrimmedNurbsFace> faces;
    // Every ADVANCED_FACE becomes one TrimmedNurbsFace.
    for (const auto& [id, rec] : t_) {
      if (!rec.isComplex && rec.name == "ADVANCED_FACE") {
        std::optional<TrimmedNurbsFace> f = mapFace(rec);
        if (!f) return {};  // any unresolved reference aborts (honest decline)
        faces.push_back(std::move(*f));
      }
    }
    return faces;
  }

 private:
  const std::map<long, Record>& t_;
  std::unordered_map<long, const Record*> trimByEdge_;  // ORIENTED_EDGE #id → CC_TRIM

  const Record* rec(long id) const {
    auto it = t_.find(id);
    return it == t_.end() ? nullptr : &it->second;
  }
  static long refOf(const Value& v) { return v.kind == Value::Kind::Ref ? v.ref : 0; }

  // CARTESIAN_POINT('',(x,y,z)) → Point3 (2-D points get z=0).
  std::optional<math::Point3> point(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "CARTESIAN_POINT" || r->args.size() < 2) return std::nullopt;
    const Value& coords = r->args[1];
    if (coords.kind != Value::Kind::List) return std::nullopt;
    math::Point3 p{};
    if (coords.list.size() >= 1) p.x = coords.list[0].number;
    if (coords.list.size() >= 2) p.y = coords.list[1].number;
    if (coords.list.size() >= 3) p.z = coords.list[2].number;
    return p;
  }
  std::optional<math::Vec3> dir(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "DIRECTION" || r->args.size() < 2) return std::nullopt;
    const Value& coords = r->args[1];
    if (coords.kind != Value::Kind::List) return std::nullopt;
    math::Vec3 d{};
    if (coords.list.size() >= 1) d.x = coords.list[0].number;
    if (coords.list.size() >= 2) d.y = coords.list[1].number;
    if (coords.list.size() >= 3) d.z = coords.list[2].number;
    return d;
  }
  std::optional<math::Ax3> placement(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "AXIS2_PLACEMENT_3D" || r->args.size() < 4)
      return std::nullopt;
    auto o = point(refOf(r->args[1]));
    auto z = dir(refOf(r->args[2]));
    auto x = dir(refOf(r->args[3]));
    if (!o || !z || !x) return std::nullopt;
    return math::Ax3::fromAxisAndRef(*o, math::Dir3{*z}, math::Dir3{*x});
  }

  // Expand a *_WITH_KNOTS (mults,knots) pair into a flat knot vector.
  static std::vector<double> flatKnots(const Value& multsV, const Value& knotsV) {
    std::vector<int> mults;
    std::vector<double> vals;
    if (multsV.kind == Value::Kind::List)
      for (const Value& m : multsV.list) mults.push_back(static_cast<int>(m.number));
    if (knotsV.kind == Value::Kind::List)
      for (const Value& k : knotsV.list) vals.push_back(k.number);
    return expandKnots(vals, mults);
  }

  // ── Surface mapping ──────────────────────────────────────────────────────────
  std::optional<FaceSurface> surface(long id) const {
    const Record* r = rec(id);
    if (!r) return std::nullopt;
    if (r->isComplex) return rationalBsplineSurface(*r);
    if (r->name == "PLANE") return analyticSurface(*r, FaceSurface::Kind::Plane, 0);
    if (r->name == "CYLINDRICAL_SURFACE") return analyticSurface(*r, FaceSurface::Kind::Cylinder, 1);
    if (r->name == "CONICAL_SURFACE") return analyticSurface(*r, FaceSurface::Kind::Cone, 2);
    if (r->name == "SPHERICAL_SURFACE") return analyticSurface(*r, FaceSurface::Kind::Sphere, 1);
    if (r->name == "TOROIDAL_SURFACE") return analyticSurface(*r, FaceSurface::Kind::Torus, 2);
    if (r->name == "B_SPLINE_SURFACE_WITH_KNOTS") return bsplineSurface(*r);
    return std::nullopt;
  }

  // nRadii: 0 (plane), 1 (radius), 2 (radius + second: semiAngle for cone / minor for torus).
  std::optional<FaceSurface> analyticSurface(const Record& r, FaceSurface::Kind k, int nRadii) const {
    if (r.args.size() < 2) return std::nullopt;
    auto ax = placement(refOf(r.args[1]));
    if (!ax) return std::nullopt;
    FaceSurface s;
    s.kind = k;
    s.frame = *ax;
    if (nRadii >= 1 && r.args.size() >= 3) s.radius = r.args[2].number;
    if (nRadii >= 2 && r.args.size() >= 4) {
      if (k == FaceSurface::Kind::Cone) s.semiAngle = r.args[3].number;
      else s.minorRadius = r.args[3].number;  // torus minor radius
    }
    return s;
  }

  // B_SPLINE_SURFACE_WITH_KNOTS('',degU,degV,grid,form,uc,vc,sp,uMults,vMults,uKnots,vKnots,form)
  std::optional<FaceSurface> bsplineSurface(const Record& r) const {
    // args: 0=name 1=degU 2=degV 3=grid 4=form 5=uClosed 6=vClosed 7=selfInt
    //       8=uMults 9=vMults 10=uKnots 11=vKnots 12=form
    if (r.args.size() < 12) return std::nullopt;
    FaceSurface s;
    s.kind = FaceSurface::Kind::BSpline;
    s.degreeU = static_cast<int>(r.args[1].number);
    s.degreeV = static_cast<int>(r.args[2].number);
    if (!readGrid(r.args[3], s)) return std::nullopt;
    s.knotsU = flatKnots(r.args[8], r.args[10]);
    s.knotsV = flatKnots(r.args[9], r.args[11]);
    return s;
  }

  // The rational surface complex instance carries B_SPLINE_SURFACE(degU,degV,grid,...),
  // B_SPLINE_SURFACE_WITH_KNOTS(uMults,vMults,uKnots,vKnots,form), RATIONAL_B_SPLINE_SURFACE(wgrid).
  std::optional<FaceSurface> rationalBsplineSurface(const Record& r) const {
    const ValueList* base = nullptr;
    const ValueList* knots = nullptr;
    const ValueList* rat = nullptr;
    for (const auto& [name, args] : r.complex) {
      if (name == "B_SPLINE_SURFACE") base = &args;
      else if (name == "B_SPLINE_SURFACE_WITH_KNOTS") knots = &args;
      else if (name == "RATIONAL_B_SPLINE_SURFACE") rat = &args;
    }
    if (!base || !knots || !rat) return std::nullopt;
    if (base->size() < 3 || knots->size() < 4 || rat->empty()) return std::nullopt;
    FaceSurface s;
    s.kind = FaceSurface::Kind::BSpline;
    s.degreeU = static_cast<int>((*base)[0].number);
    s.degreeV = static_cast<int>((*base)[1].number);
    if (!readGrid((*base)[2], s)) return std::nullopt;
    s.knotsU = flatKnots((*knots)[0], (*knots)[2]);
    s.knotsV = flatKnots((*knots)[1], (*knots)[3]);
    // weights grid row-major (U outer, V inner)
    const Value& wg = (*rat)[0];
    if (wg.kind != Value::Kind::List) return std::nullopt;
    for (const Value& row : wg.list) {
      if (row.kind != Value::Kind::List) return std::nullopt;
      for (const Value& w : row.list) s.weights.push_back(w.number);
    }
    if (static_cast<int>(s.weights.size()) != s.nPolesU * s.nPolesV) return std::nullopt;
    return s;
  }

  // Read the CARTESIAN_POINT grid (list of U-rows, each a list of V refs) into s.poles.
  bool readGrid(const Value& grid, FaceSurface& s) const {
    if (grid.kind != Value::Kind::List) return false;
    s.nPolesU = static_cast<int>(grid.list.size());
    s.nPolesV = 0;
    for (const Value& row : grid.list) {
      if (row.kind != Value::Kind::List) return false;
      if (s.nPolesV == 0) s.nPolesV = static_cast<int>(row.list.size());
      if (static_cast<int>(row.list.size()) != s.nPolesV) return false;
      for (const Value& pref : row.list) {
        auto p = point(refOf(pref));
        if (!p) return false;
        s.poles.push_back(*p);
      }
    }
    return s.nPolesU > 0 && s.nPolesV > 0;
  }

  // ── PCurve mapping ───────────────────────────────────────────────────────────
  // A PCURVE('',surf,DEFINITIONAL_REPRESENTATION('',(#curve2d),#0)) → the 2-D curve.
  std::optional<PCurve> pcurveFromPcurveEntity(long pcId) const {
    const Record* r = rec(pcId);
    if (!r || r->isComplex || r->name != "PCURVE" || r->args.size() < 3) return std::nullopt;
    const Record* drep = rec(refOf(r->args[2]));
    if (!drep || drep->isComplex || drep->name != "DEFINITIONAL_REPRESENTATION" ||
        drep->args.size() < 2)
      return std::nullopt;
    const Value& items = drep->args[1];
    if (items.kind != Value::Kind::List || items.list.empty()) return std::nullopt;
    return curve2d(refOf(items.list[0]));
  }

  std::optional<PCurve> curve2d(long id) const {
    const Record* r = rec(id);
    if (!r) return std::nullopt;
    if (r->isComplex) return rationalBspline2d(*r);
    if (r->name == "LINE") return line2d(*r);
    if (r->name == "CIRCLE") return circle2d(*r);
    if (r->name == "ELLIPSE") return ellipse2d(*r);
    if (r->name == "B_SPLINE_CURVE_WITH_KNOTS") return bspline2d(*r);
    return std::nullopt;
  }

  std::optional<math::Point3> point2(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "CARTESIAN_POINT" || r->args.size() < 2) return std::nullopt;
    const Value& coords = r->args[1];
    if (coords.kind != Value::Kind::List) return std::nullopt;
    math::Point3 p{};
    if (coords.list.size() >= 1) p.x = coords.list[0].number;
    if (coords.list.size() >= 2) p.y = coords.list[1].number;
    return p;
  }
  std::optional<math::Vec3> dir2d(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "DIRECTION" || r->args.size() < 2) return std::nullopt;
    const Value& coords = r->args[1];
    if (coords.kind != Value::Kind::List) return std::nullopt;
    math::Vec3 d{};
    if (coords.list.size() >= 1) d.x = coords.list[0].number;
    if (coords.list.size() >= 2) d.y = coords.list[1].number;
    return d;
  }
  // AXIS2_PLACEMENT_2D('',#origin,#refDir) → the 2-D center.
  std::optional<math::Point3> placement2dOrigin(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "AXIS2_PLACEMENT_2D" || r->args.size() < 2)
      return std::nullopt;
    return point2(refOf(r->args[1]));
  }

  std::optional<PCurve> line2d(const Record& r) const {
    if (r.args.size() < 3) return std::nullopt;
    auto o = point2(refOf(r.args[1]));
    const Record* vec = rec(refOf(r.args[2]));
    if (!o || !vec || vec->isComplex || vec->name != "VECTOR" || vec->args.size() < 3)
      return std::nullopt;
    auto d = dir2d(refOf(vec->args[1]));
    const double mag = vec->args[2].number;
    if (!d) return std::nullopt;
    PCurve c;
    c.kind = EdgeCurve::Kind::Line;
    c.origin2d = *o;
    c.dir2d = math::Vec3{d->x * mag, d->y * mag, 0.0};
    return c;
  }
  std::optional<PCurve> circle2d(const Record& r) const {
    if (r.args.size() < 3) return std::nullopt;
    auto o = placement2dOrigin(refOf(r.args[1]));
    if (!o) return std::nullopt;
    PCurve c;
    c.kind = EdgeCurve::Kind::Circle;
    c.origin2d = *o;
    c.dir2d = math::Vec3{r.args[2].number, 0.0, 0.0};  // radius in dir2d.x
    return c;
  }
  std::optional<PCurve> ellipse2d(const Record& r) const {
    if (r.args.size() < 4) return std::nullopt;
    auto o = placement2dOrigin(refOf(r.args[1]));
    if (!o) return std::nullopt;
    PCurve c;
    c.kind = EdgeCurve::Kind::Ellipse;
    c.origin2d = *o;
    c.dir2d = math::Vec3{r.args[2].number, r.args[3].number, 0.0};  // a,b
    return c;
  }
  // B_SPLINE_CURVE_WITH_KNOTS('',deg,poles,form,closed,selfint,mults,knots,form)
  std::optional<PCurve> bspline2d(const Record& r) const {
    if (r.args.size() < 8) return std::nullopt;
    PCurve c;
    c.kind = EdgeCurve::Kind::BSpline;
    c.degree = static_cast<int>(r.args[1].number);
    const Value& poles = r.args[2];
    if (poles.kind != Value::Kind::List) return std::nullopt;
    for (const Value& pref : poles.list) {
      auto p = point2(refOf(pref));
      if (!p) return std::nullopt;
      c.poles2d.push_back(*p);
    }
    c.knots = flatKnots(r.args[6], r.args[7]);
    return c;
  }
  // Rational 2-D complex instance.
  std::optional<PCurve> rationalBspline2d(const Record& r) const {
    const ValueList* base = nullptr;
    const ValueList* knots = nullptr;
    const ValueList* rat = nullptr;
    for (const auto& [name, args] : r.complex) {
      if (name == "B_SPLINE_CURVE") base = &args;
      else if (name == "B_SPLINE_CURVE_WITH_KNOTS") knots = &args;
      else if (name == "RATIONAL_B_SPLINE_CURVE") rat = &args;
    }
    if (!base || !knots || !rat) return std::nullopt;
    if (base->size() < 2 || knots->size() < 2 || rat->empty()) return std::nullopt;
    PCurve c;
    c.kind = EdgeCurve::Kind::BSpline;
    c.degree = static_cast<int>((*base)[0].number);
    const Value& poles = (*base)[1];
    if (poles.kind != Value::Kind::List) return std::nullopt;
    for (const Value& pref : poles.list) {
      auto p = point2(refOf(pref));
      if (!p) return std::nullopt;
      c.poles2d.push_back(*p);
    }
    c.knots = flatKnots((*knots)[0], (*knots)[1]);
    const Value& w = (*rat)[0];
    if (w.kind != Value::Kind::List) return std::nullopt;
    for (const Value& wi : w.list) c.weights.push_back(wi.number);
    if (c.weights.size() != c.poles2d.size()) return std::nullopt;
    return c;
  }

  // ── Loop / bound mapping ─────────────────────────────────────────────────────
  // ORIENTED_EDGE('',*,*,#edge_curve,orient). The pcurve + exact trim come from the
  // CC_TRIM record that references the same PCURVE the ORIENTED_EDGE's edge carries.
  // Since our emitter emits the PCURVE + CC_TRIM immediately AFTER the ORIENTED_EDGE,
  // we index CC_TRIM records and match them to loop segments positionally per loop.
  std::optional<TrimLoop> loop(long edgeLoopId) const {
    const Record* r = rec(edgeLoopId);
    if (!r || r->isComplex || r->name != "EDGE_LOOP" || r->args.size() < 2) return std::nullopt;
    const Value& edges = r->args[1];
    if (edges.kind != Value::Kind::List) return std::nullopt;
    TrimLoop tl;
    for (const Value& oe : edges.list) {
      auto seg = segment(refOf(oe));
      if (!seg) return std::nullopt;
      tl.push_back(std::move(*seg));
    }
    return tl;
  }

  // One ORIENTED_EDGE → PcurveSegment. The CC_TRIM keyed on THIS oriented edge #id
  // carries CC_TRIM(#orientedEdge,#pcurve,first,last,reversed) — the exact trim.
  std::optional<PcurveSegment> segment(long orientedEdgeId) const {
    auto it = trimByEdge_.find(orientedEdgeId);
    if (it == trimByEdge_.end()) return std::nullopt;
    const Record* trim = it->second;
    if (trim->args.size() < 5) return std::nullopt;
    const long pcId = refOf(trim->args[1]);
    auto pc = pcurveFromPcurveEntity(pcId);
    if (!pc) return std::nullopt;
    PcurveSegment seg;
    seg.curve = std::move(*pc);
    seg.first = trim->args[2].number;
    seg.last = trim->args[3].number;
    seg.reversed = (trim->args[4].kind == Value::Kind::Enum && trim->args[4].text == "T");
    return seg;
  }

  std::optional<TrimmedNurbsFace> mapFace(const Record& r) const {
    // ADVANCED_FACE('',(#bounds...),#surface,orient)
    if (r.args.size() < 3) return std::nullopt;
    const Value& boundsV = r.args[1];
    auto surf = surface(refOf(r.args[2]));
    if (!surf || boundsV.kind != Value::Kind::List) return std::nullopt;
    TrimmedNurbsFace f;
    f.surface = *surf;
    for (const Value& bref : boundsV.list) {
      const Record* b = rec(refOf(bref));
      if (!b || b->isComplex) return std::nullopt;
      const bool isOuter = b->name == "FACE_OUTER_BOUND";
      if (b->name != "FACE_OUTER_BOUND" && b->name != "FACE_BOUND") return std::nullopt;
      if (b->args.size() < 2) return std::nullopt;
      auto tl = loop(refOf(b->args[1]));
      if (!tl) return std::nullopt;
      if (isOuter) f.outer = std::move(*tl);
      else f.holes.push_back(std::move(*tl));
    }
    return f;
  }
};

}  // namespace

std::vector<TrimmedNurbsFace> readStepBrep(const std::string& step) {
  Parser p(step);
  if (!p.parseTable()) return {};
  Mapper m(p.table());
  return m.build();
}

// ═════════════════════════════════════════════════════════════════════════════
// GENERAL (external) AP203/214 IMPORT — readStepBrepExternal
//
// Reuses the Parser table (order-independent, forward-ref-safe, comment-tolerant).
// A new ExternalMapper walks the FULL topology graph (ADVANCED_FACE → FACE_*_BOUND →
// EDGE_LOOP → ORIENTED_EDGE → EDGE_CURVE → LINE / CIRCLE / B_SPLINE_CURVE_WITH_KNOTS
// + VERTEX_POINT), recovers analytic / B-spline surfaces, and DERIVES each edge's 2-D
// pcurve by inverting the 3-D edge geometry into the surface's (u,v) plane.
// ═════════════════════════════════════════════════════════════════════════════
namespace {

using math::Ax3;
using math::Point3;
using math::Vec3;

// Local (u,v,w) coordinates of a world point in an analytic frame.
Vec3 toLocal(const Ax3& f, const Point3& p) {
  const Vec3 d = p - f.origin;
  return {math::dot(d, f.x.vec()), math::dot(d, f.y.vec()), math::dot(d, f.z.vec())};
}

// Invert a world point onto an analytic surface → (u,v). `uHint` disambiguates the
// angular branch so a swept loop stays continuous. Returns false for a surface kind
// with no closed-form inverse (a B-spline surface — handled by sampled projection).
bool inverseSurface(const FaceSurface& s, const Point3& p, double uHint, ParamPoint& out) {
  using K = FaceSurface::Kind;
  const Vec3 l = toLocal(s.frame, p);
  switch (s.kind) {
    case K::Plane:
      out = {l.x, l.y};
      return true;
    case K::Cylinder:
      out = {std::atan2(l.y, l.x), l.z};
      return true;
    case K::Cone: {
      // r(v) = radius + v·sin α ; z = v·cos α  ⇒ v from z (α≠90°) else from radius.
      const double ca = std::cos(s.semiAngle);
      out.u = std::atan2(l.y, l.x);
      out.v = std::abs(ca) > 1e-12 ? l.z / ca : (std::hypot(l.x, l.y) - s.radius) /
                                                    std::max(std::sin(s.semiAngle), 1e-300);
      return true;
    }
    case K::Sphere: {
      // v ∈ [-π/2, π/2] latitude, u longitude.
      out.u = std::atan2(l.y, l.x);
      const double rc = std::hypot(l.x, l.y);
      out.v = std::atan2(l.z, rc);
      return true;
    }
    case K::Torus: {
      out.u = std::atan2(l.y, l.x);
      const double rr = std::hypot(l.x, l.y) - s.radius;  // r·cos v
      out.v = std::atan2(l.z, rr);                        // z = r·sin v
      return true;
    }
    case K::BSpline:
    case K::Bezier:
    default:
      return false;
  }
  (void)uHint;
}

// Unwrap `angle` to the branch nearest `ref` (keeps a swept angular pcurve continuous
// across the ±π seam of atan2).
double unwrapNear(double angle, double ref) {
  const double twoPi = 2.0 * M_PI;
  double a = angle;
  while (a - ref > M_PI) a -= twoPi;
  while (ref - a > M_PI) a += twoPi;
  return a;
}

// ── A resolved 3-D edge curve with a sampler over [t0,t1]. ────────────────────
struct Edge3d {
  enum class Kind { Line, Circle, Ellipse, BSpline } kind = Kind::Line;
  // Analytic.
  Ax3 frame{};
  double radius = 0.0, minorRadius = 0.0;
  Point3 lineOrigin{};
  Vec3 lineDir{1, 0, 0};
  // B-spline.
  int degree = 0;
  std::vector<Point3> poles;
  std::vector<double> weights;
  std::vector<double> knots;
  // Sample parameter range (set from the bounding vertices).
  double t0 = 0.0, t1 = 1.0;

  Point3 eval(double t) const {
    switch (kind) {
      case Kind::Line:
        return lineOrigin + lineDir * t;
      case Kind::Circle:
        return frame.origin + frame.x.vec() * (radius * std::cos(t)) +
               frame.y.vec() * (radius * std::sin(t));
      case Kind::Ellipse:
        return frame.origin + frame.x.vec() * (radius * std::cos(t)) +
               frame.y.vec() * (minorRadius * std::sin(t));
      case Kind::BSpline:
      default:
        return weights.empty()
                   ? math::curvePoint(degree, {poles.data(), poles.size()},
                                      {knots.data(), knots.size()}, t)
                   : math::nurbsCurvePoint(degree, {poles.data(), poles.size()},
                                           {weights.data(), weights.size()},
                                           {knots.data(), knots.size()}, t);
    }
  }
};

// ── ExternalMapper ────────────────────────────────────────────────────────────
class ExternalMapper {
 public:
  ExternalMapper(const std::map<long, Record>& t, ExternalImportReport* rep)
      : t_(t), rep_(rep) {}

  std::vector<TrimmedNurbsFace> build() {
    std::vector<TrimmedNurbsFace> faces;
    for (const auto& [id, rec] : t_) {
      if (rec.isComplex || rec.name != "ADVANCED_FACE") continue;
      if (rep_) ++rep_->facesSeen;
      std::string why;
      std::optional<TrimmedNurbsFace> f = mapFace(rec, why);
      if (f) {
        faces.push_back(std::move(*f));
        if (rep_) ++rep_->facesImported;
      } else {
        if (rep_) {
          ++rep_->facesSkipped;
          rep_->skipReasons.push_back("#" + std::to_string(id) + ": " + why);
        }
      }
    }
    return faces;
  }

 private:
  const std::map<long, Record>& t_;
  ExternalImportReport* rep_;

  const Record* rec(long id) const {
    auto it = t_.find(id);
    return it == t_.end() ? nullptr : &it->second;
  }
  static long refOf(const Value& v) { return v.kind == Value::Kind::Ref ? v.ref : 0; }
  static double num(const Value& v) { return v.number; }

  // ── Primitives ───────────────────────────────────────────────────────────────
  std::optional<Point3> point(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "CARTESIAN_POINT" || r->args.size() < 2)
      return std::nullopt;
    const Value& c = r->args[1];
    if (c.kind != Value::Kind::List) return std::nullopt;
    Point3 p{};
    if (c.list.size() >= 1) p.x = c.list[0].number;
    if (c.list.size() >= 2) p.y = c.list[1].number;
    if (c.list.size() >= 3) p.z = c.list[2].number;
    return p;
  }
  std::optional<Vec3> dir(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "DIRECTION" || r->args.size() < 2) return std::nullopt;
    const Value& c = r->args[1];
    if (c.kind != Value::Kind::List) return std::nullopt;
    Vec3 d{};
    if (c.list.size() >= 1) d.x = c.list[0].number;
    if (c.list.size() >= 2) d.y = c.list[1].number;
    if (c.list.size() >= 3) d.z = c.list[2].number;
    return d;
  }
  // AXIS2_PLACEMENT_3D('',#loc,#axis,#refDir). axis / refDir OPTIONAL ($) → defaults.
  std::optional<Ax3> placement(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "AXIS2_PLACEMENT_3D" || r->args.size() < 2)
      return std::nullopt;
    auto o = point(refOf(r->args[1]));
    if (!o) return std::nullopt;
    Vec3 z{0, 0, 1}, x{1, 0, 0};
    if (r->args.size() >= 3 && r->args[2].kind == Value::Kind::Ref)
      if (auto zz = dir(refOf(r->args[2]))) z = *zz;
    if (r->args.size() >= 4 && r->args[3].kind == Value::Kind::Ref)
      if (auto xx = dir(refOf(r->args[3]))) x = *xx;
    return Ax3::fromAxisAndRef(*o, math::Dir3{z}, math::Dir3{x});
  }
  std::optional<Point3> vertexPoint(long id) const {
    const Record* r = rec(id);
    if (!r || r->isComplex || r->name != "VERTEX_POINT" || r->args.size() < 2) return std::nullopt;
    return point(refOf(r->args[1]));
  }

  // ── Surface ──────────────────────────────────────────────────────────────────
  std::optional<FaceSurface> surface(long id, std::string& why) const {
    const Record* r = rec(id);
    if (!r) { why = "surface ref unresolved"; return std::nullopt; }
    if (r->isComplex) return rationalBsplineSurface(*r, why);
    if (r->name == "PLANE") return analytic(*r, FaceSurface::Kind::Plane, 0, why);
    if (r->name == "CYLINDRICAL_SURFACE") return analytic(*r, FaceSurface::Kind::Cylinder, 1, why);
    if (r->name == "CONICAL_SURFACE") return analytic(*r, FaceSurface::Kind::Cone, 2, why);
    if (r->name == "SPHERICAL_SURFACE") return analytic(*r, FaceSurface::Kind::Sphere, 1, why);
    if (r->name == "TOROIDAL_SURFACE") return analytic(*r, FaceSurface::Kind::Torus, 2, why);
    if (r->name == "B_SPLINE_SURFACE_WITH_KNOTS") return bsplineSurface(*r, why);
    why = "unsupported surface kind " + r->name;
    return std::nullopt;
  }

  std::optional<FaceSurface> analytic(const Record& r, FaceSurface::Kind k, int nRadii,
                                      std::string& why) const {
    if (r.args.size() < 2) { why = "analytic surface missing placement"; return std::nullopt; }
    auto ax = placement(refOf(r.args[1]));
    if (!ax) { why = "analytic placement unresolved"; return std::nullopt; }
    FaceSurface s;
    s.kind = k;
    s.frame = *ax;
    if (nRadii >= 1 && r.args.size() >= 3) s.radius = num(r.args[2]);
    if (nRadii >= 2 && r.args.size() >= 4) {
      if (k == FaceSurface::Kind::Cone) s.semiAngle = num(r.args[3]);
      else s.minorRadius = num(r.args[3]);
    }
    // A TOROIDAL_SURFACE carries (major, minor); the writer's Torus.radius = MAJOR.
    if (k == FaceSurface::Kind::Torus && r.args.size() >= 4) {
      s.radius = num(r.args[2]);
      s.minorRadius = num(r.args[3]);
    }
    return s;
  }

  static std::vector<double> flatKnots(const Value& multsV, const Value& knotsV) {
    std::vector<int> mults;
    std::vector<double> vals;
    if (multsV.kind == Value::Kind::List)
      for (const Value& m : multsV.list) mults.push_back(static_cast<int>(m.number));
    if (knotsV.kind == Value::Kind::List)
      for (const Value& k : knotsV.list) vals.push_back(k.number);
    return expandKnots(vals, mults);
  }

  bool readGrid(const Value& grid, FaceSurface& s) const {
    if (grid.kind != Value::Kind::List) return false;
    s.nPolesU = static_cast<int>(grid.list.size());
    s.nPolesV = 0;
    for (const Value& row : grid.list) {
      if (row.kind != Value::Kind::List) return false;
      if (s.nPolesV == 0) s.nPolesV = static_cast<int>(row.list.size());
      if (static_cast<int>(row.list.size()) != s.nPolesV) return false;
      for (const Value& pref : row.list) {
        auto p = point(refOf(pref));
        if (!p) return false;
        s.poles.push_back(*p);
      }
    }
    return s.nPolesU > 0 && s.nPolesV > 0;
  }

  std::optional<FaceSurface> bsplineSurface(const Record& r, std::string& why) const {
    if (r.args.size() < 12) { why = "B_SPLINE_SURFACE_WITH_KNOTS too few args"; return std::nullopt; }
    FaceSurface s;
    s.kind = FaceSurface::Kind::BSpline;
    s.degreeU = static_cast<int>(num(r.args[1]));
    s.degreeV = static_cast<int>(num(r.args[2]));
    if (!readGrid(r.args[3], s)) { why = "B_SPLINE_SURFACE grid unresolved"; return std::nullopt; }
    s.knotsU = flatKnots(r.args[8], r.args[10]);
    s.knotsV = flatKnots(r.args[9], r.args[11]);
    return s;
  }

  std::optional<FaceSurface> rationalBsplineSurface(const Record& r, std::string& why) const {
    const ValueList* base = nullptr;
    const ValueList* knots = nullptr;
    const ValueList* rat = nullptr;
    for (const auto& [name, args] : r.complex) {
      if (name == "B_SPLINE_SURFACE") base = &args;
      else if (name == "B_SPLINE_SURFACE_WITH_KNOTS") knots = &args;
      else if (name == "RATIONAL_B_SPLINE_SURFACE") rat = &args;
    }
    if (!base || !knots || !rat) { why = "complex surface not a rational B-spline"; return std::nullopt; }
    if (base->size() < 3 || knots->size() < 4 || rat->empty()) {
      why = "rational B-spline surface malformed"; return std::nullopt;
    }
    FaceSurface s;
    s.kind = FaceSurface::Kind::BSpline;
    s.degreeU = static_cast<int>((*base)[0].number);
    s.degreeV = static_cast<int>((*base)[1].number);
    if (!readGrid((*base)[2], s)) { why = "rational B-spline grid unresolved"; return std::nullopt; }
    s.knotsU = flatKnots((*knots)[0], (*knots)[2]);
    s.knotsV = flatKnots((*knots)[1], (*knots)[3]);
    const Value& wg = (*rat)[0];
    if (wg.kind != Value::Kind::List) { why = "weights not a list"; return std::nullopt; }
    for (const Value& row : wg.list) {
      if (row.kind != Value::Kind::List) { why = "weights row not a list"; return std::nullopt; }
      for (const Value& w : row.list) s.weights.push_back(w.number);
    }
    if (static_cast<int>(s.weights.size()) != s.nPolesU * s.nPolesV) {
      why = "weight count != pole count"; return std::nullopt;
    }
    return s;
  }

  // ── 3-D edge curve ────────────────────────────────────────────────────────────
  std::optional<Edge3d> curve3d(long id, std::string& why) const {
    const Record* r = rec(id);
    if (!r) { why = "3-D curve ref unresolved"; return std::nullopt; }
    if (r->isComplex) return rationalCurve3d(*r, why);
    if (r->name == "LINE") return line3d(*r, why);
    if (r->name == "CIRCLE") return circle3d(*r, why);
    if (r->name == "ELLIPSE") return ellipse3d(*r, why);
    if (r->name == "B_SPLINE_CURVE_WITH_KNOTS") return bspline3d(*r, why);
    why = "unsupported edge curve kind " + r->name;
    return std::nullopt;
  }
  // LINE('',#origin,#VECTOR('',#dir,mag)).
  std::optional<Edge3d> line3d(const Record& r, std::string& why) const {
    if (r.args.size() < 3) { why = "LINE too few args"; return std::nullopt; }
    auto o = point(refOf(r.args[1]));
    const Record* vec = rec(refOf(r.args[2]));
    if (!o || !vec || vec->isComplex || vec->name != "VECTOR" || vec->args.size() < 3) {
      why = "LINE vector unresolved"; return std::nullopt;
    }
    auto d = dir(refOf(vec->args[1]));
    if (!d) { why = "LINE direction unresolved"; return std::nullopt; }
    Edge3d e;
    e.kind = Edge3d::Kind::Line;
    e.lineOrigin = *o;
    e.lineDir = *d * vec->args[2].number;
    return e;
  }
  // CIRCLE('',#AXIS2_PLACEMENT_3D,radius).
  std::optional<Edge3d> circle3d(const Record& r, std::string& why) const {
    if (r.args.size() < 3) { why = "CIRCLE too few args"; return std::nullopt; }
    auto ax = placement(refOf(r.args[1]));
    if (!ax) { why = "CIRCLE placement unresolved"; return std::nullopt; }
    Edge3d e;
    e.kind = Edge3d::Kind::Circle;
    e.frame = *ax;
    e.radius = num(r.args[2]);
    return e;
  }
  std::optional<Edge3d> ellipse3d(const Record& r, std::string& why) const {
    if (r.args.size() < 4) { why = "ELLIPSE too few args"; return std::nullopt; }
    auto ax = placement(refOf(r.args[1]));
    if (!ax) { why = "ELLIPSE placement unresolved"; return std::nullopt; }
    Edge3d e;
    e.kind = Edge3d::Kind::Ellipse;
    e.frame = *ax;
    e.radius = num(r.args[2]);       // semi-major
    e.minorRadius = num(r.args[3]);  // semi-minor
    return e;
  }
  std::optional<Edge3d> bspline3d(const Record& r, std::string& why) const {
    if (r.args.size() < 8) { why = "B_SPLINE_CURVE_WITH_KNOTS too few args"; return std::nullopt; }
    Edge3d e;
    e.kind = Edge3d::Kind::BSpline;
    e.degree = static_cast<int>(num(r.args[1]));
    const Value& poles = r.args[2];
    if (poles.kind != Value::Kind::List) { why = "B_SPLINE_CURVE poles not a list"; return std::nullopt; }
    for (const Value& pref : poles.list) {
      auto p = point(refOf(pref));
      if (!p) { why = "B_SPLINE_CURVE pole unresolved"; return std::nullopt; }
      e.poles.push_back(*p);
    }
    e.knots = flatKnots(r.args[6], r.args[7]);
    return e;
  }
  std::optional<Edge3d> rationalCurve3d(const Record& r, std::string& why) const {
    const ValueList* base = nullptr;
    const ValueList* knots = nullptr;
    const ValueList* rat = nullptr;
    for (const auto& [name, args] : r.complex) {
      if (name == "B_SPLINE_CURVE") base = &args;
      else if (name == "B_SPLINE_CURVE_WITH_KNOTS") knots = &args;
      else if (name == "RATIONAL_B_SPLINE_CURVE") rat = &args;
    }
    if (!base || !knots || !rat) { why = "complex curve not a rational B-spline"; return std::nullopt; }
    if (base->size() < 2 || knots->size() < 2 || rat->empty()) {
      why = "rational B-spline curve malformed"; return std::nullopt;
    }
    Edge3d e;
    e.kind = Edge3d::Kind::BSpline;
    e.degree = static_cast<int>((*base)[0].number);
    const Value& poles = (*base)[1];
    if (poles.kind != Value::Kind::List) { why = "rational curve poles not a list"; return std::nullopt; }
    for (const Value& pref : poles.list) {
      auto p = point(refOf(pref));
      if (!p) { why = "rational curve pole unresolved"; return std::nullopt; }
      e.poles.push_back(*p);
    }
    e.knots = flatKnots((*knots)[0], (*knots)[1]);
    const Value& w = (*rat)[0];
    if (w.kind != Value::Kind::List) { why = "rational curve weights not a list"; return std::nullopt; }
    for (const Value& wi : w.list) e.weights.push_back(wi.number);
    if (e.weights.size() != e.poles.size()) { why = "rational curve weight count"; return std::nullopt; }
    return e;
  }

  // Set the edge's [t0,t1] sampling range from the bounding vertices (for the analytic
  // forms the natural parameter is angular / signed length; the vertices pin the arc).
  void setEdgeRange(Edge3d& e, const Point3& v0, const Point3& v1) const {
    switch (e.kind) {
      case Edge3d::Kind::Line: {
        const double len2 = math::normSquared(e.lineDir);
        if (len2 <= 0) { e.t0 = 0; e.t1 = 1; break; }
        e.t0 = math::dot(v0 - e.lineOrigin, e.lineDir) / len2;
        e.t1 = math::dot(v1 - e.lineOrigin, e.lineDir) / len2;
        break;
      }
      case Edge3d::Kind::Circle:
      case Edge3d::Kind::Ellipse: {
        const Vec3 d0 = v0 - e.frame.origin, d1 = v1 - e.frame.origin;
        double a0 = std::atan2(math::dot(d0, e.frame.y.vec()), math::dot(d0, e.frame.x.vec()));
        double a1 = std::atan2(math::dot(d1, e.frame.y.vec()), math::dot(d1, e.frame.x.vec()));
        // Full-circle edge: the two vertices coincide → sweep the whole 2π (CCW).
        if (math::distance(v0, v1) < 1e-9) { a0 = 0.0; a1 = 2.0 * M_PI; }
        else {
          // Partial arc: pick the MINOR arc (|Δ| ≤ π) — the region-correct default for a
          // trim-loop edge whose sweep direction is not otherwise pinned. The old code
          // forced CCW (+2π whenever a1<a0), which sent a clockwise arc THE LONG WAY —
          // on a periodic surface that turned a small seam-crossing patch into a spurious
          // near-full-period band. Choosing the nearest branch keeps the loop closing and
          // its unwrapped u-extent equal to the true subtended angle (see derivePcurve's
          // loop-threaded u-hint, which then keeps the whole loop on one branch).
          a1 = unwrapNear(a1, a0);
        }
        e.t0 = a0;
        e.t1 = a1;
        break;
      }
      case Edge3d::Kind::BSpline:
      default:
        e.t0 = e.knots.empty() ? 0.0 : e.knots.front();
        e.t1 = e.knots.empty() ? 1.0 : e.knots.back();
        break;
    }
  }

  // ── Derive a 2-D pcurve for one edge on the surface. ──────────────────────────
  // Sample the 3-D edge over [t0,t1], invert each point into (u,v) (closed-form for the
  // analytic surfaces; sampled projection for a B-spline surface via a coarse grid
  // nearest-point) and build a degree-1 (polyline) pcurve through the (u,v) samples.
  // Region-correct: the flattened trim classification ray-casts exactly this polyline.
  //
  // SEAM-CONSISTENT ACROSS A LOOP. On a periodic analytic surface (cylinder/cone/
  // sphere/torus) the angular u is inverted via atan2 into (-π,π]; an edge whose arc
  // crosses that branch cut is kept continuous WITHIN the edge by unwrapNear. But a
  // trim LOOP is a chain of edges, and each edge inverted from a fresh u-branch may
  // land on a DIFFERENT 2π period than its predecessor — leaving the loop polyline with
  // a spurious ±2π jump between edges, which classify() then reads as a self-touching
  // pinch (→ Unknown) and classifySeam() mis-unwraps into a fabricated full band. To
  // avoid that, the caller (loop()) threads a shared `loopUHint`: the FIRST edge seeds
  // it from its own start branch and the loop's u lives on ONE continuous unwrapped
  // branch thereafter. This is an EXACT identity on the periodic surface (u and u±2π are
  // the same physical point), so it never widens a tolerance or moves a point off S.
  std::optional<PCurve> derivePcurve(const FaceSurface& s, const Edge3d& e, bool reversed,
                                     std::string& why, double* loopUHint = nullptr) const {
    const int n = 24;
    std::vector<ParamPoint> uv;
    uv.reserve(n + 1);
    // Seed the branch: on a repeat edge of a periodic-surface loop, continue from the
    // previous edge's final u so the whole loop stays on one unwrapped branch.
    double uHint = (loopUHint != nullptr) ? *loopUHint : 0.0;
    bool haveHint = (loopUHint != nullptr) && std::isfinite(*loopUHint);
    for (int i = 0; i <= n; ++i) {
      const double frac = double(i) / n;
      const double t = e.t0 + (e.t1 - e.t0) * frac;
      const Point3 p = e.eval(t);
      ParamPoint q;
      if (s.kind == FaceSurface::Kind::BSpline) {
        if (!invertBspline(s, p, q)) { why = "edge point off B-spline surface"; return std::nullopt; }
      } else {
        if (!inverseSurface(s, p, uHint, q)) { why = "no analytic inverse"; return std::nullopt; }
        // Keep the angular u continuous along the edge AND continuous with the prior edge.
        if (haveHint) q.u = unwrapNear(q.u, uHint);
        uHint = q.u;
        haveHint = true;
      }
      uv.push_back(q);
    }
    // Publish this edge's END u (in loop-traversal order) so the NEXT edge continues on
    // the same branch. In traversal order the last sample is uv.back() when not reversed,
    // uv.front() when reversed (the traversal starts at the geometric end).
    if (loopUHint != nullptr && !uv.empty())
      *loopUHint = reversed ? uv.front().u : uv.back().u;
    if (reversed) std::reverse(uv.begin(), uv.end());

    PCurve c;
    c.kind = EdgeCurve::Kind::BSpline;
    c.degree = 1;
    for (const ParamPoint& q : uv) c.poles2d.push_back(Point3{q.u, q.v, 0.0});
    // Clamped degree-1 interpolating polyline: for m poles the knot vector is
    // length m+2 = {0, 0, 1, 2, ..., m-2, m-1, m-1} (first/last doubled).
    const int m = static_cast<int>(c.poles2d.size());
    if (m < 2) { why = "degenerate edge (<2 samples)"; return std::nullopt; }
    c.knots.push_back(0.0);
    for (int i = 0; i < m; ++i) c.knots.push_back(double(i));
    c.knots.back() = double(m - 1);  // clamp the trailing knot
    c.knots.push_back(double(m - 1));
    return c;
  }

  // Coarse nearest-(u,v) inverse for a B-spline surface (grid search + local refine).
  // Returns the nearest (u,v); accepts iff the residual is within a SCALE-RELATIVE
  // band of the surface's own size (so a point genuinely OFF the surface is declined,
  // but the chord of an interpolating-polyline EDGE_CURVE lying between two on-surface
  // samples — which sags off a curved patch — is still accepted as on-surface).
  bool invertBspline(const FaceSurface& s, const Point3& p, ParamPoint& out) const {
    if (s.knotsU.empty() || s.knotsV.empty() || s.poles.empty()) return false;
    // Surface length scale = pole bounding-box diagonal.
    Point3 lo = s.poles[0], hi = s.poles[0];
    for (const Point3& q : s.poles) {
      lo.x = std::min(lo.x, q.x); lo.y = std::min(lo.y, q.y); lo.z = std::min(lo.z, q.z);
      hi.x = std::max(hi.x, q.x); hi.y = std::max(hi.y, q.y); hi.z = std::max(hi.z, q.z);
    }
    const double scale = std::max(math::distance(lo, hi), 1e-12);
    const double u0 = s.knotsU.front(), u1 = s.knotsU.back();
    const double v0 = s.knotsV.front(), v1 = s.knotsV.back();
    const int gu = 32, gv = 32;
    double best = 1e300, bu = u0, bv = v0;
    for (int i = 0; i <= gu; ++i) {
      for (int j = 0; j <= gv; ++j) {
        const double u = u0 + (u1 - u0) * i / gu;
        const double v = v0 + (v1 - v0) * j / gv;
        const double d = math::normSquared(surfaceLocalWorld(s, u, v) - p);
        if (d < best) { best = d; bu = u; bv = v; }
      }
    }
    // Local refine on a shrinking window.
    double hu = (u1 - u0) / gu, hv = (v1 - v0) / gv;
    for (int it = 0; it < 60; ++it) {
      bool improved = false;
      const double cand_u[3] = {bu - hu, bu, bu + hu};
      const double cand_v[3] = {bv - hv, bv, bv + hv};
      for (double u : cand_u) {
        for (double v : cand_v) {
          const double uu = std::clamp(u, u0, u1), vv = std::clamp(v, v0, v1);
          const double d = math::normSquared(surfaceLocalWorld(s, uu, vv) - p);
          if (d < best - 1e-24) { best = d; bu = uu; bv = vv; improved = true; }
        }
      }
      if (!improved) { hu *= 0.5; hv *= 0.5; }
      if (hu < 1e-13 && hv < 1e-13) break;
    }
    out = {bu, bv};
    // Accept the nearest (u,v) unless the residual is a LARGE fraction of the surface
    // size (a genuinely-off-surface point — e.g. an edge that does not lie on S).
    return std::sqrt(best) <= 0.05 * scale;
  }

  static Point3 surfaceLocalWorld(const FaceSurface& s, double u, double v) {
    return surfaceLocal(s, u, v);
  }

  // ── Loop / bound / face ────────────────────────────────────────────────────────
  std::optional<TrimLoop> loop(const FaceSurface& s, long edgeLoopId, std::string& why) const {
    const Record* r = rec(edgeLoopId);
    if (!r || r->isComplex || r->name != "EDGE_LOOP" || r->args.size() < 2) {
      why = "EDGE_LOOP unresolved"; return std::nullopt;
    }
    const Value& edges = r->args[1];
    if (edges.kind != Value::Kind::List) { why = "EDGE_LOOP edge list malformed"; return std::nullopt; }
    TrimLoop tl;
    // Shared u-branch hint threaded through the loop's edges so a seam-crossing loop on
    // a periodic surface stays on ONE continuous unwrapped-u branch (see derivePcurve).
    // NaN on the first edge ⇒ that edge picks its own start branch; every later edge
    // continues from the previous edge's end u.
    double loopUHint = std::numeric_limits<double>::quiet_NaN();
    for (const Value& oe : edges.list) {
      auto seg = segment(s, refOf(oe), why, &loopUHint);
      if (!seg) return std::nullopt;
      tl.push_back(std::move(*seg));
    }
    if (tl.empty()) { why = "empty loop"; return std::nullopt; }
    return tl;
  }

  // ORIENTED_EDGE('',*,*,#EDGE_CURVE,orient).
  std::optional<PcurveSegment> segment(const FaceSurface& s, long orientedEdgeId,
                                       std::string& why, double* loopUHint = nullptr) const {
    const Record* oe = rec(orientedEdgeId);
    if (!oe || oe->isComplex || oe->name != "ORIENTED_EDGE" || oe->args.size() < 5) {
      why = "ORIENTED_EDGE unresolved"; return std::nullopt;
    }
    const bool oeReversed = oe->args[4].kind == Value::Kind::Enum && oe->args[4].text == "F";
    const Record* ec = rec(refOf(oe->args[3]));
    if (!ec || ec->isComplex || ec->name != "EDGE_CURVE" || ec->args.size() < 5) {
      why = "EDGE_CURVE unresolved"; return std::nullopt;
    }
    // EDGE_CURVE('',#v0,#v1,#curve,same_sense).
    auto v0 = vertexPoint(refOf(ec->args[1]));
    auto v1 = vertexPoint(refOf(ec->args[2]));
    auto curve = curve3d(refOf(ec->args[3]), why);
    if (!curve) return std::nullopt;
    const bool sameSense = !(ec->args[4].kind == Value::Kind::Enum && ec->args[4].text == "F");
    // Orient the geometric sense of the edge: EDGE_CURVE.same_sense flips the 3-D
    // curve vs the topological edge; ORIENTED_EDGE.orientation flips the edge in loop.
    Point3 a = v0.value_or(curve->eval(0.0));
    Point3 b = v1.value_or(curve->eval(1.0));
    setEdgeRange(*curve, a, b);
    const bool reversed = oeReversed ^ (!sameSense);
    auto pc = derivePcurve(s, *curve, reversed, why, loopUHint);
    if (!pc) return std::nullopt;
    PcurveSegment seg;
    seg.curve = std::move(*pc);
    seg.first = seg.curve.knots.front();
    seg.last = seg.curve.knots.back();
    seg.reversed = false;  // orientation already baked into the pcurve pole order
    return seg;
  }

  std::optional<TrimmedNurbsFace> mapFace(const Record& r, std::string& why) const {
    if (r.args.size() < 3) { why = "ADVANCED_FACE too few args"; return std::nullopt; }
    const Value& boundsV = r.args[1];
    auto surf = surface(refOf(r.args[2]), why);
    if (!surf) return std::nullopt;
    if (boundsV.kind != Value::Kind::List) { why = "ADVANCED_FACE bounds not a list"; return std::nullopt; }
    TrimmedNurbsFace f;
    f.surface = *surf;
    for (const Value& bref : boundsV.list) {
      const Record* b = rec(refOf(bref));
      if (!b || b->isComplex) { why = "face bound unresolved"; return std::nullopt; }
      const bool isOuter = b->name == "FACE_OUTER_BOUND";
      if (!isOuter && b->name != "FACE_BOUND") { why = "unexpected bound " + b->name; return std::nullopt; }
      if (b->args.size() < 2) { why = "bound too few args"; return std::nullopt; }
      auto tl = loop(*surf, refOf(b->args[1]), why);
      if (!tl) return std::nullopt;
      if (isOuter) f.outer = std::move(*tl);
      else f.holes.push_back(std::move(*tl));
    }
    if (!f.hasOuter() && f.holes.empty()) { why = "face has no bounds"; return std::nullopt; }
    // If no FACE_OUTER_BOUND was present but a single FACE_BOUND was, treat it as outer.
    if (!f.hasOuter() && f.holes.size() == 1) {
      f.outer = std::move(f.holes[0]);
      f.holes.clear();
    }
    return f;
  }
};

}  // namespace

std::vector<TrimmedNurbsFace> readStepBrepExternal(const std::string& step,
                                                   ExternalImportReport* report) {
  Parser p(step);
  if (!p.parseTable()) return {};
  ExternalMapper m(p.table(), report);
  return m.build();
}

}  // namespace cybercad::native::exchange
