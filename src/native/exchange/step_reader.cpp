// SPDX-License-Identifier: Apache-2.0
//
// step_reader.cpp — native ISO-10303-21 (STEP AP203) reader implementation.
//
// See step_reader.h for scope + the honesty contract. This file is the
// deterministic inverse of step_writer.cpp: a Part-21 tokenizer + entity table,
// then a two-pass mapper (leaf geometry → topology) that reconstructs the native
// B-rep the writer serialised, sharing vertex/edge nodes by #id exactly as the
// writer shared them (so the result re-tessellates watertight), and rebuilding the
// analytic PCURVEs the tessellator needs (STEP carries none). Anything outside the
// writer's alphabet returns a NULL Shape → the engine falls back to OCCT.
//
// OCCT-FREE. clang++ -std=c++20.

#include "native/exchange/step_reader.h"

#include "native/math/native_math.h"
#include "native/topology/accessors.h"
#include "native/topology/native_topology.h"

#include <cctype>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace cybercad::native::exchange {

namespace math = cybercad::native::math;

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

// ─────────────────────────────────────────────────────────────────────────────
// Part-21 value model. A record argument is one of the STEP scalar/aggregate
// forms; nested lists recurse. Mirrors the writer's emitted forms exactly.
// ─────────────────────────────────────────────────────────────────────────────
struct Arg;
using ArgList = std::vector<Arg>;

struct Arg {
  enum class Kind { Ref, Int, Real, Str, Enum, List, Null, Derived } kind = Kind::Null;
  long ref = 0;           ///< Ref: #id ; Int: value
  double real = 0.0;      ///< Real value
  std::string text;       ///< Str (un-doubled) / Enum (without dots)
  ArgList list;           ///< List members

  bool isRef() const noexcept { return kind == Kind::Ref; }
  bool isInt() const noexcept { return kind == Kind::Int; }
  bool isReal() const noexcept { return kind == Kind::Real; }
  bool isNumber() const noexcept { return kind == Kind::Int || kind == Kind::Real; }
  bool isEnum() const noexcept { return kind == Kind::Enum; }
  bool isList() const noexcept { return kind == Kind::List; }

  double asReal() const noexcept { return kind == Kind::Int ? static_cast<double>(ref) : real; }
  long asInt() const noexcept {
    return kind == Kind::Int ? ref : static_cast<long>(std::llround(real));
  }
};

// One combined sub-record inside a `( SUB(...) SUB(...) )` complex instance.
struct SubRecord {
  std::string keyword;
  ArgList args;
};

// A DATA-section entity instance. A simple record has one keyword + args; a
// combined instance carries its sub-records (keyword empty).
struct Record {
  std::string keyword;               ///< empty ⇒ combined instance (see subs)
  ArgList args;
  std::vector<SubRecord> subs;        ///< non-empty ⇒ combined instance
  bool combined = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Tokenizer / parser. A hand-rolled recursive-descent scanner over the DATA
// section text. On any malformed input it sets ok_=false and callers bail
// (mapper returns NULL → OCCT). No exceptions.
// ─────────────────────────────────────────────────────────────────────────────
class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s) {}

  // Parse the whole file into `records` keyed by #id. Returns false on any
  // structural failure (no DATA section, malformed record, unbalanced parens).
  bool parse(std::unordered_map<int, Record>& records) {
    const std::size_t dataPos = s_.find("DATA;");
    if (dataPos == std::string::npos) return false;
    i_ = dataPos + 5;  // just past "DATA;"

    while (ok_) {
      skipWs();
      if (i_ >= s_.size()) break;
      if (startsWith("ENDSEC;")) break;  // end of DATA
      if (s_[i_] != '#') return false;   // every DATA line begins "#id ="
      ++i_;
      const long id = readInt();
      if (!ok_ || id <= 0) return false;
      skipWs();
      if (!expect('=')) return false;
      skipWs();
      Record rec = readRecord();
      if (!ok_) return false;
      skipWs();
      if (!expect(';')) return false;
      records[static_cast<int>(id)] = std::move(rec);
    }
    return ok_;
  }

 private:
  // Read one entity instance: either NAME( args ) or a combined ( SUB(..) .. ).
  Record readRecord() {
    Record rec;
    skipWs();
    if (i_ < s_.size() && s_[i_] == '(') {
      // Combined instance: ( KEYWORD(args) KEYWORD(args) ... ).
      rec.combined = true;
      ++i_;  // consume '('
      while (ok_) {
        skipWs();
        if (i_ >= s_.size()) { ok_ = false; break; }
        if (s_[i_] == ')') { ++i_; break; }
        SubRecord sub;
        sub.keyword = readKeyword();
        skipWs();
        if (!expect('(')) break;
        sub.args = readArgList();
        if (!expect(')')) break;
        rec.subs.push_back(std::move(sub));
      }
      return rec;
    }
    rec.keyword = readKeyword();
    if (rec.keyword.empty()) { ok_ = false; return rec; }
    skipWs();
    if (!expect('(')) return rec;
    rec.args = readArgList();
    expect(')');
    return rec;
  }

  // Read a comma-separated argument list (caller has consumed the '('; this stops
  // at the matching ')' without consuming it).
  ArgList readArgList() {
    ArgList out;
    skipWs();
    if (i_ < s_.size() && s_[i_] == ')') return out;  // empty list
    while (ok_) {
      out.push_back(readArg());
      skipWs();
      if (i_ >= s_.size()) { ok_ = false; break; }
      if (s_[i_] == ',') { ++i_; continue; }
      break;  // ')' — leave for the caller
    }
    return out;
  }

  // Read one argument, dispatching on the leading character (the single documented
  // systems-band function — one arm per Part-21 value form).
  Arg readArg() {
    skipWs();
    Arg a;
    if (i_ >= s_.size()) { ok_ = false; return a; }
    const char c = s_[i_];
    if (c == '#') {
      ++i_;
      a.kind = Arg::Kind::Ref;
      a.ref = readInt();
    } else if (c == '\'') {
      a.kind = Arg::Kind::Str;
      a.text = readString();
    } else if (c == '.') {
      a.kind = Arg::Kind::Enum;
      a.text = readEnum();
    } else if (c == '(') {
      ++i_;
      a.kind = Arg::Kind::List;
      a.list = readArgList();
      expect(')');
    } else if (c == '$') {
      ++i_;
      a.kind = Arg::Kind::Null;
    } else if (c == '*') {
      ++i_;
      a.kind = Arg::Kind::Derived;
    } else if (c == '-' || c == '+' || c == '.' || std::isdigit(static_cast<unsigned char>(c))) {
      readNumber(a);
    } else {
      // A bare keyword argument (a typed value like LENGTH_MEASURE(1.E-07)): read
      // the keyword and its parenthesised body, keeping the inner value as this
      // arg (so UNCERTAINTY_MEASURE_WITH_UNIT's LENGTH_MEASURE(1.E-07) is read).
      const std::string kw = readKeyword();
      if (kw.empty()) { ok_ = false; return a; }
      skipWs();
      if (i_ < s_.size() && s_[i_] == '(') {
        ++i_;
        ArgList inner = readArgList();
        expect(')');
        // Represent the typed value by its single inner argument when unary
        // (LENGTH_MEASURE(x) → x); otherwise a list.
        if (inner.size() == 1) return inner.front();
        a.kind = Arg::Kind::List;
        a.list = std::move(inner);
      } else {
        a.kind = Arg::Kind::Enum;  // a bare keyword token
        a.text = kw;
      }
    }
    return a;
  }

  // ── Lexical helpers ─────────────────────────────────────────────────────────
  void skipWs() {
    while (i_ < s_.size()) {
      const char c = s_[i_];
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') { ++i_; continue; }
      // Part-21 comments: /* ... */
      if (c == '/' && i_ + 1 < s_.size() && s_[i_ + 1] == '*') {
        i_ += 2;
        while (i_ + 1 < s_.size() && !(s_[i_] == '*' && s_[i_ + 1] == '/')) ++i_;
        i_ = std::min(i_ + 2, s_.size());
        continue;
      }
      break;
    }
  }

  bool expect(char c) {
    if (i_ < s_.size() && s_[i_] == c) { ++i_; return true; }
    ok_ = false;
    return false;
  }

  bool startsWith(const char* kw) const {
    return s_.compare(i_, std::char_traits<char>::length(kw), kw) == 0;
  }

  long readInt() {
    skipWs();
    const std::size_t start = i_;
    if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) ++i_;
    std::size_t digits = 0;
    while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) { ++i_; ++digits; }
    if (digits == 0) { ok_ = false; return 0; }
    return std::strtol(s_.c_str() + start, nullptr, 10);
  }

  // A STEP number: INTEGER (no '.'/'E') vs REAL (has '.' or exponent).
  void readNumber(Arg& a) {
    const std::size_t start = i_;
    bool isReal = false;
    if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) ++i_;
    while (i_ < s_.size()) {
      const char c = s_[i_];
      if (std::isdigit(static_cast<unsigned char>(c))) { ++i_; continue; }
      if (c == '.') { isReal = true; ++i_; continue; }
      if (c == 'e' || c == 'E') {
        isReal = true;
        ++i_;
        if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) ++i_;
        continue;
      }
      break;
    }
    const std::string tok = s_.substr(start, i_ - start);
    if (tok.empty() || tok == "-" || tok == "+") { ok_ = false; return; }
    if (isReal) {
      a.kind = Arg::Kind::Real;
      a.real = std::strtod(tok.c_str(), nullptr);
    } else {
      a.kind = Arg::Kind::Int;
      a.ref = std::strtol(tok.c_str(), nullptr, 10);
    }
  }

  // A single-quoted string; embedded '' is un-doubled to a single quote.
  std::string readString() {
    std::string out;
    ++i_;  // opening quote
    while (i_ < s_.size()) {
      const char c = s_[i_];
      if (c == '\'') {
        if (i_ + 1 < s_.size() && s_[i_ + 1] == '\'') { out += '\''; i_ += 2; continue; }
        ++i_;  // closing quote
        return out;
      }
      out += c;
      ++i_;
    }
    ok_ = false;  // unterminated string
    return out;
  }

  // An enum .NAME. → returns NAME (without the dots).
  std::string readEnum() {
    ++i_;  // opening dot
    std::string out;
    while (i_ < s_.size() && s_[i_] != '.') { out += s_[i_]; ++i_; }
    if (i_ >= s_.size()) { ok_ = false; return out; }
    ++i_;  // closing dot
    return out;
  }

  // An entity keyword: [A-Za-z_][A-Za-z0-9_]*.
  std::string readKeyword() {
    skipWs();
    std::string out;
    while (i_ < s_.size()) {
      const char c = s_[i_];
      if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') { out += c; ++i_; }
      else break;
    }
    return out;
  }

  const std::string& s_;
  std::size_t i_ = 0;
  bool ok_ = true;
};

// ─────────────────────────────────────────────────────────────────────────────
// Mapper. Two passes over the record table, memoized by #id, mirroring the
// writer. Any out-of-scope construct sets fail_ and the whole import declines.
// ─────────────────────────────────────────────────────────────────────────────
class Mapper {
 public:
  explicit Mapper(std::unordered_map<int, Record>& records) : recs_(records) {}

  // Build the single native Solid, or a NULL Shape on any decline.
  topo::Shape build() {
    if (!validateUnitContext()) return {};

    const int brepId = findSingleManifoldBrep();
    if (brepId == 0 || fail_) return {};

    const topo::Shape solid = mapManifoldBrep(brepId);
    if (fail_ || solid.isNull()) return {};
    return solid;
  }

 private:
  std::unordered_map<int, Record>& recs_;
  bool fail_ = false;

  // Memoization caches keyed by #id (writer-shared dedup inverse).
  std::unordered_map<int, math::Point3> pointCache_;
  std::unordered_map<int, math::Dir3> dirCache_;
  std::unordered_map<int, math::Ax3> ax3Cache_;
  std::unordered_map<int, topo::Shape> vertexCache_;
  std::unordered_map<int, topo::Shape> edgeCache_;   // EDGE_CURVE #id → Forward edge

  const Record* rec(long id) {
    auto it = recs_.find(static_cast<int>(id));
    return it == recs_.end() ? nullptr : &it->second;
  }
  const Record* recOfKind(long id, const char* kw) {
    const Record* r = rec(id);
    if (!r || r->combined || r->keyword != kw) return nullptr;
    return r;
  }

  void decline() { fail_ = true; }

  // ── Unit-context gate: require SI millimetre + radian, decline otherwise ─────
  bool validateUnitContext() {
    bool sawMilliMetre = false;
    for (const auto& [id, r] : recs_) {
      if (!r.combined) continue;
      for (const SubRecord& sub : r.subs) {
        if (sub.keyword != "SI_UNIT") continue;
        // SI_UNIT(prefix, name): length must be (.MILLI., .METRE.); any other
        // length prefix/name means non-mm → decline.
        if (sub.args.size() == 2 && sub.args[1].isEnum() && sub.args[1].text == "METRE") {
          if (!(sub.args[0].isEnum() && sub.args[0].text == "MILLI")) return false;
          sawMilliMetre = true;
        }
      }
    }
    // A native/STEPControl AP203 solid always carries the mm length unit; its
    // absence means we cannot assert true-mm geometry → decline.
    return sawMilliMetre;
  }

  // Exactly one MANIFOLD_SOLID_BREP (single root, single shell). More than one
  // (assembly / void shells) declines.
  int findSingleManifoldBrep() {
    int found = 0;
    for (const auto& [id, r] : recs_) {
      if (r.combined || r.keyword != "MANIFOLD_SOLID_BREP") continue;
      if (found != 0) { decline(); return 0; }  // >1 root → assembly
      found = id;
    }
    return found;
  }

  // ── Leaf geometry (Pass A), memoized by #id ─────────────────────────────────
  std::optional<math::Point3> point(long id) {
    if (auto it = pointCache_.find(static_cast<int>(id)); it != pointCache_.end()) return it->second;
    const Record* r = recOfKind(id, "CARTESIAN_POINT");
    if (!r || r->args.size() != 2 || !r->args[1].isList() || r->args[1].list.size() < 3)
      return std::nullopt;
    const ArgList& c = r->args[1].list;
    const math::Point3 p{c[0].asReal(), c[1].asReal(), c[2].asReal()};
    pointCache_.emplace(static_cast<int>(id), p);
    return p;
  }

  std::optional<math::Dir3> direction(long id) {
    if (auto it = dirCache_.find(static_cast<int>(id)); it != dirCache_.end()) return it->second;
    const Record* r = recOfKind(id, "DIRECTION");
    if (!r || r->args.size() != 2 || !r->args[1].isList() || r->args[1].list.size() < 3)
      return std::nullopt;
    const ArgList& c = r->args[1].list;
    const math::Dir3 d{c[0].asReal(), c[1].asReal(), c[2].asReal()};
    dirCache_.emplace(static_cast<int>(id), d);
    return d;
  }

  // AXIS2_PLACEMENT_3D('',#loc,#axis(Z),#ref(X)) → Ax3 (Y = Z × X, orthonormal).
  std::optional<math::Ax3> axis2placement(long id) {
    if (auto it = ax3Cache_.find(static_cast<int>(id)); it != ax3Cache_.end()) return it->second;
    const Record* r = recOfKind(id, "AXIS2_PLACEMENT_3D");
    if (!r || r->args.size() != 4 || !r->args[1].isRef()) return std::nullopt;
    const auto o = point(r->args[1].ref);
    if (!o) return std::nullopt;
    // axis (Z) and ref_direction (X) are OPTIONAL in the schema ($), but the
    // writer always emits both; default to +Z / +X if absent.
    math::Dir3 z{0, 0, 1};
    math::Dir3 x{1, 0, 0};
    if (r->args[2].isRef()) { auto d = direction(r->args[2].ref); if (d) z = *d; }
    if (r->args[3].isRef()) { auto d = direction(r->args[3].ref); if (d) x = *d; }
    const math::Ax3 f = math::Ax3::fromAxisAndRef(*o, z, x);
    ax3Cache_.emplace(static_cast<int>(id), f);
    return f;
  }

  // The direction of a VECTOR('',#dir,mag) (magnitude ignored — used only for a
  // LINE's direction).
  std::optional<math::Dir3> vectorDir(long id) {
    const Record* r = recOfKind(id, "VECTOR");
    if (!r || r->args.size() != 3 || !r->args[1].isRef()) return std::nullopt;
    return direction(r->args[1].ref);
  }

  // RLE-expand (values, mults) → flat knot vector (inverts compressKnots).
  static std::vector<double> expandKnots(const ArgList& values, const ArgList& mults) {
    std::vector<double> flat;
    if (values.size() != mults.size()) return flat;
    for (std::size_t k = 0; k < values.size(); ++k) {
      const long m = mults[k].asInt();
      for (long j = 0; j < m; ++j) flat.push_back(values[k].asReal());
    }
    return flat;
  }

  // ── EdgeCurve (LINE / CIRCLE / B_SPLINE_CURVE_WITH_KNOTS) by #id ─────────────
  std::optional<topo::EdgeCurve> curve(long id) {
    const Record* r = rec(id);
    if (!r || r->combined) { return std::nullopt; }
    topo::EdgeCurve c;
    if (r->keyword == "LINE") {
      // LINE('',#point,#vector): frame origin = point, X = vector direction.
      if (r->args.size() != 3 || !r->args[1].isRef() || !r->args[2].isRef()) return std::nullopt;
      const auto o = point(r->args[1].ref);
      const auto d = vectorDir(r->args[2].ref);
      if (!o || !d) return std::nullopt;
      c.kind = topo::EdgeCurve::Kind::Line;
      c.frame.origin = *o;
      c.frame.x = *d;
      c.frame.z = math::Dir3{0, 0, 1};
      return c;
    }
    if (r->keyword == "CIRCLE") {
      // CIRCLE('',#axis2placement,radius).
      if (r->args.size() != 3 || !r->args[1].isRef() || !r->args[2].isNumber()) return std::nullopt;
      const auto f = axis2placement(r->args[1].ref);
      if (!f) return std::nullopt;
      c.kind = topo::EdgeCurve::Kind::Circle;
      c.frame = *f;
      c.radius = r->args[2].asReal();
      return c;
    }
    if (r->keyword == "B_SPLINE_CURVE_WITH_KNOTS") return bsplineCurve(*r);
    // Any other curve keyword (ELLIPSE, TRIMMED_CURVE, RATIONAL_B_SPLINE_*, …) is
    // out of scope.
    return std::nullopt;
  }

  // B_SPLINE_CURVE_WITH_KNOTS('',deg,(poles),form,closed,self_int,(mults),(knots),form)
  std::optional<topo::EdgeCurve> bsplineCurve(const Record& r) {
    if (r.args.size() < 8) return std::nullopt;
    if (!r.args[1].isInt() || !r.args[2].isList() || !r.args[6].isList() || !r.args[7].isList())
      return std::nullopt;
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::BSpline;
    c.degree = static_cast<int>(r.args[1].asInt());
    for (const Arg& p : r.args[2].list) {
      if (!p.isRef()) return std::nullopt;
      const auto pt = point(p.ref);
      if (!pt) return std::nullopt;
      c.poles.push_back(*pt);
    }
    c.knots = expandKnots(r.args[7].list, r.args[6].list);
    // Non-rational only: knot vector length must be nPoles + degree + 1.
    if (c.knots.size() != c.poles.size() + static_cast<std::size_t>(c.degree) + 1)
      return std::nullopt;
    return c;
  }

  // ── FaceSurface by #id ───────────────────────────────────────────────────────
  // Thin dispatcher on the surface keyword; each arm is a small builder. PLANE and
  // the quadrics share the ('',#placement[,radius[,semiAngle]]) shape.
  std::optional<topo::FaceSurface> surface(long id) {
    const Record* r = rec(id);
    if (!r || r->combined) return std::nullopt;
    using K = topo::FaceSurface::Kind;
    if (r->keyword == "PLANE") return placedSurface(*r, K::Plane, /*nRadii=*/0);
    if (r->keyword == "CYLINDRICAL_SURFACE") return placedSurface(*r, K::Cylinder, 1);
    if (r->keyword == "SPHERICAL_SURFACE") return placedSurface(*r, K::Sphere, 1);
    if (r->keyword == "CONICAL_SURFACE") return placedSurface(*r, K::Cone, 2);
    if (r->keyword == "B_SPLINE_SURFACE_WITH_KNOTS") return bsplineSurface(*r);
    return std::nullopt;  // any other surface keyword is out of scope
  }

  // PLANE/CYLINDRICAL/SPHERICAL/CONICAL_SURFACE: an AXIS2_PLACEMENT_3D plus `nRadii`
  // trailing reals (0 = plane; 1 = radius for cylinder/sphere; 2 = radius + semiAngle
  // for cone). Validates the exact arg shape and builds the FaceSurface.
  std::optional<topo::FaceSurface> placedSurface(const Record& r, topo::FaceSurface::Kind kind,
                                                 int nRadii) {
    if (r.args.size() != static_cast<std::size_t>(2 + nRadii) || !r.args[1].isRef())
      return std::nullopt;
    for (int i = 0; i < nRadii; ++i)
      if (!r.args[2 + i].isNumber()) return std::nullopt;
    const auto f = axis2placement(r.args[1].ref);
    if (!f) return std::nullopt;
    topo::FaceSurface s;
    s.kind = kind;
    s.frame = *f;
    if (nRadii >= 1) s.radius = r.args[2].asReal();
    if (nRadii >= 2) s.semiAngle = r.args[3].asReal();
    return s;
  }

  // B_SPLINE_SURFACE_WITH_KNOTS('',degU,degV,((poles)..),form,uClosed,vClosed,
  //   self_int,(uMults),(vMults),(uKnots),(vKnots),form). Poles row-major (U outer).
  std::optional<topo::FaceSurface> bsplineSurface(const Record& r) {
    if (r.args.size() < 12) return std::nullopt;
    if (!r.args[1].isInt() || !r.args[2].isInt() || !r.args[3].isList()) return std::nullopt;
    if (!r.args[8].isList() || !r.args[9].isList() || !r.args[10].isList() || !r.args[11].isList())
      return std::nullopt;
    topo::FaceSurface s;
    s.kind = topo::FaceSurface::Kind::BSpline;
    s.degreeU = static_cast<int>(r.args[1].asInt());
    s.degreeV = static_cast<int>(r.args[2].asInt());
    const ArgList& rows = r.args[3].list;  // U outer
    s.nPolesU = static_cast<int>(rows.size());
    int vCount = -1;
    for (const Arg& row : rows) {
      if (!row.isList()) return std::nullopt;
      if (vCount < 0) vCount = static_cast<int>(row.list.size());
      else if (static_cast<int>(row.list.size()) != vCount) return std::nullopt;  // ragged
      for (const Arg& p : row.list) {
        if (!p.isRef()) return std::nullopt;
        const auto pt = point(p.ref);
        if (!pt) return std::nullopt;
        s.poles.push_back(*pt);  // row-major (U outer, V inner)
      }
    }
    s.nPolesV = vCount < 0 ? 0 : vCount;
    s.knotsU = expandKnots(r.args[10].list, r.args[8].list);
    s.knotsV = expandKnots(r.args[11].list, r.args[9].list);
    if (s.nPolesU <= 0 || s.nPolesV <= 0) return std::nullopt;
    if (s.knotsU.size() != static_cast<std::size_t>(s.nPolesU + s.degreeU + 1)) return std::nullopt;
    if (s.knotsV.size() != static_cast<std::size_t>(s.nPolesV + s.degreeV + 1)) return std::nullopt;
    return s;
  }

  // ── Topology (Pass B) ───────────────────────────────────────────────────────
  topo::Shape vertex(long id) {
    if (auto it = vertexCache_.find(static_cast<int>(id)); it != vertexCache_.end()) return it->second;
    const Record* r = recOfKind(id, "VERTEX_POINT");
    if (!r || r->args.size() != 2 || !r->args[1].isRef()) { decline(); return {}; }
    const auto p = point(r->args[1].ref);
    if (!p) { decline(); return {}; }
    const topo::Shape v = topo::ShapeBuilder::makeVertex(*p);
    vertexCache_.emplace(static_cast<int>(id), v);
    return v;
  }

  // EDGE_CURVE('',#v0,#v1,#curve,same_sense) → a Forward edge sharing one node per
  // #id (the writer shares an EDGE_CURVE across the two adjacent faces).
  topo::Shape edgeCurve(long id) {
    if (auto it = edgeCache_.find(static_cast<int>(id)); it != edgeCache_.end()) return it->second;
    const Record* r = recOfKind(id, "EDGE_CURVE");
    if (!r || r->args.size() != 5 || !r->args[1].isRef() || !r->args[2].isRef() ||
        !r->args[3].isRef()) {
      decline();
      return {};
    }
    const topo::Shape v0 = vertex(r->args[1].ref);
    const topo::Shape v1 = vertex(r->args[2].ref);
    if (fail_ || v0.isNull() || v1.isNull()) { decline(); return {}; }
    const auto cv = curve(r->args[3].ref);
    if (!cv) { decline(); return {}; }

    const auto p0 = topo::pointOf(v0);
    const auto p1 = topo::pointOf(v1);
    if (!p0 || !p1) { decline(); return {}; }
    const auto range = curveRange(*cv, *p0, *p1);
    topo::EdgeCurve cc = *cv;
    if (cc.kind == topo::EdgeCurve::Kind::Line) {
      // Recompute the Line direction from the actual endpoints so C(first)=p0,
      // C(last)=p1 exactly (the stored VECTOR direction agrees, but this is exact
      // and independent of any magnitude convention).
      const math::Vec3 d = *p1 - *p0;
      cc.frame.origin = *p0;
      if (math::norm(d) > 1e-12) cc.frame.x = math::Dir3{d};
    }
    // Store v0 Forward, v1 Reversed at [first,last] (makeEdge convention). A closed
    // edge (v0==v1) keeps both (a rim circle) — makeEdgeWithVertices tolerates it.
    std::vector<topo::Shape> verts = {v0.oriented(topo::Orientation::Forward),
                                      v1.oriented(topo::Orientation::Reversed)};
    const topo::Shape edge =
        topo::ShapeBuilder::makeEdgeWithVertices(cc, range.first, range.second, std::move(verts));
    edgeCache_.emplace(static_cast<int>(id), edge);
    return edge;
  }

  // The parameter range [first,last] such that C(first)=p0 and C(last)=p1 for the
  // curve kind, matching the tessellator's edgeCurveLocal parametrization.
  std::pair<double, double> curveRange(const topo::EdgeCurve& c, const math::Point3& p0,
                                       const math::Point3& p1) {
    using K = topo::EdgeCurve::Kind;
    switch (c.kind) {
      case K::Line: {
        return {0.0, std::max(math::distance(p0, p1), 1e-12)};
      }
      case K::Circle: {
        const double a0 = circleAngle(c, p0);
        double a1 = circleAngle(c, p1);
        // Closed rim (v0==v1): a full turn.
        if (math::distance(p0, p1) <= 1e-9) return {a0, a0 + kTwoPi};
        // Keep a1 ahead of a0 (writer arcs sweep forward/CCW in the circle frame).
        while (a1 <= a0 + 1e-12) a1 += kTwoPi;
        return {a0, a1};
      }
      case K::BSpline:
      default:
        // The B-spline curve is parametrized over its knot span; the writer trims
        // exactly to [knots.front(), knots.back()].
        return {c.knots.empty() ? 0.0 : c.knots.front(),
                c.knots.empty() ? 1.0 : c.knots.back()};
    }
  }

  // Angle of a point on a circle's frame: atan2(dot(p-O,Y), dot(p-O,X)).
  static double circleAngle(const topo::EdgeCurve& c, const math::Point3& p) {
    const math::Vec3 d = p - c.frame.origin;
    const double x = math::dot(d, c.frame.x.vec());
    const double y = math::dot(d, c.frame.y.vec());
    return std::atan2(y, x);
  }

  // One entry of an EDGE_LOOP as its underlying EDGE_CURVE and endpoint VERTEX_POINT
  // ids (the writer's ORIENTED_EDGE sense flag is NOT reliable for head-to-tail
  // traversal — it emits per-face Forward edges that share one EDGE_CURVE, so the
  // sense does not encode the loop's directed walk; we reorder/reorient by vertex
  // connectivity, exactly as a STEP reader must).
  struct LoopEntry {
    long ecId = 0;
    long v0 = 0;  // EDGE_CURVE's stored start vertex #id
    long v1 = 0;  // ... end vertex #id
  };

  // The EDGE_CURVE's stored start vertex #id (args[1]) — the edge node's Forward
  // start, so an entry whose walk-tail equals it is traversed Forward.
  long originalV0(long ecId) {
    const Record* ec = recOfKind(ecId, "EDGE_CURVE");
    return (ec && ec->args.size() == 5 && ec->args[1].isRef()) ? ec->args[1].ref : 0;
  }

  // Chain the loop's edges head-to-tail by matching endpoint vertex #ids, ignoring
  // the (unreliable) ORIENTED_EDGE sense. Each returned entry is stored so v0 is the
  // walk TAIL and v1 the walk HEAD (endpoints swapped when the edge is traversed
  // against its EDGE_CURVE storage order). A closed rim edge (v0==v1) is a valid
  // single-entry loop. Returns empty on a broken (non-connected) loop → decline.
  std::vector<LoopEntry> chainLoop(const std::vector<LoopEntry>& in) {
    // A single closed edge (rim circle): its own start==end vertex.
    if (in.size() == 1) return in;

    // A full-turn periodic wall (after seam drop) is a wire of CLOSED rim circles
    // (each v0==v1), which do not chain head-to-tail — each is its own closed loop.
    // Keep them in order (the tessellator meshes the periodic surface directly).
    bool allClosed = true;
    for (const LoopEntry& e : in)
      if (e.v0 != e.v1) { allClosed = false; break; }
    if (allClosed) return in;

    std::vector<LoopEntry> remaining = in;
    std::vector<LoopEntry> out;
    // Start from the first edge in its stored orientation.
    out.push_back(remaining.front());
    remaining.erase(remaining.begin());
    long head = out.front().v1;

    while (!remaining.empty()) {
      bool matched = false;
      for (std::size_t i = 0; i < remaining.size(); ++i) {
        LoopEntry e = remaining[i];
        if (e.v0 == head) {
          out.push_back(e);
          head = e.v1;
          remaining.erase(remaining.begin() + static_cast<long>(i));
          matched = true;
          break;
        }
        if (e.v1 == head) {  // traverse this edge reversed → swap endpoints
          std::swap(e.v0, e.v1);
          out.push_back(e);
          head = e.v1;
          remaining.erase(remaining.begin() + static_cast<long>(i));
          matched = true;
          break;
        }
      }
      if (!matched) return {};  // loop does not close → broken wire
    }
    // The last head must return to the first tail for a closed loop.
    if (out.back().v1 != out.front().v0) return {};
    return out;
  }

  // Reconstruct the ORDERED, connected wire edges of an EDGE_LOOP. The writer's
  // periodic-wall SEAM (an EDGE_CURVE referenced forward AND reversed in the SAME
  // loop) is dropped so the wire matches the native rim loop; the periodic analytic
  // face still meshes closed via the structured-grid path.
  std::vector<topo::Shape> edgeLoop(long id) {
    const Record* r = recOfKind(id, "EDGE_LOOP");
    if (!r || r->args.size() != 2 || !r->args[1].isList()) { decline(); return {}; }

    // Gather the loop's oriented-edge → EDGE_CURVE endpoints, and count each
    // EDGE_CURVE's forward/reversed uses to spot the seam.
    std::unordered_map<long, int> senseMask;  // ecId → bit0=fwd, bit1=rev
    std::vector<LoopEntry> raw;
    for (const Arg& oe : r->args[1].list) {
      if (!oe.isRef()) { decline(); return {}; }
      const Record* o = recOfKind(oe.ref, "ORIENTED_EDGE");
      if (!o || o->args.size() != 5 || !o->args[3].isRef() || !o->args[4].isEnum()) {
        decline();
        return {};
      }
      const long ecId = o->args[3].ref;
      const Record* ec = recOfKind(ecId, "EDGE_CURVE");
      if (!ec || ec->args.size() != 5 || !ec->args[1].isRef() || !ec->args[2].isRef()) {
        decline();
        return {};
      }
      senseMask[ecId] |= (o->args[4].text == "T") ? 1 : 2;
      raw.push_back(LoopEntry{ecId, ec->args[1].ref, ec->args[2].ref});
    }

    std::vector<LoopEntry> entries;
    for (const LoopEntry& e : raw)
      if (senseMask[e.ecId] != 3) entries.push_back(e);  // drop the seam pair
    if (entries.empty()) { decline(); return {}; }

    const std::vector<LoopEntry> chained = chainLoop(entries);
    if (chained.empty()) { decline(); return {}; }

    // Build each shared edge node and orient it so the wire walks head-to-tail.
    std::vector<topo::Shape> edges;
    for (const LoopEntry& e : chained) {
      const topo::Shape ec = edgeCurve(e.ecId);
      if (fail_ || ec.isNull()) { decline(); return {}; }
      // The edge node stores v0(#args[1]) Forward, v1(#args[2]) Reversed. Traversing
      // v0→v1 is Forward; the chainer stores each entry already in walk order
      // (v0=tail, v1=head) with a `flip` implied when it swapped the endpoints.
      const bool forward = (e.v0 == originalV0(e.ecId));
      edges.push_back(ec.oriented(forward ? topo::Orientation::Forward
                                          : topo::Orientation::Reversed));
    }
    return edges;
  }

  // FACE_OUTER_BOUND / FACE_BOUND('',#edgeLoop,orientation) → a wire of edges.
  topo::Shape faceBound(long id) {
    const Record* r = rec(id);
    if (!r || r->combined ||
        (r->keyword != "FACE_OUTER_BOUND" && r->keyword != "FACE_BOUND") || r->args.size() != 3 ||
        !r->args[1].isRef()) {
      decline();
      return {};
    }
    std::vector<topo::Shape> edges = edgeLoop(r->args[1].ref);
    if (fail_ || edges.empty()) { decline(); return {}; }
    return topo::ShapeBuilder::makeWire(std::move(edges));
  }

  // ADVANCED_FACE('',(#bound..),#surface,same_sense) → a native Face with PCURVEs
  // reconstructed per surface kind (STEP carries no pcurve; the tessellator needs
  // them to trim the boundary).
  topo::Shape advancedFace(long id) {
    const Record* r = recOfKind(id, "ADVANCED_FACE");
    if (!r || r->args.size() != 4 || !r->args[1].isList() || !r->args[2].isRef() ||
        !r->args[3].isEnum()) {
      decline();
      return {};
    }
    const auto srf = surface(r->args[2].ref);
    if (!srf) { decline(); return {}; }

    // FACE_OUTER_BOUND first (writer emits it at index 0), then FACE_BOUND holes.
    std::vector<topo::Shape> wires;
    int outerIdx = -1;
    for (std::size_t i = 0; i < r->args[1].list.size(); ++i) {
      const Arg& b = r->args[1].list[i];
      if (!b.isRef()) { decline(); return {}; }
      const Record* br = rec(b.ref);
      if (!br) { decline(); return {}; }
      const topo::Shape wire = faceBound(b.ref);
      if (fail_ || wire.isNull()) { decline(); return {}; }
      if (br->keyword == "FACE_OUTER_BOUND" && outerIdx < 0) outerIdx = static_cast<int>(i);
      wires.push_back(wire);
    }
    if (wires.empty()) { decline(); return {}; }
    if (outerIdx < 0) outerIdx = 0;

    // Build the face node first (surface + wires) so pcurves can key on it, then
    // re-attach the analytic pcurve per edge on this face's surface.
    const topo::Orientation orient =
        r->args[3].text == "T" ? topo::Orientation::Forward : topo::Orientation::Reversed;

    std::vector<topo::Shape> holes;
    for (std::size_t i = 0; i < wires.size(); ++i)
      if (static_cast<int>(i) != outerIdx) holes.push_back(wires[i]);

    return buildFaceWithPCurves(*srf, wires[static_cast<std::size_t>(outerIdx)], holes, orient);
  }

  // Assemble a face and attach a reconstructed analytic pcurve to each edge on the
  // face's surface (matching the native construct helpers so the tessellator trims
  // and welds exactly). The pcurve is keyed to the FINAL face node.
  topo::Shape buildFaceWithPCurves(const topo::FaceSurface& srf, const topo::Shape& outer,
                                   const std::vector<topo::Shape>& holes,
                                   topo::Orientation orient) {
    const topo::Shape face = topo::ShapeBuilder::makeFace(srf, outer, holes, orient);

    // On an angular surface (cylinder/cone/sphere) the surface u wraps at ±π. A
    // face's CIRCLE arcs carry the correct UNWRAPPED angular span (curveRange keeps
    // t continuous over [first,last]); the profile LINE edges' u must be unwrapped
    // onto the SAME branch or the face's UV rectangle tears at the wrap. Use the
    // midpoint of the arcs' u-range as the reference the line u's are unwrapped to.
    const double uRef = angularURef(srf, outer);

    // Rebuild each wire's edges with a pcurve attached, keyed to `face`'s node.
    auto rebuildWire = [&](const topo::Shape& wire) -> topo::Shape {
      std::vector<topo::Shape> edges;
      for (const topo::Shape& e : wire.tshape()->children()) {
        const topo::PCurve pc = pcurveFor(srf, e, uRef);
        const topo::Shape withPc = topo::ShapeBuilder::addPCurve(e, face.tshape(), pc);
        edges.push_back(withPc.oriented(e.orientation()));
      }
      return topo::ShapeBuilder::makeWire(std::move(edges));
    };
    const topo::Shape outer2 = rebuildWire(outer);
    std::vector<topo::Shape> holes2;
    holes2.reserve(holes.size());
    for (const topo::Shape& h : holes) holes2.push_back(rebuildWire(h));
    return topo::ShapeBuilder::makeFace(srf, outer2, holes2, orient);
  }

  // A reference angular-u for a face on an angular surface: the midpoint of its
  // CIRCLE arcs' (unwrapped) u-ranges. Line profile u's are unwrapped toward it so
  // the whole face's UV lands on one continuous 2π branch. Returns 0 when there are
  // no arcs (a full seam-carrying wall has none after seam drop → its lines don't
  // need unwrapping across a shared branch) or for a plane.
  double angularURef(const topo::FaceSurface& srf, const topo::Shape& outer) {
    using K = topo::FaceSurface::Kind;
    if (srf.kind != K::Cylinder && srf.kind != K::Cone && srf.kind != K::Sphere) return 0.0;
    double sum = 0.0;
    int n = 0;
    for (const topo::Shape& e : outer.tshape()->children()) {
      const auto cr = topo::curveOf(e);
      const auto rr = topo::rangeOf(e);
      if (cr && cr->curve && cr->curve->kind == topo::EdgeCurve::Kind::Circle && rr) {
        sum += 0.5 * (rr->first + rr->last);
        ++n;
      }
    }
    return n > 0 ? sum / n : 0.0;
  }

  // Shift `u` by whole turns to the branch nearest `ref` (both radians).
  static double unwrapTo(double u, double ref) {
    while (u - ref > kTwoPi * 0.5) u -= kTwoPi;
    while (ref - u > kTwoPi * 0.5) u += kTwoPi;
    return u;
  }

  // Reconstruct the 2D pcurve of an edge on a surface. The pcurve maps the edge
  // parameter t to (u,v) so S_face(pcurve(t)) = C_edge(t) (the seam-weld contract).
  // Built analytically per (surface, curve) kind, mirroring the construct helpers.
  // `uRef` unwraps angular u's onto the face's continuous 2π branch (angularURef).
  topo::PCurve pcurveFor(const topo::FaceSurface& srf, const topo::Shape& edge, double uRef) {
    topo::PCurve pc;
    const auto cr = topo::curveOf(edge);
    const auto rr = topo::rangeOf(edge);
    if (!cr || !cr->curve || !rr) return pc;  // leaves a null-ish pcurve; face declines to mesh
    const topo::EdgeCurve& c = *cr->curve;
    const double first = rr->first, last = rr->last;

    // Sample the edge endpoints in 3D, project to the surface's (u,v).
    const math::Point3 e0 = evalEdge(c, first);
    const math::Point3 e1 = evalEdge(c, last);
    const auto uv0 = projectUV(srf, e0);
    const auto uv1 = projectUV(srf, e1);

    if (srf.kind == topo::FaceSurface::Kind::Plane) {
      // A straight edge on a plane is a straight UV line; a circle on a plane is a
      // UV circle about the (u,v) of its centre.
      if (c.kind == topo::EdgeCurve::Kind::Circle) {
        const auto ctrUV = projectUV(srf, c.frame.origin);
        pc.kind = topo::EdgeCurve::Kind::Circle;
        pc.origin2d = math::Point3{ctrUV.first, ctrUV.second, 0.0};
        pc.dir2d = math::Vec3{c.radius, 0.0, 0.0};  // dir2d.x carries the UV radius
        return pc;
      }
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = math::Point3{uv0.first, uv0.second, 0.0};
      const double len = std::max(last - first, 1e-12);
      pc.dir2d = math::Vec3{(uv1.first - uv0.first) / len, (uv1.second - uv0.second) / len, 0.0};
      return pc;
    }

    // On a cylinder/cone/sphere: u = the angular parameter, v = the axial/radial one.
    // A CIRCLE rim runs at constant v; its edge param t is an angle in the circle's
    // OWN frame, while the surface u is the angle in the SURFACE frame. The two frames
    // share the axis, so u = t + Δ with a constant offset Δ = uProj(p0) − first. We
    // then whole-turn-shift the arc's midpoint onto the face's branch (uRef) so all
    // the face's edges live on one continuous 2π branch (no wrap tear).
    if (c.kind == topo::EdgeCurve::Kind::Circle) {
      double delta = uv0.first - first;  // surface-u offset from the circle angle
      const double mid = 0.5 * (first + last) + delta;
      delta += unwrapTo(mid, uRef) - mid;  // shift so the arc midpoint is near uRef
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = math::Point3{delta, uv0.second, 0.0};  // u(t) = t + Δ
      pc.dir2d = math::Vec3{1.0, 0.0, 0.0};                // du/dt = +1 (same CCW sense)
      return pc;
    }
    // A straight (Line) profile edge on an angular surface runs at CONSTANT u; both
    // endpoints project to the same angle (modulo wrap). Unwrap that constant u to
    // the face's branch so it matches the arcs; v varies linearly.
    const bool angular = srf.kind == topo::FaceSurface::Kind::Cylinder ||
                         srf.kind == topo::FaceSurface::Kind::Cone ||
                         srf.kind == topo::FaceSurface::Kind::Sphere;
    const double len = std::max(last - first, 1e-12);
    if (angular) {
      const double u0 = unwrapTo(uv0.first, uRef);
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = math::Point3{u0, uv0.second, 0.0};
      // v changes with t; u is constant (line runs along the axial/v direction).
      pc.dir2d = math::Vec3{0.0, (uv1.second - uv0.second) / len, 0.0};
      return pc;
    }
    // Generic (should not reach here for the writer's surfaces): linear in (u,v).
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = math::Point3{uv0.first, uv0.second, 0.0};
    pc.dir2d = math::Vec3{(uv1.first - uv0.first) / len, (uv1.second - uv0.second) / len, 0.0};
    return pc;
  }

  // Evaluate an EdgeCurve at parameter t (LOCAL == world here; the reader builds
  // world-placed geometry with identity edge locations). Mirrors edgeCurveLocal.
  static math::Point3 evalEdge(const topo::EdgeCurve& c, double t) {
    using K = topo::EdgeCurve::Kind;
    switch (c.kind) {
      case K::Line:
        return c.frame.origin + c.frame.x.vec() * t;
      case K::Circle:
        return c.frame.origin + c.frame.x.vec() * (c.radius * std::cos(t)) +
               c.frame.y.vec() * (c.radius * std::sin(t));
      case K::BSpline:
      default:
        if (c.poles.empty()) return c.frame.origin;
        if (c.knots.empty()) return math::bezierPoint({c.poles.data(), c.poles.size()}, t);
        return math::curvePoint(c.degree, {c.poles.data(), c.poles.size()},
                                {c.knots.data(), c.knots.size()}, t);
    }
  }

  // Project a world point onto a surface's (u,v), inverting the elementary
  // parametrization (elementary.h). Analytic-exact for plane/cylinder/cone/sphere.
  static std::pair<double, double> projectUV(const topo::FaceSurface& s, const math::Point3& p) {
    using K = topo::FaceSurface::Kind;
    const math::Vec3 d = p - s.frame.origin;
    const double lx = math::dot(d, s.frame.x.vec());
    const double ly = math::dot(d, s.frame.y.vec());
    const double lz = math::dot(d, s.frame.z.vec());
    switch (s.kind) {
      case K::Plane:
        return {lx, ly};  // S(u,v)=O+uX+vY
      case K::Cylinder:
        return {std::atan2(ly, lx), lz};  // u=angle, v=axial
      case K::Cone: {
        const double v = std::fabs(std::cos(s.semiAngle)) > 1e-12 ? lz / std::cos(s.semiAngle) : lz;
        return {std::atan2(ly, lx), v};
      }
      case K::Sphere: {
        const double rc = std::sqrt(lx * lx + ly * ly);
        return {std::atan2(ly, lx), std::atan2(lz, rc)};
      }
      case K::BSpline:
      default:
        return {0.0, 0.0};  // B-spline faces are the box/quadric round-trip's non-goal
    }
  }

  // CLOSED_SHELL('',(#face..)) → a native Shell.
  topo::Shape closedShell(long id) {
    const Record* r = recOfKind(id, "CLOSED_SHELL");
    if (!r || r->args.size() != 2 || !r->args[1].isList()) { decline(); return {}; }
    std::vector<topo::Shape> faces;
    for (const Arg& f : r->args[1].list) {
      if (!f.isRef()) { decline(); return {}; }
      const topo::Shape face = advancedFace(f.ref);
      if (fail_ || face.isNull()) { decline(); return {}; }
      faces.push_back(face);
    }
    if (faces.empty()) { decline(); return {}; }
    return topo::ShapeBuilder::makeShell(std::move(faces));
  }

  // MANIFOLD_SOLID_BREP(name,#closedShell) → a native Solid (single shell).
  topo::Shape mapManifoldBrep(long id) {
    const Record* r = recOfKind(id, "MANIFOLD_SOLID_BREP");
    if (!r || r->args.size() != 2 || !r->args[1].isRef()) { decline(); return {}; }
    const topo::Shape shell = closedShell(r->args[1].ref);
    if (fail_ || shell.isNull()) { decline(); return {}; }
    return topo::ShapeBuilder::makeSolid({shell});
  }
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API.
// ─────────────────────────────────────────────────────────────────────────────
topo::Shape readStepString(const std::string& content) {
  std::unordered_map<int, Record> records;
  Parser parser(content);
  if (!parser.parse(records)) return {};
  if (records.empty()) return {};

  Mapper mapper(records);
  topo::Shape solid = mapper.build();
  if (solid.isNull()) return {};

  // The shared-node reconstruction is already watertight for a native-written file:
  // adjacent faces reference the SAME EDGE_CURVE node by #id, so the tessellator's
  // shared-edge weld fuses them (proven by the host round-trip gate). We deliberately
  // do NOT run heal::healShell here: it rebuilds every face as a best-fit PLANE with
  // straight Line edges (face_soup.h / assemble_shell.h), which would destroy a
  // cylinder/cone/sphere/B-spline face's geometry and its volume. The engine
  // (native_engine.cpp) self-verifies this solid robustly watertight with volume > 0
  // and, on ANY failure, declines to the OCCT STEPControl_Reader — an honest fall-
  // through, never a planarized or fabricated solid.
  return solid;
}

topo::Shape readStepFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  return readStepString(ss.str());
}

}  // namespace cybercad::native::exchange
