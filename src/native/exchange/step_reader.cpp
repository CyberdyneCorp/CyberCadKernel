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

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
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

  // Build the native shape: a single Solid for one MANIFOLD_SOLID_BREP, or a
  // Compound of Solids for several (a flat multi-solid body). A NULL Shape on any
  // decline (out of scope, transformed assembly, malformed, or a member that does
  // not reconstruct).
  topo::Shape build() {
    if (!validateUnitContext()) return {};
    // A TRANSFORMED assembly (a REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION /
    // ITEM_DEFINED_TRANSFORMATION or CARTESIAN_TRANSFORMATION_OPERATOR_3D tree) places
    // each MANIFOLD_SOLID_BREP through a conformal transform (rigid, uniform-scale, or
    // mirror). Parse that tree, compose a native Location per root, and return a PLACED
    // Compound. The disposition scan decides which path this file takes:
    //   * Compose — a CONTEXT_DEPENDENT_SHAPE_REPRESENTATION reaches a MANIFOLD_SOLID_BREP
    //     (a real product placement) → assembly().
    //   * Decline — a Form-B MAPPED_ITEM / REPRESENTATION_MAP, or a lone
    //     NEXT_ASSEMBLY_USAGE_OCCURRENCE with no composable placement → NULL → OCCT.
    //   * None    — only ANNOTATION / PMI relationship entities (an AP242 draughting /
    //     GD&T graph that does NOT reach a brep) → SKIP them and fall through to the flat
    //     multi-solid / single-solid path (import the geometry, drop the PMI). This is
    //     what lets an AP242 file whose PMI is carried by a REPRESENTATION_RELATIONSHIP
    //     import its solids instead of declining the whole file.
    const AsmKind ak = assemblyDisposition();
    if (ak == AsmKind::Decline) { decline(); return {}; }
    if (ak == AsmKind::Compose) return assembly();

    const std::vector<int> brepIds = findManifoldBreps();
    if (brepIds.empty() || fail_) return {};

    // Map each root independently. The memoization caches are keyed by #id and #ids are
    // globally unique within one Part-21 file, so two roots reference DISJOINT entity
    // ids — the shared caches never cross-link two solids, and re-use is safe.
    std::vector<topo::Shape> solids;
    solids.reserve(brepIds.size());
    for (const int id : brepIds) {
      const topo::Shape solid = mapManifoldBrep(id);
      if (fail_ || solid.isNull()) return {};
      solids.push_back(solid);
    }
    if (solids.size() == 1) return solids.front();  // AP203 first-slice path: unchanged
    return topo::ShapeBuilder::makeCompound(std::move(solids));
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

  // A resolved TRIMMED_CURVE's parameter bounds, keyed by the TRIMMED_CURVE #id. A
  // TRIMMED_CURVE wraps a basis curve with two parameter/point trims; curve() unwraps
  // it to the basis EdgeCurve and records the trims here so edgeCurve() can bound the
  // edge by the trims where they add representable value (a B-spline sub-domain). For
  // an analytic basis the endpoint VERTICES already fix the range exactly, so the trim
  // is validated but the vertex-derived range (curveRange) is kept.
  struct TrimInfo {
    bool hasT0 = false, hasT1 = false;
    double t0 = 0.0, t1 = 0.0;
  };
  std::unordered_map<int, TrimInfo> trimCache_;

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

  // All MANIFOLD_SOLID_BREP roots, in ascending #id order (deterministic — the record
  // table is unordered). One root → a single Solid; several → a multi-solid Compound
  // (see build()). Order is stable so a fixture's per-solid comparison is repeatable.
  std::vector<int> findManifoldBreps() {
    std::vector<int> ids;
    for (const auto& [id, r] : recs_) {
      if (r.combined || r.keyword != "MANIFOLD_SOLID_BREP") continue;
      ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
  }

  // How the file's transform / relationship graph dispositions the import.
  enum class AsmKind { None, Compose, Decline };

  // Classify the file's product-structure / relationship graph WITHOUT reading it as a
  // blanket "any relationship → assembly" trigger (which declined an AP242 file the
  // moment its PMI carried a REPRESENTATION_RELATIONSHIP). The decision keys on whether
  // a CONTEXT_DEPENDENT_SHAPE_REPRESENTATION actually reaches a MANIFOLD_SOLID_BREP:
  //   * Compose — at least one CDSR's child shape-representation lists a brep → a real
  //     product placement to compose (assembly()).
  //   * Decline — no brep-reaching placement, but a Form-B MAPPED_ITEM / REPRESENTATION_MAP
  //     (out of slice) or a lone NEXT_ASSEMBLY_USAGE_OCCURRENCE (an assembly usage with
  //     no composable transform) is present → NULL → OCCT (unchanged honesty).
  //   * None    — only ANNOTATION / PMI relationship entities that never reach a brep →
  //     SKIP; the flat multi-solid / single-solid path imports the geometry.
  // A FLAT multi-solid / single-solid file (no relationships at all) is also None.
  AsmKind assemblyDisposition() {
    bool brepReachingCDSR = false, hasMappedItem = false, hasNauo = false;
    for (const auto& [id, r] : recs_) {
      if (r.combined) continue;
      const std::string& kw = r.keyword;
      if (kw == "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION" && !r.args.empty() &&
          r.args[0].isRef()) {
        const auto [childSr, opId] = relationshipAndTransform(r.args[0].ref);
        if (childSr != 0 && brepOfRepresentation(childSr) != 0) brepReachingCDSR = true;
      } else if (kw == "MAPPED_ITEM" || kw == "REPRESENTATION_MAP") {
        hasMappedItem = true;
      } else if (kw == "NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
        hasNauo = true;
      }
    }
    if (brepReachingCDSR) return AsmKind::Compose;
    if (hasMappedItem || hasNauo) return AsmKind::Decline;
    return AsmKind::None;
  }

  // ── Assembly transform tree (Form A: IDT / REP_REL_WITH_TRANSFORMATION) ───────
  //
  // OCCT's STEPControl_Writer emits, per placed component, a
  // CONTEXT_DEPENDENT_SHAPE_REPRESENTATION → (REPRESENTATION_RELATIONSHIP +
  // REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION + SHAPE_REPRESENTATION_RELATIONSHIP)
  // combined instance whose transform is an ITEM_DEFINED_TRANSFORMATION carrying a
  // FROM/TO AXIS2_PLACEMENT_3D pair. The rigid placement is
  //   T = frameToWorld(to) ∘ frameToWorld(from)⁻¹.
  // We seed every MANIFOLD_SOLID_BREP at identity, apply each component's T to the
  // brep reached through its CHILD shape-representation's item list, then return a
  // placed Compound (single root → the placed solid). Any structure we cannot
  // compose exactly (Form-B MAPPED_ITEM, a non-rigid/scaled/mirrored transform, an
  // out-of-slice component, an under- or over-constrained brep) sets fail_ → NULL →
  // OCCT. We never invent a placement the file did not describe.

  // frameToWorld(Ax3): the affine map taking local coords in the placement frame to
  // world — linear columns are the frame's X/Y/Z axes, translation is its origin.
  static math::Transform frameToWorld(const math::Ax3& f) {
    const math::Vec3 x = f.x.vec(), y = f.y.vec(), z = f.z.vec();
    const math::Mat3 m{x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z};
    return math::Transform{m, f.origin.asVec()};
  }

  // The SUPPORTED conformal placement classes. A placement is composed only if its
  // linear part is CONFORMAL — MᵀM = k²·I for one scalar k (a rotation or reflection,
  // uniformly scaled). Non-uniform scale / shear (MᵀM not a scalar multiple of I) is
  // out of the honest slice and DECLINES → OCCT.
  enum class PlacementClass { Rigid, UniformScale, Mirror };

  // Classify a composed component transform, or nullopt to DECLINE. The Gram matrix
  // g = MᵀM of a conformal map equals k²·I; the determinant sign then separates a
  // proper (rigid / uniform-scale, det = +k³) from a reflection (mirror, det = −k³).
  //   * Rigid        — k ≈ 1, det ≈ +1 (today's behaviour, unchanged).
  //   * UniformScale — k ≠ 1, det > 0 (the placed solid's volume scales by k³; the
  //     tessellator renders it correctly through the located node, unmodified).
  //   * Mirror       — det < 0 (a reflection, optionally scaled; the composer applies
  //     the Location AND complements the component's orientation so the
  //     tangent-derived world normals point outward again — see assembly()).
  //   * nullopt      — non-uniform / shear / singular → DECLINE.
  static std::optional<PlacementClass> classifyPlacement(const math::Transform& t) {
    const math::Mat3& m = t.linear();
    const math::Mat3 g = m.transposed() * m;  // Gram matrix
    const double s2 = g(0, 0);
    if (s2 < 1e-12) return std::nullopt;  // singular / degenerate linear part
    const double tol = 1e-9 * (1.0 + s2);
    for (std::size_t i = 0; i < 3; ++i)
      for (std::size_t j = 0; j < 3; ++j) {
        const double target = (i == j) ? s2 : 0.0;  // conformal ⇒ g = s²·I
        if (std::fabs(g(i, j) - target) > tol) return std::nullopt;
      }
    const double k = std::sqrt(s2);
    const double det = m.determinant();
    if (det < 0.0) return PlacementClass::Mirror;
    return std::fabs(k - 1.0) < 1e-9 ? PlacementClass::Rigid : PlacementClass::UniformScale;
  }

  // CARTESIAN_TRANSFORMATION_OPERATOR_3D('',#axis1(u),#axis2(v),#origin,scale,#axis3(w))
  // → the affine map  world = origin + scale·(x·u + y·v + z·w). This is the STEP entity
  // (ISO 10303-42) a foreign system emits for a SCALED or MIRRORED instance transform
  // (the AXIS2_PLACEMENT_3D frame pair of an ITEM_DEFINED_TRANSFORMATION cannot carry a
  // scale or reflection — its axes are normalized and right-handed). Missing axes take
  // the schema defaults (u=+X, w=+Z, v=w×u ≈ +Y); a missing scale defaults to 1. The
  // linear part is built from the file's LITERAL (unit) axis vectors, so the placement
  // classifier is the sole conformality gate — a non-orthogonal triad (shear) or a
  // singular one DECLINES; a clean scaled / reflected triad is applied.
  std::optional<math::Transform> cartesianOperator(long id) {
    const Record* r = rec(id);
    if (!r || r->combined) return std::nullopt;
    if (r->keyword != "CARTESIAN_TRANSFORMATION_OPERATOR_3D" &&
        r->keyword != "CARTESIAN_TRANSFORMATION_OPERATOR")
      return std::nullopt;
    if (r->args.size() < 4 || !r->args[3].isRef()) return std::nullopt;
    const auto o = point(r->args[3].ref);
    if (!o) return std::nullopt;
    math::Dir3 u{1, 0, 0}, v{0, 1, 0}, w{0, 0, 1};
    if (r->args[1].isRef()) { if (auto d = direction(r->args[1].ref)) u = *d; }
    if (r->args[2].isRef()) { if (auto d = direction(r->args[2].ref)) v = *d; }
    if (r->args.size() > 5 && r->args[5].isRef()) { if (auto d = direction(r->args[5].ref)) w = *d; }
    double scale = 1.0;
    if (r->args.size() > 4 && r->args[4].isNumber()) scale = r->args[4].asReal();
    if (std::fabs(scale) < 1e-12) return std::nullopt;
    const math::Vec3 U = u.vec(), V = v.vec(), W = w.vec();
    const math::Mat3 lin{scale * U.x, scale * V.x, scale * W.x,
                         scale * U.y, scale * V.y, scale * W.y,
                         scale * U.z, scale * V.z, scale * W.z};
    return math::Transform{lin, o->asVec()};
  }

  // Resolve a REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION transformation_operator
  // #id (an ITEM_DEFINED_TRANSFORMATION or a CARTESIAN_TRANSFORMATION_OPERATOR_3D) to a
  // composed placement Transform + a MIRROR flag, or nullopt to DECLINE. The classifier
  // gates conformality: rigid / uniform-scale / mirror are accepted, non-uniform / shear
  // declines.
  std::optional<std::pair<math::Transform, bool>> resolveOperator(long opId) {
    std::optional<math::Transform> t = itemDefinedTransform(opId);
    if (!t) t = cartesianOperator(opId);
    if (!t) return std::nullopt;
    const auto cls = classifyPlacement(*t);
    if (!cls) return std::nullopt;
    return std::make_pair(*t, *cls == PlacementClass::Mirror);
  }

  // The list of item #ids of a *_SHAPE_REPRESENTATION (or SHAPE_REPRESENTATION):
  // its arg[1] is the list of represented_items (points/placements/breps).
  std::vector<long> representationItems(long srId) {
    std::vector<long> items;
    const Record* r = rec(srId);
    if (!r || r->combined || r->args.size() < 2 || !r->args[1].isList()) return items;
    for (const Arg& a : r->args[1].list)
      if (a.isRef()) items.push_back(a.ref);
    return items;
  }

  // The single MANIFOLD_SOLID_BREP #id reachable from a shape-representation's item
  // list, or 0 if none / more than one (an ambiguous component declines).
  long brepOfRepresentation(long srId) {
    long found = 0;
    for (const long it : representationItems(srId)) {
      if (recOfKind(it, "MANIFOLD_SOLID_BREP")) {
        if (found != 0) return 0;  // more than one brep in one component → ambiguous
        found = it;
      }
    }
    return found;
  }

  // Read an ITEM_DEFINED_TRANSFORMATION('','',#fromAx2,#toAx2) → the composed transform
  // T = frameToWorld(to) ∘ frameToWorld(from)⁻¹, or nullopt if it is not readable
  // (missing/other-typed args or an unreadable/singular placement). Conformality is
  // classified by resolveOperator/classifyPlacement, not here — an AXIS2_PLACEMENT_3D
  // frame pair is always rigid (its axes are normalized right-handed), so this path
  // yields a rigid T; a scale/mirror arrives through cartesianOperator instead.
  std::optional<math::Transform> itemDefinedTransform(long idtId) {
    const Record* r = recOfKind(idtId, "ITEM_DEFINED_TRANSFORMATION");
    if (!r || r->args.size() != 4 || !r->args[2].isRef() || !r->args[3].isRef())
      return std::nullopt;
    const auto from = axis2placement(r->args[2].ref);
    const auto to = axis2placement(r->args[3].ref);
    if (!from || !to) return std::nullopt;
    const auto fromInv = frameToWorld(*from).inverse();
    if (!fromInv) return std::nullopt;
    return frameToWorld(*to).composedWith(*fromInv);
  }

  // Locate the REPRESENTATION_RELATIONSHIP and the IDT id inside the combined
  // instance a CONTEXT_DEPENDENT_SHAPE_REPRESENTATION points to (arg[0]). The
  // instance carries a REPRESENTATION_RELATIONSHIP sub-record (rep_1=child(arg[2]),
  // rep_2=parent(arg[3])) and a REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION
  // sub-record (transform_operator=IDT(arg[0])). Returns {childSrId, idtId} or
  // {0,0} if the shape is not the expected combined form.
  std::pair<long, long> relationshipAndTransform(long relId) {
    const Record* r = rec(relId);
    if (!r || !r->combined) return {0, 0};
    long childSr = 0, idt = 0;
    for (const SubRecord& sub : r->subs) {
      if (sub.keyword == "REPRESENTATION_RELATIONSHIP") {
        if (sub.args.size() >= 3 && sub.args[2].isRef()) childSr = sub.args[2].ref;
      } else if (sub.keyword == "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
        if (!sub.args.empty() && sub.args[0].isRef()) idt = sub.args[0].ref;
      }
    }
    return {childSr, idt};
  }

  // Build a placed Compound from the assembly transform tree. Declines (fail_) on
  // any Form-B / non-uniform-shear / out-of-slice / under-constrained structure. A
  // brep-reaching CDSR places its component by a rigid, uniform-scale, or mirror
  // transform; a mirror also has its faces complemented so the world normals stay
  // outward. A CDSR that does NOT reach a brep (an AP242 PMI / annotation
  // relationship) is SKIPPED, not declined.
  topo::Shape assembly() {
    const std::vector<int> brepIds = findManifoldBreps();
    if (brepIds.empty()) { decline(); return {}; }

    // Form B (MAPPED_ITEM / REPRESENTATION_MAP) is out of this slice — decline the
    // whole file rather than place breps at the wrong (identity) location.
    for (const auto& [id, r] : recs_)
      if (!r.combined && (r.keyword == "MAPPED_ITEM" || r.keyword == "REPRESENTATION_MAP")) {
        decline();
        return {};
      }

    // Seed every root at identity; each product CDSR places exactly one brep once.
    std::unordered_map<int, math::Transform> placement;
    std::unordered_map<int, bool> placed;
    std::unordered_map<int, bool> mirrored;  // reflection ⇒ complement the faces
    for (const int id : brepIds) {
      placement.emplace(id, math::Transform::identity());
      placed.emplace(id, false);
      mirrored.emplace(id, false);
    }

    const int rootCount = static_cast<int>(brepIds.size());
    int placedCount = 0;
    for (const auto& [id, r] : recs_) {
      if (r.combined || r.keyword != "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") continue;
      if (r.args.empty() || !r.args[0].isRef()) continue;  // not a placement carrier
      const auto [childSr, opId] = relationshipAndTransform(r.args[0].ref);
      if (childSr == 0 || opId == 0) continue;              // malformed / annotation → skip
      const long brep = brepOfRepresentation(childSr);
      if (brep == 0) continue;  // PMI / annotation relationship (no brep) → SKIP, not decline
      auto pit = placement.find(static_cast<int>(brep));
      if (pit == placement.end() || placed[static_cast<int>(brep)]) {
        // A brep referenced by no seed, or placed twice → structure we do not model.
        decline();
        return {};
      }
      const auto op = resolveOperator(opId);  // rigid / uniform-scale / mirror, or decline
      if (!op) { decline(); return {}; }
      pit->second = op->first;
      mirrored[static_cast<int>(brep)] = op->second;
      placed[static_cast<int>(brep)] = true;
      ++placedCount;
    }

    // Honest completeness gate: the assembly must place all-but-one root through the
    // tree (the ROOT component carries no CDSR and stays identity). Anything else —
    // no placements found, or a brep left unplaced with no clear root — declines
    // rather than importing part of the assembly at a fabricated identity location.
    const int unplaced = rootCount - placedCount;
    if (placedCount == 0 || unplaced > 1) { decline(); return {}; }

    // Map each root and apply its composed Location. A mirrored component's faces are
    // complemented (reversedShape) so the tessellator's tangent-derived normals point
    // OUTWARD again — a reflection flips cross(∂u,∂v), which without this compensation
    // would mesh the solid inside-out (negative enclosed volume). No tessellator change.
    std::vector<topo::Shape> solids;
    solids.reserve(brepIds.size());
    for (const int id : brepIds) {
      const topo::Shape solid = mapManifoldBrep(id);
      if (fail_ || solid.isNull()) return {};
      topo::Shape placedSolid = solid;  // unplaced ROOT stays identity (flat-path identical)
      if (placed.at(id)) {
        placedSolid = solid.located(topo::Location{placement.at(id)});
        if (mirrored.at(id)) placedSolid = placedSolid.reversedShape();
      }
      solids.push_back(placedSolid);
    }
    if (solids.size() == 1) return solids.front();
    return topo::ShapeBuilder::makeCompound(std::move(solids));
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

  // ── EdgeCurve (LINE / CIRCLE / ELLIPSE / B_SPLINE_CURVE_WITH_KNOTS) by #id ────
  std::optional<topo::EdgeCurve> curve(long id) {
    const Record* r = rec(id);
    if (!r || r->combined) { return std::nullopt; }
    // SURFACE_CURVE / SEAM_CURVE / INTERSECTION_CURVE wrap a 3D `curve_3d` (arg[1])
    // together with a list of PCURVEs. A FOREIGN AP203 file (OCCT STEPControl_Writer)
    // wraps every EDGE_CURVE's 3D geometry this way; the native writer emits the 3D
    // curve directly. We recurse on the 3D curve and IGNORE the STEP pcurves — the
    // reader synthesises its own analytic pcurves per face surface (pcurveFor). Without
    // this unwrap the reader could not read any OCCT-authored STEP (it would decline on
    // the wrapper keyword). Additive: native-written files never carry these.
    if (r->keyword == "SURFACE_CURVE" || r->keyword == "SEAM_CURVE" ||
        r->keyword == "INTERSECTION_CURVE") {
      if (r->args.size() < 2 || !r->args[1].isRef()) return std::nullopt;
      return curve(r->args[1].ref);
    }
    if (r->keyword == "LINE") return lineCurve(*r);
    if (r->keyword == "CIRCLE") return circleCurve(*r);
    if (r->keyword == "ELLIPSE") return ellipseCurve(*r);
    if (r->keyword == "B_SPLINE_CURVE_WITH_KNOTS") return bsplineCurve(*r);
    if (r->keyword == "TRIMMED_CURVE") return trimmedCurve(id, *r);
    // Any other curve keyword (RATIONAL_B_SPLINE_*, …) is out of scope.
    return std::nullopt;
  }

  // TRIMMED_CURVE('',#basis,trim_1,trim_2,sense_agreement,master_representation):
  // a basis curve (LINE/CIRCLE/ELLIPSE/B_SPLINE_CURVE_WITH_KNOTS) bounded by two
  // trims. Each trim is a SET holding a PARAMETER_VALUE(real) and/or a CARTESIAN_POINT
  // ref. We unwrap to the basis EdgeCurve (mirroring the SURFACE_CURVE unwrap) and
  // cache the two PARAMETER_VALUE trims by this TRIMMED_CURVE's #id; edgeCurve() honors
  // them for a B-spline sub-domain (where the vertices alone cannot recover the covered
  // knot span). A basis outside the supported slice DECLINES → OCCT.
  std::optional<topo::EdgeCurve> trimmedCurve(long id, const Record& r) {
    if (r.args.size() < 4 || !r.args[1].isRef()) return std::nullopt;
    auto basis = curve(r.args[1].ref);
    if (!basis) return std::nullopt;
    TrimInfo ti;
    if (auto t = trimParam(r.args[2])) { ti.hasT0 = true; ti.t0 = *t; }
    if (auto t = trimParam(r.args[3])) { ti.hasT1 = true; ti.t1 = *t; }
    trimCache_[static_cast<int>(id)] = ti;
    return basis;
  }

  // Extract the PARAMETER_VALUE(real) of a trim SET, if present. The tokenizer already
  // unwraps a unary typed value (PARAMETER_VALUE(x) → x), so a SET member is either a
  // Real (the parameter) or a Ref (a CARTESIAN_POINT — ignored: the edge vertices carry
  // the exact endpoints). A bare Real (SET-less writer) is accepted too. Non-finite → none.
  static std::optional<double> trimParam(const Arg& a) {
    auto finite = [](double v) -> std::optional<double> {
      return std::isfinite(v) ? std::optional<double>{v} : std::nullopt;
    };
    if (a.isNumber()) return finite(a.asReal());
    if (a.isList())
      for (const Arg& m : a.list)
        if (m.isNumber()) return finite(m.asReal());
    return std::nullopt;
  }

  // LINE('',#point,#vector): frame origin = point, X = vector direction.
  std::optional<topo::EdgeCurve> lineCurve(const Record& r) {
    if (r.args.size() != 3 || !r.args[1].isRef() || !r.args[2].isRef()) return std::nullopt;
    const auto o = point(r.args[1].ref);
    const auto d = vectorDir(r.args[2].ref);
    if (!o || !d) return std::nullopt;
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame.origin = *o;
    c.frame.x = *d;
    c.frame.z = math::Dir3{0, 0, 1};
    return c;
  }

  // CIRCLE('',#axis2placement,radius).
  std::optional<topo::EdgeCurve> circleCurve(const Record& r) {
    if (r.args.size() != 3 || !r.args[1].isRef() || !r.args[2].isNumber()) return std::nullopt;
    const auto f = axis2placement(r.args[1].ref);
    if (!f) return std::nullopt;
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Circle;
    c.frame = *f;
    c.radius = r.args[2].asReal();
    return c;
  }

  // ELLIPSE('',#axis2placement,semiAxis1,semiAxis2): semiAxis1 is the major (X)
  // radius, semiAxis2 the minor (Y) radius, in the placement frame — the exact inputs
  // the native Ellipse EdgeCurve carries (shape.h: radius=major, minorRadius=minor)
  // and the tessellator evaluates (edge_mesher.h case K::Ellipse). Degenerate
  // (non-positive) axes decline → OCCT.
  std::optional<topo::EdgeCurve> ellipseCurve(const Record& r) {
    if (r.args.size() != 4 || !r.args[1].isRef() || !r.args[2].isNumber() || !r.args[3].isNumber())
      return std::nullopt;
    const auto f = axis2placement(r.args[1].ref);
    if (!f) return std::nullopt;
    const double a = r.args[2].asReal(), b = r.args[3].asReal();
    if (!(a > 0.0) || !(b > 0.0)) return std::nullopt;
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Ellipse;
    c.frame = *f;
    c.radius = a;
    c.minorRadius = b;
    return c;
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
    if (r->keyword == "SURFACE_OF_REVOLUTION") return surfaceOfRevolution(*r);
    return std::nullopt;  // any other surface keyword is out of scope
  }

  // AXIS1_PLACEMENT('',#location,#axis?) → (location, axis direction). Unlike an
  // AXIS2_PLACEMENT_3D it carries ONLY the main axis (the revolution axis); the axis
  // direction is optional ($) and defaults to +Z. Used by SURFACE_OF_REVOLUTION.
  std::optional<std::pair<math::Point3, math::Dir3>> axis1placement(long id) {
    const Record* r = recOfKind(id, "AXIS1_PLACEMENT");
    if (!r || r->args.size() < 2 || !r->args[1].isRef()) return std::nullopt;
    const auto loc = point(r->args[1].ref);
    if (!loc) return std::nullopt;
    math::Dir3 axis{0, 0, 1};
    if (r->args.size() >= 3 && r->args[2].isRef())
      if (auto d = direction(r->args[2].ref)) axis = *d;
    if (!axis.valid()) return std::nullopt;
    return std::make_pair(*loc, axis);
  }

  // SURFACE_OF_REVOLUTION('',#profileCurve,#axis1placement): a profile (generatrix)
  // curve revolved about an axis. The generatrix is classified and reduced to the
  // matching native analytic FaceSurface — but ONLY when the reduction is faithful:
  //   * LINE  ∥ axis            → CYLINDER (radius = dist(line, axis)).
  //   * LINE  ⟂ axis            → PLANE    (a flat annulus/disk; normal = axis).
  //   * LINE  oblique, meeting the axis → CONE (apex = the intersection, half-angle
  //                              = ∠(line, axis)). A skew oblique line → hyperboloid.
  //   * CIRCLE centred ON the axis, plane CONTAINING the axis → SPHERE (radius = R).
  // Everything else revolves to a surface with NO faithful native kind → honest
  // DECLINE → OCCT (unchanged precedent, exactly like the landed TOROIDAL_SURFACE
  // decline): an off-axis circle/arc (torus), a circle whose plane is ⟂ the axis
  // (degenerate), an ellipse / B-spline generatrix (general revolved surface), a skew
  // oblique line (hyperboloid), a line ON the axis (degenerate). Each builder also runs
  // a faithful-reduction guard (lineOnSurface / circleOnSurface) that re-evaluates the
  // candidate quadric through the generatrix's defining points and DECLINES if they do
  // not lie on it within a scale-relative tolerance — never a forced/approximate map.
  std::optional<topo::FaceSurface> surfaceOfRevolution(const Record& r) {
    if (r.args.size() != 3 || !r.args[1].isRef() || !r.args[2].isRef()) return std::nullopt;
    const auto profile = curve(r.args[1].ref);
    const auto ax = axis1placement(r.args[2].ref);
    if (!profile || !ax) return std::nullopt;
    using K = topo::EdgeCurve::Kind;
    if (profile->kind == K::Line) return revolvedLine(*profile, ax->first, ax->second);
    if (profile->kind == K::Circle) return revolvedCircle(*profile, ax->first, ax->second);
    return std::nullopt;  // Ellipse / BSpline / Bezier → general revolution → DECLINE
  }

  // Closest approach between the generatrix support line (P, unit D) and the axis
  // (L, unit A). If the two support lines are SKEW (common perpendicular > tol) the
  // revolved oblique line is a hyperboloid of one sheet → nullopt (DECLINE). Otherwise
  // returns their on-axis intersection point — the cone apex.
  static std::optional<math::Point3> lineMeetsAxis(const math::Point3& P, const math::Dir3& D,
                                                   const math::Point3& L, const math::Dir3& A) {
    const math::Vec3 d = D.vec(), a = A.vec();
    const math::Vec3 w0 = P - L;
    const double c = math::dot(d, a);
    const double denom = 1.0 - c * c;               // sin²∠ > 0 for an oblique line
    if (denom < 1e-12) return std::nullopt;         // parallel — caller handles it
    const double dw = math::dot(d, w0), aw = math::dot(a, w0);
    const math::Point3 pc = P + d * ((c * aw - dw) / denom);  // closest pt on generatrix
    const math::Point3 qc = L + a * ((aw - c * dw) / denom);  // closest pt on axis = apex
    const double scale = std::max(1.0, math::norm(w0));
    if (math::distance(pc, qc) > 1e-7 * scale) return std::nullopt;  // SKEW → hyperboloid
    return qc;
  }

  // Revolve a straight generatrix (line through P, unit direction D) about the axis
  // (through L, unit direction A) onto an EXACT native Cylinder / Plane / Cone, or
  // nullopt to DECLINE. The cone frame mirrors the direct CONICAL_SURFACE convention
  // (origin on the axis, Z = +axis, radius = the reference radius at that origin's
  // plane, semiAngle SIGNED so radius = R + v·sinα grows toward the base and reaches 0
  // at the apex) so the reconstruction is byte-identical to the analytic-keyword path.
  std::optional<topo::FaceSurface> revolvedLine(const topo::EdgeCurve& line,
                                                const math::Point3& L, const math::Dir3& A) {
    const math::Dir3 D = line.frame.x;
    if (!D.valid()) return std::nullopt;

    const math::Point3 P = line.frame.origin;
    const math::Vec3 Av = A.vec();
    const double c = math::dot(Av, D.vec());          // cos∠(axis, line)
    const double ac = std::fabs(c);
    auto footOf = [&](const math::Point3& q) { return L + Av * math::dot(q - L, Av); };

    topo::FaceSurface s;
    if (ac < 1e-7) {                                  // ⟂ → PLANE (annulus/disk)
      // A ⟂ generatrix crosses the axis (P may be ON it), so the swept radius at P can
      // be 0 — but the plane is fully fixed by foot + normal A, with the line dir D
      // (⟂ A, radial) as the X ref. The plane passes through foot at the generatrix's
      // axial height.
      s.kind = topo::FaceSurface::Kind::Plane;
      s.frame = math::Ax3::fromAxisAndRef(footOf(P), A, D);
    } else if (std::fabs(ac - 1.0) < 1e-7) {          // ∥ → CYLINDER
      const math::Point3 foot = footOf(P);
      const math::Vec3 radial = P - foot;
      const double r = math::norm(radial);
      if (r < 1e-9) return std::nullopt;              // line ON the axis → degenerate
      s.kind = topo::FaceSurface::Kind::Cylinder;
      s.frame = math::Ax3::fromAxisAndRef(foot, A, math::Dir3{radial});
      s.radius = r;
    } else {                                          // oblique → CONE (if it meets A)
      const auto apex = lineMeetsAxis(P, D, L, A);
      if (!apex) return std::nullopt;                 // skew → hyperboloid → DECLINE
      // Reference the cone at an OFF-axis generator point (P, or one step along D if P
      // happens to sit at the apex): origin = its foot on the axis, radius = its ⊥-dist
      // there, so the origin is a REGULAR point (never the apex-singular v).
      math::Point3 ref = P;
      math::Point3 foot = footOf(ref);
      if (math::norm(ref - foot) < 1e-9) { ref = P + D.vec(); foot = footOf(ref); }
      const double r = math::norm(ref - foot);
      if (r < 1e-9) return std::nullopt;
      const double hRef = math::dot(ref - L, Av);
      const double hApex = math::dot(*apex - L, Av);
      s.kind = topo::FaceSurface::Kind::Cone;
      s.frame = math::Ax3::fromAxisAndRef(foot, A, math::Dir3{ref - foot});
      s.radius = r;                                   // reference radius at foot's plane
      // Signed so radius = R + v·sinα GROWS moving +axis away from the apex (Z=+axis).
      s.semiAngle = std::acos(ac) * (hRef >= hApex ? 1.0 : -1.0);
    }
    if (!lineOnSurface(line, s)) return std::nullopt;  // faithful-reduction guard
    return s;
  }

  // Revolve a CIRCLE/arc generatrix about the axis (L, A). A native SPHERE results only
  // when the circle's CENTRE lies ON the axis AND its plane CONTAINS the axis (the
  // circle is a meridian great-circle). Any off-axis centre → a torus (no native
  // Kind::Torus); any other plane orientation → a torus/degenerate. Both DECLINE → OCCT.
  std::optional<topo::FaceSurface> revolvedCircle(const topo::EdgeCurve& circle,
                                                  const math::Point3& L, const math::Dir3& A) {
    const double R = circle.radius;
    if (!(R > 1e-12)) return std::nullopt;
    const math::Vec3 Av = A.vec();
    const math::Point3 C = circle.frame.origin;
    const math::Point3 footC = L + Av * math::dot(C - L, Av);
    if (math::distance(C, footC) > 1e-7 * std::max(1.0, R)) return std::nullopt;  // off-axis → torus
    if (std::fabs(math::dot(circle.frame.z.vec(), Av)) > 1e-7) return std::nullopt;  // plane ∦ axis

    topo::FaceSurface s;
    s.kind = topo::FaceSurface::Kind::Sphere;
    // Z = axis; X ref = the circle-plane normal, which is ⟂ A (checked above).
    s.frame = math::Ax3::fromAxisAndRef(C, A, circle.frame.z);
    s.radius = R;
    if (!circleOnSurface(circle, s)) return std::nullopt;  // faithful-reduction guard
    return s;
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
    const auto range = trimmedRange(r->args[3].ref, *cv, *p0, *p1);
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
      case K::Ellipse: {
        // The ellipse's PARAMETRIC angle t: point = O + a·cos t·X + b·sin t·Y, so
        // t = atan2(dot(d,Y)/b, dot(d,X)/a). Same CCW-forward convention as a circle.
        const double t0 = ellipseAngle(c, p0);
        double t1 = ellipseAngle(c, p1);
        if (math::distance(p0, p1) <= 1e-9) return {t0, t0 + kTwoPi};
        while (t1 <= t0 + 1e-12) t1 += kTwoPi;
        return {t0, t1};
      }
      case K::BSpline:
      default:
        // The B-spline curve is parametrized over its knot span; the writer trims
        // exactly to [knots.front(), knots.back()].
        return {c.knots.empty() ? 0.0 : c.knots.front(),
                c.knots.empty() ? 1.0 : c.knots.back()};
    }
  }

  // The edge parameter range, honoring a TRIMMED_CURVE's cached trims when `curveId`
  // resolved through one. For a B-spline basis the two PARAMETER_VALUE trims select the
  // covered knot sub-domain — information the endpoint vertices cannot recover (a
  // sub-arc of a periodic/looping spline shares endpoints with the whole curve). We
  // clamp the trims to the clamped knot span so a wide/degenerate trim reduces to the
  // full curve. For an analytic basis (line/circle/ellipse) the vertices fix the
  // endpoints exactly, so we keep the vertex-derived range (curveRange) and use the
  // trims only implicitly (validated in trimmedCurve).
  std::pair<double, double> trimmedRange(long curveId, const topo::EdgeCurve& c,
                                         const math::Point3& p0, const math::Point3& p1) {
    const auto it = trimCache_.find(static_cast<int>(curveId));
    if (it != trimCache_.end() && c.kind == topo::EdgeCurve::Kind::BSpline && !c.knots.empty() &&
        it->second.hasT0 && it->second.hasT1) {
      const double lo = c.knots.front(), hi = c.knots.back();
      double a = std::clamp(std::min(it->second.t0, it->second.t1), lo, hi);
      double b = std::clamp(std::max(it->second.t0, it->second.t1), lo, hi);
      if (b - a > 1e-9) return {a, b};  // a genuine sub-domain; degenerate → full curve
    }
    return curveRange(c, p0, p1);
  }

  // Angle of a point on a circle's frame: atan2(dot(p-O,Y), dot(p-O,X)).
  static double circleAngle(const topo::EdgeCurve& c, const math::Point3& p) {
    const math::Vec3 d = p - c.frame.origin;
    const double x = math::dot(d, c.frame.x.vec());
    const double y = math::dot(d, c.frame.y.vec());
    return std::atan2(y, x);
  }

  // Parametric angle of a point on an ellipse: point = O + a·cos t·X + b·sin t·Y ⇒
  // t = atan2(dot(p-O,Y)/b, dot(p-O,X)/a). (a=radius major, b=minorRadius.)
  static double ellipseAngle(const topo::EdgeCurve& c, const math::Point3& p) {
    const math::Vec3 d = p - c.frame.origin;
    const double x = c.radius > 1e-12 ? math::dot(d, c.frame.x.vec()) / c.radius : 0.0;
    const double y = c.minorRadius > 1e-12 ? math::dot(d, c.frame.y.vec()) / c.minorRadius : 0.0;
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
    // A VERTEX_LOOP bound (a single degenerate vertex, NO edges) is how OCCT emits a
    // FULL untrimmed periodic surface — e.g. a whole sphere, whose longitude seam and
    // both poles collapse so the face carries no real boundary EDGE_CURVE. Represent
    // it as an empty (childless) wire; advancedFace turns a genuine full sphere into a
    // BARE periodic surface that the tessellator meshes watertight over natural bounds.
    // Everything with a real EDGE_LOOP keeps the existing edge-chaining path unchanged.
    if (const Record* loop = recOfKind(r->args[1].ref, "VERTEX_LOOP")) {
      if (loop->args.size() != 2 || !loop->args[1].isRef()) { decline(); return {}; }
      return topo::ShapeBuilder::makeWire({});
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

    const topo::Orientation orient =
        r->args[3].text == "T" ? topo::Orientation::Forward : topo::Orientation::Reversed;

    // A childless bound comes from a VERTEX_LOOP face-bound (a FULL untrimmed periodic
    // surface). It closes watertight ONLY for a genuine full sphere: the tessellator
    // meshes the natural (u∈[0,2π], v∈[-π/2,π/2]) rectangle for Kind::Sphere and welds
    // the longitude seam plus both collapsed poles. Build such a face as a BARE
    // periodic surface (null outer wire). Any OTHER surface, or a sphere face that
    // ALSO carries real trim edges (a partial zone), cannot close this way → keep the
    // honest OCCT deferral (the engine's watertight self-verify is the final arbiter).
    bool anyEmpty = false, allEmpty = true;
    for (const topo::Shape& w : wires) {
      if (w.tshape()->children().empty()) anyEmpty = true;
      else allEmpty = false;
    }
    if (anyEmpty) {
      if (srf->kind == topo::FaceSurface::Kind::Sphere && allEmpty)
        return topo::ShapeBuilder::makeFace(*srf, topo::Shape{}, {}, orient);
      decline();
      return {};
    }

    if (outerIdx < 0) outerIdx = 0;

    // Build the face node first (surface + wires) so pcurves can key on it, then
    // re-attach the analytic pcurve per edge on this face's surface.
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
      // An ELLIPSE on a plane is a UV ellipse about the (u,v) of its centre; the
      // circle-frame X/Y map to the plane's u/v (the writer/reader place both frames
      // co-planar), so dir2d carries (majorUV, minorUV). Mirrors the Circle-on-plane
      // arm and the tessellator's Ellipse pcurve evaluator (trim.h case K::Ellipse).
      if (c.kind == topo::EdgeCurve::Kind::Ellipse) {
        const auto ctrUV = projectUV(srf, c.frame.origin);
        pc.kind = topo::EdgeCurve::Kind::Ellipse;
        pc.origin2d = math::Point3{ctrUV.first, ctrUV.second, 0.0};
        pc.dir2d = math::Vec3{c.radius, c.minorRadius, 0.0};  // (major,minor) UV radii
        return pc;
      }
      // A B-SPLINE edge on a plane (e.g. a spline-profile extrude CAP boundary) keeps
      // its own degree/knots; its 2D poles are the 3D poles projected to the plane's
      // (u,v). So S_plane(pcurve(t)) = C_edge(t) exactly (the spline's (x,y) IS its
      // planar image) — the seam-weld contract that closes the cap↔wall boundary. This
      // is the exact inverse of the construct spline-wall cap edge (residuals.h capEdge).
      if (c.kind == topo::EdgeCurve::Kind::BSpline && !c.poles.empty()) {
        pc.kind = topo::EdgeCurve::Kind::BSpline;
        pc.degree = c.degree;
        pc.knots = c.knots;
        pc.poles2d.reserve(c.poles.size());
        for (const math::Point3& p : c.poles) {
          const auto uv = projectUV(srf, p);
          pc.poles2d.push_back(math::Point3{uv.first, uv.second, 0.0});
        }
        pc.origin2d = pc.poles2d.front();
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
      // A straight meridian runs at CONSTANT u. If an endpoint lies ON the axis
      // (a cone apex / sphere pole), its projected angle is indeterminate
      // (atan2(0,0)=0) and must NOT be used as the constant u — otherwise an
      // apex-touching cone-wall meridian collapses onto the u=0 branch instead of
      // its true station, tearing the reconstructed cone (a non-watertight import).
      // Take the constant u from the endpoint FARTHER from the axis (the one with a
      // well-defined angle); fall back to uv0 when both are on-axis (degenerate).
      const double r0 = radialFromAxis(srf, e0);
      const double r1 = radialFromAxis(srf, e1);
      const double uPick = (r1 > r0) ? uv1.first : uv0.first;
      const double u0 = unwrapTo(uPick, uRef);
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
      case K::Ellipse:
        return c.frame.origin + c.frame.x.vec() * (c.radius * std::cos(t)) +
               c.frame.y.vec() * (c.minorRadius * std::sin(t));
      case K::BSpline:
      default:
        if (c.poles.empty()) return c.frame.origin;
        if (c.knots.empty()) return math::bezierPoint({c.poles.data(), c.poles.size()}, t);
        return math::curvePoint(c.degree, {c.poles.data(), c.poles.size()},
                                {c.knots.data(), c.knots.size()}, t);
    }
  }

  // Invert a B-spline surface: find (u,v) with S(u,v) ≈ p. A grid seed over the
  // clamped knot domain picks the nearest sample, then a damped Newton on the
  // first-order orthogonality condition (∂/∂u‖S−p‖²=0, ∂/∂v‖S−p‖²=0) polishes it to
  // fp64 using the analytic surface derivatives (math::surfaceDerivs — no NUMSCI, no
  // OCCT). This is the projection the seam-weld pcurve reconstruction needs for a
  // native B_SPLINE_SURFACE wall (e.g. a spline-profile extrude side face): the rim
  // and seam edges are straight in (u,v), so projecting their endpoints and joining
  // them by the generic linear pcurve arm reproduces the construct helper's pcurve
  // exactly (u=edge param on a rim, v=arc fraction on a seam). Non-goal for the plain
  // box/quadric round-trip, which never carries a B-spline face.
  static std::pair<double, double> projectBSplineUV(const topo::FaceSurface& s,
                                                    const math::Point3& p) {
    if (s.nPolesU <= 0 || s.nPolesV <= 0 || s.poles.empty()) return {0.0, 0.0};
    const math::SurfaceGrid grid{{s.poles.data(), s.poles.size()}, s.nPolesU, s.nPolesV};
    const std::span<const double> ku{s.knotsU.data(), s.knotsU.size()};
    const std::span<const double> kv{s.knotsV.data(), s.knotsV.size()};
    const double u0 = s.knotsU[static_cast<std::size_t>(s.degreeU)];
    const double u1 = s.knotsU[s.knotsU.size() - 1 - static_cast<std::size_t>(s.degreeU)];
    const double v0 = s.knotsV[static_cast<std::size_t>(s.degreeV)];
    const double v1 = s.knotsV[s.knotsV.size() - 1 - static_cast<std::size_t>(s.degreeV)];
    auto eval = [&](double u, double v) {
      return math::surfacePoint(s.degreeU, s.degreeV, grid, ku, kv, u, v);
    };
    auto sq = [&](const math::Point3& a) {
      const math::Vec3 d = a - p;
      return math::dot(d, d);
    };
    // (1) coarse grid seed.
    constexpr int kN = 12;
    double bu = u0, bv = v0, best = std::numeric_limits<double>::max();
    for (int i = 0; i <= kN; ++i)
      for (int j = 0; j <= kN; ++j) {
        const double u = u0 + (u1 - u0) * (double(i) / kN);
        const double v = v0 + (v1 - v0) * (double(j) / kN);
        const double d2 = sq(eval(u, v));
        if (d2 < best) { best = d2; bu = u; bv = v; }
      }
    // (2) damped Newton on the orthogonality condition via analytic derivatives.
    std::array<math::Vec3, 9> der{};  // 3×3 derivative table (maxDeriv=2)
    for (int it = 0; it < 12; ++it) {
      math::surfaceDerivs(s.degreeU, s.degreeV, grid, ku, kv, bu, bv, 2, {der.data(), der.size()});
      const math::Point3 sp{der[0].x, der[0].y, der[0].z};
      const math::Vec3 su = der[1 * 3 + 0], sv = der[0 * 3 + 1];
      const math::Vec3 suu = der[2 * 3 + 0], svv = der[0 * 3 + 2], suv = der[1 * 3 + 1];
      const math::Vec3 r = sp - p;
      const double gu = math::dot(r, su), gv = math::dot(r, sv);
      const double J00 = math::dot(su, su) + math::dot(r, suu);
      const double J01 = math::dot(su, sv) + math::dot(r, suv);
      const double J11 = math::dot(sv, sv) + math::dot(r, svv);
      const double det = J00 * J11 - J01 * J01;
      if (std::fabs(det) < 1e-30) break;
      double du = (J11 * gu - J01 * gv) / det;
      double dv = (J00 * gv - J01 * gu) / det;
      double curr = sq(sp);
      bool improved = false;
      for (int bt = 0; bt < 20; ++bt) {
        const double nu = std::clamp(bu - du, u0, u1);
        const double nv = std::clamp(bv - dv, v0, v1);
        if (sq(eval(nu, nv)) <= curr) { bu = nu; bv = nv; improved = true; break; }
        du *= 0.5; dv *= 0.5;
      }
      if (!improved) break;
    }
    return {bu, bv};
  }

  // Perpendicular distance from a point to the surface's axis (frame Z through
  // origin). Zero ⇒ the point is ON the axis, where the angular u is indeterminate
  // (a cone apex / sphere pole). Used by pcurveFor to pick a well-defined meridian u.
  static double radialFromAxis(const topo::FaceSurface& s, const math::Point3& p) {
    const math::Vec3 d = p - s.frame.origin;
    const double lx = math::dot(d, s.frame.x.vec());
    const double ly = math::dot(d, s.frame.y.vec());
    return std::sqrt(lx * lx + ly * ly);
  }

  // Forward-evaluate an analytic surface at (u,v) — the exact inverse of projectUV,
  // via the elementary parametrizations (elementary.h). Used only by the faithful-
  // reduction guard (a non-analytic kind never reaches here).
  static math::Point3 surfaceValue(const topo::FaceSurface& s, double u, double v) {
    using K = topo::FaceSurface::Kind;
    switch (s.kind) {
      case K::Plane:    return math::Plane{s.frame}.value(u, v);
      case K::Cylinder: return math::Cylinder{s.frame, s.radius}.value(u, v);
      case K::Cone:     return math::Cone{s.frame, s.radius, s.semiAngle}.value(u, v);
      case K::Sphere:   return math::Sphere{s.frame, s.radius}.value(u, v);
      default:          return s.frame.origin;
    }
  }

  // FAITHFUL-REDUCTION GUARD. A point lies on the candidate surface iff projecting it
  // and re-evaluating returns the same point within a scale-relative tolerance. The
  // reader only emits a revolved reduction after the generatrix's defining points pass
  // this — it is the "never fabricate geometry" gate (a mis-classified generatrix whose
  // points do not lie on the quadric DECLINES → OCCT rather than forcing a wrong map).
  static bool pointOnSurface(const topo::FaceSurface& s, const math::Point3& p, double scale) {
    const auto uv = projectUV(s, p);
    return math::distance(p, surfaceValue(s, uv.first, uv.second)) <= 1e-6 * std::max(1.0, scale);
  }
  // Two distinct points fix a straight generatrix (P and one unit-scale step along D).
  static bool lineOnSurface(const topo::EdgeCurve& line, const topo::FaceSurface& s) {
    const double scale = std::max(s.radius, 1.0);
    return pointOnSurface(s, line.frame.origin, scale) &&
           pointOnSurface(s, line.frame.origin + line.frame.x.vec() * scale, scale);
  }
  // Four quadrant points fix a circle generatrix against the candidate sphere.
  static bool circleOnSurface(const topo::EdgeCurve& c, const topo::FaceSurface& s) {
    const double scale = std::max(c.radius, s.radius);
    for (const double t : {0.0, 1.57079632679489662, 3.14159265358979324, 4.71238898038468986}) {
      const math::Point3 p = c.frame.origin + c.frame.x.vec() * (c.radius * std::cos(t)) +
                             c.frame.y.vec() * (c.radius * std::sin(t));
      if (!pointOnSurface(s, p, scale)) return false;
    }
    return true;
  }

  // Project a world point onto a surface's (u,v), inverting the elementary
  // parametrization (elementary.h). Analytic-exact for plane/cylinder/cone/sphere;
  // Newton-projected for a B-spline surface (projectBSplineUV).
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
        return projectBSplineUV(s, p);
      default:
        return {0.0, 0.0};
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
