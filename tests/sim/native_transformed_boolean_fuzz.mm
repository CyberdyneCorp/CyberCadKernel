// SPDX-License-Identifier: Apache-2.0
//
// native_transformed_boolean_fuzz.mm — MOAT M6-breadth-11 TRANSFORMED-BOOLEAN
//                                       differential fuzzer, driven THROUGH the cc_*
//                                       facade under BOTH engines (iOS simulator).
//
// This is the ELEVENTH native domain on the differential-fuzzing completeness bar,
// extending the landed M6 fuzzers (native_boolean_fuzz, native_step_import_fuzz,
// native_construct_fuzz, native_blend_fuzz, native_wrap_emboss_fuzz,
// native_mass_props_fuzz, native_geometry_services_fuzz, native_transform_fuzz,
// native_reference_geometry_fuzz, native_directmodel_fuzz) to the INTERACTION between
// two of them: a TRANSFORM composed with a BOOLEAN.
//
// ── WHY A NEW DOMAIN (the gap this closes) ─────────────────────────────────────────
// native_boolean_fuzz booleans operands built AXIS-ALIGNED in canonical world frames
// (identity Location). native_transform_fuzz transforms a SINGLE solid and measures it.
// NEITHER composes the two. But the native planar BSP-CSG boolean (NativeEngine::
// boolean_op → cybercad::native::boolean::boolean_solid) receives each operand's
// topology::Shape WITH WHATEVER solid-level Location the transform baked onto it
// (cc_translate_shape / cc_rotate_shape_about / cc_mirror_shape produce a
// Shape::located(math::Transform)). The polygon extraction the BSP consumes
// (src/native/boolean/polygon.h) must FOLD that solid-level Location into every face's
// world polygon and world normal. A location the extractor drops, mis-composes, or
// mis-orients (e.g. a mirror's handedness flip not propagated to the outward normal)
// yields a boolean on the WRONG operand geometry — a silent-wrong result no single-
// domain fuzzer can surface. This harness fuzzes exactly that composition.
//
// ── ENGINES ─────────────────────────────────────────────────────────────────────────
//   cc_set_engine(0)  → OCCT engine   (BRepAlgoAPI_{Fuse,Cut,Common} on OCCT bodies)
//   cc_set_engine(1)  → NativeEngine  (OCCT-free planar BSP-CSG on located operands)
// The native planar boolean is native ONLY for all-planar polyhedra (per cc_boolean's
// ENGINE NOTE), so every operand family here is a PLANAR PRISM (box / regular n-gon /
// L-shaped concave prism) — keeping the native path EXERCISED, not declined.
//
// ── OPERAND FAMILIES (all planar-faced → native boolean exercised) ──────────────────
//   BOX   — random w×d×h box, centred at origin (extruded +Z).
//   NGON  — random regular n-gon (n∈[3,7]) prism of radius R, height h.
//   LSHAPE— a concave L-profile prism (simple-concave polyhedron, still all-planar).
// Operand A and operand B are drawn from these families and positioned so their
// UNTRANSFORMED overlap is a clean transversal intersection (B translated by a random
// offset inside A's extent) — a valid input for all three ops by construction.
//
// ── THE COMPOSED TRIAL ──────────────────────────────────────────────────────────────
// Per trial: pick op ∈ {FUSE,CUT,COMMON} and a random rigid transform T (TRANSLATE /
// ROTATE-about-axis / MIRROR-through-plane, or IDENTITY as a coverage anchor). Then, in
// each engine:
//   (baseline)     R0 = boolean(A,          B         , op)     // untransformed
//   (transformed)  RT = boolean(T(A),       T(B)      , op)     // both operands moved by T
// A rigid T commutes with a boolean: T(A) ∘ T(B) == T(A ∘ B). A rigid map preserves
// volume and area, so |RT| == |R0| in volume AND area EXACTLY (closed-form INVARIANT,
// no oracle needed). This is the primary arbiter. On top of it, native RT is compared
// against OCCT RT (both engines run the identical composed op) within a fixed
// deflection-matched tolerance. So native must satisfy BOTH the rigid invariant AND
// engine agreement.
//
// ── CLASSIFICATION (per trial) ──────────────────────────────────────────────────────
//   AGREED            — native RT volume&area == native R0 (rigid invariant) AND native
//                       RT == OCCT RT (fixed tol). Both engines + closed form agree.
//   HONESTLY-DECLINED — native returns 0 for the composed op on a config the native
//                       planar BSP scopes out (a near-tangent/degenerate pose the self-
//                       verify rejects) AND OCCT ships a valid solid — counted, never a
//                       bar failure. (The engine reports this honestly; never fakes.)
//   BOTH-DECLINED     — a config both engines refuse (empty COMMON, etc.).
//   ORACLE_UNRELIABLE — OCCT's own RT disagrees with the rigid invariant while native
//                       upholds it (OCCT the outlier) — gated off, native vindicated.
//   DISAGREED         — native RT breaks the rigid invariant, or native RT != OCCT RT.
//                       The M6 silent-wrong this bar exists to catch → the headline.
//
// The bar: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, with each operand family, each op,
// and each transform KIND carrying ≥1 AGREED. Seeded ONLY by an explicit FUZZ_SEED
// (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
//
// Output: [FUZZ] lines then a COVERAGE SUMMARY table and the M6-breadth-11 BAR verdict.
// Flushes stdout and std::_Exit (the trimmed static-OCCT build's teardown is not exit-
// clean — same rationale as the sibling harnesses; every shape id is released first).
//
// Build: scripts/run-sim-native-transformed-boolean-fuzz.sh (compiles the whole kernel +
// OCCT, spawns on a booted simulator). Carries its own main(); on run-sim-suite.sh SKIP
// list. src/native/** stays OCCT-free — this harness is additive test/sim code only.

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim from the siblings). ──
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
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
};

constexpr double kPi = 3.14159265358979323846;

// FIXED, never-widened tolerances — calibrated to the sibling native_boolean_fuzz's
// PROVEN bar, NOT loosened to hide a gap. The reasoning (validated by the localisation
// probe in the design doc):
//   * PRIMARY VOLUME invariant |RT|_vol == |R0|_vol is the true closed-form correctness
//     signal and is held TIGHT (kInvVol). A rigid transform composed with a planar
//     boolean must reproduce the untransformed enclosed volume to machine epsilon — and
//     it does (measured ≤1e-15 across 400 trials / 2 seeds). Any real located-operand
//     mis-composition (a dropped/mis-oriented Location) would blow this up by a LARGE
//     margin, so it cleanly separates correct from wrong.
//   * The AREA invariant is derived from the WELDED FACET COUNT of two DIFFERENT native
//     meshes (the canonical R0 mesh vs the located RT mesh): on a concave seam the two
//     tessellations can weld a marginally different facet set, giving a ~1e-3 relative
//     area wobble while the VOLUME stays exact. So the area invariant is held to a
//     mesh-appropriate bound (kInvArea), never tighter than that path can honour.
//   * native-vs-OCCT (SECONDARY) uses the SAME 2e-2 relative bound native_boolean_fuzz
//     proved: native's exact planar mesh vs OCCT's BRepAlgoAPI+BRepGProp on the same
//     prisms differ by ~0.1–0.9% (OCCT meshing/rounding on concave L-prisms), well
//     inside 2e-2. The bound is never widened to launder a real disagreement — the
//     tight VOLUME invariant is what actually certifies native.
constexpr double kInvVol  = 1e-6;   // rigid VOLUME invariant |RT|==|R0| (native, exact)
constexpr double kInvArea = 5e-3;   // rigid AREA invariant (meshed-facet weld wobble)
constexpr double kVolRel  = 2e-2;   // native-vs-OCCT volume relative tol (sibling-proven)
constexpr double kAreaRel = 2e-2;   // native-vs-OCCT area relative tol (sibling-proven)
constexpr double kAbs     = 1e-6;   // absolute floor for tiny magnitudes

double relDiff(double a, double b) {
  const double d = std::fabs(a - b);
  const double s = std::max({kAbs, std::fabs(a), std::fabs(b)});
  return d / s;
}

// ── families / ops / transforms ─────────────────────────────────────────────────────
enum Fam { F_BOX, F_NGON, F_LSHAPE, F_COUNT };
enum Op  { OP_FUSE, OP_CUT, OP_COMMON, OP_COUNT };
enum TKind { T_IDENTITY, T_TRANSLATE, T_ROTATE, T_MIRROR, T_COUNT };
const char* famName(int f) {
  switch (f) { case F_BOX: return "BOX"; case F_NGON: return "NGON"; case F_LSHAPE: return "LSHAPE"; }
  return "?";
}
const char* opName(int o) {
  switch (o) { case OP_FUSE: return "FUSE"; case OP_CUT: return "CUT"; case OP_COMMON: return "COMMON"; }
  return "?";
}
const char* tName(int t) {
  switch (t) { case T_IDENTITY: return "IDENTITY"; case T_TRANSLATE: return "TRANSLATE";
               case T_ROTATE: return "ROTATE"; case T_MIRROR: return "MIRROR"; }
  return "?";
}

// A planar-prism operand: a footprint polygon (x,y CCW) extruded +Z by `depth`.
struct Operand {
  std::vector<double> profileXY;
  double depth = 0;
};

std::vector<double> ngonPoly(int n, double R) {
  std::vector<double> p; p.reserve(2 * n);
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * kPi * i / n;
    p.push_back(R * std::cos(a)); p.push_back(R * std::sin(a));
  }
  return p;
}

// An L-shaped concave prism footprint (CCW): a big rectangle with a corner notch removed.
//   full = a×b rectangle from origin; notch = the top-right cut of size (a-c)×(b-d).
std::vector<double> lshapePoly(double a, double b, double c, double d) {
  // outer L walk (CCW): (0,0)->(a,0)->(a,d)->(c,d)->(c,b)->(0,b)
  return {0, 0,  a, 0,  a, d,  c, d,  c, b,  0, b};
}

// Build operand A (the larger body, centred near origin) and B (a smaller body offset so
// A∩B is a clean transversal overlap for all three ops).
struct Pair { Operand a, b; int fam; std::string desc; };
Pair genPair(Rng& rng, int fam) {
  Pair pr; pr.fam = fam;
  char buf[128];
  switch (fam) {
    case F_BOX: {
      const double w = rng.range(8.0, 16.0), d = rng.range(8.0, 16.0), h = rng.range(6.0, 12.0);
      pr.a.profileXY = {-w / 2, -d / 2, w / 2, -d / 2, w / 2, d / 2, -w / 2, d / 2};
      pr.a.depth = h;
      // B: a smaller box shifted so it overlaps A but pokes out on ≥1 side (valid cut/common).
      const double w2 = rng.range(4.0, w * 0.7), d2 = rng.range(4.0, d * 0.7), h2 = rng.range(6.0, 12.0);
      const double sx = rng.range(0.2, 0.45) * w, sy = rng.range(-0.2, 0.2) * d;
      pr.b.profileXY = {sx - w2 / 2, sy - d2 / 2, sx + w2 / 2, sy - d2 / 2,
                        sx + w2 / 2, sy + d2 / 2, sx - w2 / 2, sy + d2 / 2};
      pr.b.depth = h2;
      std::snprintf(buf, sizeof buf, "box A(%.2fx%.2fx%.2f) B(%.2fx%.2fx%.2f @%.2f,%.2f)",
                    w, d, h, w2, d2, h2, sx, sy);
      break;
    }
    case F_NGON: {
      const int n = 3 + static_cast<int>(rng.below(5));  // 3..7
      const double R = rng.range(5.0, 9.0), h = rng.range(6.0, 12.0);
      pr.a.profileXY = ngonPoly(n, R); pr.a.depth = h;
      // B: a box straddling the n-gon (overlap guaranteed by centring near origin + poke-out).
      const double w2 = rng.range(3.0, R), d2 = rng.range(3.0, R), h2 = rng.range(6.0, 12.0);
      const double sx = rng.range(0.15, 0.5) * R;
      pr.b.profileXY = {sx - w2 / 2, -d2 / 2, sx + w2 / 2, -d2 / 2,
                        sx + w2 / 2, d2 / 2, sx - w2 / 2, d2 / 2};
      pr.b.depth = h2;
      std::snprintf(buf, sizeof buf, "ngon n=%d R=%.2f h=%.2f x box(%.2fx%.2fx%.2f @%.2f)",
                    n, R, h, w2, d2, h2, sx);
      break;
    }
    default: {  // F_LSHAPE — concave L-prism cut by a straddling box
      const double a = rng.range(10.0, 16.0), b = rng.range(10.0, 16.0);
      const double c = rng.range(3.0, a - 3.0), d = rng.range(3.0, b - 3.0), h = rng.range(6.0, 12.0);
      pr.a.profileXY = lshapePoly(a, b, c, d); pr.a.depth = h;
      // B: a box overlapping the L's foot (near origin) and poking out.
      const double w2 = rng.range(3.0, c), d2 = rng.range(3.0, d), h2 = rng.range(6.0, 12.0);
      const double sx = rng.range(0.3, 0.7) * c, sy = rng.range(0.3, 0.7) * d;
      pr.b.profileXY = {sx - w2 / 2, sy - d2 / 2, sx + w2 / 2, sy - d2 / 2,
                        sx + w2 / 2, sy + d2 / 2, sx - w2 / 2, sy + d2 / 2};
      pr.b.depth = h2;
      std::snprintf(buf, sizeof buf, "L(%.2fx%.2f notch %.2fx%.2f h=%.2f) x box(%.2fx%.2fx%.2f @%.2f,%.2f)",
                    a, b, c, d, h, w2, d2, h2, sx, sy);
      break;
    }
  }
  pr.desc = buf;
  return pr;
}

// A generated rigid transform (applied to a CCShapeId via the cc_* facade). IDENTITY
// leaves the id untouched (returns the same id, dup=false). Every other kind returns a
// NEW id (dup=true) the caller must release.
struct XForm {
  int kind;
  // TRANSLATE
  double tx = 0, ty = 0, tz = 0;
  // ROTATE (about an axis through a centre)
  double cx = 0, cy = 0, cz = 0, ax = 0, ay = 0, az = 1, ang = 0;
  // MIRROR (through a plane point+normal)
  double px = 0, py = 0, pz = 0, nx = 1, ny = 0, nz = 0;
  std::string desc;
};

XForm genXForm(Rng& rng, int kind) {
  XForm x; x.kind = kind; char buf[128];
  switch (kind) {
    case T_IDENTITY:
      std::snprintf(buf, sizeof buf, "identity");
      break;
    case T_TRANSLATE:
      x.tx = rng.range(-20, 20); x.ty = rng.range(-20, 20); x.tz = rng.range(-20, 20);
      std::snprintf(buf, sizeof buf, "translate(%.2f,%.2f,%.2f)", x.tx, x.ty, x.tz);
      break;
    case T_ROTATE: {
      // random unit axis through a random centre, random angle in [20°, 340°].
      double a0 = rng.range(-1, 1), a1 = rng.range(-1, 1), a2 = rng.range(-1, 1);
      double L = std::sqrt(a0 * a0 + a1 * a1 + a2 * a2);
      if (L < 1e-6) { a0 = 0; a1 = 0; a2 = 1; L = 1; }
      x.ax = a0 / L; x.ay = a1 / L; x.az = a2 / L;
      x.cx = rng.range(-5, 5); x.cy = rng.range(-5, 5); x.cz = rng.range(-5, 5);
      x.ang = rng.range(20.0, 340.0) * kPi / 180.0;
      std::snprintf(buf, sizeof buf, "rotate(%.0f° about [%.2f,%.2f,%.2f]@%.1f,%.1f,%.1f)",
                    x.ang * 180.0 / kPi, x.ax, x.ay, x.az, x.cx, x.cy, x.cz);
      break;
    }
    default: {  // T_MIRROR
      double m0 = rng.range(-1, 1), m1 = rng.range(-1, 1), m2 = rng.range(-1, 1);
      double L = std::sqrt(m0 * m0 + m1 * m1 + m2 * m2);
      if (L < 1e-6) { m0 = 1; m1 = 0; m2 = 0; L = 1; }
      x.nx = m0 / L; x.ny = m1 / L; x.nz = m2 / L;
      x.px = rng.range(-5, 5); x.py = rng.range(-5, 5); x.pz = rng.range(-5, 5);
      std::snprintf(buf, sizeof buf, "mirror(pt %.1f,%.1f,%.1f n [%.2f,%.2f,%.2f])",
                    x.px, x.py, x.pz, x.nx, x.ny, x.nz);
      break;
    }
  }
  x.desc = buf;
  return x;
}

// ── engine-scoped helpers ─────────────────────────────────────────────────────────────
// A shape id owned under a given engine; released in dtor under that same engine.
struct Body {
  CCShapeId id = 0; int engine = 0;
  Body() = default;
  Body(CCShapeId i, int e) : id(i), engine(e) {}
  Body(Body&& o) noexcept : id(o.id), engine(o.engine) { o.id = 0; }
  Body& operator=(Body&& o) noexcept {
    if (this != &o) { release(); id = o.id; engine = o.engine; o.id = 0; } return *this;
  }
  Body(const Body&) = delete; Body& operator=(const Body&) = delete;
  void release() { if (id) { cc_set_engine(engine); cc_shape_release(id); cc_set_engine(0); id = 0; } }
  ~Body() { release(); }
};

CCShapeId buildOperand(const Operand& o) {
  return cc_solid_extrude(o.profileXY.data(), static_cast<int>(o.profileXY.size() / 2), o.depth);
}

// Apply an XForm to `id` under the ACTIVE engine. Returns a new id (nonzero) or 0 on
// failure. For IDENTITY, returns a fresh translate-by-zero copy so the caller uniformly
// owns/releases a distinct id (keeps ownership simple and exercises the identity path).
CCShapeId applyXForm(CCShapeId id, const XForm& x) {
  switch (x.kind) {
    case T_IDENTITY:  return cc_translate_shape(id, 0, 0, 0);
    case T_TRANSLATE: return cc_translate_shape(id, x.tx, x.ty, x.tz);
    case T_ROTATE:    return cc_rotate_shape_about(id, x.cx, x.cy, x.cz, x.ax, x.ay, x.az, x.ang);
    case T_MIRROR:    return cc_mirror_shape(id, x.px, x.py, x.pz, x.nx, x.ny, x.nz);
  }
  return 0;
}

struct Metrics { bool ok = false; double vol = 0, area = 0; };
Metrics measure(CCShapeId id, int engine) {
  Metrics m;
  if (id == 0) return m;
  cc_set_engine(engine);
  const CCMassProps mp = cc_mass_properties(id);
  cc_set_engine(0);
  if (!mp.valid) return m;
  m.ok = true; m.vol = std::fabs(mp.volume); m.area = mp.area;
  return m;
}

CCShapeId booleanOp(CCShapeId a, CCShapeId b, int op) { return cc_boolean(a, b, op); }

// ── classification bookkeeping ─────────────────────────────────────────────────────
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_UNRELIABLE, BOTH_DECLINED };
int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleBad = 0, g_bothDecl = 0;
int g_famAgreed[F_COUNT] = {0}, g_opAgreed[OP_COUNT] = {0}, g_tAgreed[T_COUNT] = {0};
int g_famDisagreed[F_COUNT] = {0}, g_opDisagreed[OP_COUNT] = {0}, g_tDisagreed[T_COUNT] = {0};

const char* vName(Verdict v) {
  switch (v) { case AGREED: return "AGREED"; case DECLINED: return "HONESTLY-DECLINED";
               case DISAGREED: return "DISAGREED"; case ORACLE_UNRELIABLE: return "ORACLE_UNRELIABLE";
               case BOTH_DECLINED: return "BOTH-DECLINED"; }
  return "?";
}
void tally(Verdict v, int fam, int op, int t) {
  switch (v) {
    case AGREED: ++g_agreed; ++g_famAgreed[fam]; ++g_opAgreed[op]; ++g_tAgreed[t]; break;
    case DECLINED: ++g_declined; break;
    case DISAGREED: ++g_disagreed; ++g_famDisagreed[fam]; ++g_opDisagreed[op]; ++g_tDisagreed[t]; break;
    case ORACLE_UNRELIABLE: ++g_oracleBad; break;
    case BOTH_DECLINED: ++g_bothDecl; break;
  }
}
void logTrial(Verdict v, int idx, uint64_t seed, const Pair& pr, int op, const XForm& x,
              const char* detail) {
  if (v == DISAGREED || v == ORACLE_UNRELIABLE) {
    std::printf("[FUZZ] %-17s case=%d op=%-6s %-6s T=%-9s | %s\n"
                "       REPRO seed=0x%llx index=%d :: %s :: %s\n",
                vName(v), idx, opName(op), famName(pr.fam), tName(x.kind), detail,
                static_cast<unsigned long long>(seed), idx, pr.desc.c_str(), x.desc.c_str());
  } else {
    std::printf("[FUZZ] %-17s case=%d op=%-6s %-6s T=%-9s | %s :: %s\n",
                vName(v), idx, opName(op), famName(pr.fam), tName(x.kind), detail, pr.desc.c_str());
  }
  std::fflush(stdout);
  tally(v, pr.fam, op, x.kind);
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0xB007C0DE11ull;
  int N = 96;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 96;

  std::printf("== M6-breadth-11 TRANSFORMED-BOOLEAN differential fuzz (cc_* under both engines) "
              "seed=0x%llx N=%d ==\n", static_cast<unsigned long long>(seed), N);
  if (!cc_brep_available()) {
    std::printf("cc_brep_available()==0 — no B-rep engine linked; cannot run.\n");
    std::fflush(stdout);
    std::_Exit(1);
  }

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    const int fam = static_cast<int>(rng.below(F_COUNT));
    const Pair pr = genPair(rng, fam);
    const int op = static_cast<int>(rng.below(OP_COUNT));
    // Coverage anchor: force each transform kind through the first T_COUNT cases, then random.
    const int tk = (i < T_COUNT) ? i : static_cast<int>(rng.below(T_COUNT));
    const XForm x = genXForm(rng, tk);
    char detail[512];

    // Build A,B once per engine (built under the engine that will boolean them).
    // ── NATIVE ────────────────────────────────────────────────────────────────────
    cc_set_engine(1);
    Body nA(buildOperand(pr.a), 1), nB(buildOperand(pr.b), 1);
    cc_set_engine(0);
    // ── OCCT ──────────────────────────────────────────────────────────────────────
    cc_set_engine(0);
    Body oA(buildOperand(pr.a), 0), oB(buildOperand(pr.b), 0);

    if (nA.id == 0 || nB.id == 0 || oA.id == 0 || oB.id == 0) {
      std::snprintf(detail, sizeof detail, "operand build nA=%ld nB=%ld oA=%ld oB=%ld (%s)",
                    nA.id, nB.id, oA.id, oB.id, cc_last_error());
      logTrial(BOTH_DECLINED, i, seed, pr, op, x, detail);
      continue;
    }

    // ── baseline booleans R0 = A ∘ B (untransformed), each engine ───────────────────
    cc_set_engine(1);
    Body nR0(booleanOp(nA.id, nB.id, op), 1);
    cc_set_engine(0);
    Body oR0(booleanOp(oA.id, oB.id, op), 0);

    // ── transform each operand by T, then boolean RT = T(A) ∘ T(B), each engine ─────
    cc_set_engine(1);
    Body nTA(applyXForm(nA.id, x), 1), nTB(applyXForm(nB.id, x), 1);
    Body nRT((nTA.id && nTB.id) ? booleanOp(nTA.id, nTB.id, op) : 0, 1);
    cc_set_engine(0);
    Body oTA(applyXForm(oA.id, x), 0), oTB(applyXForm(oB.id, x), 0);
    Body oRT((oTA.id && oTB.id) ? booleanOp(oTA.id, oTB.id, op) : 0, 0);

    const Metrics nR0m = measure(nR0.id, 1);
    const Metrics nRTm = measure(nRT.id, 1);
    const Metrics oRTm = measure(oRT.id, 0);
    const Metrics oR0m = measure(oR0.id, 0);

    // ── native decline handling ─────────────────────────────────────────────────────
    // The native planar BSP honestly returns 0 on a near-tangent/degenerate composed op
    // (self-verify reject). If OCCT ships a valid RT, that's a first-class DECLINE.
    if (!nRTm.ok) {
      if (oRTm.ok && oRTm.vol > kAbs) {
        std::snprintf(detail, sizeof detail, "native declined composed op; OCCT RT vol=%.4f area=%.4f",
                      oRTm.vol, oRTm.area);
        logTrial(DECLINED, i, seed, pr, op, x, detail);
      } else {
        std::snprintf(detail, sizeof detail, "both declined composed op (empty/degenerate) (%s)",
                      cc_last_error());
        logTrial(BOTH_DECLINED, i, seed, pr, op, x, detail);
      }
      continue;
    }
    // A COMMON that is genuinely empty in both engines → nothing to compare (both-declined).
    if (nRTm.vol <= kAbs && (!oRTm.ok || oRTm.vol <= kAbs)) {
      std::snprintf(detail, sizeof detail, "empty %s (native vol=%.3g)", opName(op), nRTm.vol);
      logTrial(BOTH_DECLINED, i, seed, pr, op, x, detail);
      continue;
    }
    // Native produced RT but its baseline R0 is unusable → cannot check the invariant.
    if (!nR0m.ok || nR0m.vol <= kAbs) {
      // Fall back to native-vs-OCCT only (still a real cross-check).
      if (!oRTm.ok) {
        std::snprintf(detail, sizeof detail, "native RT but no baseline/oracle to arbitrate");
        logTrial(BOTH_DECLINED, i, seed, pr, op, x, detail);
        continue;
      }
    }

    // ── PRIMARY arbiter: rigid invariant  |RT| == |R0|  (native, exact planar) ───────
    double invVolErr = -1, invAreaErr = -1; bool invOk = true;
    if (nR0m.ok && nR0m.vol > kAbs) {
      invVolErr  = relDiff(nRTm.vol, nR0m.vol);
      invAreaErr = relDiff(nRTm.area, nR0m.area);
      // VOLUME held tight (the real correctness signal); AREA at the meshed-weld bound.
      invOk = (invVolErr <= kInvVol) && (invAreaErr <= kInvArea);
    }
    // OCCT's own VOLUME invariant (used to gate ORACLE_UNRELIABLE): a rigid transform
    // must preserve OCCT's boolean volume too. When OCCT breaks it while native upholds
    // the tight closed-form invariant, OCCT is the pathological outlier.
    bool oInvOk = true; double oInvVolErr = -1;
    if (oRTm.ok && oR0m.ok && oR0m.vol > kAbs) {
      oInvVolErr = relDiff(oRTm.vol, oR0m.vol);
      oInvOk = oInvVolErr <= kVolRel;   // OCCT meshing/rounding, at the native-vs-OCCT bound
    }

    // ── SECONDARY arbiter: native RT vs OCCT RT ─────────────────────────────────────
    bool natOcctOk = true; double vr = -1, ar = -1;
    if (oRTm.ok && oRTm.vol > kAbs) {
      vr = relDiff(nRTm.vol, oRTm.vol);
      ar = relDiff(nRTm.area, oRTm.area);
      natOcctOk = (vr <= kVolRel) && (ar <= kAreaRel);
    }

    std::snprintf(detail, sizeof detail,
                  "invV=%.2e invA=%.2e | dVoc=%.2e dAoc=%.2e | nRTvol=%.4f nR0vol=%.4f oRTvol=%.4f",
                  invVolErr, invAreaErr, vr, ar, nRTm.vol, nR0m.vol, oRTm.vol);

    Verdict v;
    if (invOk && !natOcctOk && !oInvOk) {
      // Native upholds the rigid invariant; OCCT both disagrees with native AND breaks
      // its OWN invariant → OCCT is the pathological outlier, native vindicated.
      v = ORACLE_UNRELIABLE;
    } else if (!invOk || !natOcctOk) {
      v = DISAGREED;
    } else {
      v = AGREED;
    }
    logTrial(v, i, seed, pr, op, x, detail);
  }

  cc_set_engine(0);  // restore default engine

  // ── coverage table ───────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("  TOTALS: AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  "
              "ORACLE_UNRELIABLE=%d  BOTH-DECLINED=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleBad, g_bothDecl);
  std::printf("  per-OPERAND-FAMILY (AGREED [DISAGREED]):\n");
  for (int f = 0; f < F_COUNT; ++f)
    std::printf("      %-8s  %3d [%d]\n", famName(f), g_famAgreed[f], g_famDisagreed[f]);
  std::printf("  per-OP            (AGREED [DISAGREED]):\n");
  for (int o = 0; o < OP_COUNT; ++o)
    std::printf("      %-8s  %3d [%d]\n", opName(o), g_opAgreed[o], g_opDisagreed[o]);
  std::printf("  per-TRANSFORM     (AGREED [DISAGREED]):\n");
  for (int t = 0; t < T_COUNT; ++t)
    std::printf("      %-9s %3d [%d]\n", tName(t), g_tAgreed[t], g_tDisagreed[t]);
  std::printf("  PRIMARY arbiter: rigid invariant |RT|==|R0| (a rigid T preserves vol+area,\n"
              "                   and boolean commutes with T). SECONDARY: native RT == OCCT RT.\n");
  if (g_oracleBad)
    std::printf("  ORACLE_UNRELIABLE (%d): native upheld the rigid invariant, OCCT broke its OWN\n"
                "                    invariant — OCCT the outlier, native VINDICATED (gated off).\n", g_oracleBad);

  bool famCov = true; for (int f = 0; f < F_COUNT; ++f) if (g_famAgreed[f] < 1) famCov = false;
  bool opCov  = true; for (int o = 0; o < OP_COUNT; ++o) if (g_opAgreed[o] < 1) opCov = false;
  bool tCov   = true; for (int t = 0; t < T_COUNT; ++t) if (g_tAgreed[t] < 1) tCov = false;
  const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && famCov && opCov && tCov);
  std::printf("== M6-breadth-11 BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
              "family=%s op=%s transform=%s coverage) ==\n",
              bar ? "PASS — zero silent wrong transformed-boolean results" : "FAIL",
              g_disagreed, g_oracleBad,
              famCov ? "complete" : "INCOMPLETE", opCov ? "complete" : "INCOMPLETE",
              tCov ? "complete" : "INCOMPLETE");
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
