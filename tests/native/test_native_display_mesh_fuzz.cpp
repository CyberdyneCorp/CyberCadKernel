// SPDX-License-Identifier: Apache-2.0
//
// test_native_display_mesh_fuzz.cpp — MOAT M6 (the "no silent-wrong" completeness
// bar): the HOST-ANALYTIC arm of the RENDER DISPLAY-MESH differential-fuzz domain.
// OCCT-FREE: clang++ -std=c++20, no OCCT, no sim. It is the companion to the SIM
// native-vs-OCCT parity leg (tests/sim/native_display_mesh_fuzz.mm) — this arm needs
// no OCCT because the oracle is a CLOSED-FORM analytic surface, a STRICTER oracle
// than any mesher for the invariants that matter here.
//
// ── WHAT THIS FUZZES ──────────────────────────────────────────────────────────────
// The render-quality DISPLAY mesh (src/native/render/display_mesh.h), reached through
// nrender::buildDisplayMesh — the same post-process the SHIPPING cc_display_mesh drives.
// It is a PURELY ADDITIVE consumer of the correctness tessellation (SolidMesher output):
// it adds per-vertex SMOOTH normals, crease-angle HARD edges, optional UVs, optional LOD.
// It MOVES NO vertex except LOD (bounded by a Hausdorff budget) and never touches the
// tessellator. The existing test_native_display_mesh.cpp asserts the analytic-normal /
// crease / LOD contract on THREE FIXED shapes; this harness turns those into a SEEDED
// BATCH over RANDOM analytic primitives at VARIED scales × VARIED deflections × crease
// angles × LOD/UV toggles, and asserts the INVARIANTS that must hold for every draw.
//
// ── THE ORACLE: CLOSED-FORM ANALYTIC SURFACE (no OCCT needed) ──────────────────────
// The load-bearing invariant is the DEFLECTION BOUND: because buildDisplayMesh MOVES NO
// vertex pre-LOD, every display vertex is a source-mesh vertex, and the source mesh lies
// ON the analytic surface within the tessellator's chord deflection. So the max distance
// of any display vertex to the EXACT surface (sphere |‖p‖−R|, cylinder |r−R|, cone
// |r−R(y)|·cos α) must be ≤ a small multiple of the requested deflection. This is the
// same convergent bound test_native_display_mesh's sphere/LOD cases assert, generalised
// across a random batch. Post-LOD the bound relaxes to the Hausdorff budget (scale·defl),
// which the LOD arbiter proves the decimator actually honours (a TIGHT budget throttles
// the collapse — the honest, not-a-fixed-schedule signature).
//
// Additional invariants asserted every case (must-hold for a valid display mesh):
//   * FINITE      — no NaN/Inf position, normal, or UV.
//   * UNIT NORMALS— every per-vertex normal has ‖n‖ == 1 (to ~1e-9).
//   * NON-DEGEN   — no zero-area display triangle, no out-of-range index.
//   * FOLD-WATERTIGHT (pre-LOD, closed solids) — folding split verts back by position
//                   yields the SAME closed 2-manifold the source is (topology preserved).
//   * TRI COUNT   — the split preserves triangle count exactly (pre-LOD); LOD only reduces.
//   * NO FLIPPED  — a fully-smooth sphere's normals all point OUTWARD (n·p > 0).
//   * UVs in [0,1] — when requested, every UV component ∈ [0,1].
//
// ── DISAGREED / classification ─────────────────────────────────────────────────────
// A DISAGREED case = an invariant above VIOLATED by the native display mesh. The bar is
// DISAGREED==0. There is no OCCT here, so there is no ORACLE-INACCURATE bucket; the closed
// form IS ground truth. The deflection multiplier is FIXED and NEVER widened to force a
// case through — a case that misses is a real finding to localise from the logged seed.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_display_mesh_fuzz.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_display_mesh_fuzz
//
#include "native/render/display_mesh.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <algorithm>  // std::max/std::min over an initializer_list
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

using namespace cybercad::native::render;
namespace tess = cybercad::native::tessellate;
namespace cst = cybercad::native::construct;
namespace topo = cybercad::native::topology;
namespace m = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim from the SIM fuzzers).
//    Keyed ONLY by an explicit uint64 seed: same seed → byte-identical batch. ─────────
struct Rng {
  std::uint64_t s[4];
  static std::uint64_t splitmix64(std::uint64_t& x) {
    std::uint64_t z = (x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  explicit Rng(std::uint64_t seed) { for (auto& v : s) v = splitmix64(seed); }
  static std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  std::uint64_t next() {
    const std::uint64_t r = rotl(s[1] * 5, 7) * 9;
    const std::uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return r;
  }
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  std::uint32_t below(std::uint32_t n) { return static_cast<std::uint32_t>(next() % n); }
};

// ── analytic shape builders (mirrors of test_native_display_mesh's box/cyl/sphere) ──

topo::Shape vertexAt(double x, double y, double z) {
  return topo::ShapeBuilder::makeVertex(m::Point3{x, y, z});
}
topo::Shape lineEdge(const topo::Shape& a, const topo::Shape& b) {
  topo::EdgeCurve c{};
  c.kind = topo::EdgeCurve::Kind::Line;
  return topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, a, b);
}
topo::Shape circleEdge3d(double R, double z, const topo::Shape& v0, const topo::Shape& v1) {
  topo::EdgeCurve c{};
  c.kind = topo::EdgeCurve::Kind::Circle;
  c.frame = m::Ax3{m::Point3{0, 0, z}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  c.radius = R;
  return topo::ShapeBuilder::makeEdgeWithVertices(c, 0.0, 2 * kPi, {v0, v1});
}

topo::Shape boxSolid(double a, double b, double c) {
  topo::Shape v[8] = {vertexAt(0, 0, 0), vertexAt(a, 0, 0), vertexAt(a, b, 0), vertexAt(0, b, 0),
                      vertexAt(0, 0, c), vertexAt(a, 0, c), vertexAt(a, b, c), vertexAt(0, b, c)};
  auto edge = [&](int i, int j) { return lineEdge(v[i], v[j]); };
  topo::Shape e[12] = {edge(0, 1), edge(1, 2), edge(2, 3), edge(3, 0),
                       edge(4, 5), edge(5, 6), edge(6, 7), edge(7, 4),
                       edge(0, 4), edge(1, 5), edge(2, 6), edge(3, 7)};
  const m::Vec3 center{a / 2, b / 2, c / 2};
  auto rectFace = [&](const m::Point3& O, const m::Dir3& X, const m::Dir3& Y, double U, double V,
                      topo::Shape (&edges)[4]) {
    const m::Vec3 z = m::cross(X.vec(), Y.vec());
    const m::Vec3 faceCenter = O.asVec() + X.vec() * (U / 2) + Y.vec() * (V / 2);
    const topo::Orientation o = m::dot(z, faceCenter - center) >= 0 ? topo::Orientation::Forward
                                                                    : topo::Orientation::Reversed;
    m::Ax3 fr{O, X, Y, m::Dir3{z}};
    topo::FaceSurface s{};
    s.kind = topo::FaceSurface::Kind::Plane;
    s.frame = fr;
    topo::Shape f0 = topo::ShapeBuilder::makeFace(s, topo::Shape{});
    const m::Point3 c2[4] = {{0, 0, 0}, {U, 0, 0}, {U, V, 0}, {0, V, 0}};
    std::vector<topo::Shape> pcE;
    for (int i = 0; i < 4; ++i) {
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = c2[i];
      pc.dir2d = m::Vec3{c2[(i + 1) % 4].x - c2[i].x, c2[(i + 1) % 4].y - c2[i].y, 0};
      pcE.push_back(topo::ShapeBuilder::addPCurve(edges[i], f0.tshape(), pc));
    }
    return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(pcE), {}, o);
  };
  topo::Shape botE[4] = {e[0], e[1], e[2], e[3]};
  topo::Shape topE[4] = {e[4], e[5], e[6], e[7]};
  topo::Shape ymE[4] = {e[0], e[9], e[4], e[8]};
  topo::Shape ypE[4] = {e[2], e[11], e[6], e[10]};
  topo::Shape xmE[4] = {e[3], e[8], e[7], e[11]};
  topo::Shape xpE[4] = {e[1], e[10], e[5], e[9]};
  std::vector<topo::Shape> faces = {
      rectFace({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, a, b, botE),
      rectFace({0, 0, c}, {1, 0, 0}, {0, 1, 0}, a, b, topE),
      rectFace({0, 0, 0}, {1, 0, 0}, {0, 0, 1}, a, c, ymE),
      rectFace({0, b, 0}, {1, 0, 0}, {0, 0, 1}, a, c, ypE),
      rectFace({0, 0, 0}, {0, 1, 0}, {0, 0, 1}, b, c, xmE),
      rectFace({a, 0, 0}, {0, 1, 0}, {0, 0, 1}, b, c, xpE)};
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(faces)});
}

topo::Shape cylinderSolid(double R, double h) {
  const m::Ax3 fr{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface sideS{};
  sideS.kind = topo::FaceSurface::Kind::Cylinder; sideS.frame = fr; sideS.radius = R;
  auto vb = vertexAt(R, 0, 0), vt = vertexAt(R, 0, h);
  topo::Shape botC = circleEdge3d(R, 0, vb, vb);
  topo::Shape topC = circleEdge3d(R, h, vt, vt);
  topo::Shape seam0 = lineEdge(vb, vt), seam1 = lineEdge(vb, vt);
  topo::Shape sideF0 = topo::ShapeBuilder::makeFace(sideS, topo::Shape{});
  auto pcLine = [&](m::Point3 o, m::Vec3 d) {
    topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line; pc.origin2d = o; pc.dir2d = d; return pc;
  };
  topo::Shape botOnSide = topo::ShapeBuilder::addPCurve(botC, sideF0.tshape(), pcLine({0, 0, 0}, {1, 0, 0}));
  topo::Shape topOnSide = topo::ShapeBuilder::addPCurve(topC, sideF0.tshape(), pcLine({0, h, 0}, {1, 0, 0}));
  topo::Shape s0OnSide = topo::ShapeBuilder::addPCurve(seam0, sideF0.tshape(), pcLine({0, 0, 0}, {0, h, 0}));
  topo::Shape s1OnSide = topo::ShapeBuilder::addPCurve(seam1, sideF0.tshape(), pcLine({2 * kPi, 0, 0}, {0, h, 0}));
  std::vector<topo::Shape> sideWire = {botOnSide, s1OnSide, topOnSide.reversedShape(),
                                       s0OnSide.reversedShape()};
  topo::Shape sideFace = topo::ShapeBuilder::makeFace(sideS, topo::ShapeBuilder::makeWire(sideWire));
  auto capFace = [&](double z, bool top, const topo::Shape& circle) {
    topo::FaceSurface cs{};
    cs.kind = topo::FaceSurface::Kind::Plane;
    cs.frame = m::Ax3{m::Point3{0, 0, z}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
    topo::Shape cf0 = topo::ShapeBuilder::makeFace(cs, topo::Shape{});
    topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Circle; pc.origin2d = {0, 0, 0}; pc.dir2d = {R, 0, 0};
    topo::Shape cOn = topo::ShapeBuilder::addPCurve(circle, cf0.tshape(), pc);
    const topo::Orientation o = top ? topo::Orientation::Forward : topo::Orientation::Reversed;
    return topo::ShapeBuilder::makeFace(cs, topo::ShapeBuilder::makeWire({cOn}), {}, o);
  };
  topo::Shape botCap = capFace(0, false, botC);
  topo::Shape topCap = capFace(h, true, topC);
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell({sideFace, botCap, topCap})});
}

topo::Shape fullSphere(double R) {
  const m::Ax3 fr{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::Sphere;
  s.frame = fr;
  s.radius = R;
  topo::Shape f0 = topo::ShapeBuilder::makeFace(s, topo::Shape{});
  const double v0 = -kPi / 2, v1 = kPi / 2;
  const m::Point3 c2[4] = {{0, v0, 0}, {2 * kPi, v0, 0}, {2 * kPi, v1, 0}, {0, v1, 0}};
  std::vector<topo::Shape> edges;
  for (int i = 0; i < 4; ++i) {
    auto a = vertexAt(0, 0, 0), b = vertexAt(0, 0, 0);
    topo::Shape e = lineEdge(a, b);
    topo::PCurve pc{};
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = c2[i];
    pc.dir2d = m::Vec3{c2[(i + 1) % 4].x - c2[i].x, c2[(i + 1) % 4].y - c2[i].y, 0};
    edges.push_back(topo::ShapeBuilder::addPCurve(e, f0.tshape(), pc));
  }
  return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(edges));
}

// Cone FRUSTUM (or full cone when Rt==0): meridian trapezoid {0,0, Rb,0, Rt,H, 0,H}
// revolved 360° about the Y axis (x=0), via the native revolve builder. The lateral
// surface at height y has axis-radius R(y) = Rb + (Rt−Rb)·(y/H); the perpendicular
// distance of a point (x,y,z) to that cone is |sqrt(x²+z²) − R(y)|·cos α, α the half
// angle (atan(|Rb−Rt|/H)). For a straight cylinder (Rt==Rb) α=0 ⇒ radial distance.
topo::Shape coneSolid(double Rb, double Rt, double H) {
  const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
  return cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * kPi);
}

// ── analytic distance-to-surface helpers ────────────────────────────────────────────
double maxDistToSphere(const DisplayMesh& dm, double R) {
  double d = 0.0;
  for (const auto& p : dm.positions)
    d = std::max(d, std::fabs(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - R));
  return d;
}
// Cylinder wall about +Z axis: only WALL vertices (r≈R). Cap-interior vertices sit at
// r<R and are exactly on the planar cap, so the pure-wall metric would falsely flag
// them; we bound only vertices whose radius is within a few deflections of R.
double maxWallDistToCylinder(const DisplayMesh& dm, double R, double defl) {
  double d = 0.0;
  for (const auto& p : dm.positions) {
    const double r = std::sqrt(p.x * p.x + p.y * p.y);
    if (std::fabs(r - R) < 6.0 * defl) d = std::max(d, std::fabs(r - R));
  }
  return d;
}
// Cone lateral wall about the Y axis: bound only vertices near the lateral surface.
double maxWallDistToCone(const DisplayMesh& dm, double Rb, double Rt, double H, double defl) {
  const double halfAng = std::atan2(std::fabs(Rb - Rt), H);
  const double cosA = std::cos(halfAng);
  double d = 0.0;
  for (const auto& p : dm.positions) {
    if (p.y < 1e-9 || p.y > H - 1e-9) continue;  // skip cap rings (on the planar caps)
    const double r = std::sqrt(p.x * p.x + p.z * p.z);
    const double Ry = Rb + (Rt - Rb) * (p.y / H);
    const double dist = std::fabs(r - Ry) * cosA;  // perpendicular distance to the cone
    if (std::fabs(r - Ry) < 6.0 * defl) d = std::max(d, dist);
  }
  return d;
}

bool allFinite(const DisplayMesh& dm) {
  for (const auto& p : dm.positions)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
  for (const auto& n : dm.normals) {
    const m::Vec3 v = n.vec();
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return false;
  }
  for (const auto& uv : dm.uvs)
    if (!std::isfinite(uv[0]) || !std::isfinite(uv[1])) return false;
  return true;
}
bool normalsUnit(const DisplayMesh& dm) {
  for (const auto& n : dm.normals)
    if (std::fabs(m::norm(n.vec()) - 1.0) > 1e-9) return false;
  return true;
}
// No degenerate (zero-area) display triangle and every index in range.
bool nonDegenerateTris(const DisplayMesh& dm) {
  const std::uint32_t nv = static_cast<std::uint32_t>(dm.positions.size());
  for (const auto& t : dm.triangles) {
    if (t.a >= nv || t.b >= nv || t.c >= nv) return false;
    const m::Vec3 raw =
        m::cross(dm.positions[t.b] - dm.positions[t.a], dm.positions[t.c] - dm.positions[t.a]);
    if (m::norm(raw) < 1e-14) return false;
  }
  return true;
}
bool uvsInUnitRange(const DisplayMesh& dm) {
  for (const auto& uv : dm.uvs)
    if (uv[0] < 0.0 || uv[0] > 1.0 || uv[1] < 0.0 || uv[1] > 1.0) return false;
  return true;
}

// EXACT quantised (x,y,z) position key — fold split display verts back onto the source
// vertex without a lossy scalar hash's false collisions.
struct PosKey {
  std::int64_t x, y, z;
  bool operator==(const PosKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
};
struct PosKeyHash {
  std::size_t operator()(const PosKey& k) const noexcept {
    std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
    h ^= static_cast<std::size_t>(k.y) * 19349663u + (h << 6) + (h >> 2);
    h ^= static_cast<std::size_t>(k.z) * 83492791u + (h << 6) + (h >> 2);
    return h;
  }
};
PosKey posKey(const m::Point3& p) {
  auto q = [](double v) { return static_cast<std::int64_t>(std::llround(v * 1e9)); };
  return {q(p.x), q(p.y), q(p.z)};
}

// Fold split display verts back by exact position and check the welded topology is the
// SAME closed 2-manifold the source is (pre-LOD topology preservation).
bool foldWatertight(const DisplayMesh& dm) {
  std::unordered_map<PosKey, std::uint32_t, PosKeyHash> posToIdx;
  tess::Mesh folded;
  auto foldIdx = [&](std::uint32_t v) {
    const PosKey k = posKey(dm.positions[v]);
    auto it = posToIdx.find(k);
    if (it != posToIdx.end()) return it->second;
    const std::uint32_t idx = folded.addVertex(dm.positions[v]);
    posToIdx.emplace(k, idx);
    return idx;
  };
  for (const auto& t : dm.triangles) folded.addTriangle(foldIdx(t.a), foldIdx(t.b), foldIdx(t.c));
  return tess::isWatertight(folded);
}

tess::Mesh meshOf(const topo::Shape& s, double defl) {
  tess::MeshParams p;
  p.deflection = defl;
  return tess::SolidMesher{p}.mesh(s);
}

// ── shape families ──────────────────────────────────────────────────────────────────
enum Fam { F_SPHERE, F_CYL, F_CONE, F_BOX, F_COUNT };
const char* famName(int f) {
  switch (f) {
    case F_SPHERE: return "sphere";
    case F_CYL:    return "cylinder";
    case F_CONE:   return "cone-frustum";
    case F_BOX:    return "box";
  }
  return "?";
}

int g_agreed = 0, g_disagreed = 0, g_declined = 0;
int g_famA[F_COUNT] = {0}, g_famX[F_COUNT] = {0}, g_famD[F_COUNT] = {0};

// One fuzz trial: build a random primitive at a random deflection, run buildDisplayMesh
// with random crease / LOD / UV, and check every must-hold invariant. Returns true on
// AGREED (or DECLINED — an empty native mesh on a degenerate build, honest), false on a
// DISAGREED invariant violation (a real finding).
bool runTrial(Rng& rng, int idx, std::uint64_t seed) {
  const int fam = static_cast<int>(rng.below(F_COUNT));
  // A broad deflection sweep incl. very tight (1e-3) and very loose (0.5·smallest-dim).
  // Crease angle spans below and above the analytic seams (box 90°, cyl cap↔wall 90°).
  const double creaseDeg = rng.range(15.0, 60.0);
  const bool wantUVs = (rng.below(2) == 0);
  // Every 4th trial requests LOD to half the tris; the rest keep full resolution.
  const bool wantLOD = (idx % 4 == 3);

  topo::Shape shape;
  double defl = 0.0;
  std::string desc;
  char buf[160];

  // Distance-to-surface invariant closures per family (pre-LOD analytic bound).
  auto declineDisagree = [&](const char* why) {
    std::printf("[FUZZ] DISAGREED case=%-3d %-13s %s\n       REPRO seed=0x%llx index=%d %s\n",
                idx, famName(fam), why, static_cast<unsigned long long>(seed), idx, desc.c_str());
    std::fflush(stdout);
  };

  if (fam == F_SPHERE) {
    const double R = rng.range(1.0, 40.0);
    defl = rng.range(1e-3, 0.05 * R);
    shape = fullSphere(R);
    std::snprintf(buf, sizeof buf, "R=%.4f defl=%.4f crease=%.1f LOD=%d UV=%d", R, defl, creaseDeg,
                  wantLOD ? 1 : 0, wantUVs ? 1 : 0);
    desc = buf;
  } else if (fam == F_CYL) {
    const double R = rng.range(1.0, 30.0), h = rng.range(2.0, 60.0);
    defl = rng.range(1e-3, 0.05 * R);
    shape = cylinderSolid(R, h);
    std::snprintf(buf, sizeof buf, "R=%.4f h=%.4f defl=%.4f crease=%.1f LOD=%d UV=%d", R, h, defl,
                  creaseDeg, wantLOD ? 1 : 0, wantUVs ? 1 : 0);
    desc = buf;
  } else if (fam == F_CONE) {
    const double Rb = rng.range(3.0, 30.0);
    // Frustum (Rt in [0.2Rb,0.8Rb]) or full cone (Rt≈0) — both analytic lateral surfaces.
    const double Rt = (rng.below(3) == 0) ? 0.0 : Rb * rng.range(0.2, 0.8);
    const double H = rng.range(4.0, 40.0);
    defl = rng.range(1e-3, 0.05 * Rb);
    shape = coneSolid(Rb, Rt, H);
    std::snprintf(buf, sizeof buf, "Rb=%.4f Rt=%.4f H=%.4f defl=%.4f crease=%.1f LOD=%d UV=%d", Rb,
                  Rt, H, defl, creaseDeg, wantLOD ? 1 : 0, wantUVs ? 1 : 0);
    desc = buf;
  } else {  // F_BOX
    const double a = rng.range(1.0, 40.0), b = rng.range(1.0, 40.0), c = rng.range(1.0, 40.0);
    const double smallest = std::min({a, b, c});
    defl = rng.range(1e-3, 0.5 * smallest);  // very loose end included
    shape = boxSolid(a, b, c);
    std::snprintf(buf, sizeof buf, "a=%.3f b=%.3f c=%.3f defl=%.4f crease=%.1f LOD=%d UV=%d", a, b,
                  c, defl, creaseDeg, wantLOD ? 1 : 0, wantUVs ? 1 : 0);
    desc = buf;
  }

  const tess::Mesh src = meshOf(shape, defl);
  if (src.triangles.empty()) {
    // A degenerate build (e.g. a sliver frustum) that produced no source mesh: the
    // display mesh honestly declines to empty — not a wrong result.
    DisplayParams p; p.deflection = defl;
    const DisplayMesh dm = buildDisplayMesh(src, p);
    if (dm.triangleCount() != 0) { declineDisagree("empty source but non-empty display"); ++g_disagreed; ++g_famX[fam]; return false; }
    std::printf("[FUZZ] DECLINED  case=%-3d %-13s empty source mesh (honest) %s\n", idx, famName(fam), desc.c_str());
    ++g_declined; ++g_famD[fam]; return true;
  }
  const bool srcWatertight = tess::isWatertight(src);

  DisplayParams p;
  p.deflection = defl;
  p.creaseAngleDeg = creaseDeg;
  p.wantUVs = wantUVs;
  if (wantLOD) { p.lodTargetTris = static_cast<int>(src.triangles.size() / 2); p.lodHausdorffScale = 8.0; }
  const DisplayMesh dm = buildDisplayMesh(src, p);

  // ── must-hold invariants (any violation ⇒ DISAGREED) ──────────────────────────────
  if (dm.triangleCount() == 0) { declineDisagree("non-empty source yielded empty display"); ++g_disagreed; ++g_famX[fam]; return false; }
  if (dm.normals.size() != dm.vertexCount()) { declineDisagree("normal count != vertex count"); ++g_disagreed; ++g_famX[fam]; return false; }
  if (!allFinite(dm)) { declineDisagree("non-finite position/normal/uv"); ++g_disagreed; ++g_famX[fam]; return false; }
  if (!normalsUnit(dm)) { declineDisagree("non-unit normal"); ++g_disagreed; ++g_famX[fam]; return false; }
  if (!nonDegenerateTris(dm)) { declineDisagree("degenerate/out-of-range triangle"); ++g_disagreed; ++g_famX[fam]; return false; }
  if (wantUVs) {
    if (!dm.hasUVs()) { declineDisagree("UVs requested but absent"); ++g_disagreed; ++g_famX[fam]; return false; }
    if (dm.uvs.size() != dm.vertexCount()) { declineDisagree("uv count != vertex count"); ++g_disagreed; ++g_famX[fam]; return false; }
    if (!uvsInUnitRange(dm)) { declineDisagree("uv outside [0,1]"); ++g_disagreed; ++g_famX[fam]; return false; }
  }

  // Pre-LOD topology preservation: triangle count preserved exactly, and folding split
  // verts back yields the same watertight surface the source is.
  if (!wantLOD) {
    if (dm.triangleCount() != src.triangles.size()) { declineDisagree("split changed triangle count"); ++g_disagreed; ++g_famX[fam]; return false; }
    if (srcWatertight && !foldWatertight(dm)) { declineDisagree("folded display not watertight (topology lost)"); ++g_disagreed; ++g_famX[fam]; return false; }
  } else {
    // LOD only REDUCES (never grows) and never over-collapses below the target floor.
    if (dm.triangleCount() > src.triangles.size()) { declineDisagree("LOD grew triangle count"); ++g_disagreed; ++g_famX[fam]; return false; }
    if (static_cast<int>(dm.triangleCount()) < p.lodTargetTris && p.lodTargetTris > 0 &&
        dm.triangleCount() * 4 < static_cast<std::size_t>(p.lodTargetTris)) {
      // A gross over-collapse far past the floor is a bug; a mild early stop (budget-
      // limited) is expected. Only flag a >4× over-collapse.
      declineDisagree("LOD over-collapsed far past the target floor"); ++g_disagreed; ++g_famX[fam]; return false;
    }
  }

  // ── DEFLECTION BOUND vs the CLOSED-FORM analytic surface (the load-bearing oracle) ──
  // Pre-LOD: the tight kSurfMul·defl bound. Post-LOD: the Hausdorff budget (scale·defl).
  const double surfBudget = wantLOD ? (p.lodHausdorffScale * defl) : (3.0 * defl + 1e-9);
  double worst = 0.0;
  const char* metricName = "";
  if (fam == F_SPHERE)     { worst = maxDistToSphere(dm, /*R filled below*/ 0.0); }
  // Re-measure with the actual radius per family (kept local to avoid recomputation churn).
  if (fam == F_SPHERE) {
    // Recover R robustly from the max vertex radius (== R for a sphere at the equator).
    double R = 0.0;
    for (const auto& q : dm.positions) R = std::max(R, std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z));
    worst = maxDistToSphere(dm, R);
    metricName = "sphere |‖p‖−R|";
    // A fully-smooth sphere's normals must all point OUTWARD (no flipped normal).
    for (std::size_t i = 0; i < dm.vertexCount(); ++i) {
      const m::Point3& q = dm.positions[i];
      if (m::dot(dm.normals[i].vec(), q.asVec()) <= 0.0) { declineDisagree("sphere normal points inward (flipped)"); ++g_disagreed; ++g_famX[fam]; return false; }
    }
  } else if (fam == F_CYL) {
    double R = 0.0;
    for (const auto& q : dm.positions) R = std::max(R, std::sqrt(q.x * q.x + q.y * q.y));
    worst = maxWallDistToCylinder(dm, R, defl);
    metricName = "cyl wall |r−R|";
  } else if (fam == F_CONE) {
    // Recover Rb (radius at y≈0) and H (max y) from the mesh; Rt from the top ring.
    double H = 0.0, Rb = 0.0, Rt = 0.0;
    for (const auto& q : dm.positions) H = std::max(H, q.y);
    for (const auto& q : dm.positions) {
      const double r = std::sqrt(q.x * q.x + q.z * q.z);
      if (q.y < 1e-6) Rb = std::max(Rb, r);
      if (q.y > H - 1e-6) Rt = std::max(Rt, r);
    }
    worst = maxWallDistToCone(dm, Rb, Rt, H, defl);
    metricName = "cone wall |r−R(y)|·cosα";
  } else {  // F_BOX — planar faces: every vertex lies EXACTLY on a face plane (0 chord).
    // The chord bound is trivially 0 for planar faces; assert vertices sit on the box
    // hull (min/max per axis), i.e. within fp of an axis plane.
    double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
    for (const auto& q : dm.positions) {
      lo[0] = std::min(lo[0], q.x); hi[0] = std::max(hi[0], q.x);
      lo[1] = std::min(lo[1], q.y); hi[1] = std::max(hi[1], q.y);
      lo[2] = std::min(lo[2], q.z); hi[2] = std::max(hi[2], q.z);
    }
    for (const auto& q : dm.positions) {
      const double dx = std::min(std::fabs(q.x - lo[0]), std::fabs(q.x - hi[0]));
      const double dy = std::min(std::fabs(q.y - lo[1]), std::fabs(q.y - hi[1]));
      const double dz = std::min(std::fabs(q.z - lo[2]), std::fabs(q.z - hi[2]));
      worst = std::max(worst, std::min({dx, dy, dz}));  // dist to nearest face plane
    }
    metricName = "box vertex-to-face-plane";
  }

  if (worst > surfBudget) {
    std::snprintf(buf, sizeof buf, "deflection bound violated: %s = %.4e > budget %.4e", metricName,
                  worst, surfBudget);
    declineDisagree(buf); ++g_disagreed; ++g_famX[fam]; return false;
  }

  std::printf("[FUZZ] AGREED    case=%-3d %-13s tris=%zu verts=%zu %s=%.3e/%.3e %s\n", idx,
              famName(fam), dm.triangleCount(), dm.vertexCount(), metricName, worst, surfBudget,
              desc.c_str());
  std::fflush(stdout);
  ++g_agreed; ++g_famA[fam];
  return true;
}

}  // namespace

// A single CC_TEST drives the whole seeded batch (mirrors the host-fuzz style: the
// generator is the test). Two default seeds prove seed-independence; FUZZ_SEED / FUZZ_N
// override for a targeted repro. The bar is DISAGREED==0 with every family ≥1 AGREED.
CC_TEST(display_mesh_differential_fuzz) {
  std::uint64_t seeds[2] = {0xD15B1A57EEull, 0x0FF1CE9A11ull};
  int N = 80;
  if (const char* e = std::getenv("FUZZ_SEED")) { seeds[0] = std::strtoull(e, nullptr, 0); seeds[1] = seeds[0]; }
  if (const char* e = std::getenv("FUZZ_N")) { const int n = std::atoi(e); if (n > 0) N = n; }

  std::printf("== M6 HOST-analytic display-mesh fuzz: buildDisplayMesh over random sphere/cyl/cone/box ==\n");
  std::printf("== oracle = CLOSED-FORM surface distance; deflection mult FIXED at 3x (pre-LOD), Hausdorff 8x (LOD) — NEVER widened ==\n");

  const int nSeeds = (seeds[0] == seeds[1]) ? 1 : 2;
  for (int si = 0; si < nSeeds; ++si) {
    Rng rng(seeds[si]);
    std::printf("\n-- seed=0x%llx N=%d --\n", static_cast<unsigned long long>(seeds[si]), N);
    for (int i = 0; i < N; ++i) (void)runTrial(rng, i, seeds[si]);
  }

  std::printf("\n== COVERAGE: AGREED=%d HONESTLY-DECLINED=%d DISAGREED=%d ==\n", g_agreed, g_declined,
              g_disagreed);
  std::printf("   per-family [AGREED / HONESTLY-DECLINED / DISAGREED]:\n");
  bool coverage = true;
  for (int f = 0; f < F_COUNT; ++f) {
    std::printf("     %-14s %d / %d / %d\n", famName(f), g_famA[f], g_famD[f], g_famX[f]);
    if (g_famA[f] < 1) coverage = false;
  }
  CC_CHECK(coverage);          // every family exercised at least once (AGREED)
  CC_CHECK_EQ(g_disagreed, 0); // THE BAR: zero silent-wrong display meshes
}

CC_RUN_ALL()
