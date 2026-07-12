// SPDX-License-Identifier: Apache-2.0
//
// bspline_sweep.cpp — NURBS roadmap Layer 6 (swept surfaces) implementation.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), §10.4 (swept surfaces).
// Two constructions:
//   * sweepTranslational — the EXACT closed-form extrusion (tabulated cylinder): the
//     tensor product of the section (U) with a degree-1 two-pole path (V). Solve-free;
//     every iso-curve is the section translated. Machine-exact, no fitting.
//   * sweepAlongTrajectory — the GENERAL sweep: place the section at K stations along the
//     spine using a ROTATION-MINIMIZING moving frame (double-reflection method, Wang et
//     al. 2008 — see the header for the anti-twist rationale), then SKIN the K transformed
//     sections via skinSurface (bspline_skin.h). The skin solves linear systems through
//     the numsci facade, so — like bspline_skin.cpp — the WHOLE file is under
//     CYBERCAD_HAS_NUMSCI. With the guard OFF the TU is inert and the Layer-6 sweep
//     functions are simply absent from the library.
//
#include "native/math/bspline_sweep.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"       // curveDerivs (trajectory point + tangent)
#include "native/math/bspline_ops.h"   // BsplineCurveData / BsplineSurfaceData
#include "native/math/bspline_skin.h"  // skinSurface (transform-then-skin)
#include "native/math/transform.h"     // Mat3 / Transform (moving-frame placement)
#include "native/math/vec.h"

#include <cmath>
#include <span>
#include <vector>

namespace cybercad::native::math {
namespace {

// Number of control points is fixed by the pole array (NOT derived from the knots — that
// would make the validity check tautological). The flat knot vector must then have length
// poles + degree + 1 for a well-formed clamped B-spline.
int nPolesOf(const BsplineCurveData& c) {
  return static_cast<int>(c.poles.size());
}

// A section / trajectory curve is well-formed iff: degree ≥ 1, has poles, and the flat
// knot vector length matches poles + degree + 1. (Same validity notion the skin uses.)
bool wellFormed(const BsplineCurveData& c) {
  const int n = nPolesOf(c);
  return c.degree >= 1 && n >= 1 &&
         static_cast<int>(c.knots.size()) == n + c.degree + 1;
}

// A well-formed RATIONAL section: well-formed geometry PLUS one strictly-positive weight
// per pole. (An empty weight vector is non-rational — declined by the rational routines.)
bool wellFormedRational(const BsplineCurveData& c) {
  if (!wellFormed(c)) return false;
  if (c.weights.size() != c.poles.size()) return false;
  for (double w : c.weights)
    if (!(w > 0.0)) return false;
  return true;
}

// The clamped trajectory domain [a,b] = [first knot, last knot].
void domainOf(const BsplineCurveData& c, double& a, double& b) {
  a = c.knots.front();
  b = c.knots.back();
}

// Trajectory point + unit tangent at parameter t (non-rational; §10.4 uses the spine's
// point and tangent to position/orient the frame). Falls back through neighbouring
// derivatives if the first derivative vanishes at t (a stationary point of the spine).
struct PointTangent {
  Point3 p;
  Dir3 tangent;
  bool ok = false;
};

PointTangent evalPointTangent(const BsplineCurveData& c, double t) {
  PointTangent r;
  std::vector<Vec3> d(2);
  curveDerivs(c.degree, c.poles, c.knots, t, 1, d);
  r.p = Point3{d[0].x, d[0].y, d[0].z};
  const Dir3 tan(d[1]);
  if (tan.valid()) {
    r.tangent = tan;
    r.ok = true;
  }
  return r;
}

// Build an orthonormal right-handed basis (u, v, w) with w = the given unit axis and u a
// stable in-plane reference (chosen from the least-aligned world axis to avoid a
// degenerate cross product). Used to seed the section reference frame and RMF station 0.
void orthonormalBasis(const Dir3& w, Vec3& u, Vec3& v) {
  const Vec3 wv = w.vec();
  // Pick the world axis least aligned with w for a stable seed.
  const double ax = std::fabs(wv.x), ay = std::fabs(wv.y), az = std::fabs(wv.z);
  Vec3 seed = (ax <= ay && ax <= az) ? Vec3{1, 0, 0}
              : (ay <= az)           ? Vec3{0, 1, 0}
                                     : Vec3{0, 0, 1};
  u = cross(seed, wv);
  const double nu = norm(u);
  u = u / nu;
  v = cross(wv, u);  // already unit (w,u orthonormal)
}

// ── Rotation-minimizing frame along the trajectory (double-reflection, Wang 2008) ──
// Given the ordered station points x_k and unit tangents t_k, propagate a reference
// normal r_k that rotates by the MINIMUM amount consistent with the tangent (no
// torsion-driven spin). Returns r_k (the normals); the third axis is t_k × r_k.
// r_0 is seeded orthogonal to t_0 from the section's reference normal projected into the
// plane ⟂ t_0 (so the swept profile's initial orientation matches the section plane).
std::vector<Vec3> rmfNormals(const std::vector<Point3>& x, const std::vector<Dir3>& t,
                             const Vec3& seedNormal) {
  const int K = static_cast<int>(x.size());
  std::vector<Vec3> r(K);
  // Seed r_0: component of seedNormal orthogonal to t_0, normalized. If seedNormal is
  // (near) parallel to t_0, fall back to a stable orthonormal seed.
  const Vec3 t0 = t[0].vec();
  Vec3 r0 = seedNormal - t0 * dot(seedNormal, t0);
  if (norm(r0) < 1e-9) {
    Vec3 u, v;
    orthonormalBasis(t[0], u, v);
    r0 = u;
  } else {
    r0 = r0 / norm(r0);
  }
  r[0] = r0;

  for (int k = 0; k + 1 < K; ++k) {
    const Vec3 xk = x[k].asVec();
    const Vec3 xk1 = x[k + 1].asVec();
    const Vec3 tk = t[k].vec();
    const Vec3 tk1 = t[k + 1].vec();

    // Reflection 1: reflect r_k and t_k across the plane bisecting x_k, x_{k+1}.
    const Vec3 v1 = xk1 - xk;
    const double c1 = dot(v1, v1);
    Vec3 rL = r[k];
    Vec3 tL = tk;
    if (c1 > 1e-30) {
      rL = r[k] - v1 * (2.0 / c1 * dot(v1, r[k]));
      tL = tk - v1 * (2.0 / c1 * dot(v1, tk));
    }
    // Reflection 2: reflect the once-reflected frame across the plane bisecting tL, t_{k+1}.
    const Vec3 v2 = tk1 - tL;
    const double c2 = dot(v2, v2);
    Vec3 rNext = rL;
    if (c2 > 1e-30) rNext = rL - v2 * (2.0 / c2 * dot(v2, rL));
    // Re-orthogonalize against t_{k+1} and normalize (guard fp drift).
    rNext = rNext - tk1 * dot(rNext, tk1);
    const double nn = norm(rNext);
    r[k + 1] = (nn > 1e-12) ? rNext / nn : rL;
  }
  return r;
}

// Place a copy of `section` at `stations` along the `trajectory` using a rotation-minimizing
// moving frame. Shared by the non-rational and rational general sweeps: the placement is a
// RIGID transform (rotate about the section origin, translate to the station point), so the
// copied section's weights (if any) are preserved EXACTLY. Returns the placed sections, or
// empty on any station-sampling / degenerate-trajectory failure (the caller declines).
std::vector<BsplineCurveData> placeSectionsAlongTrajectory(const BsplineCurveData& section,
                                                           const BsplineCurveData& trajectory,
                                                           const Dir3& sectionNormal,
                                                           int stations) {
  std::vector<BsplineCurveData> placed;
  if (stations < 2 || !sectionNormal.valid()) return placed;

  double a = 0.0, b = 0.0;
  domainOf(trajectory, a, b);
  if (!(b > a)) return placed;  // degenerate domain

  std::vector<Point3> pts;
  std::vector<Dir3> tans;
  pts.reserve(stations);
  tans.reserve(stations);
  for (int k = 0; k < stations; ++k) {
    const double t = a + (b - a) * (static_cast<double>(k) / (stations - 1));
    const PointTangent pt = evalPointTangent(trajectory, t);
    if (!pt.ok) return {};  // stationary spine point
    pts.push_back(pt.p);
    tans.push_back(pt.tangent);
  }

  // Coincident-trajectory guard: no path to sweep along.
  double pathLen = 0.0;
  for (int k = 1; k < stations; ++k) pathLen += distance(pts[k], pts[k - 1]);
  if (!(pathLen > 1e-12)) return {};

  const std::vector<Vec3> rmf = rmfNormals(pts, tans, sectionNormal.vec());

  // Section reference basis (u0, v0, n0): n0 = the section plane normal.
  Vec3 u0, v0;
  orthonormalBasis(sectionNormal, u0, v0);
  const Vec3 n0 = sectionNormal.vec();

  placed.reserve(stations);
  for (int k = 0; k < stations; ++k) {
    const Vec3 wk = tans[k].vec();
    const Vec3 uk = rmf[k];
    const Vec3 vk = cross(wk, uk);  // right-handed station basis

    // R_k = [uk vk wk] (columns) · [u0 v0 n0]^T (rows).
    Mat3 R;
    for (int rw = 0; rw < 3; ++rw) {
      R(rw, 0) = uk[rw] * u0.x + vk[rw] * v0.x + wk[rw] * n0.x;
      R(rw, 1) = uk[rw] * u0.y + vk[rw] * v0.y + wk[rw] * n0.y;
      R(rw, 2) = uk[rw] * u0.z + vk[rw] * v0.z + wk[rw] * n0.z;
    }
    const Transform place(R, pts[k].asVec());

    BsplineCurveData s = section;  // copies degree / knots / WEIGHTS
    for (Point3& p : s.poles) p = place.applyToPoint(p);
    placed.push_back(std::move(s));
  }
  return placed;
}

// Sample K station points + unit tangents evenly across a curve's clamped domain. Returns
// false on any stationary spine point or an all-coincident (zero-length) sampled path.
bool sampleStations(const BsplineCurveData& curve, int stations, std::vector<Point3>& pts,
                    std::vector<Dir3>& tans) {
  if (stations < 2) return false;
  double a = 0.0, b = 0.0;
  domainOf(curve, a, b);
  if (!(b > a)) return false;
  pts.clear();
  tans.clear();
  pts.reserve(stations);
  tans.reserve(stations);
  for (int k = 0; k < stations; ++k) {
    const double t = a + (b - a) * (static_cast<double>(k) / (stations - 1));
    const PointTangent pt = evalPointTangent(curve, t);
    if (!pt.ok) return false;
    pts.push_back(pt.p);
    tans.push_back(pt.tangent);
  }
  double pathLen = 0.0;
  for (int k = 1; k < stations; ++k) pathLen += distance(pts[k], pts[k - 1]);
  return pathLen > 1e-12;
}

// Compose the RIGID frame rotation R_k = [uk vk wk]·[u0 v0 n0]^T that maps the section
// reference basis onto the station basis. Shared by the variable and two-rail placers.
Mat3 frameRotation(const Vec3& uk, const Vec3& vk, const Vec3& wk, const Vec3& u0, const Vec3& v0,
                   const Vec3& n0) {
  Mat3 R;
  for (int rw = 0; rw < 3; ++rw) {
    R(rw, 0) = uk[rw] * u0.x + vk[rw] * v0.x + wk[rw] * n0.x;
    R(rw, 1) = uk[rw] * u0.y + vk[rw] * v0.y + wk[rw] * n0.y;
    R(rw, 2) = uk[rw] * u0.z + vk[rw] * v0.z + wk[rw] * n0.z;
  }
  return R;
}

// ── VARIABLE-SECTION placement: scale + twist about the section origin, then the rigid RMF
// placement. The scale (about the origin) and the twist (rotation about the section normal n0)
// act in the section's OWN plane BEFORE the rigid frame maps (u0,v0,n0) onto the station basis;
// composed as one affine per station. A uniform scale + rotation is a SIMILARITY, so it
// preserves the section's weights EXACTLY (the rational variant relies on this). Returns the
// placed sections, or empty on any sampling / degenerate / bad-field failure (caller declines.)
std::vector<BsplineCurveData> placeSectionsVariable(const BsplineCurveData& section,
                                                    const BsplineCurveData& trajectory,
                                                    const Dir3& sectionNormal,
                                                    const std::vector<double>& scales,
                                                    const std::vector<double>& twists,
                                                    int stations) {
  std::vector<BsplineCurveData> placed;
  if (stations < 2 || !sectionNormal.valid()) return placed;
  if (static_cast<int>(scales.size()) != stations) return placed;
  if (static_cast<int>(twists.size()) != stations) return placed;
  for (double s : scales)
    if (!(s > 0.0)) return placed;  // non-positive scale — undefined section

  std::vector<Point3> pts;
  std::vector<Dir3> tans;
  if (!sampleStations(trajectory, stations, pts, tans)) return placed;

  const std::vector<Vec3> rmf = rmfNormals(pts, tans, sectionNormal.vec());

  Vec3 u0, v0;
  orthonormalBasis(sectionNormal, u0, v0);
  const Vec3 n0 = sectionNormal.vec();

  placed.reserve(stations);
  for (int k = 0; k < stations; ++k) {
    const Vec3 wk = tans[k].vec();
    const Vec3 uk = rmf[k];
    const Vec3 vk = cross(wk, uk);
    const Mat3 R = frameRotation(uk, vk, wk, u0, v0, n0);
    const Transform place(R, pts[k].asVec());

    // Local scale about the origin, then twist about the section normal (in-plane), both
    // acting BEFORE the rigid frame. A twist about n0 keeps the section in its plane.
    const Transform scale = Transform::scaleOf(Point3{0, 0, 0}, scales[k]);
    const Transform twist =
        Transform::rotationOf(Point3{0, 0, 0}, Dir3(n0.x, n0.y, n0.z), twists[k]);
    // full = place ∘ twist ∘ scale (scale first, then twist, then rigid placement).
    const Transform full = place.composedWith(twist.composedWith(scale));

    BsplineCurveData s = section;  // copies degree / knots / WEIGHTS
    for (Point3& p : s.poles) p = full.applyToPoint(p);
    placed.push_back(std::move(s));
  }
  return placed;
}

// ── TWO-RAIL placement: anchor0 rides rail0(t), anchor1 rides rail1(t). Per station the
// section is scaled so its anchor chord matches the rail chord, oriented so the anchor chord
// aligns with the rail chord (remaining spin removed by an RMF along the rail-midpoint spine),
// and translated so anchor0 lands on rail0(t). The composed map is a SIMILARITY (uniform scale
// + rotation + translation), preserving weights exactly. Returns placed sections, or empty on
// any degenerate configuration (coincident anchors, zero rail chord, degenerate spine).
std::vector<BsplineCurveData> placeSectionsTwoRail(const BsplineCurveData& section,
                                                   const BsplineCurveData& rail0,
                                                   const BsplineCurveData& rail1,
                                                   const Dir3& sectionNormal, int anchor0,
                                                   int anchor1, int stations) {
  std::vector<BsplineCurveData> placed;
  if (stations < 2 || !sectionNormal.valid()) return placed;

  const int N = nPolesOf(section);
  if (anchor0 < 0 || anchor1 < 0 || anchor0 >= N || anchor1 >= N || anchor0 == anchor1)
    return placed;

  const Point3 A0 = section.poles[anchor0];
  const Point3 A1 = section.poles[anchor1];
  const Vec3 anchorChord = A1 - A0;
  const double anchorLen = norm(anchorChord);
  if (!(anchorLen > 1e-12)) return placed;  // coincident section anchors — no chord to scale
  const Vec3 anchorDir = anchorChord / anchorLen;

  // Rails must share a common clamped domain (the standard two-rail pre-condition).
  double a0 = 0.0, b0 = 0.0, a1 = 0.0, b1 = 0.0;
  domainOf(rail0, a0, b0);
  domainOf(rail1, a1, b1);
  if (!(b0 > a0) || !(b1 > a1)) return placed;
  if (std::fabs(a0 - a1) > 1e-9 || std::fabs(b0 - b1) > 1e-9) return placed;

  // Sample both rails + build the midpoint spine (points + tangents) for the RMF.
  std::vector<Point3> p0(stations), p1(stations), mid(stations);
  std::vector<Vec3> railChord(stations);
  for (int k = 0; k < stations; ++k) {
    const double t = a0 + (b0 - a0) * (static_cast<double>(k) / (stations - 1));
    p0[k] = curvePoint(rail0.degree, rail0.poles, rail0.knots, t);
    p1[k] = curvePoint(rail1.degree, rail1.poles, rail1.knots, t);
    railChord[k] = p1[k] - p0[k];
    if (!(norm(railChord[k]) > 1e-9)) return placed;  // rails cross/touch — undefined
    mid[k] = p0[k] + railChord[k] * 0.5;
  }

  // Midpoint-spine tangents (finite difference; the spine only carries the anti-twist frame).
  std::vector<Dir3> spineTan(stations);
  for (int k = 0; k < stations; ++k) {
    const int kp = (k + 1 < stations) ? k + 1 : k;
    const int km = (k > 0) ? k - 1 : k;
    Vec3 d = mid[kp] - mid[km];
    Dir3 td(d);
    if (!td.valid()) {
      // Fall back to a stable perpendicular to the rail chord when the spine is stationary.
      Vec3 u, v;
      orthonormalBasis(Dir3(railChord[k]), u, v);
      td = Dir3(u);
    }
    spineTan[k] = td;
  }
  double spineLen = 0.0;
  for (int k = 1; k < stations; ++k) spineLen += distance(mid[k], mid[k - 1]);
  if (!(spineLen > 1e-12)) return placed;  // degenerate spine (coincident midpoints)

  // RMF along the midpoint spine gives an anti-twist reference normal r_k ⟂ spineTan_k.
  const std::vector<Vec3> rmf = rmfNormals(mid, spineTan, sectionNormal.vec());

  // Section reference basis: u0 = anchor chord direction (the axis the rails span), n0 = the
  // section plane normal, v0 = n0 × u0 (right-handed). We map (u0,v0,n0) onto the station
  // basis (uk = rail chord dir, nk = RMF normal, vk = nk × uk).
  const Vec3 n0 = sectionNormal.vec();
  Vec3 u0 = anchorDir;
  // Orthonormalize u0 against n0 so the reference basis is orthonormal (the anchor chord is
  // assumed to lie in the section plane; project out any normal component for numerical safety).
  u0 = u0 - n0 * dot(u0, n0);
  const double nu0 = norm(u0);
  if (!(nu0 > 1e-12)) return placed;  // anchor chord parallel to the normal — ill-posed
  u0 = u0 / nu0;
  const Vec3 v0 = cross(n0, u0);

  placed.reserve(stations);
  for (int k = 0; k < stations; ++k) {
    const Vec3 uk = railChord[k] / norm(railChord[k]);  // station axis = rail chord direction
    const Vec3 nk = rmf[k];                              // RMF normal (anti-twist)
    const Vec3 vk = cross(nk, uk);                       // right-handed station basis
    // R maps (u0,v0,n0) -> (uk,vk,nk).
    const Mat3 R = frameRotation(uk, vk, nk, u0, v0, n0);

    const double s = norm(railChord[k]) / anchorLen;  // scale = rail chord / anchor chord
    // Similarity S = s·R about the section origin; then translate so anchor0 -> rail0(t).
    // full(p) = s·R·p + translation, translation chosen so full(A0) = p0[k].
    Mat3 sR;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) sR(i, j) = R(i, j) * s;
    const Vec3 trans = p0[k].asVec() - sR * A0.asVec();
    const Transform full(sR, trans);

    BsplineCurveData sec = section;  // copies degree / knots / WEIGHTS
    for (Point3& p : sec.poles) p = full.applyToPoint(p);
    placed.push_back(std::move(sec));
  }
  return placed;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Translational sweep (EXACT — tensor product of section × degree-1 path).
// ─────────────────────────────────────────────────────────────────────────────

SweepResult sweepTranslational(const BsplineCurveData& section, const Vec3& sweep) {
  SweepResult r;
  if (!section.weights.empty()) return r;   // rational — decline honestly
  if (!wellFormed(section)) return r;        // malformed section
  if (isNull(sweep)) return r;               // null sweep vector — no surface

  const int N = nPolesOf(section);

  // U = the section (degree p, section knots, N poles). V = degree-1 two poles: 0 and
  // sweep, over the clamped knot vector {0,0,1,1}. The net pole(i,j):
  //   j=0 → section.poles[i], j=1 → section.poles[i] + sweep.
  // Row-major, U outer: net[i*2 + j].
  r.surface.degreeU = section.degree;
  r.surface.degreeV = 1;
  r.surface.nPolesU = N;
  r.surface.nPolesV = 2;
  r.surface.knotsU = section.knots;
  r.surface.knotsV = {0.0, 0.0, 1.0, 1.0};
  r.surface.poles.resize(static_cast<std::size_t>(N) * 2);
  for (int i = 0; i < N; ++i) {
    r.surface.poles[static_cast<std::size_t>(i) * 2 + 0] = section.poles[i];
    r.surface.poles[static_cast<std::size_t>(i) * 2 + 1] = section.poles[i] + sweep;
  }
  // weights empty ⇒ non-rational.
  r.vParams = {0.0, 1.0};
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// EXACT rational translational sweep — rational tensor product (section × path).
// ─────────────────────────────────────────────────────────────────────────────

SweepResult sweepRationalTranslational(const BsplineCurveData& section, const Vec3& sweep) {
  SweepResult r;
  if (section.weights.empty()) return r;       // non-rational — use sweepTranslational
  if (!wellFormedRational(section)) return r;   // malformed / non-positive weight
  if (isNull(sweep)) return r;                  // null sweep vector — no surface

  const int N = nPolesOf(section);

  // U = the rational section (degree p, section knots, N poles + WEIGHTS). V = degree-1 two
  // poles {0, sweep} over {0,0,1,1}. The weight is CONSTANT in V (weight(i,0)=weight(i,1)=
  // section.weights[i]), so the rational iso-curve S(·,v) is EXACTLY the rational section
  // translated by v·sweep. Row-major, U outer: net[i*2 + j].
  r.surface.degreeU = section.degree;
  r.surface.degreeV = 1;
  r.surface.nPolesU = N;
  r.surface.nPolesV = 2;
  r.surface.knotsU = section.knots;
  r.surface.knotsV = {0.0, 0.0, 1.0, 1.0};
  r.surface.poles.resize(static_cast<std::size_t>(N) * 2);
  r.surface.weights.resize(static_cast<std::size_t>(N) * 2);
  for (int i = 0; i < N; ++i) {
    const double w = section.weights[i];
    r.surface.poles[static_cast<std::size_t>(i) * 2 + 0] = section.poles[i];
    r.surface.poles[static_cast<std::size_t>(i) * 2 + 1] = section.poles[i] + sweep;
    r.surface.weights[static_cast<std::size_t>(i) * 2 + 0] = w;
    r.surface.weights[static_cast<std::size_t>(i) * 2 + 1] = w;
  }
  r.vParams = {0.0, 1.0};
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// General sweep along a trajectory curve (transform-then-skin, §10.4).
// ─────────────────────────────────────────────────────────────────────────────

SweepResult sweepAlongTrajectory(const BsplineCurveData& section,
                                 const BsplineCurveData& trajectory,
                                 const Dir3& sectionNormal, int stations, int degreeV) {
  SweepResult r;
  if (stations < 2) return r;                                  // need ≥ 2 stations
  if (!section.weights.empty() || !trajectory.weights.empty())
    return r;                                                  // rational — decline
  if (!wellFormed(section) || !wellFormed(trajectory)) return r;
  if (!sectionNormal.valid()) return r;                        // no reference normal

  // Place the section at K stations along the spine via the rotation-minimizing frame.
  const std::vector<BsplineCurveData> placed =
      placeSectionsAlongTrajectory(section, trajectory, sectionNormal, stations);
  if (static_cast<int>(placed.size()) != stations) return r;   // sampling / degenerate fail

  // Skin the K placed sections into one tensor-product surface (Layer 6).
  const SkinResult skin = skinSurface(std::span<const BsplineCurveData>(placed), degreeV);
  if (!skin.ok) return r;

  r.surface = skin.surface;
  r.vParams = skin.vParams;
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// General rational sweep (transform-then-rational-skin).
// ─────────────────────────────────────────────────────────────────────────────

SweepResult sweepRationalAlongTrajectory(const BsplineCurveData& section,
                                         const BsplineCurveData& trajectory,
                                         const Dir3& sectionNormal, int stations,
                                         int degreeV) {
  SweepResult r;
  if (stations < 2) return r;                                  // need ≥ 2 stations
  if (section.weights.empty()) return r;                       // non-rational — decline
  if (!trajectory.weights.empty()) return r;                   // rational spine — decline
  if (!wellFormedRational(section) || !wellFormed(trajectory)) return r;
  if (!sectionNormal.valid()) return r;                        // no reference normal

  // Place the rational section at K stations (rigid transform preserves weights exactly).
  const std::vector<BsplineCurveData> placed =
      placeSectionsAlongTrajectory(section, trajectory, sectionNormal, stations);
  if (static_cast<int>(placed.size()) != stations) return r;   // sampling / degenerate fail

  // Rational-skin the K placed rational sections into one tensor-product NURBS surface.
  const SkinResult skin =
      skinRationalSurface(std::span<const BsplineCurveData>(placed), degreeV);
  if (!skin.ok) return r;

  r.surface = skin.surface;
  r.vParams = skin.vParams;
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rotational (revolved) sweep — EXACT rational surface of revolution (A7.1).
// ─────────────────────────────────────────────────────────────────────────────

SweepResult sweepRotational(const BsplineCurveData& section, const Point3& axisPoint,
                            const Dir3& axisDir, double angle) {
  SweepResult r;
  if (!wellFormed(section)) return r;               // malformed profile
  if (!section.weights.empty() && !wellFormedRational(section))
    return r;                                        // rational but bad/non-positive weight
  if (!axisDir.valid()) return r;                    // null / non-unit axis
  if (!(std::fabs(angle) > 1e-12)) return r;         // zero swept angle — no surface

  const int N = nPolesOf(section);
  const bool rational = !section.weights.empty();

  // ── V direction: degree-2 rational circular arc through `angle` (A7.1). ──
  // Split into narcs ≤ 90° segments; each segment contributes an ON-arc pole (w=1) and a
  // BETWEEN pole at radius r/cos(Δθ/2), weight cos(Δθ/2). nPolesV = 2*narcs + 1.
  constexpr double kHalfPi = 1.57079632679489661923;  // π/2 (portable, no M_PI_2 macro)
  const double absAngle = std::fabs(angle);
  const int narcs = static_cast<int>(std::ceil(absAngle / kHalfPi - 1e-12));
  const int nArc = (narcs < 1) ? 1 : narcs;
  const double dtheta = angle / nArc;                // signed per-segment sweep
  const double wm = std::cos(dtheta / 2.0);          // between-pole weight (cos half-angle)
  const int nV = 2 * nArc + 1;

  // Degree-2 clamped V-knots on [0,1]: interior breakpoints at j/nArc with multiplicity 2.
  std::vector<double> knotsV;
  knotsV.reserve(static_cast<std::size_t>(nV) + 3);
  knotsV.push_back(0.0); knotsV.push_back(0.0); knotsV.push_back(0.0);
  for (int j = 1; j < nArc; ++j) {
    const double b = static_cast<double>(j) / nArc;
    knotsV.push_back(b); knotsV.push_back(b);
  }
  knotsV.push_back(1.0); knotsV.push_back(1.0); knotsV.push_back(1.0);

  // Rotation about the axis by the cumulative angle at each ON-arc station j (j=0..nArc):
  //   station j sits at angle j*dtheta. The BETWEEN pole (j+1/2) bisects segment j.
  const Vec3 av = axisDir.vec();

  // Project a point onto the axis and get its radial offset vector O→P⊥ (foot on the axis).
  auto radialOf = [&](const Point3& p, Point3& foot, Vec3& radial, double& radius) {
    const Vec3 d = p - axisPoint;                    // axisPoint→P
    const double t = dot(d, av);                     // signed distance along the axis
    foot = axisPoint + av * t;                       // foot of perpendicular on the axis
    radial = p - foot;                               // perpendicular component
    radius = norm(radial);
  };

  // Guard: the ENTIRE profile on the axis ⇒ every radius ≈ 0 ⇒ the revolve collapses.
  double maxRadius = 0.0;
  for (int i = 0; i < N; ++i) {
    Point3 foot; Vec3 radial; double radius = 0.0;
    radialOf(section.poles[i], foot, radial, radius);
    maxRadius = std::max(maxRadius, radius);
  }
  if (!(maxRadius > 1e-12)) return r;                // degenerate: profile lies on the axis

  // Assemble the surface net (row-major U outer, V inner): pole(i, jv) = poles[i*nV + jv].
  r.surface.degreeU = section.degree;
  r.surface.degreeV = 2;
  r.surface.nPolesU = N;
  r.surface.nPolesV = nV;
  r.surface.knotsU = section.knots;
  r.surface.knotsV = std::move(knotsV);
  r.surface.poles.assign(static_cast<std::size_t>(N) * nV, Point3{});
  r.surface.weights.assign(static_cast<std::size_t>(N) * nV, 1.0);  // always rational

  for (int i = 0; i < N; ++i) {
    const Point3 P0 = section.poles[i];
    const double wP = rational ? section.weights[i] : 1.0;  // profile weight (rides through)

    Point3 foot; Vec3 radial; double radius = 0.0;
    radialOf(P0, foot, radial, radius);

    // Local right-handed frame at this profile point: e0 = radial direction (if any),
    // e1 = axis × e0 (the tangent of revolution). For an on-axis point (radius≈0) both the ON
    // and BETWEEN poles collapse to the POSITION P0 (it stays on the axis under revolve), but the
    // WEIGHT still follows the arc pattern wP·{1, cos(Δθ/2), 1, …} — forcing weight 1 there would
    // break the surface's separable weight structure wᵢⱼ = wProfileᵢ·wArcⱼ and warp the revolve.
    const bool onAxis = !(radius > 1e-12);
    Vec3 e0{0, 0, 0}, e1{0, 0, 0};
    if (!onAxis) {
      e0 = radial / radius;
      e1 = cross(av, e0);  // unit (av, e0 orthonormal)
    }

    // Point on the arc at cumulative angle a (radians about the axis, foot as center).
    auto arcPoint = [&](double a) -> Point3 {
      if (onAxis) return P0;
      const double cx = radius * std::cos(a);
      const double cy = radius * std::sin(a);
      return foot + e0 * cx + e1 * cy;
    };

    for (int seg = 0; seg < nArc; ++seg) {
      const double a0 = seg * dtheta;          // segment start angle
      const double am = a0 + dtheta / 2.0;     // segment mid angle
      const Point3 onStart = arcPoint(a0);     // ON-arc pole at segment start (weight 1)

      // BETWEEN pole: intersection of the two segment-endpoint tangents. Lies along the
      // mid direction at radius/cos(Δθ/2); weight = cos(Δθ/2). On-axis ⇒ just P0.
      Point3 between = P0;
      if (!onAxis) {
        const double rr = radius / wm;
        const double cx = rr * std::cos(am);
        const double cy = rr * std::sin(am);
        between = foot + e0 * cx + e1 * cy;
      }

      const std::size_t base = static_cast<std::size_t>(i) * nV + 2 * seg;
      r.surface.poles[base + 0] = onStart;
      r.surface.weights[base + 0] = wP * 1.0;   // ON-arc pole weight (arc weight 1)
      r.surface.poles[base + 1] = between;
      r.surface.weights[base + 1] = wP * wm;    // BETWEEN weight = wProfile · cos(Δθ/2), ALWAYS
    }
    // Final ON-arc pole at the full swept angle (weight 1).
    const Point3 onEnd = arcPoint(angle);
    const std::size_t last = static_cast<std::size_t>(i) * nV + (nV - 1);
    r.surface.poles[last] = onEnd;
    r.surface.weights[last] = wP * 1.0;
  }

  // If the profile was NON-rational, the only weights are the arc's cos-half-angle pattern —
  // that is correct (a non-rational profile revolved is still a rational surface). Keep them.
  r.vParams = {0.0, 1.0};
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Variable-section sweep (scale + twist along the spine, transform-then-skin).
// ─────────────────────────────────────────────────────────────────────────────

SweepResult sweepVariable(const BsplineCurveData& section, const BsplineCurveData& trajectory,
                          const Dir3& sectionNormal, const std::vector<double>& scales,
                          const std::vector<double>& twists, int stations, int degreeV) {
  SweepResult r;
  if (stations < 2) return r;
  if (!section.weights.empty() || !trajectory.weights.empty()) return r;  // rational — decline
  if (!wellFormed(section) || !wellFormed(trajectory)) return r;
  if (!sectionNormal.valid()) return r;

  const std::vector<BsplineCurveData> placed =
      placeSectionsVariable(section, trajectory, sectionNormal, scales, twists, stations);
  if (static_cast<int>(placed.size()) != stations) return r;  // sampling / degenerate / bad field

  const SkinResult skin = skinSurface(std::span<const BsplineCurveData>(placed), degreeV);
  if (!skin.ok) return r;

  r.surface = skin.surface;
  r.vParams = skin.vParams;
  r.ok = true;
  return r;
}

SweepResult sweepRationalVariable(const BsplineCurveData& section,
                                  const BsplineCurveData& trajectory, const Dir3& sectionNormal,
                                  const std::vector<double>& scales,
                                  const std::vector<double>& twists, int stations, int degreeV) {
  SweepResult r;
  if (stations < 2) return r;
  if (section.weights.empty()) return r;         // non-rational — use sweepVariable
  if (!trajectory.weights.empty()) return r;     // rational spine — decline
  if (!wellFormedRational(section) || !wellFormed(trajectory)) return r;
  if (!sectionNormal.valid()) return r;

  // The similarity (scale+twist) preserves weights exactly; place then rational-skin.
  const std::vector<BsplineCurveData> placed =
      placeSectionsVariable(section, trajectory, sectionNormal, scales, twists, stations);
  if (static_cast<int>(placed.size()) != stations) return r;

  const SkinResult skin =
      skinRationalSurface(std::span<const BsplineCurveData>(placed), degreeV);
  if (!skin.ok) return r;

  r.surface = skin.surface;
  r.vParams = skin.vParams;
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Two-rail sweep (anchor the section to both rails, transform-then-skin).
// ─────────────────────────────────────────────────────────────────────────────

SweepResult sweepTwoRail(const BsplineCurveData& section, const BsplineCurveData& rail0,
                         const BsplineCurveData& rail1, const Dir3& sectionNormal, int anchor0,
                         int anchor1, int stations, int degreeV) {
  SweepResult r;
  if (stations < 2) return r;
  if (!section.weights.empty() || !rail0.weights.empty() || !rail1.weights.empty())
    return r;                                    // rational — decline
  if (!wellFormed(section) || !wellFormed(rail0) || !wellFormed(rail1)) return r;
  if (!sectionNormal.valid()) return r;

  const std::vector<BsplineCurveData> placed =
      placeSectionsTwoRail(section, rail0, rail1, sectionNormal, anchor0, anchor1, stations);
  if (static_cast<int>(placed.size()) != stations) return r;  // degenerate rails / anchors / spine

  const SkinResult skin = skinSurface(std::span<const BsplineCurveData>(placed), degreeV);
  if (!skin.ok) return r;

  r.surface = skin.surface;
  r.vParams = skin.vParams;
  r.ok = true;
  return r;
}

SweepResult sweepRationalTwoRail(const BsplineCurveData& section, const BsplineCurveData& rail0,
                                 const BsplineCurveData& rail1, const Dir3& sectionNormal,
                                 int anchor0, int anchor1, int stations, int degreeV) {
  SweepResult r;
  if (stations < 2) return r;
  if (section.weights.empty()) return r;         // non-rational — use sweepTwoRail
  if (!rail0.weights.empty() || !rail1.weights.empty()) return r;  // rational rails — decline
  if (!wellFormedRational(section) || !wellFormed(rail0) || !wellFormed(rail1)) return r;
  if (!sectionNormal.valid()) return r;

  const std::vector<BsplineCurveData> placed =
      placeSectionsTwoRail(section, rail0, rail1, sectionNormal, anchor0, anchor1, stations);
  if (static_cast<int>(placed.size()) != stations) return r;

  const SkinResult skin =
      skinRationalSurface(std::span<const BsplineCurveData>(placed), degreeV);
  if (!skin.ok) return r;

  r.surface = skin.surface;
  r.vParams = skin.vParams;
  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI
