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

  // Sample the trajectory at `stations` params evenly across its clamped domain, taking
  // each station's point and unit tangent.
  double a = 0.0, b = 0.0;
  domainOf(trajectory, a, b);
  if (!(b > a)) return r;                                      // degenerate domain

  std::vector<Point3> pts;
  std::vector<Dir3> tans;
  pts.reserve(stations);
  tans.reserve(stations);
  for (int k = 0; k < stations; ++k) {
    const double t = a + (b - a) * (static_cast<double>(k) / (stations - 1));
    const PointTangent pt = evalPointTangent(trajectory, t);
    if (!pt.ok) return r;                                      // stationary spine point
    pts.push_back(pt.p);
    tans.push_back(pt.tangent);
  }

  // Coincident-trajectory guard: if every sampled point equals the first, there is no
  // path to sweep along (matches skin's coincident-sections decline).
  double pathLen = 0.0;
  for (int k = 1; k < stations; ++k) pathLen += distance(pts[k], pts[k - 1]);
  if (!(pathLen > 1e-12)) return r;

  // Rotation-minimizing normals along the spine, seeded from the section's plane normal.
  const std::vector<Vec3> rmf = rmfNormals(pts, tans, sectionNormal.vec());

  // Section reference basis (u0, v0, n0): n0 = the section plane normal, (u0,v0) span the
  // section plane. The placement at station k rotates this basis onto the station basis
  // (u_k = rmf[k], w_k = t_k, v_k = w_k × u_k), then translates so the section's ORIGIN
  // maps to the station point. The rotation R_k maps the section basis columns to the
  // station basis columns: R_k = [u_k v_k w_k] · [u0 v0 n0]^T.
  Vec3 u0, v0;
  orthonormalBasis(sectionNormal, u0, v0);
  const Vec3 n0 = sectionNormal.vec();
  // Inverse (= transpose) of the section reference basis [u0 v0 n0] (orthonormal).
  // Rows of B0^T are u0, v0, n0.

  std::vector<BsplineCurveData> placed;
  placed.reserve(stations);
  for (int k = 0; k < stations; ++k) {
    const Vec3 wk = tans[k].vec();
    const Vec3 uk = rmf[k];
    const Vec3 vk = cross(wk, uk);  // right-handed station basis

    // R_k = [uk vk wk] (columns) · [u0 v0 n0]^T (rows). Compose column-basis * row-basis.
    // For a vector p: R_k·p = uk·(u0·p) + vk·(v0·p) + wk·(n0·p).
    // Build the Mat3 explicitly (m[row][col]).
    Mat3 R;
    for (int rw = 0; rw < 3; ++rw) {
      R(rw, 0) = uk[rw] * u0.x + vk[rw] * v0.x + wk[rw] * n0.x;
      R(rw, 1) = uk[rw] * u0.y + vk[rw] * v0.y + wk[rw] * n0.y;
      R(rw, 2) = uk[rw] * u0.z + vk[rw] * v0.z + wk[rw] * n0.z;
    }
    // Affine: place the section's ORIGIN at the station point (translate = P_k, rotate
    // about the section origin). v' = R·v + P_k.
    const Transform place(R, pts[k].asVec());

    BsplineCurveData s = section;  // copy degree / knots / (empty) weights
    for (Point3& p : s.poles) p = place.applyToPoint(p);
    placed.push_back(std::move(s));
  }

  // Skin the K placed sections into one tensor-product surface (Layer 6).
  const SkinResult skin = skinSurface(std::span<const BsplineCurveData>(placed), degreeV);
  if (!skin.ok) return r;

  r.surface = skin.surface;
  r.vParams = skin.vParams;
  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI
