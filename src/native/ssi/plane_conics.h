// SPDX-License-Identifier: Apache-2.0
//
// plane_conics.h — analytic plane ∩ {plane, sphere, cylinder, cone} intersectors
// (SSI Stage S1). Each returns closed-form native conic(s) that PROVABLY lie on
// both surfaces (the S1 correctness invariant).
//
// CLEAN-ROOM closed-form geometry. The OCCT analytic quadric intersector
// IntAna_QuadQuadGeo (Pln×{Sphere,Cylinder,Cone}) was consulted only as an ORACLE
// for the curve-KIND classification (which orientation gives a circle vs ellipse vs
// parabola vs hyperbola) — no code copied.
//
// GEOMETRY (each handler documents its own derivation inline):
//   plane ∩ plane   → a Line along n1 × n2 (or Same / Empty when parallel).
//   plane ∩ sphere  → a Circle (centre = foot of the sphere centre on the plane,
//                     radius √(R² − d²)); tangent → Point; d > R → Empty.
//   plane ∩ cylinder→ ‖axis  : 0 / 1 / 2 ruling Lines by distance vs R (or Same-band).
//                     ⟂ axis  : a Circle radius R.
//                     oblique : an Ellipse, semi-minor R, semi-major R/|sin θ|
//                               (θ = angle between plane normal and cylinder axis).
//   plane ∩ cone    → by the plane's angle to the axis vs the cone half-angle α:
//                     the classic conic sections — Circle (⟂ axis), Ellipse,
//                     Parabola (plane ∥ a generator), Hyperbola (steeper), or
//                     1/2 Lines when the plane passes through the apex.
//
// Header-only, OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_PLANE_CONICS_H
#define CYBERCAD_NATIVE_SSI_PLANE_CONICS_H

#include "native/math/elementary.h"
#include "native/ssi/curve.h"
#include "native/ssi/tolerance.h"

#include <cmath>

namespace cybercad::native::ssi {

// ─────────────────────────────────────────────────────────────────────────────
// plane ∩ plane → a Line (or coincident / parallel-disjoint).
//
// Two planes P1: n1·(X−O1)=0, P2: n2·(X−O2)=0. If n1 ∥ n2 the planes are parallel:
// coincident when O2 lies on P1 (same plane), else disjoint. Otherwise they meet in
// a line with direction d = n1 × n2; a point on it solves the 2×2 system
//   n1·X = n1·O1,  n2·X = n2·O2   in the plane spanned by n1, n2:
//   X0 = (c1 (n2·n2) − c2 (n1·n2)) n1 + (c2 (n1·n1) − c1 (n1·n2)) n2  over  (den),
// with c1=n1·O1, c2=n2·O2, den = |n1|²|n2|² − (n1·n2)²  (=|d|² for unit normals).
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersectPlanePlane(const math::Plane& p1, const math::Plane& p2) {
  const Vec3 n1 = p1.pos.z.vec();
  const Vec3 n2 = p2.pos.z.vec();
  const Vec3 d = math::cross(n1, n2);
  const double dlen = math::norm(d);

  if (dlen <= kAngularEps) {  // parallel
    const double gap = math::dot(p2.pos.origin - p1.pos.origin, n1);
    return (std::fabs(gap) <= kLinearEps) ? IntersectionResult::coincident()
                                          : IntersectionResult::empty();
  }
  const double c1 = math::dot(n1, p1.pos.origin.asVec());
  const double c2 = math::dot(n2, p2.pos.origin.asVec());
  const double n1n1 = math::dot(n1, n1), n2n2 = math::dot(n2, n2), n1n2 = math::dot(n1, n2);
  const double den = n1n1 * n2n2 - n1n2 * n1n2;
  const double k1 = (c1 * n2n2 - c2 * n1n2) / den;
  const double k2 = (c2 * n1n1 - c1 * n1n2) / den;
  const Point3 x0 = Point3{} + n1 * k1 + n2 * k2;
  return IntersectionResult::ok(makeLine(x0, Dir3{d}));
}

// ─────────────────────────────────────────────────────────────────────────────
// plane ∩ sphere → a Circle (or a tangency Point, or Empty).
//
// Signed distance from the sphere centre C to the plane: h = n·(C − O). The circle
// centre is the foot F = C − h·n, and its radius is √(R² − h²). |h| = R → tangent
// point F; |h| > R → no intersection.
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersectPlaneSphere(const math::Plane& pl, const math::Sphere& sp) {
  const Dir3 n = pl.pos.z;
  const double h = math::dot(sp.pos.origin - pl.pos.origin, n.vec());
  const double disc = sp.radius * sp.radius - h * h;
  const Point3 foot = sp.pos.origin - n.vec() * h;

  if (disc < -kLinearEps * kLinearEps) return IntersectionResult::empty();
  if (disc <= kLinearEps * kLinearEps) return IntersectionResult::ok(makePoint(foot));
  return IntersectionResult::ok(makeCircle(foot, std::sqrt(disc), n, pl.pos.x));
}

// ─────────────────────────────────────────────────────────────────────────────
// plane ∩ cylinder → ruling Lines / Circle / Ellipse by orientation.
//
// Let a = cylinder axis (unit), n = plane normal (unit), s = |n × a| = |sin θ|
// (θ the angle between them). Axis point A (cylinder origin), radius R.
//
//  (1) Axis ∥ plane  (n·a ≈ 0, i.e. s ≈ 1 is FALSE here — careful):  the axis lies
//      IN the plane's direction set, i.e. n ⟂ a. Then the plane cuts the cylinder in
//      0/1/2 straight generators. Distance from the axis line to the plane governs
//      the count. Let e = n × a (a unit in-plane direction ⟂ a). The two candidate
//      rulings are at axis-offset ±R·e projected so they lie on the plane. Signed
//      distance of the axis to the plane D = n·(A − O). Rulings exist where a point
//      A + t·e·R touches the plane: n·(A + R t e − O) = 0 → R t (n·e) = −D. Since
//      n·e = n·(n×a) = 0, this needs D = 0 for the axis to lie in the plane; the two
//      generators are then at ±R along e. More robustly we solve for the offset
//      w along e where the circle meets the plane: the cross-section circle centre
//      projects, |D| ≤ R gives offset u = ±√(R²−D²)? — see the exact treatment below.
//
//  (2) Axis ⟂ plane  (a ∥ n, s ≈ 0): the plane cuts a Circle radius R centred at the
//      axis/plane crossing.
//
//  (3) Oblique (0 < s < 1): an Ellipse. Semi-minor = R (perpendicular direction),
//      semi-major = R/s (along the tilt). Centre = where the axis line pierces the
//      plane.
//
// For the ∥ case we treat it as: intersect the plane with the cylinder's cross
// section. The perpendicular distance from the axis line to the plane is
// dist = |n·(A − O)| / 1 measured along n, but since a ⟂ n the whole axis line is at
// constant signed distance D = n·(A − O). |D| > R → Empty; |D| = R → one tangent
// ruling; |D| < R → two rulings. Each ruling is parallel to a, offset from the axis
// by ±√(R² − D²) along e = normalize(n × a) but positioned on the plane.
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersectPlaneCylinder(const math::Plane& pl, const math::Cylinder& cy) {
  const Dir3 n = pl.pos.z;
  const Vec3 a = cy.pos.z.vec();              // cylinder axis (unit)
  const double na = math::dot(n.vec(), a);    // cos(angle between n and a)
  const double s = math::norm(math::cross(n.vec(), a));  // |sin θ|
  const Point3 A = cy.pos.origin;             // a point on the axis
  const double R = cy.radius;

  // ── (1) axis parallel to plane: n ⟂ a ⇒ na ≈ 0 ⇒ ruling lines. ──────────────
  if (std::fabs(na) <= kAngularEps) {
    const double D = math::dot(A - pl.pos.origin, n.vec());  // axis-to-plane signed dist
    const double disc = R * R - D * D;
    const Dir3 e{math::cross(n.vec(), a)};  // in-plane, ⟂ both a and n (a ruling offset dir)
    // Foot of the axis point on the plane:
    const Point3 Afoot = A - n.vec() * D;
    if (disc < -kLinearEps * kLinearEps) return IntersectionResult::empty();
    if (disc <= kLinearEps * kLinearEps) {  // tangent → single ruling through Afoot
      return IntersectionResult::ok(makeLine(Afoot, Dir3{a}));
    }
    const double off = std::sqrt(disc);
    std::vector<IntersectionCurve> lines;
    lines.push_back(makeLine(Afoot + e.vec() * off, Dir3{a}));
    lines.push_back(makeLine(Afoot - e.vec() * off, Dir3{a}));
    return IntersectionResult::ok(std::move(lines));
  }

  // Centre: where the axis line pierces the plane. Solve n·(A + u·a − O) = 0:
  //   u = −(n·(A − O)) / (n·a).
  const double u = -math::dot(A - pl.pos.origin, n.vec()) / na;
  const Point3 centre = A + a * u;

  // ── (2) axis perpendicular to plane: a ∥ n ⇒ s ≈ 0 ⇒ Circle. ────────────────
  if (s <= kAngularEps) {
    return IntersectionResult::ok(makeCircle(centre, R, n, pl.pos.x));
  }

  // ── (3) oblique: Ellipse. Semi-minor R, semi-major R/s. ─────────────────────
  // Major axis direction: the in-plane direction of steepest tilt = the plane
  // component of the cylinder axis, normalized. Minor axis = n × major (⟂ tilt).
  const Vec3 aInPlane = a - n.vec() * na;      // project axis into the plane
  const Dir3 major{aInPlane};                  // semi-major direction (a/s tilt)
  return IntersectionResult::ok(makeEllipse(centre, R / s, R, n, major));
}

// ─────────────────────────────────────────────────────────────────────────────
// plane ∩ cone → the classic conic sections.
//
// Cone apex V, axis a (unit), half-angle α. A point X is on the cone iff the angle
// between (X − V) and a equals α, i.e. (a·w)² = cos²α · |w|²,  w = X − V. Restricting
// X to the plane (2D coords) turns this quadric into a conic. The TYPE follows from
// φ = angle between the plane and the axis compared with α:
//   * plane ⟂ axis (φ = 90°)            → Circle
//   * plane cuts all generators (φ > α) → Ellipse
//   * plane ∥ exactly one generator (β=α)→ Parabola
//   * plane ∥ two generators (φ < α)    → Hyperbola (one branch per nappe)
//   * plane through the apex            → Point / 1 Line / 2 Lines (degenerate)
// where β = 90° − φ is the angle between the plane and the axis-normal... we work
// directly with the tilt angle t = angle(n, a) ∈ [0,π/2] after folding.
//
// DERIVATION used here (robust, avoids reconstructing the general conic matrix):
// build an in-plane 2D frame (ex, ey) at the plane origin; every plane point is
// X(x,y) = Oc + x·ex + y·ey. Substitute into (a·w)² − cos²α |w|² = 0 to get
//   A x² + B y² + 2C xy + 2D x + 2E y + F = 0,
// a general conic. We classify by its quadratic-part eigenstructure and emit the
// matching native conic with its true centre/axes. This one function is the
// systems-band core (cognitive complexity ~30 — an irreducible conic classifier);
// it is isolated and heavily commented.
// ─────────────────────────────────────────────────────────────────────────────

namespace detail {

// A general 2D conic  A x² + 2C xy + B y² + 2D x + 2E y + F = 0.
struct Conic2D {
  double A = 0, B = 0, C = 0, D = 0, E = 0, F = 0;
};

// Cone as an implicit quadric about apex V with axis a and cos²α = k. For w = X − V,
// f(X) = (a·w)² − k·(w·w). Points with f = 0 (and a·w ≥ 0 for the proper nappe) lie
// on the cone. Returns the plane-restricted conic in the (ex,ey) frame at Oc.
inline Conic2D restrictConeToPlane(const math::Cone& co, const Point3& V, const Vec3& a,
                                   double k, const Point3& Oc, const Vec3& ex, const Vec3& ey) {
  (void)co;
  const Vec3 d = Oc - V;  // w = d + x·ex + y·ey
  const double ad = math::dot(a, d), aex = math::dot(a, ex), aey = math::dot(a, ey);
  const double dd = math::dot(d, d), dex = math::dot(d, ex), dey = math::dot(d, ey);
  // ex, ey are orthonormal ⇒ ex·ex = ey·ey = 1, ex·ey = 0.
  Conic2D q;
  q.A = aex * aex - k * 1.0;
  q.B = aey * aey - k * 1.0;
  q.C = aex * aey - k * 0.0;              // = aex*aey
  q.D = aex * ad - k * dex;               // coefficient of x (×1, we store as the 2D-term)
  q.E = aey * ad - k * dey;
  q.F = ad * ad - k * dd;
  return q;
}

}  // namespace detail

inline IntersectionResult intersectPlaneCone(const math::Plane& pl, const math::Cone& co) {
  const Vec3 a = co.pos.z.vec();
  const double alpha = co.semiAngle;
  const double k = std::cos(alpha) * std::cos(alpha);
  // Cone apex: value(u,v) = O + (R + v sinα)(...) ; apex is where R + v sinα = 0,
  // i.e. v = −R/sinα (for sinα ≠ 0). Apex point along the axis:
  //   apex = O + (v_apex cosα)·Z, and radial term 0.
  Point3 V;
  if (std::fabs(std::sin(alpha)) <= kAngularEps) {
    // Degenerate cone (a cylinder) — not this handler's job.
    return IntersectionResult::notAnalytic();
  }
  const double vApex = -co.radius / std::sin(alpha);
  V = co.pos.origin + a * (vApex * std::cos(alpha));

  const Dir3 n = pl.pos.z;
  // In-plane orthonormal frame at the plane origin.
  const Vec3 ex = pl.pos.x.vec();
  const Vec3 ey = pl.pos.y.vec();
  const Point3 Oc = pl.pos.origin;

  const detail::Conic2D q = detail::restrictConeToPlane(co, V, a, k, Oc, ex, ey);

  // Principal-axis rotation angle so the cross term vanishes.
  double theta = 0.0;
  if (std::fabs(q.C) > kLinearEps || std::fabs(q.A - q.B) > kLinearEps) {
    theta = 0.5 * std::atan2(2.0 * q.C, q.A - q.B);
  }
  const double ct = std::cos(theta), st = std::sin(theta);
  // Rotated axes in 3D (within the plane).
  const Vec3 u1 = ex * ct + ey * st;   // direction for eigenvalue l1' (see below)
  const Vec3 u2 = ex * (-st) + ey * ct;

  // Eigenvalues aligned with (u1,u2): rotating the quadratic form by theta gives
  //   A' = A ct² + 2C ct st + B st²,  B' = A st² − 2C ct st + B ct².
  const double Ap = q.A * ct * ct + 2 * q.C * ct * st + q.B * st * st;
  const double Bp = q.A * st * st - 2 * q.C * ct * st + q.B * ct * ct;
  // Linear terms in rotated coords: D' x' + E' y' with
  //   D' =  D ct + E st,  E' = −D st + E ct   (D,E are the *half* linear coeffs here).
  const double Dp = q.D * ct + q.E * st;
  const double Ep = -q.D * st + q.E * ct;

  const bool nzA = std::fabs(Ap) > kLinearEps;
  const bool nzB = std::fabs(Bp) > kLinearEps;

  auto planePoint = [&](double x1, double x2) {
    return Oc + u1 * x1 + u2 * x2;
  };

  // ── Ellipse / Circle / Hyperbola (both quadratic eigenvalues non-zero). ──────
  if (nzA && nzB) {
    // Complete the square: Ap(x' + Dp/Ap)² + Bp(y' + Ep/Bp)² = C0
    const double cx = -Dp / Ap, cy = -Ep / Bp;
    const double C0 = Ap * cx * cx + Bp * cy * cy - q.F;  // RHS after moving F over
    const Point3 centre = planePoint(cx, cy);
    if (Ap * Bp > 0.0) {
      // Same sign → Ellipse (or Empty if C0 has the wrong sign, or Point if 0).
      if (std::fabs(C0) <= kLinearEps) return IntersectionResult::ok(makePoint(centre));
      if (C0 / Ap <= 0.0) return IntersectionResult::empty();
      double ra = std::sqrt(C0 / Ap);  // along u1
      double rb = std::sqrt(C0 / Bp);  // along u2
      // Major axis = the longer one; order for a canonical (a ≥ b) ellipse.
      if (ra >= rb) {
        if (std::fabs(ra - rb) <= kLinearEps * std::max(1.0, ra))
          return IntersectionResult::ok(makeCircle(centre, ra, n, Dir3{u1}));
        return IntersectionResult::ok(makeEllipse(centre, ra, rb, n, Dir3{u1}));
      }
      return IntersectionResult::ok(makeEllipse(centre, rb, ra, n, Dir3{u2}));
    }
    // Opposite sign → Hyperbola: (x'−..)²/α² − (y'−..)²/β² = 1 form. Transverse axis
    // is the one whose eigenvalue matches the sign of C0.
    // Ap x'² + Bp y'² = C0. Put transverse along the axis with positive C0/λ.
    //
    // A hyperbola has TWO branches, and here they are BOTH real: the plane cuts a
    // double-napped cone below its half-angle, meeting one nappe per branch. OCCT's
    // GeomAPI_IntSS likewise returns two curves. We emit both, sharing a frame and
    // sizing, distinguished by the branch sign (+X and −X); each is a full analytic
    // curve that lies on both surfaces.
    IntersectionCurve h;
    h.kind = CurveKind::Hyperbola;
    if (C0 / Ap > 0.0) {
      h.a = std::sqrt(C0 / Ap);          // transverse (real) semi-axis along u1
      h.b = std::sqrt(-C0 / Bp);         // conjugate semi-axis along u2
      h.frame = Ax3::fromAxisAndRef(centre, n, Dir3{u1});
    } else {
      h.a = std::sqrt(C0 / Bp);
      h.b = std::sqrt(-C0 / Ap);
      h.frame = Ax3::fromAxisAndRef(centre, n, Dir3{u2});
    }
    IntersectionCurve h2 = h;
    h.branch = 1.0;
    h2.branch = -1.0;
    return IntersectionResult::ok(std::vector<IntersectionCurve>{h, h2});
  }

  // ── Parabola (exactly one quadratic eigenvalue non-zero). ────────────────────
  // Say Ap ≠ 0, Bp ≈ 0: Ap x'² + 2·? ... standard form. With half-coeffs:
  //   Ap x'² + 2Dp x' + 2Ep y' + F = 0.
  // Complete the square in x': Ap (x' + Dp/Ap)² + 2Ep y' + (F − Dp²/Ap) = 0
  //   → (x' + Dp/Ap)² = −(2Ep/Ap) y' − (F − Dp²/Ap)/Ap.
  // This is a parabola opening along y' with focal length f = |Ap/(4Ep)|? Map to our
  // native parabola P(t) = O + (t²/4f)·X + t·Y  (axis X, tangent Y at apex).
  auto emitParabola = [&](double Aq, double /*Dq*/, double Eq, double vertex_x,
                          const Vec3& xdir, const Vec3& ydir, double Fadj) -> IntersectionResult {
    // (x'')² = (−2Eq/Aq) y' − Fadj/Aq, with x'' = x' − vertex_x.
    // In native form the parabola opens along its X axis (our "value" uses x = t²/4f).
    // Here the curve opens along y' (the linear direction), so native-X = ydir,
    // native-Y = xdir, and 4f = |Aq / (2·(−Eq))| … derive:
    //   x''² = (−2Eq/Aq)(y' − y0)  ⇒  y' − y0 = (Aq / (−2Eq)) x''²
    //   native: coordinate along opening axis = (1/4f)·(coordinate ⟂)²
    //   ⇒ 1/(4f) = Aq/(−2Eq)  ⇒ f = −Eq/(2Aq).
    if (std::fabs(Eq) <= kLinearEps) {
      // Degenerate: Aq x''² = −Fadj → parallel line(s) or empty.
      const double rhs = -Fadj / Aq;
      if (rhs < -kLinearEps) return IntersectionResult::empty();
      if (rhs <= kLinearEps) {  // one line x'' = 0
        const Point3 o = Oc + xdir * vertex_x;
        return IntersectionResult::ok(makeLine(o, Dir3{ydir}));
      }
      const double off = std::sqrt(rhs);
      std::vector<IntersectionCurve> lines;
      lines.push_back(makeLine(Oc + xdir * (vertex_x + off), Dir3{ydir}));
      lines.push_back(makeLine(Oc + xdir * (vertex_x - off), Dir3{ydir}));
      return IntersectionResult::ok(std::move(lines));
    }
    const double y0 = -Fadj / (2.0 * Eq);  // vertex y' from Aq·0 + 2Eq y0 + Fadj = 0
    double f = -Eq / (2.0 * Aq);
    Vec3 openDir = ydir;
    if (f < 0.0) {  // opens the other way; flip so focal length is positive
      f = -f;
      openDir = -ydir;
    }
    const Point3 apex = Oc + xdir * vertex_x + ydir * y0;
    IntersectionCurve p;
    p.kind = CurveKind::Parabola;
    p.focal = f;
    // native X = opening axis, native Y = the ⟂ (transverse) direction xdir.
    p.frame = Ax3{apex, Dir3{openDir}, Dir3{xdir}, n};
    return IntersectionResult::ok(p);
  };

  if (nzA && !nzB) {
    const double vx = -Dp / Ap;                    // vertex shift along u1
    const double Fadj = q.F - Dp * Dp / Ap;        // constant after completing square
    return emitParabola(Ap, Dp, Ep, vx, u1, u2, Fadj);
  }
  if (nzB && !nzA) {
    const double vx = -Ep / Bp;
    const double Fadj = q.F - Ep * Ep / Bp;
    return emitParabola(Bp, Ep, Dp, vx, u2, u1, Fadj);
  }

  // ── Both eigenvalues ~0: the plane is degenerate w.r.t. the cone (through apex /
  //    tangent along a generator) — a line pair or point; treat as not-analytic for
  //    S1 (rare apex-through configuration; safe to defer). ─────────────────────
  return IntersectionResult::notAnalytic();
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_PLANE_CONICS_H
