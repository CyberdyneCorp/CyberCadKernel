// SPDX-License-Identifier: Apache-2.0
//
// chamfer_edge_variable_freeform.h — native NURBS CHAMFER, two additive generalizations
// of the Wave-G constant chamfer (blend/chamfer_edge_nurbs.h, NURBS roadmap Layer-4):
//
//   (1) VARIABLE-DISTANCE chamfer. The setback distance is no longer constant along the
//       edge but varies — linearly from d0 (edge start) to d1 (edge end), or by a
//       caller-supplied per-station field. The two setback rails are traced with the
//       PER-STATION distance (each measured ALONG its face's surface, exactly as the
//       constant builder does), and the now-tapered rails are lofted into a ruled bevel
//       band (Piegl & Tiller). The bevel face is no longer a uniform ruled strip (the
//       chord width tapers), but the loft is unchanged. When d0==d1 (or a uniform field)
//       this reproduces the Wave-G constant chamfer BYTE-FOR-BYTE (same setbackPoint
//       calls, same over-large guard, same loft) — the reduction oracle.
//
//   (2) FREEFORM-EDGE chamfer. The two base faces are GENERAL FREEFORM NURBS surfaces
//       (not analytic primitives). At each edge station we FOOTPOINT the edge point onto
//       each freeform surface, then MARCH the setback point ALONG that surface a
//       surface-distance d — a normal-section geodesic march whose step length is
//       measured by the surface's FIRST FUNDAMENTAL FORM (E,F,G), NOT the 3-D chord.
//       This is the honest along-surface offset: on a NURBS-represented PLANE it is the
//       straight in-plane offset (I is the identity metric → arc length = 3-D length), on
//       a NURBS-represented CYLINDER wrapping it is the circumferential arc (R·Δψ = d), so
//       a NURBS plane / cylinder REPRODUCES the analytic chamfer (the freeform-reduction
//       oracle). A genuinely freeform bump gets rails that lie ON each face at surface-
//       distance d. HONEST-DECLINES when a setback would leave the face domain or the
//       rails self-lap — never a self-intersecting bevel.
//
// Both entry points are ADDITIVE. This header includes chamfer_edge_nurbs.h and REUSES
// its Substrate / EdgeStation / ChamferResult / ChamferDecline / helpers verbatim; it
// touches NONE of the Wave-G entry points (chamfer_edge_symmetric/_asymmetric/
// _distance_angle stay byte-unchanged). It reuses the freeform footpoint / first-
// fundamental-form machinery from fillet_edge_g2_freeform.h (ffdetail::Surface,
// LocalGeom, footpoint) so the surface-distance measure is the SAME first-fundamental
// form the freeform G2 fillet reads its curvatures from.
//
// CLEAN-ROOM. Uses only src/native/math (bspline surface eval + vec) + the two sibling
// blend headers. OCCT-FREE (0 OCCT/Geom/BRep/TK refs). Header-only. clang++ -std=c++20.
// Piegl & Tiller (ruled loft; surface derivatives A3.6/A4.4); first-fundamental-form
// surface-distance march for the freeform setback.
//
#ifndef CYBERCAD_NATIVE_BLEND_CHAMFER_EDGE_VARIABLE_FREEFORM_H
#define CYBERCAD_NATIVE_BLEND_CHAMFER_EDGE_VARIABLE_FREEFORM_H

#include "native/blend/chamfer_edge_nurbs.h"
#include "native/blend/fillet_edge_g2_freeform.h"
#include "native/math/vec.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace chamfer_nurbs {

// ═══════════════════════════════════════════════════════════════════════════════
// (1) VARIABLE-DISTANCE CHAMFER
// ═══════════════════════════════════════════════════════════════════════════════

// Build the chamfer between faceA (subA) and faceB (subB) along `edge`, with PER-STATION
// setbacks dA[k] on faceA and dB[k] on faceB. This is the variable-distance generalization
// of buildChamfer: it traces the two setback curves with each station's own distance,
// runs the SAME over-large / non-crossing guard, and lofts the SAME ruled bevel band. If
// every dA[k]==dA[0] and dB[k]==dB[0] it is IDENTICAL to buildChamfer(subA,subB,edge,
// dA[0],dB[0]) (same setbackPoint calls in the same order) — the constant-reduction path.
inline ChamferResult buildChamferVariable(const Substrate& subA, const Substrate& subB,
                                          const std::vector<EdgeStation>& edge,
                                          const std::vector<double>& dA,
                                          const std::vector<double>& dB) {
  ChamferResult out;
  if (edge.size() < 2 || dA.size() != edge.size() || dB.size() != edge.size()) {
    out.decline = ChamferDecline::BadArguments;
    return out;
  }
  if (subA.kind == SubstrateKind::Freeform || subB.kind == SubstrateKind::Freeform) {
    out.decline = ChamferDecline::UnsupportedSubstrate;
    return out;
  }
  for (std::size_t k = 0; k < edge.size(); ++k) {
    if (!(dA[k] > kCnEps) || !(dB[k] > kCnEps)) {
      out.decline = ChamferDecline::BadArguments;
      return out;
    }
  }

  out.setbackA.reserve(edge.size());
  out.setbackB.reserve(edge.size());

  for (std::size_t k = 0; k < edge.size(); ++k) {
    const auto& st = edge[k];
    const cmath::Vec3 nx = cmath::cross(st.nA, st.nB);
    if (!(cmath::norm(nx) > kCnEps) && cmath::norm(st.nA - st.nB) < kCnEps) {
      out.decline = ChamferDecline::DegenerateDihedral;
      return out;
    }
    const cmath::Vec3 hintA = interiorHintFor(st.tangent, st.nB);
    const cmath::Vec3 hintB = interiorHintFor(st.tangent, st.nA);
    const auto inA = faceInward(st.tangent, st.nA, hintA);
    const auto inB = faceInward(st.tangent, st.nB, hintB);
    if (!inA || !inB) {
      out.decline = ChamferDecline::DegenerateDihedral;
      return out;
    }
    const auto pA = setbackPoint(subA, st.p, st.tangent, *inA, dA[k]);
    const auto pB = setbackPoint(subB, st.p, st.tangent, *inB, dB[k]);
    if (!pA || !pB) {
      out.decline = ChamferDecline::UnsupportedSubstrate;
      return out;
    }
    out.setbackA.push_back(*pA);
    out.setbackB.push_back(*pB);
  }

  // Over-large guard — IDENTICAL to buildChamfer (chord non-crossing + rail sweep).
  {
    double refSign = 0.0;
    for (std::size_t k = 0; k < edge.size(); ++k) {
      const cmath::Vec3 chord = out.setbackB[k] - out.setbackA[k];
      const cmath::Vec3 bis = edge[k].nA + edge[k].nB;
      const cmath::Vec3 ref = cmath::cross(edge[k].tangent, bis);
      const double sgn = cmath::dot(chord, ref);
      if (k == 0) {
        refSign = sgn;
      } else if (refSign != 0.0 && sgn * refSign < 0.0) {
        out.decline = ChamferDecline::OverLargeSetback;
        return out;
      }
      if (cmath::norm(chord) < kCnEps) {
        out.decline = ChamferDecline::OverLargeSetback;
        return out;
      }
    }
    for (std::size_t k = 0; k + 1 < edge.size(); ++k) {
      const cmath::Vec3 eStep = edge[k + 1].p - edge[k].p;
      if (cmath::norm(eStep) < kCnEps) continue;
      const cmath::Vec3 aStep = out.setbackA[k + 1] - out.setbackA[k];
      const cmath::Vec3 bStep = out.setbackB[k + 1] - out.setbackB[k];
      if (cmath::dot(aStep, eStep) < 0.0 || cmath::dot(bStep, eStep) < 0.0) {
        out.decline = ChamferDecline::OverLargeSetback;
        return out;
      }
    }
  }

  // Ruled chamfer face: R(t,τ) = (1−τ)·cA(t) + τ·cB(t) — SAME loft as buildChamfer.
  const std::size_t ns = edge.size();
  out.triangles.reserve((ns - 1) * 2);
  for (std::size_t k = 0; k + 1 < ns; ++k) {
    const cmath::Point3 a = out.setbackA[k];
    const cmath::Point3 b = out.setbackB[k];
    const cmath::Point3 c = out.setbackB[k + 1];
    const cmath::Point3 d = out.setbackA[k + 1];
    out.triangles.push_back({a, b, c});
    out.triangles.push_back({a, c, d});
  }

  std::vector<cmath::Point3> corners;
  corners.reserve(2 * ns);
  for (std::size_t k = 0; k < ns; ++k) {
    corners.push_back(out.setbackA[k]);
    corners.push_back(out.setbackB[k]);
  }
  out.planarityResidual = planeResidual(corners);
  out.planarFace = (out.planarityResidual <= kCnEps * 1e3);
  return out;
}

// Arc-length fraction of each edge station (0 at start, 1 at end), by cumulative chord
// length along the polyline. Used to place the LINEAR taper d(t) = d0 + (d1−d0)·t.
inline std::vector<double> edgeArcFraction(const std::vector<EdgeStation>& edge) {
  const std::size_t n = edge.size();
  std::vector<double> cum(n, 0.0);
  for (std::size_t k = 1; k < n; ++k)
    cum[k] = cum[k - 1] + cmath::distance(edge[k].p, edge[k - 1].p);
  const double total = cum.back();
  std::vector<double> frac(n, 0.0);
  if (total > kCnEps)
    for (std::size_t k = 0; k < n; ++k) frac[k] = cum[k] / total;
  else
    for (std::size_t k = 0; k < n; ++k) frac[k] = static_cast<double>(k) / static_cast<double>(n - 1);
  return frac;
}

// ── PUBLIC: VARIABLE-DISTANCE (linear taper) ────────────────────────────────────
// SYMMETRIC variable chamfer: each face is set back the SAME per-station distance, which
// tapers LINEARLY from d0 (edge start) to d1 (edge end) by arc-length fraction. d0==d1
// reproduces chamfer_edge_symmetric(d0) exactly.
inline ChamferResult chamfer_edge_variable(const Substrate& faceA, const Substrate& faceB,
                                           const std::vector<EdgeStation>& edge, double d0,
                                           double d1) {
  if (edge.size() < 2 || !(d0 > kCnEps) || !(d1 > kCnEps)) {
    ChamferResult out;
    out.decline = ChamferDecline::BadArguments;
    return out;
  }
  const auto frac = edgeArcFraction(edge);
  std::vector<double> dA(edge.size()), dB(edge.size());
  for (std::size_t k = 0; k < edge.size(); ++k) {
    dA[k] = d0 + (d1 - d0) * frac[k];
    dB[k] = dA[k];
  }
  return buildChamferVariable(faceA, faceB, edge, dA, dB);
}

// ASYMMETRIC variable chamfer: faceA tapers d0A→d1A, faceB tapers d0B→d1B (independent
// legs), each linearly by arc-length fraction. Equal endpoints reproduce the constant
// asymmetric chamfer.
inline ChamferResult chamfer_edge_variable_asymmetric(const Substrate& faceA,
                                                      const Substrate& faceB,
                                                      const std::vector<EdgeStation>& edge,
                                                      double d0A, double d1A, double d0B,
                                                      double d1B) {
  if (edge.size() < 2 || !(d0A > kCnEps) || !(d1A > kCnEps) || !(d0B > kCnEps) ||
      !(d1B > kCnEps)) {
    ChamferResult out;
    out.decline = ChamferDecline::BadArguments;
    return out;
  }
  const auto frac = edgeArcFraction(edge);
  std::vector<double> dA(edge.size()), dB(edge.size());
  for (std::size_t k = 0; k < edge.size(); ++k) {
    dA[k] = d0A + (d1A - d0A) * frac[k];
    dB[k] = d0B + (d1B - d0B) * frac[k];
  }
  return buildChamferVariable(faceA, faceB, edge, dA, dB);
}

// FIELD variable chamfer: the caller supplies the per-station setback FIELD directly
// (dA[k], dB[k], one per edge station) — a sampled distance profile of any shape. This is
// the most general variable form; the linear-taper entry points above are the common case.
inline ChamferResult chamfer_edge_variable_field(const Substrate& faceA, const Substrate& faceB,
                                                 const std::vector<EdgeStation>& edge,
                                                 const std::vector<double>& dA,
                                                 const std::vector<double>& dB) {
  return buildChamferVariable(faceA, faceB, edge, dA, dB);
}

// ═══════════════════════════════════════════════════════════════════════════════
// (2) FREEFORM-EDGE CHAMFER
// ═══════════════════════════════════════════════════════════════════════════════

namespace ff = cybercad::native::blend::ffdetail;

// A freeform chamfer edge station: a point on the shared crease with an edge tangent, and
// warm-start (u,v) params for the footpoint onto each freeform face. `materialHint` points
// from the crease INTO the solid wedge (the direction the chamfer removes material from);
// it disambiguates which way EACH face sets back — the setback marches along each face on
// the material side, so its in-face inward direction is oriented to AGREE with this hint.
// It need not be exact (only its sign relative to each face's in-plane perpendicular is
// used), and it is INDEPENDENT of the surfaces' normal orientations (a freeform NURBS
// normal may point either way), so a robust caller supplies e.g. the wedge bisector.
struct FreeformEdgeStation {
  cmath::Point3 p;              // point on the edge (lies on both faces)
  cmath::Vec3 tangent;         // unit edge tangent at p
  cmath::Vec3 materialHint{0, 0, 0};  // into the solid wedge (orients both setbacks)
  double uA0 = 0.5, vA0 = 0.5;   // faceA footpoint warm start
  double uB0 = 0.5, vB0 = 0.5;   // faceB footpoint warm start
};

// Decline reasons for the freeform chamfer (superset of the analytic ChamferDecline; the
// freeform march adds the domain-exit / non-convergence cases).
enum class FreeformChamferDecline {
  None,
  BadArguments,          // non-positive d / too few stations
  DegenerateSurface,     // Su×Sv null at a footpoint (no tangent plane / normal)
  FootpointDiverged,     // the edge point does not footpoint onto a face
  LeftDomain,            // the surface-distance march ran past the [0,1]² face domain
  SelfLap,               // the two rails cross / a rail swept backward (over-large)
};

// The freeform chamfer result: the two setback rails (on faceA / faceB, at surface-
// distance d), the lofted bevel band, and the decline reason.
struct FreeformChamferResult {
  std::vector<cmath::Point3> setbackA;
  std::vector<cmath::Point3> setbackB;
  std::vector<std::array<cmath::Point3, 3>> triangles;
  FreeformChamferDecline decline = FreeformChamferDecline::None;
  bool ok() const { return decline == FreeformChamferDecline::None && !triangles.empty(); }
};

// ── SURFACE-DISTANCE MARCH (first fundamental form) ─────────────────────────────
// March a point along freeform surface `s`, starting at parameter (u0,v0), a SURFACE
// DISTANCE `d` in the in-face direction whose surface image is the unit world vector
// `worldDir0` (which must lie in the tangent plane at the start). This is a normal-section
// geodesic-style march: at each sub-step we
//   * read the local geometry (point, Su, Sv, unit normal, first fundamental form),
//   * PROJECT the current world direction into the tangent plane and re-normalize (so the
//     path hugs the surface — on a plane this is a no-op, on a cylinder it curls the
//     direction around the wall),
//   * solve the first-fundamental-form system [E F; F G](du,dv)=(Su·t, Sv·t) for the
//     parameter-space direction whose image is that tangent (the standard I-projection),
//   * step (u,v) so the surface arc length of the sub-step is exactly d/nSub:
//         ds = sqrt(E du² + 2F du·dv + G dv²)  (the first fundamental form line element),
//     scale (du,dv) by (d/nSub)/ds, advance, and carry the stepped world point's tangent
//     forward. Accumulating nSub sub-steps gives a total surface distance d (≤ rounding).
// Returns the final surface point + its converged (u,v), or nullopt if a sub-step leaves
// the padded [0,1]² domain or the surface degenerates.
struct MarchResult {
  cmath::Point3 p;
  double u = 0, v = 0;
};
inline std::optional<MarchResult> marchSurfaceDistance(const ff::Surface& s, double u0, double v0,
                                                       const cmath::Vec3& worldDir0, double d,
                                                       int nSub = 24) {
  double u = u0, v = v0;
  cmath::Vec3 worldDir = worldDir0;
  const double dn = cmath::norm(worldDir);
  if (!(dn > kCnCurvedEps)) return std::nullopt;
  worldDir = worldDir / dn;
  const double sub = d / static_cast<double>(nSub);

  for (int it = 0; it < nSub; ++it) {
    const ff::LocalGeom g = ff::localGeom(s, u, v);
    if (!g.ok) return std::nullopt;
    // Project the world direction into the tangent plane, re-normalize.
    cmath::Vec3 t = worldDir - g.n * cmath::dot(worldDir, g.n);
    const double tn = cmath::norm(t);
    if (!(tn > kCnCurvedEps)) return std::nullopt;
    t = t / tn;
    // Parameter-space direction (du,dv) whose image is t: [E F; F G](du,dv)=(Su·t,Sv·t).
    const double a = cmath::dot(g.Su, t);
    const double b = cmath::dot(g.Sv, t);
    const double det = g.E * g.G - g.F * g.F;
    if (!(std::fabs(det) > kCnCurvedEps)) return std::nullopt;
    double du = (a * g.G - b * g.F) / det;
    double dv = (b * g.E - a * g.F) / det;
    // Arc-length line element of this param-direction; scale so ds == sub.
    const double ds = std::sqrt(g.E * du * du + 2.0 * g.F * du * dv + g.G * dv * dv);
    if (!(ds > kCnCurvedEps)) return std::nullopt;
    const double scale = sub / ds;
    du *= scale;
    dv *= scale;
    u += du;
    v += dv;
    if (!std::isfinite(u) || !std::isfinite(v)) return std::nullopt;
    if (u < -0.02 || u > 1.02 || v < -0.02 || v > 1.02) return std::nullopt;   // left the face
    // Carry the world direction forward as the stepped surface tangent (re-projected next
    // iteration). Using the freshly-stepped surface point's tangent keeps the path a
    // second-order-correct geodesic march.
    worldDir = t;
  }
  const cmath::Point3 p = ff::surfacePoint(s, u, v);
  return MarchResult{cmath::Point3{p.x, p.y, p.z}, u, v};
}

// Build the freeform chamfer between two freeform NURBS faces `faceA`,`faceB` along the
// crease `edge` (a polyline of stations, ≥2), with per-station setbacks dA[k]/dB[k]
// measured ALONG each face's surface. At each station we footpoint the edge point onto
// each face (to seat the march params + read the tangent plane), pick the in-face inward
// direction (perpendicular to the edge tangent, into the material away from the OTHER
// face), march the surface-distance setback, and loft the two rails. HONEST-DECLINES the
// WHOLE result (empty triangles + reason) on any footpoint / domain / self-lap failure.
inline FreeformChamferResult buildFreeformChamfer(const ff::Surface& faceA, const ff::Surface& faceB,
                                                  const std::vector<FreeformEdgeStation>& edge,
                                                  const std::vector<double>& dA,
                                                  const std::vector<double>& dB, int nSub = 24) {
  FreeformChamferResult out;
  if (edge.size() < 2 || dA.size() != edge.size() || dB.size() != edge.size()) {
    out.decline = FreeformChamferDecline::BadArguments;
    return out;
  }
  for (std::size_t k = 0; k < edge.size(); ++k)
    if (!(dA[k] > kCnEps) || !(dB[k] > kCnEps)) {
      out.decline = FreeformChamferDecline::BadArguments;
      return out;
    }

  out.setbackA.reserve(edge.size());
  out.setbackB.reserve(edge.size());

  for (std::size_t k = 0; k < edge.size(); ++k) {
    const auto& st = edge[k];
    const cmath::Point3 q{st.p.x, st.p.y, st.p.z};
    const auto fa = ff::footpoint(faceA, q, st.uA0, st.vA0);
    const auto fb = ff::footpoint(faceB, q, st.uB0, st.vB0);
    if (!fa || !fb) {
      out.decline = FreeformChamferDecline::FootpointDiverged;
      return out;
    }
    if (!fa->g.ok || !fb->g.ok) {
      out.decline = FreeformChamferDecline::DegenerateSurface;
      return out;
    }
    const cmath::Vec3 nA = fa->g.n;   // unit surface normal of faceA at the edge
    const cmath::Vec3 nB = fb->g.n;   // unit surface normal of faceB at the edge
    // In-face inward direction on a face with unit normal `n`: perpendicular to the edge
    // tangent, in that face's tangent plane, oriented to AGREE with the material hint
    // (into the solid wedge). We project the edge tangent into the tangent plane, take
    // t×n as the in-plane perpendicular, and flip it to the material side. The hint is
    // normal-orientation-independent, so it works for either sign of a freeform NURBS
    // normal. Falls back to the OTHER face's normal as the hint when none is supplied.
    auto inwardOn = [&](const cmath::Vec3& n, const cmath::Vec3& nOther) -> std::optional<cmath::Vec3> {
      cmath::Vec3 tin = st.tangent - n * cmath::dot(st.tangent, n);   // edge tangent in plane
      if (!(cmath::norm(tin) > kCnCurvedEps)) return std::nullopt;
      tin = tin / cmath::norm(tin);
      cmath::Vec3 dir = cmath::cross(tin, n);                          // in-plane ⟂ edge
      if (!(cmath::norm(dir) > kCnCurvedEps)) return std::nullopt;
      dir = dir / cmath::norm(dir);
      // Orient into the material. Prefer the caller's material hint (projected into this
      // face's tangent plane); fall back to "away from the OTHER face's normal".
      cmath::Vec3 hint = st.materialHint;
      cmath::Vec3 hintIn = hint - n * cmath::dot(hint, n);            // hint in this plane
      if (!(cmath::norm(hintIn) > kCnCurvedEps)) hintIn = interiorHintFor(tin, nOther);
      if (cmath::dot(dir, hintIn) < 0.0) dir = -dir;
      return dir;
    };
    const auto inA = inwardOn(nA, nB);
    const auto inB = inwardOn(nB, nA);
    if (!inA || !inB) {
      out.decline = FreeformChamferDecline::DegenerateSurface;
      return out;
    }
    const auto mA = marchSurfaceDistance(faceA, fa->u, fa->v, *inA, dA[k], nSub);
    const auto mB = marchSurfaceDistance(faceB, fb->u, fb->v, *inB, dB[k], nSub);
    if (!mA || !mB) {
      out.decline = FreeformChamferDecline::LeftDomain;
      return out;
    }
    out.setbackA.push_back(mA->p);
    out.setbackB.push_back(mB->p);
  }

  // Self-lap / over-large guard. On a FREEFORM crease the analytic chord-vs-bisector
  // reference sign (chord·(t×(nA+nB))) is fragile: when the chord runs nearly
  // perpendicular to that reference (a symmetric wedge) it is ~0 and its sign is pure
  // rounding noise — so we do NOT use it. The robust, orientation-independent signals are
  //   (a) the chord cA→cB never collapses (rails never coincide), AND never REVERSES
  //       direction from one station to the next (chord[k]·chord[k−1] > 0) — a reversal
  //       means the two rails swapped sides (a folded band);
  //   (b) each rail sweeps WITH the edge (rail-step · edge-step ≥ 0) — a backward sweep
  //       means the setback overshot and lapped to the wrong side.
  {
    for (std::size_t k = 0; k < edge.size(); ++k) {
      const cmath::Vec3 chord = out.setbackB[k] - out.setbackA[k];
      if (cmath::norm(chord) < kCnCurvedEps) {
        out.decline = FreeformChamferDecline::SelfLap;   // rails coincident
        return out;
      }
      if (k > 0) {
        const cmath::Vec3 prev = out.setbackB[k - 1] - out.setbackA[k - 1];
        if (cmath::dot(chord, prev) < 0.0) {             // chord reversed → rails swapped
          out.decline = FreeformChamferDecline::SelfLap;
          return out;
        }
      }
    }
    for (std::size_t k = 0; k + 1 < edge.size(); ++k) {
      const cmath::Vec3 eStep = edge[k + 1].p - edge[k].p;
      if (cmath::norm(eStep) < kCnCurvedEps) continue;
      const cmath::Vec3 aStep = out.setbackA[k + 1] - out.setbackA[k];
      const cmath::Vec3 bStep = out.setbackB[k + 1] - out.setbackB[k];
      if (cmath::dot(aStep, eStep) < 0.0 || cmath::dot(bStep, eStep) < 0.0) {
        out.decline = FreeformChamferDecline::SelfLap;   // rail swept backward
        return out;
      }
    }
  }

  // Ruled bevel loft between the two freeform-traced rails.
  const std::size_t ns = edge.size();
  out.triangles.reserve((ns - 1) * 2);
  for (std::size_t k = 0; k + 1 < ns; ++k) {
    const cmath::Point3 a = out.setbackA[k];
    const cmath::Point3 b = out.setbackB[k];
    const cmath::Point3 c = out.setbackB[k + 1];
    const cmath::Point3 d = out.setbackA[k + 1];
    out.triangles.push_back({a, b, c});
    out.triangles.push_back({a, c, d});
  }
  return out;
}

// ── PUBLIC: FREEFORM-EDGE chamfer ───────────────────────────────────────────────
// Constant-distance freeform chamfer: symmetric setback d on both freeform faces,
// sampled at `nStations` (the edge is supplied as ≥2 stations; nStations is honoured by
// the caller when building the edge). Reduces to the analytic chamfer on a NURBS plane /
// cylinder.
inline FreeformChamferResult chamfer_edge_freeform(const ff::Surface& faceA,
                                                   const ff::Surface& faceB,
                                                   const std::vector<FreeformEdgeStation>& edge,
                                                   double d, int nSub = 24) {
  std::vector<double> dA(edge.size(), d), dB(edge.size(), d);
  return buildFreeformChamfer(faceA, faceB, edge, dA, dB, nSub);
}

// Variable-distance freeform chamfer: linear taper d0→d1 (same value on both faces),
// combining generalizations (1) and (2). d0==d1 reduces to chamfer_edge_freeform(d0).
inline FreeformChamferResult chamfer_edge_freeform_variable(
    const ff::Surface& faceA, const ff::Surface& faceB,
    const std::vector<FreeformEdgeStation>& edge, double d0, double d1, int nSub = 24) {
  if (edge.size() < 2) {
    FreeformChamferResult out;
    out.decline = FreeformChamferDecline::BadArguments;
    return out;
  }
  // Arc-length fraction along the edge polyline.
  const std::size_t n = edge.size();
  std::vector<double> cum(n, 0.0);
  for (std::size_t k = 1; k < n; ++k)
    cum[k] = cum[k - 1] + cmath::distance(edge[k].p, edge[k - 1].p);
  const double total = cum.back();
  std::vector<double> dA(n), dB(n);
  for (std::size_t k = 0; k < n; ++k) {
    const double frac = (total > kCnEps) ? cum[k] / total
                                         : static_cast<double>(k) / static_cast<double>(n - 1);
    dA[k] = d0 + (d1 - d0) * frac;
    dB[k] = dA[k];
  }
  return buildFreeformChamfer(faceA, faceB, edge, dA, dB, nSub);
}

}  // namespace chamfer_nurbs

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CHAMFER_EDGE_VARIABLE_FREEFORM_H
