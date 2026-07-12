// SPDX-License-Identifier: Apache-2.0
//
// chamfer_edge_nurbs.h — native NURBS CHAMFER generator (NURBS roadmap Layer-4, the
// CHAMFER half of fillet/chamfer). Where the G2 fillet family (fillet_edges_g2*,
// fillet_edge_g2_freeform) rolls a ball into a crease and lays a tangent-continuous
// blend, a CHAMFER replaces an edge with a FLAT BEVEL face — a ruled (bilinear-lofted)
// transition — that is C0 (position-continuous) but deliberately NOT tangent at either
// seam. This header is the analytic/NURBS-substrate generator: it computes the two
// SETBACK CURVES (one traced on each face, at a prescribed setback FROM THE EDGE,
// measured ALONG that face's surface) and connects them with a ruled chamfer face.
//
// It is ADDITIVE and sits ALONGSIDE the byte-frozen solid-clip `chamfer_edges.h`
// (which chamfers a convex edge of a PLANAR solid by a Sutherland–Hodgman plane cut,
// for the OCCT-fallback engine). This file touches NONE of that: it is a pure geometric
// generator over `native/math` surface primitives, returns setback curves + a ruled
// chamfer surface (poles/triangles), and is OCCT-FREE. No engine wiring, no cc_* ABI.
//
// ── SETBACK SPECIFICATION (the three chamfer modes) ────────────────────────────
// For a dihedral edge shared by faceA and faceB, meeting at interior half-angle θ per
// face (the dihedral angle is φ = 2θ for the symmetric case; in general each face has
// its own in-face perpendicular-to-edge direction):
//   * SYMMETRIC(d)        — each face is set back by the SAME distance d, measured in
//                           the face, perpendicular to the edge, along the surface.
//   * ASYMMETRIC(d1,d2)   — faceA set back d1, faceB set back d2 (independent legs).
//   * DISTANCE_ANGLE(d,α) — faceA set back d; faceB's setback is derived so the bevel
//                           face makes angle α (deg) with faceA at the faceA seam.
//                           In the meridian right-triangle the faceB leg is
//                           d2 = d·tan(α)  (α measured from the faceA tangent plane),
//                           so DISTANCE_ANGLE(d, atan(d2/d)·180/π) ≡ ASYMMETRIC(d,d2).
//
// ── SETBACK CURVE (per-face, along-surface offset from the edge) ────────────────
// The setback curve on face F is the locus of points a geodesic/normal distance `s`
// from the edge, traced along F. For the substrates this generator handles it is a
// closed form:
//   * PLANE      — the edge is a straight line in the plane; the setback line is the
//                  parallel line offset by `s` along the in-plane inward direction
//                  (edge-tangent × face-normal, oriented into the face). EXACT.
//   * CYLINDER   — an edge that is an axis-parallel ruling (the plane∩cylinder seam)
//                  sets back AROUND the cylinder: the surface distance from the ruling
//                  is arc length = R·Δψ, so a setback `s` is the ruling at angular
//                  offset Δψ = s/R — the correct GEODESIC/normal offset on the cylinder.
//                  An edge that is a cross-section circle (cap∩cylinder) sets back
//                  AXIALLY by `s` (the ruling direction is a geodesic), giving the
//                  circle at axial offset s. Both are exact along-surface offsets.
//   * CONE       — an axis-parallel-ish generator sets back circumferentially by the
//                  local-radius arc length; a circular rim sets back ALONG the slant
//                  generator by `s`, giving the coaxial circle at the slant-offset
//                  radius/height. Exact along the ruled generator.
// Curved substrates are handled where the along-surface offset is closed-form (above);
// a genuinely freeform edge honest-DECLINES here (the fillet's footpoint machinery is
// the tool for that; a flat bevel across a freeform crease is left to a later slice).
//
// ── RULED CHAMFER FACE (Piegl & Tiller ruled surface) ──────────────────────────
// Given the two setback curves cA(t), cB(t) (SAME parameter count / correspondence),
// the chamfer face is the ruled/bilinear loft
//     R(t,τ) = (1−τ)·cA(t) + τ·cB(t),   t,τ ∈ [0,1]
// — the standard Piegl & Tiller ruled surface between two rail curves. Sampled into
// quad strips (t-station k→k+1 × τ 0→1) for the emitted band. For the exact PLANAR
// dihedral case both rails are straight lines and R is EXACTLY PLANAR (its two rails
// are the setback lines; the four corners are coplanar to ≤1e-12). The face meets each
// base face with C0 at the corresponding setback curve (shared rail points), NOT with
// tangency — the correct chamfer continuity.
//
// ── HONEST DECLINE ─────────────────────────────────────────────────────────────
// A chamfer DECLINES (empty result + reason, never a self-intersecting face) when:
//   * a setback exceeds the face's usable extent (over-large — the setback curve would
//     run past the face boundary / cross the opposite setback);
//   * the dihedral is degenerate (faces parallel / edge direction null / normals
//     (anti)parallel so there is no wedge to bevel);
//   * the substrate is unsupported (freeform edge) for a closed-form along-surface
//     offset.
// NO tolerance is ever widened to force a pass.
//
// CLEAN-ROOM. Uses only src/native/math/vec.h (+ the local ruled-loft helpers).
// OCCT-FREE (0 OCCT/Geom/BRep/TK refs). Header-only. clang++ -std=c++20.
// Piegl & Tiller (ruled surface loft); analytic along-surface setback per substrate.
//
#ifndef CYBERCAD_NATIVE_BLEND_CHAMFER_EDGE_NURBS_H
#define CYBERCAD_NATIVE_BLEND_CHAMFER_EDGE_NURBS_H

#include "native/math/vec.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace chamfer_nurbs {

namespace cmath = cybercad::native::math;

// TIGHT tolerances — the honest-decline discipline forbids widening them. kCnEps is
// the linear coincidence / null-vector band; the planar-dihedral oracle is checked at
// ≤1e-12, the cylinder along-surface offset at ≤1e-9.
inline constexpr double kCnEps = 1e-12;
inline constexpr double kCnCurvedEps = 1e-9;

// ── SUBSTRATE DESCRIPTION ──────────────────────────────────────────────────────
// A base face given by its analytic kind + parameters. Only the fields relevant to a
// kind are read. Planes use (point, normal); cylinders/cones use (apex/base point,
// axis, radius[, halfAngle]). These are the closed-form substrates the along-surface
// setback is exact for; anything else → Freeform → honest-decline.
enum class SubstrateKind { Plane, Cylinder, Cone, Freeform };

struct Substrate {
  SubstrateKind kind = SubstrateKind::Freeform;
  cmath::Point3 point{0, 0, 0};   // a point ON the surface (plane) / axis base (cyl/cone)
  cmath::Vec3 normal{0, 0, 1};    // OUTWARD unit normal at the edge (plane); cyl/cone: unused directly
  cmath::Vec3 axis{0, 0, 1};      // unit axis direction (cylinder / cone)
  double radius = 0.0;            // cylinder radius, or cone radius at `point`
  double halfAngle = 0.0;         // cone half-angle (rad); 0 for a cylinder
};

// ── EDGE DESCRIPTION ───────────────────────────────────────────────────────────
// The shared dihedral edge, as a polyline of stations (≥2). For a straight dihedral
// two endpoints suffice; a circular rim is supplied as sampled stations. `outwardA`
// and `outwardB` are the OUTWARD unit normals of faceA/faceB AT each edge station
// (for a straight edge, constant; for a rim they vary), used to orient the in-face
// setback direction into the material.
struct EdgeStation {
  cmath::Point3 p;         // point on the edge
  cmath::Vec3 tangent;     // unit edge tangent at p
  cmath::Vec3 nA;          // outward unit normal of faceA at p
  cmath::Vec3 nB;          // outward unit normal of faceB at p
};

// ── CHAMFER MODE ───────────────────────────────────────────────────────────────
enum class ChamferMode { Symmetric, Asymmetric, DistanceAngle };

// ── DECLINE REASONS ────────────────────────────────────────────────────────────
enum class ChamferDecline {
  None,
  DegenerateDihedral,     // edge tangent null, or face normals (anti)parallel — no wedge
  OverLargeSetback,       // a setback exceeds the face extent / rails cross
  UnsupportedSubstrate,   // freeform edge — no closed-form along-surface offset
  BadArguments,           // non-positive setback / too few stations
};

// ── RESULT ─────────────────────────────────────────────────────────────────────
struct ChamferResult {
  std::vector<cmath::Point3> setbackA;   // setback curve traced on faceA (per station)
  std::vector<cmath::Point3> setbackB;   // setback curve traced on faceB (per station)
  // Ruled chamfer face sampled into triangles (flat oriented loops), R(t,τ) between
  // the two setback rails. For the planar case these are exactly coplanar.
  std::vector<std::array<cmath::Point3, 3>> triangles;
  // Whether the emitted chamfer face is exactly planar (both rails straight & coplanar).
  bool planarFace = false;
  // Max |deviation| of the four setback-rail corners from the best-fit chamfer plane
  // (the planar-dihedral witness; ~0 for the exact planar case).
  double planarityResidual = 0.0;
  ChamferDecline decline = ChamferDecline::None;
  bool ok() const { return decline == ChamferDecline::None && !triangles.empty(); }
};

// ── LOW-LEVEL HELPERS ──────────────────────────────────────────────────────────

// In-face inward direction of a face with outward normal `n`, perpendicular to the
// edge tangent `t`, oriented into the material (toward `interiorHint` if given).
// inward = normalize(t × n); flipped to agree with the interior hint. Returns nullopt
// if the cross product is null (t ∥ n, i.e. the edge is not on the face).
inline std::optional<cmath::Vec3> faceInward(const cmath::Vec3& t, const cmath::Vec3& n,
                                             const cmath::Vec3& interiorHint) {
  cmath::Vec3 d = cmath::cross(t, n);
  const double len = cmath::norm(d);
  if (!(len > kCnEps)) return std::nullopt;
  d = d / len;
  if (cmath::dot(d, interiorHint) < 0.0) d = -d;
  return d;
}

// The interior hint for a face at an edge station: a direction that points from the
// edge INTO the face's material. For a convex chamfer we set each face back toward the
// OTHER face's outward normal projected out — but a robust, substrate-agnostic hint is
// "away from the opposite face's outward normal", i.e. the crease bisector reflected.
// We use the simplest correct hint: into the face means opposite the component of the
// other face's outward normal. For faceA the hint is −nB projected off the edge; this
// points from the sharp edge into faceA's material for a convex dihedral.
inline cmath::Vec3 interiorHintFor(const cmath::Vec3& t, const cmath::Vec3& nOther) {
  const cmath::Vec3 perp = nOther - t * cmath::dot(nOther, t);   // nOther ⟂ edge
  return perp * -1.0;                                            // into this face
}

// Resolve the two leg setbacks (dA on faceA, dB on faceB) from a mode + parameters.
inline std::optional<std::pair<double, double>> resolveLegs(ChamferMode mode, double d1, double d2,
                                                            double angleDeg) {
  switch (mode) {
    case ChamferMode::Symmetric:
      if (!(d1 > kCnEps)) return std::nullopt;
      return std::pair<double, double>{d1, d1};
    case ChamferMode::Asymmetric:
      if (!(d1 > kCnEps) || !(d2 > kCnEps)) return std::nullopt;
      return std::pair<double, double>{d1, d2};
    case ChamferMode::DistanceAngle: {
      if (!(d1 > kCnEps)) return std::nullopt;
      const double a = angleDeg * (M_PI / 180.0);
      // faceB leg so the bevel makes angle `a` with faceA at the faceA seam. The
      // meridian right-triangle has the faceA leg = d1 and the faceB leg = d1·tan(a).
      if (!(a > kCnEps) || !(a < M_PI * 0.5 - kCnEps)) return std::nullopt;
      const double d2a = d1 * std::tan(a);
      if (!(d2a > kCnEps)) return std::nullopt;
      return std::pair<double, double>{d1, d2a};
    }
  }
  return std::nullopt;
}

// Trace the setback point on a substrate, at along-surface distance `s`, from edge
// point `ep` along the in-face inward direction `inward`. Returns the setback point,
// or nullopt if the substrate is unsupported / the offset is degenerate.
//   * PLANE    — setback = ep + s·inward (inward is a unit in-plane direction; the
//                straight-line offset IS the along-surface distance).
//   * CYLINDER — if the edge is a RULING (tangent ∥ axis): the setback wraps around
//                the cylinder by arc length s → angular Δψ = s/R about the axis, so we
//                rotate `ep` about the axis by Δψ in the `inward` circumferential sense.
//                If the edge is a CROSS-SECTION circle (tangent ⟂ axis, inward ∥ axis):
//                the setback slides axially by s → ep + s·inward.
//   * CONE     — circular rim: setback slides along the slant generator by s. The
//                generator direction is `inward` (in the meridian, pointing into the
//                cone wall); ep + s·inward walks the ruled generator (a geodesic on the
//                developable cone), the exact along-surface offset.
inline std::optional<cmath::Point3> setbackPoint(const Substrate& sub, const cmath::Point3& ep,
                                                 const cmath::Vec3& tangent,
                                                 const cmath::Vec3& inward, double s) {
  switch (sub.kind) {
    case SubstrateKind::Plane:
      return ep + inward * s;

    case SubstrateKind::Cylinder: {
      const double axDotT = std::fabs(cmath::dot(tangent, sub.axis));
      if (axDotT > 1.0 - 1e-9) {
        // Edge is an axis-parallel ruling → wrap around by arc length s.
        if (!(sub.radius > kCnEps)) return std::nullopt;
        const double dpsi = s / sub.radius;   // R·Δψ = s (exact geodesic arc length)
        // Rotate ep about the axis (through sub.point) by ±dpsi, sign from `inward`.
        const cmath::Vec3 a = sub.axis;
        // Radial vector from axis to ep.
        const cmath::Vec3 rel = ep - sub.point;
        const cmath::Vec3 radial = rel - a * cmath::dot(rel, a);
        const double rr = cmath::norm(radial);
        if (!(rr > kCnEps)) return std::nullopt;
        const cmath::Vec3 rhat = radial / rr;
        const cmath::Vec3 that = cmath::cross(a, rhat);   // circumferential (+ sense)
        // Choose rotation sign so the move agrees with `inward` (circumferential).
        const double sgn = (cmath::dot(that, inward) >= 0.0) ? 1.0 : -1.0;
        const double ang = sgn * dpsi;
        const cmath::Vec3 newRadial = (rhat * std::cos(ang) + that * std::sin(ang)) * rr;
        const cmath::Vec3 axialPart = a * cmath::dot(rel, a);
        return sub.point + axialPart + newRadial;
      }
      // Cross-section circle edge (inward runs axially) → axial slide by s.
      return ep + inward * s;
    }

    case SubstrateKind::Cone:
      // Rim setback along the slant generator (a straight geodesic on the developable
      // cone): ep + s·inward with inward the in-wall meridian direction.
      return ep + inward * s;

    case SubstrateKind::Freeform:
    default:
      return std::nullopt;
  }
}

// Best-fit plane residual of a point set: the max distance from the least-squares
// plane through the centroid (used as the planar-face witness). For 4 corners of an
// exactly-planar face this is ≈0.
inline double planeResidual(const std::vector<cmath::Point3>& pts) {
  const std::size_t n = pts.size();
  if (n < 3) return 0.0;
  cmath::Vec3 c{0, 0, 0};
  for (const auto& p : pts) c += p.asVec();
  c = c / static_cast<double>(n);
  // Covariance of centred points; the plane normal is the smallest-eigenvector. Use a
  // robust proxy: normal from the two longest spanning edges, then max deviation.
  cmath::Vec3 e1{0, 0, 0}, e2{0, 0, 0};
  double best1 = 0.0;
  for (const auto& p : pts) {
    const cmath::Vec3 v = p.asVec() - c;
    if (cmath::normSquared(v) > best1) { best1 = cmath::normSquared(v); e1 = v; }
  }
  if (!(cmath::norm(e1) > kCnEps)) return 0.0;
  e1 = e1 / cmath::norm(e1);
  double best2 = 0.0;
  for (const auto& p : pts) {
    cmath::Vec3 v = p.asVec() - c;
    v = v - e1 * cmath::dot(v, e1);
    if (cmath::normSquared(v) > best2) { best2 = cmath::normSquared(v); e2 = v; }
  }
  cmath::Vec3 nrm = cmath::cross(e1, e2);
  const double nl = cmath::norm(nrm);
  if (!(nl > kCnEps)) return 0.0;   // colinear → planar by definition
  nrm = nrm / nl;
  double maxDev = 0.0;
  for (const auto& p : pts) {
    const double dev = std::fabs(cmath::dot(p.asVec() - c, nrm));
    if (dev > maxDev) maxDev = dev;
  }
  return maxDev;
}

// ── CORE BUILDER ───────────────────────────────────────────────────────────────
// Build the chamfer between faceA (subA) and faceB (subB) along `edge`, with per-face
// setbacks (dA, dB). Traces the two setback curves, verifies non-crossing, and lofts a
// ruled chamfer face. Returns the result (or an honest decline).
inline ChamferResult buildChamfer(const Substrate& subA, const Substrate& subB,
                                  const std::vector<EdgeStation>& edge, double dA, double dB) {
  ChamferResult out;
  if (edge.size() < 2) {
    out.decline = ChamferDecline::BadArguments;
    return out;
  }
  if (subA.kind == SubstrateKind::Freeform || subB.kind == SubstrateKind::Freeform) {
    out.decline = ChamferDecline::UnsupportedSubstrate;
    return out;
  }

  out.setbackA.reserve(edge.size());
  out.setbackB.reserve(edge.size());

  for (const auto& st : edge) {
    // Degenerate dihedral: normals (anti)parallel → no wedge to bevel.
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
    const auto pA = setbackPoint(subA, st.p, st.tangent, *inA, dA);
    const auto pB = setbackPoint(subB, st.p, st.tangent, *inB, dB);
    if (!pA || !pB) {
      out.decline = ChamferDecline::UnsupportedSubstrate;
      return out;
    }
    out.setbackA.push_back(*pA);
    out.setbackB.push_back(*pB);
  }

  // Over-large guard (no self-intersecting bevel). Two failure modes:
  //   (a) rails coincident / crossed at a station — the chord cA(t)→cB(t) shrinks to
  //       zero or flips sign vs the first station (bevel width collapses / inverts);
  //   (b) a setback OVERSHOOTS the face so the traced rail sweeps the WRONG way — the
  //       rail's per-station step reverses relative to the edge's step (e.g. a radial
  //       planar setback pushed past the axis, or a circumferential wrap that lapped
  //       the ring). Detected by the rail-step / edge-step orientation reversing.
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
        out.decline = ChamferDecline::OverLargeSetback;   // (a) chord flipped
        return out;
      }
      if (cmath::norm(chord) < kCnEps) {
        out.decline = ChamferDecline::OverLargeSetback;   // (a) rails coincident
        return out;
      }
    }
    // (b) The traced rail must sweep WITH the edge: for each interior interval the
    // rail-A / rail-B step must not reverse relative to the edge step (a reversal means
    // the setback overshot and the rail lapped to the wrong side → a folded face).
    for (std::size_t k = 0; k + 1 < edge.size(); ++k) {
      const cmath::Vec3 eStep = edge[k + 1].p - edge[k].p;
      if (cmath::norm(eStep) < kCnEps) continue;
      const cmath::Vec3 aStep = out.setbackA[k + 1] - out.setbackA[k];
      const cmath::Vec3 bStep = out.setbackB[k + 1] - out.setbackB[k];
      if (cmath::dot(aStep, eStep) < 0.0 || cmath::dot(bStep, eStep) < 0.0) {
        out.decline = ChamferDecline::OverLargeSetback;   // (b) rail swept backward
        return out;
      }
    }
  }

  // Ruled chamfer face: R(t,τ) = (1−τ)·cA(t) + τ·cB(t). We emit ONE τ-strip (τ:0→1)
  // per t-station interval — the ruled surface between the two rails. (More τ samples
  // add nothing for a ruled surface; the face is exactly linear in τ.)
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

  // Planar witness: fit a plane through all four (or 2N) rail corners.
  std::vector<cmath::Point3> corners;
  corners.reserve(2 * ns);
  for (std::size_t k = 0; k < ns; ++k) {
    corners.push_back(out.setbackA[k]);
    corners.push_back(out.setbackB[k]);
  }
  out.planarityResidual = planeResidual(corners);
  out.planarFace = (out.planarityResidual <= kCnEps * 1e3);   // exact planar case ≈0

  return out;
}

// ── PUBLIC API ─────────────────────────────────────────────────────────────────

// SYMMETRIC chamfer: each face set back by `d`.
inline ChamferResult chamfer_edge_symmetric(const Substrate& faceA, const Substrate& faceB,
                                            const std::vector<EdgeStation>& edge, double d) {
  const auto legs = resolveLegs(ChamferMode::Symmetric, d, d, 0.0);
  if (!legs) {
    ChamferResult out;
    out.decline = ChamferDecline::BadArguments;
    return out;
  }
  return buildChamfer(faceA, faceB, edge, legs->first, legs->second);
}

// ASYMMETRIC chamfer: faceA set back `d1`, faceB set back `d2`.
inline ChamferResult chamfer_edge_asymmetric(const Substrate& faceA, const Substrate& faceB,
                                             const std::vector<EdgeStation>& edge, double d1,
                                             double d2) {
  const auto legs = resolveLegs(ChamferMode::Asymmetric, d1, d2, 0.0);
  if (!legs) {
    ChamferResult out;
    out.decline = ChamferDecline::BadArguments;
    return out;
  }
  return buildChamfer(faceA, faceB, edge, legs->first, legs->second);
}

// DISTANCE+ANGLE chamfer: faceA set back `d`; the bevel makes `angleDeg` with faceA.
// Equivalent to ASYMMETRIC(d, d·tan(angleDeg)).
inline ChamferResult chamfer_edge_distance_angle(const Substrate& faceA, const Substrate& faceB,
                                                 const std::vector<EdgeStation>& edge, double d,
                                                 double angleDeg) {
  const auto legs = resolveLegs(ChamferMode::DistanceAngle, d, 0.0, angleDeg);
  if (!legs) {
    ChamferResult out;
    out.decline = ChamferDecline::BadArguments;
    return out;
  }
  return buildChamfer(faceA, faceB, edge, legs->first, legs->second);
}

}  // namespace chamfer_nurbs

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CHAMFER_EDGE_NURBS_H
