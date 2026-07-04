// SPDX-License-Identifier: Apache-2.0
//
// quality.cpp — native tetrahedral mesh quality metrics (OCCT-free, TetGen-free).
//
// Every metric is computed from the 4 CORNER nodes of a tet, so a C3D10 element
// scores identically to the C3D4 of its corners (straight-edge mid-nodes lie at
// edge midpoints). Formulas (with the property that a REGULAR tet scores
// dihedral = acos(1/3) = 70.5288 deg, scaled Jacobian = 1, aspect ratio = 1):
//
//   * Signed volume         V = (1/6) det[e12,e13,e14].
//   * Scaled Jacobian       sqrt(2) * min over the 4 corners of the triple product
//                           of the 3 outgoing UNIT edge vectors (each corner's
//                           three neighbours ordered as an EVEN permutation of the
//                           tet so a positively-oriented tet gives +1). Regular -> 1,
//                           sliver -> 0, inverted -> < 0.
//                           Ref: C. Stimpson et al., "The Verdict Geometric Quality
//                           Library", Sandia SAND2007-1751; P. Knupp, SIAM J. Sci.
//                           Comput. 23(1), 2001.
//   * Dihedral angles       6 per tet, from the 4 OUTWARD face normals:
//                           dihedral = acos(-(n_k . n_l)) for the two faces sharing
//                           the edge. Regular -> all 6 = 70.5288 deg.
//                           Ref: J. Shewchuk, "What Is a Good Linear Finite
//                           Element?", 2002.
//   * Aspect ratio          radius ratio R/(3r) = R*S/(9|V|) (Verdict "aspect beta")
//                           with S = sum of face areas, inradius r = 3|V|/S,
//                           circumradius R. Regular -> 1, degenerate -> infinity.
//                           Ref: A. Liu & B. Joe, BIT 34(2), 1994; Verdict.
//
#include "native/mesh/quality.h"

#include <cmath>
#include <limits>

namespace cybercad::native::mesh {

namespace {

struct V3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

V3 sub(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 cross(const V3& a, const V3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const V3& a) { return std::sqrt(dot(a, a)); }
double det(const V3& a, const V3& b, const V3& c) { return dot(a, cross(b, c)); }
V3 scale(const V3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
V3 add3(const V3& a, const V3& b, const V3& c) {
  return {a.x + b.x + c.x, a.y + b.y + c.y, a.z + b.z + c.z};
}

constexpr double kRadToDeg = 57.295779513082320876798154814105;

// Per-element metrics from the 4 corners. metricsFinite=false marks a degenerate
// element (coincident nodes / non-finite metric) so the whole mesh is invalidated.
struct TetMetrics {
  double scaledJacobian = 0.0;
  double minDihedral = 0.0, maxDihedral = 0.0;
  double aspectRatio = 0.0;
  bool degenerate = false;
};

// Triple product of the 3 outgoing UNIT edge vectors at one corner. `o` is the
// corner; a,b,c are its three neighbours in an orientation-preserving order.
double cornerUnitTriple(const V3& o, const V3& a, const V3& b, const V3& c) {
  const V3 ea = sub(a, o), eb = sub(b, o), ec = sub(c, o);
  const double la = norm(ea), lb = norm(eb), lc = norm(ec);
  if (la == 0.0 || lb == 0.0 || lc == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return det(ea, eb, ec) / (la * lb * lc);
}

// Outward normal (un-normalised) of the face (a,b,c) opposite corner `apex`.
V3 outwardNormal(const V3& a, const V3& b, const V3& c, const V3& apex) {
  V3 n = cross(sub(b, a), sub(c, a));
  if (dot(n, sub(a, apex)) < 0.0) {
    n = scale(n, -1.0);
  }
  return n;
}

double dihedralDeg(const V3& n0, const V3& n1) {
  const double l0 = norm(n0), l1 = norm(n1);
  if (l0 == 0.0 || l1 == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double cosang = -dot(n0, n1) / (l0 * l1);
  if (cosang > 1.0) cosang = 1.0;
  if (cosang < -1.0) cosang = -1.0;
  return std::acos(cosang) * kRadToDeg;
}

// Scaled Jacobian = sqrt(2) * min over corners. Corner neighbour orderings are
// even permutations of (1,2,3,4) so a positively-oriented tet yields +1 at each.
double scaledJacobian(const V3& c1, const V3& c2, const V3& c3, const V3& c4) {
  const double d1 = cornerUnitTriple(c1, c2, c3, c4);
  const double d2 = cornerUnitTriple(c2, c1, c4, c3);
  const double d3 = cornerUnitTriple(c3, c1, c2, c4);
  const double d4 = cornerUnitTriple(c4, c1, c3, c2);
  const double m = std::min(std::min(d1, d2), std::min(d3, d4));
  return std::sqrt(2.0) * m;
}

double circumradius(const V3& a, const V3& b, const V3& c, double det6V) {
  // R = | |a|^2(b x c) + |b|^2(c x a) + |c|^2(a x b) | / (2 det[a,b,c]).
  const V3 num = add3(scale(cross(b, c), dot(a, a)), scale(cross(c, a), dot(b, b)),
                      scale(cross(a, b), dot(c, c)));
  return norm(num) / (2.0 * std::abs(det6V));
}

double faceArea(const V3& a, const V3& b, const V3& c) {
  return 0.5 * norm(cross(sub(b, a), sub(c, a)));
}

TetMetrics tetMetrics(const V3& c1, const V3& c2, const V3& c3, const V3& c4) {
  TetMetrics m;
  const V3 a = sub(c2, c1), b = sub(c3, c1), c = sub(c4, c1);
  const double det6V = det(a, b, c);          // = 6 * signed volume
  const double absV = std::abs(det6V) / 6.0;

  m.scaledJacobian = scaledJacobian(c1, c2, c3, c4);

  const V3 n1 = outwardNormal(c2, c3, c4, c1);
  const V3 n2 = outwardNormal(c1, c3, c4, c2);
  const V3 n3 = outwardNormal(c1, c2, c4, c3);
  const V3 n4 = outwardNormal(c1, c2, c3, c4);
  const double dih[6] = {
      dihedralDeg(n3, n4),  // edge (1,2)
      dihedralDeg(n2, n4),  // edge (1,3)
      dihedralDeg(n2, n3),  // edge (1,4)
      dihedralDeg(n1, n4),  // edge (2,3)
      dihedralDeg(n1, n3),  // edge (2,4)
      dihedralDeg(n1, n2),  // edge (3,4)
  };
  m.minDihedral = dih[0];
  m.maxDihedral = dih[0];
  for (int i = 1; i < 6; ++i) {
    m.minDihedral = std::min(m.minDihedral, dih[i]);
    m.maxDihedral = std::max(m.maxDihedral, dih[i]);
  }

  const double S = faceArea(c2, c3, c4) + faceArea(c1, c3, c4) +
                   faceArea(c1, c2, c4) + faceArea(c1, c2, c3);
  if (absV == 0.0 || S == 0.0 || det6V == 0.0) {
    m.aspectRatio = std::numeric_limits<double>::infinity();
  } else {
    m.aspectRatio = circumradius(a, b, c, det6V) * S / (9.0 * absV);
  }

  m.degenerate = !(std::isfinite(m.scaledJacobian) && std::isfinite(m.minDihedral) &&
                   std::isfinite(m.maxDihedral) && std::isfinite(m.aspectRatio));
  return m;
}

}  // namespace

QualityResult quality(const double* nodes, int nodeCount, const int* elements,
                      int elementCount, int nodesPerElement, double minScaledJacobian) {
  QualityResult r;
  if (nodes == nullptr || elements == nullptr || nodeCount <= 0 || elementCount <= 0 ||
      nodesPerElement < 4) {
    return r;  // valid == false
  }

  auto corner = [&](int elem, int local) -> V3 {
    const int idx = elements[static_cast<long>(elem) * nodesPerElement + local];
    if (idx < 0 || idx >= nodeCount) {
      return {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
    }
    const long base = static_cast<long>(idx) * 3;
    return {nodes[base], nodes[base + 1], nodes[base + 2]};
  };

  r.minDihedral = std::numeric_limits<double>::infinity();
  r.maxDihedral = -std::numeric_limits<double>::infinity();
  r.minScaledJacobian = std::numeric_limits<double>::infinity();
  r.maxAspectRatio = 0.0;
  double sumScaledJ = 0.0;
  bool anyDegenerate = false;

  for (int e = 0; e < elementCount; ++e) {
    const TetMetrics m = tetMetrics(corner(e, 0), corner(e, 1), corner(e, 2), corner(e, 3));
    if (m.degenerate) {
      anyDegenerate = true;
      continue;
    }
    r.minDihedral = std::min(r.minDihedral, m.minDihedral);
    r.maxDihedral = std::max(r.maxDihedral, m.maxDihedral);
    r.minScaledJacobian = std::min(r.minScaledJacobian, m.scaledJacobian);
    r.maxAspectRatio = std::max(r.maxAspectRatio, m.aspectRatio);
    sumScaledJ += m.scaledJacobian;
    if (m.scaledJacobian < minScaledJacobian) {
      r.flagged.push_back(e);
    }
  }

  if (anyDegenerate) {
    return r;  // valid stays false
  }
  r.meanScaledJacobian = sumScaledJ / static_cast<double>(elementCount);
  r.valid = true;
  return r;
}

}  // namespace cybercad::native::mesh
