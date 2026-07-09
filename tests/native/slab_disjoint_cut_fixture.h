// SPDX-License-Identifier: Apache-2.0
//
// slab_disjoint_cut_fixture.h — the reachable proof pose for MOAT M2b freeform↔analytic
// DISJOINT (MULTI-LUMP) CUT: the bowl-lidded convex-quad prism `A` PARTED by a central
// axis-aligned SLAB `B` into TWO lumps, and its closed-form CUT-volume oracle. Shared by
// the host analytic gate and the sim native-vs-OCCT gate. OCCT-FREE.
//
// Operand A = the bowl-lidded convex-quad prism of first_freeform_boolean_fixture
// (one Bézier bowl top + four planar side walls + a planar bottom).
// Operand B = a FINITE axis-aligned SLAB x∈[−s,s], y∈[−0.9,0.9], z∈[−0.9,0.9] built as
// SIX single-quad Plane faces (outward normals). Its TWO opposite faces (x=±s) both slice
// fully across A's Bézier wall, and its four other faces contain A, so removing the slab
// SEPARATES A into two lumps:
//   A − B  =  (A ∩ {x ≤ −s})  ⊎  (A ∩ {x ≥ +s})
//
// Closed-form CUT volume (no OCCT), the polynomial ∫∫ (H0 + a(x²+y²)) dA over each lump's
// footprint:
//   V(A − B) = V(A ∩ {x ≤ −s}) + V(A ∩ {x ≥ +s})
//
#ifndef CYBERCAD_TESTS_NATIVE_SLAB_DISJOINT_CUT_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_SLAB_DISJOINT_CUT_FIXTURE_H

#include "native/first_freeform_boolean_fixture.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"
#include "two_operand_fixture.h"

#include <array>
#include <vector>

namespace slab_disjoint_cut_fixture {

namespace topo = cybercad::native::topology;
namespace smath = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;
namespace tox = two_operand_fixture;

inline constexpr double kS = 0.10;   // slab half-width in x (x ∈ [−0.10, 0.10])
inline constexpr double kLat = 0.9;  // slab lateral half-extent in y and z (dwarfs A)

// The central slab B as a six-quad-face solid (outward normals; one shell), reusing the
// two-operand fixture's single-quad Plane-face builder (so `extractPolygons` yields ONE
// polygon per box side).
inline topo::Shape buildSlabB() {
  auto p = [](double x, double y, double z) { return smath::Point3{x, y, z}; };
  const double x0 = -kS, x1 = kS, y0 = -kLat, y1 = kLat, z0 = -kLat, z1 = kLat;
  std::vector<topo::Shape> faces;
  faces.push_back(tox::quadFace({p(x0, y0, z0), p(x0, y0, z1), p(x0, y1, z1), p(x0, y1, z0)}, {-1, 0, 0}));
  faces.push_back(tox::quadFace({p(x1, y0, z0), p(x1, y1, z0), p(x1, y1, z1), p(x1, y0, z1)}, {1, 0, 0}));
  faces.push_back(tox::quadFace({p(x0, y0, z0), p(x1, y0, z0), p(x1, y0, z1), p(x0, y0, z1)}, {0, -1, 0}));
  faces.push_back(tox::quadFace({p(x0, y1, z0), p(x0, y1, z1), p(x1, y1, z1), p(x1, y1, z0)}, {0, 1, 0}));
  faces.push_back(tox::quadFace({p(x0, y0, z0), p(x0, y1, z0), p(x1, y1, z0), p(x1, y0, z0)}, {0, 0, -1}));
  faces.push_back(tox::quadFace({p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1)}, {0, 0, 1}));
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
}

// Sutherland–Hodgman clip of the footprint quad keeping x ≤ xc (`ge=false`) or x ≥ xc.
inline std::vector<ffx::P2> clipX(bool ge, double xc) {
  std::vector<ffx::P2> in = ffx::quadXY(), out;
  const int n = static_cast<int>(in.size());
  for (int i = 0; i < n; ++i) {
    const ffx::P2 a = in[i], b = in[(i + 1) % n];
    const bool ai = ge ? a.x >= xc : a.x <= xc;
    const bool bi = ge ? b.x >= xc : b.x <= xc;
    if (ai) out.push_back(a);
    if (ai != bi) { const double t = (xc - a.x) / (b.x - a.x); out.push_back({xc, a.y + t * (b.y - a.y)}); }
  }
  return out;
}

// A box entirely to the +x side of A (x ∈ [2,3]) — NEITHER face straddles A's wall (the
// non-slab decline witness).
inline topo::Shape buildFarPlusXBox() {
  auto p = [](double x, double y, double z) { return smath::Point3{x, y, z}; };
  const double x0 = 2.0, x1 = 3.0, y0 = -0.9, y1 = 0.9, z0 = -0.9, z1 = 0.9;
  std::vector<topo::Shape> faces;
  faces.push_back(tox::quadFace({p(x0, y0, z0), p(x0, y0, z1), p(x0, y1, z1), p(x0, y1, z0)}, {-1, 0, 0}));
  faces.push_back(tox::quadFace({p(x1, y0, z0), p(x1, y1, z0), p(x1, y1, z1), p(x1, y0, z1)}, {1, 0, 0}));
  faces.push_back(tox::quadFace({p(x0, y0, z0), p(x1, y0, z0), p(x1, y0, z1), p(x0, y0, z1)}, {0, -1, 0}));
  faces.push_back(tox::quadFace({p(x0, y1, z0), p(x0, y1, z1), p(x1, y1, z1), p(x1, y1, z0)}, {0, 1, 0}));
  faces.push_back(tox::quadFace({p(x0, y0, z0), p(x0, y1, z0), p(x1, y1, z0), p(x1, y0, z0)}, {0, 0, -1}));
  faces.push_back(tox::quadFace({p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1)}, {0, 0, 1}));
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
}

// Closed-form volumes (no OCCT).
inline double lumpLoVolume() { return ffx::polyVolume(clipX(false, -kS)); }  // A ∩ {x ≤ −s}
inline double lumpHiVolume() { return ffx::polyVolume(clipX(true, kS)); }    // A ∩ {x ≥ +s}
inline double cutVolume() { return lumpLoVolume() + lumpHiVolume(); }         // A − B (two lumps)
inline double slabVolume() { return ffx::fullVolume() - cutVolume(); }        // A ∩ B (removed band)

}  // namespace slab_disjoint_cut_fixture

#endif  // CYBERCAD_TESTS_NATIVE_SLAB_DISJOINT_CUT_FIXTURE_H
