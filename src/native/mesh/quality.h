// SPDX-License-Identifier: Apache-2.0
//
// quality.h — native, OCCT-free, TetGen-free tetrahedral mesh quality metrics.
//
// ALWAYS compiled (default MIT build): this module has ZERO dependency on the
// optional AGPL TetGen backend. All metrics use only the 4 CORNER nodes of each
// element, so a C3D10 element scores identically to the C3D4 of its corners
// (straight-edge mid-nodes lie at edge midpoints). Formulas are the standard
// Verdict / Shewchuk definitions cited in quality.cpp.
//
#ifndef CYBERCAD_NATIVE_MESH_QUALITY_H
#define CYBERCAD_NATIVE_MESH_QUALITY_H

#include <vector>

namespace cybercad::native::mesh {

// Aggregate quality report over a whole tet mesh. Angles in degrees. A single
// regular tetrahedron scores minDihedral==maxDihedral==70.5288,
// minScaledJacobian==meanScaledJacobian==1, maxAspectRatio==1.
struct QualityResult {
  double minDihedral = 0.0, maxDihedral = 0.0;          // degrees
  double minScaledJacobian = 0.0, meanScaledJacobian = 0.0;
  double maxAspectRatio = 0.0;
  std::vector<int> flagged;   // element ids with scaledJ < minScaledJacobian
  bool valid = false;         // false on empty / degenerate input
};

// Compute the quality report from the flat POD mesh. Uses only the 4 corners of
// each element (works for C3D4 and C3D10). `nodesPerElement` is 4 or 10; only the
// first 4 entries per element are read. Returns valid=false on empty / degenerate
// input (any near-zero volume tet or a non-finite metric).
QualityResult quality(const double* nodes, int nodeCount,
                      const int* elements, int elementCount, int nodesPerElement,
                      double minScaledJacobian);

}  // namespace cybercad::native::mesh

#endif  // CYBERCAD_NATIVE_MESH_QUALITY_H
