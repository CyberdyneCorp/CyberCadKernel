// SPDX-License-Identifier: AGPL-3.0-or-later
//
// tet_mesher.h — declaration surface of the TetGen-backed tetrahedral volume
// mesher. This header is intentionally TetGen-HEADER-FREE (only std::vector
// signatures cross it), but it is the declaration half of the AGPL-consuming
// pair: its ONLY implementation, tet_mesher.cpp, links the external AGPL TetGen
// library and is compiled solely under CYBERCAD_HAS_TETGEN. The default MIT build
// never compiles tet_mesher.cpp and never references these symbols at link time.
//
// TetGen (https://tetgen.org) is OPTIONAL, EXTERNAL (referenced by absolute path,
// never vendored) and licensed AGPL-3.0. Shipping a closed application that links
// it requires a TetGen commercial license.
//
#ifndef CYBERCAD_NATIVE_MESH_TET_MESHER_H
#define CYBERCAD_NATIVE_MESH_TET_MESHER_H

#include <string>
#include <vector>

namespace cybercad::native::mesh {

enum class MeshOrder { Linear = 1, Quadratic = 2 };

// Options for tetrahedralize_surface. target_element_size in mm (<=0 => no volume
// constraint); radius_edge_ratio is TetGen's quality bound q (clamped >= 1.0);
// grading is reserved for a future mtr sizing field (currently unused beyond q).
struct VolumeMeshOptions {
  MeshOrder order = MeshOrder::Quadratic;
  double target_element_size = 0.0;
  double grading = 0.0;
  double radius_edge_ratio = 1.4;
};

// A flat tetrahedral volume mesh. connectivity holds nodes_per_elem 0-based
// indices per tet into nodes, in CalculiX C3D4/C3D10 ordering, every tet with
// positive signed volume.
struct TetMesh {
  std::vector<double> nodes;         // x,y,z triplets
  std::vector<int> connectivity;     // nodes_per_elem ints per tet
  int nodes_per_elem = 0;            // 4 (C3D4) or 10 (C3D10)
  int order = 0;                     // 4 or 10
  int node_count = 0;
  int element_count = 0;
};

struct TetMeshResult {
  bool ok = false;
  std::string message;
  TetMesh mesh;
};

// Fill a closed TRIANGLE surface (PLC) with tetrahedra using TetGen. verts =
// x,y,z triplets; tris = i,j,k 0-based triplets. On failure returns ok=false with
// a message and an empty mesh (no partial mesh is ever returned).
TetMeshResult tetrahedralize_surface(const std::vector<double>& verts,
                                     const std::vector<int>& tris,
                                     const VolumeMeshOptions& opts);

}  // namespace cybercad::native::mesh

#endif  // CYBERCAD_NATIVE_MESH_TET_MESHER_H
