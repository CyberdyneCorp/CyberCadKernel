// SPDX-License-Identifier: Apache-2.0
//
// native_mesh.h — public aggregate header for the native volume-meshing library
// (Phase 4, capability `native-meshing`). The facade includes THIS header only.
//
// Two pieces, one always-on and one optional:
//   * quality.h  — native tetrahedral mesh quality metrics. OCCT-free, TetGen-free,
//                  ALWAYS compiled in the default MIT build.
//   * tet_mesher.h — declaration of the TetGen-backed volume mesher. The header
//                  itself is TetGen-header-free (std::vector signatures only), so
//                  it is safe to include unconditionally; its ONLY implementation
//                  (tet_mesher.cpp) links the external AGPL TetGen library and is
//                  compiled solely under CYBERCAD_HAS_TETGEN. The facade #ifdef's
//                  the call sites, so with the flag OFF these symbols are never
//                  referenced at link time.
//
#ifndef CYBERCAD_NATIVE_MESH_H
#define CYBERCAD_NATIVE_MESH_H

#include "native/mesh/quality.h"
#include "native/mesh/tet_mesher.h"

#endif  // CYBERCAD_NATIVE_MESH_H
