// SPDX-License-Identifier: Apache-2.0
//
// stl_reader.h — native STL importer (issue #5). Reads an ASCII or binary STL
// (AUTO-DETECTED) as a welded triangle-soup mesh. Import-as-mesh ONLY: the result
// is an ntess::Mesh, NOT a reconstructed B-rep (out of scope). The engine wraps the
// returned mesh in a mesh-backed native body that serves display + measurement
// (bbox / area / volume-if-closed) + tessellate directly.
//
// Robustness (issue #5 contract): coincident vertices are welded within a
// tolerance; degenerate / zero-area facets are skipped; non-manifold edges and
// inconsistent winding are tolerated (the soup is stored as-is). On ANY malformed
// input the reader sets `err` and returns std::nullopt with NO partial output — the
// engine registers a body only after a valid mesh is returned, so nothing leaks.
//
// OCCT-FREE. Uses only src/native/{math,tessellate}. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_EXCHANGE_STL_READER_H
#define CYBERCAD_NATIVE_EXCHANGE_STL_READER_H

#include <optional>
#include <string>

#include "native/tessellate/mesh.h"

namespace cybercad::native::exchange {

namespace ntess = cybercad::native::tessellate;

/// Read an ASCII or binary STL (auto-detected) at `path` into a welded ntess::Mesh.
/// `weldTol` is the coincident-vertex merge grid (millimetres). On any failure sets
/// `err` and returns std::nullopt (no partial mesh).
std::optional<ntess::Mesh> stl_read(const std::string& path, std::string& err,
                                    double weldTol = 1e-6);

}  // namespace cybercad::native::exchange

#endif  // CYBERCAD_NATIVE_EXCHANGE_STL_READER_H
