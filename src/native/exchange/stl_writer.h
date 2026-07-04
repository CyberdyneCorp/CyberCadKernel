// SPDX-License-Identifier: Apache-2.0
//
// stl_writer.h — native STL (STereoLithography) exporter for a tessellated
// triangle mesh (issue #4). OCCT-FREE, host-buildable, true millimetres.
//
// The facade tessellates a body through the already-neutral tessellation path and
// hands the flat POD (fp64 vertices + int index triples) to this writer, so the
// mesh is produced ONCE (no duplicated meshing) and the same serializer serves both
// the OCCT and native engines.
//
// Determinism (issue #4 contract): the same mesh + mode produces a BYTE-IDENTICAL
// file — a fixed 80-byte header with NO timestamp/host/build-id, little-endian
// IEEE-754 float32 with -0.0 normalized to +0.0, and facets emitted in mesh order.
//
// OCCT-FREE. Standard library only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_EXCHANGE_STL_WRITER_H
#define CYBERCAD_NATIVE_EXCHANGE_STL_WRITER_H

#include <string>
#include <vector>

namespace cybercad::native::exchange {

/// Serialize a flat triangle mesh (POD) to an STL file at `path`.
///   vertices : x,y,z triplets (fp64, millimetres); length = 3 * vertexCount.
///   triangles: i,j,k index triplets into `vertices`; length = 3 * facetCount.
///   binary   : true  => 80-byte header + uint32 count + 50 bytes/facet (LE).
///              false => ASCII solid/endsolid, locale-independent floats.
/// Per-facet geometric normal = normalize((v1-v0) x (v2-v0)); a zero-area facet
/// emits a (0,0,0) normal but still writes its three vertices (export never fails
/// on a degenerate facet). Facets with an out-of-range index are skipped; the
/// emitted count reflects only the facets actually written.
/// Deterministic: same input => byte-identical output. Returns false only on an
/// I/O failure (path empty or file cannot be written).
bool stl_export_mesh(const std::vector<double>& vertices,
                     const std::vector<int>& triangles,
                     const std::string& path,
                     bool binary);

}  // namespace cybercad::native::exchange

#endif  // CYBERCAD_NATIVE_EXCHANGE_STL_WRITER_H
