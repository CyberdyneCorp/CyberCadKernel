// SPDX-License-Identifier: Apache-2.0
//
// native_exchange.h — public aggregate header for the native data-exchange
// library (Phase 4, capability #7 `native-data-exchange`, EXPORT slice).
//
// Mirrors native_math.h / native_topology.h / native_construct.h: consumers
// (the engine glue in src/engine/native) include THIS header and call the free
// functions. Everything is OCCT-FREE and builds with clang++ -std=c++20.
//
// SCOPE (honest, narrow — see openspec/NATIVE-REWRITE.md #7):
//   * NATIVE STEP EXPORT — serialise a native B-rep Solid to a valid ISO-10303-21
//     STEP AP203 file in true millimetres (step_writer.h). STEP IMPORT and IGES
//     import/export stay OCCT (parsing arbitrary STEP/IGES is out of scope) — the
//     engine keeps delegating those to the OCCT adapter.
//
#ifndef CYBERCAD_NATIVE_EXCHANGE_H
#define CYBERCAD_NATIVE_EXCHANGE_H

#include "native/exchange/step_writer.h"
#include "native/topology/native_topology.h"

#include <string>

namespace cybercad::native::exchange {

namespace topo = cybercad::native::topology;

/// True iff `solid` is a native B-rep the STEP writer can faithfully serialise
/// (every face + edge geometry kind is in scope). The engine calls this to choose
/// the native path vs the OCCT fallback. Convenience alias for canSerialize.
inline bool step_can_export_native(const topo::Shape& solid) { return canSerialize(solid); }

/// Serialise `solid` to a STEP AP203 file at `path` (true mm). Returns true on
/// success; false (writing nothing) if the shape is out of scope or the file
/// cannot be written — the engine then falls back to OCCT. `productName` names the
/// STEP PRODUCT entity.
inline bool step_export_native(const topo::Shape& solid, const std::string& path,
                               const std::string& productName = "CyberCadKernel_part") {
  return writeStepFile(solid, path, productName);
}

}  // namespace cybercad::native::exchange

#endif  // CYBERCAD_NATIVE_EXCHANGE_H
