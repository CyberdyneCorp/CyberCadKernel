#ifndef CYBERCADKERNEL_CORE_SHAPE_PROVENANCE_H
#define CYBERCADKERNEL_CORE_SHAPE_PROVENANCE_H

// Process-wide, engine-AGNOSTIC provenance for the type-erased EngineShape voids
// held in the ShapeRegistry. It answers ONE question at the engine boundary: "was
// the object behind this void built by the NATIVE engine?" — so the OCCT adapter
// can REFUSE to unwrap a foreign (native) body instead of doing an unchecked
// static_pointer_cast<OcctShape> on it and dereferencing garbage (SIGSEGV).
//
// Why it lives in src/core (not in either engine):
//   * The NATIVE holder (NativeShape) registers its own address here on
//     construction and de-registers on destruction, so identity is a stable,
//     instance-independent fact carried by the shape itself (a body built under an
//     earlier cc_set_engine(1) NativeEngine is still recognised by a later one).
//   * The OCCT adapter QUERIES it from inside src/engine/occt/** — a TU that must
//     NOT include any native-geometry header. This header includes only STL, so
//     both sides can share it with no cross-engine include and no OCCT/native
//     dependency inversion.
//
// This is the symmetric half of the guard NativeEngine already applies (it never
// hands a native void to OCCT); it protects the OTHER direction — an OCCT-active
// engine handed a native body via a foreign CCShapeId.

#include <cstddef>

namespace cyber {

// Record / forget a native-owned holder address. Called by the native shape holder
// ctor/dtor. `p` is the holder's `this` (the same pointer the registry's
// std::shared_ptr<void>::get() returns), so a live lookup is O(1) and exact.
void register_native_shape(const void* p) noexcept;
void unregister_native_shape(const void* p) noexcept;

// True iff `p` is the address of a currently-live native shape holder. A null
// pointer, or an OCCT / stub / unknown holder, returns false. Safe to call from
// any engine TU (no engine type appears in the signature).
bool is_native_shape(const void* p) noexcept;

}  // namespace cyber

#endif  // CYBERCADKERNEL_CORE_SHAPE_PROVENANCE_H
