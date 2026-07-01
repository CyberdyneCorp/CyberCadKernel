#ifndef CYBERCADKERNEL_CORE_SHAPE_REGISTRY_H
#define CYBERCADKERNEL_CORE_SHAPE_REGISTRY_H

// Thread-safe registry mapping opaque integer CCShapeId handles to engine
// shapes. The engine shape is type-erased (std::shared_ptr<void>) so no engine
// type crosses into the public surface; the OCCT adapter stores a TopoDS_Shape
// holder behind this void, a native adapter stores its own. Id 0 is the invalid
// sentinel; cc_shape_release frees a handle.

#include <cstddef>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "cybercadkernel/cc_kernel.h"

namespace cyber {

// Opaque engine shape. The concrete type lives entirely inside an engine TU.
using EngineShape = std::shared_ptr<void>;

class ShapeRegistry {
public:
    // Store a shape and return a fresh non-zero handle. A null shape yields 0.
    CCShapeId add(EngineShape shape);

    // Resolve a handle to its shape, or nullptr if the id is unknown/invalid.
    EngineShape get(CCShapeId id) const;

    // Free the handle's resources. Returns true if the id resolved.
    bool release(CCShapeId id);

    // Live handle count (for tests/diagnostics).
    std::size_t size() const;

private:
    mutable std::mutex mutex_;
    std::unordered_map<CCShapeId, EngineShape> shapes_;
    CCShapeId next_id_ = 1;  // 0 is reserved as the invalid sentinel.
};

}  // namespace cyber

#endif  // CYBERCADKERNEL_CORE_SHAPE_REGISTRY_H
