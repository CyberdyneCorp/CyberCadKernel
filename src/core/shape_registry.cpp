#include "shape_registry.h"

namespace cyber {

CCShapeId ShapeRegistry::add(EngineShape shape) {
    if (!shape) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    CCShapeId id = next_id_++;
    shapes_.emplace(id, std::move(shape));
    return id;
}

EngineShape ShapeRegistry::get(CCShapeId id) const {
    if (id == 0) {
        return nullptr;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = shapes_.find(id);
    return it == shapes_.end() ? nullptr : it->second;
}

bool ShapeRegistry::release(CCShapeId id) {
    if (id == 0) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return shapes_.erase(id) > 0;
}

std::size_t ShapeRegistry::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shapes_.size();
}

}  // namespace cyber
