#include "shape_provenance.h"

#include <mutex>
#include <unordered_set>

namespace cyber {

namespace {

// Meyers singletons: guaranteed initialised on first use, no static-init-order
// dependency (a shape may be built during another TU's static setup). The library
// is intentionally never torn down (see the leaked ShapeRegistry / engine slot), so
// these outlive every holder that could query them.
std::mutex& provenance_mutex() {
    static std::mutex m;
    return m;
}

std::unordered_set<const void*>& native_holders() {
    static std::unordered_set<const void*> s;
    return s;
}

}  // namespace

void register_native_shape(const void* p) noexcept {
    if (p == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(provenance_mutex());
    native_holders().insert(p);
}

void unregister_native_shape(const void* p) noexcept {
    if (p == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(provenance_mutex());
    native_holders().erase(p);
}

bool is_native_shape(const void* p) noexcept {
    if (p == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(provenance_mutex());
    const auto& holders = native_holders();
    return holders.find(p) != holders.end();
}

}  // namespace cyber
