#include <mutex>

#include "engine/IEngine.h"

// Active-engine selector (always compiled, engine-agnostic). Lazily creates the
// build's default engine — the stub in the no-OCCT host build, the OCCT adapter
// otherwise — via create_default_engine(), and lets a caller swap engines at
// runtime so an OCCT-backed and a native implementation can coexist and be
// compared behind the same facade call.

namespace cyber {

namespace {
std::mutex g_engine_mutex;
// Intentionally leaked (heap slot, never deleted): destroying the OCCT-backed
// engine — and the OCCT objects it owns — during static destruction races OCCT's
// own static teardown and crashes (SIGSEGV at process exit). The OS reclaims the
// memory at process end.
std::shared_ptr<IEngine>& engine_slot() {
    static std::shared_ptr<IEngine>* slot = new std::shared_ptr<IEngine>();
    return *slot;
}
}  // namespace

std::shared_ptr<IEngine> active_engine() {
    std::lock_guard<std::mutex> lock(g_engine_mutex);
    std::shared_ptr<IEngine>& engine = engine_slot();
    if (!engine) {
        engine = create_default_engine();
    }
    return engine;
}

void set_active_engine(std::shared_ptr<IEngine> engine) {
    std::lock_guard<std::mutex> lock(g_engine_mutex);
    engine_slot() = std::move(engine);
}

}  // namespace cyber
