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
std::shared_ptr<IEngine> g_active_engine;
}  // namespace

std::shared_ptr<IEngine> active_engine() {
    std::lock_guard<std::mutex> lock(g_engine_mutex);
    if (!g_active_engine) {
        g_active_engine = create_default_engine();
    }
    return g_active_engine;
}

void set_active_engine(std::shared_ptr<IEngine> engine) {
    std::lock_guard<std::mutex> lock(g_engine_mutex);
    g_active_engine = std::move(engine);
}

}  // namespace cyber
