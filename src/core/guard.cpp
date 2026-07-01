#include "guard.h"

namespace cyber {

namespace {
// Per-thread storage so a failure on one thread never clobbers another's
// message (the C ABI promises cc_last_error() is thread-local).
thread_local std::string g_last_error;
}  // namespace

void set_last_error(std::string message) noexcept {
    try {
        g_last_error = std::move(message);
    } catch (...) {
        // Never let error-reporting itself throw across the guard.
    }
}

void clear_last_error() noexcept {
    try {
        g_last_error.clear();
    } catch (...) {
    }
}

const char* last_error_cstr() noexcept {
    return g_last_error.c_str();
}

}  // namespace cyber
