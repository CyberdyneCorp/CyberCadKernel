#ifndef CYBERCADKERNEL_CORE_GUARD_H
#define CYBERCADKERNEL_CORE_GUARD_H

// Exception-to-status guard model. Every cc_* entry point runs its body through
// guard(), which catches any std::exception (and, inside OCCT translation units,
// Standard_Failure via occt_guard.h) and converts it to a caller-supplied
// fallback (0/nil) while recording a per-thread human-readable message that
// cc_last_error() returns.

#include <exception>
#include <string>
#include <utility>

namespace cyber {

// Per-thread last-error message. Set on a caught failure, read by cc_last_error.
void set_last_error(std::string message) noexcept;
void clear_last_error() noexcept;
const char* last_error_cstr() noexcept;

// Run fn(); the thread's error slot is cleared on entry, so on success it stays
// empty, while a logical failure the body records (via set_last_error) survives.
// On any std::exception (or unknown throw) the message is recorded and `fallback`
// is returned instead of letting it cross the C ABI.
template <class Fn, class T>
T guard(Fn&& fn, T fallback) {
    clear_last_error();
    try {
        return std::forward<Fn>(fn)();
    } catch (const std::exception& e) {
        set_last_error(e.what());
        return fallback;
    } catch (...) {
        set_last_error("unknown error");
        return fallback;
    }
}

// Void variant: returns true on success, false if the body threw.
template <class Fn>
bool guard_void(Fn&& fn) {
    clear_last_error();
    try {
        std::forward<Fn>(fn)();
        return true;
    } catch (const std::exception& e) {
        set_last_error(e.what());
        return false;
    } catch (...) {
        set_last_error("unknown error");
        return false;
    }
}

}  // namespace cyber

#endif  // CYBERCADKERNEL_CORE_GUARD_H
