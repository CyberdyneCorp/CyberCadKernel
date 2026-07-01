#ifndef CYBERCADKERNEL_CORE_RESULT_H
#define CYBERCADKERNEL_CORE_RESULT_H

// In-house Result<T, Error> for C++20 (std::expected is C++23 and unavailable
// on the iOS deployment toolchain). Internal code returns Result<T>; the C
// facade collapses it to 0/nil + cc_last_error.

#include <string>
#include <utility>
#include <variant>

namespace cyber {

// A human-readable failure with an optional numeric code.
struct Error {
    int code = 0;
    std::string message;

    Error() = default;
    explicit Error(std::string msg, int c = 0) : code(c), message(std::move(msg)) {}
};

inline Error make_error(std::string message, int code = 0) {
    return Error(std::move(message), code);
}

// Result<T, E>: holds either a T value or an E error. Never both, never neither.
template <class T, class E = Error>
class Result {
public:
    Result(T value) : slot_(std::move(value)) {}       // NOLINT: implicit success
    Result(E error) : slot_(std::move(error)) {}       // NOLINT: implicit failure

    static Result ok(T value) { return Result(std::move(value)); }
    static Result fail(E error) { return Result(std::move(error)); }

    bool has_value() const noexcept { return slot_.index() == 0; }
    explicit operator bool() const noexcept { return has_value(); }

    T& value() & { return std::get<0>(slot_); }
    const T& value() const& { return std::get<0>(slot_); }
    T&& value() && { return std::get<0>(std::move(slot_)); }

    const E& error() const& { return std::get<1>(slot_); }

    T value_or(T fallback) const {
        return has_value() ? std::get<0>(slot_) : std::move(fallback);
    }

private:
    std::variant<T, E> slot_;
};

// Result<void, E>: success carries no value.
template <class E>
class Result<void, E> {
public:
    Result() : error_(), ok_(true) {}
    Result(E error) : error_(std::move(error)), ok_(false) {}  // NOLINT: implicit failure

    static Result ok() { return Result(); }
    static Result fail(E error) { return Result(std::move(error)); }

    bool has_value() const noexcept { return ok_; }
    explicit operator bool() const noexcept { return ok_; }

    const E& error() const& { return error_; }

private:
    E error_;
    bool ok_;
};

}  // namespace cyber

#endif  // CYBERCADKERNEL_CORE_RESULT_H
