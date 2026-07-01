// Shape registry lifecycle: add -> get -> release, invalid ids, null shapes,
// handle uniqueness.

#include <memory>

#include "core/shape_registry.h"
#include "harness.h"

using cyber::EngineShape;
using cyber::ShapeRegistry;

namespace {
EngineShape make_shape(int tag) {
    return std::static_pointer_cast<void>(std::make_shared<int>(tag));
}
}  // namespace

CC_TEST(add_returns_nonzero_and_resolves) {
    ShapeRegistry reg;
    EngineShape s = make_shape(7);
    CCShapeId id = reg.add(s);
    CC_CHECK(id != 0);
    EngineShape got = reg.get(id);
    CC_CHECK(got != nullptr);
    CC_CHECK_EQ(*std::static_pointer_cast<int>(got), 7);
    CC_CHECK_EQ(reg.size(), std::size_t{1});
}

CC_TEST(null_shape_yields_invalid_id) {
    ShapeRegistry reg;
    CCShapeId id = reg.add(nullptr);
    CC_CHECK_EQ(id, CCShapeId{0});
    CC_CHECK_EQ(reg.size(), std::size_t{0});
}

CC_TEST(ids_are_unique) {
    ShapeRegistry reg;
    CCShapeId a = reg.add(make_shape(1));
    CCShapeId b = reg.add(make_shape(2));
    CC_CHECK(a != b);
}

CC_TEST(release_frees_and_stops_resolving) {
    ShapeRegistry reg;
    CCShapeId id = reg.add(make_shape(5));
    CC_CHECK(reg.get(id) != nullptr);
    CC_CHECK(reg.release(id));
    CC_CHECK(reg.get(id) == nullptr);
    CC_CHECK_EQ(reg.size(), std::size_t{0});
    // Double release / unknown id is a no-op returning false.
    CC_CHECK(!reg.release(id));
    CC_CHECK(!reg.release(9999));
}

CC_TEST(zero_never_resolves) {
    ShapeRegistry reg;
    CC_CHECK(reg.get(0) == nullptr);
    CC_CHECK(!reg.release(0));
}

CC_TEST(release_drops_last_owner) {
    ShapeRegistry reg;
    auto owner = std::make_shared<int>(42);
    std::weak_ptr<int> weak = owner;
    CCShapeId id = reg.add(std::static_pointer_cast<void>(owner));
    owner.reset();
    CC_CHECK(!weak.expired());  // registry still holds it
    reg.release(id);
    CC_CHECK(weak.expired());  // resources reclaimed
}

CC_RUN_ALL()
