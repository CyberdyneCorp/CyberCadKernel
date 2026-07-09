// No-op engine for the no-OCCT host build. It reports no B-rep engine is linked
// (cc_brep_available() == 0) and lets every geometric cc_* fall through to the
// IEngine defaults, which return an "unsupported" Error the facade collapses to
// 0/nil. This is what makes the host build link and unit-test without OCCT.
//
// The stub is always compiled. It only provides create_default_engine() in the
// no-OCCT build; when CYBERCAD_HAS_OCCT is defined the OCCT adapter provides the
// default (and may still fall back to this stub for capabilities it lacks).

#include <memory>
#include <string>

#include "engine/IEngine.h"

namespace cyber {

class StubEngine final : public IEngine {
public:
    std::string name() const override { return "stub"; }
    bool available() const override { return false; }
    // All geometry methods inherit the IEngine defaults (return engine_unsupported).
};

std::shared_ptr<IEngine> make_stub_engine() {
    return std::make_shared<StubEngine>();
}

// The no-OCCT build's default engine is the stub — EXCEPT under the M8 unlink
// rehearsal (CYBERCAD_M8_REHEARSAL), where native_engine.cpp instead defines
// create_default_engine() to return NativeEngine-over-stub (the post-unlink
// native-default world). Guarded out here to avoid a duplicate definition.
#if !defined(CYBERCAD_HAS_OCCT) && !defined(CYBERCAD_M8_REHEARSAL)
std::shared_ptr<IEngine> create_default_engine() {
    return make_stub_engine();
}
#endif

}  // namespace cyber
