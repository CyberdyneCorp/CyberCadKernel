#ifndef CYBERCADKERNEL_ENGINE_NATIVE_NATIVE_HEAL_HOOK_H
#define CYBERCADKERNEL_ENGINE_NATIVE_NATIVE_HEAL_HOOK_H

// native_heal_hook.h — the engine-INTERNAL shape-healing hook (Phase 4 #4
// `native-healing`). NOT a cc_* entry point and NOT part of IEngine: healing is an
// internal capability the engine invokes (e.g. after a future native STEP import,
// or to repair a native boolean/loft residual), exactly like the SSI hook.
//
// The hook implements the standard native-op discipline — native builder →
// mandatory self-verify → OCCT fallback — for healing:
//
//   tryNativeHeal(shape, tol):
//     1. run cybercad::native::heal::healShell(shape, {tol}).
//     2. status == Healed (self-verified watertight + valid) ⇒ KEEP the native
//        result (source = Native).
//     3. status == Unhealed (any reason) ⇒ DEFER: under CYBERCAD_HAS_OCCT, hand the
//        ORIGINAL shape to the OCCT adapter (BRepBuilderAPI_Sewing + ShapeFix_Shell
//        / ShapeFix_Solid) via cyber::occt::sewAndFix; without OCCT, the outcome is
//        reported Unhealed and the caller keeps the (pristine) input. src/native/**
//        never includes an OCCT header — OCCT stays confined to src/engine/occt/.
//
// This header is public-safe: it names only the native heal result + a source tag,
// no OCCT and no heavy geometry. The heal call + the OCCT deferral live in
// native_engine.cpp.

#include "native/heal/heal_result.h"

namespace cyber {

// Where the returned shape came from — the honest coexistence label.
enum class HealSource {
    Native,     // native healShell self-verified watertight + valid
    OcctFixed,  // native deferred; OCCT BRepBuilderAPI_Sewing + ShapeFix repaired it
    Unhealed,   // neither could close it within tolerance (no OCCT, or OCCT also open)
};

struct HealOutcome {
    HealSource source = HealSource::Unhealed;
    cybercad::native::heal::HealResult native;  // the native attempt (metrics + verdict)
};

// Run the native heal on `shape`; on an Unhealed native verdict, fall through to
// the OCCT sewing/ShapeFix oracle when it is linked. Returns which path produced a
// usable result and the native metrics. INTERNAL — no cc_* is added or reached.
HealOutcome tryNativeHeal(const cybercad::native::topology::Shape& shape, double tolerance);

}  // namespace cyber

#endif  // CYBERCADKERNEL_ENGINE_NATIVE_NATIVE_HEAL_HOOK_H
