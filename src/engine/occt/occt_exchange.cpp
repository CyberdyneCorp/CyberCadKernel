// OCCT engine adapter — EXCHANGE capability group.
//
// Defines the OcctEngine data-exchange overrides declared in occt_engine.h:
// STEP / IGES import and export. Like every occt_*.cpp this is an OCCT-only TU
// (it may include OpenCASCADE headers, but no OCCT type escapes into any public
// or shared header) and is NOT host-buildable — it compiles only for iOS where
// the trimmed OCCT static libs are linked (CYBERCAD_HAS_OCCT).
//
// Behaviour is a faithful re-homing of the app bridge's CYBERCAD_HAS_OCCT
// implementations of cc_step_export / cc_step_import / cc_iges_export /
// cc_iges_import: same writer/reader configuration (STEP AsIs; IGES "MM", B-rep
// mode) and the same degenerate-input guards. Imports deliberately register the
// foreign B-rep AS-IS (no BRepCheck_Analyzer::IsValid gate): STEP/IGES from other
// systems routinely carries benign tolerance warnings a strict validity check
// would reject, and IGES may legitimately degrade a solid to a shell.
//
// OCCT signals failures via Standard_Failure (which does NOT derive from
// std::exception); occt::occtGuard rethrows it as std::runtime_error so the
// facade's outer guard records the message into the per-thread cc_last_error.

#include "engine/occt/occt_engine.h"

// ── OCCT exchange headers (this TU only) ──────────────────────────────────────
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <IGESControl_Reader.hxx>
#include <IGESControl_Writer.hxx>

namespace cyber {

// ── STEP ──────────────────────────────────────────────────────────────────────

Result<void> OcctEngine::step_export(EngineShape body, const char* path) {
    return occt::occtGuard([&]() -> Result<void> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr || path == nullptr) {
            return make_error("step_export: null shape or path");
        }
        STEPControl_Writer writer;
        if (writer.Transfer(*shape, STEPControl_AsIs) != IFSelect_RetDone) {
            return make_error("step_export: transfer failed");
        }
        if (writer.Write(path) != IFSelect_RetDone) {
            return make_error("step_export: write failed");
        }
        return {};
    });
}

ShapeResult OcctEngine::step_import(const char* path) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (path == nullptr) {
            return make_error("step_import: null path");
        }
        STEPControl_Reader reader;
        if (reader.ReadFile(path) != IFSelect_RetDone) {
            return make_error("step_import: read failed");
        }
        reader.TransferRoots();
        const TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) {
            return make_error("step_import: empty result");
        }
        // Register foreign B-rep as-is — STEP from other systems often carries
        // benign tolerance warnings strict BRepCheck would reject.
        return occt::wrap(shape);
    });
}

// ── IGES ──────────────────────────────────────────────────────────────────────

Result<void> OcctEngine::iges_export(EngineShape body, const char* path) {
    return occt::occtGuard([&]() -> Result<void> {
        const TopoDS_Shape* shape = occt::unwrap(body);
        if (shape == nullptr || path == nullptr) {
            return make_error("iges_export: null shape or path");
        }
        IGESControl_Writer writer("MM", 1);  // millimetres, B-rep (BRep) mode
        if (!writer.AddShape(*shape)) {
            return make_error("iges_export: add shape failed");
        }
        writer.ComputeModel();
        if (!writer.Write(path)) {
            return make_error("iges_export: write failed");
        }
        return {};
    });
}

ShapeResult OcctEngine::iges_import(const char* path) {
    return occt::occtGuard([&]() -> ShapeResult {
        if (path == nullptr) {
            return make_error("iges_import: null path");
        }
        IGESControl_Reader reader;
        if (reader.ReadFile(path) != IFSelect_RetDone) {
            return make_error("iges_import: read failed");
        }
        reader.TransferRoots();
        const TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) {
            return make_error("iges_import: empty result");
        }
        return occt::wrap(shape);  // IGES may degrade a solid to a shell; keep as-is
    });
}

}  // namespace cyber
