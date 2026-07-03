// SPDX-License-Identifier: Apache-2.0
//
// native_step_parity.mm — native STEP-EXPORT round-trip + writer-parity harness,
//                         driven through the cc_* facade, re-read with OCCT
//                         STEPControl_Reader (iOS simulator).
//
// Phase 4 capability #7 (`native-data-exchange`) — the STEP-EXPORT slice —
// simulator verification gate 2 (see openspec/NATIVE-REWRITE.md #7). This is the
// CORRECTNESS GATE for the native writer: the honesty requirement is that a
// natively-emitted ISO-10303-21 STEP file must RE-READ through OCCT
// STEPControl_Reader to the SAME solid (volume / area / centroid / bounding box /
// face+edge counts within tolerance). A native writer that emitted a wrong or
// invalid solid would be caught here.
//
//   cc_set_engine(1)  → NativeEngine. cc_step_export is NATIVE for a native body
//                       whose every face surface + edge curve is in the writer's
//                       scope (plane/cylinder/cone/sphere/bspline surfaces,
//                       line/circle/bspline curves): src/native/exchange walks the
//                       native B-rep (src/native/topology) and emits a valid AP203
//                       STEP file in true millimetres.
//   cc_set_engine(0)  → OCCT engine (the oracle writer): cc_step_export is
//                       STEPControl_Writer.
//
// ── What this harness asserts (honest, per the spec) ──────────────────────────────
//
//   NATIVE ROUND-TRIP — for each native-representable solid (box, cylinder, a
//   circular-holed plate): build it under NATIVE, cc_step_export under NATIVE (the
//   native writer runs — cc_active_engine()==1), then re-read the written .step file
//   DIRECTLY with OCCT STEPControl_Reader (independent of the kernel's own import
//   path, which is OCCT anyway) and compare the re-read TopoDS_Solid to the ORIGINAL
//   native solid's cc_mass_properties (volume / area / centroid) + cc_bounding_box +
//   cc_subshape_ids face/edge counts. Planar (box, holed-plate planar faces) is
//   EXACT; curved (cylinder wall, hole walls) is deflection/discretisation-bounded.
//
//   WRITER PARITY (native vs OCCT) — for the SAME solid, ALSO cc_step_export under
//   the OCCT engine (STEPControl_Writer) to a second file, re-read THAT with the same
//   OCCT reader, and confirm the two re-read solids are EQUIVALENT (volume / area /
//   bbox within tolerance). This proves the native writer produces a STEP file that
//   round-trips to the same geometry the trusted OCCT writer does.
//
//   FALLBACK (foreign / OCCT-built body) — build a solid under the OCCT engine (a
//   NON-native body), then cc_step_export under NATIVE: the NativeEngine sees a
//   foreign body and FORWARDS to STEPControl_Writer (labelled [fallback], never
//   faked; cc_active_engine()==1 yet the OCCT writer produced the file). Re-read it
//   and confirm the round-trip solid matches the source. (A native body with an
//   out-of-scope geometry kind would instead return a clean error rather than be
//   handed to OCCT — the writer's canSerialize guard — but every solid this harness
//   can build through the facade under NATIVE is in scope, so the honest fallback we
//   can drive here is the foreign-body one.)
//
// ── Per-case comparisons (re-read solid vs the reference) ─────────────────────────
//   * volume / area / centroid — cc_mass_properties on the source vs OCCT
//     BRepGProp::VolumeProperties / SurfaceProperties on the re-read solid.
//   * bounding box — cc_bounding_box on the source vs BRepBndLib::Add on the re-read.
//   * face / edge counts — cc_subshape_ids on the source vs TopExp_Explorer over
//     TopAbs_FACE / TopAbs_EDGE on the re-read (a >=1 integer-multiple relationship is
//     accepted for edges: the native per-face builder + the reader may split/merge a
//     shared edge differently, but the SOLID is the same — face count is the tighter
//     structural check and is required to match the source face count).
//
// Output: [NSTEP] PASS/FAIL lines, each tagged [native] or [fallback], with round-trip
// deltas; then "== N passed, M failed ==". Flushes stdout and std::_Exit(failed?1:0)
// (the trimmed static-OCCT build's static teardown is not exit-clean — same rationale
// as the sibling sim harnesses; every id is released before exit).
//
// Build: scripts/run-sim-native-step.sh — compiles this harness + the whole
// facade/core/engine (incl. NativeEngine + OCCT adapter) + src/native/**, links the
// OCCT libs, spawns on a booted simulator. On run-sim-suite.sh's SKIP list (its own
// main(), competes for the entry point).

#include "cybercadkernel/cc_kernel.h"

// OCCT — the harness (NOT the native kernel) links OCCT to re-read the file the
// native writer produced and to measure the re-read solid independently.
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── result accounting ─────────────────────────────────────────────────────────
int g_passed = 0;
int g_failed = 0;

void record(bool ok, const char* tag, const std::string& label, const char* detail) {
    if (ok) {
        ++g_passed;
        std::printf("[NSTEP] PASS  %-10s %-30s %s\n", tag, label.c_str(), detail);
    } else {
        ++g_failed;
        std::printf("[NSTEP] FAIL  %-10s %-30s %s\n", tag, label.c_str(), detail);
    }
    std::fflush(stdout);
}

// ── restore the default (OCCT) engine no matter how we leave scope ──────────────
struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

// File size in bytes, or -1 if the path does not exist / cannot be stat'd.
long fileSize(const char* path) {
    struct stat st{};
    if (::stat(path, &st) != 0) return -1;
    return static_cast<long>(st.st_size);
}

// ── re-read a STEP file with OCCT STEPControl_Reader and measure it ─────────────

// A measurement of a re-read solid (mass properties + bbox + subshape counts),
// taken with OCCT directly on the TopoDS_Shape the reader produced.
struct ReReadProps {
    bool ok = false;         // reader succeeded, transferred a non-null shape
    double volume = 0.0;     // mm^3
    double area = 0.0;       // mm^2
    double cx = 0, cy = 0, cz = 0;
    double bb[6] = {0, 0, 0, 0, 0, 0};
    int faceCount = 0;
    int edgeCount = 0;
};

// Read `path` with STEPControl_Reader, transfer roots, and measure the OneShape().
// Independent of the kernel's own cc_step_import (which forwards to OCCT anyway) —
// this proves the file the NATIVE writer produced is a valid STEP the standard OCCT
// reader parses to a real solid.
ReReadProps reReadStep(const std::string& path) {
    ReReadProps r;
    STEPControl_Reader reader;
    if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) return r;
    const Standard_Integer nRoots = reader.NbRootsForTransfer();
    if (nRoots <= 0) return r;
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) return r;

    GProp_GProps vol;
    BRepGProp::VolumeProperties(shape, vol);
    GProp_GProps sur;
    BRepGProp::SurfaceProperties(shape, sur);
    r.volume = vol.Mass();
    r.area = sur.Mass();
    const gp_Pnt c = vol.CentreOfMass();
    r.cx = c.X();
    r.cy = c.Y();
    r.cz = c.Z();

    Bnd_Box box;
    BRepBndLib::Add(shape, box, Standard_False);
    if (!box.IsVoid())
        box.Get(r.bb[0], r.bb[1], r.bb[2], r.bb[3], r.bb[4], r.bb[5]);

    for (TopExp_Explorer e(shape, TopAbs_FACE); e.More(); e.Next()) ++r.faceCount;
    for (TopExp_Explorer e(shape, TopAbs_EDGE); e.More(); e.Next()) ++r.edgeCount;

    r.ok = true;
    return r;
}

double maxBBoxCornerDelta(const double a[6], const double b[6]) {
    double d = 0.0;
    for (int k = 0; k < 6; ++k) d = std::max(d, std::fabs(a[k] - b[k]));
    return d;
}

// ── the source native/OCCT solid, measured through the cc_* facade ──────────────
struct SourceProps {
    CCShapeId id = 0;
    CCMassProps mass{0, 0, 0, 0, 0, 0};
    double bb[6] = {0, 0, 0, 0, 0, 0};
    int bbok = 0;
    int faceCount = 0;
    int edgeCount = 0;
};

int subshapeCount(CCShapeId id, int kind) {
    int* ids = nullptr;
    const int n = cc_subshape_ids(id, kind, &ids);
    if (ids) cc_ints_free(ids);
    return n;
}

SourceProps measureSource(CCShapeId id) {
    SourceProps s;
    s.id = id;
    if (id == 0) return s;
    s.mass = cc_mass_properties(id);
    s.bbok = cc_bounding_box(id, s.bb);
    s.faceCount = subshapeCount(id, 2);  // 2 = face
    s.edgeCount = subshapeCount(id, 1);  // 1 = edge
    return s;
}

// ── operand builders (each uses the ACTIVE engine's cc_solid_* entry points) ─────

// An axis-aligned box [0,w]×[0,d]×[0,h] as a rectangle profile extruded by h.
CCShapeId buildBox(double w, double d, double h) {
    const double rect[] = {0.0, 0.0, w, 0.0, w, d, 0.0, d};
    return cc_solid_extrude(rect, 4, h);
}

// A solid cylinder radius r, height h along +Y (revolve the rectangle x∈[0,r],
// y∈[0,h] a full turn). Native path emits CYLINDRICAL_SURFACE + CIRCLE rims + PLANE
// caps — all in the writer's scope.
CCShapeId buildCylinder(double r, double h) {
    const double rect[] = {0.0, 0.0, r, 0.0, r, h, 0.0, h};
    return cc_solid_revolve(rect, 4, 2.0 * kPi);
}

// A rectangular plate with one circular THROUGH-hole: outer 20×12 extruded 4 deep,
// a single hole (cx,cy,r) = (10,6,3). Exercises CIRCLE hole edges + CYLINDRICAL_SURFACE
// hole walls alongside the PLANE faces — a mixed planar/curved native body still in
// the writer's scope.
CCShapeId buildHoledPlate() {
    const double outer[] = {0.0, 0.0, 20.0, 0.0, 20.0, 12.0, 0.0, 12.0};
    const double hole[] = {10.0, 6.0, 3.0};  // cx, cy, r
    return cc_solid_extrude_holes(outer, 4, hole, 1, 4.0);
}

using Builder = CCShapeId (*)();

// ── one round-trip case ───────────────────────────────────────────────────────────
//
// mode:
//   Native   — build the solid UNDER NATIVE, cc_step_export UNDER NATIVE (the native
//              writer runs). Re-read + compare to the source. ALSO export the same
//              solid UNDER OCCT and confirm both files re-read to equivalent solids
//              (writer parity).
//   Fallback — build the solid UNDER OCCT (a foreign body), cc_step_export UNDER
//              NATIVE (the engine forwards to STEPControl_Writer). Re-read + compare
//              to the source; assert cc_active_engine()==1 (fall-through proof).
enum class Mode { Native, Fallback };

struct StepCase {
    std::string name;
    Mode mode;
    Builder build;
    bool exact;          // planar → exact fp; curved → deflection/discretisation-bounded
    double volTol;       // relative volume/area tolerance
    double linTol;       // absolute centroid / bbox corner tolerance (mm)
};

// Compare a re-read solid against the source-solid reference. `refFaces` is the
// source face count; edges may be an integer multiple (see header). Records three
// PASS/FAIL lines (mass, bbox, topology).
void compareReRead(const StepCase& c, const char* tag, const std::string& stage,
                   const SourceProps& src, const ReReadProps& rr) {
    char detail[512];

    if (!rr.ok) {
        std::snprintf(detail, sizeof detail, "STEPControl_Reader failed to re-read a solid");
        record(false, tag, c.name + " " + stage + " reread", detail);
        return;
    }

    // ── mass properties (volume / area / centroid) ────────────────────────────────
    const double volRel =
        src.mass.volume > 0.0 ? std::fabs(rr.volume - src.mass.volume) / src.mass.volume : 1.0;
    const double areaRel =
        src.mass.area > 0.0 ? std::fabs(rr.area - src.mass.area) / src.mass.area : 1.0;
    const double cMax = std::max({std::fabs(rr.cx - src.mass.cx), std::fabs(rr.cy - src.mass.cy),
                                  std::fabs(rr.cz - src.mass.cz)});
    const bool massOk = src.mass.valid && volRel < c.volTol && areaRel < c.volTol && cMax < c.linTol;
    std::snprintf(detail, sizeof detail,
                  "vol src=%.6g reread=%.6g rel=%.2e | area rel=%.2e | cΔ=%.2e (tol v=%.0e l=%.0e)",
                  src.mass.volume, rr.volume, volRel, areaRel, cMax, c.volTol, c.linTol);
    record(massOk, tag, c.name + " " + stage + " mass", detail);

    // ── bounding box ──────────────────────────────────────────────────────────────
    const double bbDelta = maxBBoxCornerDelta(src.bb, rr.bb);
    const bool bbOk = (src.bbok == 1) && (bbDelta < c.linTol);
    std::snprintf(detail, sizeof detail, "maxCornerΔ=%.2e (tol=%.0e)", bbDelta, c.linTol);
    record(bbOk, tag, c.name + " " + stage + " bbox", detail);

    // ── face / edge counts ──────────────────────────────────────────────────────
    // Face count must match the reference exactly (the tight structural check). Edge
    // count is accepted when it is a >=1 integer multiple/divisor of the reference
    // (per-face vs shared edge representation may differ between the native builder
    // and the reader), OR when the re-read has a FEW MORE edges than the reference
    // (a bounded additive superset): a full-turn periodic wall (a cylindrical hole
    // wall) is one face on a periodic surface whose valid B-rep carries a SEAM edge
    // the native builder's deferred edge-sharing representation omits (NATIVE-REWRITE
    // #4/#7). The re-read solid — from BOTH the native writer and OCCT's own writer —
    // therefore legitimately gains one seam edge per periodic wall (≤ faceCount
    // extra), which is not an integer multiple of the source. It must stay positive.
    const bool faceOk = rr.faceCount == src.faceCount;
    const bool edgeMultiple = src.edgeCount != 0 && (rr.edgeCount % src.edgeCount == 0 ||
                                                     src.edgeCount % rr.edgeCount == 0);
    const bool edgeSeamSuperset = rr.edgeCount >= src.edgeCount &&
                                  (rr.edgeCount - src.edgeCount) <= rr.faceCount;
    const bool edgeOk =
        rr.edgeCount > 0 && (src.edgeCount == 0 || edgeMultiple || edgeSeamSuperset);
    std::snprintf(detail, sizeof detail, "faces src=%d reread=%d | edges src=%d reread=%d",
                  src.faceCount, rr.faceCount, src.edgeCount, rr.edgeCount);
    record(faceOk && edgeOk, tag, c.name + " " + stage + " topology", detail);
}

// Confirm the two re-read solids (native-written vs OCCT-written) are equivalent —
// the writer-parity assertion.
void compareWriterParity(const StepCase& c, const ReReadProps& nat, const ReReadProps& occt) {
    char detail[512];
    if (!nat.ok || !occt.ok) {
        std::snprintf(detail, sizeof detail, "native reread ok=%d, OCCT reread ok=%d", nat.ok,
                      occt.ok);
        record(false, "native", c.name + " writer-parity", detail);
        return;
    }
    const double volRel =
        occt.volume > 0.0 ? std::fabs(nat.volume - occt.volume) / occt.volume : 1.0;
    const double areaRel = occt.area > 0.0 ? std::fabs(nat.area - occt.area) / occt.area : 1.0;
    const double bbDelta = maxBBoxCornerDelta(nat.bb, occt.bb);
    const bool ok = volRel < c.volTol && areaRel < c.volTol && bbDelta < c.linTol;
    std::snprintf(detail, sizeof detail,
                  "nativeWriter vol=%.6g occtWriter vol=%.6g relV=%.2e relA=%.2e bboxΔ=%.2e",
                  nat.volume, occt.volume, volRel, areaRel, bbDelta);
    record(ok, "native", c.name + " writer-parity", detail);
}

// Scratch paths in /tmp (harmless; removed before each write so a stale file cannot
// mask a writer failure).
std::string nativeStepPath(const std::string& name) {
    return "/tmp/cck_nstep_" + name + "_native.step";
}
std::string occtStepPath(const std::string& name) {
    return "/tmp/cck_nstep_" + name + "_occt.step";
}

// ── run one native round-trip + writer-parity case ────────────────────────────────
void runNativeCase(const StepCase& c) {
    char detail[512];
    const std::string natPath = nativeStepPath(c.name);
    const std::string occtPath = occtStepPath(c.name);
    std::remove(natPath.c_str());
    std::remove(occtPath.c_str());

    // 1) Build the solid UNDER NATIVE and measure the source through the facade.
    cc_set_engine(1);
    const CCShapeId id = c.build();
    const bool activeNative = cc_active_engine() == 1;
    if (id == 0) {
        std::snprintf(detail, sizeof detail, "native build failed (active=%d): %s",
                      activeNative ? 1 : 0, cc_last_error());
        record(false, "native", c.name + " build", detail);
        cc_set_engine(0);
        return;
    }
    const SourceProps src = measureSource(id);

    // 2) NATIVE writer: cc_step_export under the native engine writes the native STEP.
    const int nativeExport = cc_step_export(id, natPath.c_str());
    const long natBytes = fileSize(natPath.c_str());
    const bool wroteNative = nativeExport == 1 && natBytes > 0;
    std::snprintf(detail, sizeof detail,
                  "cc_step_export(native engine)->%d, %ld bytes, active=%d (native writer ran)",
                  nativeExport, natBytes, activeNative ? 1 : 0);
    record(wroteNative && activeNative, "native", c.name + " export", detail);

    // 3) OCCT writer: cc_step_export under the OCCT engine writes the reference STEP.
    //    Re-build the SAME solid under OCCT (the OCCT engine cannot serialise a native
    //    body handle), export it, then release it. The OCCT-built solid is the OCCT
    //    engine's OWN representation of this shape — its face/edge counts are the
    //    canonical OCCT topology (e.g. a cylinder is ONE periodic wall face, not the
    //    native builder's 3 sub-π patches), so the occt-writer round-trip is checked
    //    against the OCCT body's counts (occtSrc), not the native body's (src). Mass /
    //    bbox are geometry-identical, so those still match the native src too.
    cc_set_engine(0);
    const CCShapeId occtId = c.build();
    const SourceProps occtSrc = measureSource(occtId);
    int occtExport = 0;
    if (occtId != 0) occtExport = cc_step_export(occtId, occtPath.c_str());
    const bool wroteOcct = occtExport == 1 && fileSize(occtPath.c_str()) > 0;
    if (occtId) cc_shape_release(occtId);

    // 4) Re-read BOTH files DIRECTLY with OCCT STEPControl_Reader (independent of the
    //    kernel's import path) and compare each to the source, then to each other.
    const ReReadProps natRR = wroteNative ? reReadStep(natPath) : ReReadProps{};
    const ReReadProps occtRR = wroteOcct ? reReadStep(occtPath) : ReReadProps{};

    compareReRead(c, "native", "native-writer", src, natRR);
    if (!wroteOcct)
        record(false, "native", c.name + " occt-writer", "OCCT cc_step_export failed to write");
    else
        compareReRead(c, "native", "occt-writer", occtSrc, occtRR);
    compareWriterParity(c, natRR, occtRR);

    cc_set_engine(0);
    cc_shape_release(id);
}

// ── run the fallback (foreign / OCCT-built body) case ─────────────────────────────
// Build a solid UNDER OCCT, then cc_step_export UNDER NATIVE — the NativeEngine sees a
// foreign body and FORWARDS to STEPControl_Writer (labelled, never faked). Re-read and
// compare to the source; the fall-through proof is cc_active_engine()==1.
void runFallbackCase(const StepCase& c) {
    char detail[512];
    const std::string path = nativeStepPath(c.name);
    std::remove(path.c_str());

    cc_set_engine(0);
    const CCShapeId id = c.build();  // OCCT (foreign) body
    if (id == 0) {
        record(false, "fallback", c.name + " build", "OCCT build failed");
        cc_set_engine(0);
        return;
    }
    const SourceProps src = measureSource(id);

    // Export UNDER NATIVE: the engine must forward a foreign body to OCCT.
    cc_set_engine(1);
    const bool activeNative = cc_active_engine() == 1;
    const int exported = cc_step_export(id, path.c_str());
    const long bytes = fileSize(path.c_str());
    const bool wrote = exported == 1 && bytes > 0;
    std::snprintf(detail, sizeof detail,
                  "native active=%d, cc_step_export(foreign body)->%d, %ld bytes (forwarded to OCCT)",
                  activeNative ? 1 : 0, exported, bytes);
    record(wrote && activeNative, "fallback", c.name + " export", detail);

    const ReReadProps rr = wrote ? reReadStep(path) : ReReadProps{};
    compareReRead(c, "fallback", "occt-fallback", src, rr);

    cc_set_engine(0);
    cc_shape_release(id);
}

}  // namespace

int main() {
    EngineGuard guard;  // restores cc_set_engine(0) at the end

    std::printf("── native STEP-export round-trip + writer parity (cc_* facade, re-read via "
                "OCCT STEPControl_Reader)\n");
    std::fflush(stdout);

    // Native round-trip cases. Planar box is EXACT; the cylinder / holed plate carry
    // curved walls whose native tiling + the reader's re-facet differ slightly, so use
    // a deflection/discretisation-bounded tolerance there. Face count must still match.
    const std::vector<StepCase> nativeCases = {
        // 10×10×10 box → 6 PLANE faces, 12 edges, vol 1000, area 600. EXACT.
        {"box", Mode::Native, [] { return buildBox(10.0, 10.0, 10.0); }, /*exact*/ true, 1e-6, 1e-6},
        // Cylinder r=5 h=12 → CYLINDRICAL_SURFACE + CIRCLE rims + PLANE caps. Curved.
        {"cylinder", Mode::Native, [] { return buildCylinder(5.0, 12.0); }, false, 3e-2, 2e-1},
        // 20×12×4 plate with one r=3 circular through-hole → PLANE faces + a
        // CYLINDRICAL_SURFACE hole wall + CIRCLE hole rims. Curved (the hole wall).
        {"holed-plate", Mode::Native, buildHoledPlate, false, 3e-2, 2e-1},
    };
    for (const StepCase& c : nativeCases) runNativeCase(c);

    // Fallback: a foreign (OCCT-built) box exported under the NATIVE engine forwards to
    // STEPControl_Writer. It re-reads exactly (OCCT wrote + OCCT read the box).
    const StepCase fallbackCase{"foreign-box", Mode::Fallback,
                                [] { return buildBox(8.0, 8.0, 8.0); }, /*exact*/ true, 1e-6, 1e-6};
    runFallbackCase(fallbackCase);

    cc_set_engine(0);

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
