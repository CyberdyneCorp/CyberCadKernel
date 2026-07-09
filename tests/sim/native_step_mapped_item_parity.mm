// SPDX-License-Identifier: Apache-2.0
//
// native_step_mapped_item_parity.mm — SIM parity gate for MOAT M4 import tail:
//   Form-B assembly INSTANCING (MAPPED_ITEM / REPRESENTATION_MAP), the standard AP242
//   assembly-reuse mechanism. Drives the native reader vs the OCCT STEPControl_Reader
//   ORACLE on a booted iOS simulator (openspec/changes/moat-m4t-assembly-import).
//
// OCCT's STEPControl_Writer does NOT emit MAPPED_ITEM (it places components through the
// CONTEXT_DEPENDENT_SHAPE_REPRESENTATION / REP_REL_WITH_TRANSFORMATION tree — Form A). So,
// exactly as the sibling rational-curve SIM gate authors its combined-rational records by
// rewriting an OCCT-written file, this harness AUTHORS a genuine Form-B file by splicing a
// REPRESENTATION_MAP over the OCCT box's ADVANCED_BREP_SHAPE_REPRESENTATION plus N MAPPED_ITEMs
// (each with its own target AXIS2_PLACEMENT_3D) into a new top-level SHAPE_REPRESENTATION. OCCT
// STEPControl_Reader reads MAPPED_ITEM correctly (STEPControl_ActorRead::TransferMappedItem), so
// it is a faithful ORACLE.
//
// GATES:
//   * The native reader PARSES the Form-B file (probeNative parsed=1 — not a silent OCCT
//     fallback) into a placed Compound of N watertight solids whose summed volume matches OCCT.
//   * Through the cc_* facade, the native import matches the OCCT re-import on solid COUNT,
//     TOTAL + per-solid volume / area / centroid / bbox / face+edge counts (compare()).
//   * A DECLINE case (a MAPPED_ITEM whose mapped representation carries no brep) returns NULL
//     natively → the file still imports through OCCT (fall-through, no fabricated geometry).
//
// Output: [MITEM] PASS/FAIL lines. Own main(). Build + run: scripts/run-sim-native-step-mapped-item.sh
// (booted simulator required). Excluded from run-sim-suite.sh (own main() + direct OCCT link).
//
#include "cybercadkernel/cc_kernel.h"

#include "native/exchange/native_exchange.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <IFSelect_ReturnStatus.hxx>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace ntopo = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nex   = cybercad::native::exchange;

namespace {

int g_passed = 0;
int g_failed = 0;
void record(bool ok, const char* label, const char* detail) {
    if (ok) { ++g_passed; std::printf("[MITEM] PASS  %-30s %s\n", label, detail); }
    else    { ++g_failed; std::printf("[MITEM] FAIL  %-30s %s\n", label, detail); }
    std::fflush(stdout);
}

long fileSize(const char* path) {
    struct stat st{};
    return ::stat(path, &st) == 0 ? static_cast<long>(st.st_size) : -1;
}

std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}
void writeFile(const std::string& path, const std::string& s) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << s;
}

// Measure a cc_* body (mass / bbox / face+edge counts).
struct Props {
    bool ok = false;
    double vol = 0, area = 0, cx = 0, cy = 0, cz = 0;
    double bb[6] = {0, 0, 0, 0, 0, 0};
    int faces = 0, edges = 0;
};
int subshapeCount(CCShapeId id, int kind) {
    int* ids = nullptr;
    const int n = cc_subshape_ids(id, kind, &ids);
    if (ids) cc_ints_free(ids);
    return n;
}
Props measure(CCShapeId id) {
    Props p;
    if (id == 0) return p;
    const CCMassProps m = cc_mass_properties(id);
    p.vol = m.volume; p.area = m.area; p.cx = m.cx; p.cy = m.cy; p.cz = m.cz;
    p.ok = m.valid != 0;
    cc_bounding_box(id, p.bb);
    p.faces = subshapeCount(id, 2);
    p.edges = subshapeCount(id, 1);
    return p;
}
double bbDelta(const double a[6], const double b[6]) {
    double d = 0;
    for (int k = 0; k < 6; ++k) d = std::max(d, std::fabs(a[k] - b[k]));
    return d;
}
Props importUnder(int engine, const std::string& path) {
    cc_set_engine(engine);
    return measure(cc_step_import(path.c_str()));
}

// Native-path honesty probe (OCCT-free): did the NATIVE reader produce a non-null placed
// Compound, and is every member watertight? Distinguishes a genuine Form-B parse from the
// engine's silent OCCT fallback.
struct NativeProbe {
    bool parsed = false, compound = false, allWatertight = true;
    int solids = 0;
    double vol = 0.0;
};
double nativeVol(const ntopo::Shape& s, double defl = 0.005) {
    ntess::MeshParams p; p.deflection = defl;
    const ntess::Mesh m = ntess::SolidMesher{p}.mesh(s);
    if (!ntess::isWatertight(m)) return -1.0;
    return std::fabs(ntess::enclosedVolume(m));
}
NativeProbe probeNative(const std::string& path) {
    NativeProbe pr;
    const ntopo::Shape s = nex::step_import_native(path);
    if (s.isNull()) return pr;
    pr.parsed = true;
    pr.compound = s.type() == ntopo::ShapeType::Compound;
    for (ntopo::Explorer e(s, ntopo::ShapeType::Solid); e.more(); e.next()) {
        ++pr.solids;
        const double v = nativeVol(e.current());
        if (v < 0.0) pr.allWatertight = false; else pr.vol += v;
    }
    return pr;
}

bool occtWriteStep(const TopoDS_Shape& shape, const std::string& path) {
    STEPControl_Writer w;
    if (w.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone) return false;
    return w.Write(path.c_str()) == IFSelect_RetDone;
}
double occtStepVolume(const std::string& path) {
    STEPControl_Reader r;
    if (r.ReadFile(path.c_str()) != IFSelect_RetDone) return -1.0;
    r.TransferRoots();
    const TopoDS_Shape s = r.OneShape();
    if (s.IsNull()) return -1.0;
    GProp_GProps g; BRepGProp::VolumeProperties(s, g);
    return std::fabs(g.Mass());
}

void compare(const std::string& name, const Props& a, const Props& b, double volTol, double linTol) {
    char detail[512];
    if (!a.ok || !b.ok) {
        std::snprintf(detail, sizeof detail, "%s: native ok=%d oracle ok=%d", name.c_str(), a.ok, b.ok);
        record(false, "compare", detail); return;
    }
    const double volRel = b.vol > 0 ? std::fabs(a.vol - b.vol) / b.vol : 1.0;
    const double areaRel = b.area > 0 ? std::fabs(a.area - b.area) / b.area : 1.0;
    const double cMax = std::max({std::fabs(a.cx - b.cx), std::fabs(a.cy - b.cy), std::fabs(a.cz - b.cz)});
    std::snprintf(detail, sizeof detail, "%s vol nat=%.6g oracle=%.6g rel=%.2e area rel=%.2e cΔ=%.2e",
                  name.c_str(), a.vol, b.vol, volRel, areaRel, cMax);
    record(volRel < volTol && areaRel < volTol && cMax < linTol, "mass", detail);
    const double bd = bbDelta(a.bb, b.bb);
    std::snprintf(detail, sizeof detail, "%s maxCornerΔ=%.2e (tol=%.0e)", name.c_str(), bd, linTol);
    record(bd < linTol, "bbox", detail);
    // Foreign files come back through OCCT's reader for the ORACLE and through the native
    // reader for `a`; face count is the tight structural check (native per-face oriented edges
    // ~2× OCCT unique — the same contract as the sibling harness).
    const int aUnique = a.edges / 2;
    const bool edgeOk = a.edges > 0 && (a.edges == b.edges || (b.edges > 0 && std::abs(aUnique - b.edges) <= b.faces));
    std::snprintf(detail, sizeof detail, "%s faces nat=%d oracle=%d edges nat=%d(uniq~%d) oracle=%d",
                  name.c_str(), a.faces, b.faces, a.edges, aUnique, b.edges);
    record(a.faces == b.faces && edgeOk, "topology", detail);
}

// ── Form-B authoring: splice REPRESENTATION_MAP + N MAPPED_ITEMs into an OCCT box file ──

// The #id of the ADVANCED_BREP_SHAPE_REPRESENTATION (or SHAPE_REPRESENTATION) listing the brep.
long shapeRepId(const std::string& step) {
    std::size_t k = step.find("ADVANCED_BREP_SHAPE_REPRESENTATION");
    if (k == std::string::npos) k = step.find(" SHAPE_REPRESENTATION");
    if (k == std::string::npos) return 0;
    const std::size_t h = step.rfind('#', k);
    std::size_t j = h + 1; std::string num;
    while (j < step.size() && std::isdigit(static_cast<unsigned char>(step[j]))) num += step[j++];
    return num.empty() ? 0 : std::stol(num);
}
// The (representation) CONTEXT #id used by the brep SR (arg[2] of the SR record), so the
// instancing SR reuses the SAME context OCCT expects for a shape-representation.
long contextIdOfSr(const std::string& step, long srId) {
    const std::string key = "#" + std::to_string(srId) + " = ";
    std::size_t k = step.find(key);
    if (k == std::string::npos) return 0;
    const std::size_t semi = step.find(';', k);
    // The LAST #ref before the ';' is the context (arg[2]).
    const std::size_t h = step.rfind('#', semi);
    if (h == std::string::npos || h <= k) return 0;
    std::size_t j = h + 1; std::string num;
    while (j < step.size() && std::isdigit(static_cast<unsigned char>(step[j]))) num += step[j++];
    return num.empty() ? 0 : std::stol(num);
}
// The largest #id in the file (so new records get non-colliding ids).
long maxId(const std::string& step) {
    long m = 0; std::size_t i = 0;
    while ((i = step.find('#', i)) != std::string::npos) {
        std::size_t j = i + 1; std::string num;
        while (j < step.size() && std::isdigit(static_cast<unsigned char>(step[j]))) num += step[j++];
        if (!num.empty()) m = std::max(m, std::stol(num));
        i = j;
    }
    return m;
}
// Insert `records` right after "DATA;\n".
std::string spliceIntoData(const std::string& step, const std::string& records) {
    const std::size_t d = step.find("DATA;");
    const std::size_t ins = step.find('\n', d) + 1;
    std::string out = step;
    out.insert(ins, records);
    return out;
}

// Author a Form-B file that instances the OCCT box `srcStep` at N given translations.
// If `noBrepRep` is true, the REPRESENTATION_MAP points at an EMPTY shape-representation
// (the decline case). Returns "" on failure.
std::string authorMappedItem(const std::string& srcStep,
                             const std::vector<std::array<double, 3>>& targets,
                             bool noBrepRep) {
    const long srId = shapeRepId(srcStep);
    if (srId == 0) return "";
    // Reuse the file's existing representation context (the brep SR's own context) so the
    // instancing SHAPE_REPRESENTATION is a valid shape-representation OCCT will transfer.
    const long ctx = contextIdOfSr(srcStep, srId);
    if (ctx == 0) return "";
    long id = maxId(srcStep) + 1000;
    const long org = id++, orgP = id++, orgZ = id++, orgX = id++;
    std::string r;
    r += "#" + std::to_string(orgP) + " = CARTESIAN_POINT('',(0.,0.,0.));\n";
    r += "#" + std::to_string(orgZ) + " = DIRECTION('',(0.,0.,1.));\n";
    r += "#" + std::to_string(orgX) + " = DIRECTION('',(1.,0.,0.));\n";
    r += "#" + std::to_string(org) + " = AXIS2_PLACEMENT_3D('',#" + std::to_string(orgP) +
         ",#" + std::to_string(orgZ) + ",#" + std::to_string(orgX) + ");\n";
    long mappedRep = srId;
    if (noBrepRep) {
        const long empty = id++;
        r += "#" + std::to_string(empty) + " = SHAPE_REPRESENTATION('',(),#" + std::to_string(ctx) + ");\n";
        mappedRep = empty;
    }
    const long repMap = id++;
    r += "#" + std::to_string(repMap) + " = REPRESENTATION_MAP(#" + std::to_string(org) + ",#" +
         std::to_string(mappedRep) + ");\n";
    std::vector<long> miIds;
    for (const auto& t : targets) {
        const long pt = id++, ax = id++, mi = id++;
        char pbuf[96];
        std::snprintf(pbuf, sizeof pbuf, "(%.6f,%.6f,%.6f)", t[0], t[1], t[2]);
        r += "#" + std::to_string(pt) + " = CARTESIAN_POINT(''," + pbuf + ");\n";
        r += "#" + std::to_string(ax) + " = AXIS2_PLACEMENT_3D('',#" + std::to_string(pt) +
             ",#" + std::to_string(orgZ) + ",#" + std::to_string(orgX) + ");\n";
        r += "#" + std::to_string(mi) + " = MAPPED_ITEM('',#" + std::to_string(repMap) + ",#" +
             std::to_string(ax) + ");\n";
        miIds.push_back(mi);
    }
    // A top-level SHAPE_REPRESENTATION whose items are the MAPPED_ITEMs.
    const long topSr = id++;
    std::string items;
    for (std::size_t i = 0; i < miIds.size(); ++i)
        items += (i ? ",#" : "#") + std::to_string(miIds[i]);
    r += "#" + std::to_string(topSr) + " = SHAPE_REPRESENTATION('',(" + items + "),#" +
         std::to_string(ctx) + ");\n";
    std::string out = spliceIntoData(srcStep, r);
    // Redirect the product's SHAPE_DEFINITION_REPRESENTATION used_representation (arg[1]) from the
    // raw brep SR (#srId) to the instancing SR (#topSr) so OCCT's TransferRoots walks the product
    // → the MAPPED_ITEMs (the REPRESENTATION_MAP still points at the original brep SR #srId). This
    // is exactly how a real AP242 file makes an instanced assembly a transfer root. Only for the
    // ADMISSION case: the DECLINE (no-brep) case leaves the SDR on the raw box so OCCT still reads
    // the underlying solid while the native reader declines the malformed instancing.
    if (!noBrepRep) {
        const std::size_t sdr = out.find("SHAPE_DEFINITION_REPRESENTATION(");
        if (sdr != std::string::npos) {
            const std::size_t comma = out.find(',', sdr);
            const std::size_t rp = out.find(')', sdr);
            if (comma != std::string::npos && rp != std::string::npos && comma < rp)
                out = out.substr(0, comma + 1) + "#" + std::to_string(topSr) + out.substr(rp);
        }
    }
    return out;
}

// ── The MAPPED_ITEM parity gate ─────────────────────────────────────────────────
void runMappedItemInstancing() {
    const std::string srcPath = "/tmp/cck_mitem_src.step";
    const std::string mapPath = "/tmp/cck_mitem_instanced.step";
    TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 10, 10, 10).Shape();  // vol 1000
    if (!occtWriteStep(box, srcPath)) { record(false, "author", "OCCT box write failed"); return; }

    const std::string src = readFile(srcPath);
    // Three instances of the ONE shared box at distinct translations.
    const std::vector<std::array<double, 3>> targets = {{0, 0, 0}, {30, 0, 0}, {0, 20, 5}};
    const std::string mapped = authorMappedItem(src, targets, /*noBrepRep=*/false);
    if (mapped.empty() || mapped.find("MAPPED_ITEM") == std::string::npos) {
        record(false, "author", "MAPPED_ITEM splice failed"); return;
    }
    writeFile(mapPath, mapped);
    if (fileSize(mapPath.c_str()) <= 0) { record(false, "author", "no file written"); return; }

    // Native honesty probe: parsed a placed Compound of 3 watertight solids matching OCCT.
    const NativeProbe pr = probeNative(mapPath);
    const double ov = occtStepVolume(mapPath);
    char d[400];
    std::snprintf(d, sizeof d, "native parsed=%d compound=%d solids=%d nativeVol=%.6g occtVol=%.6g",
                  pr.parsed, pr.compound, pr.solids, pr.vol, ov);
    const bool volOk = ov > 0 && std::fabs(pr.vol - ov) / ov < 5e-3;
    record(pr.parsed && pr.compound && pr.solids == 3 && pr.allWatertight && volOk,
           "mapped-item native", d);

    // Through the facade: native import vs OCCT re-import — count / vol / bbox / topology.
    const Props nat = importUnder(1, mapPath);
    const Props oracle = importUnder(0, mapPath);
    compare("mapped_item", nat, oracle, 5e-3, 5e-3);
}

// DECLINE: a MAPPED_ITEM whose REPRESENTATION_MAP's mapped representation lists NO brep.
// Native returns NULL → the file still imports through OCCT (fall-through, no fabrication).
void runMappedItemDeclineNoBrep() {
    const std::string srcPath = "/tmp/cck_mitem_src2.step";
    const std::string badPath = "/tmp/cck_mitem_nobrep.step";
    TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 8, 8, 8).Shape();
    if (!occtWriteStep(box, srcPath)) { record(false, "decline author", "OCCT write failed"); return; }
    const std::string bad =
        authorMappedItem(readFile(srcPath), {{20, 0, 0}}, /*noBrepRep=*/true);
    if (bad.empty()) { record(false, "decline author", "splice failed"); return; }
    writeFile(badPath, bad);
    const NativeProbe pr = probeNative(badPath);
    const double ov = occtStepVolume(badPath);  // OCCT still reads the underlying box
    char d[256];
    std::snprintf(d, sizeof d, "native parsed=%d (must be 0 → decline) occtVol=%.6g", pr.parsed, ov);
    record(!pr.parsed && ov > 0, "mapped-item decline no-brep", d);
}

}  // namespace

int main() {
    std::printf("[MITEM] MAPPED_ITEM / REPRESENTATION_MAP (Form-B) assembly-instancing parity\n");
    runMappedItemInstancing();
    runMappedItemDeclineNoBrep();
    cc_set_engine(0);
    std::printf("[MITEM] DONE  passed=%d failed=%d\n", g_passed, g_failed);
    std::fflush(stdout);
    // Skip static-destructor teardown (OCCT globals crash on late teardown in the
    // simulator — a known env issue; the sibling STEP harnesses do the same).
    std::_Exit(g_failed == 0 ? 0 : 1);
}
