// SPDX-License-Identifier: Apache-2.0
//
// native_transform_parity.mm — MOAT M-TX native-vs-OCCT AFFINE-TRANSFORM parity
//                              harness (iOS simulator). SIM GATE (b) of the two-gate
//                              discipline; gate (a) is the OCCT-free host suite
//                              tests/test_native_engine.cpp (analytic invariants).
//
// Unlike the geometry-engine fuzzer tests/sim/native_transform_fuzz.mm (which drives
// topology::Shape::located(math::Transform) + SolidMesher directly), THIS harness
// exercises the SHIPPING PATH: the public cc_* transform facade under BOTH engines —
//   cc_set_engine(0) → OCCT engine (oracle: BRepBuilderAPI_Transform(gp_Trsf) + BRepGProp)
//   cc_set_engine(1) → NativeEngine (M-TX: Shape::located(math::Transform) + SolidMesher)
// — and compares the two. It proves the ABI-level wiring landed in NativeEngine's
// translate/rotate/mirror/scale/scale_about/place_on_frame is CONSISTENT with the
// OCCT oracle AND with the closed-form analytic image (vol' = |det L|·vol), for the
// SAME base solids built on each side.
//
// Base solids: a 10×10×10 BOX (planar — meshes exactly, tight tolerance) and a
// cylinder R=2 h=5 (curved — deflection-bounded tolerance). Each is transformed by
// every op; native vs OCCT volume / area / centroid are compared, plus the box bbox
// (exact) and the analytic |det L|·vol anchor. A zero/degenerate scale is asserted an
// HONEST DECLINE on BOTH engines (cc returns 0).
//
// Links the WHOLE kernel (facade + core + engine[native+occt] + src/native) + the
// full OCCT toolkit set, exactly like native_boolean_parity.mm. On run-sim-suite.sh's
// SKIP list (own main()); std::_Exit to bypass the trimmed static-OCCT teardown.

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
int g_passed = 0, g_failed = 0;

struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

bool check(bool ok, const std::string& what, const std::string& detail = "") {
    if (ok) {
        ++g_passed;
    } else {
        ++g_failed;
        std::printf("  [FAIL] %s%s%s\n", what.c_str(), detail.empty() ? "" : " :: ",
                    detail.c_str());
    }
    std::fflush(stdout);
    return ok;
}

double relDiff(double a, double b) {
    return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : std::fabs(a - b);
}

// A 10×10×10 box (planar, exact) and an R=2 h=5 cylinder (curved) built with the
// ACTIVE engine's cc_solid_extrude / cc_solid_revolve.
constexpr double kBox[8] = {0, 0, 10, 0, 10, 10, 0, 10};
constexpr double kCyl[8] = {0, 0, 2, 0, 2, 5, 0, 5};  // revolved full-turn about +Y
CCShapeId buildBox() { return cc_solid_extrude(kBox, 4, 10.0); }
CCShapeId buildCyl() { return cc_solid_revolve(kCyl, 4, 2.0 * kPi); }

struct Measured {
    CCShapeId id = 0;
    bool present = false, valid = false;
    double vol = 0, area = 0, c[3] = {0, 0, 0}, bb[6] = {0};
    bool bbok = false;
};

Measured measure(CCShapeId id) {
    Measured m;
    m.id = id;
    m.present = id != 0;
    if (!m.present) return m;
    const CCMassProps mp = cc_mass_properties(id);
    m.valid = mp.valid != 0;
    m.vol = mp.volume;
    m.area = mp.area;
    m.c[0] = mp.cx;
    m.c[1] = mp.cy;
    m.c[2] = mp.cz;
    m.bbok = cc_bounding_box(id, m.bb) == 1;
    return m;
}

// One transform op expressed as a cc_* call on a body id (uses the active engine).
struct Op {
    std::string name;
    std::function<CCShapeId(CCShapeId)> apply;
    double absDet;  // |det L| — the analytic volume multiplier
};

struct Base {
    std::string name;
    std::function<CCShapeId()> build;
    double baseVol;
    bool planar;  // planar → tight tolerance; curved → deflection-bounded
};

void runCase(const Base& base, const Op& op) {
    const std::string tag = base.name + " / " + op.name;

    // NATIVE: build + transform under engine 1.
    cc_set_engine(1);
    CCShapeId nb = base.build();
    CCShapeId nt = nb ? op.apply(nb) : 0;
    const Measured nat = measure(nt);

    // OCCT: build + transform under engine 0 (the oracle).
    cc_set_engine(0);
    CCShapeId ob = base.build();
    CCShapeId ot = ob ? op.apply(ob) : 0;
    const Measured occ = measure(ot);

    check(nat.present && nat.valid, tag + ": native produced a valid transformed body",
          nat.present ? "not valid" : "null id");
    check(occ.present && occ.valid, tag + ": OCCT oracle produced a valid transformed body");

    if (nat.present && nat.valid && occ.present && occ.valid) {
        const double volTol = base.planar ? 1e-3 : 3e-2;
        const double areaTol = base.planar ? 1e-3 : 3e-2;
        const double centTol = base.planar ? 1e-4 : 5e-2;
        const double expectVol = base.baseVol * op.absDet;

        check(relDiff(nat.vol, occ.vol) < volTol, tag + ": volume native≈OCCT",
              "natV=" + std::to_string(nat.vol) + " occV=" + std::to_string(occ.vol));
        check(relDiff(nat.vol, expectVol) < volTol, tag + ": volume == |detL|·baseVol",
              "natV=" + std::to_string(nat.vol) + " expect=" + std::to_string(expectVol));
        check(relDiff(nat.area, occ.area) < areaTol, tag + ": area native≈OCCT",
              "natA=" + std::to_string(nat.area) + " occA=" + std::to_string(occ.area));
        const double cerr = std::sqrt((nat.c[0] - occ.c[0]) * (nat.c[0] - occ.c[0]) +
                                      (nat.c[1] - occ.c[1]) * (nat.c[1] - occ.c[1]) +
                                      (nat.c[2] - occ.c[2]) * (nat.c[2] - occ.c[2]));
        check(cerr < centTol, tag + ": centroid native≈OCCT",
              "cerr=" + std::to_string(cerr));

        // bbox parity — exact for the planar box; deflection-loose for the curved cyl.
        if (nat.bbok && occ.bbok) {
            const double bbTol = base.planar ? 1e-4 : 6e-2;
            double maxd = 0;
            for (int k = 0; k < 6; ++k) maxd = std::max(maxd, std::fabs(nat.bb[k] - occ.bb[k]));
            check(maxd < bbTol, tag + ": bbox native≈OCCT", "maxΔ=" + std::to_string(maxd));
        }
    }

    cc_set_engine(1);
    if (nt) cc_shape_release(nt);
    if (nb) cc_shape_release(nb);
    cc_set_engine(0);
    if (ot) cc_shape_release(ot);
    if (ob) cc_shape_release(ob);
}

// A zero/degenerate scale must HONESTLY DECLINE on BOTH engines (cc returns 0), never
// a faked/collapsed body — the native side must not forward its void to OCCT either.
void runDegenerateScaleDecline() {
    cc_set_engine(1);
    CCShapeId nb = buildBox();
    check(nb != 0, "degenerate-scale: native box built");
    check(cc_scale_shape(nb, 0.0) == 0, "degenerate-scale: native cc_scale_shape(0)==0 (decline)");
    check(cc_scale_shape_about(nb, 1, 1, 1, 0.0) == 0,
          "degenerate-scale: native cc_scale_shape_about(0)==0 (decline)");
    if (nb) cc_shape_release(nb);

    cc_set_engine(0);
    CCShapeId ob = buildBox();
    check(cc_scale_shape(ob, 0.0) == 0, "degenerate-scale: OCCT cc_scale_shape(0)==0 (decline)");
    if (ob) cc_shape_release(ob);
}

}  // namespace

int main() {
    EngineGuard guard;
    std::printf("== M-TX native-vs-OCCT AFFINE-TRANSFORM parity (cc_* facade, both engines) ==\n");
    std::fflush(stdout);

    const double kHalfPi = 0.5 * kPi;
    const std::vector<Op> ops = {
        {"translate(5,5,5)", [](CCShapeId s) { return cc_translate_shape(s, 5, 5, 5); }, 1.0},
        {"rotate(90°,Z@0)",
         [=](CCShapeId s) { return cc_rotate_shape_about(s, 0, 0, 0, 0, 0, 1, kHalfPi); }, 1.0},
        {"mirror(x=0)", [](CCShapeId s) { return cc_mirror_shape(s, 0, 0, 0, 1, 0, 0); }, 1.0},
        {"scale(x2@0)", [](CCShapeId s) { return cc_scale_shape(s, 2.0); }, 8.0},
        {"scale_about(x2@1,1,1)",
         [](CCShapeId s) { return cc_scale_shape_about(s, 1, 1, 1, 2.0); }, 8.0},
        {"place_on_frame(o=10,0,0;u=+Y;v=+Z)",
         [](CCShapeId s) { return cc_place_on_frame(s, 10, 0, 0, 0, 1, 0, 0, 0, 1); }, 1.0},
    };
    const std::vector<Base> bases = {
        {"BOX", buildBox, 1000.0, true},
        {"CYLINDER", buildCyl, kPi * 2.0 * 2.0 * 5.0, false},
    };

    for (const Base& b : bases)
        for (const Op& op : ops) runCase(b, op);

    runDegenerateScaleDecline();

    std::printf("== %d passed, %d failed ==\n", g_passed, g_failed);
    std::fflush(stdout);
    std::_Exit(g_failed == 0 ? 0 : 1);
}
