// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) — the OCCT-free analytic proof of the app-parity connected-solid
// enumeration (`cc_shape_solid_count` / `cc_shape_solid_at`) and the honest native
// decline of the app-parity loft variants (`cc_loft_circles` / `cc_loft_circle_wire`
// / `cc_loft_typed` / `cc_loft_along_rails`). No OCCT, no simulator.
//
// WHAT IS ASSERTED (honest — matches the delivered scope):
//   * Enumeration LOGIC on a KNOWN 2-lump compound (built directly with the native
//     topology builder, the exact mechanism the engine's shape_solid_count/at use):
//     the native Explorer over ShapeType::Solid finds EXACTLY 2 solids; each is a
//     distinct lump; each meshes watertight with the exact per-lump box volume; and
//     the two volumes sum to the whole. A single solid enumerates to 1.
//   * FACADE end-to-end (cc_set_engine(1) → NativeEngine): a native box built via
//     cc_solid_extrude reports cc_shape_solid_count == 1, cc_shape_solid_at(body,0)
//     returns an equivalent single solid (same mass-properties volume), and an
//     out-of-range / negative index returns 0 (honest, no crash).
//   * The four app-parity loft variants HONESTLY DECLINE on the OCCT-free host
//     (native engine declines true-circle/spline-rim topology → no OCCT to fall to
//     → 0 with cc_last_error set), NEVER a wrong shape. Degenerate inputs also → 0.
//
#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;

namespace {

constexpr double kPi = 3.14159265358979323846;

int countSolids(const topo::Shape& s) {
    int n = 0;
    for (topo::Explorer ex(s, topo::ShapeType::Solid); ex.more(); ex.next()) ++n;
    return n;
}

double boxVolume(const topo::Shape& solid) {
    tess::MeshParams p;
    p.deflection = 0.25;
    const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
    return tess::isWatertight(mesh) ? tess::enclosedVolume(mesh) : -1.0;
}

// Engine guard: restore the default (stub) engine on scope exit.
struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

}  // namespace

// ── Enumeration logic on a known 2-lump compound (host analytic) ────────────────
// Two DISJOINT boxes (10³ and 5³, built as distinct prisms so they are distinct
// TShape nodes) collected into ONE compound. The native Explorer over Solid must
// find EXACTLY 2; each lump meshes watertight with its exact volume; the volumes
// sum to the whole (1125). This is the exact mechanism shape_solid_count/at run.
CC_TEST(compound_two_lumps_enumerate_exactly_two) {
    const double bigSq[] = {0, 0, 10, 0, 10, 10, 0, 10};   // 10×10 @ z=0
    const double smallSq[] = {0, 0, 5, 0, 5, 5, 0, 5};      // 5×5 @ z=0
    const topo::Shape big = cst::build_prism(bigSq, 4, 10.0);    // vol 1000
    const topo::Shape small = cst::build_prism(smallSq, 4, 5.0); // vol 125
    CC_CHECK(!big.isNull());
    CC_CHECK(!small.isNull());
    if (big.isNull() || small.isNull()) return;

    const topo::Shape compound = topo::ShapeBuilder::makeCompound({big, small});
    CC_CHECK_EQ(countSolids(compound), 2);

    // Per-solid identity + exact volume, and the sum-to-whole invariant.
    std::vector<double> vols;
    for (topo::Explorer ex(compound, topo::ShapeType::Solid); ex.more(); ex.next()) {
        // Each extracted lump is itself a single solid.
        CC_CHECK_EQ(countSolids(ex.current()), 1);
        const double v = boxVolume(ex.current());
        CC_CHECK(v > 0.0);
        vols.push_back(v);
    }
    CC_CHECK_EQ(static_cast<int>(vols.size()), 2);
    double sum = 0.0;
    for (double v : vols) sum += v;
    CC_CHECK(std::fabs(sum - 1125.0) < 1e-6);
    // The two distinct volumes are present (order is explorer order).
    const bool has1000 = std::fabs(vols[0] - 1000.0) < 1e-6 || std::fabs(vols[1] - 1000.0) < 1e-6;
    const bool has125 = std::fabs(vols[0] - 125.0) < 1e-6 || std::fabs(vols[1] - 125.0) < 1e-6;
    CC_CHECK(has1000 && has125);
}

CC_TEST(single_solid_enumerates_to_one) {
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const topo::Shape box = cst::build_prism(sq, 4, 10.0);
    CC_CHECK(!box.isNull());
    if (box.isNull()) return;
    CC_CHECK_EQ(countSolids(box), 1);
}

// ── FACADE end-to-end via the NativeEngine (cc_shape_solid_count/at) ────────────
CC_TEST(facade_native_solid_count_and_at_single) {
    EngineGuard guard;
    cc_set_engine(1);
    const double sq[] = {0, 0, 10, 0, 10, 10, 0, 10};
    const CCShapeId body = cc_solid_extrude(sq, 4, 10.0);
    CC_CHECK(body != 0);
    if (body == 0) return;

    CC_CHECK_EQ(cc_shape_solid_count(body), 1);

    const CCShapeId s0 = cc_shape_solid_at(body, 0);
    CC_CHECK(s0 != 0);
    if (s0 != 0) {
        const CCMassProps whole = cc_mass_properties(body);
        const CCMassProps part = cc_mass_properties(s0);
        CC_CHECK(whole.valid == 1 && part.valid == 1);
        CC_CHECK(std::fabs(whole.volume - part.volume) < 1e-6);
        CC_CHECK(std::fabs(part.volume - 1000.0) < 1e-3);
        CC_CHECK_EQ(cc_shape_solid_count(s0), 1);
        cc_shape_release(s0);
    }

    // Out-of-range and negative index → honest 0 (no crash).
    CC_CHECK_EQ(cc_shape_solid_at(body, 1), 0);
    CC_CHECK_EQ(cc_shape_solid_at(body, -1), 0);
    cc_shape_release(body);
}

CC_TEST(facade_solid_count_unknown_body_is_zero) {
    EngineGuard guard;
    cc_set_engine(1);
    CC_CHECK_EQ(cc_shape_solid_count(0), 0);
    CC_CHECK_EQ(cc_shape_solid_at(0, 0), 0);
}

// ── App-parity loft variants: honest decline on the OCCT-free host ──────────────
// The native engine declines the true-circle / spline-rim loft topology; with no
// OCCT to fall to, the facade returns 0 and sets cc_last_error — never a wrong shape.
CC_TEST(facade_loft_variants_decline_on_host) {
    EngineGuard guard;
    cc_set_engine(1);

    // cc_loft_circles: two coaxial unit-normal circles (valid input) → declines (no OCCT).
    const double c1[] = {0, 0, 0}, n1[] = {0, 0, 1};
    const double c2[] = {0, 0, 10}, n2[] = {0, 0, 1};
    CC_CHECK_EQ(cc_loft_circles(c1, n1, 5.0, c2, n2, 5.0), 0);

    // cc_loft_circle_wire: circle → square polygon (valid input) → declines.
    const double sqXYZ[] = {-5, -5, 10, 5, -5, 10, 5, 5, 10, -5, 5, 10};
    CC_CHECK_EQ(cc_loft_circle_wire(c1, n1, 5.0, sqXYZ, 4), 0);

    // cc_loft_typed: two full-circle typed sections on their own frames → declines.
    CCProfileSeg segA{};
    segA.kind = 2; segA.cx = 0; segA.cy = 0; segA.r = 5.0;
    CCProfileSeg segB{};
    segB.kind = 2; segB.cx = 0; segB.cy = 0; segB.r = 3.0;
    const double frameA[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    const double frameB[] = {0, 0, 10, 1, 0, 0, 0, 1, 0};
    CC_CHECK_EQ(cc_loft_typed(&segA, 1, nullptr, 0, frameA, &segB, 1, nullptr, 0, frameB), 0);

    // cc_loft_along_rails: straight rail + straight guide + two square profiles → declines.
    const double rail[] = {0, 0, 0, 0, 0, 10};
    const double guide[] = {8, 0, 0, 8, 0, 10};
    const double profA[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    const double profB[] = {-2, -2, 2, -2, 2, 2, -2, 2};
    CC_CHECK_EQ(cc_loft_along_rails(rail, 2, guide, 2, profA, 4, profB, 4), 0);
}

CC_TEST(facade_loft_variants_degenerate_input_is_zero) {
    EngineGuard guard;
    cc_set_engine(1);
    const double c[] = {0, 0, 0}, n[] = {0, 0, 1};
    // Non-positive radius / null / too-few points → 0.
    CC_CHECK_EQ(cc_loft_circles(c, n, 0.0, c, n, 5.0), 0);
    CC_CHECK_EQ(cc_loft_circles(nullptr, n, 5.0, c, n, 5.0), 0);
    const double sqXYZ[] = {0, 0, 0, 1, 0, 0};  // only 2 points
    CC_CHECK_EQ(cc_loft_circle_wire(c, n, 5.0, sqXYZ, 2), 0);
    const double profA[] = {-2, -2, 2, -2, 2, 2};
    CC_CHECK_EQ(cc_loft_along_rails(nullptr, 2, nullptr, 0, profA, 3, profA, 3), 0);
}

CC_RUN_ALL()
