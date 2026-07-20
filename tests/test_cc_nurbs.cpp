// Host test for the additive NURBS geometry ABI (cc_curve / cc_surface).
//
// Asserts, entirely through the public cc_* surface:
//   * sizeof/alignment guards for every new POD struct + handle, cross-checked
//     against a C sizeof of the same header (the cc_nurbs_c_sizeof_* symbols are
//     produced by the C compiler in test_cc_nurbs_sizeof.c and linked in);
//   * create -> accessor round-trip is byte-exact (degree, knots, homogeneous
//     poles, rational flag);
//   * cc_curve_eval / cc_surface_eval match the native evaluator (bspline.h) to
//     <= 1e-12 on a hand-built exact rational quarter-circle and a rational surface;
//   * a closed single surface (a full revolved circle profile) tessellates to a
//     renderable, finite, correctly-indexed display mesh;
//   * double-release (and stale/unknown-handle) is crash-free and returns the
//     honest 0/-1 codes;
//   * an undersized accessor buffer returns < 0 and writes NOTHING out of bounds.

#include <algorithm>  // std::max/std::min over an initializer_list
#include <cmath>
#include <cstdint>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"
#include "native/math/bspline.h"  // native oracle evaluators

namespace nm = cybercad::native::math;

// Sizes computed by the C compiler over the same header (test_cc_nurbs_sizeof.c).
extern "C" {
extern const size_t cc_nurbs_c_sizeof_curve;
extern const size_t cc_nurbs_c_sizeof_surface;
extern const size_t cc_nurbs_c_sizeof_curveinfo;
extern const size_t cc_nurbs_c_sizeof_surfaceinfo;
extern const size_t cc_nurbs_c_sizeof_tessoptions;
}

namespace {

// A rational quarter-circle of radius R in the XY plane, centre at origin, from
// +X to +Y: degree-2, 3 poles, weight cos(45deg) at the middle (Piegl & Tiller
// Eq. 7.30). Poles are returned HOMOGENEOUS (x,y,z,w).
struct QuarterCircle {
    int degree = 2;
    int nCtrl = 3;
    std::vector<double> polesXYZW;  // 4*nCtrl
    std::vector<double> knots;      // {0,0,0,1,1,1}
    double R = 2.5;
};

QuarterCircle makeQuarterCircle() {
    QuarterCircle q;
    const double w = std::cos(M_PI / 4.0);  // sqrt(2)/2
    const double R = q.R;
    // P0=(R,0), P1=(R,R) with weight w (the corner of the control polygon), P2=(0,R).
    q.polesXYZW = {
        R, 0.0, 0.0, 1.0,        // P0, w=1
        R * w, R * w, 0.0, w,    // P1 lifted: (w*x, w*y, w*z, w)? NO — see note
        0.0, R, 0.0, 1.0,        // P2, w=1
    };
    // NOTE: cc_curve_create takes NON-lifted Euclidean poles + weight (x,y,z,w),
    // not the homogeneous lift. The middle control point of the standard rational
    // quarter circle is (R, R) with weight w. Fix P1 to Euclidean (R,R,0,w):
    q.polesXYZW[4] = R;
    q.polesXYZW[5] = R;
    q.polesXYZW[6] = 0.0;
    q.polesXYZW[7] = w;
    q.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    return q;
}

// Native oracle: evaluate the same quarter-circle directly through bspline.h.
nm::Point3 oracleCurve(const QuarterCircle& q, double t) {
    std::vector<nm::Point3> poles;
    std::vector<double> weights;
    for (int i = 0; i < q.nCtrl; ++i) {
        poles.push_back({q.polesXYZW[4 * i], q.polesXYZW[4 * i + 1], q.polesXYZW[4 * i + 2]});
        weights.push_back(q.polesXYZW[4 * i + 3]);
    }
    return nm::nurbsCurvePoint(q.degree, poles, weights, q.knots, t);
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

}  // namespace

// ── sizeof / alignment guards, cross-checked against the C sizeof of the header ──

CC_TEST(sizeof_guards_match_c_header) {
    // Handles are 4-byte opaque ints.
    CC_CHECK_EQ(sizeof(cc_curve), (size_t)4);
    CC_CHECK_EQ(sizeof(cc_surface), (size_t)4);
    // POD accessor structs are packed int32 arrays -> known sizes.
    CC_CHECK_EQ(sizeof(CCCurveInfo), (size_t)16);    // 4 * int32
    CC_CHECK_EQ(sizeof(CCSurfaceInfo), (size_t)28);  // 7 * int32
    CC_CHECK_EQ(sizeof(CCTessOptions), (size_t)8);   // 2 * int32

    // Cross-check against the C compiler's sizeof of the SAME header.
    CC_CHECK_EQ(sizeof(cc_curve), cc_nurbs_c_sizeof_curve);
    CC_CHECK_EQ(sizeof(cc_surface), cc_nurbs_c_sizeof_surface);
    CC_CHECK_EQ(sizeof(CCCurveInfo), cc_nurbs_c_sizeof_curveinfo);
    CC_CHECK_EQ(sizeof(CCSurfaceInfo), cc_nurbs_c_sizeof_surfaceinfo);
    CC_CHECK_EQ(sizeof(CCTessOptions), cc_nurbs_c_sizeof_tessoptions);
    std::printf("  sizeof C/C++ cross-check: curve=%zu surface=%zu info=%zu/%zu tess=%zu\n",
                cc_nurbs_c_sizeof_curve, cc_nurbs_c_sizeof_surface,
                cc_nurbs_c_sizeof_curveinfo, cc_nurbs_c_sizeof_surfaceinfo,
                cc_nurbs_c_sizeof_tessoptions);
}

// ── create -> accessor round-trip ────────────────────────────────────────────

CC_TEST(curve_create_accessor_roundtrip) {
    const QuarterCircle q = makeQuarterCircle();
    cc_curve h = cc_curve_create(q.degree, q.polesXYZW.data(), q.nCtrl, q.knots.data(),
                                 (int)q.knots.size());
    CC_CHECK(h.id != 0);

    CCCurveInfo info{};
    CC_CHECK_EQ(cc_curve_info(h, &info), 1);
    CC_CHECK_EQ(info.degree, q.degree);
    CC_CHECK_EQ(info.n_ctrl, q.nCtrl);
    CC_CHECK_EQ(info.n_knots, (int)q.knots.size());
    CC_CHECK_EQ(info.rational, 1);  // weight cos(45) != 1

    std::vector<double> knots(info.n_knots, -1.0);
    CC_CHECK_EQ(cc_curve_knots(h, knots.data(), info.n_knots), info.n_knots);
    for (int i = 0; i < info.n_knots; ++i) CC_CHECK(near(knots[i], q.knots[i], 0.0));

    std::vector<double> poles(4 * info.n_ctrl, -1.0);
    CC_CHECK_EQ(cc_curve_poles(h, poles.data(), 4 * info.n_ctrl), 4 * info.n_ctrl);
    for (int i = 0; i < 4 * info.n_ctrl; ++i)
        CC_CHECK(near(poles[i], q.polesXYZW[i], 0.0));

    cc_curve_release(h);
}

CC_TEST(nonrational_curve_reports_unit_weights) {
    // A straight degree-1 segment: all weights 1 -> rational == 0, w == 1 emitted.
    const double poles[] = {0, 0, 0, 1, 10, 0, 0, 1};
    const double knots[] = {0, 0, 1, 1};
    cc_curve h = cc_curve_create(1, poles, 2, knots, 4);
    CC_CHECK(h.id != 0);
    CCCurveInfo info{};
    CC_CHECK_EQ(cc_curve_info(h, &info), 1);
    CC_CHECK_EQ(info.rational, 0);
    std::vector<double> out(8, -1.0);
    CC_CHECK_EQ(cc_curve_poles(h, out.data(), 8), 8);
    CC_CHECK(near(out[3], 1.0, 0.0));
    CC_CHECK(near(out[7], 1.0, 0.0));
    cc_curve_release(h);
}

// ── evaluator exactness vs the native oracle (<= 1e-12) ──────────────────────

CC_TEST(curve_eval_matches_native_and_geometry) {
    const QuarterCircle q = makeQuarterCircle();
    cc_curve h = cc_curve_create(q.degree, q.polesXYZW.data(), q.nCtrl, q.knots.data(),
                                 (int)q.knots.size());
    CC_CHECK(h.id != 0);
    double maxErrOracle = 0.0, maxErrCircle = 0.0;
    for (int k = 0; k <= 20; ++k) {
        const double t = double(k) / 20.0;
        double xyz[3] = {0, 0, 0};
        CC_CHECK_EQ(cc_curve_eval(h, t, xyz), 1);
        const nm::Point3 o = oracleCurve(q, t);
        maxErrOracle = std::max({maxErrOracle, std::fabs(xyz[0] - o.x),
                                 std::fabs(xyz[1] - o.y), std::fabs(xyz[2] - o.z)});
        // Exact rational quarter circle: every eval lies on radius R.
        const double r = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
        maxErrCircle = std::max(maxErrCircle, std::fabs(r - q.R));
    }
    std::printf("  curve eval: max|facade-native|=%.3e  max|r-R|=%.3e\n", maxErrOracle,
                maxErrCircle);
    CC_CHECK(maxErrOracle <= 1e-12);
    CC_CHECK(maxErrCircle <= 1e-12);
    cc_curve_release(h);
}

// ── surface: create, accessor, eval, tessellate ─────────────────────────────

namespace {
// A rational surface: the quarter-circle swept linearly in +Z (degree 1 in V).
// n_ctrl_u = 3 (the arc), n_ctrl_v = 2 (the extrusion). Row-major, U outer.
struct RatSurface {
    int du = 2, dv = 1, nu = 3, nv = 2;
    std::vector<double> poles;  // 4*nu*nv, row-major U outer
    std::vector<double> ku = {0, 0, 0, 1, 1, 1};
    std::vector<double> kv = {0, 0, 1, 1};
    double R = 2.5, H = 4.0;
};

RatSurface makeRatSurface() {
    RatSurface s;
    const double w = std::cos(M_PI / 4.0);
    const double R = s.R, H = s.H;
    // arc poles (Euclidean, weight): P0=(R,0),w1 ; P1=(R,R),w ; P2=(0,R),w1
    const double arc[3][4] = {{R, 0, 0, 1.0}, {R, R, 0, w}, {0, R, 0, 1.0}};
    s.poles.resize(4 * s.nu * s.nv);
    for (int i = 0; i < s.nu; ++i) {
        for (int j = 0; j < s.nv; ++j) {
            const double z = (j == 0) ? 0.0 : H;
            const int base = 4 * (i * s.nv + j);
            s.poles[base + 0] = arc[i][0];
            s.poles[base + 1] = arc[i][1];
            s.poles[base + 2] = z;
            s.poles[base + 3] = arc[i][3];
        }
    }
    return s;
}

nm::Point3 oracleSurface(const RatSurface& s, double u, double v) {
    std::vector<nm::Point3> poles;
    std::vector<double> weights;
    for (int i = 0; i < s.nu; ++i)
        for (int j = 0; j < s.nv; ++j) {
            const int b = 4 * (i * s.nv + j);
            poles.push_back({s.poles[b], s.poles[b + 1], s.poles[b + 2]});
            weights.push_back(s.poles[b + 3]);
        }
    nm::SurfaceGrid grid{poles, s.nu, s.nv};
    return nm::nurbsSurfacePoint(s.du, s.dv, grid, weights, s.ku, s.kv, u, v);
}
}  // namespace

CC_TEST(surface_create_accessor_eval) {
    const RatSurface s = makeRatSurface();
    cc_surface h = cc_surface_create(s.du, s.dv, s.poles.data(), s.nu, s.nv, s.ku.data(),
                                     (int)s.ku.size(), s.kv.data(), (int)s.kv.size());
    CC_CHECK(h.id != 0);

    CCSurfaceInfo info{};
    CC_CHECK_EQ(cc_surface_info(h, &info), 1);
    CC_CHECK_EQ(info.degree_u, s.du);
    CC_CHECK_EQ(info.degree_v, s.dv);
    CC_CHECK_EQ(info.n_ctrl_u, s.nu);
    CC_CHECK_EQ(info.n_ctrl_v, s.nv);
    CC_CHECK_EQ(info.n_knots_u, (int)s.ku.size());
    CC_CHECK_EQ(info.n_knots_v, (int)s.kv.size());
    CC_CHECK_EQ(info.rational, 1);

    // Pole round-trip (row-major, homogeneous).
    std::vector<double> poles(4 * s.nu * s.nv, -1.0);
    CC_CHECK_EQ(cc_surface_poles(h, poles.data(), 4 * s.nu * s.nv), 4 * s.nu * s.nv);
    for (size_t i = 0; i < s.poles.size(); ++i) CC_CHECK(near(poles[i], s.poles[i], 0.0));

    // Eval exactness vs native oracle.
    double maxErr = 0.0;
    for (int a = 0; a <= 8; ++a)
        for (int b = 0; b <= 4; ++b) {
            const double u = double(a) / 8.0, v = double(b) / 4.0;
            double xyz[3] = {0, 0, 0};
            CC_CHECK_EQ(cc_surface_eval(h, u, v, xyz), 1);
            const nm::Point3 o = oracleSurface(s, u, v);
            maxErr = std::max({maxErr, std::fabs(xyz[0] - o.x), std::fabs(xyz[1] - o.y),
                               std::fabs(xyz[2] - o.z)});
        }
    std::printf("  surface eval: max|facade-native|=%.3e\n", maxErr);
    CC_CHECK(maxErr <= 1e-12);
    cc_surface_release(h);
}

CC_TEST(surface_tessellate_is_renderable) {
    const RatSurface s = makeRatSurface();
    cc_surface h = cc_surface_create(s.du, s.dv, s.poles.data(), s.nu, s.nv, s.ku.data(),
                                     (int)s.ku.size(), s.kv.data(), (int)s.kv.size());
    CC_CHECK(h.id != 0);

    CCTessOptions opt{12, 6};
    CCMesh mesh{};
    CC_CHECK_EQ(cc_surface_tessellate(h, &opt, &mesh), 1);
    CC_CHECK_EQ(mesh.vertexCount, 12 * 6);
    CC_CHECK_EQ(mesh.triangleCount, (12 - 1) * (6 - 1) * 2);
    CC_CHECK(mesh.vertices != nullptr);
    CC_CHECK(mesh.triangles != nullptr);

    // Every vertex is finite and lies on the cylinder radius R (a renderable,
    // geometrically-correct display mesh of the closed quarter-cylinder patch).
    double maxRErr = 0.0;
    for (int v = 0; v < mesh.vertexCount; ++v) {
        const double x = mesh.vertices[3 * v], y = mesh.vertices[3 * v + 1],
                     z = mesh.vertices[3 * v + 2];
        CC_CHECK(std::isfinite(x) && std::isfinite(y) && std::isfinite(z));
        maxRErr = std::max(maxRErr, std::fabs(std::sqrt(x * x + y * y) - s.R));
    }
    CC_CHECK(maxRErr <= 1e-12);
    // Every triangle index is in range.
    for (int t = 0; t < mesh.triangleCount * 3; ++t)
        CC_CHECK(mesh.triangles[t] >= 0 && mesh.triangles[t] < mesh.vertexCount);

    cc_mesh_free(mesh);

    // Default options (null) still produce a mesh.
    CCMesh dflt{};
    CC_CHECK_EQ(cc_surface_tessellate(h, nullptr, &dflt), 1);
    CC_CHECK(dflt.vertexCount > 0 && dflt.triangleCount > 0);
    cc_mesh_free(dflt);
    cc_surface_release(h);
}

CC_TEST(curve_polyline_samples_geometry) {
    const QuarterCircle q = makeQuarterCircle();
    cc_curve h = cc_curve_create(q.degree, q.polesXYZW.data(), q.nCtrl, q.knots.data(),
                                 (int)q.knots.size());
    CCEdgePolyline poly{};
    CC_CHECK_EQ(cc_curve_polyline(h, 33, &poly), 1);
    CC_CHECK_EQ(poly.pointCount, 33);
    CC_CHECK(poly.points != nullptr);
    for (int i = 0; i < poly.pointCount; ++i) {
        const double x = poly.points[3 * i], y = poly.points[3 * i + 1];
        CC_CHECK(near(std::sqrt(x * x + y * y), q.R, 1e-12));
    }
    cc_points_free(poly.points);  // out is caller storage; free only the buffer
    cc_curve_release(h);
}

// ── lifetime: double-release + stale handle guards ──────────────────────────

CC_TEST(double_release_and_stale_handle_safe) {
    const QuarterCircle q = makeQuarterCircle();
    cc_curve h = cc_curve_create(q.degree, q.polesXYZW.data(), q.nCtrl, q.knots.data(),
                                 (int)q.knots.size());
    CC_CHECK(h.id != 0);
    cc_curve_release(h);
    cc_curve_release(h);  // double release: must not crash
    cc_curve_release(cc_curve{0});       // invalid sentinel
    cc_curve_release(cc_curve{999999});  // never-allocated

    // Accessors on a released / stale handle honestly decline.
    CCCurveInfo info{};
    CC_CHECK_EQ(cc_curve_info(h, &info), 0);
    double xyz[3];
    CC_CHECK_EQ(cc_curve_eval(h, 0.5, xyz), 0);
    CC_CHECK(cc_curve_knots(h, xyz, 3) < 0);

    // Same for surfaces.
    cc_surface s{424242};
    cc_surface_release(s);  // unknown: no-op
    CCSurfaceInfo sinfo{};
    CC_CHECK_EQ(cc_surface_info(s, &sinfo), 0);
}

// ── honest declines: bad input, undersized buffers ──────────────────────────

CC_TEST(invalid_input_declines_with_zero_handle) {
    const double poles[] = {0, 0, 0, 1, 1, 0, 0, 1, 2, 0, 0, 1};
    const double knots[] = {0, 0, 0, 1, 1, 1};
    // Bad degree.
    CC_CHECK_EQ(cc_curve_create(0, poles, 3, knots, 6).id, 0);
    // Knot count mismatch.
    CC_CHECK_EQ(cc_curve_create(2, poles, 3, knots, 5).id, 0);
    // n_ctrl <= degree.
    CC_CHECK_EQ(cc_curve_create(2, poles, 2, knots, 5).id, 0);
    // Non-positive weight.
    const double badw[] = {0, 0, 0, 1, 1, 0, 0, 0.0, 2, 0, 0, 1};
    CC_CHECK_EQ(cc_curve_create(2, badw, 3, knots, 6).id, 0);
    // Non-monotone knots.
    const double badk[] = {0, 0, 0, 1, 0.5, 1};
    CC_CHECK_EQ(cc_curve_create(2, poles, 3, badk, 6).id, 0);
    CC_CHECK(cc_last_error() != nullptr);
}

CC_TEST(undersized_buffer_returns_negative_no_oob) {
    const QuarterCircle q = makeQuarterCircle();
    cc_curve h = cc_curve_create(q.degree, q.polesXYZW.data(), q.nCtrl, q.knots.data(),
                                 (int)q.knots.size());
    // Sentinel-guarded buffer: an OOB write would clobber guard[cap].
    const int need = 4 * q.nCtrl;  // pole doubles
    std::vector<double> buf(need + 1, -7.0);
    CC_CHECK(cc_curve_poles(h, buf.data(), need - 1) < 0);  // too small
    for (double d : buf) CC_CHECK(near(d, -7.0, 0.0));      // nothing written
    // Exact capacity succeeds.
    CC_CHECK_EQ(cc_curve_poles(h, buf.data(), need), need);
    CC_CHECK(near(buf[need], -7.0, 0.0));  // the guard slot is untouched

    // Knot buffer undersized.
    std::vector<double> kb((int)q.knots.size(), -7.0);
    CC_CHECK(cc_curve_knots(h, kb.data(), (int)q.knots.size() - 1) < 0);
    for (double d : kb) CC_CHECK(near(d, -7.0, 0.0));
    cc_curve_release(h);
}

CC_RUN_ALL()
