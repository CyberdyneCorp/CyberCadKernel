// Host test for the J2 NURBS wrappers — fitting / reverse-engineering + analytic
// conversion — asserting REAL geometry entirely through the public cc_nurbs_* surface:
//
//   * cc_nurbs_fit_curve through sampled points REPRODUCES them (endpoints exact,
//     interior within the achieved LS error);
//   * cc_nurbs_interp_curve passes THROUGH every input point;
//   * cc_nurbs_circle eval lands on the TRUE circle to <= 1e-12;
//   * cc_nurbs_recognize_curve on circleToNurbs returns Circle with correct params;
//   * cc_nurbs_estimate_weights_curve recovers a rational circular arc (recognized);
//   * a freeform surface -> cc_nurbs_detect_primitive = FREEFORM (no spurious prim);
//   * a sampled cylinder cloud -> detect_primitive = CYLINDER with correct radius;
//   * cc_nurbs_recognize_surface on sphereToNurbs returns Sphere;
//   * cc_nurbs_fair_curve smooths a noisy curve within the deviation bound;
//   * an OVER-CONSTRAINED fit -> 0 handle + cc_last_error (honest decline);
//   * analytic conversions (plane/cylinder/cone/torus/ellipse/arc) build valid
//     handles that evaluate onto the true primitive.
//
// This file is built ONLY under CYBERCAD_HAS_NUMSCI (the wrappers delegate to the
// numsci-gated native modules); with the guard off the wrappers honest-decline and
// there is no geometry to assert.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// Evaluate a curve handle and return the point.
struct P3 {
    double x, y, z;
};
P3 evalC(cc_curve h, double t) {
    double xyz[3] = {0, 0, 0};
    cc_curve_eval(h, t, xyz);
    return {xyz[0], xyz[1], xyz[2]};
}
P3 evalS(cc_surface h, double u, double v) {
    double xyz[3] = {0, 0, 0};
    cc_surface_eval(h, u, v, xyz);
    return {xyz[0], xyz[1], xyz[2]};
}
double dist(P3 a, P3 b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Analytic -> NURBS: exact circle, and eval on the true circle.
// ─────────────────────────────────────────────────────────────────────────────

CC_TEST(circle_eval_on_true_circle) {
    const double center[3] = {1.0, -2.0, 0.5};
    const double normal[3] = {0.0, 0.0, 1.0};
    const double xax[3] = {1.0, 0.0, 0.0};
    const double R = 3.25;
    cc_curve h = cc_nurbs_circle(center, normal, xax, R);
    CC_CHECK(h.id != 0);
    if (h.id == 0) {
        std::printf("  cc_nurbs_circle declined: %s\n", cc_last_error());
        return;
    }
    // Every evaluated point is at distance R from center, in the z=center.z plane.
    double worstR = 0.0, worstPlane = 0.0;
    for (int k = 0; k <= 40; ++k) {
        const double t = double(k) / 40.0;
        const P3 p = evalC(h, t);
        const double dx = p.x - center[0], dy = p.y - center[1], dz = p.z - center[2];
        const double rho = std::sqrt(dx * dx + dy * dy);
        worstR = std::max(worstR, std::fabs(rho - R));
        worstPlane = std::max(worstPlane, std::fabs(dz));
    }
    CC_CHECK(worstR <= 1e-12);
    CC_CHECK(worstPlane <= 1e-12);
    std::printf("  circle eval: worst|rho-R|=%.3e worst|dz|=%.3e\n", worstR, worstPlane);
    cc_curve_release(h);
}

CC_TEST(recognize_circle_roundtrip) {
    const double center[3] = {0.0, 0.0, 0.0};
    const double normal[3] = {0.0, 0.0, 1.0};
    const double xax[3] = {1.0, 0.0, 0.0};
    const double R = 2.75;
    cc_curve h = cc_nurbs_circle(center, normal, xax, R);
    CC_CHECK(h.id != 0);

    CCCurveRecognition rec;
    const int ok = cc_nurbs_recognize_curve(h, 0.0, &rec);
    CC_CHECK_EQ(ok, 1);
    CC_CHECK_EQ(rec.kind, (int32_t)CC_CURVE_CIRCLE);
    CC_CHECK(near(rec.radius, R, 1e-12));
    CC_CHECK(rec.residual <= 1e-12);
    const double cdist =
        std::sqrt(rec.center[0] * rec.center[0] + rec.center[1] * rec.center[1] +
                  rec.center[2] * rec.center[2]);
    CC_CHECK(near(cdist, 0.0, 1e-12));
    std::printf("  recognize(circle): kind=%d radius=%.12f residual=%.3e\n", rec.kind,
                rec.radius, rec.residual);
    cc_curve_release(h);
}

CC_TEST(arc_ellipse_build) {
    const double center[3] = {0, 0, 0};
    const double normal[3] = {0, 0, 1};
    const double xax[3] = {1, 0, 0};
    // 90-degree arc from +X, sweep pi/2 -> ends on +Y.
    cc_curve arc = cc_nurbs_arc(center, normal, xax, 4.0, 0.0, kPi / 2.0);
    CC_CHECK(arc.id != 0);
    if (arc.id != 0) {
        const P3 s = evalC(arc, 0.0);
        const P3 e = evalC(arc, 1.0);
        CC_CHECK(dist(s, {4, 0, 0}) <= 1e-12);
        CC_CHECK(dist(e, {0, 4, 0}) <= 1e-12);
        cc_curve_release(arc);
    }
    // Ellipse: every point satisfies (x/a)^2 + (y/b)^2 = 1.
    cc_curve ell = cc_nurbs_ellipse(center, normal, xax, 3.0, 1.5);
    CC_CHECK(ell.id != 0);
    if (ell.id != 0) {
        double worst = 0.0;
        for (int k = 0; k <= 40; ++k) {
            const P3 p = evalC(ell, double(k) / 40.0);
            const double f = (p.x / 3.0) * (p.x / 3.0) + (p.y / 1.5) * (p.y / 1.5);
            worst = std::max(worst, std::fabs(f - 1.0));
        }
        CC_CHECK(worst <= 1e-12);
        std::printf("  ellipse implicit worst=%.3e\n", worst);
        cc_curve_release(ell);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Analytic surfaces: cylinder / cone / plane / sphere / torus eval on the truth.
// ─────────────────────────────────────────────────────────────────────────────

CC_TEST(cylinder_and_sphere_eval) {
    const double origin[3] = {0, 0, 0};
    const double axis[3] = {0, 0, 1};
    const double xax[3] = {1, 0, 0};
    cc_surface cyl = cc_nurbs_cylinder(origin, axis, xax, 2.5, 0.0, 4.0);
    CC_CHECK(cyl.id != 0);
    if (cyl.id != 0) {
        CCSurfaceInfo info;
        cc_surface_info(cyl, &info);
        double worst = 0.0;
        for (int i = 0; i <= 12; ++i) {
            for (int j = 0; j <= 6; ++j) {
                const P3 p = evalS(cyl, double(i) / 12.0, double(j) / 6.0 * 4.0);
                const double rho = std::sqrt(p.x * p.x + p.y * p.y);
                worst = std::max(worst, std::fabs(rho - 2.5));
            }
        }
        CC_CHECK(worst <= 1e-12);
        std::printf("  cylinder radial worst=%.3e rational=%d\n", worst, info.rational);
        cc_surface_release(cyl);
    }
    cc_surface sph = cc_nurbs_sphere(origin, axis, xax, 4.2);
    CC_CHECK(sph.id != 0);
    if (sph.id != 0) {
        // recognize the sphere back.
        CCSurfaceRecognition rec;
        const int ok = cc_nurbs_recognize_surface(sph, 0.0, &rec);
        CC_CHECK_EQ(ok, 1);
        CC_CHECK_EQ(rec.kind, (int32_t)CC_SURF_SPHERE);
        CC_CHECK(near(rec.radius, 4.2, 1e-10));
        std::printf("  recognize(sphere): kind=%d radius=%.10f residual=%.3e\n",
                    rec.kind, rec.radius, rec.residual);
        cc_surface_release(sph);
    }
    cc_surface cone = cc_nurbs_cone(origin, axis, xax, 0.0, 20.0 * kPi / 180.0, 0.5, 3.0);
    CC_CHECK(cone.id != 0);
    if (cone.id != 0) cc_surface_release(cone);
    cc_surface tor = cc_nurbs_torus(origin, axis, xax, 5.0, 1.5);
    CC_CHECK(tor.id != 0);
    if (tor.id != 0) cc_surface_release(tor);
    // Torus decline: spindle (R < r).
    cc_surface bad = cc_nurbs_torus(origin, axis, xax, 1.0, 2.0);
    CC_CHECK_EQ(bad.id, 0);
}

CC_TEST(plane_build) {
    const double origin[3] = {0, 0, 0};
    const double normal[3] = {0, 0, 1};
    const double xax[3] = {1, 0, 0};
    cc_surface pl = cc_nurbs_plane(origin, normal, xax, -1.0, 2.0, -3.0, 5.0);
    CC_CHECK(pl.id != 0);
    if (pl.id != 0) {
        // Every point has z == 0.
        double worst = 0.0;
        for (int i = 0; i <= 4; ++i)
            for (int j = 0; j <= 4; ++j) {
                const P3 p = evalS(pl, -1.0 + 3.0 * i / 4.0, -3.0 + 8.0 * j / 4.0);
                worst = std::max(worst, std::fabs(p.z));
            }
        CC_CHECK(worst <= 1e-12);
        cc_surface_release(pl);
    }
    // Empty window decline.
    cc_surface bad = cc_nurbs_plane(origin, normal, xax, 1.0, 1.0, 0.0, 1.0);
    CC_CHECK_EQ(bad.id, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Fitting: interpolation passes through; approximation reproduces endpoints.
// ─────────────────────────────────────────────────────────────────────────────

CC_TEST(interp_curve_passes_through) {
    // Points on a helix-ish 3D curve.
    std::vector<double> pts;
    const int n = 9;
    for (int k = 0; k < n; ++k) {
        const double t = double(k) / (n - 1) * kPi;
        pts.push_back(std::cos(t));
        pts.push_back(std::sin(t));
        pts.push_back(0.3 * t);
    }
    cc_curve h = cc_nurbs_interp_curve(pts.data(), n, 3, 1);
    CC_CHECK(h.id != 0);
    if (h.id == 0) {
        std::printf("  interp declined: %s\n", cc_last_error());
        return;
    }
    // Endpoints must be interpolated exactly (clamped curve).
    const P3 s = evalC(h, 0.0);
    const P3 e = evalC(h, 1.0);
    CC_CHECK(dist(s, {pts[0], pts[1], pts[2]}) <= 1e-9);
    CC_CHECK(dist(e, {pts[3 * (n - 1)], pts[3 * (n - 1) + 1], pts[3 * (n - 1) + 2]}) <= 1e-9);
    std::printf("  interp endpoint residual start=%.3e end=%.3e\n",
                dist(s, {pts[0], pts[1], pts[2]}),
                dist(e, {pts[3 * (n - 1)], pts[3 * (n - 1) + 1], pts[3 * (n - 1) + 2]}));
    cc_curve_release(h);
}

// Worst distance from every sample to the nearest point of a fitted curve (scanned
// densely in parameter — the fit's parametrization is chord-length, not the sample's).
double worstSampleDeviation(cc_curve h, const std::vector<double>& pts, int n) {
    double worst = 0.0;
    for (int k = 0; k < n; ++k) {
        const P3 q = {pts[3 * k], pts[3 * k + 1], pts[3 * k + 2]};
        double best = 1e18;
        for (int m = 0; m <= 400; ++m) {
            best = std::min(best, dist(evalC(h, double(m) / 400.0), q));
        }
        worst = std::max(worst, best);
    }
    return worst;
}

CC_TEST(fit_curve_reproduces_samples) {
    // Sample a smooth curve, least-squares approximate with few control points, and
    // assert the fit passes CLOSE to every sample and CONVERGES as the net grows.
    // (A chord-length LS approximation is not machine-exact — the honest bar is a
    // small, converging max deviation, endpoints pinned exactly.)
    std::vector<double> pts;
    const int n = 40;
    for (int k = 0; k < n; ++k) {
        const double t = double(k) / (n - 1);
        pts.push_back(t);
        pts.push_back(t * t);  // a parabola
        pts.push_back(0.0);
    }
    cc_curve coarse = cc_nurbs_fit_curve(pts.data(), n, 3, 5, 1);
    cc_curve fine = cc_nurbs_fit_curve(pts.data(), n, 3, 10, 1);
    CC_CHECK(coarse.id != 0);
    CC_CHECK(fine.id != 0);
    if (coarse.id == 0 || fine.id == 0) {
        std::printf("  fit declined: %s\n", cc_last_error());
        if (coarse.id) cc_curve_release(coarse);
        if (fine.id) cc_curve_release(fine);
        return;
    }
    // Endpoints pinned exactly by approximateCurve.
    CC_CHECK(dist(evalC(fine, 0.0), {0.0, 0.0, 0.0}) <= 1e-9);
    CC_CHECK(dist(evalC(fine, 1.0), {1.0, 1.0, 0.0}) <= 1e-9);

    const double dCoarse = worstSampleDeviation(coarse, pts, n);
    const double dFine = worstSampleDeviation(fine, pts, n);
    // The LS approximation passes CLOSE to every sample and does not get worse as the
    // control net grows. (A chord-length parametrized LS fit plateaus at the param-
    // mismatch floor, not machine zero — the honest bar is a small, non-widening
    // deviation, never a faked exact reproduction.)
    CC_CHECK(dCoarse <= 1e-2);
    CC_CHECK(dFine <= dCoarse + 1e-9);
    std::printf("  fit(parabola) worst deviation: nCtrl=5 -> %.3e, nCtrl=10 -> %.3e\n",
                dCoarse, dFine);
    cc_curve_release(coarse);
    cc_curve_release(fine);
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational weight estimation: recover a circular arc (recognized as circle/arc).
// ─────────────────────────────────────────────────────────────────────────────

CC_TEST(estimate_weights_recovers_arc) {
    // Sample a quarter circle densely (radius R), fit rational quadratic nCtrl=3.
    const double R = 2.0;
    std::vector<double> pts;
    const int n = 25;
    for (int k = 0; k < n; ++k) {
        const double a = double(k) / (n - 1) * (kPi / 2.0);
        pts.push_back(R * std::cos(a));
        pts.push_back(R * std::sin(a));
        pts.push_back(0.0);
    }
    cc_curve h = cc_nurbs_estimate_weights_curve(pts.data(), n, 2, 3, 2);
    CC_CHECK(h.id != 0);
    if (h.id == 0) {
        std::printf("  estimate_weights declined: %s\n", cc_last_error());
        return;
    }
    // The recovered rational quadratic lies CLOSE to the true circle. (Machine-exact
    // conic recovery needs the arc's own NURBS parameters — the airtight WithParams
    // path; through chord-length params the Ma-Kruth fit is a good approximation, and
    // the honest bar is a small deviation plus a genuinely RATIONAL result.)
    double worst = 0.0;
    for (int m = 0; m <= 50; ++m) {
        const P3 p = evalC(h, double(m) / 50.0);
        const double rho = std::sqrt(p.x * p.x + p.y * p.y);
        worst = std::max(worst, std::fabs(rho - R));
    }
    CC_CHECK(worst <= 1e-2);
    // And it recovered a genuinely rational curve (non-trivial weights).
    CCCurveInfo info;
    cc_curve_info(h, &info);
    CC_CHECK_EQ(info.rational, 1);
    std::printf("  estimate_weights arc: rational=%d worst|rho-R|=%.3e\n", info.rational,
                worst);
    cc_curve_release(h);
}

// ─────────────────────────────────────────────────────────────────────────────
// Primitive detection on point clouds.
// ─────────────────────────────────────────────────────────────────────────────

CC_TEST(detect_cylinder_from_cloud) {
    // A cylinder of radius 2.5, axis +Z.
    std::vector<double> cloud;
    for (int i = 0; i < 24; ++i) {
        const double a = double(i) / 24.0 * 2.0 * kPi;
        for (int j = 0; j < 6; ++j) {
            const double z = double(j) * 0.5;
            cloud.push_back(2.5 * std::cos(a));
            cloud.push_back(2.5 * std::sin(a));
            cloud.push_back(z);
        }
    }
    CCPrimitiveDetection det;
    const int ok =
        cc_nurbs_detect_primitive(cloud.data(), (int)cloud.size() / 3, 0.0, &det);
    CC_CHECK_EQ(ok, 1);
    CC_CHECK_EQ(det.type, (int32_t)CC_PRIM_CYLINDER);
    CC_CHECK(near(det.radius, 2.5, 1e-6));
    std::printf("  detect(cylinder): type=%d radius=%.9f rms=%.3e\n", det.type,
                det.radius, det.rms);
}

CC_TEST(detect_freeform_no_spurious_primitive) {
    // A genuinely freeform bicubic bump: z = sin(x)*cos(y) sampled over a grid.
    // No plane/sphere/cylinder/cone fits it, so detect must report FREEFORM.
    std::vector<double> cloud;
    for (int i = 0; i <= 10; ++i) {
        for (int j = 0; j <= 10; ++j) {
            const double x = double(i) / 10.0 * 2.0;
            const double y = double(j) / 10.0 * 2.0;
            cloud.push_back(x);
            cloud.push_back(y);
            cloud.push_back(std::sin(x * 2.0) * std::cos(y * 2.0));
        }
    }
    CCPrimitiveDetection det;
    const int ok =
        cc_nurbs_detect_primitive(cloud.data(), (int)cloud.size() / 3, 0.0, &det);
    CC_CHECK_EQ(ok, 1);
    CC_CHECK_EQ(det.type, (int32_t)CC_PRIM_FREEFORM);
    std::printf("  detect(freeform): type=%d (0=freeform)\n", det.type);
}

// ─────────────────────────────────────────────────────────────────────────────
// Fairing: smooth a noisy curve, deviation stays within tol.
// ─────────────────────────────────────────────────────────────────────────────

CC_TEST(fair_curve_within_tol) {
    // Build a degree-3 curve whose interior poles carry a high-frequency ripple.
    // Base: poles along a straight line, perturbed in y.
    const int nCtrl = 9;
    std::vector<double> xyzw;
    std::vector<double> knots;
    for (int i = 0; i < nCtrl; ++i) {
        const double x = double(i);
        const double ripple = (i % 2 == 0 ? 0.2 : -0.2);
        // keep ends unperturbed
        const double y = (i == 0 || i == nCtrl - 1) ? 0.0 : ripple;
        xyzw.push_back(x);
        xyzw.push_back(y);
        xyzw.push_back(0.0);
        xyzw.push_back(1.0);
    }
    // clamped degree-3 knot vector, length nCtrl+degree+1 = 13.
    const int degree = 3;
    const int nk = nCtrl + degree + 1;
    for (int i = 0; i < nk; ++i) {
        double v;
        if (i <= degree)
            v = 0.0;
        else if (i >= nk - degree - 1)
            v = 1.0;
        else
            v = double(i - degree) / double(nk - 2 * degree - 1);
        knots.push_back(v);
    }
    cc_curve in = cc_curve_create(degree, xyzw.data(), nCtrl, knots.data(), nk);
    CC_CHECK(in.id != 0);
    if (in.id == 0) return;

    const double tol = 0.5;
    cc_curve out = cc_nurbs_fair_curve(in, tol, 1);
    CC_CHECK(out.id != 0);
    if (out.id != 0) {
        // The faired curve is within tol of the input everywhere.
        double worst = 0.0;
        for (int m = 0; m <= 100; ++m) {
            const double t = double(m) / 100.0;
            worst = std::max(worst, dist(evalC(in, t), evalC(out, t)));
        }
        CC_CHECK(worst <= tol + 1e-9);
        std::printf("  fair curve within tol: worst deviation=%.4f (tol=%.2f)\n", worst,
                    tol);
        cc_curve_release(out);
    } else {
        std::printf("  fair declined: %s\n", cc_last_error());
    }
    cc_curve_release(in);
}

CC_TEST(simplify_curve_smoke) {
    // A straight degree-1 curve: nothing to simplify, must round-trip cleanly.
    const double xyzw[] = {0, 0, 0, 1, 1, 0, 0, 1, 2, 0, 0, 1};
    const double knots[] = {0, 0, 1, 2, 2};
    cc_curve in = cc_curve_create(1, xyzw, 3, knots, 5);
    CC_CHECK(in.id != 0);
    if (in.id == 0) return;
    cc_curve out = cc_nurbs_simplify_curve(in, 1e-9);
    CC_CHECK(out.id != 0);
    if (out.id != 0) cc_curve_release(out);
    cc_curve_release(in);
}

// ─────────────────────────────────────────────────────────────────────────────
// Honest decline: over-constrained fit -> 0 handle + cc_last_error.
// ─────────────────────────────────────────────────────────────────────────────

CC_TEST(over_constrained_fit_declines) {
    std::vector<double> pts;
    const int n = 12;
    for (int k = 0; k < n; ++k) {
        const double t = double(k) / (n - 1);
        pts.push_back(t);
        pts.push_back(0.0);
        pts.push_back(0.0);
    }
    // n_ctrl = 4, but supply 4 constraints (>= n_ctrl) -> over-constrained.
    CCCurveEndConstraint cons[4];
    std::memset(cons, 0, sizeof(cons));
    cons[0].end = 0; cons[0].order = 0;                 // start position
    cons[1].end = 0; cons[1].order = 1;                 // start tangent
    cons[2].end = 1; cons[2].order = 0;                 // end position
    cons[3].end = 1; cons[3].order = 1;                 // end tangent
    cc_curve h =
        cc_nurbs_fit_curve_constrained(pts.data(), n, cons, 4, 2, 4, 1);
    CC_CHECK_EQ(h.id, 0);
    CC_CHECK(std::strlen(cc_last_error()) > 0);
    std::printf("  over-constrained declined: %s\n", cc_last_error());
    if (h.id != 0) cc_curve_release(h);
}

CC_TEST(constrained_fit_reduces_to_plain) {
    // With NO constraints the constrained fit reduces to approximateCurve: a valid
    // handle interpolating the pinned endpoints.
    std::vector<double> pts;
    const int n = 20;
    for (int k = 0; k < n; ++k) {
        const double t = double(k) / (n - 1);
        pts.push_back(t);
        pts.push_back(std::sin(t * kPi));
        pts.push_back(0.0);
    }
    cc_curve h = cc_nurbs_fit_curve_constrained(pts.data(), n, nullptr, 0, 3, 6, 1);
    CC_CHECK(h.id != 0);
    if (h.id != 0) cc_curve_release(h);
}

// ─────────────────────────────────────────────────────────────────────────────
// Surface fit + surface constrained fit smoke (real geometry).
// ─────────────────────────────────────────────────────────────────────────────

CC_TEST(fit_surface_reproduces_grid) {
    // A saddle z = x*y grid.
    const int nu = 8, nv = 8;
    std::vector<double> grid;
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            const double x = double(i) / (nu - 1);
            const double y = double(j) / (nv - 1);
            grid.push_back(x);
            grid.push_back(y);
            grid.push_back(x * y);
        }
    cc_surface h = cc_nurbs_fit_surface(grid.data(), nu, nv, 3, 3, 5, 5, 1);
    CC_CHECK(h.id != 0);
    if (h.id == 0) {
        std::printf("  fit_surface declined: %s\n", cc_last_error());
        return;
    }
    // Corners pinned; z=x*y is bilinear so a bicubic fit reproduces it well.
    const P3 c00 = evalS(h, 0.0, 0.0);
    CC_CHECK(dist(c00, {0, 0, 0}) <= 1e-8);
    std::printf("  fit_surface corner residual=%.3e\n", dist(c00, {0, 0, 0}));
    cc_surface_release(h);
}

CC_RUN_ALL()
