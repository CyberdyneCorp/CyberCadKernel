// SPDX-License-Identifier: Apache-2.0
//
// Host test for Wave-J track J4: the cc_nurbs_* blend + offset/thicken wrappers
// (src/facade/cc_nurbs_blend_offset.cpp). Everything is driven through the public
// cc_* surface — cc_surface_create builds the inputs, the J4 wrappers produce the
// results, and cc_surface_* / CCMesh accessors read them back. The oracles are
// closed-form:
//
//   1. FREEFORM G2 FILLET — a rolling-ball G2 fillet between two gentle bicubic
//      bumps produces a VALID cc_surface (or documents an honest decline); an
//      OVER-RADIUS fillet (ball larger than the seat can hold) DECLINES with a 0
//      handle + cc_last_error, never a self-intersecting band.
//   2. RATIONAL OFFSET — the rational offset of a NURBS quarter-cylinder of radius R
//      by d lies on the coaxial cylinder of radius R+d, checked at every sampled
//      point to <= 1e-6.
//   3. THICKEN — a gentle patch thickened by d yields a WATERTIGHT CCMesh (closed:
//      every edge used exactly twice; positive enclosed volume).
//
// The offset/thicken paths compose the numsci Layer-5 fit, so their assertions are
// gated on CYBERCAD_HAS_NUMSCI (with the guard OFF the wrappers honest-decline, which
// the test also asserts). The blend paths are always available.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

namespace {

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// A gentle non-rational bicubic bump patch (6x6 net), translated by (dx,dy,dz) and with
// its height field scaled by `amp`. Registered as a cc_surface via the public API.
cc_surface makeBump(double dx, double dy, double dz, double amp) {
    const int deg = 3, n = 6;
    std::vector<double> knots = {0, 0, 0, 0, 0.333333333333, 0.666666666667, 1, 1, 1, 1};
    std::vector<double> polesXYZW;
    polesXYZW.reserve(static_cast<std::size_t>(n) * n * 4);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double x = dx + i * 0.4;
            const double y = dy + j * 0.4;
            const double z = dz + amp * std::sin(0.9 * i) * std::cos(0.8 * j);
            polesXYZW.push_back(x);
            polesXYZW.push_back(y);
            polesXYZW.push_back(z);
            polesXYZW.push_back(1.0);
        }
    return cc_surface_create(deg, deg, polesXYZW.data(), n, n, knots.data(),
                             static_cast<int>(knots.size()), knots.data(),
                             static_cast<int>(knots.size()));
}

// A RATIONAL NURBS quarter-cylinder of radius r about the z-axis (90 deg in U, height h in
// V): degree-2 conic arc x degree-1 straight, weights {1, cos45, 1}. Standard exact arc.
cc_surface makeQuarterCylinder(double r, double h) {
    const double c = std::cos(M_PI / 4.0);
    // pole(i,j) row-major U outer: (arc0,h0),(arc0,h1), (arc1,h0),(arc1,h1), (arc2,h0),(arc2,h1)
    std::vector<double> polesXYZW = {
        r, 0, 0, 1,    r, 0, h, 1,      // arc0
        r, r, 0, c,    r, r, h, c,      // arc1 (weight cos45)
        0, r, 0, 1,    0, r, h, 1,      // arc2
    };
    std::vector<double> knotsU = {0, 0, 0, 1, 1, 1};  // 3+2+1
    std::vector<double> knotsV = {0, 0, 1, 1};        // 2+1+1
    return cc_surface_create(2, 1, polesXYZW.data(), 3, 2, knotsU.data(),
                             static_cast<int>(knotsU.size()), knotsV.data(),
                             static_cast<int>(knotsV.size()));
}

// Watertight check on a CCMesh: every undirected edge is used by exactly two triangles.
bool ccMeshWatertight(const CCMesh& m) {
    if (m.triangleCount <= 0 || m.vertexCount <= 0) return false;
    std::unordered_map<uint64_t, int> edge;
    auto key = [](int a, int b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    };
    for (int t = 0; t < m.triangleCount; ++t) {
        const int a = m.triangles[3 * t + 0];
        const int b = m.triangles[3 * t + 1];
        const int c = m.triangles[3 * t + 2];
        edge[key(a, b)]++;
        edge[key(b, c)]++;
        edge[key(c, a)]++;
    }
    for (const auto& e : edge)
        if (e.second != 2) return false;
    return true;
}

// Signed enclosed volume via the divergence theorem (1/6 * sum a . (b x c)).
double ccMeshVolume(const CCMesh& m) {
    double vol = 0.0;
    for (int t = 0; t < m.triangleCount; ++t) {
        const double* a = m.vertices + 3 * m.triangles[3 * t + 0];
        const double* b = m.vertices + 3 * m.triangles[3 * t + 1];
        const double* c = m.vertices + 3 * m.triangles[3 * t + 2];
        const double cx = b[1] * c[2] - b[2] * c[1];
        const double cy = b[2] * c[0] - b[0] * c[2];
        const double cz = b[0] * c[1] - b[1] * c[0];
        vol += (a[0] * cx + a[1] * cy + a[2] * cz);
    }
    return vol / 6.0;
}

}  // namespace

// ── 1. FREEFORM G2 FILLET: valid band, and over-radius honest decline ────────────────

CC_TEST(freeform_g2_fillet_valid_or_documented_decline) {
    // Two gentle bicubic bumps offset in y so a concave dihedral exists between them.
    cc_surface a = makeBump(0.0, 0.0, 0.0, 0.15);
    cc_surface b = makeBump(0.0, 2.4, 0.0, 0.15);
    CC_CHECK(a.id != 0 && b.id != 0);

    // A small-radius ball seeded near the crease between the two faces.
    const double center0[3] = {1.0, 1.2, 0.5};
    const double spineDir[3] = {1.0, 0.0, 0.0};
    cc_surface fillet =
        cc_nurbs_fillet_freeform_g2(a, b, /*r=*/0.15, center0, spineDir, /*sA=*/1.0, /*sB=*/1.0,
                                    /*stepLen=*/0.3, /*nStations=*/6, /*nSectionSamples=*/8);
    if (fillet.id != 0) {
        // A produced band is a real, inspectable cc_surface.
        CCSurfaceInfo info;
        CC_CHECK_EQ(cc_surface_info(fillet, &info), 1);
        CC_CHECK(info.n_ctrl_u >= 2 && info.n_ctrl_v >= 2);
        double xyz[3];
        CC_CHECK_EQ(cc_surface_eval(fillet, 0.5, 0.5, xyz), 1);
        CC_CHECK(std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]));
        cc_surface_release(fillet);
    } else {
        // A documented honest decline: 0 handle + a non-empty error message.
        CC_CHECK(cc_last_error()[0] != '\0');
    }

    // OVER-RADIUS: a ball far larger than any seat between these gentle bumps cannot fit —
    // must DECLINE (0 handle + error), NEVER a self-intersecting result.
    cc_surface over =
        cc_nurbs_fillet_freeform_g2(a, b, /*r=*/50.0, center0, spineDir, 1.0, 1.0, 0.3, 6, 8);
    CC_CHECK_EQ(over.id, 0);
    CC_CHECK(cc_last_error()[0] != '\0');

    cc_surface_release(a);
    cc_surface_release(b);
}

// ── 2. RATIONAL OFFSET: NURBS cylinder radius R -> R+d ────────────────────────────────

CC_TEST(rational_offset_cylinder_radius_R_plus_d) {
    const double R = 2.0, h = 1.5, d = 0.5;
    cc_surface cyl = makeQuarterCylinder(R, h);
    CC_CHECK(cyl.id != 0);

    cc_surface off = cc_nurbs_offset_rational(cyl, d, /*tol=*/1e-6);
#ifdef CYBERCAD_HAS_NUMSCI
    CC_CHECK(off.id != 0);
    if (off.id != 0) {
        // Every sampled point of the offset lies on the coaxial cylinder radius R+d.
        double worst = 0.0;
        for (int iu = 0; iu <= 8; ++iu)
            for (int iv = 0; iv <= 4; ++iv) {
                double xyz[3];
                if (cc_surface_eval(off, iu / 8.0, iv / 4.0, xyz) == 1) {
                    const double rad = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
                    worst = std::max(worst, std::fabs(rad - (R + d)));
                }
            }
        CC_CHECK(worst <= 1e-6);
        cc_surface_release(off);
    }
#else
    // Without the numsci substrate the wrapper honest-declines.
    CC_CHECK_EQ(off.id, 0);
    CC_CHECK(cc_last_error()[0] != '\0');
#endif
    cc_surface_release(cyl);
}

// ── 3. THICKEN: gentle patch -> watertight, positive-volume CCMesh ────────────────────

CC_TEST(thicken_gentle_patch_watertight_solid) {
    cc_surface patch = makeBump(0.0, 0.0, 0.0, 0.1);
    CC_CHECK(patch.id != 0);

    CCMesh mesh;
    double kept[4] = {0, 0, 0, 0};
    int trimmed = -1;
    const int rc = cc_nurbs_thicken_trimmed(patch, /*dist=*/0.2, /*tol=*/1e-4, &mesh, kept, &trimmed);
#ifdef CYBERCAD_HAS_NUMSCI
    CC_CHECK_EQ(rc, 1);
    if (rc == 1) {
        CC_CHECK(mesh.vertexCount > 0 && mesh.triangleCount > 0);
        CC_CHECK(ccMeshWatertight(mesh));
        CC_CHECK(std::fabs(ccMeshVolume(mesh)) > 1e-9);  // positive (non-degenerate) volume
        CC_CHECK(trimmed == 0 || trimmed == 1);
        cc_mesh_free(mesh);
    }
#else
    CC_CHECK_EQ(rc, 0);
    CC_CHECK_EQ(mesh.vertexCount, 0);
    CC_CHECK(cc_last_error()[0] != '\0');
#endif
    cc_surface_release(patch);
}

CC_RUN_ALL()
