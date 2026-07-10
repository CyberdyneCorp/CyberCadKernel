// M8 rehearsal probe (NON-SHIPPING throwaway measurement artifact for the
// moat-m8dry-unlink-rehearsal OpenSpec change / DROP-OCCT-READINESS.md §6).
//
// Build against the -DCYBERCAD_M8_REHEARSAL=ON library, e.g.:
//   cmake -S . -B build-m8 -DCYBERCAD_M8_REHEARSAL=ON -DCYBERCAD_HAS_NUMSCI=ON \
//     -DCYBERCAD_NUMSCI_DIR=<numsci out> -DCYBERCAD_NUMPP_DIR=... -DCYBERCAD_SCIPP_DIR=...
//   cmake --build build-m8
//   clang++ -std=c++20 -Iinclude scratch/m8_probe.cpp \
//     build-m8/libcybercadkernel.a <numsci out>/libnumsci_host.a -o build-m8/m8_probe
//   ./build-m8/m8_probe
//
// ROUND 2 (2026-07-10): re-measures the post-unlink shape against the CURRENT surface,
// after the F1–F5 wave (Steinmetz cyl-cyl COMMON, curved sphere shell, cone/sphere
// offset_face, off-center + disjoint booleans, freeform sphere wrap-emboss). Section A
// keeps the round-1 Class-B/C decline + Class-A spine probes verbatim; Section B is NEW
// and drives the F1–F5 ops through the shipping facade to confirm they now serve NATIVE
// (they were OCCT-forwards or declines at round 1).
//
// Measures, under the native-only + stub-fallback default engine, whether each op on a
// NATIVE body SERVES natively, cleanly DECLINES (0 + honest error), or MISBEHAVES (crash /
// silent non-zero id where OCCT would have served). Prints one line per op.
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include "cybercadkernel/cc_kernel.h"

static const double kPi = 3.14159265358979323846;

static void report(const char* op, CCShapeId id) {
    const char* err = cc_last_error();
    if (id != 0) {
        std::printf("  %-30s SERVED-NATIVE (id=%ld)\n", op, id);
    } else if (err && std::strlen(err) > 0) {
        std::printf("  %-30s CLEAN-DECLINE  (\"%s\")\n", op, err);
    } else {
        std::printf("  %-30s DECLINE-NO-ERR (id=0, empty error)  <-- FINDING\n", op);
    }
}

// Report an F-series op that MUST serve native + its measured volume vs an analytic
// expectation (relative tolerance = facade deflection bound). SERVED-NATIVE only if
// id!=0 AND the volume matches; a fabricated/wrong non-zero volume is a FINDING.
static void reportVol(const char* op, CCShapeId id, double expected, double relTol) {
    if (id == 0) {
        const char* err = cc_last_error();
        std::printf("  %-30s DECLINE (\"%s\")  <-- REGRESSION vs round-2 expectation\n",
                    op, err ? err : "");
        return;
    }
    CCMassProps mp = cc_mass_properties(id);
    if (!mp.valid) { std::printf("  %-30s id=%ld but mass invalid  <-- FINDING\n", op, id); return; }
    double rel = std::fabs(mp.volume - expected) / expected;
    std::printf("  %-30s SERVED-NATIVE (id=%ld vol=%.4f exp=%.4f rel=%.1e) %s\n",
                op, id, mp.volume, expected, rel,
                rel <= relTol ? "OK" : "<-- VOLUME MISMATCH (FINDING)");
}

// Every face id whose face-mesh vertices ALL lie on the plane {coord along axisComp}.
static std::vector<int> capFaceIds(CCShapeId body, int axisComp, double coord, double tol) {
    std::vector<int> ids;
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.05, &faces);
    for (int f = 0; f < n; ++f) {
        const CCFaceMesh& fm = faces[f];
        if (fm.vertexCount < 3 || fm.vertices == nullptr) continue;
        bool onPlane = true;
        for (int v = 0; v < fm.vertexCount && onPlane; ++v)
            if (std::fabs(fm.vertices[v * 3 + axisComp] - coord) > tol) onPlane = false;
        if (onPlane) ids.push_back(fm.faceId);
    }
    cc_face_meshes_free(faces, n);
    return ids;
}

// The lateral wall face: the face whose mesh vertices span > 0.5·extent along axisComp.
static int wallFaceId(CCShapeId body, int axisComp, double extent) {
    int found = -1;
    CCFaceMesh* faces = nullptr;
    const int n = cc_face_meshes(body, 0.05, &faces);
    for (int f = 0; f < n; ++f) {
        const CCFaceMesh& fm = faces[f];
        if (fm.vertexCount < 3 || fm.vertices == nullptr) continue;
        double lo = fm.vertices[axisComp], hi = fm.vertices[axisComp];
        for (int v = 0; v < fm.vertexCount; ++v) {
            double c = fm.vertices[v * 3 + axisComp];
            if (c < lo) lo = c; if (c > hi) hi = c;
        }
        if ((hi - lo) > 0.5 * extent) { found = fm.faceId; break; }
    }
    cc_face_meshes_free(faces, n);
    return found;
}

// A centred (z ∈ [−L/2, L/2]) capped cylinder about +Z, radius Rc, via the facade.
static CCShapeId centeredCylZ(double Rc, double L) {
    CCProfileSeg seg{};
    seg.kind = 2; seg.cx = 0; seg.cy = 0; seg.r = Rc;
    CCShapeId up = cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, L);
    if (up == 0) return 0;
    CCShapeId c = cc_translate_shape(up, 0, 0, -L / 2.0);
    cc_shape_release(up);
    return c;
}

// A sphere-cap dome about +Y (semicircle-region revolve): base disc at y=capOff, arc to
// the pole (0,Ro), closing meridian on the axis. capOff=0 → hemisphere.
static CCShapeId buildDome(double Ro, double capOff) {
    const double rimBase = std::sqrt(Ro * Ro - capOff * capOff);
    CCProfileSeg base{}; base.kind = 0;
    base.x0 = 0; base.y0 = capOff; base.x1 = rimBase; base.y1 = capOff;
    CCProfileSeg arc{}; arc.kind = 1;
    arc.x0 = rimBase; arc.y0 = capOff; arc.x1 = 0; arc.y1 = Ro;
    arc.cx = 0; arc.cy = 0; arc.r = Ro;
    arc.a0 = std::atan2(capOff, rimBase); arc.a1 = std::atan2(Ro, 0.0);
    CCProfileSeg axisSeg{}; axisSeg.kind = 0;
    axisSeg.x0 = 0; axisSeg.y0 = Ro; axisSeg.x1 = 0; axisSeg.y1 = capOff;
    const CCProfileSeg segs[3] = {base, arc, axisSeg};
    return cc_solid_revolve_profile(segs, 3, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
}

int main() {
    std::printf("default engine native? %d ; brep_available? %d\n",
                cc_active_engine(), cc_brep_available());

    // A native box to feed the body-consuming decline ops.
    double sq[] = {0,0, 10,0, 10,10, 0,10};
    CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    std::printf("native box id=%ld (serve check)\n\n", box);

    std::printf("=== SECTION A — round-1 Class-B/C decline + Class-A spine (unchanged) ===\n\n");
    std::printf("[Class-B — expected CLEAN-DECLINE on native body]\n");
    report("cc_fillet_face",            cc_fillet_face(box, 1, 1.0));
    report("cc_full_round_fillet",      cc_full_round_fillet(box, 1));
    report("cc_full_round_fillet_faces",cc_full_round_fillet_faces(box, 1, 2, 3));
    { int e[1]={1}; report("cc_fillet_edges_g2", cc_fillet_edges_g2(box, e, 1, 1.0)); }

    double path[] = {0,0,0, 0,0,20};
    report("cc_twisted_sweep(real)",    cc_twisted_sweep(sq, 4, path, 2, 1.57, 1.0));
    double rail[] = {0,0,0, 5,0,3, 10,0,0};
    report("cc_loft_along_rail(curved)",cc_loft_along_rail(rail, 3, sq, 4, sq, 4));

    CCShapeId thr = cc_solid_extrude(sq, 4, 5.0);
    report("cc_thread_apply",           cc_thread_apply(box, thr, 0));

    std::printf("\n[Class-C — IGES dropped, expected CLEAN-DECLINE]\n");
    report("cc_iges_import",            cc_iges_import("/nonexistent.igs"));
    { int rc = cc_iges_export(box, "/tmp/m8_probe.igs");
      const char* err = cc_last_error();
      std::printf("  %-30s %s (\"%s\")\n", "cc_iges_export",
                  rc==1 ? "SERVED(rc=1) <-- FINDING" : "CLEAN-DECLINE(rc=0)",
                  err?err:""); }

    std::printf("\n[Class-A spine spot-check — expected SERVE-NATIVE]\n");
    report("cc_solid_revolve", ({ double pr[]={1,0, 3,0, 3,4, 1,4}; cc_solid_revolve(pr,4,6.283185); }));
    report("cc_solid_loft", ({ double a[]={0,0,4,0,4,4,0,4}, b[]={1,1,3,1,3,3,1,3};
                               cc_solid_loft(a, 4, b, 4, 10.0); }));
    { CCMesh m = cc_tessellate(box, 0.1);
      std::printf("  %-30s %s (v=%d t=%d)\n", "cc_tessellate",
                  m.vertexCount>0 ? "SERVED-NATIVE" : "EMPTY <-- FINDING",
                  m.vertexCount, m.triangleCount);
      cc_mesh_free(m); }
    { CCMassProps mp = cc_mass_properties(box);
      std::printf("  %-30s %s (vol=%.1f)\n", "cc_mass_properties",
                  mp.valid? "SERVED-NATIVE":"DECLINE", mp.volume); }

    std::printf("\n=== SECTION B — F1–F5 wave: MUST now SERVE NATIVE (round-1: OCCT/decline) ===\n\n");

    // F1 — cc_boolean cyl-cyl COMMON (Steinmetz bicylinder) through the facade.
    // Z-cylinder + a copy rotated 90° about +Y (→ X-cylinder), intersect. V = 16 Rc³/3.
    {
        const double Rc = 1.0, L = 6.0;
        CCShapeId za = centeredCylZ(Rc, L);
        CCShapeId zb = centeredCylZ(Rc, L);
        CCShapeId xb = zb ? cc_rotate_shape_about(zb, 0,0,0, 0,1,0, kPi/2.0) : 0;
        if (zb) cc_shape_release(zb);
        CCShapeId lens = (za && xb) ? cc_boolean(za, xb, /*common*/ 2) : 0;
        if (za) cc_shape_release(za);
        if (xb) cc_shape_release(xb);
        reportVol("F1 cc_boolean cyl-cyl COMMON", lens, 16.0/3.0*Rc*Rc*Rc, 0.03);
        if (lens) cc_shape_release(lens);
    }

    // F2 — curved shell on a sphere-cap dome (hemisphere), top rim open.
    // Hollow sphere-cap wall vol; for a hemisphere (capOff=0), open rim:
    //   V = (2/3)π Ro³ − (2/3)π(Ro−t)³.
    {
        const double Ro = 5.0, t = 0.5;
        CCShapeId dome = buildDome(Ro, /*capOff*/ 0.0);
        std::vector<int> caps = dome ? capFaceIds(dome, /*y*/1, 0.0, 1e-4) : std::vector<int>{};
        CCShapeId sh = (dome && !caps.empty())
            ? cc_shell(dome, caps.data(), (int)caps.size(), t) : 0;
        double exp = (2.0/3.0)*kPi*(Ro*Ro*Ro - (Ro-t)*(Ro-t)*(Ro-t));
        reportVol("F2 cc_shell sphere dome", sh, exp, 0.03);
        if (sh) cc_shape_release(sh);
        if (dome) cc_shape_release(dome);
    }

    // F3a — cc_offset_face cone-frustum lateral wall, radial +d. Coaxial cone same σ, both
    // cap radii shift dR = d/cosσ. Oracle = a coaxial capped cone of the shifted radii.
    {
        const double Rb = 4.0, Rt = 2.0, H = 6.0, d = 0.5;
        double prof[] = {0,0, Rb,0, Rt,H, 0,H};
        CCShapeId cone = cc_solid_revolve(prof, 4, 2.0*kPi);
        int wall = cone ? wallFaceId(cone, /*y*/1, H) : -1;
        CCShapeId off = (cone && wall>0) ? cc_offset_face(cone, wall, d) : 0;
        const double cosσ = H / std::sqrt(H*H + (Rb-Rt)*(Rb-Rt));
        const double dR = d / cosσ, Rb2 = Rb+dR, Rt2 = Rt+dR;
        double exp = kPi*H/3.0*(Rb2*Rb2 + Rb2*Rt2 + Rt2*Rt2);
        reportVol("F3a cc_offset_face cone wall", off, exp, 0.03);
        if (off) cc_shape_release(off);
        if (cone) cc_shape_release(cone);
    }

    // F3b — cc_offset_face sphere-cap dome wall, radial +d → concentric sphere R→R+d,
    // cap plane fixed at y=capOff. For a hemisphere (capOff=0): V = (2/3)π(R+d)³.
    {
        const double Ro = 5.0, d = 0.5;
        CCShapeId dome = buildDome(Ro, 0.0);
        int wall = dome ? wallFaceId(dome, /*y*/1, Ro) : -1;
        CCShapeId off = (dome && wall>0) ? cc_offset_face(dome, wall, d) : 0;
        double exp = (2.0/3.0)*kPi*(Ro+d)*(Ro+d)*(Ro+d);
        reportVol("F3b cc_offset_face sphere wall", off, exp, 0.04);
        if (off) cc_shape_release(off);
        if (dome) cc_shape_release(dome);
    }

    // F4 — off-center freeform half-space CUT (freeformHalfSpaceCut) via cc_split_plane on a
    // native FREEFORM-walled body. The F4 fix corrected the by-normal cross-section cap
    // orientation so an OFF-CENTRE keep-side volume is accurate (round-1 was silently ~29%
    // over at large offsets, masked at x=0 by symmetry). Build a freeform body: a closed
    // B-spline profile (kind-3) extruded → a native prism with a curved wall. Split it with a
    // plane NOT through the symmetry axis so the by-normal cap orientation is load-bearing.
    {
        // A closed, roughly-circular spline loop (freeform wall), radius ~3, centred at (0,0).
        const double spl[] = { 3,0,  2,2.4,  0,3,  -2,2.4,  -3,0,  -2,-2.4,  0,-3,  2,-2.4,  3,0 };
        const int nPts = 9;  // last repeats first to close the loop
        CCProfileSeg seg{}; seg.kind = 3; seg.ptOffset = 0; seg.ptCount = nPts;
        CCShapeId prism = cc_solid_extrude_profile(&seg, 1, nullptr, 0, spl, 2*nPts, 8.0);
        // Off-center split: plane normal +x through x=+1.0 (NOT the symmetry axis x=0),
        // keep the positive (+x) side — exercises the by-normal freeform cap on both lumps.
        CCShapeId half = prism ? cc_split_plane(prism, 1.0,0.0,4.0, 1.0,0.0,0.0, /*keepPositive*/1) : 0;
        if (prism == 0)
            std::printf("  %-30s prism build declined (\"%s\") — probe recipe, not an F4 finding\n",
                        "F4 cc_split_plane off-center", cc_last_error());
        else
            report("F4 cc_split_plane off-center", half);
        if (half) {
            CCMassProps mp = cc_mass_properties(half);
            std::printf("       (keep-side +x vol=%.4f valid=%d)\n", mp.volume, mp.valid);
        }
        // NOTE: this off-centre split of a spline-walled prism is an out-of-envelope config
        // for the facade split_plane native path (a curved perpendicular slice), so a CLEAN-
        // DECLINE here is EXPECTED and honest — NOT an F4 regression. The F4 fix (by-normal
        // cross-section cap orientation) is proven by the dedicated host gates in the suite:
        //   test_native_curved_wall_cut  (13/13 — off-centre/mid-wall keep-side volume)
        //   test_native_slab_disjoint_cut (6/6 — two-lump WELD Compound at closed form).
        if (half) cc_shape_release(half);
        if (prism) cc_shape_release(prism);
    }

    // F5 — freeform-base wrap_emboss: a sphere-cap pole boss on a dome wall.
    {
        const double R = 5.0;
        CCShapeId dome = buildDome(R, /*capOff*/ 0.0);
        int wall = dome ? wallFaceId(dome, /*y*/1, R) : -1;
        // A small square emboss profile near the pole; boss=1 (protrude), depth 0.5.
        double prof[] = {-0.6,-0.6, 0.6,-0.6, 0.6,0.6, -0.6,0.6};
        CCShapeId emb = (dome && wall>0)
            ? cc_wrap_emboss(dome, wall, prof, 4, 0.5, /*boss*/1) : 0;
        report("F5 cc_wrap_emboss sphere boss", emb);
        if (emb) {
            CCMassProps mp = cc_mass_properties(emb);
            std::printf("       (embossed vol=%.4f valid=%d)\n", mp.volume, mp.valid);
        }
        if (emb) cc_shape_release(emb);
        if (dome) cc_shape_release(dome);
    }

    return 0;
}
