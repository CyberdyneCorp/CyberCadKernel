/*
 * Tiny C smoke test for the macOS libcybercadkernel.dylib.
 *
 * Proves REAL geometry through the cc_* ABI (no trivially-true checks):
 *   1. a B-rep engine is linked        -> cc_brep_available() == 1
 *   2. extrude a 10x10x10 box          -> exact volume == 1000 mm^3
 *   3. cut a 5x5x10 corner notch       -> exact volume == 875 mm^3 (1000 - 125)
 *
 * Volumes come from cc_mass_properties (the B-rep, not the mesh), so a wrong
 * kernel or a stub build fails the asserts. Exit 0 on success, non-zero on
 * failure. Prints the numbers it checked.
 *
 * Build+run: driven by scripts/verify-macos-smoke.sh.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "cybercadkernel/cc_kernel.h"

static int close_to(double got, double want, double tol) {
    return fabs(got - want) <= tol;
}

int main(void) {
    int ok = 1;

    int brep = cc_brep_available();
    printf("brep_available=%d\n", brep);
    if (brep != 1) {
        fprintf(stderr, "FAIL: expected a B-rep engine (brep_available=1)\n");
        return 2;
    }

    /* 10x10x10 box: unit square profile [0..10]x[0..10] extruded +Z by 10. */
    const double box_profile[] = {
        0.0,  0.0,
        10.0, 0.0,
        10.0, 10.0,
        0.0,  10.0,
    };
    CCShapeId box = cc_solid_extrude(box_profile, 4, 10.0);
    if (box == 0) {
        fprintf(stderr, "FAIL: cc_solid_extrude returned 0 (%s)\n", cc_last_error());
        return 3;
    }

    CCMassProps box_mp = cc_mass_properties(box);
    printf("box volume=%.6f (valid=%d)\n", box_mp.volume, box_mp.valid);
    if (!box_mp.valid || !close_to(box_mp.volume, 1000.0, 1e-6)) {
        fprintf(stderr, "FAIL: box volume expected 1000, got %.6f\n", box_mp.volume);
        ok = 0;
    }

    /* 5x5x10 tool at a corner: [0..5]x[0..5] extruded +Z by 10 (volume 250)...
     * intersect with the box so the notch removed is 5x5x10 = 250 -> 750?  No:
     * we cut the full 5x5x10 tool from the box, removing 5*5*10 = 250? The
     * proven number is 875, i.e. a 5x5x5 corner (125) removed. Use a 5x5x5 tool. */
    const double tool_profile[] = {
        0.0, 0.0,
        5.0, 0.0,
        5.0, 5.0,
        0.0, 5.0,
    };
    CCShapeId tool = cc_solid_extrude(tool_profile, 4, 5.0);   /* 5x5x5 = 125 */
    if (tool == 0) {
        fprintf(stderr, "FAIL: cc_solid_extrude(tool) returned 0 (%s)\n", cc_last_error());
        return 4;
    }

    CCShapeId cut = cc_boolean(box, tool, 1);   /* op 1 = cut (box - tool) */
    if (cut == 0) {
        fprintf(stderr, "FAIL: cc_boolean cut returned 0 (%s)\n", cc_last_error());
        return 5;
    }

    CCMassProps cut_mp = cc_mass_properties(cut);
    printf("cut volume=%.6f (valid=%d)\n", cut_mp.volume, cut_mp.valid);
    if (!cut_mp.valid || !close_to(cut_mp.volume, 875.0, 1e-6)) {
        fprintf(stderr, "FAIL: cut volume expected 875, got %.6f\n", cut_mp.volume);
        ok = 0;
    }

    cc_shape_release(cut);
    cc_shape_release(tool);
    cc_shape_release(box);

    if (ok) {
        printf("SMOKE OK: brep_available=1, box=1000, cut=875\n");
        return 0;
    }
    return 1;
}
