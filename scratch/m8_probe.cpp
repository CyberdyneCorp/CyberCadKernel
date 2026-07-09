// M8 rehearsal probe (NON-SHIPPING throwaway measurement artifact for the
// moat-m8dry-unlink-rehearsal OpenSpec change / DROP-OCCT-READINESS.md §6).
//
// Build against the -DCYBERCAD_M8_REHEARSAL=ON library, e.g.:
//   cmake -S . -B build-m8 -DCYBERCAD_M8_REHEARSAL=ON -DCYBERCAD_HAS_NUMSCI=ON \
//     -DCYBERCAD_NUMSCI_DIR=<numsci out> -DCYBERCAD_NUMPP_DIR=... -DCYBERCAD_SCIPP_DIR=...
//   cmake --build build-m8
//   clang++ -std=c++20 -stdlib=libc++ -Iinclude scratch/m8_probe.cpp \
//     build-m8/libcybercadkernel.a <numsci out>/libnumsci_host.a -o build-m8/m8_probe
//   ./build-m8/m8_probe
//
// Measures, under the native-only +
// stub-fallback default engine, whether each Class-B / Class-C op on a NATIVE body
// SERVES natively, cleanly DECLINES (0 + honest error), or MISBEHAVES (crash / silent
// non-zero id where OCCT would have served). Prints one line per op.
#include <cstdio>
#include <cstring>
#include "cybercadkernel/cc_kernel.h"

static void report(const char* op, CCShapeId id) {
    const char* err = cc_last_error();
    if (id != 0) {
        std::printf("  %-28s SERVED-NATIVE (id=%u)\n", op, id);
    } else if (err && std::strlen(err) > 0) {
        std::printf("  %-28s CLEAN-DECLINE  (\"%s\")\n", op, err);
    } else {
        std::printf("  %-28s DECLINE-NO-ERR (id=0, empty error)  <-- FINDING\n", op);
    }
}

int main() {
    std::printf("default engine native? %d ; brep_available? %d\n",
                cc_active_engine(), cc_brep_available());

    // A native box to feed the body-consuming decline ops.
    double sq[] = {0,0, 10,0, 10,10, 0,10};
    CCShapeId box = cc_solid_extrude(sq, 4, 10.0);
    std::printf("native box id=%u (serve check)\n\n", box);

    std::printf("[Class-B — expected CLEAN-DECLINE on native body]\n");
    report("cc_fillet_face",            cc_fillet_face(box, 1, 1.0));
    report("cc_full_round_fillet",      cc_full_round_fillet(box, 1));
    report("cc_full_round_fillet_faces",cc_full_round_fillet_faces(box, 1, 2, 3));
    { int e[1]={1}; report("cc_fillet_edges_g2", cc_fillet_edges_g2(box, e, 1, 1.0)); }

    // twisted_sweep with REAL twist (the B slice: native covers twist~0 only)
    double path[] = {0,0,0, 0,0,20};
    report("cc_twisted_sweep(real)",    cc_twisted_sweep(sq, 4, path, 2, 1.57, 1.0));
    // loft_along_rail with a CURVED rail (B slice: native covers straight rail only)
    double rail[] = {0,0,0, 5,0,3, 10,0,0};
    report("cc_loft_along_rail(curved)",cc_loft_along_rail(rail, 3, sq, 4, sq, 4));

    CCShapeId thr = cc_solid_extrude(sq, 4, 5.0);
    report("cc_thread_apply",           cc_thread_apply(box, thr, 0));

    std::printf("\n[Class-C — IGES dropped, expected CLEAN-DECLINE]\n");
    report("cc_iges_import",            cc_iges_import("/nonexistent.igs"));
    { int rc = cc_iges_export(box, "/tmp/m8_probe.igs");
      const char* err = cc_last_error();
      std::printf("  %-28s %s (\"%s\")\n", "cc_iges_export",
                  rc==1 ? "SERVED(rc=1) <-- FINDING" : "CLEAN-DECLINE(rc=0)",
                  err?err:""); }

    std::printf("\n[Class-A spine spot-check — expected SERVE-NATIVE]\n");
    report("cc_solid_revolve", ({ double pr[]={1,0, 3,0, 3,4, 1,4}; cc_solid_revolve(pr,4,6.283185); }));
    report("cc_solid_loft", ({ double a[]={0,0,4,0,4,4,0,4}, b[]={1,1,3,1,3,3,1,3};
                               cc_solid_loft(a, 4, b, 4, 10.0); }));
    { CCMesh m = cc_tessellate(box, 0.1);
      std::printf("  %-28s %s (v=%d t=%d)\n", "cc_tessellate",
                  m.vertexCount>0 ? "SERVED-NATIVE" : "EMPTY <-- FINDING",
                  m.vertexCount, m.triangleCount);
      cc_mesh_free(m); }
    { CCMassProps mp = cc_mass_properties(box);
      std::printf("  %-28s %s (vol=%.1f)\n", "cc_mass_properties",
                  mp.valid? "SERVED-NATIVE":"DECLINE", mp.volume); }
    return 0;
}
