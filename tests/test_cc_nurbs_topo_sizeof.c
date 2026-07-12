/* C-compiler cross-check of the J5 NURBS-topology ABI struct sizes.
 *
 * Compiled by the C compiler (NOT C++) against the same public header, so the
 * sizes the C++ test asserts are checked against what a plain-C consumer of the
 * ABI sees (the FFI/ctypes world). Mirrors test_cc_nurbs_sizeof.c. */

#include <stddef.h>
#include <stdint.h>

#define CC_KERNEL_NO_PROTOTYPES
#include "cybercadkernel/cc_kernel.h"
#undef CC_KERNEL_NO_PROTOTYPES

/* C11 static assertions: the layout a C consumer sees is fixed.
 * CCCurveHit        = double xyz[3] + tA + tB + int32 flag  → 44 → pad 48
 * CCCurveSurfaceHit = double xyz[3] + t + u + v + int32 flag → 52 → pad 56
 * CCTrimLoop        = double* + int32 + int32 + double       → 24            */
_Static_assert(sizeof(CCCurveHit) == 48, "CCCurveHit layout drift (C)");
_Static_assert(sizeof(CCCurveSurfaceHit) == 56, "CCCurveSurfaceHit layout drift (C)");
_Static_assert(sizeof(CCTrimLoop) == 24, "CCTrimLoop layout drift (C)");

/* Exported for the C++ test to compare against its own C++ sizeof. */
const size_t cc_nurbs_topo_c_sizeof_curvehit = sizeof(CCCurveHit);
const size_t cc_nurbs_topo_c_sizeof_cshit = sizeof(CCCurveSurfaceHit);
const size_t cc_nurbs_topo_c_sizeof_trimloop = sizeof(CCTrimLoop);
