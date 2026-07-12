/* C-compiler cross-check of the NURBS ABI struct sizes.
 *
 * This translation unit is compiled by the C compiler (NOT C++) against the same
 * public header, so the sizes the C++ test asserts are checked against what a
 * plain-C consumer of the ABI sees — the FFI/ctypes / KernelBridgeAPI world. It
 * both static_asserts the sizes here and exports them as extern symbols the C++
 * test compares (mirrors the add-python-binding §8.2 / test_ffi sizeof guard). */

#include <stddef.h>
#include <stdint.h>

#define CC_KERNEL_NO_PROTOTYPES
#include "cybercadkernel/cc_kernel.h"
#undef CC_KERNEL_NO_PROTOTYPES

/* C11 static assertions: the layout a C consumer sees is fixed. */
_Static_assert(sizeof(cc_curve) == 4, "cc_curve must be a 4-byte handle in C");
_Static_assert(sizeof(cc_surface) == 4, "cc_surface must be a 4-byte handle in C");
_Static_assert(sizeof(CCCurveInfo) == 16, "CCCurveInfo layout drift (C)");
_Static_assert(sizeof(CCSurfaceInfo) == 28, "CCSurfaceInfo layout drift (C)");
_Static_assert(sizeof(CCTessOptions) == 8, "CCTessOptions layout drift (C)");

/* Exported for the C++ test to compare against its own C++ sizeof. */
const size_t cc_nurbs_c_sizeof_curve = sizeof(cc_curve);
const size_t cc_nurbs_c_sizeof_surface = sizeof(cc_surface);
const size_t cc_nurbs_c_sizeof_curveinfo = sizeof(CCCurveInfo);
const size_t cc_nurbs_c_sizeof_surfaceinfo = sizeof(CCSurfaceInfo);
const size_t cc_nurbs_c_sizeof_tessoptions = sizeof(CCTessOptions);
