// SPDX-License-Identifier: Apache-2.0
//
// native_boolean_fwd.h — the shared boolean Op enum, factored out so the SSI-driven
// S5-a path (ssi_boolean.h) can name it without pulling in the whole planar/analytic
// aggregate (native_boolean.h includes ssi_boolean.h, so ssi_boolean.h cannot include
// native_boolean.h in turn). Header-only, OCCT-FREE.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_FWD_H
#define CYBERCAD_NATIVE_BOOLEAN_FWD_H

namespace cybercad::native::boolean {

/// The three set operations, matching the cc_boolean op codes (0 fuse, 1 cut a−b,
/// 2 common).
enum class Op { Fuse = 0, Cut = 1, Common = 2 };

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_FWD_H
