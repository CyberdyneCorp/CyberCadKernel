// SPDX-License-Identifier: Apache-2.0
//
// native_drafting.h — umbrella for the native drafting service (MOAT M-GS GS1):
// orthographic hidden-line removal (HLR) and 2D drawing projection, built on the
// landed native topology + tessellator + math. OCCT-FREE. Header-only.
//
// Slices:
//   * orthographic_hlr.h — polyhedral/analytic-edge HLR over a triangle occluder
//     (visible/hidden classification with edge splitting at visibility
//     transitions).
//   * silhouette.h — closed-form QUADRIC-face silhouette tracing (cylinder
//     generators + sphere great circle) emitted as world polylines that feed the
//     SAME occlusion + split path. Cone / partial-quadric / torus / freeform
//     silhouettes are declined honestly and handled in a later slice.
//
#ifndef CYBERCAD_NATIVE_DRAFTING_NATIVE_DRAFTING_H
#define CYBERCAD_NATIVE_DRAFTING_NATIVE_DRAFTING_H

#include "native/drafting/orthographic_hlr.h"
#include "native/drafting/silhouette.h"

#endif  // CYBERCAD_NATIVE_DRAFTING_NATIVE_DRAFTING_H
