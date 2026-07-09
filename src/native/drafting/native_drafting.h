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
//   * silhouette.h — closed-form ANALYTIC-face silhouette tracing (cylinder
//     generators + sphere great circle + cone/frustum contour generators + torus
//     turning contours) emitted as world polylines that feed the SAME occlusion +
//     split path. Partial-quadric and freeform (B-spline/Bézier) silhouettes are
//     declined honestly. A native REVOLVE builds a torus as rational-B-spline
//     bands (not a Kind::Torus face), so a revolve-built torus still declines;
//     a Kind::Torus face (STEP-imported) is traced.
//
#ifndef CYBERCAD_NATIVE_DRAFTING_NATIVE_DRAFTING_H
#define CYBERCAD_NATIVE_DRAFTING_NATIVE_DRAFTING_H

#include "native/drafting/orthographic_hlr.h"
#include "native/drafting/silhouette.h"

#endif  // CYBERCAD_NATIVE_DRAFTING_NATIVE_DRAFTING_H
