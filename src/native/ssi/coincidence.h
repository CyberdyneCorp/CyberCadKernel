// SPDX-License-Identifier: Apache-2.0
//
// coincidence.h — the SSI Stage-S4-a TYPED coincident-region result.
//
// Two native surfaces can share more than an isolated intersection curve: they can be
// the SAME LOCUS (a plane laid on the same plane, two coaxial-equal cylinders, the same
// sphere, …) or they can COINCIDE OVER A SUB-REGION of their parameter domains (two
// freeform patches that agree on a sub-rectangle). Downstream queries and booleans need
// this typed as a REGION descriptor, not the bare `IntersectionStatus::Coincident` flag —
// so a boolean can trim the shared face, a query can report the overlapping band, etc.
//
// A `CoincidentRegion` is exactly one of:
//   * None            — the surfaces are NOT coincident (transversal / disjoint / a mere
//                        tangency — none of which is a shared 2D locus).
//   * FullSurfaceSame — the two surfaces are the same locus over their whole domains
//                        (decided in CLOSED FORM from the surface frames + sizes on the
//                        analytic path). `regionA`/`regionB` are left at their defaults
//                        (the full-surface case needs no sub-bounds).
//   * OverlapSubRegion— they coincide on a DELIMITED parameter sub-region; `regionA` and
//                        `regionB` carry the agreement bounds on EACH surface.
//   * Undecided       — coincidence is SUSPECTED but the region cannot be robustly
//                        delimited (partial/fuzzy boundary, ambiguous domain-edge touch).
//                        This is the honest "→ OCCT" outcome: the native layer does NOT
//                        fabricate a boundary; the ENGINE owns the OCCT fallback +
//                        self-verify. A correct `Undecided` is first-class, not a failure.
//
// SCOPE (S4-a — DETECTION + CLASSIFICATION only). This header carries the RESULT; the
// analytic FullSurfaceSame predicates live in dispatch.h (`classify_degeneracy`) and the
// seeded OverlapSubRegion detector lives in seeding.cpp (under CYBERCAD_HAS_NUMSCI). This
// layer never marches through a degeneracy and never fabricates a region.
//
// Header-only, OCCT-FREE, SUBSTRATE-FREE (no native-numerics). Uses src/native/math and
// patch_bounds.h (ParamBox) only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_COINCIDENCE_H
#define CYBERCAD_NATIVE_SSI_COINCIDENCE_H

#include "native/ssi/patch_bounds.h"  // ParamBox

namespace cybercad::native::ssi {

/// The four mutually-exclusive coincidence outcomes (see the file header).
enum class CoincidenceKind {
  None,             ///< not coincident (no shared 2D locus)
  FullSurfaceSame,  ///< same locus over the whole domains (analytic, closed form)
  OverlapSubRegion, ///< coincide on a delimited param sub-region (regionA/regionB bounds)
  Undecided,        ///< coincidence suspected but not robustly delimitable → OCCT
};

/// The typed shared-locus descriptor. For `OverlapSubRegion`, `regionA`/`regionB` are the
/// agreement bounds on surface A / surface B respectively; for the other kinds they are
/// unused (left at their `ParamBox` defaults). Pure data — OCCT-free, substrate-free.
struct CoincidentRegion {
  CoincidenceKind kind = CoincidenceKind::None;
  ParamBox regionA{};  ///< OverlapSubRegion: agreement bounds on surface A
  ParamBox regionB{};  ///< OverlapSubRegion: agreement bounds on surface B

  static CoincidentRegion none() { return {CoincidenceKind::None, {}, {}}; }
  static CoincidentRegion fullSurfaceSame() { return {CoincidenceKind::FullSurfaceSame, {}, {}}; }
  static CoincidentRegion overlap(const ParamBox& a, const ParamBox& b) {
    return {CoincidenceKind::OverlapSubRegion, a, b};
  }
  static CoincidentRegion undecided() { return {CoincidenceKind::Undecided, {}, {}}; }

  bool isCoincident() const noexcept {
    return kind == CoincidenceKind::FullSurfaceSame || kind == CoincidenceKind::OverlapSubRegion;
  }
};

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_COINCIDENCE_H
