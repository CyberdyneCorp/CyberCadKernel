// SPDX-License-Identifier: Apache-2.0
//
// nurbs_freeform_split.h — NURBS roadmap LAYER 3, SLICE 3 (L3-S3): the THIRD (and
// deepest) exact-NURBS B-rep boolean — a genuine NURBS face SPLIT BY ANOTHER
// FREEFORM NURBS face, welded watertight. BOTH operands are arbitrary NURBS (not
// analytic): the general freeform↔freeform sew the readiness doc named as the
// stage-5 deep-tail wall.
//
// ── ROLE (the person-decade moat core: face∩FREEFORM-face) ──────────────────────
// L3-S1 (`nurbs_plane_split.h`) cut a NURBS wall by a PLANE (sew = curved-NURBS↔FLAT);
// L3-S2 (`nurbs_curved_split.h`) cut a NURBS wall by an ANALYTIC CURVED cutter
// (Cylinder/Sphere/Cone — sew = curved-NURBS↔analytic-CURVED, a closed-form fan on the
// true cutter surface). L3-S3 removes the last analytic crutch: the cutter G is now a
// genuine FREEFORM NURBS face, so the kept-G cap is itself a curved NURBS sub-face (no
// closed-form fan). The sew is NURBS-disk↔NURBS-disk along a shared curved seam, and
// BOTH sub-faces must weld bit-for-bit watertight along that seam — the general M0
// curved↔curved weld.
//
// The observation that makes this reachable: `boolean/freeform_freeform_cut.h`
// (`freeformFreeformClosedSeamCut`) ALREADY performs the freeform↔freeform curved↔curved
// closed-seam weld — split BOTH walls along the shared 3-D seam via `splitFaceSmoothTrim`
// (outer ring = the EXACT shared seam nodes on both sides ⇒ bit-identical chords ⇒ the
// M0 mesher position-welds them), select survivors by mesh membership, then repair
// orientation coherence (the directed-edge invariant) so the two curved caps form a
// coherent outward-normal boundary. It is proven watertight at the closed-form lens
// volume (`test_native_freeform_freeform_cut`, the COMMON leg) — BUT ONLY for
// `Kind::Bezier` walls: `ffcdetail::freeformWall` rejects any non-Bézier surface, and
// `ffcdetail::traceSharedSeam` hardcodes `makeBezierAdapter`. L3-S3 is that SAME verb
// with both walls left as genuine `Kind::BSpline`/NURBS surfaces, and the seam traced
// through the NURBS operand front-end:
//
//   1. RECOGNISE — B1 `recogniseFreeformSolid` admits both operands. It ALREADY tags
//      `Kind::BSpline` faces as `FaceRole::Freeform` (`fodetail::classifyFaceRole`), so
//      a genuine-NURBS-walled cup admits unchanged. This slice additionally requires the
//      one freeform wall of EACH operand to be `Kind::BSpline` (NURBS, not Bézier) — the
//      Bézier↔Bézier case is `freeform_freeform_cut.h`'s; L3-S3 owns the NURBS↔NURBS one.
//   2. TRACE (stage 1, WORKS-transversal/closed-loop) — `F.wall ∩ G.wall` → the CLOSED
//      interior seam WLine, via `npsdetail::makeWallAdapter` (the L3 NURBS operand
//      front-end: BSpline→`makeBSplineAdapter`, rational→`makeNurbsAdapter`) on BOTH
//      walls. The WLine node carries `(u1,v1)` on F's wall AND `(u2,v2)` on G's wall.
//   3. FIDELITY (stage 2, WLine-(u,v)-read) — the seam pcurve is READ from the WLine:
//      `(u1,v1)` on F, `(u2,v2)` on G. BOTH are round-trip-gated — `S_F(u1,v1)==node`
//      AND `S_G(u2,v2)==node` (the S(pcurve)==C invariant, evaluated on BOTH NURBS
//      surfaces via `tess::SurfaceEvaluator`) — so a drifted seam on EITHER freeform
//      operand is REJECTED, never welded. NO general `constructPcurve`.
//   4. SPLIT (stage 3, WORKS-closed-seam) — `splitFaceSmoothTrim` partitions F's NURBS
//      wall by `(u1,v1)` and G's NURBS wall by a `(u2,v2)`-re-keyed copy of the SAME
//      seam, each into a disk + annulus (surface-kind agnostic — proven on NURBS grids
//      in L3-S1/S2).
//   5. SELECT + SEW (stage 4/5, the WALL — curved-NURBS↔curved-NURBS) — survivors by B3
//      `classifyPointInMesh` membership of each sub-face centroid in the OTHER operand's
//      pre-cut mesh (COMMON = F's disk INSIDE G ∪ G's disk INSIDE F — the lens). The two
//      NURBS caps are welded via the orientation-coherence repair (`weldOrientationCoherent`
//      — the directed-edge invariant, exactly one cap reversed), then the mandatory M0
//      watertight (χ=2) + positive-enclosed-volume self-verify. The two caps share the
//      EXACT seam nodes (`splitFaceSmoothTrim`'s bit-identical outer ring on both sides),
//      so the M0 mesher position-welds NURBS-disk↔NURBS-disk watertight — the general
//      freeform↔freeform sew, resolved for the tractable single-transversal-seam pose.
//
// ── SCOPE + HONESTY (the moat discipline: a bounded slice beats a shaky general one) ──
// This slice sews the COMMON (`F ∩ G`) lens of TWO genuine-NURBS bowl-cup operands whose
// curved NURBS walls meet in ONE CLOSED interior seam. Anything outside that envelope is
// an HONEST DECLINE (a measured `NurbsFreeformSplitDecline`, NULL result): an operand B1
// declines, a wall that is not `Kind::BSpline` (a Bézier wall → `freeform_freeform_cut.h`;
// an analytic wall → L3-S2), a boundary-crossing / open / non-closed seam, a drifted seam
// that fails the fidelity gate on EITHER operand, a split that declines, an ambiguous /
// on-boundary membership, or a non-watertight / orientation-incoherent / non-positive-
// volume weld. NO tolerance is weakened; the self-verify is the M0 mesh-level watertight
// + orientation-coherent + volume one (two-sided against the closed form when supplied).
// The CUT (`F − G`) leg's SURVIVOR MEMBERSHIP — which the pre-fix code (and the Bézier
// `freeform_freeform_cut.h`) declined at because the annulus's OUTER-loop UV centroid lands
// on the pole (a fragile apex sample that cannot separate annulus-Out from disk-In) — is now
// resolved HONESTLY by a robust WINDING/RAY test over points GENUINELY INTERIOR to each
// sub-region in (u,v) (`nfsdetail::pickRegionRobust`), tie-broken by coherence, with a near-
// pole/tangent HONEST DECLINE (never a widened tolerance). Its RESIDUAL blocker is the frozen
// M0 mesher: it does not position-weld a HOLED curved annulus (`faceOutside`, seam-as-hole) to
// the curved disk (`faceInside`, seam-as-outer) across the shared closed seam — a mesher gap,
// NOT a membership one. So the CUT leg reports a MEASURED residual (`weldOpenEdges`,
// `cutMembershipResolved`) and HONEST-DECLINES (`NotWatertight`) rather than emit a leaky
// solid; it is NOT re-enabled until the frozen mesher welds that pairing. Multi-crossing /
// re-entrant splits and closed-loop-seeding-missed seams likewise stay DEFERRED.
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// L3-S1 `npsdetail::makeWallAdapter` (`nurbs_plane_split.h` — the NURBS operand SSI
// front-end), B1 `recogniseFreeformSolid` (`freeform_operand.h`), M2 freeform↔freeform
// weld helpers `ffcdetail::{rekeyToB, pickByMembership, collectAnalyticByMembership,
// weldOrientationCoherent}` (`freeform_freeform_cut.h` — surface-kind agnostic), B2
// SMOOTH-TRIM `splitFaceSmoothTrim` (`smooth_trim_split.h`), B3 `classifyPointInMesh` /
// `meshAabb` (`freeform_membership.h`), M1 `ssi::trace_intersection`, M0
// `SolidMesher`/`isWatertight`/`enclosedVolume`, the tessellate evaluators. Additive
// sibling — touches NONE of them, modifies `nurbs_plane_split.h` / `nurbs_curved_split.h`
// / `freeform_freeform_cut.h` / `ssi_boolean.{h,cpp}` / `assemble.h` / `face_split.h` /
// `ssi` / `topology` / `math` NOWHERE.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20. Substrate-gated
// (CYBERCAD_HAS_NUMSCI) because the seam is a real S3 trace, like L3-S1/S2.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_NURBS_FREEFORM_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_NURBS_FREEFORM_SPLIT_H

#include "native/boolean/freeform_freeform_cut.h"  // FfOp, ffcdetail::{rekeyToB, pickByMembership, collectAnalyticByMembership, weldOrientationCoherent}
#include "native/boolean/freeform_membership.h"    // classifyPointInMesh, meshAabb, Aabb, Membership
#include "native/boolean/freeform_operand.h"       // recogniseFreeformSolid, FreeformOperand
#include "native/boolean/nurbs_plane_split.h"      // npsdetail::makeWallAdapter, seamFidelity
#include "native/boolean/smooth_trim_split.h"      // splitFaceSmoothTrim (B2 smooth-trim)
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"                // UVRegion, buildRegion (interior-UV probe)
#include "native/topology/accessors.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

#if defined(CYBERCAD_HAS_NUMSCI)

// ─────────────────────────────────────────────────────────────────────────────
// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
// watertight keep-side solid is returned. NEVER a leaky / partial / wrong solid.
// ─────────────────────────────────────────────────────────────────────────────
enum class NurbsFreeformSplitDecline {
  Ok,
  NotAdmittedF,          ///< B1 declined operand F
  NotAdmittedG,          ///< B1 declined operand G
  WallFNotNurbs,         ///< F does not present EXACTLY one Kind::BSpline (NURBS) freeform wall
  WallGNotNurbs,         ///< G does not present EXACTLY one Kind::BSpline (NURBS) freeform wall
  SeamUnusable,          ///< M1 seam missing / < 3 nodes / not a closed interior loop
  SeamOffSurface,        ///< the WLine (u,v) pcurve does NOT round-trip on F or G (drifted seam)
  SmoothSplitFailedF,    ///< B2 smooth-trim declined F's wall split
  SmoothSplitFailedG,    ///< B2 smooth-trim declined G's wall split
  ClassifyAmbiguous,     ///< B3: a survivor sub-face is On/Unknown or on the wrong side
  WeldOpen,              ///< fewer than two survivor faces (cannot bound a solid)
  NotWatertight,         ///< self-verify: the welded result is not a closed / coherent 2-manifold
  VolumeInconsistent     ///< self-verify: the enclosed volume is non-positive / off the bound
};

inline const char* nurbsFreeformSplitDeclineName(NurbsFreeformSplitDecline d) noexcept {
  switch (d) {
    case NurbsFreeformSplitDecline::Ok: return "Ok";
    case NurbsFreeformSplitDecline::NotAdmittedF: return "NotAdmittedF";
    case NurbsFreeformSplitDecline::NotAdmittedG: return "NotAdmittedG";
    case NurbsFreeformSplitDecline::WallFNotNurbs: return "WallFNotNurbs";
    case NurbsFreeformSplitDecline::WallGNotNurbs: return "WallGNotNurbs";
    case NurbsFreeformSplitDecline::SeamUnusable: return "SeamUnusable";
    case NurbsFreeformSplitDecline::SeamOffSurface: return "SeamOffSurface";
    case NurbsFreeformSplitDecline::SmoothSplitFailedF: return "SmoothSplitFailedF";
    case NurbsFreeformSplitDecline::SmoothSplitFailedG: return "SmoothSplitFailedG";
    case NurbsFreeformSplitDecline::ClassifyAmbiguous: return "ClassifyAmbiguous";
    case NurbsFreeformSplitDecline::WeldOpen: return "WeldOpen";
    case NurbsFreeformSplitDecline::NotWatertight: return "NotWatertight";
    case NurbsFreeformSplitDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

// The honest witnesses of an L3-S3 split — the seam lies on BOTH freeform NURBS walls
// (fidelity), each wall tiles under the split, and the curved↔curved weld is watertight.
struct NurbsFreeformSplitResult {
  topo::Shape solid;                ///< the welded keep-side Solid (null on decline)
  NurbsFreeformSplitDecline decline = NurbsFreeformSplitDecline::Ok;

  double seamFidelityF = 0.0;       ///< max ‖S_F(u1,v1) − node.point‖ (S(pcurve)==C on F's NURBS wall)
  double seamFidelityG = 0.0;       ///< max ‖S_G(u2,v2) − node.point‖ (S(pcurve)==C on G's NURBS wall)
  double seamOnSurf = 0.0;          ///< max WLine node onSurfResidual (on BOTH F and G)
  int seamNodes = 0;                ///< closed-seam node count
  double areaInsideF = 0.0;         ///< UV area of F's enclosed disk sub-face
  double areaInsideG = 0.0;         ///< UV area of G's enclosed disk sub-face
  double tilingGapF = 0.0;          ///< |parent − (in+out)| on F (smooth-trim tiling residual)
  double tilingGapG = 0.0;          ///< |parent − (in+out)| on G
  bool watertight = false;          ///< M0 self-verify: the welded result is a closed 2-manifold
  double enclosedVolume = 0.0;      ///< M0 self-verify: signed-tetra enclosed volume of the keep solid

  // ── CUT-leg residual map (populated when the CUT membership RESOLVES but the weld is
  // the blocker). The robust interior-UV winding test separates annulus-Out from disk-In
  // (`cutMembershipResolved`), so the CUT decline — when it happens — is a MEASURED weld
  // residual (`weldOpenEdges` boundary edges the frozen M0 mesher left unpaired welding a
  // holed curved annulus to a curved disk across the shared seam), NOT the apex-membership
  // ambiguity the pre-fix code declined at. This is the residual map the honest CUT decline
  // reports; COMMON leaves these at their defaults.
  bool cutMembershipResolved = false;  ///< CUT: annulus/disk sub-regions robustly classified
  int weldOpenEdges = 0;               ///< CUT: unpaired boundary edges of the attempted weld (0 ⇒ watertight)

  bool ok() const noexcept { return decline == NurbsFreeformSplitDecline::Ok && !solid.isNull(); }
};

namespace nfsdetail {

// The single freeform wall of a recognised operand, REQUIRED to be a genuine NURBS
// (`Kind::BSpline`) surface with poles. Returns the OperandFace + its FaceSurface, or
// nullopt (with `why`) if the operand does not present exactly one usable NURBS wall.
// (The NURBS analog of `ffcdetail::freeformWall`, which requires `Kind::Bezier`.)
inline const topo::FaceSurface* nurbsWall(const FreeformOperand& op, const OperandFace** face,
                                          NurbsFreeformSplitDecline notNurbs,
                                          NurbsFreeformSplitDecline& why) {
  if (op.freeform.size() != 1) { why = notNurbs; return nullptr; }
  const OperandFace& wall = op.faces[op.freeform.front()];
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) { why = notNurbs; return nullptr; }
  const topo::FaceSurface& fs = *srf->surface;
  const bool isNurbs = (fs.kind == topo::FaceSurface::Kind::BSpline) && !fs.poles.empty() &&
                       fs.nPolesU >= 2 && fs.nPolesV >= 2;
  if (!isNurbs) { why = notNurbs; return nullptr; }
  *face = &wall;
  return &fs;
}

// Trace the shared seam `F.wall ∩ G.wall` as a CLOSED WLine, keyed with `(u1,v1)` on F
// and `(u2,v2)` on G — via the NURBS operand front-end (`makeWallAdapter`) on BOTH walls.
// Returns the single closed branch, or an empty WLine. (The NURBS analog of
// `ffcdetail::traceSharedSeam`, which uses `makeBezierAdapter`.)
inline ssi::WLine traceNurbsSharedSeam(const topo::FaceSurface& fsF, const topo::FaceSurface& fsG) {
  const ssi::SurfaceAdapter A = npsdetail::makeWallAdapter(fsF);
  const ssi::SurfaceAdapter B = npsdetail::makeWallAdapter(fsG);
  const ssi::TraceSet tr = ssi::trace_intersection(A, B);
  const ssi::WLine* best = nullptr;
  for (const ssi::WLine& w : tr.lines) {
    if (w.points.size() < 3) continue;
    if (w.status == ssi::TraceStatus::Closed) return w;
    if (!best || w.points.size() > best->points.size()) best = &w;
  }
  return best ? *best : ssi::WLine{};
}

// The seam fidelity on G's NURBS wall: verify S_G(u2,v2) round-trips to the node. The
// direct analog of `npsdetail::seamFidelity` (which checks F's (u1,v1) side); here the
// surface is G's wall, evaluated at the WLine's (u2,v2) via `tess::SurfaceEvaluator`.
inline double seamFidelityOnG(const topo::FaceSurface& fsG, const topo::Location& locG,
                              const ssi::WLine& seam) {
  tess::SurfaceEvaluator ev(fsG, locG);
  double maxFid = 0.0;
  for (const ssi::WLinePoint& p : seam.points)
    maxFid = std::max(maxFid, math::distance(ev.value(p.u2, p.v2), p.point));
  return maxFid;
}

// ─────────────────────────────────────────────────────────────────────────────
// ROBUST REGION MEMBERSHIP (the CUT-leg core). The Bézier/COMMON path samples a
// sub-face at its OUTER-loop UV CENTROID (`ffcdetail::subFaceCentroid3d`). For the
// disk that is fine; for the ANNULUS it is a fragile-apex sample — the annulus's
// outer loop is the (symmetric) rim, whose UV centroid lands AT THE POLE, a point
// that lies in the annulus's HOLE, not the annulus. Both the disk and the annulus
// then evaluate at the same pole point and classify identically, so the CUT leg —
// which needs the annulus OUTSIDE and the disk INSIDE — cannot separate them and
// HONEST-DECLINES (ClassifyAmbiguous). This block replaces that single fragile
// sample with a WINDING/RAY test over points GENUINELY INTERIOR to each region in
// (u,v), tie-broken by unanimity, with a near-pole/tangent HONEST DECLINE (never a
// widened tolerance): a region whose interior probes all collapse into the seam's
// ON-band is genuinely ambiguous and is declined, not guessed.
// ─────────────────────────────────────────────────────────────────────────────

// Distance in (u,v) from `q` to the closest edge of a closed hole loop.
inline double uvDistToLoop(const tess::UV& q, const tess::UVPolygon& loop) noexcept {
  const int n = static_cast<int>(loop.size());
  if (n < 2) return std::numeric_limits<double>::infinity();
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    const tess::UV& a = loop[i];
    const tess::UV& b = loop[(i + 1) % n];
    const double eu = b.u - a.u, ev = b.v - a.v;
    const double len2 = eu * eu + ev * ev;
    double t = 0.0;
    if (len2 > 0.0) t = std::clamp(((q.u - a.u) * eu + (q.v - a.v) * ev) / len2, 0.0, 1.0);
    const double du = q.u - (a.u + t * eu), dv = q.v - (a.v + t * ev);
    best = std::min(best, std::hypot(du, dv));
  }
  return best;
}

// A point genuinely interior to a sub-face region (inside the outer loop AND outside
// every hole loop). Grid-samples the UV bbox; among the interior samples, returns the
// one FARTHEST (in u,v) from every trim loop (outer + holes) — the most robust probe,
// as far from any pole-adjacent seam as the region allows. Also reports that clearance
// (`clearance`) so the caller can apply the near-pole/tangent guard. nullopt if the
// region has no usable interior sample (a degenerate / collapsed region).
struct InteriorProbe {
  tess::UV uv{};
  double clearance = 0.0;   ///< min (u,v) distance from this probe to any trim loop
  double uvExtent = 0.0;    ///< region UV bbox diagonal (the clearance scale)
};

inline std::optional<InteriorProbe> interiorUvProbe(const topo::Shape& subFace, int grid = 24) {
  const tess::UVRegion reg = tess::buildRegion(subFace, 24);
  if (!reg.hasOuter()) return std::nullopt;
  const double du = reg.box.uMax - reg.box.uMin;
  const double dv = reg.box.vMax - reg.box.vMin;
  if (!(du > 0.0) || !(dv > 0.0)) return std::nullopt;
  const double extent = std::hypot(du, dv);
  auto loopDist = [&](const tess::UV& q) {
    double d = uvDistToLoop(q, reg.outer);
    for (const tess::UVPolygon& h : reg.holes) d = std::min(d, uvDistToLoop(q, h));
    return d;
  };
  InteriorProbe best;
  bool found = false;
  for (int iu = 1; iu < grid; ++iu)
    for (int iv = 1; iv < grid; ++iv) {
      const tess::UV q{reg.box.uMin + du * iu / grid, reg.box.vMin + dv * iv / grid};
      if (!reg.inside(q)) continue;
      const double d = loopDist(q);
      if (!found || d > best.clearance) {
        best.uv = q;
        best.clearance = d;
        found = true;
      }
    }
  if (!found) return std::nullopt;
  best.uvExtent = extent;
  return best;
}

// Classify a sub-face's SIDE (In/Out) in `other`'s mesh by the winding/ray parity of a
// point GENUINELY INTERIOR to the region in (u,v), with a near-pole/tangent HONEST
// DECLINE. Returns the membership, or nullopt when the region is genuinely ambiguous:
//   * no usable interior UV probe (degenerate region), OR
//   * the best interior probe's UV clearance from the trim loops is below a fraction of
//     the region's UV extent (the region collapses onto a seam tangent to the pole ray —
//     the apex ambiguity; declined, NOT widened), OR
//   * the 3-D probe classifies On/Unknown (within the mesh's own ON-band of the seam).
inline std::optional<Membership> regionSideRobust(const topo::Shape& subFace,
                                                  const topo::FaceSurface& fs,
                                                  const topo::Location& loc,
                                                  const tess::Mesh& other, const Aabb& otherBox,
                                                  double deflection) {
  const auto probe = interiorUvProbe(subFace);
  if (!probe) return std::nullopt;
  // Near-pole/tangent guard: an interior sample must sit a real fraction of the region
  // away from EVERY trim loop, else the region is a pole-tangent sliver (apex ambiguity).
  constexpr double kMinClearanceFrac = 0.05;
  if (!(probe->clearance >= kMinClearanceFrac * probe->uvExtent)) return std::nullopt;
  const tess::SurfaceEvaluator ev(fs, loc);
  const math::Point3 p3 = ev.value(probe->uv.u, probe->uv.v);
  const Membership m = classifyPointInMesh(other, otherBox, deflection, p3);
  if (m == Membership::On || m == Membership::Unknown) return std::nullopt;
  return m;
}

// Pick the sub-face of a split whose interior region has the wanted membership in
// `other`'s mesh, classifying BOTH sub-faces INDEPENDENTLY by the robust interior probe.
// Requires the two sub-faces to resolve to COMPLEMENTARY sides (one In, one Out) — the
// coherence tie-break — so a genuinely-ambiguous split (both sides collapse, or agree)
// HONEST-DECLINES to nullopt rather than guessing. Returns the sub-face matching `want`.
inline std::optional<topo::Shape> pickRegionRobust(const SmoothFaceSplit& split,
                                                   const topo::FaceSurface& fs,
                                                   const topo::Location& loc,
                                                   const tess::Mesh& other, const Aabb& otherBox,
                                                   double deflection, Membership want) {
  const auto sIn = regionSideRobust(split.faceInside, fs, loc, other, otherBox, deflection);
  const auto sOut = regionSideRobust(split.faceOutside, fs, loc, other, otherBox, deflection);
  if (!sIn || !sOut) return std::nullopt;              // a region is genuinely ambiguous
  if (*sIn == *sOut) return std::nullopt;              // both same side ⇒ no clean split
  if (*sIn == want) return split.faceInside;
  if (*sOut == want) return split.faceOutside;
  return std::nullopt;
}

}  // namespace nfsdetail

// ─────────────────────────────────────────────────────────────────────────────
// nurbsFaceFreeformSplit — the L3-S3 verb. `F`,`G` are freeform bowl-cup operands whose
// SINGLE freeform walls are BOTH genuine NURBS (`Kind::BSpline`) surfaces, meeting in ONE
// CLOSED interior seam. For `op == FfOp::Common`, build the welded, self-verified lens
// (`F ∩ G`) Solid — or a measured decline (NULL solid). The general freeform↔freeform
// (both operands arbitrary NURBS) watertight sew, gated by the fidelity check
// (S(pcurve)==C on BOTH F and G) + the mandatory M0 watertight/coherent/volume self-verify.
//
// `analyticOpVolume` (optional, NaN ⇒ unknown): the closed-form volume of the requested
// operation. When supplied, the self-verify is TWO-SIDED (a too-SMALL wrong volume — the
// signature of an orientation-collapsed shell — is rejected as VolumeInconsistent).
//
// NOTE: `FfOp::Common` welds. `FfOp::Cut` now RESOLVES its survivor membership honestly
// (robust interior-UV winding, not a fragile apex sample) but HONEST-DECLINES at the weld
// (`NotWatertight`) — the frozen M0 mesher does not weld a holed curved annulus to a curved
// disk across the seam — reporting the measured residual (`cutMembershipResolved`,
// `weldOpenEdges`). Never a leaky/wrong solid; no tolerance widened.
// ─────────────────────────────────────────────────────────────────────────────
inline NurbsFreeformSplitResult nurbsFaceFreeformSplit(
    const topo::Shape& F, const topo::Shape& G, FfOp op, double meshDeflection = 0.005,
    double analyticOpVolume = std::numeric_limits<double>::quiet_NaN()) {
  using namespace ffcdetail;
  NurbsFreeformSplitResult r;
  auto fail = [&](NurbsFreeformSplitDecline d) -> NurbsFreeformSplitResult {
    r.decline = d;
    r.solid = topo::Shape{};
    return r;
  };

  // ── (1) B1 recognise both operands, each requiring ONE genuine NURBS wall ──────
  const auto foF = recogniseFreeformSolid(F);
  if (!foF) return fail(NurbsFreeformSplitDecline::NotAdmittedF);
  const auto foG = recogniseFreeformSolid(G);
  if (!foG) return fail(NurbsFreeformSplitDecline::NotAdmittedG);

  const OperandFace* wallF = nullptr;
  const OperandFace* wallG = nullptr;
  NurbsFreeformSplitDecline wF = NurbsFreeformSplitDecline::Ok, wG = NurbsFreeformSplitDecline::Ok;
  const topo::FaceSurface* fsF =
      nfsdetail::nurbsWall(*foF, &wallF, NurbsFreeformSplitDecline::WallFNotNurbs, wF);
  if (!fsF) return fail(wF);
  const topo::FaceSurface* fsG =
      nfsdetail::nurbsWall(*foG, &wallG, NurbsFreeformSplitDecline::WallGNotNurbs, wG);
  if (!fsG) return fail(wG);

  // ── (2) M1 TRACE the shared CLOSED curved seam (u1,v1 on F; u2,v2 on G) ────────
  const ssi::WLine seam = nfsdetail::traceNurbsSharedSeam(*fsF, *fsG);
  if (seam.points.size() < 3 || seam.status != ssi::TraceStatus::Closed)
    return fail(NurbsFreeformSplitDecline::SeamUnusable);
  r.seamNodes = static_cast<int>(seam.points.size());

  // ── (3) FIDELITY GATE (stage 2): S_F(u1,v1)==node AND S_G(u2,v2)==node ─────────
  // The seam pcurve on F's NURBS wall is (u1,v1); on G's NURBS wall is (u2,v2). BOTH
  // must round-trip — the seam lies on BOTH freeform surfaces — else a drifted seam is
  // REJECTED (never welded). The tolerance is scale-relative to F's control-net diagonal.
  math::Point3 lo = fsF->poles.front(), hi = fsF->poles.front();
  for (const math::Point3& p : fsF->poles) {
    lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
    hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
  }
  const double diag = std::max(math::distance(lo, hi), 1e-9);
  npsdetail::seamFidelity(*fsF, wallF->location, seam, r.seamFidelityF, r.seamOnSurf);
  r.seamFidelityG = nfsdetail::seamFidelityOnG(*fsG, wallG->location, seam);
  const double fidTol = 1e-6 * std::max(diag, 1.0);
  if (!(r.seamFidelityF <= fidTol) || !(r.seamFidelityG <= fidTol) || !(r.seamOnSurf <= fidTol))
    return fail(NurbsFreeformSplitDecline::SeamOffSurface);

  // ── (4) SPLIT (stage 3): smooth-trim BOTH NURBS walls by the SAME 3-D seam ─────
  // F's wall by (u1,v1); G's wall by a (u2,v2)-re-keyed copy of the SAME seam (splitFace-
  // SmoothTrim reads points[i].{u1,v1} as the pcurve on the face it is splitting).
  const SmoothSplitResult srF = splitFaceSmoothTrim(wallF->face, seam);
  if (!srF.ok()) return fail(NurbsFreeformSplitDecline::SmoothSplitFailedF);
  const SmoothSplitResult srG = splitFaceSmoothTrim(wallG->face, rekeyToB(seam));
  if (!srG.ok()) return fail(NurbsFreeformSplitDecline::SmoothSplitFailedG);
  r.areaInsideF = srF.split->areaInside;
  r.areaInsideG = srG.split->areaInside;
  r.tilingGapF = srF.tilingGap;
  r.tilingGapG = srG.tilingGap;

  // Pre-cut operand meshes (B3 membership + independent op-volume bounds).
  tess::MeshParams mp;
  mp.deflection = meshDeflection;
  const tess::Mesh meshF = tess::SolidMesher(mp).mesh(foF->solid);
  const tess::Mesh meshG = tess::SolidMesher(mp).mesh(foG->solid);
  if (!tess::isWatertight(meshF) || !tess::isWatertight(meshG))
    return fail(NurbsFreeformSplitDecline::NotWatertight);
  const Aabb bbF = meshAabb(meshF), bbG = meshAabb(meshG);

  // ── (5) SELECT survivors by B3 membership (COMMON = the lens) ──────────────────
  std::vector<topo::Shape> faces;
  if (op == FfOp::Common) {
    // F ∩ G: F's wall sub-face INSIDE G (F's disk cap) + G's wall sub-face INSIDE F
    // (G's disk cap) — the lens, a curved-NURBS-cap to curved-NURBS-cap solid.
    const auto fKeep = pickByMembership(*srF.split, meshG, bbG, meshDeflection, Membership::In);
    const auto gKeep = pickByMembership(*srG.split, meshF, bbF, meshDeflection, Membership::In);
    if (!fKeep || !gKeep) return fail(NurbsFreeformSplitDecline::ClassifyAmbiguous);
    faces.push_back(*fKeep);
    faces.push_back(*gKeep);
  } else {
    // F − G (CUT): F's annulus OUTSIDE G + F's lids OUTSIDE G + G's disk INSIDE F (the
    // new curved ceiling of the carved lens). The annulus's OUTER-loop UV centroid lands
    // on the pole (in its own hole), so the fragile-apex sample of `pickByMembership`
    // cannot separate annulus-Out from disk-In. `nfsdetail::pickRegionRobust` instead
    // classifies EACH sub-region by a point GENUINELY INTERIOR to it in (u,v), requires
    // the two to resolve to COMPLEMENTARY sides (coherence), and HONEST-DECLINES the
    // pole-tangent / genuinely-ambiguous case (never widening a tolerance).
    const auto fKeep = nfsdetail::pickRegionRobust(*srF.split, *fsF, wallF->location, meshG, bbG,
                                                   meshDeflection, Membership::Out);
    const auto gKeep = nfsdetail::pickRegionRobust(*srG.split, *fsG, wallG->location, meshF, bbF,
                                                   meshDeflection, Membership::In);
    if (!fKeep || !gKeep) return fail(NurbsFreeformSplitDecline::ClassifyAmbiguous);
    r.cutMembershipResolved = true;  // robust interior-UV winding separated annulus-Out/disk-In
    faces.push_back(*fKeep);
    faces.push_back(*gKeep);
    collectAnalyticByMembership(*foF, meshG, bbG, meshDeflection, Membership::Out, faces);
  }
  if (faces.size() < 2) return fail(NurbsFreeformSplitDecline::WeldOpen);

  // ── (6) SEW (stage 5, curved-NURBS↔curved-NURBS): weld → Solid, REPAIR orientation
  //         coherence (the directed-edge invariant — exactly one cap reversed), then the
  //         mandatory M0 watertight + volume self-verify. The two NURBS caps share the
  //         EXACT seam nodes (splitFaceSmoothTrim's bit-identical outer ring on both
  //         sides), so the M0 mesher position-welds NURBS-disk↔NURBS-disk watertight.
  //         The CUT leg additionally welds a HOLED curved annulus to the curved disk; the
  //         frozen M0 mesher does not weld that pairing (a holed curved sub-face's seam-as-
  //         hole triangulation diverges from the disk's seam-as-outer), so the CUT decline —
  //         when the membership has already RESOLVED — is a MEASURED weld residual, recorded
  //         below as `weldOpenEdges` for the honest residual map (never widened to pass). ──
  {
    // Measure the raw (unrepaired) weld's unpaired-boundary count for the residual map.
    const topo::Shape probeShell = topo::ShapeBuilder::makeShell(faces);
    const topo::Shape probeSolid = topo::ShapeBuilder::makeSolid({probeShell});
    const tess::Mesh probeMesh = tess::SolidMesher(mp).mesh(probeSolid);
    for (const auto& [edge, uses] : tess::edgeUseCounts(probeMesh))
      if (uses != 2) ++r.weldOpenEdges;
  }
  const auto welded = weldOrientationCoherent(std::move(faces), mp);
  if (!welded) return fail(NurbsFreeformSplitDecline::NotWatertight);
  const topo::Shape result = welded->first;
  const tess::Mesh& m = welded->second;
  r.watertight = true;  // weldOrientationCoherent only returns a watertight+coherent mesh
  const double v = std::fabs(tess::enclosedVolume(m));
  r.enclosedVolume = v;
  if (!(v > 0.0) || std::isnan(v)) return fail(NurbsFreeformSplitDecline::VolumeInconsistent);

  // Self-verify — op-volume UPPER bound (COMMON ⊂ F and ⊂ G ⇒ 0 < V ≤ min(V_F,V_G)).
  const double vF = std::fabs(tess::enclosedVolume(meshF));
  const double vG = std::fabs(tess::enclosedVolume(meshG));
  const double upper = (op == FfOp::Cut) ? vF : std::min(vF, vG);
  const double upperTol = 0.05 * std::max(std::max(vF, vG), 1e-12);
  if (v > upper + upperTol) return fail(NurbsFreeformSplitDecline::VolumeInconsistent);

  // TWO-SIDED band vs the closed-form op-volume, if supplied. A triangulated smooth cap
  // UNDER-estimates the true volume by O(deflection); the band scales with the deflection
  // (capped at 50%), so a too-SMALL wrong volume (orientation collapse) is an honest
  // VolumeInconsistent decline even when the coherence gate happens to pass.
  if (!std::isnan(analyticOpVolume) && analyticOpVolume > 0.0) {
    constexpr double kVolConvergeSlope = 30.0;
    const double band = std::min(0.5, kVolConvergeSlope * meshDeflection) * analyticOpVolume;
    if (std::fabs(v - analyticOpVolume) > band)
      return fail(NurbsFreeformSplitDecline::VolumeInconsistent);
  }

  r.solid = result;
  r.decline = NurbsFreeformSplitDecline::Ok;
  return r;
}

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_NURBS_FREEFORM_SPLIT_H
