// SPDX-License-Identifier: Apache-2.0
//
// seam_strip.h — the SHARED SEAM-STRIP WELD (MESH-STRIP-IMPL): mesh the seam-adjacent
// strip of a curved↔curved closed seam ONCE, shared by BOTH faces, so the two faces'
// near-seam triangles are LITERALLY the same triangles (mirror-wound) and pair 1:1 at
// the spatial weld — instead of two INDEPENDENT per-face constrained-Delaunay
// triangulations whose non-corresponding near-seam slivers collapse to a 4×-used
// (non-manifold) seam edge when welded.
//
// ── THE MEASURED ROOT CAUSE (from tracks W2 / MESH-FINE / MESH-SHARED-STRIP) ──────
// Two curved "annulus" sub-faces (produced by the freeform↔freeform boolean split)
// share a CLOSED seam loop carried as 2-pole degree-1 STRAIGHT CHORDS (isSeamChord).
// The seam appears as an OUTER wire of one sub-face and a HOLE wire of the other. The
// M0w SeamPins already makes the seam BOUNDARY vertices bit-identical in 3-D. Each
// annulus meshed ALONE is a clean 2-manifold. But near the seam the wall is
// near-vertical, so each face's ConstrainedDelaunay inserts INTERIOR curvature Steiner
// points whose S(u,v) maps ONTO the seam circle in 3-D. The two faces produce
// NON-CORRESPONDING near-seam triangulations (near-zero-3-D-area on-seam slivers +
// real ring triangles, DISTINCT vertex triples), so the SPATIAL WELD collapses a dense
// pile of on-seam vertices and a shared seam edge is used 4× — non-manifold, and the
// residual GROWS with refinement (a finer seam samples the pile more densely). No
// post-hoc weld repair fixes it: dropping the slivers to kill the non-manifold leaves
// OPEN edges (proven by track MESH-SHARED-STRIP).
//
// ── THE FIX (bounded two-phase per-face fill) ────────────────────────────────────
// PHASE 1 (this header, once per shared seam loop, keyed by the seam's shared 3-D
// geometry): compute a COLLAR ring — a one-ring-inward offset of the seam ring toward
// the seam-loop axis, in the seam plane, at radius r_seam − δ — and the fixed STRIP
// triangulation of the annular band between the seam ring and the collar ring. Both the
// collar ring and the strip triangles are computed ONLY from the SHARED seam geometry
// (the bit-identical seam d.points), so they are IDENTICAL for both faces. The collar
// sits a hair OFF each surface (δ small) — the SAME bargain the SeamPins seam boundary
// already strikes (it pins the boundary onto the flat shared chord, off the bulging
// surface). Because both faces emit the SAME strip triangles referencing the SAME 3-D
// seam+collar vertices, the shared seam edge is used exactly twice → 2-manifold.
//
// PHASE 2 (face_mesher.h): each face pins its seam+collar boundary vertices to the
// shared 3-D values, adds the collar ring as a CONSTRAINED loop, SUPPRESSES interior
// Steiner insertion inside the collar band (in the shared 3-D seam frame, so both faces
// suppress the identical band), and fills only the collar-OUTWARD remainder with its
// own CDT. The seam-adjacent triangles are the shared strip; the outer fill is per-face
// (they don't share a seam there, so no correspondence is needed).
//
// The path fires ONLY for a genuinely-shared seam loop (an isSeamChord loop carried by
// EXACTLY TWO distinct faces); every existing single-face mesh is byte-identical.
//
// OCCT-FREE. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_SEAM_STRIP_H
#define CYBERCAD_NATIVE_TESSELLATE_SEAM_STRIP_H

#include "native/math/vec.h"
#include "native/tessellate/edge_mesher.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cybercad::native::tessellate {

namespace detail {

// A shared seam-strip collar frame + per-seam-vertex collar points, keyed by the seam's
// shared 3-D geometry. Both faces read the SAME collar for a given seam vertex.
struct SeamStrip {
  math::Point3 centroid;             ///< seam-loop centroid Cc (seam-plane origin)
  math::Vec3 axis{0, 0, 1};          ///< seam-loop best-fit normal Ac (unit)
  double rSeam = 0.0;                ///< mean seam radius off the axis
  double collarInset = 0.0;          ///< δ, the inward collar offset (r_collar = rSeam − δ)
  double bandInset = 0.0;            ///< the near-seam SUPPRESSION band half-width (deflection-INDEPENDENT)
  bool valid = false;
};

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// SeamStripRegistry — built ONCE per SolidMesher::mesh call (a pre-pass over all
// faces). It records every isSeamChord loop carried by EXACTLY TWO distinct faces
// (the genuinely-shared curved↔curved seam), and for each such seam records the
// collar frame + a lookup, keyed by the seam vertex's quantized 3-D point, from a
// seam vertex to its shared 3-D COLLAR point. A face consults `collarFor(P)` while
// flattening its seam boundary: it gets the SAME collar point the other face gets,
// so the two faces' near-seam strip is bit-identical.
// ─────────────────────────────────────────────────────────────────────────────
class SeamStripRegistry {
 public:
  bool empty() const noexcept { return cells_.empty(); }

  // ── WELD-RESOLUTION-AWARE SEAM DECIMATION (MESH-WELD-TOL) ────────────────────
  // Declare the mesher's spatial weld tolerance BEFORE registering seam loops. A seam
  // ring sampled DENSER than the weld tolerance (at fine deflection the deflection-driven
  // seam sampling spacing ≪ weldTol = 0.5·deflection) would have runs of adjacent ring
  // vertices MERGE at the spatial weld — and because the collar ring mirrors the seam
  // ring 1:1, the collar (where the CDT fill meets the spliced strip) merges the same
  // way, collapsing strip quads against per-face CDT boundary triangles into 4×-used
  // edges (measured: at d=0.00125 on the asym annulus↔annulus pose the strip pass went
  // non-manifold at BOTH collar rings and the mesher fell back to the non-watertight
  // baseline). The fix: each registered ring keeps only samples ≥ 2·weldTol apart (two
  // points ≥ 2·tol apart can never share a weld cell of side tol — cell diagonal is
  // √3·tol), flagged per Entry; the face mesher builds the strip + collar loop from the
  // KEPT subset only. This is a TIGHTENING of what the weld may merge (never a widened
  // weld tolerance): a ring already sampled coarser than 2·weldTol keeps every sample —
  // bit-identical to the pre-decimation strip. Sagitta cost of the coarser seam chord is
  // (2·weldTol)²/(8·rSeam) = deflection²/(8·rSeam) ≪ deflection, so the decimated strip
  // stays within the deflection band.
  void setWeldResolution(double weldTol) noexcept {
    minKeepSpacing_ = weldTol > 0.0 ? 2.0 * weldTol : 0.0;
  }

  // Was the registered seam vertex nearest `p` KEPT by the weld-resolution decimation?
  // (Every vertex of an undecimated ring is kept.) Both faces resolve a seam sample to
  // the SAME entry, so they keep the identical subset → their strips stay bit-identical.
  bool keptSeamVertex(const math::Point3& p) const {
    const Entry* e = findEntry(p);
    return e && e->kept;
  }

  // The shared COLLAR 3-D point for the seam vertex at `p` (a seam boundary sample), on
  // the MATERIAL side indicated by `materialProbe` (a 3-D point a hair into the face's
  // material from the seam — the face's inward-UV-marched sample). Returns nullptr if `p`
  // is not on a registered shared seam. The collar is offset from the seam toward the
  // material side (increasing OR decreasing radius) — chosen by which candidate the probe
  // is radially closer to — so BOTH faces sharing the seam (whose material is on the SAME
  // 3-D side) pick the IDENTICAL collar point and their near-seam strip is bit-identical.
  const math::Point3* collarFor(const math::Point3& p, const math::Point3& materialProbe) const {
    const Entry* e = findEntry(p);
    if (!e) return nullptr;
    const detail::SeamStrip& s = strips_[static_cast<std::size_t>(e->strip)];
    const double rp = radiusOf(materialProbe, s);
    // Pick the collar candidate on the same radial side as the material probe.
    return rp >= s.rSeam ? &e->collarOut : &e->collarIn;
  }

  // The strip index of the registered seam vertex nearest `p` (for a per-loop side decision),
  // or -1. Both faces resolve a given seam boundary to the SAME strip.
  int stripIndexFor(const math::Point3& p) const {
    const Entry* e = findEntry(p);
    return e ? e->strip : -1;
  }

  // The collar candidate on the requested radial side (outward=true ⇒ collarOut). Used after a
  // per-LOOP side decision so the WHOLE loop's collar sits on ONE consistent radial side —
  // essential near a near-vertical wall where a per-vertex material probe's radius is noisy and
  // would otherwise zigzag the collar across the seam.
  const math::Point3* collarOnSide(const math::Point3& p, bool outward) const {
    const Entry* e = findEntry(p);
    if (!e) return nullptr;
    return outward ? &e->collarOut : &e->collarIn;
  }

  // The seam radius of the strip containing `p` (for the per-loop side vote), or 0.
  double seamRadiusFor(const math::Point3& p) const {
    const Entry* e = findEntry(p);
    return e ? strips_[static_cast<std::size_t>(e->strip)].rSeam : 0.0;
  }
  double radiusInStrip(const math::Point3& probe, const math::Point3& seamPt) const {
    const Entry* e = findEntry(seamPt);
    if (!e) return 0.0;
    return radiusOf(probe, strips_[static_cast<std::size_t>(e->strip)]);
  }

  // Is `p` inside the near-seam SUPPRESSION band of any registered strip (i.e. its distance
  // from that seam's axis, in the seam plane, is within ±bandInset of r_seam)? Interior
  // Steiner points in the band are SUPPRESSED so neither face triangulates the shared strip
  // independently. Tested in the shared 3-D seam frame, so both faces suppress the identical
  // band. The band half-width `bandInset` is DEFLECTION-INDEPENDENT (a fraction of rSeam, not
  // of the deflection-shrinking segLen) so the suppression does NOT degrade as the mesh
  // refines — the L3-BAND fine-deflection collapse fix.
  bool inCollarBand(const math::Point3& p) const {
    for (const detail::SeamStrip& s : strips_) {
      if (!s.valid) continue;
      const math::Vec3 d = p - s.centroid;
      const double axial = math::dot(d, s.axis);
      const math::Vec3 radial = d - s.axis * axial;
      const double r = math::norm(radial);
      // Inside the band ⇔ radius within [r_seam − bandInset, r_seam + bandInset] AND axially
      // near the seam plane (within bandInset of it, so a genuinely-interior far-from-seam
      // point is never suppressed).
      if (r >= s.rSeam - s.bandInset - kBandEps && r <= s.rSeam + s.bandInset + kBandEps &&
          std::fabs(axial) <= s.bandInset + kBandEps)
        return true;
    }
    return false;
  }

  // ── Build (SolidMesher pre-pass) ─────────────────────────────────────────────
  // Register one shared seam loop from its ORDERED, closed ring of bit-identical 3-D
  // seam vertices (`ring`, the shared EdgeCache d.points chained around the loop). The
  // collar frame + per-vertex collar points are derived ONLY from `ring`, so both faces
  // (which read the same d.points) register the identical collar.
  void addSeamLoop(const std::vector<math::Point3>& ring) {
    if (ring.size() < 3) return;
    detail::SeamStrip s = computeStrip(ring);
    if (!s.valid) return;
    // Cell size = the collar inset (≈ half the seam sample spacing) so a nearest-vertex probe
    // needs only a 3×3×3 neighbour sweep. Set on the FIRST strip; a later, finer strip shrinks
    // it (all strips then rehash to the finest cell — cheap, done rarely).
    const double cell = std::max(s.collarInset, 1e-9);
    if (cellSize_ <= 0.0) cellSize_ = cell;
    else if (cell < cellSize_) rehash(cell);
    const int stripIdx = static_cast<int>(strips_.size());
    // Order the ring around the seam axis, then flag the weld-resolution KEPT subset
    // (MESH-WELD-TOL): consecutive kept samples ≥ minKeepSpacing_ apart, so no two kept
    // ring vertices can merge at the spatial weld. Registered ONCE and shared, so both
    // faces read the identical kept subset.
    const std::vector<math::Point3> ordered = orderRingByAngle(ring, s);
    const std::vector<bool> kept = decimateRing(ordered, minKeepSpacing_);
    const std::size_t n = ordered.size();
    for (std::size_t i = 0; i < n; ++i) {
      const math::Vec3 d = ordered[i] - s.centroid;
      const double axial = math::dot(d, s.axis);
      const math::Vec3 radial = d - s.axis * axial;
      const double r = math::norm(radial);
      if (r <= s.collarInset) continue;                    // degenerate — skip this vertex
      const math::Vec3 outward = radial / r;               // unit outward radial (in seam plane)
      Entry e;
      e.seam = ordered[i];
      e.collarIn = ordered[i] - outward * s.collarInset;   // toward the axis (decreasing radius)
      e.collarOut = ordered[i] + outward * s.collarInset;  // away from the axis (increasing radius)
      e.strip = stripIdx;
      e.kept = kept[i];
      cells_[keyOf(ordered[i])].push_back(e);
    }
    strips_.push_back(s);
    matchTol_ = std::max(matchTol_, s.collarInset);
  }

 private:
  static constexpr double kSnapEps = 1e-6;   // max dist a seam sample may match a key
  static constexpr double kBandEps = 1e-12;  // band-membership slack

  struct Key {
    long long x, y, z;
    bool operator==(const Key& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
  };
  struct Hash {
    std::size_t operator()(const Key& k) const noexcept {
      std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
      h ^= static_cast<std::size_t>(k.y) * 19349663u;
      h ^= static_cast<std::size_t>(k.z) * 83492791u;
      return h;
    }
  };
  struct Entry {
    math::Point3 seam;       ///< the seam vertex (for the eps re-check)
    math::Point3 collarIn;   ///< collar candidate toward the axis (r_seam − δ)
    math::Point3 collarOut;  ///< collar candidate away from the axis (r_seam + δ)
    int strip = -1;          ///< index into strips_ (the collar frame)
    bool kept = true;        ///< weld-resolution decimation flag (MESH-WELD-TOL)
  };

  // Order the (deduped, unordered) ring samples by angle around the strip axis — the
  // deterministic cyclic order the weld-resolution decimation walks. Uses an arbitrary
  // fixed in-plane basis; determinism (not a particular start angle) is all that matters,
  // and the ONE shared registry serves both faces, so the kept subset is shared too.
  static std::vector<math::Point3> orderRingByAngle(const std::vector<math::Point3>& ring,
                                                    const detail::SeamStrip& s) {
    // In-plane basis ⊥ axis: e1 = normalize(any ⊥ axis), e2 = axis × e1.
    const math::Vec3 seed = std::fabs(s.axis.z) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
    math::Vec3 e1 = math::cross(s.axis, seed);
    e1 = e1 / math::norm(e1);
    const math::Vec3 e2 = math::cross(s.axis, e1);
    std::vector<std::pair<double, std::size_t>> ang;
    ang.reserve(ring.size());
    for (std::size_t i = 0; i < ring.size(); ++i) {
      const math::Vec3 d = ring[i] - s.centroid;
      ang.emplace_back(std::atan2(math::dot(d, e2), math::dot(d, e1)), i);
    }
    std::sort(ang.begin(), ang.end());
    std::vector<math::Point3> out;
    out.reserve(ring.size());
    for (const auto& [a, i] : ang) out.push_back(ring[i]);
    return out;
  }

  // Weld-resolution decimation (MESH-WELD-TOL): walking the angle-ordered ring, keep a
  // sample only when it is ≥ minSpacing (3-D) from the LAST kept one, so no two kept ring
  // vertices can land in one weld cell. The wrap pair (last kept → first kept) is closed
  // by un-keeping the last kept sample if it violates the spacing. A ring already sampled
  // coarser than minSpacing keeps EVERY sample (bit-identical to the undecimated strip);
  // if decimation would leave < 3 kept vertices (no valid strip ring), it is disabled and
  // every sample is kept — the pre-decimation behaviour, never a silently-degenerate ring.
  static std::vector<bool> decimateRing(const std::vector<math::Point3>& ring,
                                        double minSpacing) {
    const std::size_t n = ring.size();
    std::vector<bool> kept(n, true);
    if (minSpacing <= 0.0 || n < 3) return kept;
    std::size_t nKept = 1;      // ring[0] is always kept (the walk anchor)
    std::size_t last = 0;       // index of the last kept sample
    for (std::size_t i = 1; i < n; ++i) {
      if (math::distance(ring[i], ring[last]) >= minSpacing) { last = i; ++nKept; }
      else kept[i] = false;
    }
    // Close the loop: the wrap pair must respect the spacing too.
    if (nKept > 3 && last != 0 && math::distance(ring[last], ring[0]) < minSpacing) {
      kept[last] = false;
      --nKept;
    }
    if (nKept < 3) return std::vector<bool>(n, true);  // decimation disabled — keep all
    return kept;
  }

  long long q(double v) const noexcept {
    const double s = 1.0 / (cellSize_ > 0.0 ? cellSize_ : 1e-7);
    return static_cast<long long>(v >= 0 ? v * s + 0.5 : v * s - 0.5);
  }
  Key keyOf(const math::Point3& p) const noexcept { return Key{q(p.x), q(p.y), q(p.z)}; }

  // Re-bucket every entry at a new (finer) cell size. Rare (only when a later strip is finer).
  void rehash(double newCell) {
    std::vector<Entry> all;
    for (auto& [k, v] : cells_) for (Entry& e : v) all.push_back(e);
    cellSize_ = newCell;
    cells_.clear();
    for (const Entry& e : all) cells_[keyOf(e.seam)].push_back(e);
  }

  // The registered seam vertex NEAREST `p`, if within the match tolerance, else null. `p` is a
  // face's seam BOUNDARY sample — either the straight-chord pin point (bit-identical to the
  // registered d.points) or the bulging SURFACE point (a hair off the chord at the seam). The
  // match tolerance (≈ the collar inset ≈ half the seam sample spacing) absorbs that
  // chord↔surface gap while still resolving to the correct (nearest) seam vertex, so BOTH faces
  // map a seam sample to the SAME registered seam vertex → the SAME collar. The cell size == the
  // match tolerance, so a 3×3×3 neighbour sweep covers every candidate.
  const Entry* findEntry(const math::Point3& p) const {
    if (cells_.empty()) return nullptr;
    const Key c = keyOf(p);
    const Entry* best = nullptr;
    double bestD = matchTol_;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          const auto it = cells_.find(Key{c.x + dx, c.y + dy, c.z + dz});
          if (it == cells_.end()) continue;
          for (const Entry& e : it->second) {
            const double d = math::distance(p, e.seam);
            if (d <= bestD) { bestD = d; best = &e; }
          }
        }
    return best;
  }

  // Radius of `p` off the strip's axis, in the seam plane.
  static double radiusOf(const math::Point3& p, const detail::SeamStrip& s) {
    const math::Vec3 d = p - s.centroid;
    return math::norm(d - s.axis * math::dot(d, s.axis));
  }

  // Compute the collar frame from the seam ring: centroid, best-fit axis (normalized sum
  // of successive edge cross-products about the centroid — the loop's own normal), mean
  // seam radius off the axis, and δ = a fixed fraction of the SEAM RADIUS (deflection-
  // INDEPENDENT — see the L3-BAND note below; NOT tied to the seam segment length, which
  // shrinks with refinement and used to collapse the collar band at fine deflection).
  static detail::SeamStrip computeStrip(const std::vector<math::Point3>& ring) {
    detail::SeamStrip s;
    const std::size_t n = ring.size();
    math::Vec3 c{0, 0, 0};
    for (const math::Point3& p : ring) c = c + p.asVec();
    c = c / static_cast<double>(n);
    s.centroid = math::Point3{c.x, c.y, c.z};
    math::Vec3 nrm{0, 0, 0};
    for (std::size_t i = 0; i < n; ++i) {
      const math::Vec3 a = ring[i] - s.centroid;
      const math::Vec3 b = ring[(i + 1) % n] - s.centroid;
      nrm = nrm + math::cross(a, b);
    }
    const double nl = math::norm(nrm);
    if (nl < 1e-30) return s;             // degenerate (collinear) loop — no strip
    s.axis = nrm / nl;
    double rsum = 0.0;
    for (const math::Point3& p : ring) {
      const math::Vec3 d = p - s.centroid;
      const double axial = math::dot(d, s.axis);
      rsum += math::norm(d - s.axis * axial);
    }
    s.rSeam = rsum / static_cast<double>(n);
    // ── DEFLECTION-ROBUST COLLAR (L3-BAND fix) ─────────────────────────────────────
    // The collar inset δ = r_seam − r_collar must be tied to the SEAM GEOMETRY, NOT to the
    // deflection-driven seam segment length. The pre-fix rule δ = min(0.5·segLen, 0.25·rSeam)
    // SHRINKS with segLen: as the deflection refines, the seam is sampled more densely, segLen
    // → 0, and δ collapses to a hair (measured 4.3e-5 at d=0.002 on the asym fixture). Below a
    // threshold the collar band then stops covering the curvature Steiner points each face's CDT
    // still inserts near the near-vertical wall, and the shared seam edge is used 4× — a
    // non-manifold whose count GROWS as the mesh refines (L3-BAND: COMMON 0→1→4 over d
    // 0.0025→0.002→0.00125). Tying δ to r_seam removes the segLen dependence entirely: the collar
    // band width is now a fixed fraction of the seam radius, so the near-seam suppression does NOT
    // degrade with refinement. The upper clamp 0.25·rSeam already kept the collar a strictly-
    // interior concentric ring (never crossing the axis / self-intersecting on a modestly-curved
    // seam); a lower floor keeps a sensibly-shaped strip on a very coarse (large-segLen) seam.
    // The collar RING and the SUPPRESSION band use the SAME inset so the CDT fill (collar-outward)
    // and the spliced strip (seam→collar) tile the annulus with no gap — never opening an edge.
    // δ = a small FIXED fraction of the seam radius — a stable, one-ring-scale geometric width
    // that is DEFLECTION-INDEPENDENT. Measured on the asym fixture (r₁≈0.154, r₂≈0.365): the pile
    // that made the OUTER seam r₂ used 4× at d=0.002 is suppressed for any frac ≳ 0.015; 0.05 is a
    // comfortable margin that stays one-ring-thin (δ_outer≈0.018, δ_inner≈0.0077) — small enough
    // not to distort the near-seam surface, large enough to survive arbitrary refinement.
    constexpr double kCollarFrac = 0.05;
    s.collarInset = kCollarFrac * s.rSeam;
    s.bandInset = s.collarInset;
    s.valid = s.collarInset > 0.0 && s.rSeam > 0.0;
    return s;
  }

  std::unordered_map<Key, std::vector<Entry>, Hash> cells_;
  std::vector<detail::SeamStrip> strips_;
  double cellSize_ = 0.0;        ///< spatial hash cell size (== the collar inset ≈ match tolerance)
  double matchTol_ = kSnapEps;   ///< nearest-seam-vertex match radius (grows with the collar inset)
  double minKeepSpacing_ = 0.0;  ///< weld-resolution decimation spacing (2·weldTol; 0 = keep all)
};

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_SEAM_STRIP_H
