// SPDX-License-Identifier: Apache-2.0
//
// reverse_engineer.cpp — Layer-7 reverse-engineering PIPELINE: segment a raw composite
// point cloud into typed regions (analytic primitive / freeform patch / honest-decline)
// and fit each. See reverse_engineer.h for the design.
//
// COMPOSES the existing Layer-7 fitters — primitive_fit::detectPrimitive for the
// analytic classification and bspline_fit::approximateSurface for the freeform patch —
// and never re-implements a fit. The region-growing (k-NN adjacency + residual-frontier
// BFS) and the plane-projection grid recovery are the only self-contained logic here.
//
// Both composed fitters go through the numsci facade (lstsq / least_squares), so the
// whole TU is under CYBERCAD_HAS_NUMSCI, mirroring primitive_fit.cpp / bspline_fit.cpp.
// With the guard OFF this file is inert and the functions are absent from the library.
//
#include "native/math/reverse_engineer.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace cybercad::native::math {
namespace {

// Absolute residual of a point to a detected primitive (the same closed-form distances
// primitive_fit uses internally; recomputed here so the frontier test is exact).
double residualToPrimitive(const Point3& p, const PrimitiveDetection& d) {
  switch (d.type) {
    case PrimitiveType::Plane:
      return std::fabs(dot(d.plane.normal.vec(), p.asVec()) - d.plane.offset);
    case PrimitiveType::Sphere:
      return std::fabs(distance(p, d.sphere.center) - d.sphere.radius);
    case PrimitiveType::Cylinder: {
      const Vec3 w = p - d.cylinder.axisPoint;
      const Vec3 perp = w - d.cylinder.axis.vec() * dot(w, d.cylinder.axis.vec());
      return std::fabs(norm(perp) - d.cylinder.radius);
    }
    case PrimitiveType::Cone: {
      const Vec3 v = p - d.cone.apex;
      const double h = dot(v, d.cone.axis.vec());
      const Vec3 radv = v - d.cone.axis.vec() * h;
      const double rho = norm(radv);
      return std::fabs(rho * std::cos(d.cone.halfAngle) - h * std::sin(d.cone.halfAngle));
    }
    case PrimitiveType::Freeform:
    default:
      return 1e300;
  }
}

// Cloud extent of an index subset (max distance from its centroid) — the scale used to
// turn the absolute `tol` into detectPrimitive's RELATIVE tolerance.
double subsetExtent(std::span<const Point3> pts, const std::vector<int>& idx) {
  double cx = 0, cy = 0, cz = 0;
  for (int i : idx) { cx += pts[i].x; cy += pts[i].y; cz += pts[i].z; }
  const double inv = 1.0 / static_cast<double>(idx.size());
  const Point3 c{cx * inv, cy * inv, cz * inv};
  double e = 0.0;
  for (int i : idx) e = std::max(e, distance(pts[i], c));
  return std::max(e, 1e-12);
}

// Gather the Point3 subset for an index list (the fitters take a contiguous span).
std::vector<Point3> gather(std::span<const Point3> pts, const std::vector<int>& idx) {
  std::vector<Point3> out;
  out.reserve(idx.size());
  for (int i : idx) out.push_back(pts[i]);
  return out;
}

// detectPrimitive on an index subset, with the RELATIVE tolerance derived from the
// absolute `tol` and the subset's own extent (so `tol` is honoured in model units).
PrimitiveDetection detectOnSubset(std::span<const Point3> pts, const std::vector<int>& idx,
                                  double tol) {
  const std::vector<Point3> sub = gather(pts, idx);
  const double extent = subsetExtent(pts, idx);
  return detectPrimitive(sub, tol / extent);
}

// ── k-nearest-neighbour adjacency ────────────────────────────────────────────
// Brute-force fp64 distances (OCCT-free, deterministic). Symmetric: j is a neighbour of
// i if i is among j's k-nearest OR j is among i's k-nearest, so growth is not trapped by
// asymmetric k-NN. n is small in the oracles; this is O(n²) and airtight.
std::vector<std::vector<int>> buildAdjacency(std::span<const Point3> pts, int k) {
  const int n = static_cast<int>(pts.size());
  std::vector<std::vector<int>> adj(static_cast<std::size_t>(n));
  std::vector<std::pair<double, int>> d;
  d.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    d.clear();
    for (int j = 0; j < n; ++j) {
      if (j == i) continue;
      d.emplace_back(distance(pts[i], pts[j]), j);
    }
    const int kk = std::min(k, static_cast<int>(d.size()));
    std::partial_sort(d.begin(), d.begin() + kk, d.end());
    for (int t = 0; t < kk; ++t) adj[i].push_back(d[t].second);
  }
  // Symmetrize.
  for (int i = 0; i < n; ++i)
    for (int j : adj[i])
      if (std::find(adj[j].begin(), adj[j].end(), i) == adj[j].end()) adj[j].push_back(i);
  return adj;
}

// ── Connected components of the adjacency graph ──────────────────────────────
// A CAD part scanned as a cloud partitions FIRST by connectivity: spatially disjoint
// faces (the plane+cylinder+sphere trio) fall into separate components, so a component
// is the natural unit to classify. Within a component that is a SINGLE surface (a pure
// cylinder, a freeform bump) the whole component is one region; a heterogeneous
// component (a rounded box, faces meeting) is sub-segmented by seeded region growing.
// This is deterministic (components discovered in ascending-index BFS order).
std::vector<std::vector<int>> connectedComponents(
    int n, const std::vector<std::vector<int>>& adj) {
  std::vector<char> seen(static_cast<std::size_t>(n), 0);
  std::vector<std::vector<int>> comps;
  for (int s = 0; s < n; ++s) {
    if (seen[s]) continue;
    std::vector<int> comp;
    std::vector<int> stack{s};
    seen[s] = 1;
    while (!stack.empty()) {
      const int cur = stack.back();
      stack.pop_back();
      comp.push_back(cur);
      for (int nb : adj[cur])
        if (!seen[nb]) { seen[nb] = 1; stack.push_back(nb); }
    }
    std::sort(comp.begin(), comp.end());
    comps.push_back(std::move(comp));
  }
  return comps;
}

// ── Seeded region growing (heterogeneous-component fallback) ──────────────────
// Grow a region from `seed` over the still-`avail` points of one component, keeping
// every member consistent with the CURRENT best-fit primitive (re-fit as the region
// grows so the primitive tracks the surface, escaping the local plane/sphere basin a
// tiny seed patch would otherwise lock into). Returns the grown index set. `avail`
// marks points in this component not yet claimed by a sub-region.
std::vector<int> growRegion(std::span<const Point3> pts,
                            const std::vector<std::vector<int>>& adj, int seed,
                            const std::vector<char>& avail, const SegmentParams& prm) {
  const int n = static_cast<int>(pts.size());
  std::vector<char> inRegion(static_cast<std::size_t>(n), 0);
  std::vector<int> region;

  // Seed generously: a small patch of a curved surface fits as a PLANE (or a sphere),
  // so grow the seed neighbourhood by BFS to a ball big enough that curvature registers
  // (≈ 4× the largest primitive minimum) BEFORE the first classification. Only `avail`
  // points are eligible.
  const int seedTarget = std::max(4 * prm.minRegion, 2 * prm.kNeighbors);
  region.push_back(seed);
  inRegion[seed] = 1;
  {
    std::vector<int> frontier{seed};
    while (!frontier.empty() && static_cast<int>(region.size()) < seedTarget) {
      const int cur = frontier.front();
      frontier.erase(frontier.begin());
      for (int nb : adj[cur]) {
        if (avail[nb] && !inRegion[nb]) {
          region.push_back(nb);
          inRegion[nb] = 1;
          frontier.push_back(nb);
          if (static_cast<int>(region.size()) >= seedTarget) break;
        }
      }
    }
  }
  if (static_cast<int>(region.size()) < prm.minRegion) return region;  // too small to classify

  PrimitiveDetection fit = detectOnSubset(pts, region, prm.tol);
  if (!fit.ok) return region;

  // Frontier growth to a fixpoint under the current fit, re-fitting after each pass so
  // the primitive (and thus the admitted set) tracks the whole surface.
  bool changed = true;
  while (changed) {
    changed = false;
    std::vector<int> frontier;
    for (int r : region)
      for (int nb : adj[r])
        if (avail[nb] && !inRegion[nb]) frontier.push_back(nb);
    std::sort(frontier.begin(), frontier.end());
    frontier.erase(std::unique(frontier.begin(), frontier.end()), frontier.end());

    for (int cand : frontier) {
      if (residualToPrimitive(pts[cand], fit) <= prm.tol) {
        region.push_back(cand);
        inRegion[cand] = 1;
        changed = true;
      }
    }
    if (changed) {
      PrimitiveDetection refit = detectOnSubset(pts, region, prm.tol);
      if (refit.ok) fit = refit;
    }
  }
  return region;
}

// ── Freeform grid recovery + fit ─────────────────────────────────────────────
// A structured surface scan is a topological QUAD LATTICE: project the region onto its
// total-least-squares plane and recover the (nU×nV) grid ordering by 4-connected LATTICE
// WALKING in the projected 2-D coords — from a corner, trace one axis to size the first
// row, then step the orthogonal axis row by row. A projection onto a fitted plane does
// NOT keep the lattice axis-aligned (the plane tilt mixes the axes), so axis-coordinate
// binning is unreliable; the walk follows the actual nearest-neighbour lattice edges and
// is robust to that tilt. Once the ordering is known, approximateSurface fits the tensor
// B-spline patch. HONEST-DECLINE (return false) when the region is NOT a clean full
// rectangular lattice (scattered / degenerate / incomplete) — the caller then declines.

// Recover the nU×nV lattice ordering of the projected 2-D points `q` (region-local
// indices 0..m-1). On success `order` is the row-major (U outer) list of region-local
// indices and nU/nV are set. Returns false if no clean full lattice is found.
bool recoverLattice(const std::vector<std::array<double, 2>>& q, int& nU, int& nV,
                    std::vector<int>& order) {
  const int m = static_cast<int>(q.size());
  if (m < 4) return false;

  const auto d2 = [&](int i, int j) {
    const double dx = q[i][0] - q[j][0], dy = q[i][1] - q[j][1];
    return dx * dx + dy * dy;
  };
  // Nearest neighbour and typical spacing (median-ish via mean of nearest distances).
  double meanStep = 0.0;
  for (int i = 0; i < m; ++i) {
    double best = 1e300;
    for (int j = 0; j < m; ++j)
      if (j != i) best = std::min(best, d2(i, j));
    meanStep += std::sqrt(best);
  }
  meanStep /= static_cast<double>(m);
  if (!(meanStep > 0.0)) return false;
  const double stepTol = 0.5 * meanStep;      // half a cell: a "same lattice step"
  const double matchR2 = meanStep * meanStep * 2.25;  // (1.5·step)² neighbour search radius

  // Corner = extreme in the (1,1) projected direction (lexicographic min of a+b then a).
  int corner = 0;
  for (int i = 1; i < m; ++i) {
    const double si = q[i][0] + q[i][1], sc = q[corner][0] + q[corner][1];
    if (si < sc - 1e-15 || (std::fabs(si - sc) <= 1e-15 && q[i][0] < q[corner][0]))
      corner = i;
  }

  // The two lattice step directions: the two nearest neighbours of the corner that are
  // ~orthogonal. Collect near neighbours, pick the closest as dir1, then the closest
  // whose direction is ~perpendicular to dir1 as dir2.
  std::vector<std::pair<double, int>> near;
  for (int j = 0; j < m; ++j)
    if (j != corner && d2(corner, j) <= matchR2) near.emplace_back(d2(corner, j), j);
  std::sort(near.begin(), near.end());
  if (near.size() < 2) return false;
  const auto unit = [&](int from, int to) -> std::array<double, 2> {
    double dx = q[to][0] - q[from][0], dy = q[to][1] - q[from][1];
    const double n = std::sqrt(dx * dx + dy * dy);
    return {dx / n, dy / n};
  };
  const std::array<double, 2> dir1 = unit(corner, near[0].second);
  std::array<double, 2> dir2{0, 0};
  bool haveDir2 = false;
  for (std::size_t k = 1; k < near.size(); ++k) {
    const std::array<double, 2> c = unit(corner, near[k].second);
    if (std::fabs(dir1[0] * c[0] + dir1[1] * c[1]) < 0.35) { dir2 = c; haveDir2 = true; break; }
  }
  if (!haveDir2) return false;

  // Step helper: from `cur`, the next lattice point along `dir` is the unvisited point
  // closest to cur + meanStep·dir within stepTol. Returns -1 if none (row/col end).
  std::vector<char> used(static_cast<std::size_t>(m), 0);
  const auto step = [&](int cur, const std::array<double, 2>& dir) -> int {
    const double tx = q[cur][0] + meanStep * dir[0], ty = q[cur][1] + meanStep * dir[1];
    int best = -1;
    double bestD = stepTol * stepTol;
    for (int j = 0; j < m; ++j) {
      if (used[j] || j == cur) continue;
      const double dx = q[j][0] - tx, dy = q[j][1] - ty;
      const double dd = dx * dx + dy * dy;
      if (dd < bestD) { bestD = dd; best = j; }
    }
    return best;
  };

  // Walk the first row along dir1 to size nV.
  std::vector<int> firstRow{corner};
  used[corner] = 1;
  for (int cur = corner;;) {
    const int nx = step(cur, dir1);
    if (nx < 0) break;
    used[nx] = 1;
    firstRow.push_back(nx);
    cur = nx;
  }
  nV = static_cast<int>(firstRow.size());
  if (nV < 2 || m % nV != 0) return false;
  nU = m / nV;
  if (nU < 2) return false;

  // Walk each subsequent row: step every element of the previous row along dir2.
  order.assign(static_cast<std::size_t>(nU) * nV, -1);
  for (int j = 0; j < nV; ++j) order[j] = firstRow[j];
  for (int i = 1; i < nU; ++i) {
    for (int j = 0; j < nV; ++j) {
      const int prev = order[(i - 1) * nV + j];
      const int nx = step(prev, dir2);
      if (nx < 0) return false;  // ragged → not a full lattice
      order[i * nV + j] = nx;
      used[nx] = 1;
    }
  }
  // Every point used exactly once ⇒ a clean full lattice.
  for (int i = 0; i < m; ++i)
    if (!used[i]) return false;
  return true;
}

bool fitFreeform(std::span<const Point3> pts, const std::vector<int>& region,
                 const SegmentParams& prm, BsplineSurfaceData& surfOut, double& rmsOut) {
  const std::vector<Point3> sub = gather(pts, region);
  const PlaneFit pf = fitPlane(sub);
  if (!pf.ok) return false;

  // In-plane orthonormal frame (u, w) spanning the TLS plane, and the projected coords.
  const Vec3 nrm = pf.normal.vec();
  Vec3 t = (std::fabs(nrm.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 u = cross(nrm, t);
  u = u / norm(u);
  const Vec3 w = cross(nrm, u);
  const int m = static_cast<int>(sub.size());
  std::vector<std::array<double, 2>> q(static_cast<std::size_t>(m));
  for (int i = 0; i < m; ++i) {
    const Vec3 rel = sub[i] - pf.centroid;
    q[i] = {dot(rel, u), dot(rel, w)};
  }

  int nU = 0, nV = 0;
  std::vector<int> order;  // row-major region-local indices
  if (!recoverLattice(q, nU, nV, order)) return false;
  if (nU < prm.freeformCtrl || nV < prm.freeformCtrl) return false;

  // Assemble the row-major (U outer) grid of ORIGINAL points and fit a bicubic patch.
  std::vector<Point3> grid(static_cast<std::size_t>(nU) * nV);
  for (int f = 0; f < nU * nV; ++f) grid[f] = sub[order[f]];
  const PointGrid pg{grid, nU, nV};
  const int degU = std::min(3, nU - 1);
  const int degV = std::min(3, nV - 1);
  const int ctrlU = std::min(prm.freeformCtrl, nU);
  const int ctrlV = std::min(prm.freeformCtrl, nV);
  const SurfaceFitResult sr = approximateSurface(pg, ctrlU, ctrlV, degU, degV);
  if (!sr.ok) return false;
  if (!(sr.rmsError <= prm.tol)) return false;  // no widening — must genuinely fit
  surfOut = sr.surface;
  rmsOut = sr.rmsError;
  return true;
}

// ── Classify + fit one index set into a SegmentRegion ────────────────────────
// The 3-way decision: analytic primitive (detectPrimitive, within tol) → freeform patch
// (approximateSurface, RMS ≤ tol) → honest decline. `region` is assumed sorted and its
// size already checked ≥ minRegion by the caller (a smaller set is declined directly).
SegmentRegion classifyRegion(std::span<const Point3> pts, const std::vector<int>& region,
                             const SegmentParams& prm) {
  SegmentRegion sr;
  sr.inliers = region;

  // 1) Analytic primitive?
  const PrimitiveDetection d = detectOnSubset(pts, region, prm.tol);
  if (d.ok) {
    switch (d.type) {
      case PrimitiveType::Plane:    sr.kind = RegionKind::Plane;    sr.plane = d.plane;       break;
      case PrimitiveType::Sphere:   sr.kind = RegionKind::Sphere;   sr.sphere = d.sphere;     break;
      case PrimitiveType::Cylinder: sr.kind = RegionKind::Cylinder; sr.cylinder = d.cylinder; break;
      case PrimitiveType::Cone:     sr.kind = RegionKind::Cone;     sr.cone = d.cone;         break;
      default:                      sr.kind = RegionKind::Declined;                           break;
    }
    if (sr.kind != RegionKind::Declined) {
      sr.rms = d.rms;
      return sr;
    }
  }

  // 2) Freeform patch?
  BsplineSurfaceData surf;
  double frms = 0.0;
  if (fitFreeform(pts, region, prm, surf, frms)) {
    sr.kind = RegionKind::Freeform;
    sr.surface = std::move(surf);
    sr.rms = frms;
    return sr;
  }

  // 3) Neither → honest decline (never a forced primitive).
  sr.kind = RegionKind::Declined;
  return sr;
}

// Commit a region into the result, tallying assigned / declined.
void commit(SegmentationResult& out, SegmentRegion&& sr) {
  const int m = static_cast<int>(sr.inliers.size());
  if (sr.kind == RegionKind::Declined) out.declinedCount += m;
  else out.assignedCount += m;
  out.regions.push_back(std::move(sr));
}

// ═════════════════════════════════════════════════════════════════════════════
// ROBUST PATH — noisy / outlier-laden scans.
//
// The noise-free machinery above grows regions on EXACT residuals. Real scans carry
// Gaussian noise σ and gross outliers, so an exact frontier both over-segments (noise
// splits a face) and swallows outliers into the fit. The robust path replaces the
// primitive fit inside region growing with an OUTLIER-REJECTING estimator and grows on a
// noise-RELATIVE band, then isolates points that fit no region into an OUTLIER set.
// Everything here is deterministic: the RNG is seeded by the region's own smallest index,
// never a global clock — the output is byte-reproducible.
// ═════════════════════════════════════════════════════════════════════════════
namespace robust {

// Deterministic xorshift64* — seeded by a data-derived value (never a clock). Yields a
// bounded index for RANSAC subset selection.
struct Rng {
  std::uint64_t s;
  explicit Rng(std::uint64_t seed) : s(seed ? seed : 0x9e3779b97f4a7c15ULL) {}
  std::uint64_t next() {
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    return s * 0x2545f4914f6cdd1dULL;
  }
  int index(int n) { return static_cast<int>(next() % static_cast<std::uint64_t>(n)); }
};

// Median of a scratch vector (mutates it via nth_element). Empty → 0.
double medianOf(std::vector<double>& v) {
  if (v.empty()) return 0.0;
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  const double hi = v[mid];
  if (v.size() % 2 == 1) return hi;
  std::nth_element(v.begin(), v.begin() + (mid - 1), v.end());
  return 0.5 * (hi + v[mid - 1]);
}

// Robust noise σ from a set of signed residuals: σ ≈ 1.4826 · MAD (median absolute
// deviation), the standard consistent Gaussian-σ estimator, immune to the outlier tail.
double sigmaFromResiduals(std::vector<double> resid) {
  if (resid.empty()) return 0.0;
  std::vector<double> a = resid;
  const double med = medianOf(a);
  std::vector<double> dev;
  dev.reserve(resid.size());
  for (double r : resid) dev.push_back(std::fabs(r - med));
  return 1.4826 * medianOf(dev);
}

// RMS of the residuals of an index subset to a detected primitive (inlier RMS: caller
// passes only the inliers). Empty → 0.
double subsetRms(std::span<const Point3> pts, const std::vector<int>& idx,
                 const PrimitiveDetection& d) {
  if (idx.empty()) return 0.0;
  double s2 = 0.0;
  for (int i : idx) {
    const double r = residualToPrimitive(pts[i], d);
    s2 += r * r;
  }
  return std::sqrt(s2 / static_cast<double>(idx.size()));
}

// Outcome of a robust fit over one region: the winning primitive, its consensus inliers
// (region-local absorbed into ORIGINAL indices), and the honest inlier RMS.
struct RobustFit {
  PrimitiveDetection det{};
  std::vector<int> inliers;   ///< ORIGINAL indices consistent with `det` within band
  double inlierRms = 0.0;     ///< TRUE RMS over the inliers (≈ σ), never claimed exact
  bool ok = false;
};

// detectPrimitive on an ORIGINAL-index subset, RELATIVE tolerance derived from the subset
// extent and an ABSOLUTE band (mirrors detectOnSubset but with an explicit band so the
// robust path can pass a noise-scaled tolerance).
PrimitiveDetection detectOnSubsetBand(std::span<const Point3> pts,
                                      const std::vector<int>& idx, double band) {
  const std::vector<Point3> sub = gather(pts, idx);
  const double extent = subsetExtent(pts, idx);
  return detectPrimitive(sub, band / extent);
}

// Inliers of a detected primitive over a candidate index set, within an absolute band.
std::vector<int> inliersWithin(std::span<const Point3> pts, const std::vector<int>& cand,
                               const PrimitiveDetection& d, double band) {
  std::vector<int> in;
  in.reserve(cand.size());
  for (int i : cand)
    if (residualToPrimitive(pts[i], d) <= band) in.push_back(i);
  return in;
}

// RANSAC-style consensus + IRLS/Huber (trimmed) refine over one candidate index set.
//
// CONSENSUS: draw `ransacIters` deterministic minimal-ish subsets, fit each with
// detectPrimitive, score by the number of inliers within `band`; keep the maximal-
// consensus fit. This rejects gross outliers — a subset polluted by an outlier scores
// poorly and loses. REFINE (IRLS/Huber): re-fit detectPrimitive on the consensus inliers,
// recompute residuals, re-select inliers at the Huber threshold `band`, and iterate; this
// is hard-weight IRLS (weights 1 inside band, 0 outside) and converges to the robust fit.
// Reports the TRUE inlier RMS. HONEST-DECLINE (ok=false) when no fit explains ≥
// minInlierFrac of the candidate set.
RobustFit robustFit(std::span<const Point3> pts, const std::vector<int>& cand,
                    double band, const RobustSegmentParams& prm, std::uint64_t seed,
                    double sigma) {
  RobustFit out;
  const int m = static_cast<int>(cand.size());
  // Robust floor: a higher-DOF primitive fits ANY 6 points exactly, so require enough
  // points that an exact fit of gross outliers is statistically implausible.
  const int minReg = std::max(prm.base.minRegion, prm.robustMinRegion);
  if (m < minReg) return out;

  // Subset size: enough to define any primitive robustly (≥ 6 for cyl/cone) with a small
  // margin, capped at the candidate size. Larger subsets are more stable than strictly
  // minimal ones and keep detectPrimitive's type discrimination reliable.
  const int subSize = std::min(m, std::max(prm.base.minRegion + 2, 8));

  Rng rng(seed);
  std::vector<int> bestInliers;
  PrimitiveDetection bestDet;
  std::size_t bestCount = 0;

  for (int it = 0; it < prm.ransacIters; ++it) {
    // Draw a DISTINCT random subset of `cand` (deterministic).
    std::vector<int> pick;
    pick.reserve(static_cast<std::size_t>(subSize));
    std::vector<char> taken(static_cast<std::size_t>(m), 0);
    int guard = 0;
    while (static_cast<int>(pick.size()) < subSize && guard < 20 * subSize) {
      const int r = rng.index(m);
      ++guard;
      if (!taken[r]) { taken[r] = 1; pick.push_back(cand[r]); }
    }
    if (static_cast<int>(pick.size()) < prm.base.minRegion) continue;

    const PrimitiveDetection d = detectOnSubsetBand(pts, pick, band);
    if (!d.ok) continue;
    const std::vector<int> in = inliersWithin(pts, cand, d, band);
    if (in.size() > bestCount) {
      bestCount = in.size();
      bestInliers = in;
      bestDet = d;
    }
  }

  if (bestInliers.size() < static_cast<std::size_t>(minReg)) return out;

  // IRLS / Huber (hard-weight) refine on the consensus inliers.
  std::vector<int> inl = bestInliers;
  PrimitiveDetection det = bestDet;
  for (int it = 0; it < prm.huberIters; ++it) {
    const PrimitiveDetection refit = detectOnSubsetBand(pts, inl, band);
    if (!refit.ok) break;
    std::vector<int> reIn = inliersWithin(pts, cand, refit, band);
    if (reIn.size() < static_cast<std::size_t>(minReg)) break;
    det = refit;
    const bool converged = (reIn.size() == inl.size());
    inl = std::move(reIn);
    if (converged) break;
  }

  // Must explain a real majority of the candidate set AND meet the robust size floor —
  // else this is not a coherent region (honest decline; the caller then falls to freeform
  // / outliers). Requiring more points than the primitive DOF prevents a handful of gross
  // outliers from being fitted EXACTLY by a high-DOF cone/cylinder.
  if (static_cast<int>(inl.size()) < minReg) return out;
  if (static_cast<double>(inl.size()) <
      prm.minInlierFrac * static_cast<double>(m))
    return out;

  std::sort(inl.begin(), inl.end());
  out.det = det;
  out.inliers = std::move(inl);
  out.inlierRms = subsetRms(pts, out.inliers, det);

  // HONESTY: a genuine noisy surface has RMS commensurate with the noise (≈ σ). An
  // implausibly-perfect fit (RMS ≪ σ) over a set this size is OVERFITTING a coincidental
  // configuration (e.g. a cone threaded through near-collinear outliers). When σ is known/
  // estimated > 0, reject a primitive whose RMS is a tiny fraction of σ — decline rather
  // than report a fabricated near-exact primitive. (On truly clean data σ≈0 and this guard
  // is inert, so the noise-free reduction is preserved.)
  if (sigma > 0.0 && out.inlierRms < 1e-3 * sigma) return {};

  out.ok = true;
  return out;
}

// Solve a small symmetric positive-(semi)definite normal system A x = b (A is c×c, row-
// major) by Gaussian elimination with partial pivoting. Self-contained (no facade), used
// only for the tiny 6×6 local-quadric fit in the σ estimator. Returns false if singular.
bool solveSmall(std::vector<double> A, std::vector<double> b, int c, std::vector<double>& x) {
  for (int col = 0; col < c; ++col) {
    int piv = col;
    double best = std::fabs(A[col * c + col]);
    for (int r = col + 1; r < c; ++r) {
      const double v = std::fabs(A[r * c + col]);
      if (v > best) { best = v; piv = r; }
    }
    if (best < 1e-300) return false;
    if (piv != col) {
      for (int k = 0; k < c; ++k) std::swap(A[col * c + k], A[piv * c + k]);
      std::swap(b[col], b[piv]);
    }
    const double d = A[col * c + col];
    for (int r = 0; r < c; ++r) {
      if (r == col) continue;
      const double f = A[r * c + col] / d;
      if (f == 0.0) continue;
      for (int k = col; k < c; ++k) A[r * c + k] -= f * A[col * c + k];
      b[r] -= f * b[col];
    }
  }
  x.assign(static_cast<std::size_t>(c), 0.0);
  for (int i = 0; i < c; ++i) x[i] = b[i] / A[i * c + i];
  return true;
}

// Estimate the scan noise σ from LOCAL QUADRIC residuals. A plane residual over a k-NN
// patch CANNOT separate noise from curvature (the sagitta of a curved surface looks like
// roughness), so on a CLEAN cylinder/sphere a plane estimator reports a spurious σ and the
// band no longer collapses to base.tol. Instead, over each point's neighbourhood we fit a
// local QUADRIC height field z = a + b·u + c·w + d·u² + e·uw + f·w² in the point's tangent
// frame; the quadric ABSORBS the local curvature, so the fit residual is the pure noise.
// σ ≈ 1.4826·MAD of those residuals — the consistent Gaussian estimator, immune to the
// outlier tail. On CLEAN data (curved or flat) the residuals are ~0 so σ≈0 and the band
// collapses to base.tol, reproducing the noise-free path; on noisy data σ ≈ the true noise.
double estimateSigma(std::span<const Point3> pts,
                     const std::vector<std::vector<int>>& adj,
                     const RobustSegmentParams& prm) {
  if (prm.sigma > 0.0) return prm.sigma;
  const int n = static_cast<int>(pts.size());
  if (n < 8) return 0.0;

  std::vector<double> resids;
  resids.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    // Neighbourhood: i plus its k-NN. Need ≥ 6 points to fit the 6-term quadric.
    std::vector<int> ball{i};
    for (int nb : adj[i]) ball.push_back(nb);
    const int m = static_cast<int>(ball.size());
    if (m < 8) continue;  // enough over-determination for a stable quadric

    // Local tangent frame from the neighbourhood's TLS plane.
    const std::vector<Point3> sub = gather(pts, ball);
    const PlaneFit pf = fitPlane(sub);
    if (!pf.ok) continue;
    const Vec3 nrm = pf.normal.vec();
    Vec3 t = (std::fabs(nrm.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 uu = cross(nrm, t);
    uu = uu / norm(uu);
    const Vec3 ww = cross(nrm, uu);

    // Assemble the 6×6 normal equations for z(u,w) = θ·[1,u,w,u²,uw,w²].
    std::vector<double> AtA(36, 0.0), Atb(6, 0.0);
    for (int k = 0; k < m; ++k) {
      const Vec3 rel = sub[k] - pf.centroid;
      const double u = dot(rel, uu), w = dot(rel, ww), z = dot(rel, nrm);
      const double phi[6] = {1.0, u, w, u * u, u * w, w * w};
      for (int a = 0; a < 6; ++a) {
        Atb[a] += phi[a] * z;
        for (int b = 0; b < 6; ++b) AtA[a * 6 + b] += phi[a] * phi[b];
      }
    }
    std::vector<double> theta;
    if (!solveSmall(AtA, Atb, 6, theta)) continue;

    // Residual of the CENTER point (k for i is index 0 in `ball`/`sub`).
    const Vec3 rel0 = sub[0] - pf.centroid;
    const double u0 = dot(rel0, uu), w0 = dot(rel0, ww), z0 = dot(rel0, nrm);
    const double pred = theta[0] + theta[1] * u0 + theta[2] * w0 + theta[3] * u0 * u0 +
                        theta[4] * u0 * w0 + theta[5] * w0 * w0;
    resids.push_back(std::fabs(z0 - pred));
  }
  if (resids.empty()) return 0.0;
  return sigmaFromResiduals(std::move(resids));
}

// Build a SegmentRegion from a robust fit's winning primitive + inliers.
SegmentRegion regionFromRobustFit(const RobustFit& rf) {
  SegmentRegion sr;
  sr.inliers = rf.inliers;
  switch (rf.det.type) {
    case PrimitiveType::Plane:    sr.kind = RegionKind::Plane;    sr.plane = rf.det.plane;       break;
    case PrimitiveType::Sphere:   sr.kind = RegionKind::Sphere;   sr.sphere = rf.det.sphere;     break;
    case PrimitiveType::Cylinder: sr.kind = RegionKind::Cylinder; sr.cylinder = rf.det.cylinder; break;
    case PrimitiveType::Cone:     sr.kind = RegionKind::Cone;     sr.cone = rf.det.cone;         break;
    default:                      sr.kind = RegionKind::Declined;                                break;
  }
  sr.rms = rf.inlierRms;  // TRUE robust inlier RMS — never claimed exact
  return sr;
}

// Robust seeded region growing over one component. Seed generously, robust-fit a
// primitive (RANSAC+IRLS) on the seed ball, then grow the CONSENSUS inlier set outward by
// the noise band, re-fitting robustly so the primitive tracks the whole face. Returns the
// grown inlier set (ORIGINAL indices) plus the winning fit. `avail` marks component points
// not yet claimed. Grows only within `avail`.
RobustFit growRobustRegion(std::span<const Point3> pts,
                           const std::vector<std::vector<int>>& adj, int seed,
                           const std::vector<char>& avail, double band,
                           const RobustSegmentParams& prm, double sigma) {
  const int n = static_cast<int>(pts.size());
  const int seedTarget =
      std::max(4 * prm.base.minRegion, 2 * prm.base.kNeighbors);

  std::vector<char> inBall(static_cast<std::size_t>(n), 0);
  std::vector<int> ball{seed};
  inBall[seed] = 1;
  std::vector<int> frontier{seed};
  while (!frontier.empty() && static_cast<int>(ball.size()) < seedTarget) {
    const int cur = frontier.front();
    frontier.erase(frontier.begin());
    for (int nb : adj[cur])
      if (avail[nb] && !inBall[nb]) {
        inBall[nb] = 1;
        ball.push_back(nb);
        frontier.push_back(nb);
        if (static_cast<int>(ball.size()) >= seedTarget) break;
      }
  }
  if (static_cast<int>(ball.size()) < prm.base.minRegion) return {};

  RobustFit rf = robustFit(pts, ball, band, prm, static_cast<std::uint64_t>(seed) + 1, sigma);
  if (!rf.ok) return rf;

  // Mark the current region membership and grow the frontier under the (robust) fit.
  std::vector<char> inRegion(static_cast<std::size_t>(n), 0);
  for (int i : rf.inliers) inRegion[i] = 1;

  bool changed = true;
  while (changed) {
    changed = false;
    std::vector<int> front;
    for (int r : rf.inliers)
      for (int nb : adj[r])
        if (avail[nb] && !inRegion[nb]) front.push_back(nb);
    std::sort(front.begin(), front.end());
    front.erase(std::unique(front.begin(), front.end()), front.end());

    std::vector<int> grow = rf.inliers;
    for (int cand : front)
      if (residualToPrimitive(pts[cand], rf.det) <= band) {
        grow.push_back(cand);
        inRegion[cand] = 1;
        changed = true;
      }
    if (changed) {
      std::sort(grow.begin(), grow.end());
      RobustFit refit = robustFit(pts, grow, band, prm,
                                  static_cast<std::uint64_t>(seed) + 1, sigma);
      if (refit.ok) {
        // Adopt the refit; sync region membership to its (possibly trimmed) inliers.
        std::fill(inRegion.begin(), inRegion.end(), static_cast<char>(0));
        for (int i : refit.inliers) inRegion[i] = 1;
        rf = std::move(refit);
      } else {
        // Refit collapsed — keep the grown set under the previous fit.
        rf.inliers = std::move(grow);
        rf.inlierRms = subsetRms(pts, rf.inliers, rf.det);
      }
    }
  }
  return rf;
}

}  // namespace robust

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public entry point.
// ─────────────────────────────────────────────────────────────────────────────
SegmentationResult segmentAndFit(std::span<const Point3> points,
                                 std::span<const Vec3> /*normals*/,
                                 const SegmentParams& params) {
  SegmentationResult out;
  const int n = static_cast<int>(points.size());
  if (n == 0) return out;

  const std::vector<std::vector<int>> adj = buildAdjacency(points, params.kNeighbors);

  // Partition by connectivity first: a scanned composite splits into components at the
  // spatial gaps between faces. Each component is classified as a WHOLE (a pure surface
  // is one region); only a HETEROGENEOUS component (whole-component fit declines) is
  // sub-segmented by seeded region growing.
  const std::vector<std::vector<int>> comps = connectedComponents(n, adj);

  for (const std::vector<int>& comp : comps) {
    // Too small to classify → honest decline of the whole component.
    if (static_cast<int>(comp.size()) < params.minRegion) {
      SegmentRegion sr;
      sr.kind = RegionKind::Declined;
      sr.inliers = comp;
      commit(out, std::move(sr));
      continue;
    }

    // Whole-component classification first (the common case: one component = one surface).
    SegmentRegion whole = classifyRegion(points, comp, params);
    if (whole.kind != RegionKind::Declined) {
      commit(out, std::move(whole));
      continue;
    }

    // Heterogeneous / declined component → sub-segment by seeded region growing over the
    // component's own points (avail marks component points not yet claimed here).
    std::vector<char> avail(static_cast<std::size_t>(n), 0);
    for (int i : comp) avail[i] = 1;

    for (int seed : comp) {
      if (!avail[seed]) continue;
      std::vector<int> region = growRegion(points, adj, seed, avail, params);
      region.erase(std::remove_if(region.begin(), region.end(),
                                  [&](int i) { return avail[i] == 0; }),
                   region.end());
      if (region.empty()) { avail[seed] = 0; continue; }
      std::sort(region.begin(), region.end());
      for (int i : region) avail[i] = 0;  // claim within this component

      if (static_cast<int>(region.size()) < params.minRegion) {
        SegmentRegion sr;
        sr.kind = RegionKind::Declined;
        sr.inliers = std::move(region);
        commit(out, std::move(sr));
        continue;
      }
      commit(out, classifyRegion(points, region, params));
    }
  }

  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Robust entry point — noisy / outlier-laden scans.
//
// Same connectivity-first structure as segmentAndFit, but every classification goes
// through the robust (RANSAC + IRLS) fitter on a noise-scaled band, and points that fit
// no region within the band are collected into an explicit OUTLIER set rather than
// force-assigned. On a clean cloud (σ estimated ≈ 0) the band collapses to base.tol and
// the robust path reproduces the noise-free result.
// ─────────────────────────────────────────────────────────────────────────────
SegmentationResult segmentAndFitRobust(std::span<const Point3> points,
                                       std::span<const Vec3> /*normals*/,
                                       const RobustSegmentParams& params) {
  SegmentationResult out;
  const int n = static_cast<int>(points.size());
  if (n == 0) return out;

  const std::vector<std::vector<int>> adj = buildAdjacency(points, params.base.kNeighbors);

  // Estimate the scan noise σ (or use the caller-supplied σ), then derive the region-
  // membership band and the outlier threshold. The band is FLOORED by base.tol so a clean
  // cloud (σ≈0) reproduces the noise-free frontier exactly.
  const double sigma = robust::estimateSigma(points, adj, params);
  out.noiseSigma = sigma;
  const double band = std::max(params.base.tol, params.bandSigma * sigma);
  const double outlierBand = std::max(params.base.tol, params.outlierSigma * sigma);

  const std::vector<std::vector<int>> comps = connectedComponents(n, adj);

  // Track final assignment so leftover points become OUTLIERS (never force-assigned).
  std::vector<char> assigned(static_cast<std::size_t>(n), 0);

  for (const std::vector<int>& comp : comps) {
    if (static_cast<int>(comp.size()) < params.base.minRegion) {
      // Too small to fit anything robustly → outliers (honest; not a forced region).
      for (int i : comp) { out.outliers.push_back(i); assigned[i] = 1; }
      continue;
    }

    // NOISE-FREE REDUCTION (byte-compatible): try the EXACT noise-free classification at
    // base.tol first. If the whole component is cleanly a single primitive / freeform patch
    // within the tight tolerance, commit it verbatim (identical to segmentAndFit) and skip
    // the robust machinery — a clean component is never re-fitted through RANSAC, so its
    // params match the noise-free path to ≤1e-6 and it contributes NO outliers.
    {
      SegmentRegion exact = classifyRegion(points, comp, params.base);
      if (exact.kind != RegionKind::Declined &&
          exact.inliers.size() == comp.size()) {
        for (int i : exact.inliers) assigned[i] = 1;
        commit(out, std::move(exact));
        continue;
      }
    }

    // Whole-component robust fit first (the common case: one component = one noisy face).
    const robust::RobustFit whole = robust::robustFit(
        points, comp, band, params, static_cast<std::uint64_t>(comp[0]) + 1, sigma);
    if (whole.ok && whole.det.type != PrimitiveType::Freeform) {
      SegmentRegion sr = robust::regionFromRobustFit(whole);
      if (sr.kind != RegionKind::Declined) {
        for (int i : sr.inliers) assigned[i] = 1;
        commit(out, std::move(sr));
        // Component points NOT in the winning consensus: try to sub-segment (below) via
        // the leftover loop; anything still unassigned there becomes an outlier.
      }
    }

    // Sub-segment the component's still-unassigned points by robust seeded growing. This
    // handles heterogeneous components (multiple faces) AND the residual of a whole-fit
    // (the outlier tail / a second face). `avail` marks component points not yet claimed.
    std::vector<char> avail(static_cast<std::size_t>(n), 0);
    for (int i : comp)
      if (!assigned[i]) avail[i] = 1;

    for (int seed : comp) {
      if (!avail[seed]) continue;
      robust::RobustFit rf =
          robust::growRobustRegion(points, adj, seed, avail, band, params, sigma);
      // Restrict to still-available points (defensive; growth respects avail already).
      rf.inliers.erase(std::remove_if(rf.inliers.begin(), rf.inliers.end(),
                                      [&](int i) { return avail[i] == 0; }),
                       rf.inliers.end());

      if (!rf.ok || static_cast<int>(rf.inliers.size()) < params.base.minRegion) {
        avail[seed] = 0;  // this seed cannot anchor a region; leave for outlier collection
        continue;
      }
      SegmentRegion sr = robust::regionFromRobustFit(rf);
      if (sr.kind == RegionKind::Declined) { avail[seed] = 0; continue; }
      for (int i : rf.inliers) { avail[i] = 0; assigned[i] = 1; }
      commit(out, std::move(sr));
    }
  }

  // Everything not assigned to a region is an OUTLIER (consistent with no region within
  // the robust band) — isolated, never shoe-horned into the nearest region. A point is
  // additionally confirmed as an outlier only if it is beyond the (wider) outlier band of
  // EVERY committed primitive region; if it happens to sit within a committed region's
  // outlierBand it is folded back into that region's inliers (a genuine inlier the
  // consensus missed), keeping the false-positive rate low.
  for (int i = 0; i < n; ++i) {
    if (assigned[i]) continue;
    int host = -1;
    double bestR = outlierBand;
    for (int r = 0; r < static_cast<int>(out.regions.size()); ++r) {
      const SegmentRegion& reg = out.regions[r];
      PrimitiveDetection d;
      d.ok = true;
      switch (reg.kind) {
        case RegionKind::Plane:    d.type = PrimitiveType::Plane;    d.plane = reg.plane;       break;
        case RegionKind::Sphere:   d.type = PrimitiveType::Sphere;   d.sphere = reg.sphere;     break;
        case RegionKind::Cylinder: d.type = PrimitiveType::Cylinder; d.cylinder = reg.cylinder; break;
        case RegionKind::Cone:     d.type = PrimitiveType::Cone;     d.cone = reg.cone;         break;
        default: continue;  // Freeform / Declined regions do not adopt stray points here
      }
      const double res = residualToPrimitive(points[i], d);
      if (res <= bestR) { bestR = res; host = r; }
    }
    if (host >= 0) {
      out.regions[host].inliers.push_back(i);
      assigned[i] = 1;
      ++out.assignedCount;
    } else {
      out.outliers.push_back(i);
    }
  }

  std::sort(out.outliers.begin(), out.outliers.end());
  out.outlierCount = static_cast<int>(out.outliers.size());
  // Keep each region's inliers sorted after any late adoptions.
  for (SegmentRegion& reg : out.regions)
    std::sort(reg.inliers.begin(), reg.inliers.end());
  return out;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI
