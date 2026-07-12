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

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI
