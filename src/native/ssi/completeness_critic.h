// SPDX-License-Identifier: Apache-2.0
//
// completeness_critic.h — SSI Stage S4-f: the adaptive completeness-critic's coverage map.
//
// The fixed-resolution S2 seeder (seeding.h) emits a candidate region only once BOTH patches
// fall below `minPatchFrac`; a small intersection loop that lives entirely inside one leaf
// cell yields at most one center-seeded refine, which converges to at most ONE branch — so a
// second/smaller loop sharing that cell is MISSED (the acknowledged recall hole). The S4-f
// completeness critic RE-SEEDS finer in the regions no traced curve covers, loop-until-dry,
// to recover such loops. This header is the critic's TARGETING primitive: a coarse param-grid
// OCCUPANCY over surface A, marked from the traced WLines' footprints, and the list of grid
// cells with NO traced curve (the re-seed targets).
//
// HONEST FRAMING (must not be overclaimed): the coverage map is a RE-SEED TARGET, NOT a
// completeness proof. Marking a cell "covered" means a traced curve passed through it at the
// current polyline resolution; an UNCOVERED cell is where the critic should look next. Below
// ANY grid / subdivision resolution a smaller loop can still hide inside a "covered" cell, so
// the critic's recall is MEASURED + ASYMPTOTIC, never a guarantee (TraceSet.completenessResidual
// is always true). This header touches only src/native/math + the SSI result structs — it is
// OCCT-FREE and SUBSTRATE-FREE (no native-numerics), so it compiles without CYBERCAD_HAS_NUMSCI
// (the re-seed loop that USES it, in marching.cpp, is guarded like the rest of the tracer).
//
#ifndef CYBERCAD_NATIVE_SSI_COMPLETENESS_CRITIC_H
#define CYBERCAD_NATIVE_SSI_COMPLETENESS_CRITIC_H

#include "native/ssi/marching.h"       // WLine / TraceSet (traced polylines' footprints)
#include "native/ssi/patch_bounds.h"   // ParamBox

#include <vector>

namespace cybercad::native::ssi::critic {

// ─────────────────────────────────────────────────────────────────────────────
// Coverage — a gridN×gridN boolean occupancy over surface A's (u,v) domain, marked
// where any traced WLine node lands. `domainA` is A's full param box; `gridN` the grid
// resolution per direction. `covered[i*gridN + j]` is true iff a traced curve passed
// through cell (i,j). (We map footprints on A only: A's domain is where the re-seed's
// initial-grid pre-split runs, and every WLine node carries its A params.)
// ─────────────────────────────────────────────────────────────────────────────
struct Coverage {
  ParamBox domainA{};
  int gridN = 0;
  std::vector<char> covered{};  ///< gridN*gridN occupancy (row-major, i over U)

  bool cellCovered(int i, int j) const {
    if (i < 0 || j < 0 || i >= gridN || j >= gridN) return true;  // out of grid → treat as covered
    return covered[static_cast<std::size_t>(i) * gridN + j] != 0;
  }
};

/// Map param cell indices for a param (u,v) into the gridN grid over `dom` (clamped).
inline void cellOf(const ParamBox& dom, int gridN, double u, double v, int& i, int& j) {
  const double du = dom.du() != 0.0 ? dom.du() : 1.0;
  const double dv = dom.dv() != 0.0 ? dom.dv() : 1.0;
  i = static_cast<int>((u - dom.u0) / du * gridN);
  j = static_cast<int>((v - dom.v0) / dv * gridN);
  if (i < 0) i = 0; else if (i >= gridN) i = gridN - 1;
  if (j < 0) j = 0; else if (j >= gridN) j = gridN - 1;
}

/// Build the coverage occupancy from the traced WLines. Marks every cell a node lands in,
/// PLUS its 8-neighbours (a traced curve near a cell edge covers the adjacent cell too — a
/// conservative dilation so the critic does not re-seed a sliver next to an existing curve
/// and rediscover the same branch). Only Closed / BoundaryExit / BranchArc lines count as
/// coverage (a Failed / NearTangent stub is not a completed curve).
inline Coverage coverageOf(const std::vector<WLine>& lines, const ParamBox& domainA, int gridN) {
  Coverage cov;
  cov.domainA = domainA;
  cov.gridN = gridN > 0 ? gridN : 1;
  cov.covered.assign(static_cast<std::size_t>(cov.gridN) * cov.gridN, 0);
  for (const WLine& w : lines) {
    if (w.status == TraceStatus::Failed || w.status == TraceStatus::NearTangent) continue;
    for (const WLinePoint& p : w.points) {
      int i = 0, j = 0;
      cellOf(domainA, cov.gridN, p.u1, p.v1, i, j);
      for (int di = -1; di <= 1; ++di)
        for (int dj = -1; dj <= 1; ++dj) {
          const int ii = i + di, jj = j + dj;
          if (ii >= 0 && jj >= 0 && ii < cov.gridN && jj < cov.gridN)
            cov.covered[static_cast<std::size_t>(ii) * cov.gridN + jj] = 1;
        }
    }
  }
  return cov;
}

/// The A-domain ParamBoxes of the grid cells with NO traced curve — the critic's re-seed
/// targets. Each box is one grid cell of `cov`; the re-seed pairs it with B's full domain and
/// lets the subdivision prune. Empty when every cell is covered (nothing left to look at at
/// this grid resolution — which is NOT a proof the intersection is complete; see the header).
inline std::vector<ParamBox> uncoveredBoxes(const Coverage& cov) {
  std::vector<ParamBox> boxes;
  const double du = cov.domainA.du() / cov.gridN;
  const double dv = cov.domainA.dv() / cov.gridN;
  for (int i = 0; i < cov.gridN; ++i)
    for (int j = 0; j < cov.gridN; ++j) {
      if (cov.cellCovered(i, j)) continue;
      ParamBox b;
      b.u0 = cov.domainA.u0 + du * i;
      b.u1 = cov.domainA.u0 + du * (i + 1);
      b.v0 = cov.domainA.v0 + dv * j;
      b.v1 = cov.domainA.v0 + dv * (j + 1);
      boxes.push_back(b);
    }
  return boxes;
}

}  // namespace cybercad::native::ssi::critic

#endif  // CYBERCAD_NATIVE_SSI_COMPLETENESS_CRITIC_H
