## Why

The native rolling-ball full round in `src/engine/occt/occt_full_round_fillet.cpp`
consumes the middle strip face by carving with a tangent cylinder, but its
eligibility gate (`rollingBallEligible`) only accepts planar neighbours whose
outward normals are (anti-)PARALLEL (`nL·nR < -0.98`): a constant-radius strip on
the mid-plane. Every other planar case — two walls meeting at a real dihedral
angle (a chamfered rib, a splayed pocket wall, a draft-angled boss) — silently
takes the honest-but-lower-fidelity edge-fillet fallback and is recorded deferred,
even though a constant-radius rolling ball provably EXISTS for it.

Two non-parallel planes admit a constant-radius rolling ball just as parallel
walls do. A ball of radius `r` tangent to both planes has its centre on the
interior bisector of the dihedral; rolling it along the crease sweeps a CYLINDER
of radius `r` tangent to BOTH planes. Its axis runs along `n1 × n2` (the
planes' intersection direction) and passes through the point on the interior
bisector at perpendicular distance `r` from each plane. That cylinder is exactly
the blend surface — G1-tangent to both neighbours along the two seams — so the
same "carve the corner slab minus the tangent cylinder" construction that already
works for parallel walls extends directly to the dihedral case. Closing this gap
converts a large, common family of ribs from a partial fillet into a true
face-consuming full round with no ABI change.

## What Changes

- **Generalize the tangent-cylinder construction to the planar dihedral case.**
  Replace the parallel-only axis derivation with the general one:
  - axis DIRECTION = `normalize(n1 × n2)` (the crease / plane-intersection
    direction; for two side walls of a rib this is the strip direction),
  - axis LOCATION = the point on the interior (valley-side) dihedral bisector at
    perpendicular distance `r` from BOTH neighbour planes, solved from the two
    plane equations, with `r` fixed by the strip geometry exactly as the parallel
    case picks it (half the perpendicular strip width / the seam-edge offset).
  The blend is the portion of that cylinder spanning the two seam edges; the
  middle face is consumed by the same slab−cylinder carve so the result is
  watertight and G1-tangent to both neighbour planes along the seams.
- **Widen `rollingBallEligible` to accept non-parallel PLANAR neighbours.** Both
  faces must be planar and the crease must be well-defined (`|n1 × n2|` above a
  floor for the non-parallel branch; the near-anti-parallel branch keeps the
  existing mid-plane derivation). The parallel path is preserved as the limiting
  case and its numbers are unchanged.
- **Keep the honest fallback for curved neighbours.** Truly non-planar (curved)
  neighbours remain OUT of scope: they still take the valid standard edge-fillet
  fallback and are recorded deferred with the measured tangency gap / dihedral
  angle. No trivially-true check, no faked G1 or face-consumption claim.
- **Add an on-simulator non-parallel-rib check** to
  `tests/sim/checks_full_round_fillet.cpp` (run by `phase3_suite.cpp` via
  `scripts/run-sim-phase3-suite.sh`) asserting REAL properties for the dihedral
  case: `BRepCheck_Analyzer::IsValid`, watertight, the middle face id no longer
  resolves, a cylindrical blend whose axis runs along `n1 × n2`, and sampled seam
  normals that agree with BOTH neighbour normals within `cos(~1°)`.

This change is **internal / additive only**: no new C ABI, no `cc_*` signature
change, no POD-struct-layout change. `cc_full_round_fillet` /
`cc_full_round_fillet_faces` already exist and keep their signatures; only the OCCT
adapter's blend construction and its sim check change. The path is compiled only
under `#ifdef CYBERCAD_HAS_OCCT`; the host stub build stays a safe no-op.

## Capabilities

### New Capabilities
<!-- none — this change adds no new capability; it upgrades the existing
     full-round-fillet requirement in place (see Modified Capabilities). -->

### Modified Capabilities
- `full-round-fillet`: the rolling-ball blend requirement is upgraded so the native
  tangent-cylinder full round SHALL also handle NON-PARALLEL planar neighbour walls
  (dihedral tangent cylinder, axis along `n1 × n2`, centre on the interior
  bisector at radius `r` from both planes), producing a valid watertight solid
  G1-tangent to both neighbours with the middle face consumed. Curved (non-planar)
  neighbours MAY still fall back to a valid standard fillet, recorded deferred. No
  ABI change — the two existing `cc_*` entry points are unchanged.

## Impact

- **ABI**: none. `cc_full_round_fillet` / `cc_full_round_fillet_faces` signatures
  and all POD struct layouts are unchanged; `tests/test_abi.cpp` is unaffected.
- **Code**: `src/engine/occt/occt_full_round_fillet.cpp` only — `rollingBallEligible`
  and the axis/location math inside `buildRollingBall` (extracted into a small
  planar-dihedral solver). The slab−cylinder carve, the fallback, and both entry
  points are otherwise untouched.
- **Tests**: a new non-parallel-rib fixture + checks in
  `tests/sim/checks_full_round_fillet.cpp`; the existing parallel-rib checks stay
  green with identical expected volume/axis numbers.
- **Build**: OCCT-guarded; host build (`/opt/homebrew/opt/llvm/bin/clang++`,
  OCCT off) stays green. Verified on the iOS simulator via
  `scripts/run-sim-phase3-suite.sh`.
- **Scope**: planar dihedral neighbours are now first-class; curved neighbours
  remain deferred (honest) — no capability is claimed that isn't achieved.
