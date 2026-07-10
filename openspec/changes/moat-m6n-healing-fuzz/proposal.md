# Proposal — moat-m6n-healing-fuzz (MOAT M6-breadth-14, the FOURTEENTH domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) currently has **thirteen** landed differential-fuzzing
domains — curved booleans, STEP round-trip, construction/loft/sweep, blends,
wrap/emboss, mesh mass-properties, geometry-services, transform chains,
reference/datum geometry, direct-modeling, transformed-boolean, section curves, and
orthographic HLR. The **shape-HEALING** layer — the native OCCT-free repair pipeline
`heal::healShell` (`src/native/heal/*`: tolerant sew, vertex unify, degenerate
removal, orientation flood-fill, bounded gap-bridge, planar-hole cap, short-edge
collapse, collinear-vertex removal) — has a *curated* parity harness
(`native_heal_parity.mm`) but **no seeded differential fuzzer** that drives *random*
deliberately-defected inputs and classifies every trial against the OCCT
`BRepBuilderAPI_Sewing → ShapeFix_Shell/Solid` oracle. Healing is the last-named
remaining M6 domain ("concave-shaft blends + healing remain (gates M8)"); this change
lands it.

The healing fuzzer is the **highest-value gap** for three reasons:

1. **It is explicitly named as the remaining domain** in the M6 roadmap row, and the
   immediately-prior transform track (`moat-m6h`) *deferred* it in favour of transform
   chains, leaving "a *fuzzer* for it as future work". This change closes that.
2. **A wrong heal is the most dangerous silent-wrong-result class in the whole
   kernel.** Healing repairs *malformed* input — a heuristic that returns a
   watertight-but-WRONG solid would hand the user a corrupted model they cannot detect
   (a wrong volume presented as a valid repair). The bar exists precisely to search the
   defect space for that.
3. **It is a STABLE, non-SSI-overlapping analysis surface.** The healer is deterministic
   header-only + `heal.cpp` topology/geometry (spatial-hash weld, closed-form planarity,
   flood-fill) with a landed curated harness; it shares NO code with the concurrent M1
   SSI (`src/native/ssi`) track (the healer never calls the marcher). No product code
   changes — this is test infrastructure only.

## What answers moat-m6h's objection (the third oracle)

`moat-m6h` deferred healing because "its 'correct' output is a heuristic agreement, so
a disagreement cannot be attributed without a third oracle." This change **supplies
that third oracle**: every fixture is a defect *injected into a solid whose exact
geometry is KNOWN by construction* (a unit-scale cube V=1, or an axis-aligned box /
N-gon prism with a closed-form volume + surface area). The defect families
(sew-jitter, flipped face, near-miss seam gap, short-edge split, collinear vertex,
planar hole) are **shape-preserving** — a *correct* heal reconstructs the ORIGINAL
solid exactly — so the closed-form volume/area of the undamaged solid is an
engine-independent ground-truth arbiter. A native `Healed` result is only AGREED when
it matches BOTH OCCT and the closed form; a native result that is watertight but
differs from the closed form is a genuine `DISAGREED` (the failure this harness exists
to catch), never excused.

## What changes

- **ADD** `tests/sim/native_healing_fuzz.mm` — a deterministic seeded (splitmix64 →
  xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per trial:
  generates a random VALID base solid (unit-scale cube / random axis-aligned box /
  random N-gon prism) with a closed-form volume+area; injects one random defect family
  (SEW-JITTER, FLIPPED-FACE, SEAM-GAP in/out-of-band, SHORT-EDGE split, COLLINEAR-VERT,
  MISSING-1-HOLE, MISSING-2-OPPOSITE, MISSING-2-ADJACENT, BEYOND-TOL-GAP); heals it
  BOTH ways — native `heal::healShell` (OCCT-FREE) with the family-appropriate opt-in
  flag, and OCCT `cyber::occt::sewAndFix` at the same tolerance — and classifies the
  trial AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED.
- **ADD** `scripts/run-sim-native-healing-fuzz.sh` — cloned from
  `run-sim-native-heal.sh` + the `run-sim-native-reference-geometry-fuzz.sh` ≥2-seed
  loop (fails if ANY seed fails), linking the native `heal`+`math` TUs + the single
  OCCT oracle TU `occt_shapefix.cpp` (+ `TKShHealing`/`TKTopAlgo` for Sewing/ShapeFix).
- **ADD** `native_healing_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own `main()`,
  `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×13 → ×14.

## The bar

DISAGREED == 0 and ORACLE_UNRELIABLE == 0 across ≥2 seeds, N≥60 trials/seed, each
defect family with ≥1 non-error trial. The **equal-or-more-conservative** contract is
the load-bearing invariant: native must NEVER emit a watertight solid that differs
from the known ground truth; an honest `Unhealed` decline is always safe (native being
*more conservative* than an aggressive OCCT repair is AGREED-by-decline, not a
failure). The FIXED tolerances are NEVER widened to force a pass.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). If the fuzzer surfaces a real native
  heal bug it is REPORTED, not fixed here; a native result more correct than OCCT at a
  numeric edge is classified ORACLE-INACCURATE (native vindicated), not DISAGREED.
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
