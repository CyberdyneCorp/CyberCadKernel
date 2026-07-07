# Proposal — moat-m6-differential-fuzzing

## Why

MOAT stage **M6** is the *completeness bar* — the discipline that gates `drop-occt`
(M8). Today the native curved-boolean path is verified against OCCT on a **fixed,
hand-picked fixture set** (`native_ssi_curved_boolean_parity.mm`: ~18 curated
`{pair, op}` cases). Hand-picked fixtures prove the cases we thought of; they cannot
prove the absence of a **silent wrong result** on the inputs we did *not* think of.
The M6 roadmap (openspec/MOAT-ROADMAP.md, "M6 — Robustness completeness bar") calls
for exactly this oracle: *"differential fuzzing — random valid inputs through both
native and OCCT, assert every non-native input honestly declines with a wrong-result
count of zero."*

The failure M6 exists to catch is precise and dangerous: a native boolean that
returns a **non-null, watertight** solid whose volume/area **disagrees** with the
OCCT oracle. That is a solid that *looks* correct (it passes the self-verify's
watertight gate) but is geometrically wrong — the kind of defect that survives a
hand-picked suite and would ship the day OCCT is unlinked. A native path that
*honestly declines* (returns NULL → OCCT fallback) is NOT a failure; it is the
correct behaviour for an unsupported input. The bar is therefore **zero silent wrong
results**, not "native handles everything".

This slice is **test/harness infrastructure**, not a geometry capability. It links
OCCT as the **oracle** (exactly like the existing parity `.mm` harnesses) and adds
NO OCCT to `src/native`. It should land: it turns "native-pass = 18 on curated
fixtures" into "N seeded random-*valid* inputs, zero silent wrong results", which is
the measured discipline `drop-occt` ultimately needs.

## What Changes

1. **A new deterministic, seeded random-valid input generator** for the recognised
   curved-boolean operand families (`cyl`, `sphere`, `cone`, coaxial / orthogonal
   placements), living entirely in NEW sim/test code
   (`tests/sim/native_boolean_fuzz.mm`). The generator is a splitmix64/xoshiro-style
   PRNG seeded by an explicit integer (NO clock, NO `rand()`, NO `Date`) — the same
   seed reproduces the identical batch on any machine. It emits parameter tuples
   (axis, centre, radius, extents, half-angle) constrained to the *recognised*
   families so the native path is genuinely **exercised**, not merely declined
   (radii/extents kept positive and overlapping, cones non-degenerate, placements
   drawn from {coaxial, orthogonal-through, offset-parallel}).

2. **A dual builder** that turns each parameter tuple into BOTH a native operand
   (`nb::curved` / `makeCyl` / `makeSphere` / `makeCone`, OCCT-free) AND the
   *identical* OCCT primitive (`BRepPrimAPI_MakeCylinder` / `MakeSphere` /
   `MakeCone`), so native and oracle see the same solid.

3. **An agree / decline / DISAGREE classifier vs OCCT.** For each generated pair and
   each op `{Fuse, Cut, Common}` the harness runs the native path
   (`nb::boolean_solid`, the `cc_set_engine(1)` equivalent at the C++ boundary) AND
   the OCCT oracle (`BRepAlgoAPI_{Fuse,Cut,Common}`), then classifies EXACTLY one of:
   - **AGREED** — native result non-null AND watertight AND (volume within tol AND
     area within tol) of the OCCT oracle.
   - **HONESTLY-DECLINED** — native returned NULL (→ OCCT fallback), and the OCCT
     oracle is a valid watertight solid (the shipped result is correct).
   - **DISAGREED** — native result non-null AND watertight BUT volume OR area is
     OUTSIDE tol of OCCT. **This is the M6 failure condition** — a silent wrong
     result. A DISAGREE fails the harness and prints the reproducing **seed +
     case index + parameter tuple** as a regression find.

4. **A coverage summary** printed at the end: `N` inputs, `agreed` / `declined` /
   `DISAGREED` counts, per-family and per-op breakdown, and the batch seed. The
   **bar is `DISAGREED == 0`**; the process exits non-zero if any disagreement
   surfaces (or a fallback whose OCCT oracle is itself invalid).

5. **A new sim runner script** (`scripts/run-sim-native-boolean-fuzz.sh`), modelled
   on `run-sim-native-ssi-curved-boolean.sh`: builds the numsci iossim substrate,
   compiles the fuzz `.mm` + the native TUs, links the OCCT boolean/primitive oracle
   toolkits, and spawns the batch in a booted simulator via `xcrun simctl spawn`
   (seed and batch size overridable by env, default e.g. 128 valid pairs).

## Capabilities

### Added Capabilities
- `native-verification`: a new capability describing the **differential-fuzzing
  completeness discipline** — a deterministic seeded generator of random-*valid*
  inputs for a native capability, the agree/decline/DISAGREE classifier against the
  OCCT oracle, the coverage summary, and the **zero-silent-wrong-result bar** that
  gates `drop-occt`. First subject: the recognised curved-boolean operand families.

## Impact

- **NEW** `tests/sim/native_boolean_fuzz.mm` — seeded generator + dual builder +
  classifier + coverage summary. Objective-C++ sim harness with its own `main()`;
  links OCCT as the ORACLE only (like the sibling parity `.mm` files). On the SKIP
  list of `run-sim-suite.sh` (own `main()`, OCCT + numsci slice).
- **NEW** `scripts/run-sim-native-boolean-fuzz.sh` — build + simctl-spawn runner,
  `FUZZ_SEED` / `FUZZ_COUNT` env overridable, deterministic default seed.
- `src/native` is **UNTOUCHED** — no OCCT added to the native path, no production
  code (tessellator, boolean library) modified. The harness consumes the existing
  `nb::boolean_solid` and `nb::curved` builders read-only.
- No `cc_*` ABI change. Additive only. No existing suite regressed (the fuzz harness
  is a new, independent runner, not wired into any existing gate).
- Determinism is a hard requirement: no clock-seeded RNG, no timestamps — a
  DISAGREE is always reproducible from its printed seed + index.
