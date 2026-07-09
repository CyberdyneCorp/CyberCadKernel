# Design — moat-m6k-transformed-boolean-fuzz (MOAT M6-breadth-11)

## Context

Eleventh differential-fuzzing domain on the MOAT completeness bar, and the FIRST that
fuzzes an INTERACTION between two domains rather than a single domain in isolation.
Test-infra only: the native planar boolean core (`src/native/boolean/*`), the native
transforms (`NativeEngine::{translate,rotate_about,mirror}_shape` →
`topology::Shape::located(math::Transform)`), and the `cc_*` facade are the SYSTEM UNDER
TEST and stay byte-unchanged. The change adds one harness, one runner, one SKIP-list
line, and the roadmap row bump.

## Goals / Non-goals

- **Goal:** search the transform × boolean composition space (random rigid-placed planar
  operands × `FUSE`/`CUT`/`COMMON`) for a silent wrong native result caused by
  `Location` mis-composition, arbitrated by an independent rigid invariant, with zero
  DISAGREED across ≥2 seeds.
- **Non-goal:** fixing anything (no finding to fix — the composition is exact). No
  `src/**` or `include/**` change; no `cc_*` ABI change; no tolerance widening to launder
  a real disagreement.

## Key decisions

### 1. Domain choice: boolean-of-transformed-operands, through `cc_*` under both engines

The roadmap M6 REMAINING list names "boolean-of-transformed-operands" as the interaction
"single-domain fuzzers miss". It is chosen over the other candidates because:

- **Section-curve (GS2)** and **HLR (GS1)** are ALREADY covered — both live inside the
  landed `native_geometry_services_fuzz.mm` (GS2 `sec::sectionByPlane` incl. the OBLIQUE
  cylinder cut vs `BRepAlgoAPI_Section`; GS1 `drafting::projectOrthographic` vs the
  box-corner HLR oracle). Adding a standalone section/HLR fuzzer would duplicate coverage.
- **Healing** is a valid future pick, but it is another single-domain fuzzer; the
  transform × boolean INTERACTION is a distinct, previously un-fuzzed composition with a
  uniquely clean closed-form arbiter (below), so it is the higher-value gap now.
- **The interaction is real and reachable.** The native planar boolean receives operands
  carrying a solid-level `Location` from the transform; the polygon extractor must fold
  it. `native_boolean_fuzz` (identity-Location operands) and `native_transform_fuzz`
  (single solid) each leave this composition untested.

### 2. The rigid invariant is the PRIMARY, engine-independent arbiter

A rigid transform `T` (translate / rotate / mirror) is an isometry: it preserves volume
and area, and a boolean commutes with it — `T(A) ∘ T(B) == T(A ∘ B)`. So the
transformed-boolean's enclosed volume must EXACTLY equal the untransformed boolean's
volume:

```
|R0| = boolean(A,    B   , op)          (baseline, identity placement)
|RT| = boolean(T(A), T(B), op)          (both operands moved by T)
|RT|_vol  ==  |R0|_vol                  (rigid invariant — NO oracle needed)
```

Because every operand family is an ALL-PLANAR prism (BOX / NGON / concave LSHAPE), the
native BSP-CSG boolean produces an EXACT mesh, so this invariant holds to machine epsilon.
A dropped, mis-composed, or mis-oriented `Location` (e.g. a MIRROR handedness flip not
propagated to a face's outward normal, inverting inside/outside classification) would
change `|RT|` by a LARGE margin — the invariant cleanly separates a correct composition
from a real bug. This is the true correctness signal and is held TIGHT (`kInvVol=1e-6`).

The native-vs-OCCT comparison is the SECONDARY arbiter, at the fixed relative tolerance
`native_boolean_fuzz` proved (`2e-2`): native's exact planar mesh vs OCCT's
`BRepAlgoAPI` + `BRepGProp` on the same prisms differ by ~0.1–0.9% (OCCT meshing/rounding
on concave L-prisms), well inside `2e-2`.

### 3. Volume vs area invariant, and why they carry different tolerances

The VOLUME invariant is the enclosed volume of two solids that must be congruent under a
rigid map — exact for planar meshes, held at `1e-6`. The AREA invariant is derived from
the WELDED FACET COUNT of two DIFFERENT native meshes (the canonical `R0` mesh vs the
located `RT` mesh): on a concave seam the two tessellations can weld a marginally
different facet set, giving a ~1e-3 relative area wobble while the volume stays exact. So
the area invariant is held to a mesh-appropriate bound (`kInvArea=5e-3`) — never tighter
than that path can honour, never looser than would hide a real area discrepancy (which
would move volume too).

### 4. Localisation of the initial false positives (why the bar is honest)

The first run at over-tight bounds (`kVolRel=kAreaRel=1e-3`, `kInvRel=1e-6` on BOTH
vol+area) flagged 8+3 DISAGREED + 2+2 ORACLE_UNRELIABLE. Localisation:

- Several DISAGREED fired on `T=IDENTITY` (no transform at all) — so they were NOT a
  transform-composition bug but the plain native-vs-OCCT boolean measurement gap the
  sibling fuzzer already tolerates at `2e-2`.
- On EVERY flagged trial the PRIMARY VOLUME invariant was ≤1e-15 (measured across 400
  trials / 2 seeds: ZERO violations) — proof the native transform+boolean is exact.
- The remaining area-invariant trips were ~1e-3 (canonical-vs-located meshed-facet weld
  wobble; volume unaffected).

Calibrating the SECONDARY bound to the sibling-proven `2e-2` and the AREA invariant to
`5e-3`, while KEEPING the VOLUME invariant tight, yields DISAGREED=0 / ORACLE_UNRELIABLE=0.
No native disagreement was hidden: the exact closed-form volume invariant remains the
certifying arbiter, and no product code changed.

### 5. Operand families kept inside the native planar domain

`cc_boolean` under the native engine is native ONLY for all-planar polyhedra; a curved
operand falls through to OCCT. So the families are BOX, NGON prism (n∈[3,7]), and a
concave LSHAPE prism (a simple-concave polyhedron — still all-planar, and the most
stressing for `Location` folding + inside/outside classification). Operand B is positioned
to straddle A for a clean transversal overlap valid for all three ops.

## Risks / mitigations

- **Cross-engine measurement crash.** Measuring a native-built id under the OCCT adapter
  dereferences a `NativeShape*` as a `TopoDS_Shape*`. Mitigated: `measure()` activates the
  owning engine first; the RAII `Body` releases under its owning engine.
- **A COMMON that is genuinely empty** (operands separated by the transform on a
  degenerate config) → BOTH-DECLINED, not a false DISAGREE.
- **Non-rigid transforms** (SCALE) would break `|RT|==|R0|` (volume scales `s³`); excluded
  by construction — this slice fuzzes the rigid group where the invariant is exact.

## Verification

Runner runs ≥2 seeds (default `0xB007C0DE11` / `0x1DEA5EED77`), N=96 each, on the booted
simulator; fails if any seed fails. Both seeds: DISAGREED=0, ORACLE_UNRELIABLE=0, full
family/op/transform coverage; the rigid VOLUME invariant holds to ≤1e-15 on every trial.
Determinism re-verified (same seed twice → byte-identical output, md5 match). `git diff`
touches only `tests/sim` + `scripts` + `openspec`.
