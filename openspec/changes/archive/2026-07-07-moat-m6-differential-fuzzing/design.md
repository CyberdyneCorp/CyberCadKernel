# Design — moat-m6-differential-fuzzing

## Context

M6 is the *completeness bar*. The existing curved-boolean verification
(`tests/sim/native_ssi_curved_boolean_parity.mm` + `run-sim-native-ssi-curved-boolean.sh`)
compares the native path against the OCCT `BRepAlgoAPI` oracle on **~18 curated
`{pair, op}` fixtures**. That harness already implements the right *core* idea — it
auto-detects at runtime whether each case is a native pass or an honest fallback and
asserts the shipped result is a valid solid — but its inputs are hand-picked. This
change reuses that exact machinery and swaps the fixed fixture list for a
**deterministic seeded generator of random-valid inputs**, and adds the missing
third classification the fixture harness folds away: an explicit **DISAGREE** bucket
(watertight-but-wrong) that is the failure M6 exists to catch.

Scope discipline (from the lane brief): the harness MAY link OCCT (oracle side);
`src/native` stays OCCT-free; deterministic only (seeded RNG, no clock); additive;
no production code touched; do not regress any suite.

## Goals / Non-goals

**Goals**
- Deterministic seeded generator of random-*valid* operands in the recognised
  families (cyl / sphere / cone; coaxial / orthogonal / offset-parallel).
- Dual builder: one tuple → native operand (OCCT-free) + identical OCCT primitive.
- Three-way classifier: AGREED / HONESTLY-DECLINED / DISAGREED vs OCCT.
- Coverage summary; process exits non-zero iff any DISAGREE (bar = zero).
- Reproducible: every DISAGREE prints seed + index + tuple.

**Non-goals**
- Improving native coverage (turning declines into passes) — that is M2/M3/M5 work.
  A high decline rate is acceptable here; only a DISAGREE fails.
- Freeform / skew / general-quadric booleans — the generator stays inside the
  *recognised* families on purpose (so native is exercised, not just declined).
- Wiring the fuzz runner into any existing CI gate (it is a standalone new runner).
- Any `cc_*` ABI or `src/native` change.

## Key decisions

### 1. New capability `native-verification` (not a native-ssi delta)
Differential fuzzing is a **verification discipline**, not an SSI geometry feature.
It is subject-agnostic (curved booleans are the *first* subject; blends / construct
follow later under the same capability). Modelling it as `native-ssi` would bury a
cross-cutting completeness bar inside one geometry module. So the spec delta ADDs a
new `native-verification` capability whose four requirements are the generator, the
dual builder, the classifier, and the summary/bar — reusable as later M6 slices
fuzz other native surfaces.

### 2. Deterministic RNG: splitmix64 seed → xoshiro256** stream
No clock, no `rand()`, no `Date` (the workflow runtime forbids nondeterministic
clocks anyway, and a DISAGREE must be reproducible). A tiny inline `splitmix64`
expands the 64-bit seed into the `xoshiro256**` state; all sampling
(`next_double(lo,hi)`, `next_int(lo,hi)`, `next_choice`) draws from that one stream.
Seed and batch size are env-overridable (`FUZZ_SEED`, `FUZZ_COUNT`) with a fixed
default (e.g. seed `0x6d6f61746d36ULL` "moatm6", count 128). The generator is
header-local to the `.mm`; it produces plain `struct OperandSpec` POD (family enum,
axis, centre[3], radius, r1/half-angle, lo, hi) — no OCCT, no native type — so the
*same* tuple feeds both builders.

### 3. Valid-by-construction sampling (exercise, don't just decline)
Each pair is sampled to be non-degenerate and overlapping so the native recogniser
accepts the operands and the boolean is actually attempted:
- **radius** ∈ [0.4, 2.5], **extent** half-length ∈ [1.5, 4.0] (so `lo = c-h`,
  `hi = c+h`, `lo < hi`), **cone** `r0 ≠ r1` with both ≥ 0 and a positive axial span.
- **placement** drawn from a small recognised set so intersections are real:
  `coaxial` (shared axis, overlapping extents), `orthogonal-through` (axes at 90°,
  the classic Steinmetz / through-drill), `offset-parallel` (parallel axes, offset
  < r_a + r_b so they overlap). Family pairs drawn from
  {cyl×cyl, sphere×sphere, cone×cyl, cone×sphere, sphere×cyl} — the pairs the S1/S5
  recogniser and the OCCT primitives both express.
This mirrors the curated harness's own operand builders (`makeCyl`/`makeSphere`/
`makeCone`) — the generator just samples their parameters instead of hard-coding.

### 4. Dual builder — one tuple, two solids
`buildNative(spec)` calls the existing OCCT-free builders (`makeCyl` via
`nb::curved::buildCommonSegment` with an `AxisCylinder`; `makeSphere`; `makeCone`),
exactly as the curated harness does. `buildOcct(spec)` builds the identical
`BRepPrimAPI_MakeCylinder(gp_Ax2, r, h)` / `MakeSphere` / `MakeCone`, transformed to
the same axis/centre. A cheap self-check (native operand volume/area ≈ OCCT operand
volume/area) guards that the two builders agree BEFORE the boolean — so a downstream
DISAGREE is never an artefact of a mismatched input.

### 5. Three-way classifier (the M6 addition)
Reuse the curated harness's runtime auto-detect, then split its "fallback" fold:

```
nativeRes = nb::boolean_solid(nativeA, nativeB, op);   // native, OCCT-free
occtRes   = occtBoolean(occtA, occtB, op);             // oracle
nm = nativeMeshVerify(nativeRes);  // {present, watertight, volume, area}
om = occtVerify(occtRes);          // {valid, closedShell, volume, area}

if (nm.present && nm.watertight) {
    volOk  = rel(nm.volume, om.volume) <= VOL_TOL;
    areaOk = rel(nm.area,   om.area)   <= AREA_TOL;
    if (om.valid && volOk && areaOk)  -> AGREED
    else                              -> DISAGREED   // watertight but wrong = M6 fail
} else {
    // native NULL or non-watertight candidate -> honest decline -> OCCT fallback
    if (om.valid && om.closedShell)   -> HONESTLY_DECLINED
    else                              -> FALLBACK_ORACLE_INVALID  // also a failure
}
```

- **AGREED**: native shipped and matches OCCT.
- **HONESTLY_DECLINED**: native declined and OCCT is a valid solid → correct fallback.
- **DISAGREED**: watertight native result outside tol → **silent wrong result** → fail.
- **FALLBACK_ORACLE_INVALID**: a guard — if native declines but OCCT itself is not a
  valid solid the input was not truly valid; counted as a failure so a broken
  generator can't launder a real problem into a "decline".

`VOL_TOL` / `AREA_TOL` are FIXED relative tolerances sized to the curved-face
tessellation deflection (same basis as the curated harness's `volRel`), never
widened to the observed gap. A non-watertight native result is deliberately a
decline, not an agreement — it mirrors the engine self-verify that would discard it.

### 6. Coverage summary + exit code
Accumulate `{AGREED, HONESTLY_DECLINED, DISAGREED, FALLBACK_ORACLE_INVALID}` overall
and per (family-pair, op). Print:

```
== M6 differential-fuzz coverage ==  seed=0x…  N=384 (128 pairs × 3 ops)
  agreed             : 213
  honestly-declined  : 171
  DISAGREED          : 0
  per-op    fuse=… cut=… common=…
  per-family cyl×cyl=… sphere×sphere=… …
BAR: zero silent wrong results — PASS
```

Exit `0` iff `DISAGREED == 0 && FALLBACK_ORACLE_INVALID == 0`. On any DISAGREE, each
is printed as `DISAGREE seed=… idx=… op=… family=… spec={axis,c,r,lo,hi,…} volNat=… volOcct=… relVol=…` — a self-contained regression find.

## Harness / build shape (mirrors the curated sibling)

- `tests/sim/native_boolean_fuzz.mm` — own `main(argc,argv)`, reads `FUZZ_SEED` /
  `FUZZ_COUNT` from `getenv`, includes `native/boolean/native_boolean.h`,
  `native/boolean/curved.h`, `native/boolean/ssi_boolean.h`,
  `native/tessellate/native_tessellate.h`, and the OCCT oracle headers
  (`BRepPrimAPI_*`, `BRepAlgoAPI_*`, `BRepGProp`, `BRepCheck_*`). Reuses the curated
  harness's `nativeMeshVerify` / `occtVerify` / `occtBoolean` helper shapes.
- `scripts/run-sim-native-boolean-fuzz.sh` — copy of
  `run-sim-native-ssi-curved-boolean.sh` with the harness path and output binary name
  swapped, `FUZZ_SEED` / `FUZZ_COUNT` passed through to the `simctl spawn` env. Same
  numsci iossim substrate build, same native TU list (math + ssi/{seeding,marching} +
  numerics + boolean/ssi_boolean), same OCCT toolkit link line
  (`TKBool TKBO TKPrim TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel`).
- On the SKIP list of `run-sim-suite.sh` (own `main()`, OCCT + numsci slice) — it does
  not join any existing aggregate gate.

## Risks / mitigations

- **A DISAGREE surfaces (a real native bug).** That is the harness doing its job —
  it is REPORTED with its seed as a regression find, never hidden or tolerance-massaged.
  The lane brief explicitly expects this is possible and wants it surfaced.
- **Generator too "easy" (all declines).** Sampling is constrained to recognised
  families/placements that the S1/S5 recogniser accepts, so a meaningful fraction
  AGREE — the summary's agreed-count makes coverage visible; if it collapses to all
  declines the tuple ranges are widened toward the curated fixtures' known-native cases.
- **Nondeterminism creep.** Single seeded xoshiro stream, no clock/`rand()`; the
  determinism scenario (same seed → identical batch) is an explicit spec requirement.
- **Flaky OCCT oracle on a near-degenerate sample.** The valid-by-construction
  sampling avoids tangency/degeneracy; the `FALLBACK_ORACLE_INVALID` guard catches
  any slip instead of silently crediting it as a decline.

## Migration / rollout

Additive and standalone. No existing file changes; `src/native` untouched; no `cc_*`
ABI change. Verified by running the new script on a booted simulator and observing
`DISAGREED == 0` over the default seeded batch. Future M6 slices reuse the
`native-verification` capability to fuzz other native surfaces (blends, construct).
