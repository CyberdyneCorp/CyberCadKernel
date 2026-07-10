# Design — moat-m6q-clash-fuzz

## Context

The landed `cc_interference` op (`src/native/analysis/interference.h`) is an OCCT-FREE,
header-only clash classifier: given two solids' watertight M0 boundary meshes it returns
CLASH / TOUCHING / CLEAR + a min clearance + (on a clash) a witness. Its existing coverage
is the OCCT-free host fixtures and a five-pose hand-picked box parity harness. This change
adds the seeded, randomised, differential completeness certification.

## Goals / non-goals

- **Goal:** certify the clash classifier over random PAIRS of solids at random relative
  placements spanning CLEAR / TOUCHING / CLASH, against OCCT AND a closed-form arbiter,
  with a zero-silent-wrong-clash bar over ≥2 seeds.
- **Goal:** stress the TOUCHING knife-edge (flush contact) — the classic robustness trap —
  under randomised jitter, asserting the coplanar-safe B3 ON-band handling.
- **Non-goal:** any change to product code. The classifier, the facade, and the ABI stay
  byte-unchanged; a surfaced native limitation is reported, not fixed.

## Key decision — direct header-only differential, not a `cc_set_engine` A/B

Because `interference.h` is OCCT-FREE and header-only (like the primitives in
`native_interference_parity.mm`), the natural differential is: build the native operand
as a watertight tessellated mesh and call `meshInterference` directly, and build the
matching OCCT solid and run the OCCT oracle — placing BOTH by the SAME rigid transform.
There is no OCCT arm of the classifier to select via `cc_set_engine`; the classifier IS
the native arm, and OCCT is the external oracle. This mirrors the landed interference
parity harness exactly and keeps the link set minimal (native math TUs + OCCT oracle
toolkits), rather than linking the whole kernel for a facade A/B that would not exercise
anything more of the clash op.

## Primitives and placement

Four families, each emitting BOTH a native mesh and an OCCT solid at identical dimensions:
- **box** — 6-face axis-aligned box (closed-form: axis-aligned box∩box).
- **n-gon prism** — regular 3..8-gon extruded along Z.
- **cylinder** — 48-facet native mesh vs the true OCCT `BRepPrimAPI_MakeCylinder`.
- **sphere** — UV-tessellated native mesh vs the true OCCT `BRepPrimAPI_MakeSphere`
  (closed-form: sphere∩sphere lens).

Body A sits at identity; body B is placed by a shared `Rigid` (Rodrigues rotation about a
random unit axis + translation), applied to the native mesh vertex-wise and to the OCCT
solid via the equivalent `gp_Trsf`, so both sides are the SAME body in the SAME pose.

## The three regimes and the TOUCHING knife-edge

The generator round-robins [family × regime] so all 12 cells are populated (N=72 ⇒ 6
trials/cell). CLEAR draws a comfortable gap; CLASH draws a guaranteed positive-volume
overlap. TOUCHING draws the exact flush pose then a signed knife-edge jitter:

- **flush (j=0)** — faces exactly coincident ⇒ TOUCHING (a shared face reads `On` via the
  B3 ON-band, NEVER a clash — the coplanar-safe property the op guarantees).
- **penetrate (j<0, well past 2·deflection)** — a real thin overlap ⇒ CLASH must fire.
- **gap (j>0)** — a small clearance ⇒ CLEAR.

The flush TOUCHING pose is kept inside the op's certified contact envelope: B's contact
footprint ⊆ A's, so at least one operand has a boundary vertex on the other's face — the
seated / coincident / contained / slid assembly-mate contact the op resolves exactly.

## The faceting boundary

A faceted cylinder / sphere is inset from the true surface by up to ~2·deflection. At a
near-flush curved contact the faceted surfaces can read a hair CLEAR where OCCT's exact
B-rep reads TOUCHING — an expected facet artefact absorbed by the op's own contact band.
A curved-pair TOUCHING↔CLEAR straddle is therefore FACET-CONVERGENT (a convergent match),
not a DISAGREE. A CLASH↔CLEAR / CLASH↔TOUCHING split is always a hard state.

## Oracles and the six-way classifier

- **OCCT:** `BRepAlgoAPI_Common` volume (>1e-7 ⇒ CLASH) + `BRepExtrema_DistShapeShape`
  (≤1e-6 with no overlap ⇒ TOUCHING; else CLEAR); declines if the boolean fails.
- **closed-form (PRIMARY where present):** box∩box overlap-box volume + axis gap;
  sphere∩sphere lens volume + centre-distance regime.
- **classifier:** states agree ⇒ AGREED; native `Unknown` ⇒ HONESTLY-DECLINED (fall
  through to OCCT); a soup probe (deliberately non-watertight operand) MUST decline (else
  a hard DISAGREE — the "never guess on ambiguous mesh evidence" contract); a curved
  TOUCH↔CLEAR straddle ⇒ FACET-CONVERGENT; any other hard split arbitrated by the closed
  form (sides with native ⇒ ORACLE-INACCURATE, native vindicated; sides with OCCT ⇒
  DISAGREED, a silent-wrong clash that fails the bar).

## Determinism

The generator is a pure function of `FUZZ_SEED` via splitmix64 → xoshiro256** (verbatim
from the landed siblings). No clock, `rand()`, address, or thread scheduling. Re-running
the same seed produces a byte-identical batch (verified: identical output digest).

## The bar

Exit 0 IFF DISAGREED == 0 across ≥2 seeds (N≥60/seed; the runner fails if any seed fails)
with every populated cell truly exercised (≥1 AGREED / ORACLE-INACCURATE /
FACET-CONVERGENT). The contact band is the classifier's own and is never widened.

## Surfaced native limitation (reported, not fixed)

`interference.h` step 4 computes the min triangle–triangle distance from the six
vertex-vs-face sub-tests and does NOT evaluate the edge-edge case (the header documents
this as "a tight bound otherwise"). For two boxes whose faces are EXACTLY coplanar and
overlap in a plus-sign CROSS with NO mutually-contained vertex, the vertex-face minimum
over-estimates the distance and a genuine flush TOUCH is mis-reported CLEAR. Confirmed
OCCT-free on the host gate; correct for every in-envelope seated/coincident/contained/
slid contact. Outside the certified assembly-mate envelope; reported here for a future
product change to close with the edge-edge distance term. This test-infra track does NOT
modify the product.
