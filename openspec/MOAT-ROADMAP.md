# Dropping OCCT — the Moat Roadmap

The **complete remaining path** from "substantially native + OCCT fallback" to `#8
drop-occt` (unlink OCCT entirely). Everything reachable by a *bounded* native slice has
landed (see [NATIVE-REWRITE.md](NATIVE-REWRITE.md) and [SSI-ROADMAP.md](SSI-ROADMAP.md));
what remains is the **research-grade moat** — the small set of genuinely hard capabilities
that repeatedly blocked the bounded slices, plus the two *asymptotic* robustness tails that
have no finite "done" line.

Parent: [NATIVE-REWRITE.md](NATIVE-REWRITE.md) (#8 `drop-occt`). Enabler already built:
[SSI-ROADMAP.md](SSI-ROADMAP.md) (S1–S5 curve pipeline + curved booleans).

## The non-negotiable discipline (every stage, no exceptions)

**OCCT is the ORACLE throughout implementation — it is NOT removed until the capability it
backs is PROVEN native.** This is the same rule every landed tier used and it does not
relax for the moat:

1. **Two gates per capability.** (a) *Host analytic* — the native result matches a
   closed-form / independent computation with no OCCT linked. (b) *Sim native-vs-OCCT* —
   on a booted iOS simulator (OCCT linked) the native result matches the OCCT oracle
   (`BRepAlgoAPI` / `BRepFilletAPI` / `BRepMesh` / `GeomAPI_IntSS` / `IntPatch` /
   `STEPControl_Reader` …) on volume / area / watertightness / topology / continuity.
2. **Self-verify → OCCT fallback, always.** The native builder returns NULL when it cannot
   robustly build; the ENGINE runs the mandatory watertight + correct-value self-verify and
   DISCARDS a bad native result, falling through to OCCT. A wrong/leaky native result is
   never emitted.
3. **No fabrication, no dead code, no weakened tolerances.** An honest decline (with the
   measured gap + the specific blocker) is a first-class outcome. Capabilities that cannot
   be built are documented, not faked. (Session precedent: general-branched booleans =
   geometrically impossible; NURBS booleans = decline at recognition; curve cusp = the S4-c/d
   witness by the IFT; foreign rational B-spline = the M0 mesh gap below.)
4. **`src/native/**` stays OCCT-FREE; the `cc_*` ABI is additive-only.** OCCT lives only in
   `src/engine/occt` (the oracle + fallback). The final `drop-occt` step deletes that engine
   — but only once every stage below is native AND the completeness bar (M6) is met.

## The keystone finding (why the moat has the shape it does)

The bounded slices this project shipped repeatedly declined at **one recurring blocker**:
a general **foreign B-spline surface patch cannot be tessellated watertight**. Our *own*
B-spline faces mesh (they are bare-periodic `VERTEX_LOOP` faces from revolution/extrude);
a *foreign* B-spline patch has trimmed `EDGE_LOOP` bounds with pcurves the native mesher does
not handle. This single gap sits under **freeform booleans, freeform blends, freeform
wrap-emboss, AND foreign STEP import**. It is stage **M0** — the keystone — and unblocks the
most downstream work per unit effort.

## Stages (dependency order)

Effort is given as **first robust slice** → *the honest bounded target*; the two asymptotic
stages (M6, and the tail of M5) never fully close.

### M0 — Freeform surface meshing + trimming (the keystone) · ~1.5–3 py
A native tessellator path for a **general trimmed B-spline/NURBS face**: pcurve-bounded
`EDGE_LOOP` trimming, watertight edge-shared meshing of a rational/non-rational patch, and
the pole/degeneracy handling already proven for revolution surfaces generalised to arbitrary
patches. **Unblocks M1, M2, M3, and foreign-B-spline STEP import (M4).**
- *Oracle:* `BRepMesh_IncrementalMesh` watertightness + area/volume of the meshed solid;
  the foreign-rational-B-spline STEP fixture that currently declines (M0 is exactly the gap
  that decline exposed).
- *Bounded.* This is hard but finite: a trimmed-NURBS mesher is well-understood engineering.

### M1 — General freeform surface–surface intersection robustness (SSI S4 general) · ~2–5 py
Extend the SSI marcher (S1–S5 + S4-a…e already native) to the **general/freeform** degeneracy
regimes still deferred: general/freeform branch points (S4-d beyond Steinmetz), general
near-tangent (S4-c breadth), coincident/overlapping freeform surfaces, and self-intersection
resolution. The curve *pipeline* exists; this is the *robustness* on adversarial freeform input
— OCCT's decades-deep `IntPatch`/`IntWalk` tuning, re-earned incrementally.
- *Oracle:* `GeomAPI_IntSS` / `IntPatch` curve match (onSurf, arc length, branch/loop counts).
- *Partly asymptotic* — ships as progressively hardened slices; whatever is not robust defers.

### M2 — General freeform booleans · ~2–4 py · needs M0 + M1
Lift `recogniseCurvedSolid` to accept **freeform (B-spline/NURBS) operands** (it rejects them
today — the S5 assembler is analytic-only), split freeform faces along the S3/M1-traced WLine,
classify + weld watertight (the M0 mesher). This is the general-curved-boolean payoff the
whole SSI arc was built for.
- *Oracle:* `BRepAlgoAPI_{Fuse,Cut,Common}` volume/area/watertight on NURBS↔NURBS and
  freeform↔analytic pairs.
- *Bounded per family, asymptotic in full generality* (arbitrary self-intersecting inputs).

### M3 — General freeform blends + wrap-emboss · ~2–4 py · needs M2
The curved-curved blends and freeform-base features that sit on booleans: cyl↔cyl-canal /
elliptical-crease / variable-on-freeform fillets, general chamfers, and wrap-emboss on a
sphere/cone/freeform base (all declined this session for lack of M0/M2). General canal-surface
construction + the M0 mesher + M2 booleans.
- *Oracle:* `BRepFilletAPI` / `BRepOffsetAPI` / `cc_wrap_emboss` (volume/area/watertight/continuity).

### M4 — General STEP / AP242 import (+ IGES stays dropped) · ~1.5–3 py · needs M0
The remaining import breadth on the landed AP203+ reader: **foreign rational/general B-spline
patches** (needs M0 to mesh them), AP242 **PMI semantics** (not just skip), general **trimmed
surfaces**, deep-nested/`MAPPED_ITEM` assemblies. IGES is **descoped** (STEP-only; `cc_iges_*`
stays OCCT until removed/stubbed at drop-occt — never reimplemented).
- *Oracle:* `STEPControl_Reader` re-import (count/volume/watertight/topology) + foreign files.
- *Bounded* (mechanical parser breadth, once M0 meshes the surfaces).

### M5 — Shape-healing robustness · ~2–4 py + asymptotic tail · gates M4 quality
Beyond the landed first slice (tolerant sew + vertex/tolerance unify + degenerate removal +
orientation): **pcurve reconstruction, self-intersecting-wire repair, beyond-tolerance gap
bridging, arbitrary broken industrial B-rep**. Gates trustworthy foreign import (M4) and any
`drop-occt` decision.
- *Oracle:* `ShapeFix_*` / `BRepBuilderAPI_Sewing` on broken fixtures + real foreign files.
- **Asymptotic** — a first robust slice is bounded; the completeness against arbitrary broken
  input is the decades-deep `ShapeFix` moat, re-earned only incrementally.

### M6 — Robustness completeness bar (S4-f + coverage) · ongoing · gates drop-occt
The measured-recall / completeness discipline (SSI S4-f landed a first slice): below any fixed
resolution a smaller intersection loop can be missed. Before OCCT can be *removed* (not just
defaulted), a **measured completeness bar** across the whole native surface — booleans, blends,
import, healing — must be met and continuously guarded (loop-until-dry critics, adversarial
fuzzing vs OCCT).
- *Oracle:* differential fuzzing — random valid inputs through both native and OCCT, assert
  agreement or an honest native decline; zero silent wrong results.
- **Asymptotic** — never "done"; this is the gate that keeps `drop-occt` a decision, not a date.

### M7 — Tier-4 construction robustness · ~1–3 py · independent
The construction breadth still deferred: **guided/rail sweep** (orientation oracle),
**general/mismatched-hard loft** (beyond the landed vertex-correspondence slice),
**fine-pitch self-intersecting threads** (intersecting-helicoid trimming — needs M1/M2).
Independent of the freeform-boolean chain except fine-pitch (needs M2).
- *Oracle:* `BRepOffsetAPI_MakePipeShell` / `ThruSections` / thread fixtures (volume/watertight).

### M8 — `drop-occt` — unlink OCCT · gated on M0–M7 + M6 bar
Delete `src/engine/occt`, drop the OCCT link, remove/stub `cc_iges_*`. **Only** once every
stage above is native at the acceptance bar AND the M6 completeness bar holds (differential
fuzzing shows zero silent wrong results — every non-native input honestly declines with a
clear error rather than a fabricated shape). This is the terminal step; it does not begin
until the fallback is provably unnecessary for the supported domain.

## Sequencing

```
M0 freeform mesher/trimmer (KEYSTONE) ──┬──► M2 freeform booleans ──► M3 freeform blends/wrap
                                        │            ▲
M1 SSI S4 general robustness ───────────┴────────────┘
                                        └──► M4 general STEP/AP242 import ──► (needs M5 quality)
M5 shape-healing robustness ────────────────────────► gates M4 + M8
M6 completeness bar (S4-f + fuzzing) ───────────────► gates M8   [asymptotic]
M7 Tier-4 construction (guided sweep / hard loft / fine-pitch) ──► (fine-pitch needs M2)
                                                                        │
                          ALL of M0–M7 native at the bar + M6 holds ──► M8 drop-occt
```

## Effort rollup (honest)

| | Person-years |
|---|---|
| **Delivered + verified vs OCCT (this project)** | ≈ **3.5–4.5 py** — planar/analytic breadth, SSI S1–S5 + S4-a…e, five curved-boolean families 3/3, curved fillet/chamfer (const/variable/asym), STEP export + broad import (all quadric+torus+general revolution, trimmed, assemblies, AP242-skip), shape-healing + STEP-import first slices, mismatched loft, deboss/polygon wrap-emboss |
| **Remaining to drop OCCT (M0–M8)** | ≈ **5–11 py**, dominated by M0 (keystone) → M2/M3 (freeform booleans/blends), with M5 + M6 the asymptotic tails |
| ~~IGES~~ | descoped (STEP-only) — saved ~1.5–3 py |

## Honest framing

- **M0 is the highest-leverage single target** — it is the recurring blocker under freeform
  booleans, blends, wrap-emboss, and foreign STEP import. Doing it first unblocks the most.
- **M2/M3 are the payoff** but only reachable after M0 + M1; they are bounded per surface
  family and asymptotic only in full arbitrary generality.
- **M5 and M6 are why `drop-occt` (#8) is a long-horizon direction, not a date.** They are
  asymptotic by nature (arbitrary broken input, sub-resolution completeness); a first robust
  slice is bankable, the guarantee is re-earned continuously. OCCT stays the labelled oracle
  and fallback until the M6 bar demonstrates the fallback is unnecessary for the supported
  domain — never removed on faith.
- Every stage ships the same way the landed tiers did: **a narrow verified slice + an explicit
  OCCT-oracle gate + an honest fallback**, one capability at a time.
