## Why

`SSI-ROADMAP.md` S5 is **the payoff** — general curved booleans driven by the S3/S4
`TraceSet`. The archived `add-native-ssi-branched-boolean` (S5-d) landed the hardest slice:
the **Steinmetz bicylinder COMMON** natively. `buildSteinmetzCommon` consumes the S4-d
branched `TraceSet` (`MarchOptions.enableBranchPoints = true` → `branchPoints == 2`, four
`BranchArc` arms), splits each cylinder wall along its arcs into the two inside-the-other
lune patches, and welds the four lunes into a watertight shell sharing the four arc seams and
the two branch-point poles — verified against OCCT `BRepAlgoAPI_Common` AND the exact
analytic bicylinder volume `16 R³ / 3`. But that change shipped **COMMON only** for the
Steinmetz pair — its own honest-scope note explicitly defers FUSE / CUT to a follow-on, and
the dispatcher confirms it:

```cpp
if (op == Op::Common) return buildSteinmetzCommon(A, B, *st);
return {};  // FUSE / CUT deferred (COMMON is the guaranteed slice) → OCCT
```

So today the Steinmetz family is **1/3 native** (COMMON), while `ssi_boolean_solid`'s branched
dispatch (`tryBranchedSteinmetz`) returns NULL for `Op::Fuse` / `Op::Cut`, falling through to
OCCT `BRepAlgoAPI_{Fuse,Cut}` (an honest, verified fall-back — but a fall-back).

The GAP is again the **assembler**, not recognition or tracing: `steinmetzPreGate` +
`recogniseSteinmetzTrace` already fold the equal-R orthogonal cylinder pair and route the four
arcs, and the same branched split/weld machinery that assembles the four lunes is exactly what
FUSE and CUT need — the SAME branched trace, a DIFFERENT fragment (lune) selection plus the
original end caps. This change completes the set: Steinmetz becomes **3/3 native**, mirroring
the just-landed `complete-sphere-sphere-fuse-cut` (sphere lens COMMON → FUSE → CUT).

## What Changes

The geometry (two EQUAL cylinders A, B, radius R, axes crossing orthogonally; the intersection
self-crosses at 2 branch points into 4 arcs; each cylinder wall is split by its arcs into an
INSIDE-the-other region — the lune, used by COMMON — and an OUTSIDE-the-other region; each
cylinder also has its two original disc end caps, which lie WELL outside the intersection zone
so the arcs never touch them):

- **COMMON** `A ∩ B` (already native, `buildSteinmetzCommon`) = the four INSIDE lune patches
  (two per cylinder), welded on the four arcs + the two poles. Volume `V(A ∩ B) = 16 R³ / 3`.
- **FUSE** `A ∪ B` (new) = the four OUTSIDE wall regions (two per cylinder, the parts outside
  the other) + BOTH cylinders' original disc end caps, joined along the four arcs and the two
  poles. Volume `V(A) + V(B) − V(A ∩ B)`.
- **CUT** `A − B` (new, `A` the minuend) = A's two OUTSIDE wall regions + A's two original disc
  end caps + B's two lune patches emitted **REVERSED** (inward radial normal, so they bound the
  carved channel through A), joined along the four arcs and the two poles. Volume
  `V(A) − V(A ∩ B)`.

Concretely, reusing the S5-d branched split/weld (`orientArc`, `resampleArcByAxis`,
`groupLunes`, `appendLunePatch` with its `outwardSign`, the shared `VertexPool`, the pooled
poles):

- **Generalise the lune survival rule.** `buildSteinmetzCommon` keeps each lune whose centroid
  classifies INSIDE the other solid (`classifyPoint(other, centroid) == 1`). FUSE keeps the
  OUTSIDE lunes (`classifyPoint == -1`); CUT keeps A's OUTSIDE lunes AND B's INSIDE lunes with
  `outwardSign = −1` (reversed). No new split code — `appendLunePatch` already takes an
  `outwardSign`, and `groupLunes` already returns the clean 2+2 lune pairing per cylinder.
- **Add `appendCylinderEndCaps`** — a faceted disc-cap helper for the Steinmetz outer shell: for
  a cylinder, emit the two full-circle disc caps at `v = vLo` and `v = vHi` (outward axial
  normals `∓axis`), each fanned from the axis point to a fresh full-turn rim ring, using the
  SAME planar-facet discipline as the through-drill `appendDiskCap`. The caps are disjoint from
  the arcs (the intersection lives at `|y| ≤ R ≪ |vLo|,|vHi|`), so no seam weld is needed to
  them — they simply close the outer wall tube ends. A cap that would intersect the seam band
  (short cylinder) → NULL → OCCT.
- **Add `buildSteinmetzFuse(A, B, st)`** — mirror `buildSteinmetzCommon`: same oriented +
  resampled four arcs, same shared pole/`VertexPool` weld, but keep the OUTSIDE lunes of BOTH
  cylinders (`outwardSign = +1`, outward radial — the outer boss wall) and append all four disc
  end caps. Survival rule: a lune centroid strictly OUTSIDE the other solid. Volume of the
  welded shell = `V(A) + V(B) − 16 R³/3`.
- **Add `buildSteinmetzCut(A, B, st)`** — mirror `buildSteinmetzCommon`: `A` is the minuend
  (CUT is order-sensitive). Keep A's two OUTSIDE lunes (`outwardSign = +1`, outward) + A's two
  disc end caps, plus B's two INSIDE lunes emitted with `outwardSign = −1` (inward radial — the
  reversed channel wall), all sharing the four arcs + two poles. Volume of the welded shell =
  `V(A) − 16 R³/3`.
- **Driver dispatch** (`tryBranchedSteinmetz`): dispatch `Op::Fuse` → `buildSteinmetzFuse` and
  `Op::Cut` → `buildSteinmetzCut`, mirroring the existing `Op::Common` → `buildSteinmetzCommon`.
  Recognition, the branch-enabled re-trace, and `recogniseSteinmetzTrace` are UNCHANGED — only
  the `op` switch grows two arms. Non-Steinmetz branched pairs still return NULL → OCCT.
- **Engine self-verify — NO new oracle, NO sign change needed.** `ssiCurvedBooleanVerified`
  (`native_engine.cpp`) applies the analytic `16 R³/3` oracle ONLY to `op == 2` (COMMON), so it
  does NOT intercept Steinmetz FUSE/CUT. The generic set-algebra guard `booleanResultVerified`
  therefore runs: `vc = watertightVolume(boolean_solid(a, b, Op::Common))` = the native
  `buildSteinmetzCommon` = `16 R³/3`, and `expected = va + vb − vc` (fuse, GROWS) /
  `va − vc` (cut, SHRINKS). A mis-selected / mis-oriented / non-watertight candidate is
  DISCARDED → OCCT, never faked.

## Capabilities

### New Capabilities
<!-- none — this change COMPLETES the living native-booleans Steinmetz bicylinder op-set
(COMMON already native via the S5-d branched assembler) by adding native FUSE + CUT, and
re-affirms the S4-d branched TraceSet contract as the shared input to all three ops. -->

### Modified Capabilities
- `native-booleans`: complete the Steinmetz bicylinder op-set — add native **Fuse** (both
  cylinders' OUTSIDE wall regions + all four original disc end caps) and **Cut** (A OUTSIDE
  walls + A caps + B's lune patches REVERSED) alongside the already-native **Common** (the four
  inside lunes), all welded along the SAME four branch arcs + two branch-point poles with the
  shared-pool planar-facet lune discipline, guarded by the engine's EXISTING generic
  set-algebra self-verify (fuse grows, cut shrinks, vs the native bicylinder COMMON `16 R³/3`)
  → OCCT fallback. Reuse the S5-d lune split/weld; add a faceted cylinder end-cap helper; the
  COMMON path stays byte-identical. Other pairs unchanged; short-cylinder / non-Steinmetz /
  unresolved-branched pairs DECLINE → OCCT. No `cc_*` change.
- `native-ssi`: re-affirm the S4-d branched `TraceSet` (`branchPoints == 2`, four `BranchArc`
  arms on both cylinders, `nearTangentGaps == 0`) as the consumed input contract for ALL THREE
  Steinmetz ops — the SAME four arcs split each cylinder wall into inside/outside regions;
  COMMON/FUSE/CUT differ only in which regions survive, their orientation, and (FUSE/CUT) the
  original end caps.

## Impact

- **Scope (bounded completion).** This change ONLY completes the Steinmetz op-set (COMMON
  already native via S5-d; add FUSE + CUT). It is the SAME branched trace with a different
  fragment selection + cap handling — NOT an open-ended new family. All other pairs
  (through-drill cyl∩cyl, sphere∩sphere lens, sphere/cone∩box, cyl∩cone, cyl∩sphere, cone∩cone,
  oblique/multi-tube cyl∩cyl, freeform) are UNCHANGED and keep their existing native/decline
  behaviour.
- **ABI**: none. Invoked behind the existing `cc_boolean` op codes; no `cc_*` entry point,
  signature, or POD struct changes. Additive only.
- **Build**: extends `src/native/boolean/ssi_boolean.cpp` only — add `appendCylinderEndCaps`,
  add `buildSteinmetzFuse` + `buildSteinmetzCut` (mirroring `buildSteinmetzCommon`), extend the
  `tryBranchedSteinmetz` `op` switch. Reuses `orientArc`, `resampleArcByAxis`, `groupLunes`,
  `arcMeanU`, `luneUSamples`, `appendLunePatch`, `pushPlanarTri`, `classifyPoint`, `VertexPool`,
  `recogniseSteinmetzTrace`, `steinmetzPreGate` — all already present. No new files. Compiled
  under `CYBERCAD_HAS_NUMSCI` (the Steinmetz pair uses the S4-d branched SSI trace).
  `src/native/**` stays OCCT-free. No change to `src/native/tessellate`, the planar BSP-CSG,
  the analytic `curved.h`, the through-drill / sphere-lens builders, or `buildSteinmetzCommon`.
- **Verification (two gates, dual oracle, no weakened tolerance).**
  - **Host (no OCCT)** — inclusion-exclusion on the exact analytic bicylinder common
    `V(A ∩ B) = 16 R³/3` and the cylinder volumes `V = π R² L`:
    `FUSE = π R²(L_A + L_B) − 16 R³/3`, `CUT(A,B) = π R² L_A − 16 R³/3`. The host suite checks
    the assembled shell is watertight (`boundaryEdgeCount == 0`, every edge shared by exactly two
    faces), the enclosed volume matches the closed form within the deflection-sized band, every
    arc node lies on BOTH cylinders ≤ tol, and the two poles are pooled ONCE. Short-cylinder /
    non-Steinmetz fixtures return NULL (deferred).
  - **Sim native-vs-OCCT** — `scripts/run-sim-native-ssi-curved-boolean.sh` already exercises
    the `cyl=cyl(steinmetz)` pair across `{Fuse, Cut, Common}`; today FUSE/CUT are honest
    fall-backs (OCCT `volO` fuse `= 32.366`, cut `= 13.516`). After this change they become
    NATIVE passes vs `BRepAlgoAPI_{Fuse,Cut}` (volume + surface area + watertight closed shell +
    valid shape). The harness auto-detects native-vs-fall-back, so **native-pass rises 10 → 12**
    (the Steinmetz FUSE + CUT resolving native). No harness change is required — the two
    sub-cases flip from `[fallback]` to `[native]` when the builders ship.
- **Roadmap**: completes `SSI-ROADMAP.md` S5 Steinmetz to 3/3 native (COMMON + FUSE + CUT).
  Other S5 families and the S4 near-tangent moat remain the tail.
- **Risk (honest)**: a FUSE outside-lune misclassification, a CUT reversed-inner-lune
  orientation error, or a cap that clips the seam band yields a self-intersecting or
  wrong-signed shell — caught by the engine's mandatory watertight + correct-volume (fuse grows /
  cut shrinks vs the native bicylinder COMMON) self-verify, which DISCARDS the candidate → OCCT.
  A hairline arc-seam gap is caught by the watertight check at the mesher deflection ladder →
  discard → OCCT. If either op cannot be built watertight with the correct volume, the builder
  returns NULL → OCCT and the measured gap is reported. No case is faked, stubbed, or
  tolerance-weakened to pass. The COMMON path stays byte-identical (native-pass 10 not
  regressed).
