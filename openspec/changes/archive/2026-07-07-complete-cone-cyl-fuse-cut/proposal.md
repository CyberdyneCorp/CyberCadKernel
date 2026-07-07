## Why

`SSI-ROADMAP.md` S5 is the payoff — general curved booleans driven by the S3/S4/S1 seam trace.
The archived `add-native-cone-boolean` (S5-e) opened the CONE surface family with the
analytically-cleanest cone boolean: the **coaxial cone(frustum)∩cylinder COMMON**
(`buildConeCylCommon`). It consumes the S1 analytic circle seam (`intersectCylinderConeCoaxial`,
`nearTangentGaps == 0`, `branchPoints == 0`), splits the two coaxial walls at the single crossing
circle `s*` (where the cone cross-section radius `r_c(s*)` equals the cylinder radius `Rc`), keeps
the two INSIDE-the-other bands (cone band inside the cylinder + cylinder band inside the cone),
welds them along the pooled seam circle, and closes with two disc caps — verified against OCCT
`BRepAlgoAPI_Common` AND the exact closed-form volume `V_frustum(r(sLo) → Rc) + π Rc²·(sHi − s*)`.
For the sim fixture (cone `r_c(y) = 0.5 + 0.5y` over `[0,4]`, coaxial cylinder `Rc = 1.5` over
`[1,5]`, seam `y* = 2` in the overlap `[1,4]`) that is a watertight native COMMON (native `19.107`
vs analytic/OCCT `19.111`), raising the sim native-pass to **13**.

But `add-native-cone-boolean` shipped **COMMON only** for the coaxial cone∩cylinder pair — its own
honest-scope note explicitly defers FUSE / CUT to a follow-on, and the dispatcher confirms it:
`ssi_boolean_solid`'s `Op::Fuse` arm runs `buildFuse` (through-drill, declines a single seam) then
`buildLensFuse` (declines a non-sphere operand) and returns NULL; `Op::Cut` runs `buildCut` then
`buildLensCut` and returns NULL. So the coaxial cone∩cylinder family is **1/3 native** (COMMON),
while FUSE / CUT fall through to OCCT `BRepAlgoAPI_{Fuse,Cut}` (honest, verified fall-backs — but
fall-backs, `volO` fuse `= 41.626`, cut `= 13.352`).

The GAP is again the **assembler**, not recognition or tracing: `recogniseCurvedSolid` +
`intersectCylinderConeCoaxial` already fold the pair and closed-form the seam, and the SAME split
machinery that `buildConeCylCommon` uses to weld the two inside bands is exactly what FUSE and CUT
need — the SAME analytic circle seam, a DIFFERENT fragment (band) selection plus the operand caps
that bound the union / difference (and, for CUT, the reversed inside fragment). This change
completes the set: the coaxial cone∩cylinder family becomes **3/3 native**, mirroring the archived
`complete-steinmetz-fuse-cut` (Steinmetz COMMON → FUSE → CUT) and `complete-sphere-sphere-fuse-cut`
(sphere lens COMMON → FUSE → CUT).

## What Changes

The geometry (a coaxial frustum cone A and cylinder B sharing one axis, overlapping so their walls
cross at a single circle at `s*`; measure axial height `s` along the cone axis; `r_c(s) = R0 +
s·tanα` is the cone cross-section radius, `Rc` the cylinder radius; each wall is split by the seam
into an INSIDE-the-other axial band and an OUTSIDE band, and each operand carries its two original
disc end caps at its axial extents):

- **COMMON** `A ∩ B` (already native, `buildConeCylCommon`) = the min-radius-profile solid of
  revolution over the overlap `[sLo, sHi]`: the CONE band on the `r_c ≤ Rc` side (inside the
  cylinder) welded along the seam circle to the CYLINDER band on the `Rc ≤ r_c` side (inside the
  cone), closed by two disc caps. Volume `V_frustum(r(sLo) → Rc) + π Rc²·(sHi − s*)`.
- **FUSE** `A ∪ B` (new, `buildConeCylFuse`) = the **max-radius-profile** solid of revolution over
  the full union extent `[min(coneLo,cylLo), max(coneHi,cylHi)]`: the OUTER wall of whichever
  operand is wider at each `s` (cylinder wall on the cone-inner side, cone wall on the cyl-inner
  side, welded at the SAME seam circle), plus the wall segments beyond the overlap (each operand's
  wall where the other is absent), closed by the two terminal disc caps AND the annular step caps
  where one operand's end-cap disc protrudes past the other. Volume `V(A) + V(B) − V(A ∩ B)`.
- **CUT** `A − B` (new, `buildConeCylCut`, `A` the minuend) = A's OUTER wall (the part of A outside
  B) + A's disc caps + the annular part of A's cap outside B, joined to B's INSIDE band emitted
  **REVERSED** (inward radial normal, bounding the carved cavity) welded at the seam circle, plus
  B's cap disc inside A emitted reversed. Volume `V(A) − V(A ∩ B)`. CUT is order-sensitive and, for
  the reference fixture, is a DISCONNECTED result (a small end frustum where the cylinder scoops A
  fully through, plus a conical washer) — assembled as one shell of two closed components sharing
  the pool, whose summed mesh volume the self-verify checks.

Concretely, reusing the S5-e seam/split/weld (`recogniseCurvedSolid`, `sameAxis`, the analytic
`s*` crossing, the shared `VertexPool`, the pooled seam ring, `appendRevolvedBand`, `appendDiskCap`,
`classifyPoint`):

- **Factor the shared prologue.** Extract `coneCylSetup(A, B, seams)` from `buildConeCylCommon` —
  the gate (one `Cone` + one coaxial `Cylinder`, single strictly-interior full-circle seam, apex-
  free frustum, analytic-vs-traced seam cross-check), the azimuth resolution `N`, the shared frame
  `(O, ẑ, X, Y)`, the `ring(r, s)` functor, and the pooled seam ring at `(Rc, s*)`. All three ops
  reuse it byte-for-byte; `buildConeCylCommon` stays byte-identical (native-pass 13 not regressed).
- **Add `appendAnnulusCap(ringInner, ringOuter, axialOutward, pool, faces)`** — a flat annular ring
  (washer) between two coaxial same-station rings, axial (`±ẑ`) normal, sharing both rings through
  the pool. It closes a radial step where one operand's end-cap disc protrudes past the other
  (FUSE) or where A's cap is trimmed by B (CUT). Mirrors `appendDiskCap`'s planar-facet discipline.
- **Add `buildConeCylFuse(A, B, seams)`** — mirror `buildConeCylCommon`: same `coneCylSetup`, same
  pooled seam ring, but emit the MAX-radius outer profile (outer wall of the wider operand on each
  side, welded at the seam), the beyond-overlap wall segments, the two terminal disc caps, and the
  annular step caps. Survival rule per band: an interior sample classifies strictly OUTSIDE the
  other solid (`classifyPoint(other, mid) == -1`). Volume of the welded shell = `V(A)+V(B)−V(A∩B)`.
- **Add `buildConeCylCut(A, B, seams)`** — mirror `buildConeCylCommon`; `A` is the minuend (CUT is
  order-sensitive, matches `BRepAlgoAPI_Cut(a, b)`). Emit A's OUTER wall (outside B) + A's disc
  caps + A's cap-annulus outside B, plus B's INSIDE band emitted REVERSED (inward radial) welded at
  the seam + B's cap disc inside A reversed. Survival rule: A's outer band OUTSIDE B AND B's inner
  band INSIDE A. Volume of the welded shell = `V(A) − V(A∩B)`.
- **Driver dispatch** (`ssi_boolean_solid`): dispatch `Op::Fuse` → `buildConeCylFuse` (after
  `buildFuse` / `buildLensFuse` decline) and `Op::Cut` → `buildConeCylCut` (after `buildCut` /
  `buildLensCut` decline), mirroring the existing `Op::Common` → `buildConeCylCommon`. Recognition,
  trace, the transversality gate, and every other builder are UNCHANGED — only the `Op::Fuse` /
  `Op::Cut` arms grow one final call. A non-(cone+coaxial-cylinder) pair keeps its existing path.
- **Engine self-verify — NO new oracle, NO sign change needed.** `ssiCurvedBooleanVerified`
  (`native_engine.cpp`) applies the analytic closed-form oracle ONLY to `op == 2` (COMMON), so it
  does NOT intercept cone∩cylinder FUSE/CUT. The generic set-algebra guard `booleanResultVerified`
  therefore runs: `vc = watertightVolume(boolean_solid(a, b, Op::Common))` = the native
  `buildConeCylCommon` = `V(A ∩ B)`, and `expected = va + vb − vc` (fuse, GROWS) / `va − vc` (cut,
  SHRINKS). A mis-selected band, a mis-oriented reversed fragment, a mis-placed cap, or a hairline
  seam gap yields the wrong volume, FAILS the guard, and is DISCARDED → OCCT, never faked.

## Capabilities

### New Capabilities
<!-- none — this change COMPLETES the living native-booleans coaxial cone∩cylinder op-set (COMMON
already native via the S5-e assembler) by adding native FUSE + CUT, reusing the S1 analytic circle
seam, the shared-pool planar-facet weld, and the engine self-verify already established for the cyl
/ sphere / Steinmetz / cone-COMMON families, and re-affirms the S1 analytic circle-seam contract as
the shared input to all three cone∩cylinder ops. -->

### Modified Capabilities
- `native-booleans`: complete the coaxial cone∩cylinder op-set — add native **Fuse** (the max-
  radius-profile union: the wider operand's outer wall on each side of the seam + the beyond-overlap
  wall segments + the two terminal disc caps + the annular step caps where an end-cap disc
  protrudes) and **Cut** (`A − B`: A's outer wall + A's caps + A's cap-annulus outside B, joined to
  B's inside band emitted REVERSED + B's cap disc inside A reversed) alongside the already-native
  **Common** (the min-radius inside bands), all welded along the SAME single S1 analytic seam circle
  with the shared-pool planar-facet revolve discipline, guarded by the engine's EXISTING generic
  set-algebra self-verify (fuse grows, cut shrinks, vs the native cone∩cylinder COMMON `V(A ∩ B)`)
  → OCCT fallback. Reuse the S5-e seam/split/weld; add a flat annular-cap helper; the COMMON path
  stays byte-identical. Both new ops DECLINE (NULL → OCCT) outside their verified envelope: an apex-
  crossing / apex-in-extent frustum, a non-coaxial (transversal) pair, a cap-edge-tangent seam,
  cone∩cone, and cone∩sphere all remain the OCCT boundary. Other pairs unchanged. No `cc_*` change;
  compiled under `CYBERCAD_HAS_NUMSCI`; `src/native/**` stays OCCT-free.
- `native-ssi`: re-affirm the S1 analytic coaxial cone∩cylinder circle seam
  (`intersectCylinderConeCoaxial`) as the consumed input contract for ALL THREE cone∩cylinder ops —
  the SAME single closed full-turn circle at `s*` splits each coaxial wall into an inside/outside
  axial band; COMMON / FUSE / CUT differ only in WHICH bands survive, their orientation, and
  (FUSE/CUT) the operand end caps and annular steps. The tracer does not change; the transversal
  (non-coaxial) cone pair (a quartic space curve, `notAnalytic` here) and the apex-crossing seam
  remain the honest decline boundary → OCCT.

## Impact

- **Scope (bounded completion).** This change ONLY completes the coaxial cone∩cylinder op-set
  (COMMON already native via S5-e; add FUSE + CUT). It is the SAME analytic circle seam with a
  different fragment selection + cap handling — NOT an open-ended new family. All other pairs
  (through-drill cyl∩cyl, sphere∩sphere lens, Steinmetz bicylinder, coaxial cone∩sphere, cone∩cone,
  transversal cone∩cylinder, freeform) are UNCHANGED and keep their existing native/decline
  behaviour.
- **ABI**: none. Invoked behind the existing `cc_boolean` op codes; no `cc_*` entry point,
  signature, or POD struct changes. Additive only.
- **Build**: extends `src/native/boolean/ssi_boolean.cpp` only — factor `coneCylSetup` out of
  `buildConeCylCommon`, add `appendAnnulusCap`, add `buildConeCylFuse` + `buildConeCylCut`
  (mirroring `buildConeCylCommon`), extend the `Op::Fuse` / `Op::Cut` dispatch arms. Reuses
  `recogniseCurvedSolid`, `sameAxis`, `classifyPoint`, `appendRevolvedBand`, `appendDiskCap`,
  `pushPlanarTri`, `VertexPool`, and the S1 `intersectCylinderConeCoaxial` seam — all already
  present. No new files. Compiled under `CYBERCAD_HAS_NUMSCI`. `src/native/**` stays OCCT-free. No
  change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic `curved.h`,
  the cyl / sphere / Steinmetz builders, or `buildConeCylCommon` (byte-identical). NO engine change
  — the generic set-algebra self-verify already covers fuse/cut with the correct per-op sign.
- **Verification (two gates, dual oracle, no weakened tolerance).**
  - **Host (no OCCT)** — inclusion-exclusion on the exact closed-form cone∩cylinder common
    `V(A ∩ B) = V_frustum(r(sLo) → Rc) + π Rc²·(sHi − s*)` and the operand volumes (`V(cone
    frustum) = (π Δh/3)(r0² + r0·r1 + r1²)`, `V(cyl) = π Rc² L`): `FUSE = V(A) + V(B) − V(A ∩ B)`,
    `CUT(A,B) = V(A) − V(A ∩ B)`. The host suite asserts the assembled shell is watertight
    (`boundaryEdgeCount == 0`, every edge shared by exactly two faces), the enclosed (summed, for a
    disconnected CUT) volume matches the closed form within the deflection-sized band, every seam-
    ring node lies on BOTH walls ≤ tol, and the seam ring is pooled ONCE. Apex-in-extent /
    transversal / cap-tangent fixtures return NULL (deferred). Green with NUMSCI on AND off (the
    cone fuse/cut path correctly absent off).
  - **Sim native-vs-OCCT** — `scripts/run-sim-native-ssi-curved-boolean.sh` +
    `tests/sim/native_ssi_curved_boolean_parity.mm` already exercise the `cone=cyl(coax)` pair
    across `{Fuse, Cut, Common}`; today FUSE/CUT are honest fall-backs (OCCT `volO` fuse `= 41.626`,
    cut `= 13.352`). After this change they become NATIVE passes vs `BRepAlgoAPI_{Fuse,Cut}` (volume
    + surface area + watertight closed shell + valid shape), raising **native-pass 13 → 15** (the
    cone∩cylinder FUSE + CUT resolving native). Do NOT regress the 13 existing native passes (incl.
    cone∩cylinder COMMON). Any pair whose self-verify does not pass stays an honest fall-back with
    the measured gap reported.
- **Roadmap**: completes `SSI-ROADMAP.md` S5-e coaxial cone∩cylinder to 3/3 native (COMMON + FUSE +
  CUT). Records the measured deltas in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md`.
- **Risk (honest)**: a FUSE outer-band misclassification, a CUT reversed-inner-band orientation
  error, a mis-placed annular cap, or a hairline seam-ring gap yields a self-intersecting or wrong-
  signed shell — caught by the engine's mandatory watertight + correct-volume (fuse grows / cut
  shrinks vs the native cone∩cylinder COMMON) self-verify, which DISCARDS the candidate → OCCT. An
  apex-crossing or transversal seam is refused up front (gate) → OCCT. If either op cannot be built
  watertight with the correct volume, the builder returns NULL → OCCT and the measured gap is
  reported. No case is faked, stubbed, or tolerance-weakened. The cyl / sphere / Steinmetz paths and
  `buildConeCylCommon` stay byte-identical (native-pass 13 not regressed).
