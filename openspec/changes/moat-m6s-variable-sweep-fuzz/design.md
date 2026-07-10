# Design — moat-m6s-variable-sweep-fuzz

## Context

The M6 completeness bar drives random valid inputs through BOTH the native path and the
OCCT oracle and classifies every trial, requiring `DISAGREED == 0`. This session landed a
NEW native surface: the VARIABLE-SECTION / guide+spine sweep
(`src/native/construct/sweep.h` `build_variable_sweep` / `build_variable_sweep_tube`)
reached through the shipping `cc_variable_sweep` facade. The curated
`native_vsweep_parity.mm` harness proves both engines on FOUR hand-picked fixtures + one
deferred case; this change turns it into a *seeded, randomized batch* with a per-trial
classifier over random morph profiles, spines, and optional guide rails.

## The closed-form arbiter (why native-vs-OCCT alone is insufficient)

A sweep has no single trustworthy engine oracle: OCCT keeps a TRUE analytic B-rep from
`BRepOffsetAPI_MakePipeShell` while the native arm welds a facet tube, and a
native-vs-OCCT-only test cannot distinguish a native miss from an OCCT pipe-shell drift.
The design removes that with an exact closed form for the STRAIGHT-spine families.

A straight-spine variable sweep places, at each fraction f∈[0,1], the planar section
polygon `P(f)` whose vertex k is `(A_k + (B_k − A_k)·f) · s(f)`, where `s(f)` is the
guide-splay scale (`≡1` with no guide), in a fixed perpendicular frame, and rules the
lateral surface between adjacent stations. The cross-sectional signed area `A(f) =
polyArea(P(f))` is a polynomial in f of degree ≤ 4 (the vertex blend contributes ≤2, the
`s(f)²` scale contributes ≤2), so the EXACT volume is

> **`V = ∫₀¹ A(f) · H df`**,  H = the straight spine length,

evaluated by a **composite Simpson over 4 sub-intervals (5 samples)** — analytically exact
for a quartic. This handles the general vertex-k→vertex-k A→B morph AND the guide scale in
one integral, with no per-face assumptions. For the circle-morph family the SMOOTH
truncated-cone volume `πH/3(r0² + r0 r1 + r1²)` is used; the native 64-gon profile
inscribes it, so the fixed circle band absorbs that geometry-driven bias. The CURVED-arc
family has no simple closed form and is arbitrated against OCCT alone (a deflection-bounded
band) plus the engine-independent watertight / Euler χ=2 invariants.

Because the straight-spine closed form is exact, a native result matching it while OCCT is
the outlier is logged ORACLE-INACCURATE (native vindicated), never a bar failure.

## The native guided envelope — a REPORTED product limitation (localized here)

Bring-up surfaced a genuine native limitation, localized to the guided-straight family.
The native straight-spine guided builder (`build_variable_sweep_tube`, `nStations == 2`
for a straight spine) places the morphed+guide-scaled section only at f=0 and f=1 and
**rules LINEARLY** between them. Two independent oracles — OCCT's 24-section densified
`MakePipeShell` and the exact polygon-clip Simpson integral — AGREE to ~1e-4, while native
diverged 1–20% scaling with the guide splay ratio. First-principles analysis pins the
mechanism:

- **Constant section (A==B) + any guide scale** → native EXACT. The two-station ruling is
  a similar-polygon frustum, and the prismatoid `(H/3)(A0 + √(A0·A1) + A1)` is exact.
- **Any morph (A≠B) + constant guide scale (s≡1)** → native EXACT (the plain morph).
- **Coupled morph (A≠B) AND varying scale (s not constant)** → native DROPS a cross-term.
  The mid cross-section of the linear ruling is `0.5·s0·A + 0.5·s1·B`, whereas the true
  continuously-scaled mid section is `s(0.5)·mid(A,B)`; they differ by
  `0.25·(1−k)·(A−B)` (`k = s1/s0`), non-zero only in the coupled regime.

This is native being **LESS correct** (not oracle-inaccurate). It is REPORTED as a product
limitation — the straight-spine guided variable-sweep needs ≥3 stations to track a coupled
morph×scale law — and the product code is byte-unchanged. The fuzzer certifies the guided
family only in its two EXACT sub-regimes (constant section + splaying guide; morphing
section + guide parallel to the spine so `s≡1`), both genuinely served correctly, and
documents the coupled regime rather than laundering it with a widened tolerance.

## Detecting a native decline through the facade

`cc_variable_sweep` under `cc_set_engine(1)` forwards to OCCT on an out-of-envelope pose,
so the facade never returns 0 to distinguish a native decline from a native success. To
keep the HONESTLY-DECLINED bucket meaningful, the harness makes a **read-only probe** of
the native BUILDER directly (`ncst::build_variable_sweep`, OCCT-free, same args): a NULL
`topo::Shape` means the native arm declined and the facade shipped the OCCT delegate. This
probe never ships — the arbitrated body is always the FACADE result — and keeps AGREED
strictly the native-produced solids.

The decline probes (interleaved at a fixed 1-in-6 cadence) each trip a native builder
precondition guard verified to yield NULL while OCCT still ships: mismatched vertex counts
(`aCount != bCount`), a coincident guide start (`d0 < 1e-6`), and a non-planar (helical)
guided spine (the `spineIsStraight`/`spineIsPlanar` guard). A collapsing-guide-scale case
is deliberately NOT a decline probe: native's self-fold guard does not trip on a
strong-but-valid taper, so native builds it while OCCT can fail — that is neither a native
decline nor a trustable-oracle trial, and it is documented, not fuzzed.

## The six-way classifier

Identical discipline to the landed siblings. Per trial, gating on watertight + Euler χ=2 +
positive volume, arbitrated by the closed form (straight families) or OCCT alone (curved):

- **AGREED** — native produced a valid body matching the truth within the fixed band, and
  the oracle concurs.
- **HONESTLY-DECLINED** — the native builder returned NULL / forwarded to OCCT, and OCCT
  shipped a valid result. First-class, never a bar failure.
- **DISAGREED** — native valid but outside the truth while the oracle matches it — a silent
  wrong sweep. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed form while OCCT does not (native
  vindicated). Logged, not a failure.
- **ORACLE_UNRELIABLE** — a native miss where the oracle is also untrustworthy. FAILS the
  bar (investigate, never laundered).
- **BOTH-DECLINED** — an out-of-envelope pose both engines refuse. Logged, not a failure.

## The bar & discipline

Exit 0 IFF `DISAGREED == 0 && ORACLE_UNRELIABLE == 0`, each of the five families with ≥1
AGREED, across ≥2 seeds at N≥60/seed (the runner fails if any seed fails). The fixed bands
(native-vs-closed-form ≤ 2e-3 polygon / ≤ 1.2e-2 circle; native-vs-OCCT ≤ 5e-2 volume,
≤ 8e-2 area) are NEVER widened. `src/native/**`, `src/engine/**`, `include/**`, and the
`cc_*` ABI stay byte-unchanged.
