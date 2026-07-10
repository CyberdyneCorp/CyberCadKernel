# Proposal — moat-vsfix-coupled-morph (native variable-sweep: coupled morph×scale)

## Why

The M6 variable-sweep differential fuzzer (`moat-m6s`, `native_vsweep_fuzz.mm`) surfaced,
and confirmed by first-principles math plus two independent agreeing oracles, a genuine
correctness gap in the newly-native variable-section sweep
(`src/native/construct/sweep.h` `build_variable_sweep_tube`): on a **STRAIGHT** spine the
GUIDED builder collapsed to `nStations = 2` and ruled the section LINEARLY between the two
end stations. That two-station linear ruling reproduces the continuous guide-scale law
EXACTLY in the two disjoint sub-regimes — {constant section (A==B) + splaying guide} and
{morphing section + spine-parallel guide (scale ≡ 1)} — but DROPS the morph×scale
**cross-term** when BOTH the section morphs (A→B) AND the guide scale varies
simultaneously.

The section radius at spine fraction `f` is `g(f) = morph(f)·scale(f)`, a PRODUCT of two
f-varying terms; the swept section area is `∝ g(f)²` (degree ≤ 4 in f). A 2-station chord
approximates `g` linearly, so its area varies as `(chord g)²` and `∫(chord g)² ≠ ∫g²`
whenever `g` is non-linear. This was measured as a **1–20% native volume divergence**
(scaling with the guide splay) versus OCCT's 24-section densified `MakePipeShell` AND an
exact polygon-clip integral (the two oracles agree to ~1e-4). Native was EXACT in the two
sub-regimes; only the COUPLED case was wrong. `moat-m6s` reported this as a native
limitation and certified the guided family only within the exact envelope.

This change FIXES the coupled case natively so the guided variable sweep is correct
whenever the section both morphs and is guide-scaled.

## What changes

- **FIX** `src/native/construct/sweep.h` `build_variable_sweep_tube`: the STRAIGHT-spine
  guided path is DENSIFIED to `≥ 3` stations when the coupled morph×scale law is
  non-linear, so the product's curvature is tracked (each station's section =
  `interpolate(A,B,f)` scaled by the guide splay at `f`). A new helper
  `straightCoupledStations(rA, rB, guideScaleAt)` chooses the fewest UNIFORM bands whose
  piecewise-linear-`g` area integral `∫(chord g)²` matches the true `∫g²` to within a FIXED
  relative-volume bound `kMaxCoupledVolErr` (0.2%), driven by the section circumradius
  envelope and the guide-scale samples. When `g` is LINEAR — the two exact sub-regimes — the
  chord equals `g`, the integral error is zero, and it returns `2` (byte-identical to the
  old path). The curved-spine path additionally raises its turn-driven station count to the
  coupled count so a curved coupled morph tracks the cross-term too. The perpendicular /
  RMF framing, `assembleRingTube`, and the self-fold guard are reused BYTE-IDENTICALLY.
- **ADD** two host regression cases to `tests/native/test_native_vsweep.cpp`: a coupled
  circle→circle morph + splaying guide on a straight spine whose native volume now matches
  the EXACT polygon-clip closed form `C·H·∫₀¹ g(f)² df` (`g² ` degree-4, integrated in
  closed form) to a tight tolerance, and a convergence case asserting the residual does not
  grow as the polygon refines. The two exact sub-regime cases (pure morph frustum, pure
  guide-scaled frustum) are retained and asserted to STAY exact.
- **ADD** a coupled morph×scale straight parity case to `tests/sim/native_vsweep_parity.mm`
  (native vs OCCT `MakePipeShell` multi-section within the deflection band).
- **BROADEN** `tests/sim/native_vsweep_fuzz.mm`: remove the coupled-regime scope
  restriction on the `GUIDED_STRAIGHT` family — it now draws THREE regimes (constant+splay,
  morph+parallel, and the COUPLED morph+splay), all arbitrated against the EXACT closed
  form. The closed-form quadrature is upgraded from composite Simpson (a ~5e-4 quartic
  residual) to **Boole's rule** (exact for the ≤quartic section area, including the
  cross-term).
- **UPDATE** `openspec/MOAT-ROADMAP.md`: mark the vsweep coupled morph×scale case native and
  the `M6-breadth-19` documented limitation RESOLVED.

## The oracles / two-gate model

- **GATE (a) HOST-analytic:** the coupled morph×scale volume matches the exact polygon-clip
  closed form `C·H·∫₀¹ [(r0+Δr·f)(1+Δs·f)]² df` (C the exact N-gon area factor), converging
  as station count / polygon count increase. The two exact sub-regimes stay exact. Full host
  ctest all-green.
- **GATE (b) SIM native-vs-OCCT:** the coupled case native vs OCCT
  `BRepOffsetAPI_MakePipeShell` multi-section + `BRepGProp` within the deflection band; no
  existing vsweep parity case regresses; the broadened fuzzer certifies the coupled regime
  against the exact closed form across seeds (0 DISAGREED).

## Non-goals / discipline

- `src/native/**` stays **OCCT-FREE**; the fix touches ONLY
  `src/native/construct/sweep.h`. The `cc_*` ABI is UNCHANGED (an internal fix to
  `build_variable_sweep_tube`; `cc_variable_sweep`'s signature is unchanged). The
  tessellator, boolean, blend, analysis, interference, and exchange modules are UNTOUCHED.
- HONEST: the geometry is fixed; **no tolerance is widened** to mask it. The two-sided
  volume self-verify is reused. The regression proves convergence, not a loosened band.
- Non-planar guided spines still DECLINE → OCCT (the genuine corrected-Frenet law the RMF
  frame does not reproduce); every existing honest-decline stays a decline.
