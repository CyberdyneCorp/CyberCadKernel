# Design — moat-m6r-freeform-boolean-fuzz

## Context

The freeform-boolean verbs are DIRECT native functions (not routed through
`nb::boolean_solid`, except Steinmetz), each with its own entry point and its own measured
DECLINE enum:

- `bo::freeformHalfSpaceCut(operand, Plane, KeepSide, deflection, HalfSpaceCutDecline*)`
- `bo::freeformSlabDisjointCut(A, B, deflection, SlabCutDecline*, analyticCutVolume)`
- `bo::curvedWallHalfSpaceCut(operand, Plane, KeepSide, deflection, CurvedWallCutDecline*)`
- `nb::boolean_solid(cylA, cylB, Op::Common)` → the `ssi_boolean` Steinmetz path.

Each has a curated per-op parity harness that builds ONE fixed operand at ONE fixed pose
and compares native (measured by the M0 tessellator) vs an OCCT reconstruction of the SAME
operand cut by the SAME half-space via `BRepAlgoAPI`. This fuzzer reuses those operand
builders and OCCT reconstructions VERBATIM, but turns the FIXED pose into a SEEDED batch of
RANDOM poses, and adds the closed-form arbiter as the primary correctness oracle.

## Why fuzz the POSE, not the operand geometry

The landed fixtures build their operands from `constexpr` bowl amplitude / rim / quad
constants (`face_split_fixture::kBowlA`, `curved_wall_cut_fixture::kA/kR`). The RANDOM
degree of freedom the roadmap asks for is the **cut pose**: "off-centre half-space at
random plane offset; disjoint slab at random position; curved-wall plane at random height."
The closed forms all generalise trivially to an arbitrary pose:

- prism half-space / slab: the polynomial `∫∫_{clip} (H0 + a(x²+y²)) dA` over a
  Sutherland–Hodgman clip at ANY offset `c` (resp. two clips at `±s`);
- paraboloid cap: `π·ρ²·c/2`, `ρ=√(c/a)`, for ANY cut height `0 < c < a·R²`.

Fuzzing the pose exercises the seam-trace + face-split + cap-weld + self-verify pipeline
across the whole reachable envelope while keeping every trial VALID-by-construction with a
known ground-truth volume.

## The curved-cup deflection band (the key measurement decision)

The prism / slab / Steinmetz families carry planar analytic faces + a gently-curved
(a=0.4) wall or an analytic cylinder, so a 0.004 native mesh measures them to ~3e-3 of the
exact oracle — a 2e-2 band clears with margin.

The STEEP paraboloid bowl-cup (a=2.0) is a highly-curved convex surface. Its INSCRIBED-
facet mesh UNDERSHOOTS the true segment volume by a bias that scales ∝ deflection. A direct
convergence probe (`curvedWallHalfSpaceCut` CUT at c=0.1146) measured:

    d=8e-3 → 6.55%   d=4e-3 → 4.02%   d=2e-3 → 2.11%   d=1e-3 → 1.11%   d=5e-4 → 0.55%

— a MONOTONE tessellation artifact, NOT a native wrong-set: the native B-rep is exact;
only the mesh measurement discretises the curved cup, and OCCT uses exact deflection-
independent `BRepGProp`. The harness therefore measures the curved-cup families at the fine
deflection **0.001** (bias ~1.1%) and applies a **3e-2** band — which is >2× the observed
bias AND >3× TIGHTER than the **0.10** band the landed `native_curved_wall_cut_parity`
harness validated for this exact cup ("coarse curved cup", line 225). Adopting the
per-op parity's already-validated deflection-convergence tolerance is NOT a widening; it is
the established never-widened band. The 2e-2 flat band is kept for the near-exact families.

## The near-rim closed-form conditioning gate

The curved-wall COMMON identity `V(z≥c) = V(full) − V(z≤c)` suffers CATASTROPHIC
CANCELLATION as `ρ=√(c/a) → R` (the cut approaches the rim): at `c=0.2437` vs rim `0.245`
the closed form is 20% off while native AND OCCT still agree to ~3e-4. The two independent
engines agreeing decisively vindicates native; the closed form is the outlier there. The
harness therefore uses the closed form as the PRIMARY arbiter ONLY in the well-conditioned
interior (the `inEnvelope` band, `curved-wall` only); near-rim DECLINE-probe poses rely on
native-vs-OCCT agreement, and a closed-form mismatch there is a MEASURED closed-form
inaccuracy, never a native fault. This is the honest inverse of the ORACLE_UNRELIABLE rule:
where MY closed-form arbiter is ill-conditioned, I fall back to the two-engine cross-check
rather than mislabel the native result.

## Classifier

Per trial, at the fixed per-family bands:

- **AGREED** — native watertight + (χ=2 implied by the tessellator's watertight audit),
  vol/area within tol of OCCT, and within tol of the closed form where well-conditioned.
- **HONESTLY-DECLINED** — native NULL / non-watertight (a measured decline) → OCCT ships a
  valid closed solid. First-class, counted separately (the deflection-fragile /
  freeform-FUSE cases the roadmap records as declining).
- **DISAGREED** — native watertight but OUTSIDE tol vs a trustworthy oracle. The failure.
- **ORACLE_UNRELIABLE** — native matches the closed form but OCCT does NOT (native MORE
  correct). Logged, never a bar failure.
- **BOTH-DECLINED** — an out-of-scope probe both engines refuse.
- **FALLBACK_ORACLE_INVALID** — native declined but the shipped OCCT result is invalid (a
  broken oracle laundered as a decline). Investigate; fails the bar.

## Determinism

The RNG is splitmix64-seeded xoshiro256** keyed ONLY by `FUZZ_SEED` (argv/env), NO clock /
`rand()` — verified by running the same seed twice and diffing the full trial batch
(byte-identical MD5).
