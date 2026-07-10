# Proposal — moat-m6r-freeform-boolean-fuzz (MOAT M6-breadth-18, a freeform-boolean domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) has many landed differential-fuzzing domains — curved
booleans, STEP round-trip, construction/loft/sweep, blends, wrap/emboss, mesh
mass-properties, geometry-services, transform chains, reference/datum geometry,
direct-modeling, transformed-boolean, section curves, orthographic HLR, shape-healing,
curved-blend, draft-angle, interference/clash.

The ORIGINAL curved-boolean fuzzer (`tests/sim/native_boolean_fuzz.mm`) predates a LARGE
freeform-boolean surface that landed THIS campaign under `src/native/boolean/`:

- `half_space_cut.h` — the FIRST freeform↔analytic half-space CUT/COMMON (off-centre plane),
- `slab_disjoint_cut.h` — a disjoint / multi-lump slab CUT,
- `curved_wall_cut.h` — a dome/bowl closed-circular-seam CUT/COMMON,
- `freeform_freeform_cut.h`, `multi_face_weld.h`, `two_operand.h` — the two-operand welds,
- the `ssi_boolean` Steinmetz bicylinder COMMON reachable through `nb::boolean_solid`.

That surface is exercised ONLY by HAND-PICKED per-op parity fixtures
(`native_first_freeform_boolean_parity`, `native_curved_wall_cut_parity`,
`native_two_operand_freeform_boolean_parity`, `native_slab_disjoint_cut_parity`), each at
ONE FIXED pose. There is **no seeded differential fuzzer** that drives *random* valid cut
POSES across the freeform families and classifies every trial. This change closes exactly
that gap: it certifies the freeform-boolean surface the original fuzzer does not touch.

The freeform-boolean fuzzer is high-value for three reasons:

1. **It certifies a surface that just landed** — off-centre freeform half-space CUT/COMMON
   at a *random plane offset*, disjoint slab CUT at a *random half-width*, curved-wall
   dome/bowl CUT/COMMON at a *random cut height*, and the Steinmetz bicylinder COMMON —
   none previously under a *randomised* bar.
2. **A wrong freeform boolean is a silent-wrong-result the user cannot detect** — a
   watertight-but-wrong keep-side solid presents a corrupted cut with a wrong volume as a
   valid result. The bar searches the pose space for that.
3. **Closed-form is the PRIMARY arbiter.** Every family carries an engine-independent
   closed-form volume (a polynomial prism integral, a paraboloid-cap segment `π·ρ²·c/2`,
   the Steinmetz `16 R³/3`, and the partition identities `V(A∩B)+V(A−B)=V(A)`,
   `V(A∪B)=V(A)+V(B)−V(A∩B)`), so a native result matching exact math while OCCT is the
   outlier is vindicated, never a bar failure.

It shares NO code with the concurrent variable-sweep / N-fill fuzzers or the
interference.h edge-edge fix; it fuzzes only the STABLE landed freeform-boolean verbs.

## The oracles (closed-form the PRIMARY arbiter, OCCT the cross-check)

- **off-centre half-space** — the bowl-lidded convex-quad PRISM cut by `x = c` at a random
  offset. `V(A∩{x≤c}) = ∫∫_{Q∩{x≤c}} (H0 + a(x²+y²)) dA` is an EXACT polynomial over the
  clipped footprint (Sutherland–Hodgman + per-triangle second-moment integral), for both
  CUT (keep x≤c) and COMMON (keep x≥c). PRIMARY arbiter.
- **disjoint slab** — the SAME prism parted by a central slab `x∈[−s,s]` at a random
  half-width. `V(A−B) = V(A∩{x≤−s}) + V(A∩{x≥+s})` — the two-lump sum of the same
  polynomial. PRIMARY arbiter.
- **curved-wall CUT / COMMON** — a STEEP Bézier bowl-cup (paraboloid `z = a·r²` lidded at
  `z = a·R²`) cut by the horizontal plane `z = c` at a random height. `V(z≤c) = π·ρ²·c/2`
  with `ρ = √(c/a)` (paraboloid-segment cap); `V(z≥c) = V(full) − V(z≤c)`,
  `V(full) = π·a·R⁴/2`. PRIMARY arbiter in the well-conditioned interior.
- **bicyl Steinmetz COMMON** — equal-R orthogonal cylinders. `V = 16 R³/3`. PRIMARY arbiter.

The **OCCT oracle** reconstructs the SAME operand (a `Geom_BezierSurface` prism / bowl-cup
sewn into a solid; two `BRepPrimAPI_MakeCylinder`) and applies the reference boolean via
`BRepAlgoAPI_{Cut,Common,Fuse}`, measured by exact deflection-independent `BRepGProp`. Both
the native mesh and OCCT are compared to the closed form.

## What changes

- **ADD** `tests/sim/native_freeform_boolean_fuzz.mm` — a deterministic seeded (splitmix64
  → xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per trial: picks a
  pose-family, draws a random VALID cut pose in the reliable interior band (with a minority
  of out-of-envelope DECLINE probes), runs it through BOTH the SHIPPING native freeform
  verb (`freeformHalfSpaceCut` / `freeformSlabDisjointCut` / `curvedWallHalfSpaceCut` /
  `nb::boolean_solid`-Steinmetz) AND the OCCT oracle AND the closed-form arbiter; compares
  volume / area / watertight / Euler χ=2; and classifies AGREED / HONESTLY-DECLINED /
  DISAGREED / ORACLE_UNRELIABLE / BOTH-DECLINED.
- **ADD** `scripts/run-sim-native-freeform-boolean-fuzz.sh` — cloned from
  `run-sim-native-boolean-fuzz.sh` (links the OCCT oracle + the numsci substrate the M1
  seam tracer consumes; compiles the native math + ssi/{seeding,marching} + numerics +
  boolean/ssi_boolean TUs under `-DCYBERCAD_HAS_NUMSCI`); seeded ONLY by `FUZZ_SEED`/argv;
  default N=72.
- **ADD** `native_freeform_boolean_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
  `main()`, OCCT+numsci slice, `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×17 → ×18.

## The bar

DISAGREED == 0 across ≥2 seeds, N≥60 trials/seed, each of the five pose-families
(off-centre-halfspace / disjoint-slab / curved-wall-CUT / curved-wall-COMMON / bicyl-COMMON)
with ≥1 AGREED trial. The FIXED per-family tolerances (native-vs-OCCT & native-vs-closed
volume/area 2e-2 for the near-exact polynomial prism / slab / Steinmetz families; 3e-2 for
the STEEP paraboloid bowl-cup measured at the fine deflection 0.001 — a band >3× TIGHTER
than the 0.10 the landed `native_curved_wall_cut_parity` harness validated for this exact
cup) are NEVER widened to force a pass. An honest native NULL → OCCT decline is first-class.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). If the fuzzer surfaces a real native
  boolean bug it is REPORTED, not fixed here; a native result more correct than OCCT at a
  numeric edge is classified ORACLE_UNRELIABLE (native vindicated), not DISAGREED.
- FUSE of a freeform operand is an honest domain-level decline for this slice (no landed
  freeform-FUSE verb with a closed-form arbiter) → native NULL → OCCT, counted separately.
- Does NOT fuzz the concurrent variable-sweep / N-fill tracks nor touch interference.h —
  only the STABLE landed freeform-boolean verbs.
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
