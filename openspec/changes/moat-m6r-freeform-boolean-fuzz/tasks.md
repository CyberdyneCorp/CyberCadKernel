# Tasks — moat-m6r-freeform-boolean-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_freeform_boolean_fuzz.mm` with the shared splitmix64 →
      xoshiro256** `Rng` (keyed ONLY by `FUZZ_SEED`), a per-family native mesh measurement
      (watertight + enclosedVolume + surfaceArea) and an OCCT measurement (`BRepCheck` +
      `BRepGProp`). Reuse the landed fixture operand builders (`ffx::buildOperand` prism,
      `cwx::buildOperand` bowl-cup) and OCCT reconstructions (Geom_BezierSurface prism /
      bowl-cup sewn to a solid) verbatim from the per-op parity harnesses.
- [x] 1.2 Implement the five pose-families — off-centre-halfspace (CUT + COMMON at random
      offset `x=c`), disjoint-slab (CUT at random half-width `s`), curved-wall-CUT and
      curved-wall-COMMON (at random height `z=c`), bicyl-COMMON (Steinmetz, random R) —
      driving BOTH the SHIPPING native freeform verb (`freeformHalfSpaceCut` /
      `freeformSlabDisjointCut` / `curvedWallHalfSpaceCut` / `nb::boolean_solid`) AND the
      OCCT reference boolean (`BRepAlgoAPI_{Cut,Common,Fuse}` on the SAME reconstruction),
      with a minority of out-of-envelope DECLINE probes (near-rim / too-wide slab / near-rim
      cut) and a FUSE that honestly declines.
- [x] 1.3 Implement the CLOSED-FORM arbiters generalised for an arbitrary pose: the
      polynomial prism integral `∫∫_{clip} (H0 + a(x²+y²)) dA` (half-space + two-lump slab),
      the paraboloid-cap `π·ρ²·c/2` and its complement (curved-wall), and Steinmetz
      `16 R³/3`; plus the partition identities as cross-checks.
- [x] 1.4 Implement the classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE_UNRELIABLE / BOTH-DECLINED / FALLBACK_ORACLE_INVALID) with FIXED never-widened
      PER-FAMILY bands (flat 2e-2 for prism/slab/Steinmetz; curved-cup 3e-2 measured at
      deflection 0.001) and the near-rim closed-form conditioning gate.
- [x] 1.5 Print a per-pose-family × op coverage table; `std::_Exit(0)` IFF `DISAGREED == 0`
      (and no OPERAND_MISMATCH / FALLBACK_ORACLE_INVALID); report any DISAGREE with seed +
      case index + family/op/pose tuple.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-freeform-boolean-fuzz.sh` cloned from
      `run-sim-native-boolean-fuzz.sh` (OCCT oracle + numsci substrate + the native math /
      ssi / numerics / ssi_boolean TUs under `-DCYBERCAD_HAS_NUMSCI`), seeded ONLY by
      `FUZZ_SEED`/argv, default N=72, booting/using the simulator.
- [x] 2.2 Add `native_freeform_boolean_fuzz.mm` to the `run-sim-suite.sh` SKIP list.
- [x] 2.3 Verify `scripts/build-numsci.sh host` and `iossim` both exit 0.

## 3. Gate + docs

- [x] 3.1 Run on the booted simulator over ≥2 seeds, N≥60/seed; confirm DISAGREED == 0 on
      every seed and each of the five pose-families with ≥1 AGREED.
- [x] 3.2 Verify determinism (same seed twice → byte-identical batch).
- [x] 3.3 Update `openspec/MOAT-ROADMAP.md` M6 row: breadth ×17 → ×18.
- [x] 3.4 Structural check: `git diff` touches only `tests/sim` + `scripts` + `openspec`
      (NOT `src/native`, `src/engine`, `include`), then commit to `moat-m6r`.
