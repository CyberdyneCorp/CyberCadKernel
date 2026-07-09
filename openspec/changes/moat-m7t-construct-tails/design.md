# Design — moat-m7t-construct-tails

## Context

Both target ops already have a native code path that declines the interesting case.
The substrate (`detail::frenetSectionFrames`, `detail::rmfFrames`,
`detail::sectionRing`, `detail::assembleRingTube`, `detail::sectionSweepUnsafe`,
`detail::spineTooSharp`) is landed and proven by `guided_sweep`,
`guided_orient_sweep`, and `solid_loft_sections`. The task is to route the real-twist
and curved-rail cases through that substrate with a density that welds watertight and
converges to the analytic volume, and to keep an honest measured decline for the
genuinely intractable inputs.

## Empirical grounding (bench, host, OCCT-free)

Before writing the change, both cases were measured with a throwaway harness calling
the native `detail::` helpers directly.

**Real twist** (4×4 square, area 16, straight spine L=10, so untwisted V=160),
densified to `nStations`, `assembleRingTube`, watertight + enclosed volume across
deflections {0.05, 0.01, 0.002}:

| nStations | twist π/2 vol | twist π vol | watertight |
|-----------|---------------|-------------|------------|
| 2         | 106.5         | 53.3        | yes        |
| 16        | 159.55        | 158.66      | yes        |
| 64        | 159.84        | 159.77      | yes        |
| 128       | 159.83        | 159.84      | yes        |

The tube is watertight at *every* density; the single-band (`nStations=2`) volume is
badly under-filled because a straight ruled band chords across the smoothly-twisted
section — exactly the reason the op must densify. Densifying converges to the
area-preserving analytic 160. (The recorded "does not weld" decline was wrong; the real
error was the missing densification.)

**Curved rail** (regular 32-gon of circumradius 3 swept along a quarter-arc of radius
R=20, Pappus V = polyArea · R · φ = 882.57), RMF-transported, `assembleRingTube`:

| railStations | vol | rel err | watertight |
|--------------|-----|---------|------------|
| 4            | 852.7 | 3.4e-2 | **no** |
| 8            | 876.0 | 7.5e-3 | **no** |
| 16           | 881.0 | 1.7e-3 | yes |
| 32           | 882.2 | 4e-4   | yes |
| 128          | 882.5 | 3e-5   | yes |

A coarse rail (per-band turn too large) fails watertight — that is the honest decline
boundary; the engine `robustlyWatertight` self-verify catches it and forwards to OCCT.
At a bounded per-band turn (rail ≥ ~16 stations for a quarter-arc) the tube welds and
converges to Pappus.

## Decisions

### D1 — Densify by a per-band turn/twist bound (not a fixed count)

Both builders internally resample the (straight) spine / (curved) rail so each band's
incremental rotation stays under a small bound `kMaxBandTwist` (twist) / `kMaxBandTurn`
(rail), mirroring the already-landed `guided_orient_sweep` densification
(`kMaxPerBand ≈ 0.05 rad`). Concretely:

- twisted_sweep: `nBands = ceil(|totalTwist| / kMaxBandTwist)`, then also honor the
  original path stations (union) so a caller-supplied bent spine is respected. The
  straight-spine typical case collapses to `nStations = nBands + 1` evenly along the
  chord.
- loft_along_rail: pre-sample the RMF frames, measure the total tangent turn, set
  `nStations = ceil(totalTurn / kMaxBandTurn) + 1`, resample the rail by arc length.

The bound is chosen so the ruled bands weld (measured ≥ 16 for a quarter-arc); a rail
too tight to reach a welding density within a cap (`≤ 512` stations) falls to the
engine self-verify → OCCT.

### D2 — The Frenet/twist law must match the OCCT oracle exactly

`detail::frenetSectionFrames` + `detail::sectionRing` already reproduce OCCT
`twisted_sweep`'s literal section placement (`up=(0,1,0)`, `nrm = tan×up` with +X
fallback, `u' = u·ca − v·sa`, `v' = u·sa + v·ca`) — verified by reading
`src/engine/occt/occt_construct.cpp::twisted_sweep`. So a densified native twist and a
densified OCCT twist (same stations) build the SAME ruled sections and converge to the
same volume. For SIM parity BOTH engines are fed the identical pre-densified path, so
neither out-refines the other (the same trick the landed smooth-arc sweep parity uses).

### D3 — Curved-rail section transport: RMF, morph A→B linearly

The rail loft uses the rotation-minimizing frame (`detail::rmfFrames`, already landed)
so the section does not accumulate spurious twist around a curved rail — the
twist-free transport `MakePipeShell` approximates. Section A morphs into B by linear
per-vertex interpolation in the frame (equal counts, the existing straight-rail
contract). The straight-rail path is untouched (still the perpendicular-frame ruled
loft, EXACT vs the oracle).

### D4 — Honest declines (unchanged discipline)

- twisted_sweep: `sectionSweepUnsafe` (rim folds past the neighbor) → NULL → OCCT. A
  degenerate profile / <2 path points → NULL.
- loft_along_rail: mismatched counts, degenerate rail/section → NULL. A tight-curvature
  rail that will not weld at the density cap → the tube fails `robustlyWatertight` in
  the engine → OCCT.

Neither builder ever widens a tolerance or hands a void to OCCT.

## Gates

- **Gate 1 (host analytic, OCCT-free)** — in `test_native_sweep.cpp`:
  - twisted prism volume converges to `area·L` within a deflection-bounded tolerance
    across the density (twist preserves cross-section area on a straight spine);
    watertight at the deflection ladder; correct topology (2 caps + N·edges bands).
  - arc-rail loft volume converges to the Pappus torus-sector `polyArea·R·φ` within a
    deflection-bounded tolerance; watertight.
  - a self-folding twist and a tight rail assert NULL.
- **Gate 2 (sim parity vs OCCT)** — `native_construct_tails_parity.mm`: same densified
  input to `cc_set_engine(1)` (native) and `cc_set_engine(0)` (OCCT), compare volume /
  area / watertight / Euler χ=2 / bbox within a documented tolerance; assert the honest
  declines fall through identically (`cc_active_engine()==1`, rel 0).

## Risks

- The densified twisted tube volume converges but is a chord under-fill at any finite
  density (like OCCT's own ruled ThruSections). Parity is against OCCT fed the SAME
  stations, so both share the chord error — the *relative* delta is small. The host
  gate tolerance is deflection/density-bounded, asserting convergence, not fp-exactness.
- A near-self-folding twist just under the `sectionSweepUnsafe` bound: covered by the
  guard + the engine self-verify; if it slips through non-watertight the engine
  discards it → OCCT.
