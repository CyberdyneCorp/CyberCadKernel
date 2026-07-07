# Design — moat-m7a-construction-breadth

## Context

The base `native-construction` spec already lands the OCCT-free N-section ruled-loft
builder `build_loft_sections` (`src/native/construct/loft.h`) with a green host suite
(`tests/native/test_native_loft.cpp`, closed-form volume, no OCCT). It was not reachable
through the `cc_*` facade — only `cc_solid_loft` and `cc_solid_loft_wires` were wired. This
change ADDS the facade entry `cc_solid_loft_sections` plus its OCCT oracle and engine
dispatch, turning an already-proven native capability into a callable ABI. No native
geometry source changes; the increment is wiring + oracle + self-verify + parity gate.

## Why this is the most tractable residual (the diagnosis)

The reachable Tier-4 residuals and why they rank as they do:

| Residual | New work needed | Native geometry proven? | Closed-form gate? | Verdict |
|---|---|---|---|---|
| **N-section (≥3) planar loft — facade wiring** | expose `build_loft_sections` + OCCT `ThruSections` oracle | **Yes — already host-green** | **Yes — prismatoid / `A·H`** | **CHOSEN (landed first slice)** |
| Orientation guided sweep, straight spine | NEW OCCT `MakePipeShell(guideWire)` oracle + guide-aimed native frame | not yet built | yes (`area×length`) | DEFERRED — honest decline this iteration |
| Non-planar-cap loft | filling surface to close a non-planar end section | partial | none | harder — deferred |
| Curved-rail loft-along-rail | genuine pipe-shell morph | needs SSI | none | needs SSI — deferred |
| Fine-pitch self-intersecting thread | helicoid–helicoid trim | needs SSI (M2) | ridge volume | OUT OF SCOPE (needs M2) |

The chosen slice is the only residual whose native geometry is ALREADY proven correct
(host suite green) — so both gates are strong immediately and the risk is confined to the
facade/dispatch/oracle wiring, not to new geometry.

## The N-section loft construction (native, pre-existing)

`build_loft_sections(sectionsXYZ, counts, sectionCount)` reads `sectionCount` closed
polygons packed back to back (`counts[k]` vertices each) and builds `(sectionCount − 1)`
RULED bands between consecutive sections, capping the first and last section and sharing
each internal section as one vertex ring. Sections of differing vertex count are made
compatible by an arc-length-preserving resample (the same T1 correspondence the 2-wire
loft uses). The result is one welded `topology::Shape` `Solid`. The builder is OCCT-free
and host-buildable; this change does not touch it.

## Engine dispatch + self-verify (the new wiring)

`NativeEngine::solid_loft_sections(s, c, sc)`:

```
Shape solid = ncst::build_loft_sections(s, c, sc);
if (solid.isNull() || !robustlyWatertight(solid) || !(watertightVolume(solid) > 0.0))
    return fallback().solid_loft_sections(s, c, sc);   // → OCCT ThruSections
return track(wrapNative(std::move(solid)));
```

The self-verify is a genuine gate: a null candidate, a non-watertight mesh, or a
non-positive volume is DISCARDED and the SAME arguments are forwarded to the OCCT oracle.
`OcctEngine::solid_loft_sections` builds one `BRepBuilderAPI_MakePolygon` wire per section
and adds all wires to a single `BRepOffsetAPI_ThruSections(solid, ruled)` — the same
construction the 2-section loft uses, through more wires. `IEngine` gets a default
`solid_loft_sections` returning `engine_unsupported`, and the facade `cc_solid_loft_sections`
shim mirrors `cc_solid_loft_wires`.

## Slice boundary (native builds it; else NULL / discard → OCCT)

NATIVE (self-verified watertight, matches the oracle): planar ≥3-section chains, equal or
mismatched vertex counts — monotone tapers and symmetric-ended spools (both bands
tessellate to a matching shared-ring seam).

DISCARD → OCCT (honest decline, gap reported by the parity gate):

- a NON-PLANAR or point-collapsed section (native returns NULL),
- a mismatched-count pair whose resampled caps cannot close watertight,
- an ASYMMETRIC expand-then-contract spool (e.g. 4×4→6×6→2×2) whose two adjacent bands
  taper at different ratios: the native ruled solid's VOLUME is exact, but the native
  face-mesher splits the two faces meeting at the shared middle ring with mismatched
  interior sampling → a T-junction → a non-watertight mesh at every deflection. Rather
  than weaken the tessellator (out of scope), the self-verify DISCARDS the candidate →
  OCCT. This limitation is measured and documented in `native_loft_parity.mm`, not hidden.

## Host analytic gate (Gate 1, no OCCT linked)

`tests/native/test_native_loft.cpp` calls `build_loft_sections` directly (no `cc_*`, no
OCCT) and asserts a watertight `Solid` with the closed-form enclosed volume — the
prismatoid `h/3·(A1 + A2 + √(A1·A2))` per frustum band for a spool, `A·H` for a straight
stack — via `tess::enclosedVolume`. Covers stacked-box, square/octagon spools, mismatched
4→8 frustums, and the non-planar / degenerate / single-section declines. 21/21, 0 failed,
built with `CYBERCAD_USE_OCCT=OFF`.

## Sim parity gate (Gate 2, OCCT oracle)

`tests/sim/native_loft_parity.mm` on a booted iOS simulator, through the SAME `cc_*`
facade: `cc_solid_loft_sections` with the native engine active (`cc_set_engine(1)`) vs
`OcctEngine::solid_loft_sections` (`BRepOffsetAPI_ThruSections`, ruled). Native fixtures —
square spool 10→4→10 (vol 624), triangle stack (30), stacked box (144), octagon spool (18
faces) — must build native and match volume/area/watertight/topology; the deferred
fixtures (asymmetric spool, non-planar middle, mismatched-4to3) must delegate to OCCT with
`native active=1` and the delegated volume equal to the oracle to fp precision.

## Alternatives considered

- **Ship the orientation guided sweep as the first slice**: rejected for this iteration —
  it needs a NEW OCCT `MakePipeShell(guideWire)` oracle and a guide-aimed native frame that
  were not landed; recorded as the next M7 slice (honest decline), not foreclosed.
- **Repurpose `cc_solid_loft_wires`** to take N sections: rejected — breaks the shipped
  2-wire ABI. The additive `cc_solid_loft_sections` entry is the ABI-clean path.
- **Weaken the tessellator to make the asymmetric spool watertight**: forbidden (out of
  scope, no proven-zero-regression tessellator change); the case stays a documented
  self-verify → OCCT decline.

## Risks

- The OCCT `ThruSections` wire ordering / seam vertex may differ from the native ring
  ordering by a rotation. Mitigation: the parity gate measures mass/topology directly; a
  fixture that fails parity is a decline (discard → OCCT) with the gap reported.
- A future non-symmetric spool that self-verifies watertight but is geometrically wrong:
  guarded by the volume + watertight self-verify and the parity gate; no faked solid is
  ever emitted.
