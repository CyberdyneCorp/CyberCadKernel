# Proposal — moat-m4t4-deep-nested-trimmed

## Why

MOAT M4-tail-4 targets the STEP-import corners **beyond** the landed 2-level nested
assembly + `EDGE_LOOP`-trimmed faces: DEEP-nested 3+-level assemblies, GENERAL
trimmed surfaces, and shared-sub-assembly (multi-parent) instancing. A diagnosis of
the landed reader (`src/native/exchange/step_reader.cpp`) sorts these three candidate
gaps by what is ACTUALLY missing and what is tractable under the two-gate discipline
(HOST ANALYTIC + SIM native-vs-OCCT):

1. **DEEP-nested 3+-level assembly — ALREADY HANDLED, not a code gap.**
   `composeChain(startSr, edges)` (step_reader.cpp:711) walks parent edges from the
   leaf shape-representation to its unique root in an **unbounded `while` loop**,
   composing `W = T_root ∘ … ∘ T_start` with each level applied on the left. A
   length-1 chain reproduces the single-level placement byte-identically; a length-2
   chain is the landed 2-level case; a length-N chain composes N levels with no code
   change. The only thing missing is a **regression fixture proving 3-level** — the
   landed test suite (`tests/native/test_native_step_reader.cpp`) stops at 2 levels
   (`nested_two_level_assembly_composes_chain`). This is a proof/guard task, not new
   behaviour, so it does not by itself justify a capability change.

2. **GENERAL trimmed surface (`RECTANGULAR_TRIMMED_SURFACE`) — GENUINE gap, TRACTABLE.**
   `surface(id)` (step_reader.cpp:1008) dispatches only on the direct surface keywords
   (`PLANE` / quadrics / `TOROIDAL_SURFACE` / `B_SPLINE_SURFACE_WITH_KNOTS` /
   `SURFACE_OF_REVOLUTION`) and the combined `RATIONAL_B_SPLINE_SURFACE`; **every other
   surface keyword returns `nullopt` → `decline()` → OCCT** (step_reader.cpp:1029). A
   `RECTANGULAR_TRIMMED_SURFACE` — a basis surface re-parametrised to a rectangular
   `(u,v)` box, the way many foreign (CATIA / NX / SolidWorks AP203) writers present an
   analytic face — therefore always falls through to OCCT even when its basis is a
   surface the reader already maps exactly. This is the "general TRIMMED surfaces,
   beyond the landed `EDGE_LOOP` trims" corner, and it reduces to an **exact unwrap**:
   the same move the reader already makes for `TRIMMED_CURVE` (unwrap to the basis
   curve, keep the loop's real trim) and for `SURFACE_CURVE` / `SEAM_CURVE`.

3. **Shared-sub-assembly (multi-parent) instancing — GENUINE gap, INTRACTABLE this slice.**
   `parentEdges()` (step_reader.cpp:687) returns `nullopt` — an honest whole-file
   decline — the instant one child shape-representation carries two DISTINCT parent
   edges (`nested_ambiguous_two_parent_declines`). Admitting it needs a different
   placement model: one B-rep **instanced N times** at N per-instance world transforms,
   replacing today's "each B-rep placed exactly once" invariant. That is a larger
   restructuring than one additive reduction and stays a first-class HONEST DECLINE.

The tractable, genuinely-missing gap is **(2)**. This change adds the
`RECTANGULAR_TRIMMED_SURFACE` reduction additively, self-verified against an
independent closed-form (HOST ANALYTIC) and against the OCCT `STEPControl_Reader`
oracle (SIM), and RETAINS an honest decline for any trim it cannot faithfully
reproduce. It also lands the 3-level assembly regression fixture from (1) as a guard
that locks the "`composeChain` already composes N levels" finding, and keeps (3) an
explicit, tested decline.

## What Changes

1. **`RECTANGULAR_TRIMMED_SURFACE` unwrap in `surface(id)`** (OCCT-free, additive).
   `RECTANGULAR_TRIMMED_SURFACE('',#basis,u1,u2,v1,v2,usense,vsense)` SHALL be
   unwrapped to its `#basis` surface by recursing `surface(#basis)` — mirroring the
   existing `TRIMMED_CURVE` / `SURFACE_CURVE` unwrap. The `ADVANCED_FACE`'s existing
   `EDGE_LOOP` remains the authoritative trim (pcurves reconstructed analytically per
   basis kind, exactly as today); the rectangular `(u,v)` box is redundant with, and
   must be consistent with, that loop. The reduction is admitted ONLY when the basis
   resolves to a supported native `FaceSurface::kind` AND the face carries a real
   bound; a basis outside the supported set, a bare (loop-less) rect-trim whose
   boundary would have to be SYNTHESISED, or any non-finite / inverted `(u1,u2,v1,v2)`
   DECLINES → OCCT. No tolerance is weakened; no boundary is fabricated.
2. **A 3-level nested assembly regression fixture** in
   `tests/native/test_native_step_reader.cpp` proving `composeChain` composes
   `W = T₁ ∘ T₂ ∘ T₃` — HOST ANALYTIC against an independent matrix product — and a
   sim parity assertion (`tests/sim/native_step_import_parity.mm`) that the 3-level
   leaf lands where `STEPControl_Reader` + `BRepMesh` place it. This is a guard on
   already-shipped behaviour, added so the N-level claim is locked, not asserted.
3. **Tests for the new surface reduction**: a host round-trip that a solid whose face
   surface is a `RECTANGULAR_TRIMMED_SURFACE` over a `PLANE` (and over a
   `CYLINDRICAL_SURFACE`) imports watertight to the same solid as its un-wrapped
   basis, plus a decline test for an unsupported basis and for an inverted/degenerate
   rect box, plus the sim native-vs-OCCT parity on volume / bbox / centroid /
   topology.

No `cc_*` ABI change (the reader is behind the existing `step_import_native`
entry point); `src/native/**` stays OCCT-free; the landed single/2-level assembly and
`EDGE_LOOP`-trim paths stay BYTE-IDENTICAL (the new arm only fires on the
`RECTANGULAR_TRIMMED_SURFACE` keyword, which no landed fixture emits).

## Capabilities

### Modified Capabilities
- `native-exchange`: adds a `RECTANGULAR_TRIMMED_SURFACE` → native-basis-surface
  reduction to the native STEP importer (unwrap to the basis, keep the loop's real
  trim, decline any unfaithful case), and locks the already-shipped deep-nested
  N-level chain composition with a 3-level regression scenario. It does not alter the
  landed direct-surface, `EDGE_LOOP`-trim, `TRIMMED_CURVE`, or assembly-placement
  behaviour, and it does not change the shared-sub-assembly decline.

## Impact

- `src/native/exchange/step_reader.cpp` — one additive arm in `surface(id)` plus a
  small `rectangularTrimmedSurface(const Record&)` unwrap helper (recurses to the
  basis, validates the rect box, honours the supported-basis + real-bound guard).
  OCCT-free, host-buildable, per-function cognitive complexity kept green.
- `tests/native/test_native_step_reader.cpp` — new host cases: 3-level chain compose
  (HOST ANALYTIC), rect-trimmed-plane and rect-trimmed-cylinder round-trip, and the
  two decline cases (unsupported basis, inverted rect box).
- `tests/sim/native_step_import_parity.mm` — new sim cases: 3-level assembly and the
  rect-trimmed solid, each native-vs-OCCT on count / volume / bbox / centroid /
  topology.
- No ABI, no facade, no engine, no tessellator, no math change. The shared-sub-assembly
  path stays an honest decline (unchanged, still tested).
