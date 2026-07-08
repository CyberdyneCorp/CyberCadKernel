# Design — moat-m4t4-deep-nested-trimmed

## Context

The native STEP importer (`src/native/exchange/step_reader.cpp`, OCCT-free) reads the
AP203 manifold-solid-brep subset and a rigid / uniform-scale / mirror assembly tree,
reconstructs analytic pcurves per face, and self-verifies watertight against the OCCT
oracle downstream. M4-tail-4 is the last import corner: DEEP-nested assemblies, GENERAL
trimmed surfaces, and shared-sub-assembly instancing. This design records the
diagnosis that selects ONE tractable gap and specifies its exact, self-verifiable
reduction.

## Diagnosis — which gap is real, which is tractable

### (1) Deep-nested 3+-level assembly — already handled by `composeChain`
`composeChain` (step_reader.cpp:711) is a leaf→root walk:

```
while (true) {
  if (!visited.insert(sr).second) return nullopt;   // cycle → decline
  auto it = edges.find(sr);
  if (it == edges.end()) break;                      // unique root → stop
  auto op = resolveOperator(it->second.opId);        // per-level conformal gate
  if (!op) return nullopt;                            // dangling/non-conformal → decline
  w = op->first.composedWith(w);                     // apply this level on the LEFT
  sr = it->second.parentSr;
}
```

The loop bound is the parent-edge chain length, not a constant. Level count is data,
so 3, 4, … N levels compose with **no code change**: `W = T₁ ∘ T₂ ∘ … ∘ Tₙ`. The
completeness gate (`unplaced > 1` → decline), the conformality gate per level, and the
cycle guard all already generalise. The only thing the landed suite lacks is a fixture
that EXERCISES depth ≥ 3 — so we add one as a regression lock, not as new behaviour.

**HOST ANALYTIC proof for (1):** build three rigid frames `A, B, C` (a translate, a
90° rotate, a translate), emit CDSRs leaf→sub2 (`T₃=A`), sub2→sub1 (`T₂=B`),
sub1→root (`T₁=C`), and assert the imported leaf's world box equals the box of the
leaf geometry mapped by the INDEPENDENTLY-computed matrix product `C·B·A` — computed in
the test without touching `composeChain`. **SIM proof:** the same buffer through
`STEPControl_Reader` + `BRepMesh` matches native on count / volume / bbox / centroid.

### (2) General trimmed surface (`RECTANGULAR_TRIMMED_SURFACE`) — the chosen gap
`surface(id)` (step_reader.cpp:1008) returns `nullopt` for any keyword outside its
dispatch set (step_reader.cpp:1029), so a face whose surface is a
`RECTANGULAR_TRIMMED_SURFACE` always declines to OCCT — even when its basis is a
`PLANE` or `CYLINDRICAL_SURFACE` the reader maps exactly. This is the genuine,
tractable trimmed-surface gap "beyond the EDGE_LOOP trims".

### (3) Shared-sub-assembly — genuine but out of this slice
`parentEdges()` (step_reader.cpp:687) declines when one child SR has two distinct
parents. Admitting a shared sub-assembly requires instancing one B-rep at multiple
world transforms, which breaks the "each B-rep placed exactly once" invariant of
`assembly()` (the `placed[brep]` twice-placed guard, step_reader.cpp:773). That is a
placement-model change, not a one-line reduction; it stays an honest, tested decline.

## Decision — the `RECTANGULAR_TRIMMED_SURFACE` reduction

`RECTANGULAR_TRIMMED_SURFACE('',#basis_surface,u1,u2,v1,v2,usense,vsense)` re-parametrises
a `basis_surface` to the rectangular parameter box `[u1,u2] × [v1,v2]`. In an
AP203/AP214 manifold-solid B-rep an `ADVANCED_FACE`'s surface arg may reference this
wrapper instead of the basis surface directly; the face's `EDGE_LOOP` still carries the
real 3D boundary.

**Reduction:** in `surface(id)`, when the record keyword is
`RECTANGULAR_TRIMMED_SURFACE`, recurse `surface(#basis_surface)` and RETURN the basis
`FaceSurface` unchanged — exactly the `TRIMMED_CURVE` → basis-curve and
`SURFACE_CURVE`/`SEAM_CURVE` → `curve_3d` unwraps already in the reader. The
`ADVANCED_FACE`'s `EDGE_LOOP` is the authoritative trim; `pcurveFor` reconstructs the
analytic pcurve on the basis surface exactly as it does for a directly-referenced
basis. The rectangular box is thus **redundant** with the loop and is used only to
GUARD the reduction, never to synthesise geometry.

### Why unwrapping (and dropping the rect box) is faithful, not lossy
- The face's boundary lives in the `EDGE_LOOP`'s 3D `EDGE_CURVE`s, which the reader
  already turns into a trimmed wire with analytic pcurves. Whether the surface entity
  was the basis directly or a rect-trim wrapping it, the SAME basis surface + the SAME
  loop produce the SAME native face — and the SAME face OCCT builds (OCCT likewise
  bounds the face by its wire, not by the rect box). So native and OCCT agree.
- A `RECTANGULAR_TRIMMED_SURFACE` never changes the geometry of the basis, only its
  parameter domain; on the natively-supported bases (plane, cylinder, cone, sphere,
  B-spline) the reader's pcurve reconstruction is domain-agnostic (it projects 3D edge
  samples to `(u,v)`), so discarding the wrapper's domain is exact.

### Guards (all DECLINE → OCCT; never a fabricated face)
1. **Unsupported basis.** If `surface(#basis_surface)` returns `nullopt` (basis outside
   the supported native kinds), the rect-trim returns `nullopt` too — inherits the
   existing decline unchanged.
2. **Degenerate / inverted rect box.** If any of `u1,u2,v1,v2` is non-finite, or
   `u2 ≤ u1` or `v2 ≤ v1` (an empty or reversed domain), DECLINE — a malformed wrapper
   is not silently accepted.
3. **Bare (loop-less) rect-trim.** The reduction relies on the `ADVANCED_FACE`'s
   `EDGE_LOOP` for the boundary. A face that would need the rect box SYNTHESISED into a
   4-edge boundary wire (no real loop) is OUT OF SCOPE this slice — the existing
   childless-bound path only admits full periodic sphere/torus, and a rectangular
   analytic patch is neither, so it declines through the unchanged `advancedFace`
   childless-bound guard. We do NOT add boundary synthesis.
4. **Torus basis with real trim.** A rect-trim over a `TOROIDAL_SURFACE` inherits the
   existing "partial torus has no native trimmed-torus mesh path → decline"
   (step_reader.cpp:1908) unchanged. Unwrapping does not bypass that guard.

### Self-verification (two gates)
- **HOST ANALYTIC:** the trimmed patch's geometry equals its basis over the loop. The
  host test builds a solid whose one face's surface is a `RECTANGULAR_TRIMMED_SURFACE`
  over a `PLANE`, and asserts the imported solid is byte-for-byte the SAME native
  `Shape` (same volume, area, topology) as the identical file with the basis `PLANE`
  referenced directly — an independent equivalence, no OCCT. For the cylinder basis the
  corner points `S(uᵢ,vⱼ)` are checked closed-form against the analytic cylinder.
- **SIM native-vs-OCCT:** the same buffer through `STEPControl_Reader` + `BRepMesh`
  matches native on triangle-envelope count, enclosed volume, bbox, centroid, and
  watertight topology.

## Alternatives considered
- **Synthesise the boundary from the rect box (support bare rect-trims).** Rejected
  this slice: it adds a UV-rectangle → 4-edge-wire builder and its own faithfulness
  proof, widening scope well beyond one unwrap. The loop-backed case is the one OCCT
  solid B-reps actually emit; the bare case is deferred as an honest decline.
- **Support `CURVE_BOUNDED_SURFACE` too.** Rejected: an arbitrary boundary-curve trim
  in parameter space is a materially harder reconstruction (general `BOUNDARY_CURVE` /
  `COMPOSITE_CURVE_ON_SURFACE`), not a single unwrap. `RECTANGULAR_TRIMMED_SURFACE` is
  the tractable first slice; `CURVE_BOUNDED_SURFACE` stays a decline.
- **Tackle shared-sub-assembly instead.** Rejected as intractable this slice (needs the
  per-instance world-transform placement model, see Diagnosis (3)).

## Risks
- **A rect box inconsistent with the loop.** If a malformed foreign file gave a rect
  box that disagreed with the `EDGE_LOOP`, native follows the loop (as OCCT does), so
  the result still matches the OCCT oracle; the downstream watertight self-verify is the
  final arbiter and discards any non-watertight result → OCCT. No wrong solid escapes.
- **Byte-identity of landed paths.** The new arm only fires on the
  `RECTANGULAR_TRIMMED_SURFACE` keyword. No landed fixture (native-written or the
  foreign OCCT-authored ones) contains that keyword, so every landed import is
  unchanged; the regression suite asserts this.
