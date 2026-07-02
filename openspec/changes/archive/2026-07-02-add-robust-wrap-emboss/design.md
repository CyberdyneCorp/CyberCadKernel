## Context

`cc_wrap_emboss` wraps a 2D profile around a cylindrical face and pads it radially
to add (boss) or remove (deboss) material. The current implementation
(`occt_construct.cpp`, mirrored in the app's `KernelBridge.mm`) builds the pad
with `BRepOffsetAPI_ThruSections` lofting an inner wrapped wire (radius `rIn`) to
an outer wrapped wire (radius `rOut`). The wrapping math is sound — arc-length
`px` maps to angle `px/R`, axial `py` runs along the cylinder axis, the profile is
centred on the face's V-mid, and edges are densified so the wire follows the
curvature — but ThruSections is the weak link: on a dense high-curvature profile
the ruled loft between two many-vertex section wires tends to self-intersect or go
non-manifold, so `BRepCheck_Analyzer::IsValid` rejects it and the emboss fails.
The code then falls back to a coarse polygon, losing fidelity.

The robust approach is to stop asking ThruSections to produce a valid solid in one
shot and instead **build the pad's boundary surfaces explicitly and sew them**:
inner cap + outer cap + side walls, then `BRepBuilderAPI_Sewing` + `ShapeFix` into
a closed, healed solid. Sewing with a feature-tied tolerance and healing gaps is
much more tolerant of dense near-tangent geometry than a single ruled loft.

Constraints:
- **ABI stability**: `cc_wrap_emboss` signature and semantics unchanged; the
  robust path is internal.
- **Never regress**: keep the ThruSections build as a coarse fallback so the
  operation is at least as capable as today.
- **Real validity bar**: the pad and the final body must pass
  `BRepCheck_Analyzer::IsValid` and be watertight; the volume-change sign must be
  correct (boss adds, deboss removes).
- **Honesty**: if a case cannot be sewn into a valid solid, fall back to a valid
  coarse result and mark it deferred — do not fake a pass.

## Goals / Non-Goals

Goals:
- A robust internal pad builder (cap + cap + side walls, sewn + healed) that
  yields a valid closed solid for a dense high-curvature wrapped profile.
- Correct boss/deboss volume-change sign, unchanged wrapping math and signature.
- A valid coarse ThruSections fallback and honest deferral when the sewn build
  cannot produce a valid solid.
- On-simulator checks asserting IsValid + watertight + volume-sign, and a
  regression case where the old ThruSections path was invalid.

Non-Goals:
- Wrapping onto non-cylindrical faces (cone/sphere/BSpline) — cylinder only, as
  today.
- Changing the wrapping parameterization or the `cc_wrap_emboss` signature.
- A fully native (non-OCCT) emboss — this builds on OCCT sewing/ShapeFix
  (Phase 3 allows building on OCCT primitives/surfaces).

## Decisions

- **Explicit cap-and-side pad, not a single loft.** Build the closed pad boundary
  as: inner cap face (wrapped profile at `rIn`), outer cap face (at `rOut`), and
  one side wall per profile edge connecting the `rIn` edge to the `rOut` edge.
  Side walls are ruled/filled faces (`BRepFill_Filling` or a `GeomFill` ruled
  surface / `BRepBuilderAPI_MakeFace` over the four wrapped corner points). This
  gives OCCT a well-defined face set instead of asking a loft to infer it.
- **Sew + heal into a solid.** `BRepBuilderAPI_Sewing` at a tolerance tied to the
  feature size (a small fraction of `depth` / the profile extent) stitches the
  faces into a shell; `ShapeFix_Shell` (orient closed) + `ShapeFix_Solid` heal
  gaps and produce a solid. Gate on `BRepCheck_Analyzer::IsValid`.
- **Unchanged wrapping + boss/deboss.** Reuse the existing wrap lambda and
  densification; `boss=1` → `rIn=R, rOut=R+depth`, fuse; `boss=0` →
  `rIn=R-depth, rOut=R`, cut. The pad build changes, not the geometry mapping.
- **ThruSections retained as coarse fallback.** If the sewn pad is null/invalid,
  fall back to the existing dense-then-coarse ThruSections build (today's
  behaviour) so the op never regresses; if that also fails, return `0` with a
  `cc_last_error` reason.
- **Volume-sign gate is the correctness check, not a proxy.** The test computes
  `cc_mass_properties` before and after: boss → `V_after > V_base`; deboss →
  `V_after < V_base`, with the delta in the plausible range of the padded profile
  area × depth (not just "changed").
- **Deterministic face order.** Faces are created and sewn in a fixed order
  (inner cap, outer cap, then side walls in profile-edge order) so the sewn solid
  is reproducible.

## Risks / Trade-offs

- **Sewing tolerance sensitivity.** Too tight leaves gaps (open shell → invalid);
  too loose merges distinct features. Tied to feature size and validated by the
  IsValid + watertight checks; documented as the tuning knob.
- **Side-wall self-intersection at very high curvature.** If a wall still
  self-intersects, ShapeFix may not heal it; that case falls back to coarse
  ThruSections and is marked deferred (honest fallback), not reported as a full
  robust success.
- **Cap face from a wrapped (non-planar) boundary.** The wrapped profile at a
  fixed radius lies on the cylinder of that radius, so the "cap" is a cylindrical
  patch, not planar; built as a face on that cylinder trimmed to the wrapped
  wire. If the trim is degenerate, fall back.
- **Volume-delta bounds are approximate.** The expected delta is bounded by the
  profile area × depth with a tolerance; the check asserts sign + plausible
  magnitude, not an exact analytic volume (the wrapped pad is curved).

## Migration Plan

1. Factor the existing wrap lambda + densification out so both the ThruSections
   fallback and the new sewn builder share it.
2. Implement the sewn pad builder: inner cap, outer cap, per-edge side walls →
   `BRepBuilderAPI_Sewing` → `ShapeFix_Shell`/`ShapeFix_Solid` →
   `BRepCheck_Analyzer::IsValid`.
3. Route `wrap_emboss` through the sewn builder first, ThruSections coarse
   fallback second, `0` last (record reason).
4. iOS-sim checks: (a) dense high-curvature profile → IsValid + watertight +
   correct volume sign for boss and deboss; (b) a profile that made the old
   ThruSections path invalid now succeeds via sewing (or falls back to a valid
   coarse result and is marked deferred); (c) `cc_wrap_emboss` signature unchanged
   (`test_abi`).
5. Host stub unchanged (`0`); `run-sim-suite.sh` regression stays green.
6. `openspec validate --all --strict`; update `ROADMAP.md` Phase 3 status.

## Open Questions

- The sewing tolerance as a function of feature size (fraction of `depth` / min
  profile edge length) — pin empirically so the dense high-curvature fixture
  seals into a valid solid without merging real features.
- Whether the caps are best built as trimmed cylindrical faces or as
  `BRepFill_Filling` patches over the wrapped boundary — decide by which yields a
  valid sew on the test fixtures; document the choice.
- Whether any real-world dense profile still cannot be sewn (→ permanent coarse
  fallback + deferred note) — recorded honestly from the sim results.
