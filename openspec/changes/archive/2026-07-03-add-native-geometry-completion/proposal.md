# Proposal — add-native-geometry-completion

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capability #4 (`native-construction`) landed native
polygon extrude + line-segment revolve; `#4b` Tier A added holed / typed-profile
extrudes and typed-profile revolve; Tier B added the 2-section ruled loft; Tier C added
the constant-frame sweep (`cc_solid_sweep` + no-op `cc_twisted_sweep`); Tier D added
threads + the tapered shank. The living `native-construction` spec's deferred-ops
requirement enumerates exactly what still falls through to OCCT — and this change is the
honest attempt to CLOSE that list where the geometry is achievable natively:

- **(A)** a **kind-3 SPLINE** outer profile edge (extrude + revolve) and an **OFF-AXIS-ARC**
  revolve (a **TORUS** surface of revolution — no native `Torus` surface existed yet);
- **(B)** a **3+-section** loft chain (`cc_solid_loft` / `_wires` beyond two sections);
- **(C)** a general **non-planar spine + accumulating twist/scale** sweep
  (`cc_solid_sweep` / `cc_twisted_sweep`) and best-effort **guided/rail**
  (`cc_guided_sweep` / `cc_loft_along_rail`);
- **(D)** a **self-intersection resolver** so more **fine-pitch** thread parameters weld
  watertight.

These back real product geometry (lofted transitions, swept fillets/handles, torus
fittings, spline-profile parts, more thread pitches), so they are the next honest
increment. Doing them as one confident "native geometry complete" claim would risk faking
the genuinely hard parts, so this proposal is scoped, guarded, and honest: each area is
**ATTEMPTED** natively and **kept as labelled OCCT fall-through** wherever it cannot be made
a watertight, oracle-correct solid. Cases that genuinely need **surface–surface
intersection (SSI)** — self-intersecting sweeps, tight-curvature folds, hard pipe-shell
rails, truly self-intersecting threads — are OUT OF SCOPE (Tier 4) and MUST fall through.

## What changes

1. **(A) Spline profile edge + off-axis-arc torus revolve** (`src/native/construct/profile.h`,
   `src/native/construct/construct.h`, `src/native/math/elementary.h`,
   `src/native/topology/shape.h`, OCCT-free).
   - **kind-3 spline extrude** — resolve a kind-3 `CCProfileSeg`'s control points
     (`splineXYCount` DOUBLES = 2× the point count) into a native `BSpline` edge curve
     (`src/native/math/bspline.h`) and EXTRUDE it by the straight translation into a swept
     B-spline side wall (a `FaceSurface::Kind::BSpline` ruling of the edge along the extrude
     direction), capped with the two planar end faces (a planar spline profile). NULL (→
     OCCT) for a non-planar spline loop or a spline that self-intersects.
   - **native `Torus` surface** — add a `Torus` (`center`, axis `Ax3`, major radius `R`,
     minor radius `r`) to `src/native/math/elementary.h` with point/normal evaluation, and
     a `FaceSurface::Kind::Torus` to `src/native/topology/shape.h`.
   - **off-axis-arc torus revolve** — in `cc_solid_revolve_profile`, a kind-1 arc segment
     whose supporting-circle centre is OFF the axis sweeps a **torus band** (`R` = the
     centre's signed distance from the axis, `r` = the arc's circle radius, the band
     spanning the arc's angular sweep × the revolve angle). Rim edges are `Circle` arcs
     shared with the neighbours; a full 2π closes, a partial angle adds meridian caps. NULL
     (→ OCCT) for a spline-revolve (general B-spline surface of revolution) or a torus that
     self-intersects the shell.

2. **(B) N-section ruled loft chain** (`src/native/construct/loft.h`, OCCT-free). Extend the
   Tier-B two-section builder to a **chain of N ≥ 2 sections** with EQUAL vertex counts:
   for each consecutive pair `(k, k+1)` build the ruled bands with `detail::ruledSideFace`,
   SHARING each interior section's vertex ring between the band below and the band above,
   and cap ONLY the first and last sections with `detail::planarFace` → one watertight
   solid (a C0/ruled skin, C1 where consecutive section directions agree). NULL (→ OCCT)
   for mismatched vertex counts (ambiguous resample — OCCT re-parametrizes) or a non-planar
   end cap.

3. **(C) General sweep — RMF + twist/scale, and guided/rail** (`src/native/construct/sweep.h`,
   OCCT-free).
   - **Rotation-minimizing frame (RMF)** — replace the planar-only constant frame with a
     `detail::rmfFrames(spine, tangents)` computed by the **double-reflection method** (Wang
     et al. 2008): the frame's up-vector is parallel-transported with minimal rotation about
     the tangent, so a NON-PLANAR spine sweeps a coherent, non-twisting section. (The planar
     spine still matches OCCT's constant law as a special case, preserving Tier-C parity.)
   - **Accumulating twist + linear scale** — `cc_twisted_sweep` rotates the section about
     the frame tangent by a total `twistRadians` accumulated linearly 0→end and scales it
     linearly 1→`scaleEnd` — applied ON TOP of the RMF, so a real twist/scale is now native.
   - **Guided sweep / rail loft (best-effort)** — `cc_guided_sweep` orients the section
     using a guide curve (the frame's up-vector tracks the guide direction rather than the
     RMF default); `cc_loft_along_rail` interpolates between two end profiles along a rail.
     Native where the resulting solid is watertight + oracle-correct.
   - **Guards** — a spine/section that self-intersects or folds (curvature radius below the
     profile's radial extent — `spineTooSharp`, extended for the twist/scale envelope), or
     a guide/rail case that cannot resolve without SSI, returns a **NULL `Shape`** (→ OCCT).
     No SSI attempted here.

4. **(D) Thread self-intersection resolver** (`src/native/construct/thread.h`, OCCT-free).
   Add a resolver for the NEAR-self-intersecting band: where the radial V's flanks approach
   but do not truly cross (the current Tier-D guard was conservative), TRIM/clamp the V
   section at the pitch-room boundary so the swept bands still weld watertight, widening the
   set of thread parameters that pass `robustlyWatertight`. A **genuinely
   self-intersecting** thread (flanks truly cross — a non-manifold swept surface) still
   fails the watertight self-verify and returns NULL (→ OCCT). No SSI.

5. **`NativeEngine` glue** (`src/engine/native/native_engine.{h,cpp}`). The affected ops keep
   their native-else-fallback wiring; each runs the MANDATORY runtime self-verify
   (`robustlyWatertight` + correct volume/geometry sign) and, on a NULL native result OR a
   failed self-verify, falls through to the fallback with no interception. OCCT stays behind
   `CYBERCAD_HAS_OCCT`; the native builder never sees OCCT. An op/sub-case that cannot pass
   both gates is left as pure labelled fall-through.

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- **Surface–surface intersection (SSI) — Tier 4, explicitly not attempted.** A
  self-intersecting sweep (the swept surface folds through itself), a tight-curvature spine
  whose section folds, a hard pipe-shell rail that cannot resolve without trimming, and a
  genuinely self-intersecting thread (turns truly fold through each other) all fall through
  to OCCT. This change does not ship a solid that would need SSI.
- **Mismatched-count / non-planar loft sections** — ambiguous resample; OCCT
  re-parametrizes (`ThruSections`). NULL → OCCT.
- **Spline-revolve** — a general B-spline surface of revolution (a kind-3 segment in
  `cc_solid_revolve_profile`) — NULL → OCCT (only the off-axis-arc torus + on-axis-arc
  sphere + line Plane/Cylinder/Cone revolves are native).
- **A non-planar spline loop / self-intersecting spline profile** in the extrude — NULL →
  OCCT.
- **Any op/sub-case that fails a gate** — kept labelled OCCT fall-through; the change
  reports it honestly per area.
- All remaining `#4b` ops (`wrap_emboss`) and every feature / boolean / query / transform /
  exchange op — remain fall-through, unchanged.

## Impact

- `src/native/math/elementary.h` gains a `Torus` surface (point + normal), and
  `src/native/topology/shape.h` gains `FaceSurface::Kind::Torus`. `src/native/construct/`:
  `profile.h` gains the kind-3 spline edge extrude + the off-axis-arc torus revolve band;
  `construct.h`'s revolve classifier gains the torus case; `loft.h` gains the N-section
  chain; `sweep.h` gains the RMF transport + accumulating twist/scale + guided/rail;
  `thread.h` gains the self-intersection resolver. All stay OCCT-free, host-buildable, added
  to `native_construct.h`. New host CTest cases across `tests/test_native_profile.cpp`,
  `tests/test_native_loft.cpp`, `tests/test_native_sweep.cpp`, `tests/test_native_thread.cpp`
  (+ facade cases in `tests/test_native_engine.cpp`).
- `src/engine/native/native_engine.cpp` — the affected ops keep native-else-fallback with a
  strengthened self-verify; a spline/off-axis-arc/N-section/RMF-twist/thread result that
  self-verifies is served natively, else falls through. `native_engine.h` unchanged
  (signatures already present).
- **No** `include/cybercadkernel/cc_kernel.h` signature change; **no**
  `src/facade/cc_kernel.cpp` change (the `cc_*` entry points already route through the
  active engine). The op doc-comments are the contract this change implements natively
  where achievable.
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the new native paths. All existing suites stay green at the OCCT
  default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** exact-value / analytic unit
tests on the built native B-rep + native tessellation — watertight (`boundaryEdgeCount ==
0`) with the analytic volume/geometry for each area (spline-edge extrude; torus band
`2π²·R·r²·(angle/2π)`; summed-frustum N-section loft; RMF-sweep twist/scale volume + a
rotation-minimizing-frame invariant; the resolver welding a previously-guarded near-self-
intersecting thread), and NULL for every SSI / self-intersecting / mismatched / degenerate
input — all with no OCCT; (b) **sim parity** through the facade (`cc_set_engine(1)` vs
default) comparing native vs OCCT `ThruSections` / `MakePipe`/`MakePipeShell` / `MakeRevol`
and asserting the deferred/SSI cases identical under both engines. Each area/op lands
native only when both gates pass; otherwise it remains labelled OCCT fall-through and the
change reports the real per-area native-vs-fallback split.
