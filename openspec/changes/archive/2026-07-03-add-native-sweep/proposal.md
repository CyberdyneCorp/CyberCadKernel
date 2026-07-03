# Proposal — add-native-sweep

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capability #4 (`native-construction`) landed native
polygon extrude + line-segment revolve; `#4b` Tier A added holed / typed-profile
extrudes and typed-profile revolve; `#4b` Tier B added the 2-section ruled loft. The
living `native-construction` spec is explicit that **sweep, twisted/guided sweep, and
rail loft** all still fall through to OCCT (`#4b` Tier C). Sweeps back real product
features (pipes, ducts, extrusions along a path, twisted columns), so `cc_solid_sweep`
is the next honest increment of native coverage.

Doing all of Tier C in one shot (sweep + guided + rail + threads) would risk faking
coverage on the genuinely hard pipe-shell/guide correspondence. This proposal scopes
**`cc_solid_sweep`** for the two tractable path shapes — a **straight** path (exact)
and a **smooth curved** path via a rotation-minimizing frame (deflection-bounded) —
and is explicit that tight-curvature / self-intersecting sweeps, `cc_guided_sweep`,
and `cc_loft_along_rail` remain honest, labelled OCCT fall-through. `cc_twisted_sweep`
is attempted only as a strict extension of the working RMF sweep.

## What changes

1. **Native swept surface math** (`src/native/math`, OCCT-free). Add a swept-surface
   helper: given the profile's 3D poles under the transported frame at a parameter `v`
   along the spine, produce the `Bezier`/`BSpline` `FaceSurface` poles for one swept
   side patch (one per profile edge). The analytic `FaceSurface` kinds
   (`Plane/Cylinder/Cone/Sphere/BSpline/Bezier`) already suffice — a straight sweep of
   a straight profile edge is a `Plane` (exact), and a curved sweep tiles into
   `Bezier`/`BSpline` patches — so this is poles/knots construction, not a new surface
   kind, unless a dedicated `SweptSurface` reads cleaner.
2. **Rotation-minimizing frame (RMF) transport** (`src/native/construct/sweep.h`,
   OCCT-free). Discretize the spine (deflection-adaptive sampling), build an initial
   orthonormal frame at the path start (profile local XY ⟂ path tangent), and
   **parallel-transport** it along the spine with the **double-reflection** RMF
   (Wang et al. 2008) so the profile carries with **minimal twist**. Emit the
   transported profile section at each sample.
3. **Native sweep assembler** (`src/native/construct/sweep.h`). 
   - **Straight path** → delegate to a directional extrude of the profile along the
     path vector (reuse the #4 prism assembler generalised to an arbitrary direction):
     N `Plane` side faces + 2 `Plane` caps; volume **exact**.
   - **Smooth curved path** → one swept surface patch per profile edge across the
     transported sections + two end-cap `Plane` faces (the profile at each end),
     assembled into a watertight `Solid`.
   - **Tight-curvature / self-intersection guard** → if any spine sample's curvature
     radius is below the profile's max radial extent (the swept wall would fold), or a
     transported section self-intersects, return a **NULL `Shape`** (fall through).
4. **Native twisted sweep** (`src/native/construct/sweep.h`). Reuse the RMF transport,
   then post-compose an accumulating rotation `twistRadians · s` and a scale
   `lerp(1, scaleEnd, s)` (with `s ∈ [0,1]` the normalised arc length) onto each
   transported section before tiling. Same guard. If the base RMF sweep is not
   applicable, return NULL (fall through) — no separate path.
5. **`NativeEngine` glue** (`src/engine/native/native_engine.{h,cpp}`). `solid_sweep`
   and `twisted_sweep` — currently pure `fallback()` delegations — call the native
   builder and, on a NULL native result (a declined case or a guard trip), fall
   through to the fallback with no interception. `guided_sweep` and `loft_along_rail`
   stay **pure fall-through** (labelled). OCCT stays behind `CYBERCAD_HAS_OCCT`; the
   native builder never sees OCCT.

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- **Tight-curvature / self-intersecting sweeps** — the swept tube folds on itself;
  producing a valid trimmed B-rep is the pipe-shell robustness problem. The builder
  detects it and returns NULL → OCCT fall-through.
- **`cc_guided_sweep`** — the profile is re-oriented/re-scaled by an auxiliary guide
  curve (`BRepOffsetAPI_MakePipeShell` with a guide). Labelled, verified fall-through.
- **`cc_loft_along_rail`** — two profiles skinned along a rail (`BRepFill` along a
  rail). Labelled, verified fall-through.
- **`cc_twisted_sweep` on a straight-only or guarded path** — falls through with the
  base sweep (no separate faked twist path).
- All other `#4b` ops (threads, `wrap_emboss`) and every feature / boolean / query /
  transform / exchange op — remain fall-through, unchanged.

## Impact

- `src/native/math` gains a swept-surface poles helper; `src/native/construct/` gains
  `sweep.h` (RMF transport + sweep/twisted-sweep assemblers) and is added to
  `native_construct.h` (still OCCT-free, host-buildable). New host CTest cases in
  `tests/test_native_sweep.cpp` (+ facade cases in `tests/test_native_engine.cpp`).
- `src/engine/native/native_engine.cpp` — `solid_sweep` and `twisted_sweep` change
  from pure fall-through to "native-else-fallback"; `guided_sweep` / `loft_along_rail`
  stay fall-through with a labelled comment. `native_engine.h` unchanged (signatures
  already present).
- **No** `include/cybercadkernel/cc_kernel.h` signature change; **no**
  `src/facade/cc_kernel.cpp` change (the four `cc_*` sweep entry points already route
  through the active engine). The four sweep doc-comments are the contract this change
  implements natively for `cc_solid_sweep` (+ `cc_twisted_sweep` when applicable).
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the new native path. All existing suites stay green at the
  OCCT default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** exact-value / analytic
unit tests on the built native B-rep + native tessellation — a straight sweep is
watertight with **exact** volume `profileArea · pathLength`; a smooth curved sweep is
watertight with volume converging to `profileArea · spineArcLength` within a deflection
bound; the RMF twist over a planar arc is ~0; and tight-curvature / self-intersecting /
degenerate inputs return NULL — all with no OCCT; (b) **sim parity** through the facade
(`cc_set_engine(1)` vs default) comparing native vs OCCT `BRepOffsetAPI_MakePipe` for
`cc_solid_sweep` (straight EXACT, curved deflection-bounded) and `cc_twisted_sweep`
where applicable, and asserting the fall-through cases (tight curvature,
`cc_guided_sweep`, `cc_loft_along_rail`) identical under both engines. Done only when
both gates pass and every existing suite stays green at the OCCT default.
