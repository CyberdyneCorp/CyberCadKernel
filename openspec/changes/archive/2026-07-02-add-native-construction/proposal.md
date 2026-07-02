# Proposal — add-native-construction

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*`
facade (see `openspec/NATIVE-REWRITE.md`). Capabilities #1–#3 landed the
verified, OCCT-free foundations — `native-math` (`src/native/math`),
`native-topology` (`src/native/topology`), `native-tessellation`
(`src/native/tessellate`) — but none is wired into the engine yet. To make
progress visible and testable end-to-end, capability #4 (`native-construction`)
must be the **first engine-wired** capability: it builds real native solids from
those foundations and exposes them through the same `cc_*` call the app already
uses, so a native result can be compared A/B against the OCCT oracle at runtime.

Building ALL of construction at once (extrude, revolve, loft, sweep, threads,
holed/typed-profile variants) is not credible in one change and would risk
faking coverage. This proposal therefore scopes the native work to the two
operations whose geometry is fully expressible with the existing foundations —
**polygon extrude** and **line-segment revolve** — and is explicit that every
other construction op is DEFERRED and falls through to OCCT.

## What changes

1. **Native swept-solid constructor** (`src/native/construct/`, OCCT-free,
   host-buildable). Builds native B-rep + geometry via `ShapeBuilder`:
   - `extrudePolygon(profileXY, depth)` → prism `Solid`: bottom + top planar
     faces + one planar quad side face per profile edge, one closed shell,
     shared vertices/edges, outward-consistent orientation.
   - `revolveSegments(profileXY, angle)` → solid of revolution for a
     line-segment profile: each segment becomes a planar / cylindrical / conical
     face; full 360° closes; partial angle adds two planar cap faces.
2. **`NativeEngine : IEngine`** (`src/engine/native/`). Overrides
   `solid_extrude` and (line-segment) `solid_revolve` to call the native
   constructor; delegates EVERY other `IEngine` method to a held fallback engine
   (OCCT under `CYBERCAD_HAS_OCCT`, else the stub). Curved / typed / holed
   revolve+extrude variants and all non-construction ops fall through.
3. **Additive facade toggle** `cc_set_engine(int native)` / `int
   cc_active_engine(void)` (public C ABI, additive — not part of the mirrored
   bridge ABI, exactly like `cc_set_parallel`). `cc_set_engine(1)` installs the
   `NativeEngine` as active via `set_active_engine`; `cc_set_engine(0)` restores
   the OCCT/default engine. `cc_active_engine()` returns 1 iff the native engine
   is active. **Default stays OCCT** (the toggle is opt-in).

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- Loft (`cc_solid_loft`, `cc_solid_loft_wires`).
- Sweep and its variants (`cc_solid_sweep`, `cc_twisted_sweep`,
  `cc_loft_along_rail`, `cc_guided_sweep`).
- Threads / shanks / emboss (`cc_helical_thread`, `cc_tapered_thread`,
  `cc_tapered_shank`, `cc_wrap_emboss`).
- Holed / typed-profile extrude (`cc_solid_extrude_holes`,
  `cc_solid_extrude_polyholes`, `cc_solid_extrude_profile`,
  `cc_solid_extrude_profile_polyholes`).
- Curved revolve: `cc_solid_revolve_profile`, and `cc_solid_revolve` on a
  profile containing arc or spline segments.
- Every feature / boolean / tessellate / query / transform / exchange op.
- No native NURBS/analytic-surface **healing** or robustness hardening beyond
  what these two exact, analytic constructions require.

## Impact

- New OCCT-free library `src/native/construct/` (headers + host CTest target).
- New engine glue `src/engine/native/native_engine.{h,cpp}` (fallback delegation;
  references OCCT engine only under `CYBERCAD_HAS_OCCT`).
- Facade: two new additive C functions in `include/cybercadkernel/cc_kernel.h`
  and their thin `guard`-wrapped definitions in `src/facade/cc_kernel.cpp`. No
  existing signature or POD layout changes.
- Behaviour is unchanged by default (engine stays OCCT); only tests / callers
  that call `cc_set_engine(1)` see the native path. All existing suites stay
  green at the OCCT default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** exact-value unit
tests on the native B-rep + native tessellation (watertight, exact
counts, analytic volume/area) with no OCCT; (b) **sim parity** through the facade
(`cc_set_engine(1)` vs default) comparing native vs OCCT for the two native ops
within tight fp64 tolerance, and asserting deferred ops are identical under both
engines (fall-through proof). Done only when both gates pass and every existing
suite stays green at the OCCT default.
