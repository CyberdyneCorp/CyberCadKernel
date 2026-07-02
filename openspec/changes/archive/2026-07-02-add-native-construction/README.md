# add-native-construction

Phase 4 capability #4 (`native-construction`): the first **engine-wired** native
capability. It delivers an OCCT-free, host-buildable C++20 **swept-solid
constructor** under `src/native/construct/` that builds real native B-rep
(`src/native/topology`) with real native geometry (`src/native/math`), plus a
`NativeEngine : IEngine` (under `src/engine/native/`) that routes a subset of the
`cc_*` construction calls to that native constructor and **falls through to the
OCCT engine for everything else**. An additive facade toggle
`cc_set_engine(int native)` / `cc_active_engine(void)` — modelled exactly on the
existing `cc_set_parallel` / `cc_parallel_enabled` pattern — swaps the active
engine at runtime so the SAME `cc_*` call can be answered natively or by OCCT for
an A/B parity comparison. **The default stays OCCT**, so every existing suite is
byte-for-byte unchanged unless a test explicitly opts in.

## Honest scope (native vs deferred)

This change does NOT implement all of construction. It implements exactly two
construction operations natively, from first principles:

**NATIVE (delivered here):**

- `cc_solid_extrude` — extrude a **closed polygon profile** (the `profileXY`
  point list) along `+Z` by `depth` into a **prism solid**: one bottom planar
  face + one top planar face + one **planar quad side face per profile edge**,
  assembled into a closed shell → solid with fully native topology and geometry
  (shared vertices/edges, consistent outward orientation).
- `cc_solid_revolve` for **LINE-SEGMENT profiles** — revolve a profile made of
  straight segments about an axis: each segment sweeps to a **planar**
  (perpendicular-to-axis segment), **cylindrical** (parallel-to-axis segment),
  or **conical** (oblique segment) face of revolution. A full 360° revolution
  closes the solid; a partial angle adds the two **planar start/end cap faces**.

**DEFERRED (documented, NOT faked — falls through to OCCT):**

- `cc_solid_loft`, `cc_solid_loft_wires`
- `cc_solid_sweep`, `cc_twisted_sweep`, `cc_loft_along_rail`, `cc_guided_sweep`
- `cc_helical_thread`, `cc_tapered_thread`, `cc_tapered_shank`, `cc_wrap_emboss`
- `cc_solid_extrude_holes`, `cc_solid_extrude_polyholes`,
  `cc_solid_extrude_profile`, `cc_solid_extrude_profile_polyholes` (holed / typed
  profile extrude variants)
- `cc_solid_revolve_profile` and any `cc_solid_revolve` whose profile contains
  **arc or spline** segments (curved revolve)
- every non-construction capability (feature / boolean / tessellate / query /
  transform / exchange)

All deferred ops fall through the `NativeEngine` to the OCCT engine, so the app
keeps working unchanged behind the `cc_*` facade throughout the migration.

## Architecture / OCCT boundary

- `src/native/construct/` is **OCCT-FREE and host-buildable**: it includes only
  `src/native/math` and `src/native/topology`. It compiles and unit-tests with
  `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT and NO simulator.
- `src/engine/native/native_engine.*` is the glue. It implements `IEngine`,
  calls the OCCT-free constructor for the two native ops, and holds a
  `std::shared_ptr<IEngine>` **fallback** engine it delegates every other method
  to. The fallback is set to the OCCT engine **only inside `CYBERCAD_HAS_OCCT`**;
  on the host build the fallback is the stub, so the glue references OCCT only
  under that guard.

## Verification (two independent gates, per NATIVE-REWRITE.md)

1. **Host unit tests** — build `src/native/construct/` with `clang++ -std=c++20`
   (no OCCT, no simulator) and assert **exact** closed-form properties on the
   built native B-rep + its native tessellation (`src/native/tessellate`):
   watertight (`isWatertight`, `boundaryEdgeCount == 0`), exact face/edge/vertex
   counts, and enclosed volume / surface area matching the analytic value for a
   box, a triangular prism, a full-360 cylinder-by-revolve, a cone-by-revolve,
   and a partial-angle wedge — no OCCT anywhere.
2. **Simulator native-vs-OCCT parity through the facade** — on a booted iOS
   simulator (OCCT linked), the SAME `cc_solid_extrude` / `cc_solid_revolve` call
   is issued with the active engine set native (`cc_set_engine(1)`) and with it
   left OCCT (default). The two resulting shapes are compared through the facade
   (mass properties / bounding box / sub-shape counts / watertight tessellation)
   within a tight fp64 tolerance. The deferred ops are asserted to produce
   **identical** results under both engines (proving fall-through).

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3 suites) stays
green with the engine left at its OCCT default.
