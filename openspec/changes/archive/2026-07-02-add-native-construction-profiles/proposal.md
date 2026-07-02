# Proposal — add-native-construction-profiles

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capability #4 (`native-construction`) landed the first
engine-wired native ops — polygon extrude and line-segment revolve — but its
`README`/spec are explicit that every **holed**, **typed-profile**, and
**curved-revolve** variant is DEFERRED and still falls through to OCCT (`#4b`). Those
variants back real product features (plates with bolt holes, arc/circle-edged
profiles selectable as one edge, revolved shells with fillet-able circular edges), so
they are the next honest increment of native coverage.

Doing all of `#4b` (loft, sweep, twisted/guided sweep, threads, holed + typed
extrude, arc/spline revolve) in one change would risk faking coverage. This proposal
scopes **Tier A** — the holed / typed-profile extrudes and the typed-profile revolve —
whose geometry is fully expressible with the existing #1–#3 foundations, and is
explicit about which sub-cases (SPLINE edges, torus-revolve, spline-revolve) remain
honest OCCT fall-through.

## What changes

1. **Native holed prisms** (`src/native/construct/`, OCCT-free). Extend the #4 prism
   assembler to accept **inner wires (holes)**: a face becomes a `Plane` bounded by an
   outer wire **plus one reversed inner wire per hole**; each hole is swept along `+Z`
   into its own inner side shell so the solid is watertight with real inner-wire
   topology.
   - **Circular holes** (`holesCenterRadius` = cx,cy,r triples) → each hole wire is a
     single **TRUE `Circle` `EdgeCurve`** (one selectable edge), swept into a native
     `Cylinder` inner face.
   - **Polygon holes** (`holesXY` + per-hole `holeCounts`) → each hole wire is N line
     edges swept into N planar inner side faces.
2. **Native typed-profile prisms** (`src/native/construct/`). Build the OUTER wire from
   a `CCProfileSeg` list where each segment is a real native-math curved edge:
   **kind 0 line** → `Line` edge, **kind 1 arc** → `Circle` `EdgeCurve` over `[a0,a1]`,
   **kind 2 full circle** → a closed `Circle` edge (a one-edge circular face). The
   resulting outer profile is extruded to a prism whose curved outer edges sweep to
   native `Cylinder` side faces; circular holes are added as in (1). **kind 3 SPLINE**
   is detected and returns NULL → fall through to OCCT (not faked).
3. **Native typed-profile revolve** (`src/native/construct/`). Revolve a `CCProfileSeg`
   profile about an in-plane axis `(ax,ay,adx,ady)`. Per segment:
   - **kind 0 line** → Plane / Cylinder / Cone face of revolution (as #4, generalised
     to an arbitrary in-plane axis).
   - **kind 1 arc / kind 2 circle whose supporting circle is centred ON the axis** →
     **`Sphere`** face of revolution (degenerating to Plane/Cone at the limits).
   - **arc offset from the axis** (swept surface is a **torus** — no native analytic
     type) → NULL → fall through. **kind 3 SPLINE** and any spline-revolve → NULL →
     fall through.
   Circular edges of revolution are TRUE `Circle` edges; full 360° closes the shell,
   partial angle adds two planar meridian caps.
4. **`NativeEngine` glue** (`src/engine/native/native_engine.{h,cpp}`). The five
   overrides — currently pure `fallback()` delegations — call the native builders and,
   on a NULL native result (an unsupported sub-case), fall through to the fallback with
   no interception. OCCT stays behind `CYBERCAD_HAS_OCCT`; the native builders never
   see OCCT.

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- **kind-3 SPLINE profile edges** in `cc_solid_extrude_profile` /
  `cc_solid_extrude_profile_polyholes` — the native builder detects a kind-3 segment
  and returns NULL so the whole call falls through to OCCT.
- **Arc-revolve whose swept surface is a torus** (an arc segment not passing through
  the axis) — no native `Torus` surface type; falls through.
- **Any spline-revolve** (kind-3 segment in `cc_solid_revolve_profile`, or a B-spline
  surface of revolution) — falls through.
- All the other `#4b` ops (loft, sweep, twisted/guided sweep, threads, `wrap_emboss`)
  and every feature / boolean / query / transform / exchange op — remain #4-style
  fall-through, unchanged.

## Impact

- `src/native/construct/` gains holed-prism, typed-profile-prism, and typed-profile-
  revolve builders (still OCCT-free, host-buildable; extend the existing headers /
  add sibling headers). New host CTest cases.
- `src/engine/native/native_engine.cpp` — five methods change from pure fall-through
  to "native-else-fallback". `native_engine.h` unchanged (signatures already present).
- **No** `include/cybercadkernel/cc_kernel.h` signature or POD change; **no**
  `src/facade/cc_kernel.cpp` change (the five `cc_*` entry points already route through
  the active engine). The `CCProfileSeg` struct and the five doc-comments are the
  contract this change implements natively.
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the new native path. All existing suites stay green at the
  OCCT default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** exact-value unit tests on
the built native B-rep + native tessellation (watertight, exact inner-wire / face /
edge counts, TRUE `Circle` edges, analytic volume/area for holed / arc-profile /
circle-profile extrudes and line/arc-profile revolves) with no OCCT; (b) **sim
parity** through the facade (`cc_set_engine(1)` vs default) comparing native vs OCCT
for the five ops within a tight tolerance, and asserting the deferred sub-cases
(SPLINE edge, torus-revolve, spline-revolve) are identical under both engines
(fall-through proof). Done only when both gates pass and every existing suite stays
green at the OCCT default.
