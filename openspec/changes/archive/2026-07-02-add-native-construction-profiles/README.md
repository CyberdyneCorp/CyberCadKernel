# add-native-construction-profiles

Phase 4 capability **#4b Tier A** — the first follow-up on `native-construction`
(#4). #4 made two construction ops native (`cc_solid_extrude` of a closed polygon,
`cc_solid_revolve` of a line-segment profile) behind the `NativeEngine` fall-through
and the additive `cc_set_engine` toggle; every holed / typed-profile / curved-revolve
variant still falls through to OCCT. This change moves **Tier A of #4b** native, from
first principles on the existing #1–#3 foundations (`src/native/math`,
`src/native/topology`, `src/native/tessellate`) plus the #4 assemblers
(`src/native/construct/`), keeping `src/native/` OCCT-free and host-buildable.

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT),
and does NOT fake any sub-case — sub-cases genuinely too hard now (kind-3 SPLINE
profile edges; any spline-revolve; arc-revolve whose swept surface is a torus) are
left as honest OCCT-fallthrough in `NativeEngine` and documented as such.

## Scope (Tier A) — the five `cc_*` ops moved native (per sub-case)

| `cc_*` op | Native in this change | Falls through to OCCT (honest) |
|---|---|---|
| `cc_solid_extrude_holes` | outer polygon + circular through-holes as **inner wires with TRUE `Circle` edges** | — (fully native) |
| `cc_solid_extrude_polyholes` | outer polygon + **polygon** inner-wire holes | — (fully native) |
| `cc_solid_extrude_profile` | typed profile edges **kind 0 line / 1 arc (`Circle`) / 2 full circle** + circular holes | **kind 3 SPLINE** profile edge → fall through |
| `cc_solid_extrude_profile_polyholes` | typed profile (kind 0/1/2) + circular holes + polygon holes | **kind 3 SPLINE** profile edge → fall through |
| `cc_solid_revolve_profile` | typed profile revolved about an in-plane axis: **kind 0 line** → Plane/Cylinder/Cone, **kind 1 arc / 2 circle whose meridian is centred ON the axis** → **Sphere** (or Plane/Cone limit) | **arc offset from the axis** (swept surface is a **torus**, not natively representable) → fall through; **kind 3 SPLINE** profile edge → fall through; any spline-revolve → fall through |

### Why those sub-cases fall through (not faked)

- **Torus surface of revolution.** `topology::FaceSurface::Kind` is
  `{ Plane, Cylinder, Cone, Sphere, BSpline, Bezier }` — there is **no analytic
  `Torus`**. An arc segment that does **not** pass through the axis sweeps a torus.
  Representing it as a BSpline surface of revolution is possible but is a strictly
  harder, separately verifiable piece; Tier A defers it rather than emit an
  approximate/faked surface. An arc whose supporting circle is **centred on the
  axis** sweeps a **`Sphere`** (native) — that sub-case IS delivered.
- **Spline (kind-3) profile edges / spline-revolve.** `EdgeCurve`/`FaceSurface`
  carry `BSpline`/`Bezier`, so the data model can hold them, but building a correct,
  watertight, seam-welded B-spline **surface of revolution** (and its shared curved
  edges) is the hardest sub-case and is deferred to a later #4b tier — the native
  path must not fake it.

## Method (locked, per NATIVE-REWRITE.md)

Clean-room from the `cc_*` contract semantics (`CCProfileSeg` + the doc-comments on
the five ops in `include/cybercadkernel/cc_kernel.h`) and computational-geometry
first principles, with OCCT source (`/Users/leonardoaraujo/work/OCCT/src`:
`BRepBuilderAPI_MakeFace` inner-wire holes, `BRepPrimAPI_MakeRevol` / `BRepFill`,
`ElSLib`) consulted as a **reference oracle only** — never copied. TRUE curved
edges (arc/circle profile edges, circular hole edges, circular edges of revolution)
are built as real native-math `Circle` `EdgeCurve`s, not sampled polylines. Balance
maintainability / readability / performance — short assemblers over `ShapeBuilder`,
systems-band cognitive complexity only where the surface-of-revolution / hole-wire
classification is genuinely irreducible.

## Architecture / OCCT boundary (unchanged from #4)

- New native builders live under `src/native/construct/` and stay **OCCT-FREE and
  host-buildable** (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no
  simulator); they include only `src/native/math` + `src/native/topology` +
  `src/native/tessellate` and return `topology::Shape`.
- `src/engine/native/native_engine.{h,cpp}` gains native implementations of the five
  ops (currently pure fall-through). A sub-case the native builder does not handle
  (SPLINE edge, torus-revolve, spline-revolve) returns a NULL native shape so the
  glue **falls through to the fallback** engine — OCCT only under
  `CYBERCAD_HAS_OCCT`, else the stub — with no native interception and no fake.
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via
  `cc_set_engine(1)`), so every existing suite is unchanged unless it opts in.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT): built native B-rep +
   its native tessellation — watertight (`boundaryEdgeCount == 0`), exact inner-wire /
   face / edge counts, TRUE `Circle` edges present, and **exact** analytic volume /
   area (a plate with a round hole = `(w·h − π·r²)·d`; a plate with a polygon hole =
   `(A_outer − A_hole)·d`; an arc-profile / full-circle extrude; a line/arc-profile
   revolve → cylinder / cone / sphere volume).
2. **Simulator native-vs-OCCT parity through the facade**: the SAME five `cc_*` calls
   issued native (`cc_set_engine(1)`) vs OCCT (default), compared on mass properties /
   bbox / sub-shape counts / watertight tessellation within a documented fp64/
   deflection tolerance; and the deferred sub-cases (SPLINE edge, torus-revolve,
   spline-revolve) asserted **identical** under both engines (fall-through proof).
   Default restored in teardown.

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) stays green at the
OCCT default.
