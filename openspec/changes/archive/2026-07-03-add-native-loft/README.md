# add-native-loft

Phase 4 capability **#4b Tier B** — the loft follow-up on `native-construction`
(#4). #4 made two construction ops native (`cc_solid_extrude` of a closed polygon,
`cc_solid_revolve` of a line-segment profile); **#4b Tier A**
(`add-native-construction-profiles`) added the holed / typed-profile extrudes and
the typed-profile revolve. Loft (`cc_solid_loft`, `cc_solid_loft_wires`) was
explicitly left as OCCT-fallthrough at both #4 and Tier A. This change moves the
**two-section ruled loft** native, from first principles on the existing #1–#3
foundations (`src/native/math`, `src/native/topology`, `src/native/tessellate`)
plus the #4 / Tier-A assemblers (`src/native/construct/`), keeping `src/native/`
OCCT-free and host-buildable.

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT),
and does NOT fake any sub-case — configurations genuinely too hard now (mismatched
point counts, sections that a ruled skin cannot close cleanly, a section that
degenerates to a point, and every 3+-section / guided / rail loft) are left as
honest OCCT-fallthrough in `NativeEngine` and documented as such.

## Scope (Tier B) — the two loft `cc_*` ops, TWO equal-point-count sections

| `cc_*` op | Native in this change | Falls through to OCCT (honest) |
|---|---|---|
| `cc_solid_loft` | bottom polygon `@z=0` + top polygon `@z=depth`, **EQUAL point count** → ruled skin (one ruled side face per corresponding edge pair) capped by the bottom + top **planar** faces → watertight `Solid` | mismatched point count; a section < 3 distinct points; a section that self-intersects or that the corresponding-vertex ruling cannot close cleanly → fall through |
| `cc_solid_loft_wires` | two arbitrary **3D** wires, **EQUAL point count** → ruled skin; caps are the two section faces (planar for planar sections) | mismatched count; a section degenerating to a point; a non-planar section whose cap cannot be built cleanly; a ruling that self-intersects → fall through |

3+ section, guided, and rail lofts (`cc_loft_along_rail`, `cc_guided_sweep`) are
**Tier C** and remain OCCT-fallthrough (not in this change).

## What "ruled skin" means (the delivered geometry)

Two section wires with the **same** vertex count `n` are paired vertex-for-vertex
(`a[i] ↔ b[i]`) and edge-for-edge (`a[i]→a[i+1]` ↔ `b[i]→b[i+1]`). For each of the
`n` corresponding edge pairs we build ONE **ruled side face** — the bilinear /
degree-1 surface interpolating the two edges (a `Bezier`/`BSpline` degree-1 patch
via `src/native/math`, degenerating to a `Plane` when the four corners are
coplanar). The `n` side faces plus the bottom and top **cap faces** (planar for
planar sections) assemble into one closed, outward-oriented, watertight `Shell` →
`Solid`. This is exactly OCCT `BRepOffsetAPI_ThruSections(isSolid=true,
ruled=true)` over two sections — the reference oracle for the parity gate.

### Why the deferred configurations fall through (not faked)

- **Mismatched point counts.** A ruled skin pairs vertex `i` with vertex `i`; with
  unequal counts there is no canonical 1:1 correspondence (OCCT re-parameterises /
  inserts points — a strictly harder, separately verifiable piece). The native
  builder returns a NULL `Shape` and the engine falls through.
- **A section degenerating to a point.** A punctual section (all points coincident)
  makes the ruled side faces triangular fans to an apex; that is a genuinely
  different topology (a cone-like cap, not a quad-ruled band) and is deferred rather
  than emitting a degenerate/near-zero-area quad face.
- **Non-planar sections that cannot cap cleanly.** `cc_solid_loft_wires` accepts
  arbitrary 3D wires; a section that is not planar (its points are not co-planar
  within tolerance) has no single planar cap face, and a general non-planar cap is a
  separate hard piece — the builder returns NULL and falls through.
- **Self-intersecting sections / rulings.** If a section polygon self-intersects, or
  the corresponding-vertex ruling produces a self-intersecting side, the result is
  not a valid watertight solid; the builder rejects it (NULL → fall through) rather
  than emit an invalid B-rep.
- **3+ sections / guided / rail loft.** Out of Tier B scope entirely (Tier C).

## Method (locked, per NATIVE-REWRITE.md)

Clean-room from the `cc_*` contract semantics (the `cc_solid_loft` /
`cc_solid_loft_wires` doc-comments in `include/cybercadkernel/cc_kernel.h`:
`cc_solid_loft` = bottom XY `@z=0` + top XY `@z=depth`, same point count → ruled
solid; `cc_solid_loft_wires` = two arbitrary 3D wires, same point count) and
computational-geometry first principles, with OCCT source
(`/Users/leonardoaraujo/work/OCCT/src`:
`BRepOffsetAPI_ThruSections` with `ruled=true`, whose `CreateRuled` pairs
consecutive sections' edges into ruled surfaces) consulted as a **reference oracle
only** — never copied. Ruled side faces are real native-math degree-1
(`Bezier`/`BSpline`) surfaces — or an exact `Plane` when the corners are coplanar —
not sampled polylines. Balance maintainability / readability / performance: a short
linear assembler over `ShapeBuilder`, with the one-ruled-face-per-edge-pair loop
extracted; systems-band cognitive complexity only where the planar-vs-ruled surface
classification is genuinely irreducible.

## Architecture / OCCT boundary (unchanged from #4)

- The new native loft builder lives under `src/native/construct/` and stays
  **OCCT-FREE and host-buildable** (`/opt/homebrew/opt/llvm/bin/clang++
  -std=c++20`, no OCCT, no simulator); it includes only `src/native/math` +
  `src/native/topology` + `src/native/tessellate` and returns a `topology::Shape`.
  A **ruled / skin surface helper** is added to `src/native/math` if the existing
  `Bezier`/`BSpline` surface primitives do not already express a degree-1 skin
  directly.
- `src/engine/native/native_engine.{h,cpp}` gains native implementations of
  `solid_loft` and `solid_loft_wires` (currently pure fall-through). A configuration
  the native builder does not handle (mismatched count, punctual/non-planar section,
  non-closeable ruling) returns a NULL native shape so the glue **falls through to
  the fallback** engine — OCCT only under `CYBERCAD_HAS_OCCT`, else the stub — with
  no native interception and no fake.
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via
  `cc_set_engine(1)`), so every existing suite is unchanged unless it opts in.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT): the built native
   B-rep + its native tessellation — watertight (`boundaryEdgeCount == 0`), exact
   face/edge/vertex counts (`n` side faces + 2 caps, `n` corresponding ruled edges),
   ruled side faces carry a degree-1 skin surface (or `Plane` when coplanar), and
   **exact/convergent** analytic volume for closed-form cases (a prism when both
   sections are the identical polygon at two z-levels equals `area·depth`; a
   square→square offset loft, a frustum-like square→smaller-square loft equals the
   prismatoid volume by the prismatoid formula).
2. **Simulator native-vs-OCCT parity through the facade**: the SAME
   `cc_solid_loft` / `cc_solid_loft_wires` calls issued native (`cc_set_engine(1)`)
   vs OCCT (default), compared on mass properties / bbox / sub-shape counts /
   watertight tessellation against `BRepOffsetAPI_ThruSections(ruled=true)` within a
   documented fp64/deflection tolerance; and the deferred configurations (mismatched
   count, punctual section, non-planar wire) asserted **identical** under both
   engines (fall-through proof). Default restored in teardown.

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) stays green at the
OCCT default.
