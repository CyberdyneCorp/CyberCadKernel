# Proposal — add-native-loft

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capability #4 (`native-construction`) landed the
first engine-wired native ops — polygon extrude and line-segment revolve — and #4b
Tier A (`add-native-construction-profiles`) added the holed / typed-profile
extrudes and the typed-profile revolve. Both are explicit that **loft**
(`cc_solid_loft`, `cc_solid_loft_wires`) is DEFERRED and still falls through to OCCT
(a later #4b tier). Loft backs real product features (transition solids between two
cross-sections — adapters, chutes, blended plates), so the two-section ruled loft is
the next honest increment of native coverage.

Doing all of the remaining `#4b` (loft, sweep, twisted/guided sweep, threads,
`wrap_emboss`) in one change would risk faking coverage. This proposal scopes
**Tier B** — the **two-section ruled loft** with **equal point-count** sections,
whose geometry is fully expressible as a ruled skin (degree-1 side faces + two cap
faces) with the existing #1–#3 foundations — and is explicit about which
configurations (mismatched counts, punctual sections, non-planar wires, 3+-section /
guided / rail lofts) remain honest OCCT fall-through.

## What changes

1. **Ruled / skin surface support in native-math** (`src/native/math`, OCCT-free).
   If the existing `Bezier`/`BSpline` surface primitives do not already express a
   degree-1 (bilinear) skin between two edges directly, add a small **ruled-surface
   helper** that evaluates the surface interpolating two boundary curves at
   parameter `(u,v)` — for a straight-edge pair this is the bilinear patch, expressed
   as a degree-1 `Bezier`/`BSpline` surface (and recognised as an exact `Plane` when
   the four corners are coplanar). Host analytic tests pin known points.

2. **Native two-section ruled loft** (`src/native/construct/`, OCCT-free). A new
   builder assembles the ruled skin:
   - Validate the two sections: EQUAL vertex count `n ≥ 3`, each with ≥ 3 distinct
     points, neither degenerating to a point, each planar within tolerance for its
     cap. Any failure → return a NULL `Shape` (fall through).
   - Pair vertices `a[i] ↔ b[i]` and edges `a[i]→a[i+1]` ↔ `b[i]→b[i+1]`. Build the
     two vertical/oblique **connecting edges** `a[i]→b[i]` once each (shared between
     adjacent side faces) and one **ruled side face** per corresponding edge pair —
     a degree-1 skin surface (or `Plane` when coplanar) bounded by the four-edge
     wire (`a[i]→a[i+1]`, `a[i+1]→b[i+1]`, `b[i+1]→b[i]`, `b[i]→a[i]`).
   - Build the **bottom** cap (section A) and **top** cap (section B) as planar
     faces (planar sections). Orient every face outward.
   - Assemble one closed `Shell` (`n` side faces + 2 caps) → `Solid`; reject if the
     result is not watertight / manifold (NULL → fall through).

3. **`cc_solid_loft`** = the XY-polygon convenience: bottom `(x,y)` pairs at `z=0`,
   top `(x,y)` pairs at `z=depth`, both count `n`. It lifts the two polygons into
   the two 3D sections and calls the same builder. **`cc_solid_loft_wires`** = the
   general form: two arbitrary 3D wires (`x,y,z` triples), both count `n`, passed
   straight to the builder; caps are the two section planes (planar sections).

4. **`NativeEngine` glue** (`src/engine/native/native_engine.{h,cpp}`). The two
   overrides — currently pure `fallback()` delegations — call the native builder and,
   on a NULL native result (mismatched count, punctual/non-planar section,
   non-closeable ruling, or any 3+-section/guided/rail path which never reaches this
   builder), fall through to the fallback with no interception. OCCT stays behind
   `CYBERCAD_HAS_OCCT`; the native builder never sees OCCT.

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- **Mismatched point-count sections** in `cc_solid_loft` / `cc_solid_loft_wires` —
  no canonical 1:1 vertex correspondence; the native builder returns NULL → OCCT.
- **A section that degenerates to a point** (punctual section) — a cone-like apex
  cap, not a quad-ruled band; falls through.
- **Non-planar section wires** in `cc_solid_loft_wires` whose cap cannot be built as
  a single planar face — falls through.
- **Self-intersecting sections or rulings** that cannot close into a valid
  watertight solid — rejected (NULL → fall through), never emitted as an invalid
  B-rep.
- **3+ section lofts, `cc_loft_along_rail`, `cc_guided_sweep`** and every other
  sweep / thread / `wrap_emboss` op — Tier C and beyond; remain #4-style
  fall-through, unchanged.
- Every feature / boolean / query / transform / exchange op — out of construction;
  delegated to the fallback.

## Impact

- `src/native/math/` MAY gain a small ruled/skin-surface helper (OCCT-free,
  host-buildable) with host analytic tests; the existing `Bezier`/`BSpline` surface
  code is reused if it already suffices.
- `src/native/construct/` gains a two-section ruled-loft builder (still OCCT-free,
  host-buildable; extend the existing headers / add a sibling header aggregated by
  `native_construct.h`). New host CTest cases.
- `src/engine/native/native_engine.cpp` — `solid_loft` and `solid_loft_wires` change
  from pure fall-through to "native-else-fallback". `native_engine.h` unchanged (the
  two signatures already exist).
- **No** `include/cybercadkernel/cc_kernel.h` signature or POD change; **no**
  `src/facade/cc_kernel.cpp` change (both `cc_*` entry points already route through
  the active engine). The two doc-comments are the contract this change implements
  natively.
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the new native path. All existing suites stay green at the
  OCCT default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** exact/convergent-value
unit tests on the built native B-rep + native tessellation (watertight, exact
side-face / cap / edge / vertex counts, degree-1 ruled skin surfaces — or `Plane`
when coplanar, analytic volume for prism / square-offset / frustum-like prismatoid
lofts) with no OCCT; (b) **sim parity** through the facade (`cc_set_engine(1)` vs
default) comparing native vs OCCT `BRepOffsetAPI_ThruSections(ruled=true)` for both
loft ops within a tight tolerance, and asserting the deferred configurations
(mismatched count, punctual section, non-planar wire) are identical under both
engines (fall-through proof). Done only when both gates pass and every existing
suite stays green at the OCCT default.
