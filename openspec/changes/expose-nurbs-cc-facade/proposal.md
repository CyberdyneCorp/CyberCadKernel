## Why

Six waves (D–I) landed **28 exact-NURBS features** in `src/native/**` — surfacing
(skin / Gordon / Coons / N-sided C0·G1·G2·rational / variable & two-rail sweep /
rational revolve / G1·G2 surface join), blend (freeform-substrate G2 fillet,
vertex blend, the full chamfer family), offset/thicken (rational offset,
fold-trim, self-intersection-trimmed thicken/shell), fitting & reverse-engineering
(interpolation, approximation, rational weight estimation for curves and surfaces,
constrained fitting, analytic-primitive detection, minimal-energy fairing),
foundational ops (curve↔curve / curve↔surface intersection, exact analytic↔NURBS
conversion + recognition, tolerance-bounded knot-removal / degree reduction), and
the Layer-8 parameter-space trim boolean.

**None of them are reachable through the `cc_*` ABI.** Every wave was additive in
the OCCT-free native kernel and kept `cc_*` byte-unchanged, so the 154 existing
symbols still expose only the older engine-level surface (primitives, engine
sweep/loft/fillet/chamfer/shell, booleans). The new exact-NURBS geometry cannot be
constructed, inspected, tessellated, or **shown** from Python or the iOS bridge —
it is implemented but not usable or showcasable.

The blocker is not the algorithms; it is that `cc_*` is a **handle-based ABI for
solids and faces** with no representation for a raw NURBS **curve** or **surface**.
Exposing the new features therefore requires one deliberate, additive data-model
addition to the ABI — new opaque `cc_curve` / `cc_surface` handle types with POD
accessors — after which the feature wrappers, Python bindings, and an example
gallery follow mechanically.

## What Changes

- **Additive NURBS geometry ABI.** Add opaque `cc_curve` and `cc_surface` integer
  handle types (registry-backed, mirroring the existing shape-handle model) with:
  constructors from raw data, evaluators, POD accessors that read back
  degree / knots / control points / weights, a **display tessellation** bridge
  (`cc_surface_tessellate` → `CCMesh`; `cc_curve_polyline`), and
  `cc_curve_release` / `cc_surface_release` lifetime. Honest-decline is an error
  code + `cc_last_error` (a `0` handle), never a silently-wrong result.
- **Feature facade wrappers** (`cc_nurbs_*`) over the Wave D–I native modules,
  grouped by subsystem: fitting/reverse-engineering, surfacing, blend,
  offset/thicken, analytic conversion, intersection, and the parameter-space trim
  boolean. Each returns a `cc_curve` / `cc_surface` / `CCMesh` (or an honest
  decline).
- **Python bindings.** Extend the `cybercadkernel` package with `Curve` / `Surface`
  RAII classes over the new handles (NumPy pole/knot/weight accessors, `.eval`,
  `.tessellate` → mesh), and a `nurbs` submodule surfacing the feature wrappers,
  following the existing `api.py` RAII / `KernelError`-from-`cc_last_error` pattern.
- **Example gallery.** Mechanical-piece examples, each exercising a Wave D–I
  feature end-to-end and rendering a PNG (extending the existing 15-piece gallery).
- **pytest coverage + ABI-layout guards** for the new handles, structs, and
  wrappers, asserting real geometry (evaluation exactness, tessellation
  watertightness of a closed surface, round-trip recognition) and the
  honest-decline contract.

The existing 154 `cc_*` symbols and their POD structs stay **byte-unchanged**; all
additions are new symbols and new handle types. The iOS app's actively-called
subset is untouched (per `ios-app-compat`), so adoption is unaffected.

## Capabilities

### New Capabilities
- `nurbs-cc-facade`: an additive `cc_*` surface exposing the Wave D–I exact-NURBS
  geometry — `cc_curve` / `cc_surface` handles with POD accessors + display
  tessellation, the `cc_nurbs_*` feature wrappers, a Python `Curve`/`Surface`
  object model over them, and an example gallery — with the honest-decline
  contract preserved and the existing ABI byte-unchanged.

## Impact

- **ABI (data model): additive only.** New handle types + `cc_nurbs_*` functions +
  new POD accessor structs. The reviewable decision is the geometry-crossing model
  (handles + accessors vs. per-call serialization) — see `design.md`. No existing
  symbol or struct changes; `test_abi` and the `ios-app-compat` parity guarantee
  hold.
- **Facade / engine**: new `src/facade` wrappers bridging to `src/native/math`,
  `src/native/blend`, `src/native/topology`. The facade bridges; `src/native`
  stays OCCT-free.
- **Python tree**: new `Curve`/`Surface`/`nurbs` surface + tests + gallery.
- **Tooling & showcase only**: display tessellation of a single NURBS surface is
  for visualization; it is explicitly **not** the watertight multi-face curved-seam
  weld (the frozen-mesher wall) and does not claim a watertight solid B-rep.
- **Out of scope**: the iOS Swift binding (tracked in `add-swift-binding`, which
  would consume these same `cc_*` additions); the general freeform multi-surface
  sew and the SSI floor (kernel residuals, not exposure targets); rewiring existing
  engine-level facade ops to the exact-NURBS backends.
