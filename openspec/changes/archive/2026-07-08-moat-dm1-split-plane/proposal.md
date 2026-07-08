# Proposal — moat-dm1-split-plane (M-DM DM1, native `cc_split_plane`)

## Why

`cc_split_plane(body, o, n, keepPositive)` — cut a solid by a plane and keep ONE
half — is the **first Direct-Modeling verb** the MOAT roadmap (`openspec/MOAT-ROADMAP.md`
§M-DM) marks as **largely reachable NOW**: it "reuses the landed M2 `half_space_cut`
(B4) verb on each side + BSP for analytic solids", and it "has a closed-form partition
oracle (the two pieces sum to the whole)". Today the verb exists behind the `cc_*`
facade but is **OCCT-only**: `NativeEngine::split_plane` unconditionally
`CC_NATIVE_BODY_UNSUPPORTED` → `fallback().split_plane(...)`
(`src/engine/native/native_engine.cpp`), so every call routes to OCCT
`BRepPrimAPI_MakeHalfSpace` + `BRepAlgoAPI_Cut` (`src/engine/occt/occt_feature.cpp`).

The two substrates a native split needs already landed and are verified:

- **The freeform half-space CUT verb** — `boolean/half_space_cut.h`
  `freeformHalfSpaceCut(operand, P, KeepSide::{Below,Above})` — the first end-to-end
  freeform↔analytic half-space cut (M2/B4), gated HOST-analytic + SIM-vs-OCCT on the
  bowl-lidded convex-quad prism. `KeepSide::Below` and `KeepSide::Above` already yield
  the two complementary pieces of a plane cut (`V(x≤0)+V(x≥0)=V(full)` proven exact).
- **The analytic BSP/CSG boolean** — `boolean/native_boolean.h` `boolean_solid(a, b, op)`
  — the clean-room planar-polyhedron (and axis-aligned box⟷cylinder) cut/common that
  already computes `solid ∩ half-space` when the discard half is supplied as a large
  half-space box (exactly OCCT's `MakeHalfSpace` shape, built natively).

DM1 is the ADDITIVE assembly of these two landed verbs behind the existing facade:
route the native engine's `split_plane` to a native piece where the case is reachable,
and keep the OCCT fall-through everywhere else. It buys the first synchronous/direct
modeling operation on the native path with a **closed-form partition oracle** (the
strongest host gate the roadmap offers) and **no `cc_*` ABI change**.

## What Changes

1. **A native `split_plane` branch** in `src/engine/native/native_engine.cpp`, taken
   ONLY when `body` is a native solid in the reachable domain; otherwise the existing
   `fallback().split_plane(...)` is preserved BYTE-IDENTICAL. The branch maps
   `keepPositive` to the kept half-space (`keepPositive != 0` keeps the `+normal`
   half → `KeepSide::Above`; `0` → `KeepSide::Below`) and dispatches:
   - **Freeform-walled operand** (exactly one `Kind::Bezier`/`Kind::BSpline` wall, the
     M2 domain) → `freeformHalfSpaceCut(operand, P, side)` for the requested keep side.
   - **Analytic all-planar polyhedron** (box / prism / convex or simple-concave, every
     face a `Plane`) → `boolean_solid(operand, halfSpaceBox(P, discardSide), Op::Cut)`,
     where `halfSpaceBox` is a bbox-scaled planar box covering the DISCARD half — the
     landed BSP cut, consumed unchanged, capping the section on `P`.
   - **Axis-perpendicular cut of an axis-aligned cylinder** (the axis-aligned box⟷
     cylinder analytic slice) → the same `halfSpaceBox` cut through the native curved
     boolean; the two pieces are shorter coaxial cylinders with planar caps.
2. **A mandatory per-piece self-verify** reusing the engine's existing
   `watertightVolume` audit: the native piece is accepted ONLY when it is a closed
   watertight 2-manifold with positive enclosed volume; a candidate that fails is
   DISCARDED and the call falls through to OCCT (a native body OCCT cannot read reports
   an honest error — but a split operand is always OCCT-reconstructible, so OCCT is the
   true fall-through here).
3. **Honest declines → NULL → OCCT** for every case outside the reachable slice, each
   labelled and verified as a fall-through, never faked: an **oblique plane grazing /
   tangent to a curved face** (the trace is a general ellipse/conic, not a
   circle/line); a cut that would produce a **multi-lump piece** (the plane severs the
   solid into more than the two connected halves, or one half is disconnected); a
   **degenerate** configuration (the plane misses the solid, is coincident with a
   boundary face, or leaves a zero-volume sliver); a **multi-freeform** operand; a
   **non-native / foreign** body. Under the native engine each such case returns the
   EXACT OCCT result of the same call.
4. **`src/native/**` stays OCCT-free and the `cc_*` ABI is unchanged.** The two landed
   verbs (`freeformHalfSpaceCut`, `boolean_solid`) are consumed BYTE-IDENTICAL; the
   assembly is additive engine glue in `src/engine/native/` (which may reference the
   native headers but not OCCT). No existing analytic path changes.

## Capabilities

### Added Capabilities

- `native-directmodel`: ADDS the FIRST direct-modeling verb on the native path —
  native `cc_split_plane` — computing one watertight piece of a plane cut for a
  freeform-walled operand (via the landed M2 half-space CUT on the requested keep side)
  and for an analytic planar polyhedron / axis-aligned cylinder (via the landed BSP /
  curved boolean against a half-space box), self-verified per-piece and gated by a
  closed-form partition-closure oracle (`V(below)+V(above)=V(whole)`), with an honest
  decline → OCCT for grazing-tangent / multi-lump / degenerate / multi-freeform /
  foreign cases.

## Impact

- `src/engine/native/native_engine.cpp` — `NativeEngine::split_plane` gains the native
  branch + per-piece `watertightVolume` self-verify + labelled decline → `fallback()`.
  Cognitive complexity kept in the systems band (the driver delegates to keep-side
  mapping, domain-dispatch, and self-verify helpers; the geometry lives in the two
  already-landed verbs). The existing `CC_NATIVE_BODY_UNSUPPORTED` fall-through is the
  default for every unreachable case.
- `src/engine/native/native_engine.h` — no signature change (the `split_plane`
  override already exists).
- `src/native/boolean/half_space_cut.h`, `src/native/boolean/native_boolean.h` —
  CONSUMED unchanged (no edit); `freeformHalfSpaceCut` on both keep sides + `boolean_solid`.
- **`cc_*` ABI:** UNCHANGED. `cc_split_plane` signature, `IEngine::split_plane`, the
  facade `src/facade/cc_kernel.cpp`, and the OCCT `OcctEngine::split_plane` fallback are
  all byte-identical; DM1 only supplies a native path behind the existing seam.
- **Host gate (a) — analytic, OCCT-free:** for each reachable fixture (axis-aligned
  box; axis-aligned cylinder cut perpendicular to its axis; the bowl-lidded convex-quad
  prism) split by a plane, compute BOTH pieces (`keepPositive` 0 and 1). Each piece is
  watertight; `V(below)+V(above)` equals the whole solid's volume within the deflection
  tolerance; and each piece matches its closed-form volume where known (box: fp-exact;
  cylinder: `π·r²·h_i`; freeform: the landed closed-form band).
- **Sim gate (b) — native-vs-OCCT parity:** on a booted iOS simulator, for each keep
  side, compare the native piece against OCCT (`BRepAlgoAPI_Section` / the existing
  OCCT `split_plane` two-sided) on per-piece volume, area, watertightness (closed
  2-manifold), topology (Euler χ), and bbox — matching within the landed curved-slice
  tolerances (never widened).
- **Out of scope (declines, documented not faked):** oblique plane cutting a curved
  face (ellipse/conic section — the native slice's trace is only circle/line);
  multi-solid-result splits (a plane severing a solid into >2 lumps); coincident /
  tangent / zero-volume-sliver degeneracies; multi-freeform operands; foreign / OCCT
  bodies; the sibling DM verbs (`cc_replace_face`, `cc_replace_face_to_plane`, project)
  which are later DM slices. No `cc_*` ABI change; no OCCT linked into `src/native/**`;
  no tolerance weakened; no dead code.
