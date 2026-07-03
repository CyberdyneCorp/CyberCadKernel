# Tasks — add-native-fillets-offsets (Phase 4 #6, native-blends)

Order: planar-blend predicates → chamfer builder → constant fillet builder → offset builder
→ shell builder → builder API → self-verify guard → engine wiring → Gate 1 (host) → Gate 2
(sim parity) → docs. Native code stays OCCT-free + host-buildable
(`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No `cc_*` ABI change. Default engine stays
OCCT.

> ## SCOPE NOTE (honest — tractable-planar slice of the blend family)
> This change delivers `cc_chamfer_edges` (planar chamfer on a convex planar-planar edge),
> `cc_fillet_edges` (constant-radius rolling-ball fillet on a convex planar-dihedral edge),
> `cc_offset_face` (planar face offset), and `cc_shell` (uniform-thickness hollow of a planar
> / box-like solid) NATIVE only — verified against the OCCT oracle (EXACT for chamfer /
> offset / box-shell, deflection-bounded for the cylindrical fillet), and DISCARDED by a
> mandatory self-verify (falling through to OCCT) whenever the native result is not a valid
> watertight solid with a sane volume for the op. Curved-face inputs, concave edges,
> variable-radius (`cc_fillet_edges_variable`), `cc_fillet_face`, multi-edge interference, and
> non-native bodies FALL THROUGH to OCCT (labelled, verified, never faked). It is fully
> acceptable — and MUST be reported truthfully — if only chamfer + planar-dihedral constant
> fillet + planar offset (+ box shell) land native and everything else is OCCT-fallback.
> General blending (curved faces, concave edges, variable radius, face fillet, multi-edge
> corner interference) is future work.

## 1. Native planar-blend predicates + geometry (`src/native/blend/`, OCCT-free)

- [x] 1.1 Convex planar-dihedral edge classifier — an edge is in-slice iff it is a STRAIGHT
  segment shared by exactly two faces, BOTH `FaceSurface::kind == Plane`, and the dihedral is
  CONVEX (outward normals open away from the material). Concave / curved / non-manifold ⇒
  out-of-slice.
- [x] 1.2 In-face perpendicular-to-edge direction (for each adjoining planar face, the unit
  in-plane direction from the edge into the face interior) + setback line at a given distance.
- [x] 1.3 Rolling-ball tangent-cylinder solve for a planar dihedral (radius `r`, axis parallel
  to the edge on the interior bisector at distance `r / sin(halfAngle)`, two tangent/contact
  lines) — REUSE the Phase-3 full-round construction (`src/native/construct/`).
- [x] 1.4 Planar-cutter builder (a half-space / wedge planar cutter solid) for chamfer / shell
  subtraction via the native `src/native/boolean/boolean_solid`.
- [x] 1.5 Confirm all predicates stay within cognitive-complexity targets (short irreducible
  geometry ~5–10; systems band ≤ 25 if a loop is irreducible) with the `cognitive-complexity`
  skill.

## 2. Chamfer builder (`src/native/blend/`)

- [x] 2.1 `chamfer_edges(shape, edgeIds, edgeCount, distance)` — for each in-slice convex
  planar-planar edge: cut both faces back by `distance` and insert one new planar chamfer face
  (via the planar cutter boolean OR a direct topology edit). Return a `topology::Shape` Solid.
- [x] 2.2 DECLINE (NULL) on a curved-face edge, a concave edge, a multi-edge-interference
  selection, or degenerate input (zero-length edge, non-positive distance).

## 3. Constant fillet builder (`src/native/blend/`)

- [x] 3.1 `fillet_edges(shape, edgeIds, edgeCount, radius)` — for each in-slice convex planar-
  dihedral edge: build the rolling-ball tangent cylinder (§1.3), trim both faces to their
  contact lines, insert the cylindrical blend face + the planar setback faces, close the
  solid. Return a `topology::Shape` Solid whose blend face is a `Cylinder` of radius `r`.
- [x] 3.2 DECLINE (NULL) on a curved-face edge, a concave edge, a multi-edge-interference
  selection, or degenerate input (non-positive radius, radius too large for the edge span).

## 4. Offset builder (`src/native/blend/`)

- [x] 4.1 `offset_face(shape, faceId, distance)` — for a PLANAR selected face: translate it
  along its outward normal by `distance`, re-plane it, re-loft the adjacent side faces to the
  moved boundary, close the solid (grow for `d>0`, shrink for `d<0`). Return a `topology::Shape`.
- [x] 4.2 DECLINE (NULL) on a curved selected face, a non-prismatic adjacency, or degenerate
  input (zero-area face, inward `distance` collapsing the solid).

## 5. Shell builder (`src/native/blend/`)

- [x] 5.1 `shell(shape, faceIds, faceCount, thickness)` — for a PLANAR / box-like solid:
  offset every RETAINED face inward by `thickness` to form the inner void (opening faces left
  open), subtract the inner void from the outer solid via `boolean_solid` (offset + boolean),
  leaving a uniform-thickness wall open at the selected faces. Return a `topology::Shape`.
- [x] 5.2 DECLINE (NULL) on a non-planar / non-convex solid, a curved face, a non-uniform
  thickness, a wall ≥ half the smallest span (self-intersecting inner offset), or degenerate
  input (non-positive thickness, empty face selection).

## 6. Native builder API surface (`src/native/blend/native_blend.h`)

- [x] 6.1 Expose `chamfer_edges`, `fillet_edges`, `offset_face`, `shell` (each taking a
  `const topology::Shape&` + the op params, returning a `topology::Shape`, NULL ⇒ fall
  through) in `namespace cybercad::native::blend`.
- [x] 6.2 Preflight guards in each builder: any curved face touching the op ⇒ NULL; concave
  edge ⇒ NULL; multi-edge interference ⇒ NULL; degenerate operand ⇒ NULL.
- [x] 6.3 Add `native_blend.h` umbrella including the builder headers; keep the subtree
  OCCT-free + host-buildable (includes only `src/native/math` + `src/native/topology` +
  `src/native/tessellate` + `src/native/boolean` + `src/native/construct`).
- [x] 6.4 Verify each builder + predicate stays within cognitive-complexity targets (builders
  ≤ 15; predicates ~5–10; flag any irreducible face-rebuild / setback loop systems-band ≤ 25)
  with the `cognitive-complexity` skill.

## 7. Mandatory self-verify guard (`src/engine/native/native_engine.cpp`)

- [x] 7.1 Reuse `robustlyWatertight(solid)` for the closed-watertight-2-manifold +
  positive-volume check across the deflection ladder.
- [x] 7.2 Add a `blendResultVerified(result, in, op, param)` helper: `robustlyWatertight(result)`
  AND a per-op sane-volume check — chamfer `Vr < Vin` (`≈ Vin − ½ d²·L` on a box); fillet
  `Vr < Vin` (`≈ Vin − (1−π/4) r²·L` within the deflection bound); offset `Vr ≈ Vin + area·d`
  (grows for `d>0`, shrinks for `d<0`); shell `0 < Vr < Vin` (`≈ Vin − |inner void|`). A wrong
  direction or out-of-tolerance magnitude DISCARDS the candidate.

## 8. NativeEngine wiring (`src/engine/native/native_engine.cpp`)

- [x] 8.1 `chamfer_edges` → if the body is not native, `fallback().chamfer_edges(...)`; else
  `nblend::chamfer_edges(...)`; NULL OR failed `blendResultVerified` ⇒ fall through (labelled,
  no interception); verified ⇒ `track(wrapNative(std::move(result)))`.
- [x] 8.2 `fillet_edges` → same native-else-(self-verify)-else-fallback shape (constant radius
  only; a curved / concave / multi-edge case DECLINES → fall through).
- [x] 8.3 `offset_face` → same shape.
- [x] 8.4 `shell` → same shape.
- [x] 8.5 `fillet_edges_variable` and `fillet_face` stay pure `CC_NATIVE_BODY_UNSUPPORTED`
  fall-throughs (no native builder call) — variable radius and face fillet are out of scope.
- [x] 8.6 Confirm OCCT is referenced only under `CYBERCAD_HAS_OCCT`; the native builder
  references no OCCT / `IEngine` / `EngineShape` type; `native_engine.h` unchanged (all four
  signatures already declared).

## 9. Gate 1 — host analytic unit tests (`tests/`, no OCCT)

- [x] 9.1 `tests/test_native_blend.cpp`: box **chamfer** of one edge by `d` → watertight
  (`boundaryEdgeCount == 0`), closed 2-manifold, EXACT volume `Vbox − ½ d²·L`, one new planar
  chamfer face.
- [x] 9.2 Box convex-edge **constant fillet** by `r` → watertight, the blend face is a
  `Cylinder` of radius `r` tangent to both faces, volume `Vbox − (1−π/4) r²·L` within a
  deflection bound.
- [x] 9.3 Box top-face **offset** outward by `d` → watertight, volume grows by EXACTLY
  `area(f)·d`; offset inward shrinks by the same.
- [x] 9.4 Box **shell** to wall `t` with the top face removed → watertight, EXACT wall volume
  `Vbox − (a−2t)(b−2t)(c−t)`.
- [x] 9.5 Self-verify rejects a deliberately open / wrong-direction-volume candidate (a unit
  test on `blendResultVerified` and/or a mis-assembled shell).
- [x] 9.6 Fall-through triggers: a **curved-face** input, a **concave** edge, a
  **variable-radius** (`cc_fillet_edges_variable`) call, a **`cc_fillet_face`** call, and a
  **multi-edge-interference** selection each yield NULL from the builder (or a pure
  fall-through) → fall through.
- [x] 9.7 `tests/test_native_engine.cpp`: facade cases for native `cc_chamfer_edges` /
  `cc_fillet_edges` / `cc_offset_face` / `cc_shell` under `cc_set_engine(1)` and a fall-through
  case (curved-face input) proving delegation.
- [x] 9.8 Host CTest all green (existing + new); `test_native_tessellate`,
  `test_native_construct`, `test_native_loft`, `test_native_sweep`, `test_native_boolean`
  unchanged.

## 10. Gate 2 — simulator native-vs-OCCT parity (`tests/sim/`)

- [x] 10.1 `tests/sim/native_blend_parity.mm` + `scripts/run-sim-native-blend.sh`: through the
  `cc_*` facade under `cc_set_engine(0/1)` (OCCT default restored in a teardown guard), compare
  `cc_chamfer_edges` / `cc_offset_face` / `cc_shell` native vs OCCT
  `BRepFilletAPI_MakeChamfer` / `BRepOffsetAPI` on box cases — volume / area / bbox /
  sub-shape counts / watertightness EXACT (relative error ~0).
- [x] 10.2 `cc_fillet_edges` (constant radius) native vs `BRepFilletAPI_MakeFillet` on a box
  convex edge — deflection-bounded (volume / bbox rel within the tessellation bound; blend
  face is a cylinder of radius `r`; watertight).
- [x] 10.3 Fall-through proof cases: a curved-face input, a concave edge, a variable-radius
  (`cc_fillet_edges_variable`) call, a `cc_fillet_face` call, and a multi-edge-interference
  selection each assert `cc_active_engine()==1` AND native == OCCT oracle (delegated, no
  interception).
- [x] 10.4 Parity harness carries its own `main()` + `std::_Exit`; ADDED to
  `run-sim-suite.sh` SKIP list (`native_blend_parity.mm` — a `.mm` already excluded by the
  `*.cpp` find, so the assertion count is unaffected). `run-sim-suite.sh` re-verified against a
  freshly rebuilt SIMULATORARM64 slice: **== 221 passed, 0 failed ==**.

## 11. Docs / spec sync

- [x] 11.1 Update `openspec/NATIVE-REWRITE.md` #6 bullet: `cc_chamfer_edges` /
  `cc_fillet_edges` (constant) / `cc_offset_face` / `cc_shell` NATIVE for the tractable planar
  slice (EXACT for chamfer / offset / box-shell, deflection-bounded for the cylindrical fillet)
  vs the OCCT oracle, guarded by the mandatory self-verify; curved / concave / variable /
  fillet_face / multi-edge / non-native fall-through labelled; both gates' numbers cited;
  honest tractable-planar framing. Also notes general blending stays OCCT.
- [x] 11.2 Update `docs/STATUS-phase-4.md` with the #6 blend result table + the
  native-vs-fallback split.
- [x] 11.3 `openspec validate add-native-fillets-offsets --strict` green; when both gates are
  green + `run-sim-suite.sh` 221/221 at the OCCT default, `openspec archive
  add-native-fillets-offsets -y` (syncs the delta into `openspec/specs/native-blends`).

## Completion note — what landed NATIVE vs OCCT-fallback (honest)

Every checkbox above is done at the bar: the four planar builders + the mandatory
self-verify guard are implemented OCCT-free, the DECLINE / fall-through paths are wired
and tested, and BOTH verification gates are green. The scope split, verified:

**NATIVE (self-verified, ran native in Gate 2):**
- `cc_chamfer_edges` — convex planar-planar edge. EXACT vs OCCT (10³ box edge chamfer,
  vol o=995 n=995 rel 2.29e-16, area rel 1.92e-16, watertight, tris=16).
- `cc_offset_face` — planar face along its normal. EXACT slab (top-face offset → vol
  1500, o=n rel 4.55e-16, area rel 1.42e-16, watertight, tris=12).
- `cc_shell` — uniform-thickness hollow of a box-like planar solid. EXACT (open-top box
  t=1 → wall vol 424, o=n rel 4.02e-16, watertight, tris=52).
- `cc_fillet_edges` (CONSTANT radius) — convex PLANAR-DIHEDRAL edge, rolling-ball tangent
  cylinder. DEFLECTION-BOUNDED vs OCCT (vol o=997.854 n=997.765 rel 8.96e-05, area rel
  1.05e-04, blend face a `Cylinder` of radius r, watertight, tris=36).

**OCCT-fallback (native builder returns NULL / self-verify discards → forwarded, verified,
never faked):**
- `cc_fillet_edges` on a CURVED-face / curved-rim edge — forwarded to OCCT
  `BRepFilletAPI_MakeFillet` (vol o=n=497.562 rel 0.00e+00; curved fallback volume-bound
  only). (Tasks 2.2 / 3.2 / 9.6 / 10.3.)
- CONCAVE edges (chamfer + fillet on an L-prism reflex edge) — builder DECLINEs → NULL.
- `cc_fillet_edges_variable` (variable radius) — pure fall-through, no native builder call
  (task 8.5).
- `cc_fillet_face` — pure fall-through, no native builder call (task 8.5).
- MULTI-EDGE interference (blends overlapping at a corner) — DECLINE → NULL.
- Edge shared by ≠ 2 faces, non-planar / non-convex shell, oversized shell thickness,
  and any non-native (OCCT-built) body — DECLINE / not-native → fall-through.

**Self-verify guard proven:** a native shell of thickness 6 on a 10³ box (wall ≥ half the
smallest span → self-intersecting inner offset) is REJECTED by `blendResultVerified`
(returns id 0, honest error — no fake watertight result). (Tasks 5.2 / 7.2 / 9.5 / 10.3.)

**Gates:** Gate 1 host `test_native_blend` (10 cases) + 5 `test_native_engine` facade cases,
host CTest **18/18** (incl. `test_native_tessellate` 13/13 watertight, unperturbed), clean
build zero warnings/errors with Homebrew clang 22.1.8, `HAS_OCCT=OFF HAS_METAL=OFF`. Gate 2
`native_blend_parity.mm` **[NBLEND] 16 passed / 0 failed**; `run-sim-suite.sh` **221/221**
(the harness is on the SKIP list — own `main()` — so the OCCT-only assertion count is
unchanged). No regressions.
