# Proposal — add-native-fillets-offsets

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capabilities #1–#5 landed the native math, B-rep topology,
tessellation, construction (extrude / revolve / loft / sweep / shank), and the
planar-polyhedron boolean (BSP-CSG) foundations — all OCCT-free and host-buildable, with
`NativeEngine` serving native bodies and falling through to OCCT for the rest. Capability
**#6 `native-blends`** is the sequenced next step: the fillet / chamfer / offset / shell
feature-edit family, one of the harder OCCT dependencies because a general blend must
construct blend surfaces tangent to arbitrary (often curved) faces, trim them against the
neighbourhood, and resolve corner / multi-edge setback interference with fp64 robustness —
not just build clean topology from parameters.

A general robust fillet / offset / shell over arbitrary NURBS solids is out of reach in one
pass. Doing it all at once would risk faking coverage on the genuinely hard curved-blend and
corner-interference cases. This proposal scopes the **tractable planar slice**:

- **`cc_chamfer_edges`** on a convex edge between two planar faces (a planar cutter),
- **`cc_fillet_edges`** (constant radius) on a convex planar-dihedral edge (a rolling-ball
  tangent cylinder, reusing the Phase-3 full-round construction),
- **`cc_offset_face`** of a planar face along its normal (a planar slab), and
- **`cc_shell`** (uniform thickness) of a planar / box-like solid (offset + boolean),

each verified against the OCCT oracle (EXACT for chamfer / offset / box-shell,
deflection-bounded for the cylindrical fillet), and guarded by a **mandatory self-verify**
that discards any bad native result and falls through to OCCT. Curved-face inputs, concave
edges, variable-radius (`cc_fillet_edges_variable`), `cc_fillet_face`, multi-edge
interference, and any non-native body remain honest, labelled OCCT fall-through. This is
explicitly reported as the tractable-planar slice of the blend family; general blending is
future work.

## What changes

1. **Native blend subtree** (`src/native/blend/`, OCCT-free, host-buildable). A new subtree
   with the four planar builders, each returning a `topology::Shape` (NULL ⇒ fall through).
   It includes only `src/native/math`, `src/native/topology`, `src/native/tessellate` (for
   the self-verify), `src/native/boolean` (the planar cutter used by chamfer / shell), and
   `src/native/construct` (the tangent-cylinder reference for the fillet); never OCCT.
   - **`chamfer_edge` (`cc_chamfer_edges`).** For a convex edge between two planar faces:
     build a planar cutter that slices back both faces by `distance` and subtract it (native
     BSP-CSG boolean with a planar cutter), OR do the equivalent direct topology edit —
     cutting both faces back by `distance` along their in-face perpendiculars to the edge and
     bridging them with one new planar chamfer face.
   - **`fillet_edge` (`cc_fillet_edges`, constant radius).** For a convex planar-dihedral
     edge: compute the rolling-ball tangent cylinder (radius `r`, axis parallel to the edge,
     tangent to both planar faces at the two contact lines) by REUSING the Phase-3 full-round
     tangent-cylinder construction (`src/native/construct/`); trim both faces back to their
     contact lines, insert the cylindrical blend face + two planar setback faces at the ends,
     and close the solid.
   - **`offset_face` (`cc_offset_face`).** For a planar face: translate the face along its
     outward normal by `distance`, re-plane it, and re-loft the side faces of the adjoining
     faces to the moved boundary (grow / shrink the solid by a slab).
   - **`shell` (`cc_shell`, uniform thickness).** For a planar / box-like solid: offset the
     retained faces inward by `thickness` to form an inner shell, remove the selected
     (opening) faces, and subtract the inner void from the outer solid (offset + native
     boolean), leaving a uniform-thickness wall open at the selected faces.
2. **Planar-blend geometry predicates** (`src/native/blend/`, OCCT-free). The convex /
   planar-dihedral edge classifier (both adjoining faces planar, dihedral convex, edge a
   straight segment), the in-face perpendicular-to-edge direction, the rolling-ball
   tangent-cylinder solve (reused from construct), and the planar-cutter builder. Any
   curved-face / concave / degenerate configuration ⇒ the builder DECLINES.
3. **Mandatory self-verify guard** (`src/engine/native/native_engine.cpp`, reusing the
   existing `robustlyWatertight` from `src/native/tessellate` + the boolean set-algebra
   volume pattern). After a native builder returns a candidate solid, the engine verifies it
   is (a) a closed watertight 2-manifold across the deflection ladder and (b) has a **sane
   volume for the op** — a convex-edge fillet REDUCES the solid's volume, a chamfer REDUCES,
   an outward offset GROWS (inward shrinks), and a shell REDUCES to a wall (`0 < Vwall < Vsolid`)
   — checked against the input body's own native volume within a tolerance. If EITHER check
   fails, the candidate is **DISCARDED** and the call falls through to OCCT. This mirrors the
   boolean `booleanResultVerified` self-verify pattern already in the engine.
4. **`NativeEngine` glue** (`src/engine/native/native_engine.cpp`). `chamfer_edges`,
   `fillet_edges`, `offset_face`, and `shell` — currently `CC_NATIVE_BODY_UNSUPPORTED(...)` —
   become native-else-fallback: when the body is a native body AND the case is in the planar
   slice, run the native builder + the self-verify guard; a foreign body, a NULL native
   result (a curved / concave / variable-radius / face-fillet / multi-edge-interference
   DECLINE), or a failed self-verify falls through to the fallback with no interception.
   `fillet_edges_variable` and `fillet_face` stay pure fall-throughs. OCCT stays behind
   `CYBERCAD_HAS_OCCT`; the native builder never sees OCCT. `native_engine.h` unchanged (all
   signatures already declared).

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- **Curved-face inputs** — any fillet / chamfer / offset / shell touching a cylinder /
  sphere / cone / NURBS face needs a blend/offset surface tangent to a curved face and
  curved-surface trimming (out of scope for the planar slice). Labelled, verified OCCT
  fall-through.
- **Concave edges** — a concave-edge fillet / chamfer adds material into a reflex dihedral;
  its trimming against the neighbourhood is the robustness wall. The builder DECLINES; OCCT
  fall-through.
- **`cc_fillet_edges_variable` (variable radius)** — a variable-radius blend is a
  non-cylindrical swept surface; not implemented, pure OCCT fall-through.
- **`cc_fillet_face`** — blending a whole face; not implemented, pure OCCT fall-through.
- **Multi-edge interference** — adjacent selected edges whose blends overlap at a corner
  require setback / corner-patch interference handling; the single-edge slice DECLINES a
  multi-edge selection that interferes.
- **Non-planar / non-box shell** — `cc_shell` of a non-planar or non-convex solid; OCCT
  fall-through.
- **General robust blend / offset / shell** — full blend-surface construction, self-
  intersection trimming, tolerance reconciliation, and corner setback over arbitrary solids
  remain future work.
- Every feature / query / transform / exchange op and the already-native construction /
  boolean ops — unchanged.

## Impact

- New `src/native/blend/` subtree (chamfer / fillet / offset / shell planar builders +
  planar-blend predicates) — all OCCT-free, host-buildable, added to a `native_blend.h`
  umbrella. New host CTest cases in `tests/test_native_blend.cpp` (+ facade cases in
  `tests/test_native_engine.cpp`).
- `src/engine/native/native_engine.cpp` — `chamfer_edges`, `fillet_edges`, `offset_face`,
  and `shell` change from pure fall-through to "native-else-(self-verify)-else-fallback"; the
  self-verify guard reuses `robustlyWatertight` extended with a per-op sane-volume-direction
  check. `native_engine.h` unchanged (all four signatures already declared).
- **No** `include/cybercadkernel/cc_kernel.h` signature change; **no** `src/facade/cc_kernel.cpp`
  change (the `cc_chamfer_edges` / `cc_fillet_edges` / `cc_offset_face` / `cc_shell` entry
  points already route through the active engine). The doc-comments in `cc_kernel.h` are the
  contract this change implements natively for the planar slice.
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the new native path. All existing suites stay green at the OCCT
  default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** exact-value / analytic unit
tests on the built native blend/offset B-rep + native tessellation — a box chamfer is
watertight with EXACT volume `|box| − ½ d²·L` and one new planar chamfer face; a box convex-
edge constant fillet is watertight with a cylinder blend of radius `r` tangent to both faces
and volume `|box| − (1 − π/4) r²·L` within a deflection bound; a box face offset grows/shrinks
by EXACTLY `faceArea·d`; a box shell to wall `t` (top removed) is watertight with the exact
wall volume; a mis-assembled / wrong-direction candidate is rejected by the self-verify; and
a curved-face input / concave edge / variable-radius call / fillet_face call / multi-edge
interference each cause fall-through — all with no OCCT; (b) **sim parity** through the facade
(`cc_set_engine(1)` vs default) comparing native vs OCCT `BRepFilletAPI` / `BRepOffsetAPI`
for chamfer / offset / box-shell (EXACT) and the constant fillet (deflection-bounded), and
asserting the fall-through cases (curved, concave, variable, fillet_face, multi-edge)
identical under both engines. Done only when both gates pass and every existing suite stays
green at the OCCT default. Reported honestly as the tractable-planar slice; general blending
is future work.
