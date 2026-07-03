# Design — add-native-fillets-offsets (Phase 4 #6, native-blends)

Clean-room native fillet / chamfer / offset / shell for the **tractable planar cases**, on
the #1–#5 foundations (`src/native/math`, `src/native/topology`, `src/native/tessellate`,
`src/native/boolean`, `src/native/construct`). OCCT-free, host-buildable. OCCT
(`BRepFilletAPI_MakeFillet` / `_MakeChamfer`, `BRepOffsetAPI_MakeThickSolid` / `_MakeOffset`,
`ChFi3d`, `BRepOffset`) is a **reference ORACLE only** — consulted to confirm the tangent-
cylinder / chamfer-face / offset-slab geometry and the face-survival rules, and to compare
numerics; nothing copied.

> **HONEST SCOPE.** A general robust fillet / offset / shell over arbitrary NURBS solids is
> hard and out of reach in one pass. This change lands the **tractable-planar** slice:
> `cc_chamfer_edges` (planar chamfer on a convex planar-planar edge), `cc_fillet_edges`
> (constant-radius rolling-ball fillet on a convex planar-dihedral edge), `cc_offset_face`
> (planar face offset), and `cc_shell` (uniform-thickness hollow of a planar / box-like
> solid) — verified against the OCCT oracle (EXACT for chamfer / offset / box-shell,
> deflection-bounded for the cylindrical fillet), and DISCARDED by a mandatory self-verify
> (falling through to OCCT) whenever the native result is not a valid watertight solid with a
> sane volume for the op. Curved-face inputs, concave edges, variable-radius
> (`cc_fillet_edges_variable`), `cc_fillet_face`, multi-edge interference, and non-native
> bodies fall through to OCCT (labelled, verified, never faked). It is fully acceptable — and
> reported truthfully — if only chamfer + planar-dihedral constant fillet + planar offset
> (+ box shell) land native and everything else is OCCT-fallback.

## 1. The contract (`cc_*`)

```c
CCShapeId cc_chamfer_edges(CCShapeId body, const int *edgeIds, int edgeCount, double distance);
CCShapeId cc_fillet_edges (CCShapeId body, const int *edgeIds, int edgeCount, double radius);
CCShapeId cc_offset_face  (CCShapeId body, int faceId, double distance);
CCShapeId cc_shell        (CCShapeId body, const int *faceIds, int faceCount, double thickness);
```

- `edgeIds` / `faceIds` — 1-based sub-shape ids (as `cc_subshape_ids` enumerates).
- Result — a new watertight `Solid` body (the `cc_*` mirror of OCCT `BRepFilletAPI` /
  `BRepOffsetAPI`).

## 2. Case split (what is native vs fall-through)

```
cc_chamfer_edges / cc_fillet_edges / cc_offset_face / cc_shell
   │
   ├─ body is a native body AND the case is in the planar slice:
   │    • chamfer/fillet: every selected edge is a STRAIGHT edge between two PLANAR
   │      faces, dihedral CONVEX, no multi-edge corner interference
   │    • offset_face: the selected face is PLANAR
   │    • shell: the solid is PLANAR/box-like, retained faces planar, uniform thickness
   │      → native builder (§3) → candidate Solid
   │            │
   │            ├─ candidate PASSES self-verify (§4): closed watertight 2-manifold
   │            │  AND a sane volume for the op (fillet/chamfer reduce; offset out grows,
   │            │  in shrinks; shell reduces to a wall)
   │            │      → NATIVE result
   │            │
   │            └─ candidate FAILS self-verify → DISCARD → OCCT FALL-THROUGH (labelled)
   │
   ├─ body is NOT a native body (foreign / OCCT-built)
   │      → OCCT FALL-THROUGH (labelled)
   │
   └─ a curved face on the input, a CONCAVE edge, variable radius
      (cc_fillet_edges_variable), cc_fillet_face, or MULTI-EDGE interference
          → builder DECLINES (returns NULL Shape) → OCCT FALL-THROUGH (labelled)
```

The self-verify guard is **mandatory** and applies even to a nominally-planar case: the
engine ships a native result ONLY when it is verified valid. No unverified or wrong solid is
ever emitted.

## 3. The native builders (`src/native/blend/`)

Each builder is a small OCCT-free step over the native `topology::Shape` model. All four
share a preflight (native body, in-slice classification) then diverge.

### 3.1 Chamfer — `chamfer_edge` (`cc_chamfer_edges`)

For each selected CONVEX edge `e` between two PLANAR faces `fL`, `fR`:

- Compute the in-face perpendicular directions from the edge into each face (`dL`, `dR`),
  and the two setback lines at distance `distance` from `e` in each face.
- Build a **planar chamfer face** spanning the two setback lines (a quad along the edge), and
  cut both faces back to their setback lines.
- Two equivalent realisations (pick the more robust in implementation):
  1. **Boolean cutter** — build a planar half-space / wedge cutter that removes the material
     beyond the chamfer plane and subtract it via the native BSP-CSG boolean
     (`src/native/boolean/boolean_solid`, op = cut). The chamfer plane is the plane through
     the two setback lines.
  2. **Direct topology edit** — trim `fL`, `fR` to their setback lines, insert the new planar
     chamfer face + two triangular end caps if the edge is interior, rebuild the shell.
- EXACT vs OCCT: on a box edge the removed prism has volume `½ d² · L` (right-isosceles
  cross-section × edge length `L`), so `Vresult = Vbox − ½ d² · L`.

### 3.2 Constant fillet — `fillet_edge` (`cc_fillet_edges`, constant radius)

For each selected CONVEX edge `e` between two PLANAR faces `fL`, `fR` (a planar dihedral):

- **Rolling-ball tangent cylinder.** REUSE the Phase-3 full-round tangent-cylinder
  construction (`src/native/construct/`): the fillet is a cylinder of radius `r` whose axis
  is parallel to `e`, offset into the solid so the cylinder is tangent to `fL` and `fR` along
  two contact lines. For a dihedral half-angle, the axis lies on the interior bisector plane
  at distance `r / sin(halfAngle)` from the edge; the two tangent (contact) lines are the
  feet of the perpendiculars from the axis to each face.
- Trim `fL` and `fR` back to their contact lines, insert the **cylindrical blend face**
  (bounded by the two contact lines and the two end profiles) plus, at each end of the edge,
  the planar **setback** region needed to close the solid against the neighbouring faces.
- Deflection-bounded vs OCCT: the blend surface is analytically the same cylinder OCCT
  builds; the volume removed from a convex box edge is `(1 − π/4) r² · L` (the corner square
  minus the quarter disc, × edge length `L`), matched within the tessellation deflection
  bound.
- Multi-edge / corner interference is DECLINED in this slice (single planar-dihedral edge, or
  independent non-interfering edges only).

### 3.3 Offset — `offset_face` (`cc_offset_face`)

For a PLANAR selected face `f`:

- Translate `f` along its outward unit normal by `distance` (positive grows the solid,
  negative shrinks it), producing the moved plane and moved boundary loop.
- Re-plane `f` at the moved position; for each face adjacent to `f`, extend (or shorten) it so
  its boundary tracks the moved loop — the side faces re-loft between the old and new
  boundary rings (a prismatic slab for a box).
- Rebuild the shell and close the solid.
- EXACT vs OCCT: a planar face offset by `d` changes the volume by exactly `area(f) · d`
  (positive out / negative in), for a face whose adjacent faces are perpendicular; the design
  restricts the native path to that prismatic case and DECLINES otherwise.

### 3.4 Shell — `shell` (`cc_shell`, uniform thickness)

For a PLANAR / box-like solid with selected opening faces `F` and uniform wall `thickness`:

- Build the **inner solid** by offsetting every RETAINED face inward by `thickness` (each
  retained plane moved inward along its inward normal by `thickness`); the opening faces are
  NOT offset, so the inner solid is open at those faces (the inner void extends to the
  openings).
- Subtract the inner void from the outer solid via the native BSP-CSG boolean (offset +
  boolean), leaving a uniform-thickness wall open at the selected faces.
- EXACT vs OCCT on a box: a box `[0,a]×[0,b]×[0,c]` shelled to wall `t` with the top removed
  has volume `abc − (a−2t)(b−2t)(c−t)` (inner void); the native result matches this exactly.
- A non-planar / non-convex solid, a non-uniform thickness, or a wall thicker than half the
  smallest span (self-intersecting inner offset) is DECLINED.

Native primitives used across the four: `math` plane / line / transform, the native
`boolean_solid` planar cut (chamfer, shell), the construct tangent-cylinder (fillet), and the
`topology::Shape` builders + `uv_triangulate` polygon machinery for face rebuild.

## 4. Mandatory self-verify guard (`src/engine/native/native_engine.cpp`)

After a `nblend::*` builder returns a candidate `topology::Shape`, the engine accepts it as
native ONLY if BOTH hold (else DISCARD → fall through to OCCT):

1. **Valid watertight 2-manifold** — reuse the existing `robustlyWatertight(solid)` helper
   (closed at every deflection in the `{0.05, 0.02, 0.01, 0.005}` ladder via
   `ntess::SolidMesher` + `ntess::isWatertight`, and positive `ntess::enclosedVolume`).
2. **Sane volume for the op** — compute the candidate's native volume `Vr` and compare it to
   the input body's native volume `Vin` with the op-correct DIRECTION and (where exact)
   MAGNITUDE:
   - **chamfer** — `Vr < Vin` and `Vr ≈ Vin − ½ d²·L` (box-edge exact; general: strictly
     less, bounded by the removed-prism estimate).
   - **fillet** (convex edge) — `Vr < Vin` and `Vr ≈ Vin − (1 − π/4) r²·L` within the
     deflection bound.
   - **offset** — `Vr ≈ Vin + area(f)·d` (grows for `d>0`, shrinks for `d<0`), exact.
   - **shell** — `0 < Vr < Vin` and `Vr ≈ Vin − |inner void|`, exact on a box.
   A candidate with the WRONG direction (e.g. a fillet that grew the solid) or a wrong
   magnitude beyond tolerance is DISCARDED.

This mirrors the boolean `booleanResultVerified` self-verify pattern already in the engine,
specialised per op.

## 5. `NativeEngine` wiring

```cpp
ShapeResult NativeEngine::chamfer_edges(EngineShape body, const int* ids, int n, double d) {
    if (!isNative(body)) return fallback().chamfer_edges(body, ids, n, d);
    const auto* h = static_cast<const NativeShape*>(body.get());
    ntopo::Shape result = nblend::chamfer_edges(h->shape, ids, n, d);      // NULL ⇒ declined
    if (result.isNull() || !blendResultVerified(result, h->shape, /*op*/CHAMFER, d))
        return fallback().chamfer_edges(body, ids, n, d);                  // labelled fall-through
    return track(wrapNative(std::move(result)));
}
// fillet_edges / offset_face / shell follow the same native-else-(self-verify)-else-fallback shape.
// fillet_edges_variable and fillet_face stay pure CC_NATIVE_BODY_UNSUPPORTED fall-throughs.
```

Unlike `boolean_op` (which needs BOTH operands native), these ops act on a SINGLE body, so a
foreign body cleanly forwards to the fallback (`fallback().chamfer_edges(...)`) — no
mixed-operand error case. A NULL native result (a curved / concave / variable / face-fillet /
multi-edge DECLINE) or a failed self-verify falls through to OCCT with no interception —
identical to `cc_set_engine(0)`. OCCT appears only under `CYBERCAD_HAS_OCCT` in the fallback
wiring; the native builder references no OCCT / `IEngine` / `EngineShape` type.
(`native_engine.h` unchanged; all four signatures already declared.)

## 6. Guards (honesty — DECLINE → OCCT fall-through)

The native builder returns a **NULL `Shape`** (never a wrong solid) when:

- **Curved face** touching the operation (any adjoining `FaceSurface::kind != Plane` for the
  selected edge/face, or any curved face on a shelled solid).
- **Concave edge** — the selected edge's dihedral is reflex (the blend would add material into
  the reflex corner); DECLINE.
- **Variable radius** (`cc_fillet_edges_variable`) and **`cc_fillet_face`** — not implemented
  in this slice; the engine methods stay pure fall-throughs (no native builder call).
- **Multi-edge interference** — two or more selected edges whose blends overlap at a shared
  corner (setback interference); DECLINE (independent non-interfering edges are allowed).
- **Shell out of range** — non-planar / non-convex solid, non-uniform thickness, or a wall
  ≥ half the smallest span (self-intersecting inner offset); DECLINE.
- **Degenerate input** — empty / open operand shell, zero-length edge, zero-area face,
  non-positive radius / distance / thickness.

Even past the builder, the **self-verify** (§4) is the final backstop: any candidate that is
not a valid watertight solid with a sane volume for the op is DISCARDED.

## 7. Cognitive complexity

- Each builder is its own function (`chamfer_edge`, `fillet_edge`, `offset_face`, `shell`),
  target ≤ 15 (systems band); the tangent-cylinder solve and the in-face-perpendicular /
  dihedral-classify predicates are short irreducible geometry (~5–10) and reuse the construct
  / boolean helpers.
- The per-op dispatcher / preflight is a guard-clause chain, target ≤ 15.
- Flag any genuinely irreducible face-rebuild / setback loop as systems-band (≤ 25) per
  NATIVE-REWRITE.md; split above ~35. Verify with the `cognitive-complexity` skill.

## 8. Verification detail

- **Gate 1 (host, no OCCT):**
  - **Chamfer** — a box with one edge chamfered by `d`: watertight (`boundaryEdgeCount == 0`),
    closed 2-manifold, EXACT volume `Vbox − ½ d²·L`, one new planar chamfer face.
  - **Constant fillet** — a box convex edge filleted by `r`: watertight, the blend face is a
    Cylinder of radius `r` tangent to both faces, volume `Vbox − (1 − π/4) r²·L` within a
    deflection bound.
  - **Offset** — a box top face offset outward by `d` grows the volume by EXACTLY
    `area(f)·d`; inward shrinks by the same; watertight.
  - **Shell** — a box shelled to wall `t` with the top removed: watertight, EXACT wall volume
    `Vbox − (a−2t)(b−2t)(c−t)`.
  - **Self-verify rejects** a deliberately open / wrong-direction-volume candidate.
  - **Fall-through triggers** — a curved-face input, a concave edge, a variable-radius call,
    a `cc_fillet_face` call, and a multi-edge-interference selection each yield NULL (or a
    self-verify reject) → fall through.
- **Gate 2 (sim, OCCT oracle):** `cc_chamfer_edges` / `cc_fillet_edges` / `cc_offset_face` /
  `cc_shell` native vs `BRepFilletAPI_MakeChamfer` / `BRepFilletAPI_MakeFillet` /
  `BRepOffsetAPI` on box cases — volume / bbox / sub-shape counts / watertightness EXACT for
  chamfer / offset / box-shell (relative error ~0), deflection-bounded for the constant
  fillet; the fall-through cases (curved, concave, variable, fillet_face, multi-edge)
  asserted identical under both engines (fall-through proof, `cc_active_engine()==1`). OCCT
  default restored in teardown; the parity test carries its own `main()` (on the suite's SKIP
  list) so `run-sim-suite.sh` 221/221 is unchanged.

## References (oracle only — not copied)

- OCCT `/Users/leonardoaraujo/work/OCCT/src`: `BRepFilletAPI_MakeFillet` /
  `BRepFilletAPI_MakeChamfer`, `BRepOffsetAPI_MakeThickSolid` / `BRepOffsetAPI_MakeOffset`,
  `ChFi3d` (blend construction), `BRepOffset` (offset / shell) — the tangent-cylinder /
  chamfer-face / offset-slab geometry and the face-survival rules.
- Phase-3 full-round rolling-ball tangent-cylinder construction (`src/native/construct/`) —
  the strong reference for the constant-radius fillet's dihedral tangent cylinder.
- The native BSP-CSG boolean (`src/native/boolean/`, Naylor-Amanatides-Thibault) — the planar
  cutter for chamfer / shell.
- Computational-geometry literature — planar half-space clipping, dihedral rolling-ball
  fillet geometry, uniform-offset polyhedra; consistent with the native `src/native/math` +
  `src/native/tessellate` polygon / predicate routines.
