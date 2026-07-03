# add-native-fillets-offsets

Phase 4 capability **#6 — `native-blends`** (`openspec/NATIVE-REWRITE.md`). This change
introduces the FIRST native slice of the fillet / chamfer / offset / shell feature-edit
family in `NativeEngine`: the **tractable planar cases** of `cc_chamfer_edges`,
`cc_fillet_edges` (constant radius), `cc_offset_face`, and `cc_shell` — each guarded by a
**mandatory self-verify** that DISCARDS any bad native result and falls through to OCCT.

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT), and
does NOT fake any case: curved-face inputs, concave edges, variable-radius
(`cc_fillet_edges_variable`), `cc_fillet_face`, multi-edge interference, and anything the
native path cannot robustly handle fall through to OCCT — labelled, verified, never faked.
It is fully acceptable (and reported honestly) that only **chamfer + planar-dihedral
constant-radius fillet + planar offset (+ box-like shell)** land native and everything else
is OCCT-fallback.

## What a blend / offset is (the `cc_*` contract)

```c
CCShapeId cc_fillet_edges(CCShapeId body, const int *edgeIds, int edgeCount, double radius);
CCShapeId cc_fillet_edges_variable(CCShapeId body, const int *edgeIds, int edgeCount, double radius1, double radius2);
CCShapeId cc_chamfer_edges(CCShapeId body, const int *edgeIds, int edgeCount, double distance);
CCShapeId cc_shell(CCShapeId body, const int *faceIds, int faceCount, double thickness);
CCShapeId cc_offset_face(CCShapeId body, int faceId, double distance);
CCShapeId cc_fillet_face(CCShapeId body, int faceId, double radius);
```

- **`cc_chamfer_edges`** — cut back the two faces meeting at each selected edge by
  `distance` and bridge them with a new planar chamfer face (a 45°-style bevel). The `cc_*`
  mirror of OCCT `BRepFilletAPI_MakeChamfer`.
- **`cc_fillet_edges`** — replace each selected edge with a constant-radius rolling-ball
  (cylindrical) blend tangent to both adjoining faces. The `cc_*` mirror of OCCT
  `BRepFilletAPI_MakeFillet`.
- **`cc_offset_face`** — move a face along its normal by `distance`, growing (or shrinking)
  the solid by a slab. The `cc_*` mirror of OCCT `BRepOffsetAPI_MakeOffset`-style face
  offset.
- **`cc_shell`** — hollow a solid to a uniform wall of `thickness`, removing the selected
  faces (the openings). The `cc_*` mirror of OCCT `BRepOffsetAPI_MakeThickSolid`.

The general fillet / offset / shell over arbitrary curved B-reps is genuinely hard (blend
surface construction, self-intersection trimming, corner setback); this change lands the
**tractable planar slice** and verifies it against the OCCT oracle (EXACT for chamfer /
offset, deflection-bounded for the cylindrical fillet).

## Scope (#6, tractable-planar-first)

| Case | Native in this change | Falls through to OCCT (honest, labelled) |
|---|---|---|
| **`cc_chamfer_edges`** on a CONVEX edge between two PLANAR faces | YES — cut both faces back by `distance`, insert a new planar chamfer face. Expressible via the native BSP-CSG boolean with a planar cutter, or a direct topology edit. **EXACT vs OCCT.** | — |
| **`cc_fillet_edges`** (constant radius) on a CONVEX edge between two PLANAR faces (planar dihedral) | YES — a rolling-ball tangent cylinder reusing the Phase-3 full-round tangent-cylinder construction; replace the edge with a cylindrical blend tangent to both faces + two planar setback faces. **Deflection-bounded vs OCCT.** | — |
| **`cc_offset_face`** of a PLANAR face along its normal | YES — grow / shrink the solid by a planar slab (re-plane the offset face, re-loft the side faces). **EXACT vs OCCT.** | — |
| **`cc_shell`** (uniform thickness) of a PLANAR (box-like) solid, removing selected faces | YES — offset the remaining shell inward by `thickness` and subtract, leaving a uniform wall with the selected faces open. Expressible via offset + boolean. **EXACT vs OCCT on a box.** | — |
| Result that FAILS the self-verify guard (not watertight / wrong volume direction for the op) | — | YES — the engine **DISCARDS** the native result and falls through to OCCT (never emits a leaky / wrong solid) |
| **Curved-face** input (fillet / chamfer / offset / shell touching a cylinder / sphere / cone / NURBS face) | — | YES — blend surfaces over curved faces are out of scope for this slice; labelled OCCT fall-through |
| **Concave edge** fillet / chamfer (the blend adds rather than removes material; corner interference) | — | YES — concave blends and their trimming are the robustness wall; labelled OCCT fall-through |
| **`cc_fillet_edges_variable`** (variable radius), **`cc_fillet_face`**, **multi-edge interference** (adjacent selected edges whose blends overlap at a corner) | — | YES — variable-radius surface, face fillet, and corner-setback interference are out of scope; labelled OCCT fall-through |

### Why the hard cases fall through (not faked)

- **Curved-face inputs.** A fillet / chamfer / offset / shell touching a curved face needs a
  blend surface tangent to a curved face (a general offset/blend surface), curved setback
  edges, and curved-surface trimming — strictly harder than the planar-dihedral tangent
  cylinder / planar cutter. This slice does planar faces only.
- **Concave edges.** A concave-edge blend ADDS material into a reflex dihedral and its
  trimming against the neighbourhood is the classic robustness wall; the builder DECLINES
  rather than emit a wrong / self-intersecting solid.
- **Variable radius / face fillet / multi-edge interference.** A variable-radius blend is a
  non-cylindrical swept surface; `cc_fillet_face` blends a whole face; and multiple selected
  edges meeting at a corner require setback / corner-patch interference handling. All are out
  of scope for the constant-radius single-planar-dihedral slice.
- **The self-verify guard is mandatory.** Even for a nominally planar case, the native
  result is accepted ONLY if it is a closed watertight 2-manifold with a **sane volume** for
  the op (fillet reduces a convex-edge solid; chamfer reduces; offset outward grows; shell
  reduces to a wall); otherwise it is DISCARDED and the call falls through to OCCT. The
  engine NEVER ships an unverified blend/offset solid.

## Method (locked, per NATIVE-REWRITE.md)

CLEAN-ROOM from computational-geometry references (planar half-space clipping, dihedral
rolling-ball tangent-cylinder geometry, uniform-offset polyhedra) and the `cc_*` contract
(the doc-comments in `include/cybercadkernel/cc_kernel.h`), with OCCT source
(`/Users/leonardoaraujo/work/OCCT/src`: `BRepFilletAPI_MakeFillet` / `_MakeChamfer`,
`BRepOffsetAPI_MakeThickSolid` / `_MakeOffset`, `ChFi3d` / `BRepOffset`) consulted as a
**reference ORACLE only** — to confirm the tangent-cylinder / chamfer-face / offset-slab
geometry and the face-survival rules, and to compare numerics — never copied. The Phase-3
full-round rolling-ball tangent-cylinder logic (`src/native/construct/`) is the strong
reference for the constant-radius fillet's dihedral tangent cylinder.

## Architecture / OCCT boundary (unchanged from #1–#5)

- A new native builder lives under `src/native/blend/` (new subtree) and stays **OCCT-FREE
  and host-buildable** (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no
  simulator); it includes only `src/native/math` + `src/native/topology` +
  `src/native/tessellate` (for the watertight / volume self-verify) + `src/native/boolean`
  (the planar cutter for chamfer / shell) + `src/native/construct` (the tangent-cylinder
  reference), and returns a `topology::Shape` (NULL ⇒ the engine falls through).
- `src/engine/native/native_engine.cpp` — `chamfer_edges`, `fillet_edges`, `offset_face`,
  and `shell` (currently pure `CC_NATIVE_BODY_UNSUPPORTED` fall-throughs) become
  native-else-fallback: when the body is a native body AND the case is in the planar slice,
  it runs the native builder and applies the mandatory self-verify guard; a NULL native
  result (an unsupported / declined case), a foreign body, or a failed self-verify falls
  through to the held fallback engine with **no native interception**. `fillet_edges_variable`
  and `fillet_face` stay pure fall-throughs. OCCT stays behind `CYBERCAD_HAS_OCCT`; the
  native builder never sees OCCT.
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via `cc_set_engine(1)`),
  so every existing suite is unchanged unless it opts in.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT): the native blend/offset of
   planar solids + its native tessellation.
   - **Chamfer EXACT** — a box with one edge chamfered by `d` is watertight, has the exact
     volume `|box| − ½ d² · edgeLength` (the removed prism), and adds one new planar chamfer
     face.
   - **Constant fillet deflection-bounded** — a box convex edge filleted by radius `r` is
     watertight, the blend face is a cylinder of radius `r` tangent to both faces, and the
     volume equals `|box| − (1 − π/4) r² · edgeLength` within a deflection bound.
   - **Offset EXACT** — a box top face offset outward by `d` grows the volume by exactly
     `faceArea · d`; offset inward shrinks by the same.
   - **Shell EXACT** — a box shelled to wall `t` with the top face removed is a watertight
     open-topped box-with-walls whose volume equals `|box| − |inner void|`.
   - **Self-verify guard** — a deliberately open / wrong-direction-volume candidate is
     REJECTED by the guard.
   - **Fall-through triggers** — a curved-face input, a concave edge, a variable-radius call,
     a `cc_fillet_face` call, and a multi-edge-interference case each cause the builder to
     return NULL (or the guard to reject), so the engine falls through.
2. **Simulator native-vs-OCCT parity through the facade** (`cc_set_engine(1)` vs default):
   the SAME `cc_chamfer_edges` / `cc_fillet_edges` / `cc_offset_face` / `cc_shell` calls
   issued native vs OCCT, compared on mass properties / bbox / sub-shape counts / watertight
   tessellation against the OCCT `BRepFilletAPI` / `BRepOffsetAPI` oracle. Chamfer / offset /
   box-shell match the oracle **EXACTLY** (volume / bbox relative error ~0); the constant
   fillet matches within a deflection band. The fall-through cases (curved, concave,
   variable-radius, fillet_face, multi-edge interference) asserted **identical** under both
   engines (fall-through proof). Default restored in teardown; own `main()` (on the
   `run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged.

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3 suites) stays green at the
OCCT default. This is honestly reported as the **tractable-planar slice** of the blend
family; general fillet / chamfer / offset / shell (curved faces, concave edges, variable
radius, face fillet, multi-edge corner interference) stays OCCT-backed.
