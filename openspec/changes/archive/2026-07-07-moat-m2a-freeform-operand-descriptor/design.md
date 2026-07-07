# Design — moat-m2a-freeform-operand-descriptor (MOAT M2, B1 + minimal assembly)

Land **B1** — the freeform operand DESCRIPTOR and its `recogniseFreeformSolid` gate —
as a strictly **additive sibling** to the analytic `recogniseCurvedSolid`, and ATTEMPT
the **minimal end-to-end** freeform↔analytic boolean by composing the four landed M2
verbs, or keep the **honest decline** with the specific gap. OCCT
(`BRepAlgoAPI_{Cut,Common}`, `BRepGProp`) is the **oracle + fallback only**; it is
NEVER removed and NEVER linked into `src/native/**`.

Clean-room from the existing native substrate (`math`, `topology`, `tessellate`,
`ssi`, `boolean`); the analytic recogniser is the structural template, not code to
edit.

## 0. What the substrate already provides (verified in source)

The four M2 verbs are present and are consumed READ-ONLY:

- **M0** `tessellate/face_mesher.h trimmedFreeformMesh` — meshes a genuinely-trimmed
  `Kind::BSpline`/`Kind::Bezier` face watertight (interior curvature sampling +
  boundary-constrained triangulation + shared-edge weld anchors). `SolidMesher::mesh`
  drives it per face.
- **M1** `ssi/marching.h WLine` — a traced surface∩surface seam; `points[i].{u1,v1}`
  is the seam directly in surface A's `(u,v)` domain. B2 reads it verbatim.
- **B2** `boolean/face_split.h splitFace(face, WLine, opts) → SplitResult` — partitions
  ONE convex trimmed freeform face along ONE clean seam chord into two tiling
  sub-faces over the SAME `FaceSurface` node, with a tile-or-decline self-verify gate
  (`SplitDecline` enum). Everything outside the clean-single-chord envelope declines.
- **B3** `boolean/freeform_membership.h classifyPointInMesh(boundary, bbox, defl, p, tol)
  → Membership {In,Out,On,Unknown}` — multi-ray odd/even parity over an M0 boundary
  mesh, with an ON-band and an UNKNOWN honest decline. Consumes `meshAabb`.

The analytic sibling B1 mirrors is `ssi_boolean.h`:

- `recogniseCurvedSolid(const Shape&) → optional<CurvedSolid>` folds a solid's faces
  into ONE analytic wall + cap half-spaces, `nullopt` for anything richer (explicitly
  `default: return nullopt` on `Kind::BSpline`/`Bezier`). `worldFrame(surf, face)`
  world-places a face's surface frame (fold surface + face location).
- `topology/explore.h Explorer`, `topology/accessors.h surfaceOf`/`pointOf` walk faces
  / vertices. `Shape::isNull`, `type()`, `orientation()` give the audit primitives.

So the walk, the world-frame fold, the AABB fold (from `freeform_membership.h meshAabb`
pattern), and the four verbs all exist. **B1 is the one missing piece: the operand
data model + admission gate that names which operand is reachable and hands the verbs
their inputs.**

## 1. The descriptor — `FreeformOperand` (`boolean/freeform_operand.h`)

A value struct, cheap to copy, holding exactly what the M2 assembly reads:

```
enum class FaceRole { Freeform, AnalyticHalfSpace };

struct OperandFace {
  topo::Shape       face;      // the topology Face (carries its trimmed EDGE_LOOP verbatim)
  topo::FaceSurface surface;   // world-placed surface (worldFrame fold), kind preserved
  FaceRole          role;      // Freeform (BSpline/Bezier wall) | AnalyticHalfSpace (Plane/…)
  math::Vec3        outwardN;  // outward normal at a reference point (orientation-resolved)
};

struct FreeformOperand {
  topo::Shape              solid;       // the operand Shape, for SolidMesher::mesh (M0) + B3
  std::vector<OperandFace> faces;       // every boundary face, tagged
  std::vector<std::size_t> freeform;    // indices into faces: the freeform walls (for B2)
  std::vector<std::size_t> analytic;    // indices into faces: the cap/half-space faces
  boolean::Aabb            bbox;        // world AABB (for B3 ON-band scale)
  bool                     watertight;  // closed 2-manifold audit result (always true if admitted)
};
```

The descriptor is **faithful round-trip**: `faces[i].surface` re-evaluated equals the
face's surface; each freeform face's trimmed outer loop is the face's own
`EDGE_LOOP`; `outwardN` respects `Orientation::Reversed`. It carries NO derived mesh
(the assembly asks M0 on demand) and NO OCCT type.

## 2. The gate — `recogniseFreeformSolid(const Shape&) → optional<FreeformOperand>`

Additive sibling to `recogniseCurvedSolid`, same shape of logic:

```
if s.isNull() || s.type() != Solid                          → nullopt   (not an operand)
single-shell audit (exactly one Shell under the Solid)       → else nullopt (multi-shell)
for each Face:
    surf = surfaceOf(face)                                   ; !surf → nullopt
    switch surf.kind:
      BSpline | Bezier:
        genuinely trimmed? (real EDGE_LOOP, not full-rectangle, no inner hole loop)
                                                             ; else nullopt (bare-periodic/holed)
        role = Freeform
      Plane [| Cylinder | Sphere | Cone]:  role = AnalyticHalfSpace
      Torus | other:                                         → nullopt
    record OperandFace{face, worldFrame(surf,face), role, outwardN(face)}
require >= 1 Freeform face                                   ; else nullopt (analytic paths own it)
watertight audit: build the edge→faceCount map; EVERY edge shared by EXACTLY two
    faces (closed 2-manifold)                                ; else nullopt (open/leaky)
bbox = fold of all vertex points
return FreeformOperand{ s, faces, freeform, analytic, bbox, watertight=true }
```

Every `nullopt` is an **honest decline with a measured blocker** (the reason is what
the caller logs before falling through to OCCT). No tolerance is weakened; the audit
uses the same edge-sharing predicate the watertight self-verify uses. Cognitive
complexity is kept in the backend band by delegating (a) per-face role classification,
(b) the edge-sharing audit, and (c) the AABB/orientation fold to free helpers —
mirroring `recogniseCurvedSolid`'s per-face-fold structure.

### 2.1 Reachability (M0-meshable) — part of admission

An operand is only *reachable* if M0 can mesh it watertight. Admission does NOT
re-implement M0; the HOST-ANALYTIC gate (§4) meshes the admitted operand with
`SolidMesher` and asserts watertightness + closed-form volume. If a freeform face is
genuinely trimmed but M0 cannot mesh it watertight, the downstream self-verify
DISCARDS the assembly → OCCT — the descriptor still round-trips, but the assembly
declines. This keeps admission cheap and honest; M0 remains the reachability arbiter.

## 3. STRETCH — the minimal freeform↔analytic assembler

`freeform_boolean_solid(a, b, op) → topo::Shape` (NULL on any decline → OCCT).
Reachable ONLY for: one operand admitted by `recogniseFreeformSolid` with EXACTLY one
freeform wall, the other a single analytic **planar** half-space; `op ∈ {Cut, Common}`.

```
FA = recogniseFreeformSolid(a); if !FA → NULL
PB = plane half-space of b (its Plane face + outward normal); if not that shape → NULL
wall = FA.freeform[0].face ; sadA = surface adapter of wall (ssi)
seam = M1 trace(sadA, plane)                       ; !seam or not one clean chord → NULL
split = B2 splitFace(wall, seam)                   ; !split.ok → NULL (measured SplitDecline)
meshB = SolidMesher::mesh(b)  (M0)                 ; for B3 classification
for frag in {split.a, split.b, other faces of A}:
    v = classifyPointInMesh(meshB, bbox(b), defl, centroid(frag), tol)   (B3)
    keep frag per op rule (Cut: A-outside-B + B-inside-A reversed; Common: both-inside)
    ; any UNKNOWN/ON at a fragment centroid → NULL (honest decline, never a guess)
result = weld surviving faces watertight            (M0 shared-edge weld / assemble.h)
SELF-VERIFY (mandatory): watertight (every edge shared by exactly 2 faces) AND
    volume == exact/closed-form within tol          ; fail → DISCARD → NULL (→ OCCT)
return result
```

Every step is a landed verb consumed unchanged. The classification is done at a
fragment's **interior** point comfortably away from the B3 ON band; a near-band or
ambiguous fragment is an honest decline (NULL → OCCT), never a guessed keep/drop.

## 4. Gates (the two non-negotiable oracles)

- **(a) HOST ANALYTIC — no OCCT.** Build, on the host with no OCCT, a native freeform
  solid whose membership/volume is KNOWN in closed form (e.g. a `Kind::BSpline` wall
  coincident with a cylinder of radius `r` capped by two planes; convex extruded
  B-spline profile). Assert: `recogniseFreeformSolid` ADMITS it; every
  `OperandFace.surface`/kind/trim round-trips; roles and `outwardN` are correct;
  `bbox` is tight; `watertight` is true. A NON-admissible operand (open boundary,
  multi-shell, holed freeform face, torus, no freeform face) DECLINES to `nullopt`
  with the measured blocker. If the minimal assembly is reached, its M0-meshed volume
  matches the closed-form set-algebra value.
- **(b) SIM native-vs-OCCT.** If the minimal freeform↔analytic assembly assembles on a
  booted simulator with OCCT linked, its volume/area/watertightness match
  `BRepAlgoAPI_{Cut,Common}` (via `BRepGProp`) within a scale-relative tolerance; a
  point-sampled agreement with `BRepClass3d_SolidClassifier` on the result has ZERO
  crisp IN↔OUT disagreements. OCCT is referenced ONLY in the `src/engine/occt`
  harness.

## 5. Honesty contract (first-class decline)

`recogniseFreeformSolid` returns `nullopt` whenever the operand is not admissible; the
minimal assembler returns a NULL `Shape` whenever any verb declines or the self-verify
fails — in both cases the engine falls through to the OCCT oracle and a wrong/leaky
result is NEVER emitted. Landing **B1 alone** (the descriptor + gate, both gates green
for the descriptor) while HONEST-DECLINING the full assembly with the specific
remaining blocker (which verb declined, the measured gap) is an accepted, expected
outcome. No fabrication, no dead code, no weakened tolerance; the analytic
`recogniseCurvedSolid`/`classifyPoint` and the M0/M1/B2/B3 subsystems stay
byte-identical; `src/native/**` stays OCCT-free; `cc_*` is unchanged.

## 6. Complexity budget

Backend band (target ≤ 15). `recogniseFreeformSolid` folds per-face like
`recogniseCurvedSolid` (~20 there, isolated + documented) and delegates the
edge-sharing audit, role classification, and AABB/orientation fold to helpers so each
stays well within band. The minimal assembler is a linear pipeline of verb calls with
early-return declines — no nesting beyond the per-fragment keep loop. Both are measured
with the cognitive-complexity skill; any genuinely-irreducible fold is documented, not
mangled.
