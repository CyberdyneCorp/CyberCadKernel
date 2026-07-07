# Proposal — moat-m2a-freeform-operand-descriptor (MOAT M2, subsystem B1)

## Why

The three M2 **substrate** subsystems now exist in the tree and are consumed
unchanged by this change: M0 meshes a genuinely-trimmed freeform face watertight
(`tessellate/face_mesher.h trimmedFreeformMesh`), M1 traces a surface-surface
seam (`ssi/marching.h WLine`), B2 splits ONE trimmed freeform face along that seam
(`boolean/face_split.h splitFace`), and B3 classifies a point in a freeform-walled
solid off its M0 boundary mesh (`boolean/freeform_membership.h classifyPointInMesh`).
They are the four verbs of a freeform boolean — **mesh, trace, split, classify** —
but nothing yet says *which operand is admissible* to feed them, and nothing exposes,
in ONE place, the faces / kinds / surfaces / trims / bounds / orientation the M2
assembly must read to drive them.

The analytic curved boolean has exactly that missing piece for ITS domain:
`ssi_boolean.h recogniseCurvedSolid` folds a solid whose curved wall is ONE
elementary surface into a `CurvedSolid` descriptor (kind + frame + extent + cap
half-spaces) and returns `nullopt` for anything richer — a box, a multi-curved
solid, a torus, **or any `Kind::BSpline`/`Kind::Bezier` face**. That last decline is
the freeform gap: the analytic recogniser *cannot* describe a freeform operand, so
the M2 pipeline has no admission gate and no operand data model.

This change lands **B1** — the freeform operand DESCRIPTOR plus a
`recogniseFreeformSolid` gate — an **additive sibling** to `recogniseCurvedSolid`
(which stays byte-identical). B1 is the *recognise* step: it ADMITS one reachable
freeform operand (a solid whose boundary is genuinely-trimmed freeform faces plus
analytic cap/half-space faces, closed and watertight, meshable by M0) and exposes
exactly what the M2 assembly needs — faces with their kinds, surfaces and trims;
the world bounding box; validated outward orientation — or DECLINES to `nullopt`
with a measured blocker when the operand is not admissible. Landing B1 is the
primary, host-gated deliverable.

As a STRETCH — since M0/M1/B2/B3 are all present — this change also ATTEMPTS the
minimal end-to-end M2 freeform boolean for the SIMPLEST reachable case: a single
freeform-face plate CUT by an analytic planar half-space (or freeform↔analytic
COMMON), assembled purely from the landed verbs: recognise[B1] → trace the seam[M1]
→ split the freeform face[B2] → classify the fragments in/out[B3] → weld
watertight[M0] → mandatory self-verify. If it assembles and passes both gates it is
verified against `BRepAlgoAPI_{Cut,Common}`; if the assembly is not robustly
reachable it is an **HONEST DECLINE** with the specific remaining gap, and B1 (the
descriptor) still lands. OCCT remains the oracle and the fallback; a wrong or leaky
solid is NEVER emitted.

## What Changes

1. **A new OCCT-free, header-only freeform operand descriptor + recogniser** in
   `src/native/boolean/freeform_operand.h`:
   - `struct FreeformOperand` — the data model of a freeform-faced solid: its faces
     tagged by role (`freeform` `Kind::BSpline`/`Kind::Bezier` walls; `analytic`
     `Plane` — and, for a freeform↔analytic cutter, `Cylinder`/`Sphere`/`Cone` —
     half-space caps), each carrying its world-placed `FaceSurface`, its trimmed
     outer `EDGE_LOOP`, and its outward orientation; the world-space AABB; and a
     watertightness/closed-2-manifold flag. It exposes exactly the handles the M2
     assembly reads: the freeform `Face`s for B2, the analytic half-spaces, the M1
     surface adapters for the walls, the AABB for B3, and the operand `Shape` for the
     M0 `SolidMesher`.
   - `std::optional<FreeformOperand> recogniseFreeformSolid(const topo::Shape&)` —
     ADMITS one reachable freeform operand: a non-null `Solid` with a single shell,
     at least ONE genuinely-trimmed freeform face, every other face an admissible
     analytic cap/half-space, whose boundary is closed and watertight (every edge
     shared by exactly two faces) and whose faces round-trip (kind/surface/trim
     faithfully captured). Returns `nullopt` — an honest decline with a measured
     blocker — for a non-solid, an open/leaky boundary, a multi-shell operand, a
     freeform face that is NOT genuinely trimmed (bare-periodic → analytic paths own
     it), a hole/inner loop, a non-admissible surface kind, or any operand the M0
     mesher cannot mesh watertight.
2. **STRETCH — a minimal additive freeform↔analytic boolean assembler** (guarded,
   consuming ONLY the landed verbs) that, for the simplest reachable case — a single
   freeform-face operand CUT by / COMMON with one analytic planar half-space —
   recognises both operands[B1], traces the wall∩plane seam[M1], splits the freeform
   wall along it[B2], classifies each fragment IN/OUT at its interior point against
   the other operand's M0 boundary mesh[B3], selects the surviving fragments per the
   op rule, welds them watertight[M0/assemble], and runs the **mandatory watertight +
   volume self-verify**, DISCARDING a bad result → OCCT. If the assembly is not
   robustly reachable it DECLINES and the assembler returns a NULL `Shape` (→ OCCT).
3. **The honest-out is preserved end-to-end.** `recogniseFreeformSolid` returns
   `nullopt` when the operand is not admissible; any assembled solid runs the
   mandatory self-verify and a bad result is discarded → OCCT — NEVER an emitted
   wrong/leaky solid. The analytic `recogniseCurvedSolid` / `classifyPoint` paths and
   the M0/M1/B2/B3 subsystems are CONSUMED UNCHANGED (byte-identical). `src/native/**`
   stays OCCT-free (0 OCCT includes); the `cc_*` ABI is unchanged (internal
   substrate). No tolerance is weakened; no dead code; landing B1 while honestly
   declining the full assembly is an accepted, expected outcome.

## Capabilities

### Added Requirements

- `native-booleans`: ADDS a **freeform operand descriptor** (`FreeformOperand`) and a
  `recogniseFreeformSolid` admission gate — an additive sibling to
  `recogniseCurvedSolid` — that admits ONE reachable freeform-faced solid and exposes
  faces/kinds/surfaces/trims/bounds/orientation for the M2 assembly, or DECLINES to
  `nullopt` with a measured blocker.
- `native-booleans`: ADDS a **minimal end-to-end freeform↔analytic boolean assembly**
  (single freeform-face operand CUT/COMMON with a planar half-space) composed from the
  landed M0/M1/B2/B3 verbs, guarded by a mandatory watertight + volume self-verify that
  DISCARDS a bad result → OCCT, cross-checked against `BRepAlgoAPI_{Cut,Common}` on the
  simulator — or an HONEST DECLINE with the measured remaining gap.

## Impact

- `src/native/boolean/freeform_operand.h` — NEW header-only, OCCT-free descriptor +
  `recogniseFreeformSolid`; a sibling to `recogniseCurvedSolid`, which is NOT edited.
  Cognitive complexity kept in the backend band (the recogniser delegates the
  per-face role classification, the watertight-edge audit, and the AABB fold to
  helpers, mirroring the analytic recogniser's structure).
- `src/native/boolean/freeform_operand.h` (stretch) — an additive
  `freeform_boolean_solid(a, b, op)` that composes the landed verbs; returns a NULL
  `Shape` on any decline. Reachable ONLY for the freeform↔analytic-plane case; every
  existing boolean dispatch path is untouched.
- **Consumed UNCHANGED (proven byte-identical vs `main`):** `tessellate/face_mesher.h`
  `trimmedFreeformMesh` (M0), `ssi/marching.h` `WLine` (M1), `boolean/face_split.h`
  `splitFace` (B2), `boolean/freeform_membership.h` `classifyPointInMesh` (B3), and the
  analytic `ssi_boolean.h` `recogniseCurvedSolid` / `classifyPoint` (S5 assembler
  stays byte-identical).
- **Gates.** (a) HOST ANALYTIC — no OCCT: `recogniseFreeformSolid` admits a native
  freeform solid whose faces/kinds/trims round-trip and whose closed-form volume/area
  is known; a non-admissible operand declines with the measured blocker. (b) SIM
  native-vs-OCCT — if the minimal assembly assembles, its volume/area/watertightness
  match `BRepAlgoAPI_{Cut,Common}`; OCCT is referenced ONLY in `src/engine/occt`.
- **Out of scope (declines, documented not faked):** freeform↔freeform booleans;
  multi-freeform-face or holed operands; non-convex / multi-crossing seams (B2 already
  declines these); branch points / tangential seams (M1/B3 decline); the general M2
  assembly. No `cc_*` ABI change; no CyberCad app change; no OCCT under `src/native/**`.
