# Proposal — moat-m2asm-first-freeform-boolean (MOAT M2 assembly, subsystem B4)

## Why

The M2 substrate is complete. In the tree, consumed UNCHANGED by this change:
**B1** admits a freeform operand and hands out its faces/kinds/surfaces/trims/bounds
(`boolean/freeform_operand.h recogniseFreeformSolid`); **M0** meshes a
genuinely-trimmed freeform-walled solid watertight (`tessellate/solid_mesher.h
SolidMesher::mesh`); **M1** traces a surface∩surface seam (`ssi/marching.h WLine`);
**B2** splits ONE trimmed freeform face along that seam into two tiling sub-faces
(`boolean/face_split.h splitFace`); **B3** classifies a point IN/OUT/ON a
freeform-walled solid off its M0 boundary mesh
(`boolean/freeform_membership.h classifyPointInMesh`). Those are the verbs of a
freeform boolean — **recognise, mesh, trace, split, classify** — yet the first
end-to-end freeform boolean (the SSI-arc payoff) has never assembled.

B1's landing (commit 7ee7e2f) HONEST-DECLINED that assembly with **two measured
blockers**, both fired by a runtime witness (no dead code):

- **(i) B2 needs a straight-edged CONVEX outer loop, but a freeform WALL carries a
  smooth CLOSED (circular) trim.** The bump-capped-cylinder keystone's freeform wall
  wraps 360°, so its trim is a closed loop — B2 declined `NoOuterLoop`. Closing this
  is a **B2 smooth-trim generalisation**.
- **(ii) A half-space cut also crosses ANALYTIC cap/side faces and needs a NEW
  cross-section cap on the cut plane.** Even a polygon-trimmed wall does not close a
  boolean: the cutting plane splits the operand's planar caps/sides too, and a fresh
  face must be synthesised on the cut plane to close the solid. Neither an
  analytic-face splitter nor a cross-section-cap-synthesis weld is a landed M2 verb.
  Closing this is a **new B4 verb**.

**Diagnosis — the shortest path to ONE verified freeform boolean.** Blocker (i) is
NOT irreducible: B2 already ships, and passes on, a genuinely-trimmed Bézier "bowl"
patch bounded by a **convex straight-edged quadrilateral** cut cleanly by the plane
`x = 0` (`tests/native/face_split_fixture.h`). Wrapping that exact patch into a SOLID
— a **bowl-lidded convex-quad prism**: the bowl Bézier patch as the freeform top, four
vertical PLANAR side walls over the quad's four straight edges, and a planar bottom —
yields a watertight operand whose ONE freeform face has the convex straight-edged
outer loop B2 already splits. That **sidesteps blocker (i)** (each wall is planar
because a bowl edge over a straight UV segment is `(x,y)` linear × `z` quadratic, which
lies in the vertical plane through that segment) and defers the B2 smooth-trim
generalisation to a later wave.

Blocker (ii) is **irreducible** for any real 3-D boolean: a planar half-space cut of
ANY solid crosses that solid's analytic faces and leaves an open section that must be
capped. So the single enabler genuinely needed AND tractable for the first freeform
boolean is **B4**. With B4 landed and the bowl-lidded-prism operand sidestepping (i),
the first freeform boolean composes: a bowl-lidded quad prism **CUT** by an analytic
planar half-space.

This change lands **B4** — the analytic-face split + cross-section-cap-synthesis weld
verb — and composes the first freeform↔analytic **CUT**:
`recognise[B1] → trace[M1] → split freeform top[B2] → split crossed analytic faces +
synthesise the cross-section cap[B4] → classify survivors[B3] → weld[M0] →
mandatory self-verify`, cross-checked against `BRepAlgoAPI_Cut`. If the composition is
not robustly reachable, B4 lands proven in isolation and the assembly HONEST-DECLINES
with the next sharpened blocker. OCCT stays the oracle and the fallback; a wrong or
leaky solid is NEVER emitted.

## What Changes

1. **A new OCCT-free, header-only B4 verb** in `src/native/boolean/half_space_cut.h`
   (a strictly additive sibling — it does NOT touch B1/B2/B3/M0/M1 or the analytic
   `recogniseCurvedSolid`/`classifyPoint`):
   - **Analytic-face split.** For each analytic (planar) boundary face the cut plane
     `P` crosses, split it along its exact `Face ∩ P` line segment into a keep-side and
     a discard-side sub-face, carrying the parent's plane/pcurves verbatim; a face
     entirely on one side is kept or dropped whole. The crossing point on each face's
     boundary edge is recorded (edge id + parameter) so the section-cap boundary and
     the neighbour walls SHARE it — no T-junction.
   - **Cross-section cap synthesis.** Assemble the ordered closed loop on `P` from the
     B2 seam chord (the freeform wall's `wall ∩ P` curve, its pcurve on `P`) spliced to
     the straight crossing segments of the split analytic faces, and build ONE planar
     `Plane`-surface cap face bounded by that loop (one curved seam edge + straight
     edges), oriented outward (its normal = the cut plane's discard-facing normal).
   - **Weld.** Emit the kept freeform sub-face (B2 `faceOut`), the kept analytic
     sub-faces and whole faces, and the section cap into one shell → `Solid`, welding
     to shared vertices (the assemble.h `VertexPool` discipline) so the shell closes.
   - The verb returns a NULL `Shape` (→ OCCT) on any decline: the plane misses/does not
     cleanly cross the operand, a crossing is tangent/at-vertex, a section loop that is
     not simple/closed, or a weld that cannot close.
2. **The first freeform↔analytic-plane CUT assembler** (guarded, consuming ONLY the
   landed verbs) that recognises the operand[B1], traces `wall ∩ P`[M1], splits the
   freeform top[B2], runs B4 on the analytic faces + section cap, classifies each
   candidate survivor IN/OUT at its interior point against the pre-cut operand's M0
   mesh[B3] to confirm the keep-side selection, welds[M0], and runs the **mandatory
   watertight + closed-form-volume self-verify**, DISCARDING a bad result → OCCT.
3. **The honest-out is preserved end-to-end.** Any verb decline (B1 `nullopt`, an M1
   seam that is not one clean chord, a B2 `SplitDecline`, a B4 decline, or a B3
   `On`/`Unknown` verdict) returns a NULL `Shape` → OCCT. A wrong/leaky solid is NEVER
   emitted. The consumed B1/B2/B3/M0/M1 subsystems and the analytic
   `recogniseCurvedSolid`/`classifyPoint` paths stay BYTE-IDENTICAL. `src/native/**`
   stays OCCT-free (0 OCCT includes); the `cc_*` ABI is unchanged. No tolerance is
   weakened; no dead code. The **B2 smooth-trim (closed/circular wall) generalisation
   is explicitly DEFERRED** — the bowl-lidded-prism operand sidesteps it — and landing
   B4 + the first CUT while deferring B2-smooth-trim is the accepted outcome.

## Capabilities

### Added Requirements

- `native-booleans`: ADDS a **B4 analytic-face-split + cross-section-cap-synthesis weld
  verb** for a planar half-space cut — split each crossed analytic face along its
  `Face ∩ P` line, synthesise ONE outward-oriented section cap on `P` from the B2 seam
  chord spliced to the crossing segments, and weld survivors into a watertight `Solid`,
  or DECLINE (NULL → OCCT) with the measured blocker.
- `native-booleans`: ADDS the **first native freeform↔analytic-half-space CUT**
  (bowl-lidded convex-quad prism CUT by a planar half-space) composed from
  B1→M1→B2→B4→B3→M0 and guarded by a mandatory watertight + closed-form-volume
  self-verify that DISCARDS a bad result → OCCT — or an HONEST DECLINE with the next
  measured blocker.
- `native-booleans`: ADDS the **first-freeform-CUT parity with OCCT
  `BRepAlgoAPI_Cut`** on a booted simulator (volume/area/watertightness/topology), with
  the mandatory self-verify discarding any native result that disagrees.
- `native-booleans`: ADDS the **strictly-additive + deferral contract** — B4 and the
  first CUT preserve B1/B2/B3/M0/M1 and the analytic paths byte-identical, add no
  `cc_*` surface and no OCCT include under `src/native/**`, and explicitly DEFER the B2
  smooth-trim (closed/circular wall) generalisation.

## Impact

- `src/native/boolean/half_space_cut.h` — NEW header-only, OCCT-free B4 verb
  (analytic-face split + section-cap synthesis + weld) and the first-CUT assembler.
  Cognitive complexity kept in the backend band: the driver delegates per-face
  splitting, the section-loop assembly, and the weld to helpers, mirroring `assemble.h`.
- **Consumed UNCHANGED (proven byte-identical vs `main`):**
  `boolean/freeform_operand.h recogniseFreeformSolid` (B1), `boolean/face_split.h
  splitFace` (B2), `boolean/freeform_membership.h classifyPointInMesh` (B3),
  `tessellate/solid_mesher.h SolidMesher::mesh` (M0), `ssi/marching.h WLine` (M1), and
  the analytic `ssi_boolean.h recogniseCurvedSolid`/`classifyPoint` (S5 assembler
  byte-identical).
- **Gates.** (a) HOST ANALYTIC — no OCCT: the bowl-lidded quad prism CUT by `x = 0`
  assembles watertight and its enclosed volume equals the closed-form polynomial value
  `∫∫_{Q∩{x≤0}} (h0 + a(x²+y²)) dA` within a scale-relative tolerance; a case any verb
  declines returns NULL. (b) SIM native-vs-OCCT — the native result matches
  `BRepAlgoAPI_Cut` on volume/area/watertightness/topology, with a point batch agreeing
  with `BRepClass3d_SolidClassifier` (zero crisp IN↔OUT disagreements). OCCT is
  referenced ONLY in `src/engine/occt` / the sim harness.
- **Deferred (documented, not faked):** the **B2 smooth-trim (closed/circular freeform
  wall) generalisation** (blocker i, sidestepped by the bowl-lidded-prism operand);
  freeform↔freeform booleans; multi-freeform-face or holed operands; multi-plane /
  box cutters; COMMON and FUSE of a freeform operand; non-convex / multi-crossing
  seams (B2 declines these); branch-point / tangential seams (M1/B3 decline). No `cc_*`
  ABI change; no CyberCad app change; no OCCT under `src/native/**`.
