# Design — moat-m5b-healing-tail (bounded single planar-hole capping)

## Context

MOAT stage **M5** is shape-healing robustness beyond the landed slices, framed by the
roadmap as *bounded first slices with an asymptotic tail*. Two slices have landed on the
clean-room, OCCT-free `cybercad::native::heal` module: the base healer
(coincident-within-tolerance sew / vertex-unify / degenerate removal / orientation) and
the opt-in **bounded near-miss gap-bridging** pass. Both **honestly decline** everything
else to the OCCT `ShapeFix` oracle. This change picks **one** concrete declined defect
and heals it robustly under a stated bound.

**Diagnosis — the three roadmap-named tail classes, and which is reachable.**

| Declined class | Substrate in `heal/` today | Verdict |
|---|---|---|
| Pcurve reconstruction | none — the module reads only world-corner loops (`face_soup.h`) and synthesizes trivial `Line` pcurves (`assemble_shell.h`); pcurve reconstruction is a foreign trimmed-B-spline (STEP `EDGE_LOOP`) problem, M0/M4 territory | **out of reach** |
| Self-intersecting-wire repair | none — no 2D wire-parameter machinery; the repair (which crossing to cut / how to re-order) is ambiguous and is the deep `ShapeFix_Wire` moat | **out of reach as a bounded slice** |
| Missing-face `OpenShell` | **full** — `EdgePool` already tags every boundary edge (used once); `assemble_shell.h` already builds a planar face from a shared-vertex loop; `orient.h` + `self_verify.h` already close and prove the result | **reachable, bounded, minimal** |

**Chosen defect: a single missing planar face on an otherwise-sewn shell.** When a soup
sews cleanly (every corner paired within `tolerance`) but one face is simply absent, the
sew leaves a ring of boundary edges. The existing `heal_open_shell_unhealed` fixture (a
unit cube with its `+Z` face erased) is exactly this: after sewing, the four top corners
are already shared by the side faces, and the four boundary edges form ONE closed,
planar, simple square loop. Synthesizing one planar cap face from those shared nodes
closes the shell.

Why this defect, this slice: it is the healer's clearest structural decline, it is
**bounded** (a single planar simple hole, not arbitrary topology repair), it **reuses
the whole existing pipeline** (boundary-edge tally → shared-vertex loop → planar face
build → orientation flood-fill → self-verify) with no new geometry kernel, and it maps
onto a clean analytic host gate (capped-cube volume `= 1.0`) *and* a one-to-one OCCT
reference (`BRepBuilderAPI_MakeFace` on the free-boundary wire).

## Goals / Non-Goals

**Goals**
- An **opt-in** (`HealOptions.capPlanarHoles == true`) pass that synthesizes ONE cap
  face for a simple planar hole, producing a watertight + valid solid that matches an
  OCCT reference cap on watertightness / volume / topology.
- **Default-off**: `capPlanarHoles == false` ⇒ the pass is a no-op ⇒ `healShell` is
  byte-identical to the two landed slices ⇒ every host fixture (incl.
  `heal_open_shell_unhealed`) and the `run-sim-native-heal.sh` fixtures pass unchanged.
- An **honest decline**: any open shell outside the bound stays `Unhealed{OpenShell}`
  with the measured residual; the input is returned unchanged; nothing is faked.
- The **mandatory self-verify** gate is unchanged and still authoritative: a capped
  candidate that does not tessellate watertight with positive volume is discarded.

**Non-Goals (stay OCCT `ShapeFix`'s moat — the asymptotic tail)**
- Capping **more than one** hole (≥ 2 boundary loops — e.g. a box missing two opposite
  faces): declined; this slice caps exactly one.
- Capping a **non-planar** hole (a curved patch, or a box missing two *adjacent* faces
  whose boundary is a non-planar hexagon): declined — a non-planar hole would need a
  synthesized curved / multi-face patch.
- Capping a **self-intersecting** boundary loop: declined.
- Pcurve reconstruction, self-intersecting-wire repair, freeform re-approximation,
  non-manifold (3+-face-edge) repair — all remain declined exactly as today.
- Any `cc_*` ABI change; any tessellator change; any OCCT include under `src/native/**`.

## The bound (why it is safe, and provably not a fabricated closure)

A cap is legitimate only when the boundary edges bound ONE flat, simple hole that a
single planar face genuinely fills. Four layers enforce this; a hole that fails any
layer is declined honestly and the input is returned unchanged.

1. **Explicit opt-in flag.** `capPlanarHoles == false` (default) ⇒ the pass never runs
   (`heal.cpp` guards it) ⇒ the landed slices are byte-identical. There is no implicit
   hole-filling.

2. **Exactly one simple boundary cycle.** The pass builds boundary-vertex adjacency from
   the sew's boundary edges. It caps ONLY when every boundary vertex has exactly two
   incident boundary edges AND they form a **single** closed cycle. A branching boundary
   (some vertex with ≠ 2 boundary edges) or **two or more** disjoint loops (≥ 2 missing
   faces) ⇒ decline. This restricts the slice to one simple hole.

3. **Planarity within tolerance.** Every loop corner's distance to the loop's best-fit
   plane (Newell normal + centroid) must be `≤ tolerance`. A non-planar hole ⇒ decline.
   Closed-form; `maxCapPlanarityDev` records the largest deviation (honestly
   `≤ tolerance` on a cap).

4. **Simple polygon in-plane.** The loop, projected to its plane, must be
   non-self-intersecting (no two non-adjacent edges cross). A self-intersecting boundary
   ⇒ decline.

5. **Mandatory self-verify (unchanged, authoritative).** After the cap is added and the
   soup re-sewn + re-oriented, the candidate must tessellate watertight (`isWatertight`)
   with positive `enclosedVolume` across the deflection ladder. A cap that does not is
   discarded → `Unhealed{SelfVerifyFailed}`, input unchanged, defer to OCCT. Self-verify
   — not the pass's bookkeeping — is the final authority (the same discipline the
   gap-bridge slice relies on).

## Pipeline placement

`healShell` today (`heal.cpp`): extract soup → degenerate removal → `sew` → (opt-in
bridge) → **decline if boundary edges survive** → orient → assemble → self-verify. The
cap pass slots into the surviving-boundary branch, guarded by `capPlanarHoles`, BEFORE
the decline:

```
sew (+ optional bridge)
  └─ boundaryEdges > 0 ?
       ├─ capPlanarHoles ?  → capPlanarHole(soup, sewResult, tol)
       │      ├─ produced a cap FaceLoop → append to soup, re-sew
       │      │        └─ boundaryEdges now 0 ? → fall through to orient/assemble/self-verify
       │      └─ declined (multi-loop / non-planar / self-intersecting) → Unhealed{OpenShell}
       └─ else (default) → Unhealed{OpenShell | GapBeyondTolerance | GapBeyondBudget}   (UNCHANGED)
```

The cap face is emitted as a `FaceLoop` (world-corner loop + Newell normal) appended to
the working soup, so it is re-sewn by the **existing** `sew()` onto the SAME boundary
edge nodes (the ring's shared vertices are already unified), then re-oriented by the
**existing** flood-fill + global-sign tie-break. No new sew/orient/assemble code — the
cap is just one more planar face the proven machinery already handles.

## Why this maps one-to-one onto OCCT (the sim oracle)

`BRepBuilderAPI_Sewing` alone does NOT synthesize a missing face, so the sim oracle is
an **explicit OCCT reference cap**: from the free-boundary wire, build
`BRepBuilderAPI_MakeFace(gp_Pln, wire)`, add it to the shell, `ShapeFix_Shell` +
`ShapeFix_Solid`. On the in-scope fixture both native and OCCT produce a 6-face
watertight cube solid with the same enclosed volume (within tol), compared at the
`cybercad::native::heal` C++ boundary — no `cc_*` call. On the out-of-scope fixture
(non-planar / two-hole) OCCT `MakeFace` fails / `ShapeFix` leaves the shell open,
matching the native `Unhealed{OpenShell}` decline (parity of decline → engine defers to
`ShapeFix`). The **host analytic** gate needs no OCCT at all: a unit cube missing one
face, capped, has closed-form enclosed volume exactly `1.0`.

## Cognitive complexity

`capPlanarHole` stays in the module's systems band (`≤ 25`, matching `sew` /
`bridgeGaps`) by delegating to named `detail` helpers: `collectBoundaryEdges`,
`traceSingleLoop` (adjacency → one simple cycle, or fail), `bestFitPlane` +
`maxPlaneDeviation` (planarity), `isSimplePolygon` (in-plane self-intersection). The
top-level function is a short sequence of guarded early-returns.

## Alternatives considered

- **Cap multiple planar holes at once.** A trivial generalization (loop over all
  boundary cycles), but it widens the bound past "a simple `OpenShell`" and dilutes the
  honest first slice. Deferred; the ≥ 2-hole case declines cleanly today.
- **Add a new `UnhealedReason::HoleNotPlanar`.** Rejected — `OpenShell` already means
  exactly "boundary edges survive after sewing (a real hole)"; a non-planar or two-hole
  shell IS an open shell we decline. Reusing `OpenShell` keeps the change additive-minimal
  and the honesty typed.
- **Self-intersecting-wire repair / pcurve reconstruction instead.** Rejected as the
  next slice — neither has substrate in the corner-loop-only heal module (see the
  diagnosis table); both are genuinely OCCT's moat for now and are declined honestly.
