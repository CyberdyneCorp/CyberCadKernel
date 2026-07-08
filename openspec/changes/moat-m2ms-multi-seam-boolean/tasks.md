# Tasks — moat-m2ms-multi-seam-boolean (MOAT M2-multiseam)

Order: baseline capture → seam-graph builder (cutting-face set + junction + arc clip/join)
→ junction-joined B2 split → shell classifier → per-op seam-graph weld → mandatory
self-verify → host analytic gate → sim native-vs-OCCT parity gate → zero-regression proof
→ docs, OR HONEST DECLINE at the sharpest reachable level (§ design 9). All new native
code stays OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::boolean`. No `cc_*` ABI change. The change is strictly ADDITIVE: B1
`recogniseFreeformSolid`, B2 `splitFace`, B3 `classifyPointInMesh`, M0 `SolidMesher`, M1
`traceWallSeam`, the analytic `recogniseCurvedSolid`/`classifyPoint`, and the landed
single-seam `inter_solid_seam.h` / `two_operand.h` path stay BYTE-IDENTICAL; any M1 touch
is additive with prior controls byte-frozen. No tolerance is weakened; a correct decline
is a first-class outcome; no seam-graph stub.

## STATUS

DIAGNOSE (host-analytic, on the real fixture, no OCCT — grounding for reachability):
- **Pose confirmed.** `A` = `first_freeform_boolean_fixture` bowl-lidded prism; `B` = the
  corner box `x ∈ [0, 0.8]`, `y ∈ [0, 0.6]`, `z ∈ [−0.6, 0.2]`. The pole-straddle
  predicate (`planeStraddlesWall` on the bowl poles `x, y ∈ {−0.5, 0, 0.5}`) reports
  EXACTLY two cutting faces — `x = 0` and `y = 0` — and the four other faces contain `A`.
- **Junction confirmed.** The two cutting planes share the box vertical edge `x = 0,
  y = 0`, which pierces the bowl wall at `J = (0, 0, 0)`; the removed corner footprint
  `{(0.328, 0), (0.30, 0.32), (0, 0.30), (0, 0)}` has its re-entrant vertex exactly at `J`.
- **Closed-form oracle confirmed.** `V(A) = 0.196310`, `V(B) = 0.384000`,
  `V(A ∩ B) = 0.051275` (corner clip), hence `V(A − B) = 0.145035` (L-shaped) and
  `V(A ∪ B) = 0.529035` — all mesh-free, OCCT-free.
- **Key finding.** The removed region is a re-entrant CORNER, so CUT (`A − B`) is L-shaped
  and does NOT reduce to a half-space of `A`: the single-seam CUT/COMMON theorem does not
  apply, so CUT/COMMON must be genuine seam-graph welds — this is a true multi-seam slice,
  not a re-pose of the landed single-seam boolean.
- **Reduction found — then REFUTED (empirically, on the real fixture, no OCCT).** The
  proposed simplification was: "the two junction-bounded arcs concatenate into ONE
  boundary-to-boundary bent `WLine` (`J` interior), so B2 `splitFace` splits the wall into
  corner + L-survivor in a SINGLE unchanged call." MEASURED FALSE: `splitFace(bentWLine)`
  reaches `crossings == 2` with `tilingGap = 3.9e-16` (the partition is geometrically
  EXACT) but declines `RebuildMismatch` — B2's fixed global-density rebuild self-verify
  cannot resolve the interior valence-3 kink at `J` (the lone `segsPerEdge == 256` "OK" is
  a fixture-tuned coincidence, not a fix). The SEQUENTIAL split (split by full `arc0`, then
  split the survivor by `arc1`) also declines `RebuildMismatch` at ~1e-5·parentArea —
  passes only at `rebuildTolFrac 1e-4`, FAILS at the strict `1e-6` (weakening it is
  FORBIDDEN); snapping `arc1` to end exactly at `J` instead yields a touch
  (`CrossingsNot2`, crossings = 1). So the junction WELD is NOT reachable through the
  byte-frozen B2 split. The genuine new machinery is the cutting-face SET, the
  junction-vertex computation, per-arc clip/join, and — the named next enabler — a
  junction-AWARE split that introduces `J` as an EXACT shared valence-3 vertex.

VERDICT (post-implementation, WAVE 2): the SEAM-GRAPH BUILDER (§ design 9 LEVEL 3) AND
the junction-AWARE WALL SPLIT — the enabler the prior wave named — are BOTH LANDED and
proven in isolation against the closed-form corner oracle. The builder gives two cutting
faces, analytic junction `J` on both planes inside the trimmed wall
(`junctionPlaneResidual = 0`), two arcs joined at `J`, each arc individually B2-split.
The NEW verb `splitFaceJunction` (`src/native/boolean/junction_split.h`, additive sibling
of byte-frozen B2 — B2 is UNCHANGED) then partitions the wall into the corner sub-face
(`A ∩ {x≥0, y≥0}`) + the L-shaped survivor at `J` as an EXACT shared valence-3 vertex:
where byte-frozen B2 `splitFace(jointSeam)` declines `RebuildMismatch` (its fixed-density
reflatten shortcuts the sharp interior kink at `J`), the junction-aware verb builds the
seam as TWO edges meeting at `J`, so — the two arcs being ORTHOGONAL iso-parametric curves
(u-const / v-const → straight lines in UV, the only bend at `J`) — each half reflattens to
MACHINE PRECISION and PASSES the SAME strict rebuild tolerance (`rebuildTolFrac = 1e-6`,
NEVER weakened). MEASURED: `tilingGap`, `rebuildResidual` ~3e-16; corner UV area equals
the closed-form `Q ∩ {u≥½, v≥½}` projection to 7e-17; `J` bit-identical in both sub-loops.

The remaining full multi-seam WELD is DECLINED with the SHARPENED NEXT blocker: the corner
box straddles `A`'s footprint quad `Q`, so the `x=0`/`y=0` planes ALSO corner-clip `A`'s
flat BOTTOM quad and the TWO side walls over the `Q` edges they cross (MEASURED:
`footprintStraddlesBothPlanes()`), and two box CAP faces (the `x=0`/`y=0` planes bounded
inside `A`) must be synthesized, then the whole shell welded across MULTIPLE junction
vertices (`J` on the wall, `J'` on the bottom, the wall/plane pierce points) + self-verified
(watertight + closed-form volume). That MULTI-FACE corner-clip weld is the next enabler; the
sim `BRepAlgoAPI_*` parity lands WITH it. `freeformBooleanMultiSeam` therefore builds+proves
the graph, LANDS the junction-aware wall split, then returns NULL → OCCT with
`MultiFaceWeldUnreachable` — never a wrong/leaky/partial solid. A self-verified narrow
enabler + a sharpened decline are both first-class outcomes.

## 0. Substrate (FRESH worktree — run FIRST)

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
      export `CYBERCAD_NUMSCI_DIR=…/build-numsci/host` (host gate) and `…/iossim` (sim gate).
- [x] 0.2 Build host + NUMSCI and record the GREEN baseline for the native-booleans and
      native-ssi suites (single-seam two-operand FUSE/CUT/COMMON, freeform CUT/COMMON,
      face-split, freeform-membership, SSI-marching, seeding) — the byte-identical
      reference for §8.
- [x] 0.3 Snapshot the landed `freeformBooleanTwoOperand` single-seam results and the
      analytic `recogniseCurvedSolid`/`classifyPoint` outputs — the byte-identical
      reference for §8.

## 1. Seam-graph builder (`src/native/boolean/seam_graph.h`, NEW)

- [x] 1.1 Define `SeamGraph` (cutting-face indices `cutIdx`, per-plane `tracePlanes`,
      junction `J`, junction-joined `jointSeam`, per-plane straight chords) and
      `enum class SeamGraphDecline` (`NotPlanarB`, `NoOverlap`, `NotTwoCuttingFaces`,
      `NotContained`, `JunctionUnusable`, `SeamUnusable`, `JunctionNotJoined`,
      `SeamGraphNotClosed`).
- [x] 1.2 `buildSeamGraph(A, B)`: `extractPolygons`; identify the SET of cutting faces via
      `planeStraddlesWall` (reused); REQUIRE exactly two adjacent faces (DECLINE
      `NotTwoCuttingFaces`); require the other faces contain `A` via `aabbInsidePlane`.
- [x] 1.3 Compute the junction `J` = the box vertical edge (shared edge of the two cutting
      planes) pierced onto the Bézier wall by solving
      `signedDist(Pi, ·) = signedDist(Pj, ·) = 0` (bisection/Newton on the wall param,
      seeded from the straddle); DECLINE `JunctionUnusable` if not inside the trimmed wall.
- [x] 1.4 Trace each arc via `traceWallSeam` UNCHANGED (`≥ 2` pts, `Closed`/`BoundaryExit`);
      clip each full chord at `J` (drop the far side of the other cutting plane); join the
      two `boundary→J` sub-arcs into ONE bent `WLine`; verify coincidence at `J` within
      `weldTol` or DECLINE `JunctionNotJoined`.
- [x] 1.5 Keep per-function cognitive complexity in the backend band via per-arc /
      per-junction helpers; reuse `inter_solid_seam.h` `isdetail::` primitives unchanged.

## 2. Junction-joined split of BOTH operands (`src/native/boolean/multi_seam.h`, NEW)

> 2.1 LANDED via the junction-AWARE split (the named enabler): the NEW additive verb
> `splitFaceJunction` (`src/native/boolean/junction_split.h`) introduces `J` as an EXACT
> shared valence-3 vertex — two seam edges (arc0-half E→J, arc1-half J→X) meeting at `J`
> — so each straight-in-UV half reflattens to machine precision and PASSES the SAME strict
> rebuild tolerance the byte-frozen B2 `splitFace(jointSeam)` declines `RebuildMismatch`
> at. B2 is UNCHANGED; the new verb consumes B2's `detail::` primitives + the same
> `tess::buildRegion` reflatten + the same `SplitOptions`. 2.2–2.3 (splitting the OTHER
> crossed faces — bottom quad + two walls — + the shared valence-3 weld) are DEFERRED to
> the multi-face corner-clip weld, the sharpened next blocker.

- [x] 2.1 Split `A`'s Bézier wall along the junction seam into the corner + L-survivor
      sub-faces at an EXACT shared valence-3 vertex `J` — LANDED via `splitFaceJunction`
      (additive sibling of byte-frozen B2). Host gate: partition exact to ~3e-16, corner
      UV area == closed-form `Q ∩ {u≥½,v≥½}` to 7e-17, `J` bit-identical in both sub-loops.
- [ ] 2.2 Split each crossed PLANAR face of `A` and of `B` along its straight chord via
      the landed `cutAnalyticFace` UNCHANGED, exactly-two-crossings verified; DECLINE
      `SplitDecline` on tangent / at-vertex / wrong count. — DEFERRED (multi-face weld).
- [ ] 2.3 Build each seam edge (arc segment + straight chord) ONCE via `edgeFromPiece` and
      share it bit-exactly between its two incident sub-faces; build `J` ONCE as a shared
      vertex; pass uncrossed faces through whole. — DEFERRED (multi-face weld).

## 3. Seam-graph shell classifier (`multi_seam.h` classify step)

- [ ] 3.1 Mesh `A` and `B` (pre-split) once each with M0 → `meshA`, `meshB`.
- [ ] 3.2 Classify each `A` fragment centroid against `meshB` and each `B` fragment
      centroid against `meshA` via B3 `classifyPointInMesh` UNCHANGED → in/out.
- [ ] 3.3 DECLINE `ClassifyAmbiguous` on any `On`/`Unknown`/ON-band verdict — never guess.

## 4. Per-op seam-graph weld (`multi_seam.h` weld step)

- [ ] 4.1 Select survivors per op — FUSE = `A`-L-survivor + `B`-out + two corner-notched
      cutting sub-faces; CUT = `A`-L-survivor + the two cutting sub-faces reversed (cavity
      walls); COMMON = `A`-corner sub-face + the two cutting sub-faces.
- [ ] 4.2 Build each cutting face `Pk` as its box rectangle minus the corner notch
      (`arck` + the junction edge to `J`), generalising the single-seam annulus; weld into
      one shell → `Solid` via `assemble.h` `VertexPool` with `J` a single valence-3 shared
      vertex; DECLINE `WeldOpen` on an open shell.
- [ ] 4.3 Entry point `freeformBooleanMultiSeam(A, B, op, deflection, MultiSeamDecline*)`.

## 5. Mandatory self-verify → OCCT fallback

- [ ] 5.1 Require `isWatertight(SolidMesher::mesh(result))` (every edge shared by exactly
      two faces; the junction `J` is the sharpest incidence test), else DISCARD → NULL.
- [ ] 5.2 Require enclosed volume == closed-form op value (`V(A ∪ B) = 0.529035` FUSE,
      `V(A − B) = 0.145035` CUT, `V(A ∩ B) = 0.051275` COMMON) within the scale-relative
      deflection band, else DISCARD.
- [ ] 5.3 On DISCARD return NULL `Shape` recording the failing verb/measurement (→ OCCT);
      NEVER emit a partial/overlapping/leaky/wrong-volume solid; NEVER weaken a tolerance.

## 6. Host analytic gate (`tests/native/…`, NO OCCT)

- [x] 6.1 Extend the closed-form oracle: corner clip `V(A ∩ B) = ∫∫_{Q ∩ {x ≥ 0, y ≥ 0}}
      (H0 + a(x² + y²)) dA`; assert the identities `V(A ∪ B) = V(A) + V(B) − V(A ∩ B)` and
      `V(A ∩ B) + V(A − B) = V(A)` to machine precision — mesh-free, OCCT-free.
- [ ] 6.2 Assemble multi-seam FUSE/CUT/COMMON on the host with NO OCCT; assert each is a
      watertight `Solid` at its closed-form volume within the band.
- [x] 6.3 Assert the seam graph CLOSES (two arcs join at `J` to `weldTol`) and every
      fragment classifies CRISPLY (no `On`/`Unknown`) — provable even if the weld declines
      (§ design 9 level 3).

## 7. Sim native-vs-OCCT parity gate (booted simulator, OCCT linked)

> DEFERRED with §§2–5: with the multi-seam weld declined (→ OCCT), there is no native
> multi-seam solid to compare to `BRepAlgoAPI_*` yet. The parity gate lands WITH the
> junction-aware weld enabler. (The seam-graph builder + entry point already compile
> under the iossim toolchain — OCCT-free, `-fsyntax-only` clean.)

- [ ] 7.1 Build `A`, `B` BOTH natively and as OCCT shapes; compare native FUSE/CUT/COMMON
      to `BRepAlgoAPI_Fuse`/`_Cut`/`_Common` on VOLUME + AREA (`BRepGProp`), WATERTIGHT,
      TOPOLOGY counts, and a SPATIAL BBOX match (not volume-only).
- [ ] 7.2 A query-point batch agrees with `BRepClass3d_SolidClassifier` on the native
      result with ZERO crisp IN↔OUT disagreements.
- [ ] 7.3 A deliberately-perturbed non-watertight/wrong-volume native result is DISCARDED
      → OCCT; no wrong/leaky solid emitted; no tolerance weakened.

## 8. Zero-regression proof (MANDATORY — additive discipline)

- [x] 8.1 `src/native/**` has ZERO OCCT includes; `cc_*` ABI byte-identical (no new entry
      point, no POD layout change).
- [x] 8.2 B1/B2/B3/M0/M1, analytic `recogniseCurvedSolid`/`classifyPoint`, and the landed
      single-seam `inter_solid_seam.h` / `two_operand.h` path are BYTE-IDENTICAL vs the
      branch base; the native-booleans + native-ssi suites pass with counts UNCHANGED from
      the §0 baseline.
- [x] 8.3 If M1 is touched, prove the additive helper INERT on every existing SSI caller
      (byte-identical seeding/marching outputs) before it is used. — M1 was NOT touched:
      the graph builder consumes `traceWallSeam` byte-unchanged; the native-ssi
      (marching/boolean/curved-boolean/seeding) + native-booleans host suites pass
      unchanged (§8.2).

## 9. Docs / spec

- [x] 9.1 Record the reached decline level (§ design 9) and the measured next blocker in
      STATUS and in `openspec/MOAT-ROADMAP.md`.
- [x] 9.2 `openspec validate moat-m2ms-multi-seam-boolean --strict` passes.

## 10. Honest-out (a first-class outcome, not a fallback failure)

- [x] 10.1 Landed the provable subset — the seam-graph builder (§ design 9 level 3) AND
      the junction-AWARE wall split (the prior wave's named enabler, now a self-verified
      exact valence-3 partition) — and DECLINED the rest with the measured gap. The NEXT
      enabler is now the MULTI-FACE corner-clip weld (the `x=0`/`y=0` planes also cut `A`'s
      bottom quad + two walls; two box cap faces to synthesize; a multi-junction shell weld
      + sim `BRepAlgoAPI_*` parity), ahead of the general `≥ 3`-seam / branch-point graph.
- [x] 10.2 Confirm NO seam-graph stub, NO dead code, NO weakened tolerance shipped; every
      path emits a self-verified-correct solid OR a NULL `Shape` → OCCT.
