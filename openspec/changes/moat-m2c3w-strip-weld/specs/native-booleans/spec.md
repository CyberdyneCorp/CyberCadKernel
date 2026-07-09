# native-booleans

## ADDED Requirements

### Requirement: Two-junction WALL SPLIT along the chain seam, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only two-junction wall split
(`src/native/boolean/strip_split.h`, `splitFaceStrip`), the ADDITIVE SIBLING of the landed
one-junction `splitFaceJunction`, that GIVEN a trimmed freeform wall `face`, the chain seam
graph's bent `chainSeam` (boundary→`J1`→`J2`→boundary), and both junctions' UV + world
positions, partitions the wall into the removed STRIP sub-face
(`A ∩ {u0 ≤ u ≤ u1, v ≥ v0}`) and the SURVIVOR sub-face, sharing the bent chord as their
bit-exact common boundary with `J1`, `J2` as EXACT shared valence-3 vertices. The verb
SHALL reuse the byte-frozen B2 `face_split.h` `detail::` primitives (`flattenOuter`,
`seamCross`, `buildSeamEdge`, `restrictEdge`, `shoelace`, `segmentsCross`) and
`jsdetail::` verbatim; the only new steps SHALL be (a) building the seam as THREE
straight-in-UV edges (E→`J1`, `J1`→`J2`, `J2`→X) so each half reflattens exactly and both
interior kinks become exact vertices, and (b) a full-ring boundary wrap for the pose where
BOTH boundary crossings land on ONE outer edge (the strip pokes through a single edge),
disambiguated by the on-edge parameter. The self-verify SHALL use the SAME strict
tolerances B2 uses (tiling-gap, degeneracy floor, simple sub-loops, bit-identical shared
seam, rebuild-reflatten), NEVER weakened.

`splitFaceStrip` SHALL return a typed DECLINE and NO partial split when: the face has no
usable outer loop (`NoOuterLoop`); the chain seam has fewer than four nodes (`EmptySeam`);
it does not cross the outer loop exactly twice (`CrossingsNot2`); an interior seam node is
outside the loop (`InteriorNodeOutside`); `J1` or `J2` is not an exact interior chord node
in order (`JunctionNotOnSeam`); a sub-loop is degenerate / non-simple
(`DegenerateSubRegion`); the seam is not the bit-identical shared boundary
(`SeamNotShared`); the area-sum identity fails (`TilingGap`); or a rebuilt sub-face's
reflattened UV area does not match its combinatorial loop within the strict tolerance
(`RebuildMismatch`). The verb SHALL be OCCT-free and introduce no `cc_*` ABI surface.

#### Scenario: The two-junction split lands where byte-frozen B2 declines (host, no OCCT)

- **WHEN** `A` is the bowl-lidded convex-quad prism, `B` the edge-straddling box
  (`x ∈ [−0.15, 0.15], y ∈ [0.0, 0.8]`), and the chain seam graph is built
- **THEN** the byte-frozen B2 `splitFace` finds the two crossings but DECLINES the bent
  chain seam with `SplitDecline::TilingGap` (its whole-vertex boundary-arc walk cannot
  separate the strip from the survivor when both crossings share one edge), WHILE
  `splitFaceStrip` returns `StripSplitDecline::Ok`: the strip sub-face UV area equals the
  closed-form `Q ∩ {u0 ≤ u ≤ u1, v ≥ v0}` projection to 1e-12, the tiling gap ≈ 0, the
  rebuild residual is machine-precision (≤ 1e-12), and `J1`, `J2` are exact ordered
  interior vertices of the shared seam chord.

### Requirement: Multi-face STRIP WELD for the three-cutting-face two-junction pose (CUT/COMMON), or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only strip weld
(`src/native/boolean/multi_face_strip_weld.h`, `multiFaceStripClip`), the ADDITIVE SIBLING
of the landed corner-clip `multiFaceCornerClip`, that GIVEN the recognised operand `A`, its
three-arc/two-junction `ChainSeamGraph`, and the `StripFaceSplit`, assembles and
self-verifies a WATERTIGHT result solid for the STRIP removal
`A ∩ {x0 ≤ x ≤ x1, y ≥ y0}` for CUT (`A − B`) and COMMON (`A ∩ B`). It SHALL clip `A`'s
analytic faces to the op keep region — the FLAT bottom quad via an exact straight-edge
notch reroute around the notch corners (the three cap bottom edges), and each CURVED-topped
side wall via the BYTE-FROZEN exact-curve-crossing `hscdetail::cutAnalyticFace` (composed
twice for the back wall the strip cuts with BOTH parallel x-planes, so the wall's crossing
points are the SAME points the seam arc uses) — and SHALL synthesise the THREE box CAP
faces (on `x = x0`, `y = y0`, `x = x1`) each carrying its shared seam-arc segment, welded
across the two junction columns. The weld SHALL be admitted ONLY if the M0 mesh is
watertight AND its enclosed volume lies in the op's consistent inclusion–exclusion bound;
ANY failure SHALL return a NULL `Shape` (→ OCCT `BRepAlgoAPI_{Cut,Common}`) with a measured
`StripWeldDecline` (`NoStraddlingBottom`, `LoopOpen`, `WeldOpen`, `NotWatertight`,
`VolumeInconsistent`). No leaky / overlapping / wrong-volume solid SHALL EVER be emitted;
no tolerance SHALL be weakened. FUSE (`A ∪ B`) is NOT yet reachable (it needs a box cutting
face notched at both junction columns) and SHALL HONESTLY return NULL → OCCT. The verb
SHALL be OCCT-free, introduce no `cc_*` ABI surface, and consume the landed
`hscdetail::`/`mfwdetail::` primitives BYTE-IDENTICAL.

#### Scenario: The strip weld lands watertight CUT/COMMON at the closed-form strip volumes (host, no OCCT)

- **WHEN** the strip weld is run for the edge-straddling box at deflection 0.01
- **THEN** `multiFaceStripClip` returns a non-null watertight result solid (Euler χ = 2)
  for CUT and COMMON, with enclosed volume matching the closed-form strip oracle
  (`V(A−B)`, `V(A∩B)`) within 2% (the curved-tessellation band), and the CUT volume error
  decreases monotonically as the deflection tightens (0.02 → 0.01 → 0.005), converging from
  above.

#### Scenario: Native CUT/COMMON match OCCT BRepAlgoAPI; FUSE honestly declines (sim, native-vs-OCCT)

- **WHEN** the SAME `A` (sewn Bézier-topped solid) and box `B` are reconstructed in OCCT
  and `BRepAlgoAPI_Cut/Common` are run as the oracle, at deflections 0.01 and 0.005
- **THEN** the native CUT/COMMON result matches OCCT on volume (rel ≤ 2e-2), area
  (rel ≤ 2e-2), watertightness, topology (Euler χ = 2), bbox and one-sided Hausdorff
  (≤ 1.5·deflection), and a 4000+-point classify batch vs `BRepClass3d_SolidClassifier`
  with ZERO crisp IN↔OUT disagreements; and `multiFaceStripClip` for FUSE returns NULL
  (→ OCCT), never a leaky solid.
