# native-booleans

## ADDED Requirements

### Requirement: Partition ONE trimmed freeform face along the M1 seam into two tiling sub-faces, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only face-split
subsystem (in `src/native/boolean/`, with an optional seam-clip helper in
`src/native/ssi/`) that partitions ONE trimmed freeform face along the M1-traced seam
into TWO genuinely-trimmed sub-faces. The subsystem SHALL take (1) a `Face` whose
`FaceSurface` is `Kind::BSpline` or `Kind::Bezier` bounded by a real outer-wire
`EDGE_LOOP`, and (2) the M1 `WLine` (`src/native/ssi/marching.h`) whose per-node
`(u1,v1)` params (surface A = this face) form the seam pcurve on the face's own
`(u,v)` domain. The subsystem SHALL:

- flatten the outer `EDGE_LOOP` to a UV boundary polygon at the SAME shared per-edge
  fractions the mesher uses (`FaceMesher::buildBoundaryLoops`), so a sub-face's
  boundary points are BIT-IDENTICAL to the parent's and to any neighbouring face's;
- clip the seam polyline to the trimmed UV domain, requiring — for this first slice —
  a CONVEX outer loop and exactly ONE seam chord that cleanly ENTERS through one
  boundary edge and EXITS through another (no tangency, no re-entry), with every
  interior seam node strictly inside the outer loop;
- partition the domain into two closed UV sub-loops: L1 = one boundary arc between the
  entry and exit crossings spliced with the interior seam chord; L2 = the
  complementary boundary arc spliced with the SAME seam chord in the opposite
  traversal order, so the seam is the EXACT shared boundary of both;
- rebuild each sub-loop as a genuinely-trimmed sub-`Face` over the SAME `FaceSurface`
  node — parent boundary edges carried verbatim (their 3D `EdgeCurve` and `PCurve`
  preserved; an edge crossed mid-span split at the crossing fraction), plus ONE new
  shared seam edge whose UV `PCurve` is the `(u1,v1)` polyline and whose 3D
  `EdgeCurve` is the `WLine`'s fitted B-spline (or the polyline), added to both wires
  with opposite orientation.

The subsystem SHALL consume the landed M0 mesher and M1 tracer WITHOUT modifying
either (the tessellator and marcher remain byte-identical), SHALL keep
`src/native/**` OCCT-free (0 OCCT includes), and SHALL NOT change any `cc_*`
signature or POD layout (additive internal substrate). For any case outside this
first slice — a non-convex outer loop, zero or two-or-more boundary crossings, a
tangential/grazing seam, seam re-entry, a branch point, or a hole in the trimmed
domain — the subsystem SHALL return a NULL split (DECLINE) with the measured blocker,
and SHALL NEVER emit a partial, overlapping, or leaky split.

#### Scenario: A single seam chord across a convex trimmed freeform face yields two sub-faces that tile the original (host, analytic — no OCCT)

- GIVEN a trimmed freeform `Face` (`Kind::BSpline`/`Bezier`, convex outer `EDGE_LOOP`) and an M1 `WLine` whose `(u1,v1)` nodes form one chord that cleanly enters and exits the trimmed `(u,v)` domain, on a host build with NO OCCT linked
- WHEN the face-split subsystem partitions the domain along the seam
- THEN it SHALL emit exactly two genuinely-trimmed sub-`Face`s whose UV areas SUM to the parent outer loop's UV area within a scale-relative tolerance, whose shared seam boundary is the IDENTICAL `(u1,v1)` node sequence traversed in opposite order (bit-identical, no gap), and whose interiors do not overlap — the two sub-faces TILE the original

#### Scenario: A seam that does not cleanly cross the trimmed domain DECLINES with a measured blocker (host)

- GIVEN a trimmed freeform `Face` and an M1 `WLine` whose seam does NOT cleanly cross the convex trimmed domain — it produces zero or three-or-more boundary crossings, grazes the boundary tangentially, re-enters the domain, or leaves a degenerate sub-region
- WHEN the face-split subsystem evaluates its self-verify gate
- THEN it SHALL return a NULL split (DECLINE), report the measured blocker (crossing count, degenerate UV area, or area-sum gap), and SHALL NOT emit any partial, overlapping, or leaky sub-face — the honest decline is a first-class outcome

### Requirement: Mandatory freeform face-split self-verify gate (tile-or-decline)

Before returning two sub-faces, the freeform face-split subsystem SHALL run a
mandatory, OCCT-free self-verify gate and SHALL accept the split ONLY when EVERY
check passes; on ANY failure it SHALL DISCARD the candidate and return a NULL split.
The gate SHALL verify: (1) the seam CLEANLY crosses — exactly one entry and one exit
boundary crossing, with all interior seam nodes strictly inside the outer loop; (2)
each sub-region is NON-DEGENERATE — a simple UV loop with no self-intersection and
|signed UV area| above a scale-relative floor; (3) the seam is the EXACT shared
boundary — L1 and L2 reference the identical seam UV node sequence in opposite order
with no boundary gap beyond snap tolerance; (4) TILING — the two sub-face UV areas
sum to the parent outer loop's UV area within a scale-relative tolerance, with no
overlap. No tolerance SHALL be weakened to admit a case, and no wrong or leaky split
SHALL ever be emitted in place of an honest decline.

#### Scenario: A candidate whose sub-face areas do not sum to the parent is discarded (host)

- GIVEN a partition candidate whose two sub-face UV areas do NOT sum to the parent outer loop's UV area within the scale-relative tolerance (a gap or overlap)
- WHEN the self-verify gate evaluates the tiling identity
- THEN the gate SHALL reject the candidate AND the subsystem SHALL return a NULL split (no sub-faces emitted), reporting the measured area-sum gap

#### Scenario: A candidate with a degenerate or self-intersecting sub-region is discarded (host)

- GIVEN a partition candidate in which one sub-loop is self-intersecting or has |signed UV area| below the scale-relative floor
- WHEN the self-verify gate evaluates the non-degenerate sub-region check
- THEN the gate SHALL reject the candidate AND the subsystem SHALL return a NULL split, reporting the degenerate sub-region as the blocker

### Requirement: Freeform sub-faces mesh watertight and reproduce the parent's area, cross-checked against OCCT (simulator)

The two sub-faces produced by the face-split subsystem SHALL, on a booted iOS
simulator with OCCT linked, each mesh WATERTIGHT through the LANDED M0
`FaceMesher::trimmedFreeformMesh` path with NO modification to the tessellator, and
their boundary points along the shared seam SHALL be BIT-IDENTICAL across the two
sub-faces (the weld contract). The union of the two sub-face meshes SHALL reproduce
the parent face's surface area and topology within tolerance, cross-checked against
the OCCT oracle: each sub-face's area SHALL match its OCCT `BRepMesh_IncrementalMesh`
area within a scale-relative tolerance, and — where an OCCT freeform face-split
reference is available — the native seam split SHALL agree with the OCCT split. OCCT
SHALL be referenced ONLY in the simulator proof harness (`src/engine/occt`); the
face-split subsystem and its inputs SHALL remain OCCT-free.

#### Scenario: Each freeform sub-face meshes watertight and the union matches the parent area vs OCCT (sim, parity)

- GIVEN the two sub-`Face`s emitted for the clean single-chord split, meshed on a booted simulator with OCCT linked
- WHEN each sub-face is meshed via the M0 `FaceMesher` and its area is compared to its OCCT `BRepMesh` area, and the two sub-face meshes are unioned along the shared seam
- THEN each sub-face mesh SHALL be watertight with shared-seam boundary points bit-identical across the two sub-faces, each sub-face's native area SHALL match its OCCT `BRepMesh` area within a scale-relative tolerance, and the unioned area SHALL reproduce the parent face's area (the tessellator is byte-identical — consumed, not modified)
