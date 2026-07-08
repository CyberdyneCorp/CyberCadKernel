# native-booleans

## ADDED Requirements

### Requirement: Closed / circular smooth-trim face split into an enclosed disk and an annulus, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only smooth-trim face-split
verb (`src/native/boolean/smooth_trim_split.h`, `splitFaceSmoothTrim`) that, GIVEN a
genuinely-trimmed freeform or analytic `Face` and a seam WLine whose `(u1,v1)` polyline is
a CLOSED SMOOTH curve INTERIOR to the face's outer loop (a horizontal plane slicing a
bowl/dome cap, or a circle wrapping a curved wall — the case byte-frozen B2 `splitFace`
DECLINES with `CrossingsNot2`, `crossings == 0`), partitions the face into TWO
genuinely-trimmed sub-faces over the SAME surface: `faceInside`, the disk the seam
encloses (its OUTER wire is the seam loop); and `faceOutside`, the parent minus that disk
(the parent's outer wire plus the seam loop as a single HOLE wire). The seam loop SHALL be
built as one short STRAIGHT edge per traced polyline segment, each edge constructed ONCE
and laid onto both sub-faces with OPPOSITE orientation so the seam is their BIT-EXACT
shared boundary. The verb SHALL be a strictly ADDITIVE SIBLING of B2 `splitFace`: it SHALL
NOT modify `splitFace`, `splitFaceJunction`, the M0 tessellator, M1, or any landed weld
path, and it SHALL consume B2's `detail::` primitives (`flattenOuter`, `seamCross`,
`shoelace`, `buildSeamEdge`, `segmentsCross`) BYTE-IDENTICAL.

`splitFaceSmoothTrim` SHALL run a mandatory self-verify using the SAME `SplitOptions`
tolerances B2 uses, NEVER weakened, and SHALL return a typed DECLINE — carrying the
measured blocker — and NO sub-faces when: the face has no usable outer loop
(`NoOuterLoop`); the seam has fewer than three distinct nodes (`SeamTooShort`); the seam
does not close on itself (`SeamNotClosed`); the seam crosses or touches the boundary, or a
node leaves the face (`SeamNotInterior`); the closed loop is not a simple polygon
(`SelfIntersecting`); the enclosed disk or the annulus is below the scale-relative area
floor (`DegenerateSubRegion`); the reflattened tiling residual exceeds the strict
tolerance (`TilingGap`); or a rebuilt sub-face does not reflatten to its combinatorial
partition — exactly one hole on `faceOutside`, its outer reflattening to the parent, and
the seam reflattening bit-identically as the disk's outer and the annulus's hole
(`RebuildMismatch`). The verb SHALL remain OCCT-free, SHALL introduce no `cc_*` ABI
surface, and SHALL keep its per-function cognitive complexity within the backend band.

#### Scenario: A closed circular seam partitions a bowl cap into a disk and an annulus that tile it (host, no OCCT)

- GIVEN a quad-trimmed Bézier bowl face and the real S3 seam WLine of the bowl intersected with a horizontal plane `z = c` — a CLOSED CIRCLE of radius `ρ = √(c/a)` interior to the quad, built on the host with NO OCCT
- WHEN `splitFaceSmoothTrim(face, seam)` runs
- THEN it SHALL return `faceInside` (the disk) and `faceOutside` (the parent with the seam as a hole) whose UV areas SUM to the parent's to machine precision, with `areaInside` equal to the closed-form disk area `π·ρ²` within the inscribed-polygon band, and the seam SHALL be the bit-identical shared boundary of both sub-faces

#### Scenario: Both sub-faces mesh via M0 and their meshed areas converge to the parent's curved area (host, no OCCT)

- GIVEN the disk and annulus sub-faces from the closed-circular split, built on the host with NO OCCT
- WHEN each is meshed by the M0 `FaceMesher` at multiple deflections
- THEN each SHALL produce a non-empty mesh and their meshed surface areas SHALL SUM to the parent's true curved surface area, converging MONOTONICALLY as the deflection tightens, with NO tessellator change

#### Scenario: Byte-frozen B2 splitFace still declines the same closed seam (host, no OCCT)

- GIVEN the same closed circular seam and bowl face, built on the host with NO OCCT
- WHEN B2 `splitFace(face, seam)` runs
- THEN it SHALL DECLINE with `CrossingsNot2` and a measured `crossings == 0`, proving the smooth-trim generalisation is strictly additive and the convex straight-edged path is unchanged

#### Scenario: A seam outside the closed-interior-loop envelope DECLINES with a measured blocker (host)

- GIVEN a seam that is an OPEN chord crossing the boundary, a SELF-INTERSECTING closed loop, or a too-short (< 3 node) polyline, built on the host with NO OCCT
- WHEN `splitFaceSmoothTrim(face, seam)` runs
- THEN it SHALL return a typed DECLINE (`SeamNotInterior` / `SelfIntersecting` / `SeamTooShort`) identifying the blocker, SHALL emit NO sub-faces, and SHALL NOT weaken any tolerance to force a split
