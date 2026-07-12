# Tasks — nurbs-chamfer-l4

## 1. Substrate + edge + mode description
- [x] 1.1 `Substrate` (Plane / Cylinder / Cone / Freeform) with point/normal/axis/radius/halfAngle
- [x] 1.2 `EdgeStation` (point, unit tangent, outward normals of faceA/faceB) polyline for the shared edge
- [x] 1.3 `ChamferMode` (Symmetric / Asymmetric / DistanceAngle) + `resolveLegs` (distance-angle → `d·tan(α)`)

## 2. Per-face along-surface setback trace
- [x] 2.1 `faceInward` — in-face inward direction `t × n` oriented into the material by an interior hint
- [x] 2.2 `setbackPoint` — closed-form along-surface offset: PLANE straight line, CYLINDER geodesic (axial slide or `Δψ = d/R` circumferential wrap), CONE slant-generator slide
- [x] 2.3 freeform substrate → honest `UnsupportedSubstrate`

## 3. Ruled chamfer face + witnesses
- [x] 3.1 `buildChamfer` — trace both setback rails, loft the Piegl & Tiller ruled face `R(t,τ)=(1−τ)cA+τcB` into quad strips
- [x] 3.2 `planeResidual` — four-corner best-fit-plane deviation (the planar-face witness) + `planarFace` flag
- [x] 3.3 over-large guard — decline when the chord flips/collapses OR a rail sweeps backward relative to the edge (no self-intersecting face)

## 4. Public API
- [x] 4.1 `ChamferResult` / `ChamferDecline`
- [x] 4.2 `chamfer_edge_symmetric` / `chamfer_edge_asymmetric` / `chamfer_edge_distance_angle`

## 5. Tests + wiring
- [x] 5.1 planar-dihedral exactness gate (setback lines + exactly-planar face ≤1e-12, 45° bevel)
- [x] 5.2 mode-consistency gate (asym(d,d)≡sym, distance-angle≡asym ≤1e-12)
- [x] 5.3 cylinder-substrate geodesic setback gate (cap R−d, wall on-cylinder axial d ≤1e-9)
- [x] 5.4 honest-decline gate (bad args / freeform / degenerate dihedral / over-large)
- [x] 5.5 wire `test_native_chamfer_edge_nurbs` into CMake (CYBERCAD_HAS_NUMSCI block + macro def)

## 6. Invariants
- [x] 6.1 `src/native/**` OCCT-free (0 OCCT/Geom/BRep/TK refs in changed files)
- [x] 6.2 `cc_*` ABI byte-unchanged (additive only; frozen `chamfer_edges.h` untouched)
- [x] 6.3 no tolerance widened to force a pass; degenerate/over-large cases honest-decline
