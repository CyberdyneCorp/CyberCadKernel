# Proposal — moat-m2cf-chamfer-face (MOAT M2 full-face chamfer weld)

## Why

The native blend library has `fillet_face` (`src/native/blend/fillet_face.h`, rounds
every convex edge bounding a picked face) but no chamfer twin. The reason is the
byte-frozen SEQUENTIAL `chamfer_edges`: it chamfers picked edges one at a time, each
clip mutating the soup, so a CORNER-SHARING edge loop declines (the first edge's set-back
removes the shared corner, so the next edge is lost — `facesOnEdgeInSoup == 0` → NULL →
OCCT). A planar face's outer edge loop is EXACTLY such a corner-sharing set: consecutive
bounding edges meet at the face's corners.

The landed M2 convex-corner weld `chamfer_corner`
(`moat-m2cc-corner-chamfer-weld`) already resolves this: it resolves every chamfer plane
UP FRONT on the original soup and applies all clips together, welding a corner-sharing
loop watertight with the corner facets synthesised from the exposed rings. Crucially, at
EVERY corner of ONE face's edge loop exactly TWO picked edges meet (the two bounding
edges incident to that corner) — never a TRIPLE (the third edge through that vertex runs
off the OTHER, unpicked faces). So a full-face chamfer is a pure set of 2-edge DIHEDRAL
corners, which `chamfer_corner` welds AND which matches OCCT `BRepFilletAPI_MakeChamfer`
to fp64 (a 2-edge dihedral corner is a union of two setback half-space prisms, which OCCT
reproduces exactly). The triple-corner oracle-gap decline `chamfer_corner` carries is
therefore UNREACHABLE from a single face's loop. This is an additive,
tessellator-free (all-planar → `assembleSolid` weld), OCCT-arbitrated slice.

## What

One additive OCCT-free header-only verb `src/native/blend/chamfer_face.h`
(`chamfer_face`) — the planar sibling of `fillet_face`. GIVEN an all-planar `Solid`, a
1-based `mapShapes(Face)` face id, and a symmetric setback `distance`, it:

1. Guards the solid all-planar (`PlanarModel`) and the picked face planar (`facePlane`).
2. Collects the CONVEX planar-dihedral bounding edges of that face — probed with the
   SAME `detail::filletArc` fit guard `fillet_face` uses (a concave / curved-neighbour /
   ≠2-face / oversized edge is silently skipped — the OCCT-owned residue).
3. Calls the byte-frozen `chamfer_corner` over those edge ids and returns its result
   (which runs the mandatory watertight + shrink self-verify and DECLINES → OCCT
   otherwise).

`chamfer_face` is a strictly ADDITIVE SIBLING: it does NOT modify `chamfer_edges`,
`chamfer_corner`, `fillet_edges`, `fillet_face`, `full_round`, the M0 tessellator, or any
landed weld path, and it consumes `blend_geom.h` (`PlanarModel` / `facePlane` /
`edgeEnds` / `facesOnEdgeInSoup`), `fillet_edges.h` (`detail::filletArc`), and
`corner_chamfer_weld.h` (`chamfer_corner`) BYTE-IDENTICAL. It adds no `cc_*` ABI surface
(there is no OCCT `chamfer_face` engine method to mirror; the verb is arbitrated at the
native-blend layer by the two gates directly). It stays OCCT-free.

**Oracle-matched scope (honest).** The chamfer is EXACT planar geometry, so on a
box/prism face the native full-face result matches OCCT `BRepFilletAPI_MakeChamfer`
(adding every edge of the face) to machine ε. A non-planar face, a non-all-planar solid,
no convex bounding edge, an oversized setback, or a self-verify miss → NULL → OCCT. No
tolerance is widened; a correct decline is a first-class outcome.

## Impact

- **Additive only.** New file `chamfer_face.h`; `chamfer_edges.h`,
  `corner_chamfer_weld.h`, `fillet_edges.h`, `fillet_face.h`, the whole
  `src/native/tessellate/**` and every M0/M1/M2 weld path are BYTE-IDENTICAL.
  `src/native/**` stays OCCT-free. No `cc_*` ABI change, no engine-glue change (the verb
  is exercised directly by the two gates).
- **Gate A (HOST ANALYTIC, no OCCT)** — `tests/native/test_native_chamfer_face.cpp`
  (5/5). On a 10³ box: `chamfer_face` welds every one of the six faces watertight at the
  EXACT inclusion-exclusion closed-form volume `V = L³ − (2d²L − 4d³/3)` (959.5 at
  d=1.5, 981.333 at d=1), swept over four setbacks, plus a non-orthogonal (60°
  triangular-prism) top face and the out-of-domain declines. Registered in CMakeLists
  (always-on native suite).
- **Gate B (SIM NATIVE-vs-OCCT)** — `tests/sim/native_chamfer_face_parity.mm`,
  `scripts/run-sim-native-chamfer-face.sh`. On the booted iOS simulator: for every cube
  face and a setback sweep, native `chamfer_face` volume == OCCT
  `BRepFilletAPI_MakeChamfer` (all face edges) == closed form to fp64, native watertight;
  plus the oversized honest decline. 11 passed / 0 failed.
