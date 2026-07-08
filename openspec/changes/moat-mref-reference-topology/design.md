# Design — moat-mref-reference-topology

## Scope and non-goals

M-REF lands the READ-ONLY datum / reference-geometry queries on the native B-rep.
It builds NO new geometry (no faces/edges/solids), touches NO geometry-op module
(`boolean/`, `ssi/`, `blend/`), and adds NO `cc_*` signature. It is a thin,
OCCT-free service over the already-landed topology graph + frame math.

## OCCT is the oracle; native must MATCH it exactly on the parity-gated cells

The native semantics deliberately mirror the OCCT adapters
(`occt_query.cpp`, `occt_reference_geometry.cpp`, `occt_feature.cpp`) so a native
result is comparable to OCCT on the sim gate:

- **faceAxis / refAxisFromFace** — cylinder + cone ONLY. `cc_face_axis` errors for
  a plane/sphere/torus; matching that keeps parity honest (both return the 0
  sentinel). `cc_ref_axis_from_face` reuses the `cc_face_axis` extraction
  bit-for-bit, so the native pair is identical too.
- **refAxisFromEdge** — LINE ONLY. A circular edge yields no `gp_Lin` in the OCCT
  adapter (it errors), so a native circle axis would have no oracle to verify
  against. A circular edge is therefore an HONEST DECLINE, not a fabricated axis —
  the sharpest scope call in this track.
- **refPlaneFromFace** — the NORMAL is unambiguous and is compared exactly
  (outward, flipped for a Reversed face). The ORIGIN is only "a point on the
  plane" (a datum plane is infinite): OCCT uses the face UV-midpoint, native uses
  the outer-wire vertex centroid. Both provably lie on the same plane, so the sim
  gate compares NORMAL equality + origin COPLANARITY (`(o_native − o_occt)·n ≈ 0`),
  not origin bit-equality — the only meaningful datum-plane parity.
- **tangentChain** — `|t1·t2| ≥ 0.966` (cos 15°) at a shared vertex, grown by BFS
  over the vertex→edge ancestry, exactly like `occt_query.cpp::tangentChain`.
- **outerRimChain** — a planar face is a cap only if its plane contains ALL seed
  vertices within 1.0 model unit (rejects perpendicular side walls that share one
  seed edge); each cap contributes its OUTER wire. Same rule as the OCCT adapter.

## Why offsetFaceBoundary is deliberately narrow

`BRepOffsetAPI_MakeOffset(GeomAbs_Arc)` + `GCPnts_TangentialDeflection` produces
ARC-rounded convex corners and a deflection-sampled polyline — geometry the native
kernel cannot reproduce point-for-point without re-implementing OCCT's arc-join
offset and its sampler. Rather than emit a polyline that silently differs from the
oracle, native handles ONLY the sub-case where the two provably COINCIDE: a
planar POLYGON boundary whose offset SHARPENS every corner (an inward / corner-
sharpening offset of a convex loop) — there OCCT adds no arc, so the mitered
polygon equals OCCT's wire exactly. Everything else declines and stays on OCCT:

- non-planar face / no bounded outer wire;
- any non-line outer-wire edge (an arc boundary);
- a growing convex offset (OCCT arc-rounds the corners) — detected by comparing
  the offset signed area to the original for a convex loop;
- a self-intersecting / collapsing offset (winding flip / zero area).

The sim gate compares the enclosed AREA + axis-aligned bbox of the native inward-
offset rectangle against OCCT's inward MakeOffset result (the smaller-area of the
two signs) — orientation- and point-ordering-robust, and exact for the sharp case.

## World placement

Sub-shapes carry a `Location` cumulated by the `Explorer`; leaf geometry is stored
LOCAL. Every op bakes the `Location` into the frame/points (origin via
`applyToPoint`, axes via `applyToDir`) exactly as `BRep_Tool` bakes
`TopLoc_Location`, so results are world-placed and comparable to OCCT even under a
non-identity placement. (Unlike the analysis `resolveSub`, which declines a placed
sub-shape, M-REF handles placement correctly.)

## Decline contract at the facade boundary

A native decline is a clean `Error` from the engine method; `cc_kernel.cpp` maps
that to the documented sentinel (0 / empty) AND — because the OCCT engine is still
linked in the app — the facade re-dispatches to OCCT, which either produces the
oracle value or errors identically. A native VOID (unresolvable body) is NEVER
forwarded to the OCCT adapter (whose unchecked cast would misread it). Empty
`tangentChain` / `outerRimChain` results are a VALID answer (no chain / no cap),
matching OCCT's empty set — not a decline.
