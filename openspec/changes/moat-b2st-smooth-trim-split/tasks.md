# Tasks — moat-b2st-smooth-trim-split (MOAT M2b / B2 smooth-trim)

Order: baseline capture → closed-seam fixture → `splitFaceSmoothTrim` verb (closed
interior loop → disk + annulus-hole) → mandatory self-verify → host analytic gate
(tiling + closed-form + multi-deflection mesh) → byte-freeze proof → zero-regression
proof → docs, OR HONEST DECLINE at the sharpest reachable level. All new native code
stays OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::boolean`. No `cc_*` ABI change. The change is strictly ADDITIVE: B2
`splitFace`, `splitFaceJunction`, the analytic/freeform recognisers, M0 `SolidMesher`,
M1 `traceWallSeam`, and every landed weld path stay BYTE-IDENTICAL. No tolerance is
weakened; a correct decline is a first-class outcome; no split stub.

## STATUS

LANDED (host-analytic gate, no OCCT). `splitFaceSmoothTrim` partitions a quad-trimmed
Bézier bowl sliced by the horizontal plane `z = c` (a CLOSED CIRCULAR seam, the real
S3 trace, 241 nodes, interior to the quad) into the enclosed disk + the annulus (the
seam as a hole), tiling to machine ε at the closed-form disk area, both sub-faces
meshing watertight and converging monotonically to the true curved area. Byte-frozen
B2 `splitFace` still DECLINES the same seam.

## 1. Baseline + fixture

- [x] 1.1 Capture the byte-frozen B2 baseline: `test_native_face_split` 5/5 GREEN on
  `main`; record it stays 5/5 after this change.
- [x] 1.2 Add the closed-seam fixture `tests/native/smooth_trim_split_fixture.h`:
  reuse `face_split_fixture` bowl + quad, add the horizontal cutter plane `z = c`
  (`c = a·ρ²`, ρ = 0.20) and `closedSeamWLine()` = the real
  `ssi::trace_intersection(bowl, plane z=c)` Closed loop.
- [x] 1.3 Assert the seam really is a closed circle on the surface (status `Closed`,
  every node at radius ρ from `(½,½)` within tracer tolerance).

## 2. The verb `splitFaceSmoothTrim`

- [x] 2.1 New OCCT-free header `src/native/boolean/smooth_trim_split.h`, additive
  sibling of B2 `splitFace` — consuming its `detail::` primitives (`flattenOuter`,
  `seamCross`, `shoelace`, `buildSeamEdge`, `segmentsCross`) BYTE-IDENTICAL.
- [x] 2.2 Detect the CLOSED interior loop: dedup the closing node; require ≥ 3 distinct
  nodes; ZERO seam×boundary crossings over the closed polygon; every node strictly
  inside the outer loop; a simple (non-self-intersecting) polygon.
- [x] 2.3 Build the two sub-faces: `faceInside` = seam loop as OUTER wire; `faceOutside`
  = the parent outer wire (reused verbatim) + the seam loop as a HOLE wire. Build the
  seam as one short STRAIGHT edge per polyline segment, ONCE, laid on both sub-faces
  with opposite orientation (bit-exact shared boundary).
- [x] 2.4 Keep per-function cognitive complexity within the backend band via
  `stsdetail::` helpers (`seamLoopNodes`, `loopBoundaryCrossings`, `simpleLoop`,
  `buildLoopEdges`).

## 3. Mandatory self-verify (SAME strict tolerances as B2)

- [x] 3.1 Enclosed disk + annulus both hold area above the scale-relative floor.
- [x] 3.2 Rebuild self-verify: reflatten both sub-faces; `faceOutside` has EXACTLY one
  hole; its OUTER reflattens to the parent; the seam reflattens BIT-IDENTICALLY as the
  disk's outer and the annulus's hole; the tiling residual
  `|parent − (disk + (annulusOuter − hole))|` is machine ε. Any failure → typed
  DECLINE, no partial split, no weakened tolerance.

## 4. Host analytic gate (a)

- [x] 4.1 `tests/native/test_native_smooth_trim_split.cpp`: seam-is-closed-circle;
  B2-still-declines contrast; tiling to machine ε + closed-form `π·ρ²`; sub-faces mesh
  and TILE at deflections {0.02, 0.01, 0.005} converging monotonically to the true
  curved area; disk meshes to the closed-form circle area.
- [x] 4.2 Honest-decline battery: open chord (`SeamNotInterior`), self-intersecting
  loop (`SelfIntersecting`), too-short seam (`SeamTooShort`).
- [x] 4.3 Register the suite under `CYBERCAD_HAS_NUMSCI` (its seam is the real S3 trace).

## 5. Sim gate (b) — honest scope

- [x] 5.1 Record the OCCT-oracle limitation: a closed interior trim has no clean
  `BRepAlgoAPI` face-split oracle (it needs a splitter / section-wire reconstruction).
  The closed-form partition grounds Gate A; sim parity lands with the downstream weld
  verb that consumes `splitFaceSmoothTrim`.

## 6. Zero-regression + byte-freeze proof

- [x] 6.1 `git diff src/native` = only the NEW `smooth_trim_split.h` (0 existing files
  touched); 0 OCCT includes; no `cc_*` change.
- [x] 6.2 B2 `splitFace` (`test_native_face_split` 5/5), `junction_split.h`, the M0
  tessellator, M1, and every landed weld path BYTE-IDENTICAL; full host `ctest` GREEN.
- [x] 6.3 New header `-fsyntax-only` clean under the iossim toolchain.

## 7. Docs

- [x] 7.1 Update `openspec/MOAT-ROADMAP.md` M2 status (B2 smooth-trim LANDED, with the
  sharpened next blocker: the downstream curved-wall half-space weld that consumes the
  smooth-trim split + its sim `BRepAlgoAPI` parity gate).
