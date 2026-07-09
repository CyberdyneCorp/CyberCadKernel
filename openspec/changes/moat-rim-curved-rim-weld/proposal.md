# Proposal — moat-rim-curved-rim-weld (MOAT M0 tessellator weld for the outer curved RIM)

## Why

The just-landed `moat-m0w-closed-seam-weld` fixed the closed INNER seam (disk ∪ annulus)
of the curved-wall boolean, and isolated the SEPARATE blocker that still made curved-wall
COMMON (`KeepSide::Above`) honestly decline at fine deflections: the bowl's OUTER curved
RIM welded to the flat top lid. That rim is a genuinely CURVED shared edge (a per-segment
degree-2 Bézier arc — the bowl edge over a straight UV chord), NOT the straight seam chord
M0w pinned, so the M0w seam-chord pin does not apply to it.

ROOT CAUSE (measured): the bowl annulus (`faceOutside`, free-form) and the flat top lid
(analytic `Plane`, kept whole) SHARE the bowl's outer rim, carried on SEPARATE edge nodes
(the bowl node with a `Line` pcurve, the lid node with a degree-2 B-spline pcurve). The
solid mesher's twist pre-pass (`requireEdgeSegments`) subdivides that rim to the SAME
shared fraction list for BOTH faces, but the two faces place the interior samples at
DIFFERENT 3-D points:

- the bowl reproduces the rim curve exactly — `S_bowl(pcurve(f)) = C_edge(f)` (the pinned
  canonical rim `d.points`), so the bowl rim samples dip below the plane onto the true arc;
- the flat lid's planar pcurve stays IN the plane — `S_lid(pcurve(f)) ≠ C_edge(f)` — so the
  lid rim samples stay flat, diverging from the shared rim curve by up to ~6e-4 (far beyond
  the mesher's `kSnapEps = 1e-6` anchor-snap radius).

The subdivided rim therefore OPENS (hundreds of boundary edges used once), AND a
coarse-regime near-degenerate coincident triangulation sliver survives at some deflections
(a rim edge used by FOUR triangles — two real + two coincident slivers, one from each
face's near-collinear boundary triangulation → non-manifold). COMMON consequently declined
to NULL → OCCT at the fine end, asserted by the old
`curved_wall_common_rim_weld_fragility_is_measured_decline` test.

## What

The CURVED-edge analogue of the M0w seam-chord pin, in the byte-frozen M0 tessellator
(`src/native/tessellate/{edge_mesher,face_mesher,solid_mesher}.h`), OCCT-free and
topology-guarded, made a VERIFIED, FALLBACK-ONLY repair so the byte-identity guarantee is
structural:

0. **Verified, fallback-only pin.** `SolidMesher::mesh` SHALL first mesh the shape with the
   curved-rim pin DISABLED — exactly the pre-change tessellator — and return that BASELINE
   result when it welds watertight (every existing mesh does → byte-identical). The curved-rim
   pin SHALL be engaged ONLY on a FALLBACK pass taken when the baseline is NOT watertight (the
   bowl↔lid rim), and the pinned result SHALL be used only when it is now watertight (else the
   honest non-watertight baseline is returned → self-verify declines → OCCT). A pinned mesh can
   thus replace a baseline mesh ONLY when the baseline was non-watertight, so no
   already-watertight mesh is ever changed.

1. **Curved-rim pin.** `face_mesher.h`'s `recordSeamChordPins` (already the pinning point
   for the straight seam chord) SHALL additionally recognise a genuinely-CURVED SHARED RIM
   edge — a degree-≥2 free-form (Bézier / B-spline) arc, `edge_mesher.h`
   `isCurvedSharedRim` — and PIN the boundary samples that genuinely DIVERGE from the edge's
   ONE canonical discretization (`d.points == C_edge`) EXACTLY to those canonical points by
   UV correspondence. On the freeform↔analytic rim the bowl side reproduces `C_edge`
   (records nothing) and the flat lid side (which diverges) is pinned to the shared rim, so
   the two boundaries become bit-identical and the rim welds watertight at any deflection.

2. **Coincident-sliver drop.** `solid_mesher.h`'s spatial weld SHALL drop every copy of a
   triangle whose merged vertex triple occurs more than once (a coincident-duplicate pair on
   the same three welded vertices — the coarse-regime sliver), then compact any vertex the
   drop orphaned so the mesh stays a single closed 2-manifold with the clean Euler
   characteristic `χ = 2`. A watertight shell never legitimately carries two triangles on one
   vertex triple, so this is a pure DEFECT repair with zero area / volume change.

Both are DIVERGENCE- and TOPOLOGY-gated so they fire ONLY on the freeform↔analytic curved
rim, never a primitive, never the straight seam chord, never a curved edge shared through
one node. `isCurvedSharedRim` excludes analytic `Circle`/`Ellipse` seams BY KIND (a
cylinder cap↔side circle, a sphere / cone / revolve latitude, a torus rim), so every
analytic primitive is ineligible; the pin is additionally gated on
`‖S_face(pcurve) − C_edge‖ > kSnapEps`, which no primitive ever trips (its shared curved
edge satisfies the weld contract to fp round-off). The sliver drop and orphan compaction
are gated on a coincident-duplicate / orphan EXISTING, which no existing mesh contains.

With the rim pinned and the sliver removed, curved-wall COMMON welds WATERTIGHT (`χ = 2`)
at EVERY deflection of the full ladder and converges to the closed-form COMMON volume
`V(z≥c) = V(full) − V(z≤c)`. The old asserted-decline test is flipped to the positive
`curved_wall_common_rim_weld_watertight_across_full_ladder`. Honest decline is preserved: a
rim that STILL cannot weld returns non-watertight → NULL → OCCT, never a leaky solid.

## Impact

- **Additive, topology-guarded, byte-identical.** The curved-rim pin, the coincident-sliver
  drop, and the orphan compaction fire ONLY on the freeform↔analytic curved rim / its
  degenerate sliver; the twist pre-pass, the boundary flattener, the segment-count sizing,
  the curve evaluators, the three face-mesh arms, and every other surface kind's path are
  BYTE-IDENTICAL. `curved_wall_cut.h`, `smooth_trim_split.h`, `half_space_cut.h`, and the
  whole boolean substrate are unchanged. `src/native/**` stays OCCT-free; the `cc_*` ABI is
  additive-only (no signature or POD layout change — `git diff include/` empty).
- **Acceptance = a byte-identical battery.** A FNV hash over `{vertexCount, triangleCount,
  vertices, triangles, watertight, area, volume}` is IDENTICAL before vs after for EVERY
  existing surface kind — `Plane`, `Cylinder`, `Cone`, `Sphere`, `Bezier`, `BSpline`,
  curved seams, box / triangle-prism / cylinder / cone / sphere-revolve / thread / sweep /
  loft (straight, frustum, TWISTED ruled) / twisted-sweep / mid-wall operand /
  first-freeform operand — 14 solid kinds × 8 deflections = 112 hashes, of which 0 change.
  The ONLY meshes that change are the 5 previously-failing bowl-cup rim cases
  (non-watertight → watertight).
- **Gate A (HOST ANALYTIC, no OCCT)** — curved-wall COMMON welds watertight (`χ = 2`) and
  converges to the closed-form `V(z≥c)` across the FULL ladder `{0.012, 0.0102, 0.008,
  0.006, 0.004, 0.002, 0.001}` (rel 2.0% → 0.6%), flipping the fragility decline test to a
  pass; CUT and the closed-seam and mid-wall gates are unchanged. Full host suite `ctest`:
  all pass.
- **Gate B (SIM native-vs-OCCT parity, booted iOS simulator)** — the curved-wall CUT/COMMON
  parity harness matches OCCT `BRepAlgoAPI_Common` + `BRepGProp` at the asserted
  deflections, with COMMON now a watertight MATCH (no longer an honest decline) at its
  ladder.
- **Honest decline preserved; no global tolerance widened.** A rim that still can't weld
  returns non-watertight → NULL → OCCT. No global weld/snap tolerance is widened; the fix is
  topology-scoped canonical pinning plus a coincident-duplicate defect repair, not a
  loosened threshold.
- Lands the outer curved-RIM half of the sharpened next blocker of
  `moat-m2cw-curved-wall-cut` (isolated by `moat-m0w-closed-seam-weld`): the rim is no longer
  a weld blocker at any deflection, so curved-wall freeform boolean COMMON on the dome/bowl
  pose is robust across the full deflection ladder.
