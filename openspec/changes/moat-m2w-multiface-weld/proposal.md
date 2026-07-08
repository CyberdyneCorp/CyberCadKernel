# Proposal — moat-m2w-multiface-weld (MOAT M2-multiseam · multi-face corner-clip weld)

## Why

The prior multi-seam waves landed two byte-frozen enablers:

- `buildSeamGraph` (`seam_graph.h`) — the two-cutting-face seam graph: both arcs traced,
  the analytic junction `J` computed and verified on BOTH cutting planes, the arcs joined
  into one bent boundary→J→boundary seam.
- `splitFaceJunction` (`junction_split.h`) — the junction-aware wall split: the bowl wall
  partitioned into the corner sub-face (`A ∩ {x≥0, y≥0}`) and the L-shaped survivor at the
  EXACT valence-3 vertex `J`, at machine precision, without weakening B2's rebuild tolerance.

Both proved in isolation on the host, but `freeformBooleanMultiSeam` still returned NULL →
OCCT with `MultiFaceWeldUnreachable`, because the corner box `B` straddles the corner of
`A`'s footprint quad `Q`: the `x=0`/`y=0` planes therefore corner-clip `A`'s flat BOTTOM
quad AND the TWO side walls whose `Q` edges cross the planes, and the result additionally
needs the box CAP faces synthesized inside `A` and the whole shell welded across MULTIPLE
junctions (the wall junction `J`, the bottom junction `J'`, the wall/plane pierce points).

## What changes

This change lands that multi-FACE corner-clip WELD as ONE additive OCCT-free header,
`src/native/boolean/multi_face_weld.h` (`multiFaceCornerClip`), and wires it into
`freeformBooleanMultiSeam` so the entry point returns a self-verified watertight result
solid for all THREE ops of the single corner-clip pose:

- **CUT** (`A − B`, the L-solid) — the bowl L-survivor (`splitFaceJunction.faceSurvivor`),
  the bottom quad L-survivor (a planar corner-clip rerouting the boundary through `J'`), the
  two whole side walls, the two side walls clipped by ONE plane (byte-frozen
  `hscdetail::cutAnalyticFace`), plus the two synthesized box CAP faces (on `x=0`/`y=0`,
  sharing the bowl seam arc with the wall survivor and the vertical `J→J'` edge with each other).
- **COMMON** (`A ∩ B`, the corner piece) — the complementary keep region: the bowl corner
  sub-face, the bottom corner (a convex two-plane clip), the two corner-side clipped walls,
  and the two caps (opposite outward normal).
- **FUSE** (`A ∪ B`) — the CUT `A`-faces (caps now interior, dropped) welded to `B`'s shell:
  `B`'s four non-cutting faces WHOLE and `B`'s two cutting faces NOTCHED by the cap region
  (a rectangle-minus-notch whose curved boundary IS the shared bowl seam arc).

Each welded result is admitted ONLY if the M0 mesh is WATERTIGHT and its volume lies in the
op's consistent bound; ANY failure returns NULL → OCCT. No tolerance is weakened; no
partial/leaky/wrong-volume solid is ever emitted.

## Scope / non-goals

- IN: the single corner-clip pose (one box corner straddling `A`'s footprint quad corner,
  two orthogonal iso-parametric arcs meeting at one wall junction). Byte-frozen substrate:
  B2 `face_split.h`, single-seam `inter_solid_seam.h`/`two_operand.h`, M0 tessellator, M1
  ssi, `seam_graph.h`, `junction_split.h`.
- OUT (measured next blockers): the general `≥3`-seam / branch-point seam graph; poses whose
  bottom/side faces are non-planar or whose junction is not a single valence-3 vertex; the
  freeform↔freeform boolean.

## Verification

- GATE (a) HOST-analytic (no OCCT): watertight + enclosed volume = the closed-form corner
  oracle (`V(A−B)=0.145035`, `V(A∩B)=0.051275`, `V(A∪B)=0.529035`) to the curved-tessellation
  band, monotone-converging across a deflection sweep, plus the honest-decline envelope.
- GATE (b) SIM native-vs-OCCT on the booted simulator: each op vs `BRepAlgoAPI_Cut/Common/Fuse`
  on volume/area/watertight/Euler χ=2/bbox/Hausdorff + a `BRepClass3d_SolidClassifier` batch.

No `cc_*` ABI change; `src/native/**` stays OCCT-free and additive.
