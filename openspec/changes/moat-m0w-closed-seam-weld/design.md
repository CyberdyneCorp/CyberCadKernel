# Design — moat-m0w-closed-seam-weld

## Context

`moat-m2cw-curved-wall-cut` landed `curvedWallHalfSpaceCut` (`boolean/curved_wall_cut.h`):
a dome/bowl cut by a HORIZONTAL plane along a CLOSED CIRCULAR seam interior to the freeform
wall. `splitFaceSmoothTrim` (`smooth_trim_split.h`) resolves the wall into a curved
annulus-with-hole (`faceOutside`) + a flat disk `faceInside`; the verb synthesizes ONE
flat planar cap on the cut plane from the SAME straight seam chords. `KeepSide::Below`
(CUT) — the disk + cap keep side — welds robustly. `KeepSide::Above` (COMMON) — the curved
annulus reusing the parent rim AND carrying the seam hole — welds watertight only at
isolated deflections, so it honestly DECLINES to NULL elsewhere. That fragility is asserted
by `curved_wall_common_rim_weld_fragility_is_measured_decline` and was named the sharpened
next blocker. This change removes it in the M0 tessellator.

## The failure, precisely

The shared boundary is a CLOSED seam between two sub-faces:
- the CURVED annulus (`faceOutside`) on the genuinely-curved bowl surface, and
- the FLAT disk / synthesized cap on the cut plane.

The seam edges are degree-1 straight 3D chords, but each chord carries a pcurve on the bowl
surface. Two regimes fail:

- **COARSE (weld-tolerance fold).** The solid mesher's spatial weld fuses vertices within
  `0.5·deflection`. Around a small circle the seam is dense (399 nodes); adjacent
  same-loop seam vertices fall inside the weld cell and MERGE, collapsing the seam into a
  non-manifold fold — an edge used by more than two triangles.
- **FINE (bowl-bulge divergence).** When the shared per-edge discretization subdivides a
  chord (`n > 1` samples), the annulus places its interior seam-boundary vertices by
  evaluating `S_bowl(pcurve(t))` — which BULGES off the straight chord — while the flat cap
  places them exactly on the chord. Measured at deflection `0.004`: only 399 of ~4788
  near-seam vertices coincide. The shared closed boundary does not weld watertight
  (thousands of open edges, each used by one triangle).

ROOT CAUSE: the seam edge's canonical geometry IS the straight degree-1 chord
(`EdgeDiscretization d.points`), but the mesher re-evaluates boundary vertices through each
face's OWN surface `S_face(pcurve(t))` rather than pinning them to the shared edge's
canonical 3D points. `BoundaryAnchors` snap fires only within `kSnapEps = 1e-6` — far
tighter than the bowl bulge — so it never snaps the subdivided interior samples.

## The fix: topology-guarded canonical seam pinning

A shared CLOSED seam SHALL carry ONE canonical shared discretization — the edge's
`d.points` — consumed by BOTH sub-faces with OPPOSITE orientation. Boundary vertices that
come from such a seam edge SHALL be PINNED EXACTLY to the seam edge's canonical 3D sample
points by PARAMETRIC / UV CORRESPONDENCE (the sample index along the shared edge), NOT by
spatial-proximity snap. Both sub-faces then emit BIT-IDENTICAL 3D seam points, so:

- the seam welds without relying on the spatial weld tolerance bridging two independent
  samplings (kills the FINE bowl-bulge divergence), and
- the CDT does not densify across or fuse through the seam (kills the COARSE fold).

This is the closed-seam analogue of the landed OPEN-seam / curved-edge canonical
single-sampling fix (the "Weld a shared CURVED edge" requirement): there the fence was ONE
canonical polyline `C_edge(t)` for a curved separate-node edge; here the seam edges are
STRAIGHT chords but the divergence comes from re-evaluating through the CURVED incident
surface, so the fence is EXACT pinning of the curved sub-face's seam-boundary vertices to
the straight chord's canonical points.

## The guard (byte-identical everywhere else)

The pinning fires ONLY for the closed-seam topology:
1. the face's surface is genuinely curved (not a Plane), AND
2. the boundary edge is a straight degree-1 chord seam, AND
3. that seam is shared as a CLOSED loop.

Every other surface kind and edge is untouched:
- an analytic-primitive edge shared through ONE `TShape` node keeps its per-node
  discretization;
- a straight separate-node edge keeps its endpoint-keyed count sharing and canonical
  straight anchors;
- a curved separate-node edge keeps the landed canonical curved-polyline path;
- an OPEN seam keeps its landed path.

So the FNV hash over `{vertices, triangles, watertight, area, volume}` is IDENTICAL before
vs after for every existing kind. The ONLY meshes that may change are the previously-failing
closed-seam annulus cases (non-watertight → watertight).

## Self-verify remains the arbiter; decline preserved

`curvedWallHalfSpaceCut`'s mandatory self-verify is unchanged: a valid solid is returned
ONLY when the M0 mesh is watertight AND encloses a positive finite volume, else NULL →
OCCT. This change makes the CLOSED SEAM pass the self-verify across the full ladder rather
than only at isolated deflections; the curved-wall COMMON, which additionally welds the
freeform bowl's OUTER curved RIM to the flat top lid (a SEPARATE, pre-existing curved-edge
fragility — a free-form face subdivides the shared rim beyond a flat neighbour's need, plus
a coarse-regime near-degenerate sliver), still honestly declines at the fine end; that
decline is now isolated to the rim, not the seam. A case that STILL can't weld returns
non-watertight → NULL → OCCT — never a leaky or partial solid. NO global weld or snap
tolerance is widened; the fence is topology-scoped canonical pinning.

## Two-gate acceptance

- **Gate A (host analytic, no OCCT).** The mid-wall ANNULAR-cap CUT (disk ∪ annulus whose
  inner hole is the closed seam — the canonical annulus case) welds watertight (Euler
  `χ = 2`) and converges to the closed-form cap volume across its full ladder
  `{0.02, 0.012, 0.008, 0.005, 0.0025}` (0.62% → 0.10%); the dome CUT closed seam welds
  watertight at the fine `d = 0.004` that DECLINED before the pin. Full host `ctest`: 58/58.
  A FNV mesh-hash battery (box / bump-capped cylinder curved seam / rational-BSpline cap /
  bowl operand / mid-wall operand × 6 deflections) is BYTE-IDENTICAL (36/36) before vs after.
- **Gate B (sim native-vs-OCCT parity).** The same bowl-cup operand reconstructed in OCCT
  (`Geom_BezierSurface` bowl + planar lid, sewn) cut by `BRepAlgoAPI_Common` matches the
  native CUT/COMMON on volume (`BRepGProp`), area, watertightness, Euler `χ = 2`, bbox, and
  one-sided Hausdorff at every asserted deflection on the booted iOS simulator (45/45),
  INCLUDING the newly-added fine `CUT d = 0.004` (native watertight, volume rel 4.0%, area
  0.5%, Hausdorff 3.4e-8) that declined pre-pin; COMMON's fine decline stays an honest NULL.

## Discipline

`src/native/**` stays OCCT-free; the `cc_*` ABI is additive-only. The regression test is
the byte-identical battery PLUS the now-watertight closed-seam ladder (the flipped
fragility test).

## Alternatives considered

- **Widen the weld / snap tolerance.** Rejected — it would perturb every other mesh and
  break byte-identity, and a wider `kSnapEps` still cannot span the bowl bulge without
  fusing genuinely-distinct vertices. The fence must be topology-scoped, not a global
  threshold.
- **Densify the cap to match the annulus's bulged samples.** Rejected — the cap is flat on
  the plane; its samples are correct. Pin the CURVED sub-face's boundary to the canonical
  chord instead.
