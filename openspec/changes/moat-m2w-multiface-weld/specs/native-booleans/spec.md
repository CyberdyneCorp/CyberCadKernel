# native-booleans

## ADDED Requirements

### Requirement: Multi-face corner-clip weld for the two-cutting-face freeform↔box pose, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only multi-face
corner-clip WELD verb (`src/native/boolean/multi_face_weld.h`, `multiFaceCornerClip`)
that, GIVEN a `recogniseFreeformSolid`-admitted bowl-lidded convex-quad prism `A`, its
two-arc one-junction seam graph (from `buildSeamGraph`), and its junction-aware wall
split (from `splitFaceJunction`) for the pose in which a finite axis-aligned box `B`
straddles the corner of `A`'s footprint quad — so `B`'s two adjacent cutting planes each
slice `A`'s Bézier wall AND corner-clip `A`'s flat bottom quad and the two side walls
whose footprint edges cross the planes — assembles a WATERTIGHT result solid for each of
the three ops CUT (`A − B`), COMMON (`A ∩ B`) and FUSE (`A ∪ B`).

For CUT/COMMON the verb SHALL assemble: the bowl sub-face (`faceSurvivor` for CUT,
`faceCorner` for COMMON); the bottom quad clipped to the op's keep region (an L-shaped
survivor rerouted around the removed quadrant through the corner vertex `J'` for CUT, a
convex two-plane clip for COMMON); the side walls clipped by ONE plane through the
byte-frozen `hscdetail::cutAnalyticFace` (the away side for CUT, the corner side for
COMMON), with fully-kept or fully-dropped walls handled whole; and the TWO synthesized
box CAP faces on the cutting planes inside `A`, each bounded by the bowl seam arc half
(shared BIT-EXACT with the wall sub-face), the shared vertical `J→J'` edge, and the
bottom/outer chords. For FUSE the verb SHALL instead weld `A`'s CUT faces (the caps now
interior, dropped) to `B`'s shell: `B`'s non-cutting faces WHOLE and `B`'s two cutting
faces NOTCHED by the cap region (a rectangle-minus-notch whose curved boundary IS the
shared bowl seam arc, the notch attached along the shared `J→J'` segment).

The verb SHALL admit a welded result ONLY after a mandatory self-verify: the M0 mesh is
WATERTIGHT (every undirected edge shared by exactly two triangles) AND its enclosed
volume lies in the op's consistent bound (`0 ≤ V(A−B) ≤ V(A)`; `0 ≤ V(A∩B) ≤
min(V(A),V(B))`; `max(V(A),V(B)) ≤ V(A∪B) ≤ V(A)+V(B)`), measured from the operand and
box meshes. On ANY failure — no analytic face straddling both planes (`NoStraddlingBottom`),
a synthesized boundary that does not chain closed (`LoopOpen`), a side-wall clip decline
(`WallClipFailed`), a non-watertight weld (`NotWatertight`), or an out-of-bound volume
(`VolumeInconsistent`) — the verb SHALL return a NULL `Shape` carrying the measured
decline. It SHALL remain OCCT-free, introduce NO `cc_*` ABI surface, weaken NO tolerance,
and consume the byte-frozen substrate (B2 `face_split.h`, `seam_graph.h`,
`junction_split.h`, the `hscdetail::` primitives, `assemble.h`, the M0 mesher) UNCHANGED.

#### Scenario: All three ops weld watertight at the closed-form corner oracle volumes (host, no OCCT)

- GIVEN the bowl-lidded convex-quad prism `A`, the corner box `B` (`x∈[0,0.8], y∈[0,0.6], z∈[−0.6,0.2]`) whose `x=0` and `y=0` faces each slice `A`'s wall and also corner-clip `A`'s bottom quad and two side walls, and the landed seam graph + junction-aware wall split, built on the host with NO OCCT
- WHEN `freeformBooleanMultiSeam(A, B, op)` composes `buildSeamGraph → splitFaceJunction → multiFaceCornerClip` for each of CUT, COMMON, FUSE
- THEN each SHALL return a non-NULL watertight result solid whose enclosed volume equals the closed-form corner oracle (`V(A−B)=0.145035`, `V(A∩B)=0.051275`, `V(A∪B)=0.529035`) to the curved-tessellation band (rel ≤ 2e-2 at deflection 0.01), with NO tolerance weakened

#### Scenario: Volume monotonically converges to the closed form as deflection tightens (host)

- GIVEN the CUT weld of `A` and `B` meshed across a deflection sweep `{0.02, 0.01, 0.005}` on a host build with NO OCCT
- WHEN the enclosed volume of each welded result is compared to the closed-form `V(A−B)`
- THEN the absolute error SHALL monotonically decrease with deflection and reach ≤ 0.5% at the tightest deflection, converging from above (the curved bowl top over-meshes), never a widened tolerance

#### Scenario: A pose outside the corner-clip envelope or a failed self-verify DECLINES to NULL (host)

- GIVEN a box that presents only ONE cutting face (the single-seam path's job), or a non-freeform operand, or any weld whose mesh is not watertight or whose volume is outside the op's consistent bound, built on the host with NO OCCT
- WHEN `freeformBooleanMultiSeam` is invoked
- THEN it SHALL return a NULL `Shape` with the measured decline (`SeamGraphDeclined`, `NotAdmittedA`, or `MultiFaceWeldDeclined`) and NEVER a leaky/partial/wrong-volume solid, leaving OCCT `BRepAlgoAPI_*` as the fall-through

### Requirement: Native-vs-OCCT parity for the multi-face corner-clip weld on the booted simulator

The multi-face corner-clip weld SHALL be verified against the OCCT ORACLE on a booted iOS
simulator (SIM GATE (b), `tests/sim/native_multi_seam_freeform_boolean_parity.mm` with its
own `main()`, registered via `scripts/run-sim-native-multi-seam-freeform-boolean.sh` and a
SKIP entry in `scripts/run-sim-suite.sh`). The harness SHALL reconstruct the SAME operand
`A` in OCCT (a sewn `Geom_BezierSurface`-topped 6-face solid) and the SAME corner box `B`
as a `BRepPrimAPI_MakeBox`, run `BRepAlgoAPI_Cut`, `BRepAlgoAPI_Common` and
`BRepAlgoAPI_Fuse`, and compare the native result of each op (measured by the native M0
tessellator) on volume (rel ≤ 2e-2, cross-checked against the closed-form corner oracle),
area (rel ≤ 2e-2), watertightness (closed 2-manifold), topology (Euler χ = 2), bbox and
one-sided Hausdorff (≤ 1.5·deflection), and a `BRepClass3d_SolidClassifier` query-point
batch (zero crisp IN↔OUT disagreements), at two deflections. OCCT SHALL be the oracle ONLY
and SHALL NOT be linked into `src/native`.

#### Scenario: Each op matches BRepAlgoAPI on the booted simulator (native vs OCCT)

- GIVEN the native multi-face weld and the OCCT `BRepAlgoAPI_Cut/Common/Fuse` of the SAME reconstructed `A` and corner box `B`, on a booted iOS simulator
- WHEN each native op is meshed and compared to its OCCT counterpart at deflection 0.01 and 0.005
- THEN volume, area, watertightness, Euler χ = 2, bbox and one-sided Hausdorff SHALL agree within the fixed (never-widened) tolerances, the classify batch SHALL show zero crisp IN↔OUT disagreements, and the single-cut-box fallback SHALL DECLINE to NULL
