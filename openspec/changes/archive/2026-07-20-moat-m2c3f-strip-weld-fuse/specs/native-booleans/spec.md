# native-booleans

## ADDED Requirements

### Requirement: Multi-face STRIP WELD FUSE (A∪B) for the three-cutting-face two-junction pose, or DECLINE

The native boolean library SHALL extend the OCCT-free, header-only strip weld
(`src/native/boolean/multi_face_strip_weld.h`, `multiFaceStripClip`) to produce a WATERTIGHT
FUSE (`A ∪ B`) result solid for the edge-straddling three-cutting-face / two-junction strip
pose, in addition to the already-landed CUT and COMMON. The FUSE boundary SHALL be assembled
as `(∂A \ int B) ∪ (∂B \ int A)`: A's CUT-survivor faces (the three synthesised box caps are
interior to `A ∪ B` and dropped) welded to `B`'s six faces clipped to outside `A` — B's three
NON-cutting faces whole, and its three CUTTING faces (`x=x0`, `x=x1`, `y=y0`) clipped to
outside `A` along the shared seam arcs.

The MIDDLE cutting face's cap (`arcM`, J1→J2) spans the FULL box-middle-face width (J1 on the
`x=x0` side edge, J2 on the `x=x1` side edge) and attaches along BOTH junction columns
`[J1,J1b]` and `[J2,J2b]`. The weld SHALL therefore split the middle box face into TWO
disjoint planar pieces via an additive detail verb `mfswdetail::splitMiddleBoxFace` — a TOP
piece bounded by `arcM` + the box top edges, and a BOTTOM piece bounded by `J1b→J2b` + the
box bottom edges — so that every junction-column edge is used by EXACTLY two faces. The two
END cutting faces SHALL keep the byte-unchanged single-column corner-clip notch
(`mfwdetail::notchedBoxFace`). The `splitMiddleBoxFace` verb SHALL be axis-agnostic (derive
its column basis from the junction-column direction and the in-plane perpendicular, never
assuming which frame axis is vertical).

`multiFaceStripClip` with `StripWeldOp::Fuse` SHALL return a NULL Shape (→ OCCT fall-through)
with a measured `StripWeldDecline` (`LoopOpen` when the middle cap does not span both side
edges or a piece cannot chain closed; `NotWatertight` / `VolumeInconsistent` on self-verify)
and NEVER a leaky, overlapping or wrong-volume solid. No tolerance SHALL be weakened. The CUT
and COMMON code paths SHALL be byte-unchanged by this addition; the M0 tessellator and all
`cc_*` ABI SHALL be untouched.

#### Scenario: The strip-weld FUSE welds watertight at V(A∪B) (host, no OCCT)

- **WHEN** `A` is the bowl-lidded convex-quad prism, `B` the edge-straddling box
  (`x ∈ [−0.15, 0.15], y ∈ [0.0, 0.8], z ∈ [−0.6, 0.2]`), the chain seam graph + two-junction
  wall split are built, and `multiFaceStripClip(*op, *g, *split, StripWeldOp::Fuse, 0.01)` runs
- **THEN** the result is a non-NULL, WATERTIGHT solid whose enclosed volume equals the
  closed-form `chain_seam_fixture::volUnion()` to relative ≤ 2e-2, with
  `V(A∪B) > max(V(A), V(B))` (a discriminating union, not a subset op) and
  `max(V(A),V(B)) ≤ V ≤ V(A)+V(B)` (the inclusion–exclusion bound), and the volume converges
  monotonically to the oracle as deflection → 0.

#### Scenario: The strip-weld FUSE matches BRepAlgoAPI_Fuse (sim native-vs-OCCT)

- **WHEN** the SAME `A` and `B` are reconstructed in OCCT and `BRepAlgoAPI_Fuse` is the oracle
- **THEN** the native FUSE result (measured by the M0 tessellator) agrees with OCCT on volume
  (rel ≤ 2e-2), area (rel ≤ 2e-2), watertightness, topology (Euler χ = 2), bbox and one-sided
  Hausdorff (≤ 1.5·deflection), and a classify batch (zero crisp IN↔OUT disagreements) at the
  robust deflections 0.01 and 0.005 — with FIXED, curved-tessellation-bounded tolerances,
  never widened.
