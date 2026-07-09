# native-booleans

## ADDED Requirements

### Requirement: Walled-bowl MID-WALL freeform half-space CUT welds an annular cross-section cap, or DECLINES

The native curved-wall half-space cut verb `curvedWallHalfSpaceCut` SHALL handle the
walled-bowl MID-WALL pose in addition to the dome pose (`src/native/boolean/curved_wall_cut.h`).
The dome pose is a closed circular seam on the sole freeform wall with no analytic face
split, capped by a simple DISK. The MID-WALL pose is a solid whose sole freeform wall is
cut by a plane `P` along a CLOSED interior circular seam AND at least one of whose PLANAR
analytic faces is genuinely SPLIT by `P` (the byte-frozen `hscdetail::cutAnalyticFace` returns
`Kind::Split`). In that pose the cross-section cap on `P` is an ANNULUS, and the verb
SHALL synthesize it as ONE planar face whose OUTER wire is the wall-section polygon (the
split faces' `Face ∩ P` chords, chained by the byte-frozen `hscdetail::orderLoop` and
validated simple by `hscdetail::loopSimple`) and whose HOLE wire is the closed freeform
seam (the SAME seam nodes the B2 `splitFaceSmoothTrim` split laid on the freeform disk
sub-face, wound opposite the outer). The cap's outward normal SHALL face the DISCARD side
and SHALL be determined DETERMINISTICALLY from the plane frame's `frame.z` (a Forward face
gives `+frame.z`, a Reversed face `−frame.z`) — INDEPENDENT of the wall-chord loop
winding. The cap SHALL weld to both the kept analytic-wall sub-faces (shared crossing
endpoints) and the freeform disk sub-face (shared seam nodes) through the M0 mesher.

When NO analytic face is split, the verb SHALL keep the byte-identical single-DISK cap
path (`synthCircularCap`), so the dome pose's behaviour is unchanged. The change SHALL be
strictly ADDITIVE: it SHALL NOT modify `splitFaceSmoothTrim`, B2 `splitFace`,
`hscdetail::cutAnalyticFace` / `orderLoop` / `loopSimple` / `edgeFromPiece`,
`recogniseFreeformSolid`, the M0 tessellator, or M1, and SHALL consume their primitives
BYTE-IDENTICAL. The mandatory self-verify (M0 mesh, require WATERTIGHT AND positive finite
enclosed volume) SHALL be unchanged: the verb SHALL return a NULL Shape with a typed
`CurvedWallCutDecline` whenever any stage declines (including `AnalyticCutFailed` when the
wall-section chords do not chain into one simple closed loop), SHALL NEVER emit a leaky or
partial solid, and SHALL NOT weaken any tolerance to force a pass. The verb SHALL remain
OCCT-free, SHALL introduce no `cc_*` ABI surface, and SHALL keep its per-function
cognitive complexity within the backend band.

#### Scenario: The MID-WALL CUT welds an annular cap watertight at the closed-form volume and converges (host, no OCCT)

- GIVEN a steep degree-2 Bézier bowl over a convex quad plus four PLANAR side walls and a flat base, and a horizontal cut plane `z = c` chosen strictly below the interior-circle bound `a·d_e²` (so the real S3 seam is a CLOSED CIRCLE of radius `ρ = √(c/a)` interior to the quad AND each of the four walls is genuinely split), built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut(operand, P, KeepSide::Below, d)` runs across a deflection sweep
- THEN it SHALL return a single closed solid with SEVEN faces (the freeform disk, four lower wall trapezoids, the base, and the ANNULAR cap) whose M0 mesh is WATERTIGHT with Euler characteristic `χ = 2` at every deflection, whose enclosed volume equals the closed form `(H0 + c)·A_Q − c·π·ρ²/2` within the curved-tessellation band and CONVERGES monotonically to it as the deflection tightens

#### Scenario: The removed cap matches the bowl-cap closed form (host, no OCCT)

- GIVEN the same walled-bowl operand and mid-wall cut plane, built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut(operand, P, KeepSide::Below, d)` runs and its meshed enclosed volume is subtracted from the full operand volume
- THEN the measured removed volume SHALL match the bowl-cap closed form `V(full) − ((H0+c)·A_Q − c·π·ρ²/2)` within the curved-tessellation band, and that removed volume SHALL be a substantial (non-degenerate) fraction of the solid — asserting the annular-cap `−c·π·ρ²/2` hole term is present

#### Scenario: The dome pose keeps the byte-identical disk-cap behaviour (host, no OCCT)

- GIVEN the landed steep bowl-cup operand whose sole non-freeform face (the flat top lid) is entirely on the keep side, so NO analytic face is split, built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut` runs on it
- THEN it SHALL take the single-DISK cap path and its results (watertight CUT/COMMON, the measured COMMON rim-weld fragility decline) SHALL be UNCHANGED from before this change

#### Scenario: The MID-WALL CUT matches the OCCT boolean (sim native-vs-OCCT)

- GIVEN the SAME walled-bowl operand reconstructed in OCCT (a `Geom_BezierSurface` bowl trimmed by the convex quad, four planar walls with an exact degree-2 Bézier top edge, and a planar base, sewn into an outward-oriented solid) and cut by `BRepAlgoAPI_Cut` against a large box spanning `z ≥ c`, on a booted iOS simulator
- WHEN `curvedWallHalfSpaceCut(operand, P, KeepSide::Below, d)` runs natively (OCCT-free) at multiple deflections and is measured by the native M0 tessellator
- THEN at each deflection the native cut SHALL be a WATERTIGHT closed solid with Euler `χ = 2` and the annular seven-face topology, and SHALL match the OCCT cut on volume and area within the fixed curved-tessellation band and on bounding box and one-sided Hausdorff distance within the fixed spatial band, with the OCCT cut itself cross-checked against the closed-form volume — no tolerance widened
