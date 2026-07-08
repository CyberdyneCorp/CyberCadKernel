# native-tessellation

## ADDED Requirements

### Requirement: Weld a shared CURVED edge watertight at any deflection via ONE canonical per-edge discretization consumed by both incident faces

The library SHALL discretize a genuinely **curved** edge (3‑D curvature above zero —
a circle, ellipse, or free‑form seam such as the degree‑2 Bézier boolean trace or a
bowl‑lid parabola) that is shared by two faces through **separate edge nodes** into ONE
**canonical** per‑edge discretization — a single fraction list AND a single 3‑D polyline
`C_edge(t)` — that BOTH incident faces consume, so both faces place BIT‑IDENTICAL
boundary points at every sample and the seam welds watertight at ANY requested
deflection. The sharing SHALL be keyed by the edge's order‑independent, quantized
**endpoint pair PLUS a curve‑identity discriminator** (a quantized interior sample, e.g.
the 3‑D midpoint), so two GEOMETRICALLY DIFFERENT curves between the same endpoints never
collapse to one canonical record. This generalises the existing straight‑edge
endpoint‑keyed single‑sampling (the segment‑count sharing and canonical straight anchors)
to curved edges.

The addition SHALL be strictly additive and reachable ONLY by a curved edge that is (a)
NOT straight in 3‑D and (b) NOT already shared through a single `TShape` node. An edge
shared through ONE `TShape` node (every analytic primitive — a cylinder cap and side
share the circle node) SHALL keep its existing per‑node discretization, and a straight
separate‑node edge SHALL keep its existing endpoint‑keyed count sharing and canonical
straight anchors, so every existing face SHALL mesh **byte‑identically** — the same
triangle counts, the same watertight status, and the same enclosed volumes — PROVEN
across the full tessellation‑sensitive suite (`run-sim-suite`, STEP import, curved‑fillet,
curved‑chamfer, curved‑boolean, wrap‑emboss, loft, phase3) and a per‑surface‑kind
snapshot diffed against the base. The canonical‑curved path SHALL NOT modify the shared
segment‑count sizing, the curve evaluators, the three face‑mesh arms (`structuredGrid`,
`earClipMesh`, `trimmedFreeformMesh`), the boundary flattener, or the spatial weld. The
library SHALL remain OCCT‑free and host‑buildable, and no tolerance SHALL be weakened. If
a clean additive path that keeps every existing mesh byte‑identical AND welds the curved
seam at every deflection cannot be achieved, the change SHALL be reverted and the freeform
boolean SHALL keep the honest OCCT decline (an OCCT‑handled boolean loses nothing).

#### Scenario: A freeform boolean CUT welds watertight across a deflection sweep at the closed-form volume (host, no OCCT)

- GIVEN the bowl‑lidded convex‑quad operand (a degree‑2 Bézier top genuinely trimmed by
  the quad, four planar side walls, a planar bottom) and the half‑space CUT `x ≤ 0`,
  built on the host with no OCCT
- WHEN it is meshed through the canonical curved‑edge single‑sampling at each deflection
  in the sweep `{0.03, 0.02, 0.01, 0.008, 0.004, 0.002}`
- THEN the CUT solid SHALL be watertight at EVERY deflection (no watertight↔NotWatertight
  oscillation), AND its enclosed volume SHALL lie within the deflection band of the
  closed‑form value `∫∫_{Q∩{x≤0}} (H0 + a·(x²+y²)) dA`, converging as deflection → 0

#### Scenario: A freeform boolean COMMON welds watertight across the same sweep (host, no OCCT)

- GIVEN the same bowl operand and half‑space COMMON, built on the host with no OCCT
- WHEN it is meshed at each deflection in the sweep `{0.03, 0.02, 0.01, 0.008, 0.004, 0.002}`
- THEN the COMMON solid SHALL be watertight at EVERY deflection AND its enclosed volume
  SHALL lie within the deflection band of the closed‑form COMMON value

#### Scenario: The bowl-lid curved seam welds because both faces share ONE canonical polyline (host)

- GIVEN a genuinely curved edge (a bowl‑lid parabola — `(x,y)` linear × `z` quadratic)
  shared by the curved Bézier top face and a planar side‑wall face as SEPARATE edge nodes
- WHEN both faces are meshed with a shared edge cache at any deflection
- THEN both faces SHALL read the SAME canonical fraction list and the SAME canonical 3‑D
  polyline, place BIT‑IDENTICAL boundary points at every sample, and the two faces SHALL
  weld into a watertight seam without relying on the spatial weld tolerance bridging two
  independent samplings

#### Scenario: Two different curved edges between the same endpoints keep separate discretizations (host)

- GIVEN two geometrically DIFFERENT curved edges (for example a minor and a major arc, or
  two distinct blend seams) that happen to share the same two endpoints
- WHEN each is discretized through the canonical curved‑edge cache
- THEN the curve‑identity discriminator (the quantized interior sample) SHALL place them
  in DIFFERENT canonical records, so neither edge's sampling is corrupted by the other

#### Scenario: Every existing surface kind meshes byte-identically after the addition (host + sim)

- GIVEN faces of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`,
  `Torus`, bare‑periodic `BSpline`, `Bezier`), a planar trim, and a loft side wall,
  meshed before and after the canonical curved‑edge path is added, together with the full
  tessellation‑sensitive suite (`run-sim-suite`, STEP import, curved‑fillet,
  curved‑chamfer, curved‑boolean, wrap‑emboss, loft, phase3)
- WHEN each existing face / suite is meshed at the same deflection and compared against the
  pre‑change baseline
- THEN the triangle counts, watertight status, and enclosed volumes SHALL be IDENTICAL to
  the baseline (the canonical curved path is reachable only by a curved separate‑node
  shared edge that today welds coincidentally); if ANY differs, the change SHALL be
  reverted and the freeform boolean SHALL keep the OCCT path

#### Scenario: The native freeform CUT matches the OCCT oracle at multiple deflections, or declines honestly (sim)

- GIVEN the freeform CUT built natively and its OCCT `BRepAlgoAPI_Cut` + `BRepMesh` oracle,
  on the booted simulator with OCCT linked
- WHEN the native solid is meshed at multiple deflections (at least `0.01` and one finer)
  and compared to the oracle
- THEN at EACH deflection the native volume, area, watertight status, and triangle envelope
  SHALL match OCCT within tolerance — OR the reader SHALL decline and the file SHALL
  round‑trip through OCCT unchanged (both PASS); a non‑watertight native mesh SHALL never be
  emitted and no tolerance SHALL be weakened
