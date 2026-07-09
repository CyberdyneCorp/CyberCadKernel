# Design — moat-m6i-reference-geometry-fuzz (MOAT M6-breadth-9)

Extend the differential-fuzzing completeness bar to a NINTH native domain — the
**reference / datum-geometry + topology-read layer** the CyberCad app's datum/reference
tools read (`cc_face_axis`, `cc_ref_axis_from_face`, `cc_ref_plane_from_face`,
`cc_ref_axis_from_edge`, `cc_tangent_chain`, `cc_outer_rim_chain`,
`cc_offset_face_boundary`). This is INFRASTRUCTURE: a seeded harness, not a geometry
capability. The OCCT topology-query oracle (`gp_Cylinder`/`gp_Cone::Axis`, `gp_Pln`,
`gp_Lin`, `BRepOffsetAPI_MakeOffset`, `BRepTools::OuterWire`, `BRepAdaptor_Curve::D1`)
is the SECONDARY oracle; a THIRD, engine-independent closed-form datum image is the
PRIMARY arbiter; the bar is ZERO silent wrong datums over a seeded batch; a
reference.h scoped decline is first-class. `src/native/**` and `src/engine/**` are
UNTOUCHED — they are the system under test.

## 0. Why reference geometry (the domain choice)

The task offered direct-modeling, reference-geometry, or section-curve. Reference
geometry is the pick:

- **Cleanest closed-form arbiter.** Every datum of an analytic primitive is exact: a
  cylinder/cone axis (a line), a planar cap normal (a direction + on-plane point), a
  straight edge (a line), a convex polygon inward-offset (a shrunk polygon of exact
  area). Under a RIGID pose these transform EXACTLY, giving a third oracle independent
  of BOTH the native engine and OCCT — the pristine differential-fuzzing setup.
- **Highest-value gap.** M-REF just landed (a MUST-GO-NATIVE Class-B bucket, 22 app
  sites) with *per-op* parity (`native_reference_parity.mm`) but no *fuzz* domain.
  Direct-modeling ops gate on the `CYBERCAD_HAS_NUMSCI` freeform seam-trace substrate
  and have narrower analytic coverage; section curves have no native `cc_*` producer to
  fuzz against a native path (the app's section pipeline is OCCT-only today).

## 1. The substrate (verified in source, not assumed)

`src/native/reference/reference.h` (OCCT-FREE, header-only, `cybercad::native::
reference`):

- `faceAxis(face)` / `refAxisFromFace(face)` — cylinder/cone axis; a plane/sphere/torus
  declines (`std::nullopt`). World-placed via the sub-shape `Location`.
- `refPlaneFromFace(face)` — planar-face datum plane (outward normal reversed for a
  REVERSED face; origin = outer-wire vertex centroid, an on-plane point). Non-planar →
  decline.
- `refAxisFromEdge(edge)` — straight-edge line axis (frame X). A circular edge declines.
- `tangentChain(root, seeds)` — grow the C1-continuous (|t₁·t₂| ≥ cos 15°) connected
  edge run; a freeform edge in the walk declines (`std::nullopt`).
- `outerRimChain(root, seeds)` — the outer-wire edge ids of the planar cap face(s) the
  seed edges bound (a face qualifies iff its plane contains all seed vertices).
- `offsetFaceBoundary(face, d)` — in-plane inward miter offset of a planar POLYGON
  boundary; a non-planar/arc/growing-convex/self-intersecting offset declines.

`src/native/topology/shape.h`: `Shape::located(Location{math::Transform})` composes the
affine onto the shape's `Location` (a placement); the reference services bake that
`Location` into every datum. The harness drives EXACTLY that placed-query path.

## 2. The base families + the rigid-pose analytic image

Each base solid is built by the OCCT-FREE `src/native/construct` builders so the
construction datum is KNOWN exactly, then posed. The construction places every family so
its natural datum is axis-aligned in the LOCAL frame:

- **BOX / NGON prism** (`build_prism`, z=0 loop extruded by +depth): planar caps at
  z=0 (−Z) / z=depth (+Z) and planar side faces with known normals; all edges are
  straight lines; the caps are POLYGONS (offset applies).
- **CYLINDER** (`build_revolution` of `{0,0, R,0, R,h, 0,h}` about +Y): a Cylinder
  lateral face whose axis is +Y through the origin; circular planar caps at y=0 / y=h.
- **CONE frustum** (`build_revolution` of the trapezoid `{0,0, R0,0, R1,h, 0,h}` about
  +Y, R0≠R1): a Cone lateral face whose axis is +Y; circular planar caps.

The rigid pose `P = T ∘ R` (rotate about a random unit axis through the origin, then
translate) is built THREE independent ways in lock-step: the native `math::Transform`
(driven via `located()`), the OCCT `gp_Trsf` (driven via `BRepBuilderAPI_Transform` on
the OCCT analog base), and an engine-independent fp64 affine `Aff`. Because `P` is
rigid (orthonormal linear part, no scale/mirror), the analytic datum image is EXACT:

- axis/line direction ← `P.linear · d₀`; origin/point ← `P · o₀`.
- planar cap normal ← `P.linear · n₀`.
- inward-offset polygon area is INVARIANT under a rigid map, so it equals the closed
  form computed on the UNPOSED base polygon (a miter offset by the fixed distance).

## 3. The five reference ops + their arbiters

Per trial, on the posed native solid vs the posed OCCT analog vs the analytic image:

- **PLANE** (`refPlaneFromFace`): every OCCT planar face's (posed) outward normal must
  be matched by a native datum whose normal is parallel (dot = 1 within `dirTol=1e-9`)
  AND whose origin is coplanar (residual along the normal ≤ `ptTol=1e-7`). All OCCT
  planar faces must be matched.
- **FAXIS** (`faceAxis` = `refAxisFromFace`): the native lateral-face axis must be
  parallel to the analytic posed +Y axis AND its origin must lie on that line, AND OCCT
  `gp_Cylinder`/`gp_Cone::Axis` must concur. `refAxisFromFace == faceAxis` bit-for-bit
  (contract).
- **EAXIS** (`refAxisFromEdge`): every OCCT line edge's (posed) `gp_Lin` must be matched
  by a native straight-edge axis (dir parallel + OCCT midpoint on the native line).
- **OFFSET** (`offsetFaceBoundary`, POLYGON caps only): the native inward-offset polygon
  area must equal the closed-form miter-offset area within `offTol=1e-6`. A circular cap
  (CYLINDER/CONE) MUST decline (no polygon boundary) — matched by the closed form as an
  HONESTLY-DECLINED, and native returning a polygon there would be a DISAGREE.
- **RIM** (`outerRimChain`): a polygon cap's rim must equal the OCCT `OuterWire` line
  edge midpoint set exactly. A CIRCULAR cap is arbitrated STRUCTURALLY — the rim id set
  must equal the native cap face's own outer-wire edge ids, and the OCCT circle oracle
  must confirm a circular boundary. (The native periodic revolution cap stores its rim
  as ≥1 Circle-ARC edges with periodic seam vertices — a legitimate representational
  difference from OCCT's single seam edge; per-vertex geometry against the OCCT circle
  is therefore NOT the faithful oracle, but the rim-id ⇔ cap-outer-wire identity IS the
  closed-form ground truth for "which edges bound this cap".)
- **TANGENT** (`tangentChain`): a prism straight-edge seed grows a C1 run; the native
  grow/stop decision must be consistent with the analytic fact that all prism edges are
  straight (a grown pair must be collinear — a non-collinear grow is a DISAGREE). A
  circle-edge seed on a cylinder/cone must NOT decline (reference.h supports analytic
  Circle tangents).

## 4. Six-way classifier (fixed tolerance, NEVER widened)

- **AGREED** — native datum matches the analytic image AND the OCCT oracle within tol.
- **HONESTLY-DECLINED** — native returns `nullopt`/empty on a reference.h scoped-out case
  (circular-cap offset, freeform edge in a tangent walk) where the closed form agrees no
  closed-form datum exists. First-class, logged, NOT a bar failure.
- **DISAGREED** — native returned a datum that does NOT match the analytic image. A
  SILENT WRONG DATUM — the failure the harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the analytic image, OCCT does not (native
  vindicated by exact math). Logged, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — a scoped-out exerciser both native and the oracle refuse.
- **ORACLE_UNRELIABLE** — a core case whose OCCT oracle does not match the closed form
  (e.g. no OCCT cap oracle found). FAILS the bar (investigate; never laundered to pass).

Bar: **DISAGREED == 0 AND ORACLE_UNRELIABLE == 0**, each base family
(BOX/NGON/CYLINDER/CONE) with ≥1 AGREED, each op (PLANE/FAXIS/EAXIS/OFFSET/RIM/TANGENT)
exercised in ≥1 AGREED — proven over ≥2 seeds. The rigid tolerance is NEVER widened to
force a pass.

## 5. Determinism + wiring

The generator is seeded ONLY by an explicit `FUZZ_SEED` (argv/env, fixed default
`0x9EF12A0055`); same seed + batch size → byte-identical batch (verified by re-run
diff). The runner mirrors `run-sim-native-reference.sh`: it needs NO numsci (reference /
construct / topology / math are header-only, OCCT-FREE), compiling only the math
bezier/bspline TUs and linking the OCCT oracle toolkits (adding `TKHLR`/`TKShHealing`
for the always-linked drafting adapter). The new `.mm` is on the `run-sim-suite.sh` SKIP
list (own `main()`, `std::_Exit`).
