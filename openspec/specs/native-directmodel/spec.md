# native-directmodel Specification

## Purpose
TBD - created by archiving change moat-dm1-split-plane. Update Purpose after archive.
## Requirements
### Requirement: Native `cc_split_plane` computes one watertight piece of a plane cut for a reachable native solid

The engine SHALL compute `cc_split_plane(body, o, n, keepPositive)` — cut `body` by the
plane through `o` with normal `n` and return the single half selected by `keepPositive`
(`keepPositive != 0` keeps the `+n` half, `0` keeps the `−n` half) — NATIVELY, without
OCCT, when `body` is a native solid in the reachable domain. The engine SHALL build the
cut plane `P` with origin `o` and normal `normalize(n)`, map `keepPositive` to the kept
half-space (`keepPositive != 0` → keep signed distance `≥ 0` along `P`'s normal
(`KeepSide::Above`); `0` → keep `≤ 0` (`KeepSide::Below`)), and dispatch by operand
kind:

- **A freeform-walled operand** — exactly ONE `Kind::Bezier`/`Kind::BSpline` wall with
  planar analytic caps/sides (the landed M2 half-space-cut domain) — SHALL be split by
  invoking the landed `freeformHalfSpaceCut(operand, P, side)` verb for the requested
  keep side, CONSUMED byte-identical (both `KeepSide::Below` and `KeepSide::Above` give
  the two complementary pieces).
- **An all-planar polyhedron** — every face a `Plane` (box / prism / convex or
  simple-concave) — SHALL be split by cutting the operand against a bbox-scaled native
  half-space box covering the DISCARD half via the landed BSP boolean
  `boolean_solid(operand, halfSpaceBox, Op::Cut)`, CONSUMED byte-identical, which caps
  the section on `P`.
- **An axis-aligned cylinder cut by a plane perpendicular to its axis** (`n` parallel
  to the axis — the axis-aligned box⟷cylinder analytic slice) SHALL be split by the same
  half-space-box cut through the native curved boolean, yielding a shorter coaxial
  cylinder with a planar cap.

The returned piece SHALL be a native `topology::Shape` of type `Solid`, watertight
(closed 2-manifold, every edge shared by exactly two faces) with positive enclosed
volume. The two complementary pieces of a cut (the two keep sides) SHALL satisfy
partition-closure: `V(below) + V(above)` SHALL equal the whole solid's enclosed volume
within the tessellation deflection tolerance, and each piece SHALL match its closed-form
volume where known (an axis-aligned box: fp-exact half-volumes; an axis-aligned cylinder
cut perpendicular to its axis: `π·r²·h_i`). This native path SHALL remain OCCT-free —
the consumed verbs (`freeformHalfSpaceCut`, `boolean_solid`) reference no OCCT — and the
`cc_*` ABI SHALL be unchanged (DM1 supplies only a native path behind the existing
facade seam).

#### Scenario: An axis-aligned box split by a plane yields two watertight pieces summing to the whole (host)

- GIVEN an axis-aligned native box `B` and a plane `P` that strictly crosses it, with the native engine active (`cc_set_engine(1)`) and no OCCT
- WHEN `cc_split_plane(B, P.o, P.n, keepPositive)` is invoked for both `keepPositive = 0` and `keepPositive = 1`
- THEN each returned piece SHALL be a watertight `Solid` with positive enclosed volume AND `V(keepPositive=0) + V(keepPositive=1)` SHALL equal `|B|` within the deflection tolerance AND each half SHALL equal its fp-exact closed-form half-volume

#### Scenario: An axis-aligned cylinder cut perpendicular to its axis yields two coaxial cylinders (host)

- GIVEN a native axis-aligned cylinder of radius `r` and height `h` and a plane `P` perpendicular to its axis crossing it at height `h_0`, with the native engine active and no OCCT
- WHEN `cc_split_plane` is invoked for both keep sides
- THEN each piece SHALL be a watertight `Solid` with a planar cap on `P` AND their volumes SHALL equal `π·r²·h_0` and `π·r²·(h − h_0)` within the deflection tolerance AND SHALL sum to `π·r²·h`

#### Scenario: A freeform-walled operand is split via the landed half-space CUT on the requested keep side (host)

- GIVEN the bowl-lidded convex-quad prism (one `Kind::Bezier` freeform wall over planar caps/sides) and a plane `P` in the landed M2 domain, with the native engine active and no OCCT
- WHEN `cc_split_plane` is invoked for a keep side
- THEN the engine SHALL return the `freeformHalfSpaceCut(operand, P, side)` piece BYTE-IDENTICAL to invoking that verb directly AND the piece SHALL be watertight at its closed-form volume band AND the two keep sides SHALL satisfy `V(below) + V(above) = V(whole)`

#### Scenario: A verified native split piece is read back by the native paths (host)

- GIVEN a native `cc_split_plane` piece that PASSES the self-verify (watertight 2-manifold, positive volume)
- WHEN its mass properties, bounding box, sub-shape ids, and tessellation are queried
- THEN they SHALL be served by the native body-consuming paths with no OCCT fallback call

### Requirement: Mandatory per-piece split self-verify (discard and fall through)

The engine SHALL accept a native `cc_split_plane` piece as native ONLY when it PASSES a
mandatory self-verify: the candidate SHALL be a **closed watertight 2-manifold** with
**positive enclosed volume**, measured by the engine's existing `watertightVolume`
audit over the mesher's deflection ladder. If the candidate is NULL (the consumed verb
declined) OR fails the watertight/positive-volume check, the engine SHALL DISCARD it and
return EXACTLY the OCCT fallback (`OcctEngine::split_plane`) result for that call. The
engine SHALL NEVER emit an unverified, leaky, or wrong split piece, and SHALL NEVER hand
a native void to OCCT (a split operand is always OCCT-reconstructible, so OCCT is the
true fall-through).

#### Scenario: A non-watertight or NULL native split candidate is discarded (host)

- GIVEN a native split candidate that is NULL (the consumed verb declined) OR open / non-manifold / zero-volume, with the native engine active
- WHEN the per-piece self-verify is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL fall through to `OcctEngine::split_plane`, emitting NO leaky or wrong piece

#### Scenario: The native split piece matches the OCCT oracle per-piece on a booted simulator (sim)

- GIVEN a reachable fixture (axis-aligned box / axis-aligned cylinder cut perpendicular to its axis / bowl-lidded prism) and a keep side, on a booted iOS simulator with OCCT available as the oracle
- WHEN the native `cc_split_plane` piece is compared against the OCCT piece (`BRepAlgoAPI_Section` / the OCCT two-sided `Cut`) for the same body, plane, and keep side
- THEN the per-piece volume, area, watertightness (closed 2-manifold), topology (Euler χ = 2, single closed solid), and bounding box SHALL match within the landed curved-slice tolerances (volume relative ≤ 2e-2, area/bbox tight), with no tolerance widened

### Requirement: Grazing-tangent, multi-lump, degenerate, multi-freeform, and foreign split cases fall through to OCCT

The native `split_plane` branch SHALL DECLINE (return a NULL piece → the OCCT fallback)
for any case outside the reachable native domain, each labelled and verified as a
fall-through and NEVER faked: (1) an **oblique plane grazing / tangent to a curved
face** (the plane–curved-surface trace is a general ellipse / conic, not the circle /
line the native slice supports); (2) a **multi-solid result** — a plane that severs the
solid into more than the two connected halves, or a keep side that is disconnected into
multiple lumps; (3) a **degenerate** configuration — the plane misses the solid, is
coincident with a boundary face, or leaves a zero-volume sliver; (4) a **multi-freeform**
operand (more than one freeform wall, beyond the M2 single-wall slice); (5) a **foreign /
non-native** body with no native B-rep to split. When the native engine is active, each
such case SHALL produce EXACTLY the result of the same call under the OCCT engine
(`cc_set_engine(0)`), proving fall-through with no native interception. The change SHALL
NOT fake, stub-out, or partially implement any declined case; a correct decline is a
first-class, measured outcome.

#### Scenario: An oblique plane grazing a curved face declines to OCCT (host + parity)

- GIVEN a native solid with a curved face and an oblique plane whose trace on that face is a general ellipse / conic, with the native engine active
- WHEN `cc_split_plane` is invoked
- THEN the native branch SHALL return a NULL piece (the trace is not a circle / line) AND the result SHALL be identical to invoking the same call under `cc_set_engine(0)` (the OCCT oracle), proving fall-through

#### Scenario: A multi-lump or degenerate split declines to OCCT (host)

- GIVEN a plane that severs the solid into more than two connected halves, or that misses the solid / is coincident with a boundary face / leaves a zero-volume sliver, with the native engine active
- WHEN `cc_split_plane` is invoked
- THEN the native branch SHALL return a NULL piece (rather than emit a wrong or leaky piece) AND the engine SHALL fall through to `OcctEngine::split_plane`

#### Scenario: A multi-freeform or foreign operand falls through (host)

- GIVEN a `cc_split_plane` whose operand has more than one freeform wall, OR is a foreign / OCCT-built body with no native B-rep, with the native engine active
- WHEN `cc_split_plane` is invoked
- THEN the native branch SHALL decline / fall through to the OCCT fallback for that call, identical to `cc_set_engine(0)`

