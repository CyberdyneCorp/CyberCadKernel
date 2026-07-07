# Proposal — moat-m2c-freeform-point-in-solid (MOAT M2c / B3, first slice)

## Why

The native freeform boolean (MOAT M2) declined this session at three nameable,
bounded substrate blockers (`openspec/MOAT-ROADMAP.md` §M2, tracked on
`src/native/boolean/ssi_boolean.h`). **B3** is one of them:

> **B3 — no freeform point-in-solid.** `classifyPoint`
> (`src/native/boolean/ssi_boolean.h:247`) evaluates only the ANALYTIC
> `CurvedSolid` wall (cylinder/sphere/cone radial/spherical/conic half-space +
> planar caps); it has no point-in-freeform-solid classifier, so in/out labelling
> of split fragments is unavailable for a freeform operand.

Today `classifyPoint` takes a `CurvedSolid` — a solid whose curved boundary folds
to ONE analytic elementary surface. `recogniseCurvedSolid` returns `nullopt` the
moment it meets a `Kind::BSpline`/`Kind::Bezier` face
(`default: return std::nullopt; // BSpline / Bezier → freeform → OCCT`). So a solid
whose wall is a genuine trimmed B-spline/NURBS patch has NO membership test at all:
the S5 assembler cannot tag a split fragment IN/OUT/ON against a freeform operand,
and the whole freeform boolean stays blocked.

The enabler this needs already landed: **MOAT M0** meshes a genuinely trimmed
freeform face watertight (`src/native/tessellate/face_mesher.h`
`trimmedFreeformMesh`, consumed through `SolidMesher::mesh`). A watertight boundary
triangulation is exactly the substrate a **ray-cast odd/even-crossing** membership
test runs against. B3 is therefore buildable and self-verifiable **right now,
independently of B1/B2** (roadmap: "B2 ∥ B3 now — two isolated tracks, disjoint
algorithms"), with its OWN standalone OCCT oracle (`BRepClass3d_SolidClassifier`).

This change lands the **FIRST bounded slice** of B3, not a general classifier: a
point-in-freeform-solid test for the SIMPLEST reachable case — a **single
freeform-face solid**, points **comfortably away from the ON band** — proven
(host) against analytic truth on a freeform-walled solid whose inside/outside is
known in closed form, and (sim) against `BRepClass3d_SolidClassifier` on N random
points. Near the ON-boundary / near-tangent tail the classifier self-verifies and
returns an HONEST `UNKNOWN`/ON verdict (never a fabricated crisp IN/OUT); where
robust membership is not reachable it DECLINES with the measured gap. It NEVER
emits a wrong classification silently.

## What Changes

1. **A new header-only, OCCT-free point-in-freeform-solid classifier** in
   `src/native/boolean/` (new file `freeform_membership.h`, namespace
   `cybercad::native::boolean`). Given a **boundary triangle mesh** of a solid
   (produced by the landed M0 `SolidMesher::mesh` — CONSUMED, not rewritten) and a
   query point `p`, it returns `IN` / `OUT` / `ON` / `UNKNOWN` by:
   - **odd/even ray crossings**: cast a ray from `p` in a direction, count
     Möller–Trumbore triangle crossings; odd ⇒ inside, even ⇒ outside;
   - **multi-ray consensus**: cast several independent, non-axis-aligned ray
     directions and require their parity to AGREE. A ray that grazes a shared edge
     or vertex (a degenerate/ambiguous crossing) is discarded; if consensus cannot
     be reached across the ray budget the point is reported `UNKNOWN` (decline)
     rather than guessed;
   - **principled ON-band**: the minimum distance from `p` to any boundary triangle
     is computed; if it is within a scale-relative tolerance band the verdict is
     `ON` (the point is coincident/near-tangent to the meshed surface within the
     mesh's own deflection + tolerance envelope — an honest ON, not a crisp IN/OUT).
   The classifier is **OCCT-free** and **host-buildable** (`clang++ -std=c++20`);
   it introduces **no `cc_*` ABI** surface (internal native library only).

2. **No change to the existing analytic `classifyPoint`** or to
   `recogniseCurvedSolid`: this slice is strictly ADDITIVE and reachable only
   through the new entry point. The analytic `CurvedSolid` path stays byte-identical,
   so every existing native/analytic suite is untouched.

3. **The M0 tessellator is NOT modified.** The classifier consumes the existing
   `tessellate::SolidMesher` / `Mesh` (`src/native/tessellate/mesh.h`,
   `solid_mesher.h`) read-only. No tessellator edit, additive or otherwise.

4. **Two gates prove it (OCCT stays the oracle, never removed):**
   - **(a) HOST ANALYTIC — no OCCT linked.** Build, on the host, a freeform-walled
     solid whose inside/outside is analytically known — a B-spline surface of
     revolution / extruded B-spline profile that EXACTLY traces a known primitive
     (e.g. a B-spline wall coincident with a cylinder of radius `r`, so the analytic
     truth at any point is the cylinder half-space ∧ cap half-spaces). Mesh it with
     M0, then assert the native classifier's IN/OUT labels match the closed-form
     truth on sample points comfortably away from the ON band.
   - **(b) SIM native-vs-OCCT.** On a booted iOS simulator with OCCT linked, build
     the same trimmed-freeform-walled solid, mesh it with M0, and assert the native
     classifier AGREES with OCCT `BRepClass3d_SolidClassifier` (IN/OUT/ON within a
     tolerance band) on a batch of N random sample points.

If robust membership is NOT reachable for the trimmed-freeform case (e.g. the M0
mesh of the chosen solid is not watertight enough for stable parity, or ray
consensus cannot be made reliable away from the ON band), the change DECLINES
honestly with the measured gap (which gate, how many of N disagreed, by how much)
and the analytic `classifyPoint` path stays as-is — a first-class outcome, not a
failure.

## Impact

- **Affected specs:** `native-booleans` (ADDED requirements only — a new
  point-in-freeform-solid classifier requirement + its host-analytic and
  sim-parity self-verify requirement). No existing requirement is modified or
  removed.
- **Affected code:** NEW `src/native/boolean/freeform_membership.h` (header-only,
  OCCT-free). NEW host test + sim parity test. NO edit to `ssi_boolean.h`
  `classifyPoint`/`recogniseCurvedSolid`, NO edit to `src/native/tessellate/**`,
  NO `cc_*` ABI change, NO OCCT include under `src/native/**`.
- **Out of scope (explicitly deferred):** B1 (freeform operand recogniser),
  B2 (WLine freeform face-split), B4/B5 (freeform weld/assembly), multi-freeform-
  face solids, robust ON-boundary / near-tangent classification (the asymptotic
  tail), and any wiring of this classifier into the S5 boolean assembler. Points
  inside the ON band resolve to `ON`/`UNKNOWN` by design in this slice.
