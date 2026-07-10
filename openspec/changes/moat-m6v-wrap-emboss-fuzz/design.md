# Design — moat-m6v-wrap-emboss-fuzz

## Context

The M6 completeness bar drives random valid inputs through BOTH the native path and an
oracle and classifies every trial, requiring `DISAGREED == 0`. The landed
`native_wrap_emboss_fuzz.mm` fuzzes the wrap-emboss CYLINDER base only, by calling the
OCCT-FREE native builder DIRECTLY. A NEW arm landed in `src/native/feature/wrap_emboss.h`:
the F5 **sphere-cap pole boss** — a RAISED circular pole cap on a pure sphere-cap dome
(recognised wholesale), whose exact added volume is the spherical-shell sector
`2π(1−cosφ0)·((R+h)³−R³)/3`, reached through the shipping `cc_wrap_emboss` facade under the
NativeEngine. The curated `native_wrap_emboss_parity.mm` proves both engines on three
hand-picked domes; this change turns it into a *seeded, randomized batch* with a per-trial
classifier over random sphere-cap AND cylinder bases, driven through the FACADE.

## The two oracle regimes (why native-vs-OCCT alone is insufficient)

Wrap-emboss has NO single OCCT API, and OCCT's own `cc_wrap_emboss` implementation
**declines any non-cylindrical face**. So the arbiter is split by base:

### Cylinder base (developable — the control)

The native map is `u = px/R` (arc-length → angle), `v = py + vMid`. A footprint of flat
(shoelace) area `A` covers a `(u,v)` measure `A/R`, so the pad is the radial shell over
that measure between `R` and the target radius. The EXACT curvature-corrected changed
volume is `ΔV = A·|Rout² − R²|/(2R)` (`Rout = R+h` emboss, `R−d` deboss) — the SAME
closed form the landed cylinder fuzzer uses, universal across rectangle and polygon
footprints. Here it is the PRIMARY arbiter and OCCT — the SAME `cc_wrap_emboss` under
`cc_set_engine(0)`, which DOES wrap a cylinder — is a SECONDARY cross-check (volume + area),
so both engines and the closed form must agree.

### Sphere-cap base (NON-developable — the gap this change closes)

A sphere base is non-developable: no flat footprint area maps to a boss of volume
`A·h`. The native arm's tractable slice is an AXISYMMETRIC pole boss — raise a circular
pole-cap patch of half-angle `φ0 = ρ/R` (ρ = the footprint bbox in-radius) radially from
`R` to `R+h`. Its EXACT added volume is the spherical-shell sector
`ΔV = 2π(1−cosφ0)·((R+h)³−R³)/3`. Because OCCT's `cc_wrap_emboss` DECLINES the sphere wall,
this closed form is the SOLE arbiter for the boss increment. It is grounded by adding it to
the base-dome volume that OCCT DOES measure exactly (`cc_mass_properties`/BRepGProp on the
revolved dome — an independent OCCT reference for the BASE, not the wrap), so the native
embossed volume must equal `V_dome(OCCT) + ΔV(closed form)`. The OCCT wrap decline is itself
asserted per sphere trial as a first-class negative reference.

## Structural invariants beyond volume

Each in-scope native boss is additionally checked for: watertight (position-welded mesh,
matching the parity harness's Euclidean welder — the native mesher's coincident corners can
land a hair apart), Euler characteristic χ = 2 (a single closed genus-0 shell), a strictly
GROWING volume for a raised boss (a real emboss adds material), and mesh-volume ≈
BRep-volume (the tessellated soup matches the mass-properties reading). A native result that
is watertight but wrong-volume is the SILENT-WRONG-RESULT the bar exists to catch.

## Face resolution

The wrap face id must come from the SAME body being embossed (OCCT and native may order the
revolved sphere sectors / the cylinder's faces differently). The cylinder wall is the face
whose mesh vertices all lie at distance ≈ R from the axis; the sphere wall is the face whose
mesh vertices all lie at distance ≈ R from the dome centre — both resolved GEOMETRICALLY via
`cc_face_meshes`, identically under both engines (each meshes the same base).

## Honest scope / declines (logged, counted separately)

- **sphere + recessed** — the native arm has NO sphere-deboss path (F5 is boss=1 only), so
  this cell is a first-class DOMAIN-level decline: the generator still emits it, the native
  `cc_wrap_emboss` returns 0, and the trial is classified BOTH-DECLINED / HONESTLY-DECLINED.
  It is reported in the coverage table, never silently dropped.
- **out-of-envelope exercisers** (a general developable non-cylindrical base, a
  self-intersecting footprint, a boss half-angle reaching the dome rim, a >2π footprint,
  a deboss depth ≥ R) — native returns 0; classified BOTH-DECLINED. A native solid built
  for such an input is a SURPRISE (guard leak), flagged and failing the bar.

## Tolerances (FIXED, never widened)

The native builder facets the whole embossed solid into planar triangles at a fine FIXED
deflection, so its measured volume/area sit a small, deflection-bounded amount below the
smooth closed-form / OCCT solid. The bands are the SAME ones the curated parity harness
proved at these poses: cylinder native/OCCT/closed-form volume ≤ 2e-2, area ≤ 3e-2; sphere
native-vs-closed-form volume ≤ 1.5e-2, mesh-vs-brep ≤ 2e-2. The max observed bias is logged
so the margin is auditable. A per-trial tolerance is NEVER computed.

## Determinism

splitmix64 seeds a xoshiro256** stream keyed ONLY by an explicit `FUZZ_SEED` (argv/env, fixed
default). No clock, `rand()`, address, or thread scheduling enters the batch — same seed and
N ⇒ byte-identical sequence of base/mode/param tuples.

## Alternatives considered

- **Reconstruct the sphere boss in OCCT (fuse a concentric outer sphere-cap sector).** A
  thin curved-shell boolean at the boss rim is cancellation-fragile (the parity harness
  explicitly avoids it, using the base-dome volume + analytic delta instead). Rejected in
  favour of the exact shell-sector closed form grounded on the OCCT base-dome measurement.
- **Extend the existing `native_wrap_emboss_fuzz.mm` in place.** That harness calls the
  native builder DIRECTLY (no facade, no OCCT engine, links only the OCCT boolean toolkits)
  and is cylinder-only by construction. The freeform gap needs the FACADE-under-both-engines
  drive (to assert the OCCT sphere decline) and the whole-kernel link — a separate TU is
  cleaner and keeps the landed cylinder fuzzer byte-unchanged.
