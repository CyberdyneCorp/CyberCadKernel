# Design — moat-mpf-mass-properties-fuzz (MOAT M6-breadth-6)

Extend the differential-fuzzing completeness bar to a SIXTH native domain — the
**mass-properties query layer** the CyberCad app's MassReadout / Inertia / Measure
panels read (`cc_mass_properties`, `cc_principal_moments`). This is INFRASTRUCTURE: a
seeded harness, not a geometry capability. OCCT `BRepGProp` is the ORACLE; a
closed-form analytic ground truth is the PRIMARY arbiter; the bar is ZERO silent
wrong masses over a seeded batch; an HONEST DECLINE (no valid native mass, or the
native inertia delegation) is first-class. `src/native/**` and `src/engine/**` are
UNTOUCHED — they are the system under test.

## 0. The substrate (verified in source, not assumed)

`src/engine/native/native_engine.cpp`:

- `NativeEngine::mass_properties` (line 693) — for a native body it does NOT call
  OCCT. It meshes the B-rep at `kPropertyDeflection = 0.005` (a body already stored
  as a mesh is measured directly), then:
  - `area = ntess::surfaceArea(mesh)` — `Σ ½|(b−a)×(c−a)|`.
  - `volume = |ntess::enclosedVolume(mesh)|` — divergence-theorem signed-tetra sum,
    magnitude taken (a well-formed native solid meshes outward).
  - centroid — the volume-weighted signed-tetra centroid over the origin fan.
  - `valid = ntess::isWatertight(mesh) ∧ volume > 0`. A non-watertight mesh → no
    valid mass. **This is the honest-decline path we must exercise.**
- `NativeEngine::principal_moments` (line 1305) — `CC_NATIVE_BODY_UNSUPPORTED`:
  it **delegates to the OCCT fallback** for a native body. There is NO native inertia
  computation. Inertia is therefore not differentially fuzzable against a native
  answer; it is a documented native decline.
- `kPropertyDeflection = 0.005` (line 177) with an in-source note that this is tight
  enough that even a twisted ruled loft lands its mesh-derived volume within the
  parity tolerance of OCCT (rotated-square-twist rel ≈ 5.3e-3 < 5e-2). **The
  native mass answer is a mesh discretisation** — this is the crux of the tolerance
  design below.

`include/cybercadkernel/cc_kernel.h`: the native solid builders we drive —
`cc_solid_extrude` (box / n-gon prism), `cc_solid_revolve` (cylinder / cone / sphere
from a rectangle / trapezoid / half-disk profile), `cc_solid_revolve_profile`
(arc-segment revolve), `cc_solid_loft` (coaxial n-gon prismatoid). `CCMassProps` =
`{volume, area, cx, cy, cz, valid}`.

## 1. Why native-vs-OCCT alone is insufficient (the deflection boundary)

Native mass is `f(mesh(solid, 0.005))`; OCCT `BRepGProp` is `f(exact_brep(solid))`.
For a **planar** family (box, n-gon prism, straight loft) the mesh reproduces the
solid EXACTLY (flat faces, no chord error), so native = OCCT = analytic to machine
epsilon and a tight relative tolerance (≈1e-6) is honest. For a **curved** family
(cylinder, sphere, cone) the mesh is a faceted under-approximation, so native differs
from the exact OCCT/analytic value by the **tessellation chord error** — a real,
bounded, expected gap, NOT a native fault. The tolerance for a curved family is
therefore the **deflection-derived convergence bound** (`rel ≈ C · deflection /
featureSize`, empirically ≤ 5e-2 as the in-source note records), and it is
**matched to the deflection, never widened past it**. Holding a curved family to the
planar 1e-6 tolerance would flag correct meshes as wrong; widening the planar family
to 5e-2 would hide real faults. Each family gets the tolerance its meshing warrants.

Consequence: the **closed-form analytic value is the PRIMARY arbiter**, because it is
exact for both the planar and curved families and it lets the classifier attribute a
native-vs-OCCT gap correctly (native mesh-error vs a real bug vs an OCCT outlier).

## 2. The five-way classifier (mirrors the landed siblings)

Same discipline as `native_construct_fuzz.mm` / `native_blend_fuzz.mm`, specialised
to three measured quantities (volume, area, centroid) plus the inertia decline:

| bucket | native valid | native vs analytic | vs OCCT | verdict |
|---|---|---|---|---|
| AGREED | 1 | match (tol) | match | pass |
| HONESTLY-DECLINED | 0 | — | OCCT valid | pass, logged |
| DISAGREED | 1 | **mismatch** | — | **FAIL** |
| ORACLE-INACCURATE | 1 | match | mismatch | pass, logged (OCCT outlier) |
| BOTH-DECLINED | 0 | — | OCCT also invalid | pass, logged |

The inertia dimension is recorded as HONESTLY-DECLINED for every native body (native
delegates to OCCT). A closed-form-vs-OCCT inertia check MAY run for the exact families
(box `m/12·(...)`, cylinder, sphere `2/5 m r²`, cone) purely as **oracle-trust
telemetry** — logged, never a native differential, never a bar input.

## 3. Family generators and their analytic arbiters

All generators are pure functions of the xoshiro256** stream (splitmix64-seeded by
`FUZZ_SEED`); the SAME parameter POD feeds the native builder and the OCCT builder.

- **BOX** — `cc_solid_extrude` of an axis-aligned rectangle `w×d` to height `h`.
  Exact: `V = w·d·h`, `A = 2(wd+wh+dh)`, centroid at the box centre, inertia
  `m/12·diag(d²+h², w²+h², w²+d²)`. Planar → tight tol.
- **NGON_PRISM** — `cc_solid_extrude` of a regular `n`-gon (n∈[3,12]) radius `r` to
  height `h`. Exact `A_ngon = ½ n r² sin(2π/n)`, `V = A_ngon·h`, area
  `2A_ngon + n·s·h` (`s` = side length), centroid on axis. Planar → tight tol.
- **CYLINDER** — revolve a `r×h` rectangle profile. Exact `V = πr²h`,
  `A = 2πr² + 2πrh`, centroid mid-axis, inertia analytic. Curved → deflection tol.
- **CONE / FRUSTUM** — revolve a trapezoid (`r0`,`r1`,`h`). Exact frustum volume
  `πh/3·(r0²+r0r1+r1²)`, slant area `π(r0+r1)·√((r0−r1)²+h²)` + caps, centroid on
  axis. Curved → deflection tol.
- **SPHERE** — revolve a half-disk radius `r`. Exact `V = 4/3 πr³`, `A = 4πr²`,
  centroid at centre, inertia `2/5 m r²`. Curved → deflection tol.
- **LOFT** — `cc_solid_loft` between two coaxial regular-n-gons (radii `R0`,`R1`,
  gap `Δz`). Prismatoid band volume `Δz/3·(A0 + √(A0 A1) + A1)`, planar-trapezoid
  lateral area closed-form, centroid on axis. Planar sides → tight tol on volume.
- **REVOLVE** — `cc_solid_revolve_profile` of an arbitrary non-self-intersecting
  axial polygon through 2π. **Pappus**: `V = 2π·r̄·A_profile`, lateral
  `A = 2π·r̄·P_profile` (`r̄` = profile centroid radius). Curved → deflection tol.
- **DECLINE-exercisers** (sparse, labelled) — a zero-height or self-touching profile:
  native yields `valid == 0`; OCCT likewise fails → BOTH-DECLINED (or the OCCT-valid
  case → HONESTLY-DECLINED).

## 4. Determinism, coverage, and the bar

- RNG: splitmix64 seeds a xoshiro256** stream from an explicit `FUZZ_SEED`
  (argv[1]/env, fixed default). No clock, no `rand()`. Same seed → byte-identical
  batch (the reused harness helper already guarantees this in the four landed
  fuzzers).
- Bar: exit 0 IFF `DISAGREED == 0` across **≥2 seeds**, with each of BOX / NGON_PRISM
  / CYLINDER / CONE / SPHERE / LOFT / REVOLVE having ≥1 AGREED trial. Any DISAGREE /
  ORACLE-INACCURATE prints seed + case index + family/param + native/OCCT/analytic
  triple.
- No silent caps: a capped or skipped trial is logged; the honest scope block (native
  inertia decline; per-family deflection tolerance) is always printed.

## 5. What this change deliberately does NOT do

- Does NOT touch `src/native/**` or `src/engine/**` — the native mass path is the
  system under test; a surfaced disagreement is REPORTED with its seed, not fixed
  here (a real native-fault fix would be its own change with a regression trial).
- Does NOT fuzz native principal moments as a differential — the native path has no
  independent inertia; that dimension is an honest decline.
- Does NOT widen any tolerance: each family's tolerance is its tessellation
  convergence bound (planar → machine-tight; curved → deflection-derived).
- Does NOT cover freeform-curved inertia or a twisted-loft's curved-side inertia —
  declined honestly (no clean closed form + native has no inertia anyway).

## 6. Reuse (no new machinery invented)

The RNG helper, the AGREE/DECLINE/DISAGREE/ORACLE-INACCURATE/BOTH-DECLINE printer,
the per-family coverage tally, and the OCCT `BRepGProp` measurement block are lifted
from `native_construct_fuzz.mm` (which already measures ThruSections solids by
`BRepGProp`). This harness adds the mass-specific analytic arbiters (§3) and the
centroid comparison, and drives the native mass path via the `cc_*` facade under the
native engine rather than the native construct builders directly.
