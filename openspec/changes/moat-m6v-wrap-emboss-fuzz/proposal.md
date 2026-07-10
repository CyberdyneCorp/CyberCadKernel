# Proposal — moat-m6v-wrap-emboss-fuzz (MOAT M6-breadth, the WRAP-EMBOSS freeform-base domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) already has a landed WRAP-EMBOSS differential fuzzer —
`tests/sim/native_wrap_emboss_fuzz.mm` — but it fuzzes the CYLINDER lateral face ONLY:
rectangular pad emboss, rectangular deboss pocket, and convex N-gon emboss/deboss, driven
by calling the OCCT-FREE native builder (`feature::wrap_emboss`) DIRECTLY.

Meanwhile the native wrap-emboss surface grew a NEW arm (`src/native/feature/wrap_emboss.h`
F5): a **freeform sphere-cap pole boss** — a RAISED circular pole cap on a pure sphere-cap
dome, welded watertight, whose added volume is the EXACT spherical-shell sector
`2π(1−cosφ0)·((R+h)³−R³)/3`. This freeform-base arm has only a *curated* per-op parity
harness (`native_wrap_emboss_parity.mm`, three hand-picked domes) and **no seeded
differential fuzzer** that drives *random* sphere-cap bases (and, alongside them, random
cylinder bases) through the SHIPPING `cc_wrap_emboss` facade under BOTH engines and
classifies every trial. This change closes exactly that gap and certifies the freshly-landed
freeform-base emboss at the facade level.

This wrap-emboss freeform-base fuzzer is high-value for three reasons:

1. **It certifies a curved-base arm that just landed** — random sphere-cap domes (random
   R, cap offset, boss half-angle, height), plus random cylinder bases as the developable
   control, none previously under a *randomised* facade-level bar.
2. **A wrong boss is a silent-wrong-result the user cannot detect** — a watertight solid
   with the WRONG embossed volume hands the user a corrupted feature presented as valid.
   The bar searches the parameter space for that.
3. **It is the SHIPPING path, both engines.** It calls the exact public `cc_wrap_emboss`
   facade the app calls, once under OCCT (`cc_set_engine(0)`) and once under the
   NativeEngine (`cc_set_engine(1)`). It shares NO code with the concurrent product-fix or
   sheet-metal-fuzzer tracks; it fuzzes only the STABLE landed `cc_wrap_emboss`.

## The oracles (closed-form the PRIMARY arbiter; OCCT where the base is developable)

Wrap-emboss has NO single OCCT API, and — critically — **OCCT's own `cc_wrap_emboss`
CANNOT wrap a non-cylindrical face**: presented with a sphere wall it DECLINES (returns 0).
So the arbiter is split by base:

- **Cylinder base** (developable — the control). The PRIMARY arbiter is the closed-form
  curvature-corrected changed volume `A·|Rout²−R²|/(2R)` for a wrapped footprint of flat
  area `A` (native map `u=px/R, v=py+vMid`). OCCT participates as a SECONDARY cross-check:
  the same `cc_wrap_emboss` facade under the OCCT engine builds the true cylindrical pad,
  measured by `cc_mass_properties`, so the two engines AND the closed form must agree.
- **Sphere-cap base** (NON-developable — the gap this change closes). OCCT's
  `cc_wrap_emboss` DECLINES the sphere wall, so there is NO OCCT wrap oracle. The
  CLOSED-FORM shell-sector delta `2π(1−cosφ0)·((R+h)³−R³)/3` is the SOLE arbiter for the
  boss increment, added to the base-dome volume that OCCT DOES measure exactly
  (`cc_mass_properties` / BRepGProp on the revolved dome — an independent base reference).
  The OCCT decline is itself asserted (a first-class negative reference: the native arm
  does work OCCT's wrap cannot).

## What changes

- **ADD** `tests/sim/native_wrap_emboss_freeform_fuzz.mm` — a deterministic seeded
  (splitmix64 → xoshiro256**, keyed ONLY by `FUZZ_SEED`) differential fuzzer that, per
  trial: picks a base ∈ {cylinder, sphere-cap} and a mode ∈ {raised (boss=1), recessed
  (boss=0)}; builds a random VALID base solid (`cc_solid_extrude_profile` full circle /
  `cc_solid_revolve_profile` sphere-cap dome) IDENTICALLY under both engines; resolves the
  wrap face id GEOMETRICALLY on the body being embossed; drives `cc_wrap_emboss` under BOTH
  engines with a random pose/footprint/depth; measures volume/area/watertight/Euler χ via
  `cc_mass_properties` + `cc_tessellate`; and classifies AGREED / HONESTLY-DECLINED /
  DISAGREED / ORACLE_UNRELIABLE / BOTH-DECLINED, arbitrated by the closed-form delta
  (PRIMARY) and OCCT (cylinder only). Sparse out-of-envelope poses (general non-cylindrical
  developable, self-intersecting footprint, a sphere DEBOSS, a boss reaching the dome rim)
  exercise the native NULL → OCCT / closed-form-declines honest branch, counted separately.
- **ADD** `scripts/run-sim-native-wrap-emboss-freeform-fuzz.sh` — cloned from
  `run-sim-native-wrap-emboss.sh` (drives the `cc_*` facade under both engines, so it links
  the WHOLE kernel — facade + core + engine[native+occt] + native math — plus the full OCCT
  toolkit set, `TKHLR`/`TKShHealing` retained). Seeded ONLY by `FUZZ_SEED`/argv; default
  N=64 per seed; runs TWO default seeds and fails if either fails.
- **ADD** `native_wrap_emboss_freeform_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
  `main()`, `std::_Exit`).
- **UPDATE** `openspec/MOAT-ROADMAP.md` M6 row: breadth ×20 → ×22 (this domain; the
  concurrent sheet-metal fuzzer also bumps it — reconcile to ×22 at merge).

## The bar

DISAGREED == 0 and ORACLE_UNRELIABLE == 0 across ≥2 seeds, N≥60 trials/seed (the runner
fails if ANY seed fails), each of the four base×mode cells {cylinder,sphere}×{raised,
recessed} covered (with the sphere-recessed cell an honest DOMAIN-level decline — the
native arm has no sphere-deboss path — logged, not silently dropped). The FIXED tolerances
(cylinder native/OCCT/closed-form volume ≤ 2e-2, area ≤ 3e-2; sphere native-vs-closed-form
volume ≤ 1.5e-2, mesh-vs-brep ≤ 2e-2) are NEVER widened to force a pass. An honest native
NULL → OCCT decline is first-class.

## Non-goals / discipline

- `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay
  **byte-unchanged** (test infrastructure only). If the fuzzer surfaces a real native
  wrap-emboss bug it is REPORTED, not fixed here; a native result more correct than OCCT at
  a numeric edge is classified ORACLE-INACCURATE (native vindicated), not DISAGREED.
- Does NOT re-fuzz the cylinder-only families the landed `native_wrap_emboss_fuzz.mm`
  already covers via the direct native builder — this domain drives the FACADE under both
  engines and closes the FREEFORM sphere-cap gap (cylinder kept as the developable control).
- No new geometry capability; no widened tolerance; no `Date.now()`/`rand()`.
