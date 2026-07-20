# Proposal — moat-m3uc-unequal-canal-fillet (MOAT M3, UNEQUAL-radius cyl↔cyl canal fillet)

## Why

The equal-radius Steinmetz canal fillet (`moat-canal-cyl-cyl-fillet`) landed `cc_fillet_edges`
on the crossing crease of two EQUAL-radius orthogonal cylinders, and explicitly deferred the
tail: *"Unequal radii / non-orthogonal / TORUS / elliptical creases still decline → OCCT."*
This change lands the FIRST entry of that tail — **two ORTHOGONAL-axis cylinders of DISTINCT
radii** (the thin cylinder poking fully through the thick one).

Distinct radii change the crease TOPOLOGY, and in the more regular direction. For the thin
cyl A (axis Z, radius `Ra`) and thick cyl B (axis X, radius `Rb`), `Ra < Rb`, the two infinite
walls intersect along `cx=Ra cos u`, `cy=Ra sin u`, `cz=±√(Rb²−Ra² sin²u)`. Because `Rb > Ra`
the radicand is `≥ Rb²−Ra² > 0` for EVERY azimuth `u`, so `cz` never reaches 0: the crease is
**TWO DISJOINT CLOSED LOOPS** (top `cz>0`, bottom `cz<0`), NOT the pinched figure that shares
poles in the equal case. Each loop gets a NON-DEGENERATE canal strip (cross-section bounded
away from zero everywhere) — there is no pole and no corner patch to synthesize.

## What changes

- **`src/native/blend/canal_fillet_unequal.h` (new, header-only, OCCT-free):** recognises the
  body as an unequal orthogonal bicylinder COMMON from its planar-facet soup — recovers the two
  orthogonal axes (cross-products of facet-normal families, area-scored) and TWO DISTINCT radii
  (`Ra`, `Rb`) classified PER FACET BY RADIUS (a facet on the `Ra` cylinder has perpendicular
  distance `Ra` EXACTLY regardless of its planar-facet normal tilt — the robust discriminator a
  hard normal threshold cannot match on coarse caps). The rolling-ball centre sits at distance
  `R0a=Ra−r` from the thin axis and `R0b=Rb−r` from the thick axis (`cz=±√(R0b²−R0a² sin²u)`,
  never a pole). It rebuilds the whole filleted COMMON as a planar-facet soup: two closed canal
  strips (top/bottom loops, G1-tangent to both walls), the thin wall's waist TUBE band, and the
  two thick-wall CAP patches (ring-fanned from the loop centroid on the thick wall). One
  canonical slerp between the two wall radials is consumed bit-identically by the strip and both
  walls, so every seam welds watertight through the existing `nb::assembleSolid` — **no
  tessellator change**. A MANDATORY internal self-verify (consistently oriented + a removed-
  volume bound) rejects any large-radius fold → NULL → OCCT. Public entry
  `blend::unequal_canal_fillet_edge(solid, edgeIds, edgeCount, r, deflection)`.
- **`NativeEngine::fillet_edges` (additive candidate #7):** after the equal-radius canal arm
  (#6), try `nblend::unequal_canal_fillet_edge`, gated by the SAME `blendResultVerified` SHRINK
  self-verify (0 < Vr < Vo). EQUAL radii route to #6; an OCCT body never reaches here (a native
  unequal-bicylinder body exists only on native solids). No dead code.

## Honest scope / declines (→ OCCT)

- The two cylinders must have DISTINCT radii, axes ORTHOGONAL and CROSSING. Equal radii route
  to the Steinmetz canal (`canal_fillet.h`); non-orthogonal / non-crossing axes, a third wall
  family, or any facet on neither cylinder → NULL → OCCT.
- Ring-torus guard: `R0a = Ra − r ≥ r` (thin `Ra ≥ 2r`); strict thin/thick separation
  `R0b > R0a` (disjoint loops), else → NULL.
- Multiple picked edges / zero-or-negative radius → NULL.
- MODELING-CONVENTION GAP (reported, not gated): the idealized perpendicular cross-section
  differs from OCCT's variable-dihedral canal by a small margin; the native volume is gated
  against ITS OWN two-sided self-verify (host) and compared to OCCT with a loose
  deflection+convention tolerance (sim), never a wrong solid.
- BODY-BUILD CAVEAT (measured, honest): the native SSI boolean does build the unequal COMMON,
  but impractically slowly/densely on this pose (~48 s, ~59k facets vs 864 for the equal case) —
  a BOOLEAN-track breadth gap, NOT a fillet gap (the recognizer DOES recognize the real SSI
  soup: radii to 1e-3, zero stray facets). The host gate therefore feeds the fillet builder a
  clean, recognizable unequal-bicylinder soup built directly; the SIM gate builds via OCCT.
  This mirrors the equal canal slice's documented body-build caveat.

## Gates

- **GATE a (host, no OCCT) — GREEN.** `test_native_blend` unequal_canal_fillet_* (5 cases): a
  clean native unequal bicylinder (`Ra=1, Rb=1.5`) fillets at `r ∈ [0.1, 0.4]` to a watertight,
  consistently-oriented (χ=2) solid that REMOVES material (keeps > half the body) and CONVERGES
  as the deflection refines; an ANALYTIC G1 gate proves the strip normal equals each cylinder
  wall's radial at both seams (no OCCT, no mesh); out-of-envelope (box / equal radii / thin
  `Ra < 2r` / `r ≤ 0` / multi-edge) declines to NULL. Full host `ctest` green.
- **GATE b (sim, native-vs-OCCT) — encoded structurally.** The OCCT oracle
  (`BRepPrimAPI_MakeCylinder` ×2 + `Common` + `BRepFilletAPI_MakeFillet` + `BRepGProp`) confirms
  the unequal-bicylinder fillet is real; the native fillet is host-gated and the native
  unequal-bicylinder BODY records the boolean-track build gap honestly (same as the equal
  slice). Runs on a booted simulator.

## Impact

- The equal-radius canal arm (`canal_fillet.h`), the earlier fillet arms, and the byte-frozen
  boolean welds remain **untouched**; the unequal arm is an additive sibling reached only for a
  native unequal-bicylinder body with distinct radii.
- No `cc_*` ABI change. `src/native/**` stays OCCT-free. The tessellator is not modified, so the
  byte-identical firewall is trivially met (zero mesher diff).
