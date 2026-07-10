# Design — moat-m6o-curved-blend-fuzz

## Context

The M6 completeness bar drives random valid inputs through BOTH the native path and the
OCCT oracle and classifies every trial, requiring `DISAGREED == 0`. Fourteen domains
have landed. This session landed the large NEW native CURVED-BLEND surface
(`src/native/blend/{curved_fillet.h, curved_shell.h, curved_offset.h, canal_fillet.h}`)
reached through the shipping `cc_fillet_edges` / `cc_shell` / `cc_offset_face` facade.
The curated `native_curved_{fillet,shell,offset}_parity.mm` harnesses prove both engines
on hand-picked fixtures; this change turns them into a *seeded, randomized batch* with a
per-trial classifier, and adds the coverage the 4th blend domain explicitly deferred.

## The closed-form third oracle (why native-vs-OCCT alone is insufficient)

A curved blend has no single trustworthy oracle: OCCT keeps a TRUE analytic B-rep while
the native arm emits a deflection-bounded PLANAR-FACET weld, so the two differ by a
real, bounded, EXPECTED chord error — not a fault. If native-vs-OCCT were the only test,
a native miss and an OCCT drift would be indistinguishable. The design removes that with
a single principle:

> **Build the base solid from analytic primitives whose exact post-blend geometry has a
> closed form.**

Each base solid (capped cylinder, cone frustum, sphere-cap dome) is an analytic revolve,
so the volume of the filleted / shelled / offset result has an exact closed form:

- **FILLET** a capped cylinder cap rim (convex, ball outside): `V = π·Rc²·(h−r) +
  π·r·[R² + 2Rr·(π/4) + r²·(2/3)]`, `R = Rc−r` (the toroidal quarter-tube solid of
  revolution). The cone/dome-rim fillet has no simple torus closed form, so for those two
  rims the OCCT-via-facade volume is used as the truth proxy (the classifier's
  oracle-trust check degenerates to a self-consistency check; native-vs-OCCT under the
  deflection band remains the arbiter, and the direction/watertight/χ gates still bind).
- **SHELL** (one planar cap open): cylinder `π(Rc²H − (Rc−t)²(H−t))`; cone frustum
  outer−inner (inner inset by `t/cosσ`); sphere-cap dome outer−inner concentric segment.
- **OFFSET** (re-radius the wall by `d`): cylinder `π(Rc+d)²H`; cone
  `πH/3·((Rb+dR)²+(Rb+dR)(Rt+dR)+(Rt+dR)²)` with `dR = d/cosσ` (the wall normal is
  radial-tilted by σ so the RADIUS shifts by `d/cosσ`, not `d`); sphere-cap dome segment
  at concentric radius `Ro+d`.

A native result is AGREED only when it matches the closed form (and, where available, OCCT
concurs); a watertight native solid that misses the closed form is a genuine DISAGREE
regardless of OCCT.

## The `d/cosσ` cone-offset subtlety (localized during bring-up)

The first draft's cone-offset closed-form (and its direct-OCCT oracle) shifted both cap
radii by `d` instead of `d/cosσ`, and a shrinking narrowing cone
(`Rb=4.334 Rt=1.922 H=8.154 d=−0.580`) produced a spurious DISAGREE (native 174.886 vs a
WRONG oracle 178.738, dX=2.16e-2). Inspecting `curved_offset.h` confirmed the NATIVE arm
applies the correct perpendicular offset (`Rref → Rref + d/cosσ`, cap heights fixed) and
that `native_curved_offset_parity.mm` already uses `dR = d·√(1+tanσ²)`. So **native was
more correct than the harness's first oracle** — the classic ORACLE-INACCURATE pattern,
here caught during bring-up and fixed in the harness (test-infra), not the product. The
harness now uses `d/cosσ` for both the closed-form and the direct-OCCT cone oracle, and
the trial AGREES at dX=2.6e-3. The shrink magnitude is bounded so `minR + d/cosσ > 0`.

## The nine families and their sub-shape picks (facade, both engines)

| Family | Base solid (facade) | Blend call | Sub-shape pick (engine-local) | OCCT oracle |
|---|---|---|---|---|
| FILLET cyl | `cc_solid_extrude_profile` circle | `cc_fillet_edges` rim | rim edge z=h | facade `BRepFilletAPI` |
| FILLET cone | `cc_solid_revolve` trapezoid | `cc_fillet_edges` top rim | rim edge y=H | facade `BRepFilletAPI` |
| FILLET dome | `cc_solid_revolve_profile` arc | `cc_fillet_edges` base rim | rim edge y=capOff | facade `BRepFilletAPI` |
| SHELL cyl | cylinder | `cc_shell` top-cap open | cap faces z=H | facade `MakeThickSolid` |
| SHELL cone | frustum | `cc_shell` top-cap open | cap faces y=H | facade `MakeThickSolid` |
| SHELL dome | dome | `cc_shell` base-cap open | cap faces y=capOff | facade `MakeThickSolid` |
| OFFSET cyl | cylinder | `cc_offset_face` wall | wall face r≈Rc | direct `MakeCylinder(Rc+d)` |
| OFFSET cone | frustum | `cc_offset_face` wall | wall face r≈Rb+tanσ·y | direct `MakeCone(Rb+dR,Rt+dR)` |
| OFFSET dome | dome | `cc_offset_face` wall | wall face \|p\|≈Ro | closed-form segment `Ro+d` |

Sub-shape ids are engine-local, so the picker runs under each engine's body (resolved
from `cc_edge_polylines` / `cc_face_meshes` geometry, never a stored id). The dome
base-rim fillet is the tightest family: it AGREES on deep-enough domes (capOff ≤ ~0.5)
and HONESTLY-DECLINES on a shallow spherical cap (native NULL → OCCT); the generator draws
capOff across `[−2, 0.6]` so the family reliably shows real AGREE coverage AND exercises
the honest-decline branch — the envelope is exercised, not the bar weakened.

## The six-way classifier (identical discipline to the landed siblings)

- **AGREED** — native VALID (watertight, χ=2, correct grow/shrink) + volume within the
  deflection band of BOTH the closed form AND OCCT, which also matches the closed form.
- **HONESTLY-DECLINED** — native `cc_*` → 0/invalid (out-of-envelope pose) while OCCT
  ships. First-class, counted separately, never a bar failure.
- **DISAGREED** — native VALID but outside the closed-form truth while OCCT matches it. A
  genuine SILENT WRONG curved blend. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed form while OCCT does not (native
  vindicated). Logged, not a fault, not a bar failure.
- **ORACLE_UNRELIABLE** — a core family whose OCCT oracle misses the closed form AND
  native also missed. FAILS the bar (investigate, never launder).
- **BOTH-DECLINED** — an out-of-envelope pose both engines refuse. Logged.

## The bar

`std::_Exit(0)` IFF `DISAGREED == 0 && ORACLE_UNRELIABLE == 0` with each of the nine core
families ≥1 AGREED, over ≥2 seeds (runner fails if any seed fails), N≥60/seed. The FIXED
bands (volO<2e-2, volX<2e-2, area<4e-2) are NEVER widened. The generator is seeded ONLY by
an explicit `FUZZ_SEED` — no clock, no `rand()` — so the same seed → a byte-identical batch.

## Discipline

`src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay byte-unchanged —
test infrastructure only, driving the facade rather than modifying it. Does not fuzz
`cc_variable_sweep` or the in-progress `src/native/sheetmetal` module (concurrent tracks).
