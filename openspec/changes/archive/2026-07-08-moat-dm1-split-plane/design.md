# Design — moat-dm1-split-plane (M-DM DM1)

Assemble the FIRST native direct-modeling verb — `cc_split_plane` — by routing
`NativeEngine::split_plane` to ONE watertight piece of a plane cut where the case is
reachable, and preserving the existing OCCT fall-through everywhere else. The geometry
is NOT re-derived: DM1 composes the two already-landed, already-gated verbs
(`freeformHalfSpaceCut`, `boolean_solid`) as additive engine glue. OCCT
(`BRepPrimAPI_MakeHalfSpace` + `BRepAlgoAPI_Cut` / `BRepAlgoAPI_Section`) is the
**oracle + fixture-author + fallback only** — `src/native/**` stays OCCT-free.

## 0. What the substrate already does (verified in source)

- **`src/facade/cc_kernel.cpp` `cc_split_plane(body, ox,oy,oz, nx,ny,nz, keepPositive)`**
  → `active_engine()->split_plane(resolve(body), …, keepPositive)` → `finish_shape`.
  The ABI is a single-piece verb: `keepPositive` selects which half survives. This is
  UNCHANGED by DM1.
- **`src/engine/native/native_engine.cpp` `NativeEngine::split_plane`** today:
  `CC_NATIVE_BODY_UNSUPPORTED("split_plane", body); return fallback().split_plane(...)`
  — an unconditional OCCT fall-through. DM1 inserts a native branch BEFORE the
  fall-through; the fall-through remains the default.
- **`src/engine/occt/occt_feature.cpp` `OcctEngine::split_plane`** builds a plane face,
  a `BRepPrimAPI_MakeHalfSpace` on the DISCARD side, and `BRepAlgoAPI_Cut`s it from the
  body. This is the parity oracle and the fallback.
- **`half_space_cut.h` `freeformHalfSpaceCut(operand, P, KeepSide side, deflection, why)`**
  — returns the one watertight keep-side solid or NULL (with a measured
  `HalfSpaceCutDecline`). `KeepSide::Below` keeps signed distance `≤ 0` along `P.pos.z`;
  `Above` keeps `≥ 0`. Both sides landed and are complementary
  (`V(Below)+V(Above)=V(full)` exact).
- **`native_boolean.h` `boolean_solid(a, b, Op)`** — the clean-room BSP/CSG cut/common
  for planar polyhedra and the axis-aligned box⟷cylinder analytic slice; returns a
  watertight native `Solid` or NULL.
- **`native_engine.cpp` `watertightVolume(shape)`** — the engine's mandatory audit:
  meshes the candidate through the deflection ladder and returns the enclosed volume,
  or a negative sentinel if not watertight. DM1's per-piece self-verify reuses it.

## 1. Keep-side mapping (the one ABI subtlety)

`cc_split_plane` names the surviving half by `keepPositive` relative to the plane
NORMAL `n = (nx,ny,nz)`. `freeformHalfSpaceCut` names it by `KeepSide` relative to the
plane frame's `P.pos.z`. Build `P` with `P.pos.origin = o`, `P.pos.z = normalize(n)`,
so the two agree:

```
keepPositive != 0   ⇒  keep the +n half  ⇒  KeepSide::Above   (signed dist ≥ 0)
keepPositive == 0   ⇒  keep the −n half  ⇒  KeepSide::Below   (signed dist ≤ 0)
```

For the analytic BSP path the same `P` defines the DISCARD half-space box: the box
covers the side NOT kept, so `boolean_solid(operand, halfSpaceBox, Op::Cut)` removes
the discard half and caps the section on `P` — the native mirror of OCCT's
`MakeHalfSpace` + `Cut`.

## 2. Domain dispatch (simplest reachable cases first)

```
NativeEngine::split_plane(body, o, n, keepPositive):
  if body not a native solid            → decline → fallback().split_plane   (unchanged)
  P ← plane(origin=o, z=normalize(n));  side ← keepPositive ? Above : Below
  operand ← native B-rep of body

  A. operand has exactly ONE freeform (Bezier/BSpline) wall  [M2 domain]:
       piece ← freeformHalfSpaceCut(operand, P, side, defl, &why)
       (NULL on any HalfSpaceCutDecline → decline)
  B. operand all-planar polyhedron (every face Kind::Plane):
       box ← halfSpaceBox(P, discardSide)              // bbox-scaled planar half-space
       piece ← boolean_solid(operand, box, Op::Cut)     // landed BSP cut, capped
  C. [DEMOTED TO DECLINE — diagnosis] operand an axis-aligned cylinder, n ∥ axis:
       the perpendicular cut is geometrically `cyl − box`, which the landed curved slice
       (`curved::tryBoxCylinder`) EXPLICITLY EXCLUDES (it supports `box − cyl` only), so
       `boolean_solid` returns NULL → the self-verify falls through to an honest decline.
       Adding a native cylinder-slice verb would violate the consume-unchanged discipline,
       so DM1 leaves this as a first-class decline. (Dispatch B still tries it and NULLs.)
  else → decline → honest error (a native void is NEVER handed to OCCT)

  // mandatory per-piece self-verify (reuse watertightVolume)
  if piece.isNull() OR watertightVolume(piece) ≤ 0     → decline → fallback().split_plane
  return piece
```

`halfSpaceBox(P, discardSide)` sizes the box from the operand bounding box (like OCCT's
`L = 2·(diag + 1)`), so the section rectangle strictly overspans the operand and the
BSP cut caps exactly on `P`. It is a native planar box built through the existing
construction path — no OCCT.

## 3. Self-verify → OCCT fallback (never a leaky piece)

A returned piece is accepted native ONLY when it is a closed watertight 2-manifold with
positive enclosed volume (`watertightVolume(piece) > 0`). This catches every failure
mode the two verbs can hit near their domain edge (a grazing curved trace, a sliver, a
mis-capped section).

[CORRECTED — diagnosis] The decline behaviour is asymmetric by body kind, because a
native `EngineShape` is a type-erased `NativeShape` void that OCCT's `unwrap` would
misread (the exact hazard `CC_NATIVE_BODY_UNSUPPORTED` guards, and the reason
`boolean_op` reports a clean error rather than forwarding):
  * an **OCCT body** → `fallback().split_plane(...)` BYTE-IDENTICAL (unchanged);
  * a **native body** the composition cannot verify → the SAME clean error the pre-DM1
    unconditional path returned (`"operation not supported on a native body yet:
    split_plane …"`). DM1 NEVER hands a native void to OCCT and NEVER emits an
    unverified piece. The app rebuilds under the OCCT engine if it wants the OCCT split.
This makes the change byte-identical for every case except the newly-supported native
successes — the true zero-regression guarantee.

[SUBSTRATE GATING — required for the always-compiled engine to link without NUMSCI]
`split_plane.h` composes `freeformHalfSpaceCut`, whose freeform-wall seam trace
`ssi::trace_intersection` is DEFINED only under `CYBERCAD_HAS_NUMSCI`. Because
`native_engine.cpp` is ALWAYS compiled (host, no-OCCT, no-NUMSCI configs included), both
its `#include "native/boolean/split_plane.h"` and the native-split body are wrapped in
`#ifdef CYBERCAD_HAS_NUMSCI`; when the substrate is OFF the native branch is absent and
`split_plane` falls straight to the honest decline (the pre-DM1 behaviour). Without this
guard the non-NUMSCI engine link fails with an undefined `trace_intersection` — the
regression this design closes. The analytic paths (`boolean_solid`, `build_prism`) carry
no NUMSCI dependency and are unaffected.

## 4. The two gates

**Gate (a) — HOST analytic, OCCT-free (partition-closure oracle).** For each reachable
fixture, split by a plane and compute BOTH pieces via two calls (`keepPositive` 0 then
1). Assert, with no OCCT:

- each piece is watertight (`watertightVolume > 0`);
- `V(below) + V(above) = V(whole)` within the deflection tolerance (the closed-form
  partition-closure the roadmap promises);
- each piece matches its closed-form volume WHERE KNOWN — axis-aligned box: fp-exact
  half-volumes; axis-aligned cylinder cut ⊥ axis: `π·r²·h_i`; bowl-lidded prism: the
  landed closed-form band `∫∫ (H0 + a(x²+y²)) dA` over each half.

**Gate (b) — SIM native-vs-OCCT parity.** On a booted iOS simulator, for each keep
side, reconstruct the operand in OCCT, cut it by the same plane
(`BRepAlgoAPI_Section` / the OCCT `split_plane` two-sided `Cut`), and compare the OCCT
piece against the native piece on per-piece **volume**, **area**, **watertightness**
(closed 2-manifold), **topology** (Euler χ = 2, single closed solid), and **bbox** —
within the landed curved-slice tolerances (volume rel ≤ 2e-2, area/bbox tight),
NEVER widened. A native-vs-OCCT discrepancy on a reachable case is a BLOCKER, not a
tolerance relaxation.

## 5. Honest declines (labelled, verified as fall-through — never faked)

| Case | Why outside the slice | Behaviour |
|---|---|---|
| Oblique plane grazing / tangent to a curved face | trace is a general ellipse/conic, not circle/line (the native slice's only traces) | NULL → OCCT |
| Multi-lump piece | a plane severing the solid into > 2 connected halves, or a disconnected half | NULL → OCCT |
| Degenerate | plane misses the solid / coincident with a boundary face / zero-volume sliver | NULL → OCCT |
| Multi-freeform operand | > 1 freeform wall (beyond the M2 single-wall slice) | NULL → OCCT |
| Foreign / OCCT body | no native B-rep to split | fall-through (unchanged) |

Under the native engine each row produces EXACTLY the OCCT result of the same call
(proven by an `cc_set_engine(1)` vs `cc_set_engine(0)` comparison), so a decline costs
the app nothing and is verified, not asserted.

## 6. Discipline

- **Additive-only.** DM1 touches only `NativeEngine::split_plane` (engine glue). The
  two landed verbs and every existing analytic path are byte-identical. No `cc_*` ABI
  change; the OCCT fallback is preserved verbatim.
- **`src/native/**` OCCT-free.** The consumed headers include no OCCT; the engine glue
  lives in `src/engine/native/` and references the native headers, not OCCT.
- **No fabrication / no dead code / no weakened tolerances.** The self-verify is the
  engine's real `watertightVolume` audit; the gates use the closed-form oracle and the
  landed sim tolerances. An honest decline is a first-class, measured outcome.
