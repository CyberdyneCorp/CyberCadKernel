# Proposal — moat-m2xc-cyl-cyl-common-facade (M2 breadth · HANDOFF from M3 canal)

> **Handoff from the M3 canal-fillet track (`moat-canal-cyl-cyl-fillet`, landed
> `4351cb7`).** The native cyl↔cyl CANAL fillet is LANDED and host-gated, but the app
> cannot reach it end-to-end because a native Steinmetz bicylinder BODY cannot be built
> through the `cc_boolean` facade. This is a BOOLEAN-track gap, not a fillet gap. This
> change scopes the fix. Owner: M2 breadth track.

## Why

The M3 canal fillet rounds the crossing crease of a Steinmetz bicylinder COMMON (two
EQUAL-radius, ORTHOGONAL-axis crossing cylinders). Its host gate builds the bicylinder via
the native SSI boolean DIRECTLY (`nb::ssi_boolean_solid` / `nb::boolean_solid` on
`buildCommonSegment`-constructed cylinders) and the fillet lands watertight. But the SIM
gate — which drives the shipping `cc_*` facade — cannot build the same body:

```
cc_boolean(cylZ, cylX, /*common*/ 2)  →  0
cc_last_error(): "boolean_op: native planar-polyhedron boolean did not produce a verified
                  watertight result for these operands (curved faces or a degenerate/
                  near-tangent configuration are outside the native planar domain)"
```

So the app's `cc_fillet_edges` never receives a native Steinmetz body and the landed native
canal fillet is unreachable through the facade. Closing this gap is the single remaining link
between the landed fillet and the shipping path.

## Reproduction (grounded, current main @ 4351cb7)

- **Works (native, direct):** two axis-aligned cylinders built via
  `nb::curved::buildCommonSegment(box, AxisCylinder{axis, 0,0, Rc, lo, hi})` for axis Z and
  axis X, Rc=1, L=6 → `nb::ssi_boolean_solid(a, b, Op::Common)` and
  `nb::boolean_solid(a, b, Op::Common)` BOTH return a NON-NULL watertight Steinmetz
  (enclosed volume ≈ 16/3·Rc³ = 5.33). This is exactly what the canal host gate uses.
- **Fails (facade path):** the SIM builds each cylinder via
  `cc_solid_extrude_profile` (a kind-2 full-circle profile extruded to length L) and rotates
  one 90° about Y with `cc_rotate_shape_about` to make the X-axis cylinder, then
  `cc_boolean(za, xb, 2)`. Both cylinders build fine (non-zero ids); the COMMON declines with
  the error above.

The distinguishing variables between the two paths are (a) the **construction** of the
operand cylinders (`build_prism_profile`-extruded + `cc_rotate_shape_about`-rotated, vs
`buildCommonSegment`) and (b) the resulting **frame/location** the SSI recognizer sees.

## Suspected root cause (for the owning track to confirm)

`native_boolean.h::boolean_solid` tries `ssi_boolean_solid` FIRST; if it returns non-null it
is used, else the all-planar path runs (and two curved operands → NULL). The engine's
`booleanResultVerified` (native_engine.cpp) then gates the result — for a recognised curved
pair via `ssiCurvedBooleanVerified` (the 16 r³/3 Steinmetz oracle), else a mesh-volume
inclusion–exclusion check. The decline is therefore ONE of:

1. **SSI recognition miss** — `ssidetail::recogniseCurvedSolid` / `steinmetzPreGate` does not
   accept the facade-built cylinders (rotated/located frame, extruded-circle topology, or the
   axis-crossing / equal-radius / orthogonality test failing on the folded location), so SSI
   returns null → planar path → both curved → `boolean_solid` returns NULL. **(Most likely —
   the only difference from the working case is operand construction.)**
2. **Verify rejection** — SSI succeeds but `ssiCurvedBooleanVerified` (or the fallback
   inclusion–exclusion) rejects the result volume (e.g. a deflection-bound miss or a
   `watertightVolume` that is not `isConsistentlyOriented`).

A 20-line host probe that builds the operands BOTH ways (buildCommonSegment vs
build_prism_profile+located rotation) and prints `ssi_boolean_solid`/`boolean_solid`
non-null + watertight + `booleanResultVerified` for each will localize (1) vs (2) in one run.

## What changes

- Make `NativeEngine::boolean_op` (op = common) produce a verified watertight native
  Steinmetz for two equal-radius, orthogonal-axis, crossing cylinders built through the
  facade (`cc_solid_extrude_profile` + `cc_rotate_shape_about`), matching the behaviour that
  already works when the operands come from `buildCommonSegment`. The fix is expected to be
  in `src/native/boolean` (SSI recognition of a located/rotated cylinder operand and/or the
  curved-boolean verify), NOT in the facade or the fillet.
- Keep the honest decline for non-Steinmetz cyl-cyl configs (unequal radius, non-orthogonal,
  non-crossing, near-tangent) → OCCT.

## Impact / verification

- Unblocks the app-facing `cc_fillet_edges` canal arm (the landed `moat-canal-cyl-cyl-fillet`
  builder is already host-gated and needs no change). The SIM parity harness
  `tests/sim/native_curved_fillet_parity.mm::runCanalCase` already has the native-vs-OCCT
  comparison wired behind the body-build; once the native body builds, its `native-note`
  branch becomes a full `mass` + `tessellate` + `occt-parity` pass.
- `src/native/**` stays OCCT-free; `cc_*` ABI additive-only; tessellator untouched.
- Two-gate: host (a verified watertight native Steinmetz from facade-shaped operands, volume
  → 16/3·Rc³) + SIM native-vs-OCCT (`run-sim-native-curved-fillet.sh` canal cases flip from
  `native-note` to a full native pass).
