# Proposal — moat-m3co-curved-offset-face (MOAT M3, curved offset-face slice)

## Why

`cc_offset_face` is an app-used M3 curved residual (readiness Class-A, @10 sites): the planar
arm (`offset_face.h`) slides a planar cap along its normal, but it needs an ALL-PLANAR solid
(`PlanarModel`), so a capped **cylinder** — which carries a Cylinder lateral face — declines to
OCCT even when the picked face is the cylinder wall (the exact "offset a curved face" the app
hits). The offset of a cylinder surface is a coaxial cylinder, so re-radiusing the wall is
exactly analytic — no reason to keep OCCT on this path.

## What changes

- **`src/native/blend/curved_offset.h` (new, header-only, OCCT-free):** `detail::cappedCylGeom`
  recognises a PURE capped cylinder WHOLESALE about a picked Cylinder lateral face (every face a
  coaxial cylinder of the SAME axis/radius, or an axis-normal disc plane at exactly two heights);
  `detail::buildCappedCyl` rebuilds it at the new radius `Rc + distance` as a planar-facet soup
  (wall band of N quads + two N-gon disc caps, sharing the SAME N angular samples), welded
  watertight through the existing `nb::assembleSolid` — **no tessellator change**. Public entry
  `blend::curved_offset_face(solid, faceId, distance, deflection)`. Reuses
  `curved_fillet.h::cylinderInfo / ringPoint / sagittaSteps`.
- **`NativeEngine::offset_face` (additive candidate #2):** after the planar arm, try
  `nblend::curved_offset_face`, gated by the SAME `blendResultVerified` self-verify with the
  correctly-signed volume change (grow `d>0` → Vr>Vo, shrink `d<0` → 0<Vr<Vo). A picked planar
  face is served by the planar arm; a cone / sphere / stepped / multi-cylinder body → NULL → OCCT.

## Honest scope / declines (→ OCCT)

- The picked face must be a CYLINDER lateral face of a pure capped cylinder; a planar cap (planar
  arm), a cone / sphere / stepped-shaft / multi-radius / tilted-cap body → NULL.
- `Rc + distance` must stay `> 0` (a shrink cannot invert the tube).
- Zero offset → NULL.

## Gates

- **GATE a (host, no OCCT) — GREEN.** `test_native_blend` curved_offset_* (3 cases): grow and
  shrink a capped-cylinder wall → watertight + consistently-oriented, matching π(Rc+d)²H to the
  deflection bound (grow < 5e-3, shrink < 1e-2), correct grow/shrink direction; planar-cap /
  `Rc+d≤0` / zero / box decline. Full host `ctest` 67/67.
- **GATE b (sim, native-vs-OCCT)** — pending: add a `runOffsetCylCase` to a sim parity harness
  vs OCCT `BRepOffsetAPI` (tracked in tasks).

## Impact

- The landed blend substrate + M0/M1/M2 + the byte-frozen boolean welds remain **untouched**;
  the curved-offset arm is an additive sibling of the planar `offset_face`.
- No `cc_*` ABI change. `src/native/**` stays OCCT-free. The tessellator is not modified, so the
  byte-identical firewall is trivially met (zero mesher diff).
