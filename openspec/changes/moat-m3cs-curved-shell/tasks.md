# Tasks — moat-m3cs-curved-shell

## 1. Native builder (OCCT-free, additive)
- [x] 1.1 Add `src/native/blend/curved_shell.h` with `detail::recogniseShellBody` — wholesale
  capped-cylinder / capped-frustum recognizer (single coaxial cylinder/cone + axis-normal
  planar caps at exactly two heights), reusing `curved_fillet.h` `cylinderInfo`/`coneInfo`.
- [x] 1.2 Add `detail::removedCapHeight` — the picked cap(s) must resolve to EXACTLY one of
  the two cap heights (both caps / a picked curved wall / zero faces → decline).
- [x] 1.3 Add `detail::buildCurvedShell` — rebuild the hollow tube (outer wall, kept-cap outer
  disk, inner offset wall, kept-cap inner disk, open-end rim annulus) as a planar-facet soup
  sharing N angular samples; positive-cavity guards (Ri > 0, H > t).
- [x] 1.4 Add public `blend::curved_shell(solid, faceIds, faceCount, thickness, deflection)`.
- [x] 1.5 Include `curved_shell.h` from `native_blend.h`.

## 2. Engine wiring
- [x] 2.1 Add `nblend::curved_shell` as candidate #2 in `NativeEngine::shell`, gated by the
  existing `blendResultVerified` SHRINK self-verify (0 < Vr < Vo).

## 3. Gate A — host (analytic, no OCCT)
- [x] 3.1 `cylinder_shell_open_top_closed_form` — watertight + closed-form wall volume.
- [x] 3.2 `cylinder_shell_open_bottom_symmetry` — same wall volume opening either cap.
- [x] 3.3 `cylinder_shell_converges` — error shrinks monotonically as deflection refines.
- [x] 3.4 `frustum_shell_open_top_closed_form` — cone frustum wall volume closed form.
- [x] 3.5 `curved_shell_declines_out_of_scope` — box / both caps / picked wall / no faces /
  t≥radius / t≥height / t≤0.
- [x] 3.6 `curved_shell_declines_stepped_shaft` — multi-cylinder body declines.
- [x] 3.7 Register `test_native_curved_shell` in CMake; full host `ctest` green.

## 4. Gate B — sim (native-vs-OCCT parity)
- [x] 4.1 Add `tests/sim/native_curved_shell_parity.mm` (capped cylinder + narrowing/widening
  frustum), vs OCCT `BRepOffsetAPI_MakeThickSolid` + `BRepGProp` + closed form.
- [x] 4.2 Add `scripts/run-sim-native-curved-shell.sh` (links `TKOffset` for MakeThickSolid).
- [x] 4.3 Run on the booted simulator; all curved-shell cases PASS (10/10).

## 5. Validate
- [x] 5.1 `openspec validate moat-m3cs-curved-shell --strict`.
- [x] 5.2 Confirm `src/native/**` OCCT-free, tessellator untouched, `cc_*` ABI additive-only.
