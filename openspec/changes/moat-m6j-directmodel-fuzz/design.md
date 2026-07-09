# Design — moat-m6j-directmodel-fuzz (MOAT M6-breadth-10)

## Context

Tenth differential-fuzzing domain on the MOAT completeness bar. Test-infra only: the
native direct-modeling core (`src/native/boolean/split_plane.h`,
`src/native/directmodel/{project,replace_face,replace_face_general}.h`) and the `cc_*`
facade are the SYSTEM UNDER TEST and stay byte-unchanged. The change adds one harness,
one runner, one SKIP-list line, and the roadmap row bump.

## Goals / Non-goals

- **Goal:** search the direct-model input space (random valid solids × random DM ops) for
  a silent wrong native result, arbitrated by an independent closed form, with zero
  DISAGREED across ≥2 seeds.
- **Non-goal:** fixing any surfaced limitation (the `cc_replace_face` grow oracle gap is
  REPORTED, not fixed). No `src/**` or `include/**` change; no `cc_*` ABI change; no
  tolerance widening.

## Key decisions

### 1. Domain choice: direct-modeling, driven through `cc_*` under both engines

The roadmap M6 REMAINING list names "direct-modeling / section curves". Direct-modeling
is chosen over the other candidates because:

- **Section-curve** would fuzz `cc_section_plane` (native S1 closed-form plane∩quadric)
  vs `BRepAlgoAPI_Section` — valuable, but the native producer and its OCCT oracle already
  have a close per-case parity harness (`native_section_parity.mm`) and the native side is
  itself the closed-form arbiter, so the added fuzz value is lower.
- **HLR** already has a facade-driven `native_hlr_parity.mm`; a fuzz layer is a smaller
  delta.
- **Direct-modeling** has three landed per-op parity harnesses but no fuzz domain, drives
  the real `cc_*` + `cc_set_engine` shipping path (only `native_hlr_parity.mm` does this
  among existing harnesses), and each op has a pristine closed form — the strongest gap.

Unlike the eight internal-C++ fuzzers, this harness does NOT `#include` any native C++
header; it drives the public `cc_kernel.h` facade only, toggling engines with
`cc_set_engine`. This is the same pattern as `native_hlr_parity.mm`.

### 2. Build: whole kernel + OCCT + numsci

The harness links the WHOLE kernel (facade + core + engine[native+occt]) like
`run-sim-native-hlr.sh`, because it toggles both engines through the facade. It
additionally builds and links the **numsci iossim substrate** under
`-DCYBERCAD_HAS_NUMSCI=1` (like `run-sim-native-split-plane.sh`): `NativeEngine::split_plane`
/ `replace_face` compose `freeformHalfSpaceCut`, whose seam trace is guarded by
`CYBERCAD_HAS_NUMSCI` in `native_engine.cpp`. WITHOUT the substrate the native SPLIT/OFFSET
path is compiled out and every such trial honest-declines, so the native ops would never be
exercised — confirmed empirically (0 AGREED for SPLIT/OFFSET until numsci was linked).

### 3. Engine-matched measurement (a real hazard, guarded in the harness)

The shape registry is shared process-wide across engines. `OcctEngine::mass_properties`
unwraps a registry entry as a `TopoDS_Shape*` and calls `BRepGProp::VolumeProperties`; it
guards only against an unknown id, NOT against being handed a NATIVE body (which stores a
`NativeShape*`). Measuring a native shape while the OCCT engine is active dereferences
garbage → SIGSEGV (observed and root-caused via lldb). The harness therefore activates the
OWNING engine before every `cc_mass_properties` / `cc_bounding_box` and restores OCCT after.
This is a harness-side discipline, NOT a product bug to fix here (it is a documented
"same active engine that built the body" contract).

### 4. Closed-form arbiters and the honest ORACLE-INACCURATE / DECLINE scope

- **SPLIT:** axis-aligned box/prism → exact half-space keep-volume; all families →
  partition closure `V(keep+)+V(keep−)==V(whole)` (native cut both ways). Oblique splits
  have no simple keep-volume closed form and are arbitrated by native==OCCT + closure.
  Out-of-scope native split configs (native returns NULL) → HONESTLY-DECLINED.
- **OFFSET:** constant-cross-section families → exact `ΔV==capArea·offset`. The
  `cc_replace_face` OCCT adapter is a half-space CUT (trim-only): a GROW (`offset>0`)
  leaves the OCCT solid un-grown while native matches the exact math → the exact
  ORACLE-INACCURATE signature (native vindicated). A TRIM (`offset<0`) is AGREED. The cone
  frustum's cap-offset ΔV is NOT `capArea·offset` (conical wall changes radius), so the
  cone offset is arbitrated by native==OCCT alone with no false closed-form claim; a native
  cone-cap decline → HONESTLY-DECLINED.
- **PROJECT:** planar face → foot `p−((p−o)·n̂)n̂`; cylinder lateral → axis-radial foot.
  Cone-lateral projection is a first-class native decline (native serves plane/cyl/sphere)
  → HONESTLY-DECLINED with OCCT as the fallback oracle. Face ids differ per engine, so the
  target face is selected GEOMETRICALLY (via `cc_face_axis` for curved walls, a precomputed
  on-face probe for planar caps/sides) so both engines resolve the same geometric face; a
  native curved wall may be seam-split into multiple faces, so the best servable curved
  face is taken.

### 5. Tolerances (FIXED, never widened)

`kVolRel=2e-2`, `kAreaRel=3e-2`, `kBboxAbs=1.5e-2` (native-vs-OCCT, deflection-bounded);
`kMathRel=5e-3` (result-vs-exact-math); `kFootTol=1e-6` (projection). Observed AGREE
residuals are near machine epsilon (`dV`~1e-16..1e-15 for planar cuts/offsets, foot~0 for
projections), far under the tolerances, so the fixed bounds are not close to being strained.

## Risks

- **A DISAGREE would be a real native finding.** It is reported (seed + case index + base
  descriptor), NOT papered over. None occurred across the two default seeds.
- **Cross-engine crash** (see decision 3) — mitigated entirely in-harness; product code
  untouched.

## Migration / rollout

None. Additive test infra behind its own runner + the suite SKIP list.
