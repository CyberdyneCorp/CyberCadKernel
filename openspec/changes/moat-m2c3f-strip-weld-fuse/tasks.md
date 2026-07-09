# Tasks — moat-m2c3f-strip-weld-fuse

## 1. Implementation (additive, OCCT-free)
- [x] Add `mfswdetail::splitMiddleBoxFace` to `src/native/boolean/multi_face_strip_weld.h`:
      remove the full-width middle-face cap footprint by splitting the box middle face into
      a TOP piece (`arcM` + box top) and a BOTTOM piece (`J1b→J2b` + box bottom), axis-agnostic
      via the junction-column direction + in-plane perpendicular.
- [x] Rewire `appendFuseShell` FUSE path: two END faces via byte-unchanged `notchedBoxFace`
      (single-column notch), MIDDLE face via `splitMiddleBoxFace` (two-column split).
- [x] Confirm CUT/COMMON paths byte-unchanged; `notchedBoxFace`, `strip_split.h`,
      `seam_graph_chain.h`, the M0 tessellator and all `cc_*` ABI untouched (`git diff include/`
      empty; `git diff src/native/tessellate/` empty).

## 2. Gate A — host analytic (no OCCT)
- [x] Extend `strip_weld_lands_watertight_cut_common_at_closed_form_volumes` to include FUSE.
- [x] Add `strip_weld_fuse_lands_watertight_at_union_volume` (watertight, V=V(A∪B) closed-form,
      discriminating union, inclusion–exclusion bound).
- [x] Add `strip_weld_fuse_volume_converges_across_deflection` (monotone convergence to volUnion).
- [x] `tests/native/test_native_chain_seam.cpp` passes 10/10.

## 3. Gate B — sim native-vs-OCCT parity
- [x] Add `#include <BRepAlgoAPI_Fuse.hxx>` and build `BRepAlgoAPI_Fuse` in
      `tests/sim/native_chain_seam_weld_parity.mm`.
- [x] Promote FUSE from honest-NULL fallback to a full `OpCase` alongside CUT/COMMON.
- [ ] Run `scripts/run-sim-native-chain-seam-weld.sh` on a booted iOS simulator; FUSE parity
      passes (volume/area/watertight/Euler/bbox/Hausdorff/classify) at defl 0.01 + 0.005.

## 4. Discipline
- [x] `src/native/**` OCCT-free (no OCCT includes/symbols; only descriptive comments).
- [x] Diff confined to `multi_face_strip_weld.h` + the two test files.
- [x] OpenSpec change validates `--strict`.
