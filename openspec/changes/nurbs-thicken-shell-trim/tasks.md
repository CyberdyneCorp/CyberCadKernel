# Tasks — nurbs-thicken-shell-trim

## 1. Single-face interpenetration trim (`thickenTrimmed`)

- [x] 1.1 Add additive fields `trimmed`, `keptU0/1`, `keptV0/1` to `ThickenResult`
  (default-valued; `thickenSurface` sets `kept*` to the full domain, `trimmed = false`).
- [x] 1.2 Factor the shell assembly (sample S + O over a parameter rectangle → caps + walls →
  orient → verify closure) into a shared `assembleShell(surface, grid, d, dom, nu, nv, r)`
  helper so `thickenSurface` (full domain) and `thickenTrimmed` (sub-rectangle) share ONE
  code path — keeping the full-domain output bit-for-bit unchanged.
- [x] 1.3 Implement `thickenTrimmed`: run `offsetSurfaceTrimmed` (Wave-E Jacobian fold
  analysis) to obtain the fold-free rectangle; passthrough-delegate to `thickenSurface` when
  fold-free everywhere; assemble over the kept rectangle when partially folded; honest-decline
  when no fold-free region remains.

## 2. Adjacent-slab overlap trim (`shellTrimmed`)

- [x] 2.1 Add additive fields `trimmed`, `trimmedSeams`, `selfIntersectionFree` to
  `ShellResult` and `SelfIntersecting` to `ShellStatus`.
- [x] 2.2 Add a slab-vs-slab overlap test (`slabsOverlap`): apex extension
  `|d|/(1 + nA·nB)·‖nA+nB‖` versus the per-node face extent from the seam.
- [x] 2.3 Add a robust non-adjacent triangle-pair PIERCING self-intersection scan
  (`hasSelfIntersection`, edge-vs-triangle, scale-relative coplanar skip) as the internal
  verifier.
- [x] 2.4 Factor `thickenPatches`'s body into `buildShell(…, bool trimOverlap, r)`;
  `trimOverlap == false` reproduces the historical output bit-for-bit. With it on, overlapping
  dihedral seams are re-closed as bisector-chamfer mitres and the finished solid is verified
  self-intersection-free before `ok` is set (else `SelfIntersecting` decline).
- [x] 2.5 Add thin public wrappers: `thickenPatches` → `buildShell(false)`, `shellTrimmed` →
  `buildShell(true)`.

## 3. Invariants + tests

- [x] 3.1 Keep `src/native/**` OCCT-free (0 OCCT/Geom/BRep/TK refs in changed files).
- [x] 3.2 `cc_*` ABI byte-unchanged (additive only; no façade edits).
- [x] 3.3 Thicken regression tests: passthrough byte-identity (flat + bump),
  interpenetration-trimmed watertight + self-intersection-free (partial-fold patch),
  fully-degenerate honest-decline (over-radius dome).
- [x] 3.4 Shell regression tests: passthrough byte-identity (coplanar pair + L-shape),
  slab-overlap trimmed to a clean watertight self-intersection-free mitre (large-d L-shape),
  deep-interpenetration honest-decline (sharp concave wedge).
- [x] 3.5 `ctest` green for `test_native_nurbs_thicken` + `test_native_nurbs_shell` (+ no
  regression across the native suite); `openspec validate --changes` passes.
