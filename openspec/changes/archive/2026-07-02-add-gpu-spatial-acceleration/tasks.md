# Tasks — add-gpu-spatial-acceleration

Verification levels: **host** = the CPU brute-force reference (nearest-hit +
frustum set) runs in the no-OCCT / no-Metal host CTest (CPU fallback path);
**ios-sim-build** = the LBVH + traversal kernels compile for
`arm64-apple-ios16.0-simulator` (`-framework Metal -framework Foundation`, no
OCCT); **ios-sim-run** = the kernels run on the booted simulator GPU
("Apple iOS simulator GPU", runtime-compiled MSL) and the GPU-vs-CPU parity checks
pass — this is the acceptance bar for every parity requirement below.

## 1. Inputs + BVH layout
- [x] 1.1 Define primitive + per-primitive AABB inputs and the linear LBVH node
  layout in shared buffers. (**ios-sim-build**)
- [x] 1.2 CPU brute-force reference: nearest-hit and frustum-inside set with no
  BVH (the parity oracle). (**host**)
  <br>Both `closestHitBruteForce` and `frustumPickBruteForce` (bvh) /
  `cpuPickReference` + `cpuFrustumReference` (pick) exist as the parity oracles.

## 2. GPU LBVH build (BvhBuild)
- [x] 2.1 GPU AABB + Morton-code computation over the scene bounds. (**ios-sim-run**)
- [x] 2.2 Deterministic Morton sort (stable / index tie-break in comparator). (**ios-sim-run**)
- [x] 2.3 Build the linear (stackless) BVH node array as a `ComputeKind::BvhBuild`
  job. (**ios-sim-run**)
  <br>Verified: `[GPU] PASS bvh: LBVH builds on GPU` + `bvh: query path is bound to the Metal backend` in `run-sim-gpu-suite.sh`.

## 3. GPU ray traversal + picking (Picking)
- [x] 3.1 Stackless GPU ray traversal for nearest-hit, with a documented
  epsilon + lowest-index tie-break, as a `ComputeKind::Picking` job (single +
  batched rays). (**ios-sim-run**)
  <br>Verified: `[GPU] PASS bvh: GPU closestHit batch succeeds` and
  `[GPU] PASS pick: GPU batched ray-pick succeeds`.
- [x] 3.2 GPU frustum-pick: primitives whose AABB intersects the selection
  frustum, returned as a sorted index set. (**ios-sim-run**)
  <br>Verified on the sim: `[GPU] PASS pick: GPU frustum-pick succeeds` and
  `[GPU] PASS pick: GPU frustum set is sorted ascending` (strictly-increasing ids)
  in `tests/sim/gpu_pick_check.mm`, exercising `GpuPick::pickFrustum`.
- [x] 3.3 Parity: GPU nearest-hit (primitive id + hit point within fp32 tolerance)
  equals the CPU brute-force reference. (**ios-sim-run**)
  <br>Verified: `[GPU] PASS bvh: GPU nearest hit matches brute force (same id + t
  within fp32 tol)` and `[GPU] PASS pick: GPU ray-pick matches CPU reference
  (same id + point within tol)`.
- [x] 3.4 Parity: GPU frustum pick set equals the CPU brute-force set. (**ios-sim-run**)
  <br>Verified: `[GPU] PASS pick: GPU frustum set equals CPU reference set (same ids)`
  compares `GpuPick::pickFrustum` against `cpuFrustumReference` on a known subset
  {0,1}, plus empty-selection ({}) and all-enclosing (all ids) edge cases — each
  asserted equal to the CPU reference. A guard check confirms the reference is the
  real subset {0,1}, so the equality is not trivially-true.

## 4. Backend routing + fallback
- [x] 4.1 Route BVH build + picking through the compute backend (Metal when
  active, CPU otherwise); precision guard keeps exact fp64 work on CPU. (**host** + **ios-sim-run**)
  <br>`GpuBvh`/`GpuPick` take a `MetalBackend` for the GPU path and fall back to
  the identical fp32 CPU algorithm when the backend is null (`onGpu()`/`usesGpu()`);
  the fp32-only Metal backend + Phase-0 precision guard keep fp64 on the CPU.
- [x] 4.2 CPU fallback yields the same nearest-hit / pick set when no GPU backend
  is registered. (**host**)
  <br>Null-backend path runs the CPU brute-force reference the GPU is compared
  against; the 3.3 parity checks establish GPU == CPU.
- [x] 4.3 No `cc_*` signature change; the pick/cull path gains a GPU option
  internally. (**ios-sim-run**)
  <br>No `cc_*` signatures changed. NOTE: the GPU pick/BVH modules are validated
  standalone; they are not yet invoked from an existing `cc_*` pick/cull entry
  point (there is no OCCT-side pick path in the facade today), so "gains a GPU
  option internally" is satisfied at the module level only — wiring a facade
  pick/cull call to these modules is a follow-up.

## 5. Determinism
- [x] 5.1 Fixed tie-break + set-sort: repeated GPU nearest-hit and frustum-pick
  runs on the same scene are reproducible regardless of thread ordering. (**ios-sim-run**)
  <br>Verified: `[GPU] PASS pick: GPU ray-pick is byte-identical across 8 runs`
  (primitive id + t + hit point compared bit-for-bit via memcmp, not within a
  tolerance) and `[GPU] PASS pick: GPU frustum-pick is byte-identical across 8 runs`
  (identical sorted id set each run). Confirms the documented epsilon/lowest-index
  tie-break and ascending set-sort make the observable result independent of GPU
  thread ordering.

## 6. Validation
- [x] 6.1 On-simulator GPU-vs-CPU parity suite (LBVH nearest-hit, frustum pick)
  green within fp32 tolerance. (**ios-sim-run**)
  <br>Green: `run-sim-gpu-suite.sh` now runs 26 checks (0 failed), covering LBVH
  nearest-hit parity, the pick-module frustum-pick set parity (subset / empty /
  all-enclosing), sorted-set ordering, and 8x repeat-run determinism for both
  ray-pick and frustum-pick.
- [x] 6.2 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 2 +
  change index for `spatial-acceleration`.
