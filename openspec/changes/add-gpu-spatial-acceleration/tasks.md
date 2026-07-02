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
- [ ] 3.2 GPU frustum-pick: primitives whose AABB intersects the selection
  frustum, returned as a sorted index set. (**ios-sim-run**)
  <br>IMPLEMENTED but NOT EXERCISED on the sim: `frustumPick` (bvh) / `pickFrustum`
  (pick) plus their CPU references are coded, but the current 18-check sim suite
  only asserts ray/nearest-hit parity — no frustum-pick check is run. Follow-up:
  add a frustum-pick parity assertion to `run-sim-gpu-suite.sh`.
- [x] 3.3 Parity: GPU nearest-hit (primitive id + hit point within fp32 tolerance)
  equals the CPU brute-force reference. (**ios-sim-run**)
  <br>Verified: `[GPU] PASS bvh: GPU nearest hit matches brute force (same id + t
  within fp32 tol)` and `[GPU] PASS pick: GPU ray-pick matches CPU reference
  (same id + point within tol)`.
- [ ] 3.4 Parity: GPU frustum pick set equals the CPU brute-force set. (**ios-sim-run**)
  <br>NOT EXERCISED on the sim (depends on 3.2's suite check). The CPU reference
  and GPU frustum kernels exist; the parity assertion is not yet in the suite run.

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
- [ ] 5.1 Fixed tie-break + set-sort: repeated GPU nearest-hit and frustum-pick
  runs on the same scene are reproducible regardless of thread ordering. (**ios-sim-run**)
  <br>Deterministic by construction (documented epsilon + lowest-index tie-break
  for nearest-hit; ascending-sorted frustum set independent of thread order), and
  a design check confirms nearest-hit selection resolves the stacked/miss scene as
  intended, but no explicit repeat-run reproducibility assertion is in the sim
  suite yet. Follow-up: add a repeat-run check.

## 6. Validation
- [ ] 6.1 On-simulator GPU-vs-CPU parity suite (LBVH nearest-hit, frustum pick)
  green within fp32 tolerance. (**ios-sim-run**)
  <br>PARTIAL: LBVH nearest-hit parity is green on the simulator (all 18 checks in
  `run-sim-gpu-suite.sh` pass); the **frustum-pick** leg is not yet asserted in the
  suite (see 3.2 / 3.4).
- [x] 6.2 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 2 +
  change index for `spatial-acceleration`.
