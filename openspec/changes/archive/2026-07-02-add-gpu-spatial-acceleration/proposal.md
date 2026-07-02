## Why

Interactive selection on large models needs fast spatial queries — culling and
picking against many triangles/faces (`ROADMAP.md` Phase 2: Metal BVH +
GPU picking). Building a bounding-volume hierarchy and traversing rays through it
is classic data-parallel, fp32-tolerant GPU work: the LBVH (Morton-code) build
and ray/frustum traversal map directly onto the `add-metal-compute-backend`
dispatch path, and both `ComputeKind::BvhBuild` and `ComputeKind::Picking` already
exist in the Phase-0 interface.

Picking is a spatial query, not an exact-geometry operation, so fp32 is
acceptable: the answer is "which primitive did the ray hit / which are inside the
frustum", and it must equal a CPU brute-force reference. The exact fp64 modeling
core is untouched — this accelerates queries against the mesh/primitive AABBs, not
the B-rep.

## What Changes

- Add a **GPU LBVH build**: compute AABBs + Morton codes for the input
  primitives, sort by Morton code, and build a linear BVH — as a
  `ComputeKind::BvhBuild` job on the compute backend (fp32).
- Add **GPU ray traversal**: traverse a ray (or a batch of rays) through the LBVH
  to find the nearest hit, as a `ComputeKind::Picking` job.
- Add **GPU picking**: ray-pick (nearest primitive under a ray) and frustum-pick
  (primitives inside a selection frustum) against the BVH.
- Route all of it through the Phase-0 compute backend so it runs on Metal when
  available and on the CPU otherwise, with the same result.
- Provide a **CPU brute-force reference** (test every primitive, no BVH) as the
  oracle, and an on-simulator GPU-vs-CPU parity test: the GPU nearest-hit / pick
  set SHALL equal the brute-force reference.

No `cc_*` signature change: picking/culling internals gain a GPU path; query
answers are unchanged.

## Capabilities

### New Capabilities
- `spatial-acceleration`: GPU LBVH (Morton-code) construction, GPU ray traversal
  for nearest-hit, and GPU picking (ray and frustum against the BVH), with results
  matching a CPU brute-force reference. Depends on `metal-backend` (GPU dispatch)
  and `compute-backend` (interface, `BvhBuild`/`Picking` kinds, precision guard).

### Modified Capabilities
<!-- none — spatial-acceleration is additive; it introduces a new query
     acceleration path without changing existing cc_* signatures. -->

## Impact

- **App**: no code change — selection/culling gain a GPU acceleration path;
  answers are identical to the brute-force reference.
- **Build**: LBVH + traversal kernels are self-contained fp32 numeric code
  compiled at runtime as MSL; they do **not** link OCCT, and the GPU test suite is
  OCCT-free.
- **Determinism / precision**: fp32 spatial queries against AABBs; ties broken by
  a fixed, documented rule (e.g. lowest primitive index / nearest-then-lowest-id)
  so nearest-hit is reproducible. The exact fp64 core is untouched.
- **Risk**: fp32 AABB/traversal must not miss or mis-order hits vs brute force;
  the parity test gates this and the CPU path is always available.
