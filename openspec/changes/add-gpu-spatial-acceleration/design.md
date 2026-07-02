## Context

Selection and culling on large models are latency-critical and scale with
primitive count (`ROADMAP.md` Phase 2: Metal BVH + GPU picking). An LBVH
(linear BVH built from Morton codes) is the standard GPU-friendly acceleration
structure: compute per-primitive AABBs and Morton codes, sort, and emit a linear
hierarchy in parallel; then traverse rays through it. Both stages fit the Phase-0
compute-backend kinds `BvhBuild` and `Picking` and dispatch through the
`add-metal-compute-backend` path.

Picking is a spatial query over AABBs/primitives, not an exact B-rep operation,
so fp32 is acceptable — provided the GPU answer equals a CPU brute-force reference
(check every primitive, no BVH). The exact fp64 modeling core is not involved.

Constraints:
- **ABI stability**: no `cc_*` signature change; the GPU query path is internal.
- **fp32-only GPU**: precision guard keeps exact fp64 work on the CPU
  (`compute_backend.h`); spatial queries are display/interaction tolerant.
- **No OCCT in GPU modules**: LBVH + traversal kernels and their test suite are
  self-contained numeric code, runnable on the iOS simulator GPU.
- **Determinism by default**: a fixed tie-break rule so nearest-hit and pick sets
  are reproducible regardless of GPU thread ordering.

## Goals / Non-Goals

Goals:
- GPU LBVH build (AABB + Morton code + sort + linear hierarchy) as a `BvhBuild`
  job.
- GPU ray traversal for nearest-hit as a `Picking` job (single + batched rays).
- GPU picking: ray-pick (nearest primitive) and frustum-pick (primitives inside a
  selection frustum) against the BVH.
- A CPU brute-force reference oracle and an on-simulator GPU-vs-CPU parity test:
  nearest-hit and pick set equal the reference.
- CPU fallback producing the same answers when no GPU is present.

Non-Goals:
- Native geometry queries against exact B-rep (the BVH is over mesh/primitive
  AABBs, not fp64 surfaces).
- Continuous collision / physics; only static-scene picking + culling here.
- The Metal backend primitives (`add-metal-compute-backend`) and tessellation
  (`add-gpu-tessellation`).

## Decisions

- **LBVH via Morton codes.** Quantize primitive centroids to a Morton code over
  the scene AABB, radix-sort, and build a linear node array (Karras-style). This
  is the canonical parallel BVH build and needs no pointer chasing on the GPU.
- **Linear, stackless traversal.** Traverse the linear node array with a bounded
  loop (escape/skip indices) so the kernel has no recursion and no per-thread
  stack — friendly to Metal threadgroups.
- **Nearest-hit with a deterministic tie-break.** Ray traversal keeps the closest
  `t`; exact-`t` ties (and fp32 near-ties within a documented epsilon) are broken
  by lowest primitive index, so the nearest-hit is reproducible and matches the
  brute-force reference's identical rule.
- **Frustum-pick returns a set.** Frustum queries emit the set of primitives whose
  AABB intersects the frustum; the set (as a sorted index list) must equal the
  brute-force set.
- **Route via `BvhBuild` / `Picking`.** The registry selects Metal or CPU; the
  contract (answer equals brute force) is backend-independent, so callers don't
  branch.
- **CPU brute force is the oracle.** The reference tests every primitive with no
  BVH; the parity test asserts GPU nearest-hit id + hit point (within fp32
  tolerance) and the frustum pick set match, on the simulator GPU.

## Risks / Trade-offs

- **fp32 AABB slack near boundaries.** fp32 bounds can flip a borderline
  inside/outside or near-tie hit vs an fp64 oracle; mitigated by using the same
  fp32 arithmetic in the CPU reference (so parity is exact at fp32) and a
  documented epsilon + index tie-break. Exact geometry is never decided here.
- **Morton-sort determinism.** A stable sort (or index tie-break in the
  comparator) is required so equal Morton codes don't reorder between runs.
- **Traversal-order vs result determinism.** Threads visit nodes in arbitrary
  order; only the final nearest-hit / set is observable and is made
  order-independent by the tie-break and set-sort.
- **Simulator vs device throughput.** Parity is validated on the simulator GPU;
  real speedup on large scenes is an on-device concern (deferred, per Phase 0/1
  acceptance).

## Migration Plan

1. Define primitive/AABB inputs and the LBVH node layout in shared buffers.
2. Implement the CPU brute-force reference (nearest-hit + frustum set) — the
   oracle.
3. Implement GPU LBVH build (AABB, Morton, sort, linear hierarchy) as `BvhBuild`.
4. Implement GPU ray traversal (nearest-hit) and frustum-pick as `Picking`.
5. Add the on-simulator GPU-vs-CPU parity test (nearest-hit id + point, frustum
   set) within fp32 tolerance.
6. Route the internal pick/cull path through the compute backend; CPU fallback
   when no GPU.
7. `openspec validate --all --strict`; update `ROADMAP.md` Phase 2 status.

## Open Questions

- Morton-code bit width (30 vs 63) vs primitive count on mobile — set from the
  parity/scale tests.
- Whether frustum-pick should return AABB-intersecting primitives only, or refine
  to triangle-accurate containment on the CPU for edge primitives (AABB-level
  chosen first; CPU refinement optional follow-up).
