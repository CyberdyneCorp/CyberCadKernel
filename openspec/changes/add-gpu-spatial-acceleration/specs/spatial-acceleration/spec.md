# spatial-acceleration

## ADDED Requirements

### Requirement: GPU LBVH build
The library SHALL build a linear BVH from Morton codes on the compute backend (as
`ComputeKind::BvhBuild`) — computing per-primitive AABBs and Morton codes, sorting
them deterministically, and emitting a linear node array — using fp32 arithmetic.

#### Scenario: LBVH is built on the GPU
- GIVEN a set of primitives with AABBs, on a booted iOS simulator whose Metal
  device is "Apple iOS simulator GPU"
- WHEN the LBVH build runs on the GPU
- THEN it SHALL produce a linear BVH whose leaves reference every input primitive
  AND whose internal nodes bound their children

#### Scenario: LBVH build is deterministic
- GIVEN the same primitive set built twice on the GPU
- WHEN both builds complete
- THEN the resulting node arrays SHALL be identical (Morton ties broken by a fixed
  index rule)

### Requirement: GPU ray traversal nearest-hit
The library SHALL traverse a ray through the LBVH on the compute backend (as
`ComputeKind::Picking`) to find the nearest-hit primitive, and the result SHALL
equal a CPU brute-force reference that tests every primitive with no BVH. Ties in
hit distance SHALL be broken by a fixed, documented rule (lowest primitive index).

#### Scenario: GPU nearest-hit matches brute force
- GIVEN a scene, an LBVH built over it, and a ray, on the iOS simulator GPU
- WHEN the nearest hit is found by GPU BVH traversal and by CPU brute force
- THEN the GPU SHALL report the same hit primitive id as brute force AND the same
  hit point within the documented fp32 tolerance

#### Scenario: Nearest-hit ties are resolved deterministically
- GIVEN a ray that hits two primitives at the same distance
- WHEN GPU traversal and the CPU reference both apply the tie-break rule
- THEN both SHALL select the same primitive (lowest index) and agree

### Requirement: GPU picking (ray and frustum)
The library SHALL support GPU picking against the BVH: a ray-pick returning the
nearest primitive under a ray, and a frustum-pick returning the set of primitives
whose AABB intersects a selection frustum. Both results SHALL equal the CPU
brute-force reference.

#### Scenario: GPU ray-pick matches brute force
- GIVEN a scene BVH and a pick ray on the iOS simulator GPU
- WHEN the picked primitive is computed on the GPU and by CPU brute force
- THEN both SHALL identify the same primitive

#### Scenario: GPU frustum-pick matches brute force
- GIVEN a scene BVH and a selection frustum on the iOS simulator GPU
- WHEN the set of primitives inside the frustum is computed on the GPU and by CPU
  brute force
- THEN the two sets SHALL be equal (as sorted index lists)

### Requirement: Backend routing with CPU fallback
BVH build and picking SHALL run on the Metal backend when it is active and on the
CPU backend otherwise, selected through the compute-backend registry and precision
guard, with no `cc_*` signature change; exact fp64 modeling work SHALL never be
routed to the GPU.

#### Scenario: Same pick result with or without a GPU backend
- GIVEN a pick query run with a Metal backend registered and again with no GPU
  backend registered
- WHEN the query is evaluated in each case
- THEN both SHALL return the same nearest-hit / pick set AND no `cc_*` signature
  SHALL change

### Requirement: Deterministic GPU spatial queries
GPU spatial queries SHALL be reproducible regardless of GPU thread ordering: using
the fixed distance tie-break and a sorted pick-set output, repeated runs on the
same scene and query SHALL produce identical results.

#### Scenario: Repeated GPU queries are reproducible
- GIVEN the same scene and the same ray/frustum query run twice on the GPU
- WHEN both runs complete
- THEN the nearest-hit primitive and the frustum pick set SHALL be identical
  between runs
