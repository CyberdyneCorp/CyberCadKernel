# Design — moat-m6w-gltf-roundtrip-fuzz

## The differential (native writer vs an independent self-consistency oracle)

Per trial the harness:

1. **Deterministically generates** a synthetic triangle mesh (fp64 mm vertices + int index
   triples) from one of eleven families, using a splitmix64 → xoshiro256** stream keyed
   ONLY by `FUZZ_SEED` (env-overridable, fixed default). No clock, no `rand()`: same seed →
   byte-identical batch. Two decorrelated streams (`seed` and `seed ^ 0x9E37…`) run per
   invocation for breadth.

2. **Exports twice** through the OCCT-free native writer `gltf_export_mesh` — once as
   `.gltf` (JSON with a base64 `data:` buffer), once as `.glb` (binary container).

3. **Computes the oracle** — an INDEPENDENT reimplementation of the writer's documented
   compaction contract (`oracleCompact`): drop triangles with a negative or out-of-range
   index, keep only referenced vertices reindexed in first-touch order, weld by index reuse,
   scale mm→m. This is written from the header contract, NOT from `gltf_writer.cpp`
   internals, so it is a genuine second implementation.

4. **Validates + round-trips both legs** and classifies the trial.

## Mesh families (breadth)

`box (12 tri)`, `tetrahedron`, `multi-solid (2–4 boxes)`, `random triangle soup`,
`triangle fan`, `single triangle`, `empty (0 tri)`, `out-of-range/negative indices`,
`tiny coordinate scale` (sub-micron mm), `huge coordinate scale` (1e5–1e7 mm), and
`huge triangle count` (grid of quads). These stress the accessor min/max, fp32 precision,
buffer offsets/counts, and the compaction/skip paths respectively.

## Classification

- **AGREED** — writer succeeded, structurally valid (A), and the round-tripped mesh matches
  the independent oracle (B) on both legs. The pass state.
- **HONESTLY-DECLINED** — writer returned false (I/O) or wrote a truly 0-byte file (only
  reachable via an empty path). First-class, never a bar failure. A vertex-bearing
  0-triangle mesh is NOT a decline — the raw writer emits a valid empty-geometry (count:0)
  asset that is validated + round-tripped.
- **DISAGREED** — writer succeeded but the artifact is structurally invalid OR the
  round-tripped mesh does not match the oracle OR the two legs' buffers differ OR a
  re-export is not byte-identical. The silent-wrong failure this harness exists to catch.
  FAILS the bar.
- **ORACLE-INACCURATE** — the writer is CORRECT but the fp64 test-oracle cannot represent a
  case at full precision (two distinct fp64 mm verts collapsing to one on-disk fp32 metre),
  so a stricter compare would false-DISAGREE. Detected + excluded with a one-line inline
  justification; counted, never laundered. (Present as a safety net; does not fire under
  these families — each vertex stays fp32-distinct even at extreme scale.)
- **NATIVE-CHECK-INACCURATE** — the round-trip invariant is provably too strict for a
  legitimate writer behaviour. Reserved, counted, justified inline if it ever fires (none
  observed).

## Bar

Exit 0 IFF **DISAGREED == 0** with real coverage (AGREED > 0) across at least two seeds,
N ≥ 60 per seed. Any DISAGREED prints seed + case index + family so it is reproducible. No
tolerance is widened, no batch silently capped, no trial dropped, to make the bar pass.

## Writer header-docstring nuance (documented, NOT a defect)

`gltf_writer.h` says a null/empty mesh "writes NOTHING and returns true". The raw
`gltf_export_mesh` with a vertex-bearing **0-triangle** input actually emits a fully-formed
`count:0` asset (byteLength:0 buffer, min/max `[0,0,0]`), which is VALID glTF-2.0 and
round-trips correctly (0 verts / 0 tris). The no-op-write path is the FACADE's
(`cc_gltf_export` — its engine tessellate declines a null body first, so the raw writer is
never called with an empty mesh in shipping). This is a doc-vs-code nuance, NOT a native
defect; no product code is changed by this test-infra change.
