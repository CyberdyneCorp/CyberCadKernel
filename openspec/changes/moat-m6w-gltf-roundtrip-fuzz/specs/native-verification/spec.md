# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random glTF-mesh generator

The glTF round-trip differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL NOT
read any wall clock, `rand()`, `Date`, process id, address, or any other non-deterministic
source; the RNG SHALL be a self-contained integer generator (splitmix64 seeding a
xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED` (env-overridable, with a
fixed default). Re-running the harness with the same seed and batch size SHALL produce a
**byte-identical** sequence of meshes and export modes on any machine.

Each generated trial SHALL be a synthetic triangle mesh (fp64 millimetre vertices + integer
index triples) drawn from a family set that SHALL include, at minimum: an analytic
primitive (box, tetrahedron), a multi-solid shape, a random triangle soup, a triangle fan,
a single-triangle mesh, a vertex-bearing zero-triangle mesh, a mesh with out-of-range and
negative triangle indices, meshes at tiny (sub-micron) and huge coordinate scales, and a
mesh with a large triangle count. Each mesh SHALL be exported through the native
`gltf_export_mesh` writer as BOTH a `.gltf` (JSON with a base64 buffer) and a `.glb`
(binary container).

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the glTF round-trip fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` mesh + export-mode trials
- THEN the two sequences SHALL be byte-identical (same meshes, same modes, same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: A re-export of the same mesh and mode is byte-identical

- GIVEN a generated mesh exported through the native writer in a given container mode
- WHEN the same mesh is exported a second time in the same mode
- THEN the two files SHALL be byte-identical, proving the writer is deterministic

### Requirement: Round-trip self-consistency as the arbiter, computed independently in-test

The harness SHALL arbitrate each glTF trial by ROUND-TRIP / SELF-CONSISTENCY computed
INDEPENDENTLY in-test — because a glTF export is a WRITER with no OCCT equivalence to diff
against (OCCT's `RWGltf` is a different implementation, not a spec oracle) — with no external
dependency and no reliance on the writer's own source internals. The arbiter SHALL have two
legs:

- **(A) Structural validity.** A self-contained validator SHALL parse the emitted `.gltf`
  JSON + base64 buffer and the `.glb` container and SHALL assert: the glb magic, version,
  and total-length; the JSON and BIN chunk layout with 4-byte alignment; buffer and
  bufferView `byteLength` + `byteOffset` consistency (offsets 4-aligned and in range, each
  view spanning inside its buffer); accessor `componentType` (unsigned-int indices, float
  positions and normals) with `count × element-size` equal to the bufferView `byteLength`;
  the POSITION accessor `min`/`max` equal EXACTLY (fp32) to the componentwise extrema of the
  decoded positions; every index within `[0, vertexCount)`; unit-length normals; and NO
  NaN/Inf anywhere in the decoded buffers.

- **(B) Geometry fidelity round-trip.** The emitted buffers SHALL be parsed DIRECTLY and
  compared — vertex count, triangle count, connectivity index-for-index, and bounding box
  (in metres, to fp32 tolerance) — against an INDEPENDENT reimplementation of the writer's
  documented compaction contract (drop out-of-range / negative-index triangles, keep only
  referenced vertices reindexed in first-touch order, weld by index reuse, scale
  millimetres to metres). The `.gltf` base64 buffer and the `.glb` BIN chunk SHALL decode
  byte-identical.

The comparison tolerances SHALL be FIXED (exact fp32 for accessor bounds and per-vertex
positions; a relative fp32 tolerance for the bounding box) and SHALL NOT be widened to force
a pass.

#### Scenario: A structurally-valid export round-trips to the source mesh and is AGREED

- GIVEN a generated in-scope mesh exported as `.gltf` and `.glb`
- WHEN both legs pass the structural validator AND the parsed buffers match the independent compaction oracle on vertex count, triangle count, connectivity, and bounding box, AND the two legs' buffers decode byte-identical
- THEN the trial SHALL be classified AGREED

#### Scenario: A vertex-bearing zero-triangle mesh emits a valid empty-geometry asset

- GIVEN a mesh with vertices but no valid triangles
- WHEN it is exported through the native writer
- THEN the emitted artifact SHALL be a structurally-valid glTF-2.0 asset with zero-count accessors, a zero-length buffer, and `min`/`max` of `[0,0,0]`, and SHALL round-trip to zero vertices and zero triangles — classified AGREED, NOT a decline

#### Scenario: Out-of-range and negative triangle indices are skipped and only referenced vertices survive

- GIVEN a mesh whose triangle list contains negative and out-of-range indices alongside valid triangles
- WHEN it is exported and round-tripped
- THEN the bad triangles SHALL have been skipped, only the vertices referenced by the surviving triangles SHALL appear, reindexed in first-touch order, matching the independent oracle exactly

### Requirement: Classifier, zero-silent-wrong bar, and honest inaccuracy accounting

The harness SHALL classify each glTF trial into EXACTLY ONE bucket at the fixed tolerances:

- **AGREED** — the writer succeeded, both legs pass structural validity, and the
  round-tripped mesh matches the independent oracle.
- **HONESTLY-DECLINED** — the writer returned false (an I/O failure) or wrote a truly
  zero-byte file (reachable only via an empty path string). First-class, logged, NOT a bar
  failure.
- **DISAGREED** — the writer succeeded but the artifact is structurally invalid, OR the
  round-tripped mesh does not match the oracle, OR the two container legs' buffers differ,
  OR a re-export is not byte-identical. A genuine SILENT WRONG export — the failure this
  harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — the writer is CORRECT but the fp64 test-side oracle cannot
  represent the case at full precision (two distinct fp64 millimetre vertices collapsing to
  the same on-disk fp32 metre position), so a stricter compare would false-DISAGREE. The
  writer is faithful to its documented fp32 position contract; the oracle is the over-precise
  one. Detected, excluded, counted, and justified inline — never laundered into a pass.
- **NATIVE-CHECK-INACCURATE** — the round-trip invariant is provably too strict for a
  legitimate writer behaviour. Reserved, counted, and justified inline if it ever fires.

The harness SHALL print a coverage summary — the per-bucket counts across the batch — and
SHALL exit 0 IF AND ONLY IF the bar holds: **DISAGREED == 0** with real coverage
(AGREED > 0) proven across **at least two distinct seeds**, N ≥ 60 per seed. Any DISAGREED
SHALL be reported with its seed, case index, and mesh family so it is reproducible. The
harness SHALL NOT weaken a tolerance, silently cap the batch, or drop trials to make the bar
pass.

#### Scenario: Zero silent-wrong exports across multiple seeds passes the bar

- GIVEN the harness run over at least two distinct seeds
- WHEN no trial is classified DISAGREED and at least one trial is classified AGREED
- THEN the harness SHALL print the per-bucket coverage summary and exit 0

#### Scenario: A structurally-invalid or non-round-tripping export fails the bar and is reported with its seed

- GIVEN a batch containing at least one trial whose export is structurally invalid or does not round-trip to the source mesh
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and mesh family, so the silent-wrong export is reproducible and not laundered into a pass

#### Scenario: An fp32 quantization collision is accounted as ORACLE-INACCURATE, not a native fault

- GIVEN a trial where two distinct fp64 millimetre vertices map to the same on-disk fp32 metre position
- WHEN the classifier attributes the position mismatch against the over-precise fp64 oracle
- THEN the trial SHALL be classified ORACLE-INACCURATE (the writer is faithful to its documented fp32 contract), logged with a one-line justification, and SHALL NOT be counted as a native fault nor fail the bar
