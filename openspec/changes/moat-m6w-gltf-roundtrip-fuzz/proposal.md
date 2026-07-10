# Proposal — moat-m6w-gltf-roundtrip-fuzz (MOAT M6-breadth, the glTF ROUND-TRIP domain)

## Why

The MOAT completeness bar (removing OCCT is gated by *proven* native correctness, not
merely by defaulting to it) has landed a broad set of differential fuzzers over the native
GEOMETRY kernel — booleans, blends, sweeps, sheet-metal, wrap-emboss, N-sided fill — but
the OCCT-FREE **exchange writers** under `src/native/exchange` have only curated,
hand-authored parity tests. In particular the native **glTF 2.0 writer**
(`src/native/exchange/gltf_writer.{h,cpp}`) — the `.gltf` JSON (base64 buffer) and `.glb`
binary container the app hands to iOS RealityKit / QuickLook / SceneKit — is exercised by a
single known-cube/-cylinder facade test (`tests/native/test_native_gltf.cpp`), not by a
*seeded* differential fuzzer that drives *random* meshes and classifies every trial.

This change closes that gap. A glTF export is high-value to certify for three reasons:

1. **It is the runtime asset the app ships to the device.** A structurally-invalid glb, a
   mis-sized accessor, an out-of-range index, or a silently-dropped triangle hands the user
   a corrupted or unrenderable model presented as a valid export. The bar searches the
   input space for exactly that.
2. **A writer has no OCCT equivalence to diff against.** OCCT's `RWGltf` is a *different*
   implementation, not a spec oracle — two conforming glTF writers need not be byte- or
   structure-identical. So the natural arbiter is ROUND-TRIP / SELF-CONSISTENCY, not an
   engine-vs-engine parity, which makes this a HOST-only domain (no SIM leg — see below).
3. **The writer performs non-trivial transforms** — compaction (drop out-of-range /
   negative-index triangles, keep only referenced vertices, reindex, weld by index reuse),
   a mm→m unit scale, fp64→fp32 quantization, and accessor min/max computation — each a
   place a silent-wrong output can hide.

## The oracle (round-trip / self-consistency; no OCCT diff)

A glTF export is a WRITER, and OCCT has no glTF-equivalence guarantee to diff against, so
the arbiter is computed **independently in-test** (no external deps, no glTF reader in-repo)
in two legs:

- **(A) Structural validity.** A self-contained glTF-2.0 + `.glb` validator asserts: glb
  magic/version/total-length + JSON+BIN chunk layout with 4-byte alignment; buffer /
  bufferView `byteLength` + `byteOffset` consistency (offsets 4-aligned + in range, each
  view spanning inside its buffer); accessor `componentType` (5125 UNSIGNED_INT indices /
  5126 FLOAT positions + normals) with `count × element-size == bufferView byteLength`;
  POSITION accessor `min`/`max` EXACTLY the componentwise fp32 extrema of the decoded
  positions; every index in `[0, vertexCount)`; unit normals; and NO NaN/Inf anywhere.

- **(B) Geometry fidelity round-trip (the core DISAGREED check).** The emitted buffers are
  parsed DIRECTLY and compared — vertex count, triangle count, connectivity (index-for-
  index), and bounding box (metres, fp32 tol) — against an INDEPENDENT reimplementation of
  the writer's documented compaction contract (`gltf_writer.h`): drop out-of-range /
  negative-index triangles, keep only referenced vertices reindexed in first-touch order,
  weld by index reuse, scale mm→m (×1e-3). The `.gltf` base64 buffer and the `.glb` BIN
  chunk must decode byte-identical (single-source buffer). Determinism: same mesh + mode →
  byte-identical file.

## Why HOST-only (no SIM leg)

Every prior domain diffs a native SOLID against OCCT under the simulator. A WRITER has no
meaningful OCCT diff — `RWGltf` is a distinct implementation, not a spec oracle, so a
byte/structure comparison would not be a correctness statement. The round-trip
self-consistency arbiter is authoritative and complete on the host, and it can exercise
DEGENERATE inputs (0-triangle, all-bad-indices, huge counts, extreme scales) that the
tessellator can never produce as an *input* to the writer. Manufacturing a weak SIM leg
would add cost with no oracle value, so this domain is intentionally HOST-only.

## Scope

- Adds `tests/native/test_native_gltf_fuzz.cpp` and its CMake wiring under ctest.
- Calls the native writer `gltf_export_mesh` DIRECTLY (no engine, no numsci link).
- `src/native`, `src/engine`, `include`, and the `cc_*` ABI are byte-unchanged — this is
  test-infra only. No product code is modified.

## Non-goals

- No change to the glTF writer, the tessellator, or any geometry code.
- No new `cc_*` ABI surface.
- No OCCT-vs-native parity for the writer (there is no spec oracle to diff against).
