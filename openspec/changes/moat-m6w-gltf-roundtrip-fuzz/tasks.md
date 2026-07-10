# Tasks — moat-m6w-gltf-roundtrip-fuzz

## 1. Harness

- [x] 1.1 Add `tests/native/test_native_gltf_fuzz.cpp` with the shared splitmix64 →
      xoshiro256** `Rng` (keyed ONLY by `FUZZ_SEED`), file I/O + little-endian readers, a
      base64 `data:` URI decoder, and minimal JSON scalar/array scrapes for the writer's
      fixed flat document shape.
- [x] 1.2 Implement the INDEPENDENT oracle `oracleCompact` reproducing the writer's
      documented compaction contract (drop out-of-range/negative-index tris, keep referenced
      verts reindexed first-touch, weld by index reuse, scale mm→m), written from the
      `gltf_writer.h` header contract — NOT from the `.cpp` internals.
- [x] 1.3 Implement the STRUCTURAL validator (A): glb magic/version/total-length + JSON+BIN
      chunk layout + 4-byte alignment; buffer/bufferView byteLength+byteOffset consistency;
      accessor componentType + count×element-size == bufferView byteLength; POSITION min/max
      == exact fp32 componentwise extrema; every index in `[0,vertexCount)`; unit normals;
      no NaN/Inf.
- [x] 1.4 Implement the geometry-fidelity ROUND-TRIP (B): parse the emitted buffers, compare
      vertex count / triangle count / connectivity (index-for-index) / bbox (metres, fp32
      tol) against the oracle; require the `.gltf` base64 buffer and the `.glb` BIN chunk to
      decode byte-identical; require a re-export to be byte-identical (determinism).
- [x] 1.5 Implement the eleven mesh families (box / tet / multi-solid / soup / fan / single
      tri / 0-tri empty / out-of-range+negative indices / tiny + huge coordinate scale /
      huge triangle count).
- [x] 1.6 Implement the classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE-INACCURATE / NATIVE-CHECK-INACCURATE), with the fp32-collision ORACLE-INACCURATE
      guard justified inline, and the bar `DISAGREED == 0 AND AGREED > 0`.

## 2. Wiring

- [x] 2.1 Add `test_native_gltf_fuzz` to `CYBERCAD_TESTS` and its `_SRC` mapping in
      `CMakeLists.txt`, mirroring the existing `test_native_*` entries (always-on native
      suite — the writer is compiled unconditionally, no numsci link).
- [x] 2.2 Run under `ctest`; iterate to DISAGREED = 0 over at least two seeds, N ≥ 60 each.

## 3. Docs + validation

- [x] 3.1 Add the `+ **glTF round-trip (…)**` segment to the M6 row of
      `openspec/MOAT-ROADMAP.md`, in the same style as the existing domain segments, before
      the ` fuzzers (0 DISAGREED…` tail. (The `N native domains` total is reconciled at
      consolidation and is left untouched here.)
- [x] 3.2 `openspec validate --all --strict` passes.

## 4. Guardrails

- [x] 4.1 `src/native`, `src/engine`, `include`, and the `cc_*` ABI are byte-unchanged (test
      infra only). Verified by `git diff --stat` restricting product paths.
- [x] 4.2 No tolerance widened, no batch silently capped, no trial dropped to force a pass.
      Any real native bug surfaced is REPORTED, not silently patched.
