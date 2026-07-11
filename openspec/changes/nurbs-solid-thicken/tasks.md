# Tasks — nurbs-solid-thicken

## 1. Foundation — module + result type
- [x] 1.1 Create `src/native/math/bspline_thicken.h` — `ThickenStatus`, `ThickenResult` (carrying the
      closed `tessellate::Mesh solid` + closure invariants + geometry metrics), and the
      `thickenSurface(S, d, tol, gridU, gridV)` declaration. Reuses the Layer-1 `BsplineSurfaceData`
      input and the `tessellate::Mesh` closed-shell carrier. NOT re-exported from `native_math.h`
      (it depends on `tessellate/mesh.h`, which includes `native_math.h` — a documented cycle avoid).
- [x] 1.2 Guards — well-formed input, `|d|` above the linear tolerance (`ZeroThickness` else),
      non-degenerate parameter domain.

## 2. Compose the offset + sew the closed shell
- [x] 2.1 Offset panel via `offsetSurface(S, d, tol)` — propagate its honest declines
      (`DegenerateNormal` / `SelfIntersection` / fit failure) onto `ThickenStatus`; report the
      achieved offset error + min curvature radius. NEVER build a folded solid.
- [x] 2.2 Sample `S` and its offset locus `O = S + d·N` on a shared `(nu × nv)` grid into two cap
      panels (rational-aware `evalS` + `surfaceNormal`); the offset cap vertices are the true locus so
      they are exactly `|d|` from `S` and exactly matched to the walls.
- [x] 2.3 Four ruled SIDE WALLS joining `S`'s four boundary edges to `O`'s, REUSING the exact shared
      boundary vertices so every seam edge is used by exactly two triangles (watertight by
      construction).
- [x] 2.4 Orient coherently — BFS across shared edges flipping any inconsistent neighbour (declining a
      non-manifold seam), then fix global inside/out by the signed-volume sign so the outward normal is
      consistent and the enclosed volume is positive.
- [x] 2.5 VERIFY closure — `isWatertight` (χ = 2, zero boundary edges) + `isConsistentlyOriented`; a
      shell that fails closure DECLINES (`NotClosed`) — never returned open/leaky. Report the enclosed
      volume + mid-surface area.

## 3. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 3.1 `tests/native/test_native_nurbs_thicken.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_offset`: `_SRC` + `CYBERCAD_TESTS` under `CYBERCAD_HAS_NUMSCI`, plus the
      per-target `target_compile_definitions(... CYBERCAD_HAS_NUMSCI=1)`).
- [x] 3.2 Watertight: the solid is a closed 2-manifold (χ = 2, zero boundary edges, consistently
      oriented) for a flat patch, a curved bump, and a rational quarter-cylinder, at both signs of `d`.
- [x] 3.3 Volume: `Lx·Ly·|d|` EXACTLY (~1e-9) for a flat box; converges to `area·|d|` as `|d| → 0` for
      a curved bump; matches the annular-wedge closed form (~2e-3) for a cylinder shell.
- [x] 3.4 Offset side: every offset-cap vertex is `|d|` from `S` (projected), `offsetError` ==
      `offsetSurface.maxError`, and the original cap lies on `S`.
- [x] 3.5 Fold guard: over-radius thicken of a tight dome DECLINED (`SelfIntersection`, `ok=false`,
      empty solid), reported curvature radius ≈ dome radius; a safe small thicken succeeds and is
      closed; a near-null-normal patch declines; zero thickness declines. No crash.

## 4. SIM native-vs-OCCT parity gate — OPTIONAL FOLLOW-UP (not this pass)
- [ ] 4.1 `tests/sim/native_nurbs_thicken_parity.mm` cross-checking OCCT
      `BRepOffsetAPI_MakeThickSolid` for a couple of thicken cases. HOST is primary and sufficient.

## 5. Docs & close-out
- [x] 5.1 Update `docs/NURBS-SCOPE.md` §2/§4 Layer-5 rows (NURBS offset / thicken): surface offset →
      surface offset + SOLID thicken/shell now partial; robust self-intersecting-shell + multi-face
      shell + rational residual.
- [x] 5.2 Run `openspec validate --all --strict` (pass), full host ctest (87/87 pass, zero regression).
      `cc_*` ABI byte-unchanged (no ABI file touched); `src/native` stays OCCT-free;
      `bspline.h` / `bspline_ops.h` / `bspline_offset.h` / `tessellate/mesh.h` only `#include`d (not
      modified); `ssi/blend/boolean/topology` untouched.
