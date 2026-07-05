# Tasks — add-native-shape-healing (first native shape-healing slice)

Verification levels: **host** = OCCT-free host CTest (deliberately-broken fixtures
healed natively → watertight + valid + expected merge/drop/flip counts + enclosed
volume; the un-healable fixture reports UNHEALED with a measured residual);
**sim** = native-vs-OCCT `BRepBuilderAPI_Sewing` / `ShapeFix_Shell` / `ShapeFix_Solid`
parity on the simulator (same watertight/closed shell, same valid solid, same volume
within tol; un-healable fixture native UNHEALED matches OCCT leaving it open). Healing
is INTERNAL — no `cc_*` entry point is added or exercised; parity is asserted at the
`cybercad::native::heal` C++ boundary, exactly like native-ssi / native-topology.

## 1. Module skeleton + result type
- [x] 1.1 Create `src/native/heal/` with `native_heal.h` (aggregate header +
  namespace doc + the HONEST-SCOPE / asymptotic-completeness caveat) and
  `heal_result.h` (`HealStatus`, `UnhealedReason`, `HealMetrics`, `HealResult`,
  `HealOptions`). OCCT-free (includes only `native/math`, `native/topology`,
  `native/tessellate`). (**host**)
- [x] 1.2 Confirm the module builds under BOTH the default no-OCCT config and
  `CYBERCAD_HAS_NUMSCI` (no substrate dependency — pure spatial-hash + closed-form
  geometry), with no interaction. (**host**)

## 2. Vertex / tolerance unification
- [x] 2.1 `vertex_unify.h` — generalize the `boolean/assemble.h` `VertexPool`
  quantized spatial hash (cell side = `tolerance`, nearest-cell rounding) to
  arbitrary B-rep boundary vertices; near-coincident (≤ `tolerance`) collapse to one
  shared `topology::Vertex`; report `nMergedVerts`; never merge two > `tolerance`
  apart. (**host**)

## 3. Tolerant sewing
- [x] 3.1 `tolerant_sew.h` — edge-pair coincidence match: two edges are one shared
  edge iff their endpoints unified to the same two shared vertices AND their curves
  agree along the span (midpoint proximity + curve kind/geometry) within `tolerance`;
  merge each pair into ONE shared edge referenced by both rebuilt faces; report
  `nMergedEdges`. (**host**)
- [x] 3.2 An edge with no within-tolerance partner stays an unstitched boundary edge
  (candidate hole) — never a fabricated closure; its surviving gap feeds
  `maxResidualGap`. (**host**)

## 4. Degenerate removal
- [x] 4.1 `degenerate.h` — drop zero-length edges (endpoint separation < `tolerance`)
  and sliver / zero-area faces (area < `tolerance²`, or wire < 3 distinct vertices
  after unification); rebuild affected wires/faces; remove fully-degenerate faces;
  report `nDroppedDegenerate`. (**host**)

## 5. Orientation fix
- [x] 5.1 `orient.h` — build face adjacency across shared edges; flood-fill
  orientation (opposite traversal across each shared edge), flipping agreeing
  neighbours; report `nFlipped`. (**host**)
- [x] 5.2 Global sign tie-break: confirm outward by `tessellate::enclosedVolume > 0`;
  flip the whole shell if negative. (**host**)

## 6. Self-verify (mandatory)
- [x] 6.1 `self_verify.h` — tessellate the candidate (`tessellate::SolidMesher` →
  `Mesh`) and assert `isWatertight` AND `enclosedVolume > 0` AND all merges within
  `tolerance` before reporting `Healed`; otherwise `Unhealed` (`SelfVerifyFailed` /
  `OpenShell`) with the input shape UNCHANGED; never report `watertight == true`
  unless the mesh actually closed. (**host**)

## 7. UNHEALED honest-report path (never faked)
- [x] 7.1 Return `Unhealed` with reason + measured `maxResidualGap` for a gap beyond
  tolerance / open shell / non-manifold input / self-verify fail / out-of-scope
  (missing pcurve / self-intersecting wire / freeform); return the input UNCHANGED;
  never auto-relax `tolerance`. (**host**)
- [x] 7.2 Documented that `Unhealed` is the contract with the engine → OCCT fallback
  (`native_heal.h` namespace doc + `NATIVE-REWRITE.md` #4 entry): the caller MUST
  route an `Unhealed` shape to OCCT `BRepBuilderAPI_Sewing` + `ShapeFix`. (**host**)

## 8. Top-level orchestration
- [x] 8.1 `heal.h/.cpp` — `healShell(shape, opts)`: collect faces + boundary
  edges/vertices → vertex unify (2) → tolerant sew (3) → degenerate removal (4) →
  orientation fix (5) → assemble shell/solid → self-verify (6) → `HealResult`. Keep
  each sub-step isolated (cognitive complexity in the systems band; the orchestrator a
  flat pipeline). (**host**)

## 9. Engine-internal native-heal hook + OCCT fallback
- [x] 9.1 `src/engine/native/` — internal `tryNativeHeal(shape, tol)`: run
  `heal::healShell`; keep on `Healed`; else fall through to the OCCT adapter. Reached
  internally (NOT via `cc_*`); no `IEngine`/`cc_*` signature change. (**sim**)
- [x] 9.2 `src/engine/occt/` — OCCT fallback: `BRepBuilderAPI_Sewing` on the face
  soup + `ShapeFix_Shell` / `ShapeFix_Solid`. OCCT confined to `src/engine/occt/`;
  `src/native/**` includes NO OCCT header. (**sim**)

## 10. Fixtures (deliberately broken, one per defect + un-healable)
- [x] 10.1 `soup-cube` (six faces, independent shared edges/corners within tol) →
  watertight, V = 1, `nMergedEdges = 12`. (**host** + **sim**)
- [x] 10.2 `subtol-gap-cube` (adjacent faces pulled 0.4·tol apart) → watertight,
  residual ≤ tol. (**host** + **sim**)
- [x] 10.3 `near-coincident-verts` (scattered corner copies within tol) →
  `nMergedVerts` reduces to true corner count, watertight. (**host** + **sim**)
- [x] 10.4 `degenerate-edge` (zero-length edge in a wire) → `nDroppedDegenerate ≥ 1`,
  watertight. (**host** + **sim**)
- [x] 10.5 `sliver-face` (near-zero-area face inserted) → dropped, watertight, correct
  volume. (**host** + **sim**)
- [x] 10.6 `flipped-face` (one face wound inward) → `nFlipped = 1`, enclosed volume >
  0. (**host** + **sim**)
- [x] 10.7 `open-cube` UN-healable (one face pulled 5·tol away) → `Unhealed`,
  `reason = GapBeyondTolerance | OpenShell`, `maxResidualGap ≈ 5·tol`, shape
  unchanged; sim: OCCT sewing ALSO leaves it open at the same tol. (**host** + **sim**)

## 11. Verification (two gates)
- [x] 11.1 Host suite `tests/native/test_native_heal.cpp` — every in-scope fixture
  heals watertight + valid with the expected metrics; the un-healable fixture returns
  `Unhealed` with a measured residual. No OCCT linked. CTest green under NUMSCI OFF
  AND ON. (**host**)
- [x] 11.2 Sim parity harness `tests/sim/native_heal_parity.mm` +
  `scripts/run-sim-native-heal.sh` — same fixtures built for OCCT, `BRepBuilderAPI_Sewing`
  + `ShapeFix_Shell` / `ShapeFix_Solid`; compare watertight/closed shell, valid solid,
  volume within tol at the `cybercad::native::heal` C++ boundary; un-healable fixture
  native UNHEALED matches OCCT leaving it open. `xcrun simctl list devices booted`.
  (**sim**)
- [x] 11.3 No regressions: `run-sim-suite.sh` still green (SSI S1–S4, S5
  `native-pass=6`, native blends + #6/#7, marching, boolean, construct, STL,
  tessellation), and the tessellator is unmodified. (**sim**)
- [x] 11.4 `openspec validate add-native-shape-healing --strict` green; the first
  healing slice noted in `NATIVE-REWRITE.md` #4 / `ROADMAP.md` with the
  asymptotic-completeness caveat and the measured wins vs OCCT. (**host**)

## Deferred to OCCT `ShapeFix` (NOT in the first healing slice — honest UNHEALED)

These are out of the coincident-within-tolerance / degenerate / orientation family
and stay OCCT-backed, reported UNHEALED with the measured gap, never faked:

- [ ] **beyond-tolerance gap bridging** — a real hole wider than `tolerance` → OCCT.
- [ ] **genuinely open shell** that cannot close within tolerance → OCCT.
- [ ] **missing pcurve reconstruction** (re-projecting a 3D edge onto a face surface)
  → OCCT `ShapeFix`.
- [ ] **self-intersecting-wire repair** → OCCT `ShapeFix`.
- [ ] **general non-coincident / non-degenerate industrial B-rep repair** and
  **freeform-surface re-approximation** → OCCT.
