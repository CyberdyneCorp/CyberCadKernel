# Tasks — moat-m5b-healing-tail

Order: additive result-type fields → the bounded capping pass → the guarded `heal.cpp`
branch → host tests → sim OCCT-reference parity → docs. All new code stays **OCCT-free**
and host-buildable (`clang++ -std=c++20`), namespace `cybercad::native::heal`. **No
`cc_*` ABI change** (healing is internal). The tessellator is **not** modified. NO
tolerance is weakened; capping is opt-in (default `false`) so both landed slices are
byte-identically preserved. Keep `capPlanarHole` in the systems band (`≤ 25`) via named
helpers.

## 1. Additive result types (`src/native/heal/heal_result.h`)

- [x] 1.1 Add `bool capPlanarHoles = false;` to `HealOptions` with the doc contract:
      when `false` (default) the planar-hole cap pass is a no-op and `healShell` behaves
      identically to the two landed slices; when `true`, a single simple planar hole may
      be capped by one synthesized face, still subject to the mandatory self-verify.
- [x] 1.2 Add `int nCappedFaces = 0;` and `double maxCapPlanarityDev = 0.0;` to
      `HealMetrics` (cap faces synthesized — `≤ 1` this slice; largest coplanarity
      deviation of a capped loop, honestly `≤ tolerance`).
- [x] 1.3 Do NOT add a new `UnhealedReason` — a hole outside the bound (≥ 2 loops,
      non-planar, self-intersecting, branching) stays `OpenShell`; a cap that fails
      self-verify stays `SelfVerifyFailed`. (Confirm in the code comment that `OpenShell`
      already means "boundary edges survive after sewing.")

## 2. Bounded capping pass (`src/native/heal/cap_hole.h`, NEW, header-only)

- [x] 2.1 SPDX `Apache-2.0`; OCCT-free; consumes `native/{math,topology}` + the
      `FaceLoop` / `SewResult` sew types only. File header states the four-layer bound
      (opt-in → single simple cycle → planarity → simple polygon), the mandatory
      self-verify hand-off, the honest `OpenShell` decline, and the asymptotic-tail
      caveat.
- [x] 2.2 `collectBoundaryEdges(sewResult)` — the sides referenced by exactly one face
      (the `EdgePool` boundary tally), as unordered shared-vertex-node pairs.
- [x] 2.3 `traceSingleLoop(boundaryEdges)` — build boundary-vertex adjacency; return the
      one closed simple cycle ONLY when every boundary vertex has exactly two incident
      boundary edges AND they form a single cycle; return "declined" for a branching
      boundary or ≥ 2 disjoint loops.
- [x] 2.4 `bestFitPlane(loop)` + `maxPlaneDeviation(loop, plane)` — Newell normal +
      centroid; planarity holds iff `maxPlaneDeviation ≤ tolerance`. Record the deviation
      for `maxCapPlanarityDev`.
- [x] 2.5 `isSimplePolygon(loop, plane)` — project to the plane; decline if any two
      non-adjacent edges cross (self-intersecting boundary).
- [x] 2.6 `capPlanarHole(soup, sewResult, tol) -> CapResult{ std::optional<FaceLoop>
      cap; bool declined; double planarityDev; }` — emit ONE cap `FaceLoop` (loop corners
      + Newell normal) when all layers pass; heal.cpp appends it to the soup and re-runs
      the existing `sew()`. Cognitive complexity kept in-band via the `detail` helpers
      above (short guarded early-returns).

## 3. Guarded orchestration branch (`src/native/heal/heal.cpp`)

- [x] 3.1 In the `sr.boundaryEdges > 0` block, when `opts.capPlanarHoles == true` (and
      after the optional bridge pass) run `capPlanarHole(...)`; on a cap, append the cap
      `FaceLoop` to the working soup, `sr = sew(soup, tol)`, and record `m.nCappedFaces`
      / `m.maxCapPlanarityDev`. When `capPlanarHoles == false` the block is unchanged
      (dead-guarded — byte-identical).
- [x] 3.2 If boundary edges STILL survive (declined, or a cap that did not close the
      shell), decline with the existing reasons (`OpenShell` / `GapBeyondTolerance` /
      `GapBeyondBudget`). Input returned UNCHANGED.
- [x] 3.3 On a successful cap fall through to the UNCHANGED orient + assemble +
      **mandatory self-verify** tail; a capped candidate that fails self-verify is
      `Unhealed{SelfVerifyFailed}` (no faked closure).

## 4. Host gate (no OCCT) — `native-healing` host suite (`tests/native/test_native_heal.cpp`)

- [x] 4.1 In-scope fixture: unit-cube soup with the `+Z` face removed, `capPlanarHoles =
      true` → `Healed`, watertight + valid, `nCappedFaces == 1`,
      `maxCapPlanarityDev ≤ tolerance`, enclosed volume `= 1.0` (analytic, no OCCT).
- [x] 4.2 Default-off fixture: the SAME missing-`+Z` soup with `capPlanarHoles == false`
      still `Unhealed{OpenShell}`, input unchanged (proves no landed-slice regression —
      keep the existing `heal_open_shell_unhealed` assertion intact).
- [x] 4.3 Two-hole out-of-scope fixture: cube missing two OPPOSITE faces (two disjoint
      planar loops) with `capPlanarHoles = true` → `Unhealed{OpenShell}`, `nCappedFaces
      == 0`, input unchanged (this slice caps exactly one).
- [x] 4.4 Non-planar out-of-scope fixture: missing `+Z` with one top corner lifted out of
      the z=1 plane (the two incident axis-aligned side faces stay planar and keep that
      corner paired, so the single top boundary loop is a simple cycle with residual 0 but
      NON-PLANAR) with `capPlanarHoles = true` → `Unhealed{OpenShell}` (the planarity layer
      is the decisive refusal, `maxResidualGap == 0`), input unchanged. (Removing two
      ADJACENT faces instead orphans the shared corner → a beyond-tolerance gap that
      declines EARLIER as `GapBeyondTolerance`; the lifted-corner fixture isolates the
      planarity layer cleanly.)

## 5. Sim native-vs-OCCT parity — `tests/sim/native_heal_parity.mm`

- [x] 5.1 In-scope pair: build the OCCT reference cap —
      `BRepBuilderAPI_MakeFace(gp_Pln, freeBoundaryWire)` added to the shell +
      `ShapeFix_Shell` / `ShapeFix_Solid` — on the same missing-`+Z` cube; native (with
      `capPlanarHoles = true`) matches OCCT in watertight/closed shell, valid solid, and
      volume within tol, compared at the `cybercad::native::heal` C++ boundary (no
      `cc_*` call).
- [x] 5.2 Out-of-scope pair: native `Unhealed{OpenShell}` on the TWO-hole shell (two
      opposite missing faces) while OCCT sewing / `ShapeFix` also leaves it open → parity
      of decline; engine falls through to `ShapeFix`. (Empirical: OCCT `MakeFace(gp_Pln,
      wire)` tolerates a mildly-non-planar wire and caps it, so a single non-planar hole is
      native-conservative deferral, not a shared decline — verified by the host gate 4.4,
      not asserted on the sim.)
- [x] 5.3 Confirm `run-sim-native-heal.sh` still passes all landed fixtures (capping
      default-off) — total unchanged + the new pair.

## 6. Docs

- [x] 6.1 Update `src/native/heal/native_heal.h` scope block: add the opt-in
      single-planar-hole cap to WHAT-THIS-HEALS, keep the asymptotic-tail caveat (≥ 2
      holes / non-planar / self-intersecting still defer).
- [x] 6.2 Tick the M5 tail line in `openspec/MOAT-ROADMAP.md` (bounded single-planar-hole
      cap landed; pcurve reconstruction, self-intersecting-wire repair, ≥ 2-hole /
      non-planar missing-face, and arbitrary broken industrial B-rep stay asymptotic).

## 7. Validate & archive

- [x] 7.1 `openspec validate moat-m5b-healing-tail --strict`.
- [ ] 7.2 After merge, `openspec archive moat-m5b-healing-tail`.
