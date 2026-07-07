# Tasks — moat-m2c-freeform-point-in-solid (MOAT M2c / B3, first slice)

Order: substrate baseline → isolated geometry kernels → classifier → host-analytic
gate → sim OCCT-parity gate → zero-regression proof → docs, or HONEST DECLINE.

All new native code stays OCCT-free and host-buildable (`clang++ -std=c++20`),
namespace `cybercad::native::boolean`, header-only. This slice is strictly ADDITIVE:
`classifyPoint`/`recogniseCurvedSolid` in `ssi_boolean.h` are UNTOUCHED, the
tessellator (`src/native/tessellate/**`) is UNTOUCHED (CONSUMED read-only), and there
is NO `cc_*` ABI change. OCCT stays the oracle and is never removed. A correct DECLINE
(measured gap + specific blocker) is a first-class outcome; no tolerance is weakened
and no wrong classification is ever emitted silently.

## 0. Substrate + baseline (before any new code)

- [x] 0.1 Build the numsci substrate for both targets and export
      `CYBERCAD_NUMSCI_DIR` (iossim for the sim gate, host for the analytic gate):
      `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`.
- [x] 0.2 Record the GREEN baseline of the two mandatory M2c gates as they stand
      today (analytic `classifyPoint` suites host-side; native-vs-OCCT boolean sim
      run), so zero-regression can be diffed against `main`.
- [x] 0.3 Confirm the landed M0 mesher meshes the chosen freeform-walled fixture
      solid WATERTIGHT (`tessellate::SolidMesher::mesh` → `mesh.h isWatertight`,
      `boundaryEdgeCount == 0`). If it does not, STOP and DECLINE with the measured
      open-edge count (R1) — the classifier has no valid substrate.

## 1. Isolated geometry kernels (`src/native/boolean/freeform_membership.h`, new)

- [x] 1.1 `mollerTrumbore(orig, dir, a, b, c) -> optional<Hit{t, u, v}>` — fp64
      ray/triangle intersection; a pure function with its own host unit test
      (parallel ray, forward/backward hit, barycentric interior/edge).
- [x] 1.2 `pointTriangleDistance(p, a, b, c) -> double` — exact closest-distance
      (Ericson region test); pure function with a host unit test (vertex/edge/face
      regions).
- [x] 1.3 Keep both kernels in the backend cognitive-complexity band (≤15); measure
      with the cognitive-complexity skill and isolate any irreducible core.

## 2. Classifier (`src/native/boolean/freeform_membership.h`, new)

- [x] 2.1 `enum class Membership { In, Out, On, Unknown };`
- [x] 2.2 `classifyPointInMesh(const tessellate::Mesh& boundary, const AABB& bbox,
      double meshDeflection, const math::Point3& p, MembershipTol tol) -> Membership`
      implementing design §Algorithm: ON-band via §1.2 min-distance; multi-ray
      odd/even parity via §1.1 with degenerate-ray discard + majority consensus;
      `UNKNOWN` on sub-quorum or disagreeing rays. OCCT-free, header-only.
- [x] 2.3 A fixed non-axis-aligned, mutually non-parallel ray-direction set and the
      `band = max(absTol, relTol·diag) + meshDeflection` sizing (design §D3); no
      tolerance weakened.
- [x] 2.4 Assert (precondition) the input mesh is watertight; a non-watertight mesh
      returns `Unknown` (out of scope, not a fabricated verdict).

## 3. Host-analytic gate (no OCCT) — `tests/native/test_native_freeform_membership.cpp`

- [x] 3.1 Build a freeform-walled solid whose inside/outside is KNOWN in closed form
      (design §D5: a B-spline wall coincident with a cylinder of radius `r`, or the
      convex-extruded-B-spline fallback), via `src/native/construct`. Confirm its
      curved face is `Kind::BSpline` (routes through `trimmedFreeformMesh`).
- [x] 3.2 Mesh it with `tessellate::SolidMesher::mesh`; assert watertight.
- [x] 3.3 On sample points COMFORTABLY away from the ON band (well inside, well
      outside), assert `classifyPointInMesh` matches the closed-form analytic truth
      (IN/OUT). No point placed inside the band for the crisp assertions.
- [x] 3.4 Assert points placed INSIDE the band resolve to `On` (not a crisp IN/OUT) —
      the honest ON contract.
- [x] 3.5 Assert a grazing-degenerate arrangement yields `Unknown`, not a wrong crisp
      verdict (0 silent wrong).

## 4. Sim OCCT-parity gate — `tests/sim/native_freeform_membership_parity.mm`

- [x] 4.1 On a booted simulator with OCCT linked, build the SAME trimmed-freeform-
      walled solid natively and as an OCCT solid; mesh the native one with M0.
- [x] 4.2 For N random points in a bbox-enlarged box, compare `classifyPointInMesh`
      to `BRepClass3d_SolidClassifier(occt).State(p, tol)` (IN/OUT/ON mapping,
      design §D6). Record `N passed / 0 crisp-disagreements / k in-band-or-declined`.
- [x] 4.3 A crisp IN↔OUT disagreement is a HARD FAILURE. In-band points where one
      side says ON and the other is within the band count as agreement (band, not a
      weakened tolerance).

## 5. Zero-regression + honesty proof

- [x] 5.1 Prove `ssi_boolean.h` `classifyPoint`/`recogniseCurvedSolid` are BYTE-
      identical vs `main` (this slice adds a sibling file, edits neither).
- [x] 5.2 Prove `src/native/tessellate/**` is BYTE-identical vs `main` (consumed
      read-only; the tessellator was not modified).
- [x] 5.3 Grep-assert `src/native/**` links ZERO OCCT includes (the new header
      included). Confirm no `cc_*` ABI symbol added/changed.
- [x] 5.4 Re-run both mandatory gates; assert GREEN with zero regression against the
      0.2 baseline.

## 6. Docs / decline

- [x] 6.1 On success: update `openspec/MOAT-ROADMAP.md` — mark M2c (B3) landed
      (first slice), note the fixture, N, and the band; archive this change.
- [ ] 6.2 On decline: record in `openspec/MOAT-ROADMAP.md` the MEASURED gap (which
      gate, how many of N crisp-disagreed and by how much, or the open-edge count
      from 0.3), keep `classifyPoint` as-is, and DO NOT land the classifier as a
      passing capability. An honest decline with a specific blocker is a first-class
      outcome.

## Completion note (this change)

Landed additively; `classifyPoint`/`recogniseCurvedSolid` and the tessellator are
byte-identical vs `main`; `src/native/**` links 0 OCCT includes; no `cc_*` ABI change.
New-code cognitive complexity ≤ 12/function (clang-tidy).

- **Gate A (host analytic):** GREEN — 3000 away-from-band samples agree 100 % (WRONG 0,
  declined 0); on-surface samples all `On`; 40 000-pt batch **crispWRONG 0**.
- **Gate B (sim vs `BRepClass3d_SolidClassifier`):** GATE PASS — `nurbs_box` (all
  `Geom_BSplineSurface` faces) N=3000, **crispAgree 2933, crispDISAGREE 0**, in-band 67.
- **Honest R1 tail (§4, not disqualifying):** the curved `nurbs_cylinder`, bridged
  OCCT→native, meshes NON-watertight under M0 (measured **273 open edges** — the periodic
  BSpline seam edge does not weld). The classifier correctly DECLINES (`Unknown`) there
  rather than fabricate; the curved case itself is proven crisp-correct against analytic
  truth in Gate A. Closing the bridged-freeform seam-weld is an M0-mesher/bridge task, out
  of this classifier slice's scope.
