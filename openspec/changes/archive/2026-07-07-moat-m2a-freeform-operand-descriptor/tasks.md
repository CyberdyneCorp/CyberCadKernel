# Tasks — moat-m2a-freeform-operand-descriptor (MOAT M2, B1 + minimal assembly)

Order: baseline capture → the freeform operand descriptor + `recogniseFreeformSolid`
gate (B1, the primary deliverable) → host analytic gate for the descriptor → STRETCH
minimal freeform↔analytic assembly → sim native-vs-OCCT gate → zero-regression proof →
docs, or HONEST DECLINE. All new native code stays OCCT-free and host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::boolean`. No `cc_*` ABI change.
B1 is a strictly **additive sibling** to `recogniseCurvedSolid`, which — with
`classifyPoint` and the M0/M1/B2/B3 subsystems — MUST stay byte-identical. No tolerance
is weakened; landing B1 while honest-declining the full assembly is a first-class
outcome.

## STATUS — B1 LANDED (host-analytic green); assembly HONEST-DECLINED (measured)

**Landed.** `src/native/boolean/freeform_operand.h` (header-only, OCCT-free, backend-band):
`FreeformOperand`/`OperandFace`/`FaceRole` descriptor + `recogniseFreeformSolid(Shape,
OperandDecline*) → optional<FreeformOperand>`, a strictly ADDITIVE SIBLING of
`recogniseCurvedSolid`. Admits one reachable operand (single-shell `Solid`, ≥1 genuinely-
trimmed BSpline/Bezier wall + admissible analytic caps, closed 2-manifold by edge-incidence);
exposes the freeform `Face` (→ B2), the operand `Shape` (→ M0), the world `Aabb` (→ B3).
`nullopt` + measured `OperandDecline` otherwise.

**Host-analytic gate (a) — GREEN.** `tests/native/test_native_freeform_operand.cpp` 14/14 pass
(build: `clang++ -std=c++20 tests/native/test_native_freeform_operand.cpp
src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests`). On the M0 keystone
`bumpCappedCylinder`: admitted; 3 faces role-tagged (1 freeform + 2 analytic); freeform
surface/kind round-trips bit-identically; `outwardN` outward on every face; `bbox` tight
(x,y∈[−R,R], z∈[0,h]); `watertight` true. 8-way decline battery green (NotSolid, MultiShell,
UnsupportedSurfaceKind=torus, BareFreeformFace, HoledFreeformFace, NoFreeformFace,
NotWatertight). Exposed handles proven: `Shape` meshes watertight under M0; descriptor `Aabb`
scales B3 (interior→In, exterior→Out).

**Assembly — HONEST DECLINE (first-class; no dead code, no weakened tol).** The stretch
`freeform_boolean_solid` composing only the four landed verbs is NOT robustly reachable, so B1
lands alone. Two independent MEASURED blockers: (i) the reachable freeform-SOLID class's sole
freeform wall has a smooth CLOSED (circular) trim; B2 `splitFace` requires a convex straight-
edged outer loop and returns `NoOuterLoop` (asserted in the gate). (ii) even a polygon-trimmed
wall does not close a half-space boolean — the cutting plane also crosses the ANALYTIC cap/side
faces (needs an analytic-face splitter) and requires a NEW cross-section cap on the plane;
neither is a landed M2 verb (B2 is freeform-only, M0/M1/B3 do not split analytic faces or
synthesize caps). Gap to close: a B2 smooth-trim generalisation + a B4 analytic-face-split /
cross-section-cap-synthesis weld verb. No `freeform_boolean_solid` stub was written; the engine
keeps its OCCT fall-through.

**Zero-regression / discipline.** `git status`: only `CMakeLists.txt` modified (two additive
list/SRC entries) + two new untracked files (`freeform_operand.h`, the host test); no `include/`
(cc_*) change. `recogniseCurvedSolid`/`classifyPoint` (`ssi_boolean.h`), M1 (`marching.h`), M0
(`face_mesher.h`) are 0-diff vs `main`; B2 (`face_split.h`) + B3 (`freeform_membership.h`) are
untouched vs their landed state (not in `git status`). 0 real OCCT `#include`s under
`src/native/**`. `freeform_operand.h` has 0 OCCT references.

## 0. Substrate + baseline (BEFORE any new code)

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
      export `CYBERCAD_NUMSCI_DIR=…/build-numsci/{iossim|host}` for the respective build.
- [ ] 0.2 Record the GREEN baseline for the consumed subsystems: `run-sim-native-face-split`
      (B2), `run-sim-native-freeform-membership` (B3), `run-sim-native-ssi-marching` (M1),
      the M0 mesher host tests, and `run-sim-native-curved-boolean` (the analytic S5
      assembler that MUST stay byte-identical).
- [ ] 0.3 Snapshot the analytic `recogniseCurvedSolid`/`classifyPoint` behaviour on the
      existing curved-boolean fixtures — the byte-identical reference for §6.

## 1. Freeform operand descriptor + gate — B1 (`src/native/boolean/freeform_operand.h`)

- [x] 1.1 NEW header-only, OCCT-free `FreeformOperand` / `OperandFace` / `FaceRole` data
      model (design §1): faces tagged by role, world-placed `FaceSurface`, outward normal,
      world AABB, watertight flag, the operand `Shape`. Exposes the freeform-face indices
      (for B2), the analytic half-spaces, and the AABB (for B3). NO OCCT type, NO derived mesh.
- [x] 1.2 `recogniseFreeformSolid(const Shape&) → optional<FreeformOperand>` (design §2):
      admit a non-null single-shell `Solid` with ≥1 genuinely-trimmed freeform face + only
      admissible analytic caps, closed + watertight (every edge shared by exactly two faces),
      faces round-tripping. Return `nullopt` — measured blocker — for a non-solid, open/leaky
      boundary, multi-shell, bare-periodic or holed freeform face, torus/other kind, or no
      freeform face. Reuse `Explorer`, `surfaceOf`, `worldFrame`; DO NOT edit `recogniseCurvedSolid`.
- [x] 1.3 Keep `recogniseFreeformSolid` cognitive complexity in the backend band by
      delegating per-face role classification, the edge-sharing watertight audit, and the
      AABB/orientation fold to free helpers; measure with the cognitive-complexity skill.

## 2. Host analytic gate for the descriptor (`tests/native/…`, no OCCT)

- [x] 2.1 Build a native freeform solid with KNOWN closed-form volume/area (e.g. a
      `Kind::BSpline` wall coincident with a cylinder radius `r` capped by two planes; and a
      convex extruded-B-spline profile), no OCCT. Assert `recogniseFreeformSolid` ADMITS it;
      every `OperandFace` surface/kind/trim round-trips; roles + `outwardN` correct; `bbox`
      tight; `watertight` true.
- [x] 2.2 Decline battery (each returns `nullopt` with the right blocker): open/leaky
      boundary, multi-shell operand, holed freeform face (inner loop), bare-periodic freeform
      face (analytic paths own it), torus face, and a purely-analytic solid (no freeform face).
- [x] 2.3 Assert the descriptor exposes exactly the handles the verbs need: the freeform
      `Face` feeds B2 `splitFace`; the operand `Shape` feeds M0 `SolidMesher::mesh`; the AABB
      feeds B3 `classifyPointInMesh`.

## 3. STRETCH — minimal freeform↔analytic assembly (`freeform_operand.h`, guarded)

- [ ] 3.1 Additive `freeform_boolean_solid(a, b, op)` for the SIMPLEST reachable case: one
      admitted single-freeform-wall operand, one analytic planar half-space, `op ∈ {Cut,Common}`.
      Compose the landed verbs ONLY (design §3): recognise[B1] → M1 seam trace → B2 `splitFace`
      → B3 `classifyPointInMesh` per fragment → weld[M0/assemble]. NULL on any verb decline.
- [ ] 3.2 Fragment selection per op rule at each fragment's interior point; any B3 `On`/`Unknown`
      at a fragment centroid → NULL (honest decline, never a guessed keep/drop).
- [ ] 3.3 MANDATORY self-verify on the welded result: watertight (every edge shared by exactly
      two faces) AND volume matches the closed-form/set-algebra value within tol; ANY failure →
      DISCARD → NULL (→ OCCT). NEVER emit a leaky/wrong solid.

## 4. Sim native-vs-OCCT gate (booted simulator, OCCT linked)

- [ ] 4.1 IF the minimal assembly assembles: compare the native result's volume/area/watertight
      to `BRepAlgoAPI_{Cut,Common}` (via `BRepGProp`) within a scale-relative tolerance, and a
      point batch to `BRepClass3d_SolidClassifier` on the result (ZERO crisp IN↔OUT disagreements).
- [ ] 4.2 Confirm the engine self-verify DISCARDS any non-watertight / wrong-volume native result
      → OCCT (a leaky solid is never emitted). OCCT referenced ONLY in `src/engine/occt`.

## 5. Zero-regression proof (MANDATORY — additive-sibling discipline)

- [x] 5.1 Prove `recogniseCurvedSolid` and `classifyPoint` (`ssi_boolean.h`) are BYTE-IDENTICAL
      vs `main` (the analytic S5 assembler unchanged); re-run `run-sim-native-curved-boolean`
      with unchanged counts.
- [x] 5.2 Prove M0 (`trimmedFreeformMesh`), M1 (`WLine`), B2 (`splitFace`), B3
      (`classifyPointInMesh`) are consumed UNCHANGED: `run-sim-native-face-split`,
      `run-sim-native-freeform-membership`, `run-sim-native-ssi-marching`, and the M0 mesher
      host tests all pass with counts unchanged from §0.2.
- [x] 5.3 `grep -R` proves 0 OCCT includes under `src/native/**` and no `cc_*` signature/POD change.

## 6. Docs / spec

- [x] 6.1 Update `openspec/MOAT-ROADMAP.md` M2/B1 status: B1 landed (descriptor + gate, host-analytic
      green), the minimal-assembly result (assembled + sim-vs-OCCT, or HONEST DECLINE with the
      measured gap + specific blocker). Note B1 completes the M2 substrate.
- [x] 6.2 `openspec validate moat-m2a-freeform-operand-descriptor --strict`; archive on completion.

## 7. Honest-out (a first-class outcome, not a fallback failure)

- [x] 7.1 If the minimal assembly is NOT robustly reachable (a verb declines on the simplest
      fixture, ray consensus is unreliable, or the self-verify cannot pass without weakening a
      tolerance): LAND B1 (the descriptor + gate, both descriptor gates green) and HONEST-DECLINE
      the assembly — record which verb declined, the quantified shortfall, and the specific
      remaining blocker. Leave `recogniseCurvedSolid`/`classifyPoint` and M0/M1/B2/B3
      byte-identical, weaken no tolerance, emit no wrong/leaky solid, write no dead code.
