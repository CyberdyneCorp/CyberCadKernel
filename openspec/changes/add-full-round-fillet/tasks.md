# Tasks — add-full-round-fillet

Verification levels: **host** = the stub no-op + ABI contract run in the no-OCCT
host CTest; **ios-sim-build** = the blend builder compiles for
`arm64-apple-ios16.0-simulator` with OCCT; **ios-sim-run** = the full-round runs
on the booted simulator and the IsValid + watertight + face-gone + G1-tangency
checks pass — this is the acceptance bar for every full-round requirement.

## 1. Entry points + routing
- [x] 1.1 Add `IEngine::full_round_fillet(EngineShape body, int faceId)` +
  `full_round_fillet_faces(EngineShape body, int left, int middle, int right)`
  returning `ShapeResult`, default `engine_unsupported`; stub inherits it. (**host**)
- [x] 1.2 Add `CCShapeId cc_full_round_fillet(CCShapeId body, int faceId)` +
  `CCShapeId cc_full_round_fillet_faces(CCShapeId body, int leftFaceId, int
  middleFaceId, int rightFaceId)` to the header + facade, routed through
  `active_engine()`. Additive only. (**host**)
- [x] 1.3 `tests/test_abi.cpp` still matches `KernelBridgeAPI.h`. (**host**)

## 2. Neighbour + seam identification (OCCT adapter)
- [x] 2.1 From the middle face, use `TopExp` edge→face adjacency to find the two
  neighbour side faces and the two shared seam edges; the three-face form takes
  them explicitly. Ambiguous single-arg cases fall back / return `0`. (**ios-sim-build**)

## 3. Rolling-ball blend + rebuild
- [x] 3.1 Build a blend surface tangent to both side faces across the strip
  (OCCT `ChFi3d` blend driven to consume the middle face, or a
  `GeomFill`/`BRepFill` tangent-constrained fill between the seam edges), radius
  from local strip width. (**ios-sim-build**)
- [x] 3.2 Rebuild the solid with the middle face removed and the blend sewn in
  (`BRepBuilderAPI_Sewing` + `ShapeFix_Shell`/`ShapeFix_Solid`); gate on
  `BRepCheck_Analyzer::IsValid`. (**ios-sim-build**)

## 4. Verification (REAL properties)
- [x] 4.1 Result is `BRepCheck_Analyzer::IsValid` AND watertight (closed shell,
  no free boundary). (**ios-sim-run**)
- [x] 4.2 Target face is GONE: the consumed `middleFaceId` no longer resolves to a
  face in the rebuilt body's face set. (**ios-sim-run**)
- [x] 4.3 G1 tangency: at sample points along each seam, the blend-face normal and
  the neighbour-face normal (`BRepLProp_SLProps`) agree within the documented
  tangency tolerance (normals' dot ≥ cos(tol)) on BOTH neighbours. (**ios-sim-run**)
- [x] 4.4 Determinism: repeating the full round yields the same volume + bbox. (**ios-sim-run**)
- [x] 4.5 Honest fallback: force a case where the true full round cannot be built;
  assert it returns a VALID standard edge fillet (`BRepCheck_Analyzer::IsValid`)
  and record it deferred with the measured tangency gap — NOT a faked G1/consume
  <!-- DEFERRED (measured): non-parallel walls (off-parallel 22.62°, n_L·n_R=-0.9231)
  fall back to a VALID standard edge fillet; the flat middle face is NOT consumed
  (rebuilt vol=1597.844). True full-round is verified only for tangent/parallel-wall
  strips (rolling-ball vol=782.8319, middle face consumed, cylinder blend, G1 dot=1.000000). -->
  pass. (**ios-sim-run**)

## 5. Validation
- [x] 5.1 Host CTest green (stub `0` + ABI); `scripts/run-sim-suite.sh` unchanged
  (additive). (**host** + **ios-sim-run**)
- [x] 5.2 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 3 +
  change index for `full-round-fillet`, recording any deferred case honestly.
