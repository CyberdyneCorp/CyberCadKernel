# Tasks — moat-m2b-freeform-face-split

## 1. Substrate & guardrails
- [x] 1.1 Fresh worktree substrate: run `bash scripts/build-numsci.sh iossim` and
      `bash scripts/build-numsci.sh host`; export `CYBERCAD_NUMSCI_DIR` to the
      matching `build-numsci/{iossim,host}` before building/running tests.
- [x] 1.2 Confirm the OCCT-free invariant baseline: `src/native/**` has 0 OCCT
      includes today; record the grep so the new files can be diffed against it.
- [x] 1.3 Read-only consume check: confirm the exact consumed symbols exist and are
      untouched — `WLine`/`WLinePoint` `(u1,v1)` in `ssi/marching.h`,
      `FaceMesher::buildBoundaryLoops` + `trimmedFreeformMesh` in
      `tessellate/face_mesher.h`, `triangulateConstrained` in `uv_triangulate.h`,
      `Builder::makeFace`/`makeEdge`/`addPCurve` in `native_topology.h`.

## 2. Construct the ONE proof face (native, no OCCT)
- [x] 2.1 Build a trimmed freeform `Face` over a `FaceSurface` `Kind::BSpline` (or
      `Bezier`) with a CONVEX outer `EDGE_LOOP` whose edges carry pcurves — the
      simplest reachable case.
- [x] 2.2 Produce (or synthesise from a known analytic seam) an M1 `WLine` whose
      `(u1,v1)` nodes form a single chord that cleanly enters and exits the trimmed
      `(u,v)` domain (no tangency, no re-entry).

## 3. Seam-clip helper (OCCT-free, `ssi/` or `boolean/`)
- [x] 3.1 Flatten the outer `EDGE_LOOP` to a `UVPolygon` at the SAME shared per-edge
      fractions the mesher uses (reuse `buildBoundaryLoops` output / anchors).
- [x] 3.2 Compute the seam's boundary crossings using the existing `orient2d` /
      `segmentsCross` primitives; require exactly one ENTRY + one EXIT crossing, each
      recorded as (boundary edge index, fraction) and (seam node index, fraction).
- [x] 3.3 Verify every interior seam node is strictly inside the outer loop
      (`UVRegion::inside` even-odd); otherwise flag "beyond first slice".

## 4. Partition into two sub-loops (`src/native/boolean/`, header-only)
- [x] 4.1 Build sub-loop L1 = boundary arc entry→exit + reversed seam chord; sub-loop
      L2 = complementary boundary arc + forward seam chord (D3).
- [x] 4.2 Split any boundary edge crossed MID-edge at the crossing fraction (restrict
      `[first,last]`, reuse curve/pcurve); reuse an existing vertex when the crossing
      is within snap tolerance.
- [x] 4.3 Rebuild each sub-`Face` over the SAME `FaceSurface` node: carry parent
      boundary edges verbatim; build ONE shared seam edge (UV pcurve = `(u1,v1)`
      polyline; 3D = `WLine.curve` fitted B-spline or the polyline) added to both
      wires with opposite orientation.

## 5. Self-verify → decline gate (D5, host-checkable, no OCCT)
- [x] 5.1 Clean-crossing check (exactly one entry + one exit; interior nodes inside).
- [x] 5.2 Non-degenerate sub-region check (simple loop, |UV area| above a
      scale-relative floor).
- [x] 5.3 Exact-shared-seam check (L1/L2 reference identical seam node sequence,
      opposite order; no boundary gap beyond snap tolerance).
- [x] 5.4 Tiling identity check (`area(L1)+area(L2) == area(parent)` within
      scale-relative tolerance; no overlap). ANY failure → return NULL (decline) with
      the measured blocker; NEVER emit a partial/leaky split.

## 6. GATE (a) — HOST ANALYTIC tiling proof (no OCCT linked)
- [x] 6.1 Host test: the two sub-faces' UV areas SUM to the parent outer loop's area
      within tolerance; the seam is the EXACT shared boundary of both (bit-identical
      UV node sequence, opposite order); no overlap / no gap.
- [x] 6.2 Host test: a seam that does NOT cleanly cross (0/≥3 crossings, tangential
      graze, re-entry, degenerate sub-region) makes the splitter return NULL, and the
      measured blocker is reported (honest decline is a first-class outcome).

## 7. GATE (b) — SIM native-vs-OCCT proof (booted iOS simulator, OCCT linked)
- [x] 7.1 Each sub-face meshes WATERTIGHT via the M0 `FaceMesher` (no tessellator
      change), boundary-shared points bit-identical across the seam.
- [x] 7.2 The union of the two sub-face meshes reproduces the parent face's
      area/topology; cross-check each sub-face's `BRepMesh` area against the native M0
      mesh area (and against an OCCT face-split reference where available).

## 8. Invariants & docs
- [x] 8.1 Re-run the OCCT-free grep on the new files: `src/native/**` still 0 OCCT
      includes; confirm no `cc_*` signature/POD layout changed (additive substrate).
- [x] 8.2 Prove zero-regression: the tessellator and tracer are byte-identical
      (consumed, not modified); run the tessellation-sensitive suites unchanged.
- [x] 8.3 Update `openspec/MOAT-ROADMAP.md` B2 status to reflect the landed first
      slice (and its explicit decline envelope).
- [x] 8.4 `openspec validate moat-m2b-freeform-face-split --strict` passes; leave the
      change UNCOMMITTED in the worktree.
