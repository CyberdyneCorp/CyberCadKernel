# Tasks — add-native-wrap-emboss-breadth (#7 wrap-emboss, breadth tracks)

Verification levels: **host** = OCCT-free host CTest (footprint projection +
watertight + signed added/removed volume; builds in BOTH the default and
`CYBERCAD_HAS_NUMSCI` configs — the weld is a shared-vertex `assembleSolid`, no SSI
substrate); **sim** = native-vs-OCCT `cc_wrap_emboss` parity on a booted simulator
(volume / area / watertight / `IsValid` within a deflection-bounded tol). No `cc_*`
signature is added — every track lands behind the existing `cc_wrap_emboss`. The
raised-rectangular-pad-on-cylinder emboss is the UNTOUCHED control.

Status: **T1 landed, T2 landed (incl. the T1×T2 crossed deboss-polygon), T3 honest
decline (no cone/sphere builder — no dead code).** Host `test_native_wrap_emboss`
7/7 (default + NUMSCI CTest 36/36 green); sim `native_wrap_emboss_parity` 14/14
(control 6/6 unchanged + deboss-rect + emboss/deboss-hex parity vs OCCT).

## 1. T1 — DEBOSS (recessed rectangular pocket on a cylinder)
- [x] 1.1 Accept `boss == 0` for the rectangular-footprint-on-cylinder case;
  `rTarget = R − depth` (`depth = height`). Guard `depth < R − ε` (floor strictly
  off the axis) — else NULL → OCCT (`buildDebossedCylinder`). (**host**)
- [x] 1.2 Build the pocket: the raised OUTER cap becomes the FLOOR at `R−depth`
  (outward normal `+radial` via the unchanged `emitOuterCap`); the axial +
  circumferential walls span `ρ ∈ [R−depth, R]` with outward normals FLIPPED into
  the pocket (a `sign = −1` param on `emitAxialWalls`/`emitCircWalls`, default `+1`
  keeps the emboss control byte-identical). `tileWallWithWindow` + end caps
  unchanged. Weld watertight via `assembleSolid`. (**host**)
- [x] 1.3 Host: deboss a rectangular pocket → watertight AND volume
  `= |cyl| − footArea·depth` within the deflection bound; `depth ≥ R` returns NULL.
  (**host**) — `wrap_emboss_deboss_rectangular_pocket_watertight_volume_reduced`,
  `wrap_emboss_scope_defers`.

## 2. T2 — NON-RECTANGULAR polygon footprint (raised OR recessed on a cylinder)
- [x] 2.1 `polyFootprint`: an N-vertex (`count ≥ 3`) CLOSED SIMPLE polygon in
  `(px,py)` — non-degenerate shoelace area, re-wound CCW, non-self-intersecting
  (O(n²) proper-crossing test). Self-intersecting / degenerate ⇒ NULL → OCCT.
  (**host**)
- [x] 2.2 Project each corner to the cylinder (`u = px/R`, `v = py + vMid`); each
  edge (incl. helical) is tiled into deflection-bounded chords by angular sagitta on
  its `u`-span (`densifyPolyUV`). (**host**)
- [x] 2.3 CAP: ear-clip the densified polygon (`triangulatePolygon`), map to
  `rTarget` (`R+height` emboss / `R−height` deboss), outward `+radial`
  (`emitPolygonCap`). (**host**)
- [x] 2.4 SIDE WALLS: one ruled strip per densified polygon edge from `R` to
  `rTarget`; outward hint = the edge's exterior `(dv,−du)` normal mapped to 3D,
  flipped into the pocket for a deboss (`emitPolygonSideWalls`). (**host**)
- [x] 2.5 BASE WALL: reuse `tileWallWithWindow` for the full-turn wall minus the
  polygon's (slightly INFLATED) bounding-box window, then fill the bbox ring MINUS
  the polygon with a with-hole ear-clip that SHARES the bbox-boundary vertices (the
  shared `uSamples` grid) with the surrounding wall and the polygon-boundary
  vertices with the cap + side walls (`emitBboxMinusPolygonWall`). The bbox is
  inflated so the polygon is a strict interior hole (its extreme vertices otherwise
  pinch the ring to zero width). Cannot close / off-wall ⇒ NULL → OCCT. (**host**)
- [x] 2.6 Host: emboss AND deboss a HEXAGON on a native cylinder → watertight AND
  volume `= |cyl| ± shoelaceArea·height` within the deflection bound; a self-
  intersecting polygon returns NULL. (**host**) —
  `wrap_emboss_hexagon_pad_watertight_volume_grown`,
  `wrap_emboss_hexagon_pocket_watertight_volume_reduced`, `wrap_emboss_scope_defers`.

## 3. T3 — freeform base — HONEST DECLINE (no cone/sphere builder; no dead code)
- [x] 3.5 **HONEST GATE.** Per the diagnosis, `cc_wrap_emboss` is CYLINDER-ONLY at
  the OCCT oracle (a sphere/cone lateral face is rejected: "face is not
  cylindrical"), so a cone/sphere native path would have NO working OCCT fallback
  and NO parity oracle — teaching OCCT the cone case is scope-creep beyond this
  change. Therefore NO native cone/sphere builder is added: `cylinderWall` returns
  `nullopt` for any non-cylinder face, so T3 returns NULL for ALL freeform bases
  (there is no cone/sphere builder that is never accepted — no dead code). A
  non-cylinder NATIVE body errors honestly; a non-cylinder OCCT body forwards to the
  (cylinder-only) OCCT oracle. Recorded blocker: OCCT `cc_wrap_emboss` rejects
  `GeomAbs_Sphere` / `GeomAbs_Cone`. (**host**) — `wrap_emboss_scope_defers`
  (planar-cap face → NULL).
- [~] 3.1–3.4 `coneWall` / `conePoint` / cone-pad builder — **NOT implemented**
  (would require OCCT-oracle scope-creep for both fallback and parity; declined per
  3.5). Deferred to a future change that also teaches the OCCT oracle the cone case.

## 4. Engine self-verify (generalised, signed) → OCCT fallback
- [x] 4.1 `wrapEmbossVerified(result, original, profileXY, count, height, boss)`
  derives the sign from `boss` — `boss == 1` requires the volume to GROW, `boss == 0`
  to SHRINK (each within the deflection-bounded band; `expected > 0` guard rejects a
  pocket removing more than the body holds). (**host**)
- [x] 4.2 Footprint area = the SHOELACE polygon area (exact for the rectangle
  (== bbox), correct for any polygon). Tolerance unchanged (1% relative + floor —
  never weakened). (**host**)
- [x] 4.3 NULL builder OR failing self-verify ⇒ OCCT `cc_wrap_emboss` (OCCT body) /
  honest error (native body). Out-of-slice inputs (non-cylinder/freeform base,
  self-intersecting polygon, over-2π / off-end footprint, `depth ≥ R`) return NULL
  and defer. (**host** + **sim**)

## 5. Native dispatch wiring
- [x] 5.1 `cybercad::native::feature::wrap_emboss` routes by footprint kind
  (rectangle vs polygon) and `boss`: rectangle+emboss → the UNCHANGED
  `buildEmbossedCylinder` control; rectangle+deboss → `buildDebossedCylinder`;
  otherwise → `buildPolyCylinder(boss)`. Non-cylinder base → NULL. (**host**)

## 6. Verification (two gates)
- [x] 6.1 Host suite: `test_native_wrap_emboss.cpp` extended with T1 (deboss +
  `depth ≥ R` NULL), T2 (hexagon emboss + hexagon deboss + self-intersecting NULL +
  degenerate NULL), T3 decline (planar-cap face NULL), and the unchanged control.
  Builds in default (standalone clang++) and NUMSCI (CTest) — 7/7. (**host**)
- [x] 6.2 Sim parity: `native_wrap_emboss_parity.mm` generalised to any profile +
  `boss`; added deboss-rect (×2) and emboss/deboss-hex vs OCCT — all watertight,
  mesh vol == B-rep vol, vol rel vs OCCT ≤ 8e-3, area rel ≤ 1.6e-2. 14/14. (**sim**)
- [x] 6.3 No regression: control emboss-rect 6/6 unchanged (byte-identical builder);
  default + NUMSCI CTest 36/36 green. (**sim** + **host**)
- [x] 6.4 `openspec validate add-native-wrap-emboss-breadth --strict` green.
  (**host**)

## Deferred (NOT in this change — honest NULL → OCCT)

- [x] **Deboss with a NON-rectangular polygon footprint** (T1 × T2 crossed) —
  LANDED: `buildPolyCylinder` handles `boss == 0`; sim `deboss-hex` passes parity.
- [ ] **CONE / SPHERE / general freeform (BSpline/Bezier) base faces** → OCCT (T3
  declines them; a cone path needs the OCCT oracle taught the cone case first).
- [ ] **Self-intersecting / dense / high-curvature profiles** (the OCCT healed-sew
  oracle's domain) → OCCT.
- [ ] **Footprint that wraps `> 2π`, self-overlaps, or exceeds the face's axial
  extent; deboss with `depth ≥ R`** → NULL → OCCT — never faked with a weakened
  tolerance.
