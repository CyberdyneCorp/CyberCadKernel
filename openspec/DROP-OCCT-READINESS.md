# DROP-OCCT-READINESS — the scoped `drop-occt` checklist (M8)

**Scope of this document.** The itemized, source-grounded checklist for the **SCOPED**
`drop-occt`: OCCT leaves the *product build* entirely and survives **only as a test oracle**
(`tests/**`, differential fuzzing, native-vs-OCCT sim parity). This is *not* the removal of
OCCT from the repository — it is the removal of `src/engine/occt` from every shipped
configuration and the conversion of every remaining runtime dependency on it into either a
native path or a clean decline.

**Method / discipline.** Every fall-through below was read in source, not assumed. A
"fall-through" is any place the active native engine (`NativeEngine`,
`src/engine/native/native_engine.cpp`) hands work to the OCCT fallback. The fallback is wired
in exactly two places:

- `make_native_fallback_engine()` → `OcctEngine` under `CYBERCAD_HAS_OCCT`, else the stub
  (`native_engine.cpp:575-581`).
- `create_default_engine()` → `OcctEngine` whenever the OCCT TUs are linked
  (`src/engine/occt/occt_engine.cpp:264`).

The only OCCT source is `src/engine/occt/**` (adapter TUs: construct/feature/boolean/query/
tessellate/exchange/reference-geometry/drafting/shapefix/wrap-emboss/threads/g2-fillet/
full-round). Deleting that directory from the product build is the terminal unlink step.

**Classification rule (each site is exactly one of A/B/C):**

- **A — NOW-NATIVE.** The native path serves the *typical in-domain* input; OCCT is reached
  only on genuinely out-of-envelope / exotic sub-cases via the mandatory self-verify
  fall-through (native builder returns NULL → engine discards → forward). At unlink the OCCT
  arm is simply deleted; the residual out-of-envelope input becomes a clean decline. **No
  native work required** beyond deleting the arm. A named breadth residual is noted but does
  **not** reclassify the site.
- **B — MUST-GO-NATIVE.** A capability the supported domain needs where the native path is
  **absent** — either a hard `CC_NATIVE_BODY_UNSUPPORTED` decline on native bodies
  (`native_engine.cpp:957`, the macro forwards OCCT bodies but errors native ones) **or** the
  native path covers only a *degenerate slice* while the op's typical use forwards to OCCT.
  Roadmap stage + rough remaining py given.
- **C — OUT-OF-SCOPE-DECLINE.** A product-dropped capability. No native work; the site
  becomes a clean, honest error at unlink.

**App relevance.** Cross-referenced against `/Users/leonardoaraujo/work/cybercad`, which calls
**73 unique `cc_*` tokens** (≈69 geometry ops after excluding `*_free` / `cc_last_error` /
`cc_brep_available` infra helpers). App call-site counts below are the audited per-op usage.

---

## 1. Every OCCT fall-through site (classified, with evidence)

All `file:line` are in `src/engine/native/native_engine.cpp` unless noted.

| engine method | `cc_*` op | file:line | class | stage / py | app sites | decisive rationale (source-grounded) |
|---|---|---|---|---|---|---|
| **CONSTRUCT** ||||||
| extrude_mesh | cc_extrude | 967 | **A** | M-TX done | 10 | rewired: attempts native `build_prism`+`tessellate` FIRST (0.1 defl), forwards only on null/degenerate |
| solid_extrude | cc_solid_extrude | 647 | **A** | done | 11 | forwards only on null/degenerate (652) |
| solid_revolve | cc_solid_revolve | 657 | **A** | done | 6 | null/degenerate only (662) |
| solid_extrude_holes | cc_solid_extrude_holes | 1126 | **A** | done | 3 | null only (1129) |
| solid_extrude_polyholes | cc_solid_extrude_polyholes | 1132 | **A** | done | 5 | null only (1135) |
| solid_extrude_profile | cc_solid_extrude_profile | 1145 | **A** | done | 6 | null/verify-fail only (1150) |
| solid_extrude_profile_polyholes | cc_solid_extrude_profile_polyholes | 1153 | **A** | done | 5 | verify-fail only (1160) |
| solid_revolve_profile | cc_solid_revolve_profile | 1171 | **A** | resid M1/SSI | 4 | native serves; only spindle-torus / axis-crossing residual (1177) |
| solid_loft | cc_solid_loft | 977 | **A** | done | 8 | non-planar / verify-fail only (980) |
| solid_loft_wires | cc_solid_loft_wires | 983 | **A** | done | 6 | ditto (986) |
| solid_loft_sections | cc_solid_loft_sections | 997 | **A** | done M7a | 0 | self-folding / verify-fail only (1000) |
| solid_sweep | cc_solid_sweep | 1009 | **A** | done | 11 | tight-curvature / SSI spine only (1011) |
| twisted_sweep | cc_twisted_sweep | 1017 | **A** | done M7t | 4 | native serves plain + REAL PURE twist (densified Frenet ruled tube, area-preserving, self-verified watertight); twist+scale saddle not robustly weldable / self-folding → OCCT (1041) |
| loft_along_rail | cc_loft_along_rail | 1033 | **A** | done M7t | 8 | native serves straight rail AND smooth curved rail (RMF-transported morph densified to a bounded per-band turn, Pappus-converged); tight-kink rail / coarse section that won't weld → OCCT self-verify (1068) |
| guided_sweep | cc_guided_sweep | 1040 | **A** | resid M2/M7 | 5 | native serves general tube; self-folding SSI only (1044) |
| guided_orient_sweep | cc_guided_orient_sweep | 1054 | **A** | resid M7 | 0 | native mainline; curved-spine residual (1058); 0 app usage |
| helical_thread | cc_helical_thread | 1102 | **A** | resid M7b | 6 | coarse pitch native; fine-pitch self-intersecting → OCCT (1106) |
| tapered_thread | cc_tapered_thread | 1109 | **A** | resid M7b | 5 | as above (1113) |
| tapered_shank | cc_tapered_shank | 1116 | **A** | done | 4 | null only (1118) |
| wrap_emboss | cc_wrap_emboss | 1074 | **A** | resid M3 freeform-base | 6 | cylinder base native (typical); freeform base → OCCT (1076) |
| **FEATURE / BLEND** ||||||
| fillet_edges | cc_fillet_edges | 1204 | **A** | resid M3 | 15 | planar + convex/concave-circular + cone-frustum cap-rim + **sphere cap-rim (truncated ball / dome)** native (prismatic + tapered + revolved-ball rims = bulk); non-circular curved crease / cyl-cyl canal / freeform → OCCT (1205) |
| fillet_edges_variable | cc_fillet_edges_variable | 1250 | **A** | resid M3 | 7 | convex-circular linear-law native (1252) |
| chamfer_edges | cc_chamfer_edges | 1270 | **A** | resid M3 | 10 | planar + convex-circular native (1271) |
| chamfer_edges_asym | cc_chamfer_edges_asym | 1292 | **A** | resid M3 | 0 | convex-circular native (1294) |
| shell | cc_shell | 1311 | **A** | resid M3 curved | 12 | convex-planar native; curved/non-convex → OCCT (1312) |
| offset_face | cc_offset_face | 1321 | **A** | resid M3 curved | 10 | planar face native; curved → OCCT (1322) |
| split_plane | cc_split_plane | 1355 | **A** | DM1 done (resid DM breadth) | 4 | box / planar / single-freeform native; cyl-slice / oblique / multi-lump → OCCT (1357) |
| replace_face | cc_replace_face | 1332 | **A** | DM3 done (resid tilt breadth) | 6 | pure-offset planar retarget native via DM2 grow-then-trim (`replace_face_general.h`, both gates); non-zero tilt (foreign OCCT X-axis) / non-planar / curved neighbour → OCCT |
| replace_face_to_plane | cc_replace_face_to_plane | 1336 | **A** | DM2 done (resid DM breadth) | 4 | planar push/pull native via grow-then-trim (98a2011); non-planar / non-prismatic target → OCCT (1338) |
| project_point_on_face | cc_project_point_on_face | facade | **A** | DM4 done (resid non-analytic) | 0 | plane / cylinder / sphere closed-form foot native (`project.h`, both gates vs GeomAPI_ProjectPointOnSurf); cone / torus / freeform / ambiguous → OCCT |
| fillet_face | cc_fillet_face | 1440 | **A** (planar prism cap) | resid non-perp-wall/freeform | 7 | SERVED-NATIVE via the SPHERICAL fillet-corner weld (`fillet_corner.h`): rounds every convex edge bounding a planar face, welding the tangent-cylinder strips with a sphere-radius-r corner patch (shared great-circle arcs, pure assembly layer — no tessellator change). Both gates green (host closed-form `r²L(4−π)−4r³+(4/3)πr³` converging + SIM parity vs OCCT: watertight, χ=2, vol rel <5e-3 / analytic <1e-3, area <2e-2, bbox exact). Scope: perpendicular walls (prism cap); non-perp-wall / concave / curved / oversized / self-verify miss → OCCT |
| full_round_fillet | cc_full_round_fillet | 1477 | **A** (analytic prismatic) | resid M3 dihedral/closed-seam | 0 | prismatic rib (parallel walls, r=w/2 tangent-cylinder cap) native via `full_round.h` on the two opposite seams (both gates: analytic `(w²/2)(1−π/4)L` + OCCT full-round parity); dihedral (M2 valley-solve) / curved wall / closed-seam annulus (M2 weld) → OCCT |
| full_round_fillet_faces | cc_full_round_fillet_faces | 1481 | **A** (analytic prismatic) | resid M3 dihedral/closed-seam | 0 | explicit left/middle/right prismatic cap native via `full_round.h` (== auto entry, both gates); dihedral / curved / closed-seam / no-shared-seams → OCCT |
| fillet_edges_g2 | cc_fillet_edges_g2 | 1386 | **B** | M3 (G2) | 0 | hard decline (1387) |
| **BOOLEAN** ||||||
| boolean_op | cc_boolean | 1411 | **A** | resid M2 freeform | 13 | native planar-polyhedron + analytic-curved SSI (bulk of CAD booleans); all-OCCT operands → OCCT (1416); mixed native/OCCT → honest error; native-native curved/degenerate → honest error (NEVER faked) |
| thread_apply | cc_thread_apply | 1605 | **B** | M2/M7b | 0 | NATIVE ATTEMPT + honest decline: recognise[cylinder]→facet→planar-BSP `boolean_solid`→4-part self-verify (watertight+χ=2+consistently-oriented+two-sided closed-form-volume band); `threadApply` (`boolean/thread_apply.h`, OCCT-free). MACHINERY SOUND (cylinder−box baseline welds vs OCCT, both gates); multi-turn thread declines `NotWatertight`/`NotOriented` (measured: `build_thread` solid is watertight but sameDirectionEdgeCount=6 → invalid BSP operand + near-tangent helical BSP T-junction cracks) → OCCT per-turn oracle. Next blocker: orientation-coherent thread builder + robust dense-soup CSG with T-junction repair (M7b) |
| **TESSELLATE / QUERY / ANALYSIS** (native served; OCCT arm reached only for OCCT-built bodies, which cease to exist at unlink) ||||||
| tessellate | cc_tessellate | 669 | **A** | done | 12 | `if (!isNative) forward` (670) |
| face_meshes | cc_face_meshes | 679 | **A** | done | 9 | `!isNative` guard (680) |
| edge_polylines | cc_edge_polylines | 1457 | **A** | done | 8 | `!isNative` guard (1458) |
| mass_properties | cc_mass_properties | 722 | **A** | done | 7 | `!isNative` guard (723) |
| bounding_box | cc_bounding_box | 766 | **A** | done | 4 | `!isNative` guard (767) |
| subshape_ids | cc_subshape_ids | 810 | **A** | done | 17 | `!isNative` guard (811) |
| principal_moments | cc_principal_moments | 1882 | **A** | done GS5 | 6 | `!isNative` guard (1883) |
| check_solid | cc_check_solid | 1909 | **A** | done GS6 | 0 | `!isNative` guard (1910) |
| measure_distance | cc_measure_distance | 929 | **A** | done GS3 | 0 | `!isNative` guard (929) |
| measure_angle | cc_measure_angle | 913 | **A** | done GS3 | 0 | `!isNative` guard (913) |
| surface_curvature | cc_surface_curvature | 883 | **A** | done GS4 | 0 | `!isNative` guard (883) |
| edge_curvature | cc_edge_curvature | 900 | **A** | done GS4 | 0 | `!isNative` guard (900) |
| hlr_project | cc_hlr_project | 1572 | **A** | done M-GS GS1 | 0 | polyhedral + cyl/sphere/cone/frustum/torus(Kind::Torus) silhouette native; freeform + revolve-built torus (B-spline bands) → OCCT (1572) |
| section_plane | cc_section_plane | 1834 | **A** | resid small GS2 | 1 | native analytic; oblique-cyl / freeform → OCCT (1834) |
| **REFERENCE GEOMETRY** (NATIVE — MOAT M-REF; `src/native/reference/reference.h`, two gates green) ||||||
| face_axis | cc_face_axis | 1931 | **A** | done M-REF | 6 | native cyl/cone axis; plane/sphere/torus → decline → OCCT |
| ref_plane_from_face | cc_ref_plane_from_face | 1935 | **A** | done M-REF | 0 | native planar datum plane; non-planar → decline → OCCT |
| ref_axis_from_edge | cc_ref_axis_from_edge | 1939 | **A** | done M-REF | 0 | native line axis; circular/freeform edge → decline → OCCT |
| ref_axis_from_face | cc_ref_axis_from_face | 1943 | **A** | done M-REF | 0 | native cyl/cone axis (== face_axis); else decline → OCCT |
| tangent_chain | cc_tangent_chain | 1947 | **A** | done M-REF | 5 | native C1 walk (line/circle/ellipse); freeform edge → decline → OCCT |
| outer_rim_chain | cc_outer_rim_chain | 1951 | **A** | done M-REF | 6 | native planar-cap outer wire; empty = no cap |
| offset_face_boundary | cc_offset_face_boundary | 1955 | **A** | done M-REF | 5 | native polygon miter offset (inward-sharp case); arc/non-planar/growing-convex → decline → OCCT |
| **TRANSFORM** (M-TX: NOW native for a native body via `Shape::located(math::Transform)`; OCCT body forwards) ||||||
| translate_shape | cc_translate_shape | 2091 | **A** | M-TX done | 4 | native `located()` placement; OCCT body forwards |
| rotate_shape_about | cc_rotate_shape_about | 2058 | **A** | M-TX done | 5 | native rotation about axis; zero-axis declines |
| mirror_shape | cc_mirror_shape | 2069 | **A** | M-TX done | 4 | native reflection (det<0, valid watertight); zero-normal declines |
| scale_shape | cc_scale_shape | 2036 | **A** | M-TX done | 5 | native uniform scale; f≤0 honest decline |
| scale_shape_about | cc_scale_shape_about | 2044 | **A** | M-TX done | 4 | native uniform scale about centre; f≤0 declines |
| place_on_frame | cc_place_on_frame | 2100 | **A** | M-TX done | 5 | native rigid frame placement; degenerate frame declines |
| **EXCHANGE** ||||||
| step_export | cc_step_export | 2017 | **A** | resid small | 12 | native writer scope (plane/cyl/cone/sphere/bspline); out-of-scope → OCCT (2017) |
| step_import | cc_step_import | 2035 | **A** | resid M4 tail | 8 | native AP203 + B-spline admission (rational + trims + nested Form-A + **Form-B `MAPPED_ITEM`/`REPRESENTATION_MAP` instancing**); malformed / PMI-semantics / non-conformal / mixed-Form-A+B → OCCT (2035) |
| pmi_scan | cc_step_pmi_scan | 2041 | **A** | done | 0 | read-only native, no fall-through |
| stl_import | cc_stl_import | 2070 | **A** | done | 0 | OCCT-free native reader, **no fall-through** (2070) |
| iges_export | cc_iges_export | 2062 | **C** | DECLINE (IGES dropped) | 6 | native→error, else OCCT (2062); product-scope drop |
| iges_import | cc_iges_import | 2065 | **C** | DECLINE (IGES dropped) | 5 | **unconditional** `return fallback().iges_import` (2065) |

**Not OCCT-dependent (already fully native, not a fall-through):** `cc_stl_export`,
`cc_tet_mesh`, `cc_tet_mesh_surface`, `cc_mesh_quality` (facade → native tessellate + native
writers / TetGen). **Facade-pure fp64 (no engine call):** `cc_ref_plane_from_points`,
`cc_ref_plane_offset`, `cc_ref_axis_from_points`. **Infra (not geometry):** `set_parallel` /
`set_gpu_tessellation` forward to fallback policy (625-628) → become native no-ops at unlink.

**Site tally:** classified engine fall-through sites → **B: 7**, **C: 2**, the rest **A**
(this wave: M-TX's 6 transform ops + `extrude_mesh`, M-REF's 7 reference/topology reads,
M-DM `replace_face` (DM3 offset), and M7t's `twisted_sweep` (real twist) +
`loft_along_rail` (curved rail) all moved B→A — 17 sites cleared). The remaining **5 B**
sites are: the M3 OCCT-only fillets (`fillet_face`, `full_round_fillet`,
`full_round_fillet_faces`, `fillet_edges_g2`) and `thread_apply`; `iges_export`/`iges_import`
are the two C-class invocations.

---

## 2. MUST-GO-NATIVE (Class B) — mapped to roadmap stage + remaining py

| bucket | ops | stage | app sites | remaining py |
|---|---|---|---|---|
| **M-TX native transforms** — ✅ LANDED (native `Shape::located(math::Transform)`; two-gate proven) | translate_shape, rotate_shape_about, mirror_shape, scale_shape, scale_shape_about, place_on_frame, extrude_mesh | **M-TX (done)** | 27 + 10 | **0 (done)** |
| **M-REF reference / topology** — ✅ LANDED (native, two gates green) | face_axis, ref_plane_from_face, ref_axis_from_edge, ref_axis_from_face, tangent_chain, outer_rim_chain, offset_face_boundary | **M-REF (done)** | 22 | **0 (done)** |
| **M-DM direct modeling** — ✅ core LANDED (DM1/DM2/DM3 offset + DM4 project native) | replace_face offset (DM3) + project_point_on_face (DM4) native; only tilted/non-planar retarget breadth residual (gates M2/M3) | **M-DM (done, resid breadth)** | 6 | **~0.5 (residual)** |
| **M3 curved-blend breadth** — ✅ MAJOR SLICES LANDED (fillet_face + full_round_fillet[_faces] + chamfer_face/chamfer_corner native via the spherical/planar corner weld; **curved-edge fillet_edges native on cylinder + cone-frustum + sphere cap-rims** — the app's @15 curved-fillet gap for every analytic-revolve substrate; **curved shell** capped-cyl/cone; all two-gate vs OCCT) | **remaining:** fillet_edges_g2 (0 app sites, needs a G2 blend family), curved offset_face, cyl↔cyl canal fillet (needs a NEW tessellator crease weld), torus/elliptical-crease fillet (no OCCT-free native substrate / OCCT-built body), freeform-base wrap_emboss (needs freeform-surface breadth + a new tessellator weld) | **M3** | 14 direct | **~1–3** (down from ~3–8; the freeform-boolean spine + corner/curved-rim welds landed) |
| **M2 freeform-boolean breadth + thread_apply** — ✅ SPINE COMPLETE (half-space, corner-clip, curved-wall CUT+COMMON, ff↔ff CUT+COMMON, ≥3-seam strip all weld vs OCCT); thread_apply NATIVE MACHINERY LANDED (recognise+facet+planar-BSP+4-part self-verify, both gates; multi-turn declines with a measured reason → M7b) | boolean_op freeform/mixed residual (breadth on the landed spine), thread_apply multi-turn weld (orientation-coherent thread builder + dense-soup CSG w/ T-junction repair) | **M2 / M7b** | 0 direct | **~0.5–1** (spine done; residual breadth + thread) |
| **M7b construct tails** — ✅ M7t LANDED (twisted_sweep real twist + loft_along_rail curved rail native, both gates) | ~~twisted_sweep (real twist)~~ ✅, ~~loft_along_rail (curved rail)~~ ✅, fine-pitch thread residuals | **M7t (done) / M7b** | 12 | **~0.5 (M7b thread residual)** |
| **M-GS GS1 curved-HLR** — ✅ LANDED (native cone/frustum + torus silhouettes, two gates green) | hlr_project cone/frustum/torus(Kind::Torus) silhouettes native; freeform + revolve-built torus (B-spline bands) → OCCT | **M-GS GS1 (done)** | 0 direct | **0 (done — freeform B-spline silhouette is the honest residual)** |

**MUST-GO-NATIVE remaining-py total: ≈ 2.5–5 py (midpoint ≈ 3.5 py)** — down from the ~7 py
midpoint before the M0 tessellator welds (open-seam + closed-inner-seam + curved-rim) and the
M2 freeform-boolean spine (half-space → corner-clip → curved-wall CUT+COMMON → ff↔ff → ≥3-seam)
landed. **The coupled high-risk spine research that dominated the old estimate is DONE.** What
remains is *breadth on machinery that already works for the common case*, in one bucket:
- **M3 curved-blend residuals (~1–3 py)** — the dominant remainder: cyl↔cyl canal fillet (needs
  a new tessellator crease weld), freeform-base wrap_emboss (freeform-surface breadth + weld),
  curved offset_face, and fillet_edges_g2 (0 app sites). The **app-used** curved-edge fillet is
  already native for the analytic-revolve substrates the app exercises.
- **M2/M7b tails (~0.5–1 py)** — freeform-boolean breadth on the landed spine + the multi-turn
  thread weld (0 app sites).
- **M8 unlink execution (~0.25 py)** + the standing **M6 completeness bar** (ongoing, gates
  the unlink; now 13 native fuzz domains, 0 DISAGREED).

The whole app-facing hard-decline group (**M-TX + M-REF + M-DM + M3 fillets/shell + curved-wall
booleans**) is now native. The measured drop-OCCT payoff (docs/BENCH-*.md): native **7–20×
faster**, **~28 MB in-binary / ~112 MB dependency lighter**, **~11.5 MB less peak RAM**.

---

## 3. OUT-OF-SCOPE-DECLINE (Class C) — what the product declines

- **`cc_iges_import` / `cc_iges_export` — IGES DROPPED (product decision).** The only clean
  Class-C sites (`native_engine.cpp:2062`, `2065`). The app *uses* both (11 call-sites across
  `KernelBridge.mm` / `BRepShape.swift` / `KernelBridgeAPI.h`), so a fully-OCCT-free *app*
  needs a companion IGES decision: **(a)** drop IGES from the app, **(b)** keep a thin
  OCCT-linked IGES shim outside the OCCT-free core (app then not 100% OCCT-free), or **(c)**
  reimplement IGES natively (~1.5–3 py, out of current scope). This document assumes (a)/(b);
  at unlink both ops become a clean "IGES not supported" error.
- **Already honest declines, no native work needed:** arbitrary-broken B-rep healing beyond
  the landed M5 slices; exotic freeform booleans / blends in full generality (declined at
  recognition, never faked — the same discipline as the landed M2 declines); full PMI
  *semantics* (the census scan is native; GD&T tolerance-zone / feature-control-frame /
  datum-reference-frame modelling stays OCCT). Foreign STEP `MAPPED_ITEM` /
  `REPRESENTATION_MAP` (Form-B) assembly INSTANCING now imports NATIVELY (M4-tail,
  `moat-m4t-assembly-import`); only a non-conformal / ≠1-brep / mixed-Form-A+B mapped file declines.

---

## 4. The concrete unlink sequence

1. **AUDIT (this document).** Every fall-through enumerated + classified A/B/C against source.
   ✅ complete.
2. **Make MUST-GO-NATIVE native (Class B).** Land M-TX + M-REF (bounded, ~1–2 py) → M-DM
   (~1.5–3 py) → the M2/M3 freeform-blend/boolean breadth (~3–8 py) → M7/M7b + M-GS GS1 tails.
   Each capability follows the non-negotiable two-gate discipline (host-analytic + sim
   native-vs-OCCT) and keeps OCCT as the oracle *during* implementation.
3. **Convert the remaining fall-throughs to clean declines (Class C + A residuals).** IGES →
   honest error (or app-side shim). Each Class-A site: delete the OCCT arm so the
   out-of-envelope residual returns the existing honest self-verify error instead of
   forwarding. Confirm no supported-domain input reaches a decline (M6 differential-fuzzing
   bar: zero silent wrong results — every non-native input declines with a clear error).
4. **Delete `src/engine/occt` from the product build.** Remove the OCCT link and the
   `CYBERCAD_HAS_OCCT` product configuration so `make_native_fallback_engine()` /
   `create_default_engine()` resolve to the stub (`native_engine.cpp:575-581` #else arm;
   stub `create_default_engine`). Remove the OCCT adapter TUs from the shipped target.
5. **Keep OCCT ONLY in tests.** OCCT stays linked exclusively for the differential-fuzzing +
   sim native-vs-OCCT parity oracle under `tests/**`. It is never in a shipped artifact.

**Gate:** Step 4 does not begin until (a) every Class-B capability the supported domain needs
is native at the acceptance bar, (b) every Class-A/C residual is a clean decline, and (c) the
**M6 completeness bar holds** (differential fuzzing = 0 DISAGREED, no fabricated shapes).

---

## 5. Readiness verdict (itemized)

**Scoped `drop-occt` is architecturally reachable — the build-unlink is PROVEN today (§6
rehearsal: native-only build, 0 crash / 0 silent-wrong), and ship-quality is now blocked on only
≈ 2.5–5 py (midpoint ~3.5 py) of breadth, no longer coupled spine research.** Itemized:
**(1)** the Class-A spine — construct (extrude/revolve/loft/sweep), tessellate, query/analysis
(mass/bbox/subshape/moments/curvature/measure), and STEP/STL exchange — is native and carries the
bulk of the ~69 app `cc_*` ops. **(2)** The former unlink blockers are now LANDED native at the
two-gate bar: **M-TX transforms**, **M-REF reference-topology**, **M-DM** direct-modeling core,
the **M2 freeform-boolean spine** (half-space → corner-clip → curved-wall CUT+COMMON → ff↔ff →
≥3-seam, on the M0 open-seam/closed-inner-seam/curved-rim tessellator welds), **M3** fillet_face
/ full_round / chamfer / **curved-edge fillet on cylinder+cone+sphere** / curved shell, **M7t**
twisted_sweep + loft_along_rail, and **M-GS/M-DM/M-REF/M-TX** services. What *remains* is bounded
breadth: **M3 curved-blend residuals** (cyl↔cyl canal fillet needing a new tessellator crease
weld, freeform-base wrap_emboss, offset_face, fillet_edges_g2 [0 app sites]) ~1–3 py; **M2/M7b
tails** (freeform-boolean breadth + multi-turn thread) ~0.5–1 py; **M8 execution** ~0.25 py.
**(3)** **IGES is the only clean Class-C decline** — the app must make its own IGES decision
(drop / OCCT-shim / native). **(4)** The finish is gated on the **M6 completeness bar** (13
native fuzz domains, 0 DISAGREED) which guarantees no silent-wrong after the fallback is gone.
**(5)** Per the app-migration audit ([docs/APP-MIGRATION-READINESS.md](../docs/APP-MIGRATION-READINESS.md)),
neither remaining Class-B op (`fillet_edges_g2`, `thread_apply`) is called by the app; the true
app path is kernel *adoption* + the IGES decision + the M3 curved-blend residuals the app exercises.

---

## 6. Measured rehearsal (dated 2026-07-08) — the static audit, executed

Sections 1–5 above are a **static, source-read** classification. This section records the
first **live measurement** of the post-unlink shape, via a throwaway, non-shipping CMake
option `CYBERCAD_M8_REHEARSAL` (OpenSpec change `moat-m8dry-unlink-rehearsal`, branch
`moat-m8dry`). It does **not** delete `src/engine/occt` and does **not** change any
shipping default.

### 6.1 Method

The host build already excludes the OCCT TUs (`CYBERCAD_HAS_OCCT=OFF`), so
`make_native_fallback_engine()` already resolves to the stub. The *only* difference
between the plain host build and the true post-unlink product is which engine is the
**default** — the plain host build defaults to the STUB, so a test that never calls
`cc_set_engine(1)` measures the stub, not the native path. The rehearsal flag flips
`create_default_engine()` to return `NativeEngine(stub)` (in `native_engine.cpp`, guarded
`#if CYBERCAD_M8_REHEARSAL && !CYBERCAD_HAS_OCCT`; the stub's definition guarded out under
the same macro). That single bit makes the WHOLE suite exercise the native-only +
stub-fallback path — the exact wiring after `src/engine/occt` is deleted. Two measures were
run against `-DCYBERCAD_M8_REHEARSAL=ON` (with `-DCYBERCAD_HAS_NUMSCI=ON`): the full HOST
CTest suite, and a per-op decline probe (`scratch/m8_probe.cpp`, throwaway) driving each
Class-B/C op on a native body plus a Class-A spine spot-check.

### 6.2 Build result

The rehearsal library and all 56 test executables **configure, build, and link cleanly**.
`find build-m8 -path '*occt*' -name '*.o'` is **empty** (no OCCT TU compiled) and
`nm libcybercadkernel.a | grep OcctEngine` is **empty** (no `OcctEngine` symbol
referenced). **The terminal "delete `src/engine/occt`" step is build-safe today** — the
product build does not depend on the OCCT TUs being present.

### 6.3 HOST suite result — 53 PASS-native / 0 clean-decline-fail / 3 sentinel-flip / 0 crash

| bucket | count | tests |
|---|---|---|
| **PASS-native** | 53 | every geometry / construct / query / analysis / exchange suite, incl. the 5 heavy SSI numerics suites (pass with an adequate per-test timeout) |
| **FAIL — sentinel flip** | 3 | `test_guard`, `test_abi`, `test_native_engine` |
| **FAIL — crash / silent-wrong** | **0** | — |

All 3 "failures" share ONE root cause and are **not regressions**: each asserts the
*shipping-default* invariant that the rehearsal deliberately inverts.
- `test_guard::brep_unavailable_in_stub_build` (`cc_brep_available()==0`) and
  `stub_geometry_returns_zero_and_sets_error` (`cc_solid_extrude` returns 0): under the
  rehearsal the default engine is NativeEngine, so `cc_brep_available()` reports available
  and `cc_solid_extrude` **builds a real native solid** (returns a nonzero id). This is a
  PASS-native outcome a stub-era test flags as a failure.
- `test_abi::library_links_and_reports_no_brep` (`cc_brep_available()==0`): same flip.
- `test_native_engine::default_engine_is_not_native` (`cc_active_engine()==0`): the
  rehearsal makes the default native BY DESIGN. The other 43 cases in that suite — the
  actual native geometry ops — all pass.

### 6.4 Per-op decline probe — every B/C op cleanly declines, every A spine op serves

Default engine under rehearsal: `cc_active_engine()==1` (native), `cc_brep_available()==1`.

| op | class | measured | error surfaced |
|---|---|---|---|
| cc_fillet_face | A (prism cap) | **SERVED-NATIVE** (prism cap via `fillet_corner.h`) / clean-decline | "native fillet_face: no verified watertight result … (non-perp wall / curved / oversized → OCCT-only)" |
| cc_full_round_fillet | A (prismatic) | **SERVED-NATIVE** (prismatic rib) / clean-decline (dihedral/curved/closed-seam) | "native full_round_fillet: … dihedral / curved wall / closed-seam annulus → OCCT-only" |
| cc_full_round_fillet_faces | A (prismatic) | **SERVED-NATIVE** (prismatic rib) / clean-decline | "native full_round_fillet_faces: … → OCCT-only" |
| cc_fillet_edges_g2 | B | **CLEAN-DECLINE** | "… fillet_edges_g2 …" |
| cc_twisted_sweep (real twist) | B | **CLEAN-DECLINE** | "operation not supported by active engine: twisted_sweep" |
| cc_loft_along_rail (curved rail) | B | **CLEAN-DECLINE** | "… loft_along_rail" |
| cc_thread_apply | B | **NATIVE ATTEMPT → CLEAN-DECLINE** | native recognise+facet+planar-BSP+4-part self-verify; multi-turn thread → measured decline (NotWatertight/NotOriented) → OCCT per-turn oracle |
| cc_iges_import | C | **CLEAN-DECLINE** | "operation not supported by active engine: iges_import" |
| cc_iges_export | C | **CLEAN-DECLINE** | "… iges_export …" |
| cc_solid_revolve | A | **SERVED-NATIVE** | — |
| cc_solid_loft | A | **SERVED-NATIVE** | — |
| cc_tessellate | A | **SERVED-NATIVE** (8 v / 12 t) | — |
| cc_mass_properties | A | **SERVED-NATIVE** (vol 1000.0) | — |

Every declining op returns **id=0 with a non-empty `cc_last_error`** — the desired
drop-OCCT clean-decline. No op hit the FINDING cases (id=0 with an empty error, or a
fabricated non-zero handle where OCCT would have served).

### 6.5 Static-vs-measured discrepancies

**None material.** The static A/B/C classification is **empirically confirmed**:
- No Class-A op crashed or returned a fabricated / silently-wrong shape.
- No Class-B/C op declined without a reason; each surfaced an honest error.
- The B declines route two ways, both clean and both anticipated by §1: body-consuming
  ops with no native path (`fillet_face`, `full_round_fillet[_faces]`, `fillet_edges_g2`,
  `thread_apply`) via the `CC_NATIVE_BODY_UNSUPPORTED` macro's own error; native ops that
  return NULL on out-of-envelope input (`twisted_sweep` real-twist, `loft_along_rail`
  curved-rail) forward to the stub's `engine_unsupported`. `iges_*` (C) forward to the
  stub. Both messages differ in wording but are equally honest declines.
- The only "surprises" are the three shipping-default sentinel tests (§6.3), which are an
  artifact of the rehearsal flipping the default, not a classification error.

### 6.6 Measured unlink checklist — what actually changes if `src/engine/occt` is deleted today

Ordered by product impact. **Nothing breaks the build; nothing produces silent-wrong.**
The delete only converts these OCCT-served ops to honest declines:

1. **`fillet_face`, `full_round_fillet`, `full_round_fillet_faces`, `fillet_edges_g2`**
   (Class-B, M3) — become clean declines on ALL bodies. **7 app sites (`fillet_face`).**
   Failure mode today under rehearsal: `CC_NATIVE_BODY_UNSUPPORTED` honest error. Requires
   the **M3** OCCT-only fillet work to restore.
2. **`boolean` freeform / mixed residual** (Class-A row, but the freeform slice forwards)
   — the planar + analytic-curved bulk is native; all-OCCT-operand and native-native
   freeform-degenerate cases decline. Requires **M2** breadth.
3. **`twisted_sweep` (real twist), `loft_along_rail` (curved rail)** (Class-B, M7) — the
   degenerate slice is native; the typical use declines. **12 app sites combined.**
   Requires **M7/M7b** (~1–2 py).
4. **`thread_apply`** (Class-B, M2/M7b) — NATIVE MACHINERY LANDED (recognise[cylinder] →
   facet → planar-BSP `boolean_solid` → 4-part self-verify: watertight + χ=2 +
   consistently-oriented + two-sided closed-form-volume band; both gates green). The
   tractable planar-cutter baseline (cylinder − box) WELDS native-vs-OCCT; a multi-turn
   helical thread declines with a MEASURED reason (`build_thread` is watertight but
   `sameDirectionEdgeCount=6` → invalid BSP operand + near-tangent helical BSP T-junction
   cracks) → OCCT per-turn oracle. Next blocker: orientation-coherent thread builder +
   robust dense-soup CSG with T-junction repair (M7b). 0 direct app sites.
5. **`iges_import`, `iges_export`** (Class-C, DROPPED) — clean declines by product
   decision. **11 app sites** → needs a companion app-side IGES decision (drop / OCCT-shim
   / native), not kernel work.
6. **All Class-A residual breadth** (fillet/chamfer/shell/offset_face curved cases,
   `hlr_project` curved silhouettes, STEP out-of-scope) — the typical in-domain input is
   already served natively; only the named out-of-envelope residual declines.

**Everything else** — the Class-A construct spine (extrude/revolve/loft/sweep + M-TX
transforms + M-REF references + M-DM core), tessellate, all query/analysis, STEP/STL —
**serves natively today with OCCT unlinked.**

### 6.7 Assessment — M2/M3/remaining-B: required for unlink vs nice-to-have

- **Required to make the product BUILD OCCT-free:** *nothing further.* The rehearsal
  proves the native-only + stub-fallback configuration builds, links, and runs the full
  suite with zero crashes and zero silent-wrong results **today**.
- **Required to SHIP the scoped unlink WITHOUT regressing the supported domain:** turn the
  §6.6 declines that the supported domain actually needs back into served ops. That is the
  **M2/M3 freeform-blend/boolean breadth (~3–8 py)** for the four M3 OCCT-only fillets +
  the `boolean` freeform residual, plus the **M7/M7b construct tails (~1–2 py)** for
  `twisted_sweep`/`loft_along_rail`/fine-pitch threads, plus the IGES product decision.
- **Nice-to-have (not a blocker):** the Class-A curved-residual breadth
  (fillet/chamfer/shell/offset_face curved, `hlr_project` curved silhouettes) — these are
  out-of-envelope declines whose *typical* input is already native; they can be filled
  incrementally after unlink without a fallback.
- The **M6 completeness/fuzzing bar** remains the true gate: the rehearsal confirms the
  clean-decline contract on the ops probed, but M6's differential fuzzing is what
  guarantees **no silent-wrong across the whole input space**, which is the actual
  precondition for deleting the oracle.

---

*This is a documentation artifact, not an OpenSpec change. The living roadmap is
[MOAT-ROADMAP.md](MOAT-ROADMAP.md) (M8); parent [NATIVE-REWRITE.md](NATIVE-REWRITE.md).
The §6 rehearsal method is captured as the OpenSpec change
`moat-m8dry-unlink-rehearsal`.*
