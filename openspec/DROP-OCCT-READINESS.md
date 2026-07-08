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
| twisted_sweep | cc_twisted_sweep | 1017 | **B** | M7 / ~0.3py | 4 | native covers only twist≈0 / scale≈1 (degenerate); real twist = typical use → OCCT (1020) |
| loft_along_rail | cc_loft_along_rail | 1033 | **B** | M7 curved-rail / ~0.4py | 8 | native covers only straight rail; curved rail = typical use → OCCT (1037) |
| guided_sweep | cc_guided_sweep | 1040 | **A** | resid M2/M7 | 5 | native serves general tube; self-folding SSI only (1044) |
| guided_orient_sweep | cc_guided_orient_sweep | 1054 | **A** | resid M7 | 0 | native mainline; curved-spine residual (1058); 0 app usage |
| helical_thread | cc_helical_thread | 1102 | **A** | resid M7b | 6 | coarse pitch native; fine-pitch self-intersecting → OCCT (1106) |
| tapered_thread | cc_tapered_thread | 1109 | **A** | resid M7b | 5 | as above (1113) |
| tapered_shank | cc_tapered_shank | 1116 | **A** | done | 4 | null only (1118) |
| wrap_emboss | cc_wrap_emboss | 1074 | **A** | resid M3 freeform-base | 6 | cylinder base native (typical); freeform base → OCCT (1076) |
| **FEATURE / BLEND** ||||||
| fillet_edges | cc_fillet_edges | 1204 | **A** | resid M3 | 15 | planar + convex/concave-circular native (prismatic parts = bulk); curved/freeform → OCCT (1205) |
| fillet_edges_variable | cc_fillet_edges_variable | 1250 | **A** | resid M3 | 7 | convex-circular linear-law native (1252) |
| chamfer_edges | cc_chamfer_edges | 1270 | **A** | resid M3 | 10 | planar + convex-circular native (1271) |
| chamfer_edges_asym | cc_chamfer_edges_asym | 1292 | **A** | resid M3 | 0 | convex-circular native (1294) |
| shell | cc_shell | 1311 | **A** | resid M3 curved | 12 | convex-planar native; curved/non-convex → OCCT (1312) |
| offset_face | cc_offset_face | 1321 | **A** | resid M3 curved | 10 | planar face native; curved → OCCT (1322) |
| split_plane | cc_split_plane | 1355 | **A** | DM1 done (resid DM breadth) | 4 | box / planar / single-freeform native; cyl-slice / oblique / multi-lump → OCCT (1357) |
| replace_face | cc_replace_face | 1332 | **A** | DM3 done (resid tilt breadth) | 6 | pure-offset planar retarget native via DM2 grow-then-trim (`replace_face_general.h`, both gates); non-zero tilt (foreign OCCT X-axis) / non-planar / curved neighbour → OCCT |
| replace_face_to_plane | cc_replace_face_to_plane | 1336 | **A** | DM2 done (resid DM breadth) | 4 | planar push/pull native via grow-then-trim (98a2011); non-planar / non-prismatic target → OCCT (1338) |
| project_point_on_face | cc_project_point_on_face | facade | **A** | DM4 done (resid non-analytic) | 0 | plane / cylinder / sphere closed-form foot native (`project.h`, both gates vs GeomAPI_ProjectPointOnSurf); cone / torus / freeform / ambiguous → OCCT |
| fillet_face | cc_fillet_face | 1341 | **B** | M3 | 7 | hard decline (1342) |
| full_round_fillet | cc_full_round_fillet | 1378 | **B** | M3 | 0 | hard decline (1379) |
| full_round_fillet_faces | cc_full_round_fillet_faces | 1382 | **B** | M3 | 0 | hard decline (1383) |
| fillet_edges_g2 | cc_fillet_edges_g2 | 1386 | **B** | M3 (G2) | 0 | hard decline (1387) |
| **BOOLEAN** ||||||
| boolean_op | cc_boolean | 1411 | **A** | resid M2 freeform | 13 | native planar-polyhedron + analytic-curved SSI (bulk of CAD booleans); all-OCCT operands → OCCT (1416); mixed native/OCCT → honest error; native-native curved/degenerate → honest error (NEVER faked) |
| thread_apply | cc_thread_apply | 1440 | **B** | M2/M7b | 0 | hard decline on shaft+thread (1441-1442) |
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
| hlr_project | cc_hlr_project | 1572 | **A** | resid M-GS GS1 | 0 | polyhedral + cyl/sphere silhouette native; cone/torus/freeform → OCCT (1572) |
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
| step_import | cc_step_import | 2035 | **A** | resid M4 tail | 10 | native AP203 + B-spline admission (rational + trims + nested); malformed / PMI-semantics / `MAPPED_ITEM` → OCCT (2035) |
| pmi_scan | cc_step_pmi_scan | 2041 | **A** | done | 0 | read-only native, no fall-through |
| stl_import | cc_stl_import | 2070 | **A** | done | 0 | OCCT-free native reader, **no fall-through** (2070) |
| iges_export | cc_iges_export | 2062 | **C** | DECLINE (IGES dropped) | 6 | native→error, else OCCT (2062); product-scope drop |
| iges_import | cc_iges_import | 2065 | **C** | DECLINE (IGES dropped) | 5 | **unconditional** `return fallback().iges_import` (2065) |

**Not OCCT-dependent (already fully native, not a fall-through):** `cc_stl_export`,
`cc_tet_mesh`, `cc_tet_mesh_surface`, `cc_mesh_quality` (facade → native tessellate + native
writers / TetGen). **Facade-pure fp64 (no engine call):** `cc_ref_plane_from_points`,
`cc_ref_plane_offset`, `cc_ref_axis_from_points`. **Infra (not geometry):** `set_parallel` /
`set_gpu_tessellation` forward to fallback policy (625-628) → become native no-ops at unlink.

**Site tally:** 63 classified engine fall-through sites → **A: 47**, **B: 14**, **C: 2**
(M-TX landed: the 6 transform ops + `extrude_mesh` moved B→A). The remaining **B** sites
hard-decline on native bodies via `CC_NATIVE_BODY_UNSUPPORTED` (M-REF reference/topology
reads, M-DM `replace_face`, M3 OCCT-only fillets) plus the degenerate-slice construct ops
(`twisted_sweep`, `loft_along_rail`); `iges_export` is the one C-class invocation.

---

## 2. MUST-GO-NATIVE (Class B) — mapped to roadmap stage + remaining py

| bucket | ops | stage | app sites | remaining py |
|---|---|---|---|---|
| **M-TX native transforms** — ✅ LANDED (native `Shape::located(math::Transform)`; two-gate proven) | translate_shape, rotate_shape_about, mirror_shape, scale_shape, scale_shape_about, place_on_frame, extrude_mesh | **M-TX (done)** | 27 + 10 | **0 (done)** |
| **M-REF reference / topology** — ✅ LANDED (native, two gates green) | face_axis, ref_plane_from_face, ref_axis_from_edge, ref_axis_from_face, tangent_chain, outer_rim_chain, offset_face_boundary | **M-REF (done)** | 22 | **0 (done)** |
| **M-DM direct modeling** | replace_face (DM3) — DM2 replace_face_to_plane now native (98a2011) | **M-DM** | 6 | **~1–2** |
| **M3 OCCT-only + curved-blend breadth** | fillet_face, full_round_fillet, full_round_fillet_faces, fillet_edges_g2 + curved/freeform residuals of the A-class blends (fillet/chamfer/shell/offset_face) | **M3** (in M2/M3 breadth bucket) | 14 direct | **~3–8** (shared M2/M3) |
| **M2 freeform-boolean breadth + thread_apply** | boolean_op freeform/mixed residual, thread_apply | **M2** (same bucket) | 0 direct | (in the ~3–8) |
| **M7 / M7b construct tails** | twisted_sweep (real twist), loft_along_rail (curved rail), fine-pitch thread residuals | **M7 / M7b** | 12 | **~1–2** |
| **M-GS GS1 curved-HLR** | hlr_project cone/torus/freeform silhouettes | **M-GS GS1** | 0 direct | (substantial GS residual; gates OCCT-free 2D drawings, not solid primitives) |

**MUST-GO-NATIVE remaining-py total: ≈ 6.5–15 py (midpoint ≈ 10 py)**, dominated by the
**M2/M3 freeform breadth bucket (~3–8 py)**. The cheapest, highest-leverage slice is
**M-TX + M-REF (~1–2 py combined, 49 app call-sites)** — both are bounded (compose a
Location/affine into native topology; read native edge-sharing topology) and neither is on the
existing roadmap, yet together they are the largest *app-facing* hard-decline blocker.

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
  recognition, never faked — the same discipline as the landed M2 declines); foreign STEP
  `MAPPED_ITEM` (Form-B) assemblies and full PMI *semantics* (the census scan is native).

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

**Scoped `drop-occt` is architecturally reachable but not yet ready — blocked on ≈ 6.5–15 py
(midpoint ~10 py) of Class-B native work, concentrated in one asymptotic bucket.** Itemized:
**(1)** the Class-A spine — construct (extrude/revolve/loft/sweep), tessellate, query/analysis
(mass/bbox/subshape/moments/curvature/measure), and STEP/STL exchange — is native and already
carries the bulk of the ~69 app `cc_*` ops, forwarding to OCCT only on genuinely
out-of-envelope input via the self-verify fall-through; these need only OCCT-arm deletion.
**(2)** The unlink blockers are all **B**: the high-usage but *small and unroadmapped*
**M-TX transform (27+10 sites) + M-REF reference-topology (22 sites)** group, which hard-errors
on native bodies today yet is bounded at ~1–2 py combined and is the largest app-facing
blocker; then **M-DM** replace_face DM3 (~1–2 py; DM2 to_plane already native); then the **M2/M3** freeform
boolean/blend breadth (~3–8 py, the dominant asymptotic cost) plus **M7/M7b/M-GS** tails.
**(3)** **IGES is the only clean Class-C decline** — trivial to convert, but the app must make
its own IGES decision (drop / OCCT-shim / native). **(4)** The whole finish is gated on the
asymptotic **M5 healing** + **M6 completeness/fuzzing** bars that guarantee no silent wrong
results after the fallback is gone. **Cheapest credible path to carrying a large fraction of
the app OCCT-free: land M-TX + M-REF first (bounded, unroadmapped, ~1–2 py), before committing
to the M2/M3/M-DM moat.**

---

*This is a documentation artifact, not an OpenSpec change. The living roadmap is
[MOAT-ROADMAP.md](MOAT-ROADMAP.md) (M8); parent [NATIVE-REWRITE.md](NATIVE-REWRITE.md).*
