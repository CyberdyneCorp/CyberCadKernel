# App Migration-Readiness Audit — can CyberCad flip its geometry engine to the native kernel?

**Question.** Can the CyberCad iPad app drop OCCT and run on the CyberCadKernel native
engine *today*? If not, exactly what blocks it, and what is the shortest path?

**Answer (one line).** **No — and the first blocker is architectural, not geometric:** the
app does not link the CyberCadKernel product at all today. Once it does, the *geometry* is in
very good shape — the native engine serves the large majority of the app's real call-sites
and the app is already resilient to a clean decline — but a small, named blocking set
(curved-edge blends, IGES, a couple of construct tails) remains.

**Method / discipline.** READ-ONLY. Every claim below is tied to a specific app call-site
(`/Users/leonardoaraujo/work/cybercad`) or a specific row of the kernel readiness classification
(`openspec/DROP-OCCT-READINESS.md`, incl. the §6 M8 MEASURED rehearsal). No app or kernel
product code was modified. Where the readiness doc's audited per-op app-site count and the
app source agree, the readiness count is used; the subtle A-envelope calls were verified
directly in app source.

---

## 0. The architectural precondition (the finding that reframes the whole audit)

The audit brief asks "if the app set `cc_set_engine(native)` tomorrow…". **The app cannot set
`cc_set_engine` at all today** — and this is the single most important finding.

- The app's `cc_*` facade is its **own** OCCT bridge:
  `CyberCad/Kernel/Bridge/KernelBridge.mm` `#include`s OCCT headers directly
  (`BRepPrimAPI_*`, `BRepFilletAPI_*`, `BRepAlgoAPI_*`, `IGESControl_*`, …) and calls OCCT
  inline behind `#ifdef CYBERCAD_HAS_OCCT` (e.g. `cc_solid_extrude` → `buildExtrusion` →
  `BRepPrimAPI_MakePrism`, `KernelBridge.mm:596`). OCCT is linked via `Vendor/occt` and the
  `OTHER_LDFLAGS` in `project.yml:40` (`-lTKBRep -lTKBO -lTKFillet … -lTKDESTEP -lTKDEIGES`).
- The app has **no reference** to the CyberCadKernel product: no `libcybercadkernel.a` link,
  no `cc_kernel.h`, no `NativeEngine`, and **no `cc_set_engine` / `cc_active_engine`** anywhere
  in the app (`grep` over `*.swift/*.mm/*.h/*.yml/*.pbxproj` = 0 hits). The dual-engine
  selector lives only in the kernel product (`include/cybercadkernel/cc_kernel.h:289`).
- The app's non-OCCT fallback is a **pure-Swift `PreviewKernel`**
  (`CyberCad/Kernel/PreviewKernel.swift`), not the native kernel — used only when
  `CYBERCAD_HAS_OCCT` is off, and it does **not** provide B-rep fillet/boolean/shell.

**Good news — the ABI is compatible.** The kernel product exposes the *same* `cc_*`
signatures the app already calls (verified: `cc_fillet_edges`, `cc_boolean`, `cc_iges_import`
are byte-identical declarations in both `cybercad/.../KernelBridgeAPI.h` and
`CyberCadKernel/include/cybercadkernel/cc_kernel.h`). So the kernel product is a **drop-in
replacement** for the app's hand-rolled bridge. The migration is therefore two distinct steps,
and only the *first* is a hard prerequisite for the flip:

1. **ADOPT** — replace `KernelBridge.mm`'s OCCT bridge with a link against
   `libcybercadkernel.a` + `cc_kernel.h` (the app keeps its `cc_*` call-sites and its Swift
   `BRepShape` wrapper unchanged). This is app-integration work, not geometry.
2. **FLIP** — call `cc_set_engine(1)` to select the NativeEngine (OCCT still linked as the
   fallback/oracle), then eventually build the kernel OCCT-free (the `drop-occt` unlink).

The rest of this audit answers "once adopted, what serves / declines / breaks on the native
engine", which is the geometry-readiness question the campaign is really about.

---

## 1. The app's actual `cc_*` usage, cross-referenced to A/B/C

The readiness doc audits the app at **73 unique `cc_*` tokens (≈69 geometry ops** after
excluding `*_free`/`cc_last_error`/`cc_brep_available` infra). That is corroborated here:
`grep` over the app yields 63 distinct `cc_*` tokens in `KernelBridge.mm` plus the
loft/`shape_*` helpers, and the readiness §1 table enumerates each with a per-op app-site
count. Every geometry op the app calls maps to a readiness row; **no app-used geometry op is
missing a classification.** The app-facing ops that carry material call-site counts, with
their class from the readiness table (+ the M8-measured verdict):

### Per-op verdict table (app-used ops, grouped; MEASURED where the M8 rehearsal probed it)

| `cc_*` op | app sites | class | M8-measured verdict | app-usage-vs-envelope note |
|---|---:|---|---|---|
| **CONSTRUCT** ||||
| cc_solid_extrude / _holes / _polyholes / _profile[_polyholes] | 7/3/5/6/5 | A | SERVED-NATIVE | typical closed-profile prism in-envelope |
| cc_solid_revolve / _profile | 4/4 | A | SERVED-NATIVE | in-envelope; spindle-torus/axis-crossing residual → decline |
| cc_solid_loft / _wires / _sections | 8/6/0 | A | SERVED-NATIVE | non-planar/self-folding → decline |
| cc_solid_sweep | 11 | A | SERVED-NATIVE | tight-curvature/SSI spine → decline |
| cc_loft_along_rail | 8 | A (M7t) | SERVED-NATIVE straight+smooth-curved rail | tight-kink rail → decline |
| cc_twisted_sweep | 4 | A (M7t) | SERVED-NATIVE plain+pure-twist | twist+scale saddle → decline |
| cc_guided_sweep | 5 | A | SERVED-NATIVE | self-folding SSI → decline |
| cc_helical_thread / cc_tapered_thread | 6/5 | A (resid M7b) | coarse-pitch NATIVE | **fine-pitch self-intersecting → decline** |
| cc_tapered_shank | 4 | A | SERVED-NATIVE | null only → decline |
| cc_wrap_emboss | 6 | A (resid M3) | cylinder-base NATIVE | **freeform base → decline** |
| cc_extrude (mesh) | 10 | A (M-TX) | SERVED-NATIVE | native prism+tessellate |
| **FEATURE / BLEND** ||||
| cc_fillet_edges | 15 | A (resid M3) | planar+circular NATIVE | **curved/freeform-edge fillet → decline** (see §3) |
| cc_fillet_edges_variable | 7 | A (resid M3) | convex-circular linear-law NATIVE | non-circular → decline |
| cc_chamfer_edges | 10 | A (resid M3) | planar+convex-circular NATIVE | curved → decline |
| cc_shell | 12 | A (resid M3 curved) | convex-planar NATIVE | curved/non-convex → decline |
| cc_offset_face | 10 | A (resid M3 curved) | planar-face NATIVE | curved face → decline |
| cc_fillet_face | 7 | **A (prism cap, M3)** | SERVED-NATIVE prism cap / else CLEAN-DECLINE | **non-perp-wall / curved / concave → decline** |
| cc_split_plane / replace_face / replace_face_to_plane | 4/6/4 | A (M-DM) | planar/offset NATIVE | oblique/tilt/curved → decline |
| cc_fillet_edges_g2 | **0** | **B** | CLEAN-DECLINE | **app does NOT call it** (verified) |
| **BOOLEAN** ||||
| cc_boolean | 13 | A (resid M2) | planar+analytic-curved NATIVE | **all-OCCT-operand / freeform-degenerate → decline** |
| cc_thread_apply | **0** | **B** | CLEAN-DECLINE | **app does NOT call it** (verified) |
| **TESSELLATE / QUERY / ANALYSIS** ||||
| cc_tessellate / face_meshes / edge_polylines | 12/9/8 | A | SERVED-NATIVE | `!isNative` guard; native bodies served |
| cc_mass_properties / principal_moments / bounding_box | 7/6/4 | A | SERVED-NATIVE | native served |
| cc_subshape_ids | 17 | A | SERVED-NATIVE | native served (highest-count op) |
| cc_section_plane | 1 | A (resid GS2) | native analytic | oblique-cyl/freeform → decline |
| **REFERENCE GEOMETRY (M-REF, native)** ||||
| cc_face_axis / tangent_chain / outer_rim_chain / offset_face_boundary | 6/5/6/5 | A | SERVED-NATIVE | non-analytic edge → decline |
| **TRANSFORM (M-TX, native)** ||||
| cc_translate/rotate/mirror/scale[_about]/place_on_frame | 4/5/4/5/4/5 | A | SERVED-NATIVE | degenerate params → decline |
| **EXCHANGE** ||||
| cc_step_export / cc_step_import | 12/8 | A | SERVED-NATIVE | out-of-scope surface / PMI semantics → decline |
| cc_stl_import | 0 | A | native, no fall-through | — |
| **cc_iges_export / cc_iges_import** | **6/5** | **C** | **CLEAN-DECLINE (IGES dropped)** | **app USES both — needs a product decision (§3)** |

---

## 2. The blocking set — what stops the app flipping to native *after adoption*

Combining (a) the B-class ops the app actually calls, (b) the C-class ops the app uses, and
(c) the A-class ops whose *specific app usage* is outside the native envelope:

### (a) B-class the app calls: **NONE.**
Both remaining app-relevant B ops — `cc_fillet_edges_g2` and `cc_thread_apply` — have **0 app
call-sites** (verified in source: the only "G2" matches in the app are an unrelated
"parity item G2" naming coincidence in Measure/MassProps; "thread_apply" has zero hits). The
hard-decline B ops therefore **do not block this app**. (Note: `cc_fillet_face`,
`full_round_fillet[_faces]` were B in earlier waves but the readiness doc now classes
`cc_fillet_face` as A — served-native on a planar prism cap — so its 7 app sites are covered
in-envelope for the common case and clean-decline otherwise.)

### (b) C-class the app uses: **IGES — the real product decision.**
- `cc_iges_import` (5 sites) and `cc_iges_export` (6 sites) are the only clean Class-C
  invocations. The app wires them end-to-end: `BRepShape.importIGES`/`exportIGES`
  (`BRepShape.swift:431,665`) → `Workspace.importIGES`/`exportIGES` (`Workspace.swift:2632-2644`)
  → UI (`ModelWorkspaceChrome.swift:70` import-by-extension, `:195` "IGES (B-rep)" export button).
- Under native-only these become clean declines (M8-measured: both return id=0 with a non-empty
  error). The app needs a product decision: **(a) drop IGES from the UI, (b) keep a thin
  OCCT-linked IGES shim** (app then not 100% OCCT-free), or **(c) reimplement IGES natively
  (~1.5–3 py, out of scope).** This is app/product work, not native-kernel work.

### (c) A-class ops whose app usage is outside the native envelope (the subtle blockers).
These ops are class-A (native serves the *typical* case), but the app demonstrably drives them
with out-of-envelope input that the native path declines and forwards to OCCT today:
- **`cc_fillet_edges` on CURVED edges (15 sites, highest-usage blend).** Verified: the app
  fillets whole curved rim edges — `RevolveEdgeFilletIntegrationTests` fillets the curved rim
  of a revolve; `CylinderAxisTests.filletEdges([first], radiusMM: 4)` fillets a cylinder edge;
  `WorkspaceTests` fillets an `arc` edge. Native serves planar + convex/concave-circular
  (prismatic parts = the bulk), but **curved/freeform-edge fillets → decline** (readiness §1 row
  1204). This is the single largest A-envelope gap for the app.
- **`cc_shell` / `cc_offset_face` on curved bodies (12 / 10 sites).** Convex-planar native;
  curved/non-convex → decline (readiness rows 1311/1321).
- **`cc_boolean` freeform/mixed residual (13 sites).** Planar + analytic-curved is the bulk and
  serves native; all-OCCT-operand and native-native freeform-degenerate → decline (row 1411).
- **`cc_helical_thread` / `cc_tapered_thread` fine-pitch (6 / 5 sites).** Coarse-pitch native;
  **fine-pitch self-intersecting → decline** (M7b residual, rows 1102/1109).
- **`cc_wrap_emboss` on a freeform base (6 sites).** Cylinder-base native (the typical case);
  freeform base → decline (row 1074).

**The concrete blocking set = { IGES (product decision) } ∪ { A-envelope gaps: curved-edge
fillet, curved shell/offset, freeform/mixed boolean, fine-pitch thread, freeform wrap-emboss }.**
There is **no hard-decline B op in the app's path.**

---

## 3. Can the app flip today? (quantified)

**Prerequisite:** the app must first ADOPT the kernel product (§0) — it links its own OCCT
bridge today, so `cc_set_engine(1)` is not even callable yet. Assuming adoption, then flipping
to native:

- **Serves natively (the large majority).** The entire A-class spine the app leans on —
  extrude/revolve/loft/sweep, all transforms (M-TX), all reference geometry (M-REF), all
  query/analysis (subshape_ids @17, tessellate @12, mass_properties @7, bounding_box, moments),
  STEP import/export (@8/@12), and the *typical in-envelope* case of every blend/boolean —
  is SERVED-NATIVE. The M8 rehearsal MEASURED this: the full host suite is **53 PASS-native /
  0 crash / 0 silent-wrong**, and the per-op probe shows every A spine op served and every
  B/C op cleanly declining (id=0 + non-empty `cc_last_error`). By call-site weight, the
  overwhelming majority of the app's geometry calls (all query/analysis/transform/reference +
  in-envelope construct/blend/boolean/exchange) serve natively.
- **Clean-declines (and the app is RESILIENT to it).** The out-of-envelope A cases in §2(c) +
  IGES clean-decline (id=0). **The app is decline-resilient by construction:** `BRepShape.wrap`
  treats `id==0` as failure → `nil` (`BRepShape.swift:18`); `Workspace.applyEdgeEdit` turns a
  `nil` into a **user-facing `warn(...)`** that even appends `cc_last_error()` via
  `kernelDetail()` (`Workspace.swift`, "Fillet failed — the value may be too large … (kernel: …)"),
  pushes no history and leaves state intact; `booleanWithBody` returns `false` cleanly
  (`Workspace.swift:1046`). So an in-editor blend/boolean decline degrades to an honest
  "operation couldn't be performed" message — **not a crash, not a corrupted body.** This
  matches the kernel's clean-decline contract exactly.
  - **One resilience gap:** the IGES paths decline **silently** — `importIGES` returns `false`
    with no `warn()` (`Workspace.swift:2642`) and `exportIGES` returns `nil` so the export
    button produces no file, with no user message. Not a crash, but a poor UX under native-only.
- **Breaks (crashes / silent-wrong):** **NONE.** The M8 rehearsal measured zero crashes and
  zero silent-wrong results across the whole host suite and the per-op probe. Every declining
  op returns an honest error; no fabricated handle where OCCT would have served.

**Verdict:** After adoption, the app **would not break** on a native flip — it would *run*, with
the majority of calls served and the §2 gaps degrading to honest, mostly-surfaced declines. It
just would not yet be *feature-complete* versus the OCCT build: a user filleting a curved rim,
shelling a curved body, doing a freeform boolean, cutting a fine thread, or importing/exporting
IGES would hit a decline instead of a result.

---

## 4. Recommendation — shortest path to an app that ships on the native engine

Priority order (cheapest, highest-leverage first):

1. **App-side: ADOPT the kernel product (prerequisite, app-integration only).** Replace
   `KernelBridge.mm`'s inline OCCT with a link against `libcybercadkernel.a` + `cc_kernel.h`;
   keep every `cc_*` call-site and the Swift `BRepShape` wrapper unchanged (ABI is compatible).
   Keep OCCT linked initially so `cc_set_engine(0)` remains the safety net. **Nothing geometric
   blocks this.**
2. **App-side: make the IGES product decision** (the only C blocker the app uses). Cheapest
   ship: drop the IGES import/export UI (STEP already covers exact B-rep exchange, @8/@12 sites,
   fully A/native) — or keep a thin OCCT-linked IGES shim outside the OCCT-free core. Either
   unblocks IGES without native-kernel work.
3. **App-side: fix the two decline-UX gaps** so a native decline never fails silently — add a
   `warn()` on IGES import/export failure (and confirm the combine path surfaces its `false`).
   Small, and it makes the clean-decline contract honest to the user everywhere.
4. **Kernel-side: close the A-envelope gaps the app actually exercises**, in app-impact order:
   - **Curved-edge fillet** (`cc_fillet_edges`, 15 sites) — the single biggest app-facing gap;
     part of the **M2/M3 freeform-blend breadth (~3–8 py, shared)**.
   - **Curved shell / offset_face** (12 / 10 sites) — same M3 curved bucket.
   - **Freeform/mixed boolean residual** (`cc_boolean`, 13 sites) — **M2** breadth (~2–4 py).
   - **Fine-pitch helical/tapered thread** (6 / 5 sites) — **M7b construct tail (~0.5–2 py)**.
   - **Freeform-base wrap-emboss** (6 sites) — **M3** freeform-base.
   These are the *same* M2/M3/M7b moat the roadmap already tracks; no new work is created by the
   app — the app simply doesn't need the two hard-decline B ops (`fillet_edges_g2`,
   `thread_apply`) that the kernel still owes in the general case.
5. **Kernel-side: hold the M6 completeness/fuzzing bar** before the terminal OCCT unlink — this
   is the true gate that guarantees no silent-wrong across the whole input space once the oracle
   is gone (readiness §6.7).

**Bottom line.** The geometry is *closer* than the campaign framing implies: **no B-class op
blocks this app**, the app is already decline-resilient, and the M8 rehearsal proves zero
breakage under native-only. The real work is (1) the un-started **app-integration to adopt the
kernel product**, (2) the **IGES product decision**, and (3) filling the **curved-blend /
freeform-boolean A-envelope gaps** the app exercises — all of which are already on the moat
roadmap (M2/M3/M7b), none of which are new.

---

*Read-only audit. Sources: app `/Users/leonardoaraujo/work/cybercad`
(`KernelBridge.mm`, `BRepShape.swift`, `Workspace.swift`, `KernelBridgeAPI.h`, `project.yml`,
`openspec/specs/occt-usage/spec.md`); kernel `openspec/DROP-OCCT-READINESS.md` (§1 A/B/C table
+ §6 M8 MEASURED rehearsal), `openspec/MOAT-ROADMAP.md`,
`include/cybercadkernel/cc_kernel.h`.*
