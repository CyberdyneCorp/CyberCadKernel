# NURBS scope decision — general NURBS B-rep vs the bounded hybrid

**Status:** decision on the record (2026-07-10). **Verdict: do NOT build the general NURBS
B-rep core speculatively. The bounded analytic + freeform-hybrid approach is the correct scope
for the app as it exists; expand into NURBS only when a concrete freeform-surfacing feature on
the app roadmap forces it — and then incrementally, demand-driven.**

This document exists because "OCCT has NURBS and we don't" recurs as a question. The honest
framing below separates the *facts* (what OCCT/Parasolid provide, what the reference apps use)
from the *decision* (what this kernel should invest in, and when).

---

## 1. Do the reference apps have general NURBS? — Yes, foundationally

Every pro CAD tool the app is benchmarked against runs on a **commercial general NURBS B-rep
kernel**:

| App | Geometry kernel | NURBS B-rep? |
|---|---|---|
| **SolidWorks** | Siemens **Parasolid** | ✅ full — solids + the entire surfacing suite are NURBS |
| **Shapr3D** | Siemens **Parasolid** (licensed) | ✅ full — solids *and* surfaces are Parasolid NURBS B-rep |
| Onshape | Parasolid | ✅ |
| Fusion 360 | Autodesk **ASM** (Parasolid lineage) | ✅ |

So if the bar is *literal* parity, general NURBS is table stakes — it is the underlying
representation that makes **arbitrary combinations of operations robust** and enables **freeform
surfacing**. CyberCadKernel deliberately chose a different point in the design space (below).

---

## 2. What "NURBS in OCCT/Parasolid" actually is (a layered stack)

NURBS is not one feature — it is a stack. Here is the stack vs. what this kernel has today:

| Layer | OCCT / Parasolid | CyberCadKernel today |
|---|---|---|
| **Geometry** — rational B-spline curves/surfaces, arbitrary degree, knots/mults; De Boor eval + derivatives | full | ✅ **partial** — NURBS eval + projection (`src/native/numerics`), Bézier/BSpline trimmed faces (`shape.h`), STEP admission of B-spline surfaces/curves + trims |
| **Geometry algorithms** — knot insert/remove, degree elevate/reduce, split, reparametrize | full | ✅ **exact-NURBS kernel** — OCCT-free, substrate-free `src/native/math/bspline_ops.{h,cpp}`: knot insert/refine/remove, degree elevate/reduce, split, Bézier decompose, reparam for curves + tensor surfaces, rational-aware (homogeneous lift). Exact-preserving (knot-removal / degree-reduction report a bounded error and decline honestly). Host-analytic gate `tests/native/test_native_nurbs_ops.cpp`. Layers 2–8 below remain demand-gated. |
| **Fitting / approximation** — points→B-spline curve/surface, interpolation | full | 🟡 **partial** — OCCT-free, numsci-gated `src/native/math/bspline_fit.{h,cpp}` (NURBS-SCOPE Layer 7): chord-length / centripetal parametrization, averaging + approx knots, **non-rational** global curve INTERPOLATION (A9.1, `lin_solve` collocation), least-squares curve APPROXIMATION (A9.4/9.6, endpoints pinned, `lstsq`), and tensor-product surface interp/approx. Airtight host gate `tests/native/test_native_nurbs_fit.cpp` (interpolation exact ~1e-15, idempotent round-trip ~1e-14, monotone approximation convergence). Rational (weighted) fitting + advanced surfacing (skin/loft/Gordon) are the residual. |
| **NURBS intersection (SSI)** — arbitrary NURBS↔NURBS incl. tangent/singular | full (`IntPatch`) | 🟡 **bounded** — analytic + bounded-freeform SSI; general skew-NURBS↔NURBS honestly declines |
| **NURBS B-rep boolean** — arbitrary trimmed-NURBS faces, tolerant, non-manifold | full (`BOPAlgo`) | 🟡 **bounded** — the M2 spine covers the app's poses; general NURBS↔NURBS is the deep tail |
| **NURBS fillet/chamfer** — variable-radius / G2 on arbitrary surfaces | full (`ChFi3d`) | 🟡 analytic-revolve + canal only |
| **NURBS offset / thicken** | full (`BRepOffset`) | 🟡 analytic walls only |
| **Exact NURBS sweep/loft/surfacing** — GeomFill/BRepFill, Gordon/network/plate | full | ❌ native produces *faceted* results, not exact NURBS |
| **Trimmed-NURBS B-rep data model** — exact pcurves, tolerant topology, NURBS healing | full | 🟡 represented + meshed; not exact-B-rep-manipulated |

**Key architectural fact:** this kernel is an **analytic + tessellation hybrid with bounded
freeform support** — it *evaluates* NURBS and handles *bounded* freeform intersection/boolean/mesh,
but it is **not** a general exact-NURBS B-rep modeler. That was a deliberate choice: it let the
drop-OCCT campaign serve the app's real needs at ~15–30 py total instead of the ~20–40 py the
general NURBS core alone would cost (§4).

---

## 3. Is it useful for *this* app? — depends on the product direction

NURBS is mostly the *underlying representation*, not a user-facing feature. What users actually
*do* splits into two regimes:

| Regime | Needs general NURBS? | The app today |
|---|---|---|
| **Mechanical / parametric CAD** — sketch → extrude/revolve, booleans, fillet/chamfer, shell, draft, patterns, mates, on mostly analytic + simple-curved geometry | **No** — the bounded approach covers the common cases | ✅ this is exactly the measured `cc_*` usage (see DROP-OCCT-READINESS / APP-MIGRATION-READINESS) |
| **Freeform / industrial-design surfacing** — loft/boundary surfaces between arbitrary curves, class-A styling, organic / sub-D→NURBS, filling complex gaps, blends over already-curved regions | **Yes** — this is where the analytic/bounded approach runs out | ❌ not in the app's current surface |

**CAE angle specifically:** CAE (FEA/CFD prep) does **not** drive NURBS. It needs *good meshes* +
*accurate watertight geometry* — already provided by the native tessellator + TetGen volume
meshing + the render-quality display mesh. NURBS would matter for CAE only for isogeometric
analysis (niche) or high-fidelity NURBS *import* — and STEP already covers exact B-rep exchange.

---

## 4. Programming-years to build the general NURBS B-rep core

Rough, dependency-ordered (each builds on the one above):

| Sub-capability | Est. py | Note |
|---|---:|---|
| Complete exact-NURBS geometry kernel (knot ops, elevation, split, robust rational eval) | 1–2 | much of the eval exists |
| **General NURBS↔NURBS SSI** (all configs, tangent/singular robust) | 3–6 | research-grade; OCCT `IntPatch` is decades-refined |
| **General exact-NURBS B-rep boolean** (BOPAlgo-class, tolerant, non-manifold) | 5–10 | the hardest — *the core of a B-rep kernel* |
| General NURBS fillet/chamfer (ChFi3d-class) | 3–6 | needs the boolean + SSI |
| NURBS offset / thicken (robust) | 2–4 | |
| Exact NURBS sweep/loft/surfacing (GeomFill/Gordon/plate) | 2–4 | |
| Fitting / approximation (points→NURBS) | 1–2 | 🟡 non-rational curve+surface interp/approx **landed** (`src/native/math/bspline_fit`, Layer 7); rational (weighted) fitting + advanced surfacing residual |
| Trimmed-NURBS B-rep data model + pcurve robustness + NURBS healing | 2–4 | |
| **Total (OCCT/Parasolid-class general NURBS modeling)** | **≈ 20–40 py** | ~a decade for a small team |

For scale: this is **~10× the entire remaining drop-OCCT moat** (~1.5–3.5 py). Parasolid/OCCT
each represent ~25 years of this work; it is not obtained cheaply.

---

## 5. Decision

- **Do NOT take on the general NURBS core as one program.** It is the single largest possible
  scope item, and the app as it exists (mechanical/parametric per its measured `cc_*` usage) does
  not consume it.
- **The bounded pieces are the correct 80/20** — bounded N-sided fill (Coons/Gregory patch over
  analytic boundaries → tessellated fill), bounded freeform booleans, curved blends — deliver the
  practical freeform value at a tiny fraction of the cost, and are landed / landing demand-driven.
- **Trigger to reconsider:** if the app commits to genuine **freeform / organic / class-A
  surfacing** (the Shapr3D "arbitrary curves → smooth NURBS skin → tweak" workflow). Then build the
  NURBS layers **incrementally, driven by the specific surfacing features committed to** — SSI and
  boolean first, as they gate the rest — never speculatively.
- **CAE does not change this** — it is served by meshing + accurate geometry + STEP exchange, all
  already present.

**One-line summary:** the reference apps have general NURBS (via Parasolid) and it is foundational
to them, but it is a ~20–40 py investment this app does not need for mechanical/parametric CAD+CAE;
pursue it only if — and incrementally as — the product moves into freeform surfacing.
