# add-native-geometry-completion

Phase 4 capability **#4b Tier 1 + Tier 2#4 — native geometry completion**. The honest
batch that closes the remaining `native-construction` residuals left OCCT-fall-through
by Tiers A–D (holed / typed-profile extrudes + typed-profile revolve, 2-section ruled
loft, constant-frame sweep, threads + tapered shank). It attempts to move NATIVE — each
only where it can be verified a watertight, oracle-correct solid — four residual areas:

- **(A) Construction residuals** — a **kind-3 SPLINE** profile outer edge in
  `cc_solid_extrude_profile*` / `cc_solid_revolve_profile`, and an **OFF-AXIS-ARC** revolve
  (an arc whose supporting-circle centre lies off the axis → a **TORUS** surface of
  revolution — this change ADDS a native `Torus` surface).
- **(B) N-section loft** — `cc_solid_loft` / `cc_solid_loft_wires` for **3+ sections** via a
  C1/ruled skin chained through the N sections.
- **(C) General sweep** — a **non-planar spine + accumulating twist/scale**
  (`cc_solid_sweep` / `cc_twisted_sweep`) via a **rotation-minimizing frame (RMF)**, plus a
  **best-effort guided / rail** sweep (`cc_guided_sweep` / `cc_loft_along_rail`).
- **(D) Fine-pitch / near-self-intersecting threads** — a **self-intersection resolver** so
  more thread parameters weld watertight; genuinely self-intersecting threads still fall
  through.

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT), and
does NOT fake any sub-case — anything not genuinely native + verified (watertight solid,
correct volume/geometry) is DISCARDED and falls through to OCCT in `NativeEngine`,
labelled and verified rel~0, never faked.

## Honesty statement (read this first)

These are the residuals precisely because they are the hard part of each area. This
change is **attempt + honest fall-through**, exactly as every prior native tier:

- A native result is accepted only when it is a **valid watertight solid with the correct
  volume/geometry**, verified on BOTH gates (host analytic + sim native-vs-OCCT parity).
  Otherwise the native builder returns a NULL `Shape` (or the engine's self-verify
  DISCARDS the candidate) and the op falls through to OCCT — labelled `[fallback]`,
  verified rel~0, **never faked**.
- Cases that genuinely require **surface–surface intersection (SSI)** are explicitly OUT
  OF SCOPE here and MUST fall through to OCCT (that is Tier 4). This change does NOT
  attempt SSI. In particular: a **self-intersecting sweep** (the swept surface folds
  through itself), a **tight-curvature spine** whose section folds, a **hard
  pipe-shell rail** that cannot be resolved without trimming, and a **genuinely
  self-intersecting thread** (turns fold through each other) all stay OCCT.
- The final status is reported **per area** as the real native-vs-fallback split — an op
  is "native" only where both gates are green; otherwise it is documented deferred, its
  spec requirement satisfied by the (MODIFIED) deferred-ops requirement.

## Areas and the `cc_*` contracts

### (A) Construction residuals

- **kind-3 SPLINE profile edge** in `cc_solid_extrude_profile` /
  `cc_solid_extrude_profile_polyholes` — a `CCProfileSeg` of `kind == 3` carries a poly-line
  of control points (`splineXYCount` DOUBLES = 2× the point count, per the `cc_kernel.h`
  contract) that defines a B-spline outer edge. Natively: build a `BSpline` edge curve
  (the existing `src/native/math/bspline.h` evaluator) and EXTRUDE it into a B-spline (or
  ruled-to-a-translate) side wall; the extrude is a straight translation so the wall is a
  general cylinder over the spline — a swept B-spline surface. Cap with the two planar
  end faces (the spline profile is planar). A spline in a **revolve** profile
  (`cc_solid_revolve_profile`) is a general B-spline **surface of revolution**.
- **OFF-AXIS-ARC → TORUS** in `cc_solid_revolve_profile` — a kind-1 arc segment whose
  supporting circle centre is OFF the revolve axis sweeps a **torus** (major radius =
  centre's distance from axis, minor radius = the arc's circle radius). The existing
  revolve only handles a line (→ Plane/Cylinder/Cone) and an ON-axis arc (→ Sphere band).
  This change ADDS a native `Torus` surface (`src/native/math` + a `FaceSurface::Kind::Torus`)
  and a torus band face for the off-axis arc.

### (B) N-section loft

- **`cc_solid_loft(bottomXY, topXY, depth)` / `cc_solid_loft_wires(...)`** — Tier B did TWO
  sections; the facade also chains **≥3 sections** (a loft "chain"). Natively: skin the N
  sections pairwise (section `k` → section `k+1`) with the Tier-B `ruledSideFace` bilinear
  bands, sharing each interior section's ring between the band below and the band above (a
  C0/ruled skin; C1 where the section tangents agree), capped only at the two ends → one
  watertight solid. Requires EQUAL vertex counts across all sections (the ambiguous
  resample stays OCCT).

### (C) General sweep

- **`cc_solid_sweep` / `cc_twisted_sweep` — non-planar spine + accumulating twist/scale.**
  Tier C did straight + smooth-planar spines with a CONSTANT frame (OCCT's planar
  corrected-Frenet law) and only the no-op twist. This change transports the section along
  a **NON-PLANAR** spine with a **rotation-minimizing frame (RMF)** (double-reflection
  method, Wang et al. 2008), and applies an **accumulating twist** (linear 0→`twistRadians`)
  and **linear scale** (1→`scaleEnd`) about the frame — the general `cc_twisted_sweep`. A
  spine/section that would self-intersect or fold (tight curvature) is GUARDED and falls
  through (SSI is Tier 4).
- **`cc_guided_sweep(profileXY, pathXYZ, guideXYZ)` / `cc_loft_along_rail(railXYZ,
  profileA, profileB)` — best-effort.** A guide/rail curve drives the section
  orientation (guided sweep) or interpolates between two end profiles along a rail
  (rail loft). Native where the guide-driven frame yields a watertight, oracle-correct
  solid; the hard pipe-shell rail cases (that need SSI/trimming) fall through.

### (D) Fine-pitch / near-self-intersecting threads

- **`cc_helical_thread` / `cc_tapered_thread` — self-intersection resolver.** Tier D landed
  the radial-V helical thread native for clearing parameters but GUARDED fine-pitch /
  large-depth threads to OCCT. This change adds a **self-intersection resolver**: for the
  NEAR-self-intersecting band (the flanks approach but do not truly cross), it TRIMS /
  clamps the V so more thread parameters weld watertight; a **genuinely self-intersecting**
  thread (turns truly fold through each other — a non-manifold result no matter how the
  vertices weld) still fails the watertight self-verify and falls through to OCCT.

## Scope

| Area | Op(s) | Attempted native in this change | Falls through to OCCT (honest, labelled) |
|---|---|---|---|
| **A** | `cc_solid_extrude_profile*` (kind-3 spline edge) | **ATTEMPTED** — B-spline edge + swept-B-spline side wall + planar caps | Non-planar spline loop; a spline that self-intersects |
| **A** | `cc_solid_revolve_profile` (off-axis arc) | **ATTEMPTED** — native `Torus` surface of revolution band | Spline-revolve (general B-spline surface of revolution); an arc/torus that self-intersects the shell |
| **B** | `cc_solid_loft` / `_wires` (≥3 sections) | **ATTEMPTED** — chained ruled skin through N equal-count sections + 2 end caps | Mismatched vertex counts (ambiguous resample); a non-planar cap section |
| **C** | `cc_solid_sweep` / `cc_twisted_sweep` (non-planar + twist/scale) | **ATTEMPTED** — RMF transport + accumulating twist/linear scale | **Self-intersecting / tight-curvature** spine (SSI — Tier 4) |
| **C** | `cc_guided_sweep` / `cc_loft_along_rail` | **BEST-EFFORT** — guide/rail-driven frame where watertight + oracle-correct | **Hard pipe-shell rails** needing SSI/trimming (Tier 4) |
| **D** | `cc_helical_thread` / `cc_tapered_thread` (fine pitch) | **ATTEMPTED** — self-intersection resolver welds MORE parameters watertight | **Genuinely self-intersecting** threads (turns fold through — SSI) |

### What is explicitly NOT attempted here (SSI — Tier 4)

Surface–surface intersection is out of scope. Any case whose valid B-rep requires
intersecting two swept/curved surfaces and trimming — a self-intersecting sweep, a
tight-curvature fold, a hard pipe-shell rail, a truly self-intersecting thread — MUST fall
through to OCCT. This change does not attempt SSI and does not ship a solid that would
need it.

## Method (locked, per NATIVE-REWRITE.md)

Clean-room from the `cc_*` contracts (`include/cybercadkernel/cc_kernel.h`) and
computational-geometry first principles (B-spline evaluation — *The NURBS Book*; torus
parametrization; rotation-minimizing frames — Wang, Jüttler, Zheng, Liu 2008,
"Computation of rotation minimizing frames"; ruled/skinned surface tiling), with OCCT
source (`/Users/leonardoaraujo/work/OCCT/src`:
`BRepBuilderAPI`/`BRepPrimAPI`/`BRepOffsetAPI` — `BRepOffsetAPI_ThruSections` for the
N-section loft, `BRepOffsetAPI_MakePipe`/`MakePipeShell` + `GeomFill` for the general
sweep and its frame law, `BRepPrimAPI_MakeRevol` for the torus surface of revolution)
consulted as a **reference oracle only** — never copied. The new builders REUSE the
already-parity-verified assemblers: Tier-B `ruledSideFace`/`planarFace` (loft, sweep,
thread bands), Tier-C station-ring transport, the revolve's per-segment surface
classification, and the two-stage shared-1D-discretization tessellator's watertight weld.

## Architecture / OCCT boundary (unchanged from #4 / #4b Tiers A–D)

- New / extended native builders live under `src/native/construct/` (`profile.h` for the
  spline edge + off-axis-arc torus revolve, `loft.h` for the N-section chain, `sweep.h`
  for the RMF + twist/scale + guided/rail) and `src/native/math/` (a new `Torus` surface in
  `elementary.h`, RMF in a sweep helper) with a `FaceSurface::Kind::Torus` added to
  `src/native/topology/shape.h`. All stay **OCCT-FREE and host-buildable**
  (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no simulator); they include
  only `src/native/math` + `src/native/topology` + `src/native/tessellate` + the existing
  `src/native/construct/` assemblers and return a `topology::Shape`.
- `src/engine/native/native_engine.{h,cpp}` — `solid_loft`, `solid_loft_wires`,
  `solid_sweep`, `twisted_sweep`, `guided_sweep`, `loft_along_rail`,
  `solid_extrude_profile*`, `solid_revolve_profile`, `helical_thread`, `tapered_thread`
  keep their native-else-fallback wiring; a NULL native result (or a failed self-verify)
  falls through to the fallback with no interception. A MANDATORY runtime self-verify
  (`robustlyWatertight` + correct-volume/geometry) DISCARDS any candidate that is not a
  valid watertight solid → OCCT. OCCT stays behind `CYBERCAD_HAS_OCCT`; the native builder
  never sees OCCT.
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via `cc_set_engine(1)`),
  so every existing suite is unchanged unless it opts in.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT) on the built native B-rep +
   its native tessellation: a spline-edge extrude is watertight with the B-spline wall
   + planar caps and a deflection-bounded volume; an off-axis-arc revolve is a watertight
   torus band with the analytic torus volume `2π²·R·r²·(angle/2π)`; an N-section loft is
   watertight with the summed frustum volume; an RMF sweep is watertight with the correct
   Pappus/twist volume and a frame that is rotation-minimizing (zero twist about the
   tangent between stations, up to the applied twist); the thread self-intersection
   resolver welds a previously-guarded near-self-intersecting thread watertight, while a
   genuinely self-intersecting one still returns NULL. Every self-intersecting / SSI /
   degenerate input returns NULL.
2. **Simulator native-vs-OCCT parity through the facade**: each op called native
   (`cc_set_engine(1)`) vs OCCT (default), compared on mass properties / bbox / sub-shape
   counts / watertight tessellation within a documented deflection tolerance vs the
   `BRepOffsetAPI_ThruSections` / `MakePipe`/`MakePipeShell` / `MakeRevol` oracle. Every
   deferred sub-case (SSI, mismatched counts, genuinely self-intersecting thread) asserted
   **identical** under both engines (fall-through proof). Default restored in teardown; the
   parity harness carries its own `main()` (on the `run-sim-suite.sh` SKIP list) so the
   221-assertion suite count is unchanged.

A requirement is done only when BOTH gates are green for that op AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) stays green at the OCCT
default. **Honest outcome:** each area lands native only where both gates pass; whatever
cannot be made a watertight, oracle-correct solid (especially anything needing SSI)
remains labelled OCCT fall-through and the change reports the real per-area split.
