# Tasks — expose-nurbs-cc-facade

Verification legend: **abi** = `test_abi` + a `sizeof`-guard test cross-checked
against a C `sizeof` of the header; **host** = a native/facade ctest asserting real
geometry through the new `cc_*` calls; **py** = `python -m pytest python/tests` on
the desktop dylib; **png** = a gallery script writes a validated PNG. Acceptance
bar: every wrapped feature is constructible, inspectable, tessellable, and
**shown** from Python, with the honest-decline contract asserted and the existing
154 `cc_*` symbols byte-unchanged.

Status: **not started** — this is the scoping change. Boxes are the planned
delivery; implementation lands as waves J1–J7 (see `design.md` §7).

## 1. J1 — ABI foundation: NURBS geometry handles
- [x] 1.1 Add `cc_kernel_nurbs.h` (included by `cc_kernel.h`) declaring the opaque
  `cc_curve` / `cc_surface` handle types and the `CCCurveInfo` / `CCSurfaceInfo` /
  `CCTessOptions` POD structs. Existing header + 154 symbols byte-unchanged. (**abi**)
- [x] 1.2 Registry-backed handle allocation mirroring `CCShapeId`: create-from-data
  (`cc_curve_create`, `cc_surface_create` taking degree/knots/poles-in-homogeneous),
  `cc_curve_release` / `cc_surface_release` (idempotent, crash-free double release,
  stale-handle guard). (**host**)
- [x] 1.3 Accessors: `cc_curve_info`/`cc_surface_info` + buffer-fill
  `cc_curve_knots`/`_poles`, `cc_surface_knots_u`/`_v`/`_poles` (homogeneous
  x,y,z,w; row-major surface poles; `<0` on too-small buffer). (**host**, **abi**)
- [x] 1.4 Evaluators `cc_curve_eval` / `cc_surface_eval` — point on the exact
  rational geometry (verified against the native evaluator to ≤1e-12). (**host**)
- [x] 1.5 Display tessellation bridge `cc_surface_tessellate` → `CCMesh`,
  `cc_curve_polyline` → `CCEdgePolyline`. Single-surface display mesh ONLY; carries
  the non-watertight caveat (NOT the curved-seam weld). (**host**)
- [x] 1.6 `sizeof` guards for every new struct, cross-checked against a C `sizeof`
  of the header (as `add-python-binding` §8.2). (**abi**)

## 2. J2 — fitting / reverse-engineering + analytic conversion wrappers
- [ ] 2.1 `cc_nurbs_fit_curve`/`_surface`, `_interp_curve`, `_estimate_weights_*`,
  `_fit_curve_constrained`/`_fit_surface_constrained`, `_fair_curve`/`_surface`,
  `_simplify_curve` → `cc_curve`/`cc_surface` (or honest decline). (**host**)
- [ ] 2.2 `cc_nurbs_detect_primitive`, `_recognize_curve`/`_surface` — return the
  recognized primitive type + params, else "general". (**host**)
- [ ] 2.3 `cc_nurbs_circle`/`_arc`/`_ellipse`/`_plane`/`_cylinder`/`_cone`/`_sphere`/
  `_torus` — exact rational NURBS from analytic params. (**host**)

## 3. J3 — surfacing wrappers
- [ ] 3.1 `cc_nurbs_skin`, `_gordon`, `_coons`, `_nsided_fill` (mode C0/G1/G2/
  rational), `_sweep_variable`, `_sweep_two_rail`, `_revolve`, `_join` (mode
  G1/G2) → `cc_surface` (or decline). (**host**)

## 4. J4 — blend + offset/thicken wrappers
- [x] 4.1 `cc_nurbs_fillet_freeform_g2`, `_vertex_blend`, `_chamfer_variable`,
  `_chamfer_freeform` → `cc_surface` (or decline). (**host**)
- [x] 4.2 `cc_nurbs_offset_rational`, `_offset_trimmed`, `_thicken_trimmed`,
  `_shell_trimmed`. Thicken/shell trimmed results carry the self-intersection-free
  guarantee already proven natively; never emit a self-intersecting result. (**host**)

## 5. J5 — intersection + trim boolean wrappers
- [x] 5.1 `cc_nurbs_intersect_cc` / `_intersect_cs` — hit points + params,
  transversal/tangential flag, coincident → decline. (**host**)
- [x] 5.2 `cc_nurbs_trim_region_boolean` (∪/∩/∖) on trim regions — the trimmed-face
  assembly stage; areas machine-exact, coincident → decline. (**host**)

## 6. J6 — Python object model
- [ ] 6.1 `Curve` / `Surface` RAII classes over the new handles (context manager +
  GC backstop + stale-handle guard), NumPy pole/knot/weight accessors, `.eval`,
  `.tessellate` → `Mesh`. Low-level tables (`_cffi`/`_ffi`) extended in lockstep;
  `test_ffi` symbol-count + `sizeof` guards updated. (**py**, **abi**)
- [ ] 6.2 `nurbs` submodule surfacing every `cc_nurbs_*` wrapper, following the
  `api.py` RAII / `KernelError`-from-`cc_last_error` pattern. Honest declines raise
  `KernelError`, asserted (not fabricated). (**py**)
- [ ] 6.3 Real-geometry pytest: evaluation exactness vs analytic; a closed surface
  (revolved sphere) tessellates watertight (`trimesh.is_watertight`); round-trip
  `recognize(circle) == circle`; a fit through sampled points reproduces them;
  over-constrained fit / over-radius fillet RAISE. (**py**)

## 7. J7 — example gallery (showcase each feature)
- [ ] 7.1 One mechanical-piece example per Wave D–I feature family, each rendering a
  PNG (extending the existing 15-piece gallery under `examples/`): a skinned/
  Gordon-lofted bracket; an N-sided-filled boss (G2); a chamfered + vertex-blended
  corner; a freeform-G2-filleted boss; a reverse-engineered primitive from a point
  cloud; a faired scan patch; a trim-boolean pocket; a variable-section swept
  handle. (**png**)
- [ ] 7.2 A gallery index (thumbnails + the feature each piece exercises) linked
  from `examples/README.md`; each script is a runnable, commented usage sample of
  the `nurbs` Python API. (**png**)

## 8. Docs & validation
- [ ] 8.1 `docs/python.md` + `python/README.md`: the `Curve`/`Surface`/`nurbs`
  surface, with the display-tessellation-not-watertight caveat stated plainly.
- [ ] 8.2 `openspec/ROADMAP.md` change-index row + capability `nurbs-cc-facade`.
- [ ] 8.3 `openspec validate expose-nurbs-cc-facade --strict` green; `test_abi`
  green (existing symbols byte-unchanged); full host ctest + pytest green.

## 9. Explicit non-goals (out of scope for this change)
- [ ] 9.1 **Watertight multi-face curved-seam weld** — the frozen-mesher wall — is
  NOT a dependency of this change: the `cc_nurbs_*` exposure and gallery stand on
  single-surface display tessellation and honest face-sets regardless. It is being
  attacked **in parallel** as a companion track **W** (measurement-first; see
  `nurbs-watertight-weld` scoping / the W-wave). If W genuinely lands a watertight
  curved-seam weld for a case, the facade MAY expose that case as a sewn solid
  (still bound by the "never claim watertight unless it IS" requirement); if W
  honest-declines with a sharpened architectural map, the gallery keeps its
  face-set presentation. Either outcome leaves this change's requirements intact.
- [ ] 9.2 **iOS Swift binding** — tracked in `add-swift-binding`; it would consume
  these same `cc_*` additions, but the Swift layer is not built here.
- [ ] 9.3 **Rewiring existing engine-level facade ops** (`cc_solid_sweep`,
  `cc_fillet_edges`, …) onto the exact-NURBS backends — those keep their current
  engine implementations; this change adds a parallel `cc_nurbs_*` surface.
- [ ] 9.4 **General freeform multi-surface sew + the SSI multi-branch floor** —
  kernel residuals, not exposure targets.
