# Proposal — unblock-native-step-sphere

## Why

The native STEP import reader (`add-native-step-import` → … → `add-native-step-revolution-quadrics`,
all archived) already maps BOTH a `SPHERICAL_SURFACE` keyword AND an on-axis-circle
`SURFACE_OF_REVOLUTION` to the SAME native `FaceSurface::Kind::Sphere`. **The surface mapping is done
and host-verified** (`revolvedCircle` → `Sphere`, `placedSurface(K::Sphere)`, both asserted in
`test_native_step_reader.cpp`). Yet a **full sphere solid** authored by OCCT still DECLINES → OCCT: the
archived revolution-quadrics slice flagged this honestly as an out-of-scope residual —

> an OCCT-authored sphere is a SINGLE periodic spherical face with a pole seam + degenerate pole
> vertices, which the reader's face reconstruction does not yet cover (a periodic-pole-face gap,
> independent of the revolution reduction).

The concrete blocker is a **topology** gap, not a surface gap. OCCT writes a full sphere as **one**
`ADVANCED_FACE` over a spherical surface that is simultaneously:

1. **u-PERIODIC** — the longitude `0 / 2π` meridian is a SEAM: one `EDGE_CURVE` referenced **forward
   AND reversed** in the same `EDGE_LOOP`. The reader ALREADY drops this pair (`senseMask == 3`, the
   landed periodic-wall seam logic).
2. **DOUBLE-POLE-DEGENERATE** — at each pole (`v = ±π/2`) a whole parametric `u`-edge **collapses to a
   single point** (the pole vertex): a zero-length / on-axis DEGENERATE edge. The reader does **NOT**
   recognise these degenerate pole edges. After the seam drop they survive as two spurious "closed rim"
   entries (each `v0 == v1`) that `chainLoop` keeps and `buildFaceWithPCurves` tries to give a pcurve —
   producing a face whose boundary does not close as a real trimming wire, so the watertight
   self-verify (`robustlyWatertightImport`) fails → honest DECLINE → OCCT (`cc_step_import` currently
   falls back; `vol == OCCT`, native `rel 0`).

This slice **closes the pole-degenerate periodic sphere FACE watertight** so the full sphere imports
natively — a **reader / topology-only** fix that leans on a capability the native tessellator ALREADY
has: **the sphere's own natural parametric bounds are the COMPLETE sphere** (`SurfaceEvaluator::bounds()`
returns `u ∈ [0, 2π], v ∈ [−π/2, π/2]` for `Kind::Sphere`), and the mesher's **no-boundary structured
grid** (`FaceMesher::mesh` → `!region.hasOuter()` → `structuredGrid(hasBoundary=false)`) meshes that
full rectangle watertight — the pole rows fan to the collapsed pole point and the `u = 0 / 2π` columns
weld at the seam, EXACTLY as any analytic sphere already tessellates. This is "the sphere already
tessellates elsewhere" the GOAL points at. The reader simply has to **recognise the seam + the two
degenerate pole edges and, when a spherical face is bounded by NOTHING ELSE (a genuine FULL sphere),
build the native `Sphere` face as a BARE periodic surface** (a Face with a NULL outer wire —
`makeFace` already accepts one, line 134) so it routes to the natural-bounds structured-grid mesh that
closes watertight. No tessellator change; no writer change; no new topology primitive.

**HONEST-OUT (unconditional).** If the pole-degenerate periodic sphere face cannot be built ROBUSTLY
watertight — the face is not a genuine full sphere (a partial spherical zone with a real latitude trim
survives the seam+pole drop), the surface does not reduce to `Sphere`, the parametric coverage is not
the full turn pole-to-pole, or the assembled solid fails the engine `robustlyWatertightImport`
self-verify — the reader **KEEPS THE CURRENT honest OCCT deferral** (the sphere already imports
correctly via OCCT). A correct still-deferred outcome is acceptable and reported plainly. **No
non-watertight or wrong sphere is ever fabricated; no tolerance is weakened.**

This does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES + a general-curved kernel
still block it). It is an additive topology fix that lets ONE more foreign primitive — the full
sphere — import natively.

## What changes

1. **Degenerate-pole edge recognition in `edgeLoop` (`step_reader.cpp`).** Today `edgeLoop` drops only
   the seam pair (`senseMask == 3`). It gains a second drop: a **degenerate pole edge** — an
   `EDGE_CURVE` whose 3D curve **collapses to a single point** over its parameter range (both endpoint
   `VERTEX_POINT`s coincide within a scale-relative tolerance AND the curve sweeps zero arc length /
   lies on the revolution axis). Such an edge is NOT a real rim: it is the pole singularity of a
   periodic surface. It is dropped alongside the seam. (A genuine full-circle rim edge — `v0 == v1` but
   a NON-zero circular sweep — is untouched: the existing cylinder/cone cap-rim path is byte-unchanged.)
   A `VERTEX_LOOP` face-bound (a single-vertex pole loop, if OCCT emits one) is likewise treated as
   contributing no wire edge rather than declining.
2. **Full-periodic-sphere face → bare-surface Face (`step_reader.cpp`).** When, after dropping the seam
   AND the degenerate pole edges, an `ADVANCED_FACE` whose surface is a native `Sphere` has **no real
   trimming edge left** (the loop covered only the seam + poles → a genuine FULL sphere), `advancedFace`
   builds the native `Sphere` face with a **NULL outer wire** (a bare periodic surface). `makeFace`
   already accepts a null outer (line 134); the tessellator's `!region.hasOuter()` path then meshes the
   sphere over its natural `[0,2π] × [−π/2,π/2]` bounds — watertight (poles fan, seam welds), the SAME
   structured-grid path a native-constructed analytic sphere uses. A spherical face that still carries a
   REAL latitude-trim wire after the drop keeps the existing wire path (a partial zone) — unchanged.
3. **Faithful + watertight self-verify gate (`step_reader.cpp` + engine, unchanged logic).** The
   bare-surface `Sphere` face is emitted ONLY when the surface faithfully reduces to `Sphere` (the
   existing `circleOnSurface` / `placedSurface` guard) AND the loop genuinely covered the full sphere
   (seam + two degenerate poles, no surviving real trim). The engine `robustlyWatertightImport`
   self-verify remains the FINAL arbiter: any assembled sphere solid that leaves a gap → DECLINE → OCCT.
   No engine gate is added or weakened.
4. **Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`).**
   `step_import_native` signature unchanged. Doc-comment updated: a full **periodic double-pole
   sphere** `ADVANCED_FACE` (a `SPHERICAL_SURFACE` keyword OR an on-axis-circle `SURFACE_OF_REVOLUTION`)
   now imports natively watertight by dropping the seam + the two degenerate pole edges and meshing the
   bare periodic sphere over its natural bounds; a non-full / pole-capped spherical zone that cannot
   close, and everything the prior slices declined, stay OCCT. OCCT-free, host-buildable.
5. **Engine hook + OCCT fallback (`native_engine.cpp`) — unchanged.** `step_import` still calls
   `step_import_native` then `robustlyWatertightImport` (per-member for a Compound); a full sphere that
   now self-verifies is kept native, any NULL / leaky result re-reads the SAME file via OCCT
   `STEPControl_Reader`. `iges_*` / `step_export` untouched.
6. **Tessellator / STEP writer policy (unchanged — the discipline the GOAL demands).** No tessellator
   change: the native analytic sphere already meshes watertight via the natural-bounds structured grid
   (verified for every constructed sphere); the fix only routes the imported full-sphere FACE onto that
   existing path. `step_writer.cpp` is NOT modified. No tolerance is weakened.
7. **Verification** — extend `tests/native/test_native_step_reader.cpp` +
   `scripts/run-sim-native-step-import.sh` + `tests/sim/native_step_import_parity.mm`: (A) a host
   fixture of a SINGLE periodic `SPHERICAL_SURFACE` face with a seam + two degenerate pole edges →
   native import is valid + watertight, volume = `4/3·π·R³`, matching the analytic-keyword /
   multi-lune sphere; (B) the SIM `runRevolvedSphere` fixture (OCCT `BRepPrimAPI_MakeSphere`, the exact
   periodic-pole-face B-rep) now imports **NATIVELY** watertight matching the OCCT re-import
   (count / volume / area / watertight / bbox) — flipping the currently-deferred case to native;
   (C) honest-out fixtures: a partial spherical zone that cannot close, and a degenerate / non-sphere
   revolution, still DECLINE → OCCT matching `cc_set_engine(0)`.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT.

## Non-goals (DEFERRED — keep the honest OCCT deferral, not faked)

- **A partial spherical ZONE with a real latitude trim + a single pole cap** (a pole-capped
  hemisphere) — a harder mixed real-wire + pole-collapse face; kept an honest DECLINE unless it
  genuinely self-verifies watertight. Not forced.
- **A `TOROIDAL_SURFACE` / off-axis-circle revolution (torus)**, an **ellipse / B-spline** revolution,
  a **skew** oblique line (hyperboloid) — no native kind; unchanged honest DECLINE (the landed
  revolution-quadrics precedent).
- **Modifying the tessellator or the STEP writer** — the natural-bounds structured grid already meshes
  a full analytic sphere watertight; the writer's own full-sphere serialisation limitation
  (three-lune pole-seam) is a SEPARATE, out-of-scope writer concern and is NOT touched.
- **Inventing geometry** — the imported sphere is the EXACT analytic sphere the file describes, merely
  re-tiled onto its natural parametric mesh; a face that cannot be reduced faithfully AND self-verified
  DECLINES. No tolerance is weakened.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved kernel still
  block it. Reported honestly.

## Impact

- `src/native/exchange/step_reader.cpp` — `edgeLoop` gains degenerate-pole-edge recognition (a
  zero-arc-length / on-axis edge is dropped like the seam); `advancedFace` builds a **bare-surface**
  `Sphere` face (NULL outer wire) when the seam+pole drop leaves a genuine FULL sphere; the
  faithful-reduction + watertight gate is unchanged. Every existing leaf builder (plane / cylinder /
  cone / partial-sphere-zone / cap-rim / seam-wall / revolution reductions) is byte-unchanged for files
  without a full periodic double-pole sphere face. `step_reader.h` / `native_exchange.h` doc-comments
  updated. OCCT-free, host-buildable. `step_writer.cpp` and the tessellator are NOT modified.
- `src/native/topology/**` — no new primitive; `makeFace` already accepts a null outer wire (a bare
  periodic surface); `Kind::Sphere` already round-trips.
- `src/native/tessellate/**` — **unchanged**; the `!region.hasOuter()` natural-bounds structured grid
  already meshes an analytic `Sphere` (bounds `u∈[0,2π], v∈[−π/2,π/2]`) watertight.
- `src/native/math/**` — no change; existing `Sphere` parametrization + point-on-surface geometry.
- `src/engine/native/native_engine.cpp` — **unchanged logic** (`robustlyWatertightImport`
  self-verifies). `iges_*` / `step_export` unchanged.
- `tests/native/test_native_step_reader.cpp` — a periodic double-pole `SPHERICAL_SURFACE` face imports
  watertight (vol = `4/3·π·R³`); the honest-out declines; the prior round-trips STILL pass.
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — `runRevolvedSphere`
  now asserts a **native watertight** import matching OCCT (was: native decline → OCCT parity). Own
  `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine stays
  OCCT. The prior import slices (sim `[NIMPORT]` 65/65 incl. revolution cylinder/cone/plane/sphere-
  reduction + honest torus/general declines), STEP export, healing, SSI S1–S5, blends/#6/#7,
  curved-boolean native-pass=13, marching, phase3 do NOT regress.

## Verification

1. **Host unit (OCCT-free).** A hand-authored ISO-10303-21 buffer with a SINGLE `SPHERICAL_SURFACE`
   `ADVANCED_FACE` whose `EDGE_LOOP` is the seam meridian (forward+reversed) + two DEGENERATE pole
   edges reconstructs to a native `Sphere` solid that is valid + watertight with volume `4/3·π·R³` and
   the correct bbox, IDENTICAL to the `SPHERICAL_SURFACE`-keyword multi-lune / on-axis-circle-revolution
   sphere. A partial spherical zone that cannot close, and a non-sphere revolution, DECLINE (NULL). The
   prior single-solid / assembly / trimmed-curve / revolution round-trips are unchanged.
2. **Sim vs OCCT (simulator, OCCT linked).** `xcrun simctl list devices booted` first. OCCT
   `BRepPrimAPI_MakeSphere` authors the exact single-periodic-pole-face sphere STEP; native
   `cc_step_import` (engine 1) imports it **NATIVELY watertight**; OCCT `STEPControl_Reader` re-imports
   the same file; the two agree on solid **count / volume / area / watertight / bbox** within tolerance
   — flipping `runRevolvedSphere` from a deferred (native parsed=0 → OCCT) case to a native import. The
   torus / general-revolution declines still fall through to OCCT identical to `cc_set_engine(0)`.

Done only when the relevant gates pass and every existing suite stays green at the OCCT default.
Reported honestly: this adds **native watertight import of a full periodic double-pole SPHERE face**
(a `SPHERICAL_SURFACE` keyword or an on-axis-circle `SURFACE_OF_REVOLUTION`) by dropping the longitude
seam + the two degenerate pole edges and meshing the bare periodic sphere over its natural bounds; a
non-full / pole-capped spherical zone that cannot close, a torus / hyperboloid / general revolution,
arbitrary directly-authored surfaces, and every previously-declined construct **stay OCCT**; if the
pole-degenerate face cannot be built robustly watertight the current honest OCCT deferral is KEPT;
arbitrary / AP242-general / IGES import remain OCCT and #8 `drop-occt` stays blocked.
