# Design — add-native-geometry-completion (Phase 4 #4b Tier 1 + Tier 2#4)

## 0. Context and honest posture

This change closes the remaining `native-construction` residuals across four areas, each
ATTEMPTED natively and kept as labelled OCCT fall-through wherever it cannot be made a
watertight, oracle-correct solid:

- **(A)** kind-3 SPLINE profile edge (extrude + revolve) + OFF-AXIS-ARC → TORUS revolve.
- **(B)** 3+-section loft chain.
- **(C)** non-planar-spine + twist/scale sweep (RMF) + best-effort guided/rail.
- **(D)** a thread self-intersection RESOLVER (weld more fine-pitch params watertight).

The design below specifies each and is explicit at every hard step about what defers.
Native code stays OCCT-free + host-buildable (`clang++ -std=c++20`); OCCT is a **reference
oracle only** (`/Users/leonardoaraujo/work/OCCT/src`: `BRepBuilderAPI`, `BRepPrimAPI`,
`BRepOffsetAPI` — `ThruSections`, `MakePipe`/`MakePipeShell`, `MakeRevol` — and `GeomFill`
for the frame law). No `cc_*` ABI change; the default engine stays OCCT.

**Locked rule (every area):** a native result is accepted ONLY if it self-verifies as a
valid watertight solid with the correct volume/geometry (`NativeEngine`'s
`robustlyWatertight` + volume/geometry check); otherwise DISCARD → OCCT, labelled,
verified rel~0, never faked. **Surface–surface intersection (SSI) is NOT attempted here**
(Tier 4). Self-intersecting sweeps, tight-curvature folds, hard pipe-shell rails, and truly
self-intersecting threads MUST fall through.

---

## 1. (A) Construction residuals

### 1.1 kind-3 spline profile edge (extrude)

**Contract.** `cc_solid_extrude_profile(segs, segCount, depth, …)` /
`cc_solid_extrude_profile_polyholes(...)` take typed `CCProfileSeg`s. A `kind == 3`
segment is a B-spline outer edge; its control points live on the side-channel
(`splineXY` / `splineXYCount`, where `splineXYCount` is the number of DOUBLES = 2× the
control-point count, per the `cc_kernel.h` guard). Today `profile.h` returns
`std::nullopt`/NULL on any kind-3 segment → OCCT.

**Native construction.**
1. **Resolve the B-spline edge.** Build a `topo::EdgeCurve` of `Kind::BSpline` from the
   control points, a clamped uniform knot vector, and a chosen degree (min(3, n−1)); the
   existing `src/native/math/bspline.h` de-Boor evaluator (`curvePoint`) samples it. The
   profile loop is the ordered concatenation of the typed segments (lines / arcs / the
   spline edge), which must close and be PLANAR (all control points + segment endpoints
   coplanar with the profile plane z = const).
2. **Extrude the spline edge.** The extrude is a straight translation by `depth·ẑ`. The
   swept side face over the spline edge is a `FaceSurface::Kind::BSpline` surface: a degree
   `(p, 1)` B-spline whose U poles are the edge's control points and whose V rules from
   `pole` to `pole + depth·ẑ` (a general cylinder over the spline). The rim edges (bottom
   spline edge, top spline edge) are shared `BSpline` curves; the two ends are planar
   caps (`Plane`) over the whole profile loop (triangulated by the multi-hole cap
   triangulator, which already handles a spline-boundary loop as a dense polyline).
3. **Watertight weld.** The side wall and the caps share the spline rim edges; the
   two-stage tessellator's shared-1D discretization welds them watertight, exactly as the
   arc-cylinder wall welds today.

**Deferred (NULL → OCCT):** a NON-planar spline loop, or a spline that self-intersects
(the profile is not a simple closed curve). The volume is deflection-bounded (a spline
area has no closed form for an arbitrary control net; the host test uses a spline that
reduces to a known shape — e.g. control points on a circle → area ≈ πr² within the bound —
plus the watertight + convergence check).

### 1.2 native `Torus` surface + off-axis-arc revolve

**Contract.** `cc_solid_revolve_profile(segs, segCount, axis…, angle, …)`. Today the
classifier handles a kind-0 line (→ Plane/Cylinder/Cone) and a kind-1 arc whose supporting
circle centre lies ON the axis (→ Sphere band). A kind-1 arc whose circle centre is OFF the
axis sweeps a **torus**, which had no native surface — today it returns NULL → OCCT.

**Native `Torus` surface** (`src/native/math/elementary.h`). Add
`struct Torus { Ax3 frame; double major; double minor; }` with:
```
  point(u, v)  = C + (major + minor·cos v)·(cos u·X + sin u·Y) + minor·sin v·Z
  normal(u, v) = cos v·(cos u·X + sin u·Y) + sin v·Z   // outward
```
where `u` is the revolve angle about the axis, `v` the angle around the tube, `X,Y,Z` the
frame axes (`Z` = revolve axis, `C` = axis point at the tube-centre height). Add
`FaceSurface::Kind::Torus` to `src/native/topology/shape.h` (reusing `frame` + `radius`
for the minor radius + a second field / the existing `semiAngle` slot repurposed, or a
`radius2`; the delta spec calls for a torus-carrying `FaceSurface`).

**Off-axis-arc revolve band** (`src/native/construct/construct.h` /`profile.h`). For a
kind-1 arc with circle centre `(cx, cy)` at distance `R = dist(centre, axis)` and radius
`r`:
- major radius `R`, minor radius `r`; the torus frame's `Z` = the revolve axis, `C` = the
  axis point at the arc-centre height, and the tube-angle range `[v0, v1]` = the arc's own
  angular span, the revolve-angle range `[0, angle]` = `u`.
- the swept face is a `Torus` band trimmed to `u ∈ [0, angle]`, `v ∈ [v0, v1]`; its rim
  edges are `Circle` arcs (the arc endpoints revolved) shared with the neighbour segments.
- a full 2π revolve closes the `u`-loop (periodic — tiled in < π patches as the other
  full-turn surfaces are); a partial angle adds two planar meridian caps.

**Volume check (host).** A full-circle tube revolved 2π (a solid torus) has the exact
Pappus volume `2π²·R·r²`; a partial arc/angle scales by the swept fractions. The native
tessellation converges to this within the deflection bound.

**Deferred (NULL → OCCT):** a **spline-revolve** (a kind-3 segment → a general B-spline
surface of revolution — no native surface-of-revolution-of-a-spline), and an arc/torus
whose swept band self-intersects the rest of the shell (would need SSI to trim).

---

## 2. (B) N-section ruled loft chain

**Contract.** `cc_solid_loft(bottomXY, topXY, depth)` /
`cc_solid_loft_wires(aXYZ, bXYZ)`. Tier B skinned exactly TWO sections. The facade's loft
chain passes ≥3 sections (the multi-section overloads / the reference-geometry chain); the
native builder returns NULL for >2 sections → OCCT.

**Native chain** (`src/native/construct/loft.h`). Given N ≥ 2 planar sections
`S₀ … S_{N-1}`, each a closed polygon of the SAME vertex count `n ≥ 3`:
1. build one shared per-section vertex ring `ring[k][i] = makeVertex(S_k[i])`.
2. for each consecutive pair `(k, k+1)`, for each edge `i`, build the bilinear ruled band
   `detail::ruledSideFace(ring[k][i], ring[k][i+1], ring[k+1][i], ring[k+1][i+1], o)` with
   `o` chosen so the band normal points outward — REUSING the Tier-B band builder unchanged;
   each interior section `k` (0 < k < N−1) shares its ring between the band below `(k−1,k)`
   and the band above `(k,k+1)` → a C0 skin (C1 where the incoming/outgoing directions at
   the section agree).
3. cap ONLY `S₀` and `S_{N-1}` with `detail::planarFace` (each end section must be planar).
4. `makeShell(faces)` → `makeSolid({shell})`, oriented outward → watertight (the shared
   rings + two-stage weld close every band-to-band seam, as the Tier-B two-section loft
   already welds its single band-to-cap seam).

This is exactly `build_ruled_loft` generalized from 1 band-pair to N−1, so the assembler,
its cognitive complexity band, and the weld path are unchanged in spirit.

**Volume check (host).** A stack of N aligned convex polygon sections at increasing heights
has volume = Σ over each slab of the prismatoid/frustum volume between consecutive sections
(for a linear ruled skin, the exact between-two-parallel-sections volume). A square →
square → smaller-square stack, or a square→hex→square, gives a closed-form sum the native
mesh converges to.

**Deferred (NULL → OCCT):** MISMATCHED vertex counts across any pair (ambiguous
pairing/resample — `ThruSections` re-parametrizes; Tier-C-style resample not attempted), or
a NON-planar end cap (the ruled skin builds but a planar end cap is required for a closed
solid).

---

## 3. (C) General sweep — RMF + twist/scale, and guided/rail

### 3.1 Rotation-minimizing frame (RMF)

**Why.** Tier C used `constantFrames` — a single fixed orientation across all stations —
which matches OCCT's planar corrected-Frenet law for a PLANAR spine but is wrong for a
NON-PLANAR spine (the section must turn with the spine while NOT accumulating spurious
twist). OCCT uses `GeomFill_CorrectedFrenet`; the clean-room equivalent that avoids the
Frenet curvature/torsion singularities is a **rotation-minimizing frame**.

**Double-reflection method** (Wang, Jüttler, Zheng, Liu 2008 — "Computation of Rotation
Minimizing Frames", ACM TOG). `detail::rmfFrames(spine, tangents)`:
```
  U₀ = a reference up-vector ⟂ t₀ (as today: cross(t₀,+Y), or +X if t₀∥Y)
  for each station i → i+1:
    v₁ = x_{i+1} − x_i;         c₁ = v₁·v₁
    rL = U_i − (2/c₁)(v₁·U_i) v₁          // reflect U across the segment
    tL = t_i − (2/c₁)(v₁·t_i) v₁          // reflect t across the segment
    v₂ = t_{i+1} − tL;          c₂ = v₂·v₂
    U_{i+1} = rL − (2/c₂)(v₂·rL) v₂       // second reflection → minimal-rotation up
    frame_{i+1} = { origin x_{i+1}, tangent t_{i+1}, up U_{i+1}, right t×U }
```
This transports the section along a NON-PLANAR spine with provably minimal rotation about
the tangent (no torsion blow-up). For a PLANAR spine the RMF reduces to a constant frame,
so **Tier-C parity is preserved as a special case** (the smooth-planar-arc sweep still
matches OCCT exactly).

### 3.2 Accumulating twist + linear scale (`cc_twisted_sweep`)

On top of the RMF, at station `i` (parameter `s_i ∈ [0,1]` by arc length):
```
  twist_i = s_i · twistRadians          // rotate the section about the frame tangent
  scale_i = 1 + s_i · (scaleEnd − 1)    // scale the section in the frame plane
  placed section vertex = origin_i + scale_i·( (cosθ·u + sinθ·rhs) of the RMF-rotated local pt )
```
so `cc_twisted_sweep(profileXY, pathXYZ, twistRadians, scaleEnd)` is now native for a real
twist/scale (Tier C only handled twist≈0, scale≈1). The station rings feed the same
`ruledSideFace` bands + `planarFace` end caps as the plain sweep.

### 3.3 Guided sweep / rail loft (best-effort)

- **`cc_guided_sweep(profileXY, pathXYZ, guideXYZ)`** — the guide curve overrides the RMF
  up-vector: at each station the frame's up-vector points from the spine point toward the
  corresponding guide point (resampled to the station count), so the section tracks the
  guide's orientation. Native where the guide-driven section produces a watertight,
  oracle-correct solid.
- **`cc_loft_along_rail(railXYZ, profileA, profileB)`** — the rail is the spine; the section
  at station `i` is the linear blend of profile A and profile B by `s_i` (equal vertex
  counts), transported with the RMF. This is a rail-guided ruled loft. Native where the
  blended section stays a simple closed curve and the solid self-verifies.

### 3.4 Guards (NULL → OCCT — NO SSI)

- **Tight curvature / self-intersecting spine.** Extend `spineTooSharp`: if the local
  curvature radius drops below the section's max radial extent × the largest applied
  `scale`, the swept surface folds — return NULL. A self-crossing spine likewise NULL.
- **Twist/scale envelope.** If the accumulated twist × the section radius exceeds the
  station spacing (adjacent rings would interpenetrate), NULL.
- **Guide/rail unresolvable.** If the guide/rail case would need trimming two swept
  surfaces (SSI) to close — NULL → OCCT. This change does NOT attempt SSI.
- Degenerate input (< 3 profile pts / < 2 path pts / empty guide/rail) → NULL.

The `NativeEngine` self-verify is the backstop: any candidate that is not watertight with a
sane volume is DISCARDED → OCCT regardless of the guards.

---

## 4. (D) Thread self-intersection resolver

**Context.** Tier D landed the radial-V helical / tapered thread native for CLEARING
parameters (the flanks have room between turns) and GUARDED fine-pitch / large-depth
threads to OCCT with a conservative `2·depthMM·tan(flankAngleDeg) ≥ pitchMM·kPitchSafety`
test. Some of those guarded threads are only NEAR self-intersecting — the flanks approach
but do not truly cross — and can be welded watertight with a trimmed V.

**Resolver** (`src/native/construct/thread.h`). For a thread in the near-self-intersecting
band (flanks within `kResolveband` of touching but not crossing):
1. **Clamp the V to the pitch room.** Trim each flank's axial half-width to at most
   `pitch/2 − ε` so the base of turn `n`'s V does not overlap the base of turn `n+1`; the
   apex depth is preserved (the thread crest is unchanged), only the flank foot is clamped
   at the root — the standard truncated-thread-root geometry. The trimmed V is still a
   simple triangle/trapezoid section, tiled by the same ruled bands + caps.
2. **Re-run the watertight self-verify.** The clamped-V thread now welds `boundaryEdges ==
   0` across the deflection ladder for a WIDER set of parameters → native.

**Still deferred (NULL → OCCT — genuinely SSI):** a thread whose flanks TRULY cross (the
radial V is deeper than the whole pitch room even after root clamping, i.e. the crest of
one turn would pass the crest of the next) is genuinely self-intersecting — a non-manifold
swept surface no matter how the vertices weld — so it still fails `robustlyWatertight` and
returns NULL. Resolving that needs SSI (trimming the overlapping flanks against each other),
which is Tier 4 and NOT attempted here. The tapered tip-crossing and degenerate guards are
unchanged.

**Host check.** A thread whose parameters previously tripped the conservative guard but is
only near-self-intersecting is now watertight (`boundaryEdges == 0` across the ladder) with
the clamped-V volume; a genuinely self-intersecting thread still returns NULL.

---

## 5. NativeEngine wiring + self-verify (all areas)

`src/engine/native/native_engine.cpp` — the affected ops
(`solid_extrude_profile*`, `solid_revolve_profile`, `solid_loft`, `solid_loft_wires`,
`solid_sweep`, `twisted_sweep`, `guided_sweep`, `loft_along_rail`, `helical_thread`,
`tapered_thread`) keep native-else-fallback. Each native candidate passes through the
MANDATORY self-verify already used by the boolean/blend/thread paths:
`robustlyWatertight(mesh)` at the deflection ladder AND a volume/geometry sanity check
(positive, matching the analytic expectation where one exists, sane sign). A NULL native
result OR a failed self-verify falls through to the fallback with NO interception. OCCT is
referenced only under `CYBERCAD_HAS_OCCT`; the native builder references no OCCT / `IEngine`
/ `EngineShape` type; `native_engine.h` is unchanged (signatures already present).

## 6. Cognitive complexity

New/extended functions target the systems band (≤ 25–35, flagged for the irreducible
station/segment loops): `rmfFrames` (linear double-reflection loop, ≤ 12), the N-section
chain loop (≤ 15), the torus band builder (≤ 12), the spline-edge extrude wall (≤ 12), the
V-clamp resolver (≤ 10), the `Torus` surface math (≤ 5). Measured with the
`cognitive-complexity` skill; any assembler that exceeds the band is flagged and its loop
isolated (as the Tier-B/C/D assemblers already are).

## 7. Honest outcome (reported per area)

The change reports the real native-vs-fallback split per area after both gates:
- **(A)** spline-edge extrude + off-axis-arc torus revolve — native where watertight +
  oracle-correct; spline-revolve + non-planar/self-intersecting cases OCCT.
- **(B)** N-section loft — native for equal-count planar chains; mismatched/non-planar OCCT.
- **(C)** RMF + twist/scale sweep — native for non-self-intersecting spines; guided/rail
  best-effort; all SSI (self-intersecting / tight-curvature / hard rail) OCCT.
- **(D)** thread resolver — native for near-self-intersecting (clamped-V) threads; truly
  self-intersecting threads OCCT.

Nothing that needs SSI is shipped; nothing that fails a gate is faked.
