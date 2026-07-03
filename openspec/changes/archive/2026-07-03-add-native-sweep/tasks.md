# Tasks — add-native-sweep (Phase 4 #4b Tier C, sweep)

Order: math helper → RMF transport → straight sweep (exact) → curved sweep (RMF) →
guards → twisted sweep → engine wiring → Gate 1 (host) → Gate 2 (sim parity) → docs.
Native code stays OCCT-free + host-buildable (`/opt/homebrew/opt/llvm/bin/clang++
-std=c++20`). No `cc_*` ABI change. Default engine stays OCCT.

> ## IMPLEMENTATION NOTE (scope — BOTH gates green; constant-frame law, not RMF)
> The SHIPPED native scope is BOTH tractable Tier-C cases: (a) a STRAIGHT-path sweep →
> an EXACT directional prism (always watertight); and (b) a SMOOTH CURVED but PLANAR
> spine → a CONSTANT-frame ruled-band tube, capped at both ends, welded by the two-stage
> tessellator. FRAME-LAW PIVOT (the fix that made Gate 2 pass): the OCCT oracle
> `BRepOffsetAPI_MakePipe` uses `GeomFill_CorrectedFrenet`, which for a PLANAR spine
> collapses to a CONSTANT rotation (`GeomFill_CorrectedFrenet.cxx`, `isPlanar` →
> `Law_Constant`) — it TRANSLATES the section with a FIXED orientation, it does NOT keep
> the section perpendicular to the tangent. So `detail::constantFrames` freezes the start
> trihedron's x/y axes across every station (only the origin advances), and the swept
> volume is `profileArea × |Δspine · n̂|` (spine displacement projected onto the fixed
> section normal), NOT the Pappus arc-length volume. An earlier RMF / double-reflection
> revision (tasks §2.3/§9.3, as originally drafted) kept the section perpendicular and
> produced the Pappus volume — geometrically "nicer" but a REAL mismatch vs the oracle,
> correctly rejected by the parity gate; that RMF path was REMOVED in favour of the
> constant frame that matches the oracle. No `doubleReflectionRMF` / `SweptSurface` /
> `build_prism_dir` helper shipped: the constant frame reuses `loft.h`'s
> `ruledSideFace` (bilinear Bézier) + `construct.h`'s `planarFace`. A NON-PLANAR curved
> spine (OCCT's genuine non-constant law), a TIGHT-CURVATURE / self-intersecting spine
> (guarded by `spineTooSharp`), and any real TWIST/scale DEFER to OCCT (verified
> fall-through, never faked). BOTH gates green: host Gate 1 (11 `test_native_sweep` +
> 3 `test_native_engine` sweep cases, CTest 15/15) and Gate 2 sim OCCT parity
> (`native_sweep_parity.mm`, `[NSWEEP]` 8 native + 3 fallback = 11 passed / 0 failed —
> straight EXACT rel 7.11e-16, smooth-arc EXACT rel 1.72e-16 vs OCCT MakePipe).

## 1. Native swept-surface math (`src/native/math`, OCCT-free)

- [~] 1.1 NOT NEEDED as a new helper — the swept patch between adjacent stations is
  exactly a BILINEAR ruled face, so `sweep.h` reuses `loft.h`'s `detail::ruledSideFace`
  (a 2×2-pole degree-1 `Bezier` `FaceSurface`, row-major U outer, consistent with
  `bezierSurfacePoint`). No separate `SweptSurface` type / poles helper shipped.
- [~] 1.2 SUBSUMED by §9.2's curved-arc watertight test + the frame-invariance test
  (§9.3) — the bilinear ruled patch is already unit-tested via `loft.h` (Tier B) and
  exercised end-to-end on the quarter-arc sweep; no separate helper test was added.

## 2. Section-frame transport (`src/native/construct/sweep.h`) — CONSTANT frame (not RMF)

> PIVOT: the sweep frame law is CONSTANT, not RMF. OCCT MakePipe's default
> `GeomFill_CorrectedFrenet` collapses to a constant rotation on a PLANAR spine, so an
> RMF (rotation-minimizing) frame would NOT match the oracle. The shipped
> `detail::constantFrames` freezes the start trihedron across all stations.

- [x] 2.1 Start frame at the path start: `T0 = tangent(0)`, `x = normalize(cross(t0, ref))`,
  `y = normalize(cross(x, t0))` — the oracle's start trihedron (`nrm`, `up`). Centroid on
  `path[0]`. (`detail::startRef` / `detail::constantFrames`.)
- [x] 2.2 Spine discretization: the polyline stations are kept (a straight spine collapses
  to its two endpoints; a smooth spine keeps every station), each with a unit tangent `T_k`
  used only for outward face orientation.
- [~] 2.3 NOT SHIPPED — the `doubleReflectionRMF` (Wang et al. 2008) step was superseded
  by `detail::constantFrames`: on a planar spine the oracle's law is a CONSTANT rotation,
  so there is no per-station reflection. `constantFrames` is ~4 cognitive complexity.
- [x] 2.4 Emit the transported section `S_{k,i} = origin_k + x·px_i + y·py_i` per station
  (`detail::placeProfile`), with `(x, y)` the FIXED start axes at every station.

## 3. Straight-path sweep = directional extrude (EXACT) (`src/native/construct/`)

- [x] 3.1 Straight detector: `detail::spineIsStraight` — every point collinear with the
  first segment direction within `kProfileTol` ⇒ straight (a straight spine collapses to
  its two endpoints).
- [~] 3.2 NO separate `build_prism_dir` factored — the straight case is not special-cased
  into a distinct assembler; it is the constant-frame path with a single (2-station) band,
  which naturally yields N `Plane`-equivalent bilinear side faces + 2 planar caps. One
  assembler (`build_sweep`) handles straight and smooth-curved uniformly.
- [x] 3.3 `build_sweep` straight branch collapses to endpoints → single ruled band + caps;
  volume EXACT `profileArea · |d|` (host tests: `160`, `320`, `16·L`, pentagon `area·12`).

## 4. Smooth curved-path sweep (RMF swept surface) (`src/native/construct/sweep.h`)

- [x] 4.1 One swept patch per profile edge per spine segment: reuse `loft.h`'s
  `detail::ruledSideFace` (a bilinear `Bezier` `FaceSurface`, poles row-major U outer)
  on the two adjacent transported section rings, with SHARED boundary edges/vertices
  (welds watertight, as `loft.h`). No separate `SweptSurface` type was needed — the
  bilinear ruled face between adjacent stations is exactly the swept patch.
- [x] 4.2 End caps: `detail::planarFace` on the start ring (outward normal −t0) and the
  end ring (+t), sharing the ring vertices with the side patches.
- [x] 4.3 Assemble side patches (all stations × all edges) + 2 caps → `Shell` → `Solid`,
  oriented outward (`build_sweep`).

## 5. Guards (honest fall-through — return NULL `Shape`)

- [x] 5.1 Tight-curvature guard (`detail::spineTooSharp`): at any interior vertex the
  discrete turning radius `min(segLen)/turnAngle <` profile circumradius `R` (×1.15
  margin) ⇒ NULL.
- [x] 5.2 Self-intersection / sharp-corner guard (`detail::spineTooSharp`): a per-vertex
  turn angle > `kMaxTurn` (≈34°) ⇒ NULL (a MakePipe mitre the ruled bands do not model);
  combined with 5.1 this rejects the concave-side tube self-fold.
- [x] 5.3 Degenerate guard: `profileCount < 3`, `pathCount < 2`, zero profile area,
  zero-length path, or a collapsed tangent ⇒ NULL.

## 6. Twisted sweep (`src/native/construct/sweep.h`)

- [x] 6.1 `build_twisted_sweep(profile, path, pathCount, twistRadians, scaleEnd)` — native
  ONLY when it reduces to the plain sweep (twist ≈ 0 AND scale ≈ 1): then it forwards to
  `build_sweep`. A REAL twist/scale is NOT modelled by the constant-frame ruled band
  (a genuinely twisted skin needs the perpendicular per-station rotation OCCT
  `ThruSections` builds), so it returns NULL. No faked twist path.
- [x] 6.2 Real twist/scale or an inapplicable base sweep (guard trip) ⇒ NULL (fall
  through to OCCT `twisted_sweep`). Host-verified: `twisted_sweep_real_twist_deferred`
  (π/2 twist → NULL, 2× scale → NULL).

## 7. Native builder API surface (`src/native/construct/native_construct.h`)

- [x] 7.1 Expose `build_sweep(profileXY, profileCount, pathXYZ, pathCount)` and
  `build_twisted_sweep(...)` returning `topology::Shape` (NULL ⇒ fall through).
- [x] 7.2 Add `#include "native/construct/sweep.h"` to `native_construct.h` and update
  its header doc-comment (SUPPORTED vs DEFERRED) to move sweep from DEFERRED to
  SUPPORTED (tractable cases) and keep guided/rail/tight-curvature DEFERRED.
- [x] 7.3 Verify all new functions stay within cognitive-complexity targets
  (`build_sweep` ≤ 15; RMF step ≤ ~5; flag any irreducible station-loop, systems band
  ≤ 25) with the `cognitive-complexity` skill.

## 8. NativeEngine wiring (`src/engine/native/native_engine.cpp`)

- [x] 8.1 `solid_sweep` → `ncst::build_sweep`; NULL ⇒ `fallback().solid_sweep(...)`.
- [x] 8.2 `twisted_sweep` → `ncst::build_twisted_sweep`; NULL ⇒
  `fallback().twisted_sweep(...)`.
- [x] 8.3 `guided_sweep` and `loft_along_rail` stay **pure fall-through** with a
  labelled comment (Tier C pipe-shell/guide, not faked).
- [x] 8.4 Confirm OCCT is referenced only under `CYBERCAD_HAS_OCCT`; the native builder
  references no OCCT / `IEngine` / `EngineShape` type; `native_engine.h` unchanged.

## 9. Gate 1 — host analytic unit tests (`tests/`, no OCCT)

- [x] 9.1 `tests/test_native_sweep.cpp`: straight sweep of a square along a straight
  path length `L` → watertight (`boundaryEdgeCount == 0`), EXACT volume `A·L`, face
  count N+2.
- [x] 9.2 Curved sweep of a square along a gentle quarter-arc spine
  (`sweep_curved_arc_native_watertight` + `sweep_curved_arc_frame_volume_matches_oracle`)
  → watertight at the parity deflection band, volume = CONSTANT-frame swept volume
  `profileArea × |Δspine · n̂|` (NOT the Pappus arc-length volume — the section does not
  track the tangent). Plus a facade case `native_sweep_smooth_arc` in `test_native_engine`
  (native id ≠ 0, curved builder served by NativeEngine).
- [x] 9.3 CONSTANT-frame invariance (`sweep_curved_arc_frame_is_constant`): every station
  shares the START frame's x/y axes (`x·xk ≈ 1`, `y·yk ≈ 1` — section does NOT rotate to
  follow the tangent), the frame stays orthonormal, and each station still records its own
  unit tangent for face orientation. This is the crux of the OCCT planar-MakePipe oracle
  match (constant rotation), not an RMF/Frenet signature.
- [x] 9.4 Twisted sweep with ZERO twist + unit scale on a straight path
  (`twisted_sweep_zero_twist_is_prism`) → watertight prism vol 160 (forwards to
  `build_sweep`).
- [x] 9.5 Guards: tight-curvature, self-intersecting spine, degenerate profile/path →
  NULL `Shape`.
- [x] 9.6 `tests/test_native_engine.cpp`: facade cases for native `cc_solid_sweep`
  (straight) and `cc_twisted_sweep` under `cc_set_engine(1)`; and a fall-through case
  (`cc_guided_sweep` / tight curvature) proving delegation.
- [x] 9.7 Host CTest all green (existing + new); `test_native_tessellate`,
  `test_native_construct`, `test_native_loft` unchanged.

## 10. Gate 2 — simulator native-vs-OCCT parity (`tests/sim/`)

- [x] 10.1 GREEN — `tests/sim/native_sweep_parity.mm` + `scripts/run-sim-native-sweep.sh`:
  through the `cc_*` facade under `cc_set_engine(0/1)` (OCCT default restored in a
  teardown guard), compared `cc_solid_sweep` native vs `BRepOffsetAPI_MakePipe`. Because
  native and OCCT now share the SAME constant-frame law + polyline, BOTH cases are EXACT:
  STRAIGHT (vol o=160 n=160 rel 7.11e-16, area rel 1.48e-16, centroidΔ 1.33e-15, bbox
  maxCornerΔ 1.0e-7, identical 6-face structure, watertight 12 tris) and SMOOTH-ARC
  (vol o=330.299 n=330.299 rel 1.72e-16, area rel 1.27e-15, centroidΔ 7.11e-15, bbox
  maxCornerΔ 1.0e-7, native F = OCCT F = 98, watertight 196 tris). 8 native parity checks.
- [x] 10.2 `cc_twisted_sweep` covered as a DEFERRED fall-through case (real π/2 twist +
  0.5 scale → native NULL → OCCT `twisted_sweep`, delegated result asserted equal).
- [x] 10.3 Fall-through proof cases: `cc_twisted_sweep` (real twist), `cc_guided_sweep`,
  and `cc_loft_along_rail` each assert `cc_active_engine()==1` AND native == OCCT oracle
  (delegated, no interception). Tight-curvature fall-through is host-covered (9.5).
- [x] 10.4 Parity harness carries its own `main()` + `std::_Exit`; ADDED to
  `run-sim-suite.sh` SKIP list (`native_sweep_parity.mm` — a `.mm` already excluded by the
  `*.cpp` find, so the assertion count is unaffected). `run-sim-suite.sh` re-verified
  against a freshly rebuilt SIMULATORARM64 slice: **== 221 passed, 0 failed ==**.

## 11. Docs / spec sync

- [x] 11.1 Updated `openspec/NATIVE-REWRITE.md` #4b Tier C bullet: `cc_solid_sweep`
  (straight EXACT + smooth-planar CONSTANT-frame, EXACT vs the oracle) done at the
  verification bar; guided/rail/real-twist/tight-curve/non-planar fall-through labelled;
  both gates' numbers cited.
- [x] 11.2 Updated `docs/STATUS-phase-4.md` with the Tier C sweep result table + deltas
  and the `#4b` per-capability row.
- [x] 11.3 `openspec validate add-native-sweep --strict` green; `openspec archive
  add-native-sweep -y` run (syncs the delta into `openspec/specs/native-construction`);
  both gates green + `run-sim-suite.sh` 221/221 at the OCCT default.
