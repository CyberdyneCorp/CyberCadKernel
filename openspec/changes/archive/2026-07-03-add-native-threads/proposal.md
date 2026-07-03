# Proposal — add-native-threads

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capability #4 (`native-construction`) landed native
polygon extrude + line-segment revolve; `#4b` Tier A added holed / typed-profile
extrudes and typed-profile revolve; Tier B added the 2-section ruled loft; Tier C
added the constant-frame sweep (`cc_solid_sweep` / no-op `cc_twisted_sweep`). The
living `native-construction` spec is explicit that **threads and the tapered shank**
(`cc_helical_thread`, `cc_tapered_thread`, `cc_tapered_shank`) still fall through to
OCCT (`#4b` Tier D). Threads + shanks back real product features (screws, bolts,
wood-screw tips, machine threads), so they are the next honest increment.

Doing all of Tier D as one confident "native threads" claim would risk faking the
genuinely hard radial-V helical sweep. This proposal is scoped and honest:
**`cc_tapered_shank` is a native revolve** (a solved problem — reuse `build_revolution`)
and is expected to land native; **`cc_helical_thread` / `cc_tapered_thread` are
ATTEMPTED** with a radial-V section swept along a helical spine under an axis
auxiliary-spine law and a self-intersection guard, and **remain labelled OCCT
fall-through if that sweep cannot be made watertight + oracle-correct** for the test
cases. The change reports the real native-vs-fallback outcome per op.

## What changes

1. **Native tapered-shank revolve** (`src/native/construct/`, OCCT-free). Build the
   pointed-shank silhouette in the `(r, h)` half-plane — hold `radiusMM` for
   `fullHeightMM`, then taper linearly to a point `(0, fullHeightMM + taperHeightMM)`
   over `taperHeightMM` — as `LineSeg`s, and revolve 360° about Z with the existing
   `build_revolution` (per-segment surface classification: the parallel side →
   `Cylinder`, the tapered side → `Cone`, the bottom → planar disk; the apex is
   on-axis so it contributes no swept face). A small `build_tapered_shank(radiusMM,
   fullHeightMM, taperHeightMM, pointsPerMM)` wrapper assembles the silhouette and
   calls `build_revolution`.

2. **Native helical / radial-V thread** (`src/native/construct/thread.h`, OCCT-free)
   — ATTEMPTED.
   - **Helical spine.** Sample the helix `C(θ) = (R(θ)·cosθ, R(θ)·sinθ, pitch·θ/2π)`
     for θ ∈ [0, 2π·turns] at `samplesPerTurn` stations per turn (CAPPED to a safe
     maximum so a large `turns × samplesPerTurn` cannot blow up the station count),
     with `R(θ)` constant (`cc_helical_thread`) or a linear taper `topRadius →
     tipRadius` (`cc_tapered_thread`) at the pitch-line radius.
   - **Axis auxiliary-spine section law (the crux).** Transport the V/triangular
     section so it stays **RADIAL**: at each station the section's outward axis is the
     **radial unit vector** `r̂(θ) = (cosθ, sinθ, 0)` (apex pointing straight out from
     the Z axis) and the section's along-axis direction is `+Z` (the base runs along
     the axis) — the frame is defined by the **axis auxiliary spine** (the Z axis),
     NOT the Frenet frame of the helix, so the V does not rotate/tilt with the helix
     tangent. This mirrors OCCT `BRepOffsetAPI_MakePipeShell::SetMode(AuxiliarySpine,
     …)`, where a second spine defines the section normal.
   - **V-section.** The triangular section at each station is `{ base_lo, apex, base_hi }`
     in the local `(radial, axial)` plane: apex at radial `R(θ) + depthMM` (or the
     contract's outward apex), base at radial `R(θ)` offset ±`depthMM·tan(flankAngle)`
     along `+Z`, placed with `worldPoint = C(θ) + radial·r̂(θ) + axial·ẑ`.
   - **Tile + cap.** One bilinear ruled band per (section edge × spine segment) with
     SHARED per-station vertex rings (reuse Tier-B `detail::ruledSideFace`), the two
     helix ends capped with the planar V-section face (reuse `detail::planarFace`) →
     a watertight `Solid`.
   - **Self-intersection guard.** If `depthMM` is not safely below the pitch room
     (the inner flank of one turn would cross the outer flank of the next), or the
     transported V-section overlaps its neighbour, return a **NULL `Shape`** (fall
     through). No round-profile faking in the native path — a NULL defers to OCCT,
     which applies its documented round-profile fallback.

3. **`NativeEngine` glue** (`src/engine/native/native_engine.{h,cpp}`). `tapered_shank`,
   `helical_thread`, and `tapered_thread` — currently pure `fallback()` delegations —
   call the native builder and, on a NULL native result (a declined/guarded case),
   fall through to the fallback with no interception. OCCT stays behind
   `CYBERCAD_HAS_OCCT`; the native builder never sees OCCT. If a thread op cannot pass
   both gates, its glue is left as pure fall-through (labelled) and only
   `tapered_shank` is wired native.

## Non-goals (DEFERRED — fall through to OCCT, not implemented, not faked)

- **Fine-pitch / large-depth threads** — the radial V is deeper than the axial room
  between turns; the swept surface self-intersects. The builder detects it (depth vs
  pitch + flank-crossing test) and returns NULL → OCCT fall-through.
- **The round-profile fallback** — when the radial V cannot form a valid solid the
  native builder returns NULL and OCCT applies its documented round-profile fallback;
  the native path does not fake it.
- **Either thread op that fails the parity gate** — if the radial-V helical sweep
  cannot be made watertight + oracle-correct for the test cases, that op stays labelled
  OCCT fall-through and the change reports it honestly (only `cc_tapered_shank` lands).
- All other `#4b` ops (`wrap_emboss`, `guided_sweep`, `loft_along_rail`, real-twist
  sweeps) and every feature / boolean / query / transform / exchange op — remain
  fall-through, unchanged.

## Impact

- `src/native/construct/` gains `thread.h` (helix spine + axis-aux-spine radial-V
  transport + tile/cap assembler + self-intersection guard) and a
  `build_tapered_shank` wrapper (reusing `build_revolution`), added to
  `native_construct.h` (still OCCT-free, host-buildable). New host CTest cases in
  `tests/test_native_thread.cpp` (+ facade cases in `tests/test_native_engine.cpp`).
- `src/engine/native/native_engine.cpp` — `tapered_shank` changes from pure
  fall-through to native-else-fallback; `helical_thread` / `tapered_thread` change to
  native-else-fallback IF they pass both gates, else stay labelled fall-through.
  `native_engine.h` unchanged (signatures already present).
- **No** `include/cybercadkernel/cc_kernel.h` signature change; **no**
  `src/facade/cc_kernel.cpp` change (the three `cc_*` entry points already route
  through the active engine). The three doc-comments are the contract this change
  implements natively for `cc_tapered_shank` (+ the thread ops where achievable).
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the new native path. All existing suites stay green at the
  OCCT default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** exact-value / analytic
unit tests on the built native B-rep + native tessellation — a tapered shank is
watertight with volume `πr²·(fullHeight) + πr²·(taperHeight)/3` within a deflection
bound and the analytic bbox; a coarse-pitch / shallow-depth thread (guard passes, if
native) is watertight with a radial section at every station and a deflection-bounded
volume; fine-pitch / large-depth / degenerate inputs return NULL — all with no OCCT;
(b) **sim parity** through the facade (`cc_set_engine(1)` vs default) comparing native
vs OCCT `BRepPrimAPI_MakeRevol` (shank) / `BRepOffsetAPI_MakePipeShell` (thread) and
asserting the fall-through cases identical under both engines. Done for `tapered_shank`
when both gates pass; the thread ops land native only if both gates pass for them,
otherwise they remain labelled OCCT fall-through and the change reports that outcome.
