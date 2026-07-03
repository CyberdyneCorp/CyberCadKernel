# add-native-threads

Phase 4 capability **#4b Tier D — helical threads + tapered shank**. The next honest
increment of native construction after Tier A (holed / typed-profile extrudes +
typed-profile revolve), Tier B (2-section ruled loft), and Tier C (constant-frame
sweep). This change moves **`cc_tapered_shank`** native (a revolve of a pointed-shank
silhouette) and **attempts** **`cc_helical_thread` / `cc_tapered_thread`** native (a
radial V/triangular section swept along a helical spine), keeping every case that
cannot be made watertight + oracle-correct as honest, labelled OCCT fall-through.

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT),
and does NOT fake any sub-case — anything not genuinely native + verified falls
through to OCCT in `NativeEngine` and is documented as such.

## Honesty statement (threads are hard — read this first)

**`cc_tapered_shank` is a straightforward native revolve and is expected to land
native, exact/deflection-bounded vs OCCT.** The two thread ops are genuinely hard: a
V-section swept along a helix at fine pitch / large depth self-intersects on the
inner flank, and keeping the section RADIAL (not Frenet-rotated) requires an axis
auxiliary-spine law that the native ruled-band assembler must reproduce exactly. This
change **attempts** the threads natively behind a self-intersection guard, but **if
the radial-V helical sweep cannot be made watertight + correct for the test cases,
the thread ops REMAIN OCCT-fall-through** (labelled, verified rel~0, never faked) and
only `cc_tapered_shank` lands native. The final status will be reported honestly per
the verification gates — a thread op is "native" only when BOTH gates below are green
for it, otherwise it is documented as deferred.

## What the ops are (the `cc_*` contract)

- **`cc_tapered_shank(radiusMM, fullHeightMM, taperHeightMM, pointsPerMM)`** — a
  solid of revolution: a silhouette that holds `radiusMM` for `fullHeightMM` then
  tapers linearly to a **point** on the axis over `taperHeightMM`, revolved 360°
  about Z. This is a **revolve of a pointed-shank silhouette** — the `cc_*` mirror of
  OCCT `BRepPrimAPI_MakeRevol` on the shank silhouette wire.

- **`cc_helical_thread(majorRadiusMM, pitchMM, turns, depthMM, flankAngleDeg,
  pointsPerMM, samplesPerTurn)`** — a **V / triangular thread section** (apex outward,
  base along the axis, flank half-angle `flankAngleDeg`, radial depth `depthMM`) swept
  along a **helix** of constant radius (the pitch-line radius derived from
  `majorRadiusMM`), `turns` revolutions at axial `pitchMM` per turn. The crux: the
  section is kept **RADIAL** — its apex points straight out from the axis and its base
  runs along the axis — by an **axis auxiliary-spine law**, so the V does **NOT**
  Frenet-rotate as it winds up the helix. `samplesPerTurn` is the angular
  discretization (capped).

- **`cc_tapered_thread(topRadiusMM, tipRadiusMM, pitchMM, turns, depthMM,
  flankAngleDeg, pointsPerMM, samplesPerTurn)`** — as `cc_helical_thread` but the
  helix radius **tapers** linearly from `topRadiusMM` to `tipRadiusMM` over the
  `turns` (a conical thread), the V-section still radial at every station.

This is the `cc_*` mirror of OCCT `BRepOffsetAPI_MakePipeShell` **with an auxiliary
spine** (the axis) so the profile's normal is defined by the axis rather than the
Frenet frame of the helix — see `SetMode(AuxiliarySpine, …)` — plus
`BRepPrimAPI_MakeRevol` for the shank.

## Scope (Tier D)

| Op | Native in this change | Falls through to OCCT (honest, labelled) |
|---|---|---|
| **`cc_tapered_shank`** | **YES** — a native revolve of the pointed-shank silhouette (`radiusMM` for `fullHeightMM`, then a cone to a point over `taperHeightMM`), 360° about Z; exact/deflection-bounded vs OCCT `MakeRevol` | — |
| **`cc_helical_thread`** | **ATTEMPTED** — a radial-V section swept along a constant-radius helix with an **axis auxiliary-spine law** (the section stays radial, does NOT Frenet-rotate), tiled into ruled bands with shared per-station rings + end caps → a watertight solid, IFF it passes the self-intersection guard | **YES** when the guard trips (fine pitch / large depth → the inner flank folds), OR if the radial-V sweep cannot be made watertight + oracle-correct for the test cases (then this op stays fall-through) |
| **`cc_tapered_thread`** | **ATTEMPTED** — as `cc_helical_thread` but the helix radius tapers `topRadiusMM → tipRadiusMM`; same axis auxiliary-spine law + same guard | **YES** under the same conditions as `cc_helical_thread` |

### Why the hard cases fall through (not faked)

- **Fine-pitch / large-depth threads self-intersect.** When `depthMM` approaches or
  exceeds half the pitch (the radial V is deeper than the axial room between adjacent
  turns), the inner flank of one turn crosses the outer flank of the next — the swept
  surface self-intersects. Producing a valid trimmed B-rep there is the pipe-shell
  robustness problem OCCT resolves; Tier D **guards** it (`depth` vs `pitch` and the
  V-flank crossing test) and defers rather than emit a self-intersecting solid.
- **The radial-V helical sweep may not be watertight/correct at all** for the test
  cases. If, on the sim parity gate, the native thread cannot match the OCCT oracle
  within a deflection bound (volume / bbox / watertightness), the thread op is kept as
  **labelled OCCT fall-through** and this is reported honestly — Tier D does not ship a
  thread solid that disagrees with the oracle.
- **The round-profile fallback** noted in the `cc_*` doc-comment (a thread that falls
  back to a round profile when the radial V cannot form a valid solid) is a
  contract-level behaviour of the OCCT implementation; when the native V cannot form a
  valid solid the native builder returns NULL and the engine falls through to OCCT
  (which applies that round-profile fallback), rather than the native path faking it.

## Method (locked, per NATIVE-REWRITE.md)

Clean-room from the `cc_*` thread/shank contract (the doc-comments in
`include/cybercadkernel/cc_kernel.h`) and computational-geometry first principles
(helix parametrization, an **axis auxiliary-spine** section-transport law that fixes
the section normal to the radial direction, ruled/swept surface tiling with shared
rings, surface-of-revolution silhouette), with OCCT source
(`/Users/leonardoaraujo/work/OCCT/src`: `BRepOffsetAPI_MakePipeShell` — specifically
its `SetMode(AuxiliarySpine, CurvilinearEquivalence, KeepContact)` auxiliary-spine
mode that defines the section normal from a second spine so the section does not
Frenet-rotate; `BRepPrimAPI_MakeRevol` for the shank) consulted as a **reference
oracle only** — never copied. The shank reuses the native `build_revolution`; the
thread reuses Tier-C's constant/aux-frame station-ring transport + Tier-B's
`ruledSideFace` bilinear bands + `planarFace` caps.

## Architecture / OCCT boundary (unchanged from #4 / #4b Tier A–C)

- New native builders live under `src/native/construct/thread.h` (threads) and reuse
  `src/native/construct/construct.h`'s `build_revolution` for the shank; all stay
  **OCCT-FREE and host-buildable** (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`,
  no OCCT, no simulator); they include only `src/native/math` + `src/native/topology`
  + `src/native/tessellate` + the existing `src/native/construct/` assemblers and
  return a `topology::Shape`.
- `src/engine/native/native_engine.{h,cpp}` — `tapered_shank`, `helical_thread`, and
  `tapered_thread` (currently pure `fallback()` delegations) become native-else-
  fallback; a NULL native result (a degenerate input, a self-intersection guard trip,
  or a case the builder declines) falls through to the fallback with no interception.
  OCCT stays behind `CYBERCAD_HAS_OCCT`; the native builder never sees OCCT.
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via
  `cc_set_engine(1)`), so every existing suite is unchanged unless it opts in.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT): built native B-rep +
   its native tessellation.
   - **Tapered shank** — a shank of radius `r`, full height `h`, taper `t` is
     watertight (`boundaryEdgeCount == 0`) with volume = cylinder `πr²h` + cone
     `πr²t/3` within a deflection bound, and the analytic bbox / on-axis point.
   - **Helical / tapered thread (if native)** — a coarse-pitch, shallow-depth thread
     (guard passes) is watertight, its section stays radial at every station (the
     apex direction is the station's radial unit vector, NOT the helix tangent's
     normal — the axis-aux-spine property), and its volume is within a deflection band
     of the OCCT oracle.
   - **Guards / fall-through** — a fine-pitch / large-depth thread (depth ≥ safe
     fraction of pitch, or the flank-crossing test trips) returns a NULL `Shape`; a
     degenerate shank/thread input returns NULL.
2. **Simulator native-vs-OCCT parity through the facade**: the SAME `cc_tapered_shank`
   (and, where native, `cc_helical_thread` / `cc_tapered_thread`) calls issued native
   (`cc_set_engine(1)`) vs OCCT (default), compared on mass properties / bbox /
   sub-shape counts / watertight tessellation within a documented fp64
   (shank: deflection-bounded) tolerance vs `BRepPrimAPI_MakeRevol` /
   `BRepOffsetAPI_MakePipeShell`. The fall-through cases (fine-pitch / large-depth
   threads, and any thread op that could not be made native) asserted **identical**
   under both engines (fall-through proof). Default restored in teardown; the parity
   harness carries its own `main()` (on the `run-sim-suite.sh` SKIP list) so the
   221-assertion suite count is unchanged.

A requirement is done only when BOTH gates are green for that op AND every existing
suite (`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) stays green at
the OCCT default. **Honest outcome:** `cc_tapered_shank` is expected native; the two
thread ops land native only if both gates pass, otherwise they remain labelled OCCT
fall-through and the change reports that.
