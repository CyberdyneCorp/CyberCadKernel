# Design — add-native-threads (Phase 4 #4b Tier D)

## 0. Context and honest posture

Tier D covers three `cc_*` ops:

- `cc_tapered_shank` — a solid of revolution (a pointed shank). **Tractable; native.**
- `cc_helical_thread` / `cc_tapered_thread` — a V/triangular section swept along a
  helix, kept radial by an axis auxiliary-spine law. **Hard; attempted, guarded, and
  kept as OCCT fall-through if it cannot be made watertight + oracle-correct.**

The design below specifies both, and is explicit at each hard step about what defers.
Native code stays OCCT-free + host-buildable (`clang++ -std=c++20`); OCCT is a
**reference oracle only** (`/Users/leonardoaraujo/work/OCCT/src`:
`BRepPrimAPI_MakeRevol` for the shank, `BRepOffsetAPI_MakePipeShell` with an auxiliary
spine for the thread). No `cc_*` ABI change; the default engine stays OCCT.

## 1. Tapered shank — a revolve of a silhouette (native)

### 1.1 Contract

`cc_tapered_shank(radiusMM, fullHeightMM, taperHeightMM, pointsPerMM)`: a shank of
constant `radiusMM` over `fullHeightMM`, then a linear taper to a **point on the axis**
over `taperHeightMM`, revolved 360° about Z. `pointsPerMM` is a tessellation-density
hint (the native builder emits exact analytic surfaces; the deflection is applied at
tessellation time as elsewhere).

### 1.2 Silhouette in the `(r, h)` half-plane

Reuse the native revolve's `(r, h)` decomposition (axis = Z through the origin, radial
reference = +X). The silhouette loop (bottom-up) is:

```
  A = (0,            0)                              // bottom centre, on axis
  B = (radiusMM,     0)                              // bottom rim
  C = (radiusMM,     fullHeightMM)                   // top of full-radius section
  D = (0,            fullHeightMM + taperHeightMM)   // apex, on axis
  (close D → A along the axis)
```

As `LineSeg`s: `A→B` (bottom disk radius), `B→C` (full-radius cylinder wall), `C→D`
(the taper cone), `D→A` (the on-axis return — contributes NO swept face). Degenerate
guards: `radiusMM > 0`, `fullHeightMM ≥ 0`, `taperHeightMM ≥ 0`, and
`fullHeightMM + taperHeightMM > 0`; else NULL.

### 1.3 Revolve

Call the existing `build_revolution(segments, axis=Z-through-origin, angle=2π)`. Its
`segmentSurface` classifier already picks: `A→B` perpendicular → planar disk (the
bottom cap), `B→C` axis-parallel → `Cylinder`, `C→D` oblique → `Cone`, `D→A` on-axis →
skipped (both endpoints `onAxis`). A full turn closes the shell → a watertight `Solid`.
This is why the shank is a straightforward native op: it is a special case of the
already-native, already-parity-verified revolve.

### 1.4 Volume check (host)

Analytic volume = cylinder + cone = `π·r²·fullHeightMM + (1/3)·π·r²·taperHeightMM`.
The native tessellation's mesh volume converges to this within the deflection bound;
bbox = `[−r, r] × [−r, r] × [0, fullHeightMM + taperHeightMM]`.

### 1.5 Cognitive complexity

`build_tapered_shank` is a straight-line silhouette assembler (target ≤ 8). The heavy
lifting is in the already-shipped `build_revolution` (systems band, flagged).

## 2. Helical thread — radial-V section on an axis-aux-spine helix (attempted)

### 2.1 Contract

`cc_helical_thread(majorRadiusMM, pitchMM, turns, depthMM, flankAngleDeg, pointsPerMM,
samplesPerTurn)`: a **V/triangular** thread of radial `depthMM`, flank half-angle
`flankAngleDeg`, wound `turns` times at axial `pitchMM`/turn around the axis at the
pitch-line radius derived from `majorRadiusMM`. `cc_tapered_thread(topRadiusMM,
tipRadiusMM, …)` is the same with a linearly tapering helix radius. `samplesPerTurn`
is the angular discretization, **capped** (see §2.6).

### 2.2 Helix spine

For θ ∈ [0, 2π·turns], the pitch-line helix is

```
  R(θ) = constant (helical) | lerp(topRadius, tipRadius, θ / (2π·turns)) (tapered)
  C(θ) = ( R(θ)·cosθ,  R(θ)·sinθ,  pitchMM·θ / (2π) )
```

Sample at `nStations = turns · min(samplesPerTurn, kMaxSamplesPerTurn) + 1` spine
points (§2.6 caps the total). The **helix tangent** is NOT used to orient the section
(that is the whole point of the axis aux-spine law).

### 2.3 Axis auxiliary-spine section law (the crux — keep the V RADIAL)

OCCT's default sweep (`GeomFill_CorrectedFrenet`) would rotate the section with the
curve's Frenet frame, tilting the V as it climbs the helix — wrong for a thread. OCCT
instead offers `BRepOffsetAPI_MakePipeShell::SetMode(AuxiliarySpine,
CurvilinearEquivalence, KeepContact)`: a **second spine** defines the section normal,
so the section keeps a fixed relationship to that auxiliary curve rather than to the
main spine's Frenet frame (`/Users/leonardoaraujo/work/OCCT/src/.../
BRepOffsetAPI_MakePipeShell.hxx`, "auxiliary spine to define the Normal"). Here the
auxiliary spine is the **Z axis**: at each station the section frame is

```
  r̂(θ) = ( cosθ, sinθ, 0 )        // RADIAL out from Z (apex direction) — the aux-spine normal
  ẑ    = ( 0, 0, 1 )              // along the axis (the V base runs along Z)
  origin = C(θ)                   // the pitch-line helix point
```

The section axes `(r̂, ẑ)` are recomputed **per station from the angle θ only** — they
depend on the axis auxiliary spine, NOT on the helix tangent — so the V is **radial at
every station** and does not Frenet-rotate. (Contrast Tier C's `constantFrames`, which
froze ONE start frame; here the frame rotates with θ about Z but is pinned radial by
the axis, never tilted by the helix tangent.)

### 2.4 The V / triangular section

In the local `(radial u, axial v)` plane the V-section, apex outward, base along the
axis, is three points (a closed triangle):

```
  half = depthMM · tan(flankAngleDeg)      // axial half-width of the V at the base
  baseLo = ( 0,        −half )             // base, lower (toward −Z)
  apex   = ( depthMM,   0    )             // apex, pointing radially OUT
  baseHi = ( 0,        +half )             // base, upper (toward +Z)
```

Placed into world at station θ:

```
  P(u, v) = C(θ) + u · r̂(θ) + v · ẑ
```

so the base lies on the pitch-line cylinder (radial 0 = at `R(θ)`) and the apex is
`depthMM` further out along `r̂(θ)`. (The exact apex/base radial offsets follow the
`cc_*` contract's V geometry; the key invariant is that the apex direction is `r̂(θ)`.)

### 2.5 Tile into ruled bands + caps

Exactly the Tier-B / Tier-C machinery:

- SHARED per-station vertex ring: `ring[s][i] = makeVertex(P_s(section_i))` for the 3
  section vertices at each of the `nStations` stations.
- One bilinear ruled side band per (section edge `i` × spine segment `s`) via
  `detail::ruledSideFace(ring[s][i], ring[s][j], ring[s+1][i], ring[s+1][j], orient)`,
  with `orient` chosen so the natural bilinear normal points outward (away from the
  helix centre).
- Two end caps: `detail::planarFace` on the start ring (θ = 0, outward −ẑ-ish along
  the start section normal) and the end ring (θ = 2π·turns), sharing the ring vertices.
- `makeShell(faces)` → `makeSolid({shell})`. The two-stage tessellator welds the shared
  rings watertight (same weld path as the loft/sweep bands).

### 2.6 Sample cap (bounded work)

`samplesPerTurn` is clamped to `kMaxSamplesPerTurn` (e.g. 180 → 2° stations) and the
TOTAL station count `turns · samplesPerTurn` is clamped to `kMaxStations` (e.g. 4096)
so a pathological `turns × samplesPerTurn` cannot allocate unboundedly; if the requested
resolution would exceed the cap, the effective `samplesPerTurn` is reduced (the mesh is
still deflection-bounded vs the oracle). This matches the contract's "samplesPerTurn
capped".

### 2.7 Self-intersection guard (honest fall-through — return NULL)

A thread self-intersects when the radial V is deeper than the axial room between
adjacent turns. Guard (return NULL → OCCT fall-through) when ANY of:

- **Pitch room:** `2 · depthMM · tan(flankAngleDeg) ≥ pitchMM · kPitchSafety` — the two
  base half-widths of one turn's V already consume the whole pitch, so consecutive
  turns' flanks touch/cross (`kPitchSafety ≈ 0.95`).
- **Depth vs radius:** `depthMM ≥ R_min` for a tapered thread (the V would cross the
  axis at the tip), or `depthMM` implausibly large vs `majorRadiusMM`.
- **Flank crossing:** an explicit test that the transported baseHi of turn `n` does not
  pass the baseLo of turn `n+1` in Z (the discrete version of the pitch-room test at
  the tapered radius).

Degenerate guards: `turns > 0`, `pitchMM > 0`, `depthMM > 0`, `flankAngleDeg ∈ (0, 90)`,
`majorRadiusMM > 0` (helical) / `topRadiusMM, tipRadiusMM > 0` (tapered); else NULL.

The native builder NEVER emits the round-profile fallback itself — when the radial V
cannot form a valid solid it returns NULL and OCCT (which owns the documented
round-profile fallback) produces the result.

### 2.8 Volume / correctness check (host + oracle)

- **Host:** a coarse-pitch, shallow-depth thread (guard passes) is watertight
  (`boundaryEdgeCount == 0`); the section apex direction at each station equals `r̂(θ)`
  (radial invariant, the axis-aux-spine property — this is the crux assertion, the
  analogue of Tier C's constant-frame invariance test); the mesh volume is within a
  deflection band of the swept-V analytic estimate.
- **Sim parity:** native vs OCCT `BRepOffsetAPI_MakePipeShell` (aux-spine mode) —
  volume / bbox / centroid / watertightness within a documented deflection tolerance;
  native face count an integer multiple `k ≥ 1` of the OCCT face count (the native
  builder emits per-band faces, the oracle a periodic swept face — a representational,
  not geometric, difference, as documented for the earlier tiers).

### 2.9 Cognitive complexity

`build_helical_thread` is a linear station-loop assembler (target systems band ≤ 25,
flag if higher). The helix sampler, the axis-aux-spine frame (`r̂`/`ẑ` from θ), the
V-section builder, and the self-intersection guard are each short (≤ 10) helpers.
`build_tapered_thread` shares the same assembler with a tapering `R(θ)`.

## 3. Honest native-vs-fallback decision (per op)

| Op | Plan | Native lands iff |
|---|---|---|
| `cc_tapered_shank` | revolve of the silhouette via `build_revolution` | both gates green (expected — it is a special case of the already-verified revolve) |
| `cc_helical_thread` | radial-V axis-aux-spine helical sweep, guarded | both gates green (watertight + oracle-matched within deflection) — **else stays labelled OCCT fall-through** |
| `cc_tapered_thread` | as helical with a tapering `R(θ)` | both gates green — **else stays labelled OCCT fall-through** |

If either thread op cannot be made watertight + oracle-correct for the test cases, its
`NativeEngine` glue is left as pure fall-through (labelled), the spec's thread
requirements are recorded as **deferred** (the fall-through requirement covers them),
and the change lands with `cc_tapered_shank` native only — reported honestly in
`NATIVE-REWRITE.md` / `docs/STATUS-phase-4.md`.

## 4. NativeEngine wiring

- `tapered_shank` → `ncst::build_tapered_shank`; NULL ⇒ `fallback().tapered_shank(...)`.
- `helical_thread` → `ncst::build_helical_thread`; NULL ⇒ `fallback().helical_thread(...)`
  (or pure fall-through if the op did not pass the gates).
- `tapered_thread` → `ncst::build_tapered_thread`; NULL ⇒ `fallback().tapered_thread(...)`
  (or pure fall-through if the op did not pass the gates).
- OCCT referenced only under `CYBERCAD_HAS_OCCT`; the native builder references no OCCT
  / `IEngine` / `EngineShape` type; `native_engine.h` unchanged (signatures present).
- No `cc_*` signature or POD layout change.

## 5. Testing

- **Gate 1 (host, no OCCT):** `tests/test_native_thread.cpp` — shank watertight +
  volume; thread (if native) watertight + radial-section invariant + volume; guards +
  degenerate → NULL. Plus facade cases in `tests/test_native_engine.cpp`
  (`cc_tapered_shank` native id ≠ 0; a guarded fine-pitch thread falling through).
- **Gate 2 (sim, OCCT oracle):** `tests/sim/native_thread_parity.mm` +
  `scripts/run-sim-native-thread.sh` — `cc_set_engine(0/1)` (default restored in
  teardown), native vs `MakeRevol` / `MakePipeShell`, plus fall-through proof for the
  guarded / deferred cases. Own `main()`, on the `run-sim-suite.sh` SKIP list so the
  221-assertion count is unchanged.
