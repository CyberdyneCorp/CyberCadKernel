## Why

Native STEP/IGES **import** and general **shape healing** are the two remaining
tracks (with SSI + curved booleans) that gate the drop-OCCT endgame
(`NATIVE-REWRITE.md` #4/#3, `ROADMAP.md`). They are inseparable: **imported B-rep
always arrives with defects** — sub-tolerance gaps between faces that should share
an edge, near-coincident-but-distinct vertices, zero-length edges, sliver/zero-area
faces, and inconsistent face orientation. Without a healer, a natively-parsed STEP
face soup is not a valid, watertight solid the rest of the kernel can tessellate,
mass, boolean, or query. Healing is therefore the **gating foundation** a future
native STEP import must stand on.

This change delivers the **first native shape-healing slice**: the tractable ~80%
of the defect space — the **coincident-within-tolerance / degenerate / orientation**
family — that closed-form / spatial-hash reasoning heals exactly, verified
native-vs-OCCT. It deliberately scopes OUT the research-grade tail (pcurve
reconstruction, self-intersecting-wire repair, beyond-tolerance gap bridging), which
stays OCCT `ShapeFix`, exactly the same **narrow-verified-slice + explicit OCCT
fallback** discipline every prior native tier used (planar booleans, box∩cylinder,
native threads, SSI S1–S4).

Like SSI, healing is an **INTERNAL** capability. It is invoked by the engine (e.g.
after a future native import, or to repair a boolean/loft result), **not** exposed
on the `cc_*` C ABI. So there is **no ABI change** and it is verified at the
`cybercad::native::heal` C++/simulator boundary against OCCT
`BRepBuilderAPI_Sewing` / `ShapeFix_Shell` / `ShapeFix_Solid` — the same internal
parity discipline as `native-ssi`, `native-math`, and `native-topology`.

**Honesty is the load-bearing constraint.** The healer NEVER fabricates a closure:
it stitches two edges/vertices ONLY when they are coincident within the stated
tolerance. A gap beyond tolerance, a genuinely open shell, a missing pcurve, or a
self-intersecting wire is OUT OF SCOPE — the healer reports the result **UNHEALED**
(returns the shape unchanged / a typed "not healed" result carrying the measured
residual gap) and the engine falls through to OCCT `ShapeFix`. It NEVER weakens a
tolerance to pass and NEVER claims watertight when it is not.

## What Changes

- Add a native, OCCT-free **shape-healing module** under `src/native/heal/` that
  takes a `topology::Shape` (a face soup / open or malformed shell) and a
  `HealOptions{ tolerance }` and returns a `HealResult` — either a repaired
  connected, consistently-oriented, watertight shell/solid, or an honest
  **UNHEALED** verdict with the measured residual — implementing four
  sub-operations:
  1. **Tolerant sewing** — stitch a face soup whose shared edges/vertices are
     coincident *within tolerance* (but not topologically shared) into a connected
     shell by merging each such edge pair into ONE shared edge referenced by both
     faces (generalizing the `boolean/assemble.h` `VertexPool` + tessellator
     shared-edge weld to the B-rep edge level).
  2. **Vertex / tolerance unification** — merge near-coincident vertices onto one
     shared topology `Vertex` node via a quantized spatial hash at the weld
     tolerance (the `VertexPool` primitive, generalized to arbitrary B-rep input).
  3. **Degenerate removal** — drop zero-length edges (length < tol) and
     sliver / zero-area faces (area < tol²), rebuilding the affected wires/faces.
  4. **Orientation fix** — flood-fill orientation across shared edges so every face
     normal points consistently outward, seeded from a face whose outward direction
     is unambiguous (and confirmed by the sign of the enclosed volume of the
     resulting closed mesh).
- Add a **self-verify** step: after sewing + orientation, tessellate the candidate
  shell and confirm it is **watertight** (`tessellate::isWatertight`) with a
  consistent (positive-enclosed-volume) orientation, and that every merged pair was
  within tolerance. If watertight ⇒ report `healed = true` with the metrics; if the
  shell is still open (a real gap > tol, or an out-of-scope defect) ⇒ report
  `healed = false` (UNHEALED) with the measured `maxResidualGap`.
- Add an **engine-internal native-heal hook** (`src/engine/native/`) with the same
  try-native → self-verify → **OCCT fallback** ladder every native op uses: attempt
  the native heal, keep it only if it self-verifies watertight/valid, otherwise fall
  through to the OCCT adapter's `BRepBuilderAPI_Sewing` + `ShapeFix_Shell` /
  `ShapeFix_Solid`. **No `cc_*` entry point** is added or changed — the hook is
  reached internally (e.g. by a future import / by a repair path), exactly like the
  SSI hook.
- Report, in the `HealResult`, an honest metrics record: `healed`, `watertight`,
  `valid`, `nMergedVerts`, `nMergedEdges`, `nDroppedDegenerate`, `nFlipped`,
  `maxResidualGap`.

**Out of scope (return UNHEALED → defer to OCCT `ShapeFix`, never faked):** any gap
BEYOND tolerance; a genuinely open shell that cannot close within tolerance; missing
pcurve reconstruction; self-intersecting wires; general non-coincident /
non-degenerate industrial B-rep repair; freeform-surface re-approximation. This is
an **asymptotic-completeness** slice (like SSI S4-f): it heals the
coincident-within-tolerance / degenerate / orientation defect family — **measured
wins vs OCCT, not a guarantee** to heal arbitrary broken B-rep.

**No `cc_*` ABI change.** Healing is internal; the public C facade, POD structs, and
the tessellator are untouched. Additive only.

## Capabilities

### New Capabilities
- `native-healing`: native, OCCT-free shape healing (the first slice) for the
  coincident-within-tolerance / degenerate / orientation defect family — tolerant
  sewing of a face soup into a watertight shell, vertex/tolerance unification,
  degenerate-edge/sliver-face removal, and consistent outward orientation — with a
  mandatory self-verify (watertight + valid + all-merges-within-tolerance) and an
  honest **UNHEALED** report (measured residual gap) for anything out of scope, so
  the engine defers it to OCCT `BRepBuilderAPI_Sewing` / `ShapeFix`. Consumes
  `native-topology` (the B-rep model it reads + rebuilds) and `native-tessellation`
  (the watertight self-verify + enclosed-volume orientation check). Reuses the
  `native-booleans` `VertexPool` weld primitive. Verified at the healing-function
  level native-vs-OCCT; the gating foundation for a future native STEP import. No
  `cc_*` change.

### Modified Capabilities
<!-- none — native-healing is a new internal module. It does not modify any cc_*
     signature, POD struct, or existing native capability; it only CONSUMES
     native-topology + native-tessellation and REUSES the native-booleans
     VertexPool. The engine-internal heal hook is additive (a new private code path,
     no IEngine/cc_* signature change). -->

## Impact

- **ABI**: none. Healing is an internal native capability; no `cc_*` entry point,
  signature, or POD struct changes. The tessellator is not modified.
- **Build**: adds `src/native/heal/` (OCCT-free, header-heavy topology + math). It
  needs no numeric substrate (pure spatial-hash + closed-form geometry), so it
  builds under BOTH the default no-OCCT config and `CYBERCAD_HAS_NUMSCI` with no
  interaction. The engine-internal OCCT fallback lives under `CYBERCAD_HAS_OCCT` in
  `src/engine/` (never in `src/native/**`).
- **Verification**: two gates — **host** (OCCT-free CTest: deliberately-broken
  fixtures healed natively, asserting watertight + valid + measured merges/residual
  + the UN-healable fixture reporting UNHEALED) + **sim native-vs-OCCT**
  (`BRepBuilderAPI_Sewing` / `ShapeFix_Shell` / `ShapeFix_Solid` on the same
  fixtures: same watertight/closed shell, same valid solid, same volume within tol;
  and on the un-healable fixture, native UNHEALED matches OCCT leaving it open /
  needing more). Same internal parity discipline as native-ssi / native-topology.
- **Roadmap**: implements the first slice of `NATIVE-REWRITE.md` #4 shape healing —
  the gating foundation for a future native STEP import (#3). It does NOT complete
  healing; the research-grade tail (pcurve reconstruction, self-intersecting-wire
  repair, beyond-tolerance repair) stays OCCT `ShapeFix`.
- **Risk**: honest scope — every heal is self-verified watertight/valid before it is
  kept, and anything out of the coincident/degenerate/orientation family (or any gap
  beyond tolerance) returns UNHEALED and defers to OCCT, so the slice can never
  claim a false closure or a leaky "watertight". Completeness is asymptotic: a
  measured win over OCCT on the in-scope family, not a guarantee on arbitrary
  broken B-rep.
