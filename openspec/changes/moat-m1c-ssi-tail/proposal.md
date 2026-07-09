# Proposal — moat-m1c-ssi-tail (MOAT M1 SSI breadth #2: promote the declined off-axis quadric-intersection tail to verified)

## Why

M1b landed general skew cylinder∩cylinder and off-axis sphere∩cone as verified breadth, but
HONESTLY DECLINED the remaining off-axis quadric tail — **general cone∩cone**, **off-axis
cylinder∩cone**, and **off-axis sphere∩cylinder** — with a precisely-measured root cause:

1. **Oracle-setup mismatch.** OCCT `Geom_ConicalSurface` / `Geom_CylindricalSurface` are
   INFINITE (unbounded height, both nappes) while the native adapters are FINITE patches over
   a `ParamBox`. When an unbounded quadric pierces the other operand more than once along its
   infinite extent, `GeomAPI_IntSS` returns the full multi-loop INFINITE locus that the finite
   native trace legitimately cannot match — an apples-to-oranges comparison, not a native gap.

2. **Seeding-recall miss.** A twice-piercing off-axis pose has TWO disjoint loops, but at
   practical seed densities the coarse subdivision merges the two loops into ONE topological
   cluster (union-find over the param-adjacent leaf boxes), so the fixed-resolution seeder keeps
   only ONE representative seed → the second loop is missed.

Both are addressable without weakening any tolerance. This change promotes the tail to verified
via the two fixes M1b named as the sharpened next blocker.

## What

- **Fix 1 — domain-clipped oracle (TEST-HARNESS ONLY).** In the sim parity harness
  (`tests/sim/native_ssi_marching_parity.mm`) wrap each OCCT oracle surface in a
  `Geom_RectangularTrimmedSurface` trimmed to the SAME `[u0,u1]×[v0,v1]` the native adapter uses
  (`clipOracle`). The native adapters and the OCCT quadrics share the (u = angle, v =
  height/latitude) parameterisation, so the box maps 1:1. `GeomAPI_IntSS` on the trimmed surfaces
  produces the SAME finite locus the native trace covers — an apples-to-apples oracle. This does
  NOT touch `src/native` and never widens a tolerance.

- **Fix 2 — seeding-recall bump (src/native/ssi, ADDITIVE, default-off).** Add
  `SeedOptions::criticTargetedReseed` (+ `criticMaxCells` cost bound). When set (with
  `completenessCritic`), the S4-f completeness critic re-seeds ONLY the param cells NO traced
  curve covers: each uncovered A-cell is seeded as a RESTRICTED sub-domain (A clamped to the cell)
  against B's full domain, and the recovered seeds are accumulated. This is the residual/coverage-
  guided TARGETED re-seed — it reliably recovers the SECOND loop of a twice-piercing pose at
  practical grid densities (host-measured: fixed grid → 1 loop; bump → 2 loops, `criticRecoveredLoops
  ≥ 1`, both closed, every node on both surfaces ≤ 1e-9) and is cheaper than re-scanning the whole
  domain. It NEVER fabricates a seed (each cell's candidate must still land on BOTH surfaces) and
  NEVER widens a tolerance; it only changes WHERE the finer re-seed looks. **Both new flags default
  off**, so the fixed-resolution seed and the existing whole-domain S4-f critic are byte-identical
  for every already-passing case.

- **Promoted families (both gates green).**
  - **general cone∩cone** — two finite cones with offset apexes + tilted axes → ONE closed loop
    inside both finite patches; verified against a domain-clipped oracle.
  - **off-axis cyl∩cone** — the arc runs off the finite patch boundaries → ONE open (BoundaryExit)
    arc; verified against a domain-clipped oracle.
  - **off-axis sphere∩cyl (twice-piercing)** — the thin offset cylinder pierces the finite sphere
    on BOTH sides → TWO disjoint closed loops; the seeding-recall bump recovers the second loop and
    the domain-clipped oracle returns exactly the two finite loops.

- **Honest-decline preservation.** The S1 analytic dispatch STILL returns `NotAnalytic` for these
  non-closed-form pairs (correct — no closed form). This change verifies the S2/S3 marching path;
  it adds no fake closed form. Any pose that still cannot be robustly traced stays an honest
  decline — no tolerance is weakened to force a pass.

## Impact

- `src/native/ssi/` — ADDITIVE: two default-off `SeedOptions` fields + the targeted-reseed branch
  in the S4-f critic (`marching.cpp`). No existing behaviour changes (flags default off).
- `src/native/**` — OCCT-free and additive; `git diff src/native` touches only `ssi/seeding.h` +
  `ssi/marching.cpp`.
- Tests — additive: 3 new host cases + 3 new sim parity cases (existing cases frozen).
- `cc_*` ABI — **unchanged** (SSI is internal; asserted at the C++ boundary, no facade entry point).
- Tessellator (`src/native/tessellate/`) and boolean layer (`src/native/boolean/`) — **UNTOUCHED**.
- Spec — one new requirement in `native-ssi` (declined off-axis quadric tail promoted to verified).
- Roadmap — M1 status updated: the M1b-declined tail is promoted via domain-clipped oracle +
  targeted seeding-recall bump.
