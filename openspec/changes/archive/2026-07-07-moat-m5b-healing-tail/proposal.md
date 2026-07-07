# Proposal — moat-m5b-healing-tail

## Why

MOAT stage **M5** (`openspec/MOAT-ROADMAP.md`) is shape-healing robustness framed as
*bounded first slices with an asymptotic tail*. Two M5 slices have landed on the
clean-room, OCCT-free `cybercad::native::heal` module: the base healer (tolerant sew +
vertex/tolerance unify + degenerate removal + orientation flood-fill) and the opt-in
**bounded near-miss gap-bridging** pass (`gap_bridge.h`). Everything harder still
declines honestly to the OCCT `ShapeFix` oracle.

The roadmap M5 tail names three remaining declined classes (see
`UnhealedReason` in `src/native/heal/heal_result.h`): **pcurve reconstruction**,
**self-intersecting-wire repair**, and the **missing-face `OpenShell`**. This change
picks the SINGLE most-tractable one as the next bounded, opt-in slice, after a
diagnosis of what the existing module can actually reach:

- **Pcurve reconstruction — NOT reachable here.** The heal module carries no input
  pcurves at all: `face_soup.h` reads only world-corner loops and `assemble_shell.h`
  synthesizes trivial `Line` pcurves. Pcurve reconstruction is a *foreign trimmed
  B-spline* problem (STEP `EDGE_LOOP` bounds — M0/M4 territory), with no substrate in
  this module. Declined.
- **Self-intersecting-wire repair — NOT reachable as a bounded slice.** The module has
  no 2D wire-parameter machinery; the "repair" of a crossed wire (which crossing to
  cut, how to re-order) is genuinely ambiguous and is the deep `ShapeFix_Wire`
  moat. Declined.
- **Missing-face `OpenShell` — reachable, bounded, and reuses the whole pipeline.**
  When a soup sews cleanly (every corner paired within `tolerance`) but one face is
  simply absent, the sew leaves a ring of boundary edges (`EdgePool` already tags every
  side used exactly once). `heal.cpp` declines this today:

  ```cpp
  if (sr.boundaryEdges > 0) {
    const UnhealedReason why = sr.maxResidualGap > tol ? ... : UnhealedReason::OpenShell;
    return unhealed(shape, why, sr.maxResidualGap, m);
  }
  ```

  The existing `heal_open_shell_unhealed` host fixture (a unit cube with its `+Z` face
  erased) is exactly this defect: after sewing, the four top corners are already shared
  by the side faces and the four surviving boundary edges form ONE closed, planar,
  simple square loop. Capping that single planar hole with one synthesized planar face
  — built from the SAME shared boundary edge nodes, so it welds — closes the shell, and
  the UNCHANGED mandatory self-verify gate proves it watertight with positive volume.

This change adds exactly that ONE bounded slice: an **opt-in synthesis of a single
missing planar face** for a simple `OpenShell`, self-verified against the same
watertight + positive-volume gate, matched to an OCCT reference cap
(`BRepBuilderAPI_MakeFace` on the free-boundary wire + `ShapeFix_Solid`), with an
**honest decline** for every open shell outside the bound (more than one hole, a
non-planar hole, a self-intersecting hole, or a cap that fails self-verify) and an
explicit **asymptotic-tail caveat** for the arbitrary-broken-B-rep remainder that stays
OCCT's moat.

The non-negotiable disciplines carry over unchanged: `src/native/**` stays
**OCCT-free** (OCCT is the sim oracle only); the `cc_*` ABI is untouched (healing is
internal, no entry point); the tessellator is untouched; and NO tolerance is weakened —
a hole outside the bound is reported, not faked.

## What Changes

1. **Opt-in flag on `HealOptions`.** Add `bool capPlanarHoles = false;`. **Default
   `false` disables capping**, so `healShell` behaves byte-identically to the two landed
   slices — every existing host fixture and the `run-sim-native-heal.sh` fixtures are
   unchanged (no regression). The existing `heal_open_shell_unhealed` fixture (default
   opts) still returns `Unhealed{OpenShell}`.

2. **A bounded planar-hole capping pass** (`src/native/heal/cap_hole.h`, OCCT-free,
   header-only). After the primary weld (and optional bridging) leaves boundary edges,
   the pass traces the boundary edges into loops and synthesizes ONE cap face ONLY when
   ALL hold: (i) the boundary edges form EXACTLY ONE closed simple cycle (every boundary
   vertex has exactly two incident boundary edges — no branching, no second hole);
   (ii) the loop's corners are **coplanar within `tolerance`** (max deviation from the
   best-fit plane `≤ tolerance`); (iii) the loop is a **simple polygon** in that plane
   (no self-intersection). The cap is built from the loop's existing shared vertex/edge
   nodes (reusing the sew's boundary edge nodes so it welds), then the candidate re-runs
   orientation + the **mandatory self-verify**; a cap that does not tessellate watertight
   with positive volume is discarded → `Unhealed{SelfVerifyFailed}`.

3. **Honest decline + measured data.** Any open shell outside the bound (≥ 2 boundary
   loops, a non-planar loop, a self-intersecting loop, or a branching boundary) stays
   `Unhealed{OpenShell}` with `maxResidualGap` carrying the measured largest surviving
   gap and the input UNCHANGED — no new `UnhealedReason` is needed (`OpenShell` already
   means exactly "a real hole after sewing"). New additive `HealMetrics` fields
   `int nCappedFaces` and `double maxCapPlanarityDev` report how many holes were capped
   (`≤ 1` in this slice) and the largest coplanarity deviation of a capped loop
   (honestly `≤ tolerance`).

4. **OCCT oracle parity + honest decline.** The sim gate builds an independent OCCT
   reference cap — `BRepBuilderAPI_MakeFace(gp_Pln, freeBoundaryWire)` added to the
   shell + `ShapeFix_Solid` — for the in-scope fixture: the native capped solid matches
   OCCT in watertight/closed shell, valid solid, and volume within tolerance. The sim
   out-of-scope fixture is a TWO-hole open shell (two disjoint boundary loops):
   `Unhealed{OpenShell}` natively while OCCT sewing / `ShapeFix` also leaves it open,
   since neither invents an absent face (parity of decline). NOTE (empirical): OCCT
   `BRepBuilderAPI_MakeFace(gp_Pln, wire)` tolerates a mildly-non-planar wire (it keeps
   the wire's 3D vertices) and caps it, so native declining a single non-planar hole is
   native being MORE conservative and DEFERRING to OCCT — not a shared decline; the
   native planarity-layer decline is therefore verified by the host gate, not asserted as
   a shared sim decline. The host gate proves the capped-cube analytic volume (`= 1.0`,
   no OCCT), the two-hole and non-planar declines, and the default-off no-op.

**Additive only.** No `cc_*` entry point, signature, or POD changes; no tessellator
change; `HealOptions` / `HealMetrics` are internal C++ types extended additively; no new
`UnhealedReason` value. `src/native/**` includes no OCCT header.

## Impact

- **Specs:** `native-healing` — one ADDED requirement (the bounded single-planar-hole
  capping pass, incl. the capped-cube analytic host gate and OCCT-reference-cap sim
  parity) and one MODIFIED requirement (carve the opt-in single-planar-hole cap out of
  the "genuinely open shell / missing face is out of scope" clause, keeping the ≥ 2-hole
  / non-planar / self-intersecting remainder deferred).
- **Code:** additive `src/native/heal/cap_hole.h`; small additive fields on
  `heal_result.h` (`HealOptions.capPlanarHoles`, `HealMetrics.nCappedFaces` /
  `.maxCapPlanarityDev`); one new guarded branch in `heal.cpp` (runs only when
  `capPlanarHoles == true` and boundary edges survive after sew/bridge).
- **Tests:** new host fixtures (capped-cube heals to `V = 1.0`; two-hole and non-planar
  open shells decline; default-off no-op) in the `native-healing` host suite; a matching
  OCCT-reference-cap pair in `tests/sim/native_heal_parity.mm`.
- **No impact:** `cc_*` ABI, the tessellator, the CyberCad app, and every landed heal
  fixture (capping is default-off; `heal_open_shell_unhealed` unchanged).
