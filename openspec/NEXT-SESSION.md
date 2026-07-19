# Next session — validated work queue

Written 2026-07-19 at `732788d`. Everything below was **measured and adversarially verified** by a
five-workstream investigation; each item says whether a remedy was actually **built and run** or is
a **proposal**. Nothing here is a guess. Where an earlier belief was overturned, the correction is
stated rather than the old text edited away.

Read with [MOAT-ROADMAP.md](MOAT-ROADMAP.md) §M1, which carries the geometry history.

---

## Environment (all of this now works on Linux)

| | |
|---|---|
| Host suites | `cmake --build build-host-verify --target <t> -j8 && ./build-host-verify/<t>` |
| | marching 26/26 (~5 min) · seeding 15/15 (~55 s) · ssi 11/11 · s4f 7/7 · s4_classification 22/22 (~10 min) · exact_fuzz 147 agreed/0 disagreed · patch_gap 6/6 |
| Host parity gates | `bash scripts/run-host-sim-parity.sh <harness>` · `--list` enumerates |
| | ssi-marching 22/0 · curved-wall-cut 68/0 · abi 11/11 |
| OCCT | source-built 7.6 at `/home/leonardo/work/occt-build/install` (the distro package **cannot** build mesh-touching harnesses — it omits `NCollection_AliasedArray.hxx`) |
| Facade w/ engine | `cmake -S . -B build-host-occt -DCYBERCAD_LINUX_OCCT=ON -DCYBERCAD_HAS_NUMSCI=ON ...` |
| ⚠ | the parity runner links `build-host-occt/libcybercadkernel.a`. Change a header ⇒ **rebuild it** or the staleness guard stops you (before it would segfault). |

**Slow is not hung.** Gate A ≈ 5 min, s4_classification ≈ 10 min, the disjoint-pair repro ≈ 537 s.

---

## 1. `refineRegion` separating-slab prune — **highest value; prune validated by a corpus run**

**The measured problem.** `refineRegion` runs **once per candidate region**, not once per branch:
1 835 481 calls for 1 835 481 candidates at **205–227 µs** each = **65% of wall**, `least_squares`
99.9% of that. On the disjoint near-parallel pose all of it is waste (`converged=0`,
`offBranch=1 835 481`, `seeds=0`). The comment claiming otherwise is corrected at `seeding.cpp:900`.

**Two obvious prefilters are dead — do not retry.** The AABB test is a provable no-op (`subdivide`
already runs `aabbDisjoint` before emitting). `patchGapBound` is the wrong direction: a sound
*upper* bound certifying coincidence, where skipping a refine needs a *lower* bound.

**The remedy.** An oriented **separating-slab prune**, inserted immediately after the existing
`aabbDisjoint` test in `subdivide` (`src/native/ssi/seeding.cpp`). Project both restricted Bézier
sub-nets (`detail::bezierSubNet`, already in `patch_gap.h`) onto a direction `n` (A's midpoint
normal); prune when the projected intervals are separated by more than `gap`, mirroring
`aabbDisjoint`'s slack. Gate exactly like the coincidence certificate:
`bothFreeform && A.hasBezierNet && B.hasBezierNet`. ~35–40 lines.

| pose | candidates | wall | seeds |
|---|---|---|---|
| disjoint dz=1e-3 (target) | 1 835 481 → **0** | 634 s → **0.073 s** | 0 → 0 |
| dish × tilt s=0.2 | 90 859 → 5 112 | 35.1 → 0.56 s (**62×**) | 1 → 1 |
| dish × tilt s=0.5 | 58 761 → 9 447 | 9.55 → 0.95 s | 1 → 1 |
| dish × plane z=0.30 | 38 128 → 12 767 | 4.61 → 1.26 s | 1 → 1 |
| coincident dz=0 | 144 → 144 | unchanged (certificate short-circuits) | 0 → 0 |

Cost on ordinary poses 1.7–1.8 µs/call, 0.026–0.151 s total. `s4_classification` 8m38 → 8m41
(elementary operands ⇒ `hasBezierNet=false` ⇒ no-op).

**Soundness.** Convex-hull property: `S(box)` lies inside the sub-net hull, so disjoint projections
*prove* no crossing in that box pair — and soundness does not depend on the choice of `n` (a poor
`n` merely fails to separate). Descendant param boxes are contained in the parent's, so a
parent-level proof kills only crossing-free subtrees. Verified at **6 480 000 samples, 0 hull
violations**. Degenerate normals fail safe (`math/vec.h:119-137` keeps a near-null `Dir3` with
`valid()==false`, so `|n| ≤ 1` and separation can only be under-estimated).

> **Write the CONTAINMENT argument in the comment, not an argument from precedent.** "Box-locality
> is already trusted inside subdivide" would equally license the refine-site filter that is
> correctly rejected below.

**Why the same predicate is disqualified at the refine site:** `refineRegion` clamps into the
**full domain**, so it is effectively a global solve — **97.8% of accepted refines converge outside
their own candidate box**. Filtering there drops 25 324 / 85 678 / 49 284 seeds on the transversal
controls.

**Before landing:** (a) ship the hull-containment unit test; (b) **run the co-resident multi-loop
family to completion** — 40 poses were built and emitted no anomaly but did not finish; (c) pin the
target pose as a regression test; (d) simulator gate — `native_ssi_marching_parity`'s
bspline×bspline line *does* move (`onCurve 1.65e-07 → 1.74e-07`, benign, 3–4 orders under
tolerance, but not byte-identical).

---

## 2. Five parity assertion re-scopes — **all built and run**

**All 7 failures are harness-side. Zero kernel defects.** No `src/` change, so native suites are
invariant by construction. Sequence **geomcompletion first** — its assertion is actively harmful.

| harness | verdict | fix | measured |
|---|---|---|---|
| `native_geomcompletion_parity` | **Invalid oracle.** OCCT `twisted_sweep` at twist=π/2 is **byte-identical to twist=0** (vol 320.000000, area 352.000000, 12 tris both) — the parameter is inert, root cause `occt_construct.cpp:1241` emits only 2 wires. Native's 319.29 is *closer to analytic 320*. | Assert the twist **signature**: `bbox max|xy| ≥ 2√2 − ε` (native 2.8284, OCCT 2.0000) + faceting band. The new assertion **rejects** OCCT's prism. | 26/1 → **27/0** |
| `native_numerics_parity` | **Defective assertion.** sphere#1 target is exactly the polar axis; `u` is degenerate at `v=π/2`. `dPoint=4.965e-16` *is* agreement. | Skip the `dU` clause at the pole — **gate on surface type, not on `\|v\|−π/2 ≤ 1e-9`**, or a cylinder fixture with axial `v=π/2` silently loses its check. | 21/1 → **22/0** |
| `native_construct_profiles_parity` | **Stale delegation contract.** Native no longer delegates: 12 edges vs OCCT's 6, vol 45.5547 vs 45.6, bboxes agree to 1e-3. The `1e-9` bit-identity was only valid under delegation. | Reclassify as native with a deflection-derived volume band (`rel=9.92e-04`). | 21/1 → **22/0** |
| `native_freeform_freeform_cut_parity` + `native_nurbs_solid_boolean_parity` | **Unsatisfiable clause**, one shared cause (same `ffx::` fixture). Accuracy passes with 12× margin; only `eU < prevCut` fails, and it is arithmetically empty for *any* oracle (needs `X > 0.0366410` **and** `X < 0.0366145`). | finest-beats-coarsest + a fixed band. **Not** closed-form scoring — at the current 3 deflections `errVsCF` is *worse* than `errVsOCCT`. | 13/1 → **16/0**, 13/1 → **17/0** |

**Not built, real design content:**
- `native_tessellate_parity` — the **test-only `OcctBridge`** loses edge sharing (`addPCurve` per face returns a new TShape; cylinder edge-face incidences 6→5), not the kernel mesher. Fix: accumulate per-edge pcurves, build each edge node once. Until then, comment that the assertion measures the bridge — *"the mesher leaks on cylinders"* is the wrong conclusion to carry forward.
- `native_thread_parity` — fixture axis mismatch (`cc_solid_revolve` uses +Y, `cc_helical_thread` builds about +Z); the kernel guard is correct. **The obvious two-line fix is harmful**: rotating only the shaft makes the test pass via `honestSuccess` and the defect-2 guard is never exercised — a loudly-failing test becomes a silently-vacuous one. Both bodies must rotate. Outcome is also **state-dependent** (declines 3/3 standalone, succeeds after 13 preceding cases). Needs redesign.

---

## 3. Recall denominator in `native_ssi_seeding_parity` — **proposal**

The harness now compiles and runs (`732788d`) — 1 passed / 3 failed. **Two failures are false**: it
counts OCCT arc pieces, not connected components. Repair: merge arcs meeting at degree-2 nodes,
keep genuine junctions distinct. The hardcoded expectations (1, 2, 1) are already correct against
true components. Full detail in the harness header. Also widen `classifyBranch`'s try/catch to cover
`crossingSineOnOcct` (latent here — min sine 0.40).

**The one real gap it exposes:** `bspline × plane` at recall 0.03 — 40 arcs over 32 nodes with **16
junctions**, one connected saddle network. Marching terminates at junctions, so one seed cannot
cover it. **No currently-executing gate covers this case.**

---

## 4. M0 collar retune — **validated by a corpus run**

**The fine-deflection weld problem is RESOLVED** — the shared-strip cache and two-phase fill both
landed (`seam_strip.h` `SeamStripRegistry`, `face_mesher.h:752-790`; commits `c655fe7 5da6e5a
1b08a4a 2394032`). The roadmap's `L3-BAND` table recording `nonmanif = 0/1/4` is **measured-false**:
re-running the repo's own `measure_multiseam_fine` gives **0/0/0**.

**A different problem is live.** The deflection-robust collar makes the suppression band a fixed
fraction of `rSeam`, so it injects an error that does **not** shrink with refinement
(`seam_strip.h:224-225` builds `collarIn/collarOut` as pure radial offsets with no projection back
onto the face). COMMON is dominated by exactly the region the collar flattens, and the acceptance
band `min(0.5, 30·d)·V` tightens linearly while the error is flat ⇒ decline below ~0.002 is
structural. In every `VolumeInconsistent` row the mesher reports `wt=1 coh=1 boundaryEdges=0` —
**the weld is fine; the volume self-verify rejects.**

**Slice:** `kCollarFrac` 0.05 → 0.01 (`seam_strip.h:403`). 27 parity harnesses bit-identical,
8 verb rows improved / 0 degraded. Move the decline pins in `test_native_seam_strip_weld.cpp:111,122`
and `test_native_multiseam_asym.cpp:280,284` to `d=0.0006` (measured `NotWatertight` at both fracs) —
**keep a decline pin** so the never-leaky property stays covered. Assert `matchTol_ ≥ max seam
segment length`. **Claim two steps of improvement, not three.**

---

## 5. Fuzzers — **done, no action**

`DISAGREED == 0` across **1 468 differential trials** plus 3 fresh seeds each. All `M6 BAR: PASS`;
`FALLBACK_ORACLE_INVALID = 0`, `OPERAND_MISMATCH = 0`. Per-seed output byte-identical
(splitmix64/xoshiro256**, no clock). The soundness invariant is intact **and now seed-swept**.
Budgets: boolean_fuzz ~14 min, freeform_boolean_fuzz ~9 min, ssi_freeform_fuzz ~21 min.

---

## 6. Simulator confirmation — **the cheapest open refutation route**

Six changes await it: `moat-m1d/m1e/m1f`, `moat-gs2-section-curves`, `moat-m0-freeform-mesher`,
`moat-m2c3f-strip-weld-fuse` — plus the A2 certificate work and anything above.

Every mechanism in this document is host-independent **by construction**, but that is *inference*.
Items 1 and 4 both produce non-byte-identical output on a different toolchain and OCCT build. Also
one-line: `grep Magnitude` in the simulator's `gp_Dir.hxx` to close the last gap on §3.

---

## Still genuinely open (no mechanism)

**A2 near-parallel proximity.** The coincidence certificate solved the coincident case
(1 835 481 → 144 candidates). The band it sits inside is not solved: the expensive range is
dz ∈ [0, ~2e-2] while the *detectable* range is dz ≤ 2.9e-7 — **five orders of magnitude are
provably not coincident**. Item 1 attacks this from the cost side and largely wins on the measured
poses, but the general predicate remains open. Per-cell gap predicates are ruled out **as a class**:
restricted to a small enough sub-box a genuine tangency *is* coincident to tolerance (exact quartic
contact, 864× tolerance at the root, certifies at the first descent level; the gap-to-witness ratio
is constant in cell size). See the header of `src/native/ssi/patch_gap.h`.
