# Next session — validated work queue

Written 2026-07-19 at `732788d`; **substantially revised after execution.**

> ⚠ **READ THIS BEFORE TRUSTING ANY ENTRY BELOW.** The original document claimed everything in it
> was "measured and adversarially verified … nothing here is a guess". Executing it disproved that.
> Checked against fresh measurement, the queue was **wrong on 8 of 11 checkable claims**, and **two
> of its prescribed remedies were actively harmful** — one rested an assertion on a quantity that
> provably cannot discriminate the two solids (volume is twist-invariant by Cavalieri), and one
> would have shipped the exact vacuity it was warning against. It is reliable at naming **where**
> to look and unreliable on **mechanism** and **remedy**.
>
> **Treat every stated remedy as a hypothesis to implement and measure, never as a spec.** Refuting
> an entry is a successful outcome; several below are now marked REFUTED and are more useful for it.
>
> The same discipline failed in the other direction and cost more. A "possible soundness defect"
> (order-dependence in the multi-seam boolean) was reported, escalated above all other work, and
> bisected against a clean worktree — and **does not exist**. The probe had a header-only constant
> compiled at one value and linked against an archive built at another, so it ran both at once.
> Verifying the *archive* is not verifying the *configuration*. Use
> `scripts/run-host-suite.sh` (`518dd17`), which makes that class of error unrepresentable.

Read with [MOAT-ROADMAP.md](MOAT-ROADMAP.md) §M1, which carries the geometry history.

**On a Mac, start at [§6](#6-simulator-confirmation--the-mac-session-ready-to-run).** Five changes
are one task from done and four of them confirm through a single harness run. Nothing else in this
document needs doing there — every remaining item is Linux work.

---

## Environment (all of this now works on Linux)

| | |
|---|---|
| Host suites | **`bash scripts/run-host-suite.sh <t>`** — builds then runs, so a stale binary cannot report a verdict for code that is gone. `--list` (135 suites) · `--all`. Running `./build-host-verify/<t>` directly is the trap that produced three false conclusions. |
| | marching 26/26 (~5 min) · seeding 16/16 (~55 s) · ssi 11/11 · s4f 7/7 · s4_classification 22/22 (~10 min) · exact_fuzz 147 agreed/0 disagreed · patch_gap 10/10 |
| Host parity gates | `bash scripts/run-host-sim-parity.sh <harness>` · `--list` enumerates |
| | ssi-marching 22/0 · curved-wall-cut 68/0 · abi 11/11 |
| OCCT | source-built 7.6 at `/home/leonardo/work/occt-build/install` (the distro package **cannot** build mesh-touching harnesses — it omits `NCollection_AliasedArray.hxx`) |
| Facade w/ engine | `cmake -S . -B build-host-occt -DCYBERCAD_LINUX_OCCT=ON -DCYBERCAD_HAS_NUMSCI=ON ...` |
| ⚠ | the parity runner links `build-host-occt/libcybercadkernel.a`. Change a header ⇒ **rebuild it** or the staleness guard stops you (before it would segfault). |

**Slow is not hung.** Gate A ≈ 5 min, s4_classification ≈ 10 min, the disjoint-pair repro ≈ 537 s.

---

## 1. Separating-slab prune — ✅ **LANDED**, see [MOAT-ROADMAP.md](MOAT-ROADMAP.md) §M1 A2

Implemented as `slabSeparated` in `src/native/ssi/patch_gap.h`, called from `subdivide` right
after `aabbDisjoint`. Verified against a saved baseline archive: **seeds, branches and
coincidence verdicts identical on all 34 co-resident family poses and all 11 bench poses**;
target pose 1 835 481 → 0 candidates, 533 s → 0.056 s. Gate A green across six suites.

**What it did NOT close.** The prune fires only while `κh²/2 < dz` (see the roadmap for why the
midpoint normal makes the patch extent second-order). Below ~3e-4 on these operands nothing
separates at reachable cell sizes. The `wave × wave` family sweep was trimmed to the two poses
ABOVE that threshold for runtime — do not read its 1 891 467 → 0 as covering the band.

**Still owed:** ~~the simulator gate (item 6)~~ — **paid 2026-07-19**: sim marching parity
22/0, no seed/branch movement, only the predicted benign onCurve shift. Change archived.

**Do NOT move this predicate to the refine site.** `refineRegion` clamps into the FULL domain, so
it is effectively a global solve — **97.8% of accepted refines converge outside their own candidate
box**, and filtering there drops 25 324 / 85 678 / 49 284 seeds on the transversal controls. The
prune is sound in `subdivide` because a descendant's param boxes are contained in its parent's,
and for no other reason.

<details><summary>Original entry (superseded)</summary>

### `refineRegion` separating-slab prune — highest value; prune validated by a corpus run

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

</details>

---

## 2. Five parity assertion re-scopes — ✅ **ALL LANDED**

`4338995` geomcompletion · `232fb57` numerics · `a721d0e` construct_profiles · `fd88077` ff_cut +
nurbs_solid_boolean. **No `src/` change in any of them**, so the native suites are invariant by
construction.

| harness | before | after |
|---|---|---|
| `native_geomcompletion_parity` | 26/1 | **27/0** |
| `native_numerics_parity` | 21/1 | **22/0** |
| `native_construct_profiles_parity` | 21/1 | **25/0** |
| `native_freeform_freeform_cut_parity` | 13/1 | **15/0** |
| `native_nurbs_solid_boolean_parity` | 13/1 | **15/0** |

**Verifying each independently was load-bearing — the entry below was wrong on two of five.**

* **geomcompletion — all three claims below are FALSE.** The twist parameter is *not* inert: at
  `pathCount=2` it works at 0, π/8, π/4, 3π/8 (vol 320.000 / 311.880 / 288.758 / 311.880) and
  fails *only* at exactly π/2. Two wires is the trigger, not the cause — **a square is invariant
  under a π/2 rotation**, so both wires coincide and ThruSections pairs them by position rather
  than index, yielding the straight prism. And "native's 319.29 is closer to analytic 320" is
  **backwards**: OCCT's 320.000000 *is* 320. ⚠ **The converged volume is exactly 320 — by
  Cavalieri every cross-section of a twisted prism is the same square of area 16, so volume is
  16·20 for ANY twist.** Volume is therefore the one quantity that CANNOT discriminate the two
  solids; the proposed "faceting band" half of the fix was meaningless. What landed keys on bbox
  reach (2.8284 vs 2.0000) and lateral area, with volume only as a band *below* the closed form.
  OCCT's reach is logged but deliberately **not** a pass condition, so a future OCCT fix cannot
  turn this case into a spurious failure.
* **construct_profiles — the prescription worked but was bettered.** Reclassifying as a native
  `OpCase` gives four assertions (mass/bbox/faces/tessellate) instead of one volume band, hence
  25/0 rather than the predicted 22/0.
* **ff_cut / nurbs_solid_boolean — the arithmetic was exactly right**, re-derived independently
  before touching it. Landed as a per-deflection accuracy band plus ONE post-loop
  finest-beats-coarsest check (0.0083 → 0.0069). ⚠ That is a genuinely **weaker** property than
  the monotonicity it replaced; COMMON and FUSE keep the stronger step-wise form because they
  earn it on the same fixture.
* **numerics — as described.** Gated on surface KIND (`UDegeneracy::SphericalPole`), not on the
  value of v. The other three sphere targets keep their full `dU` check (0.0, 1.7e-11, 1.8e-11).

<details><summary>Original entry (superseded — retains the two false claims for the record)</summary>

**Not built, real design content:**
- `native_tessellate_parity` — the **test-only `OcctBridge`** loses edge sharing (`addPCurve` per face returns a new TShape; cylinder edge-face incidences 6→5), not the kernel mesher. Fix: accumulate per-edge pcurves, build each edge node once. Until then, comment that the assertion measures the bridge — *"the mesher leaks on cylinders"* is the wrong conclusion to carry forward.
- `native_thread_parity` — fixture axis mismatch (`cc_solid_revolve` uses +Y, `cc_helical_thread` builds about +Z); the kernel guard is correct. **The obvious two-line fix is harmful**: rotating only the shaft makes the test pass via `honestSuccess` and the defect-2 guard is never exercised — a loudly-failing test becomes a silently-vacuous one. Both bodies must rotate. Outcome is also **state-dependent** (declines 3/3 standalone, succeeds after 13 preceding cases). Needs redesign.

</details>

**Still open in §2 (both unbuilt, both real design content):** the `native_tessellate_parity`
`OcctBridge` edge-sharing refactor and the `native_thread_parity` redesign, described just above.

---

## 3. Recall denominator — ✅ **LANDED** (`94fc2c1`)

Arcs meeting at degree-2 nodes are merged; genuine junctions stay distinct. **1 passed / 3 failed
→ 3 passed / 1 failed.** The guard is two-sided across the SUITE, not per-pair: under-merging is
caught by sphere / skew-cyl / bspline×bspline, over-merging by `bspline × plane` **alone** (skew
cyl still reports 2 and passes under a fully-connected merge). `classifyBranch`'s try/catch now
covers `crossingSineOnOcct` — **latent, not a live crash**.

⚠ **"min sine 0.40" was wrong.** That is the *bspline × bspline* figure (0.4201). `bspline × plane`
bottoms out at **exactly 0.0000** — genuine tangencies.

**The remaining failure is the deliverable, and it stays red.** `bspline × plane` at recall 0.03 —
40 arcs over 32 nodes with **16 junctions**, one connected saddle network. Marching terminates at
junctions, so one seed cannot cover it, and **no currently-executing gate covers this case**. The
harness is expected to exit 1 until the S4-d branch-point iteration lands; it is on
`run-sim-suite.sh`'s SKIP list, so the suite does not redden.

⚠ Junction counts are **tolerance-dependent** — below ~3.7e-7 the same locus reads 64 nodes /
6 junctions / 24 components. Any statement about "16 junctions" must carry its tolerance.

Driver: `scripts/run-sim-native-ssi-seeding-parity.sh` — **not** `run-sim-native-ssi-seeding.sh`,
which builds a different harness (`native_ssi_seeding_recall.mm`) and will look deceptively green.

**gp_Dir gap — closed (Mac session 2026-07-19).** The iOS-simulator SDK's OCCT header
(`install-SIMULATORARM64/include/opencascade/gp_Dir.hxx`) contains exactly one `Magnitude`
occurrence, on the `gp_Vec` constructor: *"Raises ConstructionError if theV.Magnitude() <=
Resolution."* So on the simulator toolchain too, building a `gp_Dir` from the near-null
cross-of-normals at an exact tangency (`bspline × plane` bottoms out at sine 0.0000) throws
`Standard_ConstructionError` — the `classifyBranch` try/catch around `crossingSineOnOcct` is
required, not defensive decoration. Latent on both toolchains, live on neither.

---

## 4. M0 collar retune — **NOT READY. Its supporting claim is falsified; re-measure from scratch.**

**Do not land `kCollarFrac` 0.05 → 0.01 on the evidence below.** Two of the entry's three claims
failed fresh measurement:

| claim | measured |
|---|---|
| "8 verb rows improved / 0 degraded" | **FALSE** — all 12 asym verb rows **byte-identical** between fracs. Only `measure_multiseam_fine` triangle counts move (3930→3982, 6276→6372) |
| roadmap `L3-BAND nonmanif = 0/1/4` is measured-false | **CONFIRMED** — 0/0/0 at both fracs |
| the retune is a safe win | **UNRESOLVED** — see below |

**What the retune actually does**, on consistently-built binaries: it moves the COMMON decline
threshold below d=0.0018 (frac 0.05 declines `VolumeInconsistent`; frac 0.01 succeeds). That is a
working-band change, not an accuracy change.

⚠ **`seam_strip.h:399` records that the pile is suppressed only "for any frac ≳ 0.015".** The
proposed 0.01 is *below* that measured threshold, and at d=0.0008 both fracs decline
`NotWatertight` with **be = 17092**. Whether 0.01 trades a lower decline threshold for a worse
failure mode is the open question, and it is not answered.

⚠ **The order-dependence that appeared to invalidate this item DOES NOT EXIST** — see §1's note.
That retraction removes a blocker but supplies no evidence for the retune.

**Re-measure with `scripts/run-host-suite.sh`** (`518dd17`). Several measurements in the original
entry were taken from binaries whose headers and archive disagreed; they are void, not merely
stale. If the pins at `test_native_seam_strip_weld.cpp:111,122` and
`test_native_multiseam_asym.cpp:280,284` move, decide which outcome is CORRECT rather than
retuning to whatever the new code does — and **keep a decline pin** so the never-leaky property
stays covered.

---

## 5. Fuzzers — **done, no action**

`DISAGREED == 0` across **1 468 differential trials** plus 3 fresh seeds each. All `M6 BAR: PASS`;
`FALLBACK_ORACLE_INVALID = 0`, `OPERAND_MISMATCH = 0`. Per-seed output byte-identical
(splitmix64/xoshiro256**, no clock). The soundness invariant is intact **and now seed-swept**.
Budgets: boolean_fuzz ~14 min, freeform_boolean_fuzz ~9 min, ssi_freeform_fuzz ~21 min.

---

## 6. Simulator confirmation — ✅ **DONE (Mac session 2026-07-19 @ beafc55)**

All three runs green; all five changes confirmed and **archived**
(`openspec/changes/archive/2026-07-20-*`).

| run | result |
|---|---|
| `run-sim-native-ssi-marching.sh` | **22 passed / 0 failed.** The predicted benign shift landed exactly (bspline×bspline onCurve 1.65e-07 → 1.74e-07); every seed and branch count matched OCCT on all 22 lines — no SEED/BRANCH movement. |
| `run-sim-native-chain-seam-weld.sh` | **61 passed / 0 failed.** FUSE full OpCase green at d=0.010 and d=0.005 (vol rel 3.278e-03 / 1.824e-03, classify disagree=0). |
| full suite (acceptance bar) | **221 passed / 0 failed** — but see the script-staleness note below. |

**⚠ `run-sim-suite.sh` is stale in three ways and could not run as-is (Linux follow-up; the Mac
session's lane excluded `scripts/`).** The 221/221 above was obtained by reproducing the script's
compile/link/spawn exactly, with these corrections — each is a one-line script fix:

1. **numsci glob** (also in the marching/seam-weld runners): `build-numsci.sh iossim` writes
   `build-numsci/iossim/libnumsci_iossim_arm64.a`, but the fallback glob is
   `build-numsci/*iossim*.a` (parent dir) and misses it. Workaround used:
   `CYBERCAD_NUMSCI_DIR=$REPO/build-numsci/iossim` (the explicit-dir branch works).
2. **Missing `-lTKHLR`**: the GS1c HLR work put `hlr_project` into `occt_drafting.o`, so any
   freshly built `libcybercadkernel-SIMULATORARM64.a` needs TKHLR; the suite's `TKS` list predates
   it. (The committed archive was also stale — rebuilt with `build-xcframework.sh`, which is what
   exposed this.)
3. **SKIP list misses `native_vs_occt_bench.cpp` and `native_vs_occt_mem.cpp`** — both carry their
   own `main()` (duplicate-symbol link failure once TKHLR resolves). The suite proper is
   `full_suite.cpp` + the 7 Phase-0/1 `checks_*.cpp` modules, per the script's own comment.

Original queue entry follows for the record.

**Five changes are one task from done.** Each is complete except its simulator run; every other task
in them is checked. `openspec list` shows the counts.

| # | change | harness / script | covers |
|---|---|---|---|
| 1 | `moat-m1d-ssi-deep-tail` | `run-sim-native-ssi-marching` | 12/13 |
| 1 | `moat-m1e-ssi-wide-band` | *(same run)* | 16/17 |
| 1 | `moat-m1f-ssi-fit-conditioning` | *(same run)* | 16/17 |
| 1 | `moat-a2-ssi-slab-prune` | *(same run)* | 23/24 |
| 2 | `moat-m2c3f-strip-weld-fuse` | `run-sim-native-chain-seam-weld.sh` | 12/13 |
| 3 | acceptance bar | `run-sim-suite.sh` | 221/221 |

**Four of the five confirm through ONE harness run** (`run-sim-native-ssi-marching`). That is
efficient, and it is also the risk: if that harness moves, attribution across four changes needs a
bisect. Run it FIRST, before anything else lands.

**What benign movement looks like, so it is not chased.** The bspline×bspline line was predicted to
shift `onCurve 1.65e-07 → 1.74e-07` — 3–4 orders under tolerance, but not byte-identical. Expect
**more** lines to move than that: the slab prune changes candidate sets on TRANSVERSAL poses too,
not only disjoint ones. A line that moves while staying orders under tolerance is expected. **A
changed SEED or BRANCH count is not, and is the real signal.**

Also one-line while there: `grep Magnitude` in the simulator's `gp_Dir.hxx` to close the last gap
on §3.

**Two changes are NOT in this batch despite having a macOS task.** `moat-gs2-section-curves` (26
open) and `moat-m0-freeform-mesher` (22 open) have substantial work beyond the sim run — they are
not confirm-and-archive.

Every mechanism in this document is host-independent **by construction**, but that is *inference*,
which is exactly the kind of reasoning that failed repeatedly here. The simulator is a different
toolchain, a different OCCT build and a different architecture; it is the authority.

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
