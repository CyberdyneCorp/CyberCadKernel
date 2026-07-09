# Drop-OCCT payoff — measured: runtime MEMORY footprint, native vs OCCT

This is the **third leg** of the quantified "why" behind the drop-OCCT campaign
(NATIVE-REWRITE.md #8 `drop-occt`, MOAT-ROADMAP.md). The latency payoff
([docs/BENCH-native-vs-occt.md](BENCH-native-vs-occt.md) §1) and the binary-size payoff
(§2) are already measured; **runtime RAM is the tightest constraint on iPad and was the
missing measurement**. This doc measures the runtime memory footprint of the clean-room
native engine versus the OCCT oracle, on the SAME public `cc_*` operations.

| # | measures | harness | runner |
|---|---|---|---|
| 3 | **runtime memory** — native peak RSS / footprint vs OCCT, per op + whole-process | `tests/sim/native_vs_occt_mem.cpp` | `scripts/bench-memory-native-vs-occt.sh` |

Product code (`src/native/**`, `src/engine/**`, `include/**`, the `cc_*` ABI) is
**byte-unchanged** by this track — it adds a measurement harness + one runner + this doc.

---

## Method (honest — what exactly is counted)

- **HOST measurement** (macOS arm64, Apple Silicon) against **Homebrew OCCT 7.9.3**, NOT on
  device. Host is chosen for a stable allocator and no simulator scheduler/thermal noise. As
  with the latency bench, **absolute device RAM differs**; the portable, honest signals are
  the **ratio** (OCCT/native) and the **process-level delta**. iOS uses the same `malloc`
  family, so the direction carries; the absolute megabytes do not.
- Each op is driven through the **same public `cc_*` call the app calls**, once under
  `cc_set_engine(0)` (OCCT oracle) and once under `cc_set_engine(1)` (NativeEngine) — the
  A/B pattern of the latency bench. **Same fixed inputs** (regular N-gon prisms:
  `small`=octagon, `medium`=32-gon, `large`=96-gon) as the latency bench, for comparability.
  Deterministic — **no `rand()`, no wall-clock seeding**.
- **Three complementary memory signals are measured, each labelled:**
  1. **Per-op PEAK RSS** — `getrusage(RUSAGE_SELF).ru_maxrss`. This is a per-process
     **high-water mark that never falls**, so it is only meaningful **measured once per
     process**. The runner therefore invokes the harness **one (op,size,engine) per process
     invocation** (`--single`): each process does a bounded warm-up build then the measured
     op, so `ru_maxrss` at exit is a **clean peak attributable to that engine doing that op**,
     with no high-water contamination from a previous op. **This is the primary, robust,
     reproducible signal** (±0.1–0.3 MB native, ±0.5 MB OCCT across repeated runs).
  2. **Per-op FOOTPRINT DELTA** — `task_info(TASK_VM_INFO).phys_footprint` sampled just
     before the op and at its **live peak** (result still alive), reported as a KB delta. It
     subtracts the fixed baseline (process + static OCCT data) to isolate the op's own
     working set. **It is a supporting signal only** — footprint is quantized to VM pages and
     an op that allocates *and frees* transient memory *within* the sample window reads a
     smaller (sometimes zero) delta, so per-op deltas are **noisy run-to-run**; the peak-RSS
     column is the signal to trust for the absolute picture.
  3. **Process-level PEAK RSS** — the whole representative script (construct + all three
     booleans + tessellate + mass + section, over all three sizes) run once per engine in a
     fresh process; `ru_maxrss` is the process high-water mark. This captures each engine's
     **full working set plus (for OCCT) its static/global footprint** faulted in — the
     absolute-picture complement to the binary-size doc.
- **`phys_footprint` is what iOS charges the process** (the value Xcode's memory gauge and
  the OS memory-pressure system use); `ru_maxrss` is peak resident pages. macOS large
  allocations are `mmap`-backed and do **not** appear in `malloc_zone_statistics`
  `size_in_use` (verified empirically), so a malloc-zone delta would **under-count** geometry
  working sets — `phys_footprint`/`ru_maxrss` count them and are used instead.
- **The engine that actually SERVES each op is recorded honestly** (same rule as the latency
  bench): a native op the NativeEngine **forwards** to OCCT gets **no** native number
  (labelled `FWD`); an op the OCCT adapter does not implement (`section`) is labelled
  `native-only` and the OCCT side `DECLINED`.

---

## Results

### Per-op memory (host; peak RSS is the primary signal; footprint Δ is supporting)

`RSS MB` is the per-process peak resident high-water mark (one op per process). `Δ KB` is the
per-op footprint delta (supporting, noisy — see method). `ratio = OCCT/native footprint Δ`
where both are native-served.

| op | size | OCCT peak RSS MB | native peak RSS MB | OCCT Δ KB | native Δ KB | Δ ratio / note |
|---|---|---:|---:|---:|---:|---:|
| boolean_fuse    | small  | 22.95 | **18.22** |  304 |  352 | 0.86× |
| boolean_fuse    | medium | 25.98 | **22.73** |  448 |  560 | 0.80× |
| boolean_fuse    | large  | 34.86 | **34.27** | 1216 | 1168 | 1.04× |
| boolean_cut     | small  | 22.48 | **17.94** |  192 |  336 | 0.57× |
| boolean_cut     | medium | 25.05 | **21.31** |  432 |  560 | 0.77× |
| boolean_cut     | large  | 30.17 | **30.75** |  560 | 1632 | 0.34× |
| boolean_common  | small  | 22.47 | **17.31** |  240 |  112 | 2.14× |
| boolean_common  | medium | 24.86 | **19.81** |  416 |  512 | 0.81× |
| boolean_common  | large  | 29.45 | **26.53** |  592 | 1184 | 0.50× |
| tessellate      | small  | 22.14 | **16.22** |  384 |   96 | **4.0×** |
| tessellate      | medium | 26.58 | **16.72** | 1168 |  112 | **10.4×** |
| tessellate      | large  | 34.83 | **17.84** | 1632 |  272 | **6.0×** |
| mass_properties | small  | 18.23 | **16.30** |    0¹ |  128 | (¹) |
| mass_properties | medium | 18.97 | **16.75** |    0¹ |  160 | (¹) |
| mass_properties | large  | 20.59 | **17.88** |    0¹ |  208 | (¹) |
| section         | small  | DECLINED² | **18.53** | DECL | 0 | native-only² |
| section         | medium | DECLINED² | **19.67** | DECL | 48 | native-only² |
| section         | large  | DECLINED² | **21.95** | DECL | 96 | native-only² |
| fillet_edges    | box    | 20.20 | FORWARDED³ | 16 | FWD | fwd→OCCT³ |

Values are one representative run of `scripts/bench-memory-native-vs-occt.sh`. **Peak RSS is
reproducible** to ±0.1–0.3 MB (native) / ±0.5 MB (OCCT) across runs; the footprint **Δ is
noisy** (e.g. OCCT tessellate-large Δ was 1552/5072/4528 KB across three runs) — which is
exactly why peak RSS is the headline and Δ is labelled supporting.

### Honest caveats

1. **`mass_properties` OCCT footprint Δ reads 0 — a measurement-floor artifact, not "OCCT
   uses no memory".** OCCT's `BRepGProp` allocates and frees its transient accumulators
   *within* the before/peak sample window, so the net footprint delta rounds below one VM
   page. The **peak-RSS** column shows the real picture: OCCT 18.2–20.6 MB vs native
   16.3–17.9 MB — native lower at every size. Read the Δ=0 as "no *retained* growth
   measurable at page granularity", and lean on peak RSS.
2. **`section` is a NATIVE-ONLY capability, not a like-for-like comparison.** The OCCT engine
   *adapter in this facade* does not implement `cc_section_plane` — it honestly declines, so
   there is no OCCT memory figure to compare. The harness detects the decline at runtime and
   reports `DECLINED`; the native side is the only engine that computes the section. No ratio
   is printed — a native capability OCCT does not offer here (see the latency bench §1
   caveat 1).
3. **`fillet_edges` is FORWARDED, not native.** A rolling-ball edge fillet is a curved face,
   outside the native planar-BSP domain, so the NativeEngine forwards the whole op to OCCT.
   Measuring it "under native" would just re-measure OCCT, so the native cell is `FORWARDED`
   and no native number is claimed (after `drop-occt` this op becomes a clean decline per
   DROP-OCCT-READINESS.md).
4. **Booleans: comparable working set, lower peak.** The native BSP-CSG boolean and OCCT's
   BOPAlgo have **comparable per-op footprint deltas** (native's transient BSP node trees vs
   OCCT's transient B-rep) — the Δ ratio hovers around 0.3–2× with no consistent winner, and
   at `large` the two converge. But native's **absolute peak RSS is lower** at almost every
   size (the fixed OCCT baseline is heavier). This is reported honestly: the boolean memory
   win is in the **process baseline / peak**, not in a dramatically smaller per-op allocation.
   All boolean rows are **native-served AND exact** (the same self-verified-correct results as
   the latency bench — identical fuse volume to the OCCT oracle).

### Process-level peak RSS (whole representative script, same inputs)

The single most honest absolute number: the **same** script (construct + fuse/cut/common +
tessellate + mass + section, all three sizes) run once per engine in a fresh process.

| engine | peak RSS MB | baseline footprint MB | end footprint MB |
|---|---:|---:|---:|
| OCCT   | **46.81** | 2.45 | 28.80 |
| native | **35.28** | 2.44 | 21.70 |

**Process peak-RSS ratio (OCCT / native) = 1.33× — native uses ~11.5 MB less** doing the
identical work. Reproducible across three runs (OCCT 45.6–47.3 MB, native 35.0–35.2 MB). The
baseline footprints are identical (~2.4 MB, before either engine touches its data), so the
gap is entirely the two engines' **working set + static/global data**: OCCT faults in large
static tables and heavier transient B-rep structures; the native engine's planar-BSP working
set is lighter. The **end footprint** (after the script, retained) is likewise lower for
native (21.7 vs 28.8 MB), i.e. native both peaks lower and settles lower.

---

## Conclusion — the memory payoff

- **Runtime RAM is a real native win, on the tightest iPad axis.** Per operation, the native
  engine's **peak RSS is lower than OCCT's at essentially every op and size** — most sharply
  for **tessellate** (native ~16–18 MB flat vs OCCT 22–35 MB rising with model size; footprint
  Δ **4–10× smaller**), and for the interactive **boolean** / **mass_properties** ops (native
  16–27 MB vs OCCT 18–35 MB). **tessellate is the standout**: the native mesher's working set
  barely grows with model size, while OCCT's climbs to ~35 MB on the 96-gon.
- **Process-level, doing the same script, native peaks at 35.3 MB vs OCCT's 46.8 MB — a
  1.33× / ~11.5 MB reduction**, reproducibly, and settles ~7 MB lighter. That is headroom the
  iPad app gets back for free by dropping OCCT, on top of the ~28 MB in-binary size saving.
- **Where it is comparable / not a win:** the **per-op boolean footprint delta** is comparable
  between engines (no dramatic per-allocation win — the boolean memory benefit is in the lower
  process baseline/peak, not a smaller transient); **section** is native-only (OCCT-adapter
  declines it); **fillet_edges** forwards to OCCT (no native number — a clean decline after
  unlink). All stated honestly, none dressed up as a win.
- **Net (with the latency + size docs):** dropping OCCT is a three-sided win — **7–20× faster**
  on the hot path, **tens-to-hundreds of MB lighter to ship**, and now **~1.3× / ~11.5 MB
  lower runtime peak RSS** on the same work, with tessellate showing the biggest per-op memory
  reduction. The memory result closes the shipping case on the axis that matters most on
  device.

### Reproduce

```
scripts/build-numsci.sh host              # OCCT-free numeric substrate (host)
scripts/build-numsci.sh iossim            # (size doc; kept green here too)
scripts/bench-memory-native-vs-occt.sh    # memory table (host, Homebrew OCCT)
```

The runner compiles the harness once (whole kernel + OCCT adapter + `-lTKHLR`, the same
measurement-only link as `bench-native-vs-occt.sh`), then invokes it once per
(op,size,engine) for a clean per-op high-water mark, and once per engine for the
process-level number. `[MEMROW]` / `[PROCROW]` lines are machine-readable.
