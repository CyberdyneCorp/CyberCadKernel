# Drop-OCCT payoff — measured: native vs OCCT latency + binary size

This is the **quantified "why"** behind the drop-OCCT campaign (NATIVE-REWRITE.md #8
`drop-occt`, MOAT-ROADMAP.md): the measured performance and shipped-footprint payoff of
the clean-room native engine versus the OCCT oracle, on the SAME public `cc_*` operations.

Two measurements, both reproducible from this repo:

| # | measures | harness | runner |
|---|---|---|---|
| 1 | **latency** — native ms vs OCCT ms per op | `tests/sim/native_vs_occt_bench.cpp` | `scripts/bench-native-vs-occt.sh` |
| 2 | **binary size** — OCCT-linked vs native-only footprint | (build-driven) | `scripts/bench-binary-size.sh` |
| 3 | **runtime memory** — native peak RSS vs OCCT per op + whole-process | `tests/sim/native_vs_occt_mem.cpp` | `scripts/bench-memory-native-vs-occt.sh` |

> The **runtime-memory** third leg (peak RSS / footprint, native vs OCCT — the tightest iPad
> constraint) is measured in a companion doc:
> [docs/BENCH-memory-native-vs-occt.md](BENCH-memory-native-vs-occt.md). Headline: doing the
> **same** representative script, native peaks at **35.3 MB vs OCCT's 46.8 MB (1.33× / ~11.5 MB
> less)**; per-op, native peak RSS is lower at essentially every op, with **tessellate** the
> standout (footprint 4–10× smaller).

Product code (`src/native/**`, `src/engine/**`, `include/**`, the `cc_*` ABI) is
**byte-unchanged** by this track — it adds a measurement harness + two runners + this doc.

---

## 1. Performance — native vs OCCT per operation

### Method (deterministic, honest)

- **HOST timing** (macOS arm64, Apple Silicon) against **Homebrew OCCT 7.9.3**, NOT on
  device. Host timing is chosen deliberately: a booted iOS simulator adds scheduler /
  thermal noise that swamps a clean engine-to-engine comparison. **Absolute device latency
  differs** — the portable, honest signal is the **ratio** (native/OCCT) on the same CPU,
  same inputs, same process. Every row is host.
- Each op is driven through the **same public `cc_*` call the app calls**, once under
  `cc_set_engine(0)` (OCCT oracle) and once under `cc_set_engine(1)` (NativeEngine) — the
  pattern of `native_boolean_parity.mm` / `native_transformed_boolean_fuzz.mm`.
- **Median of N=25 measured iterations**, after **3 warm-up iterations discarded**; min/max
  captured so run-to-run variance is visible. `std::chrono::steady_clock`, `-O2`. Fixed
  deterministic inputs (regular N-gon prisms) — **no `rand()`, no wall-clock seeding**.
- Model-size spread: `small` = octagonal prism (8 sides), `medium` = 32-gon, `large` =
  96-gon — so the ratio is not a single-point claim.
- **The engine that actually serves each op is recorded honestly.** A native op that the
  NativeEngine forwards to OCCT (curved / out-of-domain) is labelled `forwarded` and given
  **no** native time (it is not a native win). An op the OCCT adapter does not implement is
  labelled `native-only` (see the section caveat).

### Results (median ms; ratio = OCCT / native, >1 ⇒ native faster)

| op | size | OCCT ms | native ms | ratio / note |
|---|---|---:|---:|---:|
| boolean_fuse    | small  |  16.94 |  1.19 | **14.2×** |
| boolean_cut     | small  |  14.54 |  0.88 | **16.6×** |
| boolean_common  | small  |  13.76 |  0.68 | **20.3×** |
| boolean_fuse    | medium |  60.26 |  5.87 | **10.3×** |
| boolean_cut     | medium |  52.35 |  4.38 | **12.0×** |
| boolean_common  | medium |  50.01 |  3.37 | **14.8×** |
| boolean_fuse    | large  | 200.29 | 23.97 | **8.4×**  |
| boolean_cut     | large  | 160.78 | 18.31 | **8.8×**  |
| boolean_common  | large  | 142.10 | 13.59 | **10.5×** |
| tessellate      | small  |   0.60 | 0.041 | **14.9×** |
| tessellate      | medium |   2.24 | 0.169 | **13.3×** |
| tessellate      | large  |   6.01 | 0.568 | **10.6×** |
| mass_properties | small  |  0.369 | 0.044 | **8.5×**  |
| mass_properties | medium |  1.403 | 0.178 | **7.9×**  |
| mass_properties | large  |  4.279 | 0.593 | **7.2×**  |
| section         | small  | DECLINED | 0.0044 | native-only¹ |
| section         | medium | DECLINED | 0.0167 | native-only¹ |
| section         | large  | DECLINED | 0.0612 | native-only¹ |
| fillet_edges    | box    |  2.16 | FORWARDED | fwd→OCCT² |

Values are the median of a representative run (`scripts/bench-native-vs-occt.sh`); the
ratios are stable to ~±5 % run-to-run (two runs agreed within that band — e.g.
boolean_fuse/small 14.8× then 14.2×). The OCCT columns for boolean show occasional high
`max` outliers (GC / allocator), which is exactly why the **median** is reported.

### Honest caveats

1. **`section` is a NATIVE-ONLY capability, not a like-for-like speedup.** The OCCT engine
   *adapter in this facade* does not implement `cc_section_plane` — it returns an honest
   decline (`operation not supported by active engine: section_plane`), so its timing would
   be a no-op, not a real section. The harness detects that decline at runtime and reports
   the OCCT side as `DECLINED`; the native side is the only engine that computes the section
   (verified: octagon r=10 cut at z=5 → 1 closed loop, length ≈ 61.23, area ≈ 282.84). We
   therefore do **not** print a ratio for section — it is a native capability OCCT does not
   offer here, worth far more than a speedup.
2. **`fillet_edges` is FORWARDED, not native.** A rolling-ball edge fillet produces a curved
   face, which is outside the native planar-BSP domain, so the NativeEngine forwards the
   whole op to OCCT. Timing it "under native" would just re-time OCCT, so the native cell is
   `FORWARDED` and no native time is claimed. This is a **decline**, honestly reported: the
   native engine does not accelerate edge fillets — it hands them to OCCT (and after
   drop-OCCT, this op becomes a clean decline per DROP-OCCT-READINESS.md).
3. All boolean rows are **native-served AND exact**: the native BSP-CSG fuse of two
   overlapping octagonal prisms yields the identical volume (4897.971) as the OCCT oracle,
   self-verified watertight + set-algebra volume before acceptance. The speedup is on
   genuinely correct results, not a cheaper-but-wrong path.

### Where native wins / is comparable / declines

- **Wins large** (8–20×): the native planar-polyhedron **boolean** across every size, and
  the native **tessellate** / **mass_properties** on native bodies. These are the app's hot
  interactive-modelling ops, and they are the bulk of what the app runs. The native win is
  biggest on small/medium models (the interactive regime) and stays ≥7× even on the 96-gon
  large case.
- **Native-only**: **section** — OCCT-adapter declines it; native serves it in ~µs–tens-of-µs.
- **Declines / forwards**: **fillet_edges** (curved fillet face → OCCT today, clean decline
  after unlink). This matches the DROP-OCCT-READINESS.md Class-B classification.

---

## 2. Binary size / footprint — OCCT-linked vs native-only

### Method

Two `libcybercadkernel.a` are compiled for the **iOS-simulator arm64 shipping slice** from
the same `src/**` TU set (facade + core + engine + `src/native/**` + the `numsci` numeric
facade), differing **only** by the OCCT adapter:

- **OCCT build**: `-DCYBERCAD_HAS_OCCT=1` — compiles `src/engine/occt/*` (16 TUs) and links
  the OCCT static toolkits.
- **native-only build**: `-DCYBERCAD_M8_REHEARSAL=1` — no OCCT TUs; NativeEngine is the
  default engine (the exact post-`drop-occt` wiring, per `moat-m8dry-unlink-rehearsal`).

Sizes are a property of the compiled objects (not timed → deterministic). The OCCT
`.a` footprint is reported three honest ways: the **linked-toolkit subset** (what the app
actually pulls in — 20 toolkits incl. TKHLR for the full adapter), the **full install**
(upper bound; the app links a subset), and a **dead-stripped final-link** delta (the truest
"what actually ships in the binary", since `-dead_strip` keeps only reachable OCCT code).

### Results (iossim arm64)

| component | MB |
|---|---:|
| `libcybercadkernel.a` — OCCT build | 3.74 |
| `libcybercadkernel.a` — native-only build | 2.66 |
| **kernel `.a` delta (the OCCT adapter code)** | **1.08** |
| OCCT `.a` linked-toolkit subset (statically archived, would be DROPPED) | **112.15** |
| OCCT `.a` full trimmed install (upper bound) | 140.78 |

| dead-stripped final-link (representative reachable set) | MB |
|---|---:|
| exe linked with OCCT kernel + OCCT toolkits | 29.76 |
| exe linked with native-only kernel | 1.71 |
| **dead-stripped link delta (OCCT code that actually ships)** | **28.05** |

- **OCCT adapter TUs eliminated:** 16 (`src/engine/occt/*.cpp`).
- **OCCT-side exported symbols in the OCCT kernel `.a`:** 259 (`nm` `OcctEngine`/`occt`).
- **OCCT archive members (`.o`) in the linked toolkit subset:** 4,500 across 20 toolkits.

### Reading the size numbers honestly

- **The static-archive subset (112.15 MB) over-counts what lands in the app binary.** A
  static archive carries every object; the final linker with `-dead_strip` keeps only
  reachable code. The **dead-stripped link delta of 28.05 MB** (OCCT exe 29.76 MB vs
  native-only 1.71 MB, for a representative reachable set: construct + boolean + tessellate +
  mass + section + fillet + STEP + query) is therefore the honest **in-binary** shipping
  number — much smaller than 112 MB, but the code that genuinely ships. A wider reachable set
  (more STEP/IGES/healing surface) pulls in more OCCT, so 28 MB is a lower bound for a real
  app. The 112 MB / 140 MB figures are the **on-disk dependency footprint you no longer
  build, vendor, ship, or code-sign** — a real supply-chain / build-time win on top of the
  in-binary saving.
- Either way the direction is unambiguous: OCCT is **hundreds of MB** of static libraries
  (140 MB trimmed; the untrimmed upstream is far larger), of which **~28 MB ships in the
  binary** for even a modest reachable set, replaced by a **~1 MB** kernel-side native-adapter
  delta. **This is the iPad shipping win**: the app stops carrying OCCT entirely.

---

## Conclusion — the drop-OCCT payoff

- **Performance:** on the ops the app runs most (boolean, tessellate, mass properties) the
  native engine is **7–20× faster** than OCCT on identical, self-verified-correct results,
  with the largest wins in the small/medium interactive regime. **section** is native-only
  (OCCT-adapter declines it). The only op that does not benefit is **fillet_edges**, which
  native forwards to OCCT today and will cleanly decline after unlink — an honest limit, not
  a regression.
- **Size:** dropping OCCT removes **112.15 MB** of statically-archived OCCT dependency
  (140.78 MB full trimmed install) plus a **1.08 MB** kernel-side adapter, in exchange for a
  ~2.66 MB native-only kernel. The **dead-stripped final-link delta is 28.05 MB** — the true
  in-binary shipping reduction for a representative reachable set (and a lower bound for a
  fuller app).
- **Net:** the native engine is both **materially faster** on the hot path **and**
  **tens-to-hundreds of MB lighter** to ship — the two-sided justification for `#8 drop-occt`. The
  remaining OCCT-only ops (fillet, and the Class-B/C tail in DROP-OCCT-READINESS.md) are the
  scope that must go native before the unlink, not blockers to the payoff being real.

### Reproduce

```
scripts/build-numsci.sh host       # OCCT-free numeric substrate (host)
scripts/build-numsci.sh iossim     # …and the iOS-sim slice (size measurement)
scripts/bench-native-vs-occt.sh    # latency table (host, Homebrew OCCT)
scripts/bench-binary-size.sh       # size table (iossim arm64)
```
