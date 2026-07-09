# Proposal — moat-mem-memory-profile (MOAT drop-OCCT PAYOFF: runtime memory, measured)

## Why

The drop-OCCT campaign (NATIVE-REWRITE.md #8 `drop-occt`, MOAT-ROADMAP.md) has measured two
legs of the shipping case: **latency** (`docs/BENCH-native-vs-occt.md` §1) and **binary size**
(§2). The **third leg — runtime memory footprint — was unmeasured**, and it is the **tightest
constraint on iPad**: an interactive CAD app is far more likely to be jetsammed for memory than
to be slow or large on disk. A product decision ("is dropping OCCT worth finishing?") needs the
runtime-RAM number, not an assumption that native "should" use less.

This change adds a MEASUREMENT track: a benchmark harness + one runner + a findings doc that
report the native-vs-OCCT **runtime memory** payoff — per-operation peak RSS / footprint under
both engines, plus a one-shot whole-process peak-RSS comparison (which also captures OCCT's
static/global footprint). No product code changes.

## What changes

- **New memory harness `tests/sim/native_vs_occt_mem.cpp`** — drives boolean
  (fuse/cut/common), tessellate, mass_properties, section, and fillet_edges through the public
  `cc_*` facade under BOTH engines via `cc_set_engine(0|1)` (the latency-bench A/B pattern),
  with the SAME fixed deterministic N-gon inputs as the latency bench. Measures three
  complementary signals: per-op **peak RSS** (`getrusage.ru_maxrss`, one op per process for a
  clean high-water mark), per-op **footprint delta** (`task_info` `phys_footprint` at the op's
  live peak), and a whole-process **peak RSS** for a representative script. Records the SERVED
  engine honestly — a forwarded op gets no native number; an OCCT-declined op (section) is
  native-only.
- **New runner `scripts/bench-memory-native-vs-occt.sh`** — compiles the harness once (whole
  kernel + OCCT adapter + `-lTKHLR`, the same measurement-only link recipe as
  `bench-native-vs-occt.sh`), then invokes it **once per (op,size,engine)** for an
  uncontaminated per-op high-water mark and **once per engine** for the process-level number.
  Scrapes `[MEMROW]` / `[PROCROW]` lines into per-op and process tables.
- **New findings doc `docs/BENCH-memory-native-vs-occt.md`** — the methodology (exactly what is
  counted, and why malloc-zone stats are rejected in favour of `phys_footprint`/`ru_maxrss`),
  both tables, and the honest conclusion (where native uses less, where comparable, where
  native-only / forwarded).
- **Pointers added:** `docs/BENCH-native-vs-occt.md` (top table + companion link) and
  `openspec/MOAT-ROADMAP.md` (a "drop-OCCT payoff — memory (measured)" note).

## Impact

- **Measurement only** — no geometry code, no ABI change, no shipping-default change.
  `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI are byte-unchanged. The
  harness drives the existing facade under the existing `cc_set_engine` toggle.
- **Key finding:** doing the **same** representative script, native peaks at **35.3 MB vs
  OCCT's 46.8 MB — 1.33× / ~11.5 MB less** runtime RAM (reproducible; identical ~2.4 MB
  baseline, so the gap is working set + OCCT static/global data), and settles ~7 MB lighter.
  Per-op, native peak RSS is lower at essentially every op/size, with **tessellate** the
  standout (footprint 4–10× smaller). `section` is native-only; `fillet_edges` forwards to
  OCCT (no native number) — reported honestly.
- Completes the three-sided shipping case for `#8 drop-occt`: faster, smaller to ship, and
  lower runtime peak RSS on the axis that matters most on device.
