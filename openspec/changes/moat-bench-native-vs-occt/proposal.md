# Proposal — moat-bench-native-vs-occt (MOAT drop-OCCT PAYOFF, measured)

## Why

The drop-OCCT campaign (NATIVE-REWRITE.md #8 `drop-occt`, MOAT-ROADMAP.md) has proven the
native engine correct op-by-op against the OCCT oracle, and `moat-m8dry-unlink-rehearsal`
proved the post-unlink build links cleanly and declines honestly. What was still missing is
the **quantified "why"** — the concrete performance and shipped-footprint payoff of dropping
OCCT for the iPad CAD app. Product / roadmap decisions ("is the moat worth finishing?") need
real numbers, not an assertion that native "should be faster and smaller".

This change adds a MEASUREMENT track: a benchmark harness + two runners + a findings doc that
report (1) native-vs-OCCT **latency** per representative `cc_*` op across model sizes, and
(2) the OCCT-linked vs native-only **binary size** payoff. No product code changes.

## What changes

- **New latency harness `tests/sim/native_vs_occt_bench.cpp`** — drives boolean
  (fuse/cut/common), tessellate, mass_properties, section, and fillet_edges through the
  public `cc_*` facade under BOTH engines via `cc_set_engine(0|1)` (the
  `native_boolean_parity` pattern), timing each with median-of-25 (3 warm-up discarded),
  fixed deterministic inputs, `steady_clock`, `-O2`. It records the SERVED engine honestly:
  a forwarded op gets no native time; an OCCT-declined op (section) is labelled native-only.
- **New runner `scripts/bench-native-vs-occt.sh`** — builds + runs the latency harness on the
  HOST (macOS arm64) against Homebrew OCCT (deterministic CPU; the native/OCCT RATIO is the
  portable signal). Links the whole kernel + OCCT adapter, exactly like the macOS desktop
  dylib recipe plus `-lTKHLR` (needed by `occt_drafting.cpp`; measurement-only link).
- **New runner `scripts/bench-binary-size.sh`** — compiles `libcybercadkernel.a` for the
  iossim arm64 slice two ways (OCCT `-DCYBERCAD_HAS_OCCT` vs native-only
  `-DCYBERCAD_M8_REHEARSAL`), and reports the kernel `.a` delta, the OCCT static-toolkit
  footprint (linked subset + full install), a dead-stripped final-link delta (the honest
  in-binary number), and the OCCT TUs/symbols eliminated.
- **New findings doc `docs/BENCH-native-vs-occt.md`** — the methodology + both tables + the
  honest conclusion (where native wins, where it is native-only, where it forwards/declines).
- **`MOAT-ROADMAP.md`** gets a short "drop-OCCT payoff (measured)" note referencing the
  numbers.

## Impact

- **Measurement only** — no geometry code, no ABI change, no shipping-default change.
  `src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI are byte-unchanged.
  The harness drives the existing facade under the existing `cc_set_engine` toggle.
- **Key performance finding:** on the hot interactive path the native engine is **7–20×**
  faster than OCCT on identical, self-verified-correct results — boolean 8–20×, tessellate
  11–15×, mass_properties 7–8× (host, ratio; biggest wins on small/medium models). `section`
  is native-only (the OCCT adapter declines it). `fillet_edges` is forwarded to OCCT (no
  native win; a clean decline after unlink) — reported honestly, not as a speedup.
- **Key size finding:** dropping OCCT removes **112 MB** of statically-archived OCCT toolkits
  (140 MB full trimmed install) + a **1.08 MB** kernel-side adapter (16 TUs, 259 symbols),
  for a **28 MB dead-stripped in-binary reduction** on a representative reachable set. This is
  the iPad shipping win.
