# Proposal — moat-sheet-metal (MOAT M-SM, first slice)

## Why

Sheet metal is a SolidWorks-class capability the CyberCad app needs and no numbered
MOAT stage covers. It is a large feature; this change lands a **bounded first slice**
and HONEST-DECLINES the rest, so the app gets the manufacturing-critical primitives —
a base flange, one edge flange with a bend, and the flat-pattern unfold — without
faking the multi-bend / miter / corner-relief surface.

There is a keystone honesty point: **OCCT core has NO sheet-metal module.** So, unlike
every other native op, there is NO OCCT oracle and NO OCCT fall-through. The ARBITER is
CLOSED FORM: a bent constant-thickness part's volume is the flat-blank volume plus the
bend-allowance (k-factor) correction, and the unfold round-trips (fold → unfold → the
flat pattern's developed area is invariant). A case the native builder cannot robustly
build is a clean `cc_last_error` decline — never a wrong or self-intersecting solid,
never a widened tolerance, never an OCCT forward.

## What Changes

1. **A NEW native sheet-metal module** `src/native/sheetmetal/` (header-only, OCCT-free,
   namespace `cybercad::native::sheetmetal`): `common.h` (decline enum, tolerances, the
   closed-form self-verify, the `FoldRecord`), `base_flange.h`, `edge_flange.h`,
   `unfold.h`, and the umbrella `sheetmetal.h`. It CONSUMES the landed
   `construct::build_prism`, the topology `ShapeBuilder`/`Explorer`, `math` elementary
   geometry, and the tessellate mesh audit. It touches no landed header.

2. **Three ADDITIVE `cc_*` ops** in `include/cybercadkernel/cc_kernel.h`:
   - `cc_sheet_base_flange(profileXY, pointCount, thickness)` — the flat sheet solid.
   - `cc_sheet_edge_flange(body, edgeId, height, bendRadius, angleDeg)` — one flange off
     one straight rim: a true partial-cylinder bend + a planar flange wall.
   - `cc_sheet_unfold(body, kFactor)` — the flat-pattern developed blank.
   Existing signatures are unchanged (additive-only).

3. **Facade + engine wiring**: three facade wrappers in `src/facade/cc_kernel.cpp`, three
   `IEngine` virtuals (default `engine_unsupported`), and their `NativeEngine` overrides
   in `src/engine/native/native_engine.{h,cpp}`. The `NativeShape` holder gains an
   additive, process-internal `FoldRecord` so `cc_sheet_unfold(body, kFactor)` develops
   the recorded fold EXACTLY (no fragile mesh reverse-engineering). Sheet metal is
   native-only: an OCCT/mesh body or an unbuildable case returns a clean error — never an
   OCCT forward (OCCT has no sheet-metal op).

4. **Tests**: host Gate (a) `tests/native/test_native_sheetmetal.cpp` (CTest
   `test_native_sheetmetal`, 9 cases) + sim Gate (b)
   `tests/sim/native_sheetmetal_selftest.mm` with its own `main()`, runner
   `scripts/run-sim-native-sheetmetal.sh`, and a SKIP entry in `run-sim-suite.sh`.

5. **Docs**: a new `M-SM` stage in `openspec/MOAT-ROADMAP.md`.

## Capabilities

- **NEW `sheet-metal`** — constant-thickness base flange, single-bend edge flange, and
  single-bend flat-pattern unfold, native-only, closed-form-verified.

## Impact

- `include/cybercadkernel/cc_kernel.h` — +3 additive declarations (no signature change).
- `src/facade/cc_kernel.cpp` — +3 guard wrappers.
- `src/engine/IEngine.h` — +3 virtuals (default unsupported).
- `src/engine/native/native_engine.h` / `.cpp` — +3 overrides, `FoldRecord` on
  `NativeShape`, the sheet-metal include and decline-text helper.
- `src/native/sheetmetal/**` — NEW module (5 headers).
- `tests/native/test_native_sheetmetal.cpp`, `tests/sim/native_sheetmetal_selftest.mm`,
  `scripts/run-sim-native-sheetmetal.sh`, `CMakeLists.txt` (test registration),
  `scripts/run-sim-suite.sh` (SKIP entry) — NEW / edited.
- `openspec/MOAT-ROADMAP.md` — new M-SM stage.
- The tessellator, boolean, construct, blend, analysis, exchange modules are UNTOUCHED.
  `git diff src/native` is OCCT-free and additive.

## Out of scope (honest-declined, sharpened next blocker)

Multi-bend interference / more than one flange, the miter between adjacent flanges, and
corner-relief cuts; a non-straight bend line; a non-recognised (non-rectangular /
freeform) base; a self-colliding flange; degenerate parameters. The sharpened next
blocker is **multi-bend** (bend-bend interference + miter + corner relief), which needs
a boolean-fused multi-region weld and a relief solver.
