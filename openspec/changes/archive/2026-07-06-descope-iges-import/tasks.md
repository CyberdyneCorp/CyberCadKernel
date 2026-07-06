# Tasks — descope native IGES import

## 1. Spec
- [x] 1.1 MODIFY the `native-exchange` requirement "STEP import and IGES export/import stay OCCT" → IGES descoped + STEP import slice landed.
- [x] 1.2 MODIFY the `native-exchange` requirement "IGES import/export and the STEP writer stay unchanged" → IGES descoped (STEP-only), preserve the writer-unchanged scenario.
- [x] 1.3 `openspec validate descope-iges-import --strict` passes.

## 2. Roadmap / docs sync
- [x] 2.1 `openspec/NATIVE-REWRITE.md` — drop IGES from the drop-OCCT blocker list + effort table; note the STEP-only decision saves ~1.5–3 py.
- [x] 2.2 `openspec/ROADMAP.md` — IGES import removed from the still-OCCT / out-of-scope import lines.
- [x] 2.3 `openspec/SSI-ROADMAP.md` — the parallel-tracks note becomes STEP-only.
- [x] 2.4 `openspec/specs/native-numerics/spec.md` — the "blocks #8" note drops IGES.
- [x] 2.5 `README.md` + `docs/ROADMAP.md` + `docs/STATUS*.md` — where-OCCT-is-still-required narrowed: IGES import descoped, STEP import native (first slice).

## 3. Non-goals (explicit)
- [x] 3.1 The `cc_iges_export` / `cc_iges_import` ABI entries are NOT removed (additive-only). They stay OCCT-backed until `drop-occt`, then removed/stubbed — never reimplemented natively.
