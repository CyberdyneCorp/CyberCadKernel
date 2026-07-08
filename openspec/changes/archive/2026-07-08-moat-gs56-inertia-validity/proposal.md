# Proposal — moat-gs56-inertia-validity (MOAT M-GS · GS5 + GS6)

## Why

Two low-risk app-facing kernel services still route to OCCT, and both are reachable
today only as an OCCT fallback:

- **GS5 — inertia / principal moments.** The native mass-properties path
  (volume / area / centroid) is native, but `NativeEngine::principal_moments` is
  `CC_NATIVE_BODY_UNSUPPORTED → OCCT` (`src/engine/native/native_engine.cpp:1576`).
  The app's `Inertia` / `MassReadout` features therefore have no native inertia
  tensor and depend on OCCT `GProp_PrincipalProps`. This is the single hole the M6
  mass-properties differential fuzzer surfaced. The fix reuses **landed** machinery:
  the M0 triangulation already computes the enclosed volume by a **signed-tetra sum**
  (`tessellate/mesh.h::enclosedVolume` — ⅙ Σ aᵢ·(bᵢ×cᵢ), the divergence theorem),
  so the *second* moments of the same tetra fan give the full inertia tensor about
  the centroid; a symmetric-3×3 eigendecomposition yields the principal moments and
  axes. No new geometry is required — only the second-moment accumulation and a
  Jacobi eigensolver.

- **GS6 — B-rep validity checking.** The app's `occt-usage` spec depends on OCCT
  `BRepCheck_Analyzer` (closedness / orientation / self-intersection / degeneracy)
  to validate imports and operation results. The native kernel has a *per-operation*
  watertight+volume **self-verify** (`robustlyWatertight`), but no **general
  standalone validity checker** the app can call on an arbitrary body. A native
  `cc_check_solid` reuses the tessellator (closed 2-manifold via
  `mesh.h::isWatertight` / `edgeUseCounts`), the topology (finite coords, degenerate
  faces/edges), and GS3 `distance.h` (triangle-pair self-intersection) to produce a
  structured validity report.

Both slices are **bounded and additive** over landed code — the expected outcome is a
clean land at the two-gate acceptance bar. Neither adds any OCCT to `src/native/**`;
OCCT stays the ORACLE only (the sim verification harness).

## What Changes

1. **GS5 native inertia** — a header-only, OCCT-free
   `src/native/analysis/inertia.h` that, from a watertight M0 `Mesh`, accumulates the
   signed-tetra second moments about the origin, shifts to the centroid (parallel-axis),
   forms the symmetric inertia tensor `I = tr(C)·Id − C`, and returns the principal
   moments `I₁ ≤ I₂ ≤ I₃` (unit density → volume inertia, matching OCCT's convention)
   plus their orthonormal principal axes via a symmetric-3×3 **Jacobi** eigensolver.
   `NativeEngine::principal_moments` is rewired from the `CC_NATIVE_BODY_UNSUPPORTED`
   stub to this native path **guarded by a watertight precondition** (`robustlyWatertight`):
   a non-watertight body cannot yield a meaningful inertia, so the engine **declines →
   OCCT** rather than emit a wrong tensor.

2. **GS6 native validity checker** — a header-only, OCCT-free
   `src/native/analysis/validity.h` and an **additive** `cc_check_solid(body,
   CCValidityReport*)` facade. The report carries independent verdicts for: finite
   coordinates; closed 2-manifold; consistent outward orientation; no degenerate
   (zero-area) face or zero-length edge; and no self-intersection — with an overall
   `valid` flag and a `first_failure` code. Each check reuses landed machinery
   (tessellator + topology + GS3 distance). Where a check is **not robustly reachable**
   (e.g. certifying no-self-intersection on a general freeform patch), the facade
   returns an **honest decline** (`decided = 0`, `cc_last_error` set) — NEVER a false
   `valid`.

3. **ABI: additive only.** GS5 adds **no** new facade symbol — it rewires the existing
   `cc_principal_moments` (`out3 = [I₁,I₂,I₃]`) to the native engine; the principal
   axes are returned by the native service and checked in the sim harness only. GS6
   adds exactly one new POD struct (`CCValidityReport`) and one new prototype
   (`cc_check_solid`); every pre-existing `cc_*` signature and struct stays
   byte-for-byte unchanged.

## Capabilities

### Modified Capabilities

- `native-analysis`: ADDS a native **inertia tensor + principal moments / axes**
  service (signed-tetra second moments over the M0 triangulation, then symmetric-3×3
  eigen) wired behind `cc_principal_moments` with a watertight precondition and honest
  decline; and ADDS a native **standalone B-rep validity checker** behind an additive
  `cc_check_solid` facade, reusing the tessellator + topology + GS3 distance, with an
  honest decline where a robust check (freeform self-intersection) is unreachable.

## Impact

- `src/native/analysis/inertia.h` — NEW, OCCT-free header-only: signed-tetra second
  moments, centroid shift, `I = tr(C)·Id − C`, symmetric-3×3 Jacobi eigensolver →
  `{principal moments, axes}`.
- `src/native/analysis/validity.h` — NEW, OCCT-free header-only: finite / closed-
  manifold / orientation / degenerate / self-intersection checks → structured report;
  honest decline on unreachable self-intersection certification.
- `src/native/analysis/native_analysis.h` — includes the two new headers; new POD
  result structs (`Inertia`, `ValidityReport`). Existing types unchanged.
- `src/engine/native/native_engine.cpp` — `principal_moments` rewired stub → native
  (watertight-guarded, decline → OCCT); NEW `check_solid` native method.
- `src/engine/IEngine.h` / `src/engine/occt/occt_engine.*` — additive `check_solid`
  virtual + OCCT oracle wrapper (`BRepCheck_Analyzer`) for the sim gate only.
- `include/cybercadkernel/cc_kernel.h` — additive `CCValidityReport` struct +
  `cc_check_solid` prototype; `cc_principal_moments` signature unchanged.
- `src/facade/cc_kernel.cpp` — additive `cc_check_solid`; `cc_principal_moments`
  now backed by the native engine.
- **Out of scope (declines, documented not faked):** certifying no-self-intersection
  on a general trimmed freeform patch (honest decline, not a false `valid`); inertia
  of a non-watertight / open body (declines → OCCT); products of inertia / a full
  tensor facade beyond the three principal moments (the ABI stays the existing 3-vector).
  No CyberCad app change beyond consuming the new symbols; no OCCT linked into
  `src/native/**`.
