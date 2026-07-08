# Design — moat-gs56-inertia-validity (MOAT M-GS · GS5 + GS6)

Two OCCT-FREE analysis services over the LANDED M0 triangulation + topology + GS3
distance, each verified at **two gates**: (A) a HOST ANALYTIC gate with no OCCT
(closed-form inertia; hand-built valid + broken validity fixtures) and (B) a SIM
native-vs-OCCT gate (`GProp_PrincipalProps`; `BRepCheck_Analyzer::IsValid`). OCCT is
the ORACLE + fallback only; `src/native/**` stays OCCT-free and host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::analysis`.

## GS5 — Inertia tensor from signed-tetra second moments

### Substrate (already native, verified in source)

`tessellate/mesh.h` already computes the enclosed volume of a watertight mesh as a
**signed-tetra sum** over the triangle fan from the origin — `enclosedVolume(m) =
⅙ Σ aᵢ·(bᵢ×cᵢ)` (divergence theorem), valid only when `isWatertight(m)`. The inertia
tensor is the **second-moment** companion of that identical fan: each triangle
`(a,b,c)` with the origin forms an oriented tetra whose signed volume
`dV = (1/6) a·(b×c)` and whose covariance contribution is the standard closed-form
tetra second moment (Tonon 2004; the same fan the volume already trusts). No new
geometry, no new sampling — the inertia reuses the exact vertices/winding the volume
self-verify already blesses.

### Algorithm (about the centroid, unit density)

1. Mesh the solid watertight at the mass-properties deflection (the existing
   `SolidMesher` path used by `mass_properties`). **Precondition:** `robustlyWatertight`
   — a body that is not closed at every deflection in the ladder cannot yield a
   meaningful inertia and **declines → OCCT**. Planar polyhedra mesh EXACTLY, so their
   inertia is exact; curved solids are deflection-bounded (same bound the native
   `mass_properties` already ships under).
2. Accumulate over the tetra fan: total signed volume `V`, first moments (→ centroid
   `C = ∫r dV / V`), and the 6 independent second moments
   `∫x², ∫y², ∫z², ∫xy, ∫yz, ∫zx` (each via the per-tetra closed form, scaled by the
   tetra's signed `dV`). A consistently outward-wound closed mesh gives `V > 0`; a net
   `V ≤ 0` is treated as non-certifiable → decline.
3. Shift the covariance to the centroid with the parallel-axis (Huygens–Steiner)
   correction, then form the symmetric inertia tensor
   `I = tr(Cov)·Id − Cov` about the centroid (unit density → mass == volume, matching
   OCCT `GProp_GProps::PrincipalProperties`).
4. Diagonalize the symmetric 3×3 `I` with a **cyclic Jacobi** eigensolver (a small,
   fully-deterministic classic rotation sweep; converges in a handful of sweeps for a
   symmetric 3×3). Eigenvalues sorted ascending → principal moments `I₁ ≤ I₂ ≤ I₃`;
   the orthonormal eigenvectors → principal axes (sign-canonicalized so the frame is
   right-handed and each axis's largest component is positive, for a stable compare).

### Wiring & ABI

`NativeEngine::principal_moments` returns `std::vector<double>{I₁, I₂, I₃}` from the
native path (exactly the shape OCCT already returns), backing the **unchanged**
`cc_principal_moments(body, out3)` ABI. The principal **axes** are returned by the
native `analysis::inertia` service and asserted against `GProp_PrincipalProps` axes in
the SIM harness (which may link native directly) — no new facade symbol is added for
GS5. An axis that OCCT reports as degenerate/indeterminate (I₁≈I₂≈I₃, a sphere/cube:
any orthonormal frame is principal) is compared on the **moments** only, and the axis
frame is accepted as long as it is orthonormal and diagonalizes `I` — never asserted
against an arbitrary OCCT tie-break.

### Verification (both gates)

- GATE A (host, no OCCT): closed-form principal moments — **box** `a×b×c`
  `I = (V/12)·{b²+c², a²+c², a²+b²}`; **cylinder** r,h `{ (V/12)(3r²+h²) [×2], V r²/2 }`;
  **sphere** r `2/5·V r²` (×3). Match within a tight relative tolerance; planar box is
  exact, curved cylinder/sphere within the deflection bound.
- GATE B (sim): native `{moments, axes}` vs `GProp_PrincipalProps::Moments` + axes on
  the same shapes (analytic + a NURBS solid), scale-relative tolerance; the moment set
  is order-insensitive (sorted), axes compared up to sign and tie-degeneracy.

## GS6 — Standalone B-rep validity checker

A structured report with independent, individually-decidable verdicts. Each reuses
landed machinery; the checker composes them and stops at the FIRST failure for the
`first_failure` code while still being able to report every field.

| Check | Verdict source (landed) |
|---|---|
| finite coords | scan topology leaf points/poles for `std::isfinite` |
| closed 2-manifold | `mesh.h::isWatertight` (every undirected edge used exactly twice) + `edgeUseCounts` (no 3+ non-manifold edge) |
| consistent outward orientation | signed enclosed volume `enclosedVolume(m)` sign is uniform / positive under a consistent winding; a flipped face breaks the exactly-twice pairing or flips a fan sign |
| no degenerate face / edge | zero-area triangle (‖(b−a)×(c−a)‖ ≈ 0) / zero-length edge (‖b−a‖ ≈ 0) scan over the mesh + topology edges |
| no self-intersection | GS3 `distance.h` triangle-pair minimum-distance over non-adjacent triangles (broad-phase by AABB); an intersecting pair has distance 0 with interior-crossing witnesses |

### Honest decline (the load-bearing discipline)

The self-intersection test is robust for the **polyhedral** mesh of a solid the native
mesher meshes watertight. It is NOT a certificate for a **general trimmed freeform
patch** whose exact surface may self-touch between mesh samples. When the body carries
a freeform face whose no-self-intersection the mesh test cannot certify, the checker
returns `decided = 0` (`first_failure = SELF_INTERSECTION_UNDECIDABLE`,
`cc_last_error` set) — an **honest decline**, NEVER `valid = 1`. Certifying "valid" on
an unverifiable freeform is exactly the forbidden false-positive.

### ABI (additive)

```c
typedef struct {
  int valid;                  /* 1 iff decided AND every check passed */
  int decided;                /* 0 = honest decline (a check unreachable) */
  int finite_coords;          /* per-check pass (1) / fail (0) */
  int closed_manifold;
  int consistent_orientation;
  int no_degenerate;
  int no_self_intersection;
  int first_failure;          /* code of the first failing/undecidable check; 0 = none */
} CCValidityReport;

int cc_check_solid(CCShapeId body, CCValidityReport *out); /* 1 report produced, 0 decline */
```

`cc_check_solid` returns `1` when a report was produced (`decided = 1`, `valid` set to
the conjunction) and `0` on an honest decline (`decided = 0`, `cc_last_error` set,
`valid` NEVER 1). `NativeEngine::check_solid` computes the report OCCT-free; the OCCT
engine wraps `BRepCheck_Analyzer::IsValid` as the SIM oracle only.

### Verification (both gates)

- GATE A (host, no OCCT): hand-built fixtures with a KNOWN state — a valid closed
  tetra/box (→ `valid = 1`); a **non-closed shell** (a removed face → `closed_manifold
  = 0`); a **flipped face** (→ `consistent_orientation = 0`); a **zero-area / zero-
  length** degenerate (→ `no_degenerate = 0`); a **self-intersecting** polyhedron (→
  `no_self_intersection = 0`); each `first_failure` names the specific invalidity.
- GATE B (sim): native report vs `BRepCheck_Analyzer::IsValid` on the SAME valid AND
  deliberately-broken (non-closed shell, flipped face, self-intersecting wire)
  fixtures — the native overall verdict matches OCCT's valid/invalid on every one; a
  freeform whose self-intersection is uncertifiable **declines**, not a false valid.

## Complexity & discipline

- Every new function stays in the backend cognitive-complexity band (≤ 15): the
  inertia driver delegates to `accumulateMoments` / `centroidShift` / `jacobiEigen`;
  the validity driver delegates to one helper per check.
- No tolerance is weakened. A decline (open body inertia; uncertifiable freeform
  validity) is a first-class, expected outcome — never a fabricated number or a false
  `valid`.
- `src/native/**` remains OCCT-free (0 OCCT includes); the `cc_*` ABI change is purely
  additive (`CCValidityReport` + `cc_check_solid`).
