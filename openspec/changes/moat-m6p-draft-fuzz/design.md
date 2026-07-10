# Design — moat-m6p-draft-fuzz

## Context

The M6 completeness bar drives random valid inputs through BOTH the native path and the
OCCT oracle and classifies every trial, requiring `DISAGREED == 0`. Fifteen domains have
landed. This session landed a NEW native surface: the molding/manufacturing DRAFT ANGLE
(`src/native/feature/draft_faces.h`) reached through the shipping `cc_draft_faces`
facade. The curated `native_draft_faces_parity.mm` harness proves both engines on THREE
hand-picked fixtures + one decline; this change turns it into a *seeded, randomized
batch* with a per-trial classifier over random prismatic bases and random draft poses.

## The closed-form arbiter (why native-vs-OCCT alone is insufficient)

A draft has no single trustworthy engine oracle: OCCT keeps a TRUE analytic B-rep while
the native arm emits a deflection-bounded facet weld, and a native-vs-OCCT-only test
cannot distinguish a native miss from an OCCT drift. The design removes that with a
closed form that is EXACT for the ideal prismatic draft.

The neutral plane is ALWAYS the base plane (origin (0,0,0), pull +Z) and every drafted
face is a vertical prism wall, so the drafted solid's cross-section at height z is the
footprint polygon with EACH drafted edge's supporting line pushed INWARD by z·tanθ
(non-drafted edges fixed):

> **`V = ∫₀^h A(z) dz`,  `A(z) = area( footprint clipped by the inward-shifted drafted
> half-planes )`.**

A(z) is a polynomial of degree ≤ 2 in z (a convex polygon's area under parallel edge
offsets), so a 3-point Simpson quadrature over [0,h] is **analytically exact**. The
clipping (Sutherland–Hodgman against each drafted edge's shifted half-plane) handles
adjacent-face corner interactions exactly — the reason the parity harness needed the
frustum formula for the 4-side box and could not simply sum per-face wedges. This
reproduces the parity harness's closed forms exactly:

- box +X single face θ=8°: A(z) = 10·(10−z·tan8°) → V = 1000 − 500·tan8°. ✓
- box 4-side θ=5°: A(z) = (10 − 2z·tan5°)² → the frustum `(H/3)(Abot+Atop+√(Abot·Atop))`. ✓

Because the closed form is exact, a native result matching it while OCCT is the outlier
is logged ORACLE-INACCURATE (native vindicated), never a bar failure. The drafted AREA
has no simple closed form (tapered walls + shrunk top cap), so AREA is cross-checked
against OCCT only, never against a fabricated analytic area.

## Both-engine facade drive + engine-local face resolution

Each trial builds the prism IDENTICALLY under both engines via `cc_solid_extrude`
(footprint in z=0, extruded +Z to h), then drafts through the SAME `cc_draft_faces`
facade: `cc_set_engine(1)` → NativeEngine `feature::draftFaces` (each drafted plane
derived from the original face, applied as an inward `splitByPlane` half-space cut, the
composite self-verified watertight / χ=2 / oriented / strict shrink, else NULL → OCCT);
`cc_set_engine(0)` → OCCT `BRepOffsetAPI_DraftAngle`. Sub-shape ids are engine-local, so
the requested drafted faces are resolved SEPARATELY under each engine's body from
geometry, never a stored id: for a given side face's mid-edge probe point + outward
normal, `sideFaceId` scans `cc_subshape_ids(body, 2)` and picks the face whose
`cc_project_point_on_face` foot distance ≈ 0 (the point lies on its plane) AND whose
outward-nudged (`+ε·n̂`) foot distance ≈ ε (the correct outward side), which disambiguates
adjacent faces. If either engine cannot resolve the requested set the trial is
BOTH-DECLINED (unposable), never laundered.

## The four families and their draft poses

| Family | Base (facade) | Drafted faces | Neutral / pull |
|---|---|---|---|
| BOX single | `cc_solid_extrude` w×d rectangle | 1 random side face | base z=0, +Z |
| BOX multi | `cc_solid_extrude` w×d rectangle | random ≥2 side subset | base z=0, +Z |
| NGON single | `cc_solid_extrude` regular n-gon (n∈[3,8]) | 1 random side face | base z=0, +Z |
| NGON multi | `cc_solid_extrude` regular n-gon (n∈[3,8]) | random ≥2 side subset | base z=0, +Z |

The draft angle is bounded so the top recession `h·tanθ` stays ≤ 35% of the base
inradius (the taper is real but the top never collapses / self-intersects), and ≤ 15°
(well below the 90° flip). The pathological collapse / self-intersecting poses are the
native ResolveFailed → OCCT decline branch, which the multi-face families exercise
naturally.

## Observed native envelope: adjacent-corner box multi-drafts HONESTLY-DECLINE

On the box multi-face family the native arm HONESTLY-DECLINES a meaningful fraction of
poses (≈ 40–45% of box-multi trials) — a sequence of tilted half-space cuts on two or
more ADJACENT box side faces produces a corner the composite self-verify rejects
(ResolveFailed), so it returns NULL → OCCT. This is FIRST-CLASS, not a fault: in every
declined case the closed form and OCCT agree EXACTLY (`volO == volX`), and where native
DOES seat a box-multi draft it matches both to ~1e-16. The n-gon multi family (obtuse
corners) seats far more readily (AGREES on 5/7 and 4/5 drafted faces at 11°). The bar is
met (each family ≥1 AGREED with real exact-match coverage); the decline fraction is
logged, never hidden, and is exactly the "equal-or-more-conservative" native discipline
the completeness bar wants — a native draft is never silently wrong.

## The six-way classifier (identical discipline to the landed siblings)

- **AGREED** — native VALID (watertight, χ=2, volume STRICTLY smaller than the base) +
  volume within the fixed band of BOTH the closed form AND OCCT, which also matches the
  closed form.
- **HONESTLY-DECLINED** — native `cc_draft_faces` → 0/invalid (out-of-envelope pose)
  while OCCT ships. First-class, counted separately, never a bar failure.
- **DISAGREED** — native VALID but outside the closed-form truth while OCCT matches it. A
  genuine SILENT WRONG draft. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed form while OCCT does not (native
  vindicated). Logged, not a fault, not a bar failure.
- **ORACLE_UNRELIABLE** — native misses the closed form AND OCCT also does not match it.
  FAILS the bar (investigate, never launder).
- **BOTH-DECLINED** — an out-of-envelope / unresolvable pose both engines refuse. Logged.

## The bar

`std::_Exit(0)` IFF `DISAGREED == 0 && ORACLE_UNRELIABLE == 0` with each of the four
families ≥1 AGREED, over ≥2 seeds (runner fails if any seed fails), N≥60/seed. The FIXED
bands (native-vs-closed-form volume < 1e-3 — planar draft volume is exact; native-vs-OCCT
volume < 2e-2, area < 3e-2) are NEVER widened. The generator is seeded ONLY by an
explicit `FUZZ_SEED` — no clock, no `rand()` — so the same seed → a byte-identical batch.

## Discipline

`src/native/**`, `src/engine/**`, `include/**`, and the `cc_*` ABI stay byte-unchanged —
test infrastructure only, driving the facade rather than modifying it. Does not fuzz
`cc_interference` (the concurrent CLASH fuzzer track) nor any other op.
