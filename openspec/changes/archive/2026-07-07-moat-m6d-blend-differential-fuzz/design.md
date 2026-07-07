# Design — moat-m6d-blend-differential-fuzz

## Context

The M6 curved-boolean fuzzer (`native_boolean_fuzz.mm`) established the pattern: a
splitmix64/xoshiro256** seeded generator emits random-valid inputs; each input is run
through BOTH the native path and an OCCT oracle built from the SAME parameters; a
classifier bins each trial AGREE / honestly-declined / DISAGREE / oracle-inaccurate /
both-declined; the process exits non-zero if any DISAGREE (a silent wrong result). M6b
(STEP import) and M6c (construction) reused that skeleton for two more domains. This
change reuses it VERBATIM for a fourth, independent domain — the native blend path — and
promotes the closed-form analytic arbiter (introduced in M6b/M6c) to the PRIMARY
correctness oracle, because one blend family's OCCT reference is itself only approximate.

The native blend library (`src/native/blend`) is OCCT-free and header-only; it depends
only on `src/native/{math,topology,tessellate,boolean,construct}` — all OCCT-free — so the
harness needs **no numsci** (unlike the boolean fuzzer, whose SSI path does). The only
compiled native TUs are `math/bezier.cpp` + `math/bspline.cpp`. The OCCT dependency is
strictly the oracle (`BRepPrimAPI_MakeBox` / `MakeCylinder` + `BRepFilletAPI_MakeFillet` /
`MakeChamfer` + `BRepGProp` + `BRepCheck`).

## The differential: native builder vs OCCT builder on the SAME edge

Pipeline per generated case (a body + a picked edge/rim + blend params):

```
params ─native primitive──▶ native body ─pick edge/rim─▶ native blend (OCCT-FREE) ─native mesh─▶ (volN, areaN, wt, solidsN)
       └OCCT primitive────▶ OCCT body   ─match edge/rim─▶ OCCT BRepFilletAPI      ─BRepGProp──▶ (volO, areaO, valid, solidsO)
                                                                                   └analytic────▶ aVol (exact removed volume)
```

Unlike the curated parity harnesses (`native_curved_fillet_parity.mm` etc.), which drive
the `cc_*` facade with `cc_set_engine` so a native decline SILENTLY forwards to OCCT — you
cannot then tell native-handled from fall-through — this harness calls the OCCT-free native
blend builders DIRECTLY, exactly as `native_construct_fuzz.mm` does. A NULL Shape or a
non-watertight candidate (which the engine's mandatory self-verify would DISCARD) is an
UNAMBIGUOUS native DECLINE, and OCCT is invoked SEPARATELY as the reference oracle.

The SAME geometric edge is blended in both bodies. For the box (a native `build_prism`
rectangle vs `BRepPrimAPI_MakeBox` of identical dimensions at the origin) the picked
native edge's world endpoints are matched to the OCCT box edge by vertex coincidence. For
the capped cylinder (a native `build_prism_profile` full-circle vs
`BRepPrimAPI_MakeCylinder`) the top circular rim is picked in both by geometry (the Circle
edge at `z = h`). This keeps the differential a true same-input comparison.

### Why generate the bodies through the construct entry points the facade uses

The capped cylinder is built with `build_prism_profile` on a single `kind-2` full-circle
`ProfileSegment` — the exact path `cc_solid_extrude_profile` takes and the one the curated
`native_curved_fillet_parity` / `native_curved_chamfer_parity` harnesses already prove
produces a body with ONE Cylinder wall + two planar caps + true Circle rim edges that the
native rim recogniser (`facesOnRim` / `rimGeom`) accepts. The box is built with
`build_prism` on a rectangle. Building through the proven facade path makes the input a
valid native B-rep AND a valid OCCT primitive, so a DISAGREE is attributable to the BLEND,
not to a bad source body.

## Decision: the closed-form analytic ground truth is the PRIMARY oracle

**Problem.** The task frames OCCT as the oracle. But a pure native-vs-OCCT comparison
reflexively blames the native builder on any disagreement — even when OCCT is the one that
is inaccurate. Two structural reasons make that a real hazard here:

1. **The native blend is a planar-facet (inscribed) approximation of a curved surface.**
   The fillet torus / chamfer frustum / capped cylinder are tiled into planar triangles at
   the blend `deflection`, so the native solid sits a small, deflection-bounded amount
   below the true OCCT solid. This is the same native-mesh-vs-OCCT-exact asymmetry the
   curved-fillet/chamfer parity harnesses carry.
2. **OCCT's own variable-radius fillet is an APPROXIMATE evolved surface.** For the
   `variable_fillet_edge` family (linear law r1→r2), `BRepFilletAPI_MakeFillet::Add(r1,r2,E)`
   builds an evolved blend whose volume can differ from the EXACT variable canal by ~2–2.6%
   — MORE than the native faceting error. Here native-vs-OCCT is a comparison of TWO
   approximations, and OCCT is frequently the larger-error one.

Empirically (measured): the native-vs-**exact-math** volume error is a tight, stable ≤
~1.6e-3 for EVERY family (including the variable fillet), while the native-vs-OCCT gap for
the variable fillet reaches ~1.7e-2 and OCCT-vs-exact-math reaches ~2.6e-2. Blindly
trusting OCCT would flag the CORRECT native variable fillet as a silent wrong result — a
false positive that papers over an OCCT limitation.

**Decision.** Every AGREE family has a **closed-form removed volume**, so the blended
solid volume is exact:

| family                         | exact removed volume |
| ------------------------------ | -------------------- |
| planar chamfer (90° box edge)  | `L·d²/2` |
| planar fillet (90° box edge)   | `L·r²(1 − π/4)` |
| curved fillet, constant `r`    | `2π(1−π/4)·r²·(Rc−r) + (π/3)·r³` |
| curved fillet, linear `r1→r2`  | `2π[(1−π/4)·Rc·⟨r²⟩ − (5/6−π/4)·⟨r³⟩]`, `⟨r²⟩=(r1²+r1r2+r2²)/3`, `⟨r³⟩=(r1³+r1²r2+r1r2²+r2³)/4` |
| curved chamfer, symmetric `d`  | `π·d²·(Rc − d/3)` |
| curved chamfer, asym `d1,d2`   | `π·d1·d2·(Rc − d2/3)` |

(The variable-fillet form is the constant Pappus form integrated azimuthally over the
linear law; `r1==r2` reduces to the constant form exactly, a checked identity.)

The classifier uses the analytic truth as the correctness oracle:

- **AGREED (clean differential)** — native-vs-OCCT volume/area/solid-count all within a
  FIXED tolerance. This is the fast path for the families where OCCT is exact (planar
  chamfer is a planar cut, native == OCCT to machine epsilon; constant fillet / cone-
  frustum chamfer agree to ~1e-3).
- When native-vs-OCCT EXCEEDS the fixed tolerance, arbitrate with exact math:
  - **native matches exact math AND OCCT matches exact math → AGREED** (native VINDICATED;
    the native-vs-OCCT gap is just two deflection-bounded approximations of the same exact
    solid — a correct native result, counted separately for audit).
  - **native matches exact math, OCCT does NOT → ORACLE-INACCURATE** (native right, OCCT is
    the outlier; logged in full, NOT a bar failure, NOT a native fault).
  - **native does NOT match exact math → DISAGREED** (native watertight but WRONG vs exact
    math — the silent wrong blend this harness exists to catch).

A native result is exonerated ONLY when it POSITIVELY matches the independent closed-form
truth; a native result that fails exact math stays DISAGREED. The tolerance is FIXED
(`kVolRelTol = 2e-2`, `kAreaRelTol = 3e-2`) and NEVER widened per-trial. Because volume is
gated by exact math, an area-only excursion (the variable fillet's faceted band area can
graze the area tol) can never produce a false DISAGREE — it falls to the volume-arbitrated
branch. This is a STRENGTHENING: the bar is now "native must match EXACT MATH or honestly
decline", stronger than "native must match an also-approximate OCCT".

## Guards (so a DISAGREE is the builder's fault, never the oracle's)

- **ORACLE_UNRELIABLE** — for a CORE family the OCCT build MUST be a valid closed solid
  with positive volume/area and ≥1 solid; if it is not, the input is not a trustworthy
  oracle → excluded from the verdict and FAILS the bar (investigate, never launder).
- The DECLINE-exerciser family (`Rc/2 < r < Rc`) may legitimately have native NULL while
  OCCT builds (HONESTLY-DECLINED) or, at the extreme, both refuse (BOTH-DECLINED) — never
  a bar failure for that out-of-scope family.

The BAR: `DISAGREED == 0` AND core-family `ORACLE_UNRELIABLE == 0`.

## Deflection / tolerance calibration (fixed, auditable)

The native blend `deflection` is a fine, FIXED `kBlendDefl = 0.004`, and the rim radius is
BOUNDED (`Rc ∈ [3, 6]`) so the angular facet count is never capped — keeping the inscribed
bias far under the fixed tolerance. The measured max native-vs-OCCT bias on AGREE is logged
in the coverage summary (vol ≈ 1.7e-2, area ≈ 2.6e-2 — driven entirely by the variable
fillet, where the analytic arbiter takes over), and the native-vs-exact-math error is ≤
~1.6e-3 for every family. The tolerances are chosen once and documented; they are never
widened per-trial to reclassify a disagreement.

## Scope honesty

The concave stepped-shaft fillet (`concave_fillet_edge`) and `offset_face` / `shell` are
part of the native blend path's claimed scope but are DELIBERATELY excluded from this first
blend-fuzz slice: a concave stepped-shaft body and a shell cavity are not yet cleanly
generatable as a seeded random family with a matching OCCT oracle. They remain covered by
the curated parity harnesses; the exclusion is a first-class DOMAIN-level honest DECLINE,
recorded in the harness header and the spec delta so no coverage is silently dropped. A
later slice can extend the generator to those families once a clean oracle is in place.

## Exit-clean note

Like the siblings, the harness `std::fflush`es and calls `std::_Exit` — OCCT static
teardown in the trimmed static simulator build is not exit-clean, so the process exits on
the verdict without running OCCT's global destructors (the SAME rationale as
`native_boolean_fuzz` / `native_construct_fuzz`).
