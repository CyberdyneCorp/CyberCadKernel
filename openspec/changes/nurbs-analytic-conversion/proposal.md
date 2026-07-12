# Proposal — nurbs-analytic-conversion (NURBS roadmap Layer 1)

## Why

The kernel represents geometry both analytically (plane / cylinder / cone / sphere /
torus surfaces; line / circle / arc / ellipse curves — `src/native/math/elementary.h`,
`torus.h`) AND as rational NURBS (`bspline.{h,cpp}`, `bspline_ops.{h,cpp}`). What is
missing is a first-class, **EXACT bidirectional** bridge between the two:

- Represent an analytic primitive as an EXACT rational NURBS (a rational quadratic
  arc with weight `cos(halfangle)`; a revolved profile for the quadrics and torus) —
  not an approximation.
- The hard inverse: RECOGNIZE when a given rational NURBS **is** exactly one of those
  analytic forms and recover its parameters exactly, else honestly report "general".

This underpins reverse-engineering (scan/STEP → analytic feature), exact booleans
(analytic↔analytic intersections instead of freeform SSI), and interoperability. It is
a Layer-1 foundational capability: pure control-point / knot arithmetic plus a
control-net algebraic certificate, OCCT-free, exactly verifiable by closed-form oracles.

## What

A new OCCT-free module `src/native/math/analytic_nurbs.{h,cpp}` (namespace
`cybercad::native::math`, beside the evaluators and `primitive_fit`):

1. **Analytic → NURBS (exact construction).** `circleToNurbs`, `arcToNurbs`,
   `ellipseToNurbs`, `lineToNurbs`; `planeToNurbs`, `cylinderToNurbs`, `coneToNurbs`,
   `sphereToNurbs`, `torusToNurbs` — the standard exact rational representations from
   Piegl & Tiller *The NURBS Book* Ch.7 (rational-quadratic conics with weight
   `cos(halfSweep)`, a full circle as 4 quarter segments; surfaces of revolution for
   the quadrics and torus). Homogeneous lift throughout.

2. **NURBS → analytic (recognition).** `recognizeCurve` / `recognizeSurface` — detect
   whether a rational NURBS curve/surface is exactly a line/circle/arc/ellipse or
   plane/cylinder/cone/sphere and recover its parameters, else report "general". The
   Layer-7 primitive-fit machinery (`primitive_fit.h`) runs on sampled points as a
   CANDIDATE GENERATOR; acceptance then requires an ALGEBRAIC exactness certificate
   (control-net level, ≤ 1e-12), never an RMS threshold.

Because recognition calls `primitive_fit` (numsci facade), the module and its host gate
build only under `CYBERCAD_HAS_NUMSCI`, exactly like `primitive_fit` / `bspline_fit`.

## Impact

Additive only. No existing file changes behavior; the `cc_*` ABI is byte-unchanged. A
new host gate `tests/native/test_native_analytic_nurbs.cpp` proves the oracles.
`src/native/**` stays OCCT-free (0 OCCT/Geom/BRep/TK references in changed files).
