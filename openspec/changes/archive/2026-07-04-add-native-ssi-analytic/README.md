# add-native-ssi-analytic

SSI Stage **S1** — native, OCCT-free, **closed-form** surface-surface intersection
for the elementary-surface conic family. Given two native analytic surfaces
(`src/native/math/`: plane, cylinder, cone, sphere, torus), a pair-dispatch
classifies the pair and either returns exact native intersection curve(s)
(`Line` / `Circle` / `Ellipse` / conic, or up to a planar quartic solved with the
native `numerics` polynomial substrate) that provably lie on BOTH surfaces and
match OCCT `GeomAPI_IntSS`, or returns **NOT-ANALYTIC** for out-of-scope pairs
(general skew quadric∩quadric, freeform/NURBS, near-tangent/coincident) so they
defer to S2/S3 marching or OCCT — never faked.

SSI is an **internal** kernel capability: **no `cc_*` ABI change**. It is verified
at the SSI-function level (native curves vs OCCT `GeomAPI_IntSS`), exactly like
native-math and native-topology parity. This is the first SSI stage and the
on-ramp to curved booleans (S5).

Reference: [../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S1 scope + two-gate
verification model).
