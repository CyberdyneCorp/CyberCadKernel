# add-native-ssi-seeding

SSI Stage **S2** — native, OCCT-free **subdivision seeding** for surface-surface
intersection. Given two native surfaces (including the freeform NURBS/Bézier and
non-closed-form quadric pairs that S1's dispatch returns `NotAnalytic` for), find
**at least one seed point on every distinct TRANSVERSAL intersection branch /
loop** so that S3 marching has a start point per branch.

The method (locked to **clean-room**): recursively subdivide each surface's
parameter domain into patches; bound each patch with an AABB (from the control-net
convex hull for B-spline/Bézier/NURBS; analytic/sampled bound for the elementary +
torus surfaces); **prune** patch pairs whose AABBs do not overlap and recurse on the
overlapping pairs down to a size threshold; **refine** each surviving candidate
region with the `native-numerics` `least_squares` substrate to drive
`S1(u1,v1) − S2(u2,v2) = 0` to a point on **both** surfaces (clamped to the param
ranges); and **dedup** seeds on the same branch by spatial clustering so the output
is ~one seed per branch. Each seed carries its `(u1,v1,u2,v2)`.

**Honest scope.** S2 targets **transversal** intersections. Near-tangent /
coincident / degenerate seeding (where the `least_squares` refine ill-conditions)
is **deferred to S4** — reported as a known gap, never faked with a fabricated seed.
Completeness (missing a small loop from too-shallow subdivision) is the honest
failure mode and is reported as a **measured branch-recall** figure vs OCCT
`GeomAPI_IntSS`, not claimed to be 100% blindly.

SSI is an **internal** kernel capability: **no `cc_*` ABI change**. Native code
stays OCCT-free (uses the NumPP/SciPP substrate behind `native-numerics`); the S2
refine module is compiled under `CYBERCAD_HAS_NUMSCI`. OCCT (`IntPatch` PrmPrm /
`IntWalk` seeding, `IntPolyh`) is used strictly as a verification **oracle**, never
copied. These seeds are the input contract for **S3** marching.

Reference: [../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S2 scope + verification
model). Extends the `native-ssi` capability from S1's closed-form dispatch.
