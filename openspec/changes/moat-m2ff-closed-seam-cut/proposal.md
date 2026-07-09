# Proposal — moat-m2ff-closed-seam-cut (MOAT M2 freeform↔freeform CLOSED-SEAM CUT / COMMON)

## Why

Every landed M2 freeform boolean welds a freeform operand against an ANALYTIC cutter:
the single-operand half-space CUT (`half_space_cut.h`) cuts a freeform wall by an
infinite PLANE; the curved-wall CUT (`curved_wall_cut.h`) cuts a freeform wall by a
horizontal PLANE along a CLOSED circular seam; the two-operand FUSE (`two_operand.h`)
welds a freeform operand against an all-PLANAR box. In every case exactly ONE side of
the shared seam is curved — the other is flat.

The M2 roadmap's sharpened next blocker (curved-wall wave) is the genuine
freeform↔freeform case: CUT / COMMON of TWO curved operands over a shared CLOSED curved
seam, where BOTH sides of the seam are curved sub-faces. This is the highest-value slice
the just-landed M0w topology-aware closed-inner-seam tessellator weld unblocks: a shared
CLOSED straight-chord seam between two CURVED sub-faces now welds watertight at any
deflection (the M0w pin fires on the seam-chord topology regardless of which surface each
side carries), so a curved-disk-to-curved-annulus weld across a closed seam is now
landable — it was the exact fragility M0w resolved, only previously exercised curved↔flat.

## What

One ADDITIVE OCCT-free header-only verb `freeformFreeformClosedSeamCut`
(`src/native/boolean/freeform_freeform_cut.h`, namespace `cybercad::native::boolean`) that
welds the CUT (`A − B`) and COMMON (`A ∩ B`) of two coaxial freeform bowl-cup operands
whose two curved walls intersect in ONE CLOSED curved seam. It:

1. **recognises** both operands via B1 `recogniseFreeformSolid` (each ONE freeform wall +
   analytic lid);
2. **traces** the shared seam `A.wall ∩ B.wall` as a CLOSED WLine via M1
   `ssi::trace_intersection` over two `makeBezierAdapter` surfaces (the WLine carries
   `(u1,v1)` on A and `(u2,v2)` on B);
3. **splits BOTH walls** by the SAME 3-D seam through the byte-frozen B2 smooth-trim
   `splitFaceSmoothTrim` — A's wall by the `(u1,v1)` pcurve, B's wall by a `(u2,v2)`
   re-keyed copy of the seam — each into a disk + an annulus;
4. **selects the survivors** for the requested op — CUT keeps A's material OUTSIDE B (A's
   annulus/disk on the keep side + A's lid) plus B's wall sub-face that lies INSIDE A (the
   new curved ceiling), all welded across the shared closed curved seam; COMMON keeps the
   lens (A's disk cap inside B + B's disk cap inside A);
5. **confirms** each survivor's membership via B3 `classifyPointInMesh` against the other
   operand's mesh (never guessed);
6. **welds** the survivors (M0 `SolidMesher`) and runs the mandatory self-verify (M0
   WATERTIGHT + a consistent op-volume bound), returning a NULL Shape (→ OCCT
   `BRepAlgoAPI_{Cut,Common}`) with a typed decline on ANY failure.

The shared seam is carried as ONE canonical closed straight-chord discretization laid on
BOTH curved sub-faces (the smooth-trim split's faithful straight-edge-per-node form), so
the M0w seam pin welds it watertight with NO tessellator change.

## Impact

- New header `src/native/boolean/freeform_freeform_cut.h`; new host suite
  `tests/native/test_native_freeform_freeform_cut.cpp` + fixture
  `tests/native/freeform_freeform_cut_fixture.h`; new sim harness
  `tests/sim/native_freeform_freeform_cut_parity.mm` + run script.
- `src/native/**` stays OCCT-FREE; no `cc_*` ABI change; the M0 tessellator, M1, B1, B2
  (`splitFace` + `splitFaceSmoothTrim`), and every landed boolean header stay
  BYTE-IDENTICAL. Strictly additive; correct decline is first-class.
