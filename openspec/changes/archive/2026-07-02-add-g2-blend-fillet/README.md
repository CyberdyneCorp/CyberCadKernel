# add-g2-blend-fillet

Curvature-continuous (G2) blend fillet (#284) — OCCT ships only G1 / circular
fillets. Builds a curvature-continuous blend surface across a fillet seam (a
higher-degree `GeomFill` / conic blend with curvature matching to the neighbour
faces). Additive `cc_*`: `cc_fillet_edges_g2(bodyId, edgeIds, radius)`. Verified
on the simulator: result `BRepCheck_Analyzer::IsValid` + watertight, and the
SECOND-order surface properties (curvature via `BRepLProp_SLProps` /
`GeomLProp`) sampled on both sides of the seam are continuous within a G2
tolerance AND measurably closer to G2 than OCCT's stock G1 fillet at the same
seam. RESEARCH-GRADE and honest: if genuine G2 is not achieved, it reports the
measured curvature gap and is marked deferred — it does NOT claim G2 unless the
numbers show it.
