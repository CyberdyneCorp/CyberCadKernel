# add-full-round-fillet

Rolling-ball / full-round fillet (#285). Replaces a narrow face (or the strip
between two side faces) with a rolling-ball blend tangent to both neighbours — the
middle face is CONSUMED — which OCCT's stock edge-fillet API does not do directly.
Additive `cc_*`: `cc_full_round_fillet(bodyId, faceId)` (three-face form:
`cc_full_round_fillet_faces(bodyId, leftFaceId, middleFaceId, rightFaceId)`).
Verified on the simulator: result `BRepCheck_Analyzer::IsValid` + watertight, the
target middle face is GONE, and the blend is G1-tangent to both neighbour faces at
the seam (sampled surface normals on both sides agree within tolerance). If a true
full-round cannot be built for a case, it falls back to a standard valid edge
fillet and is marked deferred with a note.
