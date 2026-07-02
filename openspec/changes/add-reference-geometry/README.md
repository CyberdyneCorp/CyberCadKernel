# add-reference-geometry

Datum reference geometry (planes + axes) as additive `cc_*` entry points that
return POD (origin + unit direction/normal) — no shape handle. Pure-math
constructors (plane from 3 points, offset plane, axis from 2 points) work in BOTH
the host (no-OCCT) build and the OCCT build; the derived constructors (plane from
a planar face, axis from a linear edge, axis from a cylindrical/conical face,
reusing `cc_face_axis` logic) use OCCT and return failure in the host stub.
Verified by EXACT analytic checks (known normal/direction, offset origin) plus
degenerate/colinear-input failure.
