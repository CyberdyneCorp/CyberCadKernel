# add-native-wrap-emboss-breadth

Feature #7 WRAP-EMBOSS — **widen the native slice**. The first slice
(`add-native-wrap-emboss`, archived) landed exactly ONE native case behind the
unchanged `cc_wrap_emboss` ABI: a RAISED RECTANGULAR pad on a CYLINDER lateral
face (`boss = 1`), rebuilt as a deflection-bounded planar-facet soup welded
watertight in `src/native/feature/wrap_emboss.h` and gated by the engine's
watertight + volume-INCREASING self-verify → OCCT fallback.

This change widens that native path along three honest, independent tracks — each
gated by the SAME mandatory self-verify and each returning NULL → OCCT for anything
it cannot robustly build:

- **T1 DEBOSS** (highest confidence): a RECESSED rectangular pocket cut inward
  (radius `R → R − depth`), volume SHRINKS by ≈ footprint-area × depth. A clean
  mirror of the raised-pad build with an inward offset and flipped wall
  orientations; the engine self-verify gains a volume-DECREASING branch.
- **T2 NON-RECTANGULAR profile**: an arbitrary CLOSED N-vertex POLYGON footprint
  (convex robustly; simple non-convex, e.g. L-shape, attempted via a constrained
  wall retiling and accepted only if the self-verify passes) projected onto the
  cylinder and embossed raised. The self-verify switches from the bbox area to the
  true shoelace polygon area.
- **T3 FREEFORM base** (hardest — narrow slice or honest decline): a raised
  rectangular pad on a CONE lateral face (offset along the surface normal to a
  parallel coaxial cone). SPHERE and everything else stay OCCT. If the cone weld
  cannot be made watertight with correct volume it collapses to an honest decline
  (→ OCCT) with the measured gap reported — no dead code.

The raised-rectangular-pad-on-cylinder emboss is the UNTOUCHED control. Native
stays OCCT-free; OCCT `cc_wrap_emboss` (#290, Phase 3) remains the oracle and the
fallback. No `cc_*` signature or POD struct changes.

Reference: [../../ROADMAP.md](../../ROADMAP.md) (#7 wrap-emboss),
[../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S5 → #7 wrap-emboss),
[../../NATIVE-REWRITE.md](../../NATIVE-REWRITE.md);
archived first slice `add-native-wrap-emboss`; GitHub #290 (Phase-3 OCCT oracle).
