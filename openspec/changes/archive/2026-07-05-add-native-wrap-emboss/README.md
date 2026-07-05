# add-native-wrap-emboss

Feature #7 WRAP-EMBOSS — **first native slice**. `cc_wrap_emboss` is OCCT-only
today (the Phase-3 cap-and-side + healed-sew oracle in
`src/engine/occt/occt_wrap_emboss.cpp`, GitHub #290). This change adds the FIRST
NATIVE path behind the SAME ABI: **emboss a RECTANGULAR PAD onto a CYLINDER lateral
face** (`boss = 1`).

The native builder projects the rectangular footprint onto the cylinder (arc-length
`px → angle px/R`, axial `py`, centred on the face's V-mid), builds the raised pad
as a wrapped-side + outer-cap surface set (the two axial walls follow the cylinder;
the two circumferential walls are radial planes; the outer cap is a trimmed cylinder
at `R + height`), welds it watertight, and fuses it with the base cylinder solid via
the native curved boolean / SSI seam (the pad side walls ∩ the cylinder).

The engine runs a MANDATORY watertight + volume-INCREASING self-verify and DISCARDS
a bad native result, falling through to the OCCT `cc_wrap_emboss` oracle. Everything
outside the named slice (non-rectangular / complex profiles, deboss, freeform base,
non-cylindrical faces) returns NULL → OCCT — never faked.

Reference: [../../ROADMAP.md](../../ROADMAP.md) (#7 wrap-emboss),
[../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S5 → #7 wrap-emboss),
[../../NATIVE-REWRITE.md](../../NATIVE-REWRITE.md); GitHub #290 (Phase-3 OCCT
oracle).
