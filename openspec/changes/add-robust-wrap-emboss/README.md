# add-robust-wrap-emboss

Robust wrap-emboss (#290). Replaces the fragile `BRepOffsetAPI_ThruSections`
emboss-onto-cylinder pad with a cap-and-side surface build that is explicitly
sewn and healed (`BRepBuilderAPI_Sewing` + `ShapeFix`) into a VALID solid, so a
dense / high-curvature wrapped profile still yields a watertight boss/deboss.
The existing `cc_wrap_emboss` signature is unchanged; an internal robust path is
added, with the old ThruSections build kept as a coarse fallback. Verified on the
simulator: `BRepCheck_Analyzer::IsValid` + watertight, embossed volume > base /
debossed volume < base, and a profile that made the old ThruSections path invalid
now succeeds (or falls back to a valid coarse result and is marked deferred).
