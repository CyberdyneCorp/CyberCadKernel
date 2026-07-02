# add-robust-thread-boolean

Robust thread-to-shaft boolean (#286). A feature-based apply that fuses/cuts a
helical thread into a shaft WITHOUT the minutes-long hang of a brute-force
`BRepAlgoAPI` on a fine multi-turn helix — by segmenting the boolean per-turn (or
per-turn-group) with a tuned fuzzy value and accumulating, so each sub-boolean is
small and bounded. Additive `cc_*`: `cc_thread_apply(shaftId, threadId, op)`.
Verified on the simulator: a fine multi-turn thread applied to a shaft COMPLETES
within a strict wall-clock budget (< 8 s) AND yields a `BRepCheck_Analyzer::IsValid`,
watertight solid with the correct volume-change sign. The naive full boolean
(known to hang) is NOT run in the test; completion-within-budget is asserted.
