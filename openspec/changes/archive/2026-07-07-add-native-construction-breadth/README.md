# add-native-construction-breadth

Feature #4 NATIVE CONSTRUCTION — a **breadth batch** widening the native swept-solid
constructors into three Tier-4 cases that today fall straight through to OCCT, honest
per-track. Today native construction does: extrude, revolve, spline/typed-profile
extrude, on-axis-arc + torus revolve, the 2- & N-section EQUAL-count RULED loft, the
straight / smooth-planar / RMF sweep, the twisted / scale-guided sweep, the straight-rail
loft, the tapered shank, and the fine-pitch-RESOLVED helical / tapered thread. This change
adds three tracks, each with its own honest gate (build watertight with the correct volume,
or return NULL → OCCT and REPORT the measured gap):

- **T1 — MISMATCHED-count ruled loft** (highest confidence, most tractable). The current
  ruled loft (`build_ruled_loft_sections`) requires every section to share the SAME vertex
  count `n`; an M-gon lofted to an N-gon (`M ≠ N`) returns NULL → OCCT. This adds a VERTEX
  CORRESPONDENCE — an arc-length-parameter UNION resample that inserts collinear points on
  each loop so both reach a common count `K` at matched normalized arc-length stations,
  then reuses the existing equal-count `alignSectionB` pairing + ruled bands. Because an
  inserted point on a straight polygon edge is GEOMETRY-PRESERVING (collinear, zero area
  change), the resampled loft is EXACT — its enclosed volume equals the true ruled loft
  between the M-gon and N-gon, mirroring OCCT `BRepFill_CompatibleWires`. Behind the
  UNCHANGED `cc_solid_loft` / `cc_solid_loft_wires`.

- **T2 — ORIENTATION-constraining guided sweep** (honest decline expected). The shipped
  `cc_guided_sweep` already binds the guide to a SCALE constraint (each station uniformly
  scaled by the guide splay `dist(path,guide)/d0`, fixed-world-up per-station Frenet frame)
  and is already NATIVE. A guide that constrains ORIENTATION (the section AIMED at the
  guide — the MakePipeShell `SetMode(guideWire)` frame where the section up-axis tracks the
  spine→guide direction) is a DIFFERENT frame law with NO oracle behind the fixed
  `cc_guided_sweep` scale-splay semantics: adopting it would either change that entry's
  fixed oracle (an ABI/semantics break, forbidden) or produce a solid that fails parity vs
  the shipped scale oracle (exactly the RMF-vs-constant-frame mismatch already documented
  in `sweep.h`). Attempted only for the narrowest reproducible slice (planar spine + planar
  guide, no twist), RETAINED only if it self-verifies watertight AND matches a real OCCT
  guide-orientation oracle; otherwise an HONEST DECLINE — no dead code, gap REPORTED.

- **T3 — FINE-PITCH self-intersecting thread** (honest decline expected). The helical
  thread already RESOLVES the near-touching regime (a root flat separates turns whose V
  bases meet). The remaining case is the GENUINELY self-intersecting fine-pitch thread —
  adjacent turns whose radial-V flanks CROSS in 3D at a steep helix lead (the
  `kMaxLeadRatio = 0.35` guard). A single radial-V ruled tiling cannot represent crossing
  helicoid flanks watertight; resolving it needs surface-surface intersection (trim the two
  overlapping flank helicoids — Tier 4 SSI). Attempted ONLY if a narrow slice self-verifies
  watertight + correct volume; otherwise an HONEST DECLINE — the self-intersecting thread
  stays a documented OCCT-fallthrough, gap REPORTED, with NO always-NULL dead builder.

The native paths land behind the UNCHANGED `cc_solid_loft` / `cc_solid_loft_wires` /
`cc_guided_sweep` / `cc_helical_thread` / `cc_tapered_thread` ABI — no new facade entry, no
POD change. Every candidate is accepted ONLY through the engine's mandatory watertight +
sane-volume self-verify (`robustlyWatertight` + a positive-volume check), so a NULL builder
result or a failed self-verify DISCARDS the candidate and the engine forwards the SAME
arguments to the OCCT oracle. `src/native/**` stays OCCT-FREE; OCCT is the verification
ORACLE only. Nothing outside the named slices is faked — the honest-out returns NULL → OCCT
and REPORTS the gap, and where a track (T2 / T3) is not robustly tractable it declines
WITHOUT dead code.

Reference: [../../ROADMAP.md](../../ROADMAP.md) (#4 native construction),
[../../NATIVE-REWRITE.md](../../NATIVE-REWRITE.md) (the two-gate verify + honest
fall-through discipline),
[../archive/2026-07-03-add-native-loft](../archive/2026-07-03-add-native-loft)
(the equal-count ruled loft this change extends to mismatched counts for T1),
[../archive/2026-07-03-add-native-sweep](../archive/2026-07-03-add-native-sweep)
(the sweep / guided-sweep family T2 extends), and
[../archive/2026-07-03-add-native-threads](../archive/2026-07-03-add-native-threads)
(the helical / tapered thread T3 extends). Change shape mirrors
[../archive/2026-07-07-add-native-fillet-chamfer-breadth](../archive/2026-07-07-add-native-fillet-chamfer-breadth)
(one landed track + two honest declines, each gated by the mandatory self-verify).
