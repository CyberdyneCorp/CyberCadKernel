# Descope native IGES import (STEP-only interchange)

## Why

The drop-OCCT endgame adopts **STEP as the sole native interchange format**. Native
STEP import has landed as a first slice (the AP203 manifold-solid-brep subset); native
IGES import was previously carried as an "out of scope, still OCCT" item — implicitly a
future `drop-occt` blocker. IGES is a 1980s format, frozen since 1996 (v5.3), and modern
CAD interchange is overwhelmingly STEP. Building a native IGES reader was estimated at
~1.5–3 py for low real product value.

This change makes the decision explicit: **native IGES import (and export) will NOT be
built.** It is removed from the drop-OCCT roadmap. This saves ~1.5–3 py of the remaining
drop-OCCT budget at low product cost, and removes IGES from the list of items that block
`#8 drop-occt`.

## What changes

- The two `native-exchange` requirements that framed IGES as "out of scope, stays OCCT"
  are MODIFIED to state IGES import/export are **DESCOPED**: no native path will ever be
  built; `cc_iges_export` / `cc_iges_import` remain unconditional OCCT fall-throughs (the
  `cc_*` ABI is preserved, additive-only) and are **removed/stubbed at `drop-occt`, not
  reimplemented natively**. The stale "STEP import stays OCCT" wording is corrected (the
  native STEP import slice has landed).
- No code changes. The `cc_iges_*` ABI entries are untouched (breaking the ABI is not
  permitted); they simply have no native future.

## Impact

- Roadmap: IGES import leaves the drop-OCCT blocker list; STEP import + the general
  curved/robustness tails + healing completeness are what remain.
- ABI: unchanged now. `cc_iges_*` documented as legacy OCCT-only, removed/stubbed at
  `drop-occt`.
- No behaviour change under either engine today (IGES still routes to OCCT).
