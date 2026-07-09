# Design — moat-rim-curved-rim-weld (MOAT M0 tessellator weld for the outer curved RIM)

## Context

`moat-m0w-closed-seam-weld` fixed the closed INNER seam (disk ∪ annulus) of the curved-wall
boolean and isolated the remaining curved-wall COMMON blocker: the bowl's OUTER curved RIM
welded to the flat top lid. The rim is a genuinely CURVED shared edge (a per-segment
degree-2 Bézier arc), so the M0w seam-chord pin (2-pole degree-1 straight chord) does not
apply. The M0w agent noted a GENERALIZED pin broke byte-identity on primitives — the trap
this design must avoid.

## The measured root cause

The bowl-cup operand (`curved_wall_cut_fixture`): a steep degree-2 Bézier bowl trimmed by a
rim circle, plus a flat top lid at `z = a·R²`, sharing the rim. In COMMON the kept faces are
the free-form annulus (`faceOutside`, reusing the parent rim wire) + the flat lid (kept
whole) + the cap; the annulus↔lid RIM is the failure.

The rim is carried on SEPARATE edge nodes: the bowl wire's rim edge has a `Line` pcurve on
the bowl node, the lid wire's rim edge (a distinct clone) has a degree-2 B-spline pcurve on
the lid node. The solid mesher's twist pre-pass `requireEdgeSegments` (the bowl is a twisted
free-form face) raises the rim's min-segment count, so BOTH faces subdivide the rim to the
SAME shared fraction list (7–13 samples per arc). But the two faces place the interior
samples at DIFFERENT 3-D points:

```
f      C_edge (canonical rim)      S_bowl(pcurve)  div      S_lid(pcurve)   div
0.000  (0.3483,-0.0343,0.2450)     = C_edge        ~1e-17   = C_edge        ~1e-17   (shared vertex)
0.143  (0.3486,-0.0294,0.2447)     = C_edge        ~1e-17   z stays 0.2450  2.9e-4
0.429  (0.3490,-0.0196,0.2444)     = C_edge        ~1e-17   z stays 0.2450  5.8e-4   ← peak divergence
1.000  (0.3500, 0.0000,0.2450)     = C_edge        ~1e-17   = C_edge        ~1e-17   (shared vertex)
```

The bowl reproduces the rim curve exactly; the flat lid's planar pcurve stays in the plane
`z = 0.2450` while the true rim arc dips to `0.2444` — `S_lid(pcurve) ≠ C_edge`, diverging
up to 5.8e-4, far beyond the mesher's anchor-snap radius `kSnapEps = 1e-6`. So the
subdivided rim OPENS (512 → 1024 boundary edges used once). AND a coarse-regime
near-degenerate COINCIDENT sliver survives: near a rim corner both the bowl's constrained-
Delaunay boundary and the lid's ear-clip emit a thin triangle on the SAME three (near-
collinear) rim vertices — a rim edge then used by FOUR triangles (2 real + 2 coincident
slivers), non-manifold.

## Decision

Three guarded, additive, OCCT-free pieces in the byte-frozen M0 tessellator, the first of
which makes the whole change VERIFIED and FALLBACK-ONLY.

### 0. Verified, fallback-only rim pin (the byte-identity firewall)

`SolidMesher::mesh` first meshes the shape with the curved-rim pin DISABLED — EXACTLY the
pre-change tessellator — and returns that BASELINE result if it welds watertight. Every
existing mesh welds watertight on the baseline pass, so it is returned unchanged →
BYTE-IDENTICAL. The curved-rim pin is engaged ONLY on the FALLBACK pass, taken only when the
baseline is NOT watertight (the curved-wall bowl↔lid rim), and the pinned result is used only
if it is now watertight (else the honest non-watertight baseline is returned → the caller's
self-verify declines → OCCT, never a leak). Because a pinned mesh can REPLACE a baseline mesh
only when the baseline was non-watertight, NO already-watertight mesh is ever changed — the
byte-identity guarantee is structural, not merely empirical. This is what tamed the M0w
generalized-pin trap: earlier attempts pinned unconditionally and broke the freeform-boolean
family (first-freeform / split-plane / chain-seam / two-operand / multi-seam / breadth), whose
freeform CUT results present the same "Plane face sharing a freeform-backed curved edge" shape
as the bowl↔lid rim but weld fine WITHOUT the pin; the fallback returns their byte-identical
baseline and never engages the pin.

### 1. Curved-rim pin (the curved-edge analogue of the M0w seam-chord pin)

`face_mesher.h`'s `recordSeamChordPins` is already the point where the straight seam chord
is pinned to the shared canonical discretization. Generalise its GUARD (not its body — the
pin loop is identical) to fire on TWO mutually-exclusive shapes:

- `isSeamChord` — the 2-pole degree-1 straight chord (M0w, unchanged);
- `isCurvedSharedRim` (new, `edge_mesher.h`) — a genuinely-curved degree-≥2 free-form
  (Bézier / B-spline) arc, confirmed curved in 3-D (`!edgeStraight3d`).

The pin body then pins ONLY the samples that genuinely diverge
(`‖S_face(pcurve) − C_edge‖ > kSnapEps`) to the edge's ONE canonical discretization
`d.points`. On the rim the bowl reproduces `C_edge` (records nothing — byte-identical), and
the diverging flat lid is pinned to the shared rim, so the two boundaries become
bit-identical and the rim welds at any deflection.

### 2. Coincident-sliver drop + orphan compaction

`solid_mesher.h`'s spatial weld tallies each surviving triangle by its ORDER-INDEPENDENT
merged vertex triple and DROPS every copy of any triple that occurs more than once — a
coincident-duplicate pair (the coarse-regime sliver). It then compacts any vertex the drop
orphaned so the mesh stays a single closed 2-manifold with the clean Euler characteristic
`χ = 2` (an orphan would inflate the raw vertex count and read as `χ = 3`).

## Why this is byte-identical everywhere else (avoiding the M0w generalized-pin trap)

The trap is that a divergence-ONLY pin fires on ANALYTIC primitive seams. An early attempt
proved it: a divergence-only curved pin MOVED `revolve_sphere_ish` (42 shared circle seams,
156 diverging samples via the single-pcurve fallback), and a `requireEdgeSegments`-skip of
curved edges broke `thread` / `midwall` / `first_freeform` (their curved edges legitimately
need the twist subdivision). Two layers of guarding fix this, verified empirically:

- **KIND guard.** `isCurvedSharedRim` admits ONLY degree-≥2 free-form (Bézier / B-spline)
  arcs and EXCLUDES analytic `Circle` / `Ellipse` by kind. Every analytic primitive's shared
  curved edge — cylinder cap↔side, sphere / cone / revolve latitude, torus rim — is a
  `Circle` / `Ellipse`, hence ineligible. This alone removes the `revolve_sphere_ish`
  regression.
- **DIVERGENCE gate.** Among the remaining eligible free-form arcs, the pin fires only where
  `‖S_face(pcurve) − C_edge‖ > kSnapEps`. A free-form curved edge shared through ONE node (a
  whole Bézier / B-spline primitive) has one pcurve per face reproducing `C_edge`, so it
  never diverges and is never pinned. A twisted-loft / twisted-sweep free-form seam that DOES
  need subdivision keeps its unchanged twist sizing (the pre-pass is untouched) and its two
  incident free-form faces both reproduce `C_edge`, so it does not diverge either.
- **The twist pre-pass is untouched.** No change to `requireEdgeSegments` — so every
  twisted-loft / twisted-sweep / thread mesh is byte-identical.
- **The sliver drop / orphan compaction are existence-gated.** They fire only when a
  coincident duplicate / orphan EXISTS. The FNV battery confirms NO existing mesh contains a
  coincident-duplicate triangle (thread carries a few zero-3-D-area triangles but ZERO
  duplicates) or an orphan vertex, so the drop is a no-op and the compaction is the identity
  remap (bit-identical) everywhere except the rim mesh.

Proven by a FNV hash battery over `{vertexCount, triangleCount, vertices, triangles,
watertight, area, volume}` for 14 solid kinds × 8 deflections = 112 hashes: 0 non-bowlcup
lines change; only the 5 previously-failing bowl-cup rim cases go non-watertight →
watertight.

## Alternatives considered

- **Divergence-only pin (no KIND guard)** — REJECTED: moved `revolve_sphere_ish` (the M0w
  trap). The analytic `Circle` seams are curved and diverge via the fallback pcurve.
- **Skip curved edges in `requireEdgeSegments`** — REJECTED: broke `thread` / `midwall` /
  `first_freeform`, whose curved edges legitimately need the twist subdivision.
- **Unconditional pin, gated by KIND + Plane-face + a deflection-scaled divergence band** —
  REJECTED: the band is a heuristic, not a guarantee; it still moved `first_freeform`,
  `split_plane`, and `chain_seam`, whose freeform CUT results present the same shape as the
  bowl↔lid rim within any band. Replaced by the VERIFIED FALLBACK (decision 0), which is a
  structural guarantee: an already-watertight mesh is never touched.
- **Pin gated by a cross-face "freeform-backed" registry alone** — INSUFFICIENT on its own:
  the freeform-boolean CUT caps ARE freeform-backed yet still break under the pin, so the
  registry narrows but does not by itself guarantee safety; the verified fallback is what
  makes it safe (the registry keeps the fallback pass cheap and targeted).
- **Global degenerate-triangle drop (area threshold)** — REJECTED: `thread` already carries
  zero-3-D-area triangles that are part of its manifold; a threshold drop changes its hash
  and risks its watertightness. The coincident-DUPLICATE test is exact and defect-specific.
- **Widen the weld / snap tolerance** — REJECTED (discipline): would over-merge fine
  curvature grids and is a forbidden global-tolerance change.

## Honesty + discipline

`src/native/**` stays OCCT-free. The `cc_*` ABI is additive-only (`git diff include/`
empty). No global weld / snap tolerance is widened. A rim that STILL cannot weld returns
non-watertight → NULL → OCCT (never a leaky solid) via the `curved_wall_cut` self-verify.
