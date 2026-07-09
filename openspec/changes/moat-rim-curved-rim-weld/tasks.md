# Tasks — moat-rim-curved-rim-weld (MOAT M0 tessellator weld for the outer curved RIM)

Order: reproduce (done) → guarded curved-rim pin + coincident-sliver drop in the M0
tessellator → byte-identical hash proof for every existing mesh → flip the rim-weld ladder
test → host-analytic ladder gate → sim native-vs-OCCT parity gate → run both gates → docs →
commit. All native code stays OCCT-free and host-buildable (`clang++ -std=c++20`). The
`cc_*` ABI is additive-only. The change is strictly topology-guarded and ADDITIVE: every
existing surface kind and edge meshes BYTE-IDENTICAL; the ONLY meshes allowed to change are
the previously-failing curved-wall COMMON rim cases (non-watertight → watertight). No global
tolerance is widened; a rim that still can't weld DECLINES (non-watertight → NULL → OCCT) —
a correct decline is a first-class outcome.

## STATUS — LANDED (outer curved rim); curved-wall COMMON robust across the full ladder

The outer curved rim shared by the free-form bowl annulus and the flat top lid now welds
watertight at every deflection. Root cause was: the two faces share the rim on SEPARATE
nodes and subdivide it to the same fraction list, but the flat lid's planar pcurve does NOT
reproduce the 3-D rim arc (`S_lid(pcurve) ≠ C_edge`, diverging up to ~6e-4), so the
subdivided rim opened; plus a coarse-regime coincident triangulation sliver (a rim edge used
by four triangles) survived. The fix is (1) a topology-guarded CURVED-rim pin — the
curved-edge analogue of the M0w seam-chord pin — that pins the diverging lid rim samples to
the bowl's canonical rim curve `d.points`, and (2) a coincident-duplicate triangle drop plus
orphan-vertex compaction in the spatial weld that removes the degenerate sliver and restores
`χ = 2`. Both are divergence- and topology-gated so no other mesh changes.

## 1. Reproduce + diagnose (done)

- [x] 1.1 Confirm the byte-frozen baseline: `test_native_curved_wall_cut` CUT + closed-seam
  robust, COMMON asserted as a MEASURED decline via
  `curved_wall_common_rim_weld_fragility_is_measured_decline`; full host `ctest` 59/59.
- [x] 1.2 Isolate the rim as the blocker: the bare bowl-cup operand (bowl + lid sharing the
  rim) is NON-watertight at fine deflections (512 → 1024 open rim boundary edges + 1
  persistent non-manifold edge), independent of the COMMON assembly.
- [x] 1.3 Measure the divergence: the rim is shared on SEPARATE nodes; both faces subdivide
  it to the same fraction list, but `S_bowl(pcurve(f)) = C_edge(f)` (div ≈ 1e-17) while
  `S_lid(pcurve(f))` stays flat, diverging up to ~5.8e-4 (≫ `kSnapEps = 1e-6`).
- [x] 1.4 Localize the second failure surface: a coarse-regime near-degenerate COINCIDENT
  sliver (a rim edge used by FOUR triangles = 2 real + 2 coincident slivers, one from each
  face's near-collinear rim triangulation).
- [x] 1.5 Confirm the over-subdivision source: the rim's OWN curvature needs 1 segment; the
  twist pre-pass (`requireEdgeSegments`) forces it to 7–13, creating the zigzag both faces
  triangulate into duplicate slivers.

## 2. Implement the guarded curved-rim pin + coincident-sliver drop (done)

- [x] 2.0 `solid_mesher.h`: make the pin VERIFIED and FALLBACK-ONLY — `mesh()` welds a BASELINE
  pass (rim pin OFF = pre-change tessellator) first and returns it when watertight (every
  existing mesh → byte-identical); only when the baseline is non-watertight does it re-mesh with
  the rim pin ON and use the pinned result IF it is now watertight (else return the honest
  non-watertight baseline). This is the structural byte-identity firewall that tamed the M0w
  generalized-pin trap (an unconditional / band-gated pin broke the freeform-boolean family).
- [x] 2.1 `edge_mesher.h`: add `detail::isCurvedSharedRim` — a genuinely-curved degree-≥2
  free-form (Bézier / B-spline) arc, curved in 3-D; excludes analytic `Circle`/`Ellipse` (by
  KIND — every primitive seam), `Line`, degree-1 polylines, and the 2-pole seam chord.
- [x] 2.2 `face_mesher.h` `recordSeamChordPins`: fire on `isSeamChord` OR `isCurvedSharedRim`;
  the curved-rim arm is gated on the face being a `Plane` AND the edge being FREEFORM-BACKED
  (`edge_mesher.h` `markFreeformBackedRim`/`isFreeformBackedRim` — a free-form face provably
  reproduces `d.points`), and it skips the two corner ENDPOINT samples. Pins only the samples
  that DIVERGE (`‖S_face(pcurve) − C_edge‖ > kSnapEps`). The bowl reproduces `C_edge` (records
  nothing); the diverging flat lid is pinned to the shared rim. (These gates narrow the pin; the
  verified fallback of 2.0 is what makes it byte-identity-safe.)
- [x] 2.3 `solid_mesher.h` weld: drop every copy of a coincident-duplicate triangle (same
  merged vertex triple, order-independent) — the coarse-regime sliver pair.
- [x] 2.4 `solid_mesher.h` weld: compact any vertex the drop orphaned (identity / no-op when
  no orphan exists), so the mesh stays a single closed 2-manifold with `χ = 2`.

## 3. Byte-identical proof for every existing mesh (done)

- [x] 3.1 FNV battery over `{vertexCount, triangleCount, vertices, triangles, watertight,
  area, volume}` for 14 solid kinds (`prism_box`, `prism_triangle`, `revolve_cylinder`,
  `revolve_cone`, `revolve_sphere_ish`, `thread`, `sweep`, `loft_box`, `loft_frustum`,
  `loft_twisted`, `twisted_sweep`, `bowlcup_operand`, `midwall_operand`,
  `first_freeform_operand`) × 8 deflections = 112 hashes, baseline (stashed) vs changed.
- [x] 3.2 Result: 0 non-bowlcup lines change; the 5 previously-failing bowlcup rim cases go
  non-watertight → watertight (T count −2 from the dropped sliver, V compacted by the orphan
  drop). The 3 already-watertight bowlcup deflections stay BIT-IDENTICAL.
- [x] 3.3 Guard against the M0w generalized-pin trap, in layers: a divergence-only pin moved
  `revolve_sphere_ish` (fixed by the KIND guard excluding `Circle`); a `requireEdgeSegments`
  skip broke `thread` / `midwall` / `first_freeform` (fixed by keeping the twist pre-pass
  intact); a band-gated / freeform-backed pin STILL broke the freeform-boolean family
  (`first_freeform` / `split_plane` / `chain_seam` / `two_operand` / `multi_seam` / `breadth`)
  whose CUT caps present the same shape as the rim — DEFINITIVELY fixed by the VERIFIED
  FALLBACK (2.0): the baseline (pin OFF) is returned for every already-watertight mesh, so the
  full ctest suite is regression-free and the FNV battery byte-identical.

## 4. Flip the rim-weld ladder test (done)

- [x] 4.1 Replace `curved_wall_common_rim_weld_fragility_is_measured_decline` with
  `curved_wall_common_rim_weld_watertight_across_full_ladder`: COMMON welds watertight
  (`χ = 2`) at every deflection `{0.012, 0.0102, 0.008, 0.006, 0.004, 0.002, 0.001}`, at the
  closed-form `V(z≥c)` within 3% each step and converging (finest < coarsest, finest < 1%).
- [x] 4.2 Update the `curved_wall_common_above_watertight_at_closed_form` comment and the
  file header (COMMON now robust at every deflection).

## 5. Two-gate verification

- [x] 5.1 Gate A (host analytic, no OCCT): `test_native_curved_wall_cut` all pass
  (COMMON full-ladder watertight + converging).
- [x] 5.2 Full host suite `ctest` — all pass (zero regression).
- [x] 5.3 Gate B (sim native-vs-OCCT parity): `run-sim-native-curved-wall-cut.sh` on a
  booted iOS simulator — 68/68; COMMON now a watertight MATCH to `BRepAlgoAPI_Common` +
  `BRepGProp` across `{0.0102, 0.0053, 0.004, 0.0028}` (euler=2, volume rel 1.76%→0.83%,
  area rel 0.1%→0.02%, Hausdorff ~1e-8), no longer an honest decline.

## 6. Docs + commit

- [x] 6.1 OpenSpec change (`proposal` / `tasks` / `design` + `native-tessellation` spec
  delta); `openspec validate moat-rim-curved-rim-weld --strict`.
- [x] 6.2 Update `openspec/MOAT-ROADMAP.md` (M0/M2: curved-rim weld landed, curved-wall
  COMMON robust).
- [ ] 6.3 Structural check (OCCT-free `src/native`, byte-identical proof, `cc_*` unchanged)
  and commit to `moat-rim` (concise technical message, byte-identical proof + gate numbers).
