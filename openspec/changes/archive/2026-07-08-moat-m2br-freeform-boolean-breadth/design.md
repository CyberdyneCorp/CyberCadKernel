# Design — moat-m2br-freeform-boolean-breadth

## 1. The insight: COMMON is the complementary keep-side of the SAME half-space cut

For a solid `A` and a planar half-space `H = {x ≥ 0}`:

- `A CUT H  = A \ H       = A ∩ {x < 0}`  (the landed slice keeps `x ≤ 0`).
- `A COMMON H = A ∩ H      = A ∩ {x ≥ 0}`  (the piece on the OTHER side of the plane).

The landed `freeformHalfSpaceCut(operand, P, KeepSide)` already parameterises the
keep-side: `KeepSide::Below` keeps signed-distance ≤ 0, `KeepSide::Above` keeps ≥ 0. It
already flips BOTH the keep-side selection (which B2 sub-face + which analytic
sub/whole faces survive) AND the section-cap outward normal
(`outwardCap = (side == Below ? +1 : −1)·n_P`). Therefore **COMMON needs no new geometry
verb** — it is `freeformHalfSpaceCut(A, P, KeepSide::Above)`. This change proves that the
landed CUT machinery reaches the second boolean operator, and gates it on the
independent complementary volume oracle.

## 2. The host oracle: complementary volume + exact partition closure

The operand is `A = {(x,y,z) : (x,y) ∈ Q, −H0 ≤ z ≤ a(x²+y²)}` (M2-assembly fixture,
reused UNCHANGED). Its enclosed volume over any convex sub-polygon `R ⊆ Q` is the exact
polynomial `∫∫_R (H0 + a(x²+y²)) dA`, evaluated in closed form by fan-triangulating `R`
and summing per-triangle quadratic moments (`polyVolume`, from the M2-assembly fixture).

- CUT oracle (existing): `V_below = polyVolume(clip_{x≤0}(Q))`.
- COMMON oracle (new): `V_above = polyVolume(clip_{x≥0}(Q))` via `clipXge0` (the
  Sutherland–Hodgman complement of the existing `clipXle0`).
- **Partition closure (new, mesh-free, deflection-free):** `Q` is partitioned by `x = 0`
  into `Q ∩ {x ≤ 0}` and `Q ∩ {x ≥ 0}` (they overlap only on the measure-zero chord
  `x = 0`), and the integrand is identical, so `V_below + V_above = V_full` EXACTLY. This
  is the primary COMMON oracle — it needs no mesher and no OCCT, and it is the invariant a
  correct COMMON must satisfy against the landed CUT. Measured residual: `0` (machine
  precision).

## 3. Measured reachability + the shared-curved-edge weld fragility

Driving the landed verb at `KeepSide::Above` on the host (no OCCT) yields the correct
complementary solid, but its watertightness is deflection-fragile. A deflection sweep
records (WT = watertight solid, NULL = self-verify declined `NotWatertight`; a "leak" was
NEVER observed):

| deflection | CUT (x ≤ 0) | COMMON (x ≥ 0) |
|-----------:|:-----------:|:--------------:|
| 0.030 | WT | WT |
| 0.020 | NULL | WT |
| 0.015 | WT | WT |
| 0.012 | WT | WT |
| 0.011 | WT | WT |
| 0.010 | WT | NULL |
| 0.009 | WT | WT |
| 0.008 | WT | WT |
| 0.006 | WT | WT |
| 0.005 | WT | WT |
| 0.004 | NULL | NULL |
| 0.003 | WT | WT |
| 0.002 | WT | NULL |

Reading:

1. **BOTH keep-sides are deflection-fragile** — the CUT is not special. The landed CUT
   host test passes only because it samples 0.01, a point where the CUT weld coincides
   (and where, as it happens, COMMON does not). The fragility is a property of the
   COMPOSITION, not of the COMMON keep-side.
2. **No leak, ever.** At every deflection the mandatory self-verify emits a watertight
   solid OR NULL — never a leaky/partial solid. The honest-decline discipline holds at
   100% of the sweep. This is the load-bearing safety guarantee, and it is TESTED (§5.4).
3. **Root cause.** The M0 mesher position-welds coincident boundaries by vertex identity.
   A shared CURVED edge (the degree-2 Bézier seam and the bowl's quad edges) is tessellated
   INDEPENDENTLY on each of its two incident faces (the B2 sub-face and the neighbour cap /
   wall), so the two vertex samplings coincide — and the faces weld — only at deflections
   where the sampling parameters align. At the fragile deflections one incident face emits
   one extra boundary vertex, opening a hairline crack the self-verify catches. The failing
   boundary edges lie exactly on the bowl surface along a kept quad edge (verified:
   `z = a(x²+y²)` holds at the crack coordinates), i.e. on a curved shared edge.

**This is a PRE-EXISTING M0 fragility EXPOSED by breadth, not introduced by it.** The
enabling fix is **shared-curved-edge single-sampling** in M0: tessellate each shared edge
ONCE and hand the SAME polyline to both incident faces (weld-by-shared-tessellation, not
weld-by-coincidence). That is an M0/edge-mesher enhancement, OUT of scope here (this
change does not touch M0), and is recorded as the next enabler for robust
deflection-independent watertightness of BOTH CUT and COMMON.

## 4. Why COMMON still LANDS at the host gate (honestly)

The gate does not weaken any tolerance and does not cherry-pick a lone success:

- The **partition-closure identity** (§2) is mesh-free and deflection-free — it holds at
  machine precision unconditionally. It is the primary COMMON correctness oracle.
- The **watertight-at-complementary-volume** case is gated at a deflection where the
  shared-edge weld coincides for BOTH keep-sides (the majority regime, e.g. 0.008), so the
  gate is SYMMETRIC — CUT and COMMON are both asserted watertight at the SAME deflection,
  not a per-op fudge. The volume tolerance stays the M2-assembly scale-relative band (2%);
  it is NOT loosened.
- The **fragility is itself a first-class TESTED fact** (§5.4): the sweep asserts the
  never-leak invariant, so the deflection-dependence is documented in code, not hidden.

If a future reader prefers, the partition-closure oracle alone already certifies COMMON's
correctness independent of any mesh; the watertight case adds the topological proof at a
representative resolution.

## 5. Host gate — `test_native_freeform_boolean_breadth.cpp` (NEW, OCCT-free)

- **5.1 `complement_partition_oracle_is_exact`** — `V_below + V_above == V_full` to 1e-12;
  `0 < V_above < V_full`; unit-check `clipXge0` complements `clipXle0` (their polygon areas
  sum to `area(Q)`). Pure closed-form; no mesher.
- **5.2 `common_keep_side_watertight_at_complementary_volume`** — at deflection `d*`
  (a both-sides-weld point), `freeformHalfSpaceCut(A, P, KeepSide::Above)` returns a
  watertight `Solid` whose enclosed volume = `V_above` within the 2% deflection band.
- **5.3 `cut_and_common_partition_the_operand`** — at `d*`, `vol(CUT) + vol(COMMON)` equals
  `V_full` within the (doubled) deflection band; each is watertight — the two complementary
  keep-side solids partition the operand at the mesh level too.
- **5.4 `self_verify_never_emits_a_leak_across_deflections`** — over a deflection sweep,
  for BOTH keep-sides, EVERY result is `isNull()` OR watertight (mesh `isWatertight`); a
  non-watertight solid is NEVER returned. This tests the honest-decline discipline directly
  and documents the shared-curved-edge weld fragility as a measured, first-class fact.

The test builds only under `CYBERCAD_HAS_NUMSCI` (it traces the real M1 seam through
`freeformHalfSpaceCut`), matching the M2-assembly CUT gate. It is wired into CTest
additively; the M2-assembly CUT test is unchanged.

## 6. FUSE — the next breadth blocker (diagnosis, no code)

FUSE with an INFINITE half-space is ill-defined (unbounded), so the honest FUSE target is
FUSE with a FINITE cutter — a box or a second prism. That is a **TWO-operand** boolean
`A ∪ B`, and the landed machinery is single-operand by construction:

- **B1** admits ONE operand and hands out ITS faces; there is no verb that intersects two
  solids' boundaries.
- **M1** traces ONE surface∩surface seam; a box FUSE needs up to SIX plane∩(bowl+walls)
  seams, plus edge∩face crossings, assembled into a consistent intersection graph.
- **B2** splits ONE face along ONE seam; FUSE needs BOTH operands' crossed faces split
  along MULTIPLE seams.
- **B4** caps ONE planar section of ONE operand; FUSE has NO new section to cap — instead
  it must CLASSIFY each operand's faces as inside/outside the other and WELD the surviving
  faces from BOTH operands along shared intersection curves (a two-operand face
  classification + merge), a verb that does not exist in the M2 substrate.

So FUSE is NOT robustly reachable this wave and is DECLINED as the next breadth blocker.
The measured gap: even the SINGLE-operand, single-plane COMMON sits at the edge of the M0
weld's robustness (§3); a two-operand FUSE multiplies the shared-curved-edge weld surface
several-fold. The next enabler is a **two-operand inter-solid intersection verb**
(multi-seam SSI + two-operand face classification/merge), on top of the shared-edge
single-sampling M0 fix. This is recorded, not stubbed — no partial FUSE path is written.

## 7. Additivity, complexity, gates

- NO source under `src/native/**` changes: `half_space_cut.h` (and B1/B2/B3/M0/M1) are
  consumed byte-identical; the `KeepSide::Above` path already exists. 0 OCCT includes under
  `src/native/**`; no `cc_*` change. The M2-assembly change, its fixture, and its CUT test
  are untouched — this change adds its OWN fixture-extension header and test.
- No new production function is added, so the backend cognitive-complexity band is
  unaffected; the test/fixture helpers (`clipXge0`, the sweep loop) are trivial.
- **HOST ANALYTIC gate** (no OCCT): §5. **SIM native-vs-OCCT gate**
  (`BRepAlgoAPI_Common`): a DOCUMENTED follow-up (separate simulator harness), matching the
  CUT slice's host-first landing.
