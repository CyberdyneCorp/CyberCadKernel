# Tasks — nurbs-fillet-l4-freeform-g2

## 1. Freeform surface differential geometry
- [x] 1.1 `ffdetail::Surface` view over a NURBS grid + degrees + knots (+ optional weights)
- [x] 1.2 `localGeom` — point, unit normal, first (E,F,G) and second (L,M,N) fundamental forms via `nurbsSurfaceDerivs` order 2
- [x] 1.3 `normalCurvatureAlong` — normal curvature in a given tangent direction (metric projection + II/I)

## 2. Rolling-ball centre-locus seating (the marching primitive)
- [x] 2.1 `footpoint` — Gauss–Newton projection of a point onto a surface (domain-padded, honest decline off-domain)
- [x] 2.2 `seatCenter` — Newton on the ball centre in `span{n̂A,n̂B}` driving both footpoint distances to `r`; honest decline on divergence / (anti)parallel normals
- [x] 2.3 spine march — step the centre along `n̂A × n̂B` (spineDir-oriented), warm-start footpoints across stations

## 3. G2 quintic cross-section with freeform end curvatures
- [x] 3.1 `freeformSection` — poles P0=pA, P5=pB, end tangents in the face tangent planes, offsets `q_i = (5/4)·κ_i·h²`
- [x] 3.2 `quinticPoint` / `sectionCurvature` / `sectionTangent` — evaluators + the G1/G2 witnesses (hodographs)
- [x] 3.3 simplicity guard — decline a folded (non-monotone-along-chord) section

## 4. Skin + public API
- [x] 4.1 `FreeformFilletSeed` / `FreeformFilletResult` / `FreeformFilletDecline`
- [x] 4.2 `fillet_edge_g2_freeform` — march + seat + section + skin, whole-fillet honest decline on any failure
- [x] 4.3 add header to `native_blend.h`

## 5. Tests + wiring
- [x] 5.1 analytic-reduction gate (plane κ=0 ≤1e-9, sphere κ=1/R ≤1e-9)
- [x] 5.2 G2-to-face gate (G1 tangent ≤1e-6 rad, curvature match rel ≤1e-4)
- [x] 5.3 genuinely-freeform bump gate + over-radius honest-decline
- [x] 5.4 wire `test_native_fillet_g2_freeform` into CMake (CYBERCAD_HAS_NUMSCI block + macro def)

## 6. Invariants
- [x] 6.1 `src/native/**` OCCT-free (0 OCCT/Geom/BRep/TK refs in changed files)
- [x] 6.2 `cc_*` ABI byte-unchanged (additive only)
- [x] 6.3 no widened tolerance — honest decline where the ball won't fit / curvature can't match
