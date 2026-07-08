# native-construction

## ADDED Requirements

### Requirement: Native affine transforms for a native body

The `NativeEngine` SHALL apply the rigid/affine transform ops — `translate_shape`,
`rotate_shape_about`, `mirror_shape`, `scale_shape`, `scale_shape_about`, and
`place_on_frame` (the ops behind `cc_translate_shape` / `cc_rotate_shape_about` /
`cc_mirror_shape` / `cc_scale_shape` / `cc_scale_shape_about` / `cc_place_on_frame`) —
NATIVELY when handed a NATIVE body, by composing a `math::Transform` onto the body via
`topology::Shape::located(Location{math::Transform})` (B-rep body) or by transforming
the mesh vertices/normals (imported mesh body). The result SHALL be a new native body.
A NON-native (OCCT-built) body SHALL forward to the OCCT adapter unchanged; a native
body SHALL NEVER be forwarded to OCCT (its type-erased handle would be misread).

The transform linear part SHALL match the OCCT adapter semantics: translation
`v'=v+t`; rotation about a normalized axis through a centre; uniform scale about a
centre; mirror reflection `L = I − 2 u uᵀ` (unit plane normal `u`), `t = 2(p·u)u`; and
`place_on_frame` a rigid motion whose linear part's columns are the destination frame
axes `(x-dir = u, main/z-dir = u×v, y-dir = (u×v)×u)` with translation the frame origin.

A MIRROR SHALL flip the body's orientation/handedness (the signed enclosed-volume sign)
while leaving a VALID, watertight, positive-|volume| solid.

#### Scenario: A native box is translated/rotated/scaled to the exact affine image (host)

- GIVEN the native engine active (`cc_set_engine(1)`) and a natively built 10×10×10 box
- WHEN it is transformed by `cc_translate_shape` / `cc_rotate_shape_about` / `cc_scale_shape` / `cc_scale_shape_about` / `cc_place_on_frame`
- THEN the transformed body SHALL be valid with volume = |det L|·1000, and its bounding box and centroid SHALL equal the exact affine image of the box (a rigid motion preserves the volume; a uniform scale by `s` multiplies it by `s³`)

#### Scenario: A native body is mirrored to a valid watertight solid with flipped handedness

- GIVEN the native engine active and a natively built solid
- WHEN it is mirrored by `cc_mirror_shape` across a plane
- THEN the result SHALL be a valid, watertight, positive-|volume| solid whose |volume| equals the base |volume| AND whose signed enclosed-volume sign is flipped relative to the base (handedness reversed)

#### Scenario: A non-native body forwards to OCCT unchanged

- GIVEN the OCCT engine active (`cc_set_engine(0)`) OR an OCCT-built body under the native engine
- WHEN a transform op is invoked
- THEN the result SHALL be identical to the OCCT adapter's transform of that body (no native interception)

### Requirement: Honest decline of a degenerate native transform

A native transform SHALL honestly decline — return a clean error WITHOUT forwarding the
native body to OCCT — when its linear part is NON-invertible (a zero / degenerate
uniform scale) or its parameters are degenerate (a zero rotation axis, a zero mirror
plane normal, or a `place_on_frame` frame whose `u`, `v`, or `u×v` is ~0). A B-rep
result that does not self-verify robustly watertight with positive |volume| SHALL
likewise decline rather than ship a leaky solid. The op SHALL NOT fabricate, collapse,
or widen a tolerance to force a result.

#### Scenario: A zero/degenerate scale of a native body declines (host + parity)

- GIVEN the native engine active and a native body
- WHEN `cc_scale_shape` / `cc_scale_shape_about` is invoked with factor `0` (or non-positive)
- THEN the call SHALL return `0` (a clean decline), matching the OCCT engine's own rejection of a non-positive factor, with the native body NOT forwarded to OCCT

#### Scenario: A degenerate transform parameter declines

- GIVEN the native engine active and a native body
- WHEN `cc_rotate_shape_about` is invoked with a zero axis, `cc_mirror_shape` with a zero normal, or `cc_place_on_frame` with `u ∥ v`
- THEN the call SHALL return `0` (a clean decline)

### Requirement: Native legacy mesh extrude attempts the native prism first

The legacy mesh extrude `extrude_mesh` (behind `cc_extrude`) SHALL attempt the native
prism builder (`build_prism`, the same builder `solid_extrude` uses) FIRST and, on a
non-null solid, return its tessellation at the OCCT adapter's legacy `0.1` deflection.
A NULL solid (a degenerate profile or a case the native builder defers) SHALL fall
through to the fallback engine's `extrude_mesh` with the SAME arguments. Cases the
native builder handles SHALL produce a mesh with extents identical to the OCCT prism.

#### Scenario: cc_extrude builds the prism mesh natively (host)

- GIVEN the native engine active and a closed planar profile the native builder handles
- WHEN `cc_extrude` is invoked
- THEN it SHALL return a non-empty mesh whose bounding box equals the profile extents extruded by `depth`

#### Scenario: cc_extrude of a degenerate profile falls through

- GIVEN the native engine active and a profile with fewer than 3 points
- WHEN `cc_extrude` is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to the fallback engine's `extrude_mesh` (identical to `cc_set_engine(0)`), never faking a mesh

## MODIFIED Requirements

### Requirement: Deferred construction ops fall through to OCCT (honest scope)

The following construction operations SHALL fall through the `NativeEngine` to the fallback
(OCCT) engine: `wrap_emboss`. (`solid_sweep` — for a straight / smooth-planar / non-planar
(RMF) spine — `solid_loft` / `solid_loft_wires` — for a 2- or N-section EQUAL-count OR
MISMATCHED-count planar chain — `tapered_shank`, and the well-formed `helical_thread` /
`tapered_thread` are NOW native where their result self-verifies as a watertight, oracle-correct
solid; see the native residual requirements. `twisted_sweep` with a REAL twist/scale,
`guided_sweep` for the SCALE constraint (native) and its ORIENTATION extension, and
`loft_along_rail` were ATTEMPTED but land natively only where they self-verify oracle-correct and
otherwise fall through per the sub-cases below.) These construction ops are **attempted
natively** and are native ONLY where the result is verified a valid watertight solid with the
correct volume/geometry on BOTH gates; OTHERWISE they SHALL fall through to OCCT (labelled,
verified, never faked). In addition, these **sub-cases** SHALL fall through (the native builder
returns a NULL `Shape`, or the `NativeEngine` self-verify DISCARDS the candidate, and the engine
delegates):

- **Surface–surface intersection (SSI) — Tier 4, only the named slices attempted.** A
  **self-intersecting sweep** (the swept surface folds through itself), a **tight-curvature
  spine** whose section folds (curvature radius below the section's radial extent × the applied
  scale), a **hard pipe-shell rail** that cannot close without trimming two swept surfaces, and a
  **self-intersecting thread whose flanks cross in 3D beyond the robust crossing-flank slice**
  (the crest of one turn passes the next and the trimmed solid does not self-verify watertight +
  correct volume) SHALL fall through. This change attempts SSI ONLY for the named narrow slices
  (the crossing-flank thread, `CYBERCAD_HAS_NUMSCI`-gated) and declines the rest.
- **Sweeps:** a `twisted_sweep` beyond the twist/scale envelope (accumulated twist × section
  radius exceeding the station spacing so adjacent rings interpenetrate) and a self-crossing
  spine SHALL fall through. An ORIENTATION-constraining `guided_sweep` outside the reproducible
  planar slice — a non-planar spine or guide, an accumulated-twist guide, a `guide − path`
  parallel to the tangent, or (the realistic case) no real orientation oracle behind the fixed
  scale-splay `cc_guided_sweep` semantics — SHALL fall through (an honest decline, no dead code).
- **Loft:** a chain with a **NON-PLANAR end section**, a **punctual section**, a section with
  **< 3 distinct points**, or a **resampled stacked skin that self-intersects** SHALL fall
  through. A chain with **mismatched vertex counts** is NOW native via the arc-length vertex
  correspondence (an M-gon lofts to an N-gon, geometry-preserving, self-verified) — it is no
  longer in the fall-through-only list.
- **Profile ops:** a **spline-revolve** (a kind-3 segment in `solid_revolve_profile`, or a
  general B-spline surface of revolution — NOT attempted) and a **non-planar / self-
  intersecting spline** loop in `solid_extrude_profile` /
  `solid_extrude_profile_polyholes` SHALL fall through. (A kind-3 spline edge in a PLANAR
  extrude and a kind-1 off-axis-arc → TORUS revolve are native.)
- **Threads:** a thread requiring the **round-profile fallback** (the native builder returns
  NULL and OCCT applies that fallback — the native path does not fake it) SHALL fall through; the
  tapered tip-crossing and degenerate guards are unchanged, and a **crossing-flank fine-pitch
  thread beyond the robust slice** falls through (the `kMaxLeadRatio` fold guard in force).

The rigid/affine TRANSFORM ops (`translate_shape`, `rotate_shape_about`, `mirror_shape`,
`scale_shape`, `scale_shape_about`, `place_on_frame`) and the legacy mesh extrude
(`extrude_mesh`) are NO LONGER unconditional fall-throughs for a native body — they are
NOW native (see the M-TX requirements above), declining honestly on a degenerate input.
Likewise every feature / boolean / tessellate-of-a-foreign-body / query /
exchange op SHALL fall through. The change SHALL NOT fake, stub-out, or partially implement
any deferred op or sub-case, and SHALL attempt SSI ONLY for the named narrow slices; each
deferred call SHALL produce exactly the fallback engine's result.

#### Scenario: A deferred construction op yields the OCCT result through the native engine
- GIVEN the native engine active (`cc_set_engine(1)`) on an OCCT build
- WHEN a deferred op is invoked (e.g. `cc_wrap_emboss`)
- THEN the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: An SSI sub-case outside the named slices falls through
- GIVEN the native engine active and an input whose valid B-rep would require surface–surface intersection outside the named slices (a self-intersecting sweep, a tight-curvature fold, a hard pipe-shell rail, or a self-intersecting thread whose trimmed solid does not self-verify)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return NULL (or the self-verify DISCARDS the candidate) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`), with no unverified SSI kept

#### Scenario: A deferred loft sub-case falls through
- GIVEN the native engine active and a loft chain with a NON-PLANAR end section, a punctual section, or a resampled skin that self-intersects (a MISMATCHED vertex count alone is NOT deferred — it is native via the correspondence)
- WHEN `cc_solid_loft` / `cc_solid_loft_wires` is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred guided-sweep sub-case falls through
- GIVEN the native engine active and an orientation-constraining `cc_guided_sweep` outside the reproducible planar slice, or with no real orientation oracle behind the entry
- WHEN `cc_guided_sweep` is invoked
- THEN the native builder SHALL return NULL (an honest decline, no dead code) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred profile sub-case falls through
- GIVEN the native engine active and an input hitting a deferred profile sub-case (a spline-revolve, or a non-planar / self-intersecting spline extrude loop)
- WHEN the corresponding `cc_solid_revolve_profile` / `cc_solid_extrude_profile` / `cc_solid_extrude_profile_polyholes` call is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred thread sub-case falls through
- GIVEN the native engine active and a thread requiring the round-profile fallback OR a self-intersecting fine-pitch thread whose flanks cross beyond the robust crossing-flank slice
- WHEN `cc_helical_thread` / `cc_tapered_thread` is invoked
- THEN the native builder SHALL return NULL (no faked round profile, no unverified SSI) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)
