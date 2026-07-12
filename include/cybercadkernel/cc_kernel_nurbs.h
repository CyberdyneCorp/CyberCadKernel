#ifndef CYBERCADKERNEL_CC_KERNEL_NURBS_H
#define CYBERCADKERNEL_CC_KERNEL_NURBS_H

/*
 * CyberCadKernel public C ABI — additive NURBS geometry surface (cc_curve /
 * cc_surface).
 *
 * This header is ADDITIVE. It is included by cc_kernel.h and introduces NO change
 * to any existing cc_* symbol or POD struct; the app's header set becomes a strict
 * SUPERSET, never a modification (see openspec/changes/expose-nurbs-cc-facade —
 * design.md §1, §6). It exposes the Wave D–I exact-NURBS geometry across the plain-C
 * boundary the same way cc_kernel.h exposes solids:
 *
 *   - a NURBS curve / surface is an OPAQUE integer handle (cc_curve / cc_surface),
 *     registry-backed exactly like CCShapeId — no C++ / engine type ever crosses;
 *   - its data is read back through small POD header structs (CCCurveInfo /
 *     CCSurfaceInfo) plus buffer-fill accessor calls sized from the header;
 *   - poles cross ALWAYS in HOMOGENEOUS form (x, y, z, w) so the rational and
 *     non-rational cases are uniform (a non-rational curve/surface reports
 *     rational = 0 with every w = 1);
 *   - honest-decline is a 0 (invalid) handle + cc_last_error, NEVER a
 *     plausible-but-wrong handle (the ABI-level DISAGREED = 0 discipline).
 *
 * Knot convention: FLAT knot vector (knots repeated by multiplicity), length
 * n_knots = n_ctrl + degree + 1 — the same internal representation the native
 * modules and OCCT's BSplCLib use. Surface poles/weights are ROW-MAJOR with U outer
 * (pole(i,j) at linear index i*n_ctrl_v + j, i over U, j over V).
 *
 * DISPLAY TESSELLATION CAVEAT (design.md §4). cc_surface_tessellate produces a
 * SINGLE-SURFACE DISPLAY mesh for visualization only. It is EXPLICITLY NOT the
 * watertight multi-face curved-seam weld that gates a sewn B-rep solid (the frozen
 * mesher wall). The returned CCMesh is renderable but carries no watertightness
 * guarantee across multiple faces; a multi-face assembly must be shown as a face set.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Opaque handle types (4-byte, registry-backed like CCShapeId) ────────────── */

/* A NURBS curve handle. id == 0 is the invalid / not-built sentinel. */
typedef struct {
    int32_t id;
} cc_curve;

/* A NURBS surface handle. id == 0 is the invalid / not-built sentinel. */
typedef struct {
    int32_t id;
} cc_surface;

/* ── POD accessor header structs (read-back, design.md §2) ───────────────────── */

/* Header for a cc_curve, filled by cc_curve_info. n_knots == n_ctrl + degree + 1.
 * rational is 0 when every weight is 1 (accessors still emit w = 1), 1 otherwise. */
typedef struct {
    int32_t degree;
    int32_t n_ctrl;   /* control-point count */
    int32_t n_knots;  /* = n_ctrl + degree + 1 */
    int32_t rational; /* 0 = weights all 1, 1 = rational */
} CCCurveInfo;

/* Header for a cc_surface, filled by cc_surface_info. n_knots_u == n_ctrl_u +
 * degree_u + 1 (and likewise for v). Poles are row-major, U outer:
 * total pole count = n_ctrl_u * n_ctrl_v. */
typedef struct {
    int32_t degree_u, degree_v;
    int32_t n_ctrl_u, n_ctrl_v;
    int32_t n_knots_u, n_knots_v;
    int32_t rational; /* 0 = weights all 1, 1 = rational */
} CCSurfaceInfo;

/* Options for cc_surface_tessellate (single-surface DISPLAY mesh). n_u / n_v are
 * the sample counts along the U / V parameter directions (each clamped to >= 2);
 * the mesh is an (n_u-1) * (n_v-1) * 2 triangle grid over the surface domain. A
 * value <= 0 falls back to a built-in default (16). */
typedef struct {
    int32_t n_u;
    int32_t n_v;
} CCTessOptions;

/* --- J5: intersection + trim boolean --------------------------------------- */
/* (Append-only additive section — see openspec/changes/expose-nurbs-cc-facade,
 * design.md §5 "Intersection" + "Trimmed B-rep" rows, §3 honest-decline error
 * model. All symbols below are NEW; nothing above this line changes. These wrap
 * the OCCT-free native intersectors (src/native/math/bspline_intersect.h) and the
 * parameter-space region boolean (src/native/topology/trim_boolean.h).) */

/* One curve<->curve intersection point. `xyz` is the 3-D meeting point; `tA` /
 * `tB` are the parameters on curve A / B; `tangential` is 0 for a TRANSVERSAL
 * (clean) crossing and 1 for a TANGENTIAL contact (parallel tangents). */
typedef struct {
    double xyz[3];
    double tA;
    double tB;
    int32_t tangential;
} CCCurveHit;

/* One curve<->surface intersection (pierce) point. `xyz` is the 3-D point; `t` is
 * the curve parameter; `u` / `v` are the surface parameters; `tangential` is 0 for
 * a transversal pierce, 1 when the curve is tangent to the surface at the hit. */
typedef struct {
    double xyz[3];
    double t;
    double u;
    double v;
    int32_t tangential;
} CCCurveSurfaceHit;

/* The 2-D region boolean operator (mirrors the native TrimBoolOp). */
typedef enum {
    CC_TRIM_UNION = 0,      /* A ∪ B */
    CC_TRIM_INTERSECT = 1,  /* A ∩ B */
    CC_TRIM_DIFFERENCE = 2  /* A ∖ B */
} CCTrimBoolOp;

/* One result loop of a trim-region boolean, as a closed UV polyline. `uv` is
 * `pointCount` (u, v) PAIRS (length 2*pointCount doubles, owned). Orientation is
 * encoded in `outer` (1 = CCW outer boundary, 0 = CW hole) and in the sign of
 * `signedArea` (> 0 CCW, < 0 CW). Free the whole array with
 * cc_nurbs_trim_loops_free. */
typedef struct {
    double* uv;
    int32_t pointCount;
    int32_t outer;
    double signedArea;
} CCTrimLoop;

#ifndef CC_KERNEL_NO_PROTOTYPES

/* ── Construction (registry-backed; honest-decline -> id 0 + cc_last_error) ──── */

/* Build a NURBS curve from raw data. `degree` >= 1. `polesXYZW` is n_ctrl
 * HOMOGENEOUS control points packed (x, y, z, w) — 4*n_ctrl doubles, every w > 0.
 * `knots` is the FLAT knot vector, length n_knots == n_ctrl + degree + 1, non-
 * decreasing. Returns a valid handle, or a 0-handle with cc_last_error set on
 * invalid input (bad degree, count mismatch, a non-positive weight, a non-monotone
 * knot vector). A curve whose weights are ALL 1 is stored non-rational (reported
 * rational = 0). */
cc_curve cc_curve_create(int degree, const double* polesXYZW, int n_ctrl,
                         const double* knots, int n_knots);

/* Build a NURBS (tensor-product) surface from raw data. `degreeU`/`degreeV` >= 1.
 * `polesXYZW` is (n_ctrl_u * n_ctrl_v) HOMOGENEOUS poles (x, y, z, w), ROW-MAJOR
 * with U outer (index i*n_ctrl_v + j) — 4*n_ctrl_u*n_ctrl_v doubles, every w > 0.
 * `knotsU` (length n_ctrl_u + degreeU + 1) and `knotsV` (length n_ctrl_v +
 * degreeV + 1) are FLAT, non-decreasing. Returns a valid handle, or a 0-handle
 * with cc_last_error set on invalid input. All-1 weights are stored non-rational. */
cc_surface cc_surface_create(int degreeU, int degreeV, const double* polesXYZW,
                             int n_ctrl_u, int n_ctrl_v, const double* knotsU,
                             int n_knots_u, const double* knotsV, int n_knots_v);

/* Release a NURBS curve / surface handle. Idempotent and crash-free on a double
 * release or an unknown / stale handle (mirrors cc_shape_release). */
void cc_curve_release(cc_curve h);
void cc_surface_release(cc_surface h);

/* ── Accessors (POD header + buffer-fill; design.md §2) ──────────────────────── */

/* Fill *out with the curve header. Returns 1 on success, 0 on an unknown / stale
 * handle or null out (cc_last_error set). */
int cc_curve_info(cc_curve h, CCCurveInfo* out);
int cc_surface_info(cc_surface h, CCSurfaceInfo* out);

/* Copy the FLAT knot vector into the caller buffer `out` (capacity `cap` doubles).
 * Returns the number of doubles written (n_knots), or < 0 if `cap` is too small
 * (nothing is written out of bounds), or < 0 on an unknown handle. */
int cc_curve_knots(cc_curve h, double* out, int cap);

/* Copy the HOMOGENEOUS poles (x, y, z, w per control point) into `out` (capacity
 * `cap` doubles). Writes 4*n_ctrl doubles. Non-rational -> every w = 1. Returns the
 * number of doubles written, or < 0 if `cap` is too small / unknown handle. */
int cc_curve_poles(cc_curve h, double* out, int cap);

/* Surface knot / pole accessors, same contract as the curve accessors. Poles are
 * ROW-MAJOR with U outer (pole(i,j) at 4*(i*n_ctrl_v + j)); 4*n_ctrl_u*n_ctrl_v
 * doubles total. */
int cc_surface_knots_u(cc_surface h, double* out, int cap);
int cc_surface_knots_v(cc_surface h, double* out, int cap);
int cc_surface_poles(cc_surface h, double* out, int cap);

/* ── Evaluators (point on the exact rational geometry) ───────────────────────── */

/* Evaluate the curve at parameter `t`, writing the point (x, y, z) into `xyz`
 * (3 doubles). Returns 1 on success, 0 on an unknown handle / null xyz. `t` is
 * clamped to the curve's knot domain. */
int cc_curve_eval(cc_curve h, double t, double* xyz);

/* Evaluate the surface at parameters (u, v), writing (x, y, z) into `xyz`
 * (3 doubles). Returns 1 on success, 0 on an unknown handle / null xyz. (u, v) are
 * clamped to the surface's knot domain. */
int cc_surface_eval(cc_surface h, double u, double v, double* xyz);

/* ── Display tessellation bridge (design.md §4 — DISPLAY ONLY) ───────────────── */

/* Tessellate one NURBS surface into a DISPLAY CCMesh (triangle grid over the (u,v)
 * domain), filling *out (owned by the caller; free with cc_mesh_free). `opt` may be
 * null (built-in default sampling). Returns 1 on success, 0 on an unknown handle /
 * null out (cc_last_error set; *out zeroed).
 *
 * CAVEAT (see the file header): this is a SINGLE-SURFACE display mesh for
 * visualization. It is NOT the watertight multi-face curved-seam weld — no
 * cross-face watertightness is claimed. A closed single surface (e.g. a revolved
 * sphere) tessellates to a renderable, shading-usable mesh; a multi-face assembly
 * must be shown as a set of such meshes, not a sewn solid. */
int cc_surface_tessellate(cc_surface h, const CCTessOptions* opt, CCMesh* out);

/* Sample the curve into a display polyline of `n_samples` points (clamped to >= 2)
 * over its knot domain, filling *out (a CCEdgePolyline with edgeId 0). The caller
 * owns out->points and frees it with cc_points_free(out->points); *out itself is
 * caller storage (do NOT pass it to cc_edge_polylines_free, which frees a heap
 * ARRAY). Returns 1 on success, 0 on an unknown handle / null out (cc_last_error
 * set; *out zeroed). */
int cc_curve_polyline(cc_curve h, int n_samples, CCEdgePolyline* out);

/* --- J5: intersection + trim boolean --------------------------------------- */

/* Intersect two NURBS curves. On success returns the number of isolated hit
 * points (>= 0) and, when `outHits` is non-null AND there is >= 1 hit, allocates a
 * C-owned CCCurveHit array into *outHits (free with cc_nurbs_hits_cc_free). On an
 * HONEST DECLINE — an unknown handle, or COINCIDENT / OVERLAPPING curves (an
 * infinite intersection set, never faked as points) — returns < 0, sets
 * cc_last_error, and writes *outHits = NULL. `tol` <= 0 selects the native
 * default. A count of 0 means the curves are disjoint (valid, not an error). */
int cc_nurbs_intersect_cc(cc_curve a, cc_curve b, double tol, CCCurveHit** outHits);
void cc_nurbs_hits_cc_free(CCCurveHit* hits);

/* Intersect a NURBS curve with a NURBS surface (every pierce point). Same contract
 * as cc_nurbs_intersect_cc: returns the hit count (>= 0) filling *outHits (owned;
 * free with cc_nurbs_hits_cs_free); a curve lying ON the surface over a sub-arc is
 * HONEST-DECLINED (returns < 0, *outHits = NULL, cc_last_error set). */
int cc_nurbs_intersect_cs(cc_curve c, cc_surface s, double tol,
                          CCCurveSurfaceHit** outHits);
void cc_nurbs_hits_cs_free(CCCurveSurfaceHit* hits);

/* Parameter-space trim-region boolean (design.md §5 "Trimmed B-rep"). Each region
 * is an array of loop cc_curves LIVING IN THE SAME SURFACE (u,v) DOMAIN — the
 * curve's (x, y) are read as (u, v). Loop index 0 is the OUTER loop; indices
 * 1..n-1 are HOLE loops. `op` selects Union / Intersect / Difference.
 *
 * On success returns the number of result loops (>= 0), allocates a C-owned
 * CCTrimLoop array into *outLoops (when non-null and there is >= 1 loop; free with
 * cc_nurbs_trim_loops_free), and writes the region's total signed area into *area
 * (may be null). A count of 0 with area 0 is a valid EMPTY region (e.g. Intersect
 * of disjoint inputs). On an HONEST DECLINE — an unknown handle, a malformed loop,
 * or a COINCIDENT-BOUNDARY / TANGENTIAL-only overlap (ambiguous for a region
 * boolean, never resolved into a fabricated region) — returns < 0, sets
 * cc_last_error, writes *outLoops = NULL and *area = 0. */
int cc_nurbs_trim_region_boolean(const cc_curve* regionA, int nLoopsA,
                                 const cc_curve* regionB, int nLoopsB, CCTrimBoolOp op,
                                 CCTrimLoop** outLoops, double* area);
void cc_nurbs_trim_loops_free(CCTrimLoop* loops, int count);
/* --- J4: blend + offset/thicken --- (APPEND-ONLY; do not reorder above) ─────────
 * Wave-J track J4 wrappers over the OCCT-free native blend + offset/thicken/shell
 * modules (src/native/blend, src/native/math/bspline_offset|thicken|shell). Every
 * wrapper is a thin, guarded delegation that validates its raw-C input, drives the
 * native representation, and honest-DECLINES on any genuine geometric failure — a
 * SURFACE-producing wrapper returns a 0 (invalid) cc_surface + cc_last_error, a
 * SOLID-producing wrapper returns 0 (and zeroes *out) + cc_last_error. The
 * self-intersection-free / over-radius-fillet / fully-folding-thicken cases DECLINE;
 * a self-intersecting or folded result is NEVER emitted (design.md §3, tasks.md §4).
 * The Offset/thicken/shell wrappers compose the numsci-gated Layer-5 fit; when the
 * kernel is built WITHOUT CYBERCAD_HAS_NUMSCI they honest-decline (0 / *out zeroed +
 * cc_last_error "…requires the numsci substrate"). The blend wrappers are always
 * available (their native modules are header-only over math + vec).
 * SURFACE-producing wrappers register their result as a cc_surface: a bilinear/
 * de-tensored tensor-product NURBS surface that reproduces the native band/patch grid
 * exactly (a ruled bevel is degree-1×1; a freeform-fillet skin / vertex-blend corner
 * is the sampled station×section rim grid as a degree-1×1 net). This is the DISPLAY
 * geometry crossing the boundary — the same single-surface, non-watertight caveat as
 * cc_surface_tessellate applies. SOLID-producing wrappers fill a CCMesh (owned by the
 * caller; free with cc_mesh_free) carrying the native watertight closed shell.
 */
/* FREEFORM G2 (curvature-continuous) rolling-ball fillet between two freeform cc_surface
 * faces at radius `r`, seeded by a rolling-ball centre guess + a spine-run direction.
 * `center0` / `spineDir` are 3 doubles each (x,y,z). `sA`/`sB` (±1) pick which side of
 * each face the ball centre lies on; `stepLen` is the centre advance per station;
 * `nStations` (≥2) is the number of spine stations; `nSectionSamples` (≥1) is the
 * cross-section sampling. Returns a cc_surface (the skinned fillet band, sampled to a
 * tensor grid), or a 0-handle + cc_last_error on decline (ball won't fit / over-radius /
 * section fold / too few stations). NEVER emits a self-intersecting fillet. */
cc_surface cc_nurbs_fillet_freeform_g2(cc_surface faceA, cc_surface faceB, double r,
                                       const double* center0, const double* spineDir,
                                       double sA, double sB, double stepLen, int nStations,
                                       int nSectionSamples);
/* SETBACK VERTEX BLEND: fill the N-sided corner where `nFillets` (N ≥ 3) incident fillet
 * cc_surfaces meet at a shared vertex. `fillets` is an array of N cc_surface handles;
 * `sides` is N ints (0=U0,1=U1,2=V0,3=V1 — which iso-edge of each fillet faces the gap);
 * `setbacks` is N doubles ∈ [0,1) (trim each fillet back before taking the gap curve);
 * `reverses` is N ints (0/1 — reverse the extracted gap curve for loop ordering); `mode`
 * is 1 for G1, 2 for G2. Writes the N produced corner sub-patches into `outPatches`
 * (capacity `cap` cc_surface handles) and returns the patch count, or < 0 on decline
 * (N<3, rational/malformed fillet, non-closed gap loop, G1/G2-infeasible corner, or the
 * measured blend↔fillet normal residual exceeds the G1 gate) with cc_last_error set. On
 * a too-small `cap` returns < 0 and registers nothing. */
int cc_nurbs_vertex_blend(const cc_surface* fillets, int nFillets, const int* sides,
                          const double* setbacks, const int* reverses, int mode,
                          cc_surface* outPatches, int cap);
/* VARIABLE-DISTANCE analytic chamfer along an edge shared by two ANALYTIC faces. `edge`
 * is `nStations` (≥2) stations packed 12 doubles each: (px,py,pz, tx,ty,tz, nAx,nAy,nAz,
 * nBx,nBy,nBz) = point, unit edge tangent, faceA outward normal, faceB outward normal.
 * `subA`/`subB` are analytic substrates packed 11 doubles each: (kind, pointX,Y,Z,
 * normalX,Y,Z, axisX,Y,Z is packed as normal for a plane; radius; halfAngle) — see the
 * .cpp packing note; `kind` is 0=Plane,1=Cylinder,2=Cone. The symmetric setback tapers
 * LINEARLY from `d0` (edge start) to `d1` (edge end). Returns the ruled bevel band as a
 * cc_surface (degree-1×1 tensor), or a 0-handle + cc_last_error on decline (over-large
 * setback / degenerate dihedral / unsupported freeform substrate). */
cc_surface cc_nurbs_chamfer_variable(const double* subA, const double* subB,
                                     const double* edge, int nStations, double d0,
                                     double d1);
/* FREEFORM-EDGE chamfer along a crease shared by two FREEFORM cc_surface faces, setback
 * `d` measured ALONG each face's surface (first-fundamental-form march). `edge` is
 * `nStations` (≥2) stations packed 12 doubles each: (px,py,pz, tx,ty,tz, materialHintX,Y,Z,
 * uA0,vA0, uB0,vB0 ...) — see the .cpp packing note (point, unit tangent, material hint,
 * per-face footpoint warm-starts). Returns the ruled bevel band as a cc_surface, or a
 * 0-handle + cc_last_error on decline (footpoint diverged / left domain / self-lap). NEVER
 * emits a self-intersecting bevel. */
cc_surface cc_nurbs_chamfer_freeform(cc_surface faceA, cc_surface faceB, const double* edge,
                                     int nStations, double d);
/* RATIONAL OFFSET of a cc_surface at signed distance `dist`: fits a rational approximant
 * so an exact conic (cylinder r±d, sphere R±d) offsets to the exact offset conic. `tol`
 * is the target fit deviation from the true offset locus (<=0 ⇒ default 1e-4). Returns a
 * cc_surface, or a 0-handle + cc_last_error on decline (degenerate normal, self-intersection
 * past the curvature radius, fit failure, or the numsci substrate is absent). */
cc_surface cc_nurbs_offset_rational(cc_surface h, double dist, double tol);
/* FOLD-TRIMMED OFFSET of a cc_surface at `dist`: when the offset would self-intersect over
 * PART of the domain, trims to the maximal fold-free rectangle and returns a valid offset
 * there. Registers the trimmed offset as a cc_surface (the returned handle) and, when
 * `kept` is non-null, writes the kept parameter rectangle as 4 doubles (u0,u1,v0,v1) in
 * the INPUT surface's (u,v) coords and `*trimmed` (non-null) to 0/1. Returns a 0-handle +
 * cc_last_error when no fold-free region of meaningful area remains. */
cc_surface cc_nurbs_offset_trimmed(cc_surface h, double dist, double tol, double* kept,
                                   int* trimmed);
/* SELF-INTERSECTION-TRIMMED THICKEN of a cc_surface into a CLOSED watertight solid at
 * signed thickness `dist`: when the inward offset would interpenetrate over PART of the
 * domain, trims the folded portion and builds the closed solid over the fold-free region.
 * Fills `out` (a CCMesh owned by the caller; free with cc_mesh_free) with the native
 * watertight closed shell. Returns 1 on success (out carries a closed, positive-volume
 * mesh), or 0 (out zeroed) + cc_last_error on decline (fully folding, degenerate, zero
 * thickness, non-closed, or numsci substrate absent). NEVER emits a self-intersecting
 * solid. When `kept`/`trimmed` are non-null they receive the kept rectangle / trim flag. */
int cc_nurbs_thicken_trimmed(cc_surface h, double dist, double tol, CCMesh* out, double* kept,
                             int* trimmed);
/* SLAB-OVERLAP-TRIMMED MULTI-FACE SHELL: thicken `nFaces` edge-adjacent cc_surface faces
 * by `dist` into ONE closed watertight solid, trimming any overlapping mitre at a sharp
 * dihedral seam to a clean self-intersection-free corner. `faces` is `nFaces` cc_surface
 * handles; `adjacency` is `nEdges` shared-edge records packed 5 ints each: (faceA, faceB,
 * edgeA, edgeB, reversed) with edge ids 0=U0,1=U1,2=V0,3=V1. Fills `out` (a CCMesh owned
 * by the caller; free with cc_mesh_free) with the native watertight closed shell. Returns
 * 1 on success (out carries a closed, self-intersection-free, positive-volume mesh), or 0
 * (out zeroed) + cc_last_error on decline (fold, adjacency mismatch, non-manifold seam,
 * un-trimmable overlap, non-closed, or numsci substrate absent). */
int cc_nurbs_shell_trimmed(const cc_surface* faces, int nFaces, const int* adjacency,
                           int nEdges, double dist, double tol, CCMesh* out);

#endif /* CC_KERNEL_NO_PROTOTYPES */

/* --- J2: fitting / reverse-engineering / analytic conversion --------------- */
/*
 * Additive wrappers over the OCCT-FREE native modules bspline_fit / bspline_fair /
 * bspline_simplify / primitive_fit / analytic_nurbs (bridging lives in
 * src/facade/cc_nurbs_fit.cpp). Every constructor returns a valid cc_curve /
 * cc_surface handle, or the honest decline (id 0 + cc_last_error) — never a
 * plausible-but-wrong handle, never a widened tolerance. The recognition /
 * detection calls fill a small POD result (type enum + params + RMS) and return 1
 * on success, 0 on an unknown handle / null out (cc_last_error set).
 *
 * Point / grid inputs cross as PLAIN packed double arrays (Euclidean x,y,z):
 *   - a point sequence is `pointsXYZ` of 3*n_points doubles;
 *   - a point grid is `gridXYZ` of 3*n_u*n_v doubles, ROW-MAJOR with U outer
 *     (point(i,j) at 3*(i*n_v + j), i over U, j over V) — the same layout as the
 *     surface pole net;
 *   - a per-point / per-grid weight array is `weights` of n_points (or n_u*n_v)
 *     doubles, every weight strictly positive.
 * `param_method` selects the parametrization: 0 = uniform, 1 = chord-length
 * (default), 2 = centripetal; any other value is treated as chord-length.
 */

#ifndef CC_KERNEL_NO_PROTOTYPES

/* ── Fitting / approximation (points -> cc_curve / cc_surface) ─────────────── */

/* Least-squares curve APPROXIMATION: fit a degree-`degree` B-spline with exactly
 * `n_ctrl` control points (degree+1 <= n_ctrl < n_points), endpoints pinned to the
 * first/last data point. `pointsXYZ` is 3*n_points doubles. Honest decline (id 0)
 * on a bad degree / control-point count, too few points, or a degenerate (all-
 * coincident) input. */
cc_curve cc_nurbs_fit_curve(const double* pointsXYZ, int n_points, int degree,
                            int n_ctrl, int param_method);

/* Global curve INTERPOLATION: build a degree-`degree` B-spline through EVERY input
 * point (n_points >= degree+1). Honest decline (id 0) on a bad degree, too few
 * points, or a degenerate input. */
cc_curve cc_nurbs_interp_curve(const double* pointsXYZ, int n_points, int degree,
                               int param_method);

/* Least-squares tensor-product surface APPROXIMATION over an (n_u x n_v) grid
 * (`gridXYZ` row-major, U outer) with an (n_ctrl_u x n_ctrl_v) control net,
 * degrees (degree_u, degree_v). Requires degree+1 <= n_ctrl <= n in each direction.
 * Honest decline (id 0) on bad dimensions / degenerate grid. */
cc_surface cc_nurbs_fit_surface(const double* gridXYZ, int n_u, int n_v, int degree_u,
                                int degree_v, int n_ctrl_u, int n_ctrl_v,
                                int param_method);

/* ── Rational weight ESTIMATION (Ma–Kruth: recover poles AND weights) ─────── */

/* Estimate BOTH the `n_ctrl` control points AND their weights of a degree-`degree`
 * RATIONAL B-spline fitting the data — the weights are UNKNOWN and recovered, not
 * prescribed. Over-determined: needs 3*n_points > 4*n_ctrl and degree+1 <= n_ctrl.
 * Returns a rational cc_curve on success; honest decline (id 0) when the system is
 * under-determined, rank-deficient (no stable weights), or the recovered weights
 * do not all share one sign (an invalid NURBS). A polynomial (all ≈1 weights) fit
 * is still returned (as a non-rational curve). */
cc_curve cc_nurbs_estimate_weights_curve(const double* pointsXYZ, int n_points,
                                         int degree, int n_ctrl, int param_method);

/* Surface analogue: estimate the (n_ctrl_u x n_ctrl_v) control net AND its weights
 * of a degree-(degree_u,degree_v) rational surface over the (n_u x n_v) grid.
 * Needs 3*n_u*n_v > 4*n_ctrl_u*n_ctrl_v and degree+1 <= n_ctrl <= n each way.
 * Same honest-decline contract as the curve estimator. */
cc_surface cc_nurbs_estimate_weights_surface(const double* gridXYZ, int n_u, int n_v,
                                             int degree_u, int degree_v, int n_ctrl_u,
                                             int n_ctrl_v, int param_method);

/* ── Equality-CONSTRAINED least-squares fitting ───────────────────────────── */

/* One curve end constraint for cc_nurbs_fit_curve_constrained: the `order`-th
 * derivative at `end` (end 0 = start/u=0, 1 = end/u=1) equals `value` (3 doubles).
 * order 0 = position, 1 = tangent, 2 = curvature. */
typedef struct {
    int32_t end;   /* 0 = start (u=0), 1 = end (u=1) */
    int32_t order; /* 0 = position, 1 = 1st deriv, 2 = 2nd deriv */
    double value[3];
} CCCurveEndConstraint;

/* Equality-CONSTRAINED least-squares curve fit: fit a degree-`degree`, `n_ctrl`-
 * control-point B-spline minimizing the squared distance to `pointsXYZ` SUBJECT TO
 * every constraint in `constraints` (`n_constraints` of them) holding EXACTLY.
 * Endpoints are NOT auto-pinned — pin them with explicit order-0 constraints.
 * Honest decline (id 0) on a bad degree / count, an OVER-CONSTRAINED system
 * (n_constraints >= n_ctrl), an order > 2 or order > degree constraint, or a
 * rank-deficient / inconsistent (singular-KKT) constraint set. */
cc_curve cc_nurbs_fit_curve_constrained(const double* pointsXYZ, int n_points,
                                        const CCCurveEndConstraint* constraints,
                                        int n_constraints, int degree, int n_ctrl,
                                        int param_method);

/* One surface pole constraint for cc_nurbs_fit_surface_constrained: the fitted net
 * pole at (i,j) (row-major, U outer) equals `value` (3 doubles). Fix a whole
 * boundary row to interpolate an edge curve; add the adjacent row for a G1 stitch;
 * fix a corner pole to pin a corner. */
typedef struct {
    int32_t i; /* U pole index (0..n_ctrl_u-1) */
    int32_t j; /* V pole index (0..n_ctrl_v-1) */
    double value[3];
} CCSurfacePoleConstraint;

/* Equality-CONSTRAINED least-squares tensor-product surface fit over the (n_u x
 * n_v) grid with an (n_ctrl_u x n_ctrl_v) net, degrees (degree_u,degree_v),
 * SUBJECT TO every pole constraint holding EXACTLY. With no constraints this
 * reduces to cc_nurbs_fit_surface. Honest decline (id 0) on bad dimensions, an
 * out-of-net constraint index, or an over-constrained / singular-KKT set. */
cc_surface cc_nurbs_fit_surface_constrained(const double* gridXYZ, int n_u, int n_v,
                                            const CCSurfacePoleConstraint* constraints,
                                            int n_constraints, int degree_u,
                                            int degree_v, int n_ctrl_u, int n_ctrl_v,
                                            int param_method);

/* ── Fairing (minimal-energy smoothing, hard deviation bound) ─────────────── */

/* Fair a B-spline curve: minimize the discrete bending energy over the interior
 * poles subject to the faired curve staying within `tol` of the input everywhere.
 * `keep_ends` != 0 preserves position AND end tangent (two poles per end); == 0
 * preserves only endpoint positions. Returns a NEW faired cc_curve on success.
 * Honest decline (id 0) when `tol` is too tight to reduce energy, the input is
 * rational, or the curve is too small / malformed (no faked smoothing). */
cc_curve cc_nurbs_fair_curve(cc_curve in, double tol, int keep_ends);

/* Fair a tensor-product B-spline surface: minimize the discrete thin-plate energy
 * subject to staying within `tol` of the input everywhere. `keep_boundary` != 0
 * fixes the whole boundary ring; == 0 fixes only the four corners. Returns a NEW
 * faired cc_surface, or the honest decline (id 0) on the same conditions. */
cc_surface cc_nurbs_fair_surface(cc_surface in, double tol, int keep_boundary);

/* ── Simplification (tolerance-bounded knot removal) ──────────────────────── */

/* Bounded knot removal: greedily discard interior knots while the TRUE deviation
 * of the simplified curve from the input stays <= `tol` (measured densely).
 * Returns a NEW cc_curve (== the input geometry when nothing could be removed).
 * Honest decline (id 0) only on an unknown handle / non-finite tol. */
cc_curve cc_nurbs_simplify_curve(cc_curve in, double tol);

/* ── Reverse-engineering: primitive detection / recognition ───────────────── */

/* Detected analytic primitive type (mirrors native PrimitiveType). */
typedef enum {
    CC_PRIM_FREEFORM = 0,
    CC_PRIM_PLANE = 1,
    CC_PRIM_SPHERE = 2,
    CC_PRIM_CYLINDER = 3,
    CC_PRIM_CONE = 4
} CCPrimitiveType;

/* Result of cc_nurbs_detect_primitive on a raw point cloud. Only the fields the
 * `type` selects are meaningful; the rest are 0. `rms` is the achieved RMS of the
 * winning fit (0 for FREEFORM); `rel_error` is rms / cloud-extent. */
typedef struct {
    int32_t type;        /* CCPrimitiveType */
    double rms;
    double rel_error;
    double plane_normal[3];
    double plane_offset; /* n·x = offset */
    double center[3];    /* sphere / cylinder-axis-point / cone-apex */
    double axis[3];      /* cylinder / cone unit axis */
    double radius;       /* sphere / cylinder radius */
    double half_angle;   /* cone half-angle (radians) */
} CCPrimitiveDetection;

/* Try plane / sphere / cylinder / cone against the point cloud `pointsXYZ`
 * (3*n_points doubles) and report the best fit whose RELATIVE RMS is <= `rel_tol`
 * (pass <= 0 for the built-in 1e-6). If none qualifies, `type` is CC_PRIM_FREEFORM
 * (never a spurious primitive). Fills *out and returns 1 on success; returns 0 on
 * a null out / too-few points (cc_last_error set; *out zeroed). */
int cc_nurbs_detect_primitive(const double* pointsXYZ, int n_points, double rel_tol,
                              CCPrimitiveDetection* out);

/* Recognized analytic CURVE kind (mirrors native CurveKind). */
typedef enum {
    CC_CURVE_GENERAL = 0,
    CC_CURVE_LINE = 1,
    CC_CURVE_CIRCLE = 2,
    CC_CURVE_ARC = 3,
    CC_CURVE_ELLIPSE = 4
} CCCurveKind;

/* Result of cc_nurbs_recognize_curve. Only the fields the `kind` selects are
 * meaningful. `residual` is the max control-point algebraic residual verified
 * (<= tol ⇒ exact). For LINE, line_start/line_end are the two endpoints. For
 * CIRCLE/ARC/ELLIPSE, center/normal/x_axis describe the plane frame; radius is the
 * circle radius (or ellipse major); minor_radius is the ellipse semi-minor;
 * start_angle/sweep_angle (radians) describe the arc. */
typedef struct {
    int32_t kind;   /* CCCurveKind */
    double residual;
    double line_start[3];
    double line_end[3];
    double center[3];
    double normal[3];
    double x_axis[3];
    double radius;       /* circle radius / ellipse major radius */
    double minor_radius; /* ellipse semi-minor */
    double start_angle;  /* arc, radians */
    double sweep_angle;  /* arc, radians */
} CCCurveRecognition;

/* Recognize whether the curve `h` is EXACTLY a line / circle / arc / ellipse. The
 * fit is a candidate generator; acceptance requires the control net to satisfy the
 * primitive's implicit equation to <= `tol` (pass <= 0 for the built-in 1e-12),
 * else CC_CURVE_GENERAL. Fills *out, returns 1 on success; 0 on unknown handle /
 * null out (cc_last_error set). */
int cc_nurbs_recognize_curve(cc_curve h, double tol, CCCurveRecognition* out);

/* Recognized analytic SURFACE kind (mirrors native SurfaceKind). */
typedef enum {
    CC_SURF_GENERAL = 0,
    CC_SURF_PLANE = 1,
    CC_SURF_CYLINDER = 2,
    CC_SURF_CONE = 3,
    CC_SURF_SPHERE = 4
} CCSurfaceKind;

/* Result of cc_nurbs_recognize_surface. Only the fields the `kind` selects are
 * meaningful. `origin`/`axis`/`x_axis` give the primitive frame; `radius` is the
 * cylinder / sphere / cone-reference radius; `half_angle` the cone half-angle. */
typedef struct {
    int32_t kind;   /* CCSurfaceKind */
    double residual;
    double origin[3];
    double axis[3];
    double x_axis[3];
    double radius;
    double half_angle; /* cone, radians */
} CCSurfaceRecognition;

/* Recognize whether the surface `h` is EXACTLY a plane / cylinder / cone / sphere,
 * with the same control-net algebraic-exactness certificate (<= `tol`, <= 0 ⇒
 * 1e-12). Fills *out, returns 1 on success; 0 on unknown handle / null out. */
int cc_nurbs_recognize_surface(cc_surface h, double tol, CCSurfaceRecognition* out);

/* ── Analytic -> exact rational NURBS ─────────────────────────────────────── */

/* A full circle: center (3), plane unit normal (3), in-plane unit X axis (3),
 * radius. Built as a piecewise rational-quadratic B-spline (4 quarter segments,
 * 9 poles). Honest decline (id 0) on a non-positive radius or a degenerate
 * normal / x_axis. */
cc_curve cc_nurbs_circle(const double* center, const double* normal,
                         const double* x_axis, double radius);

/* A circular arc: same frame as cc_nurbs_circle plus a start angle and sweep
 * (radians, sweep in (0, 2π]). Honest decline (id 0) on a bad radius / frame or a
 * non-positive / out-of-range sweep. */
cc_curve cc_nurbs_arc(const double* center, const double* normal, const double* x_axis,
                      double radius, double start_angle, double sweep_angle);

/* A full ellipse: center (3), normal (3), major-axis (X) direction (3), and the
 * two semi-axes (major_radius along X, minor_radius). Honest decline (id 0) on a
 * non-positive semi-axis or a degenerate frame. */
cc_curve cc_nurbs_ellipse(const double* center, const double* normal,
                          const double* x_axis, double major_radius,
                          double minor_radius);

/* A finite window [u0,u1] x [v0,v1] of a plane (origin (3), unit normal (3),
 * in-plane X axis (3)) as a bilinear 2x2 patch. Honest decline (id 0) on a
 * degenerate frame or an empty window (u1<=u0 or v1<=v0). */
cc_surface cc_nurbs_plane(const double* origin, const double* normal,
                          const double* x_axis, double u0, double u1, double v0,
                          double v1);

/* A finite-height cylinder [v0,v1] (frame origin (3), axis (3), in-plane X (3),
 * radius) as an exact rational surface of revolution. Honest decline (id 0) on a
 * non-positive radius, a degenerate frame, or an empty height. */
cc_surface cc_nurbs_cylinder(const double* origin, const double* axis,
                             const double* x_axis, double radius, double v0,
                             double v1);

/* A finite-height cone [v0,v1] (frame origin (3), axis (3), in-plane X (3),
 * reference radius at v=0, half-angle `semi_angle` in radians) as an exact
 * rational surface of revolution. Honest decline (id 0) on a degenerate frame or
 * an empty height. */
cc_surface cc_nurbs_cone(const double* origin, const double* axis,
                         const double* x_axis, double radius, double semi_angle,
                         double v0, double v1);

/* A full sphere (center (3), axis (3), in-plane X (3), radius) as an exact
 * rational surface of revolution. Honest decline (id 0) on a non-positive radius
 * or a degenerate frame. */
cc_surface cc_nurbs_sphere(const double* center, const double* axis,
                           const double* x_axis, double radius);

/* A full torus (center (3), axis (3), in-plane X (3), major_radius R, minor_radius
 * r) as an exact rational surface of revolution. Honest decline (id 0) on a non-
 * positive minor radius, a spindle torus (R < r), or a degenerate frame. */
cc_surface cc_nurbs_torus(const double* center, const double* axis,
                          const double* x_axis, double major_radius,
                          double minor_radius);

#endif /* CC_KERNEL_NO_PROTOTYPES */

/* ════════════════════════════════════════════════════════════════════════════
 * --- J3: surfacing ---  (append-only; J3 owns this delimited section)
 *
 * Thin `cc_nurbs_*` wrappers over the OCCT-free native Layer-6 surfacing modules
 * (bspline_skin / gordon / coons / nsided{,_g1,_g2} / sweep / join). Each reads
 * its input cc_curve / cc_surface handles through the PUBLIC J1 accessors, drives
 * the native builder, and REGISTERS results via the public cc_surface_create. No
 * J1-internal registry is touched. Honest-decline is a 0 (invalid) handle (or a
 * count < 0) + cc_last_error — NEVER a plausible-but-wrong handle, NEVER a widened
 * tolerance (a G2-infeasible creased N-gon declines; it is not filled with a
 * residual crease).
 *
 * BUILD GATE: the native surfacing modules compile only under CYBERCAD_HAS_NUMSCI
 * (they compose the numsci-backed skin/fit solves). With that macro OFF every J3
 * wrapper honest-declines ("surfacing requires CYBERCAD_HAS_NUMSCI") — the ABI
 * symbol is always present, only the capability is gated.
 * ════════════════════════════════════════════════════════════════════════════ */

/* N-sided fill continuity mode (see bspline_nsided{,_g1,_g2}.h). RATIONAL is the
 * exact-rational C0 fill (rational boundary arcs reproduced exactly). */
typedef enum {
    CC_NSIDED_C0 = 0,       /* midpoint-subdivision Coons sub-patches (position only) */
    CC_NSIDED_G1 = 1,       /* Gregory bicubic, tangent-plane continuous across spokes */
    CC_NSIDED_G2 = 2,       /* Gregory quintic, curvature continuous across spokes     */
    CC_NSIDED_RATIONAL = 3  /* exact-rational C0 fill (rational arcs exact)            */
} CCNSidedMode;

/* Surface-join continuity mode (see bspline_join.h). */
typedef enum {
    CC_JOIN_G1 = 1,  /* tangent-plane continuity across the shared edge */
    CC_JOIN_G2 = 2   /* curvature continuity across the shared edge      */
} CCJoinMode;

/* Which boundary of a surface is the shared edge for cc_nurbs_join (mirrors the
 * native SurfaceEdge). U0/U1 = u-min/u-max rows; V0/V1 = v-min/v-max columns. */
typedef enum {
    CC_EDGE_U0 = 0,
    CC_EDGE_U1 = 1,
    CC_EDGE_V0 = 2,
    CC_EDGE_V1 = 3
} CCSurfaceEdge;

#ifndef CC_KERNEL_NO_PROTOTYPES

/* SKIN / LOFT — build one tensor-product surface CONTAINING every section cc_curve
 * as an iso-curve. `sections` is `n` cc_curve handles (>= 2); `degreeV` is the
 * across-sections degree (clamped to n-1). If every section is rational the exact
 * RATIONAL skin is used, else the non-rational skin (mixing rational and non-
 * rational sections declines). Returns the skinned cc_surface, or a 0-handle +
 * cc_last_error on decline (rational/non-rational mix, coincident sections, a
 * malformed section, or a native decline). */
cc_surface cc_nurbs_skin(const cc_curve* sections, int n, int degreeV);

/* GORDON / NETWORK — boolean-sum surface INTERPOLATING a K u-curve + L v-curve
 * network. `uCurves`/`vCurves` are the two families (K,L >= 2); `vParams` (length
 * K) is the v-station of each u-curve and `uParams` (length L) the u-station of
 * each v-curve (the grid stations). Rational-aware: an all-rational network uses
 * the exact rational Gordon, an all-non-rational network the non-rational one.
 * Returns the Gordon cc_surface, or a 0-handle + cc_last_error on decline (an
 * inconsistent grid, a rational/non-rational mix, or a singular interpolation). */
cc_surface cc_nurbs_gordon(const cc_curve* uCurves, int nU, const cc_curve* vCurves,
                           int nV, const double* vParams, const double* uParams);

/* COONS PATCH — fill the four boundary cc_curves (c0 = edge v=0, c1 = edge v=1,
 * d0 = edge u=0, d1 = edge u=1; shared corners must coincide within `tol`) with the
 * boolean-sum surface INTERPOLATING all four boundaries. `tol <= 0` uses the native
 * default. Returns the Coons cc_surface, or a 0-handle + cc_last_error on decline
 * (mismatched corners, a rational/degenerate boundary). NON-RATIONAL boundaries. */
cc_surface cc_nurbs_coons(cc_curve c0, cc_curve c1, cc_curve d0, cc_curve d1, double tol);

/* N-SIDED FILL — fill the closed loop of `n` boundary cc_curves (>= 3, consecutive
 * corners meeting within `tol`) with the matching native multi-patch fill selected
 * by `mode` (CC_NSIDED_C0 / _G1 / _G2 / _RATIONAL). The result is 1..N tensor-
 * product patches; they are written into the caller array `outPatches` (capacity
 * `cap` cc_surface handles). RETURNS the number of patches written (>= 1), or:
 *   < 0  and cc_last_error set  → honest decline (a non-closed loop, a rational
 *         boundary where the mode forbids it, a G1/G2-INFEASIBLE creased corner —
 *         NOT filled with a residual crease — or `cap` too small: NOTHING is
 *         registered/written and no handle leaks). `tol <= 0` uses the native
 *         default. Registered handles are the caller's to release on success. */
int cc_nurbs_nsided_fill(const cc_curve* boundary, int n, CCNSidedMode mode,
                         double tol, cc_surface* outPatches, int cap);

/* VARIABLE-SECTION SWEEP — sweep `profile` along `path`, applying a per-station
 * `scales[k]` (>0) and `twists[k]` (radians) to the rotation-minimizing-frame–
 * transported section, then skin the placed sections. `sectionNormalXYZ` (3
 * doubles) is the profile plane normal; `stations` (>= 2) the sample count;
 * `degreeV` the across-stations degree. `scales`/`twists` are each `stations` long.
 * Rational-aware (rational profile → exact rational variable sweep). Returns the
 * swept cc_surface, or a 0-handle + cc_last_error on decline (< 2 stations, a
 * non-positive scale, a degenerate path, a rational/non-rational mismatch, a native
 * decline). */
cc_surface cc_nurbs_sweep_variable(cc_curve profile, cc_curve path,
                                   const double* sectionNormalXYZ, const double* scales,
                                   const double* twists, int stations, int degreeV);

/* TWO-RAIL SWEEP — sweep `profile` between `rail0` and `rail1`: the section pole
 * indices `anchor0`/`anchor1` (distinct, in range) ride the two rails at every
 * station. `sectionNormalXYZ` (3 doubles) is the profile plane normal; `stations`
 * (>= 2); `degreeV` the across-stations degree. Rational-aware (rational profile →
 * exact rational two-rail sweep; rails are non-rational). Returns the swept
 * cc_surface, or a 0-handle + cc_last_error on decline (bad anchors, coincident
 * anchors, a degenerate rail chord, a rational/non-rational mismatch, a native
 * decline). */
cc_surface cc_nurbs_sweep_two_rail(cc_curve profile, cc_curve rail0, cc_curve rail1,
                                   const double* sectionNormalXYZ, int anchor0, int anchor1,
                                   int stations, int degreeV);

/* REVOLVE — EXACT rational surface of revolution: revolve `profile` about the axis
 * (`axisPointXYZ`, `axisDirXYZ`, each 3 doubles) through `angle` radians. A straight
 * offset segment → exact cylinder; a tilted segment → exact cone; a semicircle →
 * exact sphere (points on the analytic surface to machine precision). Rational-aware
 * (a rational profile's weights ride through). Returns the revolved cc_surface, or a
 * 0-handle + cc_last_error on decline (a null/non-unit axis, a ~zero angle, or the
 * whole profile lying ON the axis). */
cc_surface cc_nurbs_revolve(cc_curve profile, const double* axisPointXYZ,
                            const double* axisDirXYZ, double angle);

/* JOIN — reposition the near-boundary control rows of two adjacent cc_surfaces
 * `a` and `b` (sharing the C0 edge named by `edgeA`/`edgeB`; `reversed` != 0 when
 * B's along-edge parameter runs opposite to A's) so they meet with continuity
 * `mode` (CC_JOIN_G1 / _G2) across that edge, with minimal control movement and the
 * shared boundary curve frozen. `maxMovementCap <= 0` means uncapped. On success the
 * two repositioned surfaces are registered into `outA`/`outB` (either may be null to
 * discard) and 1 is returned; the achieved continuity residual (G1: max unit-normal
 * mismatch in rad; G2: max relative normal-curvature mismatch) is written to
 * `*residual` when non-null. RETURNS 0 + cc_last_error on decline (a non-coincident
 * edge, an irreconcilable degree/knot mismatch, a rational mismatch, or a required
 * movement beyond `maxMovementCap`) — NOTHING is registered on decline. */
int cc_nurbs_join(cc_surface a, cc_surface b, CCSurfaceEdge edgeA, CCSurfaceEdge edgeB,
                  int reversed, CCJoinMode mode, double maxMovementCap, double* residual,
                  cc_surface* outA, cc_surface* outB);

#endif /* CC_KERNEL_NO_PROTOTYPES */

/* ═══════════════════ end --- J3: surfacing --- ════════════════════════════════ */

#ifdef __cplusplus
}
#endif

#endif /* CYBERCADKERNEL_CC_KERNEL_NURBS_H */
