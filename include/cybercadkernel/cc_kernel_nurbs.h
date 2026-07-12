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

#endif /* CC_KERNEL_NO_PROTOTYPES */

#ifdef __cplusplus
}
#endif

#endif /* CYBERCADKERNEL_CC_KERNEL_NURBS_H */
