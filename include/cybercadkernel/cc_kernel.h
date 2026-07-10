#ifndef CYBERCADKERNEL_CC_KERNEL_H
#define CYBERCADKERNEL_CC_KERNEL_H

/*
 * CyberCadKernel public C ABI (cc_*).
 *
 * This header is the STABLE, plain-C boundary of the kernel. It mirrors — byte
 * for byte — CyberCad's existing `KernelBridgeAPI.h`, so the host app can link
 * CyberCadKernel in place of its in-app OCCT bridge with no source change. No
 * C++ type and no engine (OCCT) type ever crosses this surface: bodies are
 * referenced by an opaque integer `CCShapeId`, geometry returns as POD structs.
 *
 * ABI contract: the POD structs and the 57 `cc_*` signatures below are
 * binary-compatible with `KernelBridgeAPI.h`. `tests/test_abi.cpp` static_asserts
 * the struct sizes and field offsets against the reference header to prove it.
 *
 * Define CC_KERNEL_NO_PROTOTYPES before including this header to pull in only the
 * POD struct/typedef definitions (used by the ABI contract test to compare
 * layouts without redeclaring the C-linkage functions).
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ── POD value types ─────────────────────────────────────────────────────── */

typedef struct {
    double *vertices;   /* x,y,z triplets, length = vertexCount * 3 */
    int vertexCount;
    int *triangles;     /* i,j,k triplets, length = triangleCount * 3 */
    int triangleCount;
} CCMesh;

typedef long CCShapeId;   /* 0 = invalid / not built */

/* One segment of a typed outer profile, so extruded line/arc/circle edges become
 * TRUE B-rep edges (a whole arc/circle = ONE selectable edge), not sampled
 * polygon segments. kind: 0 = line (x0,y0->x1,y1), 1 = arc (circle cx,cy,r over
 * angles a0->a1, endpoints x0,y0/x1,y1), 2 = full circle (cx,cy,r),
 * 3 = spline (a single B-spline edge fit through ptCount points read from the
 *   call's splineXY side-channel starting at ptOffset; x,y pairs). */
typedef struct {
    int kind;
    double x0, y0, x1, y1;
    double cx, cy, r;
    double a0, a1;
    int ptOffset, ptCount;   /* kind 3 only: window into splineXY */
} CCProfileSeg;

/* Exact mass properties of a solid body (volume from the B-rep, not the mesh).
 * `valid` is 0 when OCCT is unavailable or the body id is unknown. */
typedef struct {
    double volume;       /* mm^3 */
    double area;         /* mm^2 (total surface area) */
    double cx, cy, cz;   /* center of mass */
    int valid;
} CCMassProps;

/* Foot-of-perpendicular projection of a 3-D point onto a face's underlying analytic
 * surface (MOAT M-DM DM4, ADDITIVE). Mirrors OCCT GeomAPI_ProjectPointOnSurf on the
 * face's UNTRIMMED analytic surface: `foot*` is the closest surface point, `distance`
 * the minimum point→surface distance. `valid` is 1 for a definite closed-form foot;
 * it is 0 on an HONEST DECLINE (the native engine handles plane / cylinder / sphere
 * faces; a cone / torus / freeform face, or an AMBIGUOUS pose — the point lies on a
 * cylinder axis or at a sphere centre so the foot is a whole circle/sphere — declines,
 * and cc_last_error is set). The OCCT adapter is the GeomAPI_ProjectPointOnSurf oracle. */
typedef struct {
    double footX, footY, footZ;  /* the projected foot point on the surface */
    double distance;             /* minimum distance point → surface (mm) */
    int valid;                   /* 1 = definite closed-form foot; 0 = declined */
} CCProjection;

/* Interference / clash STATE in a CCInterference::state. */
enum CCClashState {
    CC_CLASH_CLEAR = 0,     /* a positive clearance gap (no contact) */
    CC_CLASH_TOUCHING = 1,  /* boundary contact, no interior overlap (distance ~= 0) */
    CC_CLASH_CLASH = 2      /* interiors overlap over a set of positive volume */
};

/* Interference / clash result between two solids (MOAT M-GS GS7, ADDITIVE). Answers
 * the assembly-mate question "do these two solids interfere?": `state` is a
 * CCClashState (CLEAR / TOUCHING / CLASH); `clash` is 1 iff state == CLASH.
 * `overlap_volume` (mm^3) is the volume of the intersection A n B (> 0 only on a
 * CLASH; the native engine computes it via the native boolean COMMON, the OCCT
 * oracle via BRepAlgoAPI_Common + BRepGProp). `min_distance` (mm) is the minimum
 * boundary clearance (meaningful for CLEAR/TOUCHING; BRepExtrema_DistShapeShape on
 * the oracle). `has_witness` is 1 when a CLASH witness is present: witness_lo/hi are
 * the overlap AABB corners and witness_point is a representative point in the
 * overlap interior. `decided` is 1 for a definite verdict; it is 0 on an HONEST
 * DECLINE (a non-watertight/ambiguous native pose, or a clash whose overlap volume
 * the native engine cannot robustly compute), in which case cc_interference returns
 * 0 and cc_last_error is set — the native engine NEVER reports a wrong clash flag or
 * overlap volume. */
typedef struct {
    int state;             /* CCClashState */
    int clash;             /* 1 iff state == CC_CLASH_CLASH */
    int decided;           /* 1 = definite verdict; 0 = honest decline */
    double overlap_volume; /* mm^3 (> 0 only on clash) */
    double min_distance;   /* mm (boundary clearance; clear/touching) */
    int has_witness;       /* 1 = the witness_* fields are meaningful (clash) */
    double witness_lo[3];  /* overlap AABB min corner */
    double witness_hi[3];  /* overlap AABB max corner */
    double witness_point[3]; /* a representative interior point of the overlap */
} CCInterference;

/* First-failing (or undecidable) check code in a CCValidityReport::first_failure.
 * 0 = none (the solid is valid). */
enum CCValidityCheck {
    CC_VALID_OK = 0,
    CC_VALID_NONFINITE = 1,               /* a coordinate is NaN/Inf */
    CC_VALID_NOT_CLOSED = 2,              /* open / non-manifold boundary */
    CC_VALID_BAD_ORIENTATION = 3,         /* inconsistent face orientation */
    CC_VALID_DEGENERATE = 4,              /* zero-area face / zero-length edge */
    CC_VALID_SELF_INTERSECT = 5,          /* self-intersecting faces */
    CC_VALID_SELF_INTERSECT_UNDECIDABLE = 6  /* honest decline (coplanar overlap) */
};

/* Structural-validity report of a solid (MOAT M-GS GS6, ADDITIVE). Each per-check
 * flag is an independent necessary condition. `decided` is 1 when the engine
 * produced a definite verdict; it is 0 on an HONEST DECLINE — a check the engine
 * cannot robustly reach (e.g. certifying no-self-intersection on a coplanar
 * overlap), in which case cc_check_solid returns 0 and `valid` is NEVER 1.
 * `valid` (meaningful only when decided) is the conjunction of every check;
 * `first_failure` is a CCValidityCheck code naming the first failing/undecidable
 * check (0 when valid). The NativeEngine fills the whole per-check breakdown from
 * the M0 mesh; the OCCT adapter reports the BRepCheck_Analyzer::IsValid overall
 * verdict (breakdown fields mirror it). */
typedef struct {
    int valid;                    /* overall structural validity (only when decided) */
    int decided;                  /* 1 = definite verdict; 0 = honest decline */
    int finite;                   /* all coordinates finite */
    int closed_manifold;          /* closed 2-manifold boundary */
    int consistent_orientation;   /* consistent outward face orientation */
    int no_degenerate;            /* no zero-area face / zero-length edge */
    int no_self_intersection;     /* no self-intersecting faces */
    int first_failure;            /* CCValidityCheck of first failing/undecidable check */
} CCValidityReport;

/* One OCCT edge as a single discretized 3-D polyline, tagged with the edge id
 * (the same 1-based id cc_subshape_ids/cc_fillet_edges use). A whole arc is ONE
 * polyline, so the UI can pick/highlight it as one edge rather than mesh
 * segments. `points` is x,y,z triplets of length pointCount*3 (owned). */
typedef struct {
    int edgeId;
    double *points;
    int pointCount;
} CCEdgePolyline;

/* One OCCT face as its own triangle mesh, tagged with the face id (the same
 * 1-based id cc_subshape_ids/cc_shell/cc_offset_face use). Lets the UI pick and
 * highlight a WHOLE face rather than mesh strips. `vertices` is x,y,z triplets
 * (length vertexCount*3); `triangles` is i,j,k face-local indices (owned). */
typedef struct {
    int faceId;
    double *vertices;
    int vertexCount;
    int *triangles;
    int triangleCount;
} CCFaceMesh;

/* ── Phase-additive (NOT part of the mirrored KernelBridgeAPI.h ABI): render-quality
 *    DISPLAY mesh ─────────────────────────────────────────────────────────────
 * A shading-ready mesh POST-PROCESSED from the correctness tessellation (the same
 * triangle mesh cc_tessellate produces). It adds per-vertex SMOOTH normals with
 * crease-angle HARD edges, optional texture coordinates, and an optional lower
 * level-of-detail — attributes the correctness mesh deliberately omits. It does
 * NOT change the tessellator or cc_tessellate: it CONSUMES that mesh.
 *
 * `positions` is x,y,z triplets (len vertexCount*3). `normals` is a UNIT normal
 * per vertex, x,y,z triplets (same length) — smooth across sub-crease surfaces,
 * split (duplicated vertex) across edges whose dihedral exceeds the crease angle
 * so hard edges stay sharp. `uvs` is u,v pairs (len vertexCount*2) when UVs were
 * requested, else NULL. `triangles` is i,j,k triplets into all per-vertex arrays.
 * Owned by the caller; free with cc_display_mesh_free. All-null / zero counts on
 * failure or an empty body (HONEST DECLINE, never a fabricated mesh). */
typedef struct {
    double *positions;   int vertexCount;   /* x,y,z triplets, len vertexCount*3 */
    double *normals;                        /* x,y,z unit triplets, len vertexCount*3 */
    double *uvs;                            /* u,v pairs, len vertexCount*2 (NULL if none) */
    int    *triangles;   int triangleCount; /* i,j,k triplets, len triangleCount*3 */
} CCDisplayMesh;

/* ── Phase-4 additive (NOT part of the mirrored KernelBridgeAPI.h ABI): tet mesh ── */

/* A tetrahedral volume mesh. nodes = x,y,z triplets (len nodeCount*3). elements =
 * nodesPerElement ints per tet into `nodes`, 0-based, in CalculiX C3D4/C3D10 order
 * (len elementCount*nodesPerElement). nodesPerElement is 4 (linear/C3D4) or
 * 10 (quadratic/C3D10). order mirrors nodesPerElement. Owned by the caller; free
 * with cc_tet_mesh_free. On failure all buffers are null / counts 0. */
typedef struct {
    double *nodes;       int nodeCount;
    int    *elements;    int elementCount;
    int     nodesPerElement;   /* 4 or 10 */
    int     order;             /* 4=linear C3D4, 10=quadratic C3D10 */
} CCTetMesh;

/* Options for cc_tet_mesh / cc_tet_mesh_surface. order: 4=linear, 10=quadratic
 * (default 10). target_element_size (mm; <=0 => no volume constraint, TetGen 'a'
 * omitted). grading (radius-edge quality ratio q, e.g. 1.4; clamp >= 1.0).
 * min_scaled_jacobian: quality gate reported back, NOT passed to TetGen. Mirrors
 * CalculiX++ VolumeMeshOptions. */
typedef struct {
    int    order;
    double target_element_size;
    double grading;
    double min_scaled_jacobian;
} CCVolumeMeshOptions;

/* Native mesh-quality report over a tet mesh (pure geometry; ALWAYS available,
 * no TetGen). Angles in degrees; a regular tet scores dihedral 70.53, scaledJ 1.
 * flagged_elements = malloc'd ids of tets with scaled Jacobian < min_scaled_jacobian
 * (len elements_below_threshold), owned by caller, freed by cc_quality_report_free.
 * valid = 0 when the input mesh is empty / degenerate. */
typedef struct {
    double min_dihedral_angle, max_dihedral_angle;
    double min_scaled_jacobian, mean_scaled_jacobian;
    double max_aspect_ratio;
    int    elements_below_threshold;
    int   *flagged_elements;
    int    valid;
} CCQualityReport;

/* AP242 PMI census (ADDITIVE, READ-ONLY). Per-class counts of the recognised PMI /
 * GD&T / draughting annotation entities in a STEP file. This is a COUNT/CLASSIFY
 * slice, NOT a GD&T semantic model: tolerance magnitudes / zones / modifiers /
 * datum reference frames are NOT reported and never invented. `unknown` counts
 * PMI-adjacent entities outside the recognised table (never faked into a class);
 * `total` is the sum of all classes. Filled by cc_step_pmi_scan. */
typedef struct {
    int dimensions;
    int tolerances;
    int datums;
    int datum_targets;
    int notes;
    int annotation_geometry;
    int unknown;
    int total;
} CCPmiSummary;
/* ── Phase-additive (NOT part of the mirrored KernelBridgeAPI.h ABI): drafting /
 *    hidden-line removal (MOAT M-GS GS1) ───────────────────────────────────── */

/* One projected 2D drawing-plane segment (drawing coordinates u,w in mm). The
 * drawing-plane basis is right = normalize(viewDir × up), trueUp = right × viewDir;
 * a world point projects to (P·right, P·trueUp). */
typedef struct {
    double ax, ay;   /* endpoint A (u, w) */
    double bx, by;   /* endpoint B (u, w) */
} CCDrawingSegment;

/* Result of an orthographic hidden-line-removal pass: DISJOINT visible + hidden
 * projected-segment sets, owned by the caller and released with cc_drawing_free.
 * On failure / honest decline both arrays are null / counts 0 and cc_last_error is
 * set. */
typedef struct {
    CCDrawingSegment *visible;   int visibleCount;
    CCDrawingSegment *hidden;    int hiddenCount;
} CCDrawing;

/* Options for cc_hlr_project. deflection: the occluder tessellation chord bound
 * (mm) used to build the triangle occluder — an explicit, caller-chosen value
 * (<= 0 => the engine default). samplesPerEdge: classification samples per edge
 * (<= 0 => default). surfaceOffset: the nudge toward the camera that discounts an
 * edge's own coplanar adjacent faces (<= 0 => default). */
typedef struct {
    double deflection;
    int    samplesPerEdge;
    double surfaceOffset;
} CCHlrOptions;

/* ── Phase-additive: planar SECTION CURVES (MOAT M-GS GS2) ─────────────────────
 * The curves a cut plane carves from a solid (NOT the cut solid). */

/* One closed section loop on the cut plane: an ordered polyline of `pointCount`
 * (x,y,z) triples — the last point is NOT a duplicate of the first (the loop is
 * implicitly closed). `shape` tags the analytic form (0 = polygon, 1 = circle,
 * 2 = ellipse); `length` is the closed-form perimeter and `area` the closed-form
 * region the loop encloses on the cut plane. */
typedef struct {
    double *pointsXYZ;   int pointCount;
    int    shape;        /* 0 polygon, 1 circle, 2 ellipse */
    double length;
    double area;
} CCSectionLoop;

/* Result of cc_section_plane: `loopCount` closed section loops owned by the caller
 * and released with cc_section_free, plus the total section-edge length and the
 * capped planar section area (sum of loop areas). On an honest decline / empty
 * section `loops` is null, `loopCount` 0, and cc_last_error is set. */
typedef struct {
    CCSectionLoop *loops;   int loopCount;
    double totalLength;
    double totalArea;
} CCSection;

#ifndef CC_KERNEL_NO_PROTOTYPES

/* ── Legacy mesh extrude ─────────────────────────────────────────────────── */

/* Extrude a closed 2D profile (x,y pairs) along +Z by `depth`. */
CCMesh cc_extrude(const double *profileXY, int pointCount, double depth);

/* Free the buffers held by a CCMesh returned from this API. */
void cc_mesh_free(CCMesh mesh);

/* ── Kernel availability + error ─────────────────────────────────────────── */

int cc_brep_available(void);                                   /* 1 when a B-rep engine is linked */
const char *cc_last_error(void);                               /* last failure message (this thread), else "" */

/* ── Parallel control ────────────────────────────────────────────────────── */

/* ADDITIVE (not part of the mirrored KernelBridgeAPI.h ABI): toggle multi-core
 * execution of boolean/mesh operations so a serial-vs-parallel A/B is possible
 * (Phase 1 determinism audit). Default ON. In a build with no B-rep engine
 * (host stub) this is a no-op and cc_parallel_enabled() reports 0. */
void cc_set_parallel(int enabled);   /* toggle multi-core execution of boolean/mesh; default on */
int  cc_parallel_enabled(void);      /* 1 if parallel is currently enabled */

/* ── GPU tessellation control ────────────────────────────────────────────── */

/* ADDITIVE (not part of the mirrored KernelBridgeAPI.h ABI): toggle the GPU
 * surface-evaluation tessellation path used by cc_tessellate / cc_face_meshes.
 * Default OFF: with it off, cc_tessellate behaves EXACTLY as the OCCT-only
 * BRepMesh path. When ON (and only on a build compiled with CYBERCAD_HAS_METAL),
 * a face that is provably an untrimmed rectangular NURBS/Bezier patch is meshed
 * from a GPU-evaluated (u,v) grid; every other face falls back to OCCT. On a
 * build without Metal (or the host stub) the setter is a no-op and the query
 * reports 0. The cc_tessellate / cc_face_meshes signatures are unchanged. */
void cc_set_gpu_tessellation(int enabled);   /* toggle the GPU tessellation path; default off */
int  cc_gpu_tessellation_enabled(void);      /* 1 iff GPU tessellation is on AND available */

/* ── Active engine selection ─────────────────────────────────────────────── */

/* ADDITIVE (not part of the mirrored KernelBridgeAPI.h ABI): swap the active
 * geometry engine, so the native C++20 engine (Phase 4 rewrite) can be A/B'd
 * against the OCCT engine behind the SAME cc_* calls. `native != 0` activates the
 * NativeEngine (native solid_extrude / solid_revolve + native tessellate / mass /
 * bbox on native bodies; every other capability falls through to the OCCT engine,
 * or the stub in a no-OCCT host build). `native == 0` restores the build's DEFAULT
 * engine (OCCT where linked, else the stub). DEFAULT IS OCCT — this must be called
 * explicitly to opt in, so all existing behaviour is unchanged until you do. Shape
 * ids created under one engine should be consumed under the same engine. */
void cc_set_engine(int native);      /* 1 = NativeEngine, 0 = OCCT/default */
int  cc_active_engine(void);         /* 1 if the NativeEngine is active, else 0 */

/* ── Construction ────────────────────────────────────────────────────────── */

CCShapeId cc_solid_extrude(const double *profileXY, int pointCount, double depth);
CCShapeId cc_solid_revolve(const double *profileXY, int pointCount, double angleRadians);

/* Ruled loft between a bottom XY section on z=0 and a top XY section on z=depth.
 * `bottomXY` / `topXY` are flat (x,y) pairs; each section is a closed polygon. With
 * EQUAL point counts (bottomCount == topCount, ≥3) the two sections are skinned by a
 * RULED surface: corresponding vertices are paired 1:1 and each corresponding edge
 * pair becomes one ruled side face, capped with the bottom + top polygons → a closed
 * watertight solid. Returns 0 on degenerate input (< 3 points either side, or
 * depth ≤ 0). Mismatched point counts fall to the engine's general loft. */
CCShapeId cc_solid_loft(const double *bottomXY, int bottomCount,
                        const double *topXY, int topCount, double depth);

/* Ruled loft between two ARBITRARY 3D section wires. `aXYZ` / `bXYZ` are flat
 * (x,y,z) triples; each is a closed polygon in space. With EQUAL point counts
 * (aCount == bCount, ≥3) the sections are skinned as in cc_solid_loft (paired
 * vertices → one ruled side face per corresponding edge pair + the two section end
 * faces) → a closed watertight solid. Returns 0 on degenerate input (< 3 points
 * either side). */
CCShapeId cc_solid_loft_wires(const double *aXYZ, int aCount,
                              const double *bXYZ, int bCount);

/* Ruled loft through 2..N ORDERED planar section wires (the ≥3-section
 * generalisation of cc_solid_loft_wires). `sectionsXYZ` holds the sections back to
 * back as flat (x,y,z) triples; `counts[k]` is the vertex count of section k (each
 * ≥3); `sectionCount` (≥2) is the number of sections. Consecutive sections are
 * skinned by RULED bands (one bilinear side face per corresponding edge pair) with
 * the first + last section capped → a closed watertight solid; internal sections
 * are shared vertex rings (not capped). Sections may differ in vertex count (they
 * are made compatible by an arc-length-preserving resample). Returns 0 on
 * degenerate input (< 2 sections, any section < 3 points, a non-planar or
 * point-collapsed section). Mirrors BRepOffsetAPI_ThruSections (ruled). */
CCShapeId cc_solid_loft_sections(const double *sectionsXYZ, const int *counts,
                                 int sectionCount);

/* Sweep a CLOSED profile (x,y pairs, ≥3 points) along a 3D polyline path
 * (x,y,z triples, ≥2 points). The profile is centred on its CENTROID and placed
 * PERPENDICULAR to the path tangent at the START: in the start frame the profile's
 * local x = normalize(cross(startTangent, +Y)) and local y = +Y (a near-vertical
 * start tangent uses +X as the reference instead so the frame never collapses). The
 * section is then transported along the spine and the swept solid is capped at both
 * ends. Returns 0 on degenerate input (< 3 profile points / < 2 path points). */
CCShapeId cc_solid_sweep(const double *profileXY, int profileCount,
                         const double *pathXYZ, int pathCount);

/* As cc_solid_sweep, but the section additionally ROTATES about the path tangent by
 * a total of `twistRadians` (accumulated linearly from 0 at the start to twistRadians
 * at the end) and SCALES linearly from 1 at the start to `scaleEnd` at the end. */
CCShapeId cc_twisted_sweep(const double *profileXY, int profileCount,
                           const double *pathXYZ, int pathCount,
                           double twistRadians, double scaleEnd);

CCShapeId cc_loft_along_rail(const double *railXYZ, int railCount,
                             const double *profileA_XY, int aCount,
                             const double *profileB_XY, int bCount);

/* Loft between two TRUE CIRCLES given by centre (x,y,z), unit normal (x,y,z), and radius:
 * builds circular section wires so the result is a smooth conical/cylindrical B-rep (one
 * side face, two circular edges), not a faceted polygon. Returns 0 on bad input (null
 * pointer or a non-positive radius). ADDITIVE (app-parity). */
CCShapeId cc_loft_circles(const double *c1, const double *n1, double r1,
                          const double *c2, const double *n2, double r2);

/* Loft a TRUE CIRCLE section (centre `cc`, unit normal `cn`, radius `cr`) to an arbitrary
 * polygon wire (`wXYZ`, x,y,z triplets, `wCount`>=3) — a smooth circle<->polygon loft with a
 * true circular rim (no faceting on the circle side). Returns 0 on bad input. ADDITIVE. */
CCShapeId cc_loft_circle_wire(const double *cc, const double *cn, double cr,
                              const double *wXYZ, int wCount);

/* Two-rail variant of cc_loft_along_rail: `railXYZ` is the spine and `guideXYZ` steers the
 * sweep as an auxiliary/guide spine, so the loft is shaped by two curves. Falls back to the
 * single-rail sweep (guide dropped) if the guided build fails, and returns 0 only if even
 * that fails. ADDITIVE (app-parity). */
CCShapeId cc_loft_along_rails(const double *railXYZ, int railCount,
                              const double *guideXYZ, int guideCount,
                              const double *profileA_XY, int aCount,
                              const double *profileB_XY, int bCount);

CCShapeId cc_guided_sweep(const double *profileXY, int profileCount,
                          const double *pathXYZ, int pathCount,
                          const double *guideXYZ, int guideCount);

/* Sweep whose section ORIENTATION is fixed by a guide wire (OCCT
 * BRepOffsetAPI_MakePipeShell + SetMode(guide), default NoContact). Distinct from
 * cc_guided_sweep (guide-SCALED loft): here the guide steers the section frame, not its
 * size. Native for a straight spine (perpendicular-plane [N,B,T] law), OCCT otherwise. */
CCShapeId cc_guided_orient_sweep(const double *profileXY, int profileCount,
                                 const double *pathXYZ, int pathCount,
                                 const double *guideXYZ, int guideCount);

CCShapeId cc_wrap_emboss(CCShapeId body, int faceId,
                         const double *profileXY, int count, double depth, int boss);

CCShapeId cc_helical_thread(double majorRadiusMM, double pitchMM, double turns,
                            double depthMM, double flankAngleDeg,
                            double pointsPerMM, int samplesPerTurn);

CCShapeId cc_tapered_thread(double topRadiusMM, double tipRadiusMM, double pitchMM,
                            double turns, double depthMM, double flankAngleDeg,
                            double pointsPerMM, int samplesPerTurn);

CCShapeId cc_tapered_shank(double radiusMM, double fullHeightMM,
                           double taperHeightMM, double pointsPerMM);

/* Extrude a closed OUTER polygon (x,y pairs on z=0) by +depth, with `holeCount`
 * circular THROUGH-HOLES packed as (cx,cy,r) triples in `holesCenterRadius`. Each
 * hole is a true circular void through the prism (a circle edge + a cylindrical
 * wall on both caps), not a sampled polygon. Returns 0 on degenerate input. */
CCShapeId cc_solid_extrude_holes(const double *outerXY, int outerCount,
                                 const double *holesCenterRadius, int holeCount, double depth);

/* As cc_solid_extrude_holes but the holes are POLYGONS: `holesXY` is a flat x,y
 * stream and `holeCounts[k]` is the vertex count of hole k (consumed in order),
 * for `holeCount` holes. Each polygon hole is a prismatic void through the solid. */
CCShapeId cc_solid_extrude_polyholes(const double *outerXY, int outerCount,
                                     const double *holesXY, const int *holeCounts,
                                     int holeCount, double depth);

/* Extrude a TYPED outer profile by +depth. `segs` is a closed loop of CCProfileSeg
 * (kind 0 line / 1 arc / 2 full circle / 3 spline) so a whole arc or circle becomes
 * ONE true B-rep edge rather than a sampled polyline; kind-3 spline windows into
 * `splineXY` (a flat x,y stream; `splineXYCount` is the number of DOUBLES in it, i.e.
 * 2× the point count — a kind-3 seg reads points [ptOffset, ptOffset+ptCount)).
 * `holesCenterRadius` are `holeCount` circular through-holes (cx,cy,r).
 * Returns 0 on degenerate/unsupported input. */
CCShapeId cc_solid_extrude_profile(const CCProfileSeg *segs, int segCount,
                                   const double *holesCenterRadius, int holeCount,
                                   const double *splineXY, int splineXYCount, double depth);

/* As cc_solid_extrude_profile with BOTH circular holes (`holesCenterRadius`, cx,cy,r
 * × circleCount) AND polygon holes (`polyXY` flat x,y stream; `polyCounts[k]` the
 * vertex count of polygon-hole k, for polyCount holes). kind-3 spline outer edges
 * window into `splineXY` (`splineXYCount` = number of DOUBLES = 2× point count).
 * Returns 0 on degenerate/unsupported input. */
CCShapeId cc_solid_extrude_profile_polyholes(const CCProfileSeg *segs, int segCount,
                                             const double *holesCenterRadius, int circleCount,
                                             const double *polyXY, const int *polyCounts, int polyCount,
                                             const double *splineXY, int splineXYCount, double depth);

/* Revolve a TYPED profile `segs` (kinds as above) about the in-plane axis through
 * point (ax,ay) with in-plane direction (adx,ady), by `angleRadians` (a full 2*pi
 * closes the solid; a partial angle caps the two open ends). Line segments sweep
 * plane/cylinder/cone surfaces; an arc whose circle centre lies on the axis sweeps
 * a sphere. kind-3 spline windows into `splineXY` (`splineXYCount` = number of
 * DOUBLES = 2× point count). Returns 0 on degenerate input. */
CCShapeId cc_solid_revolve_profile(const CCProfileSeg *segs, int segCount,
                                   double ax, double ay, double adx, double ady,
                                   const double *splineXY, int splineXYCount,
                                   double angleRadians);

/* General loft between two TYPED section profiles (ordered CCProfileSeg loops with spline
 * side-channels — same encoding as cc_solid_extrude_profile), each placed on its own plane
 * frame (origin(3)+u(3)+v(3) = 9 doubles). Curved boundaries (arcs/circles/splines) become
 * true B-rep curve edges, so composite profiles loft smoothly. Returns 0 on invalid input.
 * ADDITIVE (app-parity). */
CCShapeId cc_loft_typed(const CCProfileSeg *segsA, int countA, const double *splineA,
                        int splineACount, const double *frameA,
                        const CCProfileSeg *segsB, int countB, const double *splineB,
                        int splineBCount, const double *frameB);

/* ── Feature edits ───────────────────────────────────────────────────────── */

CCShapeId cc_fillet_edges(CCShapeId body, const int *edgeIds, int edgeCount, double radius);
CCShapeId cc_fillet_edges_variable(CCShapeId body, const int *edgeIds, int edgeCount, double radius1, double radius2);
CCShapeId cc_chamfer_edges(CCShapeId body, const int *edgeIds, int edgeCount, double distance);
/* ASYMMETRIC two-distance chamfer: distance1 = the setback on the FIRST (wall) face,
 * distance2 = the setback on the SECOND (cap) face. distance1 == distance2 is equivalent
 * to cc_chamfer_edges. Native engine: T1 oblique cone-frustum bevel on a convex circular
 * cylinder↔coaxial-cap rim (C0 at two different angles), self-verified watertight/shrink,
 * else OCCT BRepFilletAPI_MakeChamfer::Add(d1,d2,edge,face). Returns 0 on failure. */
CCShapeId cc_chamfer_edges_asym(CCShapeId body, const int *edgeIds, int edgeCount,
                                double distance1, double distance2);
CCShapeId cc_shell(CCShapeId body, const int *faceIds, int faceCount, double thickness);
CCShapeId cc_offset_face(CCShapeId body, int faceId, double distance);
CCShapeId cc_replace_face(CCShapeId body, int faceId, double offset, double tiltDeg);

CCShapeId cc_replace_face_to_plane(CCShapeId body, int faceId,
                                   double px, double py, double pz,
                                   double nx, double ny, double nz);

/* DRAFT ANGLE — taper `faceCount` PLANAR side faces of a solid (1-based face ids, the
 * same ids cc_subshape_ids/cc_fillet_edges use) about the NEUTRAL PLANE (a point
 * `neutralOrigin[3]` on it, and the PULL direction `pullDir[3]` = its normal) by
 * `angleDeg` degrees. Each drafted face pivots on its trace line with the neutral plane
 * — the trace stays put and the face tilts by the draft angle about the pull axis, so
 * the walls taper for mold release; the adjacent faces re-trim to keep the solid
 * watertight. A POSITIVE angle draws material IN as the face recedes from the neutral
 * plane along +pull (standard draft convention). Returns a new solid, or 0 on failure
 * (cc_last_error set). ADDITIVE.
 *
 * ENGINE NOTE. Under the native engine (cc_set_engine(1)) a NATIVE all-planar
 * (prismatic) body is drafted natively: each drafted plane is derived from the original
 * face geometry and applied as an inward half-space trim, then the composite is
 * SELF-VERIFIED (watertight closed 2-manifold, single lump χ=2, consistently oriented,
 * volume strictly SMALLER than the original — a draft only removes stock) and DISCARDED
 * if it fails — never a wrong/leaky solid. A curved base / non-planar neutral / a face
 * perpendicular to the pull axis (a cap, no trace line) / a degenerate or ≥90° angle /
 * a self-intersecting result falls to the OCCT oracle
 * (BRepOffsetAPI_DraftAngle + BRepGProp), on iOS/macOS where OCCT is linked; on the
 * OCCT-free host such a native-only draft returns 0 with an honest error. An OCCT body
 * forwards to OCCT unchanged. */
CCShapeId cc_draft_faces(CCShapeId body, const int *faceIds, int faceCount,
                         const double *neutralOrigin, const double *pullDir, double angleDeg);

/* Project the 3-D point (px,py,pz) onto the analytic surface of face `faceId`
 * (1-based, cc_subshape_ids order) of `body` — MOAT M-DM DM4, ADDITIVE. Returns the
 * closed-form foot-of-perpendicular + minimum distance (see CCProjection). The native
 * engine serves plane / cylinder / sphere faces of a native body in closed form and
 * honestly DECLINES cone / torus / freeform / ambiguous poses (valid = 0); the OCCT
 * engine mirrors GeomAPI_ProjectPointOnSurf on the face's untrimmed surface. */
CCProjection cc_project_point_on_face(CCShapeId body, int faceId,
                                      double px, double py, double pz);

CCShapeId cc_fillet_face(CCShapeId body, int faceId, double radius);

CCShapeId cc_split_plane(CCShapeId body, double ox, double oy, double oz,
                         double nx, double ny, double nz, int keepPositive);

/* ── Booleans ────────────────────────────────────────────────────────────── */

/* Boolean set operation on two solids: op = 0 fuse (a ∪ b), 1 cut (a − b),
 * 2 common (a ∩ b). Returns a new solid, or 0 on failure (cc_last_error set).
 *
 * ENGINE NOTE (Phase 4 #5 native-booleans). Under the DEFAULT (OCCT) engine this is
 * OCCT BRepAlgoAPI (BOPAlgo) for all solids. Under the native engine (cc_set_engine(1))
 * it is NATIVE for PLANAR-FACED polyhedra — boxes / prisms / convex or simple-concave
 * solids whose every face is a plane: a clean-room BSP-CSG computes the face-face
 * intersection splits, classifies each fragment inside/outside/on the other solid,
 * and welds the surviving fragments into a watertight solid. The native result is
 * SELF-VERIFIED (closed watertight 2-manifold AND the exact set-algebra volume
 * fuse=A+B−∩ / cut=A−∩ / common=∩) and DISCARDED if it fails — never a wrong/leaky
 * solid. Operands with any CURVED face (cylinder/sphere/cone/free-form) or a
 * degenerate/near-tangent configuration are outside the native planar domain and fall
 * through to the OCCT oracle (on iOS/macOS, where OCCT is linked); on the OCCT-free
 * host such a native-only boolean returns 0 with an honest error. Both operands must be
 * built under the same active engine: a native body is recognised process-wide (its
 * identity is not tied to a particular cc_set_engine(1) instance), so toggling the engine
 * between build and boolean is safe, and the engine NEVER hands a native body to OCCT
 * (which would misread it) — a native operand that OCCT cannot process yields a clean
 * error, and mixing a native operand with an OCCT operand is rejected. */
CCShapeId cc_boolean(CCShapeId a, CCShapeId b, int op);

/* ── Tessellation ────────────────────────────────────────────────────────── */

/* Tessellate a body for display at `deflection` (mm). */
CCMesh cc_tessellate(CCShapeId body, double deflection);

/* ADDITIVE render-quality DISPLAY mesh (NOT part of the mirrored KernelBridgeAPI.h
 * ABI). Post-processes the correctness tessellation of `body` at `deflection` into
 * a shading-ready CCDisplayMesh (see the struct above): per-vertex smooth normals,
 * crease-angle hard edges, optional UVs, optional LOD. It does NOT alter
 * cc_tessellate — that call is byte-identical whether or not this is used.
 *   creaseAngleDeg : dihedral (degrees) above which an edge is HARD (vertices are
 *                    split so the crease stays sharp); curved surfaces below it
 *                    shade smooth. Typical 20–45.
 *   lodTargetTris  : target triangle count for edge-collapse decimation; <= 0
 *                    disables LOD (the display mesh keeps the full resolution).
 *                    Decimation preserves boundary/crease edges and stays within a
 *                    Hausdorff bound derived from the deflection.
 *   wantUVs        : 1 to emit per-vertex UVs (box/planar projection, in [0,1]);
 *                    0 leaves out->uvs NULL.
 * Fills *out (owned by the caller; free with cc_display_mesh_free) and returns the
 * triangle count. Returns 0 with *out zeroed on an empty/unknown body or a mesh
 * that cannot be produced (HONEST DECLINE) — never a fabricated mesh. Under the
 * OCCT engine it consumes the OCCT tessellation; under the native engine it
 * consumes the native SolidMesher output. */
int cc_display_mesh(CCShapeId body, double deflection, double creaseAngleDeg,
                    int lodTargetTris, int wantUVs, CCDisplayMesh *out);
void cc_display_mesh_free(CCDisplayMesh *mesh);

/* ── Queries ─────────────────────────────────────────────────────────────── */

CCMassProps cc_mass_properties(CCShapeId body);
/* Principal moments of inertia (unit-density volume inertia). out3 = [I1,I2,I3]. 1 on success. */
int cc_principal_moments(CCShapeId body, double *out3);

/* Structural-validity report of a solid (MOAT M-GS GS6). Fills *out with the
 * per-check breakdown. Returns 1 when a DEFINITE verdict was produced
 * (out->decided == 1; out->valid is the overall verdict); returns 0 on an HONEST
 * DECLINE (out->decided == 0, an undecidable check, cc_last_error set) or an
 * unknown body / no B-rep engine (out zeroed) — NEVER reports valid==1 for a body
 * it cannot verify. */
int cc_check_solid(CCShapeId body, CCValidityReport *out);

/* Interference / clash detection between two solids (MOAT M-GS GS7, ADDITIVE — the
 * assembly-mate value). Fills *out with the CLASH / TOUCHING / CLEAR verdict, the
 * overlap volume, the min boundary clearance, and (on a clash) a witness AABB +
 * interior point. Returns 1 when a DEFINITE verdict was produced (out->decided == 1)
 * or 0 on an HONEST DECLINE (out->decided == 0, cc_last_error set) or unknown body
 * (out zeroed). Under the native engine both bodies must be built native (a mixed
 * native/OCCT pair is rejected); the overlap volume comes from the native boolean
 * COMMON with a two-sided self-verify, and a pose whose overlap cannot be robustly
 * computed DECLINES to the OCCT BRepAlgoAPI_Common + BRepExtrema oracle — a wrong
 * clash flag or overlap volume is NEVER returned. */
int cc_interference(CCShapeId a, CCShapeId b, CCInterference *out);

/* Exact axis-aligned bounding box of the B-rep (not the tessellation). Fills
 * out6 = [minX,minY,minZ, maxX,maxY,maxZ]. Returns 1 on success, 0 otherwise. */
int cc_bounding_box(CCShapeId body, double *out6);

/* Axis of a cylindrical / conical face (parity C10). Fills out6 = [px,py,pz, dx,dy,dz]
 * (a point on the axis + its unit direction). Returns 1 for a cylinder/cone face, else 0. */
int cc_face_axis(CCShapeId body, int faceId, double *out6);

/* Stable sub-shape ids for picking: kind = 0 vertex, 1 edge, 2 face.
 * Caller frees outIds with cc_ints_free. Returns the id count. */
int cc_subshape_ids(CCShapeId body, int kind, int **outIds);
void cc_ints_free(int *ids);

/* Number of connected solids in a body (a body may be a compound of several disjoint
 * lumps). Returns 0 if the shape has no solids or is unknown. ADDITIVE (app-parity). */
int cc_shape_solid_count(CCShapeId body);

/* The `index`-th connected solid of a body (index is 0-based), registered as its own
 * independent shape (0 on out-of-range or failure). Lets a disconnected lump be
 * selected/moved alone. ADDITIVE (app-parity). */
CCShapeId cc_shape_solid_at(CCShapeId body, int index);

/* ── Measurement & curvature analysis (MOAT M-GS, GS3 + GS4) ─────────────────
 * Exact analysis SERVICES on the native B-rep. subKind selects the sub-shape:
 * 0 = vertex, 1 = edge, 2 = face; subId is the 1-based cc_subshape_ids number.
 * Every call returns 1 on success or 0 on an HONEST DECLINE (cc_last_error set)
 * — a non-line/plane angle, a parametric-singularity curvature, or a non-
 * certifiable freeform-trimmed minimizer NEVER yields a fabricated number. */

/* Minimum distance between two entities. Fills out7 = [d, p1x,p1y,p1z, p2x,p2y,p2z]
 * (the gap + the witness point on entity A then on entity B). 1 on success, 0 on
 * decline. */
int cc_measure_distance(CCShapeId body, int subKindA, int subIdA,
                        int subKindB, int subIdB, double *out7);

/* Angle (radians) between two entities: line·line ∈ [0,π/2], plane·plane ∈ [0,π],
 * line·plane ∈ [0,π/2]. Fills *outRadians. 1 on success, 0 on decline (any non-
 * line/plane entity). */
int cc_measure_angle(CCShapeId body, int subKindA, int subIdA,
                     int subKindB, int subIdB, double *outRadians);

/* Surface curvature at face (faceId) parameter (u,v). Fills out4 = [K, H, k1, k2]
 * (Gaussian, mean, principal k1 ≥ k2). 1 on success, 0 on decline (parametric
 * singularity). */
int cc_surface_curvature(CCShapeId body, int faceId, double u, double v, double *out4);

/* Edge curvature κ at edge (edgeId) parameter t. Fills *outKappa. 1 on success,
 * 0 on decline (stationary/cusp point). */
int cc_edge_curvature(CCShapeId body, int edgeId, double t, double *outKappa);

int cc_tangent_chain(CCShapeId body, const int *edgeIds, int edgeCount, int **outIds);

int cc_outer_rim_chain(CCShapeId body, const int *edgeIds, int edgeCount, int **outIds);

/* Returns the edge count; caller frees with cc_edge_polylines_free. */
int cc_edge_polylines(CCShapeId body, CCEdgePolyline **outEdges);
void cc_edge_polylines_free(CCEdgePolyline *edges, int count);

/* Offset a face's outer boundary within its plane (negative = inward) -> a flat xyz
   polyline. Returns the point count; caller frees *outXYZ with cc_points_free. */
int cc_offset_face_boundary(CCShapeId body, int faceId, double distance, double **outXYZ);
void cc_points_free(double *p);

/* Returns the face count; caller frees with cc_face_meshes_free. */
int cc_face_meshes(CCShapeId body, double deflection, CCFaceMesh **outFaces);
void cc_face_meshes_free(CCFaceMesh *faces, int count);

/* ── Data exchange ───────────────────────────────────────────────────────── */

/* STEP exchange. cc_step_export returns 1 on success. */
int cc_step_export(CCShapeId body, const char *path);
CCShapeId cc_step_import(const char *path);

/* Read-only AP242 PMI scan of a STEP file (ADDITIVE). Recognises / classifies /
 * counts the PMI annotation entities and fills *out with the per-class census.
 * Returns 1 on success, 0 on failure (see cc_last_error) — e.g. a null path/out or
 * an unreadable file. Does NOT import geometry and does NOT alter cc_step_import:
 * the solid a STEP file imports is byte-identical whether or not this is called. */
int cc_step_pmi_scan(const char *path, CCPmiSummary *out);

/* IGES exchange (millimetres, B-rep mode). cc_iges_export returns 1 on success. */
int cc_iges_export(CCShapeId body, const char *path);
CCShapeId cc_iges_import(const char *path);

/* STL exchange (triangle mesh, true millimetres). ADDITIVE. `deflection` is the
 * chord tolerance for the tessellation the writer serialises; binary=1 => binary
 * STL (default), binary=0 => ASCII. Export is deterministic (same body + deflection
 * => byte-identical file) with a per-facet geometric normal. cc_stl_export returns
 * 1 on success, 0 on failure (see cc_last_error). cc_stl_import auto-detects ASCII
 * vs binary and returns a MESH body (triangle soup, welded vertices) usable for
 * display + measurement (bbox / area / volume-if-closed) + cc_tessellate; it is NOT
 * a B-rep reconstruction. Returns 0 on failure (malformed file → cc_last_error). */
int cc_stl_export(CCShapeId body, const char *path, double deflection, int binary);
CCShapeId cc_stl_import(const char *path);

/* glTF 2.0 mesh export (the iPad AR / RealityKit / QuickLook / web-render handoff).
 * ADDITIVE, mesh-based (native serves; no OCCT fallback needed). `deflection` is the
 * chord tolerance for the tessellation the writer serialises. Positions are emitted
 * in METRES (kernel millimetres × 1e-3, glTF's linear unit); connectivity is
 * preserved index-for-index. glb=1 => a binary .glb container (12-byte header +
 * JSON chunk + BIN chunk, 4-byte aligned); glb=0 => a self-contained .gltf JSON with
 * a base64 data-URI buffer. One mesh / one triangle primitive with POSITION + NORMAL
 * accessors (smooth per-vertex normals derived from mesh geometry), correct
 * POSITION min/max bounds, and one default metallic-roughness material. Deterministic
 * (same body + deflection => byte-identical file). A NULL / empty mesh writes an
 * empty asset and still returns 1. cc_gltf_export returns 1 on success, 0 on failure
 * (see cc_last_error). Export only (no glTF import). */
int cc_gltf_export(CCShapeId body, const char *path, double deflection, int glb);

/* USDZ mesh export (Apple QuickLook / RealityKit "View in AR"). ADDITIVE, mesh-based
 * (native serves). Packs a single ASCII-USD (.usda) UsdGeomMesh layer into a USDZ
 * container: a STORE-only (uncompressed) ZIP whose entry data is aligned to a 64-byte
 * boundary per the USDZ spec, so the crate reader can zero-copy map it. Positions in
 * METRES, metersPerUnit=1, Y-up (the QuickLook convention). Same mesh/deflection
 * contract as cc_gltf_export. Deterministic. Returns 1 on success, 0 on failure
 * (see cc_last_error). Export only. */
int cc_usdz_export(CCShapeId body, const char *path, double deflection);

/* ── Transforms ──────────────────────────────────────────────────────────── */

CCShapeId cc_scale_shape(CCShapeId body, double factor);
CCShapeId cc_scale_shape_about(CCShapeId body, double cx, double cy, double cz, double factor);
CCShapeId cc_rotate_shape_about(CCShapeId body, double cx, double cy, double cz,
                                double ax, double ay, double az, double angleRadians);
CCShapeId cc_mirror_shape(CCShapeId body, double px, double py, double pz,
                          double nx, double ny, double nz);
CCShapeId cc_translate_shape(CCShapeId body, double tx, double ty, double tz);

CCShapeId cc_place_on_frame(CCShapeId body,
                            double ox, double oy, double oz,
                            double ux, double uy, double uz,
                            double vx, double vy, double vz);

/* ── Phase-3 additive: reference geometry (datum planes / axes) ──────────────── */

/* ADDITIVE (not part of the mirrored KernelBridgeAPI.h ABI): compute datum
 * reference geometry and return it as a POD out-array with a 1/0 success flag,
 * creating NO shape handle. For a plane out6 = [ox,oy,oz, nx,ny,nz] (origin +
 * unit normal); for an axis out6 = [ox,oy,oz, dx,dy,dz] (origin + unit
 * direction). On success the normal/direction is unit-length within 1e-9.
 *
 * The three point-only constructors are exact fp64 vector math and work in EVERY
 * build (including the no-OCCT host stub). The three derived constructors read an
 * existing body's geometry and require a B-rep engine; in the host stub they
 * return 0 (unsupported) without crashing. */
int cc_ref_plane_from_points(const double p0[3], const double p1[3], const double p2[3],
                             double out6[6]);
int cc_ref_plane_offset(const double origin[3], const double normal[3], double dist,
                        double out6[6]);
int cc_ref_plane_from_face(CCShapeId body, int faceId, double out6[6]);
int cc_ref_axis_from_points(const double a[3], const double b[3], double out6[6]);
int cc_ref_axis_from_edge(CCShapeId body, int edgeId, double out6[6]);
int cc_ref_axis_from_face(CCShapeId body, int faceId, double out6[6]);

/* ── Phase-3 additive: robust thread boolean ─────────────────────────────────── */

/* ADDITIVE: apply a helical thread body to a shaft by a segmented / feature-based
 * boolean that completes within a wall-clock budget where a single brute-force
 * boolean on the full helix would hang. op = 0 fuses the thread onto the shaft
 * (external thread), op = 1 cuts it out (internal thread); any other op returns 0.
 * Returns a new body id on success, 0 on failure. Safe no-op (returns 0) in the
 * host stub. cc_boolean and the thread builders are unchanged. */
CCShapeId cc_thread_apply(CCShapeId shaft, CCShapeId thread, int op);

/* ── Phase-3 additive: full-round fillet (face-consuming rolling-ball blend) ──── */

/* ADDITIVE: replace a narrow face with a rolling-ball blend tangent to its two
 * opposite neighbour faces (consuming the middle face). cc_full_round_fillet
 * auto-detects the two opposite neighbours of faceId; cc_full_round_fillet_faces
 * consumes middleFaceId and blends tangent to leftFaceId and rightFaceId. Returns
 * a new body id on success, 0 on failure. Safe no-op (returns 0) in the host stub. */
CCShapeId cc_full_round_fillet(CCShapeId body, int faceId);
CCShapeId cc_full_round_fillet_faces(CCShapeId body, int leftFaceId, int middleFaceId,
                                     int rightFaceId);

/* ── Phase-3 additive: G2 (curvature-continuous) blend fillet ────────────────── */

/* ADDITIVE: build a curvature-continuous (G2, or best-achievable) blend along the
 * given edges at the nominal radius. Returns a new body id on success, 0 on
 * failure. The stock cc_fillet_edges (G1 baseline) is unchanged. Safe no-op
 * (returns 0) in the host stub. */
CCShapeId cc_fillet_edges_g2(CCShapeId body, const int *edgeIds, int edgeCount, double radius);

/* ── Phase-4 additive: tetrahedral volume meshing ────────────────────────────── */

/* Tet-mesh a body: tessellate its surface, then fill the PLC with TetGen. Requires
 * the OPTIONAL, EXTERNAL, AGPL TetGen backend (CYBERCAD_HAS_TETGEN). In the default
 * MIT build (flag OFF) this returns an EMPTY CCTetMesh and sets cc_last_error to a
 * "tet meshing unavailable" message — never crashes, links no AGPL code. */
CCTetMesh cc_tet_mesh(CCShapeId body, double deflection, CCVolumeMeshOptions opts);

/* Decoupled entry: tet-mesh a raw closed TRIANGLE surface (no OCCT). Same backend,
 * same unavailable behaviour when the flag is OFF. verticesXYZ = x,y,z triplets
 * (len vertexCount*3); trianglesIJK = i,j,k 0-based triplets (len triangleCount*3). */
CCTetMesh cc_tet_mesh_surface(const double *verticesXYZ, int vertexCount,
                              const int *trianglesIJK, int triangleCount,
                              CCVolumeMeshOptions opts);
void cc_tet_mesh_free(CCTetMesh mesh);

/* Native tet-mesh quality (ALWAYS available, TetGen-independent, pure geometry).
 * Returns valid=0 (and sets cc_last_error) on empty/degenerate input. */
CCQualityReport cc_mesh_quality(CCTetMesh mesh, double min_scaled_jacobian);
void cc_quality_report_free(CCQualityReport report);

/* ── Phase-additive: drafting / orthographic hidden-line removal (MOAT GS1) ──── */

/* Orthographic (parallel) hidden-line removal of a body onto a drawing plane.
 * viewDir[3] is the direction the camera looks ALONG (into the scene); up[3] is an
 * up hint (must not be parallel to viewDir). Returns the DISJOINT visible + hidden
 * 2D drawing-plane segment sets (see CCDrawing), so the app's DrawingProjector /
 * ProjectEdges / ProjectBody path can consume native HLR in place of OCCT's
 * visible/hidden compounds. ADDITIVE: no existing cc_* signature changes.
 *
 * Scope (this slice): the POLYHEDRAL core (box / prism / multi-box — planar-faced
 * solids). A body whose drawing needs a CURVED-surface silhouette (cylinder / cone
 * / sphere outline) or a FREEFORM face is DECLINED — cc_hlr_project returns an
 * empty CCDrawing with cc_last_error set, NEVER a partial or wrong classification.
 * Under the OCCT engine the result is the HLRBRep_Algo oracle. */
CCDrawing cc_hlr_project(CCShapeId body, const double viewDir[3], const double up[3],
                         CCHlrOptions opts);
void cc_drawing_free(CCDrawing drawing);

/* ── Phase-additive: planar section curves (MOAT GS2) ───────────────────────── */

/* Planar SECTION CURVES of a solid: intersect `body` with the cut plane through
 * `origin[3]` with unit `normal[3]`, returning the closed section LOOPS (NOT the
 * cut solid) that lie on the plane and on the body's faces — the app's
 * SectionGeometry / MeshSection / SectionCap path. ADDITIVE: no existing cc_*
 * signature changes.
 *
 * Scope (this slice): analytic plane / cylinder / cone / sphere faces. An OBLIQUE
 * cut of a cylindrical face, a plane COINCIDENT or TANGENT to a face, a section
 * that does not CLOSE, and FREEFORM faces are honestly DECLINED — cc_section_plane
 * returns an empty CCSection with cc_last_error set, NEVER a wrong or open section.
 * Under the native engine this is the OCCT-free section service; the OCCT engine
 * mirrors BRepAlgoAPI_Section. */
CCSection cc_section_plane(CCShapeId body, const double origin[3], const double normal[3]);
void cc_section_free(CCSection section);

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/* Release a body held in the kernel's shape registry. */
void cc_shape_release(CCShapeId body);

#endif /* CC_KERNEL_NO_PROTOTYPES */

#ifdef __cplusplus
}
#endif

#endif /* CYBERCADKERNEL_CC_KERNEL_H */
