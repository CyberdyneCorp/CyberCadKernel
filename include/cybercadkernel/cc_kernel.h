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

CCShapeId cc_guided_sweep(const double *profileXY, int profileCount,
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

/* ── Feature edits ───────────────────────────────────────────────────────── */

CCShapeId cc_fillet_edges(CCShapeId body, const int *edgeIds, int edgeCount, double radius);
CCShapeId cc_fillet_edges_variable(CCShapeId body, const int *edgeIds, int edgeCount, double radius1, double radius2);
CCShapeId cc_chamfer_edges(CCShapeId body, const int *edgeIds, int edgeCount, double distance);
CCShapeId cc_shell(CCShapeId body, const int *faceIds, int faceCount, double thickness);
CCShapeId cc_offset_face(CCShapeId body, int faceId, double distance);
CCShapeId cc_replace_face(CCShapeId body, int faceId, double offset, double tiltDeg);

CCShapeId cc_replace_face_to_plane(CCShapeId body, int faceId,
                                   double px, double py, double pz,
                                   double nx, double ny, double nz);

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

/* ── Queries ─────────────────────────────────────────────────────────────── */

CCMassProps cc_mass_properties(CCShapeId body);
/* Principal moments of inertia (unit-density volume inertia). out3 = [I1,I2,I3]. 1 on success. */
int cc_principal_moments(CCShapeId body, double *out3);

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

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/* Release a body held in the kernel's shape registry. */
void cc_shape_release(CCShapeId body);

#endif /* CC_KERNEL_NO_PROTOTYPES */

#ifdef __cplusplus
}
#endif

#endif /* CYBERCADKERNEL_CC_KERNEL_H */
