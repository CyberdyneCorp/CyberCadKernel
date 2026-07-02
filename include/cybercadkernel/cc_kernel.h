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

/* ── Construction ────────────────────────────────────────────────────────── */

CCShapeId cc_solid_extrude(const double *profileXY, int pointCount, double depth);
CCShapeId cc_solid_revolve(const double *profileXY, int pointCount, double angleRadians);

CCShapeId cc_solid_loft(const double *bottomXY, int bottomCount,
                        const double *topXY, int topCount, double depth);

CCShapeId cc_solid_loft_wires(const double *aXYZ, int aCount,
                              const double *bXYZ, int bCount);

CCShapeId cc_solid_sweep(const double *profileXY, int profileCount,
                         const double *pathXYZ, int pathCount);

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

CCShapeId cc_solid_extrude_holes(const double *outerXY, int outerCount,
                                 const double *holesCenterRadius, int holeCount, double depth);

CCShapeId cc_solid_extrude_polyholes(const double *outerXY, int outerCount,
                                     const double *holesXY, const int *holeCounts,
                                     int holeCount, double depth);

CCShapeId cc_solid_extrude_profile(const CCProfileSeg *segs, int segCount,
                                   const double *holesCenterRadius, int holeCount,
                                   const double *splineXY, int splineXYCount, double depth);

CCShapeId cc_solid_extrude_profile_polyholes(const CCProfileSeg *segs, int segCount,
                                             const double *holesCenterRadius, int circleCount,
                                             const double *polyXY, const int *polyCounts, int polyCount,
                                             const double *splineXY, int splineXYCount, double depth);

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

/* Booleans: op = 0 fuse, 1 cut (a-b), 2 common. */
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

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/* Release a body held in the kernel's shape registry. */
void cc_shape_release(CCShapeId body);

#endif /* CC_KERNEL_NO_PROTOTYPES */

#ifdef __cplusplus
}
#endif

#endif /* CYBERCADKERNEL_CC_KERNEL_H */
