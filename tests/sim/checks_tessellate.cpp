// Tessellation / display-geometry module of the CyberCadKernel OCCT runtime
// suite. Exercises the three facade calls that turn a B-rep body into the POD
// display geometry the UI consumes, plus their matching *_free helpers:
//
//   cc_tessellate        — one triangle soup for the whole body   (+ cc_mesh_free)
//   cc_face_meshes        — per-face triangle meshes (pickable faces) (+ cc_face_meshes_free)
//   cc_edge_polylines     — per-edge 3-D polylines (pickable edges)  (+ cc_edge_polylines_free)
//
// Every assertion is anchored to a body with known analytic geometry: a
// 10x10x10 axis-aligned box (extrude a 10x10 square by 10 along +Z). For that
// box we know exactly:
//   volume  = 1000 mm^3      area = 600 mm^2
//   faces   = 6              edges = 12          each edge length = 10 mm
//   bbox    = [0,0,0]..[10,10,10]
// so the display geometry is validated against those closed-form values rather
// than a smoke "id != 0" check wherever a closed-form value exists.
//
// Concrete invariants asserted per function:
//   cc_tessellate    : mesh is watertight → divergence-theorem volume == 1000;
//                      vertex bbox == box bbox; box tessellates to 12 triangles;
//                      run-to-run byte-identical (determinism). Guard: bad id → empty.
//   cc_face_meshes    : 6 faces, ids 1..6, each planar quad = 2 tris / 4 verts,
//                      Σ face-mesh triangle area == 600. Guard: bad id → 0/null.
//   cc_edge_polylines : 12 edges, ids 1..12, each line = 2 points,
//                      Σ polyline length == 120. Guard: bad id → 0/null.

#include "checks.h"

#include <string>

namespace {

const double kSquare[8] = {0, 0, 10, 0, 10, 10, 0, 10};  // 10x10 profile (CCW)

// Signed volume of a closed, outward-oriented triangle mesh via the divergence
// theorem: Σ v0·(v1×v2)/6. For a correctly-oriented watertight solid this is the
// enclosed volume (positive), giving a strong analytic check on cc_tessellate.
double meshSignedVolume(const CCMesh& m) {
  double v6 = 0.0;
  for (int t = 0; t < m.triangleCount; ++t) {
    const int ia = m.triangles[t * 3], ib = m.triangles[t * 3 + 1], ic = m.triangles[t * 3 + 2];
    const double* a = &m.vertices[ia * 3];
    const double* b = &m.vertices[ib * 3];
    const double* c = &m.vertices[ic * 3];
    // b × c
    const double cx = b[1] * c[2] - b[2] * c[1];
    const double cy = b[2] * c[0] - b[0] * c[2];
    const double cz = b[0] * c[1] - b[1] * c[0];
    v6 += a[0] * cx + a[1] * cy + a[2] * cz;
  }
  return v6 / 6.0;
}

// Axis-aligned bounds over a mesh's vertices -> out6 = [minX,minY,minZ,maxX,maxY,maxZ].
void meshBBox(const CCMesh& m, double out6[6]) {
  out6[0] = out6[1] = out6[2] = 1e300;
  out6[3] = out6[4] = out6[5] = -1e300;
  for (int i = 0; i < m.vertexCount; ++i) {
    for (int d = 0; d < 3; ++d) {
      const double x = m.vertices[i * 3 + d];
      if (x < out6[d]) out6[d] = x;
      if (x > out6[3 + d]) out6[3 + d] = x;
    }
  }
}

// Sum of triangle areas in a per-face mesh (0.5·|(v1-v0)×(v2-v0)|).
double faceMeshArea(const CCFaceMesh& f) {
  double area = 0.0;
  for (int t = 0; t < f.triangleCount; ++t) {
    const int ia = f.triangles[t * 3], ib = f.triangles[t * 3 + 1], ic = f.triangles[t * 3 + 2];
    const double* a = &f.vertices[ia * 3];
    const double* b = &f.vertices[ib * 3];
    const double* c = &f.vertices[ic * 3];
    const double e1x = b[0] - a[0], e1y = b[1] - a[1], e1z = b[2] - a[2];
    const double e2x = c[0] - a[0], e2y = c[1] - a[1], e2z = c[2] - a[2];
    const double cx = e1y * e2z - e1z * e2y;
    const double cy = e1z * e2x - e1x * e2z;
    const double cz = e1x * e2y - e1y * e2x;
    area += 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
  }
  return area;
}

// Total arc length of a polyline (sum of consecutive segment lengths).
double polylineLength(const CCEdgePolyline& e) {
  double len = 0.0;
  for (int k = 1; k < e.pointCount; ++k) {
    const double* p0 = &e.points[(k - 1) * 3];
    const double* p1 = &e.points[k * 3];
    const double dx = p1[0] - p0[0], dy = p1[1] - p0[1], dz = p1[2] - p0[2];
    len += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return len;
}

// ── cc_tessellate ───────────────────────────────────────────────────────────
void checkTessellate(Ctx& ctx, CCShapeId box) {
  CCMesh m = cc_tessellate(box, 0.1);
  if (!ctx.check(m.vertexCount > 0 && m.triangleCount > 0 && m.vertices && m.triangles,
                 "cc_tessellate box -> non-empty mesh",
                 "v=" + std::to_string(m.vertexCount) + " t=" + std::to_string(m.triangleCount))) {
    cc_mesh_free(m);
    return;
  }

  // A box is 6 planar quads -> 2 triangles each -> exactly 12 triangles.
  ctx.check(m.triangleCount == 12, "cc_tessellate box -> 12 triangles",
            "got " + std::to_string(m.triangleCount));

  // Watertight-mesh volume (divergence theorem) == analytic 1000 mm^3. This also
  // proves outward triangle orientation (sign is positive).
  const double vol = meshSignedVolume(m);
  ctx.check(near(vol, 1000.0, 1e-6), "cc_tessellate mesh volume == 1000",
            "got " + std::to_string(vol));

  // Mesh vertex bounds == exact box bbox.
  double bb[6];
  meshBBox(m, bb);
  ctx.check(near(bb[0], 0, 1e-6) && near(bb[1], 0, 1e-6) && near(bb[2], 0, 1e-6) &&
                near(bb[3], 10, 1e-6) && near(bb[4], 10, 1e-6) && near(bb[5], 10, 1e-6),
            "cc_tessellate mesh bbox == [0,0,0]..[10,10,10]");

  // Determinism: a second tessellation at the same deflection is byte-identical.
  CCMesh m2 = cc_tessellate(box, 0.1);
  ctx.check(m2.vertices && hashMesh(m) == hashMesh(m2),
            "cc_tessellate deterministic (identical hash on re-run)");
  cc_mesh_free(m2);

  cc_mesh_free(m);

  // Guard: an unknown body id yields an empty mesh (no allocation, no crash).
  CCMesh bad = cc_tessellate(0, 0.1);
  ctx.check(bad.vertexCount == 0 && bad.triangleCount == 0 &&
                bad.vertices == nullptr && bad.triangles == nullptr,
            "cc_tessellate(invalid id) -> empty mesh (guard)");
  cc_mesh_free(bad);
}

// ── cc_face_meshes ──────────────────────────────────────────────────────────
void checkFaceMeshes(Ctx& ctx, CCShapeId box) {
  CCFaceMesh* faces = nullptr;
  const int n = cc_face_meshes(box, 0.1, &faces);
  if (!ctx.check(n == 6 && faces != nullptr, "cc_face_meshes box -> 6 faces",
                 "got " + std::to_string(n))) {
    cc_face_meshes_free(faces, n);
    return;
  }

  bool idsOk = true, quadOk = true, buffersOk = true;
  double totalArea = 0.0;
  for (int i = 0; i < n; ++i) {
    const CCFaceMesh& f = faces[i];
    if (f.faceId != i + 1) idsOk = false;                 // 1-based, in map order
    if (f.vertexCount != 4 || f.triangleCount != 2) quadOk = false;  // planar quad
    if (!f.vertices || !f.triangles) buffersOk = false;
    totalArea += faceMeshArea(f);
  }

  ctx.check(idsOk, "cc_face_meshes faceIds are 1..6 in order");
  ctx.check(buffersOk, "cc_face_meshes every face has vertex/triangle buffers");
  ctx.check(quadOk, "cc_face_meshes each box face = 4 verts / 2 tris");
  ctx.check(near(totalArea, 600.0, 1e-6), "cc_face_meshes Σ triangle area == 600",
            "got " + std::to_string(totalArea));

  cc_face_meshes_free(faces, n);

  // Guard: unknown id -> 0 faces, null out pointer.
  CCFaceMesh* badFaces = reinterpret_cast<CCFaceMesh*>(0x1);
  const int bn = cc_face_meshes(0, 0.1, &badFaces);
  ctx.check(bn == 0 && badFaces == nullptr, "cc_face_meshes(invalid id) -> 0/null (guard)",
            "got " + std::to_string(bn));
  cc_face_meshes_free(badFaces, bn);
}

// ── cc_edge_polylines ───────────────────────────────────────────────────────
void checkEdgePolylines(Ctx& ctx, CCShapeId box) {
  CCEdgePolyline* edges = nullptr;
  const int n = cc_edge_polylines(box, &edges);
  if (!ctx.check(n == 12 && edges != nullptr, "cc_edge_polylines box -> 12 edges",
                 "got " + std::to_string(n))) {
    cc_edge_polylines_free(edges, n);
    return;
  }

  bool idsOk = true, lineOk = true;
  double totalLen = 0.0;
  for (int i = 0; i < n; ++i) {
    const CCEdgePolyline& e = edges[i];
    if (e.edgeId != i + 1) idsOk = false;         // 1-based, matches cc_subshape_ids
    if (e.pointCount != 2 || !e.points) lineOk = false;  // straight edge -> 2 points
    totalLen += polylineLength(e);
  }

  ctx.check(idsOk, "cc_edge_polylines edgeIds are 1..12 in order");
  ctx.check(lineOk, "cc_edge_polylines each box edge = 2-point line");
  ctx.check(near(totalLen, 120.0, 1e-6), "cc_edge_polylines Σ length == 120 (12×10mm)",
            "got " + std::to_string(totalLen));

  cc_edge_polylines_free(edges, n);

  // Guard: unknown id -> 0 edges, null out pointer.
  CCEdgePolyline* badEdges = reinterpret_cast<CCEdgePolyline*>(0x1);
  const int bn = cc_edge_polylines(0, &badEdges);
  ctx.check(bn == 0 && badEdges == nullptr, "cc_edge_polylines(invalid id) -> 0/null (guard)",
            "got " + std::to_string(bn));
  cc_edge_polylines_free(badEdges, bn);
}

}  // namespace

void run_tessellate_checks(Ctx& ctx) {
  CCShapeId box = cc_solid_extrude(kSquare, 4, 10.0);
  if (!ctx.check(box != 0, "cc_solid_extrude -> 10x10x10 box (tessellate fixture)")) {
    return;
  }

  checkTessellate(ctx, box);
  checkFaceMeshes(ctx, box);
  checkEdgePolylines(ctx, box);

  cc_shape_release(box);
}
