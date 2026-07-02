// Feature-edit correctness module for the CyberCadKernel OCCT runtime suite.
//
// Exercises the cc_* feature edits through the real OCCT adapter with valid
// inputs and asserts CONCRETE analytic invariants wherever one exists (exact
// solid volumes / bounding boxes derived by hand), plus guard behaviour (a
// deliberately degenerate input must return 0 and leave the caller's body).
//
// Base solid for every check: a 10x10x10 box (extrude the 10x10 square profile
// by 10). Its exact properties are volume 1000, area 600, 6 faces, 12 edges,
// every edge length 10 and every dihedral a convex 90°. Those constants drive
// the analytic assertions below.
//
// Face/edge ids are resolved at run time (not hard-coded) so the checks are
// independent of OCCT's sub-shape ordering: findPlanarFace() locates a face by
// the plane all its vertices lie on (via cc_face_meshes), and any single box
// edge is interchangeable for the length-10 fillet/chamfer math.

#include "checks.h"

#include <cmath>
#include <string>

namespace {

// 10x10 square profile → extrudes to a 10x10x10 box.
const double kSquare[8] = {0, 0, 10, 0, 10, 10, 0, 10};

// Removing a convex 90° edge of a solid with a fillet of radius r deletes, per
// unit edge length, the square corner minus the quarter disc: r^2 * (1 - π/4).
const double kFilletFactor = 1.0 - M_PI / 4.0;   // ≈ 0.2146018

std::string num(double v) { return std::to_string(v); }

// Exact B-rep volume of a body, or -1 if the id is unknown / OCCT unavailable.
double volumeOf(CCShapeId id) {
  const CCMassProps mp = cc_mass_properties(id);
  return mp.valid ? mp.volume : -1.0;
}

// Face id (1-based, as cc_shell / cc_offset_face expect) of the face whose
// every vertex lies on the plane axis==value (axis: 0=x,1=y,2=z). Returns 0 if
// no such face is found. Used to pick the box's top face (axis 2, value 10)
// without depending on OCCT's face ordering.
int findPlanarFace(CCShapeId body, int axis, double value, double tol) {
  CCFaceMesh* faces = nullptr;
  const int n = cc_face_meshes(body, 0.1, &faces);
  int found = 0;
  for (int i = 0; i < n && found == 0; ++i) {
    const CCFaceMesh& f = faces[i];
    if (f.vertexCount <= 0 || f.vertices == nullptr) { continue; }
    bool onPlane = true;
    for (int v = 0; v < f.vertexCount && onPlane; ++v) {
      if (std::fabs(f.vertices[v * 3 + axis] - value) > tol) { onPlane = false; }
    }
    if (onPlane) { found = f.faceId; }
  }
  cc_face_meshes_free(faces, n);
  return found;
}

// ── cc_fillet_edges: analytic volume ────────────────────────────────────────
// One convex edge (length 10), radius 1. Ends are flat (the two adjacent faces
// are perpendicular to the edge, and their edges are NOT filleted), so the
// removed volume is exactly 10 * 1^2 * (1 - π/4).
void check_fillet_edges(Ctx& ctx, CCShapeId box, int edge) {
  const int ids[1] = {edge};
  const CCShapeId f = cc_fillet_edges(box, ids, 1, 1.0);
  if (ctx.check(f != 0, "cc_fillet_edges(r=1) -> valid id")) {
    const double removed = 10.0 * 1.0 * 1.0 * kFilletFactor;   // ≈ 2.146
    const double v = volumeOf(f);
    ctx.check(near(v, 1000.0 - removed, 1e-3),
              "fillet one edge volume == 1000 - 10*r^2*(1-π/4)",
              "want " + num(1000.0 - removed) + " got " + num(v));
    cc_shape_release(f);
  }
  // Guard: radius <= 0 must return 0 (caller keeps its body).
  ctx.check(cc_fillet_edges(box, ids, 1, 0.0) == 0, "cc_fillet_edges(r=0) guarded -> 0");
  ctx.check(cc_fillet_edges(0, ids, 1, 1.0) == 0, "cc_fillet_edges(invalid body) -> 0");
}

// ── cc_fillet_edges_variable: analytic volume ───────────────────────────────
// One edge (length 10), radius 0.5 → 1.5 varying linearly in arc length.
// Removed = (1-π/4) * ∫_0^10 r(t)^2 dt, r(t)=0.5+0.1t → ∫ = (10/3)(1.5^3-0.5^3)
// = 10.8333, so removed ≈ 2.3249. Also bounded by the uniform-radius fillets.
void check_fillet_variable(Ctx& ctx, CCShapeId box, int edge) {
  const int ids[1] = {edge};
  const CCShapeId f = cc_fillet_edges_variable(box, ids, 1, 0.5, 1.5);
  if (ctx.check(f != 0, "cc_fillet_edges_variable(0.5->1.5) -> valid id")) {
    const double integral = (10.0 / 3.0) * (1.5 * 1.5 * 1.5 - 0.5 * 0.5 * 0.5);  // 10.8333
    const double removed = kFilletFactor * integral;   // ≈ 2.3249
    const double v = volumeOf(f);
    ctx.check(near(v, 1000.0 - removed, 5e-2),
              "variable fillet volume == 1000 - (1-π/4)∫r(t)^2",
              "want " + num(1000.0 - removed) + " got " + num(v));
    // Bracket: removes more than a uniform r=0.5 and less than a uniform r=1.5.
    const double vLo = 1000.0 - 10.0 * 1.5 * 1.5 * kFilletFactor;   // r=1.5 → 995.17
    const double vHi = 1000.0 - 10.0 * 0.5 * 0.5 * kFilletFactor;   // r=0.5 → 999.46
    ctx.check(v > vLo - 1e-6 && v < vHi + 1e-6,
              "variable fillet volume within uniform r bracket",
              "(" + num(vLo) + "," + num(vHi) + ") got " + num(v));
    cc_shape_release(f);
  }
  // Guard: both radii <= 0 must return 0.
  ctx.check(cc_fillet_edges_variable(box, ids, 1, 0.0, 0.0) == 0,
            "cc_fillet_edges_variable(0,0) guarded -> 0");
}

// ── cc_chamfer_edges: analytic volume ───────────────────────────────────────
// One edge (length 10), distance 1. Removed cross-section is a right isosceles
// triangle of legs 1 → area 0.5; flat perpendicular ends → removed = 10*0.5 = 5.
void check_chamfer(Ctx& ctx, CCShapeId box, int edge) {
  const int ids[1] = {edge};
  const CCShapeId f = cc_chamfer_edges(box, ids, 1, 1.0);
  if (ctx.check(f != 0, "cc_chamfer_edges(d=1) -> valid id")) {
    const double v = volumeOf(f);
    ctx.check(near(v, 995.0, 1e-3), "chamfer one edge volume == 1000 - 10*d^2/2 == 995",
              "got " + num(v));
    cc_shape_release(f);
  }
  ctx.check(cc_chamfer_edges(box, ids, 1, 0.0) == 0, "cc_chamfer_edges(d=0) guarded -> 0");
}

// ── cc_shell: analytic volume ───────────────────────────────────────────────
// Hollow the box to wall thickness 1, opening the top face. Inner cavity is
// 8 x 8 x 9 (walls of 1 on the four sides and the bottom, top open) = 576, so
// the shell volume is exactly 1000 - 576 = 424.
void check_shell(Ctx& ctx, CCShapeId box, int topFace) {
  const int ids[1] = {topFace};
  const CCShapeId f = cc_shell(box, ids, 1, 1.0);
  if (ctx.check(f != 0, "cc_shell(t=1, open top) -> valid id")) {
    const double v = volumeOf(f);
    ctx.check(near(v, 424.0, 1e-2), "shell volume == 1000 - 8*8*9 == 424", "got " + num(v));
    cc_shape_release(f);
  }
  ctx.check(cc_shell(box, ids, 1, 0.0) == 0, "cc_shell(t=0) guarded -> 0");
}

// ── cc_offset_face: analytic volume + bbox ──────────────────────────────────
// Push the top face outward by +5 (fuse a prism) → a 10x10x15 box, volume 1500,
// max Z 15. Inward -5 (cut) → 10x10x5, volume 500.
void check_offset_face(Ctx& ctx, CCShapeId box, int topFace) {
  const CCShapeId out = cc_offset_face(box, topFace, 5.0);
  if (ctx.check(out != 0, "cc_offset_face(+5) -> valid id")) {
    const double v = volumeOf(out);
    ctx.check(near(v, 1500.0, 1e-3), "offset top face +5 volume == 1500", "got " + num(v));
    double bb[6] = {0};
    ctx.check(cc_bounding_box(out, bb) == 1 && near(bb[5], 15.0, 1e-3),
              "offset +5 max Z == 15", "got " + num(bb[5]));
    cc_shape_release(out);
  }
  const CCShapeId in = cc_offset_face(box, topFace, -5.0);
  if (ctx.check(in != 0, "cc_offset_face(-5) -> valid id")) {
    ctx.check(near(volumeOf(in), 500.0, 1e-3), "offset top face -5 volume == 500",
              "got " + num(volumeOf(in)));
    cc_shape_release(in);
  }
  ctx.check(cc_offset_face(box, topFace, 0.0) == 0, "cc_offset_face(0) guarded -> 0");
}

// ── cc_replace_face: analytic volume ────────────────────────────────────────
// Retarget the top face to a plane offset -5 along its outward (+Z) normal, no
// tilt: the plane sits at Z=5 and the outward (+Z) half is trimmed away → the
// box is cut to Z in [0,5] = 10x10x5, volume 500.
void check_replace_face(Ctx& ctx, CCShapeId box, int topFace) {
  const CCShapeId f = cc_replace_face(box, topFace, -5.0, 0.0);
  if (ctx.check(f != 0, "cc_replace_face(offset=-5, tilt=0) -> valid id")) {
    const double v = volumeOf(f);
    ctx.check(near(v, 500.0, 1e-2), "replace top face to Z=5 volume == 500", "got " + num(v));
    cc_shape_release(f);
  }
}

// ── cc_replace_face_to_plane: analytic volume ───────────────────────────────
// Retarget the top face to the explicit plane point (0,0,6) normal +Z: the
// outward half (Z>6) is trimmed → box cut to Z in [0,6] = 10x10x6, volume 600.
void check_replace_to_plane(Ctx& ctx, CCShapeId box, int topFace) {
  const CCShapeId f = cc_replace_face_to_plane(box, topFace, 0, 0, 6, 0, 0, 1);
  if (ctx.check(f != 0, "cc_replace_face_to_plane(z=6) -> valid id")) {
    const double v = volumeOf(f);
    ctx.check(near(v, 600.0, 1e-2), "replace top face to plane Z=6 volume == 600",
              "got " + num(v));
    cc_shape_release(f);
  }
  // Guard: a zero-length target normal must return 0.
  ctx.check(cc_replace_face_to_plane(box, topFace, 0, 0, 6, 0, 0, 0) == 0,
            "cc_replace_face_to_plane(zero normal) guarded -> 0");
}

// ── cc_fillet_face: bounded volume ──────────────────────────────────────────
// Round every edge of the top face (four convex edges of length 10, radius 1).
// A tight closed form is impractical (the four filleted edges blend at the top
// corners), so assert a real bound: the result is a valid solid, material is
// removed, and the removed amount is ≈ 4 edges * 10 * (1-π/4) ≈ 8.58 (the corner
// blends make the true figure a little smaller), with the X/Y extent preserved.
void check_fillet_face(Ctx& ctx, CCShapeId box, int topFace) {
  const CCShapeId f = cc_fillet_face(box, topFace, 1.0);
  if (ctx.check(f != 0, "cc_fillet_face(r=1) -> valid id")) {
    const double v = volumeOf(f);
    ctx.check(v > 985.0 && v < 999.9, "fillet_face removed material, near 4-edge estimate",
              "expect (985,999.9) got " + num(v));
    double bb[6] = {0};
    ctx.check(cc_bounding_box(f, bb) == 1 && near(bb[3], 10.0, 1e-3) && near(bb[0], 0.0, 1e-3),
              "fillet_face preserves X extent [0,10]");
    cc_shape_release(f);
  }
  ctx.check(cc_fillet_face(box, topFace, 0.0) == 0, "cc_fillet_face(r=0) guarded -> 0");
}

// ── cc_split_plane: analytic volume ─────────────────────────────────────────
// Cut the box by the plane through (0,0,3) with normal +Z. Keep-positive keeps
// Z>3 (a 10x10x7 slab = 700); keep-negative keeps Z<3 (10x10x3 = 300).
void check_split_plane(Ctx& ctx, CCShapeId box) {
  const CCShapeId hi = cc_split_plane(box, 0, 0, 3, 0, 0, 1, 1);
  if (ctx.check(hi != 0, "cc_split_plane(z=3, keep +) -> valid id")) {
    ctx.check(near(volumeOf(hi), 700.0, 1e-3), "split keep +Z half volume == 700",
              "got " + num(volumeOf(hi)));
    cc_shape_release(hi);
  }
  const CCShapeId lo = cc_split_plane(box, 0, 0, 3, 0, 0, 1, 0);
  if (ctx.check(lo != 0, "cc_split_plane(z=3, keep -) -> valid id")) {
    ctx.check(near(volumeOf(lo), 300.0, 1e-3), "split keep -Z half volume == 300",
              "got " + num(volumeOf(lo)));
    cc_shape_release(lo);
  }
  // Guard: a zero-length plane normal must return 0.
  ctx.check(cc_split_plane(box, 0, 0, 3, 0, 0, 0, 1) == 0,
            "cc_split_plane(zero normal) guarded -> 0");
}

// ── cc_offset_face_boundary: analytic bbox ──────────────────────────────────
// Offset the top face's outer boundary inward by 1: the 10x10 square shrinks to
// an 8x8 square at Z=10, i.e. its XY bounding box becomes [1,1]..[9,9], Z==10.
void check_offset_boundary(Ctx& ctx, CCShapeId box, int topFace) {
  double* xyz = nullptr;
  const int n = cc_offset_face_boundary(box, topFace, -1.0, &xyz);
  if (ctx.check(n >= 4 && xyz != nullptr, "cc_offset_face_boundary(-1) -> polyline",
                "points " + num(n))) {
    double xmn = xyz[0], xmx = xyz[0], ymn = xyz[1], ymx = xyz[1];
    bool zFlat = true;
    for (int i = 0; i < n; ++i) {
      xmn = std::fmin(xmn, xyz[i * 3]);     xmx = std::fmax(xmx, xyz[i * 3]);
      ymn = std::fmin(ymn, xyz[i * 3 + 1]); ymx = std::fmax(ymx, xyz[i * 3 + 1]);
      if (std::fabs(xyz[i * 3 + 2] - 10.0) > 1e-3) { zFlat = false; }
    }
    ctx.check(near(xmn, 1.0, 1e-3) && near(xmx, 9.0, 1e-3) && near(ymn, 1.0, 1e-3)
                  && near(ymx, 9.0, 1e-3),
              "inward-1 boundary bbox == [1,1]..[9,9]",
              "x[" + num(xmn) + "," + num(xmx) + "] y[" + num(ymn) + "," + num(ymx) + "]");
    ctx.check(zFlat, "inward-1 boundary stays on Z=10 plane");
  }
  cc_points_free(xyz);
  // Guard: an out-of-range faceId must return 0 and leave *outXYZ null.
  double* bad = nullptr;
  ctx.check(cc_offset_face_boundary(box, 9999, -1.0, &bad) == 0 && bad == nullptr,
            "cc_offset_face_boundary(bad faceId) guarded -> 0");
}

}  // namespace

void run_feature_checks(Ctx& ctx) {
  std::printf("-- feature edits --\n");
  std::fflush(stdout);

  const CCShapeId box = cc_solid_extrude(kSquare, 4, 10.0);
  if (!ctx.check(box != 0, "feature base: 10x10x10 box built")) { return; }

  // Resolve the box's top face (all vertices at Z=10) and pick any single edge;
  // every box edge has length 10 so the fillet/chamfer analytics are edge-agnostic.
  const int topFace = findPlanarFace(box, 2, 10.0, 1e-6);
  if (!ctx.check(topFace != 0, "feature base: located top face (Z=10)")) {
    cc_shape_release(box);
    return;
  }
  int* edges = nullptr;
  const int nEdges = cc_subshape_ids(box, 1, &edges);
  const int edge0 = (nEdges > 0 && edges != nullptr) ? edges[0] : 1;
  ctx.check(nEdges == 12, "feature base: box has 12 edges", "got " + num(nEdges));

  check_fillet_edges(ctx, box, edge0);
  check_fillet_variable(ctx, box, edge0);
  check_chamfer(ctx, box, edge0);
  check_shell(ctx, box, topFace);
  check_offset_face(ctx, box, topFace);
  check_replace_face(ctx, box, topFace);
  check_replace_to_plane(ctx, box, topFace);
  check_fillet_face(ctx, box, topFace);
  check_split_plane(ctx, box);
  check_offset_boundary(ctx, box, topFace);

  if (edges != nullptr) { cc_ints_free(edges); }
  cc_shape_release(box);
}
