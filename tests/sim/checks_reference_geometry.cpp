// Phase-3 reference-geometry checks (iOS simulator; run by phase3_suite.cpp).
//
// The point-only trio (cc_ref_plane_from_points / cc_ref_plane_offset /
// cc_ref_axis_from_points) is exact fp64 math that lands facade-side, asserted
// here against known analytic values (host + OCCT). The DERIVED trio
// (cc_ref_plane_from_face / cc_ref_axis_from_edge / cc_ref_axis_from_face) reads a
// body's geometry through the OCCT adapter; it is exercised on a known box + a
// known cylinder and asserted against the EXACT analytic normals/directions of
// those primitives (a box's ±X/±Y/±Z faces, its four vertical edges, and a
// cylinder axis that must match cc_face_axis bit-for-bit). Degenerate inputs
// (non-planar face, non-cyl face, unknown id) must return 0.

#include "phase3_checks.h"

#include <cmath>
#include <string>

namespace {

// Unit-length within 1e-9 (the spec's guarantee for a successful constructor).
bool isUnit(const double v[3]) {
  return near(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]), 1.0, 1e-9);
}

std::string num(double v) {
  char b[64];
  std::snprintf(b, sizeof b, "%.12g", v);
  return b;
}

}  // namespace

void run_reference_geometry_checks(Ctx& ctx) {
  std::printf("-- reference geometry --\n");

  // ── Point-only trio: exact analytic values ─────────────────────────────────

  // Plane through (0,0,5),(1,0,5),(0,1,5): origin p0, unit normal ±(0,0,1).
  {
    const double p0[3] = {0, 0, 5}, p1[3] = {1, 0, 5}, p2[3] = {0, 1, 5};
    double o[6] = {0};
    const bool ok = cc_ref_plane_from_points(p0, p1, p2, o) == 1;
    ctx.check(ok && near(o[0], 0, 1e-9) && near(o[1], 0, 1e-9) && near(o[2], 5, 1e-9) &&
                  isUnit(o + 3) && near(std::fabs(o[5]), 1.0, 1e-9),
              "cc_ref_plane_from_points: z=5 plane origin + unit +Z normal");
  }
  // Colinear points must fail.
  {
    const double a[3] = {0, 0, 0}, b[3] = {1, 1, 1}, c[3] = {2, 2, 2};
    double o[6] = {0};
    ctx.check(cc_ref_plane_from_points(a, b, c, o) == 0,
              "cc_ref_plane_from_points: colinear points fail");
  }
  // Offset plane: O=(2,3,4), N=(0,0,2), dist=5 -> origin (2,3,9), unit +Z.
  {
    const double o0[3] = {2, 3, 4}, n[3] = {0, 0, 2};
    double o[6] = {0};
    const bool ok = cc_ref_plane_offset(o0, n, 5.0, o) == 1;
    ctx.check(ok && near(o[0], 2, 1e-9) && near(o[1], 3, 1e-9) && near(o[2], 9, 1e-9) &&
                  near(o[5], 1.0, 1e-9) && isUnit(o + 3),
              "cc_ref_plane_offset: origin moves dist along unit normal");
  }
  // Negative offset moves the origin the other way: O=(2,3,4), N=(0,0,1), dist=-4 -> (2,3,0).
  {
    const double o0[3] = {2, 3, 4}, n[3] = {0, 0, 1};
    double o[6] = {0};
    const bool ok = cc_ref_plane_offset(o0, n, -4.0, o) == 1;
    ctx.check(ok && near(o[2], 0, 1e-9) && isUnit(o + 3),
              "cc_ref_plane_offset: negative dist moves origin backward along normal");
  }
  // Zero-length normal must fail.
  {
    const double o0[3] = {0, 0, 0}, n[3] = {0, 0, 0};
    double o[6] = {0};
    ctx.check(cc_ref_plane_offset(o0, n, 5.0, o) == 0, "cc_ref_plane_offset: zero normal fails");
  }
  // Axis through (1,1,1)->(1,1,4): origin a, unit +Z.
  {
    const double a[3] = {1, 1, 1}, b[3] = {1, 1, 4};
    double o[6] = {0};
    const bool ok = cc_ref_axis_from_points(a, b, o) == 1;
    ctx.check(ok && near(o[0], 1, 1e-9) && near(o[1], 1, 1e-9) && near(o[2], 1, 1e-9) &&
                  near(o[5], 1.0, 1e-9) && isUnit(o + 3),
              "cc_ref_axis_from_points: origin a + unit +Z direction");
  }
  // Coincident points must fail.
  {
    const double a[3] = {1, 1, 1}, b[3] = {1, 1, 1};
    double o[6] = {0};
    ctx.check(cc_ref_axis_from_points(a, b, o) == 0,
              "cc_ref_axis_from_points: coincident points fail");
  }

  // ── Derived trio: box (planar faces + linear edges) ────────────────────────
  // sq spans x∈[-10,10], y∈[-5,5]; extrude +Z by 30 -> base z=0, top z=30.
  const double sq[8] = {-10, -5, 10, -5, 10, 5, -10, 5};
  const CCShapeId box = cc_solid_extrude(sq, 4, 30.0);
  if (box == 0) {
    ctx.defer("reference-geometry derived trio", "base box build returned 0");
    return;
  }

  // Unknown body id must fail (guard).
  {
    double o[6] = {0};
    ctx.check(cc_ref_plane_from_face(0, 1, o) == 0,
              "cc_ref_plane_from_face: unknown body id returns 0");
    ctx.check(cc_ref_axis_from_edge(0, 1, o) == 0,
              "cc_ref_axis_from_edge: unknown body id returns 0");
    ctx.check(cc_ref_axis_from_face(0, 1, o) == 0,
              "cc_ref_axis_from_face: unknown body id returns 0");
  }

  // cc_ref_plane_from_face over every face of the box.
  {
    int* faces = nullptr;
    const int nface = cc_subshape_ids(box, 2, &faces);
    int planarOk = 0, nonUnit = 0, topZ = 0, botZ = 0, offFace = 0;
    for (int i = 0; i < nface; ++i) {
      double o[6] = {0};
      if (cc_ref_plane_from_face(box, faces[i], o) != 1) {
        continue;  // every box face is planar; a 0 here is caught by planarOk count
      }
      ++planarOk;
      if (!isUnit(o + 3)) { ++nonUnit; }
      // Outward +Z face: normal (0,0,1), origin on the top plane z=30 and inside the
      // rectangle -> proves the origin lies ON the face, not merely on the plane.
      if (near(o[3], 0, 1e-9) && near(o[4], 0, 1e-9) && near(o[5], 1.0, 1e-9)) {
        ++topZ;
        const bool onFace = near(o[2], 30.0, 1e-9) && o[0] >= -10 - 1e-9 && o[0] <= 10 + 1e-9 &&
                            o[1] >= -5 - 1e-9 && o[1] <= 5 + 1e-9;
        if (!onFace) { ++offFace; }
      }
      if (near(o[3], 0, 1e-9) && near(o[4], 0, 1e-9) && near(o[5], -1.0, 1e-9)) {
        ++botZ;
        if (!(near(o[2], 0.0, 1e-9))) { ++offFace; }
      }
    }
    if (faces) { cc_ints_free(faces); }
    ctx.check(planarOk == 6, "cc_ref_plane_from_face: all 6 box faces resolve a plane",
              "got " + std::to_string(planarOk));
    ctx.check(nonUnit == 0, "cc_ref_plane_from_face: every returned normal is unit within 1e-9",
              std::to_string(nonUnit) + " non-unit");
    ctx.check(topZ == 1 && offFace == 0,
              "cc_ref_plane_from_face: unique +Z face, origin ON the top face (z=30, inside rect)");
    ctx.check(botZ == 1, "cc_ref_plane_from_face: unique outward -Z bottom face at z=0");
  }

  // cc_ref_axis_from_edge over every edge of the box: all 12 are linear; exactly
  // 4 are vertical (unit ±Z direction).
  {
    int* edges = nullptr;
    const int nedge = cc_subshape_ids(box, 1, &edges);
    int linearOk = 0, nonUnit = 0, vertical = 0;
    for (int i = 0; i < nedge; ++i) {
      double o[6] = {0};
      if (cc_ref_axis_from_edge(box, edges[i], o) != 1) { continue; }
      ++linearOk;
      if (!isUnit(o + 3)) { ++nonUnit; }
      if (near(std::fabs(o[5]), 1.0, 1e-9) && near(o[3], 0, 1e-9) && near(o[4], 0, 1e-9)) {
        ++vertical;
      }
    }
    if (edges) { cc_ints_free(edges); }
    ctx.check(linearOk == 12, "cc_ref_axis_from_edge: all 12 box edges resolve a linear axis",
              "got " + std::to_string(linearOk));
    ctx.check(nonUnit == 0, "cc_ref_axis_from_edge: every returned direction is unit within 1e-9",
              std::to_string(nonUnit) + " non-unit");
    ctx.check(vertical == 4, "cc_ref_axis_from_edge: exactly 4 vertical (±Z) edges",
              "got " + std::to_string(vertical));
  }

  cc_shape_release(box);

  // ── Derived: cylinder (cc_ref_axis_from_face == cc_face_axis) ──────────────
  {
    CCProfileSeg circle{};
    circle.kind = 2;  // full circle
    circle.cx = 0.0;
    circle.cy = 0.0;
    circle.r = 5.0;
    const CCShapeId cyl = cc_solid_extrude_profile(&circle, 1, nullptr, 0, nullptr, 0, 10.0);
    if (cyl == 0) {
      ctx.defer("cc_ref_axis_from_face (cylinder)", "cylinder build returned 0");
    } else {
      int* faces = nullptr;
      const int nface = cc_subshape_ids(cyl, 2, &faces);
      int cylMatched = 0, cylMismatch = 0, planarRejected = 0, planeOnCap = 0, planeOnCyl = 0;
      double axis[6] = {0};
      for (int i = 0; i < nface; ++i) {
        double ra[6] = {0}, fa[6] = {0};
        const int rIsAxis = cc_ref_axis_from_face(cyl, faces[i], ra);
        const int fIsAxis = cc_face_axis(cyl, faces[i], fa);
        // ref_axis_from_face MUST agree with face_axis on which faces have an axis.
        if (rIsAxis == 1 && fIsAxis == 1) {
          ++cylMatched;
          bool eq = true;
          for (int k = 0; k < 6; ++k) {
            if (!near(ra[k], fa[k], 1e-9)) { eq = false; }
          }
          if (!eq) { ++cylMismatch; }
          for (int k = 0; k < 6; ++k) { axis[k] = ra[k]; }
        } else if (rIsAxis == 0 && fIsAxis == 0) {
          ++planarRejected;
        } else {
          ++cylMismatch;  // disagreement on axis-ness
        }
        // Planar caps must yield a plane; the cylindrical wall must NOT.
        double pl[6] = {0};
        const int isPlane = cc_ref_plane_from_face(cyl, faces[i], pl);
        if (fIsAxis == 1) {
          if (isPlane == 0) { ++planeOnCyl; }  // correctly rejected the curved wall
        } else {
          if (isPlane == 1 && isUnit(pl + 3)) { ++planeOnCap; }  // planar cap -> plane
        }
      }
      if (faces) { cc_ints_free(faces); }
      ctx.check(cylMatched == 1 && cylMismatch == 0,
                "cc_ref_axis_from_face: exactly one cyl face, out6 == cc_face_axis within 1e-9");
      ctx.check(cylMatched == 1 && near(std::fabs(axis[5]), 1.0, 1e-9) &&
                    near(axis[3], 0, 1e-9) && near(axis[4], 0, 1e-9),
                "cc_ref_axis_from_face: cylinder axis is unit ±Z",
                "dir=(" + num(axis[3]) + "," + num(axis[4]) + "," + num(axis[5]) + ")");
      ctx.check(planarRejected >= 1,
                "cc_ref_axis_from_face: returns 0 for planar cap faces (matches face_axis guard)");
      ctx.check(planeOnCyl == 1,
                "cc_ref_plane_from_face: non-planar cylindrical face returns 0");
      ctx.check(planeOnCap >= 1,
                "cc_ref_plane_from_face: planar cap resolves a unit-normal plane");
      cc_shape_release(cyl);
    }
  }
}
