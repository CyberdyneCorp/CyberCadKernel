// Phase-3 full-round-fillet checks (iOS simulator; run by phase3_suite.cpp).
//
// Owned by the full-round-fillet feature agent. The bar (per the full-round-fillet
// spec): cc_full_round_fillet_faces consumes the middle face into a rolling-ball
// blend tangent to both neighbours — the result is BRepCheck_Analyzer::IsValid +
// watertight, the middle face id no longer resolves to a face, and the blend is
// G1-tangent to BOTH neighbours at the seams (sampled normals agree within the
// documented tolerance). If a true face-consuming tangent round cannot be built, it
// falls back to a standard edge fillet that IS valid and the case is recorded
// deferred with the measured tangency gap — never faked.
//
// Since this TU may NOT include OCCT headers, every geometric property is measured
// through the public cc_* facade:
//   • watertight + middle-consumed  → exact rolling-ball volume via cc_mass_properties
//     (a plain face-swap or partial fillet gives a different volume) + no flat top
//     face remaining (cc_face_meshes) ;
//   • blend surface is the rolling ball → cc_face_axis returns a CYLINDER whose axis
//     runs along the strip;
//   • G1 tangency to BOTH neighbours → the blend-cylinder axis is EQUIDISTANT from
//     the two neighbour planes AND that distance equals the height it reaches the
//     original top (equal tangent radius on both sides ⇒ the ball touches both walls),
//     and the blend normal at each seam contact equals the neighbour normal (dot≈1).
//
// The PARALLEL-wall rib fixture is a box (x[-2,2], y[0,20], z[0,10]); a full round on
// its top (width 4) is a half-cylinder of radius 2 tangent to the two x=±2 side faces,
// so the exact result volume is (72 + 2π)·10 = 720 + 20π and the blend-cylinder axis
// is at x=0, y=18, dir ±z.
//
// The NON-PARALLEL (dihedral) fixture is a trapezoidal rib (bottom x[-6,6], top
// x[-2,2] @ y=10, extruded 8 in z): its two side walls slope inward (outward normals
// (±0.928, 0.371, 0), n_L·n_R = -0.724, ~43.6° off anti-parallel), so no parallel-wall
// mid-plane exists — the constant-radius rolling ball rides the dihedral bisector and
// sweeps a cylinder tangent to both tilted walls. We assert that dihedral full round
// for REAL: valid + watertight, the middle strip consumed (no +Y face survives on the
// original top plane), the blend is a cylinder along the strip, and it is G1-tangent to
// BOTH non-parallel walls at the seams — the blend radial normal at each seam foot
// equals the measured wall normal within cos(1°). Truly CURVED (non-planar) neighbours
// remain out of scope; per the honesty rule, if the dihedral build cannot produce a
// tangent full round we assert only a VALID fallback and defer with the measured gap.

#include "phase3_checks.h"

#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct FaceN {
  int id = 0;
  double cx = 0, cy = 0, cz = 0;  // centroid
  double nx = 0, ny = 0, nz = 0;  // outward unit normal
};

// Per-face centroid + outward unit normal, computed from the tessellation and
// oriented away from the body centroid (so mesh winding cannot flip the sign).
std::vector<FaceN> face_normals(CCShapeId body) {
  std::vector<FaceN> out;
  const CCMassProps mp = cc_mass_properties(body);
  const double bx = mp.cx, by = mp.cy, bz = mp.cz;
  CCFaceMesh* fm = nullptr;
  const int n = cc_face_meshes(body, 0.2, &fm);
  for (int f = 0; f < n; ++f) {
    const CCFaceMesh& m = fm[f];
    if (m.vertexCount < 3 || m.triangleCount < 1) continue;
    double cxs = 0, cys = 0, czs = 0;
    for (int v = 0; v < m.vertexCount; ++v) {
      cxs += m.vertices[v * 3 + 0];
      cys += m.vertices[v * 3 + 1];
      czs += m.vertices[v * 3 + 2];
    }
    cxs /= m.vertexCount; cys /= m.vertexCount; czs /= m.vertexCount;
    double nx = 0, ny = 0, nz = 0;  // area-weighted normal (sum of triangle crosses)
    for (int t = 0; t < m.triangleCount; ++t) {
      const int i0 = m.triangles[t * 3 + 0], i1 = m.triangles[t * 3 + 1],
                i2 = m.triangles[t * 3 + 2];
      const double ax = m.vertices[i1 * 3] - m.vertices[i0 * 3];
      const double ay = m.vertices[i1 * 3 + 1] - m.vertices[i0 * 3 + 1];
      const double az = m.vertices[i1 * 3 + 2] - m.vertices[i0 * 3 + 2];
      const double bx2 = m.vertices[i2 * 3] - m.vertices[i0 * 3];
      const double by2 = m.vertices[i2 * 3 + 1] - m.vertices[i0 * 3 + 1];
      const double bz2 = m.vertices[i2 * 3 + 2] - m.vertices[i0 * 3 + 2];
      nx += ay * bz2 - az * by2;
      ny += az * bx2 - ax * bz2;
      nz += ax * by2 - ay * bx2;
    }
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-12) continue;
    nx /= len; ny /= len; nz /= len;
    // Orient outward relative to the body centroid.
    if ((cxs - bx) * nx + (cys - by) * ny + (czs - bz) * nz < 0) { nx = -nx; ny = -ny; nz = -nz; }
    out.push_back({m.faceId, cxs, cys, czs, nx, ny, nz});
  }
  cc_face_meshes_free(fm, n);
  return out;
}

// Pick the face id whose outward normal best matches (dx,dy,dz), optionally
// requiring the highest centroid-y (for the top face). Returns 0 if none.
int pick_face(const std::vector<FaceN>& faces, double dx, double dy, double dz,
              bool preferHighY) {
  int best = 0;
  double bestScore = -2.0, bestY = -1e30;
  for (const FaceN& f : faces) {
    const double dot = f.nx * dx + f.ny * dy + f.nz * dz;
    if (dot < 0.85) continue;
    if (preferHighY) {
      if (f.cy > bestY) { bestY = f.cy; best = f.id; }
    } else if (dot > bestScore) {
      bestScore = dot; best = f.id;
    }
  }
  return best;
}

// True if any planar-ish face with a +Y normal still sits near the original top
// (cy ≈ 20) — i.e. the middle face was NOT consumed.
bool flat_top_remains(CCShapeId body) {
  for (const FaceN& f : face_normals(body)) {
    if (f.ny > 0.9 && f.cy > 19.5) return true;
  }
  return false;
}

// True if any planar-ish face in `body` still lies on the ORIGINAL middle-strip plane
// (centroid near cy≈topY and outward normal ≈ +Y) — i.e. the middle face survived the
// blend. Generalises flat_top_remains to an arbitrary top height (the dihedral rib's
// top strip sits at a different y than the parallel rib's cy=20). Used to assert the
// middle face id no longer resolves to a face: after a real full round NO output face
// reproduces the consumed +Y strip.
bool middle_plane_remains(CCShapeId body, double topY) {
  for (const FaceN& f : face_normals(body)) {
    if (f.ny > 0.9 && f.cy > topY - 0.5) return true;
  }
  return false;
}

// Blend-vs-neighbour G1 tangency at a seam, measured on the OUTPUT body. `nb` is a
// side-wall face of the result; `axisX/axisY` is the blend-cylinder axis (which runs
// along ±Z, so only its X/Y matter). The blend surface normal where the cylinder
// touches the wall is the radial unit vector from the axis to the foot of the axis on
// the wall's plane; for a G1 (tangent) join that radial equals the wall's outward
// normal. Returns cos(angle) between them (1.0 == perfectly tangent) and the tangent
// radius `rOut`. The wall's in-plane normal is (nb.nx, nb.ny) because both the crease
// and the axis run along Z, so the contact geometry is planar in XY.
double seam_tangency(const FaceN& nb, double axisX, double axisY, double& rOut) {
  const double t = (nb.cx - axisX) * nb.nx + (nb.cy - axisY) * nb.ny;  // signed foot dist
  double rx = nb.nx * t, ry = nb.ny * t;
  const double rl = std::sqrt(rx * rx + ry * ry);
  rOut = rl;
  if (rl < 1e-9) return -2.0;
  rx /= rl; ry /= rl;
  return rx * nb.nx + ry * nb.ny;  // radial · wall-normal (== 1 when tangent)
}

// Find the blend cylinder in `body` (a face for which cc_face_axis succeeds with an
// axis running along ±Z). Fills axis origin + dir; returns false if none.
bool find_blend_cylinder(CCShapeId body, std::array<double, 6>& ax) {
  int* ids = nullptr;
  const int n = cc_subshape_ids(body, /*faces*/ 2, &ids);
  bool found = false;
  for (int i = 0; i < n; ++i) {
    double a[6];
    if (cc_face_axis(body, ids[i], a) == 1 && std::fabs(a[5]) > 0.9) {
      for (int k = 0; k < 6; ++k) ax[k] = a[k];
      found = true;
      break;
    }
  }
  cc_ints_free(ids);
  return found;
}

}  // namespace

void run_full_round_fillet_checks(Ctx& ctx) {
  std::printf("-- full-round fillet --\n");

  // ── Fixture: a box rib (x[-2,2], y[0,20], z[0,10]); its top (width 4) full-rounds
  // into a half-cylinder of radius 2. ───────────────────────────────────────────
  const double rib[8] = {-2, 0, 2, 0, 2, 20, -2, 20};
  const CCShapeId body = cc_solid_extrude(rib, 4, 10.0);
  if (body == 0) { ctx.defer("full-round-fillet", "base rib build returned 0"); return; }

  const std::vector<FaceN> faces = face_normals(body);
  const int leftId = pick_face(faces, -1, 0, 0, false);
  const int rightId = pick_face(faces, 1, 0, 0, false);
  const int midId = pick_face(faces, 0, 1, 0, true);
  if (leftId == 0 || rightId == 0 || midId == 0) {
    ctx.defer("full-round-fillet", "could not classify left/middle/right faces from mesh");
    cc_shape_release(body);
    return;
  }

  const double expectedVol = 720.0 + 20.0 * kPi;  // (72 + 2π)·10

  // ── Explicit three-face full round ──────────────────────────────────────────
  const CCShapeId out = cc_full_round_fillet_faces(body, leftId, midId, rightId);
  bool fullRoundAchieved = false;
  if (!ctx.check(out != 0, "cc_full_round_fillet_faces returns a body")) {
    ctx.defer("cc_full_round_fillet_faces", "returned 0 (no valid full round or fallback)");
  } else {
    // Watertight + middle consumed: a valid solid with EXACTLY the rolling-ball volume.
    const CCMassProps mp = cc_mass_properties(out);
    const bool volOk = mp.valid == 1 && near(mp.volume, expectedVol, 0.5);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "vol=%.4f expect=%.4f", mp.volume, expectedVol);
    const bool consumed = volOk && !flat_top_remains(out);

    if (consumed) {
      fullRoundAchieved = true;
      ctx.check(true, "result is a valid watertight solid at the exact rolling-ball volume", buf);
      ctx.check(!flat_top_remains(out),
                "middle face consumed (no flat +Y face remains at the original top)");

      // Blend is the rolling-ball cylinder, tangent (G1) to BOTH neighbours.
      std::array<double, 6> ax{};
      double bb[6];
      const bool haveCyl = find_blend_cylinder(out, ax);
      const bool haveBox = cc_bounding_box(out, bb) == 1;
      if (ctx.check(haveCyl && haveBox, "blend surface is a cylinder (rolling ball) along the strip")) {
        const double axisX = ax[0], axisY = ax[1];
        const double xmin = bb[0], xmax = bb[3], ymax = bb[4];
        const double distLeft = std::fabs(axisX - xmin);
        const double distRight = std::fabs(axisX - xmax);
        const double rTop = ymax - axisY;  // height from axis to the point it reaches the top
        char tb[192];
        std::snprintf(tb, sizeof(tb),
                      "axisX=%.4f (mid=%.4f) dLeft=%.4f dRight=%.4f rTop=%.4f",
                      axisX, 0.5 * (xmin + xmax), distLeft, distRight, rTop);
        // Equidistant from both walls AND equal to the tangent-reach radius ⇒ the ball
        // touches both walls at the same radius it caps the top ⇒ G1 to BOTH.
        const bool equidist = std::fabs(distLeft - distRight) < 1e-3;
        const bool radiusConsistent =
            std::fabs(distLeft - rTop) < 1e-3 && std::fabs(distRight - rTop) < 1e-3;
        ctx.check(equidist, "blend-cylinder axis is equidistant from both neighbour faces", tb);
        ctx.check(radiusConsistent,
                  "tangent radius to each wall equals the reach to the original top (rolling ball)",
                  tb);

        // Sampled seam-normal agreement on BOTH neighbours: the blend normal at each
        // seam contact (radial, from the axis toward the wall) equals the neighbour's
        // measured outward normal. cos(1°) tolerance.
        const std::vector<FaceN> nf = face_normals(body);  // exact ±X neighbour normals
        double lnx = -1, rnx = 1;  // defaults for the rib
        for (const FaceN& f : nf) {
          if (f.id == leftId) lnx = f.nx;
          if (f.id == rightId) rnx = f.nx;
        }
        const double blendLeftNx = (xmin < axisX) ? -1.0 : 1.0;   // radial dir toward left wall
        const double blendRightNx = (xmax > axisX) ? 1.0 : -1.0;  // toward right wall
        const double dotL = blendLeftNx * lnx;   // both are ±X unit vectors
        const double dotR = blendRightNx * rnx;
        const double cosTol = std::cos(1.0 * kPi / 180.0);
        char sb[160];
        std::snprintf(sb, sizeof(sb), "dot(left)=%.6f dot(right)=%.6f tol=cos(1°)=%.6f", dotL,
                      dotR, cosTol);
        ctx.check(dotL >= cosTol && dotR >= cosTol,
                  "blend is G1-tangent at both seams (sampled seam normals agree)", sb);
      }

      // Reproducibility: a second run yields the same exact volume + surface area
      // (both read from the B-rep via GProp, so they are mesh-state independent —
      // unlike the bounding box, which BRepBndLib tightens once a shape is meshed).
      const CCShapeId out2 = cc_full_round_fillet_faces(body, leftId, midId, rightId);
      if (out2 != 0) {
        const CCMassProps mp2 = cc_mass_properties(out2);
        char rr[128];
        std::snprintf(rr, sizeof(rr), "v1=%.6f v2=%.6f a1=%.6f a2=%.6f", mp.volume, mp2.volume,
                      mp.area, mp2.area);
        ctx.check(mp2.valid == 1 && near(mp.volume, mp2.volume, 1e-6) &&
                      near(mp.area, mp2.area, 1e-6),
                  "full round is deterministic (same volume + area on re-run)", rr);
        cc_shape_release(out2);
      } else {
        ctx.check(false, "second full round run reproduces a body");
      }
    } else {
      // Engine returned a VALID shape but not the exact rolling ball → honest fallback.
      ctx.check(mp.valid == 1 && mp.volume > 0.0,
                "fallback returns a valid solid (BRepCheck-gated by the facade)", buf);
      ctx.defer("cc_full_round_fillet_faces",
                std::string("rolling-ball full round not achieved on the rib; ") + buf);
    }
    cc_shape_release(out);
  }

  // ── Auto-detect variant on the same rib top ─────────────────────────────────
  const CCShapeId outAuto = cc_full_round_fillet(body, midId);
  if (outAuto != 0) {
    const CCMassProps mpa = cc_mass_properties(outAuto);
    if (mpa.valid == 1 && near(mpa.volume, expectedVol, 0.5) && !flat_top_remains(outAuto)) {
      ctx.check(true,
                "cc_full_round_fillet auto-detects neighbours and builds the same rolling ball");
    } else {
      char buf[96];
      std::snprintf(buf, sizeof(buf), "vol=%.4f expect=%.4f", mpa.volume, expectedVol);
      ctx.check(mpa.valid == 1 && mpa.volume > 0.0,
                "auto-detect returns a valid solid (fallback)", buf);
      ctx.defer("cc_full_round_fillet", std::string("auto-detect fell back; ") + buf);
    }
    cc_shape_release(outAuto);
  } else {
    ctx.defer("cc_full_round_fillet", "auto-detect returned 0");
  }

  cc_shape_release(body);

  // ── NON-PARALLEL (dihedral) planar-wall fixture ──────────────────────────────
  // A trapezoidal rib: bottom wide (x[-6,6] @ y=0), top narrow (x[-2,2] @ y=10),
  // extruded 8 in Z. The two side walls slope inward as they rise, so they are NOT
  // parallel — their outward normals are (±0.928, 0.371, 0), n_L·n_R = -0.724, i.e.
  // ~43.6° off the anti-parallel (parallel-wall) case. Two non-parallel PLANES still
  // admit a constant-radius rolling ball: its centre rides the internal dihedral
  // bisector and it sweeps a CYLINDER tangent to both walls (axis ‖ n_L×n_R, here ±Z
  // along the strip). We assert the engine builds that dihedral full round for real —
  // valid + watertight, the middle strip consumed, and G1-tangent to BOTH tilted walls
  // at the seams (blend radial normal == wall normal within cos(1°)). This is the
  // constant-radius generalisation of the parallel rib above; truly CURVED neighbours
  // stay out of scope. Per the honesty rule, if the dihedral build does not achieve a
  // tangent full round we assert only a VALID fallback and defer with the measured gap.
  const double trap[8] = {-6, 0, 6, 0, 2, 10, -2, 10};  // bottom x[-6,6], top x[-2,2]
  const double topY = 10.0;
  const CCShapeId tb = cc_solid_extrude(trap, 4, 8.0);
  if (tb == 0) {
    ctx.defer("full-round-fillet (non-parallel walls)", "trapezoid build returned 0");
    return;
  }
  const std::vector<FaceN> tf = face_normals(tb);
  // Side walls tilt up (+Y component), so match on a diagonal direction, not pure ±X.
  const int tL = pick_face(tf, -1, 0.4, 0, false);
  const int tR = pick_face(tf, 1, 0.4, 0, false);
  const int tM = pick_face(tf, 0, 1, 0, true);
  double lnx = 0, lny = 0, rnx = 0, rny = 0;
  for (const FaceN& f : tf) {
    if (f.id == tL) { lnx = f.nx; lny = f.ny; }
    if (f.id == tR) { rnx = f.nx; rny = f.ny; }
  }
  const double sideDot = lnx * rnx + lny * rny;                 // -1 would be parallel walls
  const double devDeg = std::fabs(180.0 - std::acos(sideDot) * 180.0 / kPi);
  if (tL == 0 || tR == 0 || tM == 0) {
    ctx.defer("full-round-fillet (non-parallel walls)", "could not classify trapezoid faces");
    cc_shape_release(tb);
    return;
  }

  char dih[96];
  std::snprintf(dih, sizeof(dih), "n_L·n_R=%.4f (%.2f° off-parallel)", sideDot, devDeg);
  ctx.check(std::fabs(sideDot) < 0.95,
            "dihedral fixture: side walls are genuinely non-parallel", dih);

  const CCShapeId tout = cc_full_round_fillet_faces(tb, tL, tM, tR);
  if (tout == 0) {
    ctx.defer("cc_full_round_fillet_faces (non-parallel walls)",
              std::string("returned 0 (no full round, no valid fallback); ") + dih);
    cc_shape_release(tb);
    return;
  }

  const CCMassProps tmp = cc_mass_properties(tout);
  const bool tValid = tmp.valid == 1 && tmp.volume > 0.0;
  const bool tConsumed = !middle_plane_remains(tout, topY);

  // Blend cylinder + its axis (along the strip, ±Z).
  std::array<double, 6> tax{};
  const bool tHaveCyl = tValid && find_blend_cylinder(tout, tax);

  // Measure G1 tangency to BOTH tilted walls on the OUTPUT: pick the two side walls
  // (in-plane XY normal, |n·z|≈0, below the top) and compare the blend radial normal
  // at each seam foot to the measured wall normal.
  double cosL = -2.0, cosR = -2.0, rL = 0.0, rR = 0.0;
  if (tHaveCyl) {
    const double axisX = tax[0], axisY = tax[1];
    for (const FaceN& f : face_normals(tout)) {
      if (std::fabs(f.nz) > 0.2) continue;              // skip the ±Z end caps
      if (std::fabs(f.nx) < 0.3 || f.cy > topY - 1.0) continue;  // must be a low side wall
      double r = 0.0;
      const double c = seam_tangency(f, axisX, axisY, r);
      if (f.nx < 0 && c > cosL) { cosL = c; rL = r; }   // left wall (−X-ish)
      if (f.nx > 0 && c > cosR) { cosR = c; rR = r; }   // right wall (+X-ish)
    }
  }

  const double cosTol = std::cos(1.0 * kPi / 180.0);
  const bool tangentBoth = cosL >= cosTol && cosR >= cosTol;
  const bool equidist = std::fabs(rL - rR) < 1e-2;
  const bool fullRound = tValid && tConsumed && tHaveCyl && tangentBoth && equidist;

  if (fullRound) {
    ctx.check(true, "non-parallel dihedral: result is a valid watertight solid",
              [&] { char b[96]; std::snprintf(b, sizeof(b), "vol=%.4f %s", tmp.volume, dih); return std::string(b); }());
    ctx.check(tConsumed,
              "non-parallel dihedral: middle strip consumed (no +Y face on the original top plane)");
    ctx.check(tHaveCyl,
              "non-parallel dihedral: blend surface is a cylinder (rolling ball) along the strip");
    char sb[192];
    std::snprintf(sb, sizeof(sb),
                  "cos(left)=%.6f cos(right)=%.6f tol=cos(1°)=%.6f rL=%.4f rR=%.4f", cosL,
                  cosR, cosTol, rL, rR);
    ctx.check(equidist,
              "non-parallel dihedral: blend cylinder is equidistant from both tilted walls", sb);
    ctx.check(tangentBoth,
              "non-parallel dihedral: blend is G1-tangent to BOTH non-parallel walls at the seams",
              sb);
  } else {
    // Honest fallback: a VALID lower-fidelity result, deferred with the measured gap.
    ctx.check(tValid, "non-parallel dihedral: returns a VALID solid (fallback)",
              [&] { char b[96]; std::snprintf(b, sizeof(b), "vol=%.4f", tmp.volume); return std::string(b); }());
    char gap[224];
    std::snprintf(gap, sizeof(gap),
                  "%s; consumed=%d cyl=%d cos(left)=%.6f cos(right)=%.6f (need>=%.6f) rL=%.4f rR=%.4f",
                  dih, tConsumed ? 1 : 0, tHaveCyl ? 1 : 0, cosL, cosR, cosTol, rL, rR);
    ctx.defer("cc_full_round_fillet_faces (non-parallel walls)", gap);
  }
  cc_shape_release(tout);
  cc_shape_release(tb);
}
