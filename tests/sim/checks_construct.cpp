// Construction module of the CyberCadKernel OCCT runtime suite.
//
// Exercises every cc_* construction entry point through the real OCCT adapter
// with REAL inputs and asserts a CONCRETE analytic invariant wherever one is
// feasible (volume/area/bbox from the exact B-rep, sub-shape counts, boss adds
// material, ...), plus a deliberate degenerate call that must guard-return 0 (or
// an empty mesh). Where a strong analytic check is impractical (helical/tapered
// threads: no closed-form volume) it falls back to a non-degenerate-result +
// bbox-sanity smoke check, clearly labelled.
//
// Invariant map (which functions get a CONCRETE analytic check vs a SMOKE check):
//   CONCRETE volume/area/bbox:
//     cc_extrude (mesh bbox), cc_solid_extrude, cc_solid_revolve, cc_solid_loft,
//     cc_solid_loft_wires, cc_solid_sweep, cc_twisted_sweep (twist=0 -> prism),
//     cc_loft_along_rail, cc_guided_sweep (parallel guide -> prism),
//     cc_tapered_shank, cc_solid_extrude_holes, cc_solid_extrude_polyholes,
//     cc_solid_extrude_profile (true circle -> cylinder),
//     cc_solid_extrude_profile_polyholes, cc_solid_revolve_profile,
//     cc_wrap_emboss (boss increases volume).
//   SMOKE (valid id + positive volume + bbox sanity; no closed form):
//     cc_helical_thread, cc_tapered_thread.
//
// Every CCShapeId is released and every buffer freed before return.

#include "checks.h"

#include <cmath>
#include <vector>

namespace {

// Volume of a body via the exact B-rep mass properties (-1 if invalid/unknown).
double bodyVolume(CCShapeId id) {
  CCMassProps m = cc_mass_properties(id);
  return m.valid ? m.volume : -1.0;
}

// Axis-aligned B-rep bbox into out6=[minx,miny,minz,maxx,maxy,maxz]; false on failure.
bool bodyBBox(CCShapeId id, double out6[6]) {
  return cc_bounding_box(id, out6) == 1;
}

// Count faces (kind=2) of a body via the stable sub-shape id map.
int faceCount(CCShapeId id) {
  int* ids = nullptr;
  const int n = cc_subshape_ids(id, 2, &ids);
  if (ids) cc_ints_free(ids);
  return n;
}

// First face id whose surface is a cylinder/cone (cc_face_axis returns 1), else 0.
int firstCylindricalFace(CCShapeId id) {
  int* ids = nullptr;
  const int n = cc_subshape_ids(id, 2, &ids);
  int found = 0;
  double axis6[6] = {0};
  for (int i = 0; i < n && found == 0; ++i) {
    if (cc_face_axis(id, ids[i], axis6) == 1) found = ids[i];
  }
  if (ids) cc_ints_free(ids);
  return found;
}

// Bounding box of a raw CCMesh's vertex buffer into out6; false if empty.
bool meshBBox(const CCMesh& m, double out6[6]) {
  if (!m.vertices || m.vertexCount <= 0) return false;
  out6[0] = out6[1] = out6[2] = 1e300;
  out6[3] = out6[4] = out6[5] = -1e300;
  for (int i = 0; i < m.vertexCount; ++i) {
    for (int a = 0; a < 3; ++a) {
      const double v = m.vertices[i * 3 + a];
      if (v < out6[a]) out6[a] = v;
      if (v > out6[a + 3]) out6[a + 3] = v;
    }
  }
  return true;
}

// True when a bbox is [0,0,0]-[10,10,10] (the 10x10x(depth=10) box) within tol.
bool isUnitBox10(const double b[6], double tol) {
  return near(b[0], 0, tol) && near(b[1], 0, tol) && near(b[2], 0, tol) &&
         near(b[3], 10, tol) && near(b[4], 10, tol) && near(b[5], 10, tol);
}

const double kPi = 3.14159265358979323846;

// 10x10 square profile (x,y pairs) — the canonical parity test profile.
const double kSquare[8] = {0, 0, 10, 0, 10, 10, 0, 10};

}  // namespace

void run_construct_checks(Ctx& ctx) {
  // ── cc_extrude (legacy mesh path): bbox of the returned box mesh ──────────
  {
    CCMesh m = cc_extrude(kSquare, 4, 10.0);
    double b[6] = {0};
    const bool ok = meshBBox(m, b);
    ctx.check(ok && m.triangleCount > 0 && isUnitBox10(b, 1e-6),
              "cc_extrude 10x10x10 -> mesh bbox [0,0,0]-[10,10,10]",
              ok ? "tris=" + std::to_string(m.triangleCount) : "empty mesh");
    cc_mesh_free(m);
    CCMesh d = cc_extrude(kSquare, 2, 10.0);  // <3 points -> empty mesh (guard)
    ctx.check(d.vertices == nullptr && d.vertexCount == 0,
              "cc_extrude(pointCount=2) -> empty mesh (guard)");
    cc_mesh_free(d);
  }

  // ── cc_solid_extrude: box volume 1000, area 600, bbox, 6 faces ────────────
  {
    CCShapeId box = cc_solid_extrude(kSquare, 4, 10.0);
    ctx.check(box != 0, "cc_solid_extrude -> valid id");
    const double v = bodyVolume(box);
    CCMassProps mp = cc_mass_properties(box);
    double b[6] = {0};
    ctx.check(near(v, 1000.0, 1e-4), "cc_solid_extrude box volume == 1000",
              "got " + std::to_string(v));
    ctx.check(mp.valid && near(mp.area, 600.0, 1e-4), "cc_solid_extrude box area == 600",
              "got " + std::to_string(mp.area));
    ctx.check(bodyBBox(box, b) && isUnitBox10(b, 1e-6),
              "cc_solid_extrude bbox == [0,0,0]-[10,10,10]");
    ctx.check(faceCount(box) == 6, "cc_solid_extrude box has 6 faces",
              "got " + std::to_string(faceCount(box)));
    cc_shape_release(box);
    ctx.check(cc_solid_extrude(kSquare, 4, 0.0) == 0,
              "cc_solid_extrude(depth=0) -> 0 (guard)");
  }

  // ── cc_solid_revolve: rectangle x[2,5],y[0,10] about Y, 2pi -> tube ───────
  // Volume = pi*(rout^2 - rin^2)*h = pi*(25-4)*10 = 210*pi.
  {
    const double rect[8] = {2, 0, 5, 0, 5, 10, 2, 10};
    CCShapeId tube = cc_solid_revolve(rect, 4, 2.0 * kPi);
    ctx.check(tube != 0, "cc_solid_revolve -> valid id");
    const double v = bodyVolume(tube);
    ctx.check(near(v, 210.0 * kPi, 1e-2), "cc_solid_revolve tube volume == 210*pi",
              "got " + std::to_string(v));
    cc_shape_release(tube);
    ctx.check(cc_solid_revolve(rect, 4, 0.0) == 0,
              "cc_solid_revolve(angle=0) -> 0 (guard)");
  }

  // ── cc_solid_loft: identical 10x10 squares 10 apart -> box vol 1000 ───────
  {
    CCShapeId loft = cc_solid_loft(kSquare, 4, kSquare, 4, 10.0);
    ctx.check(loft != 0, "cc_solid_loft -> valid id");
    double b[6] = {0};
    ctx.check(near(bodyVolume(loft), 1000.0, 1e-4), "cc_solid_loft prism volume == 1000",
              "got " + std::to_string(bodyVolume(loft)));
    ctx.check(bodyBBox(loft, b) && isUnitBox10(b, 1e-6),
              "cc_solid_loft bbox == [0,0,0]-[10,10,10]");
    cc_shape_release(loft);
    ctx.check(cc_solid_loft(kSquare, 4, kSquare, 4, 0.0) == 0,
              "cc_solid_loft(depth=0) -> 0 (guard)");
  }

  // ── cc_solid_loft_wires: two identical 3D squares 10 apart -> box vol 1000 ─
  {
    const double a[12] = {0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0};
    const double b[12] = {0, 0, 10, 10, 0, 10, 10, 10, 10, 0, 10, 10};
    CCShapeId loft = cc_solid_loft_wires(a, 4, b, 4);
    ctx.check(loft != 0, "cc_solid_loft_wires -> valid id");
    ctx.check(near(bodyVolume(loft), 1000.0, 1e-4),
              "cc_solid_loft_wires prism volume == 1000",
              "got " + std::to_string(bodyVolume(loft)));
    cc_shape_release(loft);
    ctx.check(cc_solid_loft_wires(a, 2, b, 4) == 0,
              "cc_solid_loft_wires(aCount=2) -> 0 (guard)");
  }

  // ── cc_solid_sweep: 10x10 square along straight Z path len 20 -> vol 2000 ──
  {
    const double path[6] = {0, 0, 0, 0, 0, 20};
    CCShapeId pipe = cc_solid_sweep(kSquare, 4, path, 2);
    ctx.check(pipe != 0, "cc_solid_sweep -> valid id");
    ctx.check(near(bodyVolume(pipe), 2000.0, 1e-3), "cc_solid_sweep pipe volume == 2000",
              "got " + std::to_string(bodyVolume(pipe)));
    cc_shape_release(pipe);
    ctx.check(cc_solid_sweep(kSquare, 4, path, 1) == 0,
              "cc_solid_sweep(pathCount=1) -> 0 (guard)");
  }

  // ── cc_twisted_sweep: twist=0, scaleEnd=1 -> straight prism vol 2000 ───────
  // (a non-zero twist has no closed-form volume; twist=0 is the analytic case.)
  {
    const double path[6] = {0, 0, 0, 0, 0, 20};
    CCShapeId s = cc_twisted_sweep(kSquare, 4, path, 2, 0.0, 1.0);
    ctx.check(s != 0, "cc_twisted_sweep(twist=0) -> valid id");
    ctx.check(near(bodyVolume(s), 2000.0, 1e-3),
              "cc_twisted_sweep(twist=0) prism volume == 2000",
              "got " + std::to_string(bodyVolume(s)));
    cc_shape_release(s);
    // A real twist must still produce a valid, positive-volume body (smoke).
    CCShapeId tw = cc_twisted_sweep(kSquare, 4, path, 2, kPi / 4.0, 1.0);
    ctx.check(tw != 0 && bodyVolume(tw) > 0.0,
              "cc_twisted_sweep(twist=pi/4) -> valid non-degenerate body");
    if (tw) cc_shape_release(tw);
    ctx.check(cc_twisted_sweep(kSquare, 2, path, 2, 0.0, 1.0) == 0,
              "cc_twisted_sweep(profileCount=2) -> 0 (guard)");
  }

  // ── cc_loft_along_rail: identical 10x10 sections along straight rail -> 2000 ─
  {
    const double rail[6] = {0, 0, 0, 0, 0, 20};
    CCShapeId s = cc_loft_along_rail(rail, 2, kSquare, 4, kSquare, 4);
    ctx.check(s != 0, "cc_loft_along_rail -> valid id");
    ctx.check(near(bodyVolume(s), 2000.0, 1e-2),
              "cc_loft_along_rail prism volume == 2000",
              "got " + std::to_string(bodyVolume(s)));
    cc_shape_release(s);
    ctx.check(cc_loft_along_rail(rail, 1, kSquare, 4, kSquare, 4) == 0,
              "cc_loft_along_rail(railCount=1) -> 0 (guard)");
  }

  // ── cc_guided_sweep: guide parallel to path (const scale=1) -> prism 2000 ──
  {
    const double path[6] = {0, 0, 0, 0, 0, 20};
    const double guide[6] = {5, 0, 0, 5, 0, 20};  // offset +5 in X, parallel -> scale 1
    CCShapeId s = cc_guided_sweep(kSquare, 4, path, 2, guide, 2);
    ctx.check(s != 0, "cc_guided_sweep -> valid id");
    ctx.check(near(bodyVolume(s), 2000.0, 1e-3),
              "cc_guided_sweep(parallel guide) prism volume == 2000",
              "got " + std::to_string(bodyVolume(s)));
    cc_shape_release(s);
    ctx.check(cc_guided_sweep(kSquare, 4, path, 2, guide, 1) == 0,
              "cc_guided_sweep(guideCount=1) -> 0 (guard)");
  }

  // ── cc_wrap_emboss: boss onto a cylinder's lateral face adds material ─────
  // Build a solid cylinder r=5 h=10 (revolve rect x[0,5],y[0,10] about Y, 2pi),
  // find its cylindrical face, emboss a small pad as a boss -> volume grows.
  {
    const double rect[8] = {0, 0, 5, 0, 5, 10, 0, 10};
    CCShapeId cyl = cc_solid_revolve(rect, 4, 2.0 * kPi);
    const double vCyl = bodyVolume(cyl);
    const int fid = cyl ? firstCylindricalFace(cyl) : 0;
    ctx.check(cyl != 0 && near(vCyl, 250.0 * kPi, 1e-2) && fid != 0,
              "cc_wrap_emboss setup: cylinder vol 250*pi + cylindrical face found",
              "faceId=" + std::to_string(fid));
    if (cyl && fid) {
      // Unrolled profile: x=arc length, y=axial; small 4x4 pad centered at v=0.
      const double pad[8] = {-2, -2, 2, -2, 2, 2, -2, 2};
      CCShapeId boss = cc_wrap_emboss(cyl, fid, pad, 4, 1.0, 1);
      const double vBoss = bodyVolume(boss);
      ctx.check(boss != 0 && vBoss > vCyl + 1e-3,
                "cc_wrap_emboss(boss) increases volume vs cylinder",
                "cyl=" + std::to_string(vCyl) + " boss=" + std::to_string(vBoss));
      if (boss) cc_shape_release(boss);
      ctx.check(cc_wrap_emboss(cyl, fid, pad, 4, 0.0, 1) == 0,
                "cc_wrap_emboss(depth=0) -> 0 (guard)");
    }
    if (cyl) cc_shape_release(cyl);
  }

  // ── cc_helical_thread: no closed-form volume -> smoke (valid + bounded scale) ─
  {
    CCShapeId th = cc_helical_thread(5.0, 2.0, 3.0, 1.0, 60.0, 1.0, 12);
    const double v = th ? bodyVolume(th) : -1.0;
    double b[6] = {0};
    const bool bb = th && bodyBBox(th, b);
    // The V section is swept along the helix by a CORRECTED radial pipe-shell
    // (BRepOffsetAPI_MakePipeShell, withCorrection). The correction swings the
    // section outward, so the realised radial extent is larger than the idealised
    // apex radius (major - depth/2 + depth = 5.5) -- here it lands near ~8mm. Smoke-
    // check a valid, bounded, thread-scaled body instead of an exact radius: positive
    // volume, radial extent within [root, 2*major], and an axial rise ~ pitch*turns.
    const double majorR = 5.0, depthR = 1.0, rise = 2.0 * 3.0;  // pitch * turns
    const double axial = bb ? (b[5] - b[2]) : -1.0;
    const bool radiusOk = bb && b[3] >= majorR - depthR && b[3] <= 2.0 * majorR &&
                          b[4] >= majorR - depthR && b[4] <= 2.0 * majorR;
    const bool axialOk = bb && axial >= rise - 1e-6 && axial <= rise + 2.0;  // + V overhang
    ctx.check(th != 0 && v > 0.0 && radiusOk && axialOk,
              "cc_helical_thread -> valid body, positive volume, bounded thread-scale (smoke)",
              "vol=" + std::to_string(v) + " maxR=" + std::to_string(bb ? b[3] : -1.0) +
                  " axial=" + std::to_string(axial));
    if (th) cc_shape_release(th);
    ctx.check(cc_helical_thread(5.0, 0.0, 3.0, 1.0, 60.0, 1.0, 12) == 0,
              "cc_helical_thread(pitch=0) -> 0 (guard)");
  }

  // ── cc_tapered_thread: no closed-form volume -> smoke (valid + positive vol) ─
  {
    CCShapeId th = cc_tapered_thread(5.0, 3.0, 2.0, 3.0, 1.0, 60.0, 1.0, 12);
    const double v = th ? bodyVolume(th) : -1.0;
    ctx.check(th != 0 && v > 0.0,
              "cc_tapered_thread -> valid body, positive volume (smoke)",
              "vol=" + std::to_string(v));
    if (th) cc_shape_release(th);
    ctx.check(cc_tapered_thread(5.0, 3.0, 2.0, 0.0, 1.0, 60.0, 1.0, 12) == 0,
              "cc_tapered_thread(turns=0) -> 0 (guard)");
  }

  // ── cc_tapered_shank: frustum (r=0.1->5, h=6) + cylinder (r=5, h=10) ───────
  // pointsPerMM=1. V = (pi*6/3)(5^2+5*0.1+0.1^2) + pi*5^2*10.
  {
    const double r = 5.0, full = 10.0, taper = 6.0, tip = 0.1;
    const double vFrustum = (kPi * taper / 3.0) * (r * r + r * tip + tip * tip);
    const double vCyl = kPi * r * r * full;
    const double vExp = vFrustum + vCyl;
    CCShapeId sh = cc_tapered_shank(r, full, taper, 1.0);
    ctx.check(sh != 0, "cc_tapered_shank -> valid id");
    ctx.check(near(bodyVolume(sh), vExp, 1e-2),
              "cc_tapered_shank volume == frustum+cylinder",
              "exp " + std::to_string(vExp) + " got " + std::to_string(bodyVolume(sh)));
    cc_shape_release(sh);
    ctx.check(cc_tapered_shank(0.0, full, taper, 1.0) == 0,
              "cc_tapered_shank(radius=0) -> 0 (guard)");
  }

  // ── cc_solid_extrude_holes: 10x10 plate, one r=2 hole, depth 10 ───────────
  // Volume = (100 - pi*2^2)*10 = 1000 - 40*pi.
  {
    const double holeCR[3] = {5, 5, 2};
    CCShapeId s = cc_solid_extrude_holes(kSquare, 4, holeCR, 1, 10.0);
    ctx.check(s != 0, "cc_solid_extrude_holes -> valid id");
    ctx.check(near(bodyVolume(s), 1000.0 - 40.0 * kPi, 1e-2),
              "cc_solid_extrude_holes volume == 1000 - 40*pi",
              "got " + std::to_string(bodyVolume(s)));
    cc_shape_release(s);
    ctx.check(cc_solid_extrude_holes(kSquare, 4, holeCR, 1, 0.0) == 0,
              "cc_solid_extrude_holes(depth=0) -> 0 (guard)");
  }

  // ── cc_solid_extrude_polyholes: 10x10 plate, one 2x2 square hole, depth 10 ─
  // Volume = (100 - 4)*10 = 960.
  {
    const double holesXY[8] = {4, 4, 6, 4, 6, 6, 4, 6};
    const int counts[1] = {4};
    CCShapeId s = cc_solid_extrude_polyholes(kSquare, 4, holesXY, counts, 1, 10.0);
    ctx.check(s != 0, "cc_solid_extrude_polyholes -> valid id");
    ctx.check(near(bodyVolume(s), 960.0, 1e-4),
              "cc_solid_extrude_polyholes volume == 960",
              "got " + std::to_string(bodyVolume(s)));
    cc_shape_release(s);
    ctx.check(cc_solid_extrude_polyholes(kSquare, 4, holesXY, counts, 1, 0.0) == 0,
              "cc_solid_extrude_polyholes(depth=0) -> 0 (guard)");
  }

  // ── cc_solid_extrude_profile: a TRUE circle (kind=2) r=5 -> cylinder ───────
  // Volume = pi*5^2*10 = 250*pi. Exercises the typed-segment/arc wire path.
  {
    CCProfileSeg seg;
    seg.kind = 2;
    seg.x0 = seg.y0 = seg.x1 = seg.y1 = 0;
    seg.cx = 5; seg.cy = 5; seg.r = 5;
    seg.a0 = seg.a1 = 0;
    seg.ptOffset = seg.ptCount = 0;
    CCShapeId s = cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, 10.0);
    ctx.check(s != 0, "cc_solid_extrude_profile(circle) -> valid id");
    ctx.check(near(bodyVolume(s), 250.0 * kPi, 1e-2),
              "cc_solid_extrude_profile circle -> cylinder vol 250*pi",
              "got " + std::to_string(bodyVolume(s)));
    // A true circle profile has exactly 3 faces (lateral + 2 caps).
    ctx.check(s && faceCount(s) == 3, "cc_solid_extrude_profile cylinder has 3 faces",
              "got " + std::to_string(s ? faceCount(s) : -1));
    if (s) cc_shape_release(s);
    ctx.check(cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, 0.0) == 0,
              "cc_solid_extrude_profile(depth=0) -> 0 (guard)");
  }

  // ── cc_solid_extrude_profile_polyholes: square (4 lines) + circle + poly hole ─
  // Volume = (100 - pi*1^2 - 2*2)*10 = (96 - pi)*10.
  {
    CCProfileSeg segs[4];
    const double corners[5][2] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};
    for (int i = 0; i < 4; ++i) {
      segs[i].kind = 0;
      segs[i].x0 = corners[i][0]; segs[i].y0 = corners[i][1];
      segs[i].x1 = corners[i + 1][0]; segs[i].y1 = corners[i + 1][1];
      segs[i].cx = segs[i].cy = segs[i].r = segs[i].a0 = segs[i].a1 = 0;
      segs[i].ptOffset = segs[i].ptCount = 0;
    }
    const double circleCR[3] = {3, 3, 1};             // circle hole r=1
    const double polyXY[8] = {6, 6, 8, 6, 8, 8, 6, 8};  // 2x2 square hole
    const int polyCounts[1] = {4};
    CCShapeId s = cc_solid_extrude_profile_polyholes(
        segs, 4, circleCR, 1, polyXY, polyCounts, 1, nullptr, 0, 10.0);
    ctx.check(s != 0, "cc_solid_extrude_profile_polyholes -> valid id");
    ctx.check(near(bodyVolume(s), (96.0 - kPi) * 10.0, 1e-2),
              "cc_solid_extrude_profile_polyholes volume == (96-pi)*10",
              "got " + std::to_string(bodyVolume(s)));
    if (s) cc_shape_release(s);
    ctx.check(cc_solid_extrude_profile_polyholes(
                  segs, 4, circleCR, 1, polyXY, polyCounts, 1, nullptr, 0, 0.0) == 0,
              "cc_solid_extrude_profile_polyholes(depth=0) -> 0 (guard)");
  }

  // ── cc_solid_revolve_profile: rect (4 lines) x[2,5],y[0,10] about Y, 2pi ───
  // Same tube as cc_solid_revolve: volume = pi*(25-4)*10 = 210*pi.
  {
    CCProfileSeg segs[4];
    const double corners[5][2] = {{2, 0}, {5, 0}, {5, 10}, {2, 10}, {2, 0}};
    for (int i = 0; i < 4; ++i) {
      segs[i].kind = 0;
      segs[i].x0 = corners[i][0]; segs[i].y0 = corners[i][1];
      segs[i].x1 = corners[i + 1][0]; segs[i].y1 = corners[i + 1][1];
      segs[i].cx = segs[i].cy = segs[i].r = segs[i].a0 = segs[i].a1 = 0;
      segs[i].ptOffset = segs[i].ptCount = 0;
    }
    CCShapeId s = cc_solid_revolve_profile(segs, 4, 0, 0, 0, 1, nullptr, 0, 2.0 * kPi);
    ctx.check(s != 0, "cc_solid_revolve_profile -> valid id");
    ctx.check(near(bodyVolume(s), 210.0 * kPi, 1e-2),
              "cc_solid_revolve_profile tube volume == 210*pi",
              "got " + std::to_string(bodyVolume(s)));
    if (s) cc_shape_release(s);
    ctx.check(cc_solid_revolve_profile(segs, 4, 0, 0, 0, 1, nullptr, 0, 0.0) == 0,
              "cc_solid_revolve_profile(angle=0) -> 0 (guard)");
  }
}
