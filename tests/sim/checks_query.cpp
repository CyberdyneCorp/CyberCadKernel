// Query-module checks for the CyberCadKernel OCCT runtime suite (iOS simulator).
//
// Exercises the read-only "query" surface of the cc_* facade against real OCCT
// geometry and asserts CONCRETE analytic invariants wherever feasible:
//
//   cc_mass_properties   volume / area / centre-of-mass of a 20x10x30 box
//   cc_principal_moments principal volume-inertia of that box vs the closed-form
//                        cuboid formula  I = V*(d1^2 + d2^2)/12
//   cc_bounding_box      exact B-rep extents of the box
//   cc_face_axis         axis of a true cylindrical face (revolved-free: a full
//                        circle profile extruded +Z) is the +Z line through (0,0)
//   cc_subshape_ids      a box has 8 vertices / 12 edges / 6 faces
//   cc_tangent_chain     the 8 tangent-continuous rim edges of a rounded-rectangle
//                        prism grow into one chain from any single rim seed
//   cc_outer_rim_chain   two spread top-rim seeds grow into the whole 8-edge cap
//                        loop while the flat side walls are rejected
//   (+ cc_ints_free)     every id buffer returned here is freed
//
// Each degenerate/guard path (unknown body id, null seed, non-cylindrical face)
// is asserted to return the documented failure sentinel (valid==0 / return 0).
//
// Runs only meaningfully with CYBERCAD_HAS_OCCT=ON (the sim slice); full_suite
// aborts before this module when no B-rep engine is linked, and every cc_* here
// safely returns the failure sentinel on the host stub, so this file also
// compiles + links cleanly in the OCCT-OFF host build.

#include "checks.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <set>
#include <string>
#include <vector>

namespace {

// 20 (x) x 10 (y) rectangle, CCW — extruded +Z gives a 20x10xD box whose three
// side lengths are all distinct, so the principal moments are distinguishable.
const double kRect20x10[8] = {0, 0, 20, 0, 20, 10, 0, 10};

std::string num(double v) { return std::to_string(v); }

// ── cc_mass_properties + cc_principal_moments ───────────────────────────────
// A solid a×b×c box (unit density) has V=abc, surface area 2(ab+bc+ca), centroid
// at the box centre, and principal moments I_i = V*(sum of the OTHER two squared
// dims)/12. We build a 20×10×30 box and check all of them.
void checkMassAndMoments(Ctx& ctx) {
  const double a = 20.0, b = 10.0, c = 30.0;   // x, y, z extents
  CCShapeId box = cc_solid_extrude(kRect20x10, 4, c);
  if (!ctx.check(box != 0, "query: cc_solid_extrude -> 20x10x30 box id")) { return; }

  const double V = a * b * c;                              // 6000
  const double area = 2.0 * (a * b + b * c + c * a);       // 2200
  CCMassProps mp = cc_mass_properties(box);
  ctx.check(mp.valid == 1, "cc_mass_properties valid==1");
  ctx.check(mp.valid && near(mp.volume, V, 1e-4),
            "cc_mass_properties volume == 6000", "got " + num(mp.volume));
  ctx.check(mp.valid && near(mp.area, area, 1e-4),
            "cc_mass_properties area == 2200", "got " + num(mp.area));
  ctx.check(mp.valid && near(mp.cx, a / 2, 1e-6) && near(mp.cy, b / 2, 1e-6)
                && near(mp.cz, c / 2, 1e-6),
            "cc_mass_properties centroid == (10,5,15)",
            "got (" + num(mp.cx) + "," + num(mp.cy) + "," + num(mp.cz) + ")");

  // Principal moments: closed-form cuboid values, then sorted so the check is
  // independent of the order OCCT reports the principal axes in.
  double got[3] = {0, 0, 0};
  int ok = cc_principal_moments(box, got);
  ctx.check(ok == 1, "cc_principal_moments -> 1");
  double exp[3] = {V * (b * b + c * c) / 12.0,   // about x: 500000
                   V * (a * a + c * c) / 12.0,    // about y: 650000
                   V * (a * a + b * b) / 12.0};   // about z: 250000
  std::sort(got, got + 3);
  std::sort(exp, exp + 3);
  const double tol = exp[2] * 1e-4;              // ~65, values differ by >=150000
  ctx.check(ok && near(got[0], exp[0], tol) && near(got[1], exp[1], tol)
                && near(got[2], exp[2], tol),
            "cc_principal_moments == cuboid formula {250000,500000,650000}",
            "got {" + num(got[0]) + "," + num(got[1]) + "," + num(got[2]) + "}");

  cc_shape_release(box);
}

// ── cc_bounding_box ─────────────────────────────────────────────────────────
void checkBoundingBox(Ctx& ctx) {
  CCShapeId box = cc_solid_extrude(kRect20x10, 4, 30.0);
  if (!ctx.check(box != 0, "query: box for bbox")) { return; }
  double bb[6] = {0, 0, 0, 0, 0, 0};
  int ok = cc_bounding_box(box, bb);
  ctx.check(ok == 1 && near(bb[0], 0, 1e-6) && near(bb[1], 0, 1e-6) && near(bb[2], 0, 1e-6)
                && near(bb[3], 20, 1e-6) && near(bb[4], 10, 1e-6) && near(bb[5], 30, 1e-6),
            "cc_bounding_box == [0,0,0, 20,10,30]",
            "got [" + num(bb[0]) + "," + num(bb[1]) + "," + num(bb[2]) + ", "
                + num(bb[3]) + "," + num(bb[4]) + "," + num(bb[5]) + "]");
  cc_shape_release(box);
}

// ── cc_face_axis ────────────────────────────────────────────────────────────
// Extrude a TRUE full-circle profile (CCProfileSeg kind 2) +Z to get a cylinder
// with a genuine cylindrical lateral face. cc_face_axis on that face must return
// the +Z axis through (0,0); on either planar cap it must return 0 (guard).
void checkFaceAxis(Ctx& ctx) {
  const double r = 5.0, depth = 10.0;
  CCProfileSeg circle{};
  circle.kind = 2; circle.cx = 0.0; circle.cy = 0.0; circle.r = r;
  CCShapeId cyl = cc_solid_extrude_profile(&circle, 1, nullptr, 0, nullptr, 0, depth);
  if (!ctx.check(cyl != 0, "query: cc_solid_extrude_profile -> cylinder id")) { return; }

  CCMassProps mp = cc_mass_properties(cyl);
  const double Vexp = M_PI * r * r * depth;     // pi*25*10
  ctx.check(mp.valid && near(mp.volume, Vexp, 1e-3),
            "cylinder volume == pi*r^2*h", "got " + num(mp.volume));

  int* faces = nullptr;
  int nface = cc_subshape_ids(cyl, 2, &faces);
  int cylFaces = 0, planarZeros = 0;
  double out6[6] = {0};
  for (int i = 0; i < nface; ++i) {
    double a[6] = {0, 0, 0, 0, 0, 0};
    if (cc_face_axis(cyl, faces[i], a) == 1) {
      ++cylFaces;
      for (int k = 0; k < 6; ++k) { out6[k] = a[k]; }   // remember the cylindrical face's axis
    } else {
      ++planarZeros;                                     // a planar cap → correctly rejected
    }
  }
  if (faces) { cc_ints_free(faces); }

  ctx.check(cylFaces == 1, "cc_face_axis identifies exactly one cylindrical face",
            "got " + std::to_string(cylFaces));
  ctx.check(planarZeros >= 1, "cc_face_axis returns 0 for planar cap faces (guard)",
            "got " + std::to_string(planarZeros));
  const bool zParallel = near(std::fabs(out6[5]), 1.0, 1e-6)
                && near(out6[3], 0.0, 1e-6) && near(out6[4], 0.0, 1e-6);
  ctx.check(cylFaces == 1 && zParallel,
            "cc_face_axis direction is +/-Z", "dir=(" + num(out6[3]) + ","
                + num(out6[4]) + "," + num(out6[5]) + ")");
  ctx.check(cylFaces == 1 && near(out6[0], 0.0, 1e-6) && near(out6[1], 0.0, 1e-6),
            "cc_face_axis point lies on x=y=0",
            "pt=(" + num(out6[0]) + "," + num(out6[1]) + ")");

  cc_shape_release(cyl);
}

// ── cc_subshape_ids ─────────────────────────────────────────────────────────
// A rectangular prism (box) has exactly 8 vertices, 12 edges, 6 faces, and the
// ids are the dense 1..N range.
void checkSubshapeIds(Ctx& ctx) {
  CCShapeId box = cc_solid_extrude(kRect20x10, 4, 30.0);
  if (!ctx.check(box != 0, "query: box for subshape ids")) { return; }

  struct { int kind; int expect; const char* name; } cases[] = {
      {0, 8, "cc_subshape_ids vertices == 8"},
      {1, 12, "cc_subshape_ids edges == 12"},
      {2, 6, "cc_subshape_ids faces == 6"}};
  for (auto& tc : cases) {
    int* ids = nullptr;
    int n = cc_subshape_ids(box, tc.kind, &ids);
    bool dense = (n == tc.expect);
    for (int i = 0; i < n && dense; ++i) { if (ids[i] != i + 1) { dense = false; } }
    ctx.check(dense, tc.name, "got " + std::to_string(n));
    if (ids) { cc_ints_free(ids); }
  }
  cc_shape_release(box);
}

// Build a rounded-rectangle (W x H, corner radius rr) as tangent line+arc segs.
// Every line<->arc junction is C1, so the extruded prism's top rim is one tangent
// chain of exactly 8 edges — the strongest clean input for the chain queries.
std::vector<CCProfileSeg> roundedRect(double W, double H, double rr) {
  auto line = [](double x0, double y0, double x1, double y1) {
    CCProfileSeg s{}; s.kind = 0; s.x0 = x0; s.y0 = y0; s.x1 = x1; s.y1 = y1; return s;
  };
  auto arc = [](double cx, double cy, double r, double a0, double a1) {
    CCProfileSeg s{}; s.kind = 1; s.cx = cx; s.cy = cy; s.r = r; s.a0 = a0; s.a1 = a1; return s;
  };
  const double P = M_PI;
  return {
      line(rr, 0, W - rr, 0),                       // bottom
      arc(W - rr, rr, rr, -P / 2, 0),               // BR corner (17,0)->(20,3)
      line(W, rr, W, H - rr),                        // right
      arc(W - rr, H - rr, rr, 0, P / 2),            // TR corner (20,7)->(17,10)
      line(W - rr, H, rr, H),                        // top
      arc(rr, H - rr, rr, P / 2, P),                // TL corner (3,10)->(0,7)
      line(0, H - rr, 0, rr),                        // left
      arc(rr, rr, rr, P, 3 * P / 2)};               // BL corner (0,3)->(3,0)
}

// ── cc_tangent_chain + cc_outer_rim_chain ───────────────────────────────────
// Identify the top-rim edge ids robustly via cc_edge_polylines (an edge whose
// every sampled point sits at z == depth). Seeding either chain query with one
// of those edges must return the whole 8-edge rim.
void checkChains(Ctx& ctx) {
  const double depth = 5.0;
  std::vector<CCProfileSeg> segs = roundedRect(20.0, 10.0, 3.0);
  CCShapeId body = cc_solid_extrude_profile(segs.data(), static_cast<int>(segs.size()),
                                            nullptr, 0, nullptr, 0, depth);
  if (!ctx.check(body != 0, "query: rounded-rect prism id (chains)")) { return; }

  // Find the top-rim edges (all polyline points at z == depth).
  CCEdgePolyline* polys = nullptr;
  int nedge = cc_edge_polylines(body, &polys);
  std::vector<int> topRim;
  std::vector<std::array<double, 2>> topMid;  // XY midpoint of each rim edge
  for (int i = 0; i < nedge; ++i) {
    const CCEdgePolyline& e = polys[i];
    if (e.pointCount < 2 || e.points == nullptr) { continue; }
    bool allTop = true;
    for (int p = 0; p < e.pointCount; ++p) {
      if (!near(e.points[p * 3 + 2], depth, 1e-6)) { allTop = false; break; }
    }
    if (allTop) {
      topRim.push_back(e.edgeId);
      double mx = 0.0, my = 0.0;
      for (int p = 0; p < e.pointCount; ++p) { mx += e.points[p * 3]; my += e.points[p * 3 + 1]; }
      topMid.push_back({mx / e.pointCount, my / e.pointCount});
    }
  }
  if (polys) { cc_edge_polylines_free(polys, nedge); }

  ctx.check(topRim.size() == 8, "rounded-rect top rim has 8 tangent edges",
            "got " + std::to_string(topRim.size()));
  if (topRim.empty()) { cc_shape_release(body); return; }

  const int seed = topRim.front();
  const std::set<int> rimSet(topRim.begin(), topRim.end());

  // cc_tangent_chain: one rim seed grows to all 8 tangent-continuous rim edges.
  int* tchain = nullptr;
  int tn = cc_tangent_chain(body, &seed, 1, &tchain);
  bool tAllRim = (tn == static_cast<int>(topRim.size()));
  for (int i = 0; i < tn && tAllRim; ++i) { if (!rimSet.count(tchain[i])) { tAllRim = false; } }
  ctx.check(tn == 8, "cc_tangent_chain(1 rim seed) == 8 edges", "got " + std::to_string(tn));
  ctx.check(tAllRim, "cc_tangent_chain returns exactly the top-rim set");
  if (tchain) { cc_ints_free(tchain); }

  // cc_outer_rim_chain resolves the cap by requiring ALL seeds coplanar with the
  // face — that coplanarity gate is exactly what rejects the flat side walls. A
  // LONE seed on a straight-walled prism is geometrically ambiguous: the top cap
  // and the abutting flat side wall are both planar and both contain the seed
  // edge's two endpoints, so a single seed can't distinguish them (it drags that
  // one side wall's bottom+vertical edges in). Seed two rim edges from opposite
  // sides so no single side wall holds both; the chain then grows to the whole
  // 8-edge cap loop and drops every side wall.
  int seedB = seed;
  double bestD2 = -1.0;
  for (std::size_t i = 0; i < topRim.size(); ++i) {
    const double dx = topMid[i][0] - topMid[0][0], dy = topMid[i][1] - topMid[0][1];
    const double d2 = dx * dx + dy * dy;
    if (d2 > bestD2) { bestD2 = d2; seedB = topRim[i]; }  // farthest rim edge => opposite side
  }
  const int rimSeeds[2] = {seed, seedB};
  int* rchain = nullptr;
  int rn = cc_outer_rim_chain(body, rimSeeds, 2, &rchain);
  bool rAllRim = (rn == static_cast<int>(topRim.size()));
  for (int i = 0; i < rn && rAllRim; ++i) { if (!rimSet.count(rchain[i])) { rAllRim = false; } }
  ctx.check(rn == 8, "cc_outer_rim_chain(2 spread rim seeds) == 8 edges",
            "got " + std::to_string(rn));
  ctx.check(rAllRim, "cc_outer_rim_chain returns exactly the top-rim set");
  if (rchain) { cc_ints_free(rchain); }

  cc_shape_release(body);
}

// ── Degenerate / guard behaviour ────────────────────────────────────────────
// Every query returns its documented failure sentinel for an unknown body id or
// a null/empty seed, without touching caller output past the sentinel.
void checkDegenerate(Ctx& ctx) {
  const CCShapeId bad = 999999;   // never registered

  CCMassProps mp = cc_mass_properties(bad);
  ctx.check(mp.valid == 0, "cc_mass_properties(bad id) valid==0");

  double buf[6] = {0};
  ctx.check(cc_principal_moments(bad, buf) == 0, "cc_principal_moments(bad id) == 0");
  ctx.check(cc_bounding_box(bad, buf) == 0, "cc_bounding_box(bad id) == 0");
  ctx.check(cc_face_axis(bad, 1, buf) == 0, "cc_face_axis(bad id) == 0");

  int* ids = reinterpret_cast<int*>(0x1);
  ctx.check(cc_subshape_ids(bad, 1, &ids) == 0 && ids == nullptr,
            "cc_subshape_ids(bad id) == 0 and nulls out");

  const int seed = 1;
  int* out = reinterpret_cast<int*>(0x1);
  ctx.check(cc_tangent_chain(bad, &seed, 1, &out) == 0 && out == nullptr,
            "cc_tangent_chain(bad id) == 0 and nulls out");
  out = reinterpret_cast<int*>(0x1);
  ctx.check(cc_outer_rim_chain(bad, &seed, 1, &out) == 0 && out == nullptr,
            "cc_outer_rim_chain(bad id) == 0 and nulls out");

  // Null/empty seed guard on a VALID body (so the guard, not a bad id, is tested).
  CCShapeId box = cc_solid_extrude(kRect20x10, 4, 30.0);
  if (ctx.check(box != 0, "query: box for null-seed guard")) {
    out = reinterpret_cast<int*>(0x1);
    ctx.check(cc_tangent_chain(box, nullptr, 0, &out) == 0 && out == nullptr,
              "cc_tangent_chain(null seed) == 0");
    out = reinterpret_cast<int*>(0x1);
    ctx.check(cc_outer_rim_chain(box, nullptr, 0, &out) == 0 && out == nullptr,
              "cc_outer_rim_chain(null seed) == 0");
    cc_shape_release(box);
  }
}

}  // namespace

void run_query_checks(Ctx& ctx) {
  checkMassAndMoments(ctx);
  checkBoundingBox(ctx);
  checkFaceAxis(ctx);
  checkSubshapeIds(ctx);
  checkChains(ctx);
  checkDegenerate(ctx);
}
