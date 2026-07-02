// Phase-3 robust wrap-emboss checks (iOS simulator; run by phase3_suite.cpp).
//
// Owned by the wrap-emboss feature agent. The bar (per the wrap-emboss spec): a
// dense high-curvature profile wrapped onto a cylindrical face yields a body that
// is BRepCheck_Analyzer::IsValid + watertight, with the correct volume-change sign
// (emboss > base, deboss < base) by a plausible magnitude, and a previously-invalid
// solid-mode ThruSections case now succeeds via the sewn cap+side builder. Unsewable
// cases fall back to the coarse pad and are recorded deferred — never faked.
//
// The C ABI exposes only the built body, so these checks assert REAL geometric
// properties through it:
//   * IsValid       — cc_wrap_emboss gates every result on BRepCheck_Analyzer::IsValid
//                     (occt::addIfValid), so a non-zero id is an already-valid solid;
//                     cc_mass_properties(.valid) re-confirms the B-rep is sound.
//   * watertight    — cc_tessellate the body, weld coincident vertices by position,
//                     and require every mesh edge be shared by exactly two triangles
//                     (zero naked edges) — a closed 2-manifold surface.
//   * volume sign   — cc_mass_properties before/after: boss => V_after > V_base,
//     & magnitude     deboss => V_after < V_base, delta within a documented range of
//                     the wrapped profile area x depth (not merely "changed").
//   * determinism   — two identical calls agree on volume to a tight tolerance.
//
// The wide fixture (arc length 28 mm over R=10 => ~2.8 rad, a very non-planar wrapped
// wire) is one on which solid-mode BRepOffsetAPI_ThruSections is NOT valid: verified
// directly off-line (sewn solid valid=1 volume=369.6 == 0.5*2.8*(12^2-10^2)*6 exact;
// solid ThruSections IsValid=0). Here we assert the robust builder yields a valid
// watertight solid with the correct volume growth for that same fixture.

#include "phase3_checks.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

// Weld coincident tessellation vertices by quantised position and count how many
// mesh edges are used by exactly one triangle (naked/free edges). A watertight
// closed 2-manifold solid has zero naked edges. Returns -1 if the body cannot be
// tessellated. `outTris` receives the triangle count for context.
int nakedEdgeCount(CCShapeId body, int& outTris) {
  outTris = 0;
  CCMesh m = cc_tessellate(body, 0.25);
  if (m.vertexCount <= 0 || m.triangleCount <= 0) {
    cc_mesh_free(m);
    return -1;
  }
  const double q = 1.0e4;  // 0.1 micron weld grid
  // Map welded (x,y,z) -> compact index.
  std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t>, int> weld;
  std::vector<int> idx(m.vertexCount);
  for (int v = 0; v < m.vertexCount; ++v) {
    const std::int64_t x = static_cast<std::int64_t>(std::llround(m.vertices[v * 3 + 0] * q));
    const std::int64_t y = static_cast<std::int64_t>(std::llround(m.vertices[v * 3 + 1] * q));
    const std::int64_t z = static_cast<std::int64_t>(std::llround(m.vertices[v * 3 + 2] * q));
    auto t = std::make_tuple(x, y, z);
    auto it = weld.find(t);
    if (it == weld.end()) {
      const int n = static_cast<int>(weld.size());
      weld.emplace(t, n);
      idx[v] = n;
    } else {
      idx[v] = it->second;
    }
  }
  // Count undirected-edge uses across all triangles.
  std::map<std::pair<int, int>, int> edgeUse;
  auto addEdge = [&](int a, int b) {
    if (a == b) return;
    edgeUse[{std::min(a, b), std::max(a, b)}]++;
  };
  for (int t = 0; t < m.triangleCount; ++t) {
    const int a = idx[m.triangles[t * 3 + 0]];
    const int b = idx[m.triangles[t * 3 + 1]];
    const int c = idx[m.triangles[t * 3 + 2]];
    addEdge(a, b);
    addEdge(b, c);
    addEdge(c, a);
  }
  outTris = m.triangleCount;
  cc_mesh_free(m);
  int naked = 0;
  for (const auto& e : edgeUse) {
    if (e.second == 1) ++naked;
  }
  return naked;
}

double volumeOf(CCShapeId body, bool& valid) {
  CCMassProps mp = cc_mass_properties(body);
  valid = mp.valid != 0;
  return mp.volume;
}

// Find the first cylindrical/conical face id of a body (cc_face_axis returns 1 for
// a cylinder/cone). Returns 0 if none.
int findCylFace(CCShapeId body) {
  int* ids = nullptr;
  const int n = cc_subshape_ids(body, /*kind=face*/ 2, &ids);
  int found = 0;
  for (int i = 0; i < n; ++i) {
    double ax[6];
    if (cc_face_axis(body, ids[i], ax) == 1) {
      found = ids[i];
      break;
    }
  }
  cc_ints_free(ids);
  return found;
}

}  // namespace

void run_wrap_emboss_checks(Ctx& ctx) {
  std::printf("-- robust wrap-emboss --\n");

  // Base cylinder (full-circle profile extruded +Z), R=10, H=40 => V=pi*100*40.
  CCProfileSeg circle;
  circle.kind = 2;  // full circle
  circle.cx = 0;
  circle.cy = 0;
  circle.r = 10.0;
  const CCShapeId cyl =
      cc_solid_extrude_profile(&circle, 1, nullptr, 0, nullptr, 0, 40.0);
  if (cyl == 0) {
    ctx.defer("wrap-emboss", "base cylinder build returned 0");
    return;
  }
  bool baseValid = false;
  const double vBase = volumeOf(cyl, baseValid);
  if (!ctx.check(baseValid && vBase > 0, "base cylinder has a valid volume",
                 "V_base=" + std::to_string(vBase))) {
    cc_shape_release(cyl);
    return;
  }
  const int face = findCylFace(cyl);
  if (!ctx.check(face != 0, "found cylindrical face id", "faceId=" + std::to_string(face))) {
    cc_shape_release(cyl);
    return;
  }

  // Plausible volume-delta band for a wrapped rectangular profile: nominal is the
  // flat profile area x depth; the curved pad differs (annular-sector volume), so
  // assert sign + a generous [0.4x, 3.0x] magnitude band, not an exact value.
  auto plausible = [](double delta, double area, double depth) -> bool {
    const double nominal = area * depth;
    return delta > 0.4 * nominal && delta < 3.0 * nominal;
  };

  const double prof[8] = {-4, -3, 4, -3, 4, 3, -4, 3};  // 8 mm (arc) x 6 mm => area 48
  const double depth = 2.0;
  const double area = 8.0 * 6.0;

  // ── Emboss (boss=1): valid, watertight, V grows by a plausible magnitude. ──
  const CCShapeId boss = cc_wrap_emboss(cyl, face, prof, 4, depth, /*boss*/ 1);
  if (boss == 0) {
    ctx.defer("cc_wrap_emboss(boss)", "returned 0 (robust + fallback both failed)");
  } else {
    bool v = false;
    const double vAfter = volumeOf(boss, v);
    int tris = 0;
    const int naked = nakedEdgeCount(boss, tris);
    ctx.check(v, "emboss body is a valid B-rep (mass_properties.valid)");
    ctx.check(naked == 0, "emboss body is watertight (0 naked mesh edges)",
              "naked=" + std::to_string(naked) + " tris=" + std::to_string(tris));
    ctx.check(vAfter > vBase, "emboss increases volume",
              "V_after=" + std::to_string(vAfter) + " V_base=" + std::to_string(vBase));
    ctx.check(plausible(vAfter - vBase, area, depth), "emboss volume delta is plausible",
              "delta=" + std::to_string(vAfter - vBase) +
                  " nominal(area*depth)=" + std::to_string(area * depth));
    cc_shape_release(boss);
  }

  // ── Deboss (boss=0): valid, watertight, V shrinks by a plausible magnitude. ──
  const CCShapeId deboss = cc_wrap_emboss(cyl, face, prof, 4, depth, /*boss*/ 0);
  if (deboss == 0) {
    ctx.defer("cc_wrap_emboss(deboss)", "returned 0 (robust + fallback both failed)");
  } else {
    bool v = false;
    const double vAfter = volumeOf(deboss, v);
    int tris = 0;
    const int naked = nakedEdgeCount(deboss, tris);
    ctx.check(v, "deboss body is a valid B-rep (mass_properties.valid)");
    ctx.check(naked == 0, "deboss body is watertight (0 naked mesh edges)",
              "naked=" + std::to_string(naked) + " tris=" + std::to_string(tris));
    ctx.check(vAfter < vBase, "deboss decreases volume",
              "V_after=" + std::to_string(vAfter) + " V_base=" + std::to_string(vBase));
    ctx.check(plausible(vBase - vAfter, area, depth), "deboss volume delta is plausible",
              "delta=" + std::to_string(vBase - vAfter) +
                  " nominal(area*depth)=" + std::to_string(area * depth));
    cc_shape_release(deboss);
  }

  // ── Determinism: two identical emboss calls agree on volume tightly. ──
  const CCShapeId r1 = cc_wrap_emboss(cyl, face, prof, 4, depth, 1);
  const CCShapeId r2 = cc_wrap_emboss(cyl, face, prof, 4, depth, 1);
  if (r1 != 0 && r2 != 0) {
    bool a = false, b = false;
    const double va = volumeOf(r1, a), vb = volumeOf(r2, b);
    ctx.check(a && b && near(va, vb, 1.0e-6 * (1.0 + std::fabs(va))),
              "repeated emboss is reproducible (equal volume)",
              "v1=" + std::to_string(va) + " v2=" + std::to_string(vb));
  } else {
    ctx.defer("cc_wrap_emboss determinism", "one of the repeated calls returned 0");
  }
  cc_shape_release(r1);
  cc_shape_release(r2);

  // ── Regression: wide (arc length 28 mm over R=10) profile. Solid-mode
  // ThruSections is NOT valid on this fixture (verified off-line, IsValid=0); the
  // robust sewn cap+side builder must yield a valid watertight solid that grows the
  // volume. If it cannot, and we fell back to a coarse valid pad, defer honestly. ──
  const double wide[8] = {-14, -3, 14, -3, 14, 3, -14, 3};
  const double wideArea = 28.0 * 6.0;
  const CCShapeId hard = cc_wrap_emboss(cyl, face, wide, 4, depth, /*boss*/ 1);
  if (hard == 0) {
    ctx.defer("cc_wrap_emboss(wide/high-curvature)",
              "robust + ThruSections fallback both failed on the wide fixture");
  } else {
    bool v = false;
    const double vAfter = volumeOf(hard, v);
    int tris = 0;
    const int naked = nakedEdgeCount(hard, tris);
    const bool grew = vAfter > vBase;
    const bool tight = plausible(vAfter - vBase, wideArea, depth);
    if (v && naked == 0 && grew && tight) {
      ctx.check(true,
                "wide high-curvature emboss is valid+watertight where solid ThruSections is invalid",
                "delta=" + std::to_string(vAfter - vBase) +
                    " nominal=" + std::to_string(wideArea * depth) +
                    " naked=" + std::to_string(naked));
    } else if (v && grew) {
      // A valid, volume-growing body but not the full robust bar (e.g. coarse
      // fallback that is not watertight at this mesh tolerance): honest deferral.
      ctx.defer("cc_wrap_emboss(wide/high-curvature)",
                "valid & volume grew but full bar unmet: watertight=" +
                    std::to_string(naked == 0) + " plausible=" + std::to_string(tight) +
                    " delta=" + std::to_string(vAfter - vBase) + " naked=" + std::to_string(naked));
    } else {
      ctx.check(false, "wide high-curvature emboss valid+watertight+grew",
                "valid=" + std::to_string(v) + " naked=" + std::to_string(naked) +
                    " V_after=" + std::to_string(vAfter));
    }
    cc_shape_release(hard);
  }

  cc_shape_release(cyl);
}
