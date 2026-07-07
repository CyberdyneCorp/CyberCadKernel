// SPDX-License-Identifier: Apache-2.0
//
// native_face_split_parity.mm — MOAT M2b / B2 (freeform face-split) SIM GATE (b):
// native-vs-OCCT on a booted iOS simulator.
//
// The native B2 splitter (src/native/boolean/face_split.h, OCCT-FREE) partitions ONE
// trimmed freeform Bézier face along the real M1 seam (ssi::trace_intersection) into
// TWO genuinely-trimmed sub-faces, which the M0 FaceMesher meshes with NO tessellator
// change. This harness proves, against the OCCT ORACLE:
//
//   1. WATERTIGHT per sub-face — each sub-face mesh is a 2-manifold-with-boundary
//      (no non-manifold edge), and every sub-face vertex lies on the OCCT Bézier
//      surface within the deflection bound.
//   2. THE SEAM WELDS + TOPOLOGY — welding the two sub-face meshes by position yields
//      a single 2-manifold DISK (Euler χ = V−E+F = 1): the shared seam points are
//      bit-identical (boundary edges drop by the seam length), so the union reproduces
//      the parent face's topology with no crack.
//   3. AREA TILING vs OCCT — areaIn + areaOut ≈ the OCCT BRepGProp surface area of the
//      SAME parent trimmed Bézier face (BRepMesh oracle), i.e. the split tiles the true
//      curved face; and the native parent mesh area matches OCCT too (M0 fidelity).
//
// OCCT is the ORACLE ONLY, never linked into src/native. The seam/trace/split/mesh are
// 100% native; OCCT builds the same trimmed Bézier face and measures its area.
//
// Build: scripts/run-sim-native-face-split.sh (mirrors run-sim-native-ssi-marching:
// OCCT oracle slice + the NumPP/SciPP numsci archive for the S3 corrector).
//
#include "native/boolean/face_split.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "../native/face_split_fixture.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_face_split_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopLoc_Location.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Lin2d.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>

namespace bo = cybercad::native::boolean;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace fx = face_split_fixture;
namespace nm = cybercad::native::math;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[NFS] %-28s %-22s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

// ── OCCT parent: the SAME bowl Bézier patch trimmed by the SAME convex quad ─────
static TopoDS_Face buildOcctParent() {
  const std::vector<nm::Point3> poles = fx::bowlPoles();  // row-major, U outer, 3×3
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const nm::Point3& p = poles[i * 3 + j];
      arr.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);

  const auto& q = fx::quadUV();
  BRepBuilderAPI_MakeWire mkWire;
  for (int k = 0; k < 4; ++k) {
    const nm::Point3 a = q[k];
    const nm::Point3 b = q[(k + 1) % 4];
    const gp_Pnt2d p0(a.x, a.y), p1(b.x, b.y);
    const double len = p0.Distance(p1);
    gp_Dir2d dir(p1.X() - p0.X(), p1.Y() - p0.Y());
    Handle(Geom2d_Line) line = new Geom2d_Line(p0, dir);
    Handle(Geom2d_TrimmedCurve) seg = new Geom2d_TrimmedCurve(line, 0.0, len);
    TopoDS_Edge e = BRepBuilderAPI_MakeEdge(seg, surf, 0.0, len).Edge();
    mkWire.Add(e);
  }
  TopoDS_Wire wire = mkWire.Wire();
  TopoDS_Face face = BRepBuilderAPI_MakeFace(surf, wire, /*Inside=*/Standard_True).Face();
  BRepLib::BuildCurves3d(face);
  return face;
}

static double occtSurfaceArea(const TopoDS_Face& f, double defl) {
  BRepMesh_IncrementalMesh mesher(f, defl);
  mesher.Perform();
  GProp_GProps props;
  BRepGProp::SurfaceProperties(f, props);
  return props.Mass();
}

// ── Native mesh helpers ─────────────────────────────────────────────────────────
struct WeldKey {
  long long x, y, z;
  bool operator==(const WeldKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
};
struct WeldHash {
  std::size_t operator()(const WeldKey& k) const noexcept {
    std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
    h ^= static_cast<std::size_t>(k.y) * 19349663u;
    h ^= static_cast<std::size_t>(k.z) * 83492791u;
    return h;
  }
};
static long long qcoord(double v) noexcept {
  const double s = 1.0 / 1e-9;  // 1e-9 weld quantum (≫ ULP of the seam, ≪ any feature)
  return static_cast<long long>(v >= 0 ? v * s + 0.5 : v * s - 0.5);
}

// Concatenate two face meshes and weld coincident vertices by position → one mesh.
static ntess::Mesh weldUnion(const ntess::Mesh& a, const ntess::Mesh& b) {
  ntess::Mesh out;
  std::unordered_map<WeldKey, std::uint32_t, WeldHash> idx;
  auto append = [&](const ntess::Mesh& m) {
    std::vector<std::uint32_t> remap(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
      const nm::Point3& p = m.vertices[i];
      const WeldKey k{qcoord(p.x), qcoord(p.y), qcoord(p.z)};
      auto it = idx.find(k);
      if (it == idx.end()) {
        const auto ni = static_cast<std::uint32_t>(out.vertices.size());
        idx.emplace(k, ni);
        out.vertices.push_back(p);
        remap[i] = ni;
      } else {
        remap[i] = it->second;
      }
    }
    for (const ntess::Triangle& t : m.triangles)
      out.triangles.push_back(ntess::Triangle{remap[t.a], remap[t.b], remap[t.c]});
  };
  append(a);
  append(b);
  return out;
}

// Euler characteristic V − E + F of a triangle mesh (E = unique undirected edges).
static long eulerChar(const ntess::Mesh& m) {
  const long V = static_cast<long>(m.vertices.size());
  const long F = static_cast<long>(m.triangles.size());
  const long E = static_cast<long>(ntess::edgeUseCounts(m).size());
  return V - E + F;
}

// Max distance of any mesh vertex from the OCCT Bézier surface.
static double maxDistToSurface(const ntess::Mesh& m, const Handle(Geom_Surface)& s) {
  double worst = 0.0;
  for (const nm::Point3& v : m.vertices) {
    GeomAPI_ProjectPointOnSurf proj(gp_Pnt(v.x, v.y, v.z), s);
    if (proj.NbPoints() > 0) worst = std::max(worst, static_cast<double>(proj.LowerDistance()));
  }
  return worst;
}

int main() {
  std::printf("== MOAT M2b/B2 freeform face-split: native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  const double defl = 5e-3;

  // ── native: build parent, trace the real M1 seam, split ────────────────────
  const nt::Shape face = fx::parentFace();
  const cybercad::native::ssi::WLine seam = fx::seamWLine();
  report("split", "seam-traced", seam.points.size() >= 2, "real S3 WLine");

  const bo::SplitResult r = bo::splitFace(face, seam);
  {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "crossings=%d decline=%s gap=%.2e", r.crossings,
                  bo::declineName(r.decline), r.tilingGap);
    report("split", "verified-split", r.ok(), buf);
  }
  if (!r.ok()) {
    std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
    std::fflush(stdout);
    std::_Exit(g_fail == 0 ? 0 : 1);
  }
  const bo::FaceSplit& s = *r.split;

  // ── native: mesh the parent + both sub-faces (M0, unchanged) ────────────────
  ntess::MeshParams mp;
  mp.deflection = defl;
  const ntess::FaceMesher fm(mp);
  const ntess::Mesh mParent = fm.mesh(face);
  const ntess::Mesh mIn = fm.mesh(s.faceIn);
  const ntess::Mesh mOut = fm.mesh(s.faceOut);

  report("subfaceIn", "meshes", !mIn.triangles.empty(), "M0 trimmed-freeform");
  report("subfaceOut", "meshes", !mOut.triangles.empty(), "M0 trimmed-freeform");

  // (1) each sub-face is a 2-manifold-with-boundary (no crack inside the sub-face).
  report("subfaceIn", "two-manifold", ntess::isTwoManifold(mIn), "no non-manifold edge");
  report("subfaceOut", "two-manifold", ntess::isTwoManifold(mOut), "no non-manifold edge");

  // ── OCCT oracle parent ─────────────────────────────────────────────────────
  const TopoDS_Face occtFace = buildOcctParent();
  Handle(Geom_Surface) occtSurf = BRep_Tool::Surface(occtFace);

  // (1b) every sub-face vertex lies on the true OCCT surface within the bound.
  {
    const double dIn = maxDistToSurface(mIn, occtSurf);
    const double dOut = maxDistToSurface(mOut, occtSurf);
    char b1[80], b2[80];
    std::snprintf(b1, sizeof(b1), "maxDist=%.3e defl=%.3e", dIn, defl);
    std::snprintf(b2, sizeof(b2), "maxDist=%.3e defl=%.3e", dOut, defl);
    report("subfaceIn", "vertices-on-surface", dIn <= defl * 1.5 + 1e-7, b1);
    report("subfaceOut", "vertices-on-surface", dOut <= defl * 1.5 + 1e-7, b2);
  }

  // (2) SEAM WELDS + TOPOLOGY: weld the two sub-face meshes → one 2-manifold disk.
  const ntess::Mesh welded = weldUnion(mIn, mOut);
  const std::size_t beIn = ntess::boundaryEdgeCount(mIn);
  const std::size_t beOut = ntess::boundaryEdgeCount(mOut);
  const std::size_t beWeld = ntess::boundaryEdgeCount(welded);
  {
    const bool manifold = ntess::isTwoManifold(welded);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "boundary %zu+%zu -> %zu", beIn, beOut, beWeld);
    // The seam edges (boundary in each half) become interior after the weld, so the
    // welded boundary is STRICTLY fewer than the two halves summed (bit-identical seam
    // points), and no edge is non-manifold.
    report("seam-weld", "welds-manifold", manifold && beWeld < beIn + beOut, buf);
  }
  {
    const long chi = eulerChar(welded);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "euler=%ld (disk=1)", chi);
    report("union", "topology-disk", chi == 1, buf);
  }

  // (3) AREA TILING vs OCCT.
  const double aParentNat = ntess::surfaceArea(mParent);
  const double aIn = ntess::surfaceArea(mIn);
  const double aOut = ntess::surfaceArea(mOut);
  const double aOcct = occtSurfaceArea(occtFace, defl);
  {
    const double rel = std::fabs(aParentNat - aOcct) / aOcct;
    char buf[112];
    std::snprintf(buf, sizeof(buf), "native=%.5f occt=%.5f rel=%.3e", aParentNat, aOcct, rel);
    report("parent", "area-vs-occt", rel <= 0.02, buf);  // M0 mesh reproduces true area
  }
  {
    const double sum = aIn + aOut;
    const double relNat = std::fabs(sum - aParentNat) / aParentNat;
    const double relOcct = std::fabs(sum - aOcct) / aOcct;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "in+out=%.5f nat=%.5f occt=%.5f relNat=%.2e relOcct=%.2e",
                  sum, aParentNat, aOcct, relNat, relOcct);
    // The headline claim: the two sub-faces TILE the true curved parent face.
    report("split", "area-tiles-occt", relNat <= 1e-3 && relOcct <= 0.02, buf);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
