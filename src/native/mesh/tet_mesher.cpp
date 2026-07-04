// SPDX-License-Identifier: AGPL-3.0-or-later
//
// tet_mesher.cpp — the SOLE consumer of the external AGPL TetGen library.
//
// This translation unit is EXCLUDED from the default MIT build glob and compiled
// only under CYBERCAD_HAS_TETGEN (see CMakeLists.txt). The TetGen include dir and
// -DTETLIBRARY are scoped to this file alone via set_source_files_properties, so
// no other TU can even include <tetgen.h>. TetGen is referenced by absolute path
// and never vendored; it is licensed AGPL-3.0.
//
// Pipeline: validate -> marshal the triangle surface into a tetgenio PLC -> build
// the switch string (p q<ratio> [a<vol>] Q [Y]) -> tetrahedralize -> read LINEAR
// tets, enforce positive signed volume, then build C3D10 mid-edge nodes NATIVELY
// in CalculiX shape10tet order (TetGen's own -o2 ordering is NOT used).
//
#include "native/mesh/tet_mesher.h"

// Defensive: this TU is compiled ONLY under CYBERCAD_HAS_TETGEN (CMake scopes the
// TetGen include dir and -DTETLIBRARY to it alone). The guard means an accidental
// inclusion in the default MIT glob would compile to an empty TU instead of failing
// on the missing AGPL <tetgen.h> — it never silently pulls AGPL code into the
// default build (that requires the flag, which adds the include path and the lib).
#ifdef CYBERCAD_HAS_TETGEN

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <utility>

#include <tetgen.h>

namespace cybercad::native::mesh {

namespace {

TetMeshResult fail(const std::string& msg) {
  TetMeshResult r;
  r.ok = false;
  r.message = msg;
  return r;
}

// Validate before any allocation so an early return never leaks.
const char* validateSurface(const std::vector<double>& verts, const std::vector<int>& tris) {
  if (verts.empty() || tris.empty()) {
    return "tet mesher: empty surface (no vertices or triangles)";
  }
  if (verts.size() % 3 != 0) {
    return "tet mesher: vertex array length not a multiple of 3";
  }
  if (tris.size() % 3 != 0) {
    return "tet mesher: triangle array length not a multiple of 3";
  }
  const long nPts = static_cast<long>(verts.size() / 3);
  const long nTri = static_cast<long>(tris.size() / 3);
  if (nPts < 4 || nTri < 4) {
    return "tet mesher: surface too small to enclose a volume (need >= 4 points and "
           "4 triangles)";
  }
  for (int idx : tris) {
    if (idx < 0 || idx >= nPts) {
      return "tet mesher: triangle vertex index out of range";
    }
  }
  return nullptr;
}

// Marshal the triangle surface into `in`. All arrays use new[] because TetGen's
// tetgenio destructor frees them with delete[].
void marshalInput(tetgenio& in, const std::vector<double>& verts,
                  const std::vector<int>& tris) {
  const int nPts = static_cast<int>(verts.size() / 3);
  const int nTri = static_cast<int>(tris.size() / 3);

  in.firstnumber = 0;
  in.numberofpoints = nPts;
  in.pointlist = new REAL[static_cast<long>(nPts) * 3];
  for (long i = 0; i < static_cast<long>(nPts) * 3; ++i) {
    in.pointlist[i] = static_cast<REAL>(verts[i]);
  }

  in.numberoffacets = nTri;
  in.facetlist = new tetgenio::facet[nTri];
  in.facetmarkerlist = new int[nTri];
  for (int t = 0; t < nTri; ++t) {
    tetgenio::facet& f = in.facetlist[t];
    f.numberofpolygons = 1;
    f.polygonlist = new tetgenio::polygon[1];
    f.numberofholes = 0;
    f.holelist = nullptr;
    tetgenio::polygon& p = f.polygonlist[0];
    p.numberofvertices = 3;
    p.vertexlist = new int[3];
    p.vertexlist[0] = tris[static_cast<long>(t) * 3 + 0];
    p.vertexlist[1] = tris[static_cast<long>(t) * 3 + 1];
    p.vertexlist[2] = tris[static_cast<long>(t) * 3 + 2];
    in.facetmarkerlist[t] = 0;
  }
}

// Build the TetGen switch string. Always p (PLC) + q<ratio> + Q (quiet).
//
// a<vol> is appended only when a positive target element size is given. On that
// SIZED path we deliberately DO NOT emit Y: Y forbids Steiner points on the input
// boundary, and TetGen cannot honor a small max-volume cap while the coarse input
// boundary is frozen — it silently returns the unrefined mesh (verified: a unit
// cube stays 6 tets under "pq..aQY" regardless of the cap, vs 100s under "pq..aQ").
// When a target size is requested, boundary refinement is expected and desired, so
// we let TetGen split boundary faces to meet the cap; the mesh stays watertight.
//
// On the UNSIZED pure-fill path we keep Y to preserve the input boundary
// triangulation so the mesh boundary matches the CAD tessellation exactly.
std::string buildSwitches(const VolumeMeshOptions& opts) {
  double q = opts.radius_edge_ratio;
  if (!(q >= 1.0)) {
    q = 1.0;
  }
  char buf[128];
  if (opts.target_element_size > 0.0) {
    const double h = opts.target_element_size;
    const double maxvol = (h * h * h) / (6.0 * std::sqrt(2.0));  // regular-tet volume
    std::snprintf(buf, sizeof(buf), "pq%.*ga%.*gQ", 10, q, 10, maxvol);
  } else {
    std::snprintf(buf, sizeof(buf), "pq%.*gQY", 10, q);
  }
  return std::string(buf);
}

// Signed 6*volume of a tet given its 4 corners (row-major xyz).
double signedVol6(const REAL* c0, const REAL* c1, const REAL* c2, const REAL* c3) {
  const double ax = c1[0] - c0[0], ay = c1[1] - c0[1], az = c1[2] - c0[2];
  const double bx = c2[0] - c0[0], by = c2[1] - c0[1], bz = c2[2] - c0[2];
  const double cx = c3[0] - c0[0], cy = c3[1] - c0[1], cz = c3[2] - c0[2];
  return ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
}

// Pack an undirected edge (a,b) into a stable key for mid-node deduplication.
uint64_t edgeKey(int a, int b) {
  if (a > b) {
    const int t = a;
    a = b;
    b = t;
  }
  return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
         static_cast<uint64_t>(static_cast<uint32_t>(b));
}

}  // namespace

TetMeshResult tetrahedralize_surface(const std::vector<double>& verts,
                                     const std::vector<int>& tris,
                                     const VolumeMeshOptions& opts) {
  if (const char* err = validateSurface(verts, tris)) {
    return fail(err);
  }

  tetgenio in, out;  // stack objects: destructors free all new[] arrays on return.
  marshalInput(in, verts, tris);

  const std::string switches = buildSwitches(opts);
  tetgenbehavior b;
  // parse_commandline takes char*; copy into a mutable buffer.
  std::vector<char> sw(switches.begin(), switches.end());
  sw.push_back('\0');
  if (!b.parse_commandline(sw.data())) {
    return fail("tet mesher: TetGen rejected switch string '" + switches + "'");
  }

  try {
    tetrahedralize(&b, &in, &out);
  } catch (...) {
    return fail("tet mesher: TetGen failed (non-watertight / degenerate surface?)");
  }

  if (out.numberoftetrahedra <= 0 || out.numberofcorners < 4 || out.pointlist == nullptr) {
    return fail("tet mesher: TetGen produced no tetrahedra");
  }

  const int nOut = out.numberofpoints;
  const int nTet = out.numberoftetrahedra;
  const int corners = out.numberofcorners;  // 4 for a linear mesh

  TetMesh m;
  m.nodes.assign(out.pointlist, out.pointlist + static_cast<long>(nOut) * 3);

  // Read the 4 linear corners of every tet, enforcing positive signed volume by
  // swapping the first two corners when the orientation is negative.
  std::vector<std::array<int, 4>> tets;
  tets.reserve(nTet);
  for (int t = 0; t < nTet; ++t) {
    const int* row = &out.tetrahedronlist[static_cast<long>(t) * corners];
    int v0 = row[0], v1 = row[1], v2 = row[2], v3 = row[3];
    const REAL* p0 = &m.nodes[static_cast<long>(v0) * 3];
    const REAL* p1 = &m.nodes[static_cast<long>(v1) * 3];
    const REAL* p2 = &m.nodes[static_cast<long>(v2) * 3];
    const REAL* p3 = &m.nodes[static_cast<long>(v3) * 3];
    if (signedVol6(p0, p1, p2, p3) < 0.0) {
      const int tmp = v0;
      v0 = v1;
      v1 = tmp;
    }
    tets.push_back({v0, v1, v2, v3});
  }

  if (opts.order == MeshOrder::Linear) {
    m.nodes_per_elem = 4;
    m.order = 4;
    m.connectivity.reserve(static_cast<long>(nTet) * 4);
    for (const auto& tet : tets) {
      m.connectivity.insert(m.connectivity.end(), tet.begin(), tet.end());
    }
    m.node_count = static_cast<int>(m.nodes.size() / 3);
    m.element_count = nTet;
    TetMeshResult r;
    r.ok = true;
    r.mesh = std::move(m);
    return r;
  }

  // C3D10: build mid-edge nodes natively at edge midpoints, deduplicated and
  // appended AFTER all corner/Steiner nodes so corner indices stay stable and the
  // mesh stays watertight (shared edges share one mid-node). CalculiX shape10tet
  // order: c1 c2 c3 c4, then mid(1,2) mid(2,3) mid(3,1) mid(1,4) mid(2,4) mid(3,4).
  std::map<uint64_t, int> midOf;
  auto midNode = [&](int a, int c) -> int {
    const uint64_t key = edgeKey(a, c);
    auto it = midOf.find(key);
    if (it != midOf.end()) {
      return it->second;
    }
    const long ai = static_cast<long>(a) * 3, ci = static_cast<long>(c) * 3;
    const int id = static_cast<int>(m.nodes.size() / 3);
    m.nodes.push_back(0.5 * (m.nodes[ai + 0] + m.nodes[ci + 0]));
    m.nodes.push_back(0.5 * (m.nodes[ai + 1] + m.nodes[ci + 1]));
    m.nodes.push_back(0.5 * (m.nodes[ai + 2] + m.nodes[ci + 2]));
    midOf.emplace(key, id);
    return id;
  };

  m.nodes_per_elem = 10;
  m.order = 10;
  m.connectivity.reserve(static_cast<long>(nTet) * 10);
  for (const auto& tet : tets) {
    const int c1 = tet[0], c2 = tet[1], c3 = tet[2], c4 = tet[3];
    const int e5 = midNode(c1, c2), e6 = midNode(c2, c3), e7 = midNode(c3, c1);
    const int e8 = midNode(c1, c4), e9 = midNode(c2, c4), e10 = midNode(c3, c4);
    const int row[10] = {c1, c2, c3, c4, e5, e6, e7, e8, e9, e10};
    m.connectivity.insert(m.connectivity.end(), row, row + 10);
  }
  m.node_count = static_cast<int>(m.nodes.size() / 3);
  m.element_count = nTet;

  TetMeshResult r;
  r.ok = true;
  r.mesh = std::move(m);
  return r;
}

}  // namespace cybercad::native::mesh

#endif  // CYBERCAD_HAS_TETGEN
