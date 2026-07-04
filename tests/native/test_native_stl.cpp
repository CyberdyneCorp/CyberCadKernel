// SPDX-License-Identifier: Apache-2.0
//
// Facade-level tests for STL exchange (issues #4 export + #5 import). OCCT-FREE:
// they drive the cc_* ABI under the NATIVE engine (cc_set_engine(1)) so a real
// triangle mesh is produced on the host with no OCCT. They cover:
//   * binary export round-trip (facet count field matches file size; re-import →
//     same triangle count + bounding box),
//   * well-formed ASCII export (balanced keywords, no decimal comma),
//   * deterministic byte-identical export (binary + ASCII),
//   * ASCII/binary auto-detect importing identically,
//   * the regression where a BINARY header starts with "solid " (size-identity wins),
//   * malformed input failing cleanly (0 + cc_last_error, no crash),
//   * measurement of an imported closed box (watertight volume) vs an open soup.
//
#include "cybercadkernel/cc_kernel.h"

#include "harness.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

std::string tmpPath(const char* name) {
    std::filesystem::path p = std::filesystem::temp_directory_path();
    p /= name;
    return p.string();
}

// Build a 10×10×10 native box (native engine must be active).
CCShapeId buildBox() {
    const double profile[] = {0, 0, 10, 0, 10, 10, 0, 10};
    return cc_solid_extrude(profile, 4, 10.0);
}

std::vector<unsigned char> readBytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const std::streamsize n = f.tellg();
    std::vector<unsigned char> b(static_cast<std::size_t>(n < 0 ? 0 : n));
    f.seekg(0);
    if (!b.empty()) f.read(reinterpret_cast<char*>(b.data()), static_cast<std::streamsize>(b.size()));
    return b;
}

std::uint32_t u32le(const std::vector<unsigned char>& b, std::size_t o) {
    return static_cast<std::uint32_t>(b[o]) | (static_cast<std::uint32_t>(b[o + 1]) << 8) |
           (static_cast<std::uint32_t>(b[o + 2]) << 16) |
           (static_cast<std::uint32_t>(b[o + 3]) << 24);
}

int countOf(const std::string& hay, const std::string& needle) {
    int n = 0;
    for (std::size_t p = 0; (p = hay.find(needle, p)) != std::string::npos; p += needle.size()) ++n;
    return n;
}

std::string readText(const std::string& path) {
    const std::vector<unsigned char> b = readBytes(path);
    return std::string(reinterpret_cast<const char*>(b.data()), b.size());
}

}  // namespace

// 1 ── binary export round-trips to the same triangle count + bounding box ────────
CC_TEST(stl_binary_export_roundtrip) {
    cc_set_engine(1);
    const CCShapeId box = buildBox();
    CC_CHECK(box != 0);

    const std::string path = tmpPath("cc_stl_roundtrip.stl");
    CC_CHECK(cc_stl_export(box, path.c_str(), 0.1, 1) == 1);

    const std::vector<unsigned char> bytes = readBytes(path);
    CC_CHECK(bytes.size() >= 84);
    const std::uint32_t count = u32le(bytes, 80);
    CC_CHECK(count > 0);
    CC_CHECK(bytes.size() == static_cast<std::size_t>(84) + 50u * count);
    // The 80-byte header must NOT masquerade as ASCII.
    CC_CHECK(std::memcmp(bytes.data(), "solid", 5) != 0);

    CCMesh src = cc_tessellate(box, 0.1);
    CC_CHECK(static_cast<std::uint32_t>(src.triangleCount) == count);
    cc_mesh_free(src);

    const CCShapeId imported = cc_stl_import(path.c_str());
    CC_CHECK(imported != 0);
    CCMesh re = cc_tessellate(imported, 0.1);
    CC_CHECK(static_cast<std::uint32_t>(re.triangleCount) == count);
    cc_mesh_free(re);

    double a[6] = {0}, b[6] = {0};
    CC_CHECK(cc_bounding_box(box, a) == 1);
    CC_CHECK(cc_bounding_box(imported, b) == 1);
    for (int i = 0; i < 6; ++i) CC_CHECK(std::fabs(a[i] - b[i]) < 1e-3);

    cc_shape_release(box);
    cc_shape_release(imported);
}

// 2 ── ASCII export is well-formed with balanced keywords and no decimal comma ────
CC_TEST(stl_ascii_export_wellformed) {
    cc_set_engine(1);
    const CCShapeId box = buildBox();
    CC_CHECK(box != 0);

    const std::string path = tmpPath("cc_stl_ascii.stl");
    CC_CHECK(cc_stl_export(box, path.c_str(), 0.1, 0) == 1);

    const std::string s = readText(path);
    CC_CHECK(!s.empty());
    CC_CHECK(s.rfind("solid CyberCadKernel", 0) == 0);
    CC_CHECK(countOf(s, "endsolid CyberCadKernel") == 1);
    const int facets = countOf(s, "facet normal");
    CC_CHECK(facets > 0);
    CC_CHECK(countOf(s, "outer loop") == facets);
    CC_CHECK(countOf(s, "endloop") == facets);
    CC_CHECK(countOf(s, "endfacet") == facets);
    CC_CHECK(countOf(s, "vertex ") == facets * 3);
    CC_CHECK(s.find(',') == std::string::npos);  // never a decimal comma

    cc_shape_release(box);
}

// 3 ── export is deterministic (byte-identical on repeat), binary + ASCII ─────────
CC_TEST(stl_export_deterministic) {
    cc_set_engine(1);
    const CCShapeId box = buildBox();
    CC_CHECK(box != 0);

    const std::string b1 = tmpPath("cc_stl_det_b1.stl"), b2 = tmpPath("cc_stl_det_b2.stl");
    CC_CHECK(cc_stl_export(box, b1.c_str(), 0.1, 1) == 1);
    CC_CHECK(cc_stl_export(box, b2.c_str(), 0.1, 1) == 1);
    CC_CHECK(readBytes(b1) == readBytes(b2));

    const std::string a1 = tmpPath("cc_stl_det_a1.stl"), a2 = tmpPath("cc_stl_det_a2.stl");
    CC_CHECK(cc_stl_export(box, a1.c_str(), 0.1, 0) == 1);
    CC_CHECK(cc_stl_export(box, a2.c_str(), 0.1, 0) == 1);
    CC_CHECK(readBytes(a1) == readBytes(a2));

    cc_shape_release(box);
}

// 4 ── ASCII and binary of the same body auto-detect and import identically ───────
CC_TEST(stl_autodetect_ascii_and_binary) {
    cc_set_engine(1);
    const CCShapeId box = buildBox();
    CC_CHECK(box != 0);

    const std::string bp = tmpPath("cc_stl_ad_b.stl"), ap = tmpPath("cc_stl_ad_a.stl");
    CC_CHECK(cc_stl_export(box, bp.c_str(), 0.1, 1) == 1);
    CC_CHECK(cc_stl_export(box, ap.c_str(), 0.1, 0) == 1);

    const CCShapeId fromBin = cc_stl_import(bp.c_str());
    const CCShapeId fromAsc = cc_stl_import(ap.c_str());
    CC_CHECK(fromBin != 0);
    CC_CHECK(fromAsc != 0);

    CCMesh mb = cc_tessellate(fromBin, 0.1);
    CCMesh ma = cc_tessellate(fromAsc, 0.1);
    CC_CHECK(mb.triangleCount == ma.triangleCount);
    cc_mesh_free(mb);
    cc_mesh_free(ma);

    double db[6] = {0}, da[6] = {0};
    CC_CHECK(cc_bounding_box(fromBin, db) == 1);
    CC_CHECK(cc_bounding_box(fromAsc, da) == 1);
    for (int i = 0; i < 6; ++i) CC_CHECK(std::fabs(db[i] - da[i]) < 1e-3);

    cc_shape_release(box);
    cc_shape_release(fromBin);
    cc_shape_release(fromAsc);
}

// 5 ── regression: a BINARY STL whose header starts with "solid " imports as binary
CC_TEST(stl_autodetect_binary_header_starting_solid) {
    // Hand-craft a 1-facet binary STL with a deceptive "solid " header.
    std::vector<unsigned char> b(84 + 50, 0);
    std::memcpy(b.data(), "solid this looks like ascii but is binary", 41);
    b[80] = 1;  // count = 1 (LE)
    // Facet: normal (0,0,0) then a proper non-degenerate triangle.
    auto putf = [](std::vector<unsigned char>& v, std::size_t o, float f) {
        std::uint32_t u;
        std::memcpy(&u, &f, 4);
        v[o] = u & 0xFF;
        v[o + 1] = (u >> 8) & 0xFF;
        v[o + 2] = (u >> 16) & 0xFF;
        v[o + 3] = (u >> 24) & 0xFF;
    };
    const float verts[9] = {0, 0, 0, 10, 0, 0, 0, 10, 0};
    for (int i = 0; i < 9; ++i) putf(b, 84 + 12 + i * 4, verts[i]);

    const std::string path = tmpPath("cc_stl_deceptive.stl");
    {
        std::ofstream f(path, std::ios::binary);
        f.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
    }

    cc_set_engine(1);
    const CCShapeId id = cc_stl_import(path.c_str());
    CC_CHECK(id != 0);  // parsed as binary (size-identity wins over the "solid" text)
    CCMesh m = cc_tessellate(id, 0.1);
    CC_CHECK(m.triangleCount == 1);
    cc_mesh_free(m);
    cc_shape_release(id);
}

// 6 ── malformed / missing input fails cleanly (0 + cc_last_error, no crash) ───────
CC_TEST(stl_import_malformed_reports_error) {
    cc_set_engine(1);
    CC_CHECK(cc_stl_import("/no/such/file/really.stl") == 0);
    CC_CHECK(std::strlen(cc_last_error()) > 0);

    // A tiny garbage file (< 15 bytes) → fail cleanly.
    const std::string path = tmpPath("cc_stl_garbage.stl");
    {
        std::ofstream f(path, std::ios::binary);
        const char junk[] = "garbage";
        f.write(junk, sizeof(junk) - 1);
    }
    CC_CHECK(cc_stl_import(path.c_str()) == 0);
    CC_CHECK(std::strlen(cc_last_error()) > 0);
}

// 7 ── measurement: a closed box has watertight volume; an open soup does not ──────
CC_TEST(stl_import_measurement) {
    cc_set_engine(1);
    const CCShapeId box = buildBox();
    CC_CHECK(box != 0);
    const std::string bp = tmpPath("cc_stl_meas_box.stl");
    CC_CHECK(cc_stl_export(box, bp.c_str(), 0.1, 1) == 1);

    const CCShapeId closed = cc_stl_import(bp.c_str());
    CC_CHECK(closed != 0);
    const CCMassProps mp = cc_mass_properties(closed);
    CC_CHECK(mp.valid == 1);
    CC_CHECK(mp.area > 0.0);
    CC_CHECK(std::fabs(mp.volume - 1000.0) < 1.0);  // 10^3 mm^3

    // An open square (2 triangles, no closure): area valid, not watertight.
    const std::string openPath = tmpPath("cc_stl_open.stl");
    {
        std::ofstream f(openPath, std::ios::binary);
        f << "solid open\n"
          << "  facet normal 0 0 1\n    outer loop\n"
          << "      vertex 0 0 0\n      vertex 10 0 0\n      vertex 10 10 0\n"
          << "    endloop\n  endfacet\n"
          << "  facet normal 0 0 1\n    outer loop\n"
          << "      vertex 0 0 0\n      vertex 10 10 0\n      vertex 0 10 0\n"
          << "    endloop\n  endfacet\n"
          << "endsolid open\n";
    }
    const CCShapeId open = cc_stl_import(openPath.c_str());
    CC_CHECK(open != 0);
    const CCMassProps om = cc_mass_properties(open);
    CC_CHECK(om.area > 0.0);   // ~100 mm^2
    CC_CHECK(om.valid == 0);   // not watertight → volume-if-closed is invalid

    cc_shape_release(box);
    cc_shape_release(closed);
    cc_shape_release(open);
}

// 8 ── cross-tool ASCII with leading '+' signs and a degenerate facet imports,
//       skipping only the zero-area facet (tolerate-and-recover path) ──────────────
CC_TEST(stl_import_tolerates_degenerate_and_plus_signs) {
    cc_set_engine(1);
    const std::string path = tmpPath("cc_stl_degenerate.stl");
    {
        std::ofstream f(path, std::ios::binary);
        // Two good triangles (leading '+' coordinates) + one zero-area facet.
        f << "solid mixed\n"
          << "  facet normal 0 0 1\n    outer loop\n"
          << "      vertex +0.0 +0.0 +0.0\n      vertex +10.0 +0.0 +0.0\n"
          << "      vertex +10.0 +10.0 +0.0\n"
          << "    endloop\n  endfacet\n"
          << "  facet normal 0 0 1\n    outer loop\n"
          << "      vertex +0.0 +0.0 +0.0\n      vertex +10.0 +10.0 +0.0\n"
          << "      vertex +0.0 +10.0 +0.0\n"
          << "    endloop\n  endfacet\n"
          << "  facet normal 0 0 0\n    outer loop\n"  // degenerate: 3 identical verts
          << "      vertex 5 5 0\n      vertex 5 5 0\n      vertex 5 5 0\n"
          << "    endloop\n  endfacet\n"
          << "endsolid mixed\n";
    }
    const CCShapeId id = cc_stl_import(path.c_str());
    CC_CHECK(id != 0);  // imports despite the bad facet
    CCMesh m = cc_tessellate(id, 0.1);
    CC_CHECK(m.triangleCount == 2);  // only the two valid facets survive
    cc_mesh_free(m);
    cc_shape_release(id);
}

CC_RUN_ALL()
