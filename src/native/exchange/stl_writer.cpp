// SPDX-License-Identifier: Apache-2.0
//
// stl_writer.cpp — deterministic ASCII / binary STL serializer (issue #4).
// See stl_writer.h for the contract. OCCT-FREE, clang++ -std=c++20.

#include "native/exchange/stl_writer.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace cybercad::native::exchange {

namespace {

// Fixed 80-byte binary header. MUST NOT begin with "solid" (so a reader never
// mistakes a binary file for ASCII) and carries NO timestamp/host/build-id, so the
// output is byte-identical on repeat.
constexpr char kBinaryHeader[] = "CyberCadKernel binary STL";

// Normalize -0.0f to +0.0f so a signed-zero coordinate hashes/serializes stably.
float to_f32(double d) {
    float f = static_cast<float>(d);
    return f == 0.0f ? 0.0f : f;
}

void put_u16le(std::vector<unsigned char>& b, std::uint16_t v) {
    b.push_back(static_cast<unsigned char>(v & 0xFF));
    b.push_back(static_cast<unsigned char>((v >> 8) & 0xFF));
}

void put_u32le(std::vector<unsigned char>& b, std::uint32_t v) {
    b.push_back(static_cast<unsigned char>(v & 0xFF));
    b.push_back(static_cast<unsigned char>((v >> 8) & 0xFF));
    b.push_back(static_cast<unsigned char>((v >> 16) & 0xFF));
    b.push_back(static_cast<unsigned char>((v >> 24) & 0xFF));
}

void put_f32le(std::vector<unsigned char>& b, float f) {
    put_u32le(b, std::bit_cast<std::uint32_t>(to_f32(f)));
}

// One facet as the three vertex coordinate triples plus its geometric normal
// (already fp64, normalized; zero-length => (0,0,0)).
struct Facet {
    std::array<double, 3> n;
    std::array<double, 3> v0;
    std::array<double, 3> v1;
    std::array<double, 3> v2;
};

std::array<double, 3> facet_normal(const std::array<double, 3>& a, const std::array<double, 3>& b,
                                   const std::array<double, 3>& c) {
    const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
    const double vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
    const double nx = uy * vz - uz * vy;
    const double ny = uz * vx - ux * vz;
    const double nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-12) return {0.0, 0.0, 0.0};  // degenerate facet: legal (0,0,0)
    return {nx / len, ny / len, nz / len};
}

// Build the emitted facet list (skipping any triangle with an out-of-range index),
// so both writers share one deterministic ordering.
std::vector<Facet> collect_facets(const std::vector<double>& vertices,
                                  const std::vector<int>& triangles) {
    const std::size_t vertCount = vertices.size() / 3;
    const std::size_t triCount = triangles.size() / 3;
    std::vector<Facet> facets;
    facets.reserve(triCount);
    auto at = [&](int idx) -> std::array<double, 3> {
        const std::size_t o = static_cast<std::size_t>(idx) * 3;
        return {vertices[o], vertices[o + 1], vertices[o + 2]};
    };
    for (std::size_t t = 0; t < triCount; ++t) {
        const int i = triangles[t * 3], j = triangles[t * 3 + 1], k = triangles[t * 3 + 2];
        if (i < 0 || j < 0 || k < 0) continue;
        if (static_cast<std::size_t>(i) >= vertCount || static_cast<std::size_t>(j) >= vertCount ||
            static_cast<std::size_t>(k) >= vertCount)
            continue;
        const auto v0 = at(i), v1 = at(j), v2 = at(k);
        facets.push_back(Facet{facet_normal(v0, v1, v2), v0, v1, v2});
    }
    return facets;
}

// Append a float32-rounded value in locale-independent scientific notation with 6
// fractional digits (e.g. "1.000000e+01"), so ASCII and binary of the same body
// agree within float32 epsilon and no locale can emit a decimal comma.
void append_ascii_float(std::string& s, double value) {
    char buf[32];
    const float f = to_f32(value);
    // std::to_chars floating-point overloads are unavailable on lower iOS deployment
    // targets, so format with snprintf (equivalent "1.000000e+01" shape for a float32)
    // and normalise any locale decimal comma to '.' — the file must stay locale-free.
    const int n = std::snprintf(buf, sizeof(buf), "%.6e", static_cast<double>(f));
    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(buf)) return;
    for (int x = 0; x < n; ++x)
        if (buf[x] == ',') buf[x] = '.';
    s.append(buf, buf + n);
}

void append_vertex(std::string& s, const char* tag, const std::array<double, 3>& p) {
    s += tag;
    append_ascii_float(s, p[0]);
    s += ' ';
    append_ascii_float(s, p[1]);
    s += ' ';
    append_ascii_float(s, p[2]);
    s += '\n';
}

bool write_all(const std::string& path, const void* data, std::size_t size) {
    if (path.empty()) return false;
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    if (size > 0) out.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
    return static_cast<bool>(out);
}

bool write_binary(const std::vector<Facet>& facets, const std::string& path) {
    std::vector<unsigned char> bytes;
    bytes.reserve(84 + 50 * facets.size());
    bytes.resize(80, 0);  // zero-padded header
    std::memcpy(bytes.data(), kBinaryHeader, std::strlen(kBinaryHeader));
    put_u32le(bytes, static_cast<std::uint32_t>(facets.size()));
    for (const Facet& f : facets) {
        for (int c = 0; c < 3; ++c) put_f32le(bytes, static_cast<float>(f.n[c]));
        for (int c = 0; c < 3; ++c) put_f32le(bytes, static_cast<float>(f.v0[c]));
        for (int c = 0; c < 3; ++c) put_f32le(bytes, static_cast<float>(f.v1[c]));
        for (int c = 0; c < 3; ++c) put_f32le(bytes, static_cast<float>(f.v2[c]));
        put_u16le(bytes, 0);  // attribute byte count
    }
    return write_all(path, bytes.data(), bytes.size());
}

bool write_ascii(const std::vector<Facet>& facets, const std::string& path) {
    std::string s;
    s.reserve(facets.size() * 220 + 32);
    s += "solid CyberCadKernel\n";
    for (const Facet& f : facets) {
        s += "  facet normal ";
        append_ascii_float(s, f.n[0]);
        s += ' ';
        append_ascii_float(s, f.n[1]);
        s += ' ';
        append_ascii_float(s, f.n[2]);
        s += '\n';
        s += "    outer loop\n";
        append_vertex(s, "      vertex ", f.v0);
        append_vertex(s, "      vertex ", f.v1);
        append_vertex(s, "      vertex ", f.v2);
        s += "    endloop\n";
        s += "  endfacet\n";
    }
    s += "endsolid CyberCadKernel\n";
    return write_all(path, s.data(), s.size());
}

}  // namespace

bool stl_export_mesh(const std::vector<double>& vertices, const std::vector<int>& triangles,
                     const std::string& path, bool binary) {
    const std::vector<Facet> facets = collect_facets(vertices, triangles);
    return binary ? write_binary(facets, path) : write_ascii(facets, path);
}

}  // namespace cybercad::native::exchange
