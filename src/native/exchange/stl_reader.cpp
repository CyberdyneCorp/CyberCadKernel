// SPDX-License-Identifier: Apache-2.0
//
// stl_reader.cpp — auto-detecting ASCII/binary STL parser with vertex welding and
// degenerate-facet tolerance (issue #5). See stl_reader.h for the contract.
// OCCT-FREE, clang++ -std=c++20.

#include "native/exchange/stl_reader.h"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace cybercad::native::exchange {

namespace {

namespace math = cybercad::native::math;

enum class Format { Ascii, Binary };

std::uint32_t read_u32le(const std::vector<unsigned char>& b, std::size_t off) {
    return static_cast<std::uint32_t>(b[off]) | (static_cast<std::uint32_t>(b[off + 1]) << 8) |
           (static_cast<std::uint32_t>(b[off + 2]) << 16) |
           (static_cast<std::uint32_t>(b[off + 3]) << 24);
}

float read_f32le(const std::vector<unsigned char>& b, std::size_t off) {
    return std::bit_cast<float>(read_u32le(b, off));
}

// Read the whole file to a byte buffer. Sets `err` and returns nullopt on failure.
std::optional<std::vector<unsigned char>> read_file(const std::string& path, std::string& err) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        err = "cannot open file: " + path;
        return std::nullopt;
    }
    const std::streamsize size = f.tellg();
    if (size < 0) {
        err = "cannot determine file size: " + path;
        return std::nullopt;
    }
    std::vector<unsigned char> buf(static_cast<std::size_t>(size));
    f.seekg(0);
    if (size > 0 && !f.read(reinterpret_cast<char*>(buf.data()), size)) {
        err = "failed to read file: " + path;
        return std::nullopt;
    }
    return buf;
}

bool ascii_signature(const std::vector<unsigned char>& b) {
    std::string_view sv(reinterpret_cast<const char*>(b.data()), b.size());
    std::size_t i = 0;
    while (i < sv.size() && (sv[i] == ' ' || sv[i] == '\t' || sv[i] == '\n' || sv[i] == '\r'))
        ++i;
    const std::string_view rest = sv.substr(i);
    // A binary header can also begin "solid ...", but detect_format runs the
    // decisive size-identity and non-text-byte checks FIRST; by the time we get
    // here the head is all text. A "solid" lead followed by either a "facet" or an
    // "endsolid" keyword is a real ASCII STL — the "endsolid" alternative catches a
    // well-formed ZERO-facet solid, which carries no "facet" but still closes with
    // "endsolid" and must not be misread as (a too-small) binary file.
    if (rest.rfind("solid", 0) != 0) return false;
    return rest.find("facet") != std::string_view::npos ||
           rest.find("endsolid") != std::string_view::npos;
}

// Auto-detect the format. First decisive check wins: a byte-exact binary size
// wins even when the 80-byte header starts with "solid"; else a non-text byte in
// the head implies binary; else an ASCII "solid ... facet" signature; else binary.
Format detect_format(const std::vector<unsigned char>& b) {
    const std::size_t S = b.size();
    if (S >= 84) {
        const std::uint32_t claimed = read_u32le(b, 80);
        if (claimed <= (S - 84) / 50 + 1 &&
            84 + 50 * static_cast<std::uint64_t>(claimed) == static_cast<std::uint64_t>(S))
            return Format::Binary;
    }
    const std::size_t n = S < 512 ? S : 512;
    for (std::size_t i = 0; i < n; ++i) {
        const unsigned char c = b[i];
        if (c == 0x00 || c < 0x09 || c > 0x7E) return Format::Binary;  // non-text byte
    }
    return ascii_signature(b) ? Format::Ascii : Format::Binary;
}

// Parse a binary STL into raw triangle-soup vertices (3 per triangle). Bounds-checked.
bool parse_binary(const std::vector<unsigned char>& b, std::vector<math::Point3>& out,
                  std::string& err) {
    if (b.size() < 84) {
        err = "binary STL too small for header";
        return false;
    }
    const std::uint32_t n = read_u32le(b, 80);
    if (84 + 50 * static_cast<std::uint64_t>(n) > static_cast<std::uint64_t>(b.size())) {
        err = "binary STL facet count exceeds file size";
        return false;
    }
    out.reserve(static_cast<std::size_t>(n) * 3);
    std::size_t off = 84;
    for (std::uint32_t f = 0; f < n; ++f) {
        std::size_t vp = off + 12;  // skip the 12-byte normal
        for (int v = 0; v < 3; ++v) {
            out.push_back({read_f32le(b, vp), read_f32le(b, vp + 4), read_f32le(b, vp + 8)});
            vp += 12;
        }
        off += 50;  // skip the trailing uint16 attribute
    }
    return true;
}

bool is_ws(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v'; }

// Whitespace tokenizer over the file bytes.
struct Tokenizer {
    std::string_view s;
    std::size_t i = 0;
    bool next(std::string_view& out) {
        while (i < s.size() && is_ws(s[i])) ++i;
        if (i >= s.size()) return false;
        const std::size_t start = i;
        while (i < s.size() && !is_ws(s[i])) ++i;
        out = s.substr(start, i - start);
        return true;
    }
};

bool parse_double(std::string_view tok, double& out) {
    // std::from_chars deliberately rejects a leading '+' on the mantissa; strip it so
    // that cross-tool ASCII STLs writing "+1.5" style coordinates still import.
    if (!tok.empty() && tok.front() == '+') tok.remove_prefix(1);
    const auto res = std::from_chars(tok.data(), tok.data() + tok.size(), out);
    return res.ec == std::errc() && res.ptr == tok.data() + tok.size();
}

// Parse an ASCII STL: every "vertex x y z" keyword contributes one soup vertex.
bool parse_ascii(const std::vector<unsigned char>& b, std::vector<math::Point3>& out,
                 std::string& err) {
    Tokenizer tk{std::string_view(reinterpret_cast<const char*>(b.data()), b.size())};
    std::string_view tok;
    while (tk.next(tok)) {
        if (tok != "vertex") continue;
        std::array<double, 3> xyz{};
        for (int k = 0; k < 3; ++k) {
            std::string_view num;
            if (!tk.next(num) || !parse_double(num, xyz[k])) {
                err = "ASCII STL: malformed vertex coordinate";
                return false;
            }
        }
        out.push_back({xyz[0], xyz[1], xyz[2]});
    }
    if (out.empty()) {
        err = "ASCII STL: no vertices found";
        return false;
    }
    return true;
}

// Weld grid key: quantized integer coordinates.
struct WeldKey {
    std::int64_t x, y, z;
    bool operator==(const WeldKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
};
struct WeldKeyHash {
    std::size_t operator()(const WeldKey& k) const noexcept {
        // Unsigned multiply avoids signed-overflow UB for large quantized coordinates.
        const std::uint64_t hx = static_cast<std::uint64_t>(k.x) * 73856093u;
        const std::uint64_t hy = static_cast<std::uint64_t>(k.y) * 19349663u;
        const std::uint64_t hz = static_cast<std::uint64_t>(k.z) * 83492791u;
        return static_cast<std::size_t>(hx ^ hy ^ hz);
    }
};

// Build a welded mesh from the raw triangle soup. Coincident vertices merge onto a
// tolerance grid; degenerate (repeated-index or zero-area) triangles are skipped.
std::optional<ntess::Mesh> weld_mesh(const std::vector<math::Point3>& raw, double weldTol,
                                     std::string& err) {
    if (raw.size() < 3 || raw.size() % 3 != 0) {
        err = "STL has no complete triangles";
        return std::nullopt;
    }
    const double tol = weldTol > 0.0 ? weldTol : 1e-6;
    ntess::Mesh mesh;
    std::unordered_map<WeldKey, std::vector<std::uint32_t>, WeldKeyHash> index;
    // Weld onto a tolerance grid, but SEARCH the 3×3×3 neighbourhood of cells: two
    // coincident coordinates can round into ADJACENT cells (a vertex straddling a
    // cell boundary), and a single-cell lookup would leave them unmerged — foreign
    // STLs whose shared vertices carry sub-tolerance jitter would then under-weld.
    // With a cell side of `tol`, any vertex within `tol` of `p` is guaranteed to
    // lie in one of the 27 neighbour cells, so we find it and compare true distance.
    auto weld = [&](const math::Point3& p) -> std::uint32_t {
        const std::int64_t cx = std::llround(p.x / tol);
        const std::int64_t cy = std::llround(p.y / tol);
        const std::int64_t cz = std::llround(p.z / tol);
        for (std::int64_t dx = -1; dx <= 1; ++dx)
            for (std::int64_t dy = -1; dy <= 1; ++dy)
                for (std::int64_t dz = -1; dz <= 1; ++dz) {
                    const auto it = index.find(WeldKey{cx + dx, cy + dy, cz + dz});
                    if (it == index.end()) continue;
                    for (const std::uint32_t id : it->second)
                        if (math::norm(mesh.vertices[id] - p) <= tol) return id;
                }
        const std::uint32_t id = mesh.addVertex(p);
        index[WeldKey{cx, cy, cz}].push_back(id);
        return id;
    };
    for (std::size_t t = 0; t + 2 < raw.size(); t += 3) {
        const std::uint32_t a = weld(raw[t]);
        const std::uint32_t b = weld(raw[t + 1]);
        const std::uint32_t c = weld(raw[t + 2]);
        if (a == b || b == c || a == c) continue;  // collapsed on the weld grid
        const math::Vec3 ab = mesh.vertices[b] - mesh.vertices[a];
        const math::Vec3 ac = mesh.vertices[c] - mesh.vertices[a];
        if (0.5 * math::norm(math::cross(ab, ac)) < 1e-14) continue;  // zero-area facet
        mesh.addTriangle(a, b, c);
    }
    if (mesh.triangles.empty()) {
        err = "STL has no valid (non-degenerate) triangles after welding";
        return std::nullopt;
    }
    return mesh;
}

}  // namespace

std::optional<ntess::Mesh> stl_read(const std::string& path, std::string& err, double weldTol) {
    auto bytes = read_file(path, err);
    if (!bytes) return std::nullopt;
    if (bytes->size() < 15) {
        err = "STL file too small to be valid";
        return std::nullopt;
    }
    const Format fmt = detect_format(*bytes);
    std::vector<math::Point3> raw;
    const bool parsed = fmt == Format::Binary ? parse_binary(*bytes, raw, err)
                                              : parse_ascii(*bytes, raw, err);
    if (!parsed) return std::nullopt;
    return weld_mesh(raw, weldTol, err);
}

}  // namespace cybercad::native::exchange
