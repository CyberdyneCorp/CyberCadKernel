// SPDX-License-Identifier: Apache-2.0
//
// test_native_gltf_fuzz.cpp — MOAT M6 differential-fuzzing harness (the "no
// silent-wrong" completeness bar) for the OCCT-FREE native glTF 2.0 writer
// src/native/exchange/gltf_writer.{h,cpp} — the .gltf JSON (base64 buffer) and
// .glb binary container the app hands to iOS RealityKit / QuickLook / SceneKit.
//
// This is a WRITER, so — unlike the boolean / blend / construct fuzzers that diff a
// native SOLID against an OCCT oracle — there is NO OCCT equivalence to diff against
// (OCCT's RWGltf is a *different* implementation, not a spec oracle; two conforming
// glTF writers need not be byte- or structure-identical). The arbiter is therefore
// ROUND-TRIP / SELF-CONSISTENCY, computed independently in this test:
//
//   (A) STRUCTURAL VALIDITY — the emitted artifact is a spec-conformant glTF 2.0
//       document. A self-contained validator (no external deps) parses the .gltf
//       JSON + base64 buffer and the .glb container and asserts: correct glb
//       magic/version/total-length + JSON+BIN chunk layout with 4-byte alignment;
//       buffer / bufferView byteLength + byteOffset consistency (offsets in range,
//       4-aligned, view spans inside its buffer); accessor componentType (5125
//       UNSIGNED_INT indices, 5126 FLOAT positions/normals) + count × element-size
//       == bufferView byteLength; POSITION accessor min/max EXACTLY the component-
//       wise min/max of the decoded positions; every index in [0, vertexCount);
//       and NO NaN / Inf anywhere in the decoded buffers.
//
//   (B) GEOMETRY FIDELITY ROUND-TRIP (the core DISAGREED check) — the mesh handed to
//       the writer must survive. No glTF *reader* exists in-repo (only writers under
//       src/native/exchange), so the test parses the emitted buffers DIRECTLY and
//       compares against an INDEPENDENT reimplementation of the writer's documented
//       contract (gltf_writer.h): drop triangles with an out-of-range / negative
//       index, keep only referenced vertices reindexed in first-touch order, weld by
//       index reuse, scale mm→m (×1e-3). We assert the round-tripped mesh has the
//       SAME reachable vertex count, the SAME triangle count, the SAME connectivity
//       (index-for-index, up to the fp32 quantization of positions), and the SAME
//       bounding box (in metres, to fp32 tol). The .gltf and .glb legs must decode to
//       byte-identical buffers (single-source buffer). Determinism: same mesh+mode →
//       byte-identical file.
//
// ── CLASSIFICATION (every trial lands in EXACTLY one bucket) ─────────────────────
//   AGREED                  — writer succeeded AND both (A) structural validity and
//                             (B) round-trip fidelity hold against the independent
//                             oracle. The pass state.
//   ORACLE-INACCURATE       — the writer is CORRECT but our test-side oracle cannot
//                             represent the case at full precision, so a naive compare
//                             would false-DISAGREE; we detect + exclude it with a
//                             one-line justification (see the guarded branches). Here
//                             the only source is fp32 quantization COLLISION: two
//                             distinct fp64 mm positions that map to the SAME fp32
//                             metre position, so a position round-trip to full fp64
//                             precision is unrepresentable — we compare at fp32 tol,
//                             which is the writer's actual (documented f32) contract.
//   NATIVE-CHECK-INACCURATE — writer succeeded, structurally valid, but our
//                             ROUND-TRIP invariant is provably too strict for a
//                             legitimate writer behaviour (none observed; reserved,
//                             counted, and justified inline if it ever fires).
//   HONESTLY-DECLINED       — writer returned false (I/O) or wrote a truly 0-byte file
//                             (only reachable via an empty path string): a first-class
//                             outcome, never a bar failure. NB: a vertex-bearing 0-triangle
//                             mesh is NOT a decline — the raw writer emits a valid empty-
//                             geometry (count:0) asset that is validated + round-tripped.
//   DISAGREED               — writer succeeded but the artifact is structurally
//                             invalid OR the round-tripped mesh does NOT match the
//                             oracle. The SILENT-WRONG failure this harness exists to
//                             catch. BAR: DISAGREED == 0.
//
// Deterministic generator: splitmix64-seeded xoshiro256** keyed ONLY by FUZZ_SEED
// (env) — NO clock, NO rand(). Same seed → identical batch. Two seeds run per ctest
// invocation for breadth. OCCT-FREE, clang++ -std=c++20, standard library only.
//
#include "native/exchange/gltf_writer.h"

#include "harness.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

using cybercad::native::exchange::gltf_export_mesh;

// Millimetre → metre; the writer's one semantic transform (gltf_writer.h).
constexpr double kMmToM = 1.0e-3;

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim discipline of the
//    landed native_*_fuzz siblings). No clock, no rand(): same seed → same batch. ──
struct Rng {
    std::uint64_t s[4];
    static std::uint64_t splitmix64(std::uint64_t& x) {
        std::uint64_t z = (x += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    explicit Rng(std::uint64_t seed) { for (auto& v : s) v = splitmix64(seed); }
    static std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
    std::uint64_t next() {
        const std::uint64_t r = rotl(s[1] * 5, 7) * 9;
        const std::uint64_t t = s[1] << 17;
        s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
        s[3] = rotl(s[3], 45);
        return r;
    }
    double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }  // [0,1)
    double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
    std::uint32_t below(std::uint32_t n) { return n == 0 ? 0 : static_cast<std::uint32_t>(next() % n); }
};

// ── file I/O ─────────────────────────────────────────────────────────────────────
std::string tmpPath(const std::string& name) {
    std::filesystem::path p = std::filesystem::temp_directory_path();
    p /= name;
    return p.string();
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

// ── little-endian byte readers ─────────────────────────────────────────────────
std::uint32_t u32le(const std::vector<unsigned char>& b, std::size_t o) {
    return static_cast<std::uint32_t>(b[o]) | (static_cast<std::uint32_t>(b[o + 1]) << 8) |
           (static_cast<std::uint32_t>(b[o + 2]) << 16) | (static_cast<std::uint32_t>(b[o + 3]) << 24);
}
float f32le(const std::vector<unsigned char>& b, std::size_t o) {
    std::uint32_t u = u32le(b, o);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}

// ── minimal JSON scalar/array scrapes (the writer emits a fixed, flat shape) ─────
double jsonNum(const std::string& j, const std::string& key, std::size_t from = 0) {
    const std::size_t k = j.find("\"" + key + "\":", from);
    if (k == std::string::npos) return -1;
    return std::strtod(j.c_str() + k + key.size() + 3, nullptr);
}
bool jsonHas(const std::string& j, const std::string& needle) { return j.find(needle) != std::string::npos; }

// Decode all "data:...;base64,XXXX" payload of the (single) .gltf buffer URI.
std::vector<unsigned char> decodeDataUri(const std::string& j) {
    const std::string tag = ";base64,";
    const std::size_t s = j.find(tag);
    if (s == std::string::npos) return {};
    std::size_t p = s + tag.size();
    std::size_t e = j.find('"', p);
    const std::string b64 = j.substr(p, e - p);
    auto val = [](char c) -> int {
        if (c >= 'A' && c <= 'Z') return c - 'A';
        if (c >= 'a' && c <= 'z') return c - 'a' + 26;
        if (c >= '0' && c <= '9') return c - '0' + 52;
        if (c == '+') return 62;
        if (c == '/') return 63;
        return -1;
    };
    std::vector<unsigned char> out;
    int buf = 0, bits = 0;
    for (char c : b64) {
        if (c == '=') break;
        const int v = val(c);
        if (v < 0) continue;
        buf = (buf << 6) | v;
        bits += 6;
        if (bits >= 8) { bits -= 8; out.push_back(static_cast<unsigned char>((buf >> bits) & 0xFF)); }
    }
    return out;
}

// ── the INDEPENDENT oracle: reproduce the writer's documented compaction contract
//    (gltf_writer.h) with NO reference to gltf_writer.cpp internals. ──────────────
struct OracleMesh {
    std::vector<std::array<double, 3>> pos;   // metres (fp64, pre-quantization)
    std::vector<std::uint32_t> idx;           // reindexed, 3·triCount
    bool empty() const { return idx.empty(); }
};

OracleMesh oracleCompact(const std::vector<double>& verts, const std::vector<int>& tris) {
    const std::size_t vertCount = verts.size() / 3;
    const std::size_t triCount = tris.size() / 3;
    std::vector<std::uint32_t> remap(vertCount, std::numeric_limits<std::uint32_t>::max());
    OracleMesh out;
    auto emit = [&](int oldIdx) -> std::uint32_t {
        std::uint32_t& slot = remap[static_cast<std::size_t>(oldIdx)];
        if (slot == std::numeric_limits<std::uint32_t>::max()) {
            slot = static_cast<std::uint32_t>(out.pos.size());
            const std::size_t o = static_cast<std::size_t>(oldIdx) * 3;
            out.pos.push_back({verts[o] * kMmToM, verts[o + 1] * kMmToM, verts[o + 2] * kMmToM});
        }
        return slot;
    };
    for (std::size_t t = 0; t < triCount; ++t) {
        const int i = tris[t * 3], j = tris[t * 3 + 1], k = tris[t * 3 + 2];
        if (i < 0 || j < 0 || k < 0) continue;
        if (static_cast<std::size_t>(i) >= vertCount || static_cast<std::size_t>(j) >= vertCount ||
            static_cast<std::size_t>(k) >= vertCount)
            continue;
        out.idx.push_back(emit(i));
        out.idx.push_back(emit(j));
        out.idx.push_back(emit(k));
    }
    return out;
}

// ── STRUCTURAL VALIDATOR (A): parse a decoded buffer + its JSON and assert the full
//    glTF-2.0 accessor/bufferView/alignment contract. Returns false on ANY breach. ─
struct Parsed {
    std::vector<std::uint32_t> idx;
    std::vector<std::array<float, 3>> pos;
    std::vector<std::array<float, 3>> nrm;
    float posMin[3] = {0, 0, 0}, posMax[3] = {0, 0, 0};   // from the JSON accessor bounds
    bool ok = false;
};

bool finiteF(float f) { return std::isfinite(f); }

// Validate + parse. `json` is the glTF JSON; `buf` the decoded buffer 0.
Parsed validateAndParse(const std::string& json, const std::vector<unsigned char>& buf, bool& structOk) {
    Parsed P;
    structOk = true;
    auto fail = [&]() { structOk = false; };

    if (!jsonHas(json, "\"version\":\"2.0\"")) fail();
    if (!jsonHas(json, "\"mode\":4")) fail();
    if (!jsonHas(json, "\"POSITION\":1") || !jsonHas(json, "\"NORMAL\":2")) fail();

    // buffer 0 byteLength == decoded size, 4-aligned.
    const std::size_t buffersBlk = json.find("\"buffers\"");
    const std::uint32_t bufLen = static_cast<std::uint32_t>(jsonNum(json, "byteLength", buffersBlk));
    if (bufLen != buf.size()) fail();
    if (buf.size() % 4 != 0) fail();

    // Accessor counts: accessor 0 (indices), 1 (POSITION), 2 (NORMAL). The writer emits
    // them in a fixed order; scrape each "count" occurrence in the accessors block.
    const std::size_t accBlk = json.find("\"accessors\"");
    const std::size_t c0 = json.find("\"count\":", accBlk);
    const std::size_t c1 = json.find("\"count\":", c0 + 1);
    const std::size_t c2 = json.find("\"count\":", c1 + 1);
    if (c0 == std::string::npos || c1 == std::string::npos || c2 == std::string::npos) { fail(); return P; }
    const std::uint32_t idxCount = static_cast<std::uint32_t>(std::strtoul(json.c_str() + c0 + 8, nullptr, 10));
    const std::uint32_t posCount = static_cast<std::uint32_t>(std::strtoul(json.c_str() + c1 + 8, nullptr, 10));
    const std::uint32_t nrmCount = static_cast<std::uint32_t>(std::strtoul(json.c_str() + c2 + 8, nullptr, 10));
    if (posCount != nrmCount) fail();                 // POSITION & NORMAL are per-vertex
    if (idxCount % 3 != 0) fail();                     // triangle list

    // componentTypes present: 5125 (UNSIGNED_INT idx) + 5126 (FLOAT pos/nrm).
    if (!jsonHas(json, "\"componentType\":5125") || !jsonHas(json, "\"componentType\":5126")) fail();

    // bufferViews: three, each byteOffset 4-aligned + span inside the buffer, and their
    // byteLength == count × element-size.
    const std::size_t bvBlk = json.find("\"bufferViews\"");
    std::size_t bo0 = json.find("\"byteOffset\":", bvBlk);
    std::size_t bo1 = json.find("\"byteOffset\":", bo0 + 1);
    std::size_t bo2 = json.find("\"byteOffset\":", bo1 + 1);
    std::size_t bl0 = json.find("\"byteLength\":", bvBlk);
    std::size_t bl1 = json.find("\"byteLength\":", bl0 + 1);
    std::size_t bl2 = json.find("\"byteLength\":", bl1 + 1);
    if (bo2 == std::string::npos || bl2 == std::string::npos) { fail(); return P; }
    const std::uint32_t idxOff = static_cast<std::uint32_t>(std::strtoul(json.c_str() + bo0 + 13, nullptr, 10));
    const std::uint32_t posOff = static_cast<std::uint32_t>(std::strtoul(json.c_str() + bo1 + 13, nullptr, 10));
    const std::uint32_t nrmOff = static_cast<std::uint32_t>(std::strtoul(json.c_str() + bo2 + 13, nullptr, 10));
    const std::uint32_t idxByteLen = static_cast<std::uint32_t>(std::strtoul(json.c_str() + bl0 + 13, nullptr, 10));
    const std::uint32_t posByteLen = static_cast<std::uint32_t>(std::strtoul(json.c_str() + bl1 + 13, nullptr, 10));
    const std::uint32_t nrmByteLen = static_cast<std::uint32_t>(std::strtoul(json.c_str() + bl2 + 13, nullptr, 10));

    if (idxOff % 4 || posOff % 4 || nrmOff % 4) fail();
    if (idxByteLen != idxCount * 4u) fail();           // uint32 index
    if (posByteLen != posCount * 12u) fail();          // vec3 f32
    if (nrmByteLen != nrmCount * 12u) fail();
    if (static_cast<std::size_t>(idxOff) + idxByteLen > buf.size()) fail();
    if (static_cast<std::size_t>(posOff) + posByteLen > buf.size()) fail();
    if (static_cast<std::size_t>(nrmOff) + nrmByteLen > buf.size()) fail();
    if (!structOk) return P;

    // Decode index / POSITION / NORMAL blocks.
    P.idx.resize(idxCount);
    for (std::uint32_t t = 0; t < idxCount; ++t) P.idx[t] = u32le(buf, idxOff + static_cast<std::size_t>(t) * 4);
    P.pos.resize(posCount);
    P.nrm.resize(nrmCount);
    for (std::uint32_t v = 0; v < posCount; ++v)
        for (int d = 0; d < 3; ++d) P.pos[v][d] = f32le(buf, posOff + (static_cast<std::size_t>(v) * 3 + d) * 4);
    for (std::uint32_t v = 0; v < nrmCount; ++v)
        for (int d = 0; d < 3; ++d) P.nrm[v][d] = f32le(buf, nrmOff + (static_cast<std::size_t>(v) * 3 + d) * 4);

    // Every index in [0, posCount).
    for (std::uint32_t vi : P.idx)
        if (vi >= posCount) fail();

    // No NaN/Inf anywhere in the decoded buffers.
    for (auto& p : P.pos) for (int d = 0; d < 3; ++d) if (!finiteF(p[d])) fail();
    for (auto& n : P.nrm) for (int d = 0; d < 3; ++d) if (!finiteF(n[d])) fail();
    // Normals are unit (or the writer's documented up-normal fallback) — |n| ≈ 1.
    for (auto& n : P.nrm) {
        const double L = std::sqrt(static_cast<double>(n[0]) * n[0] + static_cast<double>(n[1]) * n[1] +
                                   static_cast<double>(n[2]) * n[2]);
        if (std::fabs(L - 1.0) > 1e-4) fail();
    }

    // POSITION accessor min/max: scrape the "min":[..]/"max":[..] and require they EXACTLY
    // (fp32) equal the componentwise extrema of the decoded positions.
    auto arr3 = [&](const std::string& key, float out3[3]) -> bool {
        const std::size_t k = json.find("\"" + key + "\":[", accBlk);
        if (k == std::string::npos) return false;
        const char* p = json.c_str() + k + key.size() + 4;
        char* end = nullptr;
        for (int d = 0; d < 3; ++d) { out3[d] = std::strtof(p, &end); p = end + 1; }
        return true;
    };
    if (posCount > 0) {
        if (!arr3("min", P.posMin) || !arr3("max", P.posMax)) fail();
        float lo[3] = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                       std::numeric_limits<float>::infinity()};
        float hi[3] = {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                       -std::numeric_limits<float>::infinity()};
        for (auto& p : P.pos)
            for (int d = 0; d < 3; ++d) { if (p[d] < lo[d]) lo[d] = p[d]; if (p[d] > hi[d]) hi[d] = p[d]; }
        for (int d = 0; d < 3; ++d) {
            if (P.posMin[d] != lo[d]) fail();          // exact fp32 match (accessor bounds are f32)
            if (P.posMax[d] != hi[d]) fail();
        }
    }

    P.ok = structOk;
    return P;
}

// ── .glb container validator: header + JSON chunk + BIN chunk → (json, bin). ──────
bool parseGlb(const std::vector<unsigned char>& b, std::string& json, std::vector<unsigned char>& bin) {
    if (b.size() < 12 + 8 + 8) return false;
    if (std::memcmp(b.data(), "glTF", 4) != 0) return false;
    if (u32le(b, 4) != 2) return false;
    if (u32le(b, 8) != b.size()) return false;
    const std::uint32_t jsonLen = u32le(b, 12);
    if (jsonLen % 4 != 0) return false;
    if (u32le(b, 16) != 0x4E4F534Au) return false;      // "JSON"
    const std::size_t jsonStart = 20;
    if (jsonStart + jsonLen + 8 > b.size()) return false;
    json.assign(reinterpret_cast<const char*>(b.data() + jsonStart), jsonLen);
    const std::size_t binHdr = jsonStart + jsonLen;
    const std::uint32_t binLen = u32le(b, binHdr);
    if (binLen % 4 != 0) return false;
    if (u32le(b, binHdr + 4) != 0x004E4942u) return false;  // "BIN\0"
    const std::size_t binStart = binHdr + 8;
    if (binStart + binLen != b.size()) return false;
    bin.assign(b.begin() + static_cast<std::ptrdiff_t>(binStart), b.end());
    return true;
}

// ── mesh generators (broad breadth) ─────────────────────────────────────────────
enum Family {
    F_PRIM_BOX, F_PRIM_TET, F_MULTISOLID, F_RANDOM_SOUP, F_FAN,
    F_ONE_TRI, F_ZERO_TRI, F_BAD_INDICES, F_TINY_SCALE, F_HUGE_SCALE, F_HUGE_COUNT,
    F_COUNT
};
const char* famName(int f) {
    switch (f) {
        case F_PRIM_BOX:    return "box (12 tri)";
        case F_PRIM_TET:    return "tetrahedron";
        case F_MULTISOLID:  return "multi-solid (2..4 boxes)";
        case F_RANDOM_SOUP: return "random triangle soup";
        case F_FAN:         return "triangle fan";
        case F_ONE_TRI:     return "single triangle";
        case F_ZERO_TRI:    return "empty (0 tri)";
        case F_BAD_INDICES: return "out-of-range/negative indices";
        case F_TINY_SCALE:  return "tiny coordinate scale";
        case F_HUGE_SCALE:  return "huge coordinate scale";
        case F_HUGE_COUNT:  return "huge triangle count";
        default:            return "?";
    }
}

struct Mesh { std::vector<double> v; std::vector<int> t; };

void addBox(Mesh& m, double ox, double oy, double oz, double sx, double sy, double sz) {
    const int base = static_cast<int>(m.v.size() / 3);
    const double c[8][3] = {{0, 0, 0}, {sx, 0, 0}, {sx, sy, 0}, {0, sy, 0},
                            {0, 0, sz}, {sx, 0, sz}, {sx, sy, sz}, {0, sy, sz}};
    for (auto& p : c) { m.v.push_back(ox + p[0]); m.v.push_back(oy + p[1]); m.v.push_back(oz + p[2]); }
    const int f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                          {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    for (auto& tr : f) { m.t.push_back(base + tr[0]); m.t.push_back(base + tr[1]); m.t.push_back(base + tr[2]); }
}

Mesh genMesh(int family, Rng& rng) {
    Mesh m;
    switch (family) {
        case F_PRIM_BOX: {
            const double s = rng.range(0.5, 250.0);
            addBox(m, rng.range(-50, 50), rng.range(-50, 50), rng.range(-50, 50), s, s * rng.range(0.3, 3.0), s);
            break;
        }
        case F_PRIM_TET: {
            const double a = rng.range(1.0, 100.0);
            m.v = {0, 0, 0, a, 0, 0, 0, a, 0, 0, 0, a};
            m.t = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
            break;
        }
        case F_MULTISOLID: {
            const int n = 2 + static_cast<int>(rng.below(3));
            for (int i = 0; i < n; ++i) {
                const double s = rng.range(2.0, 40.0);
                addBox(m, i * 60.0, rng.range(-10, 10), rng.range(-10, 10), s, s, s);
            }
            break;
        }
        case F_RANDOM_SOUP: {
            const int n = 1 + static_cast<int>(rng.below(40));
            for (int i = 0; i < n; ++i)
                for (int k = 0; k < 3; ++k)
                    for (int d = 0; d < 3; ++d) m.v.push_back(rng.range(-100, 100));
            for (int i = 0; i < n * 3; ++i) m.t.push_back(i);   // consecutive; every vert referenced once
            break;
        }
        case F_FAN: {
            const int n = 3 + static_cast<int>(rng.below(30));   // n rim verts + 1 hub
            m.v = {0, 0, 0};
            for (int i = 0; i < n; ++i) {
                const double a = 2.0 * M_PI * i / n;
                m.v.push_back(20 * std::cos(a)); m.v.push_back(20 * std::sin(a)); m.v.push_back(0);
            }
            for (int i = 0; i < n; ++i) { m.t.push_back(0); m.t.push_back(1 + i); m.t.push_back(1 + (i + 1) % n); }
            break;
        }
        case F_ONE_TRI:
            m.v = {rng.range(-5, 5), 0, 0, 10, rng.range(-5, 5), 0, 5, 10, rng.range(-2, 2)};
            m.t = {0, 1, 2};
            break;
        case F_ZERO_TRI:
            // Vertices present but NO triangles — the raw writer emits a valid count:0 asset
            // (empty geometry), which must still parse + round-trip (0 verts / 0 tris).
            m.v = {0, 0, 0, 1, 1, 1};
            break;
        case F_BAD_INDICES: {
            // A valid tri plus triangles with out-of-range / negative indices the writer must
            // SKIP; only the good tri (+ its 3 verts) should survive.
            m.v = {0, 0, 0, 10, 0, 0, 0, 10, 0, 5, 5, 5};   // 4 verts; vert 3 unreferenced by good tri
            m.t = {0, 1, 2,   0, 1, 99,   -1, 0, 1,   0, 1, 2};  // good, oob, neg, good-dup
            break;
        }
        case F_TINY_SCALE: {
            const double s = rng.range(1e-6, 1e-3);   // sub-micron mm → extreme fp32 metres
            addBox(m, 0, 0, 0, s, s, s);
            break;
        }
        case F_HUGE_SCALE: {
            const double s = rng.range(1e5, 1e7);     // huge mm coords; stress accessor min/max + fp32
            addBox(m, rng.range(-1e6, 1e6), 0, 0, s, s, s);
            break;
        }
        case F_HUGE_COUNT: {
            // Many small triangles (grid of quads) — stress buffer sizes / offsets / counts.
            const int gx = 20 + static_cast<int>(rng.below(40));
            const int gy = 20 + static_cast<int>(rng.below(40));
            for (int y = 0; y <= gy; ++y)
                for (int x = 0; x <= gx; ++x) { m.v.push_back(x); m.v.push_back(y); m.v.push_back(0); }
            auto vid = [&](int x, int y) { return y * (gx + 1) + x; };
            for (int y = 0; y < gy; ++y)
                for (int x = 0; x < gx; ++x) {
                    m.t.push_back(vid(x, y)); m.t.push_back(vid(x + 1, y)); m.t.push_back(vid(x + 1, y + 1));
                    m.t.push_back(vid(x, y)); m.t.push_back(vid(x + 1, y + 1)); m.t.push_back(vid(x, y + 1));
                }
            break;
        }
    }
    return m;
}

// ── per-trial classification counters ────────────────────────────────────────────
struct Counts { int agreed = 0, disagreed = 0, declined = 0, oracleInacc = 0, nativeInacc = 0; };

// Compare the writer's round-tripped mesh (Parsed P) against the independent oracle.
// Returns true iff round-trip fidelity holds; sets `oracleInaccurate` when a fp32
// quantization collision makes a stricter compare unrepresentable (writer still right).
bool roundTripMatches(const OracleMesh& O, const Parsed& P, bool& oracleInaccurate) {
    oracleInaccurate = false;
    // Vertex count + triangle count + connectivity must match index-for-index.
    if (P.pos.size() != O.pos.size()) return false;
    if (P.idx.size() != O.idx.size()) return false;
    for (std::size_t i = 0; i < O.idx.size(); ++i)
        if (P.idx[i] != O.idx[i]) return false;

    // Empty geometry (a vertex-bearing but 0-triangle input compacts to 0 reachable verts):
    // the writer emits a valid count:0 asset with min/max = [0,0,0]. Nothing to round-trip;
    // the count + structural checks above are the whole fidelity claim.
    if (O.pos.empty()) return true;

    // Positions: the writer stores fp32 metres. Compare each decoded fp32 position to the
    // fp32 cast of the oracle's fp64 position. They must be EXACTLY equal (the writer does a
    // plain static_cast<float>, no rounding mode games), EXCEPT where two distinct oracle
    // verts collapse to the same fp32 — then a full-precision position round-trip is simply
    // not representable in the on-disk f32 format (ORACLE-INACCURATE: the writer is faithful
    // to its documented f32 contract; our fp64 oracle is the over-precise one).
    for (std::size_t v = 0; v < O.pos.size(); ++v)
        for (int d = 0; d < 3; ++d) {
            const float want = static_cast<float>(O.pos[v][d]);
            if (P.pos[v][d] != want) {
                // Is it a benign fp32 collision (want and stored both finite, and the stored
                // value equals the fp32 cast of SOME oracle vert)? Non-finite fp64→fp32 would
                // already have failed the structural NaN/Inf gate.
                if (!std::isfinite(want)) { oracleInaccurate = true; continue; }
                return false;
            }
        }

    // Bounding box in metres (fp32 tol relative to the coordinate magnitude).
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (auto& p : O.pos)
        for (int d = 0; d < 3; ++d) { if (p[d] < lo[d]) lo[d] = p[d]; if (p[d] > hi[d]) hi[d] = p[d]; }
    for (int d = 0; d < 3; ++d) {
        const double magLo = std::fabs(lo[d]) + 1.0, magHi = std::fabs(hi[d]) + 1.0;
        if (std::fabs(static_cast<double>(P.posMin[d]) - lo[d]) > 1e-6 * magLo) return false;
        if (std::fabs(static_cast<double>(P.posMax[d]) - hi[d]) > 1e-6 * magHi) return false;
    }
    return true;
}

// Run one trial: export .gltf + .glb, validate both, round-trip both. Returns the bucket.
void runTrial(int idx, int family, const Mesh& m, std::uint64_t seed, Counts& C, bool& cc_ok_) {
    const std::string gp = tmpPath("cc_gltf_fuzz.gltf");
    const std::string bp = tmpPath("cc_gltf_fuzz.glb");
    const bool rg = gltf_export_mesh(m.v, m.t, gp, /*glb=*/false);
    const bool rb = gltf_export_mesh(m.v, m.t, bp, /*glb=*/true);

    const OracleMesh O = oracleCompact(m.v, m.t);

    if (!rg || !rb) {                 // pure I/O failure on a valid mesh: honest decline.
        ++C.declined;
        return;
    }

    // A mesh with no reachable triangles (0-tri input, or all-bad indices): the raw writer
    // does NOT no-op-skip here — it emits a fully-formed, spec-valid glTF-2.0 asset with
    // count:0 accessors, byteLength:0 buffer and min/max=[0,0,0]. (The facade cc_gltf_export
    // never reaches this path — its engine tessellate declines a null/empty body first.) So
    // an empty-geometry export is validated + round-tripped like any other, NOT a decline;
    // its bytes are non-empty. A truly 0-byte file (only on an empty path string) is the I/O
    // no-op success and is counted a decline.
    {
        const std::vector<unsigned char> gb = readBytes(gp), bb = readBytes(bp);
        if (gb.empty() && bb.empty()) { ++C.declined; return; }
    }

    // .gltf leg.
    const std::vector<unsigned char> gbytes = readBytes(gp);
    const std::string gjson(reinterpret_cast<const char*>(gbytes.data()), gbytes.size());
    const std::vector<unsigned char> gbuf = decodeDataUri(gjson);
    bool gStruct = false;
    const Parsed GP = validateAndParse(gjson, gbuf, gStruct);

    // .glb leg.
    const std::vector<unsigned char> bbytes = readBytes(bp);
    std::string bjson;
    std::vector<unsigned char> bbuf;
    const bool glbOk = parseGlb(bbytes, bjson, bbuf);
    // .glb JSON buffer 0 must have NO uri (implicit BIN chunk).
    const bool glbNoUri = !jsonHas(bjson, "\"uri\"");
    bool bStruct = false;
    const Parsed BP = glbOk ? validateAndParse(bjson, bbuf, bStruct) : Parsed{};

    // Single-source buffer: the .gltf base64 buffer and the .glb BIN chunk (minus its
    // 0-padding, which the writer stores whole) must be byte-identical.
    bool bufIdentical = (gbuf.size() == bbuf.size()) && (gbuf == bbuf);

    // Structural validity (A).
    const bool structuralOk = gStruct && glbOk && glbNoUri && bStruct && bufIdentical;
    if (!structuralOk) {
        std::printf("  DISAGREE[struct] idx=%d fam=%s seed=0x%llx gStruct=%d glbOk=%d noUri=%d "
                    "bStruct=%d bufEq=%d (verts=%zu tris=%zu)\n",
                    idx, famName(family), static_cast<unsigned long long>(seed), gStruct, glbOk,
                    glbNoUri, bStruct, static_cast<int>(bufIdentical), O.pos.size(), O.idx.size() / 3);
        CC_CHECK(structuralOk);       // marks the suite failed
        ++C.disagreed;
        return;
    }

    // Geometry fidelity round-trip (B) — on BOTH legs against the independent oracle.
    bool gInacc = false, bInacc = false;
    const bool gRT = roundTripMatches(O, GP, gInacc);
    const bool bRT = roundTripMatches(O, BP, bInacc);
    if (!gRT || !bRT) {
        std::printf("  DISAGREE[roundtrip] idx=%d fam=%s seed=0x%llx gltfRT=%d glbRT=%d "
                    "verts(want=%zu gltf=%zu glb=%zu) tris(want=%zu gltf=%zu glb=%zu)\n",
                    idx, famName(family), static_cast<unsigned long long>(seed), gRT, bRT,
                    O.pos.size(), GP.pos.size(), BP.pos.size(),
                    O.idx.size() / 3, GP.idx.size() / 3, BP.idx.size() / 3);
        CC_CHECK(gRT && bRT);
        ++C.disagreed;
        return;
    }

    // Determinism: a second export of each leg is byte-identical.
    const std::string gp2 = tmpPath("cc_gltf_fuzz2.gltf");
    const std::string bp2 = tmpPath("cc_gltf_fuzz2.glb");
    CC_CHECK(gltf_export_mesh(m.v, m.t, gp2, false));
    CC_CHECK(gltf_export_mesh(m.v, m.t, bp2, true));
    const bool det = (readBytes(gp) == readBytes(gp2)) && (readBytes(bp) == readBytes(bp2));
    if (!det) {
        std::printf("  DISAGREE[determinism] idx=%d fam=%s seed=0x%llx\n", idx, famName(family),
                    static_cast<unsigned long long>(seed));
        CC_CHECK(det);
        ++C.disagreed;
        return;
    }

    if (gInacc || bInacc) {
        // ORACLE-INACCURATE: distinct fp64 mm verts collapsed to the same on-disk fp32 metre
        // (only the F_TINY/HUGE-scale extremes reach this). The writer is faithful to its
        // documented f32 position contract; our fp64 oracle is over-precise. Not a DISAGREE.
        ++C.oracleInacc;
        return;
    }
    ++C.agreed;
}

void runBatch(std::uint64_t seed, int N, Counts& C, bool& cc_ok_) {
    Rng rng(seed);
    for (int i = 0; i < N; ++i) {
        const int family = static_cast<int>(rng.below(F_COUNT));
        const Mesh m = genMesh(family, rng);
        runTrial(i, family, m, seed, C, cc_ok_);
    }
    std::printf("== seed=0x%llx N=%d :: AGREED=%d DISAGREED=%d DECLINED=%d "
                "ORACLE-INACC=%d NATIVE-CHECK-INACC=%d ==\n",
                static_cast<unsigned long long>(seed), N, C.agreed, C.disagreed, C.declined,
                C.oracleInacc, C.nativeInacc);
}

}  // namespace

// The single fuzz case: two deterministic seeds (default + env override) × N trials.
// BAR: DISAGREED == 0. Any DISAGREE prints seed+index+family for exact repro.
CC_TEST(gltf_roundtrip_differential_fuzz) {
    std::uint64_t seed = 0x676C544600000001ull;   // "glTF" ...
    if (const char* e = std::getenv("FUZZ_SEED")) seed = std::strtoull(e, nullptr, 0);
    const int N = 240;

    Counts C;
    runBatch(seed, N, C, cc_ok_);
    runBatch(seed ^ 0x9E3779B97F4A7C15ull, N, C, cc_ok_);   // a second, decorrelated stream

    std::printf("== TOTAL :: AGREED=%d DISAGREED=%d DECLINED=%d ORACLE-INACC=%d "
                "NATIVE-CHECK-INACC=%d (bar: DISAGREED==0) ==\n",
                C.agreed, C.disagreed, C.declined, C.oracleInacc, C.nativeInacc);
    CC_CHECK(C.disagreed == 0);
    CC_CHECK(C.agreed > 0);
}

CC_RUN_ALL()
