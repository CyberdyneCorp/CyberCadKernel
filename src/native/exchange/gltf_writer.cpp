// SPDX-License-Identifier: Apache-2.0
//
// gltf_writer.cpp — native glTF 2.0 serializer (.gltf JSON + base64 buffer, and
// .glb binary container). See gltf_writer.h for the contract. OCCT-FREE,
// clang++ -std=c++20, standard library only.

#include "native/exchange/gltf_writer.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>

namespace cybercad::native::exchange {

namespace {

// Millimetre → metre (glTF's linear unit). One semantic transform; connectivity
// and (metre-space) bounds round-trip exactly under it.
constexpr double kMmToM = 1.0e-3;

// ── low-level little-endian byte emission (glTF buffers are always LE) ─────────

float to_f32(double d) {
  float f = static_cast<float>(d);
  return f == 0.0f ? 0.0f : f;  // normalize -0.0 for stable/deterministic output
}

void put_u32le(std::vector<unsigned char>& b, std::uint32_t v) {
  b.push_back(static_cast<unsigned char>(v & 0xFF));
  b.push_back(static_cast<unsigned char>((v >> 8) & 0xFF));
  b.push_back(static_cast<unsigned char>((v >> 16) & 0xFF));
  b.push_back(static_cast<unsigned char>((v >> 24) & 0xFF));
}

void put_f32le(std::vector<unsigned char>& b, float f) {
  put_u32le(b, std::bit_cast<std::uint32_t>(f));
}

// Pad `b` with `fill` bytes up to the next 4-byte boundary (glTF alignment rule).
void pad4(std::vector<unsigned char>& b, unsigned char fill = 0) {
  while (b.size() % 4 != 0) b.push_back(fill);
}

// ── mesh compaction: keep only referenced vertices, drop bad triangles ─────────

// A compacted, reindexed mesh: fp64 positions (metres), uint32 index triples, and
// smooth per-vertex normals derived from area-weighted face normals.
struct Compacted {
  std::vector<std::array<double, 3>> pos;   // metres
  std::vector<std::array<float, 3>> nrm;    // unit normals (f32)
  std::vector<std::uint32_t> idx;           // 3·triCount
};

std::array<double, 3> faceNormal(const std::array<double, 3>& a, const std::array<double, 3>& b,
                                 const std::array<double, 3>& c) {
  const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
  const double vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
  // Cross product magnitude is proportional to 2·triangle area, so accumulating the
  // UN-normalized cross gives an area-weighted vertex normal (larger faces dominate).
  return {uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx};
}

// Build the compacted mesh. Triangles with an out-of-range index are skipped
// (matching the STL writer). Only referenced vertices are kept and reindexed 0..N.
Compacted compact(const std::vector<double>& vertices, const std::vector<int>& triangles) {
  const std::size_t vertCount = vertices.size() / 3;
  const std::size_t triCount = triangles.size() / 3;

  // remap[old] = new index (or npos if not yet referenced)
  std::vector<std::uint32_t> remap(vertCount, std::numeric_limits<std::uint32_t>::max());
  Compacted out;
  std::vector<std::array<double, 3>> normAccum;  // fp64 accumulator, parallel to out.pos

  auto srcAt = [&](int i) -> std::array<double, 3> {
    const std::size_t o = static_cast<std::size_t>(i) * 3;
    return {vertices[o] * kMmToM, vertices[o + 1] * kMmToM, vertices[o + 2] * kMmToM};
  };
  auto emit = [&](int oldIdx) -> std::uint32_t {
    std::uint32_t& slot = remap[static_cast<std::size_t>(oldIdx)];
    if (slot == std::numeric_limits<std::uint32_t>::max()) {
      slot = static_cast<std::uint32_t>(out.pos.size());
      out.pos.push_back(srcAt(oldIdx));
      normAccum.push_back({0.0, 0.0, 0.0});
    }
    return slot;
  };

  for (std::size_t t = 0; t < triCount; ++t) {
    const int i = triangles[t * 3], j = triangles[t * 3 + 1], k = triangles[t * 3 + 2];
    if (i < 0 || j < 0 || k < 0) continue;
    if (static_cast<std::size_t>(i) >= vertCount || static_cast<std::size_t>(j) >= vertCount ||
        static_cast<std::size_t>(k) >= vertCount)
      continue;
    const std::uint32_t a = emit(i), b = emit(j), c = emit(k);
    out.idx.push_back(a);
    out.idx.push_back(b);
    out.idx.push_back(c);
    const auto n = faceNormal(out.pos[a], out.pos[b], out.pos[c]);
    for (std::uint32_t v : {a, b, c})
      for (int d = 0; d < 3; ++d) normAccum[v][d] += n[d];
  }

  // Normalize accumulated normals; a vertex touched only by degenerate faces gets a
  // fallback up-normal (valid unit vector, so the accessor stays well-formed).
  out.nrm.resize(out.pos.size());
  for (std::size_t v = 0; v < out.pos.size(); ++v) {
    const auto& a = normAccum[v];
    const double len = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (len < 1e-20)
      out.nrm[v] = {0.0f, 0.0f, 1.0f};
    else
      out.nrm[v] = {to_f32(a[0] / len), to_f32(a[1] / len), to_f32(a[2] / len)};
  }
  return out;
}

// ── binary buffer assembly (shared by .gltf base64 and .glb BIN chunk) ─────────

struct BufferLayout {
  std::vector<unsigned char> bytes;  // the whole glTF buffer (4-byte aligned blocks)
  std::uint32_t idxOffset = 0, idxLength = 0;   // ELEMENT_ARRAY bufferView
  std::uint32_t posOffset = 0, posLength = 0;   // ARRAY bufferView (POSITION)
  std::uint32_t nrmOffset = 0, nrmLength = 0;   // ARRAY bufferView (NORMAL)
  std::array<float, 3> posMin{}, posMax{};      // POSITION accessor bounds (metres)
};

BufferLayout buildBuffer(const Compacted& m) {
  BufferLayout L;
  // Block 1: indices (uint32). Offset 0, already 4-aligned.
  L.idxOffset = 0;
  for (std::uint32_t i : m.idx) put_u32le(L.bytes, i);
  L.idxLength = static_cast<std::uint32_t>(L.bytes.size() - L.idxOffset);
  pad4(L.bytes);

  // Block 2: POSITION (vec3 f32). Track min/max for the accessor bounds.
  L.posOffset = static_cast<std::uint32_t>(L.bytes.size());
  L.posMin = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
              std::numeric_limits<float>::infinity()};
  L.posMax = {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
              -std::numeric_limits<float>::infinity()};
  for (const auto& p : m.pos) {
    for (int d = 0; d < 3; ++d) {
      const float f = to_f32(p[d]);
      put_f32le(L.bytes, f);
      if (f < L.posMin[d]) L.posMin[d] = f;
      if (f > L.posMax[d]) L.posMax[d] = f;
    }
  }
  L.posLength = static_cast<std::uint32_t>(L.bytes.size() - L.posOffset);
  pad4(L.bytes);

  // Block 3: NORMAL (vec3 f32).
  L.nrmOffset = static_cast<std::uint32_t>(L.bytes.size());
  for (const auto& n : m.nrm)
    for (int d = 0; d < 3; ++d) put_f32le(L.bytes, n[d]);
  L.nrmLength = static_cast<std::uint32_t>(L.bytes.size() - L.nrmOffset);
  pad4(L.bytes);

  if (m.pos.empty()) L.posMin = L.posMax = {0.0f, 0.0f, 0.0f};
  return L;
}

// ── base64 (standard alphabet, '=' padding) for the .gltf data URI ─────────────

std::string base64(const std::vector<unsigned char>& in) {
  static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((in.size() + 2) / 3 * 4);
  std::size_t i = 0;
  for (; i + 3 <= in.size(); i += 3) {
    const std::uint32_t n = (in[i] << 16) | (in[i + 1] << 8) | in[i + 2];
    out.push_back(T[(n >> 18) & 63]);
    out.push_back(T[(n >> 12) & 63]);
    out.push_back(T[(n >> 6) & 63]);
    out.push_back(T[n & 63]);
  }
  const std::size_t rem = in.size() - i;
  if (rem == 1) {
    const std::uint32_t n = in[i] << 16;
    out.push_back(T[(n >> 18) & 63]);
    out.push_back(T[(n >> 12) & 63]);
    out.push_back('=');
    out.push_back('=');
  } else if (rem == 2) {
    const std::uint32_t n = (in[i] << 16) | (in[i + 1] << 8);
    out.push_back(T[(n >> 18) & 63]);
    out.push_back(T[(n >> 12) & 63]);
    out.push_back(T[(n >> 6) & 63]);
    out.push_back('=');
  }
  return out;
}

// ── JSON assembly ──────────────────────────────────────────────────────────────

// Locale-independent float32 formatting (round-trip enough shape for JSON numbers;
// %.9g captures a float32 exactly). Normalizes any locale decimal comma to '.'.
std::string jnum(float f) {
  char buf[32];
  const int n = std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(f));
  std::string s(buf, buf + (n > 0 ? n : 0));
  for (char& ch : s)
    if (ch == ',') ch = '.';
  return s;
}

std::string jsonEscape(const std::string& s) {
  std::string out;
  for (char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out.push_back(c);
    }
  }
  return out;
}

// The glTF JSON document. `bufferUri` is either a base64 data URI (.gltf) or empty
// (.glb, where the BIN chunk is the implicit buffer 0).
std::string buildJson(const Compacted& m, const BufferLayout& L, const std::string& name,
                      const std::string& bufferUri) {
  const std::uint32_t indexCount = static_cast<std::uint32_t>(m.idx.size());
  const std::uint32_t vertCount = static_cast<std::uint32_t>(m.pos.size());
  const std::uint32_t byteLength = static_cast<std::uint32_t>(L.bytes.size());
  const std::string nm = jsonEscape(name);

  std::string j;
  j.reserve(2048);
  j += "{";
  j += "\"asset\":{\"version\":\"2.0\",\"generator\":\"CyberCadKernel glTF writer\"},";
  j += "\"scene\":0,";
  j += "\"scenes\":[{\"nodes\":[0]}],";
  j += "\"nodes\":[{\"mesh\":0,\"name\":\"" + nm + "\"}],";

  // material — a neutral metallic-roughness PBR default (double-sided so a mesh with
  // any back-facing triangles still renders in QuickLook / RealityKit).
  j += "\"materials\":[{\"name\":\"CyberCadKernel_default\",\"doubleSided\":true,";
  j += "\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.8,0.8,0.82,1],";
  j += "\"metallicFactor\":0,\"roughnessFactor\":0.8}}],";

  // mesh — one primitive, POSITION+NORMAL, indexed triangles (mode 4), material 0.
  j += "\"meshes\":[{\"name\":\"" + nm + "\",\"primitives\":[{";
  j += "\"attributes\":{\"POSITION\":1,\"NORMAL\":2},\"indices\":0,\"material\":0,\"mode\":4}]}],";

  // accessors: 0=indices, 1=POSITION (+bounds), 2=NORMAL.
  j += "\"accessors\":[";
  j += "{\"bufferView\":0,\"byteOffset\":0,\"componentType\":5125,\"count\":" +
       std::to_string(indexCount) + ",\"type\":\"SCALAR\"},";
  j += "{\"bufferView\":1,\"byteOffset\":0,\"componentType\":5126,\"count\":" +
       std::to_string(vertCount) + ",\"type\":\"VEC3\",\"min\":[" + jnum(L.posMin[0]) + "," +
       jnum(L.posMin[1]) + "," + jnum(L.posMin[2]) + "],\"max\":[" + jnum(L.posMax[0]) + "," +
       jnum(L.posMax[1]) + "," + jnum(L.posMax[2]) + "]},";
  j += "{\"bufferView\":2,\"byteOffset\":0,\"componentType\":5126,\"count\":" +
       std::to_string(vertCount) + ",\"type\":\"VEC3\"}],";

  // bufferViews: 0=indices (target 34963), 1=POSITION, 2=NORMAL (target 34962).
  j += "\"bufferViews\":[";
  j += "{\"buffer\":0,\"byteOffset\":" + std::to_string(L.idxOffset) + ",\"byteLength\":" +
       std::to_string(L.idxLength) + ",\"target\":34963},";
  j += "{\"buffer\":0,\"byteOffset\":" + std::to_string(L.posOffset) + ",\"byteLength\":" +
       std::to_string(L.posLength) + ",\"target\":34962},";
  j += "{\"buffer\":0,\"byteOffset\":" + std::to_string(L.nrmOffset) + ",\"byteLength\":" +
       std::to_string(L.nrmLength) + ",\"target\":34962}],";

  // buffer 0 — base64 data URI for .gltf, or bare byteLength for .glb.
  j += "\"buffers\":[{";
  if (!bufferUri.empty()) j += "\"uri\":\"" + bufferUri + "\",";
  j += "\"byteLength\":" + std::to_string(byteLength) + "}]";
  j += "}";
  return j;
}

// ── I/O ─────────────────────────────────────────────────────────────────────────

bool writeAll(const std::string& path, const void* data, std::size_t size) {
  if (path.empty()) return false;
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  if (size > 0) out.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
  return static_cast<bool>(out);
}

// Assemble the .glb binary container: 12-byte header + JSON chunk + BIN chunk.
bool writeGlb(const std::string& json, const std::vector<unsigned char>& bin,
              const std::string& path) {
  // JSON chunk data padded to 4 bytes with spaces (0x20); BIN chunk padded with 0x00.
  std::vector<unsigned char> jsonChunk(json.begin(), json.end());
  while (jsonChunk.size() % 4 != 0) jsonChunk.push_back(0x20);
  std::vector<unsigned char> binChunk = bin;
  while (binChunk.size() % 4 != 0) binChunk.push_back(0x00);

  const std::uint32_t total = 12 + 8 + static_cast<std::uint32_t>(jsonChunk.size()) + 8 +
                              static_cast<std::uint32_t>(binChunk.size());
  std::vector<unsigned char> out;
  out.reserve(total);
  // header
  const char magic[4] = {'g', 'l', 'T', 'F'};
  out.insert(out.end(), magic, magic + 4);
  put_u32le(out, 2);      // version
  put_u32le(out, total);  // total length
  // JSON chunk
  put_u32le(out, static_cast<std::uint32_t>(jsonChunk.size()));
  put_u32le(out, 0x4E4F534A);  // "JSON"
  out.insert(out.end(), jsonChunk.begin(), jsonChunk.end());
  // BIN chunk
  put_u32le(out, static_cast<std::uint32_t>(binChunk.size()));
  put_u32le(out, 0x004E4942);  // "BIN\0"
  out.insert(out.end(), binChunk.begin(), binChunk.end());
  return writeAll(path, out.data(), out.size());
}

}  // namespace

bool gltf_export_mesh(const std::vector<double>& vertices, const std::vector<int>& triangles,
                      const std::string& path, bool glb, const std::string& name) {
  const Compacted m = compact(vertices, triangles);
  const BufferLayout L = buildBuffer(m);

  if (glb) {
    const std::string json = buildJson(m, L, name, /*bufferUri=*/"");
    return writeGlb(json, L.bytes, path);
  }
  const std::string uri = "data:application/octet-stream;base64," + base64(L.bytes);
  const std::string json = buildJson(m, L, name, uri);
  return writeAll(path, json.data(), json.size());
}

}  // namespace cybercad::native::exchange
