// SPDX-License-Identifier: Apache-2.0
//
// usdz_writer.cpp — native USDA (ASCII USD) mesh layer + USDZ (STORE-zip, 64-byte
// data-aligned) packager. See usdz_writer.h for the contract. OCCT-FREE,
// clang++ -std=c++20, standard library only.

#include "native/exchange/usdz_writer.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>

namespace cybercad::native::exchange {

namespace {

constexpr double kMmToM = 1.0e-3;  // kernel millimetres → USD metres

// ── mesh compaction (shared shape with the glTF writer: keep referenced verts) ─

struct Compacted {
  std::vector<std::array<double, 3>> pos;  // metres
  std::vector<std::uint32_t> idx;          // 3·triCount
};

Compacted compact(const std::vector<double>& vertices, const std::vector<int>& triangles) {
  const std::size_t vertCount = vertices.size() / 3;
  const std::size_t triCount = triangles.size() / 3;
  std::vector<std::uint32_t> remap(vertCount, std::numeric_limits<std::uint32_t>::max());
  Compacted out;
  auto srcAt = [&](int i) -> std::array<double, 3> {
    const std::size_t o = static_cast<std::size_t>(i) * 3;
    return {vertices[o] * kMmToM, vertices[o + 1] * kMmToM, vertices[o + 2] * kMmToM};
  };
  auto emit = [&](int oldIdx) -> std::uint32_t {
    std::uint32_t& slot = remap[static_cast<std::size_t>(oldIdx)];
    if (slot == std::numeric_limits<std::uint32_t>::max()) {
      slot = static_cast<std::uint32_t>(out.pos.size());
      out.pos.push_back(srcAt(oldIdx));
    }
    return slot;
  };
  for (std::size_t t = 0; t < triCount; ++t) {
    const int i = triangles[t * 3], j = triangles[t * 3 + 1], k = triangles[t * 3 + 2];
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

// ── USD ASCII (.usda) layer text ────────────────────────────────────────────────

// Locale-independent float formatting (%.9g captures fp32 shape; commas → '.').
std::string num(double v) {
  char buf[40];
  const int n = std::snprintf(buf, sizeof(buf), "%.9g", v);
  std::string s(buf, buf + (n > 0 ? n : 0));
  for (char& ch : s)
    if (ch == ',') ch = '.';
  return s;
}

// Sanitize a name into a valid USD identifier ([A-Za-z_][A-Za-z0-9_]*).
std::string usdIdent(const std::string& in) {
  std::string s;
  for (char c : in) {
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                    c == '_';
    s.push_back(ok ? c : '_');
  }
  if (s.empty() || (s[0] >= '0' && s[0] <= '9')) s.insert(s.begin(), '_');
  return s;
}

std::string buildUsda(const Compacted& m, const std::string& name) {
  const std::string ident = usdIdent(name);
  std::string s;
  s.reserve(4096 + m.pos.size() * 48 + m.idx.size() * 8);

  // Layer header: USD ASCII magic, metres, Y-up (QuickLook convention), default prim.
  s += "#usda 1.0\n";
  s += "(\n";
  s += "    defaultPrim = \"" + ident + "\"\n";
  s += "    metersPerUnit = 1\n";
  s += "    upAxis = \"Y\"\n";
  s += ")\n\n";

  s += "def Mesh \"" + ident + "\"\n{\n";

  // faceVertexCounts — one 3 per triangle.
  const std::size_t triCount = m.idx.size() / 3;
  s += "    int[] faceVertexCounts = [";
  for (std::size_t t = 0; t < triCount; ++t) {
    if (t) s += ", ";
    s += "3";
  }
  s += "]\n";

  // faceVertexIndices — flat index list.
  s += "    int[] faceVertexIndices = [";
  for (std::size_t i = 0; i < m.idx.size(); ++i) {
    if (i) s += ", ";
    s += std::to_string(m.idx[i]);
  }
  s += "]\n";

  // points — vertex positions (metres).
  s += "    point3f[] points = [";
  for (std::size_t v = 0; v < m.pos.size(); ++v) {
    if (v) s += ", ";
    s += "(" + num(m.pos[v][0]) + ", " + num(m.pos[v][1]) + ", " + num(m.pos[v][2]) + ")";
  }
  s += "]\n";

  // subdivisionScheme none => render as a polygon mesh (not Catmull-Clark).
  s += "    uniform token subdivisionScheme = \"none\"\n";

  // A neutral display colour so QuickLook shows a shaded surface without a material
  // network (a constant primvar is valid USD and enough for the AR preview).
  s += "    color3f[] primvars:displayColor = [(0.8, 0.8, 0.82)] (\n";
  s += "        interpolation = \"constant\"\n";
  s += "    )\n";

  s += "}\n";
  return s;
}

// ── CRC-32 (IEEE 802.3, the ZIP polynomial) ─────────────────────────────────────

std::uint32_t crc32(const unsigned char* data, std::size_t n) {
  static std::uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      table[i] = c;
    }
    init = true;
  }
  std::uint32_t c = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < n; ++i) c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

// ── STORE-only ZIP with 64-byte data alignment (USDZ container) ─────────────────

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

// USDZ requires each entry's DATA to start on a 64-byte boundary. The local file
// header is 30 bytes + name + extra; we size the extra field so that
// (headerStart + 30 + name + extra) % 64 == 0. We use a generic "extra" record
// (id 0x1986, chosen per the USD reference impl's alignment padding) whose payload
// is zero-filled — readers ignore unknown extra ids.
constexpr std::uint32_t kUsdzAlign = 64;

struct ZipEntry {
  std::string name;
  std::vector<unsigned char> data;
  std::uint32_t crc = 0;
  std::uint32_t localHeaderOffset = 0;
  std::uint16_t extraLen = 0;
};

std::vector<unsigned char> buildZip(std::vector<ZipEntry>& entries) {
  std::vector<unsigned char> z;

  for (ZipEntry& e : entries) {
    e.crc = crc32(e.data.data(), e.data.size());
    e.localHeaderOffset = static_cast<std::uint32_t>(z.size());

    // Compute the extra-field length that 64-byte-aligns the data start.
    const std::uint32_t fixed = 30 + static_cast<std::uint32_t>(e.name.size());
    const std::uint32_t base = e.localHeaderOffset + fixed;
    // extra field must itself be >= 4 (id + size) if present; find the smallest
    // extraLen >= 0 making (base + extraLen) % 64 == 0, but if a nonzero pad < 4 is
    // required we add a full 64 so the 4-byte extra header fits.
    std::uint32_t pad = (kUsdzAlign - (base % kUsdzAlign)) % kUsdzAlign;
    if (pad != 0 && pad < 4) pad += kUsdzAlign;
    e.extraLen = static_cast<std::uint16_t>(pad);

    // ── local file header ──
    put_u32le(z, 0x04034b50);           // local file header signature
    put_u16le(z, 20);                   // version needed (2.0)
    put_u16le(z, 0);                    // general purpose bit flag
    put_u16le(z, 0);                    // compression method 0 = STORE
    put_u16le(z, 0);                    // last mod time (fixed → deterministic)
    put_u16le(z, 0);                    // last mod date
    put_u32le(z, e.crc);                // crc-32
    put_u32le(z, static_cast<std::uint32_t>(e.data.size()));  // compressed size
    put_u32le(z, static_cast<std::uint32_t>(e.data.size()));  // uncompressed size
    put_u16le(z, static_cast<std::uint16_t>(e.name.size()));  // file name length
    put_u16le(z, e.extraLen);           // extra field length
    z.insert(z.end(), e.name.begin(), e.name.end());
    if (e.extraLen >= 4) {
      // extra field record: 2-byte id, 2-byte data-size, then zero payload.
      put_u16le(z, 0x1986);                                          // padding id
      put_u16le(z, static_cast<std::uint16_t>(e.extraLen - 4));      // data size
      for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(e.extraLen - 4); ++i)
        z.push_back(0);
    }
    // data (aligned to 64 by construction)
    z.insert(z.end(), e.data.begin(), e.data.end());
  }

  // ── central directory ──
  const std::uint32_t cdStart = static_cast<std::uint32_t>(z.size());
  for (const ZipEntry& e : entries) {
    put_u32le(z, 0x02014b50);  // central dir header signature
    put_u16le(z, 20);          // version made by
    put_u16le(z, 20);          // version needed
    put_u16le(z, 0);           // flags
    put_u16le(z, 0);           // method STORE
    put_u16le(z, 0);           // time
    put_u16le(z, 0);           // date
    put_u32le(z, e.crc);
    put_u32le(z, static_cast<std::uint32_t>(e.data.size()));
    put_u32le(z, static_cast<std::uint32_t>(e.data.size()));
    put_u16le(z, static_cast<std::uint16_t>(e.name.size()));
    put_u16le(z, 0);  // extra length in central dir (0 — alignment lives in local hdr)
    put_u16le(z, 0);  // comment length
    put_u16le(z, 0);  // disk number start
    put_u16le(z, 0);  // internal attrs
    put_u32le(z, 0);  // external attrs
    put_u32le(z, e.localHeaderOffset);
    z.insert(z.end(), e.name.begin(), e.name.end());
  }
  const std::uint32_t cdSize = static_cast<std::uint32_t>(z.size()) - cdStart;

  // ── end of central directory ──
  put_u32le(z, 0x06054b50);
  put_u16le(z, 0);  // disk number
  put_u16le(z, 0);  // cd start disk
  put_u16le(z, static_cast<std::uint16_t>(entries.size()));  // entries on this disk
  put_u16le(z, static_cast<std::uint16_t>(entries.size()));  // total entries
  put_u32le(z, cdSize);
  put_u32le(z, cdStart);
  put_u16le(z, 0);  // comment length
  return z;
}

bool writeAll(const std::string& path, const void* data, std::size_t size) {
  if (path.empty()) return false;
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  if (size > 0) out.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
  return static_cast<bool>(out);
}

}  // namespace

bool usda_export_mesh(const std::vector<double>& vertices, const std::vector<int>& triangles,
                      const std::string& path, const std::string& name) {
  const Compacted m = compact(vertices, triangles);
  const std::string usda = buildUsda(m, name);
  return writeAll(path, usda.data(), usda.size());
}

bool usdz_export_mesh(const std::vector<double>& vertices, const std::vector<int>& triangles,
                      const std::string& path, const std::string& name) {
  const Compacted m = compact(vertices, triangles);
  const std::string usda = buildUsda(m, name);

  ZipEntry entry;
  // The default layer QuickLook opens is the FIRST entry. USDZ convention: the model
  // layer is named "<name>.usda" (the package's default layer).
  entry.name = usdIdent(name) + ".usda";
  entry.data.assign(usda.begin(), usda.end());

  std::vector<ZipEntry> entries{std::move(entry)};
  const std::vector<unsigned char> zip = buildZip(entries);
  return writeAll(path, zip.data(), zip.size());
}

}  // namespace cybercad::native::exchange
