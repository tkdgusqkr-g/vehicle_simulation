#include "vsim/visual/gltf.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace vsim {

std::string write_glb(const std::vector<GltfNode>& nodes, std::string_view generator) {
  using nlohmann::json;
  std::string bin;
  const auto append = [&](const void* data, std::size_t bytes) {
    const std::size_t offset = bin.size();
    bin.append(static_cast<const char*>(data), bytes);
    while (bin.size() % 4 != 0) {
      bin.push_back('\0');
    }
    return offset;
  };
  json buffer_views = json::array(), accessors = json::array(), materials = json::array();
  json meshes = json::array(), gltf_nodes = json::array(), scene_nodes = json::array();
  const auto view = [&](std::size_t offset, std::size_t bytes, int target) {
    buffer_views.push_back({{"buffer", 0}, {"byteOffset", offset}, {"byteLength", bytes}, {"target", target}});
    return buffer_views.size() - 1;
  };
  constexpr int kArrayBuffer = 34962, kElementBuffer = 34963, kFloat = 5126, kUint = 5125;
  // glTF colour factors are linear; section colours are sRGB.
  const auto linear = [](float c) { return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f); };

  for (const GltfNode& node : nodes) {
    json primitives = json::array();
    for (const MeshSection& m : *node.sections) {
      const std::size_t n = m.vertex_count();
      if (n == 0 || m.indices.empty()) {
        continue;
      }
      std::vector<float> pos(3 * n), nor(3 * n);
      float pmin[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max()};
      float pmax[3] = {-pmin[0], -pmin[0], -pmin[0]};
      for (std::size_t i = 0; i < n; ++i) {
        const float p[3] = {m.positions[3 * i], m.positions[3 * i + 2], -m.positions[3 * i + 1]};
        const float q[3] = {m.normals[3 * i], m.normals[3 * i + 2], -m.normals[3 * i + 1]};
        for (int a = 0; a < 3; ++a) {
          pos[3 * i + a] = p[a];
          nor[3 * i + a] = q[a];
          pmin[a] = std::min(pmin[a], p[a]);
          pmax[a] = std::max(pmax[a], p[a]);
        }
      }
      const auto pos_view = view(append(pos.data(), pos.size() * 4), pos.size() * 4, kArrayBuffer);
      const auto nor_view = view(append(nor.data(), nor.size() * 4), nor.size() * 4, kArrayBuffer);
      const auto uv_view = view(append(m.uvs.data(), m.uvs.size() * 4), m.uvs.size() * 4, kArrayBuffer);
      const auto idx_view =
          view(append(m.indices.data(), m.indices.size() * 4), m.indices.size() * 4, kElementBuffer);
      accessors.push_back({{"bufferView", pos_view}, {"componentType", kFloat}, {"count", n}, {"type", "VEC3"},
                           {"min", {pmin[0], pmin[1], pmin[2]}}, {"max", {pmax[0], pmax[1], pmax[2]}}});
      accessors.push_back({{"bufferView", nor_view}, {"componentType", kFloat}, {"count", n}, {"type", "VEC3"}});
      accessors.push_back({{"bufferView", uv_view}, {"componentType", kFloat}, {"count", n}, {"type", "VEC2"}});
      accessors.push_back({{"bufferView", idx_view}, {"componentType", kUint}, {"count", m.indices.size()},
                           {"type", "SCALAR"}});
      const std::size_t a = accessors.size() - 4;
      json material = {{"name", m.name},
                       {"pbrMetallicRoughness",
                        {{"baseColorFactor", {linear(m.color[0]), linear(m.color[1]), linear(m.color[2]), m.color[3]}},
                         {"metallicFactor", m.metallic},
                         {"roughnessFactor", m.roughness}}}};
      if (m.flags & kDoubleSided) {
        material["doubleSided"] = true;
      }
      material["extras"] = {{"cast_shadow", (m.flags & kCastShadow) != 0}};
      materials.push_back(material);
      primitives.push_back({{"attributes", {{"POSITION", a}, {"NORMAL", a + 1}, {"TEXCOORD_0", a + 2}}},
                            {"indices", a + 3},
                            {"material", materials.size() - 1}});
    }
    json gnode = {{"name", node.name}};
    if (!primitives.empty()) {
      meshes.push_back({{"name", node.name}, {"primitives", primitives}});
      gnode["mesh"] = meshes.size() - 1;
    }
    const Vec3& t = node.translation;
    if (t.x != 0.0 || t.y != 0.0 || t.z != 0.0) {
      gnode["translation"] = {t.x, t.z, -t.y};
    }
    if (!node.extras.empty()) {
      gnode["extras"] = json::parse(node.extras);
    }
    gltf_nodes.push_back(gnode);
    scene_nodes.push_back(gltf_nodes.size() - 1);
  }

  json doc;
  doc["asset"] = {{"version", "2.0"}, {"generator", std::string(generator)}};
  doc["buffers"] = {{{"byteLength", bin.size()}}};
  doc["bufferViews"] = buffer_views;
  doc["accessors"] = accessors;
  doc["materials"] = materials;
  doc["meshes"] = meshes;
  doc["nodes"] = gltf_nodes;
  doc["scenes"] = {{{"nodes", scene_nodes}}};
  doc["scene"] = 0;

  std::string text = doc.dump();
  while (text.size() % 4 != 0) {
    text.push_back(' ');
  }
  const auto u32 = [](std::string& out, std::uint32_t v) {
    char bytes[4];
    std::memcpy(bytes, &v, 4);  // glTF is little-endian, as are the targets
    out.append(bytes, 4);
  };
  std::string glb;
  u32(glb, 0x46546C67);  // "glTF"
  u32(glb, 2);
  u32(glb, static_cast<std::uint32_t>(12 + 8 + text.size() + 8 + bin.size()));
  u32(glb, static_cast<std::uint32_t>(text.size()));
  u32(glb, 0x4E4F534A);  // "JSON"
  glb += text;
  u32(glb, static_cast<std::uint32_t>(bin.size()));
  u32(glb, 0x004E4942);  // "BIN\0"
  glb += bin;
  return glb;
}

std::string to_glb(const VisualModel& model) {
  using nlohmann::json;
  const auto vec = [](const Vec3& v) { return json::array({v.x, v.y, v.z}); };
  std::vector<GltfNode> nodes;
  for (const VisualPart& part : model.parts) {
    GltfNode node;
    node.name = part.name;
    node.sections = &part.sections;
    const char* role = part.role == PartRole::Body          ? "body"
                       : part.role == PartRole::Wheel         ? "wheel"
                       : part.role == PartRole::Upright       ? "upright"
                       : part.role == PartRole::SteeringWheel ? "steering_wheel"
                                                              : "link";
    json extras = {{"role", role}, {"cg_height", model.cg_height}};
    if (part.corner >= 0) {
      extras["corner"] = part.corner;
    }
    if (part.role != PartRole::Body) {
      extras["pivot"] = vec(part.pivot);
      node.translation = part.pivot;
    }
    if (part.role == PartRole::SteeringWheel) {
      extras["axis"] = vec(part.axis);
    }
    if (part.role == PartRole::Link) {
      extras["outer"] = vec(part.outer);
      extras["steers"] = part.steers;
    }
    node.extras = extras.dump();
    nodes.push_back(std::move(node));
  }
  return write_glb(nodes, "vsim formula car");
}

}  // namespace vsim
