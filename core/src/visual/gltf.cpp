#include "vsim/visual/gltf.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>

#include "vsim/util/paths.hpp"

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

namespace {

using nlohmann::json;
using Matrix = std::array<double, 16>;  // column-major, as glTF

Matrix identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }

Matrix multiply(const Matrix& a, const Matrix& b) {
  Matrix r{};
  for (int c = 0; c < 4; ++c) {
    for (int row = 0; row < 4; ++row) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        sum += a[k * 4 + row] * b[c * 4 + k];
      }
      r[c * 4 + row] = sum;
    }
  }
  return r;
}

Matrix node_matrix(const json& node) {
  if (node.contains("matrix")) {
    Matrix m{};
    for (int i = 0; i < 16; ++i) {
      m[i] = node["matrix"][i].get<double>();
    }
    return m;
  }
  const auto vec = [&](const char* key, std::array<double, 4> fallback) {
    if (node.contains(key)) {
      for (std::size_t i = 0; i < node[key].size(); ++i) {
        fallback[i] = node[key][i].get<double>();
      }
    }
    return fallback;
  };
  const auto t = vec("translation", {0, 0, 0, 0});
  const auto q = vec("rotation", {0, 0, 0, 1});
  const auto s = vec("scale", {1, 1, 1, 0});
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),
                       2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                       2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y)};
  return {r[0] * s[0], r[1] * s[0], r[2] * s[0], 0, r[3] * s[1], r[4] * s[1], r[5] * s[1], 0,
          r[6] * s[2], r[7] * s[2], r[8] * s[2], 0, t[0],        t[1],        t[2],        1};
}

class GlbFile {
 public:
  explicit GlbFile(const std::string& bytes) {
    const auto u32 = [&](std::size_t at) {
      if (at + 4 > bytes.size()) {
        throw std::invalid_argument("glTF: file too short");
      }
      std::uint32_t v = 0;
      std::memcpy(&v, bytes.data() + at, 4);
      return v;
    };
    if (u32(0) != 0x46546C67 || u32(4) != 2) {
      throw std::invalid_argument("glTF: not a binary glTF 2.0 file (.glb)");
    }
    std::size_t at = 12;
    while (at + 8 <= bytes.size()) {
      const std::uint32_t length = u32(at), type = u32(at + 4);
      if (at + 8 + length > bytes.size()) {
        throw std::invalid_argument("glTF: chunk runs past the end");
      }
      if (type == 0x4E4F534A) {
        doc = json::parse(bytes.substr(at + 8, length));
      } else if (type == 0x004E4942) {
        bin = bytes.substr(at + 8, length);
      }
      at += 8 + length;
    }
    if (doc.is_null()) {
      throw std::invalid_argument("glTF: no JSON chunk");
    }
  }

  // Accessor as doubles, `components` per element.
  std::vector<double> read(std::size_t accessor_index, int& components) const {
    const json& a = doc.at("accessors").at(accessor_index);
    const std::string type = a.at("type");
    components = type == "SCALAR" ? 1 : type == "VEC2" ? 2 : type == "VEC3" ? 3 : type == "VEC4" ? 4 : 0;
    if (components == 0) {
      throw std::invalid_argument("glTF: unsupported accessor type " + type);
    }
    const std::size_t count = a.at("count");
    std::vector<double> out(count * static_cast<std::size_t>(components), 0.0);
    if (!a.contains("bufferView")) {
      return out;  // all zeros (sparse accessors not supported)
    }
    const json& view = doc.at("bufferViews").at(a.at("bufferView").get<std::size_t>());
    if (view.value("buffer", 0) != 0) {
      throw std::invalid_argument("glTF: only the binary chunk buffer is supported");
    }
    const int component_type = a.at("componentType");
    const std::size_t size = component_type == 5126 || component_type == 5125 ? 4 : component_type == 5123 || component_type == 5122 ? 2 : 1;
    const std::size_t stride = view.value("byteStride", static_cast<std::size_t>(components) * size);
    const std::size_t base = view.value("byteOffset", std::size_t{0}) + a.value("byteOffset", std::size_t{0});
    const bool normalized = a.value("normalized", false);
    for (std::size_t i = 0; i < count; ++i) {
      for (int c = 0; c < components; ++c) {
        const std::size_t at = base + i * stride + static_cast<std::size_t>(c) * size;
        if (at + size > bin.size()) {
          throw std::invalid_argument("glTF: accessor runs past the buffer");
        }
        double v = 0.0;
        switch (component_type) {
          case 5126: { float f; std::memcpy(&f, bin.data() + at, 4); v = f; break; }
          case 5125: { std::uint32_t u; std::memcpy(&u, bin.data() + at, 4); v = u; break; }
          case 5123: { std::uint16_t u; std::memcpy(&u, bin.data() + at, 2); v = normalized ? u / 65535.0 : u; break; }
          case 5122: { std::int16_t u; std::memcpy(&u, bin.data() + at, 2); v = normalized ? std::max(-1.0, u / 32767.0) : u; break; }
          case 5121: { std::uint8_t u; std::memcpy(&u, bin.data() + at, 1); v = normalized ? u / 255.0 : u; break; }
          case 5120: { std::int8_t u; std::memcpy(&u, bin.data() + at, 1); v = normalized ? std::max(-1.0, u / 127.0) : u; break; }
          default: throw std::invalid_argument("glTF: unsupported component type");
        }
        out[i * static_cast<std::size_t>(components) + static_cast<std::size_t>(c)] = v;
      }
    }
    return out;
  }

  json doc;
  std::string bin;
};

float srgb(double linear) {
  const double c = std::clamp(linear, 0.0, 1.0);
  return static_cast<float>(c <= 0.0031308 ? 12.92 * c : 1.055 * std::pow(c, 1.0 / 2.4) - 0.055);
}

// glTF (x, y up, z) to vsim (x, y left, z up).
Vec3 from_gltf(double x, double y, double z) { return {x, -z, y}; }

void append_mesh(const GlbFile& file, std::size_t mesh_index, const Matrix& world, const Vec3& pivot,
                 std::vector<MeshSection>& out) {
  const json& doc = file.doc;
  const json& mesh = doc.at("meshes").at(mesh_index);
  for (const json& primitive : mesh.at("primitives")) {
    if (primitive.value("mode", 4) != 4) {
      continue;  // triangles only
    }
    const json& attributes = primitive.at("attributes");
    if (!attributes.contains("POSITION")) {
      continue;
    }
    MeshSection section;
    section.name = "part";
    section.flags = kCastShadow;
    section.metallic = 1.0f;  // glTF defaults
    section.roughness = 1.0f;
    if (primitive.contains("material")) {
      const json& m = doc.at("materials").at(primitive["material"].get<std::size_t>());
      section.name = m.value("name", std::string("part"));
      const json pbr = m.value("pbrMetallicRoughness", json::object());
      if (pbr.contains("baseColorFactor")) {
        for (int i = 0; i < 3; ++i) {
          section.color[i] = srgb(pbr["baseColorFactor"][i].get<double>());
        }
        section.color[3] = static_cast<float>(pbr["baseColorFactor"][3].get<double>());
      }
      section.metallic = pbr.value("metallicFactor", 1.0f);
      section.roughness = pbr.value("roughnessFactor", 1.0f);
      if (m.value("doubleSided", false)) {
        section.flags |= kDoubleSided;
      }
      const json extras = m.value("extras", json::object());
      if (extras.contains("cast_shadow") && !extras["cast_shadow"].get<bool>()) {
        section.flags &= ~static_cast<std::uint32_t>(kCastShadow);
      }
    }
    int pc = 0, nc = 0, uc = 0, ic = 0;
    const std::vector<double> pos = file.read(attributes["POSITION"].get<std::size_t>(), pc);
    const std::size_t n = pos.size() / 3;
    std::vector<double> nor, uv;
    if (attributes.contains("NORMAL")) {
      nor = file.read(attributes["NORMAL"].get<std::size_t>(), nc);
    }
    if (attributes.contains("TEXCOORD_0")) {
      uv = file.read(attributes["TEXCOORD_0"].get<std::size_t>(), uc);
    }
    // Normals transform with the inverse transpose; for rotation + uniform
    // or axis scale, the scaled upper 3x3 renormalised is close enough.
    for (std::size_t i = 0; i < n; ++i) {
      const double x = pos[3 * i], y = pos[3 * i + 1], z = pos[3 * i + 2];
      const double wx = world[0] * x + world[4] * y + world[8] * z + world[12];
      const double wy = world[1] * x + world[5] * y + world[9] * z + world[13];
      const double wz = world[2] * x + world[6] * y + world[10] * z + world[14];
      Vec3 normal{0, 0, 1};
      if (!nor.empty()) {
        const double a = nor[3 * i], b = nor[3 * i + 1], c = nor[3 * i + 2];
        normal = from_gltf(world[0] * a + world[4] * b + world[8] * c, world[1] * a + world[5] * b + world[9] * c,
                           world[2] * a + world[6] * b + world[10] * c)
                     .normalized();
      }
      section.add_vertex(from_gltf(wx, wy, wz) - pivot, normal, uv.empty() ? 0.0 : uv[2 * i],
                         uv.empty() ? 0.0 : uv[2 * i + 1]);
    }
    if (primitive.contains("indices")) {
      const std::vector<double> idx = file.read(primitive["indices"].get<std::size_t>(), ic);
      for (const double v : idx) {
        if (v >= static_cast<double>(n)) {
          throw std::invalid_argument("glTF: index out of range");
        }
        section.indices.push_back(static_cast<std::uint32_t>(v));
      }
    } else {
      for (std::size_t i = 0; i < n; ++i) {
        section.indices.push_back(static_cast<std::uint32_t>(i));
      }
    }
    // A transform with negative determinant mirrors the triangles.
    const double det = world[0] * (world[5] * world[10] - world[9] * world[6]) -
                       world[4] * (world[1] * world[10] - world[9] * world[2]) +
                       world[8] * (world[1] * world[6] - world[5] * world[2]);
    if (det < 0.0) {
      for (std::size_t t = 0; t + 2 < section.indices.size(); t += 3) {
        std::swap(section.indices[t + 1], section.indices[t + 2]);
      }
    }
    if (nor.empty()) {
      geometry::smooth_normals(section, 0, 0);
    }
    out.push_back(std::move(section));
  }
}

}  // namespace

VisualModel read_vehicle_glb(const std::string& bytes) {
  const GlbFile file(bytes);
  const json& doc = file.doc;
  VisualModel model;
  model.name = doc.value("asset", json::object()).value("generator", std::string("glTF model"));
  VisualPart body;
  body.name = "body";
  body.role = PartRole::Body;
  const json nodes = doc.value("nodes", json::array());
  const auto vec = [](const json& v) { return Vec3{v[0].get<double>(), v[1].get<double>(), v[2].get<double>()}; };

  // Walk the scene; a node with extras.role starts a part for its subtree.
  struct Item {
    std::size_t node;
    Matrix parent;
    VisualPart* part;
  };
  std::vector<Item> stack;
  std::vector<VisualPart> parts;
  parts.reserve(nodes.size());
  const json scene_nodes = doc.contains("scenes") ? doc["scenes"].at(doc.value("scene", 0)).value("nodes", json::array())
                                                  : json::array();
  // Depth first in file order (the stack pops the last push first).
  for (auto n = scene_nodes.rbegin(); n != scene_nodes.rend(); ++n) {
    stack.push_back({n->get<std::size_t>(), identity(), nullptr});
  }
  bool any_role = false;
  while (!stack.empty()) {
    const Item item = stack.back();
    stack.pop_back();
    const json& node = nodes.at(item.node);
    const Matrix world = multiply(item.parent, node_matrix(node));
    VisualPart* part = item.part;
    const json extras = node.value("extras", json::object());
    if (!part && extras.contains("role")) {
      any_role = true;
      const std::string role = extras["role"];
      VisualPart p;
      p.name = node.value("name", role);
      p.role = role == "wheel" ? PartRole::Wheel
               : role == "upright" ? PartRole::Upright
               : role == "steering_wheel" ? PartRole::SteeringWheel
               : role == "link" ? PartRole::Link
                                : PartRole::Body;
      p.corner = extras.value("corner", -1);
      if (extras.contains("pivot")) {
        p.pivot = vec(extras["pivot"]);
      }
      if (extras.contains("axis")) {
        p.axis = vec(extras["axis"]);
      }
      if (extras.contains("outer")) {
        p.outer = vec(extras["outer"]);
      }
      p.steers = extras.value("steers", false);
      if (extras.contains("cg_height")) {
        model.cg_height = extras["cg_height"].get<double>();
      }
      if ((p.role == PartRole::Wheel || p.role == PartRole::Upright || p.role == PartRole::Link) &&
          (p.corner < 0 || p.corner > 3)) {
        throw std::invalid_argument("glTF: part '" + p.name + "' needs extras.corner 0..3");
      }
      if (p.role == PartRole::Body) {
        part = &body;
      } else {
        parts.push_back(std::move(p));
        part = &parts.back();
      }
    }
    VisualPart* target = part ? part : &body;
    if (node.contains("mesh")) {
      // Links are stored in their own frame (a tube along +x): no pivot shift
      // other than the node origin, which is the pivot.
      append_mesh(file, node["mesh"].get<std::size_t>(), world, target == &body ? Vec3{} : target->pivot,
                  target->sections);
    }
    const json children = node.value("children", json::array());
    for (auto child = children.rbegin(); child != children.rend(); ++child) {
      stack.push_back({child->get<std::size_t>(), world, part});
    }
  }
  (void)any_role;
  if (!body.sections.empty()) {
    model.parts.insert(model.parts.begin(), std::move(body));
  }
  for (VisualPart& p : parts) {
    model.parts.push_back(std::move(p));
  }
  if (model.parts.empty()) {
    throw std::invalid_argument("glTF: no meshes in the scene");
  }
  return model;
}

VisualModel read_vehicle_glb_file(const std::filesystem::path& file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot open " + path_to_utf8(file));
  }
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  try {
    return read_vehicle_glb(bytes);
  } catch (const std::exception& e) {
    throw std::invalid_argument(path_to_utf8(file) + ": " + e.what());
  }
}

VisualModel make_vehicle_visual(const ParamTree& params) {
  const std::string file = params.get_string("vehicle.visual.model", "");
  if (file.empty()) {
    return make_formula_car(params);
  }
  const std::filesystem::path path =
      path_from_utf8(params.get_string("project.dir", ".")) / path_from_utf8(file);
  VisualModel model = read_vehicle_glb_file(path);
  model.name = path_to_utf8(path.filename());
  if (!params.has("vehicle.visual.cg_height_m")) {
    model.cg_height = params.get_double("vehicle.cg_height_m", model.cg_height);
  }
  return model;
}

}  // namespace vsim
