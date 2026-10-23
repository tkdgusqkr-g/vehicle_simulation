#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "vsim/visual/mesh.hpp"
#include "vsim/visual/vehicle_visual.hpp"

namespace vsim {

// One node of a glTF scene: its mesh sections (one primitive and material
// each) placed at `translation`. Coordinates in the vsim frame; the file
// gets glTF's y-up frame: (x, y, z) -> (x, z, -y).
struct GltfNode {
  std::string name;
  const std::vector<MeshSection>* sections = nullptr;
  Vec3 translation;
  std::string extras;  // JSON object text, or empty
};

// Binary glTF 2.0. Materials are metallic-roughness PBR named after the
// sections (renderers key procedural textures on the names).
std::string write_glb(const std::vector<GltfNode>& nodes, std::string_view generator);

// The car as glTF: one node per part, named after it, with
// extras {role, corner, pivot, axis, outer, steers, cg_height} in the vsim
// car frame (x forward, y left, z up, m) so a viewer can animate the parts.
std::string to_glb(const VisualModel& model);

// Reads a car model written by to_glb or by the vehicle builder (binary glTF;
// nodes with extras.role are moving parts, anything else is body). Node
// transforms are baked into the vertices; parts keep vertices relative to
// their pivot. Throws on malformed files.
VisualModel read_vehicle_glb(const std::string& bytes);
VisualModel read_vehicle_glb_file(const std::filesystem::path& file);

// The car to show: vehicle.visual.model (a .glb relative to the project
// directory) if set, else the procedural formula student car.
VisualModel make_vehicle_visual(const ParamTree& params);

}  // namespace vsim
