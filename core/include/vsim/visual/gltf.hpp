#pragma once

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

}  // namespace vsim
