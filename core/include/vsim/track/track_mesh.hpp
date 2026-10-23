#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "vsim/track/track.hpp"
#include "vsim/visual/mesh.hpp"

namespace vsim {

struct TrackMesh {
  std::vector<MeshSection> sections;
  std::array<double, 3> min{}, max{};  // bounding box of everything
};

struct TrackMeshOptions {
  double step = 0.5;           // [m] road strip resolution along the track
  double line_width = 0.12;    // [m] painted edge lines
  double ground_margin = 40.0; // [m] grass around the track's bounding box
  double cone_spacing = 5.0;   // [m] along each boundary; 0: no cones
  double cone_offset = 0.35;   // [m] outside the boundary
  bool scenery = true;         // trees, grandstand, pit tents, start gantry
};

// Scenery for a renderer (design doc §16.3): asphalt, edge lines,
// start/finish and sector lines, cones (formula student colours: blue on the
// left boundary, yellow on the right, orange at the start), a ground plane
// and, with options.scenery, trees, a grandstand, pit tents and a start
// gantry placed deterministically from the track's name. Sections: ground,
// asphalt, edge_lines, start_line, sector_lines, cones_*, and tree_trunk,
// tree_leaves, tree_leaves_dark, grandstand, seats, tent_roof, tent_frame,
// gantry, banner (empty ones are left out).
TrackMesh make_track_mesh(const Track& track, const TrackMeshOptions& options = {});

// The mesh as binary glTF 2.0 (.glb), y up as glTF requires, one material per
// section (PBR, named after it). Opens in Blender, Unreal, three.js.
std::string to_glb(const TrackMesh& mesh);

}  // namespace vsim
