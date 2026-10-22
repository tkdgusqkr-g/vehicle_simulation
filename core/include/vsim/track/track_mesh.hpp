#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "vsim/track/track.hpp"

namespace vsim {

// One coloured part of the track scenery. Coordinates are in the vsim frame
// (x east, y north, z up, metres); triangles wind counter-clockwise seen from
// their front (from above for the road).
struct MeshSection {
  std::string name;
  std::array<float, 4> color{1.0f, 1.0f, 1.0f, 1.0f};  // base colour, sRGB 0..1
  std::vector<float> positions;       // x, y, z per vertex
  std::vector<float> normals;         // x, y, z per vertex
  std::vector<float> uvs;             // u, v per vertex
  std::vector<std::uint32_t> indices;  // three per triangle

  std::size_t vertex_count() const { return positions.size() / 3; }
};

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
};

// Scenery for a renderer (design doc §16.3): asphalt, edge lines,
// start/finish and sector lines, cones (formula student colours: blue on the
// left boundary, yellow on the right, orange at the start) and a ground
// plane. Sections: ground, asphalt, edge_lines, start_line, sector_lines,
// cones_blue, cones_yellow, cones_orange (empty ones are left out).
TrackMesh make_track_mesh(const Track& track, const TrackMeshOptions& options = {});

// The mesh as binary glTF 2.0 (.glb), y up as glTF requires, one material per
// section. Opens in Blender, Unreal (Interchange / glTF importer), three.js.
std::string to_glb(const TrackMesh& mesh);

}  // namespace vsim
