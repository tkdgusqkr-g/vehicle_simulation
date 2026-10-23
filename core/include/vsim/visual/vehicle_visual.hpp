#pragma once

#include <string>
#include <vector>

#include "vsim/sim/params.hpp"
#include "vsim/visual/mesh.hpp"

namespace vsim {

// How a renderer moves a part of the car (design doc §16.3).
enum class PartRole : std::uint32_t {
  Body = 1,           // with the body: position, yaw, then pitch/roll about the CG
  Wheel = 2,          // at the hub of `corner`: steer about z, then spin about y
  Upright = 3,        // at the hub of `corner`: steer only
  SteeringWheel = 4,  // with the body, turned by the steering wheel angle about `axis`
  Link = 5,           // unit tube along +x stretched from `pivot` (body) to `outer` (hub frame)
};

// One moving piece of the car. Car frame: origin on the ground below the CG,
// x forward, y left, z up, metres.
struct VisualPart {
  std::string name;
  PartRole role = PartRole::Body;
  int corner = -1;    // 0 FL, 1 FR, 2 RL, 3 RR for Wheel, Upright, Link
  Vec3 pivot;         // Wheel/Upright: hub; SteeringWheel: centre; Link: inboard end (body)
  Vec3 axis;          // SteeringWheel: rotation axis (towards the driver)
  Vec3 outer;         // Link: outboard end relative to the hub
  bool steers = false;  // Link: the outboard end turns with the wheel (tie rod)
  // Body: car frame. Wheel/Upright/SteeringWheel: relative to the pivot.
  // Link: a tube from x = 0 to x = 1 (radius in metres), stretched in x only.
  std::vector<MeshSection> sections;
};

struct VisualModel {
  std::string name;
  std::vector<VisualPart> parts;
  double cg_height = 0.3;  // body roll and pitch pivot height
};

// Procedural formula student car built to the vehicle's dimensions:
// monocoque with cockpit and driver, side pods, front and rear wings,
// undertray and diffuser, roll hoops, wheels (tyre, 10" rim, disc, hub
// motor), uprights and suspension links from the hardpoints
// (vehicle.suspension.<axle>.hardpoints, defaults if absent). Livery from
// vehicle.visual.color / accent ("#rrggbb").
VisualModel make_formula_car(const ParamTree& params);

// Wheel hub position of a corner in the car frame.
Vec3 hub_position(const ParamTree& params, int corner);

}  // namespace vsim
