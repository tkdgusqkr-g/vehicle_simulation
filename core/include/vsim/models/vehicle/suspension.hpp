#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "vsim/sim/params.hpp"
#include "vsim/util/vec3.hpp"

namespace vsim {

// Double wishbone with a push rod, left corner of an axle (design doc §5.5).
// Axle frame at design ride height: x forward from the axle line, y left from
// the centreline, z up from the ground. The right corner is the mirror image.
struct WishboneGeometry {
  Vec3 lower_front, lower_rear, lower_outer;  // inner pivots, ball joint
  Vec3 upper_front, upper_rear, upper_outer;
  Vec3 tie_inner, tie_outer;                  // rack end, steering arm (toe link at the rear)
  Vec3 push_outer, push_inner;                // on the lower wishbone, on the rocker
  Vec3 wheel_centre;
  // Optional rocker: pivot axis and the spring (or damper) ends. Without it
  // the spring moves 1 / motion_ratio per unit of wheel travel.
  bool has_rocker = false;
  Vec3 rocker_pivot, rocker_axis{1.0, 0.0, 0.0}, spring_rocker, spring_chassis;
  double motion_ratio = 1.0;   // wheel travel / spring travel, without a rocker
  double static_camber = 0.0;  // [rad], top of the wheel outwards +
  double static_toe = 0.0;     // [rad], toe-in +
  bool push_on_upright = false;

  // vehicle.suspension.<axle>.hardpoints.* (defaults: the example car),
  // static_camber_deg, static_toe_deg, motion_ratio and the optional rocker
  // points rocker_pivot, rocker_axis, spring_rocker, spring_chassis.
  static WishboneGeometry from_params(const ParamTree& params, bool front);
};

// The corner at one wheel travel and rack position, in the axle frame.
struct CornerPose {
  Vec3 wheel_centre, contact_patch, spin_axis;  // spin axis: unit, outwards
  Vec3 lower_ball, upper_ball, tie_outer, push_outer;
  double steer = 0.0;     // [rad] heading of the wheel, left +
  double camber = 0.0;    // [rad] top outwards +
  double spring = 0.0;    // [m] spring compression from design
  double travel = 0.0;    // [m] wheel centre rise from design
};

// Solves the linkage: lower wishbone angle from the wheel travel, the upper
// wishbone from the kingpin length, the upright's rotation about the kingpin
// from the tie rod length, and the rocker from the push rod length.
class WishboneSolver {
 public:
  explicit WishboneSolver(WishboneGeometry geometry);

  const WishboneGeometry& geometry() const { return g_; }
  // travel: wheel centre rise [m]; rack: inner tie rod end moved towards +y
  // [m]. Throws std::runtime_error if the linkage does not close.
  CornerPose solve(double travel, double rack) const;

 private:
  struct Angles {
    double lower = 0.0, upper = 0.0, upright = 0.0, rocker = 0.0;
  };
  CornerPose pose(double lower_angle, double rack, Angles& guess) const;

  WishboneGeometry g_;
  Vec3 lower_axis_, upper_axis_;
  double kingpin_length_ = 0.0, tie_length_ = 0.0, push_length_ = 0.0, spring_length_ = 0.0;
  Vec3 spin_axis0_;
  double tyre_radius_ = 0.0;  // wheel centre height at design
};

// Kinematics and compliance-free "K" curves of a left corner over wheel
// travel and rack travel, interpolated bilinearly.
struct KcPoint {
  double steer = 0.0, camber = 0.0;          // [rad]
  double cp_x = 0.0, cp_y = 0.0;             // contact patch moved from design [m]
  double dcp_x = 0.0, dcp_y = 0.0;           // d(contact patch) / d(travel)
  double spring = 0.0, dspring = 1.0;        // spring compression [m] and d / d(travel)
  double dsteer_drack = 0.0;                 // [rad/m]
  // Moment about the steering axis (towards steer-left) per unit tyre force
  // in the wheel frame at the contact patch (x, y, z) and per unit Mz.
  double arm_x = 0.0, arm_y = 0.0, arm_z = 0.0, arm_mz = 1.0;
};

class KcTable {
 public:
  KcTable() = default;
  // Travel range +-travel_range, rack range +-rack_range (0: no steering).
  static KcTable build(const WishboneSolver& solver, double travel_range, std::size_t travel_points,
                       double rack_range, std::size_t rack_points);

  KcPoint at(double travel, double rack) const;
  double travel_range() const { return travel_range_; }
  double rack_range() const { return rack_range_; }

 private:
  double travel_range_ = 0.0, rack_range_ = 0.0;
  std::size_t nt_ = 0, nr_ = 0;
  std::vector<KcPoint> points_;  // [rack][travel]
};

// Static figures of a corner for reports and the vehicle builder.
struct KcSummary {
  double kingpin_inclination_deg = 0.0;  // top inboard +
  double caster_deg = 0.0;               // top rearward +
  double scrub_radius_m = 0.0;           // axis meets the ground inboard of the contact patch +
  double mechanical_trail_m = 0.0;       // axis meets the ground ahead of the contact patch +
  double roll_centre_height_m = 0.0;
  double camber_gain_deg_per_m = 0.0;    // in bump
  double bump_steer_deg_per_m = 0.0;     // toe change in bump, left +
  double motion_ratio = 1.0;             // wheel travel / spring travel
  double anti_geometry = 0.0;            // d(contact patch x) / d(travel)
  double ackermann_percent = 0.0;        // front: inner wheel extra angle vs. full Ackermann at 10 deg
};
KcSummary summarize(const WishboneSolver& solver, double half_track, double wheelbase, bool front);

}  // namespace vsim
