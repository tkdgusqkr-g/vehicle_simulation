#pragma once

#include <memory>
#include <vector>

#include "vsim/sim/block.hpp"
#include "vsim/track/track.hpp"

namespace vsim {

// Lap and sector timing on the scenario's track (design doc §14), every plant
// step so crossing times can be interpolated between steps.
//
// Closed tracks: the first crossing of the start/finish line starts lap 1;
// every later crossing completes a lap. Open tracks (acceleration, skidpad):
// the run is timed from the start line to the finish line. A lap is invalid
// once the car's centre leaves the track boundaries.
//
// Inputs:  veh.chassis.{x,y,yaw}
// Outputs: track.{s,n,heading_error,distance,lap,lap_time,last_lap,best_lap,
//          last_lap_valid,lap_valid,sector,last_sector,off_track,off_track_count,finished}
class LapTimer final : public Block {
 public:
  std::string_view name() const override { return "lap_timer"; }
  void init(const ParamTree& params, Context& ctx) override;
  double period() const override { return period_; }
  void step(double t, SignalBus& bus) override;

 private:
  void cross(double t_cross, SignalBus& bus);

  std::shared_ptr<const Track> track_;
  double period_ = 0.001;
  // Boundaries along the unwrapped distance from the start line, ascending,
  // repeating every lap on closed tracks: sectors then the finish.
  std::vector<double> marks_;
  double lap_length_ = 0.0;   // closed: track length; open: start to finish

  bool first_ = true;
  double s_ = 0.0;
  double distance_ = 0.0;     // unwrapped, from the start line
  double t_prev_ = 0.0;
  double next_mark_ = 0.0;    // distance of the next boundary
  std::size_t mark_index_ = 0;
  int lap_ = 0;               // completed laps
  bool timing_ = false;
  bool finished_ = false;
  double lap_start_ = 0.0;
  double sector_start_ = 0.0;
  double last_lap_ = 0.0;
  double best_lap_ = 0.0;
  double last_sector_ = 0.0;
  bool lap_valid_ = true;
  bool last_lap_valid_ = false;
  bool off_track_ = false;
  int off_track_count_ = 0;

  SignalId x_in_ = 0, y_in_ = 0, yaw_in_ = 0;
  SignalId s_out_ = 0, n_out_ = 0, heading_out_ = 0, distance_out_ = 0;
  SignalId lap_out_ = 0, lap_time_out_ = 0, last_lap_out_ = 0, best_lap_out_ = 0;
  SignalId last_valid_out_ = 0, valid_out_ = 0, sector_out_ = 0, last_sector_out_ = 0;
  SignalId off_out_ = 0, off_count_out_ = 0, finished_out_ = 0;
};

// Start pose for a track: `distance` metres before the start line, on the
// centerline, facing along it. Returns {x, y, yaw}.
struct TrackPose {
  double x = 0.0, y = 0.0, yaw = 0.0, s = 0.0;
};
TrackPose start_pose(const Track& track, double distance);

}  // namespace vsim
