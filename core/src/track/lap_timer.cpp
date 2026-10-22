#include "vsim/track/lap_timer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "vsim/sim/context.hpp"

namespace vsim {

void LapTimer::init(const ParamTree& /*params*/, Context& ctx) {
  track_ = ctx.track();
  if (!track_) {
    throw std::logic_error("LapTimer needs a track");
  }
  period_ = ctx.dt();
  const Track& track = *track_;
  const double start = track.start_s();
  marks_.clear();
  if (track.closed()) {
    lap_length_ = track.length();
    for (const double s : track.sector_s()) {
      const double offset = track.wrap(s - start);
      if (offset > 0.0) {
        marks_.push_back(offset);
      }
    }
  } else {
    lap_length_ = track.finish_s() - start;
    if (!(lap_length_ > 0.0)) {
      throw std::invalid_argument("track '" + track.name() + "': the finish must lie after the start");
    }
    for (const double s : track.sector_s()) {
      if (s - start > 0.0 && s - start < lap_length_) {
        marks_.push_back(s - start);
      }
    }
  }
  std::sort(marks_.begin(), marks_.end());
  marks_.push_back(lap_length_);

  SignalBus& bus = ctx.bus();
  x_in_ = bus.declare("veh.chassis.x", "m", "position east");
  y_in_ = bus.declare("veh.chassis.y", "m", "position north");
  yaw_in_ = bus.declare("veh.chassis.yaw", "rad", "heading, counter-clockwise from east");
  s_out_ = bus.declare("track.s", "m", "position along the centerline");
  n_out_ = bus.declare("track.n", "m", "lateral offset from the centerline, positive left");
  heading_out_ = bus.declare("track.heading_error", "rad", "vehicle yaw minus track heading");
  distance_out_ = bus.declare("track.distance", "m", "distance travelled along the track since the start line");
  lap_out_ = bus.declare("track.lap", "-", "completed laps");
  lap_time_out_ = bus.declare("track.lap_time", "s", "time of the current lap");
  last_lap_out_ = bus.declare("track.last_lap", "s", "time of the last completed lap (0: none)");
  best_lap_out_ = bus.declare("track.best_lap", "s", "best valid lap time (0: none)");
  last_valid_out_ = bus.declare("track.last_lap_valid", "-", "1 if the last completed lap stayed on track");
  valid_out_ = bus.declare("track.lap_valid", "-", "1 while the current lap is on track");
  sector_out_ = bus.declare("track.sector", "-", "current sector, from 0");
  last_sector_out_ = bus.declare("track.last_sector", "s", "time of the last completed sector");
  off_out_ = bus.declare("track.off_track", "-", "1 while the car's centre is outside the boundaries");
  off_count_out_ = bus.declare("track.off_track_count", "-", "times the car left the track");
  finished_out_ = bus.declare("track.finished", "-", "1 after the finish line of an open track");
}

void LapTimer::cross(double t_cross, SignalBus& /*bus*/) {
  if (!timing_) {
    timing_ = true;
    lap_start_ = sector_start_ = t_cross;
    lap_valid_ = !off_track_;
    mark_index_ = 0;
    next_mark_ += marks_[0];
    return;
  }
  const bool lap_end = mark_index_ + 1 == marks_.size();
  last_sector_ = t_cross - sector_start_;
  sector_start_ = t_cross;
  if (!lap_end) {
    ++mark_index_;
    next_mark_ += marks_[mark_index_] - marks_[mark_index_ - 1];
    return;
  }
  last_lap_ = t_cross - lap_start_;
  last_lap_valid_ = lap_valid_;
  if (lap_valid_ && (best_lap_ == 0.0 || last_lap_ < best_lap_)) {
    best_lap_ = last_lap_;
  }
  ++lap_;
  if (!track_->closed()) {
    timing_ = false;
    finished_ = true;
    next_mark_ = std::numeric_limits<double>::infinity();
    return;
  }
  lap_start_ = t_cross;
  lap_valid_ = !off_track_;
  mark_index_ = 0;
  next_mark_ += marks_[0];
}

void LapTimer::step(double t, SignalBus& bus) {
  const Track& track = *track_;
  const double x = bus.get(x_in_);
  const double y = bus.get(y_in_);
  const CurveProjection p = first_ ? track.project(x, y) : track.project(x, y, s_);
  const double start = track.start_s();
  double distance = 0.0;
  if (first_) {
    distance = track.closed() ? std::remainder(p.s - start, track.length()) : p.s - start;
    next_mark_ = 0.0;
    t_prev_ = t;
  } else if (track.closed()) {
    distance = distance_ + std::remainder(p.s - s_, track.length());
  } else {
    distance = p.s - start;
  }

  // Off track: the car's centre beyond a boundary.
  const Track::Widths w = track.widths_at(p.s);
  const bool off = p.n > w.left || p.n < -w.right;
  if (off && !off_track_) {
    ++off_track_count_;
    lap_valid_ = false;
  }
  off_track_ = off;

  // Line crossings between the previous and this step, interpolated in time.
  if (!first_) {
    while (distance >= next_mark_ && distance > distance_) {
      const double f = (next_mark_ - distance_) / (distance - distance_);
      cross(t_prev_ + std::clamp(f, 0.0, 1.0) * (t - t_prev_), bus);
    }
  }
  first_ = false;
  s_ = p.s;
  distance_ = distance;
  t_prev_ = t;

  bus.set(s_out_, p.s);
  bus.set(n_out_, p.n);
  bus.set(heading_out_, std::remainder(bus.get(yaw_in_) - p.heading, 2.0 * std::numbers::pi));
  bus.set(distance_out_, distance);
  bus.set(lap_out_, lap_);
  bus.set(lap_time_out_, timing_ ? t - lap_start_ : (finished_ ? last_lap_ : 0.0));
  bus.set(last_lap_out_, last_lap_);
  bus.set(best_lap_out_, best_lap_);
  bus.set(last_valid_out_, last_lap_valid_ ? 1.0 : 0.0);
  bus.set(valid_out_, lap_valid_ ? 1.0 : 0.0);
  bus.set(sector_out_, static_cast<double>(timing_ ? mark_index_ : 0));
  bus.set(last_sector_out_, last_sector_);
  bus.set(off_out_, off_track_ ? 1.0 : 0.0);
  bus.set(off_count_out_, off_track_count_);
  bus.set(finished_out_, finished_ ? 1.0 : 0.0);
}

TrackPose start_pose(const Track& track, double distance) {
  const double s = track.closed() ? track.wrap(track.start_s() - distance)
                                  : std::max(0.0, track.start_s() - distance);
  const CurveSample sample = track.at(s);
  return {sample.x, sample.y, sample.heading, s};
}

}  // namespace vsim
