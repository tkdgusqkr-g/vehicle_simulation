#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace vsim {

// Track as the user edits it (design doc §14.1): control points of the
// centerline with boundary widths, start/finish and sector markers at control
// points, and driving lines as lateral offsets from the centerline (D-10).
// Stored as *.track.json; TUMFTM CSV (x_m, y_m, w_tr_right_m, w_tr_left_m)
// can be read too.
struct TrackDefinition {
  struct Point {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;           // [m] elevation of the centerline
    double w_left = 1.5;      // [m] centerline to the left boundary
    double w_right = 1.5;     // [m] centerline to the right boundary
    double bank_deg = 0.0;    // cross slope, left edge up +
  };
  // A driving line: offset n [m, positive left] at u = s / length in [0, 1].
  struct Line {
    std::string name;
    std::vector<std::pair<double, double>> offsets;
  };

  // Road surface. Friction is relative to the tire data (1: as measured).
  struct Surface {
    double mu = 1.0;
    std::string roughness;               // ISO 8608 class "A".."H"; empty: smooth
    double grass_mu = 0.6;               // beyond the boundaries and kerbs
    std::string grass_roughness = "C";
  };
  // Kerb along one boundary between two control points, outside the edge.
  struct Kerb {
    int from_point = 0;
    int to_point = 1;
    std::string side = "left";  // left | right
    double width = 1.0;         // [m]
    double height = 0.03;       // [m] crest
  };
  // Area with its own friction (wet patch, oil, gravel) between two control
  // points and two lateral offsets.
  struct Patch {
    int from_point = 0;
    int to_point = 1;
    double n_min = -100.0;  // [m], positive left
    double n_max = 100.0;
    double mu = 1.0;
  };

  static constexpr int kSchemaVersion = 1;

  std::string name;
  bool closed = true;
  std::vector<Point> points;
  int start_point = 0;
  int finish_point = -1;  // open tracks; -1 = last point
  std::vector<int> sector_points;
  std::vector<Line> lines;
  Surface surface;
  std::vector<Kerb> kerbs;
  std::vector<Patch> patches;

  static TrackDefinition from_json(std::string_view text);
  static TrackDefinition from_file(const std::filesystem::path& file);
  std::string to_json() const;
  // Throws std::invalid_argument describing the first problem.
  void validate() const;
};

// One sample of a resampled curve.
struct CurveSample {
  double s = 0.0;          // [m] arc length
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;    // [rad], counter-clockwise from +x
  double curvature = 0.0;  // [1/m], positive turning left
};

// Where a point lies relative to a curve.
struct CurveProjection {
  double s = 0.0;        // [m] along the curve
  double n = 0.0;        // [m] lateral offset, positive left
  double heading = 0.0;  // [rad] curve heading there
  double distance = 0.0; // |n| unless beyond an open curve's ends
};

// A curve resampled at (nearly) constant arc-length steps, with projection.
class SampledCurve {
 public:
  SampledCurve() = default;
  SampledCurve(std::vector<CurveSample> samples, bool closed);

  bool closed() const { return closed_; }
  double length() const { return length_; }
  const std::vector<CurveSample>& samples() const { return samples_; }
  // Interpolated sample at s (wrapped on closed curves, clamped on open ones).
  CurveSample at(double s) const;
  double wrap(double s) const;
  // Nearest point. With a hint (previous s), only +-window around it is
  // searched, which keeps the projection on the right branch where a
  // course crosses itself (skidpad).
  CurveProjection project(double x, double y, std::optional<double> hint = {}, double window = 25.0) const;

 protected:
  std::size_t index_at(double s) const;

  std::vector<CurveSample> samples_;
  bool closed_ = false;
  double length_ = 0.0;
};

enum class SurfaceKind : int { Road = 0, Kerb = 1, Grass = 2 };

// The ground at one point.
struct RoadPoint {
  double z = 0.0;                 // [m] height
  double dzdx = 0.0, dzdy = 0.0;  // world gradient
  double mu = 1.0;                // friction relative to the tire data
  SurfaceKind kind = SurfaceKind::Road;
  double s = 0.0, n = 0.0;        // where it lies on the track
};

// Track geometry for the simulation: the centerline as a C2 cubic spline
// (periodic when closed) through the control points with chord-length
// parameters, resampled every ~0.25 m with heading, curvature and widths.
class Track : public SampledCurve {
 public:
  static constexpr double kStep = 0.25;  // [m]

  struct Widths {
    double left = 0.0;
    double right = 0.0;
  };

  explicit Track(TrackDefinition definition);

  const TrackDefinition& definition() const { return def_; }
  const std::string& name() const { return def_.name; }
  Widths widths_at(double s) const;
  // Arc length at a control point.
  double point_s(int index) const;
  double start_s() const { return point_s(def_.start_point); }
  double finish_s() const;
  std::vector<double> sector_s() const;

  // "center" or one of the definition's lines; throws if unknown.
  SampledCurve line(std::string_view name) const;
  // Offset of a line at centerline position s.
  double line_offset(const TrackDefinition::Line& line, double s) const;
  // Geometry problems worth showing (tight radius, folded boundaries).
  std::vector<std::string> warnings() const;

  // The ground at world (x, y) (design doc §14.1): centerline elevation (C2
  // spline through the control points) and bank, kerbs, roughness (ISO 8608,
  // fixed by the track's name) and friction; beyond the boundaries and kerbs
  // it is grass. `hint` (s of an earlier call nearby) keeps the search local
  // and is updated.
  RoadPoint road(double x, double y, double* hint = nullptr) const;
  // Height without roughness at (s, n), for meshes.
  double surface_height(double s, double n) const;
  // Anything but a flat, smooth plane at z = 0.
  bool has_relief() const { return relief_; }
  // Centerline elevation at s.
  double elevation(double s) const;

 private:
  struct Wave {
    double ks, kn, amplitude, phase;  // z = a cos(ks s + kn n + phase)
  };
  struct Ground {
    double z, dzds, dzdn;
  };
  Ground centre_ground(double s, double n) const;
  double kerb_at(double s, double e, bool left, Ground& g) const;
  static void add_waves(const std::vector<Wave>& waves, double s, double n, Ground& g);
  std::vector<Wave> make_waves(const std::string& iso_class, std::uint64_t seed) const;

  TrackDefinition def_;
  std::vector<double> point_s_;
  std::vector<Widths> widths_;  // per sample
  std::vector<double> z_, tan_bank_;  // per sample
  std::vector<Wave> road_waves_, grass_waves_;
  bool relief_ = false;
};

// ISO 8608 road class: displacement PSD at 0.1 cycles/m [m^3] (geometric mean
// of the class), "A".."H". Throws for anything else.
double iso8608_gd0(const std::string& road_class);

// Track file for a scenario: track.file (relative to the scenario's
// directory) or track.template (a name in templates/tracks). Empty if
// neither is given; throws if the template cannot be found.
std::optional<std::filesystem::path> resolve_track_file(const std::string& file, const std::string& template_name,
                                                        const std::filesystem::path& scenario_dir);

}  // namespace vsim
