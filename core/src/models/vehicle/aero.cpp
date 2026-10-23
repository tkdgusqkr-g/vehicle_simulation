#include "vsim/models/vehicle/aero.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace vsim {
namespace {

std::vector<double> axis(const ParamTree& params, const std::string& key) {
  std::vector<double> mm = params.get_double_list(key);
  if (mm.size() < 2 || !std::is_sorted(mm.begin(), mm.end()) ||
      std::adjacent_find(mm.begin(), mm.end()) != mm.end()) {
    throw std::invalid_argument(key + ": at least two ascending ride heights");
  }
  for (double& v : mm) {
    v *= 1e-3;
  }
  return mm;
}

// Rows of a table (a list of lists), row-major; `fill` if the table is missing.
std::vector<double> table(const ParamTree& params, const std::string& key, std::size_t rows, std::size_t cols,
                          double fill, bool required) {
  if (!params.has_prefix(key)) {
    if (required) {
      throw std::invalid_argument(key + " is missing");
    }
    return std::vector<double>(rows * cols, fill);
  }
  std::vector<double> out;
  out.reserve(rows * cols);
  for (std::size_t i = 0; i < rows; ++i) {
    const std::string row_key = key + "." + std::to_string(i);
    const std::vector<double> row = params.has_prefix(row_key) ? params.get_double_list(row_key) : std::vector<double>{};
    if (row.size() != cols) {
      throw std::invalid_argument(key + ": " + std::to_string(rows) + " rows (front_mm) of " + std::to_string(cols) +
                                  " values (rear_mm)");
    }
    out.insert(out.end(), row.begin(), row.end());
  }
  if (params.has_prefix(key + "." + std::to_string(rows))) {
    throw std::invalid_argument(key + ": more rows than front_mm has points");
  }
  return out;
}

// Index of the cell holding v and the weight of its upper point, held at the edges.
std::pair<std::size_t, double> locate(const std::vector<double>& grid, double v) {
  if (v <= grid.front()) {
    return {0, 0.0};
  }
  if (v >= grid.back()) {
    return {grid.size() - 2, 1.0};
  }
  const std::size_t i = static_cast<std::size_t>(std::upper_bound(grid.begin(), grid.end(), v) - grid.begin()) - 1;
  return {i, (v - grid[i]) / (grid[i + 1] - grid[i])};
}

}  // namespace

AeroMap AeroMap::from_params(const ParamTree& params) {
  AeroMap map;
  const std::string m = "vehicle.aero.ride_height_map";
  map.enabled = params.has_prefix(m);
  if (!map.enabled) {
    return map;
  }
  map.front = axis(params, m + ".front_mm");
  map.rear = axis(params, m + ".rear_mm");
  const std::size_t rows = map.front.size(), cols = map.rear.size();
  map.cla_factor = table(params, m + ".cla_factor", rows, cols, 1.0, true);
  map.front_ratio_shift = table(params, m + ".front_ratio_shift", rows, cols, 0.0, false);
  map.cda_factor = table(params, m + ".cda_factor", rows, cols, 1.0, false);
  for (std::size_t i = 0; i < rows * cols; ++i) {
    if (!(map.cla_factor[i] >= 0.0 && map.cda_factor[i] > 0.0)) {
      throw std::invalid_argument(m + ": cla_factor must not be negative, cda_factor must be positive");
    }
  }
  return map;
}

AeroMap::Sample AeroMap::at(double front_change, double rear_change) const {
  if (!enabled) {
    return {};
  }
  const auto [i, fi] = locate(front, front_change);
  const auto [j, fj] = locate(rear, rear_change);
  const std::size_t cols = rear.size();
  const auto blend = [&](const std::vector<double>& t) {
    const double low = t[i * cols + j] + fj * (t[i * cols + j + 1] - t[i * cols + j]);
    const double high = t[(i + 1) * cols + j] + fj * (t[(i + 1) * cols + j + 1] - t[(i + 1) * cols + j]);
    return low + fi * (high - low);
  };
  return {blend(cla_factor), blend(front_ratio_shift), blend(cda_factor)};
}

}  // namespace vsim
