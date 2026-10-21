#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace vsim {

// Writes logged signals to an MCAP file: one JSON message per log tick on the
// "signals" channel, so PlotJuggler and Foxglove open it without plugins.
// Timestamps are sim time, which keeps the file identical across reruns.
class McapLogger {
 public:
  McapLogger(const std::filesystem::path& file, std::vector<std::string> signal_names);
  ~McapLogger();
  McapLogger(const McapLogger&) = delete;
  McapLogger& operator=(const McapLogger&) = delete;

  // `values` is aligned with the names given at construction.
  void write(std::uint64_t time_ns, const std::vector<double>& values);
  void close();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vsim
