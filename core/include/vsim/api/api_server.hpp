#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace vsim {

// WebSocket endpoint of the Sim API (design doc §4.4). Incoming messages are
// queued and handed to the sim loop, which handles them between ticks; the
// loop pushes state to every connected client with broadcast().
class ApiServer {
 public:
  ApiServer(int port, std::string host);
  ~ApiServer();
  ApiServer(const ApiServer&) = delete;
  ApiServer& operator=(const ApiServer&) = delete;

  // Starts listening; returns an error message on failure.
  std::optional<std::string> start();
  void stop();

  // Sent to every client when it connects (and kept for later clients).
  void set_hello(std::string text);
  void broadcast(const std::string& text);
  std::vector<std::string> take_messages();
  std::size_t client_count() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vsim
