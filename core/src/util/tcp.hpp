#pragma once

// Internal: blocking TCP with timeouts (POSIX sockets / Winsock) for the
// external plugin's lockstep link, and starting a helper process.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace vsim {

class TcpStream {
 public:
  ~TcpStream();
  TcpStream(const TcpStream&) = delete;
  TcpStream& operator=(const TcpStream&) = delete;

  // Sends everything or throws.
  void send_all(const void* data, std::size_t size);
  // Receives exactly `size` bytes within `timeout_s` or throws (closed, timed out).
  void receive_all(void* data, std::size_t size, double timeout_s);
  std::string peer() const { return peer_; }

 private:
  friend class TcpListener;
  explicit TcpStream(std::intptr_t socket, std::string peer);
  std::intptr_t socket_;
  std::string peer_;
};

class TcpListener {
 public:
  // Listens on host:port (port 0: any free one).
  TcpListener(const std::string& host, int port);
  ~TcpListener();
  TcpListener(const TcpListener&) = delete;
  TcpListener& operator=(const TcpListener&) = delete;

  int port() const { return port_; }
  // The next connection within `timeout_s`, or throws.
  std::unique_ptr<TcpStream> accept(double timeout_s);

 private:
  std::intptr_t socket_;
  int port_ = 0;
};

// A UDP socket bound to host:port (port 0: any), non-blocking receive.
class UdpSocket {
 public:
  UdpSocket(const std::string& host, int port);
  ~UdpSocket();
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  int port() const { return port_; }
  void send_to(const std::string& host, int port, const void* data, std::size_t size);
  // One datagram if one is waiting (up to `capacity` bytes), else an empty vector.
  std::vector<std::uint8_t> receive(std::size_t capacity = 2048);

 private:
  std::intptr_t socket_;
  int port_ = 0;
};

// "host:port" split; throws if malformed.
void split_address(const std::string& address, std::string& host, int& port);

// A started program (argv[0] looked up in PATH), with extra environment
// variables; killed on destruction if it is still running.
class ChildProcess {
 public:
  ChildProcess(const std::vector<std::string>& argv, const std::vector<std::pair<std::string, std::string>>& env,
               const std::string& working_dir);
  ~ChildProcess();
  ChildProcess(const ChildProcess&) = delete;
  ChildProcess& operator=(const ChildProcess&) = delete;

  // True while it runs; its exit code once it has ended.
  bool running(int* exit_code = nullptr);
  // Waits up to timeout_s for it to end; kills it otherwise.
  void finish(double timeout_s);

 private:
  std::intptr_t handle_ = 0;
  bool done_ = false;
  int code_ = 0;
};

}  // namespace vsim
