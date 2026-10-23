#include "vsim/can/device.hpp"

#include <cstring>
#include <map>
#include <stdexcept>

#include "../plugin/plugin_hosts.hpp"
#include "../util/tcp.hpp"
#include "vsim/util/paths.hpp"

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <fcntl.h>
#endif

namespace vsim {

std::vector<std::uint8_t> encode_udp_frames(const std::vector<CanFrame>& frames) {
  std::vector<std::uint8_t> out;
  for (const CanFrame& f : frames) {
    out.push_back(static_cast<std::uint8_t>((f.extended ? 1 : 0) | (f.fd ? 2 : 0) | (f.fd && f.brs ? 4 : 0)));
    for (int i = 0; i < 4; ++i) {
      out.push_back(static_cast<std::uint8_t>(f.id >> (8 * i)));
    }
    out.push_back(static_cast<std::uint8_t>(f.data.size()));
    out.insert(out.end(), f.data.begin(), f.data.end());
  }
  return out;
}

std::vector<CanFrame> decode_udp_frames(const std::vector<std::uint8_t>& d) {
  std::vector<CanFrame> frames;
  std::size_t p = 0;
  while (p + 6 <= d.size()) {
    CanFrame f;
    const std::uint8_t flags = d[p];
    f.extended = (flags & 1) != 0;
    f.fd = (flags & 2) != 0;
    f.brs = (flags & 4) != 0;
    f.id = static_cast<std::uint32_t>(d[p + 1]) | (static_cast<std::uint32_t>(d[p + 2]) << 8) |
           (static_cast<std::uint32_t>(d[p + 3]) << 16) | (static_cast<std::uint32_t>(d[p + 4]) << 24);
    const std::size_t length = d[p + 5];
    p += 6;
    if (p + length > d.size() || length > 64) {
      break;  // a malformed tail
    }
    f.data.assign(d.begin() + static_cast<std::ptrdiff_t>(p), d.begin() + static_cast<std::ptrdiff_t>(p + length));
    p += length;
    frames.push_back(std::move(f));
  }
  return frames;
}

namespace {

// --- UDP ---------------------------------------------------------------------------------------
class UdpDevice final : public CanDevice {
 public:
  UdpDevice(const ParamTree& params, const std::string& p) {
    std::string host;
    int port = 0;
    split_address(params.get_string(p + ".local", "127.0.0.1:0"), host, port);
    socket_ = std::make_unique<UdpSocket>(host, port);
    split_address(params.get_string(p + ".remote"), remote_host_, remote_port_);
    describe_ = "udp " + host + ":" + std::to_string(socket_->port()) + " -> " + remote_host_ + ":" +
                std::to_string(remote_port_);
  }
  void send(const CanFrame& frame) override {
    const std::vector<std::uint8_t> d = encode_udp_frames({frame});
    socket_->send_to(remote_host_, remote_port_, d.data(), d.size());
  }
  std::vector<CanFrame> receive() override {
    std::vector<CanFrame> out;
    for (std::vector<std::uint8_t> d = socket_->receive(); !d.empty(); d = socket_->receive()) {
      for (CanFrame& f : decode_udp_frames(d)) {
        out.push_back(std::move(f));
      }
    }
    return out;
  }
  std::string describe() const override { return describe_; }

 private:
  std::unique_ptr<UdpSocket> socket_;
  std::string remote_host_, describe_;
  int remote_port_ = 0;
};

#ifdef __linux__
// --- SocketCAN (Linux) -------------------------------------------------------------------------
class SocketCanDevice final : public CanDevice {
 public:
  SocketCanDevice(const ParamTree& params, const std::string& p) {
    channel_ = params.get_string(p + ".channel", "can0");
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
      throw std::runtime_error(p + ": cannot open a CAN socket (" + std::strerror(errno) + ")");
    }
    int on = 1;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof on);  // FD frames too, if the interface can
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      ::close(fd_);
      throw std::runtime_error(p + ": no CAN interface '" + channel_ + "' (ip link set " + channel_ + " up?)");
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
      ::close(fd_);
      throw std::runtime_error(p + ": cannot bind to " + channel_ + " (" + std::strerror(errno) + ")");
    }
    fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  }
  ~SocketCanDevice() override { ::close(fd_); }
  void send(const CanFrame& f) override {
    canfd_frame frame{};
    frame.can_id = f.id | (f.extended ? CAN_EFF_FLAG : 0u);
    frame.len = static_cast<std::uint8_t>(std::min<std::size_t>(f.data.size(), f.fd ? 64 : 8));
    std::memcpy(frame.data, f.data.data(), frame.len);
    if (f.fd) {
      frame.flags = f.brs ? CANFD_BRS : 0;
    }
    if (::write(fd_, &frame, f.fd ? CANFD_MTU : CAN_MTU) < 0) {
      ++dropped_;  // transmit queue full or the bus is off: the frame is lost, as on the car
    }
  }
  std::vector<CanFrame> receive() override {
    std::vector<CanFrame> out;
    for (;;) {
      canfd_frame frame{};
      const ssize_t n = ::read(fd_, &frame, sizeof frame);
      if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU)) {
        break;
      }
      if ((frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) != 0) {
        continue;
      }
      CanFrame f;
      f.extended = (frame.can_id & CAN_EFF_FLAG) != 0;
      f.id = frame.can_id & (f.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
      f.fd = n == static_cast<ssize_t>(CANFD_MTU);
      f.brs = f.fd && (frame.flags & CANFD_BRS) != 0;
      f.data.assign(frame.data, frame.data + frame.len);
      out.push_back(std::move(f));
    }
    return out;
  }
  std::string describe() const override {
    return "socketcan " + channel_ + (dropped_ ? " (" + std::to_string(dropped_) + " frames not sent)" : "");
  }

 private:
  int fd_ = -1;
  long dropped_ = 0;
  std::string channel_;
};
#endif

// --- PEAK PCAN-Basic (classic CAN) -------------------------------------------------------------
struct PcanMsg {  // TPCANMsg
  std::uint32_t id;
  std::uint8_t type;
  std::uint8_t len;
  std::uint8_t data[8];
};
struct PcanTimestamp {  // TPCANTimestamp
  std::uint32_t millis;
  std::uint16_t millis_overflow;
  std::uint16_t micros;
};
#ifdef _WIN32
#define VSIM_CAN_API __stdcall
#else
#define VSIM_CAN_API
#endif

class PcanDevice final : public CanDevice {
 public:
  PcanDevice(const ParamTree& params, const std::string& p) {
#ifdef _WIN32
    const std::string default_library = "PCANBasic.dll";
#else
    const std::string default_library = "libpcanbasic.so";
#endif
    library_ = std::make_unique<DynamicLibrary>(path_from_utf8(params.get_string(p + ".library", default_library)));
    initialize_ = reinterpret_cast<decltype(initialize_)>(symbol("CAN_Initialize"));
    uninitialize_ = reinterpret_cast<decltype(uninitialize_)>(symbol("CAN_Uninitialize"));
    read_ = reinterpret_cast<decltype(read_)>(symbol("CAN_Read"));
    write_ = reinterpret_cast<decltype(write_)>(symbol("CAN_Write"));
    const std::string channel = params.get_string(p + ".channel", "PCAN_USBBUS1");
    channel_ = channel_handle(channel, p);
    const auto bitrate = static_cast<long>(params.get_double(p + ".bitrate", 500e3));
    static const std::map<long, std::uint16_t> btr = {{1000000, 0x0014}, {800000, 0x0016}, {500000, 0x001C},
                                                      {250000, 0x011C},  {125000, 0x031C}, {100000, 0x432F},
                                                      {50000, 0x472F},   {20000, 0x532F},  {10000, 0x672F}};
    const auto it = btr.find(bitrate);
    if (it == btr.end()) {
      throw std::invalid_argument(p + ".bitrate: PCAN-Basic takes 10k, 20k, 50k, 100k, 125k, 250k, 500k, 800k or 1M");
    }
    const std::uint32_t status = initialize_(channel_, it->second, 0, 0, 0);
    if (status != 0) {
      throw std::runtime_error(p + ": CAN_Initialize(" + channel + ") failed, status 0x" + hex(status));
    }
    describe_ = "pcan " + channel;
  }
  ~PcanDevice() override { uninitialize_(channel_); }
  void send(const CanFrame& f) override {
    PcanMsg m{};
    m.id = f.id;
    m.type = f.extended ? 0x02 : 0x00;
    m.len = static_cast<std::uint8_t>(std::min<std::size_t>(f.data.size(), 8));
    std::memcpy(m.data, f.data.data(), m.len);
    write_(channel_, &m);
  }
  std::vector<CanFrame> receive() override {
    std::vector<CanFrame> out;
    for (int k = 0; k < 1000; ++k) {
      PcanMsg m{};
      PcanTimestamp ts{};
      if (read_(channel_, &m, &ts) != 0) {
        break;  // queue empty (or an error: nothing to read)
      }
      if ((m.type & 0x81) != 0) {
        continue;  // status or remote frame
      }
      CanFrame f;
      f.id = m.id;
      f.extended = (m.type & 0x02) != 0;
      f.data.assign(m.data, m.data + std::min<int>(m.len, 8));
      out.push_back(std::move(f));
    }
    return out;
  }
  std::string describe() const override { return describe_; }

 private:
  void* symbol(const char* name) {
    void* s = library_->symbol(name);
    if (s == nullptr) {
      throw std::runtime_error(path_to_utf8(library_->file()) + " has no " + name);
    }
    return s;
  }
  static std::string hex(std::uint32_t v) {
    char b[16];
    std::snprintf(b, sizeof b, "%X", v);
    return b;
  }
  static std::uint16_t channel_handle(const std::string& name, const std::string& p) {
    // PCAN_USBBUS1..8 = 0x51..0x58, 9..16 = 0x509..0x510; PCAN_PCIBUS1..8 = 0x41..0x48; or a number.
    const auto number = [&](const std::string& prefix) {
      return name.rfind(prefix, 0) == 0 ? std::stoi(name.substr(prefix.size())) : 0;
    };
    if (const int n = number("PCAN_USBBUS"); n >= 1 && n <= 16) {
      return static_cast<std::uint16_t>(n <= 8 ? 0x50 + n : 0x500 + n);
    }
    if (const int n = number("PCAN_PCIBUS"); n >= 1 && n <= 8) {
      return static_cast<std::uint16_t>(0x40 + n);
    }
    try {
      return static_cast<std::uint16_t>(std::stoul(name, nullptr, 0));
    } catch (const std::exception&) {
      throw std::invalid_argument(p + ".channel: PCAN_USBBUSn, PCAN_PCIBUSn or a handle number, not '" + name + "'");
    }
  }

  std::unique_ptr<DynamicLibrary> library_;
  std::uint32_t(VSIM_CAN_API* initialize_)(std::uint16_t, std::uint16_t, std::uint8_t, std::uint32_t,
                                           std::uint16_t) = nullptr;
  std::uint32_t(VSIM_CAN_API* uninitialize_)(std::uint16_t) = nullptr;
  std::uint32_t(VSIM_CAN_API* read_)(std::uint16_t, PcanMsg*, PcanTimestamp*) = nullptr;
  std::uint32_t(VSIM_CAN_API* write_)(std::uint16_t, PcanMsg*) = nullptr;
  std::uint16_t channel_ = 0;
  std::string describe_;
};

// --- Kvaser CANlib -----------------------------------------------------------------------------
class KvaserDevice final : public CanDevice {
 public:
  KvaserDevice(const ParamTree& params, const std::string& p) {
#ifdef _WIN32
    const std::string default_library = "canlib32.dll";
#else
    const std::string default_library = "libcanlib.so.1";
#endif
    library_ = std::make_unique<DynamicLibrary>(path_from_utf8(params.get_string(p + ".library", default_library)));
    const auto get = [&](const char* name) {
      void* s = library_->symbol(name);
      if (s == nullptr) {
        throw std::runtime_error(path_to_utf8(library_->file()) + " has no " + name);
      }
      return s;
    };
    auto init = reinterpret_cast<void(VSIM_CAN_API*)()>(get("canInitializeLibrary"));
    auto open = reinterpret_cast<int(VSIM_CAN_API*)(int, int)>(get("canOpenChannel"));
    auto bus_params = reinterpret_cast<int(VSIM_CAN_API*)(int, long, unsigned, unsigned, unsigned, unsigned, unsigned)>(
        get("canSetBusParams"));
    auto bus_on = reinterpret_cast<int(VSIM_CAN_API*)(int)>(get("canBusOn"));
    bus_off_ = reinterpret_cast<decltype(bus_off_)>(get("canBusOff"));
    close_ = reinterpret_cast<decltype(close_)>(get("canClose"));
    write_ = reinterpret_cast<decltype(write_)>(get("canWrite"));
    read_ = reinterpret_cast<decltype(read_)>(get("canRead"));
    init();
    const int channel = static_cast<int>(params.get_int(p + ".channel", 0));
    handle_ = open(channel, 0x0020);  // canOPEN_ACCEPT_VIRTUAL
    if (handle_ < 0) {
      throw std::runtime_error(p + ": canOpenChannel(" + std::to_string(channel) + ") failed, status " +
                               std::to_string(handle_));
    }
    const auto bitrate = static_cast<long>(params.get_double(p + ".bitrate", 500e3));
    static const std::map<long, long> predefined = {{1000000, -1}, {500000, -2}, {250000, -3}, {125000, -4},
                                                    {100000, -5},  {62000, -6},  {50000, -7},  {83000, -8},
                                                    {10000, -9}};
    const auto it = predefined.find(bitrate);
    if (it == predefined.end() || bus_params(handle_, it->second, 0, 0, 0, 0, 0) != 0 || bus_on(handle_) != 0) {
      close_(handle_);
      throw std::runtime_error(p + ": cannot go bus-on at " + std::to_string(bitrate) + " bit/s");
    }
    describe_ = "kvaser " + std::to_string(channel);
  }
  ~KvaserDevice() override {
    bus_off_(handle_);
    close_(handle_);
  }
  void send(const CanFrame& f) override {
    unsigned flags = f.extended ? 0x0004 : 0x0002;  // canMSG_EXT / canMSG_STD
    if (f.fd) {
      flags |= 0x010000 | (f.brs ? 0x020000 : 0);  // canFDMSG_FDF, canFDMSG_BRS
    }
    std::vector<std::uint8_t> data = f.data;
    write_(handle_, static_cast<long>(f.id), data.data(), static_cast<unsigned>(data.size()), flags);
  }
  std::vector<CanFrame> receive() override {
    std::vector<CanFrame> out;
    for (int k = 0; k < 1000; ++k) {
      long id = 0;
      std::uint8_t data[64] = {};
      unsigned dlc = 0, flags = 0;
      unsigned long time = 0;
      if (read_(handle_, &id, data, &dlc, &flags, &time) != 0) {
        break;  // canERR_NOMSG
      }
      if ((flags & (0x0020 | 0x0001)) != 0) {
        continue;  // error or remote frame
      }
      CanFrame f;
      f.id = static_cast<std::uint32_t>(id);
      f.extended = (flags & 0x0004) != 0;
      f.fd = (flags & 0x010000) != 0;
      f.brs = (flags & 0x020000) != 0;
      f.data.assign(data, data + std::min<unsigned>(dlc, f.fd ? 64u : 8u));
      out.push_back(std::move(f));
    }
    return out;
  }
  std::string describe() const override { return describe_; }

 private:
  std::unique_ptr<DynamicLibrary> library_;
  int(VSIM_CAN_API* bus_off_)(int) = nullptr;
  int(VSIM_CAN_API* close_)(int) = nullptr;
  int(VSIM_CAN_API* write_)(int, long, void*, unsigned, unsigned) = nullptr;
  int(VSIM_CAN_API* read_)(int, long*, void*, unsigned*, unsigned*, unsigned long*) = nullptr;
  int handle_ = -1;
  std::string describe_;
};

}  // namespace

std::unique_ptr<CanDevice> open_can_device(const ParamTree& params, const std::string& prefix) {
  const std::string type = params.get_string(prefix + ".type");
  if (type == "udp") {
    return std::make_unique<UdpDevice>(params, prefix);
  }
  if (type == "socketcan") {
#ifdef __linux__
    return std::make_unique<SocketCanDevice>(params, prefix);
#else
    throw std::invalid_argument(prefix + ".type: socketcan is Linux only (pcan or kvaser on Windows)");
#endif
  }
  if (type == "pcan") {
    return std::make_unique<PcanDevice>(params, prefix);
  }
  if (type == "kvaser") {
    return std::make_unique<KvaserDevice>(params, prefix);
  }
  throw std::invalid_argument(prefix + ".type '" + type + "' (expected socketcan, pcan, kvaser or udp)");
}

}  // namespace vsim
