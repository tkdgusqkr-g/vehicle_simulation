// External components (design doc §11, §12.5): a separate program driven in
// lockstep over TCP, sdk/c/include/vsim_external.h describes the protocol.

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>

#include "../util/tcp.hpp"
#include "plugin_hosts.hpp"
#include "vsim/plugin/external.hpp"
#include "vsim/util/paths.hpp"
#include "vsim_external.h"

namespace vsim {
namespace {

using Json = nlohmann::json;

std::mutex g_endpoints_mutex;
std::map<std::string, ExternalEndpoint*> g_endpoints;

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
  }
}
void put_f64(std::vector<std::uint8_t>& out, double v) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, 8);
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<std::uint8_t>(bits >> (8 * i)));
  }
}

class Reader {
 public:
  Reader(const std::vector<std::uint8_t>& data, std::string what) : d_(data), what_(std::move(what)) {}
  std::uint32_t u32() {
    need(4);
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      v |= static_cast<std::uint32_t>(d_[p_ + static_cast<std::size_t>(i)]) << (8 * i);
    }
    p_ += 4;
    return v;
  }
  std::uint8_t u8() {
    need(1);
    return d_[p_++];
  }
  double f64() {
    need(8);
    std::uint64_t bits = 0;
    for (int i = 0; i < 8; ++i) {
      bits |= static_cast<std::uint64_t>(d_[p_ + static_cast<std::size_t>(i)]) << (8 * i);
    }
    p_ += 8;
    double v = 0.0;
    std::memcpy(&v, &bits, 8);
    return v;
  }
  std::vector<std::uint8_t> bytes(std::size_t n) {
    need(n);
    std::vector<std::uint8_t> out(d_.begin() + static_cast<std::ptrdiff_t>(p_),
                                  d_.begin() + static_cast<std::ptrdiff_t>(p_ + n));
    p_ += n;
    return out;
  }

 private:
  void need(std::size_t n) const {
    if (p_ + n > d_.size()) {
      throw std::runtime_error(what_ + ": message too short");
    }
  }
  const std::vector<std::uint8_t>& d_;
  std::size_t p_ = 0;
  std::string what_;
};

class ExternalPlugin final : public PluginInstance, public ExternalEndpoint {
 public:
  ExternalPlugin(const ParamTree& params, const ComponentSpec& spec, double period) : name_(spec.name) {
    const std::string p = spec.prefix + ".";
    const std::string listen = params.get_string(p + "listen", "127.0.0.1:0");
    const auto colon = listen.rfind(':');
    const std::string host = colon == std::string::npos ? listen : listen.substr(0, colon);
    const int listen_port = colon == std::string::npos ? 0 : std::stoi(listen.substr(colon + 1));
    timeout_ = params.get_double(p + "timeout_s", 10.0);
    const double connect_timeout = params.get_double(p + "connect_timeout_s", 30.0);
    buses_ = params.get_string_list(p + "can");
    period_ = period > 0.0 ? period : params.get_double(p + "period", 0.01);
    if (!(period_ > 0.0) || !(timeout_ > 0.0)) {
      throw std::invalid_argument(spec.prefix + ": period and timeout_s must be positive");
    }
    listener_ = std::make_unique<TcpListener>(host, listen_port);
    const std::string address = host + ":" + std::to_string(listener_->port());
    const std::vector<std::string> command = params.get_string_list(p + "command");
    if (!command.empty()) {
      child_ = std::make_unique<ChildProcess>(command, std::vector<std::pair<std::string, std::string>>{
                                                           {"VSIM_EXTERNAL", address}, {"VSIM_COMPONENT", name_}},
                                              path_to_utf8(spec.project_dir));
    } else {
      std::fprintf(stderr, "external component %s: waiting for its program on %s (VSIM_EXTERNAL=%s)\n",
                   name_.c_str(), address.c_str(), address.c_str());
    }
    // Wait for the connection, giving up early if the program it started has ended.
    for (double waited = 0.0;; waited += 0.2) {
      try {
        stream_ = listener_->accept(std::min(0.2, connect_timeout - waited));
        break;
      } catch (const std::exception& e) {
        int code = 0;
        if (child_ && !child_->running(&code)) {
          throw std::runtime_error("external component " + name_ + ": its program exited (code " +
                                   std::to_string(code) + ") before connecting");
        }
        if (waited + 0.2 >= connect_timeout) {
          throw std::runtime_error("external component " + name_ + ": " + e.what());
        }
      }
    }
    // HELLO: who it is and its ports.
    const auto [type, hello] = read_message(connect_timeout);
    if (type != VSIM_EXT_HELLO) {
      throw std::runtime_error("external component " + name_ + ": expected HELLO, got message type " +
                               std::to_string(type));
    }
    Json h;
    try {
      h = Json::parse(std::string(hello.begin(), hello.end()));
    } catch (const std::exception& e) {
      throw std::runtime_error("external component " + name_ + ": HELLO is not JSON (" + e.what() + ")");
    }
    if (h.value("protocol", 0) != VSIM_EXTERNAL_PROTOCOL) {
      throw std::runtime_error("external component " + name_ + ": protocol " + std::to_string(h.value("protocol", 0)) +
                               ", this vsim speaks " + std::to_string(VSIM_EXTERNAL_PROTOCOL));
    }
    std::vector<PortInfo> in, out;
    for (const Json& port : h.value("ports", Json::array())) {
      PortInfo info{port.value("name", ""), port.value("unit", ""), port.value("dir", "in") == "in"};
      if (info.name.empty()) {
        throw std::runtime_error("external component " + name_ + ": a port without a name");
      }
      (info.input ? in : out).push_back(info);
    }
    ports_ = in;
    ports_.insert(ports_.end(), out.begin(), out.end());
    inputs_ = in.size();
    outputs_ = out.size();
    info_ = {"external", stream_->peer(), h.value("name", ""), h.value("version", "")};
    // SETUP: period, params, port order, buses.
    Json setup = {{"protocol", VSIM_EXTERNAL_PROTOCOL}, {"component", name_}, {"period", period_},
                  {"params", Json::parse(component_params_json(params, spec))}, {"buses", buses_}};
    setup["inputs"] = Json::array();
    setup["outputs"] = Json::array();
    for (const PortInfo& port : ports_) {
      setup[port.input ? "inputs" : "outputs"].push_back(port.name);
    }
    const std::string text = setup.dump();
    send_message(VSIM_EXT_SETUP, std::vector<std::uint8_t>(text.begin(), text.end()));
    if (!buses_.empty()) {
      std::lock_guard<std::mutex> lock(g_endpoints_mutex);
      g_endpoints[name_] = this;
    }
  }

  ~ExternalPlugin() override {
    {
      std::lock_guard<std::mutex> lock(g_endpoints_mutex);
      if (const auto it = g_endpoints.find(name_); it != g_endpoints.end() && it->second == this) {
        g_endpoints.erase(it);
      }
    }
    try {
      if (stream_) {
        send_message(VSIM_EXT_END, {});
      }
    } catch (const std::exception&) {
      // the program already went away
    }
    stream_.reset();
    if (child_) {
      child_->finish(5.0);
    }
  }

  const std::vector<PortInfo>& ports() const override { return ports_; }
  double period() const override { return period_; }
  Info info() const override { return info_; }

  void step(double t, const double* in, double* out) override {
    std::vector<std::uint8_t> m;
    m.reserve(16 + 8 * inputs_);
    put_f64(m, t);
    put_u32(m, static_cast<std::uint32_t>(inputs_));
    for (std::size_t i = 0; i < inputs_; ++i) {
      put_f64(m, in[i]);
    }
    std::vector<ExternalFrame> rx;
    {
      std::lock_guard<std::mutex> lock(frames_mutex_);
      rx.swap(rx_);
    }
    put_u32(m, static_cast<std::uint32_t>(rx.size()));
    for (const ExternalFrame& f : rx) {
      m.push_back(static_cast<std::uint8_t>(f.bus));
      m.push_back(static_cast<std::uint8_t>((f.extended ? VSIM_FRAME_EXTENDED : 0) | (f.fd ? VSIM_FRAME_FD : 0) |
                                            (f.brs ? VSIM_FRAME_BRS : 0)));
      put_u32(m, f.id);
      m.push_back(static_cast<std::uint8_t>(f.data.size()));
      m.insert(m.end(), f.data.begin(), f.data.end());
    }
    send_message(VSIM_EXT_STEP, m);
    const auto [type, reply] = read_message(timeout_);
    if (type != VSIM_EXT_RESULT) {
      throw std::runtime_error("external component " + name_ + ": expected RESULT at t = " + std::to_string(t) +
                               " s, got message type " + std::to_string(type));
    }
    Reader r(reply, "external component " + name_ + " RESULT");
    const std::uint32_t n_out = r.u32();
    if (n_out != outputs_) {
      throw std::runtime_error("external component " + name_ + ": " + std::to_string(n_out) + " outputs, declared " +
                               std::to_string(outputs_));
    }
    for (std::size_t i = 0; i < outputs_; ++i) {
      out[i] = r.f64();
    }
    const std::uint32_t frames = r.u32();
    std::vector<ExternalFrame> sent;
    for (std::uint32_t k = 0; k < frames; ++k) {
      ExternalFrame f;
      f.bus = r.u8();
      const std::uint8_t flags = r.u8();
      f.extended = (flags & VSIM_FRAME_EXTENDED) != 0;
      f.fd = (flags & VSIM_FRAME_FD) != 0;
      f.brs = (flags & VSIM_FRAME_BRS) != 0;
      f.id = r.u32();
      const std::uint8_t length = r.u8();
      if (length > 64 || (!f.fd && length > 8)) {
        throw std::runtime_error("external component " + name_ + ": frame of " + std::to_string(length) + " bytes");
      }
      f.data = r.bytes(length);
      f.time = t;
      if (f.bus >= buses_.size()) {
        throw std::runtime_error("external component " + name_ + ": frame on bus " + std::to_string(f.bus) +
                                 ", it is on " + std::to_string(buses_.size()));
      }
      sent.push_back(std::move(f));
    }
    std::lock_guard<std::mutex> lock(frames_mutex_);
    tx_.insert(tx_.end(), sent.begin(), sent.end());
  }

  const std::vector<std::string>& buses() const override { return buses_; }
  void receive(const ExternalFrame& frame) override {
    std::lock_guard<std::mutex> lock(frames_mutex_);
    rx_.push_back(frame);
  }
  std::vector<ExternalFrame> take_sent() override {
    std::lock_guard<std::mutex> lock(frames_mutex_);
    std::vector<ExternalFrame> out;
    out.swap(tx_);
    return out;
  }

 private:
  void send_message(int type, const std::vector<std::uint8_t>& payload) {
    std::vector<std::uint8_t> m;
    put_u32(m, static_cast<std::uint32_t>(payload.size() + 1));
    m.push_back(static_cast<std::uint8_t>(type));
    m.insert(m.end(), payload.begin(), payload.end());
    stream_->send_all(m.data(), m.size());
  }

  std::pair<int, std::vector<std::uint8_t>> read_message(double timeout) {
    std::uint8_t head[5];
    try {
      stream_->receive_all(head, 5, timeout);
      const std::uint32_t length = static_cast<std::uint32_t>(head[0]) | (static_cast<std::uint32_t>(head[1]) << 8) |
                                   (static_cast<std::uint32_t>(head[2]) << 16) |
                                   (static_cast<std::uint32_t>(head[3]) << 24);
      if (length < 1 || length > (1u << 24)) {
        throw std::runtime_error("bad message length " + std::to_string(length));
      }
      std::vector<std::uint8_t> payload(length - 1);
      if (!payload.empty()) {
        stream_->receive_all(payload.data(), payload.size(), timeout);
      }
      return {head[4], std::move(payload)};
    } catch (const std::exception& e) {
      throw std::runtime_error("external component " + name_ + ": " + e.what());
    }
  }

  std::string name_;
  double period_ = 0.01, timeout_ = 10.0;
  std::vector<std::string> buses_;
  std::unique_ptr<TcpListener> listener_;
  std::unique_ptr<ChildProcess> child_;
  std::unique_ptr<TcpStream> stream_;
  std::vector<PortInfo> ports_;
  std::size_t inputs_ = 0, outputs_ = 0;
  Info info_;
  std::mutex frames_mutex_;
  std::vector<ExternalFrame> rx_, tx_;
};

}  // namespace

std::unique_ptr<PluginInstance> make_external_plugin(const ParamTree& params, const ComponentSpec& spec,
                                                     double period) {
  return std::make_unique<ExternalPlugin>(params, spec, period);
}

ExternalEndpoint* find_external_endpoint(const std::string& component) {
  std::lock_guard<std::mutex> lock(g_endpoints_mutex);
  const auto it = g_endpoints.find(component);
  return it == g_endpoints.end() ? nullptr : it->second;
}

}  // namespace vsim
