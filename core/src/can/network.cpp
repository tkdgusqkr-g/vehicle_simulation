#include "vsim/can/network.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>

#include "vsim/can/dbc.hpp"
#include "vsim/can/frame.hpp"
#include "vsim/plugin/units.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-12;

struct Node {
  std::string name;
  bool controller = false;
  double ppm = 0.0, jitter = 0.0, offset = 0.0, task = 0.0, recovery = 0.1;
  std::optional<Rng> rng;
  int tec = 0;
  bool bus_off = false;
  double bus_off_until = 0.0;
  std::int64_t bus_off_count = 0;
  int tec_max = 0;
  std::int64_t task_k = 0;
  double next_task = kInf;
  SignalId tec_out = 0, bus_off_out = 0;
  // Supply from the electrical network (elec.can_node.<name>.on): without it
  // the node is silent; at power-up its schedule restarts from that moment.
  std::optional<SignalId> power_in;
  bool powered = true;
  double epoch = 0.0;  // node-clock time its schedule counts from
};

struct SignalMap {
  std::optional<SignalId> bus_signal;  // sampled by the sender, written at the receivers
  UnitConversion to_dbc, from_dbc;
  SignalId decoded = 0;  // can.<msg>.<signal>
  double held = 0.0;     // last received, DBC units
};

struct Message {
  DbcMessage def;
  std::string label;  // signal prefix: can.<label>.
  std::size_t bus = 0;
  std::optional<std::size_t> node;
  bool from_controller = false;
  bool generated = false;  // extra traffic (can.load)
  double cycle = 0.0;
  std::int64_t k = 0;
  double next_request = kInf;
  std::vector<SignalMap> map;
  std::vector<std::int64_t> mux_values;
  std::size_t mux_next = 0;
  double timeout = 0.0;
  bool zero_on_timeout = false;  // receivers fall back to 0 (an inverter without commands)
  double last_delivery = -kInf;
  double first_request = kInf;
  std::optional<std::size_t> task_node;
  std::deque<double> recent;  // delivery times not yet counted into a task window
  double window_start = 0.0;
  bool timed_out = false;
  std::int64_t sent = 0, delivered = 0, lost = 0, timeouts = 0, missed = 0, doubled = 0, skipped = 0;
  double lat_min = kInf, lat_max = 0.0, lat_sum = 0.0;
  SignalId latency_out = 0, age_out = 0, timeout_out = 0, lost_out = 0, missed_out = 0, doubled_out = 0;
};

struct Pending {
  std::size_t message;
  double ready, request;
  CanFrame frame;
  std::uint64_t key, seq;
};

struct Delivery {
  std::size_t message;
  double at, request;
  std::vector<std::uint8_t> data;
};

struct Bus {
  std::string name;
  double bitrate = 500e3, data_bitrate = 2e6;
  double free_at = 0.0;
  std::vector<Pending> pending;
  std::deque<std::pair<double, double>> busy;
  SignalId load_out = 0;
  double load_sum = 0.0, load_max = 0.0;
  std::int64_t load_samples = 0, frames = 0, errors = 0;
  std::unique_ptr<std::ofstream> asc, candump;
};

struct Fault {
  std::string type;
  std::optional<std::size_t> message, node, bus;
  double at = 0.0, until = kInf, probability = 1.0, value = 0.0, jitter = 0.0;
  bool active(double t) const { return t >= at && t < until; }
};

// Arbitration: lower wins. Base ID first, a standard frame beats an
// extended one with the same base (RTR/SRR), then the extended bits.
std::uint64_t priority_key(std::uint32_t id, bool extended) {
  const std::uint64_t base = extended ? (id >> 18) : id;
  return (base << 19) | (extended ? ((std::uint64_t{1} << 18) | (id & 0x3FFFFU)) : 0U);
}

std::string hex_bytes(const std::vector<std::uint8_t>& data, bool spaced) {
  std::string out;
  char b[4];
  for (const std::uint8_t v : data) {
    std::snprintf(b, sizeof b, spaced ? " %02X" : "%02X", v);
    out += b;
  }
  return out;
}

}  // namespace

struct CanNetwork::Impl {
  bool timing = false, faults = false;
  std::string fidelity;
  double dt = 0.001, load_window = 0.1;
  std::vector<Bus> buses;
  std::vector<Node> nodes;
  std::vector<Message> messages;
  std::vector<Fault> fault_list;
  std::vector<Delivery> deliveries;
  std::vector<std::pair<SignalId, double>> restore;
  std::optional<Rng> fault_rng;
  std::uint64_t seq = 0;
  std::string log_mode = "asc";

  std::size_t node_index(const std::string& name) {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (nodes[i].name == name) {
        return i;
      }
    }
    Node n;
    n.name = name;
    nodes.push_back(std::move(n));
    return nodes.size() - 1;
  }

  double clock_to_sim(const Node* node, double local) const {
    return timing && node != nullptr ? local / (1.0 + node->ppm * 1e-6) : local;
  }

  double draw_jitter(Node* node) {
    return timing && node != nullptr && node->jitter > 0.0 ? node->jitter * node->rng->uniform() : 0.0;
  }

  void schedule_next(Message& m) {
    Node* node = m.node ? &nodes[*m.node] : nullptr;
    const double local = (node ? node->epoch + node->offset : 0.0) + static_cast<double>(m.k) * m.cycle;
    m.next_request = clock_to_sim(node, local) + draw_jitter(node);
  }

  bool fault_hits(const std::string& type, const Message& m, double t, double* value = nullptr,
                  double* jitter = nullptr) {
    if (!faults) {
      return false;
    }
    bool hit = false;
    for (const Fault& f : fault_list) {
      if (f.type != type || !f.active(t)) {
        continue;
      }
      // Every filter the fault gives must match (none: all frames).
      const bool matches = (!f.message || messages[*f.message].def.name == m.def.name) &&
                           (!f.node || (m.node && *f.node == *m.node)) && (!f.bus || *f.bus == m.bus);
      if (matches && fault_rng->uniform() < f.probability) {
        hit = true;
        if (value != nullptr) {
          *value += f.value;
        }
        if (jitter != nullptr) {
          *jitter += f.jitter * fault_rng->uniform();
        }
      }
    }
    return hit;
  }

  bool forced_bus_off(std::size_t node, double t) const {
    if (!faults) {
      return false;
    }
    for (const Fault& f : fault_list) {
      if (f.type == "bus_off" && f.node && *f.node == node && f.active(t)) {
        return true;
      }
    }
    return false;
  }

  bool node_silent(std::size_t node, double t) {
    Node& n = nodes[node];
    if (n.bus_off && t >= n.bus_off_until) {
      n.bus_off = false;
      n.tec = 0;
    }
    return n.bus_off || !n.powered || forced_bus_off(node, t);
  }

  // Nodes losing power drop what they had queued; nodes getting it back boot
  // and start their message and task schedules afresh.
  void update_power(double t, const SignalBus& bus) {
    for (std::size_t ni = 0; ni < nodes.size(); ++ni) {
      Node& n = nodes[ni];
      if (!n.power_in) {
        continue;
      }
      const bool powered = bus.get(*n.power_in) > 0.5;
      if (powered == n.powered) {
        continue;
      }
      n.powered = powered;
      if (!powered) {
        for (Bus& b : buses) {
          std::erase_if(b.pending, [&](const Pending& p) {
            return messages[p.message].node && *messages[p.message].node == ni;
          });
        }
        continue;
      }
      n.epoch = timing ? t * (1.0 + n.ppm * 1e-6) : t;
      n.tec = 0;
      n.bus_off = false;
      for (Message& m : messages) {
        if (m.node && *m.node == ni && m.cycle > 0.0) {
          m.k = 0;
          schedule_next(m);
        }
      }
      if (n.task > 0.0) {
        n.task_k = 0;
        n.next_task = clock_to_sim(&n, n.epoch + n.offset) + draw_jitter(&n);
      }
    }
  }

  CanFrame build_frame(Message& m, const SignalBus& bus) {
    CanFrame f;
    f.id = m.def.id;
    f.extended = m.def.extended;
    f.fd = m.def.fd;
    f.data.assign(static_cast<std::size_t>(m.def.length), 0);
    std::optional<std::int64_t> mux;
    if (!m.mux_values.empty()) {
      mux = m.mux_values[m.mux_next];
      m.mux_next = (m.mux_next + 1) % m.mux_values.size();
    }
    for (std::size_t i = 0; i < m.def.signals.size(); ++i) {
      const DbcSignal& s = m.def.signals[i];
      if (s.mux_value && (!mux || *s.mux_value != *mux)) {
        continue;
      }
      if (s.multiplexer && !s.mux_value && mux) {
        s.write(f.data, s.decode(static_cast<std::uint64_t>(*mux)));
      } else if (m.map[i].bus_signal) {
        s.write(f.data, m.map[i].to_dbc.apply(bus.get(*m.map[i].bus_signal)));
      }
    }
    return f;
  }

  // Requests of the senders on one side due in [t, t + dt).
  void send(double t, const SignalBus& bus, bool controllers) {
    for (std::size_t mi = 0; mi < messages.size(); ++mi) {
      Message& m = messages[mi];
      if (m.from_controller != controllers || m.cycle <= 0.0) {
        continue;
      }
      while (m.next_request < t + dt - kEps) {
        const double request = std::max(m.next_request, 0.0);
        ++m.k;
        schedule_next(m);
        m.first_request = std::min(m.first_request, request);
        if ((m.node && node_silent(*m.node, request)) || fault_hits("skip", m, request)) {
          ++m.skipped;
          continue;
        }
        CanFrame frame = build_frame(m, bus);
        ++m.sent;
        if (!timing) {
          deliveries.push_back({mi, request, request, frame.data});
          continue;
        }
        buses[m.bus].pending.push_back({mi, request, request, std::move(frame),
                                        priority_key(m.def.id, m.def.extended), seq++});
      }
    }
  }

  void log_frame(Bus& b, std::size_t bus_index, const Message& m, const CanFrame& f, double at) {
    char id[16];
    std::snprintf(id, sizeof id, f.extended ? "%X" : "%03X", f.id);
    if (b.asc) {
      char line[96];
      if (f.fd) {
        std::snprintf(line, sizeof line, "%11.6f CANFD %3zu Rx %10s%s %-32s %d 0 %X %2zu", at, bus_index + 1, id,
                      f.extended ? "x" : " ", m.def.name.c_str(), f.brs ? 1 : 0,
                      static_cast<unsigned>(length_to_dlc(static_cast<int>(f.data.size()))), f.data.size());
      } else {
        std::snprintf(line, sizeof line, "%11.6f %zu  %-15s Rx   d %zu", at, bus_index + 1,
                      (std::string(id) + (f.extended ? "x" : "")).c_str(), f.data.size());
      }
      *b.asc << line << hex_bytes(f.data, true) << "\n";
    }
    if (b.candump) {
      char line[64];
      std::snprintf(line, sizeof line, "(%.6f) %s %s%s", at, b.name.c_str(), id, f.fd ? (f.brs ? "##1" : "##0") : "#");
      *b.candump << line << hex_bytes(f.data, false) << "\n";
    }
  }

  // The bus over [t, t + dt): arbitration, transmission, errors.
  void advance(double t) {
    const double end = t + dt;
    for (std::size_t bi = 0; bi < buses.size(); ++bi) {
      Bus& b = buses[bi];
      for (;;) {
        if (b.pending.empty()) {
          break;
        }
        double earliest = kInf;
        for (const Pending& p : b.pending) {
          earliest = std::min(earliest, p.ready);
        }
        const double start = std::max(b.free_at, earliest);
        if (start >= end) {
          break;
        }
        std::size_t best = b.pending.size();
        for (std::size_t i = 0; i < b.pending.size(); ++i) {
          const Pending& p = b.pending[i];
          if (p.ready <= start + kEps &&
              (best == b.pending.size() || p.key < b.pending[best].key ||
               (p.key == b.pending[best].key && p.seq < b.pending[best].seq))) {
            best = i;
          }
        }
        Pending p = b.pending[best];
        Message& m = messages[p.message];
        if (m.node && node_silent(*m.node, start)) {
          ++m.lost;
          b.pending.erase(b.pending.begin() + static_cast<std::ptrdiff_t>(best));
          continue;
        }
        const double duration = frame_duration(p.frame, b.bitrate, b.data_bitrate);
        if (fault_hits("error_rate", m, start)) {
          // Error frame part-way through; the frame stays queued for a retry.
          const double busy = fault_rng->uniform() * duration + 17.0 / b.bitrate;
          b.busy.emplace_back(start, start + busy);
          b.free_at = start + busy;
          ++b.errors;
          if (m.node) {
            Node& n = nodes[*m.node];
            n.tec += 8;
            n.tec_max = std::max(n.tec_max, n.tec);
            if (n.tec >= 256) {
              n.bus_off = true;
              n.bus_off_until = start + busy + n.recovery;
              ++n.bus_off_count;
              // Its queued frames never go out.
              for (auto it = b.pending.begin(); it != b.pending.end();) {
                if (messages[it->message].node == m.node) {
                  ++messages[it->message].lost;
                  it = b.pending.erase(it);
                } else {
                  ++it;
                }
              }
            }
          }
          continue;
        }
        b.pending.erase(b.pending.begin() + static_cast<std::ptrdiff_t>(best));
        b.free_at = start + duration;
        b.busy.emplace_back(start, start + duration);
        ++b.frames;
        if (m.node) {
          Node& n = nodes[*m.node];
          n.tec = std::max(0, n.tec - 1);
        }
        log_frame(b, bi, m, p.frame, start + duration);
        double delay = 0.0, jitter = 0.0;
        fault_hits("delay", m, start, &delay, &jitter);
        if (fault_hits("drop", m, start)) {
          ++m.lost;
          continue;
        }
        deliveries.push_back({p.message, start + duration + delay + jitter, p.request, std::move(p.frame.data)});
      }
      // Load over the last window.
      while (!b.busy.empty() && b.busy.front().second < end - load_window) {
        b.busy.pop_front();
      }
    }
  }

  void deliver(double t, SignalBus& bus, bool to_controllers) {
    std::stable_sort(deliveries.begin(), deliveries.end(),
                     [](const Delivery& a, const Delivery& b) { return a.at < b.at; });
    std::vector<Delivery> keep;
    for (Delivery& d : deliveries) {
      Message& m = messages[d.message];
      if (m.from_controller == to_controllers || d.at > t + dt * 0.5) {
        keep.push_back(std::move(d));
        continue;
      }
      std::optional<std::int64_t> mux;
      if (const DbcSignal* ms = m.def.multiplexer(); ms != nullptr && !ms->mux_value) {
        mux = static_cast<std::int64_t>(ms->read_raw(d.data));
      }
      for (std::size_t i = 0; i < m.def.signals.size(); ++i) {
        const DbcSignal& s = m.def.signals[i];
        if (s.mux_value && (!mux || *s.mux_value != *mux)) {
          continue;
        }
        const double value = s.read(d.data);
        bus.set(m.map[i].decoded, value);
        m.map[i].held = value;
      }
      ++m.delivered;
      const double latency = d.at - d.request;
      m.lat_min = std::min(m.lat_min, latency);
      m.lat_max = std::max(m.lat_max, latency);
      m.lat_sum += latency;
      m.last_delivery = std::max(m.last_delivery, d.at);
      bus.set(m.latency_out, latency);
      if (m.task_node) {
        m.recent.push_back(d.at);
      }
    }
    deliveries = std::move(keep);
  }

  // Received values stand in for the mapped signals until the plant has read them.
  void apply(double t, SignalBus& bus, bool to_controllers) {
    for (Message& m : messages) {
      if (m.from_controller == to_controllers) {
        continue;
      }
      if (m.cycle > 0.0 && t >= m.first_request) {
        const double age = t - std::max(m.last_delivery, m.first_request);
        bus.set(m.age_out, age);
        const bool out = m.timeout > 0.0 && age > m.timeout;
        if (out && !m.timed_out) {
          ++m.timeouts;
        }
        m.timed_out = out;
        bus.set(m.timeout_out, out ? 1.0 : 0.0);
      }
      bus.set(m.lost_out, static_cast<double>(m.lost));
      for (const SignalMap& s : m.map) {
        if (s.bus_signal) {
          restore.emplace_back(*s.bus_signal, bus.get(*s.bus_signal));
          bus.set(*s.bus_signal, m.timed_out && m.zero_on_timeout ? 0.0 : s.from_dbc.apply(s.held));
        }
      }
    }
  }

  // Receiver tasks (on the receiving controller's clock): how many new frames
  // each task run finds.
  void tasks(double t, SignalBus& bus) {
    for (std::size_t ni = 0; ni < nodes.size(); ++ni) {
      Node& n = nodes[ni];
      if (!n.powered) {
        continue;  // in reset: no task runs (restarted at power-up)
      }
      while (n.task > 0.0 && n.next_task <= t + kEps) {
        const double boundary = n.next_task;
        ++n.task_k;
        n.next_task =
            clock_to_sim(&n, n.epoch + n.offset + static_cast<double>(n.task_k) * n.task) + draw_jitter(&n);
        for (Message& m : messages) {
          if (!m.task_node || *m.task_node != ni) {
            continue;
          }
          int count = 0;
          while (!m.recent.empty() && m.recent.front() < boundary) {
            m.recent.pop_front();
            ++count;
          }
          if (boundary <= m.first_request + m.cycle) {
            continue;  // the sender has not had a full cycle yet
          }
          m.missed += count == 0 ? 1 : 0;
          m.doubled += count >= 2 ? 1 : 0;
          bus.set(m.missed_out, static_cast<double>(m.missed));
          bus.set(m.doubled_out, static_cast<double>(m.doubled));
        }
      }
    }
  }

  void node_outputs(double t, SignalBus& bus) {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      bus.set(nodes[i].tec_out, nodes[i].tec);
      bus.set(nodes[i].bus_off_out, node_silent(i, t) ? 1.0 : 0.0);
    }
  }

  void load_outputs(double t, SignalBus& bus) {
    const double end = t + dt;
    const double begin = std::max(0.0, end - load_window);
    for (Bus& b : buses) {
      double busy = 0.0;
      for (const auto& [s, e] : b.busy) {
        busy += std::max(0.0, std::min(e, end) - std::max(s, begin));
      }
      const double load = busy / load_window;  // before a full window: the time before 0 counts as idle
      bus.set(b.load_out, load);
      if (end >= load_window) {
        b.load_sum += load;
        b.load_max = std::max(b.load_max, load);
        ++b.load_samples;
      }
    }
  }
};

CanNetwork::CanNetwork() : impl_(std::make_unique<Impl>()) {}
CanNetwork::~CanNetwork() { close_logs(); }

void CanNetwork::close_logs() {
  for (Bus& b : impl_->buses) {
    if (b.asc) {
      *b.asc << "End TriggerBlock\n";
      b.asc.reset();
    }
    b.candump.reset();
  }
}

bool CanNetwork::configured(const ParamTree& params) {
  return params.get_bool("can.enabled", true) && params.has_prefix("can.buses.0");
}

void CanNetwork::init(const ParamTree& params, Context& ctx) {
  Impl& s = *impl_;
  SignalBus& bus = ctx.bus();
  s.dt = ctx.dt();
  s.fidelity = params.get_string("can.fidelity",
                                 ctx.realism().is_real(RealismCategory::Comms) ? "faults" : "ideal");
  if (s.fidelity != "ideal" && s.fidelity != "timing" && s.fidelity != "faults") {
    throw std::invalid_argument("can.fidelity '" + s.fidelity + "' (expected ideal, timing or faults)");
  }
  s.timing = s.fidelity != "ideal";
  s.faults = s.fidelity == "faults";
  s.load_window = params.get_double("can.load_window_s", 0.1);
  s.log_mode = params.get_string("can.log", "asc");
  s.fault_rng.emplace(ctx.make_rng("can.faults"));
  const std::filesystem::path project = path_from_utf8(params.get_string("project.dir", "."));

  // Nodes as configured (the rest come from the DBCs as devices).
  for (std::size_t i = 0;; ++i) {
    const std::string p = "can.nodes." + std::to_string(i);
    if (!params.has_prefix(p)) {
      break;
    }
    Node& n = s.nodes[s.node_index(params.get_string(p + ".name"))];
    const std::string role = params.get_string(p + ".role", "device");
    if (role != "device" && role != "controller") {
      throw std::invalid_argument(p + ".role '" + role + "' (expected device or controller)");
    }
    n.controller = role == "controller";
    n.ppm = params.get_double(p + ".clock_ppm", 0.0);
    n.jitter = params.get_double(p + ".jitter_us", 0.0) * 1e-6;
    n.offset = params.get_double(p + ".offset_ms", 0.0) * 1e-3;
    n.task = params.get_double(p + ".task_ms", 0.0) * 1e-3;
    n.recovery = params.get_double(p + ".bus_off_recovery_ms", 100.0) * 1e-3;
    if (std::abs(n.ppm) > 1e5 || n.jitter < 0.0 || n.task < 0.0) {
      throw std::invalid_argument(p + ": clock_ppm, jitter_us or task_ms out of range");
    }
  }

  // Buses and their DBCs.
  std::map<std::string, int> label_count;
  for (std::size_t i = 0;; ++i) {
    const std::string p = "can.buses." + std::to_string(i);
    if (!params.has_prefix(p)) {
      break;
    }
    Bus b;
    b.name = params.get_string(p + ".name", "can" + std::to_string(i));
    b.bitrate = params.get_double(p + ".bitrate", 500e3);
    b.data_bitrate = params.get_double(p + ".data_bitrate", 2e6);
    if (!(b.bitrate > 0.0) || !(b.data_bitrate > 0.0)) {
      throw std::invalid_argument(p + ": bitrate must be positive");
    }
    b.load_out = bus.declare("can." + b.name + ".load", "-", "bus load over the last load window");
    std::vector<std::string> files = params.get_string_list(p + ".dbc");
    if (files.empty() && params.has(p + ".dbc")) {
      files.push_back(params.get_string(p + ".dbc"));
    }
    for (const std::string& file : files) {
      std::filesystem::path path = path_from_utf8(file);
      if (path.is_relative()) {
        path = project / path;
      }
      const Dbc dbc = Dbc::from_file(path);
      for (const DbcMessage& def : dbc.messages) {
        Message m;
        m.def = def;
        m.bus = s.buses.size();
        m.cycle = def.cycle_time;
        if (!def.transmitter.empty()) {
          m.node = s.node_index(def.transmitter);
        }
        ++label_count[def.name];
        s.messages.push_back(std::move(m));
      }
    }
    s.buses.push_back(std::move(b));
  }
  if (s.buses.empty()) {
    throw std::invalid_argument("can.buses: no bus");
  }
  for (std::size_t i = 0; i < s.nodes.size(); ++i) {
    Node& n = s.nodes[i];
    n.rng.emplace(ctx.make_rng("can.node." + n.name));
    n.tec_out = bus.declare("can." + n.name + ".tec", "-", "transmit error counter");
    n.bus_off_out = bus.declare("can." + n.name + ".bus_off", "-", "1 while the node is bus-off");
    n.power_in = bus.find("elec.can_node." + n.name + ".on");
    if (n.task > 0.0) {
      n.next_task = s.clock_to_sim(&n, n.offset) + s.draw_jitter(&n);
    }
  }

  // Extra traffic: frames from no configured node (a datalogger, a gateway).
  for (std::size_t i = 0;; ++i) {
    const std::string p = "can.load." + std::to_string(i);
    if (!params.has_prefix(p)) {
      break;
    }
    const std::string bus_name = params.get_string(p + ".bus", s.buses.front().name);
    const auto bi = std::find_if(s.buses.begin(), s.buses.end(), [&](const Bus& b) { return b.name == bus_name; });
    if (bi == s.buses.end()) {
      throw std::invalid_argument(p + ".bus: no bus '" + bus_name + "'");
    }
    const auto count = params.get_int(p + ".count", 1);
    const auto first = static_cast<std::uint32_t>(params.get_int(p + ".id", 0x7F0));
    for (std::int64_t k = 0; k < count; ++k) {
      Message m;
      m.def.id = first + static_cast<std::uint32_t>(k);
      m.def.extended = params.get_bool(p + ".extended", false);
      m.def.length = static_cast<int>(params.get_int(p + ".length", 8));
      m.def.fd = m.def.length > 8;
      char name[32];
      std::snprintf(name, sizeof name, "LOAD_%X", m.def.id);
      m.def.name = name;
      m.bus = static_cast<std::size_t>(bi - s.buses.begin());
      m.cycle = params.get_double(p + ".period_ms", 10.0) * 1e-3;
      m.generated = true;
      ++label_count[m.def.name];
      s.messages.push_back(std::move(m));
    }
  }

  // Signal map, outputs, schedules.
  const ParamTree map = params.subtree("can.map");
  for (const auto& [key, value] : map.entries()) {
    const std::size_t dot = key.find('.');
    const std::string msg = key.substr(0, dot);
    const std::string sig = dot == std::string::npos ? "" : key.substr(dot + 1, key.find('.', dot + 1) - dot - 1);
    const auto it = std::find_if(s.messages.begin(), s.messages.end(), [&](const Message& m) { return m.def.name == msg; });
    if (it == s.messages.end()) {
      throw std::invalid_argument("can.map." + msg + ": no message '" + msg + "' in the DBCs");
    }
    if (it->def.signal(sig) == nullptr) {
      throw std::invalid_argument("can.map." + msg + "." + sig + ": message " + msg + " has no signal '" + sig + "'");
    }
  }
  for (Message& m : s.messages) {
    m.label = label_count[m.def.name] > 1 ? s.buses[m.bus].name + "." + m.def.name : m.def.name;
    m.from_controller = m.node && s.nodes[*m.node].controller;
    const std::string base = "can." + m.label + ".";
    m.map.resize(m.def.signals.size());
    for (std::size_t i = 0; i < m.def.signals.size(); ++i) {
      const DbcSignal& sig = m.def.signals[i];
      SignalMap& sm = m.map[i];
      sm.decoded = bus.declare(base + sig.name, sig.unit, sig.comment.empty() ? "CAN signal (received)" : sig.comment);
      sm.held = sig.decode(0);
      bus.set(sm.decoded, sm.held);
      const std::string key = "can.map." + m.def.name + "." + sig.name;
      if (!params.has(key) && !params.has_prefix(key)) {
        continue;
      }
      const std::string name = params.has(key + ".signal") ? params.get_string(key + ".signal") : params.get_string(key);
      const std::string dbc_unit = params.get_string(key + ".unit", sig.unit);
      sm.bus_signal = bus.find(name);
      if (!sm.bus_signal) {
        throw std::invalid_argument(key + ": no signal '" + name + "' on the bus");
      }
      try {
        sm.to_dbc = unit_conversion(bus.info(*sm.bus_signal).unit, dbc_unit);
        sm.from_dbc = unit_conversion(dbc_unit, bus.info(*sm.bus_signal).unit);
      } catch (const std::invalid_argument& e) {
        throw std::invalid_argument(key + ": " + e.what() + " (give {signal: ..., unit: ...} for the DBC side)");
      }
    }
    for (const DbcSignal& sig : m.def.signals) {
      if (sig.mux_value &&
          std::find(m.mux_values.begin(), m.mux_values.end(), *sig.mux_value) == m.mux_values.end()) {
        m.mux_values.push_back(*sig.mux_value);
      }
    }
    m.latency_out = bus.declare(base + "latency", "s", "request to delivery of the last frame");
    m.age_out = bus.declare(base + "age", "s", "time since the last frame arrived");
    m.timeout_out = bus.declare(base + "timeout", "-", "1 while no frame came within the timeout");
    m.lost_out = bus.declare(base + "lost", "-", "frames lost (bus-off, drops)");
    const std::string timeout_key = "can.timeouts." + m.def.name;
    m.timeout = params.get_double(params.has(timeout_key + ".after_s") ? timeout_key + ".after_s" : timeout_key,
                                  3.0 * m.cycle);
    const std::string action = params.get_string(timeout_key + ".then", "hold");
    if (action != "hold" && action != "zero") {
      throw std::invalid_argument(timeout_key + ".then '" + action + "' (expected hold or zero)");
    }
    m.zero_on_timeout = action == "zero";
    // A controller receiving with a task of the message's period: count the
    // frames each task run finds (one expected).
    for (const DbcSignal& sig : m.def.signals) {
      for (const std::string& r : sig.receivers) {
        for (std::size_t ni = 0; ni < s.nodes.size(); ++ni) {
          const Node& n = s.nodes[ni];
          if (!m.task_node && n.name == r && n.controller && n.task > 0.0 &&
              std::abs(n.task - m.cycle) < 0.01 * n.task) {
            m.task_node = ni;
          }
        }
      }
    }
    if (m.task_node) {
      m.missed_out = bus.declare(base + "missed", "-", "receiver task runs that found no new frame");
      m.doubled_out = bus.declare(base + "doubled", "-", "receiver task runs that found two or more frames");
    }
    if (m.generated && !s.timing) {
      m.cycle = 0.0;  // extra traffic only matters for timing
    }
    if (m.cycle > 0.0) {
      s.schedule_next(m);
    }
  }

  // Faults.
  for (std::size_t i = 0;; ++i) {
    const std::string p = "can.faults." + std::to_string(i);
    if (!params.has_prefix(p)) {
      break;
    }
    Fault f;
    f.type = params.get_string(p + ".type");
    if (f.type != "drop" && f.type != "delay" && f.type != "error_rate" && f.type != "bus_off" && f.type != "skip") {
      throw std::invalid_argument(p + ".type '" + f.type + "' (expected drop, delay, error_rate, bus_off or skip)");
    }
    if (params.has(p + ".message")) {
      const std::string name = params.get_string(p + ".message");
      const auto it = std::find_if(s.messages.begin(), s.messages.end(),
                                   [&](const Message& m) { return m.def.name == name; });
      if (it == s.messages.end()) {
        throw std::invalid_argument(p + ".message: no message '" + name + "'");
      }
      f.message = static_cast<std::size_t>(it - s.messages.begin());
    }
    if (params.has(p + ".node")) {
      const std::string name = params.get_string(p + ".node");
      const auto it = std::find_if(s.nodes.begin(), s.nodes.end(), [&](const Node& n) { return n.name == name; });
      if (it == s.nodes.end()) {
        throw std::invalid_argument(p + ".node: no node '" + name + "'");
      }
      f.node = static_cast<std::size_t>(it - s.nodes.begin());
    }
    if (params.has(p + ".bus")) {
      const std::string name = params.get_string(p + ".bus");
      const auto it = std::find_if(s.buses.begin(), s.buses.end(), [&](const Bus& b) { return b.name == name; });
      if (it == s.buses.end()) {
        throw std::invalid_argument(p + ".bus: no bus '" + name + "'");
      }
      f.bus = static_cast<std::size_t>(it - s.buses.begin());
    }
    f.at = params.get_double(p + ".at_s", 0.0);
    f.until = params.get_double(p + ".until_s", kInf);
    f.probability = params.get_double(p + ".probability", 1.0);
    f.value = params.get_double(p + ".value_ms", 0.0) * 1e-3;
    f.jitter = params.get_double(p + ".jitter_ms", 0.0) * 1e-3;
    if (f.type == "bus_off" && !f.node) {
      throw std::invalid_argument(p + ": bus_off needs a node");
    }
    s.fault_list.push_back(f);
  }
}

void CanNetwork::before_controllers(double t, SignalBus& bus) {
  Impl& s = *impl_;
  s.update_power(t, bus);
  s.send(t, bus, false);
  s.deliver(t, bus, true);
  s.apply(t, bus, true);
  s.tasks(t, bus);
}

void CanNetwork::after_controllers(double t, SignalBus& bus) {
  Impl& s = *impl_;
  s.update_power(t, bus);
  s.send(t, bus, true);
  s.deliver(t, bus, false);
  s.apply(t, bus, false);
  if (s.timing) {
    s.advance(t);
  }
  s.load_outputs(t, bus);
  s.node_outputs(t, bus);
}

void CanNetwork::after_plant_inputs(SignalBus& bus) {
  Impl& s = *impl_;
  for (auto it = s.restore.rbegin(); it != s.restore.rend(); ++it) {
    bus.set(it->first, it->second);
  }
  s.restore.clear();
}

void CanNetwork::open_logs(const std::filesystem::path& dir) {
  Impl& s = *impl_;
  const bool asc = s.log_mode == "asc" || s.log_mode == "both";
  const bool candump = s.log_mode == "candump" || s.log_mode == "both";
  for (Bus& b : s.buses) {
    if (asc) {
      b.asc = std::make_unique<std::ofstream>(dir / (b.name + ".asc"));
      *b.asc << "date Thu Jan 1 00:00:00.000 am 1970\nbase hex  timestamps absolute\nno internal events logged\n"
             << "// vsim virtual CAN, bus " << b.name << ", " << b.bitrate << " bit/s, fidelity " << s.fidelity
             << "\nBegin TriggerBlock Thu Jan 1 00:00:00.000 am 1970\n   0.000000 Start of measurement\n";
    }
    if (candump) {
      b.candump = std::make_unique<std::ofstream>(dir / (b.name + ".log"));
    }
  }
}

std::string CanNetwork::summary_json() const {
  const Impl& s = *impl_;
  nlohmann::json out;
  out["fidelity"] = s.fidelity;
  for (const Bus& b : s.buses) {
    out["buses"][b.name] = {{"bitrate", b.bitrate},
                            {"frames", b.frames},
                            {"error_frames", b.errors},
                            {"load_mean", b.load_samples ? b.load_sum / static_cast<double>(b.load_samples) : 0.0},
                            {"load_max", b.load_max}};
  }
  for (const Message& m : s.messages) {
    nlohmann::json j = {{"bus", s.buses[m.bus].name}, {"sent", m.sent},       {"delivered", m.delivered},
                        {"lost", m.lost},             {"skipped", m.skipped}, {"timeouts", m.timeouts}};
    if (m.delivered > 0) {
      j["latency_ms"] = {{"min", m.lat_min * 1e3},
                         {"mean", m.lat_sum / static_cast<double>(m.delivered) * 1e3},
                         {"max", m.lat_max * 1e3}};
    }
    if (m.task_node) {
      j["missed"] = m.missed;
      j["doubled"] = m.doubled;
    }
    out["messages"][m.label] = j;
  }
  for (const Node& n : s.nodes) {
    out["nodes"][n.name] = {{"role", n.controller ? "controller" : "device"},
                            {"tec_max", n.tec_max},
                            {"bus_off_count", n.bus_off_count}};
  }
  return out.dump();
}

namespace {

class CanBlock final : public Block {
 public:
  CanBlock(CanNetwork& net, bool after) : net_(net), after_(after) {}
  std::string_view name() const override { return after_ ? "can_tx" : "can_rx"; }
  // The network starts with the second block, once every controller has
  // declared the signals the map may name.
  void init(const ParamTree& params, Context& ctx) override {
    period_ = ctx.dt();
    if (after_) {
      net_.init(params, ctx);
    }
  }
  double period() const override { return period_; }
  void step(double t, SignalBus& bus) override {
    if (after_) {
      net_.after_controllers(t, bus);
    } else {
      net_.before_controllers(t, bus);
    }
  }
  void after_plant_inputs(SignalBus& bus) override {
    if (after_) {
      net_.after_plant_inputs(bus);
    }
  }

 private:
  CanNetwork& net_;
  bool after_;
  double period_ = 0.001;
};

}  // namespace

std::unique_ptr<Block> CanNetwork::make_block(bool after_controllers) {
  return std::make_unique<CanBlock>(*this, after_controllers);
}

}  // namespace vsim
