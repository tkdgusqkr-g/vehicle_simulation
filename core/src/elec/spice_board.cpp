#include "vsim/elec/spice_board.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "../plugin/plugin_hosts.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

std::string trim(std::string_view s) {
  const auto first = s.find_first_not_of(" \t\r");
  if (first == std::string_view::npos) {
    return {};
  }
  const auto last = s.find_last_not_of(" \t\r");
  return std::string(s.substr(first, last - first + 1));
}

std::vector<std::string> split_words(const std::string& line) {
  std::vector<std::string> words;
  std::istringstream in(line);
  for (std::string w; in >> w;) {
    words.push_back(w);
  }
  return words;
}

// --- ngspice, loaded once per process ------------------------------------------------------

struct VectorInfo {  // sharedspice.h: vector_info
  char* v_name;
  int v_type;
  short v_flags;
  double* v_realdata;
  void* v_compdata;
  int v_length;
};

using SendChar = int(char*, int, void*);
using SendStat = int(char*, int, void*);
using ControlledExit = int(int, bool, bool, int, void*);
using BgRunning = int(bool, int, void*);
using GetVsrc = int(double*, double, char*, int, void*);

struct NgSpice {
  std::unique_ptr<DynamicLibrary> library;
  int (*init)(SendChar*, SendStat*, ControlledExit*, void*, void*, BgRunning*, void*) = nullptr;
  int (*init_sync)(GetVsrc*, void*, void*, int*, void*) = nullptr;
  int (*command)(char*) = nullptr;
  int (*circ)(char**) = nullptr;
  VectorInfo* (*vector)(char*) = nullptr;
};

std::mutex g_mutex;                  // guards the globals below
NgSpice* g_ngspice = nullptr;        // never unloaded
SpiceBoard::Impl* g_active = nullptr;  // the board ngspice runs for
std::vector<std::string> g_errors;
bool g_exited = false;

std::vector<std::string> library_names(const std::string& library) {
  std::vector<std::string> names;
  if (!library.empty()) {
    names.push_back(library);
  }
  if (const char* env = std::getenv("VSIM_NGSPICE"); env != nullptr && *env != '\0') {
    names.emplace_back(env);
  }
#ifdef _WIN32
  names.insert(names.end(), {"ngspice.dll", "libngspice-0.dll", "ngspice42.dll"});
#elif defined(__APPLE__)
  names.insert(names.end(), {"libngspice.0.dylib", "libngspice.dylib", "/opt/homebrew/lib/libngspice.dylib"});
#else
  names.insert(names.end(), {"libngspice.so.0", "libngspice.so"});
#endif
  return names;
}

int on_char(char* text, int, void*) {
  if (text != nullptr) {
    const std::string s(text);
    if (s.rfind("stderr", 0) == 0) {
      g_errors.push_back(trim(s.substr(6)));
      if (g_errors.size() > 50) {
        g_errors.erase(g_errors.begin());
      }
    }
  }
  return 0;
}
int on_stat(char*, int, void*) { return 0; }
int on_exit(int, bool, bool, int, void*) {
  g_exited = true;
  return 0;
}
int on_bg(bool, int, void*) { return 0; }
int on_vsrc(double* value, double time, char* name, int, void*);

NgSpice& load_ngspice(const std::string& library) {
  if (g_ngspice != nullptr) {
    return *g_ngspice;
  }
  std::string tried;
  for (const std::string& name : library_names(library)) {
    try {
      auto api = std::make_unique<NgSpice>();
      api->library = std::make_unique<DynamicLibrary>(path_from_utf8(name));
      const auto get = [&](const char* symbol) {
        void* p = api->library->symbol(symbol);
        if (p == nullptr) {
          throw std::runtime_error(name + " has no " + symbol + " (not an ngspice shared library?)");
        }
        return p;
      };
      api->init = reinterpret_cast<decltype(api->init)>(get("ngSpice_Init"));
      api->init_sync = reinterpret_cast<decltype(api->init_sync)>(get("ngSpice_Init_Sync"));
      api->command = reinterpret_cast<decltype(api->command)>(get("ngSpice_Command"));
      api->circ = reinterpret_cast<decltype(api->circ)>(get("ngSpice_Circ"));
      api->vector = reinterpret_cast<decltype(api->vector)>(get("ngGet_Vec_Info"));
      api->init(on_char, on_stat, on_exit, nullptr, nullptr, on_bg, nullptr);
      int ident = 0;
      api->init_sync(on_vsrc, nullptr, nullptr, &ident, nullptr);
      g_ngspice = api.release();
      return *g_ngspice;
    } catch (const std::exception& e) {
      tried += "\n  " + std::string(e.what());
    }
  }
  throw std::runtime_error("board: ngspice not found (install ngspice's shared library, or set board.library or "
                           "VSIM_NGSPICE to its file):" + tried);
}

void command(NgSpice& ng, const std::string& text) {
  std::vector<char> buffer(text.begin(), text.end());
  buffer.push_back('\0');
  ng.command(buffer.data());
}

}  // namespace

std::string prepare_board_netlist(std::string_view text, const std::string& dir, const std::vector<std::string>& inputs,
                                  const std::vector<std::string>& saves, double max_step, double stop_time) {
  std::vector<std::string> lines;
  {
    std::string line;
    std::istringstream in{std::string(text)};
    while (std::getline(in, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      lines.push_back(line);
    }
  }
  std::map<std::string, bool> used;
  for (const std::string& name : inputs) {
    used[lower(name)] = false;
  }
  std::ostringstream out;
  bool in_control = false;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    const std::string& line = lines[i];
    if (i == 0) {
      out << line << "\n";  // the title
      continue;
    }
    const std::string t = trim(line);
    const std::string low = lower(t);
    if (low.rfind(".control", 0) == 0) {
      in_control = true;
      continue;
    }
    if (in_control) {
      in_control = low.rfind(".endc", 0) != 0;
      continue;
    }
    if (low.rfind(".end", 0) == 0 && (low.size() == 4 || std::isspace(static_cast<unsigned char>(low[4])))) {
      break;
    }
    // Analyses and outputs are the co-simulation's own.
    bool skip = false;
    for (const char* card : {".tran", ".op", ".ac", ".dc", ".noise", ".tf", ".disto", ".pz", ".sens", ".four",
                             ".save", ".print", ".plot", ".probe", ".meas", ".measure"}) {
      const std::string c(card);
      if (low.rfind(c, 0) == 0 && (low.size() == c.size() || std::isspace(static_cast<unsigned char>(low[c.size()])))) {
        skip = true;
      }
    }
    if (skip) {
      out << "* (co-simulation) " << t << "\n";
      continue;
    }
    // Includes relative to the netlist's folder.
    if (low.rfind(".include", 0) == 0 || low.rfind(".inc ", 0) == 0 || low.rfind(".lib", 0) == 0) {
      std::vector<std::string> words = split_words(t);
      if (words.size() >= 2) {
        std::string file = words[1];
        const bool quoted = file.size() >= 2 && (file.front() == '"' || file.front() == '\'');
        if (quoted) {
          file = file.substr(1, file.size() - 2);
        }
        std::filesystem::path p = path_from_utf8(file);
        if (p.is_relative()) {
          p = path_from_utf8(dir) / p;
        }
        words[1] = "\"" + path_to_utf8(p.lexically_normal()) + "\"";
        std::string rebuilt;
        for (const std::string& w : words) {
          rebuilt += (rebuilt.empty() ? "" : " ") + w;
        }
        out << rebuilt << "\n";
        continue;
      }
    }
    // A named input source: its value comes from the simulation.
    const std::vector<std::string> words = split_words(t);
    if (!words.empty()) {
      const auto it = used.find(lower(words[0]));
      if (it != used.end()) {
        if (words.size() < 3 || (std::tolower(static_cast<unsigned char>(words[0][0])) != 'v' &&
                                 std::tolower(static_cast<unsigned char>(words[0][0])) != 'i')) {
          throw std::invalid_argument("board.inputs." + words[0] + ": not a voltage or current source line");
        }
        // Continuation lines of the old source go too.
        while (i + 1 < lines.size() && trim(lines[i + 1]).rfind('+', 0) == 0) {
          ++i;
        }
        out << words[0] << " " << words[1] << " " << words[2] << " dc 0 external\n";
        it->second = true;
        continue;
      }
    }
    out << line << "\n";
  }
  for (const auto& [name, found] : used) {
    if (!found) {
      throw std::invalid_argument("board.inputs: the netlist has no source '" + name + "'");
    }
  }
  char tran[128];
  std::snprintf(tran, sizeof tran, ".tran %.9g %.9g 0 %.9g", max_step, stop_time, max_step);
  out << tran << "\n";
  if (!saves.empty()) {
    out << ".save";
    for (const std::string& s : saves) {
      out << " " << s;
    }
    out << "\n";
  }
  out << ".end\n";
  return out.str();
}

struct SpiceBoard::Impl {
  struct Input {
    std::string source;              // lower case
    std::optional<SignalId> signal;  // else constant
    double value = 0.0;
  };
  struct Output {
    std::string vector;  // ngspice vector name
    SignalId signal = 0;
    double pending = 0.0;
  };
  NgSpice* ng = nullptr;
  double period = 0.001, max_step = 1e-5;
  std::vector<Input> inputs;
  std::map<std::string, double> held;  // source (lower case) -> value over the running period
  std::vector<Output> outputs;
  std::string netlist;
  bool started = false;
  double solved_to = 0.0;
  SignalId points_out = 0;

  ~Impl() {
    if (g_active != this) {
      return;  // never loaded (failed init), nothing in ngspice
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    command(*ng, "remcirc");
    command(*ng, "destroy all");
    g_active = nullptr;
  }

  [[noreturn]] void fail(const std::string& what) const {
    std::string text = "board: " + what;
    for (const std::string& e : g_errors) {
      text += "\n  ngspice: " + e;
    }
    throw std::runtime_error(text);
  }

  // Value of a vector at time `at` (linear between the last two points around it).
  double value_at(const std::string& name, double at) {
    std::vector<char> n(name.begin(), name.end());
    n.push_back('\0');
    // ngGet_Vec_Info returns one shared struct: read each vector before the next call.
    const VectorInfo* v = ng->vector(n.data());
    if (v == nullptr || v->v_realdata == nullptr || v->v_length < 1) {
      fail("no vector '" + name + "' (a node or source the netlist does not have?)");
    }
    const int len = v->v_length;
    const double last = v->v_realdata[len - 1];
    const double before = len > 1 ? v->v_realdata[len - 2] : last;
    char time_name[] = "time";
    const VectorInfo* t = ng->vector(time_name);
    if (t == nullptr || t->v_length != len || len < 2) {
      return last;
    }
    const double t1 = t->v_realdata[len - 1], t0 = t->v_realdata[len - 2];
    const double f = t1 > t0 ? std::clamp((at - t0) / (t1 - t0), 0.0, 1.0) : 1.0;
    return before + f * (last - before);
  }

  void advance(double to) {
    command(*ng, "delete all");
    char stop[64];
    std::snprintf(stop, sizeof stop, "stop when time > %.12g", to);
    command(*ng, stop);
    command(*ng, started ? "resume" : "run");
    started = true;
    if (g_exited) {
      fail("ngspice stopped (see its messages)");
    }
    solved_to = to;
  }
};

namespace {

// ngspice asks for an external source's value at each of its time points: the held input.
int on_vsrc(double* value, double, char* name, int, void*) {
  *value = 0.0;
  if (g_active != nullptr && name != nullptr) {
    const auto it = g_active->held.find(lower(name));
    if (it != g_active->held.end()) {
      *value = it->second;
    }
  }
  return 0;
}

}  // namespace

SpiceBoard::SpiceBoard() = default;
SpiceBoard::~SpiceBoard() = default;

bool SpiceBoard::configured(const ParamTree& params) {
  return params.has_prefix("board") && params.get_bool("board.enabled", true);
}

std::string SpiceBoard::after(const ParamTree& params) {
  const std::string where = params.get_string("board.after", "sensors");
  if (where != "sensors" && where != "vcu") {
    throw std::invalid_argument("board.after: sensors or vcu, not '" + where + "'");
  }
  return where;
}

bool SpiceBoard::available(const std::string& library) {
  std::lock_guard<std::mutex> lock(g_mutex);
  try {
    load_ngspice(library);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

double SpiceBoard::period() const { return impl_ ? impl_->period : 0.001; }

const std::string& SpiceBoard::prepared_netlist() const { return impl_->netlist; }

void SpiceBoard::init(const ParamTree& params, Context& ctx) {
  impl_.reset();
  auto impl = std::make_unique<Impl>();
  const double rate = params.get_double("board.rate_hz", 1.0 / params.get_double("time.dt", 0.001));
  impl->max_step = 1e-6 * params.get_double("board.max_step_us", 10.0);
  if (!(rate > 0.0) || !(impl->max_step > 0.0)) {
    throw std::invalid_argument("board: rate_hz and max_step_us must be positive");
  }
  impl->period = 1.0 / rate;
  const std::string netlist_key = params.get_string("board.netlist");
  std::filesystem::path file = path_from_utf8(netlist_key);
  if (file.is_relative()) {
    file = path_from_utf8(params.get_string("project.dir", ".")) / file;
  }
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    throw std::invalid_argument("board.netlist: cannot read " + path_to_utf8(file));
  }
  std::stringstream text;
  text << in.rdbuf();

  SignalBus& bus = ctx.bus();
  std::vector<std::string> sources;
  const ParamTree in_tree = params.subtree("board.inputs");
  std::map<std::string, bool> seen;
  for (const auto& [key, value] : in_tree.entries()) {
    const std::string source = key.substr(0, key.find('.'));
    if (seen.emplace(source, true).second) {
      Impl::Input input;
      input.source = lower(source);
      if (in_tree.has(source + ".value")) {
        input.value = in_tree.get_double(source + ".value");
      } else {
        const std::string signal = in_tree.has(source + ".signal") ? in_tree.get_string(source + ".signal")
                                                                   : in_tree.get_string(source);
        input.signal = bus.find(signal);
        if (!input.signal) {
          throw std::invalid_argument("board.inputs." + source + ": no signal '" + signal + "'");
        }
      }
      sources.push_back(source);
      impl->inputs.push_back(input);
    }
  }
  std::vector<std::string> saves;
  const ParamTree out_tree = params.subtree("board.outputs");
  for (const auto& [key, signal] : out_tree.entries()) {
    Impl::Output output;
    const std::string k = lower(trim(key));
    if (k.rfind("i(", 0) == 0 && k.back() == ')') {
      const std::string source = k.substr(2, k.size() - 3);
      output.vector = source + "#branch";
      saves.push_back("i(" + source + ")");
    } else {
      output.vector = k;
      saves.push_back("v(" + k + ")");
    }
    output.signal = bus.declare(signal, k.rfind("i(", 0) == 0 ? "A" : "V", "board " + key);
    impl->outputs.push_back(output);
  }
  if (impl->outputs.empty()) {
    throw std::invalid_argument("board.outputs: at least one node or source current");
  }
  impl->points_out = bus.declare("board.time_points", "-", "ngspice time points so far (cost of the board)");

  const double t_end = params.get_double("time.t_end", 0.0);
  const double stop = t_end > 0.0 ? t_end + 1.0 : 3600.0;
  impl->netlist = prepare_board_netlist(text.str(), path_to_utf8(file.parent_path()), sources, saves, impl->max_step,
                                        stop);

  std::lock_guard<std::mutex> lock(g_mutex);
  impl->ng = &load_ngspice(params.get_string("board.library", ""));
  if (g_active != nullptr) {
    throw std::runtime_error("board: ngspice already runs a board in this process (one at a time)");
  }
  g_errors.clear();
  g_exited = false;
  for (const Impl::Input& input : impl->inputs) {
    impl->held[input.source] = input.value;
  }
  std::vector<std::string> lines;
  {
    std::istringstream nl(impl->netlist);
    for (std::string line; std::getline(nl, line);) {
      lines.push_back(line);
    }
  }
  std::vector<char*> pointers;
  for (std::string& line : lines) {
    pointers.push_back(line.data());
  }
  pointers.push_back(nullptr);
  g_active = impl.get();
  if (impl->ng->circ(pointers.data()) != 0 || g_exited) {
    g_active = nullptr;
    impl->fail("the netlist did not load");
  }
  for (const std::string& e : g_errors) {
    if (lower(e).find("error") != std::string::npos) {
      command(*impl->ng, "remcirc");
      g_active = nullptr;
      impl->fail("the netlist did not load");
    }
  }
  impl_ = std::move(impl);
}

void SpiceBoard::step(double t, SignalBus& bus) {
  Impl& b = *impl_;
  std::lock_guard<std::mutex> lock(g_mutex);
  // Inputs as they are now; the circuit catches up to t with the previous ones.
  if (!b.started) {
    for (const Impl::Input& input : b.inputs) {
      b.held[input.source] = input.signal ? bus.get(*input.signal) : input.value;
    }
  }
  if (t > b.solved_to || !b.started) {
    b.advance(std::max(t, 1e-9));
  }
  for (Impl::Output& o : b.outputs) {
    o.pending = b.value_at(o.vector, t);
    bus.set(o.signal, o.pending);
  }
  char time_name[] = "time";
  if (const VectorInfo* v = b.ng->vector(time_name); v != nullptr) {
    bus.set(b.points_out, static_cast<double>(v->v_length));
  }
  for (const Impl::Input& input : b.inputs) {
    if (input.signal) {
      b.held[input.source] = bus.get(*input.signal);
    }
  }
}

}  // namespace vsim
