// FMI 2.0 Co-Simulation host (design doc §11.2).

#include <miniz.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "plugin_hosts.hpp"
#include "vsim/util/paths.hpp"
#include "vsim_fmi2.h"

namespace vsim {
namespace {

// --- A small XML reader for modelDescription.xml (tags and attributes only) ----------------

struct XmlTag {
  std::string name;
  std::map<std::string, std::string> attrs;
  bool closing = false;       // </name>
  bool self_closing = false;  // <name/>
};

std::string xml_unescape(const std::string& s) {
  std::string out;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] != '&') {
      out += s[i];
      continue;
    }
    const std::size_t end = s.find(';', i);
    if (end == std::string::npos) {
      out += s[i];
      continue;
    }
    const std::string entity = s.substr(i + 1, end - i - 1);
    if (entity == "amp") {
      out += '&';
    } else if (entity == "lt") {
      out += '<';
    } else if (entity == "gt") {
      out += '>';
    } else if (entity == "quot") {
      out += '"';
    } else if (entity == "apos") {
      out += '\'';
    } else if (!entity.empty() && entity[0] == '#') {
      const long code = entity.size() > 1 && (entity[1] == 'x' || entity[1] == 'X')
                            ? std::strtol(entity.c_str() + 2, nullptr, 16)
                            : std::strtol(entity.c_str() + 1, nullptr, 10);
      if (code < 0x80) {
        out += static_cast<char>(code);
      } else if (code < 0x800) {
        out += static_cast<char>(0xC0 | (code >> 6));
        out += static_cast<char>(0x80 | (code & 0x3F));
      } else {
        out += static_cast<char>(0xE0 | (code >> 12));
        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (code & 0x3F));
      }
    } else {
      out += s.substr(i, end - i + 1);
    }
    i = end;
  }
  return out;
}

std::vector<XmlTag> xml_tags(const std::string& text) {
  std::vector<XmlTag> tags;
  std::size_t i = 0;
  while ((i = text.find('<', i)) != std::string::npos) {
    if (text.compare(i, 4, "<!--") == 0) {
      i = text.find("-->", i);
      if (i == std::string::npos) {
        break;
      }
      continue;
    }
    if (text.compare(i, 2, "<?") == 0 || text.compare(i, 2, "<!") == 0) {
      i = text.find('>', i);
      continue;
    }
    XmlTag tag;
    std::size_t j = i + 1;
    if (j < text.size() && text[j] == '/') {
      tag.closing = true;
      ++j;
    }
    const auto is_space = [](char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; };
    std::size_t name_end = j;
    while (name_end < text.size() && !is_space(text[name_end]) && text[name_end] != '>' && text[name_end] != '/') {
      ++name_end;
    }
    tag.name = text.substr(j, name_end - j);
    j = name_end;
    while (j < text.size()) {
      while (j < text.size() && is_space(text[j])) {
        ++j;
      }
      if (j >= text.size()) {
        break;
      }
      if (text[j] == '>') {
        break;
      }
      if (text[j] == '/' && j + 1 < text.size() && text[j + 1] == '>') {
        tag.self_closing = true;
        ++j;
        break;
      }
      const std::size_t eq = text.find('=', j);
      if (eq == std::string::npos) {
        break;
      }
      std::string key = text.substr(j, eq - j);
      while (!key.empty() && is_space(key.back())) {
        key.pop_back();
      }
      std::size_t q = eq + 1;
      while (q < text.size() && is_space(text[q])) {
        ++q;
      }
      if (q >= text.size() || (text[q] != '"' && text[q] != '\'')) {
        throw std::invalid_argument("modelDescription.xml: malformed attribute in <" + tag.name + ">");
      }
      const char quote = text[q];
      const std::size_t close = text.find(quote, q + 1);
      if (close == std::string::npos) {
        throw std::invalid_argument("modelDescription.xml: unterminated attribute in <" + tag.name + ">");
      }
      tag.attrs[key] = xml_unescape(text.substr(q + 1, close - q - 1));
      j = close + 1;
    }
    tags.push_back(std::move(tag));
    i = text.find('>', j);
    if (i == std::string::npos) {
      break;
    }
  }
  return tags;
}

std::string attr(const XmlTag& tag, const std::string& key, const std::string& fallback = "") {
  const auto it = tag.attrs.find(key);
  return it == tag.attrs.end() ? fallback : it->second;
}

struct Variable {
  std::string name, causality = "local", variability = "continuous", type, unit, declared_type, start;
  fmi2ValueReference vr = 0;
};

struct ModelDescription {
  std::string fmi_version, model_name, guid, generation_tool, model_identifier;
  bool cosimulation = false, model_exchange = false;
  std::vector<Variable> variables;
};

ModelDescription read_model_description(const std::filesystem::path& file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    throw std::invalid_argument("FMU has no modelDescription.xml (" + path_to_utf8(file) + ")");
  }
  std::stringstream ss;
  ss << in.rdbuf();
  ModelDescription md;
  std::map<std::string, std::string> type_units;  // TypeDefinitions: SimpleType name -> unit
  std::string simple_type;
  bool in_variable = false;
  for (const XmlTag& tag : xml_tags(ss.str())) {
    if (tag.closing) {
      if (tag.name == "ScalarVariable") {
        in_variable = false;
      } else if (tag.name == "SimpleType") {
        simple_type.clear();
      }
      continue;
    }
    if (tag.name == "fmiModelDescription") {
      md.fmi_version = attr(tag, "fmiVersion");
      md.model_name = attr(tag, "modelName");
      md.guid = attr(tag, "guid");
      md.generation_tool = attr(tag, "generationTool");
    } else if (tag.name == "CoSimulation") {
      md.cosimulation = true;
      md.model_identifier = attr(tag, "modelIdentifier");
    } else if (tag.name == "ModelExchange") {
      md.model_exchange = true;
    } else if (tag.name == "SimpleType") {
      simple_type = tag.self_closing ? "" : attr(tag, "name");
    } else if (!simple_type.empty() && (tag.name == "Real" || tag.name == "Integer")) {
      type_units[simple_type] = attr(tag, "unit");
    } else if (tag.name == "ScalarVariable") {
      Variable v;
      v.name = attr(tag, "name");
      v.causality = attr(tag, "causality", "local");
      v.variability = attr(tag, "variability", "continuous");
      v.vr = static_cast<fmi2ValueReference>(std::strtoul(attr(tag, "valueReference", "0").c_str(), nullptr, 10));
      md.variables.push_back(v);
      in_variable = !tag.self_closing;
    } else if (in_variable && !md.variables.empty() &&
               (tag.name == "Real" || tag.name == "Integer" || tag.name == "Boolean" || tag.name == "String" ||
                tag.name == "Enumeration")) {
      Variable& v = md.variables.back();
      v.type = tag.name == "Enumeration" ? "Integer" : tag.name;
      v.unit = attr(tag, "unit");
      v.declared_type = attr(tag, "declaredType");
      v.start = attr(tag, "start");
    }
  }
  for (Variable& v : md.variables) {
    if (v.unit.empty() && !v.declared_type.empty()) {
      const auto it = type_units.find(v.declared_type);
      if (it != type_units.end()) {
        v.unit = it->second;
      }
    }
  }
  return md;
}

// --- Unpacking ------------------------------------------------------------------------------

std::filesystem::path unpack_fmu(const std::filesystem::path& fmu, const std::string& sha) {
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / "vsim-fmu" / sha.substr(0, 20);
  if (std::filesystem::exists(dir / "modelDescription.xml")) {
    return dir;  // named by content: already unpacked
  }
  const std::filesystem::path partial = dir.string() + ".partial";
  std::filesystem::remove_all(partial);
  std::filesystem::create_directories(partial);
  mz_zip_archive zip{};
  const std::string zip_path = path_to_utf8(fmu);
  if (!mz_zip_reader_init_file(&zip, zip_path.c_str(), 0)) {
    throw std::invalid_argument("'" + zip_path + "' is not a readable FMU (zip) file");
  }
  const mz_uint count = mz_zip_reader_get_num_files(&zip);
  for (mz_uint i = 0; i < count; ++i) {
    mz_zip_archive_file_stat stat{};
    if (!mz_zip_reader_file_stat(&zip, i, &stat) || mz_zip_reader_is_file_a_directory(&zip, i)) {
      continue;
    }
    const std::string name = stat.m_filename;
    const std::filesystem::path rel = std::filesystem::path(name).lexically_normal();
    if (rel.is_absolute() || name.find("..") != std::string::npos || rel.has_root_name()) {
      mz_zip_reader_end(&zip);
      throw std::invalid_argument("'" + zip_path + "' has an unsafe entry '" + name + "'");
    }
    const std::filesystem::path out = partial / rel;
    std::filesystem::create_directories(out.parent_path());
    if (!mz_zip_reader_extract_to_file(&zip, i, path_to_utf8(out).c_str(), 0)) {
      mz_zip_reader_end(&zip);
      throw std::runtime_error("cannot unpack '" + name + "' from '" + zip_path + "'");
    }
  }
  mz_zip_reader_end(&zip);
  std::error_code ec;
  std::filesystem::rename(partial, dir, ec);
  if (ec) {  // another run unpacked it meanwhile
    std::filesystem::remove_all(partial, ec);
  }
  return dir;
}

std::string file_uri(const std::filesystem::path& dir) {
  std::string path = path_to_utf8(std::filesystem::absolute(dir));
  std::replace(path.begin(), path.end(), '\\', '/');
  std::string out = "file://";
  if (path.empty() || path[0] != '/') {
    out += '/';
  }
  static const char* hex = "0123456789ABCDEF";
  for (const unsigned char c : path) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '/' || c == ':' ||
        c == '-' || c == '_' || c == '.' || c == '~') {
      out += static_cast<char>(c);
    } else {
      out += '%';
      out += hex[c >> 4];
      out += hex[c & 0xF];
    }
  }
  return out;
}

std::string platform_folder() {
#if defined(_WIN32)
  return "win64";
#elif defined(__APPLE__)
  return "darwin64";
#else
  return "linux64";
#endif
}

void fmu_logger(fmi2ComponentEnvironment env, fmi2String instance, fmi2Status status, fmi2String category,
                fmi2String message, ...) {
  char text[2048];
  va_list args;
  va_start(args, message);
  std::vsnprintf(text, sizeof text, message ? message : "", args);
  va_end(args);
  (void)env;
  if (status >= fmi2Warning) {
    std::fprintf(stderr, "FMU %s [%s] %s: %s\n", instance ? instance : "", category ? category : "",
                 status == fmi2Warning ? "warning" : "error", text);
  }
}

template <class T>
T* load(const DynamicLibrary& lib, const char* name) {
  auto* f = reinterpret_cast<T*>(lib.symbol(name));
  if (f == nullptr) {
    throw std::invalid_argument(path_to_utf8(lib.file()) + " does not export " + name);
  }
  return f;
}

// Port names become keys of the project file: no dots or brackets.
std::string port_name(std::string name) {
  for (char& c : name) {
    if (c == '.' || c == '[' || c == ']' || c == ',' || c == ' ') {
      c = '_';
    }
  }
  while (!name.empty() && name.back() == '_') {
    name.pop_back();
  }
  return name;
}

class FmuPlugin final : public PluginInstance {
 public:
  FmuPlugin(const ParamTree& params, const ComponentSpec& spec, double period) : name_(spec.name) {
    const std::filesystem::path fmu =
        find_plugin_file(spec, params.get_string(spec.prefix + ".path", ""), {".fmu"}, true);
    const std::filesystem::path dir =
        std::filesystem::is_directory(fmu) ? fmu : unpack_fmu(fmu, sha256_file(fmu));
    const ModelDescription md = read_model_description(dir / "modelDescription.xml");
    if (md.fmi_version.rfind("2.", 0) != 0) {
      throw std::invalid_argument(path_to_utf8(fmu) + ": FMI version '" + md.fmi_version +
                                  "' (this host runs FMI 2.0 Co-Simulation FMUs)");
    }
    if (!md.cosimulation) {
      throw std::invalid_argument(path_to_utf8(fmu) +
                                  ": no Co-Simulation interface (Model Exchange FMUs are not supported yet; "
                                  "export the model for Co-Simulation)");
    }
    const std::filesystem::path binary =
        dir / "binaries" / platform_folder() / (md.model_identifier + shared_library_extension());
    if (!std::filesystem::exists(binary)) {
      throw std::invalid_argument(path_to_utf8(fmu) + ": no binary for this platform (" +
                                  path_to_utf8(binary.lexically_relative(dir)) + ")");
    }
    library_ = std::make_unique<DynamicLibrary>(binary);
    instantiate_ = load<fmi2InstantiateTYPE>(*library_, "fmi2Instantiate");
    free_ = load<fmi2FreeInstanceTYPE>(*library_, "fmi2FreeInstance");
    auto* setup = load<fmi2SetupExperimentTYPE>(*library_, "fmi2SetupExperiment");
    auto* enter = load<fmi2EnterInitializationModeTYPE>(*library_, "fmi2EnterInitializationMode");
    auto* exit = load<fmi2ExitInitializationModeTYPE>(*library_, "fmi2ExitInitializationMode");
    terminate_ = load<fmi2TerminateTYPE>(*library_, "fmi2Terminate");
    get_real_ = load<fmi2GetRealTYPE>(*library_, "fmi2GetReal");
    get_integer_ = load<fmi2GetIntegerTYPE>(*library_, "fmi2GetInteger");
    get_boolean_ = load<fmi2GetBooleanTYPE>(*library_, "fmi2GetBoolean");
    set_real_ = load<fmi2SetRealTYPE>(*library_, "fmi2SetReal");
    set_integer_ = load<fmi2SetIntegerTYPE>(*library_, "fmi2SetInteger");
    set_boolean_ = load<fmi2SetBooleanTYPE>(*library_, "fmi2SetBoolean");
    auto* set_string = load<fmi2SetStringTYPE>(*library_, "fmi2SetString");
    do_step_ = load<fmi2DoStepTYPE>(*library_, "fmi2DoStep");

    for (const bool input : {true, false}) {
      for (const Variable& v : md.variables) {
        if (v.causality == (input ? "input" : "output") && (v.type == "Real" || v.type == "Integer" ||
                                                            v.type == "Boolean")) {
          ports_.push_back({port_name(v.name), v.unit, input});
          (input ? inputs_ : outputs_).push_back({v.vr, v.type});
        }
      }
    }
    period_ = period;
    if (!(period_ > 0.0)) {
      throw std::invalid_argument(spec.prefix + ".period: give the FMU's communication step");
    }
    info_ = {"fmu", path_to_utf8(fmu), md.model_name, md.generation_tool};

    callbacks_ = {fmu_logger, std::calloc, std::free, nullptr, nullptr};
    const std::string resources = file_uri(dir / "resources");
    component_ = instantiate_(spec.name.c_str(), fmi2CoSimulation, md.guid.c_str(), resources.c_str(), &callbacks_,
                              fmi2False, fmi2False);
    if (component_ == nullptr) {
      throw std::runtime_error("FMU " + spec.name + ": fmi2Instantiate failed");
    }
    // Parameters by variable name (dots and brackets as in the FMU, or as port names).
    const ParamTree plugin_params = params.subtree(spec.prefix + ".params");
    for (const auto& [key, value] : plugin_params.entries()) {
      const auto v = std::find_if(md.variables.begin(), md.variables.end(), [&](const Variable& x) {
        return (x.name == key || port_name(x.name) == key) && x.causality != "input" && x.causality != "output";
      });
      if (v == md.variables.end()) {
        throw std::invalid_argument(spec.prefix + ".params." + key + ": the FMU has no parameter '" + key + "'");
      }
      fmi2Status s = fmi2OK;
      if (v->type == "Real") {
        const fmi2Real x = std::stod(value);
        s = set_real_(component_, &v->vr, 1, &x);
      } else if (v->type == "Integer") {
        const fmi2Integer x = static_cast<fmi2Integer>(std::stol(value));
        s = set_integer_(component_, &v->vr, 1, &x);
      } else if (v->type == "Boolean") {
        const fmi2Boolean x = value == "true" || value == "1" ? fmi2True : fmi2False;
        s = set_boolean_(component_, &v->vr, 1, &x);
      } else {
        const fmi2String x = value.c_str();
        s = set_string(component_, &v->vr, 1, &x);
      }
      check(s, "setting parameter " + key);
    }
    check(setup(component_, fmi2False, 0.0, 0.0, fmi2False, 0.0), "fmi2SetupExperiment");
    check(enter(component_), "fmi2EnterInitializationMode");
    check(exit(component_), "fmi2ExitInitializationMode");
  }

  ~FmuPlugin() override {
    if (component_ != nullptr) {
      terminate_(component_);
      free_(component_);
    }
  }

  const std::vector<PortInfo>& ports() const override { return ports_; }
  double period() const override { return period_; }

  // Inputs at t, one communication step, outputs: like a native block, the
  // outputs computed from the inputs at t hold until the next step.
  void step(double t, const double* in, double* out) override {
    for (std::size_t i = 0; i < inputs_.size(); ++i) {
      set(inputs_[i], in[i]);
    }
    check(do_step_(component_, t, period_, fmi2True), "fmi2DoStep at t = " + std::to_string(t));
    for (std::size_t i = 0; i < outputs_.size(); ++i) {
      out[i] = get(outputs_[i]);
    }
  }
  Info info() const override { return info_; }

 private:
  struct Ref {
    fmi2ValueReference vr;
    std::string type;
  };

  void check(fmi2Status s, const std::string& what) const {
    if (s != fmi2OK && s != fmi2Warning) {
      throw std::runtime_error("FMU " + name_ + ": " + what + " failed (status " + std::to_string(s) + ")");
    }
  }
  void set(const Ref& r, double value) {
    if (r.type == "Real") {
      check(set_real_(component_, &r.vr, 1, &value), "fmi2SetReal");
    } else if (r.type == "Integer") {
      const fmi2Integer x = static_cast<fmi2Integer>(std::lround(value));
      check(set_integer_(component_, &r.vr, 1, &x), "fmi2SetInteger");
    } else {
      const fmi2Boolean x = value != 0.0 ? fmi2True : fmi2False;
      check(set_boolean_(component_, &r.vr, 1, &x), "fmi2SetBoolean");
    }
  }
  double get(const Ref& r) {
    if (r.type == "Real") {
      fmi2Real x = 0.0;
      check(get_real_(component_, &r.vr, 1, &x), "fmi2GetReal");
      return x;
    }
    if (r.type == "Integer") {
      fmi2Integer x = 0;
      check(get_integer_(component_, &r.vr, 1, &x), "fmi2GetInteger");
      return x;
    }
    fmi2Boolean x = fmi2False;
    check(get_boolean_(component_, &r.vr, 1, &x), "fmi2GetBoolean");
    return x ? 1.0 : 0.0;
  }

  std::string name_;
  std::unique_ptr<DynamicLibrary> library_;
  fmi2InstantiateTYPE* instantiate_ = nullptr;
  fmi2FreeInstanceTYPE* free_ = nullptr;
  fmi2TerminateTYPE* terminate_ = nullptr;
  fmi2GetRealTYPE* get_real_ = nullptr;
  fmi2GetIntegerTYPE* get_integer_ = nullptr;
  fmi2GetBooleanTYPE* get_boolean_ = nullptr;
  fmi2SetRealTYPE* set_real_ = nullptr;
  fmi2SetIntegerTYPE* set_integer_ = nullptr;
  fmi2SetBooleanTYPE* set_boolean_ = nullptr;
  fmi2DoStepTYPE* do_step_ = nullptr;
  fmi2CallbackFunctions callbacks_{};
  fmi2Component component_ = nullptr;
  std::vector<PortInfo> ports_;
  std::vector<Ref> inputs_, outputs_;
  double period_ = 0.0;
  Info info_;
};

}  // namespace

std::unique_ptr<PluginInstance> make_fmu_plugin(const ParamTree& params, const ComponentSpec& spec, double period) {
  return std::make_unique<FmuPlugin>(params, spec, period);
}

}  // namespace vsim
