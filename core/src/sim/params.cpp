#include "vsim/sim/params.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

std::string join_key(std::string_view prefix, std::string_view key) {
  if (prefix.empty()) {
    return std::string(key);
  }
  std::string joined(prefix);
  joined += '.';
  joined += key;
  return joined;
}

bool starts_with(std::string_view text, std::string_view prefix) {
  return text.substr(0, prefix.size()) == prefix;
}

void flatten(const YAML::Node& node, const std::string& prefix, ParamTree::Entries& out) {
  switch (node.Type()) {
    case YAML::NodeType::Map:
      for (const auto& item : node) {
        flatten(item.second, join_key(prefix, item.first.as<std::string>()), out);
      }
      break;
    case YAML::NodeType::Sequence:
      for (std::size_t i = 0; i < node.size(); ++i) {
        flatten(node[i], join_key(prefix, std::to_string(i)), out);
      }
      break;
    case YAML::NodeType::Scalar:
      if (prefix.empty()) {
        throw std::invalid_argument("parameter file must contain a mapping at the top level");
      }
      out[prefix] = node.Scalar();
      break;
    case YAML::NodeType::Null:
    case YAML::NodeType::Undefined:
      break;
  }
}

ParamTree::Entries parse_yaml(std::istream& in, const std::string& origin) {
  ParamTree::Entries entries;
  try {
    flatten(YAML::Load(in), "", entries);
  } catch (const YAML::Exception& error) {
    throw std::runtime_error(origin + ": " + error.what());
  }
  return entries;
}

std::string_view trim(std::string_view text) {
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) {
    text.remove_prefix(1);
  }
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) {
    text.remove_suffix(1);
  }
  return text;
}

std::string lower(std::string_view text) {
  std::string out(text);
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

[[noreturn]] void bad_value(std::string_view key, std::string_view value, std::string_view expected) {
  throw std::invalid_argument("parameter '" + std::string(key) + "': expected " +
                              std::string(expected) + ", got '" + std::string(value) + "'");
}

double parse_double(std::string_view key, std::string_view raw_text) {
  std::string_view text = trim(raw_text);
  const std::string special = lower(text);
  if (special == ".inf" || special == "+.inf") {
    return std::numeric_limits<double>::infinity();
  }
  if (special == "-.inf") {
    return -std::numeric_limits<double>::infinity();
  }
  if (special == ".nan") {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (!text.empty() && text.front() == '+') {
    text.remove_prefix(1);
  }
  double value = 0.0;
  const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
  if (text.empty() || error != std::errc() || end != text.data() + text.size()) {
    bad_value(key, raw_text, "a number");
  }
  return value;
}

std::int64_t parse_int(std::string_view key, std::string_view raw_text) {
  std::string_view text = trim(raw_text);
  if (!text.empty() && text.front() == '+') {
    text.remove_prefix(1);
  }
  std::int64_t value = 0;
  const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
  if (text.empty() || error != std::errc() || end != text.data() + text.size()) {
    bad_value(key, raw_text, "an integer");
  }
  return value;
}

bool parse_bool(std::string_view key, std::string_view raw_text) {
  const std::string text = lower(trim(raw_text));
  if (text == "true" || text == "yes" || text == "on" || text == "1") {
    return true;
  }
  if (text == "false" || text == "no" || text == "off" || text == "0") {
    return false;
  }
  bad_value(key, raw_text, "true or false");
}

bool is_index(std::string_view text) {
  return !text.empty() &&
         std::all_of(text.begin(), text.end(), [](unsigned char c) { return std::isdigit(c); });
}

// Nested form of the flat entries, used to emit YAML.
struct YamlNode {
  bool leaf = false;
  std::string value;
  std::map<std::string, YamlNode> children;
};

void emit(YAML::Emitter& out, const YamlNode& node) {
  if (node.leaf) {
    out << node.value;
    return;
  }
  bool sequence = !node.children.empty();
  for (std::size_t i = 0; sequence && i < node.children.size(); ++i) {
    sequence = node.children.count(std::to_string(i)) == 1;
  }
  if (sequence) {
    out << YAML::BeginSeq;
    for (std::size_t i = 0; i < node.children.size(); ++i) {
      emit(out, node.children.at(std::to_string(i)));
    }
    out << YAML::EndSeq;
    return;
  }
  out << YAML::BeginMap;
  for (const auto& [key, child] : node.children) {
    out << YAML::Key << key << YAML::Value;
    emit(out, child);
  }
  out << YAML::EndMap;
}

}  // namespace

ParamTree ParamTree::from_yaml_file(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot open parameter file '" + path_to_utf8(path) + "'");
  }
  ParamTree tree;
  tree.entries_ = parse_yaml(in, path_to_utf8(path));
  return tree;
}

ParamTree ParamTree::from_yaml_string(std::string_view yaml) {
  std::istringstream in{std::string(yaml)};
  ParamTree tree;
  tree.entries_ = parse_yaml(in, "<string>");
  return tree;
}

bool ParamTree::has(std::string_view key) const { return entries_.find(key) != entries_.end(); }

bool ParamTree::has_prefix(std::string_view prefix) const {
  const std::string head = join_key(prefix, "");
  const auto it = entries_.lower_bound(head);
  return it != entries_.end() && starts_with(it->first, head);
}

const std::string& ParamTree::raw(std::string_view key) const {
  const auto it = entries_.find(key);
  if (it == entries_.end()) {
    throw std::out_of_range("missing parameter '" + std::string(key) + "'");
  }
  return it->second;
}

double ParamTree::get_double(std::string_view key) const { return parse_double(key, raw(key)); }

double ParamTree::get_double(std::string_view key, double fallback) const {
  return has(key) ? get_double(key) : fallback;
}

std::int64_t ParamTree::get_int(std::string_view key) const { return parse_int(key, raw(key)); }

std::int64_t ParamTree::get_int(std::string_view key, std::int64_t fallback) const {
  return has(key) ? get_int(key) : fallback;
}

bool ParamTree::get_bool(std::string_view key, bool fallback) const {
  return has(key) ? parse_bool(key, raw(key)) : fallback;
}

std::string ParamTree::get_string(std::string_view key) const { return raw(key); }

std::string ParamTree::get_string(std::string_view key, std::string_view fallback) const {
  return has(key) ? raw(key) : std::string(fallback);
}

std::vector<std::string> ParamTree::get_string_list(std::string_view key) const {
  if (has(key)) {
    return {raw(key)};
  }
  const std::string head = join_key(key, "");
  std::vector<std::pair<std::int64_t, std::string>> items;
  for (auto it = entries_.lower_bound(head); it != entries_.end() && starts_with(it->first, head);
       ++it) {
    const std::string_view rest = std::string_view(it->first).substr(head.size());
    if (!is_index(rest)) {
      throw std::invalid_argument("parameter '" + std::string(key) + "' must be a list of scalars");
    }
    items.emplace_back(parse_int(it->first, rest), it->second);
  }
  std::sort(items.begin(), items.end());
  std::vector<std::string> values;
  values.reserve(items.size());
  for (auto& item : items) {
    values.push_back(std::move(item.second));
  }
  return values;
}

void ParamTree::set(std::string_view key, std::string_view value) {
  if (key.empty()) {
    throw std::invalid_argument("parameter key must not be empty");
  }
  const std::string head = join_key(key, "");
  for (auto it = entries_.lower_bound(head); it != entries_.end() && starts_with(it->first, head);) {
    it = entries_.erase(it);
  }
  for (std::size_t dot = key.find('.'); dot != std::string_view::npos;
       dot = key.find('.', dot + 1)) {
    if (const auto parent = entries_.find(key.substr(0, dot)); parent != entries_.end()) {
      entries_.erase(parent);
    }
  }
  entries_.insert_or_assign(std::string(key), std::string(value));
}

void ParamTree::merge(const ParamTree& other, std::string_view prefix) {
  for (const auto& [key, value] : other.entries_) {
    set(join_key(prefix, key), value);
  }
}

ParamTree ParamTree::subtree(std::string_view prefix) const {
  ParamTree result;
  const std::string head = join_key(prefix, "");
  for (auto it = entries_.lower_bound(head); it != entries_.end() && starts_with(it->first, head);
       ++it) {
    result.entries_.emplace(it->first.substr(head.size()), it->second);
  }
  return result;
}

std::string ParamTree::to_yaml() const {
  YamlNode root;
  for (const auto& [key, value] : entries_) {
    YamlNode* node = &root;
    std::size_t begin = 0;
    while (true) {
      const std::size_t dot = key.find('.', begin);
      node = &node->children[key.substr(begin, dot - begin)];
      if (dot == std::string::npos) {
        break;
      }
      begin = dot + 1;
    }
    node->leaf = true;
    node->value = value;
  }
  YAML::Emitter out;
  emit(out, root);
  return std::string(out.c_str()) + "\n";
}

}  // namespace vsim
