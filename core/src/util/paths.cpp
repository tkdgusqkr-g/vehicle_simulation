#include "vsim/util/paths.hpp"

namespace vsim {

std::filesystem::path path_from_utf8(std::string_view text) {
  return std::filesystem::path(std::u8string(text.begin(), text.end()));
}

std::string path_to_utf8(const std::filesystem::path& path) {
  const std::u8string text = path.generic_u8string();
  return std::string(text.begin(), text.end());
}

}  // namespace vsim
