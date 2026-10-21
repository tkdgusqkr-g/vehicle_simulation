#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace vsim {

// Paths in YAML, JSON and Python are UTF-8. On Windows, std::filesystem::path
// built from std::string uses the ANSI code page, which breaks non-ASCII
// (e.g. Korean) folder names; these helpers convert explicitly.
std::filesystem::path path_from_utf8(std::string_view text);
std::string path_to_utf8(const std::filesystem::path& path);

}  // namespace vsim
