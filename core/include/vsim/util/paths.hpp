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

// The running executable, or empty if the platform cannot tell.
std::filesystem::path executable_path();

// A directory of the source tree such as "config/input", looked up from
// `nearby` (if given), the working directory and the executable's directory,
// each upwards (build trees and projects sit inside the repository). Empty if
// not found.
std::filesystem::path find_resource_dir(std::string_view relative, const std::filesystem::path& nearby = {});

}  // namespace vsim
