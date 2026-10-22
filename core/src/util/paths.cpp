#include "vsim/util/paths.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <system_error>
#include <vector>

namespace vsim {

std::filesystem::path path_from_utf8(std::string_view text) {
  return std::filesystem::path(std::u8string(text.begin(), text.end()));
}

std::string path_to_utf8(const std::filesystem::path& path) {
  const std::u8string text = path.generic_u8string();
  return std::string(text.begin(), text.end());
}

std::filesystem::path executable_path() {
#ifdef _WIN32
  std::vector<wchar_t> buffer(32768);
  const DWORD size = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
  return size > 0 ? std::filesystem::path(std::wstring(buffer.data(), size)) : std::filesystem::path();
#else
  std::error_code error;
  const std::filesystem::path self = std::filesystem::read_symlink("/proc/self/exe", error);
  return error ? std::filesystem::path() : self;
#endif
}

std::filesystem::path find_resource_dir(std::string_view relative, const std::filesystem::path& near) {
  const std::filesystem::path target = path_from_utf8(relative);
  std::vector<std::filesystem::path> starts;
  std::error_code error;
  if (!near.empty()) {
    starts.push_back(std::filesystem::absolute(near, error));
  }
  starts.push_back(std::filesystem::current_path(error));
  if (const std::filesystem::path exe = executable_path(); !exe.empty()) {
    starts.push_back(exe.parent_path());
  }
  for (std::filesystem::path dir : starts) {
    for (int level = 0; level < 6 && !dir.empty(); ++level) {
      const std::filesystem::path candidate = dir / target;
      if (std::filesystem::is_directory(candidate, error)) {
        return candidate;
      }
      if (dir == dir.parent_path()) {
        break;
      }
      dir = dir.parent_path();
    }
  }
  return {};
}

}  // namespace vsim
