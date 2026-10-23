#pragma once

#include <filesystem>

namespace vsim {

// Registers the `python` plugin type through the Python host library
// (vsim_python_host, built next to vsim-core): loaded the first time a
// Python component is created, so vsim-core itself does not need Python.
void register_python_host(const std::filesystem::path& library);

}  // namespace vsim
