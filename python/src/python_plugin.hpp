#pragma once

namespace vsim {

// Registers the `python` plugin type (design doc §11.2). `embedded`: the
// host is vsim-core, which starts an interpreter the first time a Python
// component loads; otherwise the host is the Python module itself.
void register_python_plugins(bool embedded);

}  // namespace vsim
