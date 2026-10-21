pybind11_add_module(_vsim src/bindings.cpp)
target_link_libraries(_vsim PRIVATE vsim_core)
# Build straight into the package directory so `import vsim` works with
# PYTHONPATH=python (the generator expression stops per-config subfolders).
set_target_properties(_vsim PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "$<1:${CMAKE_CURRENT_SOURCE_DIR}/vsim>")
