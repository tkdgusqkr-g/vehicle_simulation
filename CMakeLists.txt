cmake_minimum_required(VERSION 3.25)

project(vsim VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Static libraries end up inside the Python extension module.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(VSIM_BUILD_APP "Build the vsim-core executable" ON)
option(VSIM_BUILD_TESTS "Build C++ unit tests" ON)
option(VSIM_BUILD_PYTHON "Build the Python extension module" ON)
option(VSIM_WITH_SDL "Steering wheel input and force feedback (SDL3) in the apps" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Warnings)
include(Dependencies)

add_subdirectory(core)
add_subdirectory(sdk)

if(VSIM_BUILD_PYTHON)
  add_subdirectory(python)
endif()

if(VSIM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(core/tests)
endif()
