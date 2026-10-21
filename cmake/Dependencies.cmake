# Third-party dependencies, fetched at configure time.
include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# yaml-cpp 0.8.0 declares cmake_minimum_required(3.4), which CMake 4 rejects
# unless a policy floor is given.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# --- yaml-cpp: scenario and project files -----------------------------------
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
  GIT_SHALLOW TRUE
  SYSTEM)

# --- nlohmann/json: run metadata, log records, Sim API messages -------------
FetchContent_Declare(nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  SYSTEM)

# --- mcap: run logs (header-only, used without compression) -----------------
FetchContent_Declare(mcap
  GIT_REPOSITORY https://github.com/foxglove/mcap.git
  GIT_TAG releases/cpp/v2.1.3
  GIT_SHALLOW TRUE
  # Only the headers are needed; pointing at a directory without a
  # CMakeLists.txt keeps FetchContent from configuring the repository.
  SOURCE_SUBDIR cpp/mcap/include)

FetchContent_MakeAvailable(yaml-cpp nlohmann_json mcap)

add_library(vsim_mcap INTERFACE)
target_include_directories(vsim_mcap SYSTEM INTERFACE "${mcap_SOURCE_DIR}/cpp/mcap/include")
target_compile_definitions(vsim_mcap INTERFACE MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)

# --- IXWebSocket: Sim API server for the live view --------------------------
if(VSIM_BUILD_APP)
  set(USE_TLS OFF CACHE BOOL "" FORCE)
  set(USE_ZLIB OFF CACHE BOOL "" FORCE)
  set(IXWEBSOCKET_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(ixwebsocket
    GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
    GIT_TAG v11.4.6
    GIT_SHALLOW TRUE
    SYSTEM)
  FetchContent_MakeAvailable(ixwebsocket)
endif()

# --- GoogleTest ---------------------------------------------------------------
if(VSIM_BUILD_TESTS)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    GIT_SHALLOW TRUE
    SYSTEM)
  FetchContent_MakeAvailable(googletest)
endif()

# --- pybind11: installed with pip (`pip install pybind11`) ------------------
if(VSIM_BUILD_PYTHON)
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
  if(NOT pybind11_DIR)
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
      OUTPUT_VARIABLE _vsim_pybind11_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _vsim_pybind11_result)
    if(_vsim_pybind11_result EQUAL 0)
      set(pybind11_DIR "${_vsim_pybind11_dir}")
    endif()
  endif()
  find_package(pybind11 CONFIG REQUIRED)
endif()
