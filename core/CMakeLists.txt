# Version and commit, baked in for run metadata (design doc R-32).
set(VSIM_GIT_COMMIT "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    OUTPUT_VARIABLE _vsim_commit
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _vsim_commit_result
    ERROR_QUIET)
  if(_vsim_commit_result EQUAL 0)
    set(VSIM_GIT_COMMIT "${_vsim_commit}")
  endif()
endif()
configure_file(src/version.cpp.in "${CMAKE_CURRENT_BINARY_DIR}/generated/version.cpp" @ONLY)

add_library(vsim_core STATIC
  src/control/reference_vcu.cpp
  src/driver/drivers.cpp
  src/driver/profile.cpp
  src/input/force_feedback.cpp
  src/input/wheel_profile.cpp
  src/link/shared_memory.cpp
  src/link/state_link.cpp
  src/log/mcap_logger.cpp
  src/models/driveline/driveline.cpp
  src/models/powertrain/battery.cpp
  src/models/powertrain/motor.cpp
  src/models/tire/magic_formula_tire.cpp
  src/models/tire/simple_tire.cpp
  src/models/tire/tire_factory.cpp
  src/models/vehicle/corners.cpp
  src/models/vehicle/single_track.cpp
  src/models/vehicle/suspension.cpp
  src/models/vehicle/two_track.cpp
  src/plugin/fmu_plugin.cpp
  src/plugin/native_plugin.cpp
  src/plugin/plugin.cpp
  src/plugin/python_host.cpp
  src/plugin/units.cpp
  src/sensors/sensor_chain.cpp
  src/sensors/sensor_system.cpp
  src/sim/integrator.cpp
  src/sim/params.cpp
  src/sim/realism.cpp
  src/sim/recorder.cpp
  src/sim/rng.cpp
  src/sim/scenario.cpp
  src/sim/signal_bus.cpp
  src/sim/simulation.cpp
  src/track/lap_timer.cpp
  src/track/speed_profile.cpp
  src/track/track.cpp
  src/track/track_mesh.cpp
  src/util/paths.cpp
  src/visual/gltf.cpp
  src/visual/mesh.cpp
  src/visual/vehicle_visual.cpp
  "${CMAKE_CURRENT_BINARY_DIR}/generated/version.cpp")
target_include_directories(vsim_core PUBLIC include "${PROJECT_SOURCE_DIR}/sdk/c/include"
                           PRIVATE "${PROJECT_SOURCE_DIR}/sdk/fmi2/include")
target_link_libraries(vsim_core PRIVATE yaml-cpp nlohmann_json::nlohmann_json vsim_mcap vsim_miniz ${CMAKE_DL_LIBS})
# Plugins built with vsim (sdk/examples) are found without a path.
set(VSIM_PLUGIN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/plugins" CACHE INTERNAL "")
target_compile_definitions(vsim_core PRIVATE VSIM_DEFAULT_PLUGIN_DIR="${VSIM_PLUGIN_OUTPUT_DIR}")
if(UNIX AND NOT APPLE)
  target_link_libraries(vsim_core PRIVATE rt)  # shm_open on glibc < 2.34
endif()
vsim_set_warnings(vsim_core)

if(VSIM_BUILD_APP)
  add_library(vsim_api STATIC src/api/api_server.cpp)
  target_include_directories(vsim_api PUBLIC include)
  target_link_libraries(vsim_api PRIVATE ixwebsocket)
  vsim_set_warnings(vsim_api)

  add_executable(vsim-core apps/vsim-core/main.cpp)
  target_link_libraries(vsim-core PRIVATE vsim_core vsim_api nlohmann_json::nlohmann_json)
  # Python plugins (design doc §11.2): a host library with the interpreter,
  # next to vsim-core, loaded the first time a Python component is used.
  if(VSIM_BUILD_PYTHON AND TARGET Python::Python AND TARGET pybind11::embed)
    add_library(vsim_python_host MODULE "${PROJECT_SOURCE_DIR}/python/src/python_plugin.cpp")
    target_link_libraries(vsim_python_host PRIVATE vsim_core pybind11::embed)
    target_include_directories(vsim_python_host PRIVATE "${PROJECT_SOURCE_DIR}/sdk/c/include")
    target_compile_definitions(vsim_python_host PRIVATE VSIM_PYTHON_HOST_LIBRARY
                               VSIM_PYTHON_DIR="${PROJECT_SOURCE_DIR}/python")
    set_target_properties(vsim_python_host PROPERTIES PREFIX "")
    add_dependencies(vsim-core vsim_python_host)
    target_compile_definitions(vsim-core PRIVATE VSIM_WITH_PYTHON)
  endif()
  if(WIN32)
    target_link_libraries(vsim-core PRIVATE winmm shell32)
  endif()
  vsim_set_warnings(vsim-core)

  add_executable(vsim-link apps/vsim-link/main.cpp)
  target_link_libraries(vsim-link PRIVATE vsim_core)
  if(WIN32)
    target_link_libraries(vsim-link PRIVATE winmm)
  endif()
  vsim_set_warnings(vsim-link)

  if(VSIM_WITH_SDL)
    # Steering wheel input and force feedback (design doc §9.3).
    add_library(vsim_input STATIC src/input/input_service.cpp)
    target_include_directories(vsim_input PUBLIC include)
    target_link_libraries(vsim_input PUBLIC vsim_core PRIVATE SDL3::SDL3)
    vsim_set_warnings(vsim_input)
    target_link_libraries(vsim-core PRIVATE vsim_input)
    target_compile_definitions(vsim-core PRIVATE VSIM_HAS_SDL=1)

    add_executable(vsim-input apps/vsim-input/main.cpp)
    target_link_libraries(vsim-input PRIVATE vsim_core SDL3::SDL3)
    vsim_set_warnings(vsim-input)
  endif()
endif()
