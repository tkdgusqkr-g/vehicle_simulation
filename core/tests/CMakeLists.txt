add_executable(vsim_tests
  test_can.cpp
  test_car_sound.cpp
  test_driveline.cpp
  test_elec.cpp
  test_environment.cpp
  test_full_vehicle.cpp
  test_input.cpp
  test_integrator.cpp
  test_lap.cpp
  test_mpc.cpp
  test_link.cpp
  test_params.cpp
  test_plugins.cpp
  test_powertrain.cpp
  test_rng.cpp
  test_sensors.cpp
  test_signal_bus.cpp
  test_simulation.cpp
  test_single_track.cpp
  test_suspension.cpp
  test_thermal.cpp
  test_tire.cpp
  test_track.cpp
  test_track_mesh.cpp
  test_visual.cpp
  test_two_track.cpp)
target_link_libraries(vsim_tests PRIVATE vsim_core GTest::gtest_main nlohmann_json::nlohmann_json)
target_compile_definitions(vsim_tests PRIVATE VSIM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
add_dependencies(vsim_tests simple_vcu tv_cpp tv_fmu fake_ert_vcu)  # the example plugins the tests load
vsim_set_warnings(vsim_tests)
# The Unreal plugin's engine-free headers (the car sound synthesizer).
set_property(SOURCE test_car_sound.cpp APPEND PROPERTY INCLUDE_DIRECTORIES
             ${PROJECT_SOURCE_DIR}/ui/unreal/VsimViewer/Plugins/VsimLink/Source/VsimLink/Public)

include(GoogleTest)
gtest_discover_tests(vsim_tests DISCOVERY_MODE PRE_TEST)

# Wheel input service and the vsim-input tool (needs SDL3).
if(VSIM_BUILD_APP AND VSIM_WITH_SDL)
  add_executable(vsim_input_tests test_input_service.cpp)
  target_link_libraries(vsim_input_tests PRIVATE vsim_input SDL3::SDL3 GTest::gtest_main)
  vsim_set_warnings(vsim_input_tests)
  gtest_discover_tests(vsim_input_tests DISCOVERY_MODE PRE_TEST)
  # Runs without any device attached (CI).
  add_test(NAME vsim_input_list COMMAND vsim-input list)
endif()

# Shared-memory link across processes: vsim-core plus vsim-link as the viewer.
if(VSIM_BUILD_APP)
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME vsim_link_e2e
             COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/link_e2e.py $<TARGET_FILE:vsim-core>
                     $<TARGET_FILE:vsim-link>
                     ${PROJECT_SOURCE_DIR}/templates/projects/example_fs_ev/scenarios/drive.yaml)
    set_tests_properties(vsim_link_e2e PROPERTIES TIMEOUT 60)
  endif()
endif()

# Python plugins in vsim-core's embedded interpreter.
if(VSIM_BUILD_APP AND TARGET pybind11::embed)
  add_test(NAME vsim_core_python_plugin
           COMMAND vsim-core --scenario ${PROJECT_SOURCE_DIR}/templates/projects/example_fs_ev/scenarios/skidpad_auto.yaml
                   --mode fast --duration 2 --no-link --no-wheel --set components.tv_py.enabled=true)
  set_tests_properties(vsim_core_python_plugin PROPERTIES TIMEOUT 60)
endif()

# The Unreal plugin against stand-in engine headers (no Unreal here).
if(Python3_Interpreter_FOUND AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_test(NAME vsim_unreal_plugin_syntax
           COMMAND Python3::Interpreter ${PROJECT_SOURCE_DIR}/ui/unreal/tests/check_plugin_syntax.py ${CMAKE_CXX_COMPILER})
endif()
