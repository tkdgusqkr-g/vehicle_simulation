add_executable(vsim_tests
  test_driveline.cpp
  test_integrator.cpp
  test_params.cpp
  test_powertrain.cpp
  test_rng.cpp
  test_signal_bus.cpp
  test_simulation.cpp
  test_single_track.cpp
  test_tire.cpp
  test_two_track.cpp)
target_link_libraries(vsim_tests PRIVATE vsim_core GTest::gtest_main nlohmann_json::nlohmann_json)
target_compile_definitions(vsim_tests PRIVATE VSIM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
vsim_set_warnings(vsim_tests)

include(GoogleTest)
gtest_discover_tests(vsim_tests DISCOVERY_MODE PRE_TEST)
