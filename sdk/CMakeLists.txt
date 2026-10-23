# Plugin SDK examples (design doc §11.6), built into ${VSIM_PLUGIN_OUTPUT_DIR}
# where vsim finds them by name (components.<name>.path: tv_cpp).

if(WIN32)
  set(VSIM_FMI2_PLATFORM win64)
elseif(APPLE)
  set(VSIM_FMI2_PLATFORM darwin64)
else()
  set(VSIM_FMI2_PLATFORM linux64)
endif()

function(vsim_example_plugin target output_dir)
  set_target_properties(${target} PROPERTIES
    PREFIX ""  # the same file name on every platform
    LIBRARY_OUTPUT_DIRECTORY "$<1:${output_dir}>"
    RUNTIME_OUTPUT_DIRECTORY "$<1:${output_dir}>"
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden)
  if(MSVC)
    target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS)
  endif()
endfunction()

# C: the whole ABI in one file, taking the vcu slot.
add_library(simple_vcu MODULE examples/simple_vcu_c/simple_vcu.c)
target_include_directories(simple_vcu PRIVATE c/include)
vsim_example_plugin(simple_vcu "${VSIM_PLUGIN_OUTPUT_DIR}")

# C++ with the header-only helper: torque vectoring after the VCU.
add_library(tv_cpp MODULE examples/tv_cpp/tv_cpp.cpp)
target_include_directories(tv_cpp PRIVATE c/include cpp/include)
vsim_example_plugin(tv_cpp "${VSIM_PLUGIN_OUTPUT_DIR}")

# The same law as an FMI 2.0 Co-Simulation FMU: binary under
# binaries/<platform>/, zipped with modelDescription.xml into tv_fmu.fmu.
set(_stage "${CMAKE_CURRENT_BINARY_DIR}/tv_fmu_stage")
add_library(tv_fmu MODULE examples/tv_fmu/tv_fmu.c)
target_include_directories(tv_fmu PRIVATE fmi2/include)
vsim_example_plugin(tv_fmu "${_stage}/binaries/${VSIM_FMI2_PLATFORM}")
add_custom_command(TARGET tv_fmu POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/examples/tv_fmu/modelDescription.xml" "${_stage}/"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${VSIM_PLUGIN_OUTPUT_DIR}"
  COMMAND ${CMAKE_COMMAND} -E tar cf "${VSIM_PLUGIN_OUTPUT_DIR}/tv_fmu.fmu" --format=zip modelDescription.xml binaries
  WORKING_DIRECTORY "${_stage}"
  COMMENT "Packaging tv_fmu.fmu"
  VERBATIM)

# The Simulink wrapper template (sdk/simulink) on a stand-in for Embedded
# Coder output, so the code vsim_export writes is compiled and tested here.
include(simulink/vsim_ert_wrapper.cmake)
set(_fake "${CMAKE_CURRENT_SOURCE_DIR}/simulink/tests/fake_ert")
vsim_fill_ert_wrapper(OUT "${CMAKE_CURRENT_BINARY_DIR}/fake_vcu_vsim.c" MODEL fake_vcu STEP 0.01 VERSION 1.0
  PORTS "in|APPS|%|APPS|double" "in|WheelSpeedFL|rpm|WheelSpeedFL|single" "in|Enable||Enable|boolean"
        "out|TorqueCmd|N*m|TorqueCmd|double" "out|WheelSpeedEcho|rpm|WheelSpeedEcho|single"
        "out|Count||Count|int32")
add_library(fake_ert_vcu MODULE "${_fake}/fake_vcu.c" "${CMAKE_CURRENT_BINARY_DIR}/fake_vcu_vsim.c")
target_include_directories(fake_ert_vcu PRIVATE c/include "${_fake}")
vsim_example_plugin(fake_ert_vcu "${VSIM_PLUGIN_OUTPUT_DIR}")
