# Fills vsim_ert_wrapper.c.in for an Embedded Coder model, as vsim_export.m
# does in MATLAB (the two must write the same port table):
#
#   vsim_fill_ert_wrapper(OUT <file.c> MODEL <name> STEP <s> VERSION <v>
#     PORTS "in|APPS|%|APPS|double" "out|TorqueCmd|N*m|TorqueCmd|double" ...)
#
# Port: direction|vsim port name|unit|field in <model>_U/_Y|type (double,
# single, int8, uint8, int16, uint16, int32, uint32, boolean).
set(_vsim_ert_template "${CMAKE_CURRENT_LIST_DIR}/vsim_ert_wrapper.c.in")

function(vsim_fill_ert_wrapper)
  cmake_parse_arguments(A "" "OUT;MODEL;STEP;VERSION" "PORTS" ${ARGN})
  set(inputs "")
  set(outputs "")
  set(n_in 0)
  set(n_out 0)
  foreach(port IN LISTS A_PORTS)
    string(REPLACE "|" ";" f "${port}")
    list(GET f 0 dir)
    list(GET f 1 name)
    list(GET f 2 unit)
    list(GET f 3 field)
    list(GET f 4 type)
    string(TOUPPER "${type}" type)
    if(dir STREQUAL "in")
      string(APPEND inputs "    {{\"${name}\", \"${unit}\", VSIM_IN}, offsetof(ExtU_${A_MODEL}_T, ${field}), VSIM_T_${type}},\n")
      math(EXPR n_in "${n_in} + 1")
    else()
      string(APPEND outputs "    {{\"${name}\", \"${unit}\", VSIM_OUT}, offsetof(ExtY_${A_MODEL}_T, ${field}), VSIM_T_${type}},\n")
      math(EXPR n_out "${n_out} + 1")
    endif()
  endforeach()
  file(READ "${_vsim_ert_template}" text)
  string(REPLACE "@INPUTS@" "${inputs}" text "${text}")
  string(REPLACE "@OUTPUTS@" "${outputs}" text "${text}")
  string(REPLACE "@N_IN@" "${n_in}" text "${text}")
  string(REPLACE "@N_OUT@" "${n_out}" text "${text}")
  string(REPLACE "@MODEL@" "${A_MODEL}" text "${text}")
  string(REPLACE "@STEP@" "${A_STEP}" text "${text}")
  string(REPLACE "@VERSION@" "${A_VERSION}" text "${text}")
  file(WRITE "${A_OUT}" "${text}")
endfunction()
