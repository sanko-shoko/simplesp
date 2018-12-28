set(target "testcore")
message(STATUS "${target}")

project(${target})

include(../../cmake_devel.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "_devel"
)
