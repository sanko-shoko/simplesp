set(target "sp_slic")
message(STATUS "${target}")

project(${target})

include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "sp"
)
