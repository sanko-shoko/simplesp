set(target "testcore")
message(STATUS "${target}")

project(${target})

include(../../cmake_devel.txt)

use_external_library(${target} glew)
use_external_library(${target} imgui)

set_target_properties(${target} PROPERTIES
    FOLDER "_devel"
)
