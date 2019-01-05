set(target "bitmarker")
message(STATUS "${target}")

project(${target})

include(../../cmake_base.txt)

use_external(${target} imgui)

set_target_properties(${target} PROPERTIES
    FOLDER "_devel"
)
