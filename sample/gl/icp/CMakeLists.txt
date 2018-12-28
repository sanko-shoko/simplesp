set(target "gl_icp")
message(STATUS "${target}")

project(${target})

include(../../cmake_base.txt)
include(../../cmake_imgui.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
