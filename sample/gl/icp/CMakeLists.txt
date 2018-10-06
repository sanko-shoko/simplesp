set(target "gl_icp")
message(STATUS "${target}")

include(../../cmake_imgui.txt)
include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
