set(target "gl_imgui")
message(STATUS "${target}")

include(../../cmake_imgui.txt)
include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
