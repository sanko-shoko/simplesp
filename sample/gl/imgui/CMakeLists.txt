set(target "gl_imgui")
message(STATUS "${target}")

include(../../cmake_base.txt)
include(../../cmake_imgui.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
