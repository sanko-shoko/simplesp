set(target "bitmarker")
message(STATUS "${target}")

include(../../cmake_imgui.txt)
include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "_devel"
)
