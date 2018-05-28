set(target "gl_randomdot")
message(STATUS "${target}")

include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
