set(target "gl_shader")
message(STATUS "${target}")

project(${target})

include(../../cmake_base.txt)
target_link_libraries(${target} glew)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
