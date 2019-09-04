set(target "gl_pathtracing")
message(STATUS "${target}")

project(${target})

include(../../cmake_base.txt)

use_external_library(${target} glew)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
