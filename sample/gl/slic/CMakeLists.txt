﻿set(target "gl_slic")
message(STATUS "${target}")

project(${target})

include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
