﻿set(target "gl_icp")
message(STATUS "${target}")

project(${target})

include(../../cmake_base.txt)

use_external_library(${target} imgui)

set_target_properties(${target} PROPERTIES
    FOLDER "gl"
)
