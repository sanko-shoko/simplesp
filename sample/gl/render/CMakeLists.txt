set(target "gl_render")
message(STATUS "${target}")

include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
	FOLDER "gl"
)
