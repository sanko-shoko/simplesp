set(target "sp_rot")
message(STATUS "${target}")

include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
	FOLDER "sp"
)
