set(target "xx_gtmaker")
message(STATUS "${target}")

include(../../cmake_imgui.txt)
include(../../cmake_tinyfd.txt)
include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
	FOLDER "xx"
)
