set(target "cv_dotpattern")
message(STATUS "${target}")

include(../../cmake_base.txt)

set_target_properties(${target} PROPERTIES
	FOLDER "cv"
)
