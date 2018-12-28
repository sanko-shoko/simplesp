set(target "tinyfd")
message(STATUS "${target}")

project(${target})

file(GLOB MAIN *.h *.c *.hpp *.cpp)
source_group("main" FILES ${MAIN})

add_library(${target} ${MAIN})

set_target_properties(${target} PROPERTIES
    FOLDER "_external"
)
