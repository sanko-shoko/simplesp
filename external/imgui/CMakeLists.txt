set(target "imgui")
message(STATUS "${target}")

project(${target})

file(GLOB MAIN *.h *.c *.hpp *.cpp)
source_group("main" FILES ${MAIN})

add_library(${target} ${MAIN})

target_link_libraries(${target} ${OpenGL_LIBS})

set_target_properties(${target} PROPERTIES
    FOLDER "_external"
)
