set(target "glew")
message(STATUS "${target}")

project(${target})

file(GLOB MAIN ${SIMPLESP_DIR}/external/GL/*glew.h *.c)
source_group("main" FILES ${MAIN})

add_compile_definitions(GLEW_STATIC)

add_library(${target} ${MAIN})

target_link_libraries(${target} ${OpenGL_LIBS})

set_target_properties(${target} PROPERTIES
    FOLDER "_external"
)
