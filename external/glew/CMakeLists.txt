set(target "glew")
message(STATUS "${target}")

project(${target})

include_directories(./)

file(GLOB MAIN GL/*.h *.c)
source_group("main" FILES ${MAIN})

add_compile_options(-D GLEW_STATIC)

add_library(${target} ${MAIN})

target_link_libraries(${target} ${SP_GL_LIBS})

set_target_properties(${target} PROPERTIES
    FOLDER "_external"
)
