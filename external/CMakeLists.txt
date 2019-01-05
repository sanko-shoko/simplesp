## common
add_subdirectory(tinyfd)

## gl code
if(SP_USE_OPENGL)
    add_subdirectory(glew)
    add_subdirectory(imgui)
endif(SP_USE_OPENGL)

