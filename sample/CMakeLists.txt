## sp code
add_subdirectory(sp)

## cv code
if(SP_USE_OPENCV)
	add_subdirectory(cv)
endif(SP_USE_OPENCV)

## gl code
if(SP_USE_OPENGL)
	add_subdirectory(gl)
endif(SP_USE_OPENGL)


## xx code
option(SP_XX "SP_XX" OFF)
if(SP_XX)
	add_subdirectory(xx)
endif(SP_XX)


## devel
option(SP_DEVEL "SP_DEVEL" OFF)
if(SP_DEVEL)
	add_subdirectory(_devel)
endif(SP_DEVEL)

