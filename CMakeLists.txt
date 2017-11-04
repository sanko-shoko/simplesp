cmake_minimum_required(VERSION 2.8)
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

project(simplesp)

include_directories(${CMAKE_SOURCE_DIR})

## external
include_directories(${CMAKE_SOURCE_DIR}/external)
link_directories(${CMAKE_SOURCE_DIR}/external)
file(GLOB EX_LIBS ${CMAKE_SOURCE_DIR}/external/*.lib)


## openCV
option(SP_USE_OPENCV "SP_USE_OPENCV" OFF)
if(SP_USE_OPENCV)
	find_package(OpenCV REQUIRED)
	
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIB_DIRS})
endif(SP_USE_OPENCV)


## openGL
option(SP_USE_OPENGL "SP_USE_OPENGL" OFF)
if(SP_USE_OPENGL)
	find_package(OpenGL REQUIRED)
	find_package(GLFW REQUIRED)
	
    include_directories(${GLFW_INCLUDE_DIR})
	set(OpenGL_LIBS ${OPENGL_LIBRARY} ${GLFW_LIB})
endif()


## OpenMP
option(SP_USE_OPENMP "SP_USE_OPENMP" ON)
if(SP_USE_OPENMP) 
	find_package(OpenMP)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


## folder
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "cmake")


add_subdirectory(sample)

