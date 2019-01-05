cmake_minimum_required(VERSION 3.1)
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set (CMAKE_CXX_STANDARD 11)

project(simplesp)

## output
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/)


## simplesp
find_package(SIMPLESP REQUIRED)
include_directories(${SIMPLESP_DIR})


## external
include_directories(${SIMPLESP_DIR}/external)
link_directories(${SIMPLESP_DIR}/external)
file(GLOB SP_EX_LIBS ${SIMPLESP_DIR}/external/*.lib)

function(use_external target name)
    include_directories("${SIMPLESP_DIR}/external/${name}")
    target_link_libraries(${target} ${name})
endfunction()


## openCV
option(SP_USE_OPENCV "SP_USE_OPENCV" OFF)
if(SP_USE_OPENCV)
    find_package(OpenCV REQUIRED)
    
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIB_DIRS})
    
    set(SP_CV_LIBS ${OpenCV_LIBS})
endif(SP_USE_OPENCV)


## openGL
option(SP_USE_OPENGL "SP_USE_OPENGL" OFF)
if(SP_USE_OPENGL)
    find_package(OpenGL REQUIRED)
    find_package(GLFW REQUIRED)
    
    include_directories(${GLFW_INCLUDE_DIR})
    
    set(SP_GL_LIBS ${OPENGL_LIBRARY} ${GLFW_LIB})
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

add_subdirectory(external)
add_subdirectory(sample)

