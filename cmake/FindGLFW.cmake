# GLFW_FOUND
# GLFW_INCLUDE_DIR
# GLFW_LIB
#

find_path (GLFW_DIR NAMES include/GLFW/glfw3.h
    PATHS
        "${GLFW_ROOT_DIR}"
        /usr/
        /usr/local/
)

if (GLFW_DIR)
    find_path (GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h PATHS "${GLFW_DIR}/include")

    if (WIN32)
        if (MSVC11 OR (${MSVC_VERSION} EQUAL 1700))
            find_library (GLFW_LIB NAMES glfw3
                PATHS
                    "${GLFW_DIR}/lib"
                    "${GLFW_DIR}/lib-vc2012"
            )
        elseif (MSVC12 OR (${MSVC_VERSION} EQUAL 1800))
            find_library (GLFW_LIB NAMES glfw3
                PATHS
                    "${GLFW_DIR}/lib"
                    "${GLFW_DIR}/lib-vc2013"
            )
        elseif (MSVC14 AND (${MSVC_VERSION} EQUAL 1900))
            find_library (GLFW_LIB NAMES glfw3
                PATHS
                    "${GLFW_DIR}/lib"
                    "${GLFW_DIR}/lib-vc2015"
            )
        elseif (MSVC14 AND (${MSVC_VERSION} GREATER_EQUAL 1910) AND (${MSVC_VERSION} LESS 1920))
            find_library (GLFW_LIB NAMES glfw3
                PATHS
                    "${GLFW_DIR}/lib"
                    "${GLFW_DIR}/lib-vc2017"
            )
        elseif (MSVC14 AND (${MSVC_VERSION} GREATER_EQUAL 1920) AND (${MSVC_VERSION} LESS 1930))
            find_library (GLFW_LIB NAMES glfw3
                PATHS
                    "${GLFW_DIR}/lib"
                    "${GLFW_DIR}/lib-vc2019"
            )
        elseif (MINGW)
            if (CMAKE_CL_64)
                find_library (GLFW_LIB NAMES glfw3
                    PATHS
                        "${GLFW_DIR}/lib"
                        "${GLFW_DIR}/lib-mingw-w64"
                )
            else ()
                find_library (GLFW_LIB NAMES glfw3
                    PATHS
                        "${GLFW_DIR}/lib"
                        "${GLFW_DIR}/lib-mingw"
                )
            endif ()
        else()
            find_library (GLFW_LIB NAMES glfw3
                PATHS
                    "${GLFW_DIR}/lib"
            )
        endif()
    else ()
        find_library (GLFW_LIB NAMES glfw glfw3
            PATHS
                "${GLFW_DIR}/lib"
                "${GLFW_DIR}/lib/x11"
        )
        if (APPLE)
            list (APPEND GLFW_LIB
                "-framework Cocoa"
                "-framework CoreVideo"
                "-framework IOKit"
            )
        else ()
            find_package(Threads REQUIRED)
        
            find_package(X11 REQUIRED)

            # Set up library and include paths
            list(APPEND GLFW_INCLUDE_DIR "${X11_X11_INCLUDE_PATH}")
            list(APPEND GLFW_LIB "${X11_X11_LIB}" "${CMAKE_THREAD_LIBS_INIT}" "${CMAKE_DL_LIBS}")

            # Check for XRandR (modern resolution switching and gamma control)
            if (NOT X11_Xrandr_FOUND)
                message(FATAL_ERROR "The RandR library and headers were not found")
            endif()

            list(APPEND GLFW_INCLUDE_DIR "${X11_Xrandr_INCLUDE_PATH}")
            list(APPEND GLFW_LIB "${X11_Xrandr_LIB}")

            # Check for Xinerama (legacy multi-monitor support)
            if (NOT X11_Xinerama_FOUND)
                message(FATAL_ERROR "The Xinerama library and headers were not found")
            endif()

            list(APPEND GLFW_INCLUDE_DIR "${X11_Xinerama_INCLUDE_PATH}")
            list(APPEND GLFW_LIB "${X11_Xinerama_LIB}")

            # Check for Xkb (X keyboard extension)
            if (NOT X11_Xkb_FOUND)
                message(FATAL_ERROR "The X keyboard extension headers were not found")
            endif()

            list(APPEND GLFW_INCLUDE_DIR "${X11_Xkb_INCLUDE_PATH}")

            # Check for Xcursor
            if (NOT X11_Xcursor_FOUND)
                message(FATAL_ERROR "The Xcursor libraries and headers were not found")
            endif()

            list(APPEND GLFW_INCLUDE_DIR "${X11_Xcursor_INCLUDE_PATH}")
            list(APPEND GLFW_LIB "${X11_Xcursor_LIB}")

            # Check for Xrandr
            if(NOT X11_Xrandr_FOUND)
                message(FATAL_ERROR "Xrandr library not found - required for GLFW")
            endif()

            list(APPEND GLFW_LIB "${X11_Xrandr_LIB}")

            # Check for xf86vmode
            if(NOT X11_xf86vmode_FOUND)
                message(FATAL_ERROR "xf86vmode library not found - required for GLFW")
            endif()

            list(APPEND GLFW_LIB "${X11_Xxf86vm_LIB}")

        endif ()
    endif()
endif (GLFW_DIR)

if(GLFW_INCLUDE_DIR AND GLFW_LIB)
    SET(GLFW_FOUND TRUE)
    message(STATUS "GLFW include: ${GLFW_INCLUDE_DIR}")
    message(STATUS "GLFW library: ${GLFW_LIB}")
else()
    message(SEND_ERROR "GLFW not found")
endif(GLFW_INCLUDE_DIR AND GLFW_LIB)


