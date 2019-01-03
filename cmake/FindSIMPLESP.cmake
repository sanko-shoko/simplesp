# SIMPLESP_FOUND
# SIMPLESP_DIR
#

find_path (SIMPLESP_DIR NAMES simplesp.h
    PATHS
        "${CMAKE_BINARY_DIR}"
        "${SIMPLESP_ROOT_DIR}"
        /usr/
        /usr/local/
)

if(SIMPLESP_DIR)
    set(SIMPLESP_FOUND TRUE)
endif(SIMPLESP_DIR)