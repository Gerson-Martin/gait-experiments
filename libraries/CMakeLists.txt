cmake_minimum_required(VERSION 3.0)

# Find cmake modules - they all must be in the cmake subdirectory
# let cmake use them
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

#old simple way
add_subdirectory(i-plot)

find_package(fcontrol)



if(FCONTROL_LIBRARY)
    message(STATUS "Using system fcontrol library found.")
    set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} ${FCONTROL_INCLUDE_DIRS})
    set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} ${FCONTROL_LIBRARIES})
else()
    message(STATUS "System installed fcontrol not found. Compiling from source")
    add_subdirectory(fcontrol)
endif()



set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES}
    PARENT_SCOPE)
set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES}
    PARENT_SCOPE)
