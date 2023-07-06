cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(fcontrol)

# Find cmake modules - they all must be in the cmake subdirectory
# let cmake use them
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#deps
#1-fftw
#find_package(FFTW REQUIRED)
#2-eigen
#find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Eigen3 NO_MODULE)

message(STATUS "EIGEN: " ${EIGEN3_VERSION} " (system installed version)")

if(Eigen3_FOUND=FALSE)
    add_subdirectory(3rdparty/eigen)
    set(EIGEN3_INCLUDE_DIR 3rdparty/eigen)
endif(Eigen3_FOUND=FALSE)
include_directories(${EIGEN3_INCLUDE_DIR})

# source files for fcontrol
aux_source_directory(./src FCONTROL_SRCS)
# include files for fcontrol
FILE(GLOB_RECURSE FCONTROL_INCLUDE_FILES src/*.h src/*.hpp)
FILE(GLOB_RECURSE FCONTROL_SHARE_FILES files/*)

set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} ${PROJECT_NAME})# PARENT_SCOPE)
set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR})
    #PARENT_SCOPE)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${FCONTROL_SRCS} ${FCONTROL_INCLUDE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11")
#target_compile_features(${PROJECT_NAME} PUBLIC cxx_11)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

## In case of system installation ("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")=TRUE
if("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    #install target
    message(STATUS "Configuring " ${PROJECT_NAME} " as main project. Use only if you intend to install library")
    install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION bin/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
    install(FILES ${FCONTROL_INCLUDE_FILES}
        DESTINATION include/${PROJECT_NAME})
    install(FILES ${FCONTROL_SHARE_FILES}
        DESTINATION share/${PROJECT_NAME})

    #Save the exports inside build as fcontrolConfig.cmake
    set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})

    #build tree
    export(EXPORT ${PROJECT_NAME}Targets FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        "include(\"\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Targets.cmake\")")

    #install tree
    install(EXPORT ${PROJECT_NAME}Targets FILE "${PROJECT_NAME}Targets.cmake"
        DESTINATION ${ConfigPackageLocation})
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        DESTINATION ${ConfigPackageLocation})

    if(UNIX)
        file(WRITE ${CMAKE_BINARY_DIR}/fcontrol.conf "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
        install(FILES ${CMAKE_BINARY_DIR}/fcontrol.conf
                DESTINATION "/etc/ld.so.conf.d")
    endif()

    # uninstall target
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

## In case of submodule ("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")=FALSE
else("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    message(STATUS "Configuring " ${PROJECT_NAME} " as sub-project in " ${CMAKE_CURRENT_SOURCE_DIR})

    set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} PARENT_SCOPE)
    set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES} PARENT_SCOPE)

    # Config files
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/files/phi.m_exp.csv ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)


endif() #("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")


target_link_libraries( ${PROJECT_NAME} ${FFTW_LIBRARIES} ${Eigen3_LIBRARIES})

