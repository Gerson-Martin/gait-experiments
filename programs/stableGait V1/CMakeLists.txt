if(NOT TARGET ROBOTICSLAB::KdlVectorConverterLib AND (NOT DEFINED ENABLE_stableGait OR ENABLE_stableGait))
    message(WARNING "KdlVectorConverterLib not found, disabling stableGait")
endif()

cmake_dependent_option(ENABLE_stableGait "Enable/disable stableGait program" ON
                       TARGET ROBOTICSLAB::KdlVectorConverterLib OFF)

if(ENABLE_stableGait)

    add_executable(stableGait main.cpp
                              LimitChecker.hpp
                              LimitChecker.cpp
                              StepGenerator.hpp
                              StepGenerator.cpp
                              GaitSpecs.hpp
                              TrajectoryGenerator.hpp
                              TrajectoryGenerator.cpp
                              TargetBuilder.hpp
                              TargetBuilder.cpp
                              LIMP.hpp
                              LIMP.cpp
                              VelocityProfile_Custom.cpp
                              VelocityProfile_Custom.h)

    target_link_libraries(stableGait YARP::YARP_os
                                     YARP::YARP_init
                                     YARP::YARP_dev
                                     ROBOTICSLAB::KdlVectorConverterLib
                                     ROBOTICSLAB::KinematicsDynamicsInterfaces
                                     ${SUBDIR_LINK_NAMES})

    install(TARGETS stableGait)

else()

    set(ENABLE_stableGait OFF CACHE BOOL "Enable/disable stableGait program" FORCE)

endif()
