option(ENABLE_zmpViewer "Enable/disable zmpViewer program" ON)

if(ENABLE_zmpViewer)

    add_executable(zmpViewer main.cpp
                             ZmpViewer.cpp
                             ZmpViewer.hpp
                             walkingRobot.cpp
                             walkingRobot.hpp)

    target_link_libraries(zmpViewer YARP::YARP_os
                                    YARP::YARP_init
                                    YARP::YARP_sig
                                    YARP::YARP_dev
                                    ${orocos_kdl_LIBRARIES}
                                    ROBOTICSLAB::KdlVectorConverterLib
                                    ROBOTICSLAB::KinematicsDynamicsInterfaces)

    install(TARGETS zmpViewer)

endif()
