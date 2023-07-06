
#ifndef __WALKING_ROBOT_HPP__
#define __WALKING_ROBOT_HPP__

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionDirect.h>
#include <ICartesianSolver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <KdlVectorConverter.hpp> // TODO: unused
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

class walkingRobot{
public:
    walkingRobot(){}
    bool initWalkingRobot(std::string robotRemote, std::string local);
    bool initSensor(std::string sensorRemote, double period);
    yarp::dev::PolyDriver robotDevice_Right;
    yarp::dev::PolyDriver robotDevice_Left;
    yarp::dev::PolyDriver solverDevice_Right;
    yarp::dev::PolyDriver solverDevice_Left;
    roboticslab::ICartesianSolver * solver_Right;
    roboticslab::ICartesianSolver * solver_Left;
    std::vector<double> previousJointPose_Right;
    std::vector<double> previousJointPose_Left;
    yarp::dev::IEncoders * enc_Right;
    yarp::dev::IControlLimits * limits_Right;
    yarp::dev::IEncoders * enc_Left;
    yarp::dev::IControlLimits * limits_Left;
    yarp::os::ResourceFinder rf;
    int sensorIndex;
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::ISixAxisForceTorqueSensors * sensor;
    KDL::Rotation R_N_sensor;
    KDL::Rotation R_N_sole;
    std::string local,robotRemote;
    YARP_LOG_COMPONENT(WR, "walkingRobot")

    bool parseFrameRotation(const yarp::os::Value &value, const std::string &name, KDL::Rotation &rot);
};

#endif
