#include "walkingRobot.hpp"
#include <string.h>
bool walkingRobot::initWalkingRobot(std::string robotRemote, std::string local){
    this->robotRemote=robotRemote;
    this->local=local;
    // ----- robot device -----
    yarp::os::Property robotOptions {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotRemote+"/rightLeg")},
        {"local", yarp::os::Value(local + robotRemote+"/rightLeg")}
    };

    if (!robotDevice_Right.open(robotOptions))
    {
        yCError(WR) << "Failed to open robot device";
        return false;
    }



    if (!robotDevice_Right.view(enc_Right) || !robotDevice_Right.view(limits_Right) || !robotDevice_Right.view(posdRight) || !robotDevice_Right.view(modeRight))
    {
        yCError(WR) << "Failed to view robot control interfaces";
        return false;
    }


    enc_Right->getAxes(&numJoints);
    previousJointPose_Right.resize(numJoints);

    int retry = 0;

    while (!enc_Right->getEncoders(previousJointPose_Right.data()))
    {
        if (++retry == 10)
        {
            yCError(WR) << "Failed to get initial joint pose";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }

    yarp::os::Bottle bMin_right, bMax_right;

    for (int joint = 0; joint < numJoints; joint++)
    {
        double qMin, qMax;

        if (!limits_Right->getLimits(joint, &qMin, &qMax))
        {
            yCError(WR) << "Unable to retrieve position limits for joint" << joint;
            return false;
        }

        bMin_right.addFloat64(qMin);
        bMax_right.addFloat64(qMax);
    }
    yCInfo(WR) << "Joint position limits:" << bMin_right.toString() << bMax_right.toString();

    if (!modeRight->setControlModes(std::vector(numJoints, VOCAB_CM_POSITION_DIRECT).data()))
    {
        yCError(WR) << "Failed to set posd control mode";
        return false;
    }



    yarp::os::Property robotOptions2 {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotRemote+"/leftLeg")},
        {"local", yarp::os::Value(local + robotRemote+"/leftLeg")}
    };

    if (!robotDevice_Left.open(robotOptions2))
    {
        yCError(WR) << "Failed to open robot device";
        return false;
    }



    if (!robotDevice_Left.view(enc_Left) || !robotDevice_Left.view(limits_Left) || !robotDevice_Left.view(posdLeft) || !robotDevice_Left.view(modeLeft))
    {
        yCError(WR) << "Failed to view robot control interfaces";
        return false;
    }



    numJoints;
    enc_Left->getAxes(&numJoints);
    previousJointPose_Left.resize(numJoints);

    retry = 0;

    while (!enc_Left->getEncoders(previousJointPose_Left.data()))
    {
        if (++retry == 10)
        {
            yCError(WR) << "Failed to get initial joint pose";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }

     yarp::os::Bottle bMin_left, bMax_left;

    for (int joint = 0; joint < numJoints; joint++)
    {
        double qMin, qMax;

        if (!limits_Left->getLimits(joint, &qMin, &qMax))
        {
            yCError(WR) << "Unable to retrieve position limits for joint" << joint;
            return false;
        }

        bMin_left.addFloat64(qMin);
        bMax_left.addFloat64(qMax);
    }
    yCInfo(WR) << "Joint position limits:" << bMin_left.toString() << bMax_left.toString();

    if (!modeLeft->setControlModes(std::vector(numJoints, VOCAB_CM_POSITION_DIRECT).data()))
    {
        yCError(WR) << "Failed to set posd control mode";
        return false;
    }
    // ----- solver device -----

   yarp::os::Property solverOptions_Left {
       {"device", yarp::os::Value("KdlSolver")},
       {"kinematics", yarp::os::Value("teo-leftLeg.ini")},
       {"ikPos", yarp::os::Value("st")},
       {"invKinStrategy", yarp::os::Value("humanoidGait")},
       {"quiet", yarp::os::Value::getNullValue()}
   };

   solverOptions_Left.put("mins", yarp::os::Value::makeList(bMin_left.toString().c_str()));
   solverOptions_Left.put("maxs", yarp::os::Value::makeList(bMax_left.toString().c_str()));

   if (!solverDevice_Left.open(solverOptions_Left))
   {
       yCError(WR) << "Failed to open solver device";
       return false;
   }

   if (!solverDevice_Left.view(solver_Left))
   {
       yCError(WR) << "Failed to view solver interface";
       return false;
   }

//------------------------------------------------------------------------

   yarp::os::Property solverOptions_Right {
       {"device", yarp::os::Value("KdlSolver")},
       {"kinematics", yarp::os::Value("teo-rightLeg.ini")},
       {"ikPos", yarp::os::Value("st")},
       {"invKinStrategy", yarp::os::Value("humanoidGait")},
       {"quiet", yarp::os::Value::getNullValue()}
   };

   solverOptions_Right.put("mins", yarp::os::Value::makeList(bMin_right.toString().c_str()));
   solverOptions_Right.put("maxs", yarp::os::Value::makeList(bMax_right.toString().c_str()));

   if (!solverDevice_Right.open(solverOptions_Right))
   {
       yCError(WR) << "Failed to open solver device";
       return false;
   }

   if (!solverDevice_Right.view(solver_Right))
   {
       yCError(WR) << "Failed to view solver interface";
       return false;
   }
   return true;
}

bool walkingRobot::initSensor(std::string sensorRemote,double period)
{
    // ----- sensor device -----

    auto sensorFrameRPY = *yarp::os::Value::makeValue("(0 -90 0)");

    if (!parseFrameRotation(sensorFrameRPY, "sensor frame", R_N_sensor))
    {
        return false;
    }
    yarp::os::Property sensorOptions {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value(sensorRemote)},
        {"local", yarp::os::Value(local + sensorRemote)},
        {"timeout", yarp::os::Value(period * 0.5)}
    };

    if (!sensorDevice.open(sensorOptions))
    {
        yCError(WR) << "Failed to open sensor device";
        return false;
    }

    if (!sensorDevice.view(sensor))
    {
        yCError(WR) << "Failed to view sensor interface";
        return false;
    }
    sensorIndex=0;

    int retry = 0;

    while (sensor->getSixAxisForceTorqueSensorStatus(sensorIndex) != yarp::dev::MAS_OK)
    {
        if (++retry == 10)
        {
            yCError(WR) << "Failed to get first sensor read";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }
    return true;
}
bool walkingRobot::parseFrameRotation(const yarp::os::Value & value, const std::string & name, KDL::Rotation & rot)
{
    if (!value.isNull())
    {
        if (!value.isList() || value.asList()->size() != 3)
        {
            yCError(WR) << "Parameter" << name << "must be a list of 3 doubles";
            return false;
        }

        yCInfo(WR) << name << "RPY [deg]:" << value.toString();

        auto roll = value.asList()->get(0).asFloat64() * KDL::deg2rad;
        auto pitch = value.asList()->get(1).asFloat64() * KDL::deg2rad;
        auto yaw = value.asList()->get(2).asFloat64() * KDL::deg2rad;

        // sequence (old axes): 1. R_x(roll), 2. R_y(pitch), 3. R_z(yaw)
        rot = KDL::Rotation::RPY(roll, pitch, yaw);
    }
    else
    {
        yCInfo(WR) << "Using no" << name;
        rot = KDL::Rotation::Identity();
    }

    return true;
}
bool walkingRobot::checkLimitsRight(std::vector<double> &JointPose){


    for(int i=0;i<JointPose.size();i++){
      double qmax, qmin;
      if( !limits_Right->getLimits(i,&qmin,&qmax) || (qmin>JointPose[i] || qmax<JointPose[i])){
          yCError(WR) <<"N de articulacion: "<<i<<", Valor:"<<JointPose[i]<<", qmin="<<qmin<<", qmax="<<qmax;
          return false;
      }
    }
    return true;
}

bool walkingRobot::moviLeftPos(std::vector<double> &cartesianPosition)
{
    std::vector<double> JointPose(numJoints);
    if(inv_Left(cartesianPosition,JointPose)){
        posdLeft->setPositions(JointPose.data());
        return true;
    }
    else{
        yCError(WR) <<"Movi Left failure";
        return false;
    }
}

bool walkingRobot::moviRightPos(std::vector<double> &cartesianPosition)
{
    std::vector<double> JointPose(numJoints);
    if(inv_Right(cartesianPosition,JointPose)){
        posdRight->setPositions(JointPose.data());
        return true;
    }
    else{
        yCError(WR) <<"Movi Right failure";
        return false;
    }
}
bool walkingRobot::checkLimitsLeft(std::vector<double> &JointPose){


    for(int i=0;i<JointPose.size();i++){
      double qmax, qmin;
      if( !limits_Left->getLimits(i,&qmin,&qmax) || (qmin>JointPose[i] || qmax<JointPose[i])){
          yCError(WR) <<"N de articulacion: "<<i<<", Valor:"<<JointPose[i]<<", qmin="<<qmin<<", qmax="<<qmax;
          return false;
      }
    }
    return true;
}

bool walkingRobot::moviLeftJoint(std::vector<double> &JointPose)
{
    if(checkLimitsLeft(JointPose)){
        posdLeft->setPositions(JointPose.data());
        return true;
    }
    else{
        yCError(WR) <<"value exceed one or more joint limits";
        return false;
    }
}
bool walkingRobot::moviRightJoint(std::vector<double> &JointPose)
{
    if(checkLimitsRight(JointPose)){
        posdRight->setPositions(JointPose.data());
        return true;
    }
    else{
        yCError(WR) <<"value exceed one or more joint limits";
        return false;
    }
}

bool walkingRobot::inv_Right(std::vector<double> &cartesianPosition, std::vector<double> &JointPostion)
{
    std::vector<double> currentQ(numJoints);
    if (!enc_Right->getEncoders(currentQ.data()))
    {
        yCError(WR) << "getEncoders() failed";
        return false;
    }

    if (!solver_Right->invKin(cartesianPosition, currentQ, JointPostion))
    {
        yCError(WR) << "invKin() failed";
        return false;
    }

    return true;
}

bool walkingRobot::inv_Left(std::vector<double> &cartesianPosition, std::vector<double> &JointPostion)
{
    std::vector<double> currentQ(numJoints);
    if (!enc_Left->getEncoders(currentQ.data()))
    {
        yCError(WR) << "getEncoders() failed";
        return false;
    }

    if (!solver_Left->invKin(cartesianPosition, currentQ, JointPostion))
    {
        yCError(WR) << "invKin() failed";
        return false;
    }
    return true;
}


bool walkingRobot::stat_Right(std::vector<double> &currentCartesianPosition)
{
    std::vector<double> currentQ(numJoints);
    if (!enc_Right->getEncoders(currentQ.data()))
    {
        yCError(WR) << "getEncoders() failed";
        return false;
    }
    if (!solver_Right->fwdKin(currentQ, currentCartesianPosition))
    {
        yCError(WR) << "invKin() failed";
        return false;
    }
    return true;
}



bool walkingRobot::stat_Left(std::vector<double> &currentCartesianPosition)
{
    std::vector<double> currentQ(numJoints);
    if (!enc_Left->getEncoders(currentQ.data()))
    {
        yCError(WR) << "getEncoders() failed";
        return false;
    }
    if (!solver_Left->fwdKin(currentQ, currentCartesianPosition))
    {
        yCError(WR) << "invKin() failed";
        return false;
    }
    return true;
}
