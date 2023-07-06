// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ZmpViewer.hpp"


#include <algorithm> // std::min

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Value.h>
#include <yarp/sig/ImageDraw.h>

using namespace roboticslab;

constexpr auto DEFAULT_LOCAL_PORT = "/zmpViewer";

namespace
{
    YARP_LOG_COMPONENT(ZMPV, "rl.ZmpViewer")
    bool parseFrameRotation(const yarp::os::Value & value, const std::string & name, KDL::Rotation & rot)
    {
        if (!value.isNull())
        {
            if (!value.isList() || value.asList()->size() != 3)
            {
                yCError(ZMPV) << "Parameter" << name << "must be a list of 3 doubles";
                return false;
            }

            yCInfo(ZMPV) << name << "RPY [deg]:" << value.toString();

            auto roll = value.asList()->get(0).asFloat64() * KDL::deg2rad;
            auto pitch = value.asList()->get(1).asFloat64() * KDL::deg2rad;
            auto yaw = value.asList()->get(2).asFloat64() * KDL::deg2rad;

            // sequence (old axes): 1. R_x(roll), 2. R_y(pitch), 3. R_z(yaw)
            rot = KDL::Rotation::RPY(roll, pitch, yaw);
        }
        else
        {
            yCInfo(ZMPV) << "Using no" << name;
            rot = KDL::Rotation::Identity();
        }

        return true;
    }
}

bool ZmpViewer::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(ZMPV) << "Config:" << rf.toString();
    system("yarpview --name /sole:i &");
    width = rf.check("width", yarp::os::Value(0.0), "sole width [m]").asFloat64() * 1000;
    height = rf.check("height", yarp::os::Value(0.0), "sole height [m]").asFloat64() * 1000;
    cx = rf.check("cx", yarp::os::Value(width/2), "x coordinate of TCP from bottom left sole corner [m]").asFloat64() * 1000;
    cy = rf.check("cy", yarp::os::Value(height/2), "y coordinate of TCP from bottom left sole corner [m]").asFloat64() * 1000;


    period = rf.check("periodMs", yarp::os::Value(0), "period [ms]").asInt32() * 0.001;

    if (period <= 0)
    {
        yCError(ZMPV) << "Missing or invalid period parameter:" << static_cast<int>(period * 1000) << "(ms)";
        return false;
    }

    if (width <= 0 || height <= 0)
    {
        yCError(ZMPV) << "Invalid sole dimensions:" << width << "x" << height << "[mm]";
        return false;
    }

    if (!rf.check("remote", "remote port prefix"))
    {
        yCError(ZMPV) << "Missing remote port prefix";
        return false;
    }

    auto local = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PORT), "local port prefix").asString();
    auto remote = rf.find("remote").asString();


    // ----- robot device -----

    if (!rf.check("robotRemote", "remote robot port to connect to"))
    {
        yCError(ZMPV) << "Missing parameter: robotRemote";
        return false;
    }

    auto robotRemote = rf.find("robotRemote").asString();

    yarp::os::Property robotOptions {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotRemote+"/rightLeg")},
        {"local", yarp::os::Value(local + robotRemote+"/rightLeg")}
    };

    if (!robotDevice_Right.open(robotOptions))
    {
        yCError(ZMPV) << "Failed to open robot device";
        return false;
    }



    if (!robotDevice_Right.view(enc_Right) || !robotDevice_Right.view(limits_Right))
    {
        yCError(ZMPV) << "Failed to view robot control interfaces";
        return false;
    }

    int numJoints;
    enc_Right->getAxes(&numJoints);
    previousJointPose_Right.resize(numJoints);

    int retry = 0;

    while (!enc_Right->getEncoders(previousJointPose_Right.data()))
    {
        if (++retry == 10)
        {
            yCError(ZMPV) << "Failed to get initial joint pose";
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
            yCError(ZMPV) << "Unable to retrieve position limits for joint" << joint;
            return false;
        }

        bMin_right.addFloat64(qMin);
        bMax_right.addFloat64(qMax);
    }




    yarp::os::Property robotOptions2 {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotRemote+"/leftLeg")},
        {"local", yarp::os::Value(local + robotRemote+"/leftLeg")}
    };

    if (!robotDevice_Left.open(robotOptions2))
    {
        yCError(ZMPV) << "Failed to open robot device";
        return false;
    }



    if (!robotDevice_Left.view(enc_Left) || !robotDevice_Left.view(limits_Left))
    {
        yCError(ZMPV) << "Failed to view robot control interfaces";
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
            yCError(ZMPV) << "Failed to get initial joint pose";
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
            yCError(ZMPV) << "Unable to retrieve position limits for joint" << joint;
            return false;
        }

        bMin_left.addFloat64(qMin);
        bMax_left.addFloat64(qMax);
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
       yCError(ZMPV) << "Failed to open solver device";
       return false;
   }

   if (!solverDevice_Left.view(solver_Left))
   {
       yCError(ZMPV) << "Failed to view solver interface";
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
       yCError(ZMPV) << "Failed to open solver device";
       return false;
   }

   if (!solverDevice_Right.view(solver_Right))
   {
       yCError(ZMPV) << "Failed to view solver interface";
       return false;
   }

   legs.initWalkingRobot(robotRemote,local);



    // ----- sensor device -----

    auto sensorFrameRPY = rf.check("sensorFrameRPY", yarp::os::Value::getNullValue(), "sensor frame RPY rotation regarding TCP frame [deg]");

    if (!parseFrameRotation(sensorFrameRPY, "sensor frame", R_N_sensor))
    {
        return false;
    }

    auto soleFrameRPY = rf.check("soleFrameRPY", yarp::os::Value::getNullValue(), "sole frame RPY rotation regarding TCP frame [deg]");

    if (!parseFrameRotation(soleFrameRPY, "sole frame", R_N_sole))
    {
        return false;
    }

    if (!rf.check("sensorName", "remote FT sensor name to connect to via MAS client"))
    {
        yCError(ZMPV) << "Missing parameter: sensorName";
        return false;
    }

    auto sensorName = rf.find("sensorName").asString();

    if (!rf.check("sensorRemote", "remote FT sensor port to connect to via MAS client"))
    {
        yCError(ZMPV) << "Missing parameter: sensorRemote";
        return false;
    }

    auto sensorRemote = rf.find("sensorRemote").asString();

    yarp::os::Property sensorOptions {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value(sensorRemote)},
        {"local", yarp::os::Value(local + sensorRemote)},
        {"timeout", yarp::os::Value(period * 0.5)}
    };

    if (!sensorDevice.open(sensorOptions))
    {
        yCError(ZMPV) << "Failed to open sensor device";
        return false;
    }

    if (!sensorDevice.view(sensor))
    {
        yCError(ZMPV) << "Failed to view sensor interface";
        return false;
    }

    sensorIndex = -1;

    for (auto i = 0; i < sensor->getNrOfSixAxisForceTorqueSensors(); i++)
    {
        std::string temp;

        if (sensor->getSixAxisForceTorqueSensorName(i, temp) && temp == sensorName)
        {
            sensorIndex = i;
            yInfo() <<"------------------------------------"<<i<< temp;
            break;
        }
    }

    if (sensorIndex == -1)
    {
        yCError(ZMPV) << "Failed to find sensor with name" << sensorName;
        return false;
    }

    retry = 0;

    while (sensor->getSixAxisForceTorqueSensorStatus(sensorIndex) != yarp::dev::MAS_OK)
    {
        if (++retry == 10)
        {
            yCError(ZMPV) << "Failed to get first sensor read";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }


    if (!imagePort.open(local + "/sole:o"))
    {
        yCError(ZMPV) << "Unable not open local image port" << imagePort.getName();
        return false;
    }

    imagePort.setWriteOnly();

    if (!zmpPort.open(local + "/zmp:i"))
    {
        yCError(ZMPV) << "Unable to open local ZMP port" << zmpPort.getName();
        return false;
    }

    zmpPort.setReadOnly();

    yarp::os::ContactStyle style;
    style.carrier = "fast_tcp";
    style.expectReply = false;
    style.persistent = true;
    style.persistenceType = yarp::os::ContactStyle::END_WITH_TO_PORT;

    if (!yarp::os::Network::connect(remote + "/zmp:o", zmpPort.getName(), style))
    {
        yCError(ZMPV) << "Unable to connect to remote port";
        return false;
    }

//    zmpPortReader.attach(zmpPort);
//    zmpPortReader.useCallback(*this);
    system("sleep 0.25");
    system("yarp connect /zmpViewer/sole:o /sole:i");
    return true;
}

bool ZmpViewer::close()
{
    system("pkill yarpview");
    imagePort.interrupt();
    imagePort.close();
    zmpPortReader.interrupt();
    zmpPortReader.disableCallback();
    zmpPort.close();

    return true;
}

bool ZmpViewer::updateModule()
{

    KDL::Wrench wrench_N_Left,wrench_N_Right; // expressed in TCP frame

    if (!readSensor(wrench_N_Right,wrench_N_Left))
    {
        yCWarning(ZMPV) << "it isn't read sensor:";
        return false;
    }
    double H=2.5*1/100;
    double Px_zmp=0;
    double Pz_zmp= (wrench_N_Right.torque.data[1]+wrench_N_Right.force.data[2]*H)/wrench_N_Right.force.data[0];
    double Py_zmp= -(wrench_N_Right.torque.data[2]-wrench_N_Right.force.data[1]*H)/wrench_N_Right.force.data[0];
    KDL::Vector Pzmp(Px_zmp,Py_zmp,Pz_zmp);
    auto ZMP=getZMP(wrench_N_Right,wrench_N_Left);
    int x = ZMP.y()*1000;
    int y = ZMP.x()*1000;
//    static const double throttle = 5.0;
//    const double now = yarp::os::SystemClock::nowSystem();

//    mutex.lock();
//    double stamp = lastStamp;
//    int x = lastZmpX;
//    int y = lastZmpY;
//    mutex.unlock();

//    if (now - stamp > getPeriod())
//    {
//        yCWarningThrottle(ZMPV, throttle) << "No ZMP data received in the last" << getPeriod() << "seconds";
//        drawAndPublishImage(false, x, y);
//    }
    yInfo()<<x<<y;
    drawAndPublishImage(true, x, y);
    return true;
}

double ZmpViewer::getPeriod()
{
    return 0.1; // [s]
}

bool ZmpViewer::readSensor(KDL::Wrench & wrench_N_Right,KDL::Wrench & wrench_N_Left) 
{
    yarp::sig::Vector RightSensor_ft;yarp::sig::Vector LeftSensor_ft;
    double timestamp;
    int sensorIndexLeft=sensorIndex+1;
    if (!sensor->getSixAxisForceTorqueSensorMeasure(sensorIndexLeft, LeftSensor_ft, timestamp) && !sensor->getSixAxisForceTorqueSensorMeasure(sensorIndex, RightSensor_ft, timestamp))
    {
        yCWarning(ZMPV) << "Failed to retrieve current sensor measurements";
        return false;
    }
    if (!sensor->getSixAxisForceTorqueSensorMeasure(sensorIndex, RightSensor_ft, timestamp))
    {
        yCWarning(ZMPV) << "Failed to retrieve current sensor measurements";
        return false;
    }
    KDL::Wrench currentWrench_RightLegSensor (
        KDL::Vector(RightSensor_ft[0], RightSensor_ft[1], RightSensor_ft[2]), // force
        KDL::Vector(RightSensor_ft[3], RightSensor_ft[4], RightSensor_ft[5]) // torque
    );

    KDL::Wrench currentWrench_LeftLegSensor (
        KDL::Vector(LeftSensor_ft[0], LeftSensor_ft[1], LeftSensor_ft[2]), // force
        KDL::Vector(LeftSensor_ft[3], LeftSensor_ft[4], LeftSensor_ft[5]) // torque
    );
    wrench_N_Right = R_N_sensor * currentWrench_RightLegSensor;
    wrench_N_Left = R_N_sensor * currentWrench_LeftLegSensor;
    return true;
}




void ZmpViewer::onRead(yarp::os::Bottle & b)
{

    yCDebug(ZMPV) << b.toString();

    if (b.size() != 2)
    {
        yCWarning(ZMPV) << "Invalid bottle size:" << b.size();
        return;
    }

    int x = b.get(0).asFloat64() * 1000;
    int y = b.get(1).asFloat64() * 1000;

    {
        std::lock_guard lock(mutex);
        lastStamp = yarp::os::SystemClock::nowSystem();
        lastZmpX = x;
        lastZmpY = y;
    }

    drawAndPublishImage(true, x, y);
}
KDL::Vector ZmpViewer::getZMP(KDL::Wrench &wrench_N_Right,KDL::Wrench &wrench_N_Left){
    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;
    KDL::Frame ZMP_Left_TCP,ZMP_Right_TCP;
    double H=2.5*1/100;
    yCDebug(ZMPV)<<"fuerza izquierda"<<wrench_N_Left.force.data[0]<<"Dercha"<<wrench_N_Right.force.data[0];
    //ZMP IN left leg
    if(wrench_N_Left.force.data[0] < 0.001)
        zmpLeftDefined = 0.0;
    else
    {

        double Px_zmp_l=0;
        double Pz_zmp_l= (wrench_N_Left.torque.data[1]+wrench_N_Left.force.data[2]*H)/wrench_N_Left.force.data[0];
        double Py_zmp_l= -(wrench_N_Left.torque.data[2]-wrench_N_Left.force.data[1]*H)/wrench_N_Left.force.data[0];
    //    KDL::Vector Pzmp_Left(Px_zmp_l,Py_zmp_l,Pz_zmp_l);
        ZMP_Left_TCP=KDL::Frame(KDL::Vector(Px_zmp_l,Py_zmp_l,Pz_zmp_l));
        zmpLeftDefined=1.0;
         yCDebug(ZMPV) <<"Pierna izquierda sobre el suelo";
         yCDebug(ZMPV)<<Px_zmp_l<<Py_zmp_l<<Pz_zmp_l;
    }

    //ZMP in right leg
    if(wrench_N_Right.force.data[0] < 0.001)
        zmpRightDefined = 0.0;
    else
    {
        double Px_zmp_r=0;
        double Pz_zmp_r= (wrench_N_Right.torque.data[1]+wrench_N_Right.force.data[2]*H)/wrench_N_Right.force.data[0];
        double Py_zmp_r= -(wrench_N_Right.torque.data[2]-wrench_N_Right.force.data[1]*H)/wrench_N_Right.force.data[0];
        KDL::Vector Pzmp_Right(Px_zmp_r,Py_zmp_r,Pz_zmp_r);
        ZMP_Right_TCP=KDL::Frame(KDL::Vector(Px_zmp_r,Py_zmp_r,Pz_zmp_r));
        zmpRightDefined=1.0;
         yCDebug(ZMPV) <<"Pierna derecha sobre el suelo";
         yCDebug(ZMPV)<<Px_zmp_r<<Py_zmp_r<<Pz_zmp_r;
    }
    double totalZ = wrench_N_Left.force.data[0]*zmpLeftDefined+ wrench_N_Right.force.data[0]*zmpRightDefined;


    if(!enc_Right->getEncoders(previousJointPose_Right.data()))
    {
         yCError(ZMPV) << "Failed to get initial joint pose";

    }
    solver_Right->fwdKin(previousJointPose_Right,rightInitial);
    auto right_leg=roboticslab::KdlVectorConverter::vectorToFrame(rightInitial);
    if(!enc_Left->getEncoders(previousJointPose_Left.data()))
    {
         yCError(ZMPV) << "Failed to get initial joint pose";

    }
    solver_Left->fwdKin(previousJointPose_Left,leftInitial);
    auto left_leg=roboticslab::KdlVectorConverter::vectorToFrame(leftInitial);

    yDebug()<<left_leg.p.data;
    KDL::Frame ZMP_Left_BASE=left_leg*ZMP_Left_TCP;
    KDL::Frame ZMP_Right_BASE=right_leg*ZMP_Right_TCP;


    auto ZMP=(wrench_N_Left.force.data[0]*zmpLeftDefined/totalZ)*ZMP_Left_BASE.p+
            (wrench_N_Right.force.data[0]*zmpRightDefined/totalZ)*ZMP_Right_BASE.p;
    yInfo()<<"ZMP GLOBAL"<<leftInitial[0]<<leftInitial[1]<<leftInitial[2];
    return ZMP;
}
void ZmpViewer::drawAndPublishImage(bool isActive, int zmpX, int zmpY)
{
    static const yarp::sig::PixelRgb BLACK(0, 0, 0);
    static const yarp::sig::PixelRgb WHITE(255, 255, 255);
    static const yarp::sig::PixelRgb GREY(100, 100, 100);
    static const yarp::sig::PixelRgb RED(255, 0, 0);
    static const yarp::sig::PixelRgb BLUE(0, 0, 255);

    static const int margin = std::min(width, height) * 0.5;

    static yarp::sig::ImageOf<yarp::sig::PixelRgb> blankImage;

    if (blankImage.getRowSize() == 0)
    {
        blankImage.resize(width*4 + margin * 2, height + margin * 2);

        for (auto i = 0; i < blankImage.width(); i++)
        {
            for (auto j = 0; j < blankImage.height(); j++)
            {
                blankImage.pixel(i, j) = WHITE;
            }
        }
//        yInfo()<<blankImage.width()<<"x"<<blankImage.height();
        // footprint menos 0.07*1000
//        yarp::sig::draw::addRectangleOutline(blankImage, BLACK, margin + width * 0.5, margin + height * 0.5, width * 0.5, height * 0.5);
//        yarp::sig::draw::addRectangleOutline(blankImage, BLACK, margin + width* (2.5), margin + height * 0.5, width * 0.5, height * 0.5);

//        yarp::sig::draw::addRectangleOutline(blankImage, BLACK, rightInitial[1]*1000+BaseFrameReference_x,rightInitial[0]*1000+BaseFrameReference_y, width * 0.5, height * 0.5);
//        yarp::sig::draw::addRectangleOutline(blankImage, BLACK, leftInitial[1]*1000+BaseFrameReference_x,leftInitial[0]*1000+BaseFrameReference_y, width * 0.5, height * 0.5);

        // TCP
//        yarp::sig::draw::addCircle(blankImage, BLUE, BaseFrameReference_x, BaseFrameReference_y, 8);
    }
    int BaseFrameReference_x=blankImage.width()/2;
    int BaseFrameReference_y=blankImage.height()/2-70;



    yInfo()<<"---------derecha"<<rightInitial[1]<<" Izquierda"<<leftInitial[1];
    auto & image = imagePort.prepare();
    image = blankImage;
    int position_right_TCP_x=rightInitial[1]*1000+BaseFrameReference_x;
    int position_right_TCP_y=rightInitial[0]*1000+BaseFrameReference_y;
    int position_left_TCP_x= leftInitial[1]*1000+BaseFrameReference_x;
    int position_left_TCP_y=leftInitial[0]*1000+BaseFrameReference_y;
    yarp::sig::draw::addRectangleOutline(image, BLACK,position_right_TCP_x ,position_right_TCP_y+70, width * 0.5, height * 0.5);
    yarp::sig::draw::addRectangleOutline(image, BLACK,position_left_TCP_x,position_left_TCP_y+70, width * 0.5, height * 0.5);

    // TCP
    yarp::sig::draw::addCircle(image, BLUE, position_right_TCP_x, position_right_TCP_y, 8);
    yarp::sig::draw::addCircle(image, BLUE, position_left_TCP_x, position_left_TCP_y, 8);
    yarp::sig::draw::addCircle(image, BLACK, BaseFrameReference_x, BaseFrameReference_y, 8);

    // ZMP
    yarp::sig::draw::addCircle(image, isActive ? RED : GREY, BaseFrameReference_x + zmpX,  BaseFrameReference_y + zmpY, 5);

    imagePort.write();
}
