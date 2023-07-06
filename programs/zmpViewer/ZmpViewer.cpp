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
int PHASE_WALKING=0;//Double support 0 right single support  -1 left single support  1, 2 mean error
namespace
{
    YARP_LOG_COMPONENT(ZMPV, "rl.ZmpViewer")

}

bool ZmpViewer::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(ZMPV) << "Config:" << rf.toString();
    system("yarpview --name /sole:i   & ");
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
    if (!rf.check("sensorRemote", "remote FT sensor port to connect to via MAS client"))
    {
        yCError(ZMPV) << "Missing parameter: sensorRemote";
        return false;
    }

    auto sensorRemote = rf.find("sensorRemote").asString();


    legs.initWalkingRobot(robotRemote,local);
    legs.initSensor(sensorRemote,period);



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

    system("sleep 0.25  > /dev/null");
    system("yarp connect /zmpViewer/sole:o /sole:i  > /dev/null");
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
    int sensorIndexLeft=legs.sensorIndex+1;
    if (!legs.sensor->getSixAxisForceTorqueSensorMeasure(sensorIndexLeft, LeftSensor_ft, timestamp) && !legs.sensor->getSixAxisForceTorqueSensorMeasure(legs.sensorIndex, RightSensor_ft, timestamp))
    {
        yCWarning(ZMPV) << "Failed to retrieve current sensor measurements";
        return false;
    }
    if (!legs.sensor->getSixAxisForceTorqueSensorMeasure(legs.sensorIndex, RightSensor_ft, timestamp))
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
    wrench_N_Right = legs.R_N_sensor * currentWrench_RightLegSensor;
    wrench_N_Left = legs.R_N_sensor * currentWrench_LeftLegSensor;
    return true;
}



KDL::Vector ZmpViewer::getZMP(KDL::Wrench &wrench_N_Right,KDL::Wrench &wrench_N_Left){
    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;
    KDL::Frame ZMP_Left_TCP,ZMP_Right_TCP;
    double H=2.5*1/100;
    yCDebug(ZMPV)<<"fuerza izquierda"<<wrench_N_Left.force.data[0]<<"Derecha"<<wrench_N_Right.force.data[0];
    //ZMP IN left leg
    if(wrench_N_Left.force.data[0] < 1.00)
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
    if(wrench_N_Right.force.data[0] < 1.00)
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

    if (zmpRightDefined==1.0 && zmpLeftDefined==1.0){PHASE_WALKING=0;}
    else if(zmpRightDefined==0.0 && zmpLeftDefined==1.0){PHASE_WALKING=1;}
    else if(zmpRightDefined==1.0 && zmpLeftDefined==0.0){PHASE_WALKING=-1;}
    else {PHASE_WALKING=2;}
    if(!legs.enc_Right->getEncoders(legs.previousJointPose_Right.data()))
    {
         yCError(ZMPV) << "Failed to get initial joint pose";

    }
    legs.solver_Right->fwdKin(legs.previousJointPose_Right,rightInitial);
    auto right_leg=roboticslab::KdlVectorConverter::vectorToFrame(rightInitial);
    if(!legs.enc_Left->getEncoders(legs.previousJointPose_Left.data()))
    {
         yCError(ZMPV) << "Failed to get initial joint pose";

    }
    legs.solver_Left->fwdKin(legs.previousJointPose_Left,leftInitial);
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
    yarp::sig::PixelRgb LeftActive,RightActive;
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

    }

    int BaseFrameReference_x=blankImage.width()/2;
    int BaseFrameReference_y=blankImage.height()/2-70;


    if(PHASE_WALKING==1){
        isActive=true;
        LeftActive=BLUE;
        RightActive=RED;
    }else if(PHASE_WALKING==-1){
        LeftActive=RED;
        RightActive=BLUE;
        isActive=true;
    }else if(PHASE_WALKING==0){
        RightActive=BLUE;
        LeftActive=BLUE;
        isActive=true;
    }else{
        isActive=false;
    }
    yInfo()<<"---------derecha"<<rightInitial[1]<<" Izquierda"<<leftInitial[1];
    auto & image = imagePort.prepare();
    image = blankImage;
    int position_right_TCP_x=rightInitial[1]*1000+BaseFrameReference_x;
    int position_right_TCP_y=rightInitial[0]*1000+BaseFrameReference_y;
    int position_left_TCP_x= leftInitial[1]*1000+BaseFrameReference_x;
    int position_left_TCP_y=leftInitial[0]*1000+BaseFrameReference_y;
    yarp::sig::draw::addRectangleOutline(image, RightActive,position_right_TCP_x ,position_right_TCP_y+70, width * 0.5, height * 0.5);
    yarp::sig::draw::addRectangleOutline(image, LeftActive,position_left_TCP_x,position_left_TCP_y+70, width * 0.5, height * 0.5);

    // TCP
    yarp::sig::draw::addCircle(image, BLACK, position_right_TCP_x, position_right_TCP_y, 8);
    yarp::sig::draw::addCircle(image, BLACK, position_left_TCP_x, position_left_TCP_y, 8);
    yarp::sig::draw::addCircle(image, BLACK, BaseFrameReference_x, BaseFrameReference_y, 8);

    // ZMP
    yarp::sig::draw::addCircle(image, isActive ? RED : GREY, BaseFrameReference_x + zmpX,  BaseFrameReference_y + zmpY, 5);

    imagePort.write();
}
