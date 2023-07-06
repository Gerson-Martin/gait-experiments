// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ONE_FOOT_STAND_HPP__
#define __ONE_FOOT_STAND_HPP__

#include <vector>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include <fcontrol.h>
#include <ICartesianSolver.h>
#include <ICartesianControl.h>
#include <fstream>  // Para ofstream
#include <iostream> // Para cout

namespace roboticslab
{
    struct System
    {
        struct turnAnkleTCP
        {
            double x;
            double y;
            double z;
            double gx;
            double gy;
            double gz;
        };
        struct ZMP
        {
            double x;
            double y;
            double z;
        };
        struct Target
        {
            double x;
            double y;
            double z;
        };

        turnAnkleTCP input;
        ZMP output;
        Target target;
    };
/**
 * @ingroup oneFootStand
 *
 * @brief Uses ZMP to maintain stability while standing on one foot.
 */
class OneFootStand : public yarp::os::RFModule,
                     public yarp::os::PeriodicThread
{
public:
    OneFootStand()
        : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes, yarp::os::PeriodicThreadClock::Absolute)
    {PIDBlock a(1.0/25.0,0.0,0.0,0.02);PID=a;}

    ~OneFootStand() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    double getPeriod() override;
    bool close() override;
    void EMA_Exponential_Filter(KDL::Vector &data,double alpha);

    PIDBlock PID;

    KDL::Frame getTrayectoryWithCOMangle(KDL::Frame &point, double diff_x, KDL::Frame &Leg, double diff_angle);
    void printAndSave(double angle, double x, KDL::Vector Pzmp, KDL::Vector target_ZMP);
protected:
    void run() override;

private:
    void printAndSave(std::vector<double> vd, KDL::Vector Pzmp, KDL::Vector target_ZMP, KDL::Vector controlSignal, std::vector<double> vd2);
    void saveData(System datos);
    bool readSensor(KDL::Wrench & wrench_N) const;
    void publishProjection(const KDL::Vector & p_N_zmp);
    std::vector<System> datos;
    yarp::dev::PolyDriver robotDevice;
    yarp::dev::IPositionDirect * posd;

    yarp::dev::PolyDriver solverDevice;
    roboticslab::ICartesianSolver * solver;
//    roboticslab::ICartesianControl * iCartesianControl;

    int sensorIndex;
    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::ISixAxisForceTorqueSensors * sensor;

    KDL::Rotation R_N_sensor;
    KDL::Rotation R_N_sole;

    bool dryRun;
    std::ofstream data;

    double period;
    double ikStep;
    double maxSpeed;
    double maxAcceleration;

    KDL::Vector datoAnterior;

    double previousStep {0.0};
    std::vector<double> previousJointPose;


    yarp::os::BufferedPort<yarp::os::Bottle> zmpPort;
};

} // namespace roboticslab

#endif // __ONE_FOOT_STAND_HPP__
