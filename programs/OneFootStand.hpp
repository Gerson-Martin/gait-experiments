// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ONE_FOOT_STAND_HPP__
#define __ONE_FOOT_STAND_HPP__

#include <vector>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>

#include <fcontrol.h>
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

        turnAnkleTCP input;
        ZMP output;
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
    {}

    ~OneFootStand() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    double getPeriod() override;
    bool close() override;
    void EMA_Exponential_Filter(KDL::Vector &data,double alpha);
    PIDBlock PID(kp,ki,kd,ts);
protected:
    void run() override;

private:
    void saveData(System datos);
    bool readSensor(KDL::Wrench & wrench_N) const;
    bool selectZmp(const KDL::Vector & axis, KDL::Vector & zmp) const;
    void publishProjection(const KDL::Vector & p_N_zmp);
    std::vector<double> computeStep(const KDL::Vector & p);
    std::vector<System> datos;
    yarp::dev::PolyDriver cartesianDevice;
    roboticslab::ICartesianControl * iCartesianControl;

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
    double kp=1.6984;
    double ki=0.2149;
    double kd=0;
    double ts=0.02;

    yarp::os::BufferedPort<yarp::os::Bottle> zmpPort;
};

} // namespace roboticslab

#endif // __ONE_FOOT_STAND_HPP__
