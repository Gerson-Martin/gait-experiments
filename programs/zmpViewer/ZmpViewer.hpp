// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ZMP_VIEWER_HPP__
#define __ZMP_VIEWER_HPP__

#include <mutex>

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/TypedReaderCallback.h>
#include <kdl/utilities/utility.h> // KDL::deg2rad
#include <KdlVectorConverter.hpp> // TODO: unused
#include <yarp/sig/Image.h>
#include <yarp/os/PeriodicThread.h>
#include "walkingRobot.hpp"
namespace roboticslab
{

/**
 * @ingroup zmpViewer
 * @brief Connects to a remote ZMP publisher port and generates an image.
 */
class ZmpViewer : public yarp::os::RFModule,
                  public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    ~ZmpViewer() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;

    bool updateModule() override;

    double getPeriod() override;

    bool close() override;

    bool readSensor(KDL::Wrench &wrench_N_Right, KDL::Wrench &wrench_N_Left);
    KDL::Vector getZMP(KDL::Wrench &wrench_N_Right, KDL::Wrench &wrench_N_Left);
private:
    void drawAndPublishImage(bool isActive, int zmpX, int zmpY);

    yarp::os::Port zmpPort;
    yarp::os::PortReaderBuffer<yarp::os::Bottle> zmpPortReader;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> imagePort;
    walkingRobot legs;

    int width;
    int height;
    int cx;
    int cy;
    double period;
    std::vector<double> leftInitial,rightInitial;

    double lastStamp {0.0};
    int lastZmpX {0};
    int lastZmpY {0};
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __ZMP_VIEWER_HPP__
