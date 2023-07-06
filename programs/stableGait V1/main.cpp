// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * @defgroup stableGait stableGait
 * @brief An attempt to perform gait on a humanoid robot with simple, non-dynamic stability assumptions.
 *
 * <b>Building</b>
 *
\verbatim
mkdir build; cd build; cmake ..
make -j$(nproc)
\endverbatim
 *
 * <b>Running example with teoSim</b>
 * First we must run a YARP name server if it is not running in our current namespace:
 *
\verbatim
[on terminal 1] yarp server
\endverbatim
 *
 * The following is an example for the simulated robot's legs:
 *
\verbatim
[on terminal 2] teoSim
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/leftLeg/CartesianControl --kinematics teo-leftLeg.ini --local /BasicCartesianControl/teoSim/leftLeg --remote /teoSim/leftLeg --ik st --invKinStrategy humanoidGait
[on terminal 4] yarpdev --device BasicCartesianControl --name /teoSim/rightLeg/CartesianControl --kinematics teo-rightLeg.ini --local /BasicCartesianControl/teoSim/rightLeg --remote /teoSim/rightLeg --ik st --invKinStrategy humanoidGait
[on terminal 5] stableGait --distance 1.0 --dry
\endverbatim
 */

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Timer.h>

#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/utilities/error.h>

#include <ICartesianControl.h>
#include <KdlVectorConverter.hpp>

#include "GaitSpecs.hpp"
#include "LimitChecker.hpp"
#include "StepGenerator.hpp"
#include "TrajectoryGenerator.hpp"
#include "TargetBuilder.hpp"
#include "fcontrol.h"
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>

constexpr auto DEFAULT_TRAJ_VEL = 0.175; // [m/s]
constexpr auto DEFAULT_TRAJ_ACC = 5.0; // [m/s^2]
constexpr auto DEFAULT_CMD_PERIOD = 0.02; // [s]
constexpr auto DEFAULT_FOOT_LENGTH = 0.245; // [m]
constexpr auto DEFAULT_FOOT_WIDTH = 0.12; // [m]
constexpr auto DEFAULT_FOOT_MARGIN = 0.02; // [m]
constexpr auto DEFAULT_FOOT_STABLE = 0.04; // [m]
constexpr auto DEFAULT_FOOT_LIFT = 0.005; // [m]
constexpr auto DEFAULT_GAIT_SEP = 0.12; //0.085; // [m]
constexpr auto DEFAULT_GAIT_HOP = 0.01; // [m]
constexpr auto DEFAULT_TOLERANCE = 0.001; // [m]

//const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/TrajectoriaGenerada.txt";
const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/TrayectoriaArticulares.txt";
const char DELIMITADOR =' ';
std::ofstream data1;
void example(double zModel,double g);
double norm1(MatrixXd W);
MatrixXd discretizar(MatrixXd M,double Ts);
std::vector<double> findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts);
namespace rl = roboticslab;

int main(int argc, char * argv[])
{
    // Find YARP network.

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    // Parse arguments.

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    yDebug() << rf.toString();

    std::string robotPrefix = rf.check("prefix", yarp::os::Value("/teoSim")).asString();

    double distance = rf.check("distance", yarp::os::Value(1.0), "distance to travel [m]").asFloat64();
    double trajVel = rf.check("vel", yarp::os::Value(DEFAULT_TRAJ_VEL), "velocity [m/s]").asFloat64();
    double trajAcc = rf.check("acc", yarp::os::Value(DEFAULT_TRAJ_ACC), "acceleration [m/s^2]").asFloat64();
    double period = rf.check("period", yarp::os::Value(DEFAULT_CMD_PERIOD), "command period [s]").asFloat64();
    double footLength = rf.check("length", yarp::os::Value(DEFAULT_FOOT_LENGTH), "foot length [m]").asFloat64();
    double footWidth = rf.check("width", yarp::os::Value(DEFAULT_FOOT_WIDTH), "foot width [m]").asFloat64();
    double footMargin = rf.check("margin", yarp::os::Value(DEFAULT_FOOT_MARGIN), "foot stability outer margin [m]").asFloat64();
    double footStable = rf.check("stable", yarp::os::Value(DEFAULT_FOOT_STABLE), "foot stability inner margin [m]").asFloat64();
    double footLift = rf.check("lift", yarp::os::Value(DEFAULT_FOOT_LIFT), "lift [m]").asFloat64();
    double gaitSep = rf.check("sep", yarp::os::Value(DEFAULT_GAIT_SEP), "foot separation [m]").asFloat64();
    double gaitHop = rf.check("hop", yarp::os::Value(DEFAULT_GAIT_HOP), "hop [m]").asFloat64();
    double tolerance = rf.check("tolerance", yarp::os::Value(DEFAULT_TOLERANCE), "tolerance [m]").asFloat64();

    bool dryRun = rf.check("dry", "dry run");
    bool once = rf.check("once", "run once");

    if (distance <= 0.0)
    {
        yError() << "Illegal argument: '--distance' must be greater than '0', was:" << distance;
        return 1;
    }

    if (trajVel <= 0.0)
    {
        yError() << "Illegal argument: '--vel' must be greater than '0', was:" << trajVel;
        return 1;
    }

    if (trajAcc <= 0.0)
    {
        yError() << "Illegal argument: '--acc' must be greater than '0', was:" << trajAcc;
        return 1;
    }

    if (period <= 0.0)
    {
        yError() << "Illegal argument: '--period' must be greater than '0', was:" << period;
        return 1;
    }

    if (footLength <= 0.0)
    {
        yError() << "Illegal argument: '--length' must be greater than '0', was:" << footLength;
        return 1;
    }

    if (footWidth <= 0.0)
    {
        yError() << "Illegal argument: '--width' must be greater than '0', was:" << footWidth;
        return 1;
    }

    if (footMargin <= 0.0)
    {
        yError() << "Illegal argument: '--margin' must be greater than '0', was:" << footMargin;
        return 1;
    }

    if (footStable <= 0.0)
    {
        yError() << "Illegal argument: '--stable' must be greater than '0', was:" << footStable;
        return 1;
    }

    if (footLift < 0.0)
    {
        yError() << "Illegal argument: '--lift' must be greater than or equal to '0', was:" << footLift;
        return 1;
    }

    if (gaitSep <= 0.0)
    {
        yError() << "Illegal argument: '--sep' must be greater than '0', was:" << gaitSep;
        return 1;
    }

    if (gaitHop < 0.0)
    {
        yError() << "Illegal argument: '--hop' must be greater than or equal to '0', was:" << gaitHop;
        return 1;
    }

    if (tolerance < 0.0)
    {
        yError() << "Illegal argument: '--tolerance' must be greater than '0', was:" << tolerance;
        return 1;
    }

    // Create devices (left leg).

    yarp::os::Property leftLegDeviceOptions {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"cartesianRemote", yarp::os::Value(robotPrefix + "/leftLeg/CartesianControl")},
        {"cartesianLocal", yarp::os::Value("/stableGait/leftLeg")}
    };

    yarp::dev::PolyDriver leftLegDevice(leftLegDeviceOptions);

    if (!leftLegDevice.isValid())
    {
        yError() << "Cartesian device (left leg) not available";
        return 1;
    }

    rl::ICartesianControl * iCartesianControlLeftLeg;

    if (!leftLegDevice.view(iCartesianControlLeftLeg))
    {
        yError() << "Cannot view iCartesianControlLeftLeg";
        return 1;
    }

    if (!iCartesianControlLeftLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        yError() << "Cannot preset streaming command (left leg)";
        return 1;
    }

    // Create devices (right leg).

    yarp::os::Property rightLegDeviceOptions {
        {"device", yarp::os::Value("CartesianControlClient")},
        {"cartesianRemote", yarp::os::Value(robotPrefix + "/rightLeg/CartesianControl")},
        {"cartesianLocal", yarp::os::Value("/stableGait/rightLeg")}
    };

    yarp::dev::PolyDriver rightLegDevice(rightLegDeviceOptions);

    if (!rightLegDevice.isValid())
    {
        yError() << "Cartesian device (right leg) not available";
        return 1;
    }

    rl::ICartesianControl * iCartesianControlRightLeg;

    if (!rightLegDevice.view(iCartesianControlRightLeg))
    {
        yError() << "Cannot view iCartesianControlRightLeg";
        return 1;
    }
    if (!iCartesianControlRightLeg->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI))
    {
        yError() << "Cannot preset streaming command (right leg)";
        return 1;
    }

    // Initialize specs and components.

    std::vector<double> x_leftInitial;

    if (!iCartesianControlLeftLeg->stat(x_leftInitial))
    {
        yError() << "Cannot stat left leg";
        return 1;
    }

    data1.open(NOMBRE_ARCHIVO, std::fstream::out);
    data1 << "Time" << DELIMITADOR
         << "Left_Leg_x" << DELIMITADOR << "Left_Leg_y"<< DELIMITADOR <<"Left_Leg_z" << DELIMITADOR
         << "Right_Leg_x" << DELIMITADOR<< "Right_Leg_y" << DELIMITADOR << "Right_Leg_z"<< DELIMITADOR
         << "COM_x"  << DELIMITADOR<< "COM_y"  << DELIMITADOR<< "COM_z" <<"\n";




    x_leftInitial[2] += KDL::epsilon; // initial pose is hard to attain
    KDL::Frame H_leftInitial = rl::KdlVectorConverter::vectorToFrame(x_leftInitial);

    FootSpec footSpec;
    footSpec.length = footLength;
    footSpec.width = footWidth;
    footSpec.margin = footMargin;
    footSpec.stable = footStable;
    footSpec.lift = footLift;

    GaitSpec gaitSpec;
    gaitSpec.sep = gaitSep;
    gaitSpec.hop = gaitHop;

//    LimitChecker limitChecker(footSpec, tolerance, iCartesianControlLeftLeg, iCartesianControlRightLeg);
//    limitChecker.estimateParameters(gaitSpec);
//    limitChecker.setReference(gaitSpec);
    gaitSpec.squat = 0.82;//iterateSquat();
    gaitSpec.step =0.05;//iterateStep(gaitSpec);

    StepGenerator stepGenerator(footSpec, H_leftInitial);
    TrajectoryGenerator trajectoryGenerator(footSpec, distance, trajVel, trajAcc);
    TargetBuilder targetBuilder(iCartesianControlLeftLeg, iCartesianControlRightLeg);

    TargetBuilder::Targets pointsLeft, pointsRight,COM;
     MatrixXd Ad(4,4);
     Ad<<  1.002483812350260   ,  0.020016556008700, 0.0    ,0.0,
          0.248484006047246   , 1.002483812350260  , 0.0    ,0.0,
          0.0      , 0.0    , 1.002483812350260  ,0.020016556008700,
          0.0      ,  0.0   , 0.248484006047246 ,1.002483812350260 ;
     MatrixXd Bd(4,2);
     Bd<<0.0002 ,       0.0,
         0.0200 ,       0.0,
            0.0 ,    0.0002,
            0.0 ,    0.0200;
     MatrixXd Cd(2,4);
     Cd<<1,0,0,0,
         0,0,1,0;
     MatrixXd Dd(2,2);
     Dd<<0.0,0.0,
         0.0,0.0;
     std::vector<double> x0{0.100000000000000,
                            -0.352184197096915,
                            -0.100000000000000,
                             0.352475872232879},u0{0,0};
    StateSpace LIMP(Ad,Bd,Cd,Dd,period);
    //LIMP.printSystem();
    std::vector<double>x1=findInitialConditions(0.3,0.01,0.12,0.79,9.807,0.02);
//    std::cout<<" x0:"<<x1[0]<<" dx0:"<<x1[1]<<" y0:"<<x1[2]<<" dy0:"<<x1[3]<<std::endl;
    LIMP.InitialCondition(x1);
    std::vector<KDL::Frame> trajCOM;

    double tSingleSupport=2.64;

    KDL::RotationalInterpolation_SingleAxis orient;
    double eqradius=1.0;
    KDL::Path_Composite * pathCom = new KDL::Path_Composite();
    KDL::Rotation rot;

    trajCOM.emplace_back(rot,KDL::Vector(x1[2]+0.3,x1[0]-0.12,gaitSpec.squat));

    double vel=0.1;
    KDL::VelocityProfile_Rectangular profRect(vel);


    bool hasSolution = false;
    double maxDuration;
    do
    {

        // Generate steps.

        stepGenerator.configure(gaitSpec);

        std::vector<KDL::Frame> steps, com;
        stepGenerator.generate(distance, steps, com);


        {
            auto && log = yInfo();
            log << steps.size() << "steps ([x, y]):";

            for (int i = 0; i < steps.size(); i++)
            {
                const KDL::Vector & p = steps[i].p;
                std::ostringstream oss;
                oss << "[" << p.x() << " " << p.y() << "]";
                log << oss.str();
            }
        }
        {
            auto && log = yInfo();
            log << com.size() << "COM ([x, y]):";

            for (int i = 0; i < com.size(); i++)
            {
                const KDL::Vector & p = com[i].p;
                std::ostringstream oss;
                oss << "[" << p.x() << " " << p.y() << "]";
                log << oss.str();
            }
        }
        // Generate trajectories.

        trajectoryGenerator.configure(steps, com);

        KDL::Trajectory_Composite comTraj, leftTraj, rightTraj;

        try
        {
            trajectoryGenerator.generate(comTraj, leftTraj, rightTraj);
        }
        catch (const KDL::Error_MotionPlanning & e)
        {
            yWarning() << "Error:" << e.Description()<<"____Tipo: "<<e.GetType();
            continue;
        }

//                double duration=comTraj.Duration();
//                double time=0.0;
//                while(time<duration){
//                    auto t=rl::KdlVectorConverter::frameToVector(comTraj.Pos(time));
//                    data1 <<t[0]<<DELIMITADOR<<t[1]<< DELIMITADOR<<t[2]<<DELIMITADOR<<t[3]<<DELIMITADOR<<t[4]<< DELIMITADOR<<t[5]<<"\n";
//                    time+=0.02;
//                }



        yInfo() << "CoM:" << comTraj.Duration() << "[s], left:" << leftTraj.Duration() << "[s], right:" << rightTraj.Duration() << "[s]";

        double minDuration = std::min(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());
        maxDuration = std::max(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());

        if (maxDuration - minDuration > 1.0)
        {
            yWarning() << "Duration difference exceeds 1.0 seconds:" << maxDuration - minDuration;
           continue;
        }

        // Build target points.

        targetBuilder.configure(&comTraj, &leftTraj, &rightTraj);
        COM=targetBuilder.build(period, pointsLeft, pointsRight);
//        for (int i = 0; i < pointsLeft.size(); i++)
//        {
//             data1<<period*i<< DELIMITADOR << pointsLeft[i][0]<< DELIMITADOR << pointsLeft[i][1]<<DELIMITADOR<< pointsLeft[i][2]<< DELIMITADOR
//                  << pointsLeft[i][3]<< DELIMITADOR << pointsLeft[i][4]<<DELIMITADOR<< pointsLeft[i][5]<< DELIMITADOR
//                  << pointsRight[i][0]<<DELIMITADOR << pointsRight[i][1]<<DELIMITADOR << pointsRight[i][2]<<DELIMITADOR
//                  << pointsRight[i][3]<< DELIMITADOR << pointsRight[i][4]<<DELIMITADOR<< pointsRight[i][5]<< DELIMITADOR
//                  << COM[i][0]<<DELIMITADOR << COM[i][1]<<DELIMITADOR << COM[i][2]<<DELIMITADOR
//                  << COM[i][3]<<DELIMITADOR << COM[i][4]<<DELIMITADOR << COM[i][5]
//                  << "\n";
//        }

        std::vector<double> q,q2;

        for (int i = 0; i < pointsLeft.size(); i++)
        {
            iCartesianControlLeftLeg->inv(pointsLeft[i], q);
            iCartesianControlRightLeg->inv(pointsRight[i], q2);
            data1<<i*period<<" "<<q[0]<<" " << q[1]<<" " << q[2]<<" " << q[3]<<" "<< q[4]<<" "<<q[5]<<" "
                <<q2[0]<<" "<< q2[1]<<" "<< q2[2]<<" "<< q2[3]<<" "<< q2[4]<<" "<<q2[5]<<"\n";
        }






        if (!targetBuilder.validate(pointsLeft, pointsRight))
        {
            yWarning() << "IK failed";
            continue;
        }
        else
        {
            hasSolution = true;
            break;
        }
    }
    while (!once);// && limitChecker.updateSpecs(gaitSpec));
    yDebug() << "----------------------------------";
    //system("xterm -e matlab");
    if (!hasSolution)
    {
        yError() << "No valid solution found";
        return 1;
    }

    // Configure worker.

    if (!dryRun)
    {
        yarp::os::TimerSettings timerSettings(period, maxDuration / period, maxDuration);

        yarp::os::Timer::TimerCallback callback = [&](const yarp::os::YarpTimerEvent & event)
        {
            iCartesianControlLeftLeg->movi(pointsLeft[event.runCount]);
            iCartesianControlRightLeg->movi(pointsRight[event.runCount]);

            return true;
        };

        yarp::os::Timer timer(timerSettings, callback, true);

        // Execute trajectory.

        if (timer.start())
        {
            yarp::os::Time::delay(maxDuration);
            timer.stop();
        }
    }

    return 0;
}
void example(double zModel,double g){
    MatrixXd A(4,4);
    A<< 0  , 1, 0,    0,
        g/zModel ,0, 0,    0,
        0   , 0 ,0   , 1,
        0 ,   0 ,g/zModel ,0;
    MatrixXd B(4,2);
    B<<    0.0 ,  0.0,
           1.0 ,  0.0,
           0.0 ,  0.0,
           0.0 ,  1.0;
    MatrixXd C(2,4);
    C<<1,0,0,0,
        0,0,1,0;
    MatrixXd D(2,2);
    D<<0.0,0.0,
        0.0,0.0;

    MatrixXd Ad=discretizar(A,0.02);
    std::cout<<Ad<<std::endl;
}
MatrixXd discretizar(MatrixXd M,double Ts){
    MatrixXd W=M*Ts;

    // Taylor series for exp(A)
    MatrixXd E(W.cols(),W.cols());E.setZero( W.cols(),W.cols());
    MatrixXd F(W.cols(),W.cols());F.setZero( W.cols(),W.cols());

    for (int i=0;i<W.rows();i++) {
        F(i,i)=1.0;
    }


    double k = 1.0;

    while(norm1(F) > 0.0)
    {
       E = E + F;
       F = W*F/k;
       k = k+1.0;
    }
    return E;
}
double norm1(MatrixXd W){
    double sumCol=0.0;
    double aboutSumCol=0.0;
    for (int i=0;i<W.cols();i++) {
        for (int j=0;j<W.rows();j++) {
            sumCol=W(j,i)+sumCol;
        }
        if(sumCol>aboutSumCol){
            aboutSumCol=sumCol;
        }
    }
    return aboutSumCol;
}
std::vector<double> findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts){
    double Tc = sqrt(zModel/g);
    double y_mid = 0.0;
    double E = -g/(2*zModel)*y_mid*y_mid + 0.5*dy_mid*dy_mid;
    double y0 = -stepLength/2;
    double dy0 = sqrt(2*(E+g/(2*zModel)*y0*y0));
    double tsinglesupport = 2*asinh(stepLength/2/(Tc*dy_mid))*Tc;
    tsinglesupport = floor(tsinglesupport/Ts)*Ts;
    double tf = tsinglesupport/2;
    double dx0 = -x0/Tc * sinh(tf/Tc) / cosh(tf/Tc);
    std::vector<double> InitialCondition{x0,dx0,y0,dy0};
//    std::cout<< std::fixed << std::setprecision(10)<<" x0:"<<x0<<" y0:"<<y0<<" dx0:"<<dx0<<" dy0:"<<dy0<<std::endl;
    return InitialCondition;
}
