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

constexpr auto DEFAULT_TRAJ_VEL = 0.175;     // [m/s]
constexpr auto DEFAULT_TRAJ_ACC = 5.0;       // [m/s^2]
constexpr auto DEFAULT_CMD_PERIOD = 0.02;    // [s]
constexpr auto DEFAULT_FOOT_LENGTH = 0.245;  // [m]
constexpr auto DEFAULT_FOOT_WIDTH = 0.12;    // [m]
constexpr auto DEFAULT_FOOT_LIFT = 0.012;    // [m]
constexpr auto DEFAULT_GAIT_SEP = 0.1285;      // [m]
constexpr auto DEFAULT_GAIT_SQUAT= 0.82;     // [m]
constexpr auto DEFAULT_GAIT_STEP=0.05;      // [m]

const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/TrajectoriaGenerada.txt";
const std::string NOMBRE_ARCHIVO4 = "/home/gerson/MATLAB-Drive/TrajectoriaReal.txt";
const std::string NOMBRE_ARCHIVO2 = "/home/gerson/MATLAB-Drive/TrajectoriaGeneradaGlobal.txt";
const std::string NOMBRE_ARCHIVO3 = "/home/gerson/MATLAB-Drive/TrayectoriaArticulares.txt";
const char DELIMITADOR =' ';
std::ofstream data1,data2,dataJoints,data3;
void example(double zModel,double g);
double norm1(MatrixXd W);
MatrixXd discretizar(MatrixXd M,double Ts);
std::vector<double> findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts);
namespace rl = roboticslab;

int main(int argc, char * argv[])
{
    // Find YARP network.
//    system("squatAndBalance --z 0.0001");
//    system("sleep 0.1");
//    system("exit");
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
    double footLift = rf.check("lift", yarp::os::Value(DEFAULT_FOOT_LIFT), "lift [m]").asFloat64();
    double gaitSep = rf.check("sep", yarp::os::Value(DEFAULT_GAIT_SEP), "foot separation [m]").asFloat64();
    double gaitStep =rf.check("step", yarp::os::Value(DEFAULT_GAIT_STEP), "lenght Step [m]").asFloat64();
    double gaitSquat = rf.check("sep", yarp::os::Value(DEFAULT_GAIT_SQUAT), "Squat hight of Base robot[m]").asFloat64();

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


        if (gaitStep <= 0.0)
        {
            yError() << "Illegal argument: '--step' must be greater than '0', was:" << gaitStep;
            return 1;
        }
        if (gaitSquat <= 0.0)
        {
            yError() << "Illegal argument: '--squat' must be greater than '0', was:" << gaitSquat;
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

    std::vector<double> leftInitial,rightInitial;

    if (!iCartesianControlLeftLeg->stat(leftInitial))
    {
        yError() << "Cannot stat left leg";
        return 1;
    }

    if (!iCartesianControlRightLeg->stat(rightInitial))
    {
        yError() << "Cannot stat left leg";
        return 1;
    }

    data1.open(NOMBRE_ARCHIVO, std::fstream::out);
    data2.open(NOMBRE_ARCHIVO2, std::fstream::out);
    data3.open(NOMBRE_ARCHIVO4, std::fstream::out);
    dataJoints.open(NOMBRE_ARCHIVO3, std::fstream::out);
    data1 << "Time" << DELIMITADOR
         << "Left_Leg_x" << DELIMITADOR << "Left_Leg_y"<< DELIMITADOR <<"Left_Leg_z" << DELIMITADOR
         << "Left_Leg_rotx" << DELIMITADOR << "Left_Leg_roty"<< DELIMITADOR <<"Left_Leg_rotz" << DELIMITADOR
         << "Right_Leg_x" << DELIMITADOR<< "Right_Leg_y" << DELIMITADOR << "Right_Leg_z"<< DELIMITADOR
         << "Right_Leg_rotx" << DELIMITADOR<< "Right_Leg_roty" << DELIMITADOR << "Right_Leg_rotz"<< DELIMITADOR
         << "COM_x"  << DELIMITADOR<< "COM_y"  << DELIMITADOR<< "COM_z"<<DELIMITADOR
         << "COM_rotx"  << DELIMITADOR<< "COM_roty"  << DELIMITADOR<< "COM_rotz" <<"\n";

    data2 << "Time" << DELIMITADOR
         << "Left_Leg_x" << DELIMITADOR << "Left_Leg_y"<< DELIMITADOR <<"Left_Leg_z" << DELIMITADOR
         << "Right_Leg_x" << DELIMITADOR<< "Right_Leg_y" << DELIMITADOR << "Right_Leg_z"<< DELIMITADOR
         << "COM_x"  << DELIMITADOR<< "COM_y"  << DELIMITADOR<< "COM_z" <<"\n";



    leftInitial[2] += KDL::epsilon; // initial pose is hard to attain
    rightInitial[2] += KDL::epsilon; // initial pose is hard to attain
    KDL::Frame H_leftInitial = rl::KdlVectorConverter::vectorToFrame(leftInitial);
    KDL::Frame H_rightInitial = rl::KdlVectorConverter::vectorToFrame(rightInitial);

    FootSpec footSpec;
    footSpec.length = footLength;
    footSpec.width = footWidth;
    footSpec.lift = footLift;

    GaitSpec gaitSpec;
    gaitSpec.sep = gaitSep;
    gaitSpec.step = gaitStep;
    gaitSpec.squat = gaitSquat;

    StepGenerator stepGenerator(footSpec, H_rightInitial, H_leftInitial);
    TrajectoryGenerator trajectoryGenerator(footSpec, distance, trajVel, trajAcc, period);
    TargetBuilder targetBuilder(iCartesianControlLeftLeg, iCartesianControlRightLeg);

    TargetBuilder::Targets pointsLeft, pointsRight, COM;


    bool hasSolution = false;
    double maxDuration;
    do
    {

        // Generate steps.

        stepGenerator.configure(gaitSpec);

        std::vector<KDL::Frame> steps, com;
        std::vector<std::tuple<KDL::Frame,double>> _COM;
        stepGenerator.generate(distance, steps, _COM);


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
//        {
//            auto && log = yInfo();
//            log << _COM.size() << "COM ([x, y]):";

//            for (int i = 0; i < _COM.size(); i++)
//            {
//                const KDL::Vector & p = std::get<0>(_COM[i]).p;
//                std::ostringstream oss;
//                oss << "[" << p.x() << " " << p.y() << "]";
//                log << oss.str();
//            }
//        }

//        {
//            auto && log = yInfo();
//            log << com.size() << "COM ([x, y]):";

//            for (int i = 0; i < com.size(); i++)
//            {
//                const KDL::Vector & p = com[i].p;
//                std::ostringstream oss;
//                oss << "[" << p.x() << " " << p.y() << "]";
//                log << oss.str();
//            }
//        }
        // Generate trajectories.

        trajectoryGenerator.configure(steps, _COM);

        KDL::Trajectory_Composite comTraj, leftTraj, rightTraj;

        try
        {
            trajectoryGenerator.generate2(comTraj, leftTraj, rightTraj);

        }
        catch (const KDL::Error_MotionPlanning & e)
        {
            yWarning() << "Error:" << e.Description()<<"____Tipo: "<<e.GetType();
            continue;
        }

        yWarning() << "Hola______________________";


        double duration=leftTraj.Duration();
                double time=0.0;
                while(time<duration){
                    std::vector<double> t=rl::KdlVectorConverter::frameToVector(leftTraj.Pos(time));
                    std::vector<double> t2=rl::KdlVectorConverter::frameToVector(rightTraj.Pos(time));
                    std::vector<double> t3=rl::KdlVectorConverter::frameToVector(comTraj.Pos(time));
                    data2 <<time<<DELIMITADOR<<t[0]<<DELIMITADOR<<t[1]<< DELIMITADOR<<t[2]<<DELIMITADOR<<t[3]<<DELIMITADOR<<t[4]<< DELIMITADOR<<t[5]<<DELIMITADOR
                          <<t2[0]<<DELIMITADOR<<t2[1]<< DELIMITADOR<<t2[2]<<DELIMITADOR<<t2[3]<<DELIMITADOR<<t2[4]<< DELIMITADOR<<t2[5]<<DELIMITADOR
                          <<t3[0]<<DELIMITADOR<<t3[1]<< DELIMITADOR<<t3[2]<<DELIMITADOR<<t3[3]<<DELIMITADOR<<t3[4]<< DELIMITADOR<<t3[5]<<"\n";
                    time+=0.02;
                }



        yInfo() << "CoM:" << comTraj.Duration() << "[s], left:" << leftTraj.Duration() << "[s], right:" << rightTraj.Duration() << "[s]";

        double minDuration = std::min(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());
        maxDuration = std::max(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());
        yWarning() << "Hola______________________";

        if (maxDuration - minDuration > 1.0)
        {
            yWarning() << "Duration difference exceeds 1.0 seconds:" << maxDuration - minDuration;
           continue;
        }

        // Build target points.
        yWarning() << "Hola______________________";

        targetBuilder.configure(&comTraj, &leftTraj, &rightTraj);
        COM=targetBuilder.build(period, pointsLeft, pointsRight);
        yWarning() << "Hola______________________4";

        for (int i = 0; i < pointsLeft.size(); i++)
        {
             data1<<period*i<< DELIMITADOR << pointsLeft[i][0]<< DELIMITADOR << pointsLeft[i][1]<<DELIMITADOR<< pointsLeft[i][2]<< DELIMITADOR
                  << pointsLeft[i][3]<< DELIMITADOR << pointsLeft[i][4]<<DELIMITADOR<< pointsLeft[i][5]<< DELIMITADOR
                  << pointsRight[i][0]<<DELIMITADOR << pointsRight[i][1]<<DELIMITADOR << pointsRight[i][2]<<DELIMITADOR
                  << pointsRight[i][3]<< DELIMITADOR << pointsRight[i][4]<<DELIMITADOR<< pointsRight[i][5]<< DELIMITADOR
                  << COM[i][0]<<DELIMITADOR << COM[i][1]<<DELIMITADOR << COM[i][2]<<DELIMITADOR
                  << COM[i][3]<<DELIMITADOR << COM[i][4]<<DELIMITADOR << COM[i][5]
                  << "\n";
        }
        yWarning() << "Hola______________________5";

        std::vector<double> q,q2;

        for (int i = 0; i < pointsLeft.size(); i++)
        {
            iCartesianControlLeftLeg->inv(pointsLeft[i], q);
            iCartesianControlRightLeg->inv(pointsRight[i], q2);
            dataJoints<<i*period*4<<" "<<q[0]<<" " << q[1]<<" " << q[2]<<" " << q[3]<<" "<< q[4]<<" "<<q[5]<<" "
                <<q2[0]<<" "<< q2[1]<<" "<< q2[2]<<" "<< q2[3]<<" "<< q2[4]<<" "<<q2[5]<<"\n";
        }




        if (!targetBuilder.validate(pointsLeft, pointsRight))
        {
            yWarning() << "IK failed";
            break;//continue;
        }
        else
        {
            hasSolution = true;
            break;
        }
    }
    while (!once);
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
        maxDuration*=1*2;
        period=0.02*2;
        yarp::os::TimerSettings timerSettings(period, maxDuration / period, maxDuration);
        std::vector<double> x;int i=0;
        yarp::os::Timer::TimerCallback callback = [&](const yarp::os::YarpTimerEvent & event)
        {
            iCartesianControlLeftLeg->movi(pointsLeft[event.runCount]);
            iCartesianControlLeftLeg->stat(x);
            data3<<period*i<<DELIMITADOR<<x[0]<<DELIMITADOR<<x[1]<<DELIMITADOR<<x[2]
                   <<DELIMITADOR<<x[3]<<DELIMITADOR<<x[4]<<DELIMITADOR<<x[5]<<DELIMITADOR;

            iCartesianControlRightLeg->movi(pointsRight[event.runCount]);
            iCartesianControlRightLeg->stat(x);
            data3<<x[0]<<DELIMITADOR<<x[1]<<DELIMITADOR<<x[2]
                               <<DELIMITADOR<<x[3]<<DELIMITADOR<<x[4]<<DELIMITADOR<<x[5]<<"\n";

//            std::cout<<period*i<<std::endl;
             i++;
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
