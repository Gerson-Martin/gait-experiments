// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OneFootStand.hpp"

#include <cmath> // std::abs, std::copysign

#include <algorithm> // std::copy, std::max, std::min
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>
#include <unistd.h>
#include <kdl/utilities/utility.h> // KDL::deg2rad
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>

#include <KdlVectorConverter.hpp> // TODO: unused

using namespace roboticslab;
const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/IdSystem.csv";
const char DELIMITADOR =' ';
double first;
std::ofstream data1;

const std::string NOMBRE_ARCHIVO2 = "/home/gerson/MATLAB-Drive/Control.txt";
const char DELIMITADOR2 =' ';


namespace
{
    YARP_LOG_COMPONENT(OFS, "rl.OneFootStand")

    bool parseFrameRotation(const yarp::os::Value & value, const std::string & name, KDL::Rotation & rot)
    {
        if (!value.isNull())
        {
            if (!value.isList() || value.asList()->size() != 3)
            {
                yCError(OFS) << "Parameter" << name << "must be a list of 3 doubles";
                return false;
            }

            yCInfo(OFS) << name << "RPY [deg]:" << value.toString();

            auto roll = value.asList()->get(0).asFloat64() * KDL::deg2rad;
            auto pitch = value.asList()->get(1).asFloat64() * KDL::deg2rad;
            auto yaw = value.asList()->get(2).asFloat64() * KDL::deg2rad;

            // sequence (old axes): 1. R_x(roll), 2. R_y(pitch), 3. R_z(yaw)
            rot = KDL::Rotation::RPY(roll, pitch, yaw);
        }
        else
        {
            yCInfo(OFS) << "Using no" << name;
            rot = KDL::Rotation::Identity();
        }

        return true;
    }
}

constexpr auto DEFAULT_LOCAL_PREFIX = "/oneFootStand";
constexpr auto DEFAULT_IK_STEP = 0.001; // [m]
constexpr auto DEFAULT_MAX_SPEED = 0.01; // [m/s]
constexpr auto DEFAULT_MAX_ACCELERATION = 0.1; // [m/s^2]
constexpr auto REF_FRAME = ICartesianSolver::TCP_FRAME;

bool OneFootStand::configure(yarp::os::ResourceFinder & rf)
{

    yCDebug(OFS) << "Config:" << rf.toString();

    period = rf.check("periodMs", yarp::os::Value(0), "period [ms]").asInt32() * 0.001;

    if (period <= 0)
    {
        yCError(OFS) << "Missing or invalid period parameter:" << static_cast<int>(period * 1000) << "(ms)";
        return false;
    }

    ikStep = rf.check("ikStep", yarp::os::Value(DEFAULT_IK_STEP), "IK step [m]").asFloat64();

    if (ikStep <= 0.0)
    {
        yCError(OFS) << "Invalid IK step parameter:" << ikStep << "(m)";
        return false;
    }

    maxSpeed = rf.check("maxSpeed", yarp::os::Value(DEFAULT_MAX_SPEED), "max speed [m/s]").asFloat64();

    if (maxSpeed <= 0.0)
    {
        yCError(OFS) << "Invalid max speed parameter:" << maxSpeed << "(m/s)";
        return false;
    }

    maxAcceleration = rf.check("maxAcceleration", yarp::os::Value(DEFAULT_MAX_ACCELERATION), "max acceleration [m/s^2]").asFloat64();

    if (maxAcceleration <= 0.0)
    {
        yCError(OFS) << "Invalid max acceleration parameter:" << maxAcceleration << "(m/s^2)";
        return false;
    }

    dryRun = rf.check("dryRun", "process sensor loops, but don't send motion command");

    if (dryRun)
    {
        yCInfo(OFS) << "Dry run mode enabled, robot will perform no motion";
    }

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

    auto localPrefix = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PREFIX), "local port prefix").asString();

    // ----- sensor device -----

    if (!rf.check("sensorName", "remote FT sensor name to connect to via MAS client"))
    {
        yCError(OFS) << "Missing parameter: sensorName";
        return false;
    }

    auto sensorName = rf.find("sensorName").asString();

    if (!rf.check("sensorRemote", "remote FT sensor port to connect to via MAS client"))
    {
        yCError(OFS) << "Missing parameter: sensorRemote";
        return false;
    }

    auto sensorRemote = rf.find("sensorRemote").asString();

    yarp::os::Property sensorOptions {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value(sensorRemote)},
        {"local", yarp::os::Value(localPrefix + sensorRemote)},
        {"timeout", yarp::os::Value(period * 0.5)}
    };

    if (!sensorDevice.open(sensorOptions))
    {
        yCError(OFS) << "Failed to open sensor device";
        return false;
    }

    if (!sensorDevice.view(sensor))
    {
        yCError(OFS) << "Failed to view sensor interface";
        return false;
    }

    sensorIndex = -1;

    for (auto i = 0; i < sensor->getNrOfSixAxisForceTorqueSensors(); i++)
    {
        std::string temp;

        if (sensor->getSixAxisForceTorqueSensorName(i, temp) && temp == sensorName)
        {
            sensorIndex = i;
            break;
        }
    }

    if (sensorIndex == -1)
    {
        yCError(OFS) << "Failed to find sensor with name" << sensorName;
        return false;
    }

    int retry = 0;

    while (sensor->getSixAxisForceTorqueSensorStatus(sensorIndex) != yarp::dev::MAS_OK)
    {
        if (++retry == 10)
        {
            yCError(OFS) << "Failed to get first sensor read";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }

    // ----- robot device -----

    if (!rf.check("robotRemote", "remote robot port to connect to"))
    {
        yCError(OFS) << "Missing parameter: robotRemote";
        return false;
    }

    auto robotRemote = rf.find("robotRemote").asString();

    yarp::os::Property robotOptions {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotRemote)},
        {"local", yarp::os::Value(localPrefix + robotRemote)}
    };

    if (!robotDevice.open(robotOptions))
    {
        yCError(OFS) << "Failed to open robot device";
        return false;
    }

    yarp::dev::IControlLimits * limits;
    yarp::dev::IControlMode * mode;
    yarp::dev::IEncoders * enc;

    if (!robotDevice.view(limits) || !robotDevice.view(mode) || !robotDevice.view(enc) || !robotDevice.view(posd))
    {
        yCError(OFS) << "Failed to view robot control interfaces";
        return false;
    }

    int numJoints;
    enc->getAxes(&numJoints);
    previousJointPose.resize(numJoints);

    retry = 0;

    while (!enc->getEncoders(previousJointPose.data()))
    {
        if (++retry == 10)
        {
            yCError(OFS) << "Failed to get initial joint pose";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }

    yCInfo(OFS) << "Initial joint pose:" << previousJointPose;

    yarp::os::Bottle bMin, bMax;

    for (int joint = 0; joint < numJoints; joint++)
    {
        double qMin, qMax;

        if (!limits->getLimits(joint, &qMin, &qMax))
        {
            yCError(OFS) << "Unable to retrieve position limits for joint" << joint;
            return false;
        }

        bMin.addFloat64(qMin);
        bMax.addFloat64(qMax);
    }

    yCInfo(OFS) << "Joint position limits:" << bMin.toString() << bMax.toString();

    if (!mode->setControlModes(std::vector(numJoints, VOCAB_CM_POSITION_DIRECT).data()))
    {
        yCError(OFS) << "Failed to set posd control mode";
        return false;
    }

    // ----- solver device -----

   if (!rf.check("kinematics", "description file for kinematics"))
   {
       yCError(OFS) << "Missing parameter: kinematics";
       return false;
   }

   yarp::os::Property solverOptions {
       {"device", yarp::os::Value("KdlSolver")},
       {"kinematics", rf.find("kinematics")},
       {"ikPos", yarp::os::Value("st")},
       {"invKinStrategy", yarp::os::Value("humanoidGait")},
       {"quiet", yarp::os::Value::getNullValue()}
   };

   solverOptions.put("mins", yarp::os::Value::makeList(bMin.toString().c_str()));
   solverOptions.put("maxs", yarp::os::Value::makeList(bMax.toString().c_str()));

   if (!solverDevice.open(solverOptions))
   {
       yCError(OFS) << "Failed to open solver device";
       return false;
   }

   if (!solverDevice.view(solver))
   {
       yCError(OFS) << "Failed to view solver interface";
       return false;
   }



    // ----- ZMP publisher port -----

    if (!zmpPort.open(localPrefix + "/zmp:o"))
    {
        yCError(OFS) << "Failed to open ZMP publisher port" << zmpPort.getName();
        return false;
    }

    zmpPort.setWriteOnly();
    data.open(NOMBRE_ARCHIVO, std::fstream::out);
    data1.open(NOMBRE_ARCHIVO2, std::fstream::out);
    data << "pos x" << DELIMITADOR <<"pos y"  << DELIMITADOR << "pos z"  << DELIMITADOR
            << "turn x" << DELIMITADOR <<"turn y"  << DELIMITADOR << "turn z"  << DELIMITADOR
            << "ZMPx"  << DELIMITADOR <<"ZMPy" << DELIMITADOR << "ZMPz" << DELIMITADOR
            << "targetZMPx"  << DELIMITADOR <<"targetZMPy" << DELIMITADOR << "targetZMPz" <<"\n";
    first = yarp::os::Time::now();

    double kp=7.1;//3.5817;//2.887;//0.4753;//0.4625;//1.6984;
    double ki=0.0;//1.3198;//0.0459;//0.0;//0.2149;
    double kd=0.4658;//0.0;//0.0;//0.1195;
    double ts=0.02;
    PIDBlock pid(kp,ki,kd,ts);
    PID=pid;
    return yarp::os::PeriodicThread::setPeriod(period) && yarp::os::PeriodicThread::start();
}

bool OneFootStand::readSensor(KDL::Wrench & wrench_N) const
{
    yarp::sig::Vector outSensor;
    double timestamp;
    if (!sensor->getSixAxisForceTorqueSensorMeasure(sensorIndex, outSensor, timestamp))
    {
        yCWarning(OFS) << "Failed to retrieve current sensor measurements";
        return false;
    }

    KDL::Wrench currentWrench_sensor (
        KDL::Vector(outSensor[0], outSensor[1], outSensor[2]), // force
        KDL::Vector(outSensor[3], outSensor[4], outSensor[5]) // torque
    );
    //yCWarning(OFS) <<"SENSOR: " <<outSensor[0]<<"," <<outSensor[1]<<","<< outSensor[2];
    wrench_N = R_N_sensor * currentWrench_sensor;
    return true;
}


void OneFootStand::publishProjection(const KDL::Vector & p_N_zmp)
{
    KDL::Vector p_sole_zmp = R_N_sole.Inverse() * p_N_zmp;

    // R_N_sole should make Z axis orthogonal to the sole plane, thus we only need X and Y
    zmpPort.prepare() = {yarp::os::Value(p_sole_zmp.x()), yarp::os::Value(p_sole_zmp.y())};

    zmpPort.write();
}


bool a=true;
KDL::Vector error;
KDL::Vector target_ZMP;



void OneFootStand::run()
{
     yCDebug(OFS) << "Aqui0";
    KDL::Wrench wrench_N; // expressed in TCP frame

    if (!readSensor(wrench_N))
    {
        yarp::os::PeriodicThread::askToStop();
        return;
    }

    auto forceNorm = wrench_N.force.Norm();

    if (KDL::Equal(forceNorm, 0.0))
    {
        yCWarning(OFS) << "Zero force detected, skipping this iteration";
        return;
    }
    double H=2.5*1/100;
    double Px_zmp=0;
    double Pz_zmp= (wrench_N.torque.data[1]+wrench_N.force.data[2]*H)/wrench_N.force.data[0];
    double Py_zmp= -(wrench_N.torque.data[2]-wrench_N.force.data[1]*H)/wrench_N.force.data[0];
    KDL::Vector Pzmp(Px_zmp,Py_zmp,Pz_zmp);
    EMA_Exponential_Filter(Pzmp,0.7);
 yCDebug(OFS) << "Aqui0";
    if (zmpPort.getOutputCount() > 0)
    {   publishProjection(Pzmp);

    }

    std::vector<double> leftInitial,rightInitial;
    //iCartesianControl->stat(leftInitial);

    solver->fwdKin(previousJointPose,leftInitial);
//    KDL::Vector Pos(0,0,0);
//    KDL::Frame COM(Pos);
//    KDL::Frame H_leftInitial = KdlVectorConverter::vectorToFrame(leftInitial);

















    if(a){
       target_ZMP=Pzmp/Pzmp.Norm()*0.03;
       target_ZMP = Pzmp - target_ZMP;
       a=false;
    }

    double ep=0.002;
    double now = yarp::os::Time::now()-first;
    KDL::Vector  controlSignal;
    error=(target_ZMP-Pzmp);
    std::vector<double> nextPosition;
   if( now<10.0 /*&& sqrt(error[2]*error[2])>=ep*/){
    double ts=0.02;
    double velMax=6/2;//[º/s]
    double velMax2=0.1/1;//[m/s]
    double grados_en_Ts=velMax*ts;//[º]
    double metros_en_Ts=velMax2*ts;
    double conDegToRad=3.14/180;
    double limitDegreeMaxTs=grados_en_Ts*conDegToRad;//0.0174/50.0; //0.9*ts*velMax*conDegToRad;
    double limitMetrosMaxTs=metros_en_Ts;
    double convDegreeToZMP=8.0*conDegToRad/5;//[rad de giro del robot/m de ZMP]
    //x_pie=z_base
    //y_pie=-y_base
    //z_pie=x_base;
    //en el pie el zmp esta en el plano z-y
    //si el angulo es positivo en pierna izquierda debe ser opuesto para pierna izquierda
    controlSignal[1]=-error[1]*convDegreeToZMP*copysign(1.0,leftInitial[2]);//
    controlSignal[0]=-error[2]/100;

        if (controlSignal[1]>=limitDegreeMaxTs){
                yCDebug(OFS) << "Error Saturación";
            controlSignal[1]=limitDegreeMaxTs;
        }
        else if (controlSignal[1]<=-limitDegreeMaxTs){
             yCDebug(OFS) << "Error Saturación 2";
            controlSignal[1]= -limitDegreeMaxTs;
        }
        if (controlSignal[0]>=limitMetrosMaxTs){
            yCDebug(OFS) << "Error Saturación 3";
            controlSignal[0]=limitMetrosMaxTs;
        }
        else if (controlSignal[0]<=-limitMetrosMaxTs){
             yCDebug(OFS) << "Error Saturación 4";
            controlSignal[0]= -limitMetrosMaxTs;
        }




    leftInitial[0]+=controlSignal[0];
    solver->invKin(leftInitial, previousJointPose, previousJointPose);
    previousJointPose[5]=+controlSignal[1];
//    auto nextPoint=/*H_leftInitial;//*/getTrayectoryWithCOMangle(H_leftInitial,0.0,COM,controlSignal[1]);




//    //PID.AntiWindup(-limit,limit);
//    controlSignal[0]=PID.UpdateControl(error[0]); //error*1/25;//(kp*error+ki*sumaError+kd*(error-errorAnterior));
//    controlSignal[1]=PID.UpdateControl(error[1]);
//    controlSignal[2]=PID.UpdateControl(error[2]);

//    if (controlSignal[1]>=limit){
//            yCDebug(OFS) << "Error Saturación";
//        controlSignal[1]=limit;
//    }
//    else if (controlSignal[1]<=-limit){
//         yCDebug(OFS) << "Error Saturación 2";
//        controlSignal[1]= -limit;
//    }
//    if (controlSignal[2]>=limit){
//        yCDebug(OFS) << "Error Saturación 3";
//        controlSignal[2]=limit;
//    }
//    else if (controlSignal[2]<=-limit){
//         yCDebug(OFS) << "Error Saturación 4";
//        controlSignal[2]= -limit;
//        }

//    /*vd[0]=0.0;//arriba +
//    vd[1]=-controlSignal[1];// izquierda +
//    vd[2]=-controlSignal[2];//adelante +
//    */
//     //std::vector<double> q(6,0.0),x(6,0.0);
//     //iCartesianControl->stat(x);
//      //yCDebug(OFS) << "Posicion Cartesiana:"<<x[0]<<" ; "<<x[1]<<" ; "<<x[2]<<" ; "<<x[3]<<" ; "<<x[4]<<" ; "<<x[5];
//    //iCartesianControl->inv(x,q);
//    vd[5]= -controlSignal[1];
//    vd[4]= controlSignal[2];
//    vd[3]=0;
//   // yCDebug(OFS) << "Posicion Articular :"<<q[0]<<" ; "<<q[1]<<" ; "<<q[2]<<" ; "<<q[3]<<" ; "<<q[4]<<" ; "<<q[5];
    std::vector<double> q;
//   if(!iCartesianControl->inv(nextPosition,q)){
//       yWarning() << "IK failing at left leg:" << nextPosition[0] << nextPosition[1] << nextPosition[2] << nextPosition[3] << nextPosition[4] << nextPosition[5];
//       yDebug() << "-------left leg joins:" << q[0]<<"," << q[1]<<"," << q[2]<<"," << q[3]<<"," << q[4]<<"," <<q[5];
//   }
//   else iCartesianControl->movi(nextPosition);
         // assume this is reachable
     posd->setPositions(previousJointPose.data());
   }else
   {yCDebug(OFS) << "Fin del programa";close();}////
   /// \brief printAndSave

    printAndSave(previousJointPose,Pzmp,target_ZMP,controlSignal,leftInitial);
    yCDebug(OFS) << "Aqui";
    printAndSave(controlSignal[1],controlSignal[0],Pzmp,target_ZMP);









}



























//    error=(Pzmp-target_ZMP);
//    vd[1]=controlSignal[1];// izquierda +
//    vd[2]=controlSignal[2];//adelante +
//    vd[5]= controlSignal[1];
//    vd[4]= -controlSignal[2];
void OneFootStand::printAndSave(std::vector<double> vd, KDL::Vector Pzmp, KDL::Vector target_ZMP,KDL::Vector controlSignal,std::vector<double> vd2){
    yCDebug(OFS) << "ZMP : "<<Pzmp[0]<<" ; "<<Pzmp[1]<<" ; "<<Pzmp[2];
    yCDebug(OFS) << "Error : "<<error[0]<<" ; "<<error[1]<<" ; "<<error[2];
    yCDebug(OFS) << "CONTROL : "<<controlSignal[0]<<" ; "<<controlSignal[1]<<" ; "<<controlSignal[2];
    yCDebug(OFS) << "Posicion Siguiente : "<<vd[0]<<" ; " <<vd[1]<<" ; "<<vd[2]<<vd[3]<<" ; "<<vd[4]<<" ; "<<vd[5];
    yCDebug(OFS) << "POsicion Actual : " <<vd2[0]<<" ; " <<vd2[1]<<" ; "<<vd2[2]<<vd2[3]<<" ; "<<vd2[4]<<" ; "<<vd2[5];
    System dato;
    dato.input.x=vd[0];
    dato.input.y=vd[1];
    dato.input.z=vd[2];
    dato.input.gx=vd[3];
    dato.input.gy=vd[4];
    dato.input.gz=vd[5];
    dato.output.x=Pzmp[0];
    dato.output.y=Pzmp[1];
    dato.output.z=Pzmp[2];
    dato.target.x=target_ZMP[0];
    dato.target.y=target_ZMP[1];
    dato.target.z=target_ZMP[2];
    saveData(dato);
}
void OneFootStand::printAndSave(double angle,double x, KDL::Vector Pzmp, KDL::Vector target_ZMP){
    System dato;
    dato.input.x=angle;
    dato.input.y=angle;
    dato.input.z=angle;
    dato.input.gx=x;
    dato.input.gy=x;
    dato.input.gz=x;
    dato.output.x=Pzmp[0];
    dato.output.y=Pzmp[1];
    dato.output.z=Pzmp[2];
    dato.target.x=target_ZMP[0];
    dato.target.y=target_ZMP[1];
    dato.target.z=target_ZMP[2];
    data1 <<angle<< DELIMITADOR2<<x<< DELIMITADOR2<<Pzmp[0]<<DELIMITADOR2<<Pzmp[1]<<DELIMITADOR2<<Pzmp[2]<<DELIMITADOR2
         <<target_ZMP[0]<<DELIMITADOR2<<target_ZMP[1]<<DELIMITADOR2<<target_ZMP[2]<<"\n";
}

bool OneFootStand::updateModule()
{
    return yarp::os::PeriodicThread::isRunning();
}

bool OneFootStand::interruptModule()
{
    yarp::os::PeriodicThread::stop();
    return 0 ;//|| iCartesianControl->stopControl();
}

double OneFootStand::getPeriod()
{
    return 1.0; // [s]
}
void OneFootStand::EMA_Exponential_Filter(KDL::Vector &data,double alpha){
    data=alpha*data+(1-alpha)*datoAnterior;
}
bool OneFootStand::close()
{
    data.close();
    zmpPort.interrupt();
    zmpPort.close();
    sensorDevice.close();
    solverDevice.close();
    robotDevice.close();
    return true;
}

KDL::Frame OneFootStand::getTrayectoryWithCOMangle(KDL::Frame &point,double diff_x, KDL::Frame &Base, double diff_angle)
{

    KDL::Frame start(point);
    KDL::Vector diff=(start.p-Base.p);
    double L=(start.p-Base.p).Norm();
    double initial_angle=atan2(diff.z(),diff.y());

    if(point.p.y()<=0){
        diff_angle=diff_angle;
    }else
    diff_angle=-diff_angle;
    double z=0,y=0,x=0,x0=start.p.x();
    KDL::Rotation rot2,rot=start.M;
    KDL::Frame  a;
    z=L*sin(initial_angle+diff_angle)+Base.p.z();
    y=L*cos(initial_angle+diff_angle)+Base.p.y();
    x=x0+diff_x;
    rot.DoRotX(-2*diff_angle);
//    data1 <<x<< DELIMITADOR2<<y<< DELIMITADOR2<<z<<DELIMITADOR2<<rot2.GetRot().x()<<"\n";
    KDL::Frame current_point(rot,KDL::Vector(x,y,z));
    return current_point;

}

void OneFootStand::saveData(System system)
{
        data << system.input.x << DELIMITADOR <<system.input.y << DELIMITADOR << system.input.z << DELIMITADOR
             << system.input.gx << DELIMITADOR <<system.input.gy << DELIMITADOR << system.input.gz << DELIMITADOR
             << system.output.x << DELIMITADOR <<system.output.y << DELIMITADOR << system.output.z << DELIMITADOR
             << system.target.x << DELIMITADOR <<system.target.y << DELIMITADOR << system.target.z << "\n";


}




















//KDL::Frame OneFootStand::getTrayectoryWithCOMangle(KDL::Frame &point,double distance, KDL::Frame &Base, double angle)
//{

//    KDL::Frame start(point);
//    KDL::Vector diff=(start.p-Base.p);
//    double L=(start.p-Base.p).Norm();
//    double initial_angle=atan2(diff.z(),diff.y());
//    if(point.p.y()<=0){
//        angle=angle;
//    }else
//    angle=-angle;
//    double z=0,y=0,x=0,x0=start.p.x(),n=100.0,diff_angle=(angle/n),diff_x=distance/n;
//     std::cout<<"ANGULO:  "<<angle<<std::endl;
//     std::cout<<"diff_angle:  "<<diff_angle<<std::endl;
//     std::cout<<"diff_x:  "<<diff_x<<std::endl;
//     std::cout<<"initial_angle:  "<<initial_angle<<std::endl;
//    KDL::Rotation rot2,rot=start.M;
//    KDL::Frame  a;
//    for(double i=0;i<=n;i+=1){
//        z=L*sin(initial_angle+diff_angle*i)+Base.p.z();
//        y=L*cos(initial_angle+diff_angle*i)+Base.p.y();
//        x=x0+/*sin(i/n)*distance;//*/diff_x*i;
//        rot2=rot.RotX(point.M.GetRot().x()-diff_angle*i*2);
//        data1 <<x<< DELIMITADOR2<<y<< DELIMITADOR2<<z<<DELIMITADOR2<<rot2.GetRot().x()<<"\n";
//        KDL::Frame current_point(rot2,KDL::Vector(x,y,z));
//        a=current_point;
//    }

//    return a;

//}
