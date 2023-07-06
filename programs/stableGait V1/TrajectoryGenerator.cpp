// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-




#include "TrajectoryGenerator.hpp"
#include <yarp/os/LogStream.h>
#include <kdl/path_circle.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include "VelocityProfile_Custom.h"
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include "LIMP.hpp"
#include <iomanip>



const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/TrajectoriaGeneradanew.txt";
const char DELIMITADOR =' ';


constexpr auto DEFAULT_SQUAT_DURATION = 5.0; // [s]
constexpr auto DEFAULT_RADIUS = 0.005; // [m]
constexpr auto DEFAULT_EQ_RADIUS = 1.0; // [m]
constexpr auto DURATION_ONE_STEP = 5.0; // [s]
constexpr auto DURATION_COM_INITIAL_POSITION = 1.0; // [s]

TrajectoryGenerator::TrajectoryGenerator(FootSpec _footSpec, double _distance, double _vel, double _acc)
    : footSpec(_footSpec),
      distance(_distance),
      vel(_vel),
      acc(_acc),
      radius(DEFAULT_RADIUS),
      eqradius(DEFAULT_EQ_RADIUS)
{}

void TrajectoryGenerator::configure(const std::vector<KDL::Frame> & _steps, const std::vector<KDL::Frame> & _com)
{
    steps = _steps;
    com = _com;
}

void TrajectoryGenerator::generate(KDL::Trajectory_Composite & comTraj, KDL::Trajectory_Composite & leftTraj, KDL::Trajectory_Composite & rightTraj)
{

    data1.open(NOMBRE_ARCHIVO, std::fstream::out);
    /**Empezamos flexionando la piernas del robot y llevarlo a la posicion inicial ***/
    double nStep=0;
    KDL::Frame rightLeg=steps[nStep];
    KDL::Frame leftLeg=steps[nStep+1];

    KDL::Path_Composite * pathSquatDown = new KDL::Path_Composite();
    KDL::Path_Composite * pathSquatUp = new KDL::Path_Composite();

    pathSquatDown->Add(new KDL::Path_Line(com[0], com[1], orient.Clone(), eqradius));
    pathSquatDown->Add(new KDL::Path_Line(com[1], com[2], orient.Clone(), eqradius));
//    std::cout<<com[1]<<com[2];
    pathSquatUp->Add(new KDL::Path_Line(com[com.size() - 3], com[com.size() - 2], orient.Clone(), eqradius));
    pathSquatUp->Add(new KDL::Path_Line(com[com.size() - 2], com[com.size() - 1], orient.Clone(), eqradius));

    // FIXME: hardcoded, check behavior on simulator
    KDL::VelocityProfile * profSquatDown = new KDL::VelocityProfile_TrapHalf(0.1, 0.1, true);
    KDL::VelocityProfile * profSquatUp = new KDL::VelocityProfile_TrapHalf(0.1, 0.1, false);

    comTraj.Add(new KDL::Trajectory_Segment(pathSquatDown, profSquatDown, DEFAULT_SQUAT_DURATION));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));


    KDL::Path_Composite* pathOneStep=getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+2],200);
    KDL::VelocityProfile_Rectangular profVelOneStep(pathOneStep->PathLength()/DURATION_ONE_STEP);

    rightTraj.Add(new KDL::Trajectory_Segment(pathOneStep, profVelOneStep.Clone(), DURATION_ONE_STEP));
    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
    comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[2]));
    nStep++;
    rightLeg=steps[nStep+1];

    /***Asumimos pasos constantes y rectos modificaremso esto en un futuro**/
    double stepsSep=abs(steps[3].p.y()-steps[2].p.y())/2.0;
    double lenghtStep=abs(steps[2].p.x()-steps[1].p.x());
    double dy=0.01;
    double zModel=com[2].p.z();
    double g=9.807;
    double Ts=0.02;
    std::cout<<"sep:"<<stepsSep<<" longitud: "<<lenghtStep<<" liftFoot: "<<footSpec.lift<<std::endl;
    LIMP limp(Ts,zModel,g);
    std::vector<double> x0=findInitialConditions(lenghtStep,dy,stepsSep,zModel,g,Ts);
    double tSingleSupport=x0[x0.size()-1];
    x0.pop_back();
    limp.setInitialCondition(x0,tSingleSupport);
    std::cout<<"sep:"<<x0[0]<<" longitud: "<<x0[2]<<std::endl;

    /**Una vez tenemos los pasos procederemos a calcular las trayectorias de las piernas y del centro de masas con el LIMP ***/
    //los dos primeros steps son las posiciones iniciales de los pies y los dos ultimos son las posiciones finales por tanto nos iteresan los
    //steps intermedios.
//    KDL::VelocityProfile_Rectangular profRect(vel);

    std::vector<KDL::Frame> stepsLIMP{steps.begin()+2,steps.end()-2};


        std::vector<std::vector<double>>x;
        limp.getTrayectory(stepsLIMP,x,footSpec.length,footSpec.width);
    //pathCom->Add(new KDL::Path_Line(trajCOM[trajCOM.size()-2], trajCOM[trajCOM.size()-1], orient.Clone(), eqradius));
    //std::cout<<"longitud: "<<footSpec.length<<"Ancho: "<<footSpec.width<<std::endl;


    KDL::Vector LIMP_initialPosition(x[0][0],x[0][2],x[0][4]);
    KDL::Frame LIMP_COM_initialPosition(LIMP_initialPosition);
    KDL::Path_Line* COM_initialPosition =new KDL::Path_Line(com[2], LIMP_COM_initialPosition, orient.Clone(), eqradius);
    KDL::VelocityProfile_Rectangular profVel_COM_initialPosition(COM_initialPosition->PathLength()/DURATION_COM_INITIAL_POSITION);
    comTraj.Add(new KDL::Trajectory_Segment(COM_initialPosition,profVel_COM_initialPosition.Clone(),DURATION_COM_INITIAL_POSITION));
    rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, leftLeg));
    double before_time=0.0;
    double phaseTime=0.0;

    KDL::Path_Composite * pathCom1 = new KDL::Path_Composite();
    std::vector<double> velLIMP;

//   //0 double support
//   //-1 single support right
//   //1 single support left
   for (int i = 0; i<(x.size()-1); i+=1)
   {
       //yDebug()<<i<<" ; phaseTime--------------------------------: "<<x.size();

//       std::cout<<x[i][x[0].size()-1]<<" "<<x[i+1][x[0].size()-1]<<std::endl;
       if(x[i][x[0].size()-1]!=x[i+1][x[0].size()-1]){
           phaseTime=i*Ts-before_time;
           before_time=i*Ts;
           if(x[i][x[0].size()-1]==-1){
               nStep++;
               //std::cout<<"Pierna Dercha"<<" paso 1:"<< steps[nStep].p<<" paso 2:"<<steps[nStep+2].p<<std::endl;
               //rightTraj.Add(new KDL::Trajectory_Segment(new KDL::Path_Line(rightLeg,rightLeg,orient.Clone(),eqradius), profRect.Clone(), phaseTime));
               rightTraj.Add(new KDL::Trajectory_Stationary(phaseTime, rightLeg));
               KDL::VelocityProfile_Rectangular profRect((leftLeg.p-steps[nStep+1].p).Norm()/phaseTime);
               leftTraj.Add(new KDL::Trajectory_Segment(getPathLegSwing(footSpec.lift,leftLeg,steps[nStep+1],200), profRect.Clone(), phaseTime));

               leftLeg=steps[nStep+1];
           }else if (x[i][x[0].size()-1]==0) {
               //std::cout<<"doble soporte"<<std::endl;
               rightTraj.Add(new KDL::Trajectory_Stationary(phaseTime, rightLeg));
               leftTraj.Add(new KDL::Trajectory_Stationary(phaseTime, leftLeg));
               //rightTraj.Add(new KDL::Trajectory_Segment(new KDL::Path_Line(rightLeg,rightLeg,orient.Clone(),eqradius), profRect.Clone(), phaseTime));
               //leftTraj.Add(new KDL::Trajectory_Segment(new KDL::Path_Line(leftLeg,leftLeg,orient.Clone(),eqradius), profRect.Clone(), phaseTime));
           }else{
               //std::cout<<"Pierna Izquierda"<<std::endl;
               nStep++;
               leftTraj.Add(new KDL::Trajectory_Stationary(phaseTime, leftLeg));
               KDL::VelocityProfile_Rectangular profRect((rightLeg.p-steps[nStep+1].p).Norm()/phaseTime);
               //leftTraj.Add(new KDL::Trajectory_Segment(new KDL::Path_Line(leftLeg,leftLeg,orient.Clone(),eqradius), profRect.Clone(), phaseTime));
               rightTraj.Add(new KDL::Trajectory_Segment(getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+1],200), profRect.Clone(), phaseTime));

               rightLeg=steps[nStep+1];

           }

       }

        //data1 <<x[i][0]<<DELIMITADOR<<x[i][1]<< DELIMITADOR<<x[i][2]<<DELIMITADOR<<x[i][3]<<"\n";
       //data1 <<x[i][1]<< DELIMITADOR<<x[i][3]<< DELIMITADOR<<x[i][4/*x[0].size()-1*/]<<"\n";
       KDL::Vector COM_nextPosition(x[i+1][0],x[i+1][2],x[i+1][4]),COM_currentPosition(x[i][0],x[i][2],x[i][4]);
       //velLIMP.push_back(sqrt(x[i][1]*x[i][1]+x[i][3]*x[i][3]+x[i][5]*x[i][5]));
       //velLIMP.push_back(sqrt(x[i][0]*x[i][0]+x[i][2]*x[i][2]+x[i][4]*x[i][4]));
       KDL::Frame COM_P1(COM_currentPosition),COM_P2(COM_nextPosition);
//       KDL::Path_Line * pathCom1 =new KDL::Path_Line(COM_P1,COM_P2, orient.Clone(), eqradius);
       pathCom1->Add(new KDL::Path_Line(COM_P1,COM_P2, orient.Clone(), eqradius));
//       KDL::VelocityProfile_Rectangular profRect2(pathCom1->PathLength()/Ts);
////       profRect2.SetMax(pathCom1->PathLength()/Ts);
//       comTraj.Add(new KDL::Trajectory_Segment(pathCom1, profRect2.Clone(), profRect2.Duration()));
   }
   KDL::VelocityProfile_Rectangular profRect2(pathCom1->PathLength()/before_time);
   KDL::VelocityProfile_Custom profLIMP(velLIMP,Ts);
//   profLIMP.Write(data1);
  comTraj.Add(new KDL::Trajectory_Segment(pathCom1, profRect2.Clone(), before_time));
//   comTraj.Add(new KDL::Trajectory_Segment(pathCom1, profLIMP.Clone(), profLIMP.Duration()));


   KDL::Vector LIMP_endPosition(x[x.size()-1][0],x[x.size()-1][2],x[x.size()-1][4]);
   KDL::Frame LIMP_COM_endPosition(LIMP_endPosition);
   KDL::Path_Line* COM_endPosition =new KDL::Path_Line(LIMP_COM_endPosition,com[com.size() - 3], orient.Clone(), eqradius);

   KDL::VelocityProfile_Rectangular profVel_COM_endPosition(COM_endPosition->PathLength()/DURATION_COM_INITIAL_POSITION);
   comTraj.Add(new KDL::Trajectory_Segment(COM_endPosition,profVel_COM_endPosition.Clone(),DURATION_COM_INITIAL_POSITION));
   rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, rightLeg));
   leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, leftLeg));




   KDL::Path_Composite* pathEndStep=getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+2],200);
   KDL::VelocityProfile_Rectangular profVelEndStep(pathEndStep->PathLength()/DURATION_ONE_STEP);

   rightTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profVelEndStep.Clone(), DURATION_ONE_STEP));
   leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
   comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[com.size() - 3]));
   nStep++;
   rightLeg=steps[nStep+1];

   comTraj.Add(new KDL::Trajectory_Segment(pathSquatUp, profSquatUp, DEFAULT_SQUAT_DURATION));
   rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
   leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));


//   rightTraj.Add(new KDL::Trajectory_Segment(getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+2],200), profRect0.Clone(), comTraj.Duration()));
//   leftTraj.Add(new KDL::Trajectory_Stationary(comTraj.Duration(), leftLeg));
//   comTraj.Add(new KDL::Trajectory_Stationary(comTraj.Duration(), com[com.size() - 3]));
//   nStep++;
//   rightLeg=steps[nStep+1];



}





double TrajectoryGenerator::getDuration(const KDL::Vector & v)
{
    return v.x() / vel;
}

KDL::Frame TrajectoryGenerator::makeHop(const KDL::Frame & H1, const KDL::Frame & H2)
{
     //yDebug() <<0.5 * (H2.p.x() - H1.p.x());
    return KDL::Frame(H1.M, KDL::Vector(0.5 * (H2.p.x() - H1.p.x())+H1.p.x(), H1.p.y(), footSpec.lift));
}

std::vector<double> TrajectoryGenerator::findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts){
    double Tc = sqrt(zModel/g);
    double y_mid = 0.0;
    double E = -g/(2*zModel)*y_mid*y_mid + 0.5*dy_mid*dy_mid;
    double y0 = -stepLength/2;
    double dy0 = sqrt(2*(E+g/(2*zModel)*y0*y0));
    double tsinglesupport = 2*asinh(stepLength/2/(Tc*dy_mid))*Tc;
    tsinglesupport = floor(tsinglesupport/Ts)*Ts;
    double tf = tsinglesupport/2;
    double dx0 = -x0/Tc * sinh(tf/Tc) / cosh(tf/Tc);
    std::vector<double> InitialCondition{x0,dx0,y0,dy0,tsinglesupport};
    std::cout<<std::setprecision(10)<<" x0:"<<x0<<" y0:"<<y0<<" dx0:"<<dx0<<" dy0:"<<dy0<<std::endl;
    return InitialCondition;
}

KDL::Path_Composite* TrajectoryGenerator::getPathLegSwing(double h,KDL::Frame &step_start,KDL::Frame &step_end,double nPoints=200){
    auto v= step_end.p-step_start.p;
    double L=v.Norm();
    double foothold_x=step_start.p.x();
    double foothold_y=step_start.p.y();
    KDL::Path_Composite * LegSwing = new KDL::Path_Composite();
    KDL::Frame before_point;
    double angle=atan2(v.x(),v.y());
    double z=0;
    double dx=L/nPoints;
    double y=0;
    double x=0;
    before_point=step_start;
    for(double i=0;i<L;i+=dx){
        z=generateCurveLeg(h,L,i);
        x=i*sin(angle)+foothold_x;
        y=i*cos(angle)+foothold_y;
//        data1 <<x<< DELIMITADOR<<y<< DELIMITADOR<<z<<"\n";
        KDL::Frame current_point(step_start.M,KDL::Vector(x,y,z));
        LegSwing->Add(new KDL::Path_Line(before_point, current_point, orient.Clone(), eqradius));
        before_point=current_point;
    }
    return LegSwing;

}

double TrajectoryGenerator::generateCurveLeg(double h,double L,double x)
{
    double a=-4*h/(L*L);
    double b=0,c=h;
    double y=a*(x-L/2)*(x-L/2)+b*(x-L/2)+c;
    return y;
}
