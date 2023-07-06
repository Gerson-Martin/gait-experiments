// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-




#include "TrajectoryGenerator.hpp"
#include <yarp/os/LogStream.h>
#include <kdl/path_circle.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include "LIMP.hpp"
#include <iomanip>



const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/TrajectoriaGeneradanew.txt";
const char DELIMITADOR =' ';


constexpr auto DEFAULT_SQUAT_DURATION = 1.0; // [s]
constexpr auto DEFAULT_DURATION_DS = 3.0;
constexpr auto DEFAULT_RADIUS = 0.005; // [m]
constexpr auto DEFAULT_EQ_RADIUS = 1.0; // [m]
constexpr auto DURATION_ONE_STEP = 3.0; // [s]
constexpr auto DURATION_COM_INITIAL_POSITION = 1.0; // [s]
constexpr auto DEFAULT_ACELERATION_GRAVITY = 9.807; //[m/s^2]
constexpr auto DEFAULT_INITIAL_VELOCITY_LIMP =0.01;
constexpr auto DEFAULT_TURN_ANGLE_COM = 4.0*3.14159265/180.0; //[rad]
constexpr auto DURATION_TURN_ANGLE_COM = 3.0;

TrajectoryGenerator::TrajectoryGenerator(FootSpec _footSpec, double _distance, double _vel, double _acc, double _Ts)
    : footSpec(_footSpec),
      distance(_distance),
      vel(_vel),
      acc(_acc),
      radius(DEFAULT_RADIUS),
      eqradius(DEFAULT_EQ_RADIUS),
      Ts(_Ts)
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
//    KDL::Path_Composite * pathSquatUp = new KDL::Path_Composite();

    pathSquatDown->Add(new KDL::Path_Line(com[0], com[1], orient.Clone(), eqradius));
    // FIXME: hardcoded, check behavior on simulator
    KDL::VelocityProfile * profSquatDown = new KDL::VelocityProfile_TrapHalf(pathSquatDown->PathLength()/DEFAULT_SQUAT_DURATION*2,0.1,true);
//    KDL::VelocityProfile * profSquatUp = new KDL::VelocityProfile_TrapHalf(0.1, 0.1,false);
    comTraj.Add(new KDL::Trajectory_Segment(pathSquatDown, profSquatDown, DEFAULT_SQUAT_DURATION));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));

    KDL::Frame PointEnterFoot(com[2]);
    PointEnterFoot.p[1]-=(footSpec.width/2);
    KDL::Path * pathEnterFoot = new KDL::Path_Line(com[1], PointEnterFoot, orient.Clone(), eqradius);
    KDL::VelocityProfile_Rectangular* profEnterFoot = new KDL::VelocityProfile_Rectangular(pathEnterFoot->PathLength()/(DEFAULT_DURATION_DS/2));
    comTraj.Add(new KDL::Trajectory_Segment(pathEnterFoot, profEnterFoot, DEFAULT_DURATION_DS/2));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_DURATION_DS/2, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_DURATION_DS/2, leftLeg));



//    KDL::Path_Composite* pathFootSingleCOM=getTrayectoryWithCOMangle(PointEnterFoot,0.0, leftLeg,DEFAULT_TURN_ANGLE_COM);
//    KDL::Frame beforeTurnCOM(comTraj.Pos(comTraj.Duration()));
//    pathFootSingleCOM->Add(getTrayectoryWithCOMangle(beforeTurnCOM,0.0, leftLeg,-DEFAULT_TURN_ANGLE_COM));
//    KDL::VelocityProfile * profFootSingleCOM= new KDL::VelocityProfile_Rectangular((pathFootSingleCOM->PathLength()/DURATION_TURN_ANGLE_COM));
//    KDL::Path * pathOneStep=getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+2],200);
//    KDL::VelocityProfile_Rectangular profVelOneStep(pathOneStep->PathLength()/DURATION_TURN_ANGLE_COM);
//    comTraj.Add(new KDL::Trajectory_Segment(pathFootSingleCOM, profFootSingleCOM, DEFAULT_TURN_ANGLE_COM));
//    rightTraj.Add(new KDL::Trajectory_Segment(pathOneStep, profVelOneStep.Clone(), DURATION_TURN_ANGLE_COM));
//    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_TURN_ANGLE_COM, leftLeg));

//    rightLeg=steps[nStep+1];
//    nStep++;

    ///Levantamos el pie y giramos el torso
    KDL::Path* pathFootSingleCOM=getTrayectoryWithCOMangle(PointEnterFoot,0.0, leftLeg,DEFAULT_TURN_ANGLE_COM);
    KDL::VelocityProfile * profFootSingleCOM= new KDL::VelocityProfile_TrapHalf((pathFootSingleCOM->PathLength()/DURATION_TURN_ANGLE_COM)*2,0.2,false);
    KDL::Frame H=pathFootSingleCOM->Pos(pathFootSingleCOM->PathLength())*PointEnterFoot.Inverse();
    KDL::Frame a=rightLeg;
    std::cout<<a.p[0]<<std::endl;
    a.p[2]+=footSpec.lift;
    KDL::Frame Rightlift=H*a;
    Rightlift.M=rightLeg.M;
    std::cout<<Rightlift<<std::endl<<rightLeg<<std::endl<<steps[nStep+2]<<std::endl;
    KDL::Path* pathFootSingle=new KDL::Path_Line(rightLeg, Rightlift, orient.Clone(), eqradius);
    KDL::VelocityProfile * profFootSingle=new KDL::VelocityProfile_Trap((pathFootSingle->PathLength()/DURATION_TURN_ANGLE_COM)*2,0.2);
    comTraj.Add(new KDL::Trajectory_Segment(pathFootSingleCOM, profFootSingleCOM, DURATION_TURN_ANGLE_COM));
    rightTraj.Add(new KDL::Trajectory_Segment(pathFootSingle, profFootSingle, DURATION_TURN_ANGLE_COM));
    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_TURN_ANGLE_COM, leftLeg));

    ///Llevamos el pie a la posicion final.//////////////////////////////
    Rightlift=rightTraj.Pos(rightTraj.Duration());
    auto v4=H*steps[nStep+2];
    v4.M=steps[nStep+2].M;
    KDL::Path* pathOneStep=/*new KDL::Path_Line(Rightlift, v4, orient.Clone(), eqradius);//*/getPathLegSwing(footSpec.lift,Rightlift,v4,H,200);

    KDL::VelocityProfile_Rectangular profVelOneStep(pathOneStep->PathLength()/DURATION_ONE_STEP);
    KDL::Frame beforeTurnCOM(comTraj.Pos(comTraj.Duration()));
    pathFootSingleCOM=getTrayectoryWithCOMangle(beforeTurnCOM,0.0, leftLeg,-DEFAULT_TURN_ANGLE_COM);
    profFootSingle= new KDL::VelocityProfile_TrapHalf(pathFootSingleCOM->PathLength()/DURATION_ONE_STEP*2,0.1,true);
    comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, beforeTurnCOM));
    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
    rightTraj.Add(new KDL::Trajectory_Segment(pathOneStep, profVelOneStep.Clone(), DURATION_ONE_STEP));
    rightLeg=steps[nStep+1];
    nStep++;

    //desgiramos el torso/////////////////////////////////
    comTraj.Add(new KDL::Trajectory_Segment(pathFootSingleCOM, profFootSingle, DURATION_TURN_ANGLE_COM));
    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_TURN_ANGLE_COM, leftLeg));
    rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_TURN_ANGLE_COM, rightTraj.Pos(rightTraj.Duration())));



    //    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
//    comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[2]));
//
//



return;
//    KDL::Frame Aux(com[2]);
//    Aux.p[1]-=(footSpec.width/2);
//    pathSquatDown->Add(new KDL::Path_Line(com[1], Aux, orient.Clone(), eqradius));

//    pathSquatUp->Add(new KDL::Path_Line(com[com.size() - 3], com[com.size() - 2], orient.Clone(), eqradius));
//    pathSquatUp->Add(new KDL::Path_Line(com[com.size() - 2], com[com.size() - 1], orient.Clone(), eqradius));





//    comTraj.Add(new KDL::Trajectory_Segment(pathSquatDown, profSquatDown, DEFAULT_SQUAT_DURATION));
//    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
//    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));

//    KDL::Path_Composite * pathFirstStepCOM = new KDL::Path_Composite();
//    pathFirstStepCOM->Add(new KDL::Path_Line(aux, com[2], orient.Clone(), eqradius));
//    pathFirstStepCOM->Add(new KDL::Path_Line(com[2], aux, orient.Clone(), eqradius));



//    KDL::Path_Composite* pathOneStep=getPathLegSwing(/*footSpec.lift*/0.01,rightLeg,steps[nStep+2],200);
//    KDL::VelocityProfile_Rectangular profVelOneStep(pathOneStep->PathLength()/DURATION_ONE_STEP);

//    rightTraj.Add(new KDL::Trajectory_Segment(pathOneStep, profVelOneStep.Clone(), DURATION_ONE_STEP));
//    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
//    comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[2]));
//    nStep++;
//    rightLeg=steps[nStep+1];



















//    /***Asumimos pasos constantes y rectos modificaremso esto en un futuro**/
//    double stepsSep=abs(steps[3].p.y()-steps[2].p.y())/2.0;
//    double lenghtStep=abs(steps[3].p.x()-steps[2].p.x());
//    double dy=DEFAULT_INITIAL_VELOCITY_LIMP;
//    double zModel=com[2].p.z();
//    double g=DEFAULT_ACELERATION_GRAVITY;
//    LIMP limp(Ts,zModel,g);
//    std::vector<double> x0=findInitialConditions(lenghtStep,dy,stepsSep,zModel,g,Ts);
//    double tSingleSupport=x0[x0.size()-1];
//    x0.pop_back();
//    limp.setInitialCondition(x0,tSingleSupport);
//    std::cout<<"sep:"<<x0[0]<<" longitud: "<<x0[2]<<std::endl;

//    /**Una vez tenemos los pasos procederemos a calcular las trayectorias de las piernas y del centro de masas con el LIMP ***/
//    //los dos primeros steps son las posiciones iniciales de los pies y los dos ultimos son las posiciones finales por tanto nos iteresan los
//    //steps intermedios.

//    std::vector<KDL::Frame> stepsLIMP{steps.begin()+2,steps.end()-2};


//    std::vector<std::vector<double>>x;
//    limp.getTrayectory(steps,x,footSpec.length,footSpec.width);

//    KDL::Vector LIMP_initialPosition(x[0][0],x[0][2],x[0][4]);
//    KDL::Frame LIMP_COM_initialPosition(LIMP_initialPosition);
//    KDL::Path_Line* COM_initialPosition =new KDL::Path_Line(com[2], LIMP_COM_initialPosition, orient.Clone(), eqradius);
//    KDL::VelocityProfile_Rectangular profVel_COM_initialPosition(COM_initialPosition->PathLength()/DURATION_COM_INITIAL_POSITION);
//    comTraj.Add(new KDL::Trajectory_Segment(COM_initialPosition,profVel_COM_initialPosition.Clone(),DURATION_COM_INITIAL_POSITION));
//    rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, rightLeg));
//    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, leftLeg));
//    double before_time=0.0;
//    double phaseTime=0.0;
//    KDL::Trajectory_Composite * parcialCOMtrayectory=new KDL::Trajectory_Composite();

//   KDL::Path_Composite* PathLegSwing;
//   for (int i = 0; i<(x.size()-1); i+=1)
//   {
//      KDL::Vector COM_nextPosition(x[i+1][0],x[i+1][2],x[i+1][4]),COM_currentPosition(x[i][0],x[i][2],x[i][4]);
//      KDL::Frame COM_P1(COM_currentPosition),COM_P2(COM_nextPosition);
//      KDL::Path_Line * pathCom1 =new KDL::Path_Line(COM_P1,COM_P2, orient.Clone(), eqradius);
//      KDL::VelocityProfile_Rectangular profRect2(pathCom1->PathLength()/Ts);
////       comTraj.Add(new KDL::Trajectory_Segment(pathCom1, profRect2.Clone(), Ts));
//      parcialCOMtrayectory->Add(new KDL::Trajectory_Segment(pathCom1, profRect2.Clone(), Ts));

//         //0 double support
//         //-1 single support right
//         //1 single support left
//       if(x[i][x[0].size()-1]!=x[i+1][x[0].size()-1] || i==(x.size()-2)){
//           if(i==(x.size()-2))i++;
//           phaseTime=i*Ts-before_time;
//           before_time=i*Ts;
//           if(x[i][x[0].size()-1]==-1){
//               nStep++;
//               rightTraj.Add(new KDL::Trajectory_Stationary(phaseTime, rightLeg));
////               leftTraj.Add(new KDL::Trajectory_Stationary(phaseTime/3, leftLeg));
//               PathLegSwing=getPathLegSwing(footSpec.lift,leftLeg,steps[nStep+1],200);
////               KDL::VelocityProfile_Rectangular profRect(PathLegSwing->PathLength()/(phaseTime*1/3));
////               leftTraj.Add(new KDL::Trajectory_Segment(PathLegSwing, profRect.Clone(), phaseTime*1/3));
//               KDL::VelocityProfile_Rectangular profRect(PathLegSwing->PathLength()/(phaseTime));
//               leftTraj.Add(new KDL::Trajectory_Segment(PathLegSwing, profRect.Clone(), phaseTime));
//               leftLeg=steps[nStep+1];
////               leftTraj.Add(new KDL::Trajectory_Stationary(phaseTime/3, leftLeg));
////               comTraj.Add(parcialCOMtrayectory);
//                comTraj.Add( getTrayectoryWithCOMangle(parcialCOMtrayectory,rightLeg, 4.0*3.14159/180.0));

//           }else if (x[i][x[0].size()-1]==0) {
//               rightTraj.Add(new KDL::Trajectory_Stationary(phaseTime, rightLeg));
//               leftTraj.Add(new KDL::Trajectory_Stationary(phaseTime, leftLeg));
//               comTraj.Add(parcialCOMtrayectory);
//           }else{
//               nStep++;
//               leftTraj.Add(new KDL::Trajectory_Stationary(phaseTime, leftLeg));
////               rightTraj.Add(new KDL::Trajectory_Stationary(phaseTime/3, rightLeg));
//               PathLegSwing=getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+1],200);
////               KDL::VelocityProfile_Rectangular profRect(PathLegSwing->PathLength()/(phaseTime*1/3));
////               rightTraj.Add(new KDL::Trajectory_Segment(PathLegSwing, profRect.Clone(), phaseTime*1/3));
//               KDL::VelocityProfile_Rectangular profRect(PathLegSwing->PathLength()/(phaseTime));
//               rightTraj.Add(new KDL::Trajectory_Segment(PathLegSwing, profRect.Clone(), phaseTime));
//               rightLeg=steps[nStep+1];
////               rightTraj.Add(new KDL::Trajectory_Stationary(phaseTime, rightLeg));
//               comTraj.Add(getTrayectoryWithCOMangle(parcialCOMtrayectory,leftLeg, 4.0*3.14159/180.0));
////               getTrayectoryWithCOMangle(parcialCOMtrayectory,leftLeg, 5.0*3.14159/180.0);
////               comTraj.Add(parcialCOMtrayectory);

////               break;

//           }
//        parcialCOMtrayectory=new KDL::Trajectory_Composite();
//       }



//   }

////   return;
//   //Moviment of COM from LIMP end point  to final support foot
//   KDL::Vector LIMP_endPosition(x[x.size()-1][0],x[x.size()-1][2],x[x.size()-1][4]);
//   KDL::Frame LIMP_COM_endPosition(LIMP_endPosition);
//   KDL::Path_Line* COM_endPosition =new KDL::Path_Line(LIMP_COM_endPosition,com[com.size() - 3], orient.Clone(), eqradius);

//   KDL::VelocityProfile_Rectangular profVel_COM_endPosition(COM_endPosition->PathLength()/DURATION_COM_INITIAL_POSITION);
//   comTraj.Add(new KDL::Trajectory_Segment(COM_endPosition,profVel_COM_endPosition.Clone(),DURATION_COM_INITIAL_POSITION));
//   rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, rightLeg));
//   leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, leftLeg));



//    //last step in the trayectory
//   bool movingLegRight=steps[nStep].p.y()<=0.0;
//   if(movingLegRight){
//   KDL::Path_Composite* pathEndStep=getPathLegSwing(/*footSpec.lift*/0.01,rightLeg,steps[nStep+2],200);
//   KDL::VelocityProfile_Rectangular profVelEndStep(pathEndStep->PathLength()/DURATION_ONE_STEP);

//   rightTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profVelEndStep.Clone(), DURATION_ONE_STEP));
//   leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
//   comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[com.size() - 3]));
//   nStep++;
//   rightLeg=steps[nStep+1];
//    }else{
//       KDL::Path_Composite* pathEndStep=getPathLegSwing(footSpec.lift,leftLeg,steps[nStep+2],200);
//       KDL::VelocityProfile_Rectangular profVelEndStep(pathEndStep->PathLength()/DURATION_ONE_STEP);

//       leftTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profVelEndStep.Clone(), DURATION_ONE_STEP));
//       rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, rightLeg));
//       comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[com.size() - 3]));
//       nStep++;
//       leftLeg=steps[nStep+1];
//   }
//    //Go back to final position
//   comTraj.Add(new KDL::Trajectory_Segment(pathSquatUp, profSquatUp, DEFAULT_SQUAT_DURATION));
//   rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
//   leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));
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

KDL::Path_Composite* TrajectoryGenerator::getPathLegSwing(double h,KDL::Frame &step_start,KDL::Frame &step_end,KDL::Frame H,double nPoints=200){
    auto v= step_end.p-step_start.p;
    double L=v.Norm();
    double foothold_x=step_start.p.x();
    double foothold_y=step_start.p.y();
    double foothold_z=step_start.p.z();
    KDL::Path_Composite * LegSwing = new KDL::Path_Composite();
    KDL::Frame before_point;
    double angle=atan2(v.x(),v.y());
    double z=0;
    double dx=L/nPoints;
    double y=0;
    double x=0;
    before_point=step_start;
    std::cout<<step_end.M.GetRot().y()<<std::endl;
    std::cout<<step_start.M.GetRot().y()<<std::endl;
    KDL::Rotation rot;
    auto diff_angle=(step_end.M.GetRot()-step_start.M.GetRot())/nPoints;
    if(h<foothold_z)
        h=foothold_z;
    h-=step_end.p.z();
  std::cout<<foothold_z<<" "<<h<< " "<<-v.z()<<std::endl;
//    KDL::Rotation rot=step_start.M.GetRot(step_end.M);
    if(foothold_z>0){
        for(double i=0;i<L;i+=dx){
            z=generateCurveLeg(h,L,-v.z(),i);
            x=i*sin(angle)+foothold_x;
            y=i*cos(angle)+foothold_y;

            auto v2=step_start.M.GetRot()+diff_angle*i/dx;
//            rot=step_start.M.RotX(step_start.M.GetRot().x()+diff_anglex*i/dx);
            rot=step_start.M.Rot2(v2/v2.Norm(),v2.Norm());
//            std::cout<<rot.GetRot().y()<<std::endl;
            KDL::Frame current_point(rot,KDL::Vector(x,y,z));current_point=H*current_point;
            data1 <<current_point.p.x()<< DELIMITADOR<<current_point.p.y()<< DELIMITADOR<<current_point.p.z()<<"\n";
            LegSwing->Add(new KDL::Path_Line(before_point, current_point, orient.Clone(), eqradius));
            before_point=current_point;
        }
    }else
    {
        for(double i=0;i<L;i+=dx){
            z=generateCurveLeg(h,L,i)+foothold_z;
            x=i*sin(angle)+foothold_x;
            y=i*cos(angle)+foothold_y;
//            data1 <<x<< DELIMITADOR<<y<< DELIMITADOR<<z<<"\n";
            auto v2=step_start.M.GetRot()+diff_angle*i/dx;
//            rot=step_start.M.RotX(step_start.M.GetRot().x()+diff_anglex*i/dx);
            rot=step_start.M.Rot2(v2/v2.Norm(),v2.Norm());
            KDL::Frame current_point(rot,KDL::Vector(x,y,z));
            LegSwing->Add(new KDL::Path_Line(before_point, current_point, orient.Clone(), eqradius));
            before_point=current_point;
        }
    }
    return LegSwing;

}

KDL::Trajectory_Composite *TrajectoryGenerator::getTrayectoryWithCOMangle(KDL::Trajectory_Composite* trajCOM, KDL::Frame Leg, double angle)
{

    KDL::Frame start(trajCOM->Pos(0));
    KDL::Vector diff=(start.p-Leg.p);
    double L=(start.p-Leg.p).Norm();
    double initial_angle=atan2(diff.z(),diff.y());
    if(Leg.p.y()<=0){
        angle=abs(angle);
    }else
    angle=-abs(angle);
    double z=0,y=0,x=0,n=trajCOM->Duration()/Ts,diff_angle=(angle/n)*2;
    KDL::Rotation rot2,rot=start.M;
    KDL::Path_Line* COMpath;
    KDL::Trajectory_Composite * parcialCOMtrayectory=new KDL::Trajectory_Composite();

    for(double i=0;i<(n-1);i+=1){
        if(i<=n/2){
        z=L*sin(initial_angle+diff_angle*i)+Leg.p.z();
        y=L*cos(initial_angle+diff_angle*i)+Leg.p.y();
        x=trajCOM->Pos(i*Ts).p.x();
        rot2=rot.RotX(diff_angle*i);
        }else{
            z=L*sin(initial_angle+angle-diff_angle*(i-n/2))+Leg.p.z();
            y=L*cos(initial_angle+angle-diff_angle*(i-n/2))+Leg.p.y();
            x=trajCOM->Pos(i*Ts).p.x();
//            rot.DoRotX(-diff_angle);
            rot2=rot.RotX((angle-diff_angle*(i-n/2)));

        }
//        data1 <<x<< DELIMITADOR<<y<< DELIMITADOR<<z<<DELIMITADOR<<rot2.GetRot().x()<<"\n";
        KDL::Frame current_point(rot2,KDL::Vector(x,y,z));
        COMpath =new KDL::Path_Line(start,current_point,orient.Clone(),eqradius);
        KDL::VelocityProfile_Rectangular profRect2(COMpath->PathLength()/Ts);
        parcialCOMtrayectory->Add(new KDL::Trajectory_Segment(COMpath, profRect2.Clone(), Ts));
        start=current_point;
    }
//std::cout<<n<<" ,"<<trajCOM->Duration()<<" ,"<<parcialCOMtrayectory->Duration()<<std::endl;

    return parcialCOMtrayectory;

}

KDL::Path_Composite* TrajectoryGenerator::getTrayectoryWithCOMangle(KDL::Frame &point,double distance, KDL::Frame Leg, double angle)
{

    KDL::Frame start(point);
    /*std::cout<<point.M.GetRot().x()<<std::endl;*/

    KDL::Vector diff=(start.p-Leg.p);
    double L=(start.p-Leg.p).Norm();
    double initial_angle=atan2(diff.z(),diff.y());
    if(Leg.p.y()<=0){
        angle=angle;
    }else
    angle=-angle;
    double z=0,y=0,x=0,x0=start.p.x(),n=100,diff_angle=(angle/n),diff_x=distance/n;
    KDL::Rotation rot2,rot=start.M;
    KDL::Path_Line* COMpath;
    KDL::Path_Composite * parcialCOMpath=new KDL::Path_Composite();

    for(double i=0;i<=n;i+=1){
        z=L*sin(initial_angle+diff_angle*i)+Leg.p.z();
        y=L*cos(initial_angle+diff_angle*i)+Leg.p.y();
        x=x0+/*sin(i/n)*distance;//*/diff_x*i;
        rot2=rot.RotX(point.M.GetRot().x()+diff_angle*i*2);
//        data1 <<x<< DELIMITADOR<<y<< DELIMITADOR<<z<<DELIMITADOR<<rot2.GetRot().x()<<"\n";
        KDL::Frame current_point(rot2,KDL::Vector(x,y,z));
        COMpath =new KDL::Path_Line(start,current_point,orient.Clone(),eqradius);
        parcialCOMpath->Add(COMpath);
        start=current_point;
    }
//    std::cout<<rot2.GetRot().x()<<std::endl;
//    std::cout<<"----------------------"<<std::endl;
//std::cout<<n<<" ,"<<trajCOM->Duration()<<" ,"<<parcialCOMtrayectory->Duration()<<std::endl;

    return parcialCOMpath;

}
double TrajectoryGenerator::generateCurveLeg(double h,double L,double x)
{
    double a=-4*h/(L*L);
    double b=0,c=h;
    double y=a*(x-L/2)*(x-L/2)+b*(x-L/2)+c;
    return y;
}

double TrajectoryGenerator::generateCurveLeg(double h_max, double L, double h_ini, double x)
{
    double coef=1-h_max/h_ini;
    double xh=L*(coef+sqrt(coef*coef-coef));
    double a=h_ini/(-L*L+2*xh*L);
    double y=a*(x-xh)*(x-xh)+h_max;
    return y;
}
