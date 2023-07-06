// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StepGenerator.hpp"
#include "LIMP.hpp"
#include <yarp/os/LogStream.h>
#include <cmath>
#include<iostream>
StepGenerator::StepGenerator(FootSpec _footSpec, const KDL::Frame & _initialPoseRight,const KDL::Frame & _initialPoseLeft)
    : footSpec(_footSpec),
      leftInitialPose(_initialPoseLeft),
      rightInitialPose(_initialPoseRight)
{}

void StepGenerator::configure(GaitSpec _gaitSpec)
{
    gaitSpec = _gaitSpec;
}
void StepGenerator::generate(double distance, std::vector<KDL::Frame> & steps, std::vector<tuple<KDL::Frame,double>> & COM)
{
    steps.clear();
    COM.clear();

    double leftInitialSep = leftInitialPose.p.y();
    double RightInitialSep = rightInitialPose.p.y();
    double stableSep = gaitSpec.sep;
    double stepSep = gaitSpec.sep;
    double initialSquat = -leftInitialPose.p.z();
    double hopSquat =gaitSpec.squat;

    double travelled = 0.0;
    bool movingRightFoot = true;

    KDL::Rotation rot = leftInitialPose.M;
    double phaseWalking=0;
    /***Asumimos pasos constantes y rectos modificaremso esto en un futuro**/
    double stepsSep=gaitSpec.sep;
    double lenghtStep=gaitSpec.step;
    double dy=0.01;
    double zModel=gaitSpec.squat;
    double g=9.807;
    LIMP limp(0.02,zModel,g);
    std::vector<double> x0=findInitialConditions(lenghtStep,dy,stepsSep,zModel,g,0.02);
    double tSingleSupport=x0[x0.size()-1];
    x0.pop_back();
    limp.setInitialCondition(x0,tSingleSupport);
//    std::cout<<"sep:"<<x0[0]<<" longitud: "<<x0[2]<<std::endl;
        std::vector<tuple<KDL::Frame,double>> a;

    if(movingRightFoot){
    steps.emplace_back(rot, KDL::Vector(0, RightInitialSep, 0));
    steps.emplace_back(rot, KDL::Vector(0, leftInitialSep, 0));
    COM.push_back(make_tuple(KDL::Frame(KDL::Vector(0, 0, initialSquat)),phaseWalking));
    COM.push_back(make_tuple(KDL::Frame(KDL::Vector(0, 0, hopSquat)),phaseWalking));
    COM.push_back(make_tuple(KDL::Frame(KDL::Vector(0, stableSep, hopSquat)),phaseWalking));
    }
    else{
        steps.emplace_back(rot, KDL::Vector(0, leftInitialSep, 0));
        steps.emplace_back(rot, KDL::Vector(0, RightInitialSep, 0));
        COM.push_back(make_tuple(KDL::Frame(KDL::Vector(0, 0, initialSquat)),phaseWalking));
        COM.push_back(make_tuple(KDL::Frame(KDL::Vector(0, 0, hopSquat)),phaseWalking));
        COM.push_back(make_tuple(KDL::Frame(KDL::Vector(0, -stableSep, hopSquat)),phaseWalking));
    }




    if (distance <= gaitSpec.step)
    {
        steps.emplace_back(rot, KDL::Vector(distance, RightInitialSep, 0));
        steps.emplace_back(rot, KDL::Vector(distance, leftInitialSep, 0));
        COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, -stableSep, hopSquat)),phaseWalking));
        COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, 0, hopSquat)),phaseWalking));
        COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, 0, initialSquat)),phaseWalking));

        return;
    }



//    if (movingRightFoot)
//    {
//        steps.emplace_back(rot, KDL::Vector(gaitSpec.step/2, -stepSep, 0));
//    }
//    else
//    {
//        steps.emplace_back(rot, KDL::Vector(gaitSpec.step/2, stepSep, 0));
//    }
//    movingRightFoot = !movingRightFoot;
//    travelled += gaitSpec.step/2;
    do
    {
        travelled += gaitSpec.step;
        bool isLastStep = travelled >= (distance - 1e-9);
        travelled = std::min(travelled, distance);

        if (isLastStep)
        {
            limp.getTrayectory(steps,COM,footSpec.length,footSpec.width);
            distance=steps[steps.size()-1].p.x()+ gaitSpec.step;
            phaseWalking=0.0;
            if (movingRightFoot)
            {
                steps.emplace_back(rot, KDL::Vector(distance, RightInitialSep, 0));
                steps.emplace_back(rot, KDL::Vector(distance, leftInitialSep, 0));
                COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, -stableSep, hopSquat)),phaseWalking));
                COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, 0, hopSquat)),phaseWalking));
                COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, 0, initialSquat)),phaseWalking));
            }
            else
            {
                steps.emplace_back(rot, KDL::Vector(distance, leftInitialSep, 0));
                steps.emplace_back(rot, KDL::Vector(distance, RightInitialSep, 0));
                COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, stableSep, hopSquat)),phaseWalking));
                COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, 0, hopSquat)),phaseWalking));
                COM.push_back(make_tuple(KDL::Frame(KDL::Vector(distance, 0, initialSquat)),phaseWalking));
            }

            break;
        }

        if (movingRightFoot)
        {
            steps.emplace_back(rot, KDL::Vector(travelled, -stepSep, 0));
        }
        else
        {
            steps.emplace_back(rot, KDL::Vector(travelled, stepSep, 0));
        }

        movingRightFoot = !movingRightFoot;
    }
    while (true);


    /**Una vez tenemos los pasos procederemos a calcular las trayectorias de las piernas y del centro de masas con el LIMP ***/
    //los dos primeros steps son las posiciones iniciales de los pies y los dos ultimos son las posiciones finales por tanto nos iteresan los
    //steps intermedios.




//    yDebug()<<std::get<1>(a[200])<<"Hola como estan";
}

void StepGenerator::generate(double distance, std::vector<KDL::Frame> & steps, std::vector<KDL::Frame> & com)
{
    steps.clear();
    com.clear();

    double leftInitialSep = leftInitialPose.p.y();
    double RightInitialSep = rightInitialPose.p.y();
    double stableSep = gaitSpec.sep;
    double stepSep = gaitSpec.sep;
    double initialSquat = -leftInitialPose.p.z();
    double hopSquat =gaitSpec.squat;

    double travelled = 0.0;
    bool movingRightFoot = true;

    KDL::Rotation rot = leftInitialPose.M;

    /***Asumimos pasos constantes y rectos modificaremso esto en un futuro**/
    double stepsSep=gaitSpec.sep;
    double lenghtStep=gaitSpec.step;
    double dy=0.01;
    double zModel=gaitSpec.squat;
    double g=9.807;
    LIMP limp(0.02,zModel,g);
    std::vector<double> x0=findInitialConditions(lenghtStep,dy,stepsSep,zModel,g,0.02);
    double tSingleSupport=x0[x0.size()-1];
    x0.pop_back();
    limp.setInitialCondition(x0,tSingleSupport);
//    std::cout<<"sep:"<<x0[0]<<" longitud: "<<x0[2]<<std::endl;
        std::vector<tuple<KDL::Frame,double>> a;

    if(movingRightFoot){
    steps.emplace_back(rot, KDL::Vector(0, RightInitialSep, 0));
    steps.emplace_back(rot, KDL::Vector(0, leftInitialSep, 0));
    com.emplace_back(KDL::Vector(0, 0, initialSquat));
    com.emplace_back(KDL::Vector(0, 0, hopSquat));
    com.emplace_back(KDL::Vector(0, stableSep, hopSquat));
    }
    else{
        steps.emplace_back(rot, KDL::Vector(0, leftInitialSep, 0));
        steps.emplace_back(rot, KDL::Vector(0, RightInitialSep, 0));
        com.emplace_back(KDL::Vector(0, 0, initialSquat));
        com.emplace_back(KDL::Vector(0, 0, hopSquat));
        com.emplace_back(KDL::Vector(0, -stableSep, hopSquat));
    }




    if (distance <= gaitSpec.step)
    {


        steps.emplace_back(rot, KDL::Vector(distance, RightInitialSep, 0));
        steps.emplace_back(rot, KDL::Vector(distance, leftInitialSep, 0));

        com.emplace_back(KDL::Vector(distance, -stableSep, hopSquat));
        com.emplace_back(KDL::Vector(distance, 0, hopSquat));
        com.emplace_back(KDL::Vector(distance, 0, initialSquat));

        return;
    }



//    if (movingRightFoot)
//    {
//        steps.emplace_back(rot, KDL::Vector(gaitSpec.step/2, -stepSep, 0));
//    }
//    else
//    {
//        steps.emplace_back(rot, KDL::Vector(gaitSpec.step/2, stepSep, 0));
//    }
//    movingRightFoot = !movingRightFoot;
//    travelled += gaitSpec.step/2;
    do
    {
        travelled += gaitSpec.step;
        bool isLastStep = travelled >= (distance - 1e-9);
        travelled = std::min(travelled, distance);

        if (isLastStep)
        {
//            limp.getTrayectory(steps,a,footSpec.length,footSpec.width);
//            distance=steps[steps.size()-1].p.x()+ gaitSpec.step;
            if (movingRightFoot)
            {
                steps.emplace_back(rot, KDL::Vector(distance, RightInitialSep, 0));
                steps.emplace_back(rot, KDL::Vector(distance, leftInitialSep, 0));

                com.emplace_back(KDL::Vector(distance, -stableSep, hopSquat));
                com.emplace_back(KDL::Vector(distance, 0, hopSquat));
                com.emplace_back(KDL::Vector(distance, 0, initialSquat));
            }
            else
            {
                steps.emplace_back(rot, KDL::Vector(distance, leftInitialSep, 0));
                steps.emplace_back(rot, KDL::Vector(distance, RightInitialSep, 0));

                com.emplace_back(KDL::Vector(distance, stableSep, hopSquat));
                com.emplace_back(KDL::Vector(distance, 0, hopSquat));
                com.emplace_back(KDL::Vector(distance, 0, initialSquat));
            }

            break;
        }

        if (movingRightFoot)
        {
            steps.emplace_back(rot, KDL::Vector(travelled, -stepSep, 0));
        }
        else
        {
            steps.emplace_back(rot, KDL::Vector(travelled, stepSep, 0));
        }

        movingRightFoot = !movingRightFoot;
    }
    while (true);


    /**Una vez tenemos los pasos procederemos a calcular las trayectorias de las piernas y del centro de masas con el LIMP ***/
    //los dos primeros steps son las posiciones iniciales de los pies y los dos ultimos son las posiciones finales por tanto nos iteresan los
    //steps intermedios.




//    yDebug()<<std::get<1>(a[200])<<"Hola como estan";
}
std::vector<double> StepGenerator::findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts){
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
