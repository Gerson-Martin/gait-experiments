// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StepGenerator.hpp"
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
            if (movingRightFoot)
            {
                steps.emplace_back(rot, KDL::Vector(distance, RightInitialSep, 0));
                steps.emplace_back(rot, KDL::Vector(distance, leftInitialSep, 0));

                com.emplace_back(KDL::Vector(travelled, -stableSep, hopSquat));
                com.emplace_back(KDL::Vector(travelled, 0, hopSquat));
                com.emplace_back(KDL::Vector(travelled, 0, initialSquat));
            }
            else
            {
                steps.emplace_back(rot, KDL::Vector(distance, leftInitialSep, 0));
                steps.emplace_back(rot, KDL::Vector(distance, RightInitialSep, 0));

                com.emplace_back(KDL::Vector(travelled, stableSep, hopSquat));
                com.emplace_back(KDL::Vector(travelled, 0, hopSquat));
                com.emplace_back(KDL::Vector(travelled, 0, initialSquat));
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
//        travelled += gaitSpec.step;
    }
    while (true);
}
