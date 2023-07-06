// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TargetBuilder.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <KdlVectorConverter.hpp>

TargetBuilder::TargetBuilder(walkingRobot &_legs):
   legs(&_legs),
    tCom(0),
    tLeft(0),
    tRight(0),
    maxTime(0.0)
{}

void TargetBuilder::configure(KDL::Trajectory * _tCom, KDL::Trajectory * _tLeft, KDL::Trajectory * _tRight)
{
    tCom = _tCom;
    tLeft = _tLeft;
    tRight = _tRight;

    maxTime = std::max(tCom->Duration(), std::max(tLeft->Duration(), tRight->Duration()));
}
double periodo;


TargetBuilder::Targets TargetBuilder::build(double period, Targets & vLeft, Targets & vRight)
{
    vLeft.clear();
    vRight.clear();
    periodo=period;
    double t = 0.0;
    Targets com;

    while (t < maxTime)
    {
        KDL::Frame H_com = tCom->Pos(t);
        KDL::Frame H_left = tLeft->Pos(t);
        KDL::Frame H_right = tRight->Pos(t);

        KDL::Frame H_com_inv = H_com.Inverse();
        KDL::Frame H_com_left = H_com_inv * H_left;
        KDL::Frame H_com_right = H_com_inv * H_right;

        vLeft.push_back(rl::KdlVectorConverter::frameToVector(H_com_left));
        vRight.push_back(rl::KdlVectorConverter::frameToVector(H_com_right));
        com.push_back(rl::KdlVectorConverter::frameToVector(H_com));
        int i=vLeft.size()-1;
         t += period;
    }
    return com;
}


bool TargetBuilder::validate(TargetBuilder::Targets &vLeft, TargetBuilder::Targets &vRight)
{
    std::vector<double> q;

    for (int i = 0; i < vLeft.size(); i++)
    {
        if (!legs->inv_Left(vLeft[i], q))
        {
            yWarning() << "IK failing at left leg:" << vLeft[i][0] << vLeft[i][1] << vLeft[i][2] << vLeft[i][3] << vLeft[i][4] << vLeft[i][5];
            yDebug() << "-------left leg joins:" << q[0]<<"," << q[1]<<"," << q[2]<<"," << q[3]<<"," << q[4]<<"," <<q[5];
             yWarning() <<"Tiempo: "<<periodo*i<<"; index i: "<<i;
            return false;
        }

        if (!legs->inv_Right(vRight[i], q))
        {
            yWarning() << "IK failing at right leg:" << vRight[i][0] << vRight[i][1] << vRight[i][2] << vRight[i][3] << vRight[i][4] << vRight[i][5];
            yWarning() <<"Tiempo: "<<periodo*i<<"; index i: "<<i;
            yDebug() << "-------right leg joins:" << q[0]<<"," << q[1]<<"," << q[2]<<"," << q[3]<<"," << q[4]<<"," <<q[5];
            return false;
        }
    }

    return true;
}
