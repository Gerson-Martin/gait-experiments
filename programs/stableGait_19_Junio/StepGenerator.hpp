// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STEP_GENERATOR_HPP__
#define __STEP_GENERATOR_HPP__

#include <vector>

#include <kdl/frames.hpp>

#include "GaitSpecs.hpp"

class StepGenerator
{
public:
    StepGenerator(FootSpec footSpec, const KDL::Frame & initialPoseRight,const KDL::Frame & initialPoseLeft);
    void configure(GaitSpec gaitSpec);
    void generate(double distance, std::vector<KDL::Frame> & stepsLeft, std::vector<KDL::Frame> & stepsRight);

    void generate(double distance, std::vector<KDL::Frame> &steps, std::vector<std::tuple<KDL::Frame, double> > &COM);
private:
    std::vector<double> findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts);
    FootSpec footSpec;
    GaitSpec gaitSpec;
    KDL::Frame leftInitialPose;
    KDL::Frame rightInitialPose;
};

#endif // __STEP_GENERATOR_HPP__
