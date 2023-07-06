// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_GENERATOR_HPP__
#define __TRAJECTORY_GENERATOR_HPP__

#include <vector>

#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>

#include "GaitSpecs.hpp"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(FootSpec footSpec, double distance, double vel, double acc);
    void configure(const std::vector<KDL::Frame> & steps, const std::vector<KDL::Frame> & com);
    void generate(KDL::Trajectory_Composite & comTraj, KDL::Trajectory_Composite & leftTraj, KDL::Trajectory_Composite & rightTraj);
    std::ofstream data1;
private:
    double getDuration(const KDL::Vector & v);
    KDL::Frame makeHop(const KDL::Frame & H1, const KDL::Frame & H2);
    std::vector<double> findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts);
    double generateCurveLeg(double h,double L,double x);
    KDL::Path_Composite * getPathLegSwing(double h, KDL::Frame &step_start, KDL::Frame &step_end, double nPoints);
    FootSpec footSpec;
    double distance;
    double vel;
    double acc;
    double radius;
    double eqradius;
    std::vector<KDL::Frame> steps;
    std::vector<KDL::Frame> com;
    KDL::RotationalInterpolation_SingleAxis orient;
};

#endif // __TRAJECTORY_GENERATOR_HPP__
