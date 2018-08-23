/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Mark Moll */

#include <ompl/control/planners/syclop/GridDecomposition.h>
#include "BenchmarkTypes.h"


bool SE2BaseBenchmark::getStartState(ompl::base::ScopedState<ompl::base::SE2StateSpace>& start)
{
    try
    {
        start->setX(std::stod(bo_.declared_options_["problem.start.x"]));
        start->setY(std::stod(bo_.declared_options_["problem.start.y"]));
        start->setYaw(std::stod(bo_.declared_options_["problem.start.theta"]));
    }
    catch(std::invalid_argument &)
    {
        std::cerr << "Unable to read start state" << std::endl;
        return false;
    }
    return true;
}

bool SE2BaseBenchmark::getGoalState(ompl::base::ScopedState<ompl::base::SE2StateSpace>& goal)
{
    try
    {
        goal->setX(std::stod(bo_.declared_options_["problem.goal.x"]));
        goal->setY(std::stod(bo_.declared_options_["problem.goal.y"]));
        goal->setYaw(std::stod(bo_.declared_options_["problem.goal.theta"]));
    }
    catch(std::invalid_argument &)
    {
        std::cerr << "Unable to read goal state" << std::endl;
        return false;
    }
    return true;
}

void SE2BaseBenchmark::setBounds(const ompl::base::StateSpacePtr& space)
{
    try
    {
        if (bo_.declared_options_.find("problem.volume.min.x") != bo_.declared_options_.end() && bo_.declared_options_.find("problem.volume.min.y") != bo_.declared_options_.end() &&
            bo_.declared_options_.find("problem.volume.max.x") != bo_.declared_options_.end() && bo_.declared_options_.find("problem.volume.max.y") != bo_.declared_options_.end())
        {
            ompl::base::RealVectorBounds bounds(2);
            bounds.setLow(0, std::stod(bo_.declared_options_["problem.volume.min.x"]));
            bounds.setLow(1, std::stod(bo_.declared_options_["problem.volume.min.y"]));
            bounds.setHigh(0, std::stod(bo_.declared_options_["problem.volume.max.x"]));
            bounds.setHigh(1, std::stod(bo_.declared_options_["problem.volume.max.y"]));
            space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
        }
    }
    catch(std::invalid_argument &)
    {
    }
}


bool SE3BaseBenchmark::getStartState(ompl::base::ScopedState<ompl::base::SE3StateSpace>& start)
{
    try
    {
        start->setXYZ(std::stod(bo_.declared_options_["problem.start.x"]),
                      std::stod(bo_.declared_options_["problem.start.y"]),
                      std::stod(bo_.declared_options_["problem.start.z"]));
        start->rotation().setAxisAngle(std::stod(bo_.declared_options_["problem.start.axis.x"]),
                                       std::stod(bo_.declared_options_["problem.start.axis.y"]),
                                       std::stod(bo_.declared_options_["problem.start.axis.z"]),
                                       std::stod(bo_.declared_options_["problem.start.theta"]));
    }
    catch(std::invalid_argument &)
    {
        std::cerr << "Unable to read start state" << std::endl;
        return false;
    }
    return true;
}

bool SE3BaseBenchmark::getGoalState(ompl::base::ScopedState<ompl::base::SE3StateSpace>& goal)
{
    try
    {
        goal->setXYZ(std::stod(bo_.declared_options_["problem.goal.x"]),
                     std::stod(bo_.declared_options_["problem.goal.y"]),
                     std::stod(bo_.declared_options_["problem.goal.z"]));
        goal->rotation().setAxisAngle(std::stod(bo_.declared_options_["problem.goal.axis.x"]),
                                      std::stod(bo_.declared_options_["problem.goal.axis.y"]),
                                      std::stod(bo_.declared_options_["problem.goal.axis.z"]),
                                      std::stod(bo_.declared_options_["problem.goal.theta"]));
    }
    catch(std::invalid_argument &)
    {
        std::cerr << "Unable to read goal state" << std::endl;
        return false;
    }
    return true;
}

void SE3BaseBenchmark::setBounds(const ompl::base::StateSpacePtr& space)
{
    try
    {
        if (bo_.declared_options_.find("problem.volume.min.x") != bo_.declared_options_.end() && bo_.declared_options_.find("problem.volume.min.y") != bo_.declared_options_.end() &&
            bo_.declared_options_.find("problem.volume.min.z") != bo_.declared_options_.end() &&
            bo_.declared_options_.find("problem.volume.max.x") != bo_.declared_options_.end() && bo_.declared_options_.find("problem.volume.max.y") != bo_.declared_options_.end() &&
            bo_.declared_options_.find("problem.volume.max.y") != bo_.declared_options_.end())
            {
            ompl::base::RealVectorBounds bounds(3);
            bounds.setLow(0, std::stod(bo_.declared_options_["problem.volume.min.x"]));
            bounds.setLow(1, std::stod(bo_.declared_options_["problem.volume.min.y"]));
            bounds.setLow(2, std::stod(bo_.declared_options_["problem.volume.min.z"]));
            bounds.setHigh(0, std::stod(bo_.declared_options_["problem.volume.max.x"]));
            bounds.setHigh(1, std::stod(bo_.declared_options_["problem.volume.max.y"]));
            bounds.setHigh(2, std::stod(bo_.declared_options_["problem.volume.max.z"]));
            space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
        }
    }
    catch(std::invalid_argument &)
    {
    }
}


void SE2Benchmark::configure()
{
    setup_se2_ = std::make_shared<ompl::app::SE2RigidBodyPlanning>();
    setup_se2_->setRobotMesh(getRobotMesh());
    setup_se2_->setEnvironmentMesh(getEnvironmentMesh());

    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(setup_se2_->getStateSpace());
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(setup_se2_->getStateSpace());
    if (!getStartState(start))
        return;
    if (!getGoalState(goal))
        return;

    auto t = std::stod(bo_.declared_options_["problem.threshold"]);
    setup_se2_->setStartAndGoalStates(start, goal, t);
    setBounds(setup_se2_->getStateSpace());
    setup_se2_->setOptimizationObjective(getOptimizationObjective(setup_se2_->getSpaceInformation()));
    setup_se2_->setup();
    setup_se2_->print();
    benchmark_ = std::make_shared<ompl::tools::Benchmark>(*setup_se2_, bo_.declared_options_["problem.name"]);
}

void SE3Benchmark::configure()
{
    setup_se3_ = std::make_shared<ompl::app::SE3RigidBodyPlanning>();
    setup_se3_->setRobotMesh(getRobotMesh());
    setup_se3_->setEnvironmentMesh(getEnvironmentMesh());
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup_se3_->getStateSpace());
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(setup_se3_->getStateSpace());
    if (!getStartState(start))
        return;
    if (!getGoalState(goal))
        return;

    auto t = std::stod(bo_.declared_options_["problem.threshold"]);
    setup_se3_->setStartAndGoalStates(start, goal, t);
    setBounds(setup_se3_->getStateSpace());
    setup_se3_->setOptimizationObjective(getOptimizationObjective(setup_se3_->getSpaceInformation()));
    setup_se3_->setup();
    setup_se3_->print();
    benchmark_ = std::make_shared<ompl::tools::Benchmark>(*setup_se3_, bo_.declared_options_["problem.name"]);
}

void KinematicCarBenchmark::configure()
{
    setup_kinematicCar_ = std::make_shared<ompl::app::KinematicCarPlanning>();
    setup_kinematicCar_->setRobotMesh(getRobotMesh());
    setup_kinematicCar_->setEnvironmentMesh(getEnvironmentMesh());

    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(setup_kinematicCar_->getStateSpace());
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(setup_kinematicCar_->getStateSpace());
    if (!getStartState(start))
        return;
    if (!getGoalState(goal))
        return;

    auto t = std::stod(bo_.declared_options_["problem.threshold"]);
    setup_kinematicCar_->setStartAndGoalStates(start, goal, t);
    setBounds(setup_kinematicCar_->getStateSpace());
    setup_kinematicCar_->setOptimizationObjective(getOptimizationObjective(setup_kinematicCar_->getSpaceInformation()));
    setup_kinematicCar_->setup();
    setup_kinematicCar_->print();
    benchmark_ = std::make_shared<ompl::tools::Benchmark>(*setup_kinematicCar_, bo_.declared_options_["problem.name"]);
}

void DynamicCarBenchmark::configure()
{
    setup_dynamicCar_ = std::make_shared<ompl::app::DynamicCarPlanning>();
    setup_dynamicCar_->setRobotMesh(getRobotMesh());
    setup_dynamicCar_->setEnvironmentMesh(getEnvironmentMesh());

    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(setup_dynamicCar_->getGeometricComponentStateSpace());
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(setup_dynamicCar_->getGeometricComponentStateSpace());
    if (!getStartState(start))
        return;
    if (!getGoalState(goal))
        return;

    auto t = std::stod(bo_.declared_options_["problem.threshold"]);
    setup_dynamicCar_->setStartAndGoalStates(
        setup_dynamicCar_->getFullStateFromGeometricComponent(start),
        setup_dynamicCar_->getFullStateFromGeometricComponent(goal), t);
    setBounds(setup_dynamicCar_->getGeometricComponentStateSpace());
    setup_dynamicCar_->setOptimizationObjective(getOptimizationObjective(setup_dynamicCar_->getSpaceInformation()));
    setup_dynamicCar_->setup();
    setup_dynamicCar_->print();
    benchmark_ = std::make_shared<ompl::tools::Benchmark>(*setup_dynamicCar_, bo_.declared_options_["problem.name"]);
}

void BlimpBenchmark::configure()
{
    setup_blimp_ = std::make_shared<ompl::app::BlimpPlanning>();
    setup_blimp_->setRobotMesh(getRobotMesh());
    setup_blimp_->setEnvironmentMesh(getEnvironmentMesh());
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup_blimp_->getGeometricComponentStateSpace());
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(setup_blimp_->getGeometricComponentStateSpace());
    if (!getStartState(start))
        return;
    if (!getGoalState(goal))
        return;

    auto t = std::stod(bo_.declared_options_["problem.threshold"]);
    setup_blimp_->setStartAndGoalStates(
        setup_blimp_->getFullStateFromGeometricComponent(start),
        setup_blimp_->getFullStateFromGeometricComponent(goal), t);
    setBounds(setup_blimp_->getGeometricComponentStateSpace());
    setup_blimp_->setOptimizationObjective(getOptimizationObjective(setup_blimp_->getSpaceInformation()));
    setup_blimp_->setup();
    setup_blimp_->print();
    benchmark_ = std::make_shared<ompl::tools::Benchmark>(*setup_blimp_, bo_.declared_options_["problem.name"]);
}
void QuadrotorBenchmark::configure()
{
    setup_quadrotor_ = std::make_shared<ompl::app::QuadrotorPlanning>();
    setup_quadrotor_->setRobotMesh(getRobotMesh());
    setup_quadrotor_->setEnvironmentMesh(getEnvironmentMesh());
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup_quadrotor_->getGeometricComponentStateSpace());
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(setup_quadrotor_->getGeometricComponentStateSpace());
    if (!getStartState(start))
        return;
    if (!getGoalState(goal))
        return;

    auto t = std::stod(bo_.declared_options_["problem.threshold"]);
    setup_quadrotor_->setStartAndGoalStates(
        setup_quadrotor_->getFullStateFromGeometricComponent(start),
        setup_quadrotor_->getFullStateFromGeometricComponent(goal), t);
    setBounds(setup_quadrotor_->getGeometricComponentStateSpace());
    setup_quadrotor_->setOptimizationObjective(getOptimizationObjective(setup_quadrotor_->getSpaceInformation()));
    setup_quadrotor_->setup();
    setup_quadrotor_->print();
    benchmark_ = std::make_shared<ompl::tools::Benchmark>(*setup_quadrotor_, bo_.declared_options_["problem.name"]);
}
