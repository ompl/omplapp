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

/* Author: Ioan Sucan */

#include "GeometricBenchmark.h"

void SE2Benchmark::configure(void)
{
    setup_se2_.reset(new ompl::app::SE2RigidBodyPlanning());
    setup_se2_->setRobotMesh((bo_.path_ / bo_.declared_options_["problem.robot"]).string());
    setup_se2_->setEnvironmentMesh((bo_.path_ / bo_.declared_options_["problem.world"]).string());
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(setup_se2_->getStateSpace());
    try
    {
        start->setX(boost::lexical_cast<double>(bo_.declared_options_["problem.start.x"]));
        start->setY(boost::lexical_cast<double>(bo_.declared_options_["problem.start.y"]));
        start->setYaw(boost::lexical_cast<double>(bo_.declared_options_["problem.start.theta"]));
    }
    catch(boost::bad_lexical_cast &)
    {
        std::cerr << "Unable to read start state" << std::endl;
        return;
    }
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(setup_se2_->getStateSpace());

    try
    {
        goal->setX(boost::lexical_cast<double>(bo_.declared_options_["problem.goal.x"]));
        goal->setY(boost::lexical_cast<double>(bo_.declared_options_["problem.goal.y"]));
        goal->setYaw(boost::lexical_cast<double>(bo_.declared_options_["problem.goal.theta"]));
    }
    catch(boost::bad_lexical_cast &)
    {
        std::cerr << "Unable to read goal state" << std::endl;
        return;
    }

    try
    {
        double t = boost::lexical_cast<double>(bo_.declared_options_["problem.threshold"]);
        setup_se2_->setStartAndGoalStates(start, goal, t);
    }
    catch(boost::bad_lexical_cast &)
    {
        setup_se2_->setStartAndGoalStates(start, goal);
    }

    try
    {
        if (bo_.declared_options_.find("problem.volume.min.x") != bo_.declared_options_.end() && bo_.declared_options_.find("problem.volume.min.y") != bo_.declared_options_.end() &&
            bo_.declared_options_.find("problem.volume.max.x") != bo_.declared_options_.end() && bo_.declared_options_.find("problem.volume.max.y") != bo_.declared_options_.end())
        {
            ompl::base::RealVectorBounds bounds(2);
            bounds.setLow(0, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.min.x"]));
            bounds.setLow(1, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.min.y"]));
            bounds.setHigh(0, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.max.x"]));
            bounds.setHigh(1, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.max.y"]));
            setup_se2_->getStateSpace()->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
        }
    }
    catch(boost::bad_lexical_cast &)
    {
    }
    setup_se2_->setup();
    setup_se2_->print();
    benchmark_.reset(new ompl::tools::Benchmark(*setup_se2_, bo_.declared_options_["problem.name"]));
}

void SE3Benchmark::configure(void)
{
    setup_se3_.reset(new ompl::app::SE3RigidBodyPlanning());
    setup_se3_->setRobotMesh((bo_.path_ / bo_.declared_options_["problem.robot"]).string());
    setup_se3_->setEnvironmentMesh((bo_.path_ / bo_.declared_options_["problem.world"]).string());
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup_se3_->getStateSpace());
    try
    {
        start->setXYZ(boost::lexical_cast<double>(bo_.declared_options_["problem.start.x"]),
                      boost::lexical_cast<double>(bo_.declared_options_["problem.start.y"]),
                      boost::lexical_cast<double>(bo_.declared_options_["problem.start.z"]));
        start->rotation().setAxisAngle(boost::lexical_cast<double>(bo_.declared_options_["problem.start.axis.x"]),
                                       boost::lexical_cast<double>(bo_.declared_options_["problem.start.axis.y"]),
                                       boost::lexical_cast<double>(bo_.declared_options_["problem.start.axis.z"]),
                                       boost::lexical_cast<double>(bo_.declared_options_["problem.start.theta"]));
    }
    catch(boost::bad_lexical_cast &)
    {
        std::cerr << "Unable to read start state" << std::endl;
        return;
    }

    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(setup_se3_->getStateSpace());
    try
    {
        goal->setXYZ(boost::lexical_cast<double>(bo_.declared_options_["problem.goal.x"]),
                     boost::lexical_cast<double>(bo_.declared_options_["problem.goal.y"]),
                     boost::lexical_cast<double>(bo_.declared_options_["problem.goal.z"]));
        goal->rotation().setAxisAngle(boost::lexical_cast<double>(bo_.declared_options_["problem.goal.axis.x"]),
                                      boost::lexical_cast<double>(bo_.declared_options_["problem.goal.axis.y"]),
                                      boost::lexical_cast<double>(bo_.declared_options_["problem.goal.axis.z"]),
                                      boost::lexical_cast<double>(bo_.declared_options_["problem.goal.theta"]));
    }
    catch(boost::bad_lexical_cast &)
    {
        std::cerr << "Unable to read goal state" << std::endl;
        return;
    }

    try
    {
        double t = boost::lexical_cast<double>(bo_.declared_options_["problem.threshold"]);
        setup_se3_->setStartAndGoalStates(start, goal, t);
    }
    catch(boost::bad_lexical_cast &)
    {
        setup_se3_->setStartAndGoalStates(start, goal);
    }

    try
    {
        if (bo_.declared_options_.find("problem.volume.min.x") != bo_.declared_options_.end() && bo_.declared_options_.find("problem.volume.min.y") != bo_.declared_options_.end() &&
            bo_.declared_options_.find("problem.volume.min.z") != bo_.declared_options_.end() &&
            bo_.declared_options_.find("problem.volume.max.x") != bo_.declared_options_.end() && bo_.declared_options_.find("problem.volume.max.y") != bo_.declared_options_.end() &&
            bo_.declared_options_.find("problem.volume.max.y") != bo_.declared_options_.end())
	    {
            ompl::base::RealVectorBounds bounds(3);
            bounds.setLow(0, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.min.x"]));
            bounds.setLow(1, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.min.y"]));
            bounds.setLow(2, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.min.z"]));
            bounds.setHigh(0, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.max.x"]));
            bounds.setHigh(1, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.max.y"]));
            bounds.setHigh(2, boost::lexical_cast<double>(bo_.declared_options_["problem.volume.max.z"]));
            setup_se3_->getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
        }
    }
    catch(boost::bad_lexical_cast &)
    {
    }

    setup_se3_->setup();
    setup_se3_->print();
    benchmark_.reset(new ompl::tools::Benchmark(*setup_se3_, bo_.declared_options_["problem.name"]));
}

