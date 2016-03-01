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

#include "CFGBenchmark.h"
#include "omplapp/apps/SE2RigidBodyPlanning.h"
#include "omplapp/apps/SE3RigidBodyPlanning.h"
#include "omplapp/apps/BlimpPlanning.h"
#include "omplapp/apps/DynamicCarPlanning.h"
#include "omplapp/apps/KinematicCarPlanning.h"
#include "omplapp/apps/QuadrotorPlanning.h"



class SE2BaseBenchmark : public CFGBenchmark
{
public:
    SE2BaseBenchmark(const BenchmarkOptions &bo) : CFGBenchmark(bo)
    {
    }
protected:
    bool getStartState(ompl::base::ScopedState<ompl::base::SE2StateSpace>& start);
    bool getGoalState(ompl::base::ScopedState<ompl::base::SE2StateSpace>& goal);
    void setBounds(const ompl::base::StateSpacePtr& space);
};

class SE3BaseBenchmark : public CFGBenchmark
{
public:
    SE3BaseBenchmark(const BenchmarkOptions &bo) : CFGBenchmark(bo)
    {
    }
protected:
    bool getStartState(ompl::base::ScopedState<ompl::base::SE3StateSpace>& start);
    bool getGoalState(ompl::base::ScopedState<ompl::base::SE3StateSpace>& goal);
    void setBounds(const ompl::base::StateSpacePtr& space);
};

class SE2Benchmark : public SE2BaseBenchmark
{
public:
    SE2Benchmark(const BenchmarkOptions &bo) : SE2BaseBenchmark(bo)
    {
    }
protected:
    virtual ompl::control::DecompositionPtr allocDecomposition()
    {
        return setup_se2_->allocDecomposition();
    }
    virtual void configure(void);
    std::shared_ptr<ompl::app::SE2RigidBodyPlanning> setup_se2_;
};

class SE3Benchmark : public SE3BaseBenchmark
{
public:
    SE3Benchmark(const BenchmarkOptions &bo) : SE3BaseBenchmark(bo)
    {
    }
protected:
    virtual ompl::control::DecompositionPtr allocDecomposition()
    {
        return setup_se3_->allocDecomposition();
    }
    virtual void configure(void);
    std::shared_ptr<ompl::app::SE3RigidBodyPlanning> setup_se3_;
};

class KinematicCarBenchmark : public SE2BaseBenchmark
{
public:
    KinematicCarBenchmark(const BenchmarkOptions &bo) : SE2BaseBenchmark(bo)
    {
    }
protected:
    virtual ompl::control::DecompositionPtr allocDecomposition()
    {
        return setup_kinematicCar_->allocDecomposition();
    }
    virtual void configure(void);
    std::shared_ptr<ompl::app::KinematicCarPlanning> setup_kinematicCar_;
};

class DynamicCarBenchmark : public SE2BaseBenchmark
{
public:
    DynamicCarBenchmark(const BenchmarkOptions &bo) : SE2BaseBenchmark(bo)
    {
    }
protected:
    virtual ompl::control::DecompositionPtr allocDecomposition()
    {
        return setup_dynamicCar_->allocDecomposition();
    }
    virtual void configure(void);
    std::shared_ptr<ompl::app::DynamicCarPlanning> setup_dynamicCar_;
};

class BlimpBenchmark : public SE3BaseBenchmark
{
public:
    BlimpBenchmark(const BenchmarkOptions &bo) : SE3BaseBenchmark(bo)
    {
    }
protected:
    virtual ompl::control::DecompositionPtr allocDecomposition()
    {
        return setup_blimp_->allocDecomposition();
    }
    virtual void configure(void);
    std::shared_ptr<ompl::app::BlimpPlanning> setup_blimp_;
};

class QuadrotorBenchmark : public SE3BaseBenchmark
{
public:
    QuadrotorBenchmark(const BenchmarkOptions &bo) : SE3BaseBenchmark(bo)
    {
    }
protected:
    virtual ompl::control::DecompositionPtr allocDecomposition()
    {
        return setup_quadrotor_->allocDecomposition();
    }
    virtual void configure(void);
    std::shared_ptr<ompl::app::QuadrotorPlanning> setup_quadrotor_;
};
