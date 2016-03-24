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

#include <ompl/control/planners/syclop/Decomposition.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include "BenchmarkOptions.h"

class CFGBenchmark
{
public:

    CFGBenchmark(const BenchmarkOptions &bo) : bo_(bo)
    {
    }

    virtual ~CFGBenchmark(void)
    {
    }

    bool isValid(void) const
    {
        return benchmark_.get();
    }

    void runBenchmark(void);

    void setup(void);

protected:
    std::string getRobotMesh();
    std::string getEnvironmentMesh();

    virtual void configure(void) = 0;

    ompl::base::OptimizationObjectivePtr getOptimizationObjective(const ompl::base::SpaceInformationPtr &si);

    // decomposition, needed for Syclop
    virtual ompl::control::DecompositionPtr allocDecomposition() = 0;

    // optional post-run events
    virtual void saveAllPaths(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run);
    virtual void saveBestPath(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run);

    BenchmarkOptions                                             bo_;
    std::map<ompl::base::Planner*, BenchmarkOptions::ContextOpt> pcontext_;
    BenchmarkOptions::ContextOpt                                 activeParams_;
    ompl::base::Cost                                             defaultCostThreshold_;
    std::shared_ptr<ompl::tools::Benchmark>                      benchmark_;

    // When the best path is saved, we keep it here
    ompl::base::PathPtr                                          bestPath_;
    unsigned int                                                 bestPathIndex_;

private:

    ompl::base::PlannerPtr allocPlanner(const ompl::base::SpaceInformationPtr &si, const std::string &name, const BenchmarkOptions::AllOptions &opt);
    ompl::base::ValidStateSamplerPtr allocValidStateSampler(const ompl::base::SpaceInformation *si, const std::string &type);
    void setupBenchmark(void);
    void preSwitchEvent(const ompl::base::PlannerPtr &planner);
};
