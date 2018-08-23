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

#include <omplapp/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/cforest/CForest.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>

#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>

#include <fstream>

namespace {
    boost::filesystem::path getAbsolutePath(const boost::filesystem::path& path, const boost::filesystem::path& prefix)
    {
        return (path.is_absolute()) ? path : (prefix / path);
    }
}

std::string CFGBenchmark::getRobotMesh()
{
    return getAbsolutePath(
        boost::filesystem::path(bo_.declared_options_["problem.robot"]), bo_.path_).string();
}
std::string CFGBenchmark::getEnvironmentMesh()
{
    return getAbsolutePath(
        boost::filesystem::path(bo_.declared_options_["problem.world"]), bo_.path_).string();
}

ompl::base::PlannerPtr CFGBenchmark::allocPlanner(const ompl::base::SpaceInformationPtr &si, const std::string &name, const BenchmarkOptions::AllOptions &opt)
{
    const ompl::control::SpaceInformationPtr siC = std::dynamic_pointer_cast<ompl::control::SpaceInformation>(si);
    ompl::base::PlannerPtr p;

    if (siC)
    {
        if (name == "rrt")
            p = std::make_shared<ompl::control::RRT>(siC);
        else if (name == "est")
            p = std::make_shared<ompl::control::EST>(siC);
        else if (name == "kpiece")
            p = std::make_shared<ompl::control::KPIECE1>(siC);
        else if (name == "pdst")
            p = std::make_shared<ompl::control::PDST>(siC);
        else if (name == "sycloprrt")
            p = std::make_shared<ompl::control::SyclopRRT>(siC, allocDecomposition());
        else if (name == "syclopest")
            p = std::make_shared<ompl::control::SyclopEST>(siC, allocDecomposition());
        else
            std::cerr << "Unknown planner: " << name << std::endl;
    }
    else
    {
        if (name == "rrt")
            p = std::make_shared<ompl::geometric::RRT>(si);
        else if (name == "rrtconnect")
            p = std::make_shared<ompl::geometric::RRTConnect>(si);
        else if (name == "lazyrrt")
            p = std::make_shared<ompl::geometric::LazyRRT>(si);
        else if (name == "rrtstar")
            p = std::make_shared<ompl::geometric::RRTstar>(si);
        else if (name == "lbtrrt")
            p = std::make_shared<ompl::geometric::LBTRRT>(si);
        else if (name == "trrt")
            p = std::make_shared<ompl::geometric::TRRT>(si);
        else if (name == "est")
            p = std::make_shared<ompl::geometric::EST>(si);
        else if (name == "biest")
            p = std::make_shared<ompl::geometric::BiEST>(si);
        else if (name == "projest")
            p = std::make_shared<ompl::geometric::ProjEST>(si);
        else if (name == "sbl")
            p = std::make_shared<ompl::geometric::SBL>(si);
        else if (name == "kpiece")
            p = std::make_shared<ompl::geometric::KPIECE1>(si);
        else if (name == "bkpiece")
            p = std::make_shared<ompl::geometric::BKPIECE1>(si);
        else if (name == "lbkpiece")
            p = std::make_shared<ompl::geometric::LBKPIECE1>(si);
        else if (name == "prm")
            p = std::make_shared<ompl::geometric::PRM>(si);
        else if (name == "lazyprm")
            p = std::make_shared<ompl::geometric::LazyPRM>(si);
        else if (name == "prmstar")
            p = std::make_shared<ompl::geometric::PRMstar>(si);
        else if (name == "lazyprmstar")
            p = std::make_shared<ompl::geometric::LazyPRMstar>(si);
        else if (name == "spars")
            p = std::make_shared<ompl::geometric::SPARS>(si);
        else if (name == "spars2")
            p = std::make_shared<ompl::geometric::SPARStwo>(si);
        else if (name == "stride")
            p = std::make_shared<ompl::geometric::STRIDE>(si);
        else if (name == "pdst")
            p = std::make_shared<ompl::geometric::PDST>(si);
        else if (name == "fmt")
            p = std::make_shared<ompl::geometric::FMT>(si);
		else if (name == "bfmt")
            p = std::make_shared<ompl::geometric::BFMT>(si);
        else if (name == "aps")
            p = std::make_shared<ompl::geometric::AnytimePathShortening>(si);
        else if (name == "cforest")
            p = std::make_shared<ompl::geometric::CForest>(si);
        else
            std::cerr << "Unknown planner: " << name << std::endl;
    }

    if (p)
    {
        pcontext_[p.get()] = opt.c;
        auto iter = opt.p.find("name");
        if (iter != opt.p.end())
        {
            p->setName(iter->second);
            BenchmarkOptions::PlannerOpt temp = opt.p;
            temp.erase(iter->first);
            p->params().setParams(temp);
        }
        else
            p->params().setParams(opt.p);
        std::cout << "Allocated " << p->getName() << std::endl;
    }
    return p;
}

ompl::base::ValidStateSamplerPtr CFGBenchmark::allocValidStateSampler(const ompl::base::SpaceInformation *si, const std::string &type)
{
    ompl::base::ValidStateSamplerPtr vss;
    if (type == "uniform")
        vss = std::make_shared<ompl::base::UniformValidStateSampler>(si);
    else if (type == "gaussian")
        vss = std::make_shared<ompl::base::GaussianValidStateSampler>(si);
    else if (type == "obstacle_based")
        vss = std::make_shared<ompl::base::ObstacleBasedValidStateSampler>(si);
    else if (type == "max_clearance")
        vss = std::make_shared<ompl::base::MaximizeClearanceValidStateSampler>(si);
    else if (type == "bridge_test")
        vss = std::make_shared<ompl::base::BridgeTestValidStateSampler>(si);
    else
        std::cerr << "Unknown sampler type: " << type << std::endl;
    if (vss)
        vss->params().setParams(activeParams_, true);
    return vss;
}

ompl::base::OptimizationObjectivePtr CFGBenchmark::getOptimizationObjective(const ompl::base::SpaceInformationPtr &si)
{
    ompl::base::OptimizationObjectivePtr opt;
    if (bo_.declared_options_.find("problem.objective") == bo_.declared_options_.end())
    {
        return opt;
    }
    std::string objective = bo_.declared_options_["problem.objective"];

    if (objective.substr(0,6) == std::string("length"))
        opt = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
    else if (objective.substr(0,17) == std::string("max_min_clearance"))
        opt = std::make_shared<ompl::base::MaximizeMinClearanceObjective>(si);
    else if (objective.substr(0,15) == std::string("mechanical_work"))
        opt = std::make_shared<ompl::base::MechanicalWorkOptimizationObjective>(si);

    if (opt && bo_.declared_options_.find("problem.objective.threshold") != bo_.declared_options_.end())
    {
        std::string threshold = bo_.declared_options_["problem.objective.threshold"];
        try
        {
            opt->setCostThreshold(ompl::base::Cost(std::stod(threshold)));
        }
        catch(std::invalid_argument &)
        {
            OMPL_WARN("Unable to parse optimization threshold: %s", threshold.c_str());
        }
    }
    if (opt)
        defaultCostThreshold_ = opt->getCostThreshold();

    return opt;
}

void CFGBenchmark::setupBenchmark()
{
    for (auto & planner : bo_.planners_)
        for (auto & option : planner.second)
            benchmark_->addPlannerAllocator(
                [this, planner, option](const ompl::base::SpaceInformationPtr &si)
                {
                    return allocPlanner(si, planner.first, option);
                });
    benchmark_->setPlannerSwitchEvent(
        [this](const ompl::base::PlannerPtr planner)
        {
            preSwitchEvent(planner);
        });
    if (bo_.declared_options_.find("benchmark.save_paths") != bo_.declared_options_.end())
    {
        std::string savePathArg = bo_.declared_options_["benchmark.save_paths"];
        if (savePathArg.substr(0,3) == std::string("all")) // starts with "all"
            benchmark_->setPostRunEvent(
                [this](const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &properties)
                {
                    saveAllPaths(planner, properties);
                });
        else if (savePathArg.substr(0,8) == std::string("shortest")
            || savePathArg.substr(0,4) == std::string("best")) // starts with "shortest" or "best"
            benchmark_->setPostRunEvent(
                [this](const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &properties)
                {
                    saveBestPath(planner, properties);
                });
    }
}

void CFGBenchmark::preSwitchEvent(const ompl::base::PlannerPtr &planner)
{
    activeParams_ = pcontext_[planner.get()];
    if (activeParams_.find("sampler") != activeParams_.end())
        planner->getSpaceInformation()->setValidStateSamplerAllocator(
            [this](const ompl::base::SpaceInformation *si)
            {
                return allocValidStateSampler(si, activeParams_["sampler"]);
            });
    else
        planner->getSpaceInformation()->clearValidStateSamplerAllocator();
    planner->getSpaceInformation()->params().setParams(activeParams_, true);

    ompl::base::OptimizationObjectivePtr opt = planner->getProblemDefinition()->getOptimizationObjective();
    if (opt)
        opt->setCostThreshold(defaultCostThreshold_);
    if (opt && activeParams_.find("objective.threshold") != activeParams_.end())
    {
        std::string threshold = activeParams_["objective.threshold"];
        try
        {
            opt->setCostThreshold(ompl::base::Cost(std::stod(threshold)));
        }
        catch(std::invalid_argument &)
        {
            OMPL_WARN("Unable to parse optimization threshold: %s", threshold.c_str());
        }
    }

    bestPath_.reset();
    bestPathIndex_ = 0;
}

void CFGBenchmark::saveAllPaths(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties& /*run*/)
{
    ompl::base::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
    if (pdef->hasSolution())
    {
        const ompl::tools::Benchmark::Status& status = benchmark_->getStatus();
        std::string fname = benchmark_->getExperimentName() + std::string("_")
            + status.activePlanner + std::string("_") + std::to_string(status.activeRun)
            + std::string(".path");
        std::ofstream pathfile(fname.c_str());
        ompl::base::PathPtr path = pdef->getSolutionPath();
        auto* geoPath = dynamic_cast<ompl::geometric::PathGeometric*>(path.get());
        if (geoPath != nullptr)
        {
            geoPath->interpolate();
            geoPath->printAsMatrix(pathfile);
        }
        else {
            auto* controlPath = dynamic_cast<ompl::control::PathControl*>(path.get());
            if (controlPath != nullptr)
            {
                controlPath->interpolate();
                controlPath->printAsMatrix(pathfile);
            }
            else
                pdef->getSolutionPath()->print(pathfile);
        }
    }
}
void CFGBenchmark::saveBestPath(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties& /*run*/)
{
    ompl::base::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
    const ompl::tools::Benchmark::Status& status = benchmark_->getStatus();
    if (pdef->hasSolution() && !pdef->hasApproximateSolution())
    {
        const ompl::base::PathPtr &path = pdef->getSolutionPath();
        const ompl::base::OptimizationObjectivePtr &opt = pdef->getOptimizationObjective();
        ompl::base::Cost cost = path->cost(opt);
        if (!bestPath_ || opt->isCostBetterThan(cost, bestPath_->cost(opt)))
        {
            bestPath_ = path;
            bestPathIndex_ = status.activeRun;
        }
    }
    if (status.activeRun == benchmark_->getRecordedExperimentData().runCount - 1 && bestPath_)
    {
        std::string fname = benchmark_->getExperimentName() + std::string("_")
                          + status.activePlanner + std::string("_") + std::to_string(bestPathIndex_)
                          + std::string(".path");
        std::ofstream pathfile(fname.c_str());

        auto* geoPath = dynamic_cast<ompl::geometric::PathGeometric*>(bestPath_.get());
        if (geoPath != nullptr)
        {
            geoPath->interpolate();
            geoPath->printAsMatrix(pathfile);
        }
        else {
            auto* controlPath = dynamic_cast<ompl::control::PathControl*>(bestPath_.get());
            if (controlPath != nullptr)
            {
                controlPath->interpolate();
                controlPath->printAsMatrix(pathfile);
            }
            else
                bestPath_->print(pathfile);
        }
    }
}

void CFGBenchmark::setup()
{
    configure();
    if (benchmark_)
        setupBenchmark();
}

void CFGBenchmark::runBenchmark()
{
    if (!isValid())
        return;

    double tl = 0.0;
    double ml = 0.0;
    unsigned int rc = 0;

    try
    {
        tl = std::stod(bo_.declared_options_["benchmark.time_limit"]);
        ml = std::stod(bo_.declared_options_["benchmark.mem_limit"]);
        rc = std::stoul(bo_.declared_options_["benchmark.run_count"]);
    }
    catch(std::invalid_argument &)
    {
        std::cerr << "Unable to parse benchmark parameters" << std::endl;
        return;
    }

    ompl::tools::Benchmark::Request req;
    req.maxTime = tl;
    req.maxMem = ml;
    req.runCount = rc;
    req.timeBetweenUpdates = .5;
    req.displayProgress = true;
    req.saveConsoleOutput = false;
    req.useThreads = true;
    benchmark_->benchmark(req);
    if (!bo_.declared_options_["benchmark.output"].empty())
        benchmark_->saveResultsToFile(((bo_.path_ / bo_.declared_options_["benchmark.output"]) / bo_.outfile_).string().c_str());
    else
            benchmark_->saveResultsToFile((bo_.path_ / bo_.outfile_).string().c_str());
}
