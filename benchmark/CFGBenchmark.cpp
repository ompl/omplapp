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

#include "CFGBenchmark.h"

#include <omplapp/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/gnat/GNAT.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>

ompl::base::PlannerPtr CFGBenchmark::allocPlanner(const ompl::base::SpaceInformationPtr &si, const std::string &name, const BenchmarkOptions::AllOptions &opt)
{
    ompl::base::Planner *p = NULL;
    if (name == "rrt")
	p = new ompl::geometric::RRT(si);
    else if (name == "rrtconnect")
	p = new ompl::geometric::RRTConnect(si);
    else if (name == "lazyrrt")
	p = new ompl::geometric::LazyRRT(si);
    else if (name == "est")
	p = new ompl::geometric::EST(si);
    else if (name == "sbl")
	p = new ompl::geometric::SBL(si);
    else if (name == "kpiece")
	p = new ompl::geometric::KPIECE1(si);
    else if (name == "bkpiece")
	p = new ompl::geometric::BKPIECE1(si);
    else if (name == "lbkpiece")
	p = new ompl::geometric::LBKPIECE1(si);
    else if (name == "prm")
	p = new ompl::geometric::PRM(si);
    else if (name == "gnat")
	p = new ompl::geometric::GNAT(si);
    else
	std::cerr << "Unknown planner: " << name << std::endl;
    if (p)
    {
	p->params().setParams(opt.p);
	pcontext_[p] = opt.c;
	std::cout << "Allocated " << p->getName() << std::endl;
    }
    return ompl::base::PlannerPtr(p);
}

ompl::base::ValidStateSamplerPtr CFGBenchmark::allocValidStateSampler(const ompl::base::SpaceInformation *si, const std::string &type)
{
    ompl::base::ValidStateSampler *vss = NULL;
    if (type == "uniform")
	vss = new ompl::base::UniformValidStateSampler(si);
    else if (type == "gaussian")
	vss = new ompl::base::GaussianValidStateSampler(si);
    else if (type == "obstacle_based")
	vss = new ompl::base::ObstacleBasedValidStateSampler(si);
    else if (type == "max_clearance")
	vss = new ompl::base::MaximizeClearanceValidStateSampler(si);
    else
	std::cerr << "Unknown sampler type: " << type << std::endl;
    if (vss)
    {
	vss->params().setParams(activeParams_, true);
	return ompl::base::ValidStateSamplerPtr(vss);
    }
    else
	return ompl::base::ValidStateSamplerPtr();
}

void CFGBenchmark::setupBenchmark(void)
{
    for (std::map<std::string, std::vector<BenchmarkOptions::AllOptions> >::iterator it = bo_.planners_.begin() ; it != bo_.planners_.end() ; ++it)
	for (std::size_t i = 0 ; i < it->second.size() ; ++i)
	    benchmark_->addPlannerAllocator(boost::bind(&CFGBenchmark::allocPlanner, this, _1,
							boost::cref(it->first), boost::cref(it->second[i])));
    benchmark_->setPlannerSwitchEvent(boost::bind(&CFGBenchmark::preSwitchEvent, this, _1));
}

void CFGBenchmark::preSwitchEvent(const ompl::base::PlannerPtr &planner)
{
    activeParams_ = pcontext_[planner.get()];
    if (activeParams_.find("sampler") != activeParams_.end())
	planner->getSpaceInformation()->setValidStateSamplerAllocator(boost::bind(&CFGBenchmark::allocValidStateSampler, this, _1,
										  boost::cref(activeParams_["sampler"])));
    else
	planner->getSpaceInformation()->clearValidStateSamplerAllocator();
    planner->getSpaceInformation()->params().setParams(activeParams_, true);
}

void CFGBenchmark::setup(void)
{
    configure();
    if (benchmark_)
	setupBenchmark();    
}

void CFGBenchmark::runBenchmark(void)
{
    if (!isValid())
	return;

    double tl = 0.0;
    double ml = 0.0;
    unsigned int rc = 0;
    
    try
    {
	tl = boost::lexical_cast<double>(bo_.declared_options_["benchmark.time_limit"]);
	ml = boost::lexical_cast<double>(bo_.declared_options_["benchmark.mem_limit"]);
	rc = boost::lexical_cast<unsigned int>(bo_.declared_options_["benchmark.run_count"]);
    }
    catch(boost::bad_lexical_cast &)
    {
	std::cerr << "Unable to parse benchmark parameters" << std::endl;
	return;
    }
    
    ompl::Benchmark::Request req;
    req.maxTime = tl;
    req.maxMem = ml;
    req.runCount = rc;
    req.displayProgress = true;
    req.saveConsoleOutput = false;
    req.useThreads = true;
    benchmark_->benchmark(req);
    if (!bo_.declared_options_["benchmark.output"].empty())
	benchmark_->saveResultsToFile(((bo_.path_ / bo_.declared_options_["benchmark.output"]) / bo_.outfile_).string().c_str());
    else
	benchmark_->saveResultsToFile((bo_.path_ / bo_.outfile_).string().c_str());
}
