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

#include "BenchmarkOptions.h"

#include <boost/lexical_cast.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <fstream>

bool BenchmarkOptions::readOptions(const char *filename)
{
    static const std::string KNOWN_PLANNERS[] = {
        "rrtconnect", "lazyrrt", "lazyprm",
        "kpiece", "bkpiece", "lbkpiece",
        "est", "sbl", "prm", "rrt",
        "stride", "pdst",
        "rrtstar", "prmstar", "lazyprmstar",
        "spars", "spars2", "lbtrrt", "trrt",
        "fmt",
        "syclopest", "sycloprrt",
        "aps", "cforest"
    };

    std::ifstream cfg(filename);
    if (!cfg.good())
    {
        std::cerr << "Unable to open file '" << filename << "'" << std::endl;
        return false;
    }

    boost::program_options::options_description desc;
    desc.add_options()
    ("problem.name", boost::program_options::value<std::string>(), "Experiment name")
    ("problem.world", boost::program_options::value<std::string>(), "CAD file describing the environment")
    ("problem.robot", boost::program_options::value<std::string>(), "CAD file describing the robot")
    ("problem.objective", boost::program_options::value<std::string>(), "Optimization objective")
    ("problem.objective.threshold", boost::program_options::value<std::string>(), "Threshold to achieve optimization objective")
    ("problem.control", boost::program_options::value<std::string>(), "Type of control-based system")
    ("problem.start.x", boost::program_options::value<std::string>(), "Start position: x value")
    ("problem.start.y", boost::program_options::value<std::string>(), "Start position: y value")
    ("problem.start.z", boost::program_options::value<std::string>(), "Start position: z value")
    ("problem.start.axis.x", boost::program_options::value<std::string>(), "Start position: rotation axis x value")
    ("problem.start.axis.y", boost::program_options::value<std::string>(), "Start position: rotation axis y value")
    ("problem.start.axis.z", boost::program_options::value<std::string>(), "Start position: rotation axis z value")
    ("problem.start.theta", boost::program_options::value<std::string>(), "Start position: theta value")
    ("problem.goal.x", boost::program_options::value<std::string>(), "Goal position: x value")
    ("problem.goal.y", boost::program_options::value<std::string>(), "Goal position: y value")
    ("problem.goal.z", boost::program_options::value<std::string>(), "Goal position: z value")
    ("problem.goal.axis.x", boost::program_options::value<std::string>(), "Goal position: rotation axis x value")
    ("problem.goal.axis.y", boost::program_options::value<std::string>(), "Goal position: rotation axis y value")
    ("problem.goal.axis.z", boost::program_options::value<std::string>(), "Goal position: rotation axis z value")
    ("problem.goal.theta", boost::program_options::value<std::string>(), "Goal position: theta value")
    ("problem.threshold", boost::program_options::value<std::string>()->default_value("1e-6"), "Threshold to reach goal position")
    ("problem.solution_length", boost::program_options::value<std::string>(), "Maximum desired solution length")
    ("problem.volume.min.x", boost::program_options::value<std::string>(), "Min X for bounding volume")
    ("problem.volume.min.y", boost::program_options::value<std::string>(), "Min Y for bounding volume")
    ("problem.volume.min.z", boost::program_options::value<std::string>(), "Min Z for bounding volume")
    ("problem.volume.max.x", boost::program_options::value<std::string>(), "Max X for bounding volume")
    ("problem.volume.max.y", boost::program_options::value<std::string>(), "Max Y for bounding volume")
    ("problem.volume.max.z", boost::program_options::value<std::string>(), "Max Z for bounding volume")

    ("benchmark.time_limit", boost::program_options::value<std::string>(), "Time limit for each run of a planner")
    ("benchmark.mem_limit", boost::program_options::value<std::string>(), "Memory limit for each run of a planner")
    ("benchmark.run_count", boost::program_options::value<std::string>(), "Number of times to run each planner")
    ("benchmark.output", boost::program_options::value<std::string>(), "Location where to save the results")
    ("benchmark.save_paths", boost::program_options::value<std::string>(), "Save none (default), all paths, shortest path per planner");

    boost::program_options::variables_map vm;
    boost::program_options::parsed_options po = boost::program_options::parse_config_file(cfg, desc, true);
    cfg.close();
    boost::program_options::store(po, vm);
    declared_options_.clear();
    for (boost::program_options::variables_map::iterator it = vm.begin() ; it != vm.end() ; ++it)
        declared_options_[it->first] = boost::any_cast<std::string>(vm[it->first].value());

    std::vector<std::string> unr = boost::program_options::collect_unrecognized(po.options, boost::program_options::exclude_positional);
    planners_.clear();

    // gather context params for all the planners (specified as part of the problem)
    ContextOpt problem_context;

    // gather context params for a particular planner; this is used as a buffer, only if the params are specified before the planner
    ContextOpt temp_context;

    // last planner that was initialized (to be able to specify context params)
    std::string last_planner;

    for (std::size_t i = 0 ; i < unr.size() / 2 ; ++i)
    {
        std::string key = boost::to_lower_copy(unr[i * 2]);
        std::string val = unr[i * 2 + 1];

        // read problem context params
        if (key.substr(0, 8) == "problem.")
        {
            std::string p = key.substr(8);
            problem_context[p] = val;
            continue;
        }

        // the only params that follow must be planner specific
        if (key.substr(0, 8) != "planner.")
            continue;

        // if they are context params specific for that planner, use them as such
        std::string op = key.substr(8);
        if (op.substr(0, 8) == "problem.")
        {
            std::string p = op.substr(8);
            if (last_planner.empty())
                temp_context[p] = val;
            else
                planners_[last_planner].back().c[p] = val;
            continue;
        }

        // read planner specific params
        for (std::size_t i = 0 ; i < sizeof(KNOWN_PLANNERS) / sizeof(std::string) ; ++i)
            if (op.substr(0, KNOWN_PLANNERS[i].length()) == KNOWN_PLANNERS[i])
            {
                if (op == KNOWN_PLANNERS[i])
                {
                    planners_[op].resize(planners_[op].size() + 1);
                    planners_[op].back().c = temp_context;
                    last_planner = op;
                    temp_context.clear();
                }
                else
                    if (op.length() > KNOWN_PLANNERS[i].length() && op[KNOWN_PLANNERS[i].length()] == '.')
                    {
                        last_planner = KNOWN_PLANNERS[i];
                        op = op.substr(KNOWN_PLANNERS[i].length() + 1);
                        if (planners_[KNOWN_PLANNERS[i]].empty())
                            planners_[KNOWN_PLANNERS[i]].resize(1);
                        planners_[KNOWN_PLANNERS[i]].back().p[op] = val;
                        if (!temp_context.empty())
                        {
                            planners_[KNOWN_PLANNERS[i]].back().c = temp_context;
                            temp_context.clear();
                        }
                    }
            }

        // now we merge the global (problem context) in all the planner specific contexts
        if (!problem_context.empty())
            for (std::map<std::string, std::vector<AllOptions> >::iterator it = planners_.begin() ; it != planners_.end() ; ++it)
                for (std::size_t i = 0 ; i < it->second.size() ; ++i)
                {
                    ContextOpt &planner_options = it->second[i].c;
                    ContextOpt backup = planner_options;
                    planner_options = problem_context;
                    for (std::map<std::string, std::string>::iterator jt = backup.begin() ; jt != backup.end() ; ++jt)
                        planner_options[jt->first] = jt->second;
                }
    }

    boost::filesystem::path path(filename);
#if BOOST_VERSION < 104600
    path_ = boost::filesystem::complete(path);
#else
    path_ = boost::filesystem::absolute(path);
#endif
    outfile_ = path_.filename();
    path_.remove_filename();
    outfile_.replace_extension(".log");

    return true;
}

bool BenchmarkOptions::isSE2Problem(void) const
{
    return declared_options_.find("problem.start.x") != declared_options_.end() &&  declared_options_.find("problem.start.y") != declared_options_.end() &&
    declared_options_.find("problem.start.theta") != declared_options_.end() &&
    declared_options_.find("problem.goal.x") != declared_options_.end() &&  declared_options_.find("problem.goal.y") != declared_options_.end() &&
    declared_options_.find("problem.goal.theta") != declared_options_.end() &&
    declared_options_.find("problem.start.z") == declared_options_.end() && declared_options_.find("problem.start.axis.x") == declared_options_.end() &&
    declared_options_.find("problem.start.axis.y") == declared_options_.end() &&
    declared_options_.find("problem.start.axis.z") == declared_options_.end() && declared_options_.find("problem.goal.z") == declared_options_.end() &&
    declared_options_.find("problem.goal.axis.x") == declared_options_.end() &&
    declared_options_.find("problem.goal.axis.y") == declared_options_.end() && declared_options_.find("problem.goal.axis.z") == declared_options_.end();
}

bool BenchmarkOptions::isSE3Problem(void) const
{
    return declared_options_.find("problem.start.x") != declared_options_.end() &&  declared_options_.find("problem.start.y") != declared_options_.end() &&
    declared_options_.find("problem.start.theta") != declared_options_.end() &&
    declared_options_.find("problem.goal.x") != declared_options_.end() &&  declared_options_.find("problem.goal.y") != declared_options_.end() &&
    declared_options_.find("problem.goal.theta") != declared_options_.end() &&
    declared_options_.find("problem.start.z") != declared_options_.end() && declared_options_.find("problem.start.axis.x") != declared_options_.end() &&
    declared_options_.find("problem.start.axis.y") != declared_options_.end() &&
    declared_options_.find("problem.start.axis.z") != declared_options_.end() && declared_options_.find("problem.goal.z") != declared_options_.end() &&
    declared_options_.find("problem.goal.axis.x") != declared_options_.end() &&
    declared_options_.find("problem.goal.axis.y") != declared_options_.end() && declared_options_.find("problem.goal.axis.z") != declared_options_.end();
}
