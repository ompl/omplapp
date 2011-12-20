/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
using namespace ompl;

#include <boost/lexical_cast.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <fstream>

static const std::string KNOWN_PLANNERS[] = {
    "rrtconnect", "lazyrrt",
    "kpiece", "bkpiece", "lbkpiece",
    "est", "sbl", "prm", "rrt"
};

/*
base::ValidStateSamplerPtr allocUniformStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::UniformValidStateSampler(si));
}

base::ValidStateSamplerPtr allocGaussianStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::GaussianValidStateSampler(si));
}

base::ValidStateSamplerPtr allocObstacleStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::ObstacleBasedValidStateSampler(si));
}

base::ValidStateSamplerPtr allocMaximizeClearanceStateSampler(const base::SpaceInformation *si)
{
    base::MaximizeClearanceValidStateSampler *s = new base::MaximizeClearanceValidStateSampler(si);
    s->setNrImproveAttempts(5);
    return base::ValidStateSamplerPtr(s);
}

*/
class GeometricPlanningBenchmark
{
public:
    GeometricPlanningBenchmark(const char *filename)
    {
        if (readOptions(filename))
        {
            boost::filesystem::path path(filename);
#if BOOST_VERSION < 104600
            path_ = boost::filesystem::complete(path);
#else
            path_ = boost::filesystem::absolute(path);
#endif
	    outfile_ = path_.filename();
            path_.remove_filename();
	    outfile_.replace_extension(".log");
	    
            if (isSE2Problem())
                configureSE2();
            else
                if (isSE3Problem())
                    configureSE3();
                else
                    std::cerr << "Unknown problem specification" << std::endl;
            if (benchmark_)
                setupBenchmark();
        }
    }

    bool isValid(void) const
    {
        return benchmark_;
    }

    bool isSE2Problem(void) const
    {
        return opt_.find("problem.start.x") != opt_.end() &&  opt_.find("problem.start.y") != opt_.end() && opt_.find("problem.start.theta") != opt_.end() &&
            opt_.find("problem.goal.x") != opt_.end() &&  opt_.find("problem.goal.y") != opt_.end() && opt_.find("problem.goal.theta") != opt_.end() &&
            opt_.find("problem.start.z") == opt_.end() && opt_.find("problem.start.axis.x") == opt_.end() && opt_.find("problem.start.axis.y") == opt_.end() &&
            opt_.find("problem.start.axis.z") == opt_.end() && opt_.find("problem.goal.z") == opt_.end() && opt_.find("problem.goal.axis.x") == opt_.end() &&
            opt_.find("problem.goal.axis.y") == opt_.end() && opt_.find("problem.goal.axis.z") == opt_.end();
    }

    bool isSE3Problem(void) const
    {
        return opt_.find("problem.start.x") != opt_.end() &&  opt_.find("problem.start.y") != opt_.end() && opt_.find("problem.start.theta") != opt_.end() &&
            opt_.find("problem.goal.x") != opt_.end() &&  opt_.find("problem.goal.y") != opt_.end() && opt_.find("problem.goal.theta") != opt_.end() &&
            opt_.find("problem.start.z") != opt_.end() && opt_.find("problem.start.axis.x") != opt_.end() && opt_.find("problem.start.axis.y") != opt_.end() &&
            opt_.find("problem.start.axis.z") != opt_.end() && opt_.find("problem.goal.z") != opt_.end() && opt_.find("problem.goal.axis.x") != opt_.end() &&
            opt_.find("problem.goal.axis.y") != opt_.end() && opt_.find("problem.goal.axis.z") != opt_.end();
    }

    void benchmark(void)
    {
        if (!isValid())
            return;
        double tl = 0.0;
        double ml = 0.0;
        unsigned int rc = 0;

        try
        {
            tl = boost::lexical_cast<double>(opt_["benchmark.time_limit"]);
            ml = boost::lexical_cast<double>(opt_["benchmark.mem_limit"]);
            rc = boost::lexical_cast<unsigned int>(opt_["benchmark.run_count"]);
        }
        catch(boost::bad_lexical_cast &)
        {
            std::cerr << "Unable to parse benchmark parameters" << std::endl;
            return;
        }

        benchmark_->benchmark(tl, ml, rc, true, true);
        if (!opt_["benchmark.output"].empty())
            benchmark_->saveResultsToFile(((path_ / opt_["benchmark.output"]) / outfile_).string().c_str ());
        else
            benchmark_->saveResultsToFile((path_ / outfile_).string().c_str());
    }

private:

    typedef std::map<std::string, std::string> PlannerOpt;
    typedef std::map<std::string, std::string> ContextOpt;
    struct Options
    {
        ContextOpt c;
        PlannerOpt p;
    };

    boost::filesystem::path                      path_;
    boost::filesystem::path                      outfile_;
    std::map<std::string, std::string>           opt_;
    ContextOpt                                   context_;
    std::map<std::string, std::vector<Options> > planners_;
    std::map<base::Planner*, ContextOpt>         pcontext_;
    boost::shared_ptr<app::SE3RigidBodyPlanning> setup_se3_;
    boost::shared_ptr<app::SE2RigidBodyPlanning> setup_se2_;
    boost::shared_ptr<Benchmark>                 benchmark_;

    bool readOptions(const char *filename)
    {
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
            ("problem.threshold", boost::program_options::value<std::string>(), "Threshold to reach goal position")

            ("benchmark.time_limit", boost::program_options::value<std::string>(), "Time limit for each run of a planner")
            ("benchmark.mem_limit", boost::program_options::value<std::string>(), "Memory limit for each run of a planner")
            ("benchmark.run_count", boost::program_options::value<std::string>(), "Number of times to run each planner")
            ("benchmark.output", boost::program_options::value<std::string>(), "Location where to save the results");

        boost::program_options::variables_map vm;
        boost::program_options::parsed_options po = boost::program_options::parse_config_file(cfg, desc, true);
        cfg.close();
        boost::program_options::store(po, vm);
        opt_.clear();
        for (boost::program_options::variables_map::iterator it = vm.begin() ; it != vm.end() ; ++it)
            opt_[it->first] = boost::any_cast<std::string>(vm[it->first].value());

        std::vector<std::string> unr = boost::program_options::collect_unrecognized(po.options, boost::program_options::exclude_positional);
        planners_.clear();
        context_.clear();
        ContextOpt temp_context;
        std::string last_planner;
        for (std::size_t i = 0 ; i < unr.size() / 2 ; ++i)
        {
            std::string key = boost::to_lower_copy(unr[i * 2]);
            std::string val = unr[i * 2 + 1];
            if (key.substr(0, 8) == "problem.")
            {
                std::string p = key.substr(8);
                context_[p] = val;
                continue;
            }

            if (key.substr(0, 8) != "planner.")
                continue;

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
                        if (op[KNOWN_PLANNERS[i].length()] == '.')
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
                    break;
                }
        }
        return true;
    }

    void configureSE3(void)
    {
        setup_se3_.reset(new app::SE3RigidBodyPlanning());
        setup_se3_->setRobotMesh((path_ / opt_["problem.robot"]).string());
        setup_se3_->setEnvironmentMesh((path_ / opt_["problem.world"]).string());
        base::ScopedState<base::SE3StateSpace> start(setup_se3_->getStateSpace());
        try
        {
            start->setXYZ(boost::lexical_cast<double>(opt_["problem.start.x"]),
                          boost::lexical_cast<double>(opt_["problem.start.y"]),
                          boost::lexical_cast<double>(opt_["problem.start.z"]));
            start->rotation().setAxisAngle(boost::lexical_cast<double>(opt_["problem.start.axis.x"]),
                                           boost::lexical_cast<double>(opt_["problem.start.axis.y"]),
                                           boost::lexical_cast<double>(opt_["problem.start.axis.z"]),
                                           boost::lexical_cast<double>(opt_["problem.start.theta"]));
        }
        catch(boost::bad_lexical_cast &)
        {
            std::cerr << "Unable to read start state" << std::endl;
            return;
        }
        base::ScopedState<base::SE3StateSpace> goal(setup_se3_->getStateSpace());
        try
        {
            goal->setXYZ(boost::lexical_cast<double>(opt_["problem.goal.x"]),
                         boost::lexical_cast<double>(opt_["problem.goal.y"]),
                         boost::lexical_cast<double>(opt_["problem.goal.z"]));
            goal->rotation().setAxisAngle(boost::lexical_cast<double>(opt_["problem.goal.axis.x"]),
                                          boost::lexical_cast<double>(opt_["problem.goal.axis.y"]),
                                          boost::lexical_cast<double>(opt_["problem.goal.axis.z"]),
                                          boost::lexical_cast<double>(opt_["problem.goal.theta"]));
        }
        catch(boost::bad_lexical_cast &)
        {
            std::cerr << "Unable to read goal state" << std::endl;
            return;
        }
        try
        {
            double t = boost::lexical_cast<double>(opt_["problem.threshold"]);
            setup_se3_->setStartAndGoalStates(start, goal, t);
        }
        catch(boost::bad_lexical_cast &)
        {
            setup_se3_->setStartAndGoalStates(start, goal);
        }
        setup_se3_->setup();
        setup_se3_->params().setParams(context_);
        setup_se3_->setup();
        setup_se3_->print();
        benchmark_.reset(new Benchmark(*setup_se3_, opt_["problem.name"]));
    }

    void configureSE2(void)
    {
        setup_se2_.reset(new app::SE2RigidBodyPlanning());
        setup_se2_->setRobotMesh((path_ / opt_["problem.robot"]).string());
        setup_se2_->setEnvironmentMesh((path_ / opt_["problem.world"]).string());
        base::ScopedState<base::SE2StateSpace> start(setup_se2_->getStateSpace());
        try
        {
            start->setX(boost::lexical_cast<double>(opt_["problem.start.x"]));
            start->setY(boost::lexical_cast<double>(opt_["problem.start.y"]));
            start->setYaw(boost::lexical_cast<double>(opt_["problem.start.theta"]));
        }
        catch(boost::bad_lexical_cast &)
        {
            std::cerr << "Unable to read start state" << std::endl;
            return;
        }
        base::ScopedState<base::SE2StateSpace> goal(setup_se2_->getStateSpace());
        try
        {
            goal->setX(boost::lexical_cast<double>(opt_["problem.goal.x"]));
            goal->setY(boost::lexical_cast<double>(opt_["problem.goal.y"]));
            goal->setYaw(boost::lexical_cast<double>(opt_["problem.goal.theta"]));
        }
        catch(boost::bad_lexical_cast &)
        {
            std::cerr << "Unable to read goal state" << std::endl;
            return;
        }
        try
        {
            double t = boost::lexical_cast<double>(opt_["problem.threshold"]);
            setup_se2_->setStartAndGoalStates(start, goal, t);
        }
        catch(boost::bad_lexical_cast &)
        {
            setup_se2_->setStartAndGoalStates(start, goal);
        }
        setup_se2_->setup();
        setup_se2_->params().setParams(context_);
        setup_se2_->setup();
        setup_se2_->print();
        benchmark_.reset(new Benchmark(*setup_se2_, opt_["problem.name"]));
    }

    base::PlannerPtr allocPlanner(const base::SpaceInformationPtr &si, const std::string &name, const Options &opt)
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
        else
            std::cerr << "Unknown planner: " << name << std::endl;
        if (p)
        {
            p->params().setParams(opt.p);
            pcontext_[p] = opt.c;
            std::cout << "Allocated " << p->getName() << std::endl;
        }
        return base::PlannerPtr(p);
    }

    void setupBenchmark(void)
    {
        for (std::map<std::string, std::vector<Options> >::iterator it = planners_.begin() ; it != planners_.end() ; ++it)
            for (std::size_t i = 0 ; i < it->second.size() ; ++i)
                benchmark_->addPlannerAllocator(boost::bind(&GeometricPlanningBenchmark::allocPlanner, this, _1,
                                                            boost::cref(it->first), boost::cref(it->second[i])));
        benchmark_->setPlannerSwitchEvent(boost::bind(&GeometricPlanningBenchmark::preSwitchEvent, this, _1));
        //        benchmark_->setPostRunEvent(boost::bind(&GeometricPlanningBenchmark::postRunEvent, this, _1, _2));
    }

    void preSwitchEvent(const base::PlannerPtr &planner)
    {
        planner->getSpaceInformation()->params().setParams(pcontext_[planner.get()]);
    }

};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage:\n\t " << argv[0] << " problem.cfg" << std::endl;
        return 1;
    }

    GeometricPlanningBenchmark gpb(argv[1]);
    gpb.benchmark();

    return 0;
}
