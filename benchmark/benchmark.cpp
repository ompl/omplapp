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
#include <fstream>

static const std::string KNOWN_PLANNERS[] = {
    "rrt", "rrtconnect", "lazyrrt",
    "kpiece", "bkpiece", "lbkpiece",
    "est", "sbl", "prm"
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

void benchmark0(std::string& benchmark_name, app::SE3RigidBodyPlanning& setup,
                double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = std::string("cubicles");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(200.49);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
    setup.setup();

    std::vector<double> cs(3);
    cs[0] = 35; cs[1] = 35; cs[2] = 35;
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 500;
}

void benchmark1(std::string& benchmark_name, app::SE3RigidBodyPlanning& setup,
                double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = std::string("Twistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.);
    goal->setY(160.);
    goal->setZ(-400.);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0,400.);
    bounds.setHigh(1,275.);
    bounds.setHigh(2,-100.);
    bounds.setLow(0,60.);
    bounds.setLow(1,0.);
    bounds.setLow(2,-480.);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 90.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 50;
}

void preRunEvent(const base::PlannerPtr &planner)
{
}

void postRunEvent(const base::PlannerPtr &planner, Benchmark::RunProperties &run)
{
}
*/

class GeometricPlanningBenchmark
{
public:
    GeometricPlanningBenchmark(const char *filename)
    {
        if (readOptions(filename))
        {
            if (isSE2Problem())
                configureSE2();
            else
                if (isSE3Problem())
                    configureSE3();
                else
                    std::cerr << "Unknown problem specification" << std::endl;
            if (benchmark_)
                allocPlanners();
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
            benchmark_->saveResultsToFile(opt_["benchmark.output"].c_str());
        else
            benchmark_->saveResultsToFile();
    }

private:

    std::map<std::string, std::string>                         opt_;
    std::map<std::string, std::map<std::string, std::string> > planners_;
    boost::shared_ptr<app::SE3RigidBodyPlanning>               setup_se3_;
    boost::shared_ptr<app::SE2RigidBodyPlanning>               setup_se2_;
    boost::shared_ptr<Benchmark>                               benchmark_;

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
            ("benchmark.run_count", boost::program_options::value<std::string>(), "Number of times to run each planner");

        boost::program_options::variables_map vm;
        boost::program_options::parsed_options po = boost::program_options::parse_config_file(cfg, desc, true);
        cfg.close();
        boost::program_options::store(po, vm);
        std::vector<std::string> unr = boost::program_options::collect_unrecognized(po.options, boost::program_options::exclude_positional);
        std::map<std::string, std::string> rest;
        for (std::size_t i = 0 ; i < unr.size() / 2 ; ++i)
            rest[boost::to_lower_copy(unr[i * 2])] = unr[i * 2 + 1];

        opt_.clear();
        for (boost::program_options::variables_map::iterator it = vm.begin() ; it != vm.end() ; ++it)
            opt_[it->first] = boost::any_cast<std::string>(vm[it->first].value());

        planners_.clear();
        for (std::map<std::string, std::string>::iterator it = rest.begin() ; it != rest.end() ; ++it)
        {
            if (it->first.substr(0, 8) != "planner.")
                continue;
            std::string op = it->first.substr(8);
            for (std::size_t i = 0 ; i < sizeof(KNOWN_PLANNERS) / sizeof(std::string) ; ++i)
                if (op.substr(0, KNOWN_PLANNERS[i].length()) == KNOWN_PLANNERS[i])
                {
                    if (op == KNOWN_PLANNERS[i])
                        planners_[op];
                    else
                        if (op[KNOWN_PLANNERS[i].length()] == '.')
                        {
                            op = op.substr(KNOWN_PLANNERS[i].length() + 1);
                            planners_[KNOWN_PLANNERS[i]][op] = it->second;
                        }
                    break;
                }
        }
        return true;
    }

    void configureSE3(void)
    {
        setup_se3_.reset(new app::SE3RigidBodyPlanning());
        setup_se3_->setRobotMesh(opt_["problem.robot"].c_str());
        setup_se3_->setEnvironmentMesh(opt_["problem.world"].c_str());
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
        benchmark_.reset(new Benchmark(*setup_se3_, opt_["problem.name"]));
    }

    void configureSE2(void)
    {
        setup_se2_.reset(new app::SE2RigidBodyPlanning());
        setup_se2_->setRobotMesh(opt_["problem.robot"].c_str());
        setup_se2_->setEnvironmentMesh(opt_["problem.world"].c_str());
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
        benchmark_.reset(new Benchmark(*setup_se2_, opt_["problem.name"]));
    }

    base::PlannerPtr allocPlanner(const base::SpaceInformationPtr &si, const std::string &name, const std::map<std::string, std::string> &opt) const
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
            p->setParams(opt);
        return base::PlannerPtr(p);
    }

    void allocPlanners(void)
    {
        for (std::map<std::string, std::map<std::string, std::string> >::iterator it = planners_.begin() ; it != planners_.end() ; ++it)
            benchmark_->addPlannerAllocator(boost::bind(&GeometricPlanningBenchmark::allocPlanner, this, _1, boost::cref(it->first), boost::cref(it->second)));
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
