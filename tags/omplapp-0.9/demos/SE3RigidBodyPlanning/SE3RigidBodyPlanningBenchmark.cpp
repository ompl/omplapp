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
#include <omplapp/SE3RigidBodyPlanning.h>
#include <ompl/benchmark/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/BasicPRM.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
using namespace ompl;

void benchmark0(std::string& benchmark_name, app::SE3RigidBodyPlanning& setup,
    double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = std::string("cubicles");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/cubicles_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateManifold> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(70.57);
    start->setZ(40.62);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateManifold> goal(start);
    goal->setX(200.49);
    goal->setY(70.57);
    goal->setZ(40.62);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setup.getSpaceInformation()->setValidStateSamplerAllocator(base::UniformValidStateSampler::allocator());

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 500;
}

void benchmark1(std::string& benchmark_name, app::SE3RigidBodyPlanning& setup,
    double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = std::string("Twistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateManifold> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateManifold> goal(start);
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
    setup.getStateManifold()->as<base::SE3StateManifold>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 90.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 50;
}

int main(int argc, char **argv)
{
    app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;
    int benchmark_id = argc==1 ? 0 : (atoi(argv[1]) % 2);

    if (benchmark_id==0)
        benchmark0(benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else
        benchmark1(benchmark_name, setup, runtime_limit, memory_limit, run_count);

    // create the benchmark object and add all the planners we'd like to run
    Benchmark b(setup, benchmark_name);
    b.addPlanner(base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    //    b.addPlanner(base::PlannerPtr(new geometric::LazyRRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::LBKPIECE1(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::KPIECE1(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::SBL(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::BasicPRM(setup.getSpaceInformation())));

    // run all planners with a uniform valid state sampler on the benchmark problem
    setup.getSpaceInformation()->setValidStateSamplerAllocator(base::UniformValidStateSampler::allocator());
    b.setExperimentName(benchmark_name + "_uniform_sampler");
    b.benchmark(runtime_limit, memory_limit, run_count, true);
    b.saveResultsToFile();


    // run all planners with a Gaussian valid state sampler on the benchmark problem
    setup.getSpaceInformation()->setValidStateSamplerAllocator(base::GaussianValidStateSampler::allocator());
    b.setExperimentName(benchmark_name + "_gaussian_sampler");
    b.benchmark(runtime_limit, memory_limit, run_count, true);
    b.saveResultsToFile();


    // run all planners with a obstacle-based valid state sampler on the benchmark problem
    setup.getSpaceInformation()->setValidStateSamplerAllocator(base::ObstacleBasedValidStateSampler::allocator());
    b.setExperimentName(benchmark_name + "_obstaclebased_sampler");
    b.benchmark(runtime_limit, memory_limit, run_count, true);
    b.saveResultsToFile();

    return 0;
}
