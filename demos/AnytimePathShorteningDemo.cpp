/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Ryan Luna */

#include "AnytimePathShorteningDemo.h"

#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/Console.h>
#include <omplapp/apps/SE2RigidBodyPlanning.h>
#include <omplapp/apps/SE3MultiRigidBodyPlanning.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

#include <cmath>

using namespace ompl;

std::shared_ptr<geometric::SimpleSetup> allocProblem(const ProblemType &probType)
{
    // Explicitly use the FCL collision checker.  PQP is not multi-threaded, and
    // multiple planners using the same SpaceInformation instance simultaneously
    // will serialize when checking if a state is in collision.

    switch (probType)
    {
        case BARRIERS:
            return std::make_shared<app::SE2RigidBodyPlanning>();
        case CUBICLES:
            return std::make_shared<app::SE3RigidBodyPlanning>();
        case EASY:
            return std::make_shared<app::SE3MultiRigidBodyPlanning>(2);
        default:
            OMPL_ERROR("Unknown problem type in allocProblem");
            return nullptr;
    }
}

void setStartAndGoalStates(const ProblemType &probType, std::shared_ptr<geometric::SimpleSetup> &setup)
{
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR);
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR);

    switch (probType)
    {
        case BARRIERS:
        {
            robot_fname += "/2D/Barriers_easy_robot.dae";
            env_fname += "/2D/Barriers_easy_env.dae";

            // define start state
            base::ScopedState<base::SE2StateSpace> start(setup->getSpaceInformation());
            start->setX(34.81);
            start->setY(-75.0);
            start->setYaw(0.0);

            // define goal state
            base::ScopedState<base::SE2StateSpace> goal(start);
            goal->setX(620.0);
            goal->setY(-375.0);
            goal->setYaw(3.14159);

            // set the start & goal states
            setup->setStartAndGoalStates(start, goal);
        }
        break;

        case CUBICLES:
        {
            robot_fname += "/3D/cubicles_robot.dae";
            env_fname += "/3D/cubicles_env.dae";

            // define start state
            base::ScopedState<base::SE3StateSpace> start(setup->getSpaceInformation());
            start->setX(-4.96);
            start->setY(-40.62);
            start->setZ(70.57);
            start->rotation().setIdentity();

            // define goal state
            base::ScopedState<base::SE3StateSpace> goal(start);
            goal->setX(200.49);
            goal->setY(-40.62);
            goal->setZ(70.57);
            goal->rotation().setIdentity();

            // set the start & goal states
            setup->setStartAndGoalStates(start, goal);
        }
        break;

        case EASY:
        {
            robot_fname += "/3D/Easy_robot.dae";
            env_fname += "/3D/Easy_env.dae";

            base::ScopedState<base::CompoundStateSpace> start(setup->getSpaceInformation());
            base::ScopedState<base::CompoundStateSpace> goal(start);

            // Robot 1
            start[0] = 270.0;
            start[1] = 100.0;
            start[2] = -150.0;
            start[3] = 1.0;
            start[4] = 0.0;
            start[5] = 0.0;
            start[6] = 0.0;

            // Robot 2
            start[7] = 270.0;
            start[8] = 100.0;
            start[9] = -425.0;
            start[10] = 1.0;
            start[11] = 0.0;
            start[12] = 0.0;
            start[13] = 0.0;

            // Robot 1
            goal[0] = 270.0;
            goal[1] = 100.0;
            goal[2] = -425.0;
            goal[3] = 1.0;
            goal[4] = 0.0;
            goal[5] = 0.0;
            goal[6] = 0.0;

            goal[7] = 270.0;
            goal[8] = 100.0;
            goal[9] = -150.0;
            goal[10] = 1.0;
            goal[11] = 0.0;
            goal[12] = 0.0;
            goal[13] = 0.0;

            // set the start & goal states
            setup->setStartAndGoalStates(start, goal);
        }
        break;

        default:
            OMPL_ERROR("FATAL: Unknown problem type in setStartAndGoalStates");
            break;
    }

    dynamic_cast<app::RigidBodyGeometry *>(setup.get())->setRobotMesh(robot_fname);
    dynamic_cast<app::RigidBodyGeometry *>(setup.get())->setEnvironmentMesh(env_fname);
}

void solve(const ProblemType &probType, const OptimizationType &optType, const double &runtime,
           const std::vector<PlannerType> &planners)
{
    auto setup(allocProblem(probType));

    setStartAndGoalStates(probType, setup);

    // setting collision checking resolution to 1% of the space extent
    setup->getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    base::PlannerPtr planner;
    switch (optType)
    {
        case SHORTCUT:
            planner = std::make_shared<geometric::AnytimePathShortening>(setup->getSpaceInformation());
            planner->as<geometric::AnytimePathShortening>()->setHybridize(false);
            break;

        case HYBRIDIZE:
            planner = std::make_shared<geometric::AnytimePathShortening>(setup->getSpaceInformation());
            planner->as<geometric::AnytimePathShortening>()->setShortcut(false);
            break;

        case ALTERNATE:
            planner = std::make_shared<geometric::AnytimePathShortening>(setup->getSpaceInformation());
            break;

        case NONE:
            planner = allocPlanner(planners[0], setup->getSpaceInformation());
            break;

        default:
            OMPL_ERROR("Unhandled optimization case");
            return;
    }

    if (optType != NONE)
    {
        // Adding planners
        for (auto i : planners)
        {
            base::PlannerPtr subPlanner = allocPlanner(i, setup->getSpaceInformation());
            planner->as<geometric::AnytimePathShortening>()->addPlanner(subPlanner);
        }

        std::cout << "Planner suite: " << std::endl;
        for (size_t i = 0; i < planners.size(); ++i)
            std::cout << i + 1 << ": " << planner->as<geometric::AnytimePathShortening>()->getPlanner(i)->getName()
                      << std::endl;
    }
    else
    {
        std::cout << "Planner suite: " << std::endl;
        std::cout << "1: " << planner->getName() << std::endl;
    }

    setup->setup();
    setup->print();

    tools::Benchmark benchmark(*setup);
    tools::Benchmark::Request request(runtime, 1e6, 50, 0.5, true, false, true);

    if (probType == BARRIERS)
        benchmark.setExperimentName("Barriers*");
    else if (probType == CUBICLES)
        benchmark.setExperimentName("Cubicles*");
    else
        benchmark.setExperimentName("EasySwap*");
    benchmark.addPlanner(planner);
    benchmark.benchmark(request);
    benchmark.saveResultsToFile();
}

int main(int argc, char **argv)
{
    // Expecting the following arguments:
    // 1 - problem: barriers, cubicles, easy
    // 2 - optimization type: shortcut, hybridize, alternate
    // 3 - runtime, in seconds
    // 4 - planner(s)

    if (argc < 5)
    {
        if (argc != 2)
        {
            std::cout << "Usage:" << std::endl;
            std::cout << "demo_AnytimePathShortening ([barriers|cubicles|easy] [shortcut|hybridize|alternate|none] "
                         "[runtime(seconds)] [planner1] (planner2)...)|(input file)"
                      << std::endl;
            return 0;
        }
    }

    ProblemType probType;
    OptimizationType optType;
    double runtime;
    std::vector<PlannerType> planners;
    parseArguments(argc, argv, probType, optType, runtime, planners);
    if (!validArguments(probType, optType, runtime, planners))
        return 1;

    solve(probType, optType, runtime, planners);
}
