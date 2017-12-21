/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Mark Moll */

#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/DynamicCarPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

void dynamicCarSetup(app::DynamicCarPlanning &setup)
{
    // plan for dynamic car in SE(2)
    base::StateSpacePtr stateSpace(setup.getStateSpace());

    // set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE2StateSpace>(0)->setBounds(bounds);

    // define start state
    base::ScopedState<> start(stateSpace);
    start[0] = start[1] = start[2] = start[3] = start[4] = 0.;

    // define goal state
    base::ScopedState<> goal(stateSpace);
    goal[0] = goal[1] = 8.;
    goal[2] = 0;
    goal[3] = goal[4] = 0.;

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, .5);

    // optionally, set a planner
    // setup.setPlanner(std::make_shared<control::EST>(setup.getSpaceInformation()));
    // setup.setPlanner(std::make_shared<control::RRT>(setup.getSpaceInformation()));
    // setup.setPlanner(std::make_shared<control::KPIECE1>(setup.getSpaceInformation()));
    // setup.setPlanner(std::make_shared<control::PDST>(setup.getSpaceInformation()));
    std::vector<double> cs(2);
    cs[0] = cs[1] = 0.1;
    setup.setup();
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);
}

void dynamicCarDemo(app::DynamicCarPlanning &setup)
{
    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;

    // try to solve the problem
    if (setup.solve(40))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        control::PathControl &path(setup.getSolutionPath());
        // path.interpolate(); // uncomment if you want to plot the path
        path.printAsMatrix(std::cout);
        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is "
                      << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        }
    }
}
void dynamicCarBenchmark(app::DynamicCarPlanning &setup)
{
    tools::Benchmark::Request request(100., 10000., 10);  // runtime (s), memory (MB), run count

    setup.setup();

    tools::Benchmark b(setup, setup.getName());
    b.addPlanner(std::make_shared<control::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<control::KPIECE1>(setup.getSpaceInformation()));
    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int argc, char ** /*unused*/)
{
    app::DynamicCarPlanning car;
    dynamicCarSetup(car);

    // If any command line arguments are given, solve the problem multiple
    // times with different planners and collect benchmark statistics.
    // Otherwise, solve the problem once and print the path.
    if (argc > 1)
        dynamicCarBenchmark(car);
    else
        dynamicCarDemo(car);
    return 0;
}
