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

#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/BlimpPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

void blimpSetup(app::BlimpPlanning &setup)
{
    base::StateSpacePtr stateSpace(setup.getStateSpace());

    // set the bounds for the R^3 part of SE(3)
    base::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(0)->setBounds(bounds);

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getGeometricComponentStateSpace());
    start->setX(0.);
    start->setY(0.);
    start->setZ(0.);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(setup.getGeometricComponentStateSpace());
    goal->setX(5.);
    goal->setY(5.);
    goal->setZ(5.);
    goal->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(setup.getFullStateFromGeometricComponent(start),
                                setup.getFullStateFromGeometricComponent(goal), .5);
}

void blimpDemo(app::BlimpPlanning &setup)
{
    std::vector<double> coords;

    std::cout << "\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;
    setup.setPlanner(std::make_shared<control::RRT>(setup.getSpaceInformation()));

    // try to solve the problem
    if (setup.solve(40))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        control::PathControl &path(setup.getSolutionPath());
        path.interpolate();  // uncomment if you want to plot the path
        path.printAsMatrix(std::cout);

        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is "
                      << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        }
    }
}

void blimpBenchmark(app::BlimpPlanning &setup)
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
    app::BlimpPlanning blimp;
    blimpSetup(blimp);

    // If any command line arguments are given, solve the problem multiple
    // times with different planners and collect benchmark statistics.
    // Otherwise, solve the problem once and print the path.
    if (argc > 1)
        blimpBenchmark(blimp);
    else
        blimpDemo(blimp);
    return 0;
}
