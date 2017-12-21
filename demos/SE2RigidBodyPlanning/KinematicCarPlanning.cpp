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
#include <omplapp/apps/KinematicCarPlanning.h>
#include <omplapp/config.h>
#include <boost/math/constants/constants.hpp>

using namespace ompl;

void kinematicCarSetup(app::KinematicCarPlanning &setup)
{
    // plan for kinematic car in SE(2)
    base::StateSpacePtr SE2(setup.getStateSpace());

    // set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    SE2->as<base::SE2StateSpace>()->setBounds(bounds);

    // define start state
    base::ScopedState<base::SE2StateSpace> start(SE2);
    start->setX(0);
    start->setY(0);
    start->setYaw(0);

    // define goal state
    base::ScopedState<base::SE2StateSpace> goal(SE2);
    goal->setX(2);
    goal->setY(2);
    goal->setYaw(boost::math::constants::pi<double>());

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, .1);
}

void kinematicCarDemo(app::KinematicCarPlanning &setup)
{
    setup.setPlanner(std::make_shared<control::KPIECE1>(setup.getSpaceInformation()));
    std::vector<double> cs(2);
    cs[0] = cs[1] = 0.1;
    setup.setup();
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    // try to solve the problem
    if (setup.solve(20))
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
void kinematicCarBenchmark(app::KinematicCarPlanning &setup)
{
    tools::Benchmark::Request request(20., 10000., 10);  // runtime (s), memory (MB), run count

    tools::Benchmark b(setup, setup.getName());
    b.addPlanner(std::make_shared<control::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<control::KPIECE1>(setup.getSpaceInformation()));
    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int argc, char ** /*unused*/)
{
    app::KinematicCarPlanning regularCar;

    kinematicCarSetup(regularCar);

    // If any command line arguments are given, solve the problem multiple
    // times with different planners and collect benchmark statistics.
    // Otherwise, solve the problem once for each car type and print the path.
    if (argc > 1)
        kinematicCarBenchmark(regularCar);
    else
        kinematicCarDemo(regularCar);
    return 0;
}
