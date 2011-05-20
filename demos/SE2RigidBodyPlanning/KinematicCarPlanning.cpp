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

#include <ompl/benchmark/Benchmark.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <omplapp/apps/KinematicCarPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

void kinematicCarSetup(app::KinematicCarPlanning& setup)
{
    // plan for kinematic car in SE(2)
    const base::StateSpacePtr &SE2 = setup.getGeometricComponentStateSpace();

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
    goal->setYaw(M_PI);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, .1);
}

void kinematicCarDemo(app::KinematicCarPlanning& setup)
{
    std::cout<<"\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;
    //setup.setPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));

    // try to solve the problem
    if (setup.solve(20))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        control::PathControl& path(setup.getSolutionPath());
        for (unsigned int i=0; i<path.states.size(); ++i)
        {
            const base::SE2StateSpace::StateType& s = *path.states[i]->as<base::SE2StateSpace::StateType>();
            std::cout << s.getX() <<' '<< s.getY() << ' ' << s.getYaw() << ' ';
            if (i==0)
                // null controls applied for zero seconds to get to start state
                std::cout << "0 0 0";
            else
            {
                // print controls and control duration needed to get from state i-1 to state i
                const double* c = path.controls[i-1]->as<control::RealVectorControlSpace::ControlType>()->values;
                std::cout << c[0] << ' ' << c[1] << ' ' << path.controlDurations[i-1];
            }
            std::cout << std::endl;
        }
        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is " <<
                setup.getGoal()->getDifference() << std::endl;
        }
    }

}
void kinematicCarBenchmark(app::KinematicCarPlanning& setup)
{
    double runtime_limit = 20.0;
    double memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    int    run_count     = 10;

    Benchmark b(setup, setup.getName());
    b.addPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
    b.benchmark(runtime_limit, memory_limit, run_count, true);
    b.saveResultsToFile();
}

int main(int argc, char* argv[])
{
    app::KinematicCarPlanning regularCar;
    app::ReedsSheppCarPlanning rsCar;
    app::DubinsCarPlanning dCar;

    kinematicCarSetup(regularCar);
    kinematicCarSetup(rsCar);
    kinematicCarSetup(dCar);

    // If any command line arguments are given, solve the problem multiple
    // times for each car type with different planners and collect benchmark
    // statistics. Otherwise, solve the problem once for each car type and
    // print the path.
    if (argc>1)
    {
        kinematicCarBenchmark(regularCar);
        kinematicCarBenchmark(rsCar);
        kinematicCarBenchmark(dCar);
    }
    else
    {
        kinematicCarDemo(regularCar);
        kinematicCarDemo(rsCar);
        kinematicCarDemo(dCar);
    }
    return 0;
}
