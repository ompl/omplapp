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

#include <omplapp/apps/KinematicCarPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

int main()
{
    // plan for kinematic car in SE(2)
    app::KinematicCarPlanning setup;
    base::StateManifoldPtr SE2 = setup.getStateManifold();

    // set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    setup.getStateManifold()->as<base::SE2StateManifold>()->setBounds(bounds);

    // define start state
    base::ScopedState<base::SE2StateManifold> start(setup.getSpaceInformation());
    start->setX(0);
    start->setY(0);
    start->setYaw(0);

    // define goal state
    base::ScopedState<base::SE2StateManifold> goal(start);
    goal->setX(2);
    goal->setY(2);
    goal->setYaw(M_PI);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    // try to solve the problem
    if (setup.solve(10))
    {
        // simplify & print the solution
        setup.getSolutionPath().print(std::cout);
    }

    return 0;
}
