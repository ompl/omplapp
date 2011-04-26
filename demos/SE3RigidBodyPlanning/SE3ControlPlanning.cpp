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

#include <omplapp/apps/SE3ControlPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

int main()
{
    // plan in SE3
    app::SE3ControlPlanning setup;
    const base::StateManifoldPtr &SE3 = setup.getGeometricComponentStateManifold();

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/cubicles_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // define start state
    base::ScopedState<base::SE3StateManifold> start(SE3);
    start->setX(-4.96);
    start->setY(70.57);
    start->setZ(40.62);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateManifold> goal(start);
    goal->setX(200.49);
    goal->setY(70.57);
    goal->setZ(40.62);
    goal->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(setup.getFullStateFromGeometricComponent(start),
        setup.getFullStateFromGeometricComponent(goal));

    // we call setup() just so print() can show more information
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
