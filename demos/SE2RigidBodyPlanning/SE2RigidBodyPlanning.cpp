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

#include <omplapp/apps/SE2RigidBodyPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

int main()
{
    // plan in SE2
    app::SE2RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/maze_env1.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/robot1.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    // define starting state
    base::ScopedState<base::SE2StateManifold> start(setup.getSpaceInformation());
    start->setX(0.0);
    start->setY(0.02);

    // define goal state
    base::ScopedState<base::SE2StateManifold> goal(start);
    start->setX(0.0);
    start->setY(0.16);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // attempt to solve the problem, and print it to sceen if a solution is found
    if (setup.solve())
        setup.getSolutionPath().print(std::cout);

    return 0;
}
