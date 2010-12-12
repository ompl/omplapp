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

#include <omplapp/SE2RigidBodyPlanning.h>

using namespace ompl;

int main()
{
    // plan in SE2
    app::SE2RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/Easy_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/Easy_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    // define starting state
    base::ScopedState<base::SE2StateManifold> start(setup.getSpaceInformation());
    start->setX(270.40);
    start->setY(-196.82);

    // define goal state
    base::ScopedState<base::SE2StateManifold> goal(start);
    start->setX(270.40);
    start->setY(-396.82);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // attempt to solve the problem, and print it to sceen if a solution is found
    if (setup.solve())
        setup.getSolutionPath().print(std::cout);

    return 0;
}
