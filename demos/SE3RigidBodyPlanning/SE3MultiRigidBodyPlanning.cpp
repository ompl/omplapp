/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2012, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Ryan Luna */

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <omplapp/apps/AppBase.h>
#include <omplapp/apps/SE3MultiRigidBodyPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

int main()
{
    // plan for 2 rigid bodies in SE3
    app::SE3MultiRigidBodyPlanning setup(2);

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname);  // The first mesh should use setRobotMesh.
    setup.addRobotMesh(robot_fname);  // Subsequent robot meshes MUST use addRobotMesh!
    setup.setEnvironmentMesh(env_fname);

    // constructing start and goal states
    base::ScopedState<base::CompoundStateSpace> start(setup.getSpaceInformation());
    base::ScopedState<base::CompoundStateSpace> goal(setup.getSpaceInformation());

    auto *start1 = start->as<base::SE3StateSpace::StateType>(0);
    // define start state (robot 1)
    start1->setX(-4.96);
    start1->setY(-40.62);
    start1->setZ(70.57);
    start1->rotation().setIdentity();

    // define goal state (robot 1)
    auto *goal1 = goal->as<base::SE3StateSpace::StateType>(0);
    goal1->setX(200.49);
    goal1->setY(-40.62);
    goal1->setZ(70.57);
    goal1->rotation().setIdentity();

    auto *start2 = start->as<base::SE3StateSpace::StateType>(1);
    // define start state (robot 2)
    start2->setX(200.49);
    start2->setY(-40.62);
    start2->setZ(70.57);
    start2->rotation().setIdentity();

    // define goal state (robot 2)
    auto *goal2 = goal->as<base::SE3StateSpace::StateType>(1);
    goal2->setX(-4.96);
    goal2->setY(-40.62);
    goal2->setZ(70.57);
    goal2->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // use RRTConnect for planning
    setup.setPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    // try to solve the problem
    if (setup.solve(30))
    {
        if (setup.haveExactSolutionPath())
        {
            // simplify & print the solution
            setup.simplifySolution();
            setup.getSolutionPath().printAsMatrix(std::cout);
        }
        else
        {
            std::cout << "Exact solution not found" << std::endl;
        }
    }

    return 0;
}
