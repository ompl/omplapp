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

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/multiplan/OptimizePlan.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

int main()
{
    // plan in SE3
    app::SE3RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
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
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // make sure the planners run until the time limit, and get the best possible solution
    setup.getProblemDefinition()->setOptimizationObjective(
        std::make_shared<base::PathLengthOptimizationObjective>(setup.getSpaceInformation()));

    setup.setup();

    std::stringstream res;

    // run with RRT*
    setup.setPlanner(std::make_shared<geometric::RRTstar>(setup.getSpaceInformation()));
    res << "RRT*" << std::endl;
    for (double time = 1.0; time < 10.1; time = time + 1.0)
    {
        setup.clear();
        double length = -1.0;
        // try to solve the problem
        if (setup.solve(time) && setup.haveExactSolutionPath())
            length = setup.getSolutionPath().length();
        res << "time = " << setup.getLastPlanComputationTime() << " \t length = " << length << std::endl;
    }

    tools::OptimizePlan op(setup.getProblemDefinition());
    res << "RRTConnect with path hybridization (one thread)" << std::endl;
    for (double time = 1.0; time < 10.1; time = time + 1.0)
    {
        setup.clear();
        op.clearPlanners();

        // add one planer only, so there is only one planning thread in use
        op.addPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));

        double length = -1.0;
        double duration = 0.0;

        ompl::time::point start = ompl::time::now();
        // try to solve the problem
        if (op.solve(time, 30, 1) && setup.haveExactSolutionPath())
        {
            duration = ompl::time::seconds(ompl::time::now() - start);
            length = setup.getSolutionPath().length();
        }
        else
            duration = ompl::time::seconds(ompl::time::now() - start);

        res << "time = " << duration << "s \t length = " << length << std::endl;
    }

    res << "RRTConnect with path hybridization (four threads)" << std::endl;
    for (double time = 1.0; time < 10.1; time = time + 1.0)
    {
        setup.clear();
        op.clearPlanners();

        // add one planer only, so there is only one planning thread in use
        op.addPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));
        op.addPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));
        op.addPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));
        op.addPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));

        double length = -1.0;
        double duration = 0.0;

        ompl::time::point start = ompl::time::now();
        // try to solve the problem
        if (op.solve(time, 30, 4) && setup.haveExactSolutionPath())
        {
            duration = ompl::time::seconds(ompl::time::now() - start);
            length = setup.getSolutionPath().length();
        }
        else
            duration = ompl::time::seconds(ompl::time::now() - start);

        res << "time = " << duration << "s \t length = " << length << std::endl;
    }

    std::cout << res.str();

    return 0;
}
