/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Javier V. Gomez */

#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <omplapp/config.h>

using namespace ompl;

int main()
{
    // plan in SE3
    app::SE3RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/alpha_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/alpha_env-1.5.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-21.91);
    start->setY(-4.11);
    start->setZ(-14.14);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-21.91);
    goal->setY(-4.11);
    goal->setZ(68.86);
    goal->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // setting bounds so it is the same as for alpha 1.5 cfg problem.
    base::RealVectorBounds bounds(3);
    bounds.setLow(0, -281.64);
    bounds.setLow(1, -119.64);
    bounds.setLow(2, -176.86);
    bounds.setHigh(0, 189.05);
    bounds.setHigh(1, 189.18);
    bounds.setHigh(2, 174.86);
    setup.getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

    // make sure the planners run until the time limit, and get the best possible solution
    setup.getProblemDefinition()->setOptimizationObjective(
        base::OptimizationObjectivePtr(new base::PathLengthOptimizationObjective(setup.getSpaceInformation())));

    // run with RRT*
    std::cout << "RRT*" << std::endl;
    setup.setPlanner(base::PlannerPtr(new geometric::RRTstar(setup.getSpaceInformation())));
    setup.setup();
    double length = -1.0;
    for (double time = 1.0 ; time > 0.0 ; time = time + 10.0)
    {
        std::cout << "Computing up to " << time << " seconds" << std::endl;
        setup.clear();
        length = -1.0;
        // try to solve the problem
        if (setup.solve(time) && setup.haveExactSolutionPath())
        {
            length = setup.getSolutionPath().length();
            break;
        }
    }
    std::cout << "time = "  << setup.getLastPlanComputationTime() << " \t length = " << length << std::endl;

     // run with CForest, 2 threads of RRTstar.
    std::cout << "CForest" << std::endl;
    setup.clear();
    setup.setPlanner(base::PlannerPtr(new geometric::CForest(setup.getSpaceInformation())));
    setup.setup();
    for (double time = 1.0 ; time > 0.0 ; time = time + 10.0)
    {
        std::cout << "Computing up to " << time << " seconds" << std::endl;
        setup.clear();
        length = -1.0;
        // try to solve the problem
        if (setup.solve(time) && setup.haveExactSolutionPath())
        {
            length = setup.getSolutionPath().length();
            break;
        }
    }
    std::cout << "time = "  << setup.getLastPlanComputationTime() << " \t length = " << length << std::endl;

    return 0;
}
