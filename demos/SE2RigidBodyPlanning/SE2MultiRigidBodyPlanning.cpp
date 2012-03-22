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

#include <omplapp/apps/SE2MultiRigidBodyPlanning.h>
#include <omplapp/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using namespace ompl;

// prints the path for robot #index
void printMultiRobotPath (const geometric::PathGeometric& path, unsigned int index, std::ostream& o = std::cout)
{   
    base::SE2StateSpace se2;
    const base::SE2StateSpace::StateType *state;

    for (unsigned int i = 0; i < path.getStateCount(); ++i)
    {
        state = path.getState(i)->as<base::CompoundStateSpace::StateType>()->as<base::SE2StateSpace::StateType>(index);
        se2.printState(state, o);
    }
}

int main()
{
    // plan for two bodies in SE2
    app::SE2MultiRigidBodyPlanning setup(2);

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
    setup.setRobotMesh(robot_fname.c_str());    // The first mesh should use setRobotMesh.
    setup.addRobotMesh(robot_fname.c_str());    // Subsequent robot meshes MUST use addRobotMesh!
    setup.setEnvironmentMesh(env_fname.c_str());
    
    // constructing start and goal states
    base::ScopedState<base::CompoundStateSpace> start(setup.getSpaceInformation());
    base::ScopedState<base::CompoundStateSpace> goal(setup.getSpaceInformation());

    // define starting state for robot 1
    base::SE2StateSpace::StateType* start1 = start.get()->as<base::SE2StateSpace::StateType>(0);
    start1->setX(0.0);
    start1->setY(0.0);

    // define goal state for robot 1
    base::SE2StateSpace::StateType* goal1 = goal.get()->as<base::SE2StateSpace::StateType>(0);
    goal1->setX(26.0);
    goal1->setY(0.0);
    
    // define starting state for robot 2
    base::SE2StateSpace::StateType* start2 = start.get()->as<base::SE2StateSpace::StateType>(1);
    start2->setX(26.0);
    start2->setY(0.0);

    // define goal state for robot 2
    base::SE2StateSpace::StateType* goal2 = goal.get()->as<base::SE2StateSpace::StateType>(1);
    goal2->setX(0.0);
    goal2->setY(0.0);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);
    
    // use RRTConnect for planning
    setup.setPlanner (base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));

    // attempt to solve the problem, and print it to screen if a solution is found
    if (setup.solve(60))
    {
        std::cout << "Robot 1:" << std::endl;
        printMultiRobotPath(setup.getSolutionPath(), 0); // Path for robot 1
        std::cout << std::endl << "Robot 2:" << std::endl;
        printMultiRobotPath(setup.getSolutionPath(), 1); // Path for robot 2
    }

    return 0;
}
