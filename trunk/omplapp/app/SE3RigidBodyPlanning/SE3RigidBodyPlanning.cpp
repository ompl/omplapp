#include "common/config.h"
#include "common/SE3RigidBodyPlanning.h"
using namespace ompl;

int main()
{
    app::SE3RigidBodyPlanning setup;
    std::string robot_fname = std::string(OMPL_RESOURCE_DIR) + "/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPL_RESOURCE_DIR) + "/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());
    
    base::ScopedState<base::SE3StateManifold> start(setup.getSpaceInformation());
    start->setX(0);
    start->setY(0);
    start->setZ(0);
    start->rotation().setAxisAngle(0,0,0,0);

    base::ScopedState<base::SE3StateManifold> goal(start);
    goal->setX(0);
    goal->setY(0);
    goal->setZ(100);
    goal->rotation().setAxisAngle(0,0,0,0);
    
    setup.setStartAndGoalStates(start, goal);
    start.print();
    goal.print();
    if (setup.solve())
    {
	setup.simplifySolution();
	setup.getSolutionPath().print(std::cout);
    }
    
    return 0;
}
