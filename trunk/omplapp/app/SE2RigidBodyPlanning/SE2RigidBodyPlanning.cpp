#include "common/config.h"
#include "common/SE2RigidBodyPlanning.h"

using namespace ompl;

int main()
{
    app::SE2RigidBodyPlanning setup;
    std::string robot_fname = std::string(OMPL_RESOURCE_DIR) + "/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPL_RESOURCE_DIR) + "/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());
    
    base::ScopedState<> start(setup.getSpaceInformation());
    start.random();
    
    base::ScopedState<base::SE2StateManifold> goal(start);
    goal->setYaw(0.0);
    goal->setX(2.0);
    goal->setY(3.0);
    
    setup.setStartAndGoalStates(start, goal);

    if (setup.solve())
    {
	setup.getSolutionPath().print(std::cout);
    }
    
    return 0;
}
