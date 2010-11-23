#include "common/config.h"
#include "common/SE2RigidBodyPlanning.h"

using namespace ompl;

int main()
{
    app::SE2RigidBodyPlanning setup;
    std::string robot_fname = std::string(OMPL_RESOURCE_DIR) + "/Easy_robot.dae";
    std::string env_fname = std::string(OMPL_RESOURCE_DIR) + "/Easy_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());
    
    base::ScopedState<base::SE2StateManifold> start(setup.getSpaceInformation());
    start->setX(270.40);
    start->setY(-196.82);
    
    base::ScopedState<base::SE2StateManifold> goal(start);
    start->setX(270.40);
    start->setY(-396.82);
    
    setup.setStartAndGoalStates(start, goal);

    if (setup.solve())
    {
	setup.getSolutionPath().print(std::cout);
    }
    
    return 0;
}
