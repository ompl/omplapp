#include "common/config.h"
#include "common/SE3RigidBodyPlanning.h"
using namespace ompl;

int main()
{
    app::SE3RigidBodyPlanning setup;
    std::string robot_fname = std::string(OMPL_RESOURCE_DIR) + "/alpha_robot.dae";
    std::string env_fname = std::string(OMPL_RESOURCE_DIR) + "/alpha_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());
    
    base::ScopedState<base::SE3StateManifold> start(setup.getSpaceInformation());
    start->setX(17.49);
    start->setY(10.12);
    start->setZ(-1.39);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateManifold> goal(start);
    start->setX(17.49);
    start->setY(10.12);
    start->setZ(-40.39);
    goal->rotation().setIdentity();
        
    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    setup.setup();
    
    setup.print();
    if (setup.solve(60))
    {
	setup.simplifySolution();
	setup.getSolutionPath().print(std::cout);
    }
    
    return 0;
}
