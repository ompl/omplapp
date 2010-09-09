#include "common/config.h"
#include "common/SE3RigidBodyPlanning.h"
using namespace ompl;

int main()
{
    app::SE3RigidBodyPlanning setup;
    std::string robot_fname = std::string(OMPL_RESOURCE_DIR) + "/alpha_robot.stl";
    std::string env_fname = std::string(OMPL_RESOURCE_DIR) + "/alpha_env.stl";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());
    
    base::ScopedState<base::SE3StateManifold> start(setup.getSpaceInformation());
    start->setX(82.07);
    start->setY(46.98);
    start->setZ(316.01);
    start->rotation().setAxisAngle(1, 0, 0, M_PI * -35.0/180.0);

    base::ScopedState<base::SE3StateManifold> goal(start);
    goal->setX(91.07);
    goal->setY(17.98);
    goal->setZ(328.01);
    goal->rotation().setIdentity();
        
    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.05);
    setup.setup();
    
    setup.print();
    if (setup.solve(60))
    {
	setup.simplifySolution();
	setup.getSolutionPath().print(std::cout);
    }
    
    return 0;
}
