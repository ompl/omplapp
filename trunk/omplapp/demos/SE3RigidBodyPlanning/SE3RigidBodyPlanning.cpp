#include <omplapp/config.h>
#include <omplapp/SE3RigidBodyPlanning.h>

using namespace ompl;

int main()
{
    app::SE3RigidBodyPlanning setup;
    std::string robot_fname = std::string(OMPL_RESOURCE_DIR) + "/alpha_robot.dae";
    std::string env_fname = std::string(OMPL_RESOURCE_DIR) + "/alpha_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());


    base::ScopedState<base::SE3StateManifold> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(70.57);
    start->setZ(40.62);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateManifold> goal(start);
    goal->setX(200.49);
    goal->setY(70.57);
    goal->setZ(40.62);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setup.setup();
    
    setup.print();
    if (setup.solve(60))
    {
	setup.simplifySolution();
	setup.getSolutionPath().print(std::cout);
    }
    
    return 0;
}
