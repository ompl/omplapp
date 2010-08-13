#include "common/SE2RigidBodyPlanning.h"

using namespace ompl;

int main()
{
    app::SE2RigidBodyPlanning setup;
    setup.setMeshes("Twistycool_robot.dae", "Twistycool_env.dae");
    
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
