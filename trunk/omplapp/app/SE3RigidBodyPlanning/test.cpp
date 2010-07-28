#include "SE3RigidBodyPlanning.h"

using namespace ompl;

int main()
{
    app::SE3RigidBodyPlanning setup;
    setup.setMeshes("sphere.stl", "sphere.stl");
    
    base::ScopedState<> start(setup.getSpaceInformation());
    start.random();
    
    base::ScopedState<base::SE3StateManifold> goal(start);
    goal.random();
    goal->setX(2);
    goal->setY(3);
    
    setup.setStartAndGoalStates(start, goal);

    if (setup.solve())
    {
	setup.getSolutionPath().print(std::cout);
    }
    
    return 0;
}
