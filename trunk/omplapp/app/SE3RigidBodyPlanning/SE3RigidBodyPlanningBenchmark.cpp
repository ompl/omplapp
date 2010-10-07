#include "common/config.h"
#include "common/SE3RigidBodyPlanning.h"
#include <ompl/geometric/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
using namespace ompl;

int main()
{
    app::SE3RigidBodyPlanning setup;
    std::string robot_fname = std::string(OMPL_RESOURCE_DIR) + "/cubicles_robot.dae";
    std::string env_fname = std::string(OMPL_RESOURCE_DIR) + "/cubicles_env.dae";
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
    
    geometric::Benchmark b(setup);
    b.addPlanner(base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::LBKPIECE1(setup.getSpaceInformation())));
    
    b.benchmark(10, 1, 50);
    b.saveResultsToStream();

    return 0;
}
