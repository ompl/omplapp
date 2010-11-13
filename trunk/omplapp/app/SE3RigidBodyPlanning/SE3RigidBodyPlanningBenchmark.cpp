#include "common/config.h"
#include "common/SE3RigidBodyPlanning.h"
#include <ompl/benchmark/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
using namespace ompl;

int main(int argc, char **argv)
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
    setup.getSpaceInformation()->setValidStateSamplerAllocator(base::UniformValidStateSampler::allocator());


    const double RUNTIME_LIMIT = 10.0;
    const double MEMORY_LIMIT  = 500.0;
    const int    RUN_COUNT     = 500;
    

    Benchmark b(setup, "cubicles_uniform_sampler");
    b.addPlanner(base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::LazyRRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::LBKPIECE1(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::KPIECE1(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::SBL(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    
    b.benchmark(RUNTIME_LIMIT, MEMORY_LIMIT, RUN_COUNT, true);
    b.saveResultsToFile();

    
    setup.getSpaceInformation()->setValidStateSamplerAllocator(base::GaussianValidStateSampler::allocator());
    b.setExperimentName("cubicles_gaussian_sampler");
    b.benchmark(RUNTIME_LIMIT, MEMORY_LIMIT, RUN_COUNT, true);
    b.saveResultsToFile();
    

    setup.getSpaceInformation()->setValidStateSamplerAllocator(base::ObstacleBasedValidStateSampler::allocator());
    b.setExperimentName("cubicles_obstaclebased_sampler");
    b.benchmark(RUNTIME_LIMIT, MEMORY_LIMIT, RUN_COUNT, true);
    b.saveResultsToFile();
    
    return 0;
}
