/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2012, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Ryan Luna */


#include <omplapp/config.h>
#include <omplapp/apps/SE3MultiRigidBodyPlanning.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/gnat/GNAT.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
using namespace ompl;



base::ValidStateSamplerPtr allocUniformStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::UniformValidStateSampler(si));
}

base::ValidStateSamplerPtr allocGaussianStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::GaussianValidStateSampler(si));
}

base::ValidStateSamplerPtr allocObstacleStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::ObstacleBasedValidStateSampler(si));
}

base::ValidStateSamplerPtr allocMaximizeClearanceStateSampler(const base::SpaceInformation *si)
{
    base::MaximizeClearanceValidStateSampler *s = new base::MaximizeClearanceValidStateSampler(si);
    s->setNrImproveAttempts(5);
    return base::ValidStateSamplerPtr(s);
}



// prints the individual path for robot #index
void printMultiRobotPath (const geometric::PathGeometric& path, unsigned int index, std::ostream& o = std::cout)
{   
    base::SE3StateSpace se3;
    const base::SE3StateSpace::StateType *state;

    for (unsigned int i = 0; i < path.getStateCount(); ++i)
    {
        state = path.getState(i)->as<base::CompoundStateSpace::StateType>()->as<base::SE3StateSpace::StateType>(index);
        se3.printState(state, o);
    }
}

int main()
{
    // plan for 2 rigid bodies in SE3
    app::SE3MultiRigidBodyPlanning setup(2);

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/ring.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/ring.dae";
    setup.setRobotMesh(robot_fname.c_str());  // The first mesh should use setRobotMesh.
    setup.addRobotMesh(robot_fname.c_str());  // Subsequent robot meshes MUST use addRobotMesh!

    setup.setEnvironmentMesh(env_fname.c_str());

    // constructing start and goal states
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(setup.getSpaceInformation());
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(setup.getSpaceInformation());

    base::SE3StateSpace se3;

    ompl::base::SE3StateSpace::StateType* start1 = start.get()->as<ompl::base::SE3StateSpace::StateType>(0);
    // define start state (robot 1)
    start1->setX(54.48);
    start1->setY(47.96);
    start1->setZ(3.59);
    start1->rotation().setIdentity();
    se3.printState(start1,std::cout);
    // define goal state (robot 1)
    ompl::base::SE3StateSpace::StateType* goal1 = goal.get()->as<ompl::base::SE3StateSpace::StateType>(0);
    goal1->setX(54.48);
    goal1->setY(47.96);
    goal1->setZ(3.59);
    goal1->rotation().setAxisAngle(0.0,0.0,-1.0,0.785398163397);
    se3.printState(goal1,std::cout);

    ompl::base::SE3StateSpace::StateType* start2 = start.get()->as<ompl::base::SE3StateSpace::StateType>(1);
    // define start state (robot 2)
    start2->setX(54.48);
    start2->setY(47.96);
    start2->setZ(-6.0);
    start2->rotation().setAxisAngle(1.0,0.0,0.0,0.0);
    se3.printState(start2,std::cout);

    // define goal state (robot 2)
    ompl::base::SE3StateSpace::StateType* goal2 = goal.get()->as<ompl::base::SE3StateSpace::StateType>(1);
    goal2->setX(45.48);
    goal2->setY(47.96);
    goal2->setZ(-6.0);
    goal2->rotation().setAxisAngle(-0.57735026919,0.57735026919,0.57735026919,2.09439510239);
    se3.printState(goal2,std::cout);
    
 /*   ompl::base::SE3StateSpace::StateType* start3 = start.get()->as<ompl::base::SE3StateSpace::StateType>(2);
    // define start state (robot 3)
    start3->setX(54.48);
    start3->setY(47.96);
    start3->setZ(12.0);
    start3->rotation().setAxisAngle(0.0,0.0,1.0,1.57079632679);

    // define goal state (robot 3)
    ompl::base::SE3StateSpace::StateType* goal3 = goal.get()->as<ompl::base::SE3StateSpace::StateType>(2);
    goal3->setX(62.48);
    goal3->setY(36.96);
    goal3->setZ(10.0);
    goal3->rotation().setAxisAngle(-0.223667258103,0.106683809581,0.968809332339,2.27585417028);
    */


    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // use RRT for planning
    setup.setPlanner (base::PlannerPtr(new geometric::GNAT(setup.getSpaceInformation())));

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    // try to solve the problem
        if (setup.solve(30))
          {
          if (setup.haveExactSolutionPath() || true)
          {
    // simplify & print the solution
    setup.simplifySolution();
    std::cout << "Robot #1:" << std::endl;
    printMultiRobotPath (setup.getSolutionPath(), 0);  // Robot #0's path
    std::cout << std::endl << "Robot #2:" << std::endl;
    printMultiRobotPath (setup.getSolutionPath(), 1);  // Robot #1's path
    }
    else
    {
    std::cout << "Exact solution not found" << std::endl;
    }
    }
/*
    std::string benchmark_name("Ring");
    double runtime_limit=10, memory_limit=8000;
    int run_count=3;
    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    // optionally set pre & pos run events
    //b.setPreRunEvent(boost::bind(&preRunEvent, _1));
    //b.setPostRunEvent(boost::bind(&postRunEvent, _1, _2));

//    b.addPlanner(base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));
//    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
//    b.addPlanner(base::PlannerPtr(new geometric::BKPIECE1(setup.getSpaceInformation())));
//    b.addPlanner(base::PlannerPtr(new geometric::LBKPIECE1(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::KPIECE1(setup.getSpaceInformation())));
//    b.addPlanner(base::PlannerPtr(new geometric::SBL(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));
//    b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    base::PlannerPtr p(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,3));
    p->setName("GNAT-3");
//    b.addPlanner(p);
    base::PlannerPtr p1(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,6));
    p1->setName("GNAT-6");
    b.addPlanner(p1);
    base::PlannerPtr p2(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,6));
    p2->params().setParam("propagate_while_valid","0");
    p2->setName("GNAT-6-No-Prop");
    b.addPlanner(p2);

    // run all planners with a obstacle-based valid state sampler on the benchmark problem
    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocObstacleStateSampler);
    b.setExperimentName(benchmark_name);
    b.benchmark(request);
    b.saveResultsToFile("Ring-Multi-full.log");
    */
}
