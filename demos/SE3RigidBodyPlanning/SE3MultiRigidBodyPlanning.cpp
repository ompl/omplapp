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
#include <boost/lexical_cast.hpp>
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

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout<<"Please provide an even number of robots to plan with. i.e. 4"<<std::endl;
        std::cout<<"Optionally input the planner number see src for codes"<<std::endl;
        exit(0);
    }
    int plannerNumber(std::numeric_limits<int>::max());
    if(argc > 2)
    {
        plannerNumber = boost::lexical_cast<int>(std::string(argv[2]));
    }
    size_t ROBOT_COUNT = boost::lexical_cast<size_t>(std::string(argv[1]));
    std::cout<<"Planning on "<<ROBOT_COUNT<<" robots."<<std::endl;
    // plan for 2 rigid bodies in SE3
    app::SE3MultiRigidBodyPlanning setup(ROBOT_COUNT);

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/ring.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Easy_env.dae";
    setup.setRobotMesh(robot_fname.c_str());  // The first mesh should use setRobotMesh.
    setup.addRobotMesh(robot_fname.c_str());  // Subsequent robot meshes MUST use addRobotMesh!

    setup.setEnvironmentMesh(env_fname.c_str());

    // constructing start and goal states
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(setup.getSpaceInformation());
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(setup.getSpaceInformation());

    base::SE3StateSpace se3;

    for(size_t k=0; k<ROBOT_COUNT/2; k++)
    {
        ompl::base::SE3StateSpace::StateType* start1 = start.get()->as<ompl::base::SE3StateSpace::StateType>(2*k);
        // define start state (robot 1)
        start1->setX(300 + 54.48);
        start1->setY(36.96);
        start1->setZ(-275.00 + 2*k*10.0);
        start1->rotation().setAxisAngle(0,0,0,1);
        se3.printState(start1,std::cout);
        // define goal state (robot 1)
        ompl::base::SE3StateSpace::StateType* goal1 = goal.get()->as<ompl::base::SE3StateSpace::StateType>(2*k);
        goal1->setX(300 + 54.48);
        goal1->setY(170.96 - 2*k*10.0);
        goal1->setZ(-450);
        goal1->rotation().setAxisAngle(0.338308450819,0.374031211022,-0.863509145963,1.79883884924);
        se3.printState(goal1,std::cout);

        ompl::base::SE3StateSpace::StateType* start2 = start.get()->as<ompl::base::SE3StateSpace::StateType>(2*k+1);
        // define start state (robot 2)
        start2->setX(300 + 54.48);
        start2->setY(36.96);
        start2->setZ(-275 + (2*k+1)*10.0);
        start2->rotation().setAxisAngle(-0.73579887177,0.655163379006,-0.171350422223,3.42251548301);
        se3.printState(start2,std::cout);

        // define goal state (robot 2)
        ompl::base::SE3StateSpace::StateType* goal2 = goal.get()->as<ompl::base::SE3StateSpace::StateType>(2*k+1);
        goal2->setX(300+56.48);
        goal2->setY(35.96);
        goal2->setY(170.96 - (2*k+1)*10.0);
        goal2->setZ(-450.00);
        goal2->rotation().setAxisAngle(-0.268761115857,0.67719659038,0.673026403874,3.62553819142);
        se3.printState(goal2,std::cout);
    }


    // set the start & goal states
    //setup.setStartAndGoalStates(start,goal);
    setup.setStartState(start);
    setup.setGoalState(goal,1e-4);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.001);

    // use RRT for planning
    //base::PlannerPtr plnr(new geometric::KPIECE1(setup.getSpaceInformation()));
    //base::PlannerPtr plnr(new geometric::RRTConnect(setup.getSpaceInformation()));
    base::PlannerPtr plnr(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,3));
    setup.setPlanner(plnr); 

    // we call setup just so print() can show more information
    setup.setup();
    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocObstacleStateSampler);
    setup.params().setParam("goal_bias","0.9");
    setup.print();

    //     try to solve the problem

    if(argc>3)
        if (setup.solve(1000))
        {
            if (setup.haveExactSolutionPath())
            {
                // simplify & print the solution
                setup.simplifySolution();
                std::cout << "Robot #1:" << std::endl;
                printMultiRobotPath (setup.getSolutionPath(), 0);  // Robot #0's path
                std::cout << std::endl << "Robot #2:" << std::endl;
                printMultiRobotPath (setup.getSolutionPath(), 1);  // Robot #1's path
                std::cout << "Exact solution found, proceeding" << std::endl;
            }
            else
            {
                std::cout << "Exact solution not found" << std::endl;
                exit(0);
            }
        }

    std::string benchmark_name("Ring");
    double runtime_limit=1000, memory_limit=8000;
    int run_count=1;
    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    // optionally set pre & pos run events
    //b.setPreRunEvent(boost::bind(&preRunEvent, _1));
    //b.setPostRunEvent(boost::bind(&postRunEvent, _1, _2));

    if(plannerNumber & (1))
        b.addPlanner(base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));
    if(plannerNumber & (1<<1))
        b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    if(plannerNumber & (1<<2))
        b.addPlanner(base::PlannerPtr(new geometric::BKPIECE1(setup.getSpaceInformation())));
    if(plannerNumber & (1<<3))
        b.addPlanner(base::PlannerPtr(new geometric::LBKPIECE1(setup.getSpaceInformation())));
    if(plannerNumber & (1<<4))
        b.addPlanner(base::PlannerPtr(new geometric::KPIECE1(setup.getSpaceInformation())));
    if(plannerNumber & (1<<5))
        b.addPlanner(base::PlannerPtr(new geometric::SBL(setup.getSpaceInformation())));
    if(plannerNumber & (1<<6))
        b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));
    if(plannerNumber & (1<<7))
        b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    if(plannerNumber & (1<<8))
    {
        base::PlannerPtr p(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,3));
        p->setName("GNAT-3");
        b.addPlanner(p);
    }
    if(plannerNumber & (1<<9))
    {
        base::PlannerPtr p1(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,6));
        p1->setName("GNAT-6");
        b.addPlanner(p1);
    }
    if(plannerNumber & (1<<10))
    {
        base::PlannerPtr p2(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,6));
        p2->params().setParam("propagate_while_valid","0");
        p2->setName("GNAT-6-No-Prop");
        b.addPlanner(p2);
    }
    if(plannerNumber & (1<<11))
    {
        base::PlannerPtr p3(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,6*ROBOT_COUNT));
        p3->setName("GNAT-Native");
        b.addPlanner(p3);
    }
    if(plannerNumber & (1<<12))
    {
        base::PlannerPtr p4(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,6*ROBOT_COUNT));
        p4->params().setParam("propagate_while_valid","0");
        p4->setName("GNAT-Native-No-Prop");
        b.addPlanner(p4);
    }


    // run all planners with a obstacle-based valid state sampler on the benchmark problem
    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocObstacleStateSampler);
    b.setExperimentName(benchmark_name);
    b.benchmark(request);
    b.saveResultsToFile("Ring-Multi-full.log");
}
