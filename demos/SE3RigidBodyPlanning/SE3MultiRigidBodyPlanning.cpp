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
#include <omplapp/apps/AppBase.h>
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

class MultiSE3RigidBodyPlanning : public app::AppBase<app::GEOMETRIC>
{
    public:
        /// @brief Constructs an instance of multiple rigid bodies for 3D geometric planning.  n is the number of independent bodies in SE(3)
        MultiSE3RigidBodyPlanning(unsigned int n) : app::AppBase<app::GEOMETRIC>(base::StateSpacePtr(new base::CompoundStateSpace()), app::Motion_3D), n_(n)
    {
        assert (n > 0);
        name_ = "Rigid body planning (n*3D)";
        // Adding n SE(3) StateSpaces
        for (unsigned int i = 0; i < n_; ++i)
            si_->getStateSpace()->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::SE3StateSpace()), 1.0);
    }

        virtual ~MultiSE3RigidBodyPlanning(void) {}

        bool isSelfCollisionEnabled(void) const 
        {
            // Make sure that self collision is enabled to avoid inter-rigid body collision
            return true; 
        }

        /// @brief Constructs the default start state where all robots begin at their geometric center.
        /// If robots are all using the same mesh, this state is not likely to be valid.
        virtual base::ScopedState<> getDefaultStartState(void) const
        {
            base::ScopedState<> st(getStateSpace());
            base::CompoundStateSpace::StateType* c_st = st.get()->as<base::CompoundStateSpace::StateType>();
            for (unsigned int i = 0; i < n_; ++i)
            {
                aiVector3D s = getRobotCenter(i);
                base::SE3StateSpace::StateType* sub = c_st->as<base::SE3StateSpace::StateType>(i);
                sub->setX(s.x);
                sub->setY(s.y);
                sub->setZ(s.z);
                sub->rotation().setIdentity();
            }
            return st;
        }

        virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
        {
            // States are composed of only geometric components.  No work needed here.
            return state;
        }

        /// @brief Returns the state space corresponding for the indexth rigid body
        virtual const base::StateSpacePtr& getGeometricComponentStateSpace(unsigned int index) const
        {
            return getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(index);
        }

        virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
        {
            // Return the zeroth component.  All components are the same.
            return getGeometricComponentStateSpace(0);
        }

        virtual unsigned int getRobotCount(void) const 
        {
            return n_; 
        }

        virtual void inferEnvironmentBounds(void)
        {
            // Infer bounds for all n SE(3) spaces
            for (unsigned int i = 0; i < n_; ++i)
                InferEnvironmentBounds(getGeometricComponentStateSpace(i), *static_cast<RigidBodyGeometry*>(this));
        }

        virtual void inferProblemDefinitionBounds(void)
        {
            // Make sure that all n SE(3) spaces get the same bounds
            for (unsigned int i = 0; i < n_; ++i)
                InferProblemDefinitionBounds(app::AppTypeSelector<app::GEOMETRIC>::SimpleSetup::getProblemDefinition(), getGeometricStateExtractor(), factor_, add_,
                        n_, getGeometricComponentStateSpace(i), mtype_);
        }

    protected:
        /// @brief Returns the SE3 state corresponding to the indexth rigid body in the compound state
        virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const
        {
            assert (index < n_);
            const base::SE3StateSpace::StateType* st = state->as<base::CompoundStateSpace::StateType>()->
                as<base::SE3StateSpace::StateType>(index);
            return static_cast<const base::State*>(st);
        }

        /// @brief The number of independent rigid bodies to plan for
        unsigned int n_;
};

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
    MultiSE3RigidBodyPlanning setup(2);

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname.c_str());  // The first mesh should use setRobotMesh.
    setup.addRobotMesh(robot_fname.c_str());  // Subsequent robot meshes MUST use addRobotMesh!

    setup.setEnvironmentMesh(env_fname.c_str());

    // constructing start and goal states
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(setup.getSpaceInformation());
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(setup.getSpaceInformation());

    ompl::base::SE3StateSpace::StateType* start1 = start.get()->as<ompl::base::SE3StateSpace::StateType>(0);
    // define start state (robot 1)
    start1->setX(-4.96);
    start1->setY(-40.62);
    start1->setZ(70.57);
    start1->rotation().setIdentity();

    // define goal state (robot 1)
    ompl::base::SE3StateSpace::StateType* goal1 = goal.get()->as<ompl::base::SE3StateSpace::StateType>(0);
    goal1->setX(200.49);
    goal1->setY(-40.62);
    goal1->setZ(70.57);
    goal1->rotation().setIdentity();

    ompl::base::SE3StateSpace::StateType* start2 = start.get()->as<ompl::base::SE3StateSpace::StateType>(1);
    // define start state (robot 2)
    start2->setX(200.49);
    start2->setY(-40.62);
    start2->setZ(70.57);
    start2->rotation().setIdentity();

    // define goal state (robot 2)
    ompl::base::SE3StateSpace::StateType* goal2 = goal.get()->as<ompl::base::SE3StateSpace::StateType>(1);
    goal2->setX(-4.96);
    goal2->setY(-40.62);
    goal2->setZ(70.57);
    goal2->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // use RRT for planning
    setup.setPlanner (base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    // try to solve the problem
    /*    if (setup.solve(30))
          {
          if (setup.haveExactSolutionPath())
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
    }*/

    std::string benchmark_name("Cubicles");
    double runtime_limit=1000, memory_limit=8000;
    int run_count=30;
    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    // optionally set pre & pos run events
    //b.setPreRunEvent(boost::bind(&preRunEvent, _1));
    //b.setPostRunEvent(boost::bind(&postRunEvent, _1, _2));

    b.addPlanner(base::PlannerPtr(new geometric::RRTConnect(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::BKPIECE1(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::LBKPIECE1(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::KPIECE1(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::SBL(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    base::PlannerPtr p(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,3));
    p->setName("GNAT-3");
    b.addPlanner(p);
    base::PlannerPtr p1(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,6));
    p1->setName("GNAT-6");
    b.addPlanner(p1);
    base::PlannerPtr p2(new geometric::GNAT(setup.getSpaceInformation(),false,16,12,18,6,12));
    p2->setName("GNAT-12");
    b.addPlanner(p2);

    // run all planners with a obstacle-based valid state sampler on the benchmark problem
    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocObstacleStateSampler);
    b.setExperimentName(benchmark_name);
    b.benchmark(request);
    b.saveResultsToFile("Cubicles-Multi-full.log");
}
