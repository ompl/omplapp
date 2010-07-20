#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    /// cast the abstract state type to the type we expect
    const ob::SE3StateManifold::StateType *se3state = state->as<ob::SE3StateManifold::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateManifold::StateType *pos = se3state->as<ob::RealVectorStateManifold::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const ob::QuaternionStateManifold::StateType *rot = se3state->as<ob::QuaternionStateManifold::StateType>(1);
    
    /// check validity of state defined by pos & rot
    
    return true;
}

int main(int argc, char **argv)
{
    /// construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::SE3StateManifold());

    /// set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    
    manifold->as<ob::SE3StateManifold>()->setBounds(bounds);
    
    /// construct an instance of  space information from this manifold
    ob::SpaceInformationPtr si(new ob::SpaceInformation(manifold));

    /// set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));

    /// perform setup steps for this space; this also checks all settings are correct
    si->setup();
    
    /// print the settings for this space
    si->printSettings(std::cout);

    /// create a random start state
    ob::ScopedState<> start(manifold);
    start.random();

    /// create a random goal state
    ob::ScopedState<> goal(manifold);
    goal.random();
    
    /// create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    /// set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    
    /// print the problem settings
    pdef->print(std::cout);
    
    /// create a planner for the defined space
    ob::PlannerPtr planner(new og::RRTConnect(si));

    /// set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    /// perform setup steps for the planner
    planner->setup();
    
    /// attempt to solve the problem within one second of planning time
    bool solved = planner->solve(1.0);

    if (solved)
    {
	/// get the goal representation from the problem definition (not the same as the goal state)
	/// and inquire about the found path
	ob::PathPtr path = pdef->getGoal()->getSolutionPath();
	std::cout << "Found solution:" << std::endl;

	/// print the path to screen
	path->print(std::cout);
    }
    else
	std::cout << "No solution found" << std::endl;
    
    return 0;
}
