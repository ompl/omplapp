/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "ompl/base/GoalState.h"
#include "ompl/base/extension/RealVectorManifold.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/SimpleSetup.h"

#include "../../resources/config.h"
#include "environment2D.h"
#include <iostream>
#include <libgen.h>

using namespace ompl;

static const double SOLUTION_TIME = 1.0;

/** \brief Declare a class used in validating states. Such a class
    definition is needed for any use of a kinematic planner */
class myStateValidityChecker : public base::StateValidityChecker
{
public:

    myStateValidityChecker(base::SpaceInformation *si, const std::vector< std::vector<int> > &grid) :
	base::StateValidityChecker(si), m_grid(grid)
    {
    }
    
    virtual bool isValid(const base::State *state) const
    {
	const ext::RealVectorState *rstate = static_cast<const ext::RealVectorState*>(state);
	
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x = (int)(rstate->values[0]);
	int y = (int)(rstate->values[1]);
	return m_grid[x][y] == 0; // 0 means valid state
    }
    
protected:
    
    std::vector< std::vector<int> > m_grid;

};

class myManifold : public ext::RealVectorManifold
{
public:
    
    myManifold(unsigned int dim) : ext::RealVectorManifold(dim)
    {
    }
    
    virtual double distance(const base::State *state1, const base::State *state2) const
    {
	const ext::RealVectorState *rstate1 = static_cast<const ext::RealVectorState*>(state1);
	const ext::RealVectorState *rstate2 = static_cast<const ext::RealVectorState*>(state2);

	int x1 = (int)(rstate1->values[0]);
	int y1 = (int)(rstate1->values[1]);
	
	int x2 = (int)(rstate2->values[0]);
	int y2 = (int)(rstate2->values[1]);

	return abs(x1 - x2) + abs(y1 - y2);
    }
};

class mySetup
{
public:
    
    mySetup(Environment2D &env, const base::PlannerAllocator &pa) : setup(base::ManifoldPtr(new myManifold(2)), pa)
    {
	ext::RealVectorBounds bounds(2);
	
	std::vector<double> lowBound, upBound;
	bounds.low[0] = 0.0;
	bounds.low[1] = 0.0;
	bounds.high[0] = (double)env.width - 0.000000001;
	bounds.high[1] = (double)env.height - 0.000000001;
	
	static_cast<ext::RealVectorManifold*>(setup.getSpaceInformation()->getManifold().get())->setBounds(bounds);
	setup.setStateValidityChecker(base::StateValidityCheckerPtr(new myStateValidityChecker(setup.getSpaceInformation().get(), env.grid)));

	setup.getPathSimplifier()->setMaxSteps(50);
	setup.getPathSimplifier()->setMaxEmptySteps(10);
	setup.getSpaceInformation()->setStateValidityCheckingResolution(1.0);
	
	setup.getSpaceInformation()->printSettings();
	
	/* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
	base::ScopedState<ext::RealVectorState> state(setup.getSpaceInformation());
	state->values[0] = env.start.first;
	state->values[1] = env.start.second;
	setup.getProblemDefinition()->addStartState(state);

	base::GoalState *goal = new base::GoalState(setup.getSpaceInformation());
	goal->state = setup.getSpaceInformation()->allocState();
	static_cast<ext::RealVectorState*>(goal->state)->values[0] = env.goal.first;
	static_cast<ext::RealVectorState*>(goal->state)->values[1] = env.goal.second;
	goal->threshold = 1e-3; // this is basically 0, but we want to account for numerical instabilities 

	setup.setGoal(base::GoalPtr(goal));
    }
    
    geometric::SimpleSetup* operator->(void)
    {
	return &setup;
    }

private:
    
    geometric::SimpleSetup setup;
};


/** Define a function that constructs planner instances */
base::PlannerPtr allocPlanner(const base::SpaceInformationPtr &si)
{
    geometric::RRT *rrt = new geometric::RRT(si);
    rrt->setRange(10.0);
    return base::PlannerPtr(rrt);
}




/** A base class for testing planners */
class TestPlanner
{
public:
    TestPlanner(void)
    {
    }
    
    virtual ~TestPlanner(void)
    {
    }
    
    virtual bool execute(Environment2D &env, bool show = false, double *time = NULL, double *pathLength = NULL)
    {	 
	bool result = true;
	
	mySetup setup(env, boost::bind(&allocPlanner, _1));
 
	/* start counting time */
	ompl::time::point startTime = ompl::time::now();	
	
	/* call the planner to solve the problem */
	if (setup->solve(SOLUTION_TIME))
	{
	    ompl::time::duration elapsed = ompl::time::now() - startTime;
	    if (time)
		*time += ompl::time::seconds(elapsed);
	    if (show)
		printf("Found solution in %f seconds!\n", ompl::time::seconds(elapsed));
	    
	    geometric::PathGeometric &path = setup->getSolutionPath();
	    
	    /* make the solution more smooth */
	    
	    startTime = ompl::time::now();
	    setup->getPathSimplifier()->reduceVertices(path);
	    elapsed = ompl::time::now() - startTime;

	    if (time)
		*time += ompl::time::seconds(elapsed);
	    
	    if (show)
		printf("Simplified solution in %f seconds!\n", ompl::time::seconds(elapsed));

	    /* fill in values that were linearly interpolated */
	    path.interpolate();
	    
	    if (pathLength)
		*pathLength += path.length();

	    if (show)
	    {
		printEnvironment(std::cout, env);
		std::cout << std::endl;	    
	    }
	    
	    Environment2D temp = env;
	    /* display the solution */	    
	    for (unsigned int i = 0 ; i < path.states.size() ; ++i)
	    {
		int x = (int)(static_cast<ext::RealVectorState*>(path.states[i])->values[0]);
		int y = (int)(static_cast<ext::RealVectorState*>(path.states[i])->values[1]);
		if (temp.grid[x][y] == T_FREE || temp.grid[x][y] == T_PATH)
		    temp.grid[x][y] = T_PATH;
		else
		{
		    temp.grid[x][y] = T_ERROR;
		    result = false;
		}		
	    }
	    
	    if (show)
		printEnvironment(std::cout, temp);
	}
	else
	    result = false;
	
	return result;
    }    
};

class PlanTest : public testing::Test
{
public:
    
    void runPlanTest(TestPlanner *p, double *success, double *avgruntime, double *avglength)
    {    
	double time   = 0.0;
	double length = 0.0;
	int    good   = 0;
	int    N      = 100;

	for (int i = 0 ; i < N ; ++i)
	    if (p->execute(env, false, &time, &length))
		good++;
	
	*success    = 100.0 * (double)good / (double)N;
	*avgruntime = time / (double)N;
	*avglength  = length / (double)N;

	if (verbose)
	{
	    printf("    Success rate: %f%%\n", *success);
	    printf("    Average runtime: %f\n", *avgruntime);
	    printf("    Average path length: %f\n", *avglength);
	}
    }
    
protected:
    
    PlanTest(void) 
    {
	verbose = true;
    }
    
    void SetUp(void)
    {
	/* load environment */
	boost::filesystem::path path(TEST_RESOURCES_DIR);
	path = path / "env1.txt";
	loadEnvironment(path.string().c_str(), env);
	
	if (env.width * env.height == 0)
	{
	    std::cerr << "The environment has a 0 dimension. Cannot continue" << std::endl;
	    FAIL();	    
	}
    }
    
    void TearDown(void)
    {
    }

    Environment2D env;
    bool          verbose;
};

TEST_F(PlanTest, geometric_RRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner p;
    runPlanTest(&p, &success, &avgruntime, &avglength);
    
    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.01);
    EXPECT_TRUE(avglength < 70.0);
}

TEST(ScopedStateTest, Simple)
{
    base::ManifoldPtr m(new ext::RealVectorManifold(2));
    
    base::ScopedState<ext::RealVectorManifold::StateType> s1(m);
    s1->values[0] = 1.0;
    s1->values[1] = 2.0;
    
    base::ScopedState<ext::RealVectorManifold::StateType> s2 = s1;
    EXPECT_TRUE(s2->values[1] == s1->values[1]);
    
    base::ScopedState<> s3 = s1;
    EXPECT_TRUE(s2 == s3);
    
    base::ScopedState<> s4 = s3;

    base::ScopedState<ext::RealVectorManifold::StateType> s5 = s4;
    EXPECT_TRUE(s5 == s1);

    s1->values[1] = 4.0;
    
    EXPECT_TRUE(s5 != s1);
    
    base::ScopedState<ext::RealVectorManifold::StateType> s6(s4);
    EXPECT_TRUE(s6 != s1);
    s1 = s4;
    s4 = s1;
    EXPECT_TRUE(s6 == s1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
