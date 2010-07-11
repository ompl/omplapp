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

/* \author Ioan Sucan */

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>

void ompl::geometric::RRT::freeMemory(void)
{
    std::vector<Motion*> motions;
    m_nn.list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
	if (motions[i]->state)
	    m_si->freeState(motions[i]->state);
	delete motions[i];
    }
}

bool ompl::geometric::RRT::solve(double solveTime)
{
    base::Goal                 *goal   = m_pdef->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    
    if (!goal)
    {
	m_msg.error("Goal undefined");
	return false;
    }
    
    time::point endTime = time::now() + time::seconds(solveTime);

    for (unsigned int i = m_addedStartStates ; i < m_pdef->getStartStateCount() ; ++i, ++m_addedStartStates)
    {
	const base::State *st = m_pdef->getStartState(i);
	if (m_si->satisfiesBounds(st) && m_si->isValid(st))
	{
	    Motion *motion = new Motion(m_si);
	    m_si->copyState(motion->state, st);
	    m_nn.add(motion);
	}
	else
	    m_msg.error("Initial state is invalid!");
    }
    
    if (m_nn.size() == 0)
    {
	m_msg.error("There are no valid initial states!");
	return false;	
    }    

    m_msg.inform("Starting with %u states", m_nn.size());
    
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(m_si);
    base::State *rstate = rmotion->state;
    base::State *xstate = m_si->allocState();
    
    while (time::now() < endTime)
    {

	/* sample random state (with goal biasing) */
	if (goal_s && m_rng.uniform01() < m_goalBias)
	    goal_s->sampleGoal(rstate);
	else
	    m_sCore->sample(rstate);

	/* find closest state in the tree */
	Motion *nmotion = m_nn.nearest(rmotion);
	base::State *dstate = rstate;
	
	/* find state to add */
	double d = m_si->distance(nmotion->state, rmotion->state);
	if (d > m_maxDistance)
	{
	    m_si->getManifold()->interpolate(nmotion->state, rmotion->state, m_maxDistance / d, xstate);
	    dstate = xstate;
	}
	
	if (m_si->checkMotion(nmotion->state, dstate))
	{
	    /* create a motion */
	    Motion *motion = new Motion(m_si);
	    m_si->copyState(motion->state, dstate);
	    motion->parent = nmotion;
	    
	    m_nn.add(motion);
	    double dist = 0.0;
	    bool solved = goal->isSatisfied(motion->state, &dist);
	    if (solved)
	    {
		approxdif = dist;
		solution = motion;
		break;
	    }
	    if (dist < approxdif)
	    {
		approxdif = dist;
		approxsol = motion;
	    }
	}
    }
    
    bool approximate = false;
    if (solution == NULL)
    {
	solution = approxsol;
	approximate = true;
    }
    
    if (solution != NULL)
    {
	/* construct the solution path */
	std::vector<Motion*> mpath;
	while (solution != NULL)
	{
	    mpath.push_back(solution);
	    solution = solution->parent;
	}

	/* set the solution path */
	PathGeometric *path = new PathGeometric(m_si);
   	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	    path->states.push_back(m_si->cloneState(mpath[i]->state));
	goal->setDifference(approxdif);
	goal->setSolutionPath(base::PathPtr(path), approximate);

	if (approximate)
	    m_msg.warn("Found approximate solution");
    }

    m_si->freeState(xstate);
    if (rmotion->state)
	m_si->freeState(rmotion->state);
    delete rmotion;
	
    m_msg.inform("Created %u states", m_nn.size());
    
    return goal->isAchieved();
}

void ompl::geometric::RRT::getPlannerData(base::PlannerData &data) const
{
    std::vector<Motion*> motions;
    m_nn.list(motions);
    data.states.resize(motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	data.states[i] = motions[i]->state;
}
