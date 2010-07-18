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

#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/base/GoalSampleableRegion.h"

void ompl::geometric::RRTConnect::setup(void)
{
    Planner::setup();
    if (m_maxDistance < std::numeric_limits<double>::epsilon())
    {
	m_maxDistance = m_si->getStateValidityCheckingResolution() * 10.0;
	m_msg.warn("Maximum motion extension distance is %f", m_maxDistance);
    }
}

void ompl::geometric::RRTConnect::freeMemory(void)
{
    std::vector<Motion*> motions;
    m_tStart.list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
	if (motions[i]->state)
	    m_si->freeState(motions[i]->state);
	delete motions[i];
    }
    
    m_tGoal.list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
	if (motions[i]->state)
	    m_si->freeState(motions[i]->state);
	delete motions[i];
    }
}

void ompl::geometric::RRTConnect::clear(void)
{
    freeMemory();
    m_tStart.clear();
    m_tGoal.clear();
    m_addedStartStates = 0;
    m_sampledGoalsCount = 0;
}

ompl::geometric::RRTConnect::GrowState ompl::geometric::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree.nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;
    
    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = m_si->distance(nmotion->state, rmotion->state);
    if (d > m_maxDistance)
    {
	m_si->getStateManifold()->interpolate(nmotion->state, rmotion->state, m_maxDistance / d, tgi.xstate);
	dstate = tgi.xstate;
	reach = false;
    }

    if (m_si->checkMotion(nmotion->state, dstate))
    {
	/* create a motion */
	Motion *motion = new Motion(m_si);
	m_si->copyState(motion->state, dstate);
	motion->parent = nmotion;
	motion->root = nmotion->root;
	tgi.xmotion = motion;
	
	tree.add(motion);
	if (reach)
	    return REACHED;
	else
	    return ADVANCED;
    }
    else
	return TRAPPED;
}

bool ompl::geometric::RRTConnect::solve(double solveTime)
{
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(m_pdef->getGoal().get());
    
    if (!goal)
    {
	m_msg.error("Unknown type of goal (or goal undefined)");
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
	    motion->root = st;
	    m_tStart.add(motion);
	}
	else
	    m_msg.error("Initial state is invalid!");
    }    
    
    if (m_tStart.size() == 0)
    {
	m_msg.error("Motion planning start tree could not be initialized!");
	return false;
    }

    if (goal->maxSampleCount() <= 0)
    {
	m_msg.error("Insufficient states in sampleable goal region");
	return false;
    }
    
    m_msg.inform("Starting with %d states", (int)(m_tStart.size() + m_tGoal.size()));

    TreeGrowingInfo tgi;
    tgi.xstate = m_si->allocState();
    
    Motion   *rmotion   = new Motion(m_si);
    base::State *rstate = rmotion->state;
    base::State *gstate = m_si->allocState();
    bool   startTree    = true;

    while (time::now() < endTime)
    {
	TreeData &tree      = startTree ? m_tStart : m_tGoal;
	startTree = !startTree;
	TreeData &otherTree = startTree ? m_tStart : m_tGoal;
		
	// if there are any goals left to sample
	if (m_sampledGoalsCount < goal->maxSampleCount())
	{
	    // if we have not sampled too many goals already
	    if (m_tGoal.size() == 0 || m_sampledGoalsCount < m_tGoal.size() / 2)
	    {
		bool firstAttempt = true;
		
		while ((m_tGoal.size() == 0 || firstAttempt) && m_sampledGoalsCount < goal->maxSampleCount() && time::now() < endTime)
		{
		    firstAttempt = false;
		    goal->sampleGoal(gstate);
		    m_sampledGoalsCount++;
		    if (m_si->satisfiesBounds(gstate) && m_si->isValid(gstate))
		    {
			Motion* motion = new Motion(m_si);
			m_si->copyState(motion->state, gstate);
			motion->root = motion->state;
			m_tGoal.add(motion);
		    }
		}
		
		if (m_tGoal.size() == 0)
		{
		    m_msg.error("Unable to sample any valid states for goal tree");
		    break;
		}
	    }
	}
	
	/* sample random state */
	m_sCore->sample(rstate);
	
	GrowState gs = growTree(tree, tgi, rmotion);
	
	if (gs != TRAPPED)
	{
	    /* remember which motion was just added */
	    Motion *addedMotion = tgi.xmotion;
	    
	    /* attempt to connect trees */
	    
	    /* if reached, it means we used rstate directly, no need top copy again */
	    if (gs != REACHED)
		m_si->copyState(rstate, tgi.xstate);

	    GrowState gsc = ADVANCED;
	    while (gsc == ADVANCED)
		gsc = growTree(otherTree, tgi, rmotion);

	    /* if we connected the trees in a valid way (start and goal pair is valid)*/
	    if (gsc == REACHED && goal->isStartGoalPairValid(startTree ? tgi.xmotion->root : addedMotion->root,
							     startTree ? addedMotion->root : tgi.xmotion->root))
	    {
		/* construct the solution path */
		Motion *solution = tgi.xmotion;
		std::vector<Motion*> mpath1;
		while (solution != NULL)
		{
		    mpath1.push_back(solution);
		    solution = solution->parent;
		}
		
		solution = addedMotion;
		std::vector<Motion*> mpath2;
		while (solution != NULL)
		{
		    mpath2.push_back(solution);
		    solution = solution->parent;
		}
		
		if (!startTree)
		    mpath2.swap(mpath1);

		std::vector<Motion*> sol;
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
		    sol.push_back(mpath1[i]);
		sol.insert(sol.end(), mpath2.begin(), mpath2.end());

		PathGeometric *path = new PathGeometric(m_si);
		for (unsigned int i = 0 ; i < sol.size() ; ++i)
		    path->states.push_back(m_si->cloneState(sol[i]->state));
		
		goal->setDifference(0.0);
		goal->setSolutionPath(base::PathPtr(path));
		break;
	    }
	}
    }
    
    m_si->freeState(tgi.xstate);
    m_si->freeState(rstate);
    delete rmotion;
    m_si->freeState(gstate);
    
    m_msg.inform("Created %u states (%u start + %u goal)", m_tStart.size() + m_tGoal.size(), m_tStart.size(), m_tGoal.size());
    
    return goal->isAchieved();
}

void ompl::geometric::RRTConnect::getPlannerData(base::PlannerData &data) const
{
    std::vector<Motion*> motions;
    m_tStart.list(motions);
    data.states.resize(motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	data.states[i] = motions[i]->state;
    m_tGoal.list(motions);
    unsigned int s = data.states.size();
    data.states.resize(s + motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	data.states[s + i] = motions[i]->state;
}
