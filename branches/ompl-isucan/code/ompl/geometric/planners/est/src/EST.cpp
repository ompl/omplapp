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

#include "ompl/geometric/planners/est/EST.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>
#include <cassert>

void ompl::geometric::EST::setup(void)
{
    Planner::setup();
    if (!m_projectionEvaluator)
	throw Exception("No projection evaluator specified");
    m_projectionEvaluator->checkCellDimensions();
    if (m_projectionEvaluator->getDimension() <= 0)
	throw Exception("Dimension of projection needs to be larger than 0");
    if (m_maxDistance < std::numeric_limits<double>::epsilon())
    {
	m_maxDistance = m_si->getStateValidityCheckingResolution() * 10.0;
	m_msg.warn("Maximum motion extension distance is %f", m_maxDistance);
    }
    m_tree.grid.setDimension(m_projectionEvaluator->getDimension());
}

void ompl::geometric::EST::clear(void)
{
    freeMemory();
    m_tree.grid.clear();
    m_tree.size = 0;
    m_addedStartStates = 0;
}

void ompl::geometric::EST::freeMemory(void)
{
    for (Grid<MotionSet>::iterator it = m_tree.grid.begin(); it != m_tree.grid.end() ; ++it)
    {
	for (unsigned int i = 0 ; i < it->second->data.size() ; ++i)
	{
	    if (it->second->data[i]->state)
		m_si->freeState(it->second->data[i]->state);
	    delete it->second->data[i];
	}
    }
}

bool ompl::geometric::EST::solve(double solveTime)
{
    base::Goal                   *goal = m_pdef->getGoal().get();
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
	    addMotion(motion);
	}
	else
	    m_msg.error("Initial state is invalid!");
    }
    
    if (m_tree.grid.size() == 0)
    {
	m_msg.error("There are no valid initial states!");
	return false;	
    }    

    m_msg.inform("Starting with %u states", m_tree.size);
        
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = m_si->allocState();
    
    while (time::now() < endTime)
    {
	/* Decide on a state to expand from */
	Motion *existing = selectMotion();
	assert(existing);
	
	/* sample random state (with goal biasing) */
	if (goal_s && m_rng.uniform01() < m_goalBias)
	    goal_s->sampleGoal(xstate);
	else
	    m_sCore->sampleNear(xstate, existing->state, m_maxDistance);
	
	if (m_si->checkMotion(existing->state, xstate))
	{
	    /* create a motion */
	    Motion *motion = new Motion(m_si);
	    m_si->copyState(motion->state, xstate);
	    motion->parent = existing;
	    
	    addMotion(motion);
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
    
    m_msg.inform("Created %u states in %u cells", m_tree.size, m_tree.grid.size());
    
    return goal->isAchieved();
}

ompl::geometric::EST::Motion* ompl::geometric::EST::selectMotion(void)
{
    double sum  = 0.0;
    Grid<MotionSet>::Cell* cell = NULL;
    double prob = m_rng.uniform01() * (m_tree.grid.size() - 1);
    for (Grid<MotionSet>::iterator it = m_tree.grid.begin(); it != m_tree.grid.end() ; ++it)
    {
	sum += (double)(m_tree.size - it->second->data.size()) / (double)m_tree.size;
	if (prob < sum)
	{
	    cell = it->second;
	    break;
	}
    }
    if (!cell && m_tree.grid.size() > 0)
	cell = m_tree.grid.begin()->second;
    return cell && !cell->data.empty() ? cell->data[m_rng.uniformInt(0, cell->data.size() - 1)] : NULL;
}

void ompl::geometric::EST::addMotion(Motion *motion)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
    Grid<MotionSet>::Cell* cell = m_tree.grid.getCell(coord);
    if (cell)
	cell->data.push_back(motion);
    else
    {
	cell = m_tree.grid.createCell(coord);
	cell->data.push_back(motion);
	m_tree.grid.add(cell);
    }
    m_tree.size++;
}

void ompl::geometric::EST::getPlannerData(base::PlannerData &data) const
{
    std::vector<MotionSet> motions;
    m_tree.grid.getContent(motions);
    data.states.resize(0);
    data.states.reserve(m_tree.size);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
	    data.states.push_back(motions[i][j]->state);
}
