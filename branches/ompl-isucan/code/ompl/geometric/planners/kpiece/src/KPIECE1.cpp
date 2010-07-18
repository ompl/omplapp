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

#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>
#include <cassert>

void ompl::geometric::KPIECE1::setup(void)
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

void ompl::geometric::KPIECE1::clear(void)
{
    freeMemory();
    m_tree.grid.clear();
    m_tree.size = 0;
    m_tree.iteration = 1;
    m_addedStartStates = 0;
}

void ompl::geometric::KPIECE1::freeMemory(void)
{
    freeGridMotions(m_tree.grid);
}

void ompl::geometric::KPIECE1::freeGridMotions(Grid &grid)
{
    for (Grid::iterator it = grid.begin(); it != grid.end() ; ++it)
	freeCellData(it->second->data);
}

void ompl::geometric::KPIECE1::freeCellData(CellData *cdata)
{
    for (unsigned int i = 0 ; i < cdata->motions.size() ; ++i)
	freeMotion(cdata->motions[i]);
    delete cdata;
}

void ompl::geometric::KPIECE1::freeMotion(Motion *motion)
{
    if (motion->state)
	m_si->freeState(motion->state);
    delete motion;
}

bool ompl::geometric::KPIECE1::solve(double solveTime)
{
    base::Goal                   *goal = m_pdef->getGoal().get();
    base::GoalRegion           *goal_r = dynamic_cast<base::GoalRegion*>(goal);
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
	    addMotion(motion, 1.0);
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
    
    double improveValue = m_maxDistance;

    while (time::now() < endTime)
    {
	m_tree.iteration++;
	
	/* Decide on a state to expand from */
	Motion     *existing = NULL;
	Grid::Cell *ecell = NULL;
	selectMotion(existing, ecell);
	assert(existing);
	
	/* sample random state (with goal biasing) */
	if (m_rng.uniform01() < m_goalBias)
	{
	    if (goal_s)
		goal_s->sampleGoal(xstate);
	    else
	    {
		if (approxsol && goal_r)
		{
		    m_si->copyState(xstate, approxsol->state);
		    m_msg.debug("Start Running HCIK (%f)...", improveValue);			
		    if (m_hcik.tryToImprove(*goal_r, xstate, improveValue))
			improveValue /= 2.0;
		    else
			m_sCore->sampleNear(xstate, existing->state, m_maxDistance);
		    m_msg.debug("End Running HCIK");			
		}
		else
		    m_sCore->sampleNear(xstate, existing->state, m_maxDistance);
	    }
	}
	else
	    m_sCore->sampleNear(xstate, existing->state, m_maxDistance);
	
	double failTime = 0.0;
	bool keep = m_si->checkMotion(existing->state, xstate, xstate, &failTime);
	if (!keep && failTime > m_minValidPathPercentage)
	    keep = true;
	
	if (keep)
	{
	    /* create a motion */
	    Motion *motion = new Motion(m_si);
	    m_si->copyState(motion->state, xstate);
	    motion->parent = existing;
	    
	    double dist = 0.0;
	    bool solved = goal->isSatisfied(motion->state, &dist);
	    addMotion(motion, dist);
	    
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
	    ecell->data->score *= m_goodScoreFactor;
	}
	else
	    ecell->data->score *= m_badScoreFactor;
	
	m_tree.grid.update(ecell);
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
    
    m_msg.inform("Created %u states in %u cells (%u internal + %u external)", m_tree.size, m_tree.grid.size(),
		 m_tree.grid.countInternal(), m_tree.grid.countExternal());
    
    return goal->isAchieved();
}

bool ompl::geometric::KPIECE1::selectMotion(Motion* &smotion, Grid::Cell* &scell)
{
    scell = m_rng.uniform01() < std::max(m_selectBorderPercentage, m_tree.grid.fracExternal()) ?
	m_tree.grid.topExternal() : m_tree.grid.topInternal();
    if (scell && !scell->data->motions.empty())
    {
	scell->data->selections++;
	smotion = scell->data->motions[m_rng.halfNormalInt(0, scell->data->motions.size() - 1)];
	return true;
    }
    else
	return false;
}

unsigned int ompl::geometric::KPIECE1::addMotion(Motion *motion, double dist)
{
    Grid::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
    Grid::Cell* cell = m_tree.grid.getCell(coord);
    unsigned int created = 0;
    if (cell)
    {
	cell->data->motions.push_back(motion);
	cell->data->coverage += 1.0;
	m_tree.grid.update(cell);
    }
    else
    {
	cell = m_tree.grid.createCell(coord);
	cell->data = new CellData();
	cell->data->motions.push_back(motion);
	cell->data->coverage = 1.0;
	cell->data->iteration = m_tree.iteration;
	cell->data->selections = 1;
	cell->data->score = 1.0 / (1e-3 + dist);
	m_tree.grid.add(cell);
	created = 1;
    }
    m_tree.size++;
    return created;
}

void ompl::geometric::KPIECE1::getPlannerData(base::PlannerData &data) const
{
    data.states.resize(0);
    data.states.reserve(m_tree.size);
    
    std::vector<CellData*> cdata;
    m_tree.grid.getContent(cdata);
    for (unsigned int i = 0 ; i < cdata.size() ; ++i)
	for (unsigned int j = 0 ; j < cdata[i]->motions.size() ; ++j)
	    data.states.push_back(cdata[i]->motions[j]->state); 
}
