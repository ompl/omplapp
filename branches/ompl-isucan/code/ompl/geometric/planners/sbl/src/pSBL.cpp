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

#include "ompl/geometric/planners/sbl/pSBL.h"
#include "ompl/base/GoalState.h"
#include <boost/thread.hpp>
#include <limits>
#include <cassert>

void ompl::geometric::pSBL::setup(void)
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
    m_tStart.grid.setDimension(m_projectionEvaluator->getDimension());
    m_tGoal.grid.setDimension(m_projectionEvaluator->getDimension());
}

void ompl::geometric::pSBL::clear(void)
{
    freeMemory();
    
    m_tStart.grid.clear();
    m_tStart.size = 0;
    
    m_tGoal.grid.clear();
    m_tGoal.size = 0;
    
    m_removeList.motions.clear();
    
    m_addedStartStates = 0;
}

void ompl::geometric::pSBL::freeGridMotions(Grid<MotionSet> &grid)
{
    for (Grid<MotionSet>::iterator it = grid.begin(); it != grid.end() ; ++it)
	for (unsigned int i = 0 ; i < it->second->data.size() ; ++i)
	{
	    if (it->second->data[i]->state)
		m_si->freeState(it->second->data[i]->state);
	    delete it->second->data[i];
	}
}

void ompl::geometric::pSBL::threadSolve(unsigned int tid, time::point endTime, SolutionInfo *sol)
{   
    base::GoalState *goal = static_cast<base::GoalState*>(m_pdef->getGoal().get());
    
    std::vector<Motion*> solution;
    base::State *xstate = m_si->allocState();
    bool      startTree = m_sCoreArray[tid]->getRNG().uniformBool();
    
    while (!sol->found && time::now() < endTime)
    {
	bool retry = true;
	while (retry && !sol->found && time::now() < endTime)
	{
	    m_removeList.lock.lock();
	    if (!m_removeList.motions.empty())
	    {
		if (m_loopLock.try_lock())
		{
		    retry = false;
		    std::map<Motion*, bool> seen;
		    for (unsigned int i = 0 ; i < m_removeList.motions.size() ; ++i)
			if (seen.find(m_removeList.motions[i].motion) == seen.end())
			    removeMotion(*m_removeList.motions[i].tree, m_removeList.motions[i].motion, seen);
		    m_removeList.motions.clear();
		    m_loopLock.unlock();
		}
	    }
	    else
		retry = false;
	    m_removeList.lock.unlock();
	}
	
	if (sol->found || time::now() > endTime)
	    break;
	
	m_loopLockCounter.lock();
	if (m_loopCounter == 0)
	    m_loopLock.lock();
	m_loopCounter++;
	m_loopLockCounter.unlock();
	

	TreeData &tree      = startTree ? m_tStart : m_tGoal;
	startTree = !startTree;
	TreeData &otherTree = startTree ? m_tStart : m_tGoal;
	
	Motion *existing = selectMotion(m_sCoreArray[tid]->getRNG(), tree);
	m_sCoreArray[tid]->sampleNear(xstate, existing->state, m_maxDistance);
	
	/* create a motion */
	Motion *motion = new Motion(m_si);
	m_si->copyState(motion->state, xstate);
	motion->parent = existing;
	motion->root = existing->root;
	
	existing->lock.lock();
	existing->children.push_back(motion);
	existing->lock.unlock();
	
	addMotion(tree, motion);

	if (checkSolution(m_sCoreArray[tid]->getRNG(), !startTree, tree, otherTree, motion, solution))
	{
	    sol->lock.lock();
	    if (!sol->found)
	    {
		sol->found = true;
		PathGeometric *path = new PathGeometric(m_si);
		for (unsigned int i = 0 ; i < solution.size() ; ++i)
		    path->states.push_back(m_si->cloneState(solution[i]->state));
		goal->setDifference(0.0);
		goal->setSolutionPath(base::PathPtr(path));
	    }
	    sol->lock.unlock();
	}

	
	m_loopLockCounter.lock();
	m_loopCounter--;
	if (m_loopCounter == 0)
	    m_loopLock.unlock();
	m_loopLockCounter.unlock();
    }
    
    m_si->freeState(xstate);    
}

bool ompl::geometric::pSBL::solve(double solveTime)
{
    base::GoalState *goal = dynamic_cast<base::GoalState*>(m_pdef->getGoal().get());
    
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
	    motion->valid = true;
	    motion->root = st;
	    addMotion(m_tStart, motion);
	}
	else
	    m_msg.error("Initial state is invalid!");
    }
    
    if (m_tGoal.size == 0)
    {	   
	if (m_si->satisfiesBounds(goal->state) && m_si->isValid(goal->state))
	{
	    Motion *motion = new Motion(m_si);
	    m_si->copyState(motion->state, goal->state);
	    motion->valid = true;
	    motion->root = goal->state;
	    addMotion(m_tGoal, motion);
	}
	else
	    m_msg.error("Goal state is invalid!");
    }
    
    if (m_tStart.size == 0 || m_tGoal.size == 0)
    {
	m_msg.error("Motion planning trees could not be initialized!");
	return false;
    }
    
    m_msg.inform("Starting with %d states", (int)(m_tStart.size + m_tGoal.size));
    
    SolutionInfo sol;
    sol.found = false;
    m_loopCounter = 0;
    
    std::vector<boost::thread*> th(m_threadCount);
    for (unsigned int i = 0 ; i < m_threadCount ; ++i)
	th[i] = new boost::thread(boost::bind(&pSBL::threadSolve, this, i, endTime, &sol));
    for (unsigned int i = 0 ; i < m_threadCount ; ++i)
    {
	th[i]->join();
	delete th[i];
    }
        
    m_msg.inform("Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", m_tStart.size + m_tGoal.size, m_tStart.size, m_tGoal.size,
	     m_tStart.grid.size() + m_tGoal.grid.size(), m_tStart.grid.size(), m_tGoal.grid.size());
    
    return goal->isAchieved();
}

bool ompl::geometric::pSBL::checkSolution(RNG &rng, bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);

    otherTree.lock.lock();    
    Grid<MotionSet>::Cell* cell = otherTree.grid.getCell(coord);
    
    if (cell && !cell->data.empty())
    {
	Motion *connectOther = cell->data[rng.uniformInt(0, cell->data.size() - 1)];
	otherTree.lock.unlock();    
	
	if (m_pdef->getGoal()->isStartGoalPairValid(start ? motion->root : connectOther->root, start ? connectOther->root : motion->root))
	{
	    Motion *connect = new Motion(m_si);
	    
	    m_si->copyState(connect->state, connectOther->state);
	    connect->parent = motion;
	    connect->root = motion->root;
	    
	    motion->lock.lock();
	    motion->children.push_back(connect);
	    motion->lock.unlock();
	    
	    addMotion(tree, connect);
	    
	    if (isPathValid(tree, connect) && isPathValid(otherTree, connectOther))
	    {
		/* extract the motions and put them in solution vector */
		
		std::vector<Motion*> mpath1;
		while (motion != NULL)
		{
		    mpath1.push_back(motion);
		    motion = motion->parent;
		}
		
		std::vector<Motion*> mpath2;
		while (connectOther != NULL)
		{
		    mpath2.push_back(connectOther);
		    connectOther = connectOther->parent;
		}
		
		if (!start)
		    mpath1.swap(mpath2);
		
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
		    solution.push_back(mpath1[i]);
		solution.insert(solution.end(), mpath2.begin(), mpath2.end());
		
		return true;
	    }
	}
    }
    else
	otherTree.lock.unlock();    
    
    return false;
}

bool ompl::geometric::pSBL::isPathValid(TreeData &tree, Motion *motion)
{
    std::vector<Motion*> mpath;
    
    /* construct the solution path */
    while (motion != NULL)
    {  
	mpath.push_back(motion);
	motion = motion->parent;
    }
    
    bool result = true;
    
    /* check the path */
    for (int i = mpath.size() - 1 ; result && i >= 0 ; --i)
    {
	mpath[i]->lock.lock();
	if (!mpath[i]->valid)
	{
	    if (m_si->checkMotion(mpath[i]->parent->state, mpath[i]->state))
		mpath[i]->valid = true;
	    else
	    {
		// remember we need to remove this motion
		PendingRemoveMotion prm;
		prm.tree = &tree;
		prm.motion = mpath[i];
		m_removeList.lock.lock();
		m_removeList.motions.push_back(prm);
		m_removeList.lock.unlock();
		result = false;
	    }
	}
	mpath[i]->lock.unlock();
    }
    
    return result;
}

ompl::geometric::pSBL::Motion* ompl::geometric::pSBL::selectMotion(RNG &rng, TreeData &tree)
{
    double sum  = 0.0;
    Grid<MotionSet>::Cell* cell = NULL;
    tree.lock.lock();
    double prob = rng.uniform01() * (tree.grid.size() - 1);
    for (Grid<MotionSet>::iterator it = tree.grid.begin(); it != tree.grid.end() ; ++it)
    {
	sum += (double)(tree.size - it->second->data.size()) / (double)tree.size;
	if (prob < sum)
	{
	    cell = it->second;
	    break;
	}
    }
    if (!cell && tree.grid.size() > 0)
	cell = tree.grid.begin()->second;
    ompl::geometric::pSBL::Motion* result = cell->data[rng.uniformInt(0, cell->data.size() - 1)];
    tree.lock.unlock();
    return result;
}

void ompl::geometric::pSBL::removeMotion(TreeData &tree, Motion *motion, std::map<Motion*, bool> &seen)
{
    /* remove from grid */
    seen[motion] = true;

    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
    Grid<MotionSet>::Cell* cell = tree.grid.getCell(coord);
    if (cell)
    {
	for (unsigned int i = 0 ; i < cell->data.size(); ++i)
	    if (cell->data[i] == motion)
	    {
		cell->data.erase(cell->data.begin() + i);
		tree.size--;
		break;
	    }
	if (cell->data.empty())
	{
	    tree.grid.remove(cell);
	    tree.grid.destroyCell(cell);
	}
    }
    
    /* remove self from parent list */
    
    if (motion->parent)
    {
	for (unsigned int i = 0 ; i < motion->parent->children.size() ; ++i)
	    if (motion->parent->children[i] == motion)
	    {
		motion->parent->children.erase(motion->parent->children.begin() + i);
		break;
	    }
    }    
    
    /* remove children */
    for (unsigned int i = 0 ; i < motion->children.size() ; ++i)
    {
	motion->children[i]->parent = NULL;
	removeMotion(tree, motion->children[i], seen);
    }

    if (motion->state)
	m_si->freeState(motion->state);
    delete motion;
}

void ompl::geometric::pSBL::addMotion(TreeData &tree, Motion *motion)
{
    Grid<MotionSet>::Coord coord;
    m_projectionEvaluator->computeCoordinates(motion->state, coord);
    tree.lock.lock();
    Grid<MotionSet>::Cell* cell = tree.grid.getCell(coord);
    if (cell)
	cell->data.push_back(motion);
    else
    {
	cell = tree.grid.createCell(coord);
	cell->data.push_back(motion);
	tree.grid.add(cell);
    }
    tree.size++;
    tree.lock.unlock();
}

void ompl::geometric::pSBL::getPlannerData(base::PlannerData &data) const
{
    data.states.resize(0);
    data.states.reserve(m_tStart.size + m_tGoal.size);
    
    std::vector<MotionSet> motions;
    m_tStart.grid.getContent(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
	    data.states.push_back(motions[i][j]->state);    

    motions.clear();
    m_tGoal.grid.getContent(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
	    data.states.push_back(motions[i][j]->state);    
}

void ompl::geometric::pSBL::setThreadCount(unsigned int nthreads)
{
    assert(nthreads > 0);		
    m_threadCount = nthreads;
    m_sCoreArray.resize(m_threadCount);
}
