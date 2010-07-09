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

#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/GoalState.h"
#include "ompl/base/GoalStates.h"

#include <sstream>

bool ompl::base::ProblemDefinition::hasStartState(const State *state, unsigned int *startIndex)
{
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
	if (m_si->equalStates(state, m_startStates[i]))
	{
	    if (startIndex)
		*startIndex = i;
	    return true;
	}
    return false;
}

bool ompl::base::ProblemDefinition::fixInvalidInputState(State *state, double dist, bool start, unsigned int attempts)
{ 
    bool result = false;

    bool b = m_si->satisfiesBounds(state);
    bool v = false;
    if (b)
    {
	v = m_si->isValid(state);
	if (!v)
	    m_msg.debug("%s state is not valid", start ? "Start" : "Goal");
    }
    else
	m_msg.debug("%s state is not within space bounds", start ? "Start" : "Goal");
    
    if (!b || !v)
    {
	std::stringstream ss;
	m_si->printState(state, ss);
	ss << " within distance " << dist;
	m_msg.debug("Attempting to fix %s state %s", start ? "start" : "goal", ss.str().c_str());
	
	State *temp = m_si->allocState();
	if (m_si->searchValidNearby(temp, state, dist, attempts))
	{
	    m_si->copyState(state, temp);
	    result = true;
	}
	else
	    m_msg.warn("Unable to fix %s state", start ? "start" : "goal");
	m_si->freeState(temp);
    }
    
    return result;    
}

bool ompl::base::ProblemDefinition::fixInvalidInputStates(double distStart, double distGoal, unsigned int attempts)
{
    bool result = true;
    
    // fix start states
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
	if (!fixInvalidInputState(m_startStates[i], distStart, true, attempts))
	    result = false;
    
    // fix goal state
    GoalState *goal = dynamic_cast<GoalState*>(m_goal.get());
    if (goal)
    {
	if (!fixInvalidInputState(goal->state, distGoal, false, attempts))
	    result = false;
    }

    // fix goal state
    GoalStates *goals = dynamic_cast<GoalStates*>(m_goal.get());
    if (goals)
    {
	for (unsigned int i = 0 ; i < goals->states.size() ; ++i)
	    if (!fixInvalidInputState(goals->states[i], distGoal, false, attempts))
		result = false;
    }
    
    return result;    
}

bool ompl::base::ProblemDefinition::isTrivial(unsigned int *startIndex, double *distance) const
{
    if (!m_goal)
    {
	m_msg.error("Goal undefined");
	return false;
    }
    
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
    {
	const State *start = m_startStates[i];
	if (start && m_si->isValid(start) && m_si->satisfiesBounds(start))
	{
	    double dist;
	    if (m_goal->isSatisfied(start, &dist))
	    {
		if (startIndex)
		    *startIndex = i;
		if (distance)
		    *distance = dist;
		return true;
	    }	    
	}
	else
	{
	    m_msg.error("Initial state is in collision!");
	}
    }
    
    return false;    
}

void ompl::base::ProblemDefinition::print(std::ostream &out) const
{
    out << "Start states:" << std::endl;
    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
	m_si->printState(m_startStates[i], out);
    if (m_goal)
	m_goal->print(out);
    else
	out << "Goal = NULL" << std::endl;
}
