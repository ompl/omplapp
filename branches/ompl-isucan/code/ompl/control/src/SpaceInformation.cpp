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

#include "ompl/control/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <utility>
#include <limits>

void ompl::control::SpaceInformation::setup(void)
{
    base::SpaceInformation::setup();
    if (m_minSteps > m_maxSteps)
	throw Exception("The minimum number of steps cannot be larger than the maximum number of steps");
    if (m_minSteps == 0 && m_maxSteps == 0)
    {
	m_minSteps = 1;
	m_maxSteps = 2;
	m_msg.warn("Assuming propagation will always have 1 or 2 steps");
    }
    if (m_minSteps < 1)
	throw Exception("The minimum number of steps must be at least 1");
    
    if (m_controlManifold->getDimension() <= 0)
	throw Exception("The dimension of the control manifold we plan in must be > 0");
    
    if (m_stepSize < std::numeric_limits<double>::round_error())
    {
	m_stepSize = m_resolution;
	m_msg.warn("The propagation step size is assumed to be the same as the state validity checking resolution: %f", m_stepSize);
    }
}

unsigned int ompl::control::SpaceInformation::propagate(const base::State *state, const Control* control, unsigned int steps, base::State *result, bool stopBeforeInvalid) const
{
    if (steps == 0)
    {
	copyState(result, state);
	return 0;
    }
    if (stopBeforeInvalid)
    {
	// perform the first step of propagation
	PropagationResult pr = m_controlManifold->propagate(state, control, m_stepSize, result);
	
	// if the propagator we use cannot tell state validity, we ignore the PropagationResult
	if (pr == PROPAGATION_START_UNKNOWN)
	{
	    // if we found a valid state after one step, we can go on
	    if (isValid(result))
	    {
		base::State *temp1 = result;
		base::State *temp2 = allocState();
		base::State *toDelete = temp2;
		unsigned int r = steps;

		// for the remaining number of steps
		for (unsigned int i = 1 ; i < steps ; ++i)
		{
		    m_controlManifold->propagate(temp1, control, m_stepSize, temp2);
		    if (isValid(temp2))
			std::swap(temp1, temp2);
		    else
		    {
			// the last valid state is temp1; make sure result contains that state
			if (temp1 != result)
			    copyState(result, temp1);
			r = i;
			break;
		    }
		}

		// if we finished the for-loop without finding an invalid state, the last valid state is temp2
		// make sure result contains that information
		if (r == steps && result != temp2)
		    copyState(result, temp2);
		
		// free the temporary memory
		freeState(toDelete);
		
		return r;
	    }
	    // if the first propagation step produced an invalid step, return 0 steps
	    // the last valid state is the starting one (assumed to be valid)
	    else
	    {
		copyState(result, state);
		return 0;
	    }
	}
	// if it looks like the employed propagator CAN tell state validity
	else
	{
	    // it is assumed the starting state is valid
	    assert(pr == PROPAGATION_START_VALID);
	    
	    base::State *temp1 = allocState();
	    base::State *temp2 = result;
	    base::State *temp3 = allocState();
	    unsigned int r = steps;
	    
	    // for the remaining number of steps
	    for (unsigned int i = 1 ; i < steps ; ++i)
	    {
		pr = m_controlManifold->propagate(temp2, control, m_stepSize, temp3);
		
		// compute the validity of temp2
		bool valid = pr == PROPAGATION_START_UNKNOWN ? isValid(temp2) : pr == PROPAGATION_START_VALID;
		
		if (valid)
		{
		    std::swap(temp1, temp2);
		    std::swap(temp2, temp3);
		}
		else
		{
		    // we found temp2 to be invalid; the valid state before temp2 is temp1;
		    // if however we are at the first step, temp1 is not yet initialized
		    // and the correct value of result should be the start state
		    if (temp1 != result)
			copyState(result, i == 1 ? state : temp1);
		    r = i - 1;
		    break;
		}
	    }
	    
	    // if we finished the for-loop without finding an invalid state, 
	    // we need to check the final state for validity (this is temp2, because of the swap that gets executed)
	    if (r == steps)
	    {
		if (isValid(temp2))
		{	
		    if (result != temp2)
			copyState(result, temp2);
		}
		else
		{
		    r--;
		    if (result != temp1)
			copyState(result, temp1);
		}
	    }
	    
	    // free the temporary memory
	    if (temp1 != result)
		freeState(temp1);
	    if (temp2 != result)
		freeState(temp2);
	    if (temp3 != result)
		freeState(temp3);
	    
	    return r;
	}
    }
    // if we simply need to propagate forward, we do so
    else
    {
	m_controlManifold->propagate(state, control, m_stepSize, result);
        for (unsigned int i = 1 ; i < steps ; ++i)
	    m_controlManifold->propagate(result, control, m_stepSize, result);
	return steps;
    }
}

unsigned int ompl::control::SpaceInformation::propagate(const base::State *state, const Control* control, unsigned int steps, std::vector<base::State*> &result, bool includeStart, bool stopBeforeInvalid, bool alloc) const
{   
    if (alloc)
    {
	result.resize(steps + (includeStart ? 1 : 0));
	if (includeStart)
	    result[0] = allocState();
    }
    else
    {
	if (result.empty())
	    return 0;
	steps = std::min(steps, (unsigned int)result.size() - (includeStart ? 1 : 0));
    }
    
    unsigned int st = 0;
    if (includeStart)
    {
	st++;
	steps++;
	copyState(result[0], state);
    }
    
    if (stopBeforeInvalid)
    {
	
	if (st < steps)
	{
	    if (alloc)
		result[st] = allocState();
	    PropagationResult pr = m_controlManifold->propagate(state, control, m_stepSize, result[st]);
	    st++;
	    
	    if (pr == PROPAGATION_START_UNKNOWN)
	    {
		if (isValid(result[st-1]))
		{
		    while (st < steps)
		    {
			if (alloc)
			    result[st] = allocState();
			m_controlManifold->propagate(result[st-1], control, m_stepSize, result[st]);
			st++;
			if (!isValid(result[st-1]))
			{
			    if (alloc)
			    {
				freeState(result[st-1]);
				result.resize(st);
			    }
			    break;
			}
		    }
		}
		else
		{
		    if (alloc)
		    {
			freeState(result[st-1]);
			result.resize(st);
		    }
		}
	    }
	    else
	    {
		// we know pr = PROPAGATION_START_VALID at this point
		while (st < steps)
		{
		    if (alloc)
			result[st] = allocState();
		    
		    PropagationResult pr = m_controlManifold->propagate(result[st-1], control, m_stepSize, result[st]);
		    bool valid = pr == PROPAGATION_START_UNKNOWN ? isValid(result[st-1]) : pr == PROPAGATION_START_VALID;
		    
		    if (!valid)
		    {
			// the state at st-2 is the last valid one
			if (alloc)
			{
			    freeState(result[st]);
			    freeState(result[st-1]);
			    result.resize(st - 1);
			}
			st--;
			break;
		    }
		    st++;
		}
		if (st == steps)
		{
		    if (!isValid(result[st-1]))
		    {
			st--;
			if (alloc)
			{
			    freeState(result[st]);
			    result.resize(st);
			}
		    }
		}
	    }
	}
    }
    else
    {
	if (st < steps)
	{ 
	    if (alloc)
		result[st] = allocState();
	    m_controlManifold->propagate(state, control, m_stepSize, result[st]);
	    st++;
	    
	    while (st < steps)
	    {
		if (alloc)
		    result[st] = allocState();
		m_controlManifold->propagate(result[st-1], control, m_stepSize, result[st]);
		st++;	    
	    }
	}
    }
    
    return st;
}

void ompl::control::SpaceInformation::printSettings(std::ostream &out) const
{
    base::SpaceInformation::printSettings(out);
    out << "  - control manifold:" << std::endl;
    m_controlManifold->printSettings(out);
    out << "  - propagation step size: " << m_stepSize << std::endl;
    out << "  - propagation duration: [" << m_minSteps << ", " << m_maxSteps << "]" << std::endl;
}

