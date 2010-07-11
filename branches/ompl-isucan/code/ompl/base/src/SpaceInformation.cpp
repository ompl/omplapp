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

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <utility>
#include <limits>
#include <cmath>
#include <cassert>

ompl::base::SpaceInformation::SpaceInformation(const ManifoldPtr &manifold) : m_manifold(manifold), m_resolution(0.0), m_setup(false)
{
    if (!m_manifold)
	throw Exception("Invalid manifold definition");
}

void ompl::base::SpaceInformation::setup(void)
{
    if (m_setup)
	m_msg.warn("Space information setup called multiple times");
    
    if (!m_stateValidityChecker)
	throw Exception("State validity checker not set!");
    
    if (m_manifold->getDimension() <= 0)
	throw Exception("The dimension of the manifold we plan in must be > 0");
    
    if (m_resolution < std::numeric_limits<double>::epsilon())
	throw Exception("The resolution at which states need to be checked for validity must be > 0.0");
    
    m_setup = true;
}

bool ompl::base::SpaceInformation::isSetup(void) const
{
    return m_setup;
}

bool ompl::base::SpaceInformation::searchValidNearby(State *state, const State *near, double distance, unsigned int attempts) const
{
    assert(near != state);
    
    copyState(state, near);
    
    // fix bounds, if needed
    if (!satisfiesBounds(state))
	enforceBounds(state);
    
    bool result = isValid(state);
    
    if (!result)
    {
	// try to find a valid state nearby
	StateSamplerPtr ss = allocStateSampler();
	State        *temp = m_manifold->allocState();
	copyState(temp, state);	
	for (unsigned int i = 0 ; i < attempts && !result ; ++i)
	{
	    ss->sampleNear(state, temp, distance);
	    result = isValid(state);
	}
	m_manifold->freeState(temp);
    }
    
    return result;
}

bool ompl::base::SpaceInformation::checkMotion(const State *s1, const State *s2, State *lastValidState, double *lastValidTime) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!isValid(s2))
	return false;

    bool result = true;
    int nd = (int)ceil(distance(s1, s2) / m_resolution);
    
    /* temporary storage for the checked state */
    State *test = allocState();
    
    for (int j = 1 ; j < nd ; ++j)
    {
	m_manifold->interpolate(s1, s2, (double)j / (double)nd, test);
	if (!isValid(test))
	{
	    if (lastValidState)
		m_manifold->interpolate(s1, s2, (double)(j - 1) / (double)nd, lastValidState);
	    if (lastValidTime)
		*lastValidTime = (double)(j - 1) / (double)nd;
	    result = false;
	    break;
	}
    }
    freeState(test);
    
    return result;
}

bool ompl::base::SpaceInformation::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!isValid(s2))
	return false;
    
    bool result = true;
    int nd = (int)ceil(distance(s1, s2) / m_resolution);
    
    /* initialize the queue of test positions */
    std::queue< std::pair<int, int> > pos;
    if (nd >= 2)
	pos.push(std::make_pair(1, nd - 1));
    
    /* temporary storage for the checked state */
    State *test = allocState();
    
    /* repeatedly subdivide the path segment in the middle (and check the middle) */
    while (!pos.empty())
    {
	std::pair<int, int> x = pos.front();
	
	int mid = (x.first + x.second) / 2;
	m_manifold->interpolate(s1, s2, (double)mid / (double)nd, test);
	
	if (!isValid(test))
	{
	    result = false;
	    break;
	}
		
	pos.pop();
	
	if (x.first < mid)
	    pos.push(std::make_pair(x.first, mid - 1));
	if (x.second > mid)
	    pos.push(std::make_pair(mid + 1, x.second));
    }
    
    freeState(test);
    
    return result;
}

std::size_t ompl::base::SpaceInformation::getMotionStates(const State *s1, const State *s2, std::vector<base::State*> &states, double factor, bool endpoints, bool alloc) const
{
    int nd = (int)ceil(distance(s1, s2) / (m_resolution * factor));
    
    if (alloc)
    {
	states.resize(nd + (endpoints ? 1 : -1));
	if (endpoints)
	    states[0] = allocState();
    }
    
    std::size_t added = 0;
    
    if (endpoints && states.size() > 0)
    {
	copyState(states[0], s1);
	added++;
    }
    
    /* find the states in between */
    for (int j = 1 ; j < nd && added < states.size() ; ++j)
    {
	if (alloc)
	    states[added] = allocState();
	m_manifold->interpolate(s1, s2, (double)j / (double)nd, states[added]);
	added++;
    }
    
    if (added < states.size() && endpoints)
    {
	if (alloc)
	    states[added] = allocState();
	copyState(states[added], s2);
	added++;
    }
    
    return added;
}

void ompl::base::SpaceInformation::printSettings(std::ostream &out) const
{
    out << "State space settings:" << std::endl;
    out << "  - manifold:" << std::endl;
    m_manifold->printSettings(out);
    out << "  - state validity check resolution: " << m_resolution << std::endl;
}
