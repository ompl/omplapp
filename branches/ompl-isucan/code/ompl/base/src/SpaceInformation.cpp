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

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <cassert>

/*
void ompl::base::SpaceInformation::copyState(State *destination, const State *source) const
{
    memcpy(destination->values, source->values, sizeof(double) * m_stateDimension);
}

bool ompl::base::SpaceInformation::equalStates(const State *a, const State *b) const
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	if (a->values[i] != b->values[i])
	    return false;
    return true;
}
*/

void ompl::base::SpaceInformation::setup(void)
{
    if (m_setup)
	m_msg.warn("Space information setup called multiple times");
    
    if (!m_stateValidityChecker)
	throw Exception("State validity checker not set!");
    
    if (m_manifold->getDimension() <= 0)
	throw Exception("The dimension of the manifold we plan in must be > 0");
    
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

void ompl::base::SpaceInformation::printSettings(std::ostream &out) const
{
    out << "State space settings:" << std::endl;
    m_manifold->printSettings(out);
}
