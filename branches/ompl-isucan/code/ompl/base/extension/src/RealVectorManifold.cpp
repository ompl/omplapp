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

#include "ompl/base/extension/RealVectorManifold.h"
#include <cstring>
#include <limits>

ompl::ext::RealVectorStateUniformSampler::RealVectorStateUniformSampler(const base::Manifold *manifold) :
    base::StateSampler(manifold), 
    m_bounds(static_cast<const RealVectorManifold*>(manifold)->getBounds())
{
}

void ompl::ext::RealVectorStateUniformSampler::sample(base::State *state)
{
    const unsigned int dim = m_manifold->getDimension();
    RealVectorState *rstate = static_cast<RealVectorState*>(state);
    for (unsigned int i = 0 ; i < dim ; ++i)
	rstate->values[i] = m_rng.uniformReal(m_bounds.first[i], m_bounds.second[i]);
}

void ompl::ext::RealVectorStateUniformSampler::sampleNear(base::State *state, const base::State *near, const double distance)
{
    const unsigned int dim = m_manifold->getDimension();
    RealVectorState *rstate = static_cast<RealVectorState*>(state);
    const RealVectorState *rnear = static_cast<const RealVectorState*>(near);
    for (unsigned int i = 0 ; i < dim ; ++i)
	rstate->values[i] =
	    m_rng.uniformReal(std::max(m_bounds.first[i], rnear->values[i] - distance), 
			      std::min(m_bounds.second[i], rnear->values[i] + distance));
}

unsigned int ompl::ext::RealVectorManifold::getDimension(void) const
{
    return m_dimension;
}

void ompl::ext::RealVectorManifold::enforceBounds(base::State *state) const
{
    RealVectorState *rstate = static_cast<RealVectorState*>(state);    
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
    {
	if (rstate->values[i] > m_bounds.second[i])
	    rstate->values[i] = m_bounds.second[i];
	else
	    if (rstate->values[i] < m_bounds.first[i])
		rstate->values[i] = m_bounds.first[i];
    }
}    
	    	    
bool ompl::ext::RealVectorManifold::satisfiesBounds(const base::State *state) const
{
    const RealVectorState *rstate = static_cast<const RealVectorState*>(state);    
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
	if (rstate->values[i] - std::numeric_limits<double>::round_error() > m_bounds.second[i] ||
	    rstate->values[i] + std::numeric_limits<double>::round_error() < m_bounds.first[i])
	    return false;
    return true;
}

void ompl::ext::RealVectorManifold::copyState(base::State *destination, const base::State *source) const
{
    memcpy(static_cast<RealVectorState*>(destination)->values,
	   static_cast<const RealVectorState*>(source)->values, m_stateBytes);    
}

double ompl::ext::RealVectorManifold::distance(const base::State *state1, const base::State *state2) const
{
    double dist = 0.0;
    const double *s1 = static_cast<const RealVectorState*>(state1)->values;
    const double *s2 = static_cast<const RealVectorState*>(state2)->values;
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
    {	 
	double diff = (s1++) - (s2++);
	dist += diff * diff;
    }
    return dist;
}

bool ompl::ext::RealVectorManifold::equalStates(const base::State *state1, const base::State *state2) const
{
    const double *s1 = static_cast<const RealVectorState*>(state1)->values;
    const double *s2 = static_cast<const RealVectorState*>(state2)->values;
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
    {	 
	double diff = (s1++) - (s2++);
	if (fabs(diff) > std::numeric_limits<double>::round_error())
	    return false;
    }
    return true;
}

void ompl::ext::RealVectorManifold::interpolate(const base::State *from, const base::State *to, const double t, base::State *state) const
{
    const RealVectorState *rfrom = static_cast<const RealVectorState*>(from);
    const RealVectorState *rto = static_cast<const RealVectorState*>(to);
    const RealVectorState *rstate = static_cast<RealVectorState*>(state);
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
	rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
}

ompl::base::StateSamplerPtr ompl::ext::RealVectorManifold::allocStateSampler(void) const 
{
    return base::StateSamplerPtr(new RealVectorStateUniformSampler(this));
}

ompl::base::State* ompl::ext::RealVectorManifold::allocState(void) const
{
    RealVectorState *rstate = new RealVectorState();
    rstate->values = new double[m_dimension];
    return rstate;
}

void ompl::ext::RealVectorManifold::freeState(base::State *state) const
{
    RealVectorState *rstate = static_cast<RealVectorState*>(state);
    delete[] rstate->values;
    delete rstate;
}

void ompl::ext::RealVectorManifold::printState(const base::State *state, std::ostream &out) const
{
    if (state)
    {
	const RealVectorState *rstate = static_cast<const RealVectorState*>(state);
	for (unsigned int i = 0 ; i < m_dimension ; ++i)
	    out << rstate->values[i] << " ";
	out << std::endl;
    }
    else
	out << "NULL" << std::endl;
}
