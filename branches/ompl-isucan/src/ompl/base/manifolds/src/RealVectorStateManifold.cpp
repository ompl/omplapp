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

/* Author: Ioan Sucan */

#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/util/Exception.h"
#include <cstring>
#include <limits>
#include <cmath>

void ompl::base::RealVectorStateUniformSampler::sample(State *state)
{
    const unsigned int dim = manifold_->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorStateManifold*>(manifold_)->getBounds();
    
    RealVectorState *rstate = static_cast<RealVectorState*>(state);
    for (unsigned int i = 0 ; i < dim ; ++i)
	rstate->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}

void ompl::base::RealVectorStateUniformSampler::sampleNear(State *state, const State *near, const double distance)
{
    const unsigned int dim = manifold_->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorStateManifold*>(manifold_)->getBounds();

    RealVectorState *rstate = static_cast<RealVectorState*>(state);
    const RealVectorState *rnear = static_cast<const RealVectorState*>(near);
    for (unsigned int i = 0 ; i < dim ; ++i)
	rstate->values[i] =
	    rng_.uniformReal(std::max(bounds.low[i], rnear->values[i] - distance), 
			      std::min(bounds.high[i], rnear->values[i] + distance));
}

void ompl::base::RealVectorStateManifold::setup(void)
{
    StateManifold::setup();
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	if (bounds_.low[i] + std::numeric_limits<double>::epsilon() > bounds_.high[i])
	    throw Exception("Bounds for real vector state manifold seem to be incorrect (lower bound must be stricly less than upper bound). Sampling will not be possible");
}

void ompl::base::RealVectorStateManifold::setBounds(const RealVectorBounds &bounds)
{
    if (bounds.low.size() != bounds.high.size())
	throw Exception("Lower and upper bounds are not of same dimension");
    if (bounds.low.size() != dimension_)
	throw Exception("Bounds do not match dimension of manifold");
    bounds_ = bounds;
}

unsigned int ompl::base::RealVectorStateManifold::getDimension(void) const
{
    return dimension_;
}

void ompl::base::RealVectorStateManifold::enforceBounds(State *state) const
{
    RealVectorState *rstate = static_cast<RealVectorState*>(state);
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
	if (rstate->values[i] > bounds_.high[i])
	    rstate->values[i] = bounds_.high[i];
	else
	    if (rstate->values[i] < bounds_.low[i])
		rstate->values[i] = bounds_.low[i];
    }
}    
	    	    
bool ompl::base::RealVectorStateManifold::satisfiesBounds(const State *state) const
{
    const RealVectorState *rstate = static_cast<const RealVectorState*>(state);    
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	if (rstate->values[i] - std::numeric_limits<double>::epsilon() > bounds_.high[i] ||
	    rstate->values[i] + std::numeric_limits<double>::epsilon() < bounds_.low[i])
	    return false;
    return true;
}

void ompl::base::RealVectorStateManifold::copyState(State *destination, const State *source) const
{
    memcpy(static_cast<RealVectorState*>(destination)->values,
	   static_cast<const RealVectorState*>(source)->values, stateBytes_);    
}

double ompl::base::RealVectorStateManifold::distance(const State *state1, const State *state2) const
{
    double dist = 0.0;
    const double *s1 = static_cast<const RealVectorState*>(state1)->values;
    const double *s2 = static_cast<const RealVectorState*>(state2)->values;
    
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {	 
	double diff = (*s1++) - (*s2++);
	dist += diff * diff;
    }
    return sqrt(dist);
}

bool ompl::base::RealVectorStateManifold::equalStates(const State *state1, const State *state2) const
{
    const double *s1 = static_cast<const RealVectorState*>(state1)->values;
    const double *s2 = static_cast<const RealVectorState*>(state2)->values;
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {	 
	double diff = (*s1++) - (*s2++);
	if (fabs(diff) > std::numeric_limits<double>::epsilon())
	    return false;
    }
    return true;
}

void ompl::base::RealVectorStateManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const RealVectorState *rfrom = static_cast<const RealVectorState*>(from);
    const RealVectorState *rto = static_cast<const RealVectorState*>(to);
    const RealVectorState *rstate = static_cast<RealVectorState*>(state);
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
}

ompl::base::StateSamplerPtr ompl::base::RealVectorStateManifold::allocUniformStateSampler(void) const 
{
    return StateSamplerPtr(new RealVectorStateUniformSampler(this));
}

ompl::base::State* ompl::base::RealVectorStateManifold::allocState(void) const
{
    RealVectorState *rstate = new RealVectorState();
    rstate->values = new double[dimension_];
    return rstate;
}

void ompl::base::RealVectorStateManifold::freeState(State *state) const
{
    RealVectorState *rstate = static_cast<RealVectorState*>(state);
    delete[] rstate->values;
    delete rstate;
}

void ompl::base::RealVectorStateManifold::printState(const State *state, std::ostream &out) const
{
    if (state)
    {
	const RealVectorState *rstate = static_cast<const RealVectorState*>(state);
	for (unsigned int i = 0 ; i < dimension_ ; ++i)
	    out << rstate->values[i] << " ";
	out << std::endl;
    }
    else
	out << "NULL" << std::endl;
}

void ompl::base::RealVectorStateManifold::printSettings(std::ostream &out) const
{
    out << "Real vector state manifold of dimension " << dimension_ << " with bounds: " << std::endl;
    out << "  - min: ";
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	out << bounds_.low[i] << " ";
    out << std::endl;    
    out << "  - max: ";
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	out << bounds_.high[i] << " ";
    out << std::endl;
}
