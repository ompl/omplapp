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

#include "ompl/base/manifolds/SO2StateManifold.h"
#include <algorithm>
#include <cmath>

void ompl::base::SO2StateUniformSampler::sample(State *state)
{
    state->as<SO2State>()->value = rng_.uniformReal(-M_PI, M_PI);
}

void ompl::base::SO2StateUniformSampler::sampleNear(State *state, const State *near, const double distance)
{
    double &v = state->as<SO2State>()->value;
    v = rng_.uniformReal(near->as<SO2State>()->value - distance, near->as<SO2State>()->value + distance);
    // we don't need something as general as enforceBounds() since we know the input states are within bounds
    if (v < -M_PI)
	v += 2.0 * M_PI;
    else
	if (v > M_PI)
	    v -= 2.0 * M_PI;    
}

unsigned int ompl::base::SO2StateManifold::getDimension(void) const
{
    return 1;
}

void ompl::base::SO2StateManifold::enforceBounds(State *state) const
{
    double v = fmod(state->as<SO2State>()->value, 2.0 * M_PI);
    if (v < -M_PI)
	v += 2.0 * M_PI;
    else
	if (v > M_PI)
	    v -= 2.0 * M_PI;
    state->as<SO2State>()->value = v;
}    
	    	    
bool ompl::base::SO2StateManifold::satisfiesBounds(const State *state) const
{
    return (state->as<SO2State>()->value < M_PI + std::numeric_limits<double>::epsilon()) && 
	    (state->as<SO2State>()->value > -M_PI - std::numeric_limits<double>::epsilon());
}

void ompl::base::SO2StateManifold::copyState(State *destination, const State *source) const
{
    destination->as<SO2State>()->value = source->as<SO2State>()->value;
}

double ompl::base::SO2StateManifold::distance(const State *state1, const State *state2) const
{
    // assuming the states 1 & 2 are within bounds
    double d = fabs(state1->as<SO2State>()->value - state2->as<SO2State>()->value);
    return (d > M_PI) ? 2.0 * M_PI - d : d;
}

bool ompl::base::SO2StateManifold::equalStates(const State *state1, const State *state2) const
{
    return fabs(state1->as<SO2State>()->value - state2->as<SO2State>()->value) < std::numeric_limits<double>::epsilon();
}

void ompl::base::SO2StateManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    double diff = to->as<SO2State>()->value - from->as<SO2State>()->value;
    if (fabs(diff) <= M_PI)
	state->as<SO2State>()->value = from->as<SO2State>()->value + diff * t;
    else
    {
	double &v = state->as<SO2State>()->value;
	if (diff > 0.0)
	    diff = 2.0 * M_PI - diff;
	else
	    diff = -2.0 * M_PI - diff;
	v = from->as<SO2State>()->value - diff * t;
	// input states are within bounds, so the following check is sufficient
	if (v > M_PI)
	    v -= 2.0 * M_PI;
	else
	    if (v < -M_PI)
		v += 2.0 * M_PI;	
    }
}

ompl::base::StateSamplerPtr ompl::base::SO2StateManifold::allocUniformStateSampler(void) const 
{
    return StateSamplerPtr(new SO2StateUniformSampler(this));
}

ompl::base::State* ompl::base::SO2StateManifold::allocState(void) const
{
    return new SO2State();
}

void ompl::base::SO2StateManifold::freeState(State *state) const
{
    delete static_cast<SO2State*>(state);
}

void ompl::base::SO2StateManifold::printState(const State *state, std::ostream &out) const
{
    if (state)
	out << state->as<SO2State>()->value << std::endl;
    else
	out << "NULL" << std::endl;
}

void ompl::base::SO2StateManifold::printSettings(std::ostream &out) const
{
    out << "SO2 state manifold" << std::endl;
}
