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

#include "ompl/base/manifolds/QuaternionStateManifold.h"
#include <algorithm>
#include <limits>
#include <cmath>

void ompl::base::QuaternionStateUniformSampler::sample(State *state)
{
    rng_.quaternion(&state->as<QuaternionState>()->x);
}

void ompl::base::QuaternionStateUniformSampler::sampleNear(State *state, const State *near, const double distance)
{
    /** \todo How do we sample near a quaternion ? */
    sample(state);
}

unsigned int ompl::base::QuaternionStateManifold::getDimension(void) const
{
    return 3;
}

double ompl::base::QuaternionStateManifold::norm(const State *state) const
{
    const QuaternionState *qstate = static_cast<const QuaternionState*>(state);
    double nrmSqr = qstate->x * qstate->x + qstate->y * qstate->y + qstate->z * qstate->z + qstate->w * qstate->w;
    return (fabs(nrmSqr - 1.0) > std::numeric_limits<double>::epsilon()) ? sqrt(nrmSqr) : 1.0;
}

void ompl::base::QuaternionStateManifold::enforceBounds(State *state) const
{
    double nrm = norm(state);
    if (fabs(nrm - 1.0) > std::numeric_limits<double>::epsilon())
    {
	QuaternionState *qstate = static_cast<QuaternionState*>(state);
	qstate->x /= nrm;
	qstate->y /= nrm;
	qstate->z /= nrm;
	qstate->w /= nrm;	
    }
}    
	    	    
bool ompl::base::QuaternionStateManifold::satisfiesBounds(const State *state) const
{
    return fabs(norm(state) - 1.0) < std::numeric_limits<double>::epsilon();
}

void ompl::base::QuaternionStateManifold::copyState(State *destination, const State *source) const
{
    const QuaternionState *qsource = static_cast<const QuaternionState*>(source);
    QuaternionState *qdestination = static_cast<QuaternionState*>(destination);
    qdestination->x = qsource->x;
    qdestination->y = qsource->y;
    qdestination->z = qsource->z;
    qdestination->w = qsource->w;
}


/*
Based on code from :

Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
*/
double ompl::base::QuaternionStateManifold::distance(const State *state1, const State *state2) const
{
    const QuaternionState *qs1 = static_cast<const QuaternionState*>(state1);
    const QuaternionState *qs2 = static_cast<const QuaternionState*>(state2);
    double dq = fabs(qs1->x * qs2->x + qs1->y * qs2->y + qs1->z * qs2->z + qs1->w * qs2->w);
    if (dq > 1.0 - std::numeric_limits<double>::epsilon())
	return 0.0;
    else
	return acos(dq) * 2.0;
}

bool ompl::base::QuaternionStateManifold::equalStates(const State *state1, const State *state2) const
{
    return distance(state1, state2) < std::numeric_limits<double>::epsilon();
}

/*
Based on code from :

Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
*/
void ompl::base::QuaternionStateManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    double theta = distance(from, to) / 2.0;
    if (theta > std::numeric_limits<double>::epsilon())
    {
	double d = 1.0 / sin(theta);
	double s0 = sin((1.0 - t) * theta);
	double s1 = sin(t * theta);
	
	const QuaternionState *qs1 = static_cast<const QuaternionState*>(from);
	const QuaternionState *qs2 = static_cast<const QuaternionState*>(to);
	QuaternionState       *qr  = static_cast<QuaternionState*>(state);
	double dq = qs1->x * qs2->x + qs1->y * qs2->y + qs1->z * qs2->z + qs1->w * qs2->w;
	if (dq < 0)  // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
	    s1 = -s1;
	
	qr->x = (qs1->x * s0 + qs2->x * s1) * d;
	qr->y = (qs1->y * s0 + qs2->y * s1) * d;
	qr->z = (qs1->z * s0 + qs2->z * s1) * d;
	qr->w = (qs1->w * s0 + qs2->w * s1) * d;
    }
    else
    {
	if (state != from)
	    copyState(state, from);
    }
}

ompl::base::StateSamplerPtr ompl::base::QuaternionStateManifold::allocUniformStateSampler(void) const 
{
    return StateSamplerPtr(new QuaternionStateUniformSampler(this));
}

ompl::base::State* ompl::base::QuaternionStateManifold::allocState(void) const
{
    return new QuaternionState();
}

void ompl::base::QuaternionStateManifold::freeState(State *state) const
{
    delete static_cast<QuaternionState*>(state);
}

void ompl::base::QuaternionStateManifold::printState(const State *state, std::ostream &out) const
{
    if (state)
    {
	const QuaternionState *qstate = static_cast<const QuaternionState*>(state);
	out << qstate->x << " " << qstate->y << " " << qstate->z << " " << qstate->w << std::endl;
    }
    else
	out << "NULL" << std::endl;
}

void ompl::base::QuaternionStateManifold::printSettings(std::ostream &out) const
{
    out << "SO(3) state manifold (represented using quaternions)" << std::endl;
}
