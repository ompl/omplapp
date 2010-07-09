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

#include "ompl/base/Manifold.h"

ompl::base::Manifold::~Manifold(void)
{
    if (m_lowerBound)
	freeState(m_lowerBound);
    if (m_upperBound)
	freeState(m_upperBound);
}

void ompl::base::Manifold::setBounds(const State *lower, const State *upper)
{
    setLowerBound(lower);
    setUpperBound(upper);    
}

void ompl::base::Manifold::setUpperBound(const State *bound)
{
    if (!m_upperBound)
	m_upperBound = allocState();
    copyState(m_upperBound, bound);
}

void ompl::base::Manifold::setLowerBound(const State *bound)
{
    if (!m_lowerBound)
	m_lowerBound = allocState();
    copyState(m_lowerBound, bound);
}

void ompl::base::Manifold::printState(State *state, std::ostream &out) const
{
    out << state << std::endl;
}

ompl::base::CompoundManifold::~CompoundManifold(void)
{
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	delete m_components[i];
}

void ompl::base::CompoundManifold::addManifold(Manifold *component, double weight)
{
    m_components.push_back(component);
    m_weights.push_back(weight);
}

unsigned int ompl::base::CompoundManifold::getDimension(void) const
{
    unsigned int dim = 0;
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	dim += m_components[i]->getDimension();
    return dim;
}

void ompl::base::CompoundManifold::enforceBounds(State *state) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	m_components[i]->enforceBounds(cstate->components[i]);
}

bool ompl::base::CompoundManifold::satisfiesBounds(const State *state) const
{   
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	if (!m_components[i]->satisfiesBounds(cstate->components[i]))
	    return false;
    return true;
}

void ompl::base::CompoundManifold::copyState(State *destination, const State *source) const
{   
    CompoundState      *cdest = static_cast<CompoundState*>(destination);
    const CompoundState *csrc = static_cast<const CompoundState*>(source);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	m_components[i]->copyState(cdest->components[i], csrc->components[i]);
}

double ompl::base::CompoundManifold::distance(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    double dist = 0.0;
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	dist += m_weights[i] * m_components[i]->distance(cstate1->components[i], cstate2->components[i]);
    return dist;
}

bool ompl::base::CompoundManifold::equalStates(const State *state1, const State *state2) const
{	
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	if (!m_components[i]->equalStates(cstate1->components[i], cstate2->components[i]))
	    return false;
    return true;
}

void ompl::base::CompoundManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const CompoundState *cfrom  = static_cast<const CompoundState*>(from);
    const CompoundState *cto    = static_cast<const CompoundState*>(to);
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	m_components[i]->interpolate(cfrom->components[i], cto->components[i], t, cstate);
}

ompl::base::StateSampler* ompl::base::CompoundManifold::allocStateSampler(void) const
{
    CompoundStateSampler *ss = new CompoundStateSampler(this);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	ss->addSampler(m_components[i]->allocStateSampler());
    return ss;
}

ompl::base::State* ompl::base::CompoundManifold::allocState(void) const
{
    CompoundState *state = new CompoundState();
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	state->components[i] = m_components[i]->allocState();
    return state;
}

void ompl::base::CompoundManifold::freeState(State *state) const 
{	
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	m_components[i]->freeState(cstate->components[i]);
    delete state;
}

void ompl::base::CompoundManifold::setUpperBound(const State *bound)
{
    Manifold::setUpperBound(bound);
    const CompoundState *cstate = static_cast<const CompoundState*>(bound);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	m_components[i]->setUpperBound(cstate->components[i]);
}

void ompl::base::CompoundManifold::setLowerBound(const State *bound)
{
    Manifold::setLowerBound(bound);
    const CompoundState *cstate = static_cast<const CompoundState*>(bound);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	m_components[i]->setLowerBound(cstate->components[i]);
}

void ompl::base::CompoundManifold::printState(const State *state, std::ostream &out) const
{
    out << "Compound [" << std::endl;
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
	m_components[i]->printState(cstate->components[i], out);
    out << "]" << std::endl;
}
