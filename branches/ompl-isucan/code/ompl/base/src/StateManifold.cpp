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

#include "ompl/base/StateManifold.h"
#include "ompl/util/Exception.h"

void ompl::base::StateManifold::setup(void)
{
}

void ompl::base::StateManifold::setStateSamplerAllocator(const StateSamplerAllocator &ssa)
{
    m_ssa = ssa;
}

ompl::base::StateSamplerPtr ompl::base::StateManifold::allocStateSampler(void) const
{
    if (m_ssa)
	return m_ssa(this);
    else
	return allocUniformStateSampler();
}

void ompl::base::StateManifold::printState(const State *state, std::ostream &out) const
{
    out << "State instance: " << state << std::endl;
}

void ompl::base::StateManifold::printSettings(std::ostream &out) const
{
    out << "StateManifold instance: " << this << std::endl;
}

void ompl::base::CompoundStateManifold::addSubManifold(const StateManifoldPtr &component, double weight)
{
    if (m_locked)
	throw Exception("This manifold is locked. No further components can be added");
    if (weight < 0.0)
	throw Exception("Submanifold weight cannot be negative");    
    m_components.push_back(component);
    m_weights.push_back(weight);
    m_componentCount = m_components.size();
}

unsigned int ompl::base::CompoundStateManifold::getSubManifoldCount(void) const
{
    return m_componentCount;
}

const ompl::base::StateManifoldPtr& ompl::base::CompoundStateManifold::getSubManifold(const unsigned int index) const
{
    if (m_componentCount > index)
	return m_components[index];
    else
	throw Exception("Submanifold index does not exist");
}

double ompl::base::CompoundStateManifold::getSubManifoldWeight(const unsigned int index) const
{
    if (m_componentCount > index)
	return m_weights[index];
    else
	throw Exception("Submanifold index does not exist");
}

void ompl::base::CompoundStateManifold::setSubManifoldWeight(const unsigned int index, double weight)
{
    if (weight < 0.0)
	throw Exception("Submanifold weight cannot be negative");
    if (m_componentCount > index)
	m_weights[index] = weight;
    else
	throw Exception("Submanifold index does not exist");
}

unsigned int ompl::base::CompoundStateManifold::getDimension(void) const
{
    unsigned int dim = 0;
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	dim += m_components[i]->getDimension();
    return dim;
}

void ompl::base::CompoundStateManifold::enforceBounds(State *state) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->enforceBounds(cstate->components[i]);
}

bool ompl::base::CompoundStateManifold::satisfiesBounds(const State *state) const
{   
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	if (!m_components[i]->satisfiesBounds(cstate->components[i]))
	    return false;
    return true;
}

void ompl::base::CompoundStateManifold::copyState(State *destination, const State *source) const
{   
    CompoundState      *cdest = static_cast<CompoundState*>(destination);
    const CompoundState *csrc = static_cast<const CompoundState*>(source);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->copyState(cdest->components[i], csrc->components[i]);
}

double ompl::base::CompoundStateManifold::distance(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    double dist = 0.0;
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	dist += m_weights[i] * m_components[i]->distance(cstate1->components[i], cstate2->components[i]);
    return dist;
}

bool ompl::base::CompoundStateManifold::equalStates(const State *state1, const State *state2) const
{	
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	if (!m_components[i]->equalStates(cstate1->components[i], cstate2->components[i]))
	    return false;
    return true;
}

void ompl::base::CompoundStateManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const CompoundState *cfrom  = static_cast<const CompoundState*>(from);
    const CompoundState *cto    = static_cast<const CompoundState*>(to);
    CompoundState       *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->interpolate(cfrom->components[i], cto->components[i], t, cstate->components[i]);
}

ompl::base::StateSamplerPtr ompl::base::CompoundStateManifold::allocUniformStateSampler(void) const
{
    CompoundStateSampler *ss = new CompoundStateSampler(this);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	ss->addSampler(m_components[i]->allocUniformStateSampler());
    return StateSamplerPtr(ss);
}

ompl::base::StateSamplerPtr ompl::base::CompoundStateManifold::allocStateSampler(void) const
{
    if (m_ssa)
	return m_ssa(this);
    else
    {
	CompoundStateSampler *ss = new CompoundStateSampler(this);
	for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	    ss->addSampler(m_components[i]->allocStateSampler());
	return StateSamplerPtr(ss);
    }
}

ompl::base::State* ompl::base::CompoundStateManifold::allocState(void) const
{
    CompoundState *state = new CompoundState();
    state->components = new State*[m_componentCount];
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	state->components[i] = m_components[i]->allocState();
    return static_cast<State*>(state);
}

void ompl::base::CompoundStateManifold::freeState(State *state) const 
{	
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->freeState(cstate->components[i]);
    delete[] cstate->components;
    delete cstate;
}

void ompl::base::CompoundStateManifold::lock(void)
{
    m_locked = true;
}

void ompl::base::CompoundStateManifold::printState(const State *state, std::ostream &out) const
{
    out << "Compound state [" << std::endl;
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->printState(cstate->components[i], out);
    out << "]" << std::endl;
}

void ompl::base::CompoundStateManifold::printSettings(std::ostream &out) const
{
    out << "Compound state manifold [" << std::endl;
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->printSettings(out);
    out << "]" << std::endl;
}
