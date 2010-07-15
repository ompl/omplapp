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
#include "ompl/util/Exception.h"
#include <algorithm>
#include <functional>

ompl::base::Manifold::~Manifold(void)
{ 
}

void ompl::base::Manifold::printState(const State *state, std::ostream &out) const
{
    out << "State instance: " << state << std::endl;
}

void ompl::base::Manifold::printSettings(std::ostream &out) const
{
    out << "Manifold instance: " << this << std::endl;
}

void ompl::base::CompoundManifold::addManifold(const ManifoldPtr &component, double weight)
{
    m_components.push_back(component);
    m_weights.push_back(weight);
    m_componentCount = m_components.size();
}

std::size_t ompl::base::CompoundManifold::getManifoldCount(void) const
{
    return m_componentCount;
}

const ompl::base::ManifoldPtr& ompl::base::CompoundManifold::getManifold(const std::size_t index) const
{
    if (m_componentCount > index)
	return m_components[index];
    else
	throw Exception("Manifold index does not exist");
}

double ompl::base::CompoundManifold::getManifoldWeight(const std::size_t index) const
{
    if (m_componentCount > index)
	return m_weights[index];
    else
	throw Exception("Manifold index does not exist");
}

unsigned int ompl::base::CompoundManifold::getDimension(void) const
{
    unsigned int dim = 0;
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	dim += m_components[i]->getDimension();
    return dim;
}

void ompl::base::CompoundManifold::enforceBounds(State *state) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->enforceBounds(cstate->components[i]);
}

bool ompl::base::CompoundManifold::satisfiesBounds(const State *state) const
{   
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	if (!m_components[i]->satisfiesBounds(cstate->components[i]))
	    return false;
    return true;
}

void ompl::base::CompoundManifold::copyState(State *destination, const State *source) const
{   
    CompoundState      *cdest = static_cast<CompoundState*>(destination);
    const CompoundState *csrc = static_cast<const CompoundState*>(source);
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->copyState(cdest->components[i], csrc->components[i]);
}

double ompl::base::CompoundManifold::distance(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    double dist = 0.0;
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	dist += m_weights[i] * m_components[i]->distance(cstate1->components[i], cstate2->components[i]);
    return dist;
}

bool ompl::base::CompoundManifold::equalStates(const State *state1, const State *state2) const
{	
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	if (!m_components[i]->equalStates(cstate1->components[i], cstate2->components[i]))
	    return false;
    return true;
}

void ompl::base::CompoundManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const CompoundState *cfrom  = static_cast<const CompoundState*>(from);
    const CompoundState *cto    = static_cast<const CompoundState*>(to);
    CompoundState       *cstate = static_cast<CompoundState*>(state);
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->interpolate(cfrom->components[i], cto->components[i], t, cstate->components[i]);
}

ompl::base::StateSamplerPtr ompl::base::CompoundManifold::allocStateSampler(void) const
{
    CompoundStateSampler *ss = new CompoundStateSampler(this);
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	ss->addSampler(m_components[i]->allocStateSampler());
    return StateSamplerPtr(ss);
}

ompl::base::State* ompl::base::CompoundManifold::allocState(void) const
{
    CompoundState *state = new CompoundState();
    state->components = new State*[m_componentCount];
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	state->components[i] = m_components[i]->allocState();
    return static_cast<State*>(state);
}

void ompl::base::CompoundManifold::freeState(State *state) const 
{	
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->freeState(cstate->components[i]);
    delete[] cstate->components;
    delete cstate;
}

void ompl::base::CompoundManifold::printState(const State *state, std::ostream &out) const
{
    out << "Compound state [" << std::endl;
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->printState(cstate->components[i], out);
    out << "]" << std::endl;
}

void ompl::base::CompoundManifold::printSettings(std::ostream &out) const
{
    out << "Compound manifold [" << std::endl;
    for (std::size_t i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->printSettings(out);
    out << "]" << std::endl;
}
