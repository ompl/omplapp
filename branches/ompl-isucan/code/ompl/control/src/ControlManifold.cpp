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

#include "ompl/control/ControlManifold.h"
#include "ompl/util/Exception.h"

const ompl::base::StateManifoldPtr& ompl::control::ControlManifold::getStateManifold(void) const
{
    return m_stateManifold;
}

void ompl::control::ControlManifold::printControl(const Control *control, std::ostream &out) const
{
    out << "Control instance: " << control << std::endl;
}

void ompl::control::ControlManifold::printSettings(std::ostream &out) const
{
    out << "ControlManifold instance: " << this << std::endl;
}

void ompl::control::CompoundControlManifold::addSubManifold(const ControlManifoldPtr &component)
{
    if (m_locked)
	throw Exception("This manifold is locked. No further components can be added");
    
    m_components.push_back(component);
    m_componentCount = m_components.size();
}

unsigned int ompl::control::CompoundControlManifold::getSubManifoldCount(void) const
{
    return m_componentCount;
}

const ompl::control::ControlManifoldPtr& ompl::control::CompoundControlManifold::getSubManifold(const unsigned int index) const
{
    if (m_componentCount > index)
	return m_components[index];
    else
	throw Exception("Submanifold index does not exist");
}

unsigned int ompl::control::CompoundControlManifold::getDimension(void) const
{
    unsigned int dim = 0;
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	dim += m_components[i]->getDimension();
    return dim;
}

ompl::control::Control* ompl::control::CompoundControlManifold::allocControl(void) const
{
    CompoundControl *control = new CompoundControl();
    control->components = new Control*[m_componentCount];
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	control->components[i] = m_components[i]->allocControl();
    return static_cast<Control*>(control);
}

void ompl::control::CompoundControlManifold::freeControl(Control *control) const
{  
    CompoundControl *ccontrol = static_cast<CompoundControl*>(control);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->freeControl(ccontrol->components[i]);
    delete[] ccontrol->components;
    delete ccontrol;
}

void ompl::control::CompoundControlManifold::copyControl(Control *destination, const Control *source) const
{  
    CompoundControl      *cdest = static_cast<CompoundControl*>(destination);
    const CompoundControl *csrc = static_cast<const CompoundControl*>(source);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->copyControl(cdest->components[i], csrc->components[i]);
}

bool ompl::control::CompoundControlManifold::equalControls(const Control *control1, const Control *control2) const
{ 
    const CompoundControl *ccontrol1 = static_cast<const CompoundControl*>(control1);
    const CompoundControl *ccontrol2 = static_cast<const CompoundControl*>(control2);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	if (!m_components[i]->equalControls(ccontrol1->components[i], ccontrol2->components[i]))
	    return false;
    return true;
}

void ompl::control::CompoundControlManifold::nullControl(Control *control) const
{   
    CompoundControl *ccontrol = static_cast<CompoundControl*>(control);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->nullControl(ccontrol->components[i]);
}

ompl::control::ControlSamplerPtr ompl::control::CompoundControlManifold::allocControlSampler(void) const
{
    CompoundControlSampler *ss = new CompoundControlSampler(this);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	ss->addSampler(m_components[i]->allocControlSampler());
    return ControlSamplerPtr(ss);
}

ompl::control::PropagationResult ompl::control::CompoundControlManifold::propagate(const base::State *state, const Control* control, const double duration, base::State *result) const
{
    const base::CompoundState *cstate = static_cast<const base::CompoundState*>(state);
    const CompoundControl *ccontrol = static_cast<const CompoundControl*>(control);
    base::CompoundState *cresult = static_cast<base::CompoundState*>(result);
    PropagationResult pr = PROPAGATION_START_VALID;
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
    {
	PropagationResult p = m_components[i]->propagate(cstate->components[i], ccontrol->components[i], duration, cresult->components[i]);
	if (pr == PROPAGATION_START_VALID)
	    if (p != PROPAGATION_START_VALID)
		pr = p;
    }
    return pr;
}

void ompl::control::CompoundControlManifold::lock(void)
{
    m_locked = true;
}

void ompl::control::CompoundControlManifold::printControl(const Control *control, std::ostream &out) const
{
    out << "Compound control [" << std::endl;
    const CompoundControl *ccontrol = static_cast<const CompoundControl*>(control);
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->printControl(ccontrol->components[i], out);
    out << "]" << std::endl;
}

void ompl::control::CompoundControlManifold::printSettings(std::ostream &out) const
{
    out << "Compound control manifold [" << std::endl;
    for (unsigned int i = 0 ; i < m_componentCount ; ++i)
	m_components[i]->printSettings(out);
    out << "]" << std::endl;
}
