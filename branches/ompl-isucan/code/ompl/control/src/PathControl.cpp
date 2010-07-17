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

#include "ompl/control/PathControl.h"
#include "ompl/util/Exception.h"
#include <numeric>
#include <cmath>

ompl::control::PathControl::PathControl(const base::SpaceInformationPtr &si) : base::Path(si)
{
    if (!dynamic_cast<const SpaceInformation*>(m_si.get()))
	throw Exception("Cannot create a path with controls from a space that does not support controls");
}

ompl::control::PathControl::PathControl(const PathControl &path) : base::Path(path.m_si)
{
    states.resize(path.states.size());
    controls.resize(path.controls.size());

    for (unsigned int i = 0 ; i < states.size() ; ++i)
	states[i] = m_si->cloneState(path.states[i]);
    
    const SpaceInformation *si = static_cast<const SpaceInformation*>(m_si.get());
    for (unsigned int i = 0 ; i < controls.size() ; ++i)
	controls[i] = si->cloneControl(path.controls[i]);
    
    controlDurations = path.controlDurations;
}

double ompl::control::PathControl::length(void) const
{
    return std::accumulate(controlDurations.begin(), controlDurations.end(), 0.0);
}

void ompl::control::PathControl::print(std::ostream &out) const
{
    const SpaceInformation *si = static_cast<const SpaceInformation*>(m_si.get());
    double res = si->getPropagationStepSize();
    out << "Control path with " << states.size() << " states" << std::endl;
    for (unsigned int i = 0 ; i < controls.size() ; ++i)
    {
	out << "At state ";
	m_si->printState(states[i], out);
	out << "  apply control ";
	si->printControl(controls[i], out);
	out << "  for " << (int)round(controlDurations[i]/res) << " steps" << std::endl;
    }
    out << "Arrive at state ";
    m_si->printState(states[controls.size()], out);
    out << std::endl;
}

void ompl::control::PathControl::interpolate(void) 
{
    const SpaceInformation *si = static_cast<const SpaceInformation*>(m_si.get());
    std::vector<base::State*> newStates;
    std::vector<Control*> newControls;
    std::vector<double> newControlDurations;
    
    double res = si->getPropagationStepSize();
    for (unsigned int  i = 0 ; i < controls.size() ; ++i)
    {
	int steps = (int)round(controlDurations[i] / res);
	assert(steps >= 0);
	if (steps == 0)
	{
	    newStates.push_back(states[i]);
	    newControls.push_back(controls[i]);
	    newControlDurations.push_back(controlDurations[i]);
	    continue;
	}
	std::vector<base::State*> istates;
	si->propagate(states[i], controls[i], steps, istates, false, true);
	newStates.push_back(states[i]);
	newStates.insert(newStates.end(), istates.begin(), istates.end());
	newControls.push_back(controls[i]);
	newControlDurations.push_back(res);
	for (int j = 1 ; j < steps; ++j)
	{
	    newControls.push_back(si->cloneControl(controls[i]));
	    newControlDurations.push_back(res);
	}
    }
    newStates.push_back(states[controls.size()]);
    states.swap(newStates);
    controls.swap(newControls);
    controlDurations.swap(newControlDurations);
}

bool ompl::control::PathControl::check(void) const
{
    bool valid = true;
    const SpaceInformation *si = static_cast<const SpaceInformation*>(m_si.get());
    base::State *dummy = m_si->allocState();
    for (unsigned int  i = 0 ; i < controls.size() ; ++i)
	if (si->propagateWhileValid(states[i], controls[i], controlDurations[i], dummy) != controlDurations[i])
	{
	    valid = false;
	    break;
	}
    m_si->freeState(dummy);
    return valid;
}

void ompl::control::PathControl::freeMemory(void)
{
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	m_si->freeState(states[i]);
    const SpaceInformation *si = static_cast<const SpaceInformation*>(m_si.get());
    for (unsigned int i = 0 ; i < controls.size() ; ++i)
	si->freeControl(controls[i]);
}
